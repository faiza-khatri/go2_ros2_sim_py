#!/usr/bin/env python3
"""
Frontier + information-gain viewpoint exploration node.

Pipeline (upgraded from the greedy-nearest MVP):
  1. Subscribe to a 2D nav_msgs/OccupancyGrid (/projected_map from octomap_server,
     itself fed by GLIM's 3D map). Cell values: 0=free, 100=occupied, -1=unknown.
  2. Detect frontier cells: free cells 8-adjacent to >=1 unknown cell.
  3. Cluster frontier cells into connected components (poor-man's WFD).
  4. For each cluster, compute a safe standoff viewpoint (pulled back from the
     obstacle-adjacent boundary into clear free space), oriented to face the
     unknown region.
  5. Score each viewpoint by an actual visibility/information-gain estimate
     (raycast a synthetic sensor sweep from the candidate pose and count how
     many currently-unknown cells it would newly observe), not just raw
     frontier cluster pixel count.
  6. Take the top-K candidate viewpoints by utility (gain / distance^alpha,
     a cost-utility formulation in the same spirit as next-best-view / FALCON-
     style planners) and order them into a short visitation queue via a cheap
     greedy nearest-neighbor chain from the robot's current pose -- a light
     stand-in for FALCON's global coverage-path-guided visitation ordering,
     without the full TSP solver.
  7. Drain the queue one NavigateToPose goal at a time; only re-run full
     detection + rescoring once the queue empties or a goal fails, so the
     robot commits to a locally coherent sweep instead of re-planning myopically
     after every single step.
  8. Stop when no frontier clusters remain above the minimum size, anywhere
     inside the configured world bounds.
"""

import math
from collections import deque

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener


FREE = 0
UNKNOWN = -1
OCC_THRESHOLD = 50  # cells >= this are treated as occupied/obstacle

MIN_FRONTIER_CELLS = 6          # ignore tiny noisy clusters
GOAL_STANDOFF_M = 0.6           # pull goal back from the frontier boundary into free space
MIN_OBSTACLE_CLEARANCE_M = 0.28  # matches robot footprint half-width (~0.18m) + small margin,
                                  # not an arbitrarily conservative fixed value
REPLAN_ON_FAILURE_BLACKLIST_M = 0.75  # don't re-pick a goal within this radius of a failed one

SENSOR_RANGE_M = 6.0     # synthetic sensor range used for the info-gain raycast estimate
NUM_RAYS = 72            # angular resolution of the info-gain raycast sweep (5 deg steps)
TOP_K_CANDIDATES = 5     # how many best-utility viewpoints to chain into a visitation queue
UTILITY_DISTANCE_ALPHA = 1.0  # cost-utility exponent: utility = gain / (dist + eps)^alpha


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        self.declare_parameter('map_topic', '/projected_map')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('min_frontier_cells', MIN_FRONTIER_CELLS)
        self.declare_parameter('bounds_min_x', -25.0)
        self.declare_parameter('bounds_max_x', 25.0)
        self.declare_parameter('bounds_min_y', -25.0)
        self.declare_parameter('bounds_max_y', 25.0)
        self.declare_parameter('sensor_range_m', SENSOR_RANGE_M)
        self.declare_parameter('top_k_candidates', TOP_K_CANDIDATES)

        self.map_topic = self.get_parameter('map_topic').value
        self.base_frame = self.get_parameter('robot_base_frame').value
        self.global_frame = self.get_parameter('global_frame').value
        self.min_frontier_cells = self.get_parameter('min_frontier_cells').value
        self.bounds_min_x = self.get_parameter('bounds_min_x').value
        self.bounds_max_x = self.get_parameter('bounds_max_x').value
        self.bounds_min_y = self.get_parameter('bounds_min_y').value
        self.bounds_max_y = self.get_parameter('bounds_max_y').value
        self.sensor_range_m = self.get_parameter('sensor_range_m').value
        self.top_k_candidates = self.get_parameter('top_k_candidates').value

        qos = QoSProfile(depth=5)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.durability = QoSDurabilityPolicy.VOLATILE

        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.map_cb, qos)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.latest_map = None
        self.exploring = False
        self.failed_goals = []       # list of (x, y) we shouldn't retry near
        self.visited_frontier_count = 0
        self.goal_queue = []         # ordered list of (x, y, yaw) still to visit this "sweep"
        self.stall_count = 0         # consecutive cycles where no candidate was selectable
        self.declared_complete = False
        self.stall_limit = 5         # ~10s of no-progress before declaring complete

        self.timer = self.create_timer(2.0, self.explore_step)

        self.get_logger().info(
            f'Frontier explorer up. Listening on {self.map_topic}. '
            f'Bounds x[{self.bounds_min_x},{self.bounds_max_x}] '
            f'y[{self.bounds_min_y},{self.bounds_max_y}]')

    def map_cb(self, msg: OccupancyGrid):
        self.latest_map = msg

    def get_robot_pose_xy(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.global_frame, self.base_frame, rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except Exception as e:
            self.get_logger().warn(f'tf lookup {self.global_frame}->{self.base_frame} failed: {e}')
            return None

    def explore_step(self):
        if self.exploring:
            return  # a NavigateToPose goal is already in flight

        # Drain the current visitation queue before doing any new detection work --
        # this is what makes the robot commit to a locally coherent sweep instead of
        # re-planning myopically after every single goal.
        if self.goal_queue:
            next_goal = self.goal_queue.pop(0)
            self.send_nav_goal(next_goal)
            return

        if self.latest_map is None:
            self.get_logger().info('waiting for map...', throttle_duration_sec=5.0)
            return

        robot_xy = self.get_robot_pose_xy()
        if robot_xy is None:
            return

        grid_msg = self.latest_map
        w, h = grid_msg.info.width, grid_msg.info.height
        data = np.array(grid_msg.data, dtype=np.int8).reshape(h, w)

        clusters = self.detect_frontier_clusters(grid_msg, data)
        if not clusters:
            self.announce_complete(data, 'no frontier clusters remain')
            return

        candidates = self.score_candidates(clusters, data, grid_msg, robot_xy)
        if not candidates:
            self.stall_count += 1
            if self.stall_count >= self.stall_limit:
                self.announce_complete(
                    data, f'{len(clusters)} frontier cluster(s) remain but none have '
                          f'been reachable/selectable for {self.stall_count} consecutive checks')
            return
        self.stall_count = 0
        self.declared_complete = False

        # Take the top-K by utility, then chain them into a short visitation
        # order via greedy nearest-neighbor from the robot's current pose --
        # a lightweight stand-in for a proper coverage-path TSP ordering.
        top = sorted(candidates, key=lambda c: c['utility'], reverse=True)[:self.top_k_candidates]
        ordered = self.nn_chain_order(top, robot_xy)
        self.goal_queue = [(c['x'], c['y'], c['yaw']) for c in ordered]

        self.get_logger().info(
            f'Planned a {len(self.goal_queue)}-viewpoint sweep from '
            f'{len(candidates)} candidates (of {len(clusters)} frontier clusters).')

    def compute_coverage_pct(self, data):
        """Fraction of the grid that is known (free or occupied) vs still unknown --
        the real ground-truth completion metric, independent of frontier-cluster edge cases."""
        total = data.size
        if total == 0:
            return 0.0
        known = np.count_nonzero(data != UNKNOWN)
        return 100.0 * known / total

    def announce_complete(self, data, reason):
        if self.declared_complete:
            return  # already announced, don't spam every 2s
        self.declared_complete = True
        pct = self.compute_coverage_pct(data)
        self.get_logger().info(
            f'Exploration complete: {reason}. '
            f'{self.visited_frontier_count} viewpoint(s) visited. '
            f'Map coverage (known/unknown cells within current grid): {pct:.1f}%.')

    # ---------- frontier detection ----------

    def detect_frontier_clusters(self, grid_msg: OccupancyGrid, data):
        h, w = data.shape
        free_mask = (data == FREE)
        unknown_mask = (data == UNKNOWN)

        frontier_mask = np.zeros_like(free_mask)
        padded = np.pad(unknown_mask, 1, mode='constant', constant_values=False)
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                shifted = padded[1 + dy:1 + dy + h, 1 + dx:1 + dx + w]
                frontier_mask |= (free_mask & shifted)

        visited = np.zeros_like(frontier_mask)
        clusters = []
        ys, xs = np.nonzero(frontier_mask)
        frontier_cells = set(zip(ys.tolist(), xs.tolist()))

        for start in frontier_cells:
            if visited[start]:
                continue
            q = deque([start])
            visited[start] = True
            cluster = []
            while q:
                cy, cx = q.popleft()
                cluster.append((cy, cx))
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        if dx == 0 and dy == 0:
                            continue
                        ny, nx = cy + dy, cx + dx
                        if (ny, nx) in frontier_cells and not visited[ny, nx]:
                            visited[ny, nx] = True
                            q.append((ny, nx))
            if len(cluster) >= self.min_frontier_cells:
                clusters.append(cluster)

        return clusters

    def in_bounds(self, wx, wy):
        return (self.bounds_min_x <= wx <= self.bounds_max_x and
                self.bounds_min_y <= wy <= self.bounds_max_y)

    def has_clearance(self, data, grid_msg, wx, wy, min_clearance_m):
        res = grid_msg.info.resolution
        ox = grid_msg.info.origin.position.x
        oy = grid_msg.info.origin.position.y
        h, w = data.shape
        col = int((wx - ox) / res)
        row = int((wy - oy) / res)
        r_cells = max(1, int(math.ceil(min_clearance_m / res)))
        r0, r1 = max(0, row - r_cells), min(h, row + r_cells + 1)
        c0, c1 = max(0, col - r_cells), min(w, col + r_cells + 1)
        if r1 <= r0 or c1 <= c0:
            return False
        window = data[r0:r1, c0:c1]
        return not np.any(window >= OCC_THRESHOLD)

    def find_standoff_goal(self, data, grid_msg, cluster, robot_xy):
        """Search for a safe standoff point near the frontier centroid, pulled back
        into free space. Tries multiple angular directions around the centroid (not
        just straight back toward the robot) so a frontier in a narrow gap between
        obstacles still gets a fair chance -- a single blocked line-of-retreat no
        longer kills the whole cluster."""
        wx, wy = self.cluster_centroid_world(cluster, grid_msg)
        rx, ry = robot_xy
        dx, dy = rx - wx, ry - wy
        dist_to_robot = math.hypot(dx, dy)
        base_angle = math.atan2(dy, dx) if dist_to_robot > 1e-3 else 0.0

        # try the direct line back toward the robot first (cheapest, usually best),
        # then fan out to either side in case that line is blocked but the gap
        # is still passable from a slightly different angle
        angle_offsets = [0.0, 0.4, -0.4, 0.8, -0.8, 1.2, -1.2, 1.6, -1.6]
        standoffs = (GOAL_STANDOFF_M, GOAL_STANDOFF_M * 0.6, GOAL_STANDOFF_M * 0.3, 0.0)

        for angle_off in angle_offsets:
            angle = base_angle + angle_off
            ux, uy = math.cos(angle), math.sin(angle)
            for standoff in standoffs:
                gx = wx + ux * standoff
                gy = wy + uy * standoff
                if self.in_bounds(gx, gy) and self.has_clearance(data, grid_msg, gx, gy, MIN_OBSTACLE_CLEARANCE_M):
                    yaw = math.atan2(wy - gy, wx - gx)
                    return (gx, gy, yaw)
        return None

    def cluster_centroid_world(self, cluster, grid_msg: OccupancyGrid):
        res = grid_msg.info.resolution
        ox = grid_msg.info.origin.position.x
        oy = grid_msg.info.origin.position.y
        ys = [c[0] for c in cluster]
        xs = [c[1] for c in cluster]
        mean_row = sum(ys) / len(ys)
        mean_col = sum(xs) / len(xs)
        wx = ox + (mean_col + 0.5) * res
        wy = oy + (mean_row + 0.5) * res
        return wx, wy

    # ---------- information-gain scoring ----------

    def compute_information_gain(self, data, grid_msg, wx, wy):
        """Raycast a synthetic 360-degree sensor sweep from (wx, wy) and count how
        many currently-UNKNOWN cells would become observable (rays stop at the
        first occupied cell or at sensor_range_m). This is a real visibility-based
        utility estimate, not just a proxy like frontier cluster pixel count."""
        res = grid_msg.info.resolution
        ox = grid_msg.info.origin.position.x
        oy = grid_msg.info.origin.position.y
        h, w = data.shape
        col0 = (wx - ox) / res
        row0 = (wy - oy) / res
        max_range_cells = int(self.sensor_range_m / res)

        gain = 0
        for i in range(NUM_RAYS):
            angle = 2.0 * math.pi * i / NUM_RAYS
            dx = math.cos(angle)
            dy = math.sin(angle)
            for step in range(1, max_range_cells + 1):
                c = int(round(col0 + dx * step))
                r = int(round(row0 + dy * step))
                if not (0 <= r < h and 0 <= c < w):
                    break
                val = data[r, c]
                if val >= OCC_THRESHOLD:
                    break  # ray blocked by an obstacle
                if val == UNKNOWN:
                    gain += 1
        return gain

    def score_candidates(self, clusters, data, grid_msg, robot_xy):
        rx, ry = robot_xy
        candidates = []
        n_blacklisted = 0
        n_no_clearance = 0
        n_too_close = 0
        n_out_of_bounds = 0

        for cluster in clusters:
            cx, cy = self.cluster_centroid_world(cluster, grid_msg)
            if not self.in_bounds(cx, cy):
                n_out_of_bounds += 1
                continue

            goal = self.find_standoff_goal(data, grid_msg, cluster, robot_xy)
            if goal is None:
                n_no_clearance += 1
                continue
            gx, gy, yaw = goal

            if self.is_blacklisted(gx, gy):
                n_blacklisted += 1
                continue
            dist = math.hypot(gx - rx, gy - ry)
            if dist < 0.3:
                n_too_close += 1
                continue

            gain = self.compute_information_gain(data, grid_msg, gx, gy)
            utility = gain / ((dist + 0.5) ** UTILITY_DISTANCE_ALPHA)
            candidates.append({
                'x': gx, 'y': gy, 'yaw': yaw,
                'gain': gain, 'dist': dist, 'utility': utility,
                'cluster_size': len(cluster),
            })

        if not candidates and clusters:
            self.get_logger().info(
                f'{len(clusters)} frontier cluster(s) found but none selectable: '
                f'{n_out_of_bounds} outside world bounds, '
                f'{n_no_clearance} failed obstacle clearance, '
                f'{n_blacklisted} blacklisted from prior failures, '
                f'{n_too_close} already at robot position.')

        return candidates

    def nn_chain_order(self, candidates, robot_xy):
        """Cheap greedy nearest-neighbor chain over a small candidate set -- a
        lightweight stand-in for a proper coverage-path TSP ordering. Cost is
        negligible since len(candidates) <= top_k_candidates (~5)."""
        remaining = list(candidates)
        ordered = []
        cur = robot_xy
        while remaining:
            remaining.sort(key=lambda c: math.hypot(c['x'] - cur[0], c['y'] - cur[1]))
            nxt = remaining.pop(0)
            ordered.append(nxt)
            cur = (nxt['x'], nxt['y'])
        return ordered

    def is_blacklisted(self, x, y):
        for (bx, by) in self.failed_goals:
            if math.hypot(x - bx, y - by) < REPLAN_ON_FAILURE_BLACKLIST_M:
                return True
        return False

    # ---------- nav2 goal dispatch ----------

    def send_nav_goal(self, xyyaw):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('navigate_to_pose action server not available yet')
            return

        wx, wy, yaw = xyyaw
        pose = PoseStamped()
        pose.header.frame_id = self.global_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = wx
        pose.pose.position.y = wy
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.exploring = True
        self.get_logger().info(
            f'Sending exploration goal: ({wx:.2f}, {wy:.2f}, yaw={math.degrees(yaw):.0f}deg) '
            f'[{len(self.goal_queue)} more queued]')
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(lambda f: self.goal_response_cb(f, (wx, wy)))

    def goal_response_cb(self, future, xy):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by nav2')
            self.failed_goals.append(xy)
            self.exploring = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self.goal_result_cb(f, xy))

    def goal_result_cb(self, future, xy):
        status = future.result().status
        if status != 4:  # 4 = SUCCEEDED
            self.get_logger().warn(f'Goal to {xy} did not succeed (status={status}), blacklisting')
            self.failed_goals.append(xy)
            self.goal_queue.clear()  # abandon the rest of this sweep, replan fresh next step
        else:
            self.get_logger().info(f'Reached frontier goal {xy}')
            self.visited_frontier_count += 1
        self.exploring = False


def main():
    rclpy.init()
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
