#!/usr/bin/env python3
# Author: lnotspotl
# Modified for ros2 by: abutalipovvv
import numpy as np
from enum import Enum

# Определение состояний поведения робота
class BehaviorState(Enum):
    REST = 0
    TROT = 1
    CRAWL = 2
    STAND = 3

# Класс для хранения состояния робота
class State(object):
    def __init__(self, default_height):
        # Скорость робота
        self.velocity = np.array([0., 0.])
        self.yaw_rate = 0.
        self.robot_height = -default_height

        # Позиции ног
        self.foot_locations = np.zeros((3, 4))

        # Локальная позиция и ориентация тела
        self.body_local_position = np.array([0., 0., 0.])
        self.body_local_orientation = np.array([0., 0., 0.])
        
        # IMU данные
        self.imu_roll = 0.
        self.imu_pitch = 0.
        
        # Тики и состояние поведения
        self.ticks = 0
        self.behavior_state = BehaviorState.REST

# Класс для хранения команды робота
class Command(object):
    def __init__(self, default_height):
        # Скорость робота
        self.velocity = np.array([0., 0.])
        self.yaw_rate = 0.
        self.robot_height = -default_height

        # События поведения
        self.trot_event = False
        self.crawl_event = False
        self.rest_event = False
        self.stand_event = False
