import struct, sys, math

path = sys.argv[1] if len(sys.argv) > 1 else "LivoxMid360.mat3x4f"
with open(path, "rb") as f:
    data = f.read()

entry_size = 3 * 4 * 4  # 3x4 floats, 4 bytes each
n = len(data) // entry_size
print(f"File size: {len(data)} bytes")
print(f"Number of rays in one pattern cycle: {n}")

azimuths = []
elevations = []
for i in range(n):
    chunk = data[i*entry_size:(i+1)*entry_size]
    vals = struct.unpack("<12f", chunk)
    r00, r01, r02, _, r10, r11, r12, _, r20, r21, r22, _ = vals
    fx, fy, fz = r02, r12, r22   # was r00, r10, r20
    az = math.degrees(math.atan2(fy, fx))
    el = math.degrees(math.atan2(fz, math.sqrt(fx*fx+fy*fy)))
    azimuths.append(az)
    elevations.append(el)

print(f"Azimuth range:   {min(azimuths):.2f} to {max(azimuths):.2f} deg (span {max(azimuths)-min(azimuths):.2f})")
print(f"Elevation range: {min(elevations):.2f} to {max(elevations):.2f} deg (span {max(elevations)-min(elevations):.2f})")
