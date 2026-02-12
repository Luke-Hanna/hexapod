import ydlidar
import time
import math

PORT = "/dev/ttyUSB0"
BAUDRATE = 128000

# Initialize LiDAR
laser = ydlidar.CYdLidar()
laser.setlidaropt(ydlidar.LidarPropSerialPort, PORT)
laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, BAUDRATE)
laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
laser.setlidaropt(ydlidar.LidarPropSampleRate, 9)
laser.setlidaropt(ydlidar.LidarPropAutoReconnect, True)

if not laser.initialize():
    print("❌ Failed to initialize LiDAR")
    exit(1)

laser.turnOn()
scan = ydlidar.LaserScan()

try:
    while True:
        if laser.doProcessSimple(scan):
            points = scan.points
            if not points:
                continue

            # Create a simple 36-character radar (10° per char)
            radar = [" "] * 36
            for p in points:
                if p.range > 0:
                    angle_idx = int((p.angle % 360) / 10)
                    radar[angle_idx] = "*"

            # Print a circular-like radar
            print("".join(radar), end="\r")

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nStopping LiDAR")
finally:
    laser.turnOff()
    laser.disconnecting()
