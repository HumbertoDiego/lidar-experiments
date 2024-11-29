# LiDAR-experiments
Learning steps for LiDAR usage and its possibilities in conjunction with another sensors

# 3D LiDAR

## Summary

* [1. Unitree 4D L1 RM Presentation](#section-1)
* [2. Installation on Windows via USB](#section-2)
* [3. Get data via Windows SDK](#section-3)

## <a name="section-1"></a> 1. Unitree 4D L1 RM Presentation

<img src="imgs/ld1rm.jpeg">

There are a lot of 360° LIDAR out there, but not like this one, we will be using here the **Unitree 4D LiDAR L1 RM** from [Unitree Robotics](https://www.unitree.com/). It's specs:

- 4D LiDAR: 3D Position + 1D Grayscale:
- Max Range: 30M（L1 RM @90% reflectity，15M（L1 RM @10% reflectity）
- Horizontal Scanning Frequency: 11Hz
- Vertical Scanning Frequency: 180Hz
- Effective Frequency: 21600 points/s
- IMU Reporting Frequency: 250Hz

Has support for ROS1, ROS2, Python routines that run on Ubuntu and Windows. Following the manufacturer instructions to have it running. 

## <a name="section-2"></a> 2. Installation on Windows via USB

After download [Driver and Unilidar software](https://www.unitree.com/download), just plug the 4-pin serial port of the L1 and The 12V power supply into the adapter module. Then, from the adaptor to your PC. Open the Unilidar software, choose the COM port and Click on "Open Serial Port" and you should se something like this:

<img src="imgs/unilidar-software.png">

## <a name="section-3"></a> 3. Get data via Windows SDK

Download [Unilidar SDK](https://github.com/unitreerobotics/unilidar_sdk/releases), find the Windows installer and install `unitree_lidar_sdk-1.0.10-win64.exe`,  then go to `C:\Program Files\unitree_lidar_sdk 1.0.10`, open a terminal there.

Run the publisher with:

```shell
PS C:\Program Files\unitree_lidar_sdk 1.0.10> .\bin\unilidar_publisher_udp.exe COM8
Unilidar Configuration:
        serial_port = COM8
        destination_ip = 127.0.0.1
        destination_port = 12345
Unilidar initialization succeed!
Set Lidar working mode to: NORMAL ...
lidar firmware version = 1.2.3
lidar sdk version = 1.0.10
create udp socket success.
Data type size:
        sizeof(PointUnitree) = 24
        sizeof(ScanUnitree) = 2896
        sizeof(IMUUnitree) = 56
IMU message is sending!
        Data format: | uint32_t msgType | uint32_t dataSize | IMUUnitree data |
        MsgType = 101, SentSize=64, DataSize = 56
Scan message is sending!
        Data format: | uint32_t msgType| uint32_t dataSize | ScanUnitree data |
        MsgType = 102, SentSize=2904, DataSize = 2896
```

Run the subscriber with:

```shell
PS C:\Program Files\unitree_lidar_sdk 1.0.10> .\bin\unilidar_subscriber_udp.exe 12345
Input UDP Port: 12345
Unilidar Configuration:
        port = 12345
create udp socket success.
udp bind port 12345 success.
received data from 127.0.0.1:60890
msgType = 101
An IMU msg is parsed!
        stamp = 1728402436.712314, id = 149
        quaternion (x, y, z, w) = [-0.0092, 0.0045, 0.1630, 0.9849]

received data from 127.0.0.1:60890
msgType = 102
A Scan msg is parsed!
        stamp = 1728402436.709529, id = 17797
        Scan size  = 101
        first 10 points (x,y,z,intensity,time,ring) =
          (-0.019380, 0.018544, 0.044670, 151.000000, 0.000000, 0)
          (-0.011568, 0.014079, 0.045056, 155.000000, 0.000023, 0)
          (-0.019421, 0.018424, 0.046030, 156.000000, 0.000046, 0)
          (-0.019418, 0.018351, 0.046709, 158.000000, 0.000069, 0)
          (-0.011572, 0.013941, 0.046387, 159.000000, 0.000093, 0)
          (-0.011552, 0.013884, 0.046828, 159.000000, 0.000116, 0)
          (-0.019314, 0.018081, 0.048733, 158.000000, 0.000139, 0)
          (-0.019248, 0.017975, 0.049403, 156.000000, 0.000162, 0)
          (-0.019166, 0.017860, 0.050070, 156.000000, 0.000185, 0)
          (-0.019069, 0.017738, 0.050732, 155.000000, 0.000208, 0)
```

There is another option of subscriber in `examples/unilidar_subcriber_udp.py`, you can edit to get only the point cloud data and save in CSV file:

```python
import socket
import struct

# IP and Port
UDP_IP = "0.0.0.0"
UDP_PORT = 12345

# Point Type
class PointUnitree:
    def __init__(self, x, y, z, intensity, time, ring):
        self.x = x
        self.y = y
        self.z = z
        self.intensity = intensity
        self.time = time
        self.ring = ring

# Scan Type
class ScanUnitree:
    def __init__(self, stamp, id, validPointsNum, points):
        self.stamp = stamp
        self.id = id
        self.validPointsNum = validPointsNum
        self.points = points

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

# Calculate Struct Sizes
imuDataStr = "=dI4f3f3f"
imuDataSize = struct.calcsize(imuDataStr)

pointDataStr = "=fffffI"
pointSize = struct.calcsize(pointDataStr)

scanDataStr = "=dII" + 120 * "fffffI"
scanDataSize = struct.calcsize(scanDataStr)

print("pointSize = " +str(pointSize) + ", scanDataSize = " + str(scanDataSize) + ", imuDataSize = " + str(imuDataSize))

with open("x_y_z_i_t_r.csv", "w") as f:
    while True:
        # Recv data
        data, addr = sock.recvfrom(10000)
        msgType = struct.unpack("=I", data[:4])[0]

        elif msgType == 102:  # Scan Message
            print("msgType =", msgType)
            length = struct.unpack("=I", data[4:8])[0]
            stamp = struct.unpack("=d", data[8:16])[0]
            id = struct.unpack("=I", data[16:20])[0]
            validPointsNum = struct.unpack("=I", data[20:24])[0]
            pointStartAddr = 24
            for i in range(validPointsNum):
                pointData = struct.unpack(pointDataStr, data[pointStartAddr: pointStartAddr+pointSize])
                pointStartAddr = pointStartAddr + pointSize
                point = PointUnitree(*pointData)
                msg = f"{point.x}, {point.y}, {point.z}, {point.intensity}, {point.time}, {point.ring}\n"
                print(msg)
                f.write(msg)

sock.close()
```

Save this file at `examples/subcriber.py`. Give appropriate permissions on folder, run the publisher then run it with:

```shell
PS C:\Program Files\unitree_lidar_sdk 1.0.10> python .\examples\subscriber.py
pointSize = 24, scanDataSize = 2896, imuDataSize = 52
msgType = 102
-0.01041092723608017, -0.014962182380259037, 0.04488921910524368, 170.0, 0.0, 0
-0.008276902139186859, -0.006224430166184902, 0.044892363250255585, 169.0, 2.3148148102336563e-05, 0
-0.008257731795310974, -0.006226567085832357, 0.04510129988193512, 168.0, 4.6296296204673126e-05, 0
-0.008237323723733425, -0.006223335396498442, 0.045309823006391525, 166.0, 6.944444612599909e-05, 0
-0.008215712383389473, -0.006214742548763752, 0.04551779106259346, 158.0, 9.259259240934625e-05, 0
...
```

On [Cloud Compare](https://cloudcompare.org/release/index.html) check our CSV file:



