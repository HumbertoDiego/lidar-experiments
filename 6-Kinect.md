# LiDAR-experiments
Learning steps for LiDAR usage and its possibilities in conjunction with another sensors

# Kinect

## Summary

* [1. Software setup](#section-1)
    * [1.1 Raspberry Access Point](#section-11)
    * [1.2 Install ROS on Ubuntu Xenial](#section-12)
    * [1.3 Install ROS freenect package](#section-13)
* [2. Launch ROS kinetic freenect](#section-2)
* [3. Run applications through Virtual Machine](#section-3)

## <a name="section-1"></a> 1. Software setup

Kinect is one of the most popular source of PointClouds - array of points with 3D coordinates information. It has proprietary connector - actually it's USB+12V bus, and needs adapter for PC connection Despite that Kinect is bigger than ASUS Xtion, it has also tilt motor, microphone array, accelerometer and better support from ROS community. 

Requirements:

* Kinect 360
* Raspiberry Pi model 3b
* Sd card with [ubuntu-mate-16.04-desktop-armhf-raspberry-pi.img](https://releases.ubuntu-mate.org/archived/16.04/armhf/) installed
* Power supply 5V - 3A
* Internet connection plugged in via Ethernet cable (only initial steps)

### <a name="section-11"></a> 1.1 Raspberry Access Point

On Ubuntu Xenial if you want to receive an static IP address every time you connect at a certain network adapter, you will need to add at the bottom of `/etc/network/interfaces`:

```shell
sudo nano /etc/network/interfaces
    ##
    ...
    auto enxb827eb4e360a
    iface enxb827eb4e360a inet static
    address 192.168.1.20
    netmask 255.255.255.0
    gateway 192.168.1.1
    dns-nameservers 8.8.8.8 8.8.4.4
```

To use wifi to act as access point named `raspAP` with password `raspberry`:

```shell
sudo nmcli d wifi hotspot ifname wlan0 ssid raspAP password raspberry
```

The command place the raspberry as a router with IP 10.42.0.1 and also as a DHCP server assigning IPs to conected decices.

To make the access point persistent at every boot add to `etc/rc.local` but before `exit 0`:

```shell
sudo nano /etc/rc.local

    ##
    sudo nmcli d wifi hotspot ifname wlan0 ssid raspAP password raspberry

    exit 0
```

<!-- 
Optionnaly, use ethernet cable between raspberry and other computer to increase baudrate:

```shell
nmcli connection add type ethernet ifname enxb827eb4e360a con-name mycon ipv4.method shared
sudo nmcli c up mycon
```
-->

### <a name="section-12"></a> 1.2 Install ROS on Ubuntu Xenial

Configure your Ubuntu repositories to allow "restricted," "universe," and "multiverse." 

Setup your computer to accept software from packages.ros.org.

```shell
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

Set up your keys

```shell
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

ROS Installation

```shell
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
# rosinstall and rosdep
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo rosdep init
rosdep update
# Individual Packages: (replace underscores with dashes of the package name):
sudo apt-get install ros-kinetic-PACKAGE
```

Environment setup

```shell
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### <a name="section-13"></a> 1.3 Install ROS freenect package

This [package](https://wiki.ros.org/freenect_launch) contains launch files for using a Microsoft Kinect using the [libfreenect](https://openkinect.org/wiki/Getting_Started#Ubuntu/Debian) library.

Starting from Ubuntu 11.10 (Oneiric) and Debian 7 (Wheezy), Ubuntu and Debian provide official packages of libfreenect. You can install them easily in a console:

```shell
sudo apt-get install freenect
# To start the demo applications use:
freenect-glview
```

Plug your Kinect. 3 devices must be listed with:

```shell
lsusb | grep Xbox
Bus 001 Device 021: ID 045e:02ae Microsoft Corp. Xbox NUI Camera
Bus 001 Device 019: ID 045e:02b0 Microsoft Corp. Xbox NUI Motor
Bus 001 Device 020: ID 045e:02ad Microsoft Corp. Xbox NUI Audio
```

ROS freenect package:

```shell
sudo apt install ros-kinetic-freenect-launch
sudo apt install ros-kinetic-depthimage-to-laserscan
roscore
roslaunch freenect_launch freenect.launch
rviz
```

You should see a very lag 

## <a name="section-2"></a> 2. Launch ROS kinetic freenect

Remember we have setup an access point with our Raspberry? Now it's time. Connect to it:

<img src="imgs/WindowsConnectiontoRaspAP.png">

Your IP is somewhere `10.42.0.X`, the raspberry IP is `10.42.0.1`. On Windows, you can check using Powershell with:

```
> ipconfig /all
Adaptador de Rede sem Fio Wi-Fi:
...
   Endereço IPv4. . . . . . . .  . . . . . . . : 10.42.0.19(Preferencial)
...
   Gateway Padrão. . . . . . . . . . . . . . . : 10.42.0.1
...
```

Now your PC and Raspberry are in the same network you can SSH into it , check environment variables and launch ROS freenect:

```shell
> ssh ubuntu@10.42.0.1
ubuntu@10.42.0.1's password: 
ubuntu@ubuntu-desktop:~$ env | grep ROS
ROS_ROOT=/opt/ros/kinetic/share/ros
ROS_PACKAGE_PATH=/opt/ros/kinetic/share
ROS_MASTER_URI=http://localhost:11311
ROS_PYTHON_VERSION=2
ROS_VERSION=1
ROSLISP_PACKAGE_DIRECTORIES=
ROS_DISTRO=kinetic
ROS_IP=10.42.0.1
ROS_ETC_DIR=/opt/ros/kinetic/etc/ros
ubuntu@ubuntu-desktop:~$ roslaunch freenect_launch freenect.launch 
```

List all possible topics with:

```shell
ubuntu@ubuntu-desktop:~$ rostopic list
/camera/depth/camera_info
/camera/depth/disparity
/camera/depth/image
/camera/depth/image/compressed
/camera/depth/image/compressed/parameter_descriptions
/camera/depth/image/compressed/parameter_updates
/camera/depth/image/compressedDepth
/camera/depth/image/compressedDepth/parameter_descriptions
/camera/depth/image/compressedDepth/parameter_updates
/camera/depth/image/theora
/camera/depth/image/theora/parameter_descriptions
/camera/depth/image/theora/parameter_updates
/camera/depth/image_raw
/camera/depth/image_raw/compressed
/camera/depth/image_raw/compressed/parameter_descriptions
/camera/depth/image_raw/compressed/parameter_updates
/camera/depth/image_raw/compressedDepth
/camera/depth/image_raw/compressedDepth/parameter_descriptions
/camera/depth/image_raw/compressedDepth/parameter_updates
/camera/depth/image_raw/theora
/camera/depth/image_raw/theora/parameter_descriptions
/camera/depth/image_raw/theora/parameter_updates
/camera/depth/image_rect
/camera/depth/image_rect/compressed
/camera/depth/image_rect/compressed/parameter_descriptions
/camera/depth/image_rect/compressed/parameter_updates
/camera/depth/image_rect/compressedDepth
/camera/depth/image_rect/compressedDepth/parameter_descriptions
/camera/depth/image_rect/compressedDepth/parameter_updates
/camera/depth/image_rect/theora
/camera/depth/image_rect/theora/parameter_descriptions
/camera/depth/image_rect/theora/parameter_updates
/camera/depth/image_rect_raw
/camera/depth/image_rect_raw/compressed
/camera/depth/image_rect_raw/compressed/parameter_descriptions
/camera/depth/image_rect_raw/compressed/parameter_updates
/camera/depth/image_rect_raw/compressedDepth
/camera/depth/image_rect_raw/compressedDepth/parameter_descriptions
/camera/depth/image_rect_raw/compressedDepth/parameter_updates
/camera/depth/image_rect_raw/theora
/camera/depth/image_rect_raw/theora/parameter_descriptions
/camera/depth/image_rect_raw/theora/parameter_updates
/camera/depth/points
/camera/depth_rectify_depth/parameter_descriptions
/camera/depth_rectify_depth/parameter_updates
/camera/depth_registered/camera_info
/camera/depth_registered/disparity
/camera/depth_registered/hw_registered/image_rect
/camera/depth_registered/hw_registered/image_rect/compressed
/camera/depth_registered/hw_registered/image_rect/compressed/parameter_descriptions
/camera/depth_registered/hw_registered/image_rect/compressed/parameter_updates
/camera/depth_registered/hw_registered/image_rect/compressedDepth
/camera/depth_registered/hw_registered/image_rect/compressedDepth/parameter_descriptions
/camera/depth_registered/hw_registered/image_rect/compressedDepth/parameter_updates
/camera/depth_registered/hw_registered/image_rect/theora
/camera/depth_registered/hw_registered/image_rect/theora/parameter_descriptions
/camera/depth_registered/hw_registered/image_rect/theora/parameter_updates
/camera/depth_registered/hw_registered/image_rect_raw
/camera/depth_registered/hw_registered/image_rect_raw/compressed
/camera/depth_registered/hw_registered/image_rect_raw/compressed/parameter_descriptions
/camera/depth_registered/hw_registered/image_rect_raw/compressed/parameter_updates
/camera/depth_registered/hw_registered/image_rect_raw/compressedDepth
/camera/depth_registered/hw_registered/image_rect_raw/compressedDepth/parameter_descriptions
/camera/depth_registered/hw_registered/image_rect_raw/compressedDepth/parameter_updates
/camera/depth_registered/hw_registered/image_rect_raw/theora
/camera/depth_registered/hw_registered/image_rect_raw/theora/parameter_descriptions
/camera/depth_registered/hw_registered/image_rect_raw/theora/parameter_updates
/camera/depth_registered/image
/camera/depth_registered/image/compressed
/camera/depth_registered/image/compressed/parameter_descriptions
/camera/depth_registered/image/compressed/parameter_updates
/camera/depth_registered/image/compressedDepth
/camera/depth_registered/image/compressedDepth/parameter_descriptions
/camera/depth_registered/image/compressedDepth/parameter_updates
/camera/depth_registered/image/theora
/camera/depth_registered/image/theora/parameter_descriptions
/camera/depth_registered/image/theora/parameter_updates
/camera/depth_registered/image_raw
/camera/depth_registered/image_raw/compressed
/camera/depth_registered/image_raw/compressed/parameter_descriptions
/camera/depth_registered/image_raw/compressed/parameter_updates
/camera/depth_registered/image_raw/compressedDepth
/camera/depth_registered/image_raw/compressedDepth/parameter_descriptions
/camera/depth_registered/image_raw/compressedDepth/parameter_updates
/camera/depth_registered/image_raw/theora
/camera/depth_registered/image_raw/theora/parameter_descriptions
/camera/depth_registered/image_raw/theora/parameter_updates
/camera/depth_registered/points
/camera/depth_registered/sw_registered/camera_info
/camera/depth_registered/sw_registered/image_rect
/camera/depth_registered/sw_registered/image_rect/compressed
/camera/depth_registered/sw_registered/image_rect/compressed/parameter_descriptions
/camera/depth_registered/sw_registered/image_rect/compressed/parameter_updates
/camera/depth_registered/sw_registered/image_rect/compressedDepth
/camera/depth_registered/sw_registered/image_rect/compressedDepth/parameter_descriptions
/camera/depth_registered/sw_registered/image_rect/compressedDepth/parameter_updates
/camera/depth_registered/sw_registered/image_rect/theora
/camera/depth_registered/sw_registered/image_rect/theora/parameter_descriptions
/camera/depth_registered/sw_registered/image_rect/theora/parameter_updates
/camera/depth_registered/sw_registered/image_rect_raw
/camera/depth_registered/sw_registered/image_rect_raw/compressed
/camera/depth_registered/sw_registered/image_rect_raw/compressed/parameter_descriptions
/camera/depth_registered/sw_registered/image_rect_raw/compressed/parameter_updates
/camera/depth_registered/sw_registered/image_rect_raw/compressedDepth
/camera/depth_registered/sw_registered/image_rect_raw/compressedDepth/parameter_descriptions
/camera/depth_registered/sw_registered/image_rect_raw/compressedDepth/parameter_updates
/camera/depth_registered/sw_registered/image_rect_raw/theora
/camera/depth_registered/sw_registered/image_rect_raw/theora/parameter_descriptions
/camera/depth_registered/sw_registered/image_rect_raw/theora/parameter_updates
/camera/depth_registered_rectify_depth/parameter_descriptions
/camera/depth_registered_rectify_depth/parameter_updates
/camera/driver/parameter_descriptions
/camera/driver/parameter_updates
/camera/ir/camera_info
/camera/ir/image_raw
/camera/ir/image_raw/compressed
/camera/ir/image_raw/compressed/parameter_descriptions
/camera/ir/image_raw/compressed/parameter_updates
/camera/ir/image_raw/compressedDepth
/camera/ir/image_raw/compressedDepth/parameter_descriptions
/camera/ir/image_raw/compressedDepth/parameter_updates
/camera/ir/image_raw/theora
/camera/ir/image_raw/theora/parameter_descriptions
/camera/ir/image_raw/theora/parameter_updates
/camera/ir/image_rect_ir
/camera/ir/image_rect_ir/compressed
/camera/ir/image_rect_ir/compressed/parameter_descriptions
/camera/ir/image_rect_ir/compressed/parameter_updates
/camera/ir/image_rect_ir/compressedDepth
/camera/ir/image_rect_ir/compressedDepth/parameter_descriptions
/camera/ir/image_rect_ir/compressedDepth/parameter_updates
/camera/ir/image_rect_ir/theora
/camera/ir/image_rect_ir/theora/parameter_descriptions
/camera/ir/image_rect_ir/theora/parameter_updates
/camera/ir_rectify_ir/parameter_descriptions
/camera/ir_rectify_ir/parameter_updates
/camera/projector/camera_info
/camera/rgb/camera_info
/camera/rgb/image_color
/camera/rgb/image_color/compressed
/camera/rgb/image_color/compressed/parameter_descriptions
/camera/rgb/image_color/compressed/parameter_updates
/camera/rgb/image_color/compressedDepth
/camera/rgb/image_color/compressedDepth/parameter_descriptions
/camera/rgb/image_color/compressedDepth/parameter_updates
/camera/rgb/image_color/theora
/camera/rgb/image_color/theora/parameter_descriptions
/camera/rgb/image_color/theora/parameter_updates
/camera/rgb/image_mono
/camera/rgb/image_mono/compressed
/camera/rgb/image_mono/compressed/parameter_descriptions
/camera/rgb/image_mono/compressed/parameter_updates
/camera/rgb/image_mono/compressedDepth
/camera/rgb/image_mono/compressedDepth/parameter_descriptions
/camera/rgb/image_mono/compressedDepth/parameter_updates
/camera/rgb/image_mono/theora
/camera/rgb/image_mono/theora/parameter_descriptions
/camera/rgb/image_mono/theora/parameter_updates
/camera/rgb/image_raw
/camera/rgb/image_raw/compressed
/camera/rgb/image_raw/compressed/parameter_descriptions
/camera/rgb/image_raw/compressed/parameter_updates
/camera/rgb/image_raw/compressedDepth
/camera/rgb/image_raw/compressedDepth/parameter_descriptions
/camera/rgb/image_raw/compressedDepth/parameter_updates
/camera/rgb/image_raw/theora
/camera/rgb/image_raw/theora/parameter_descriptions
/camera/rgb/image_raw/theora/parameter_updates
/camera/rgb/image_rect_color
/camera/rgb/image_rect_color/compressed
/camera/rgb/image_rect_color/compressed/parameter_descriptions
/camera/rgb/image_rect_color/compressed/parameter_updates
/camera/rgb/image_rect_color/compressedDepth
/camera/rgb/image_rect_color/compressedDepth/parameter_descriptions
/camera/rgb/image_rect_color/compressedDepth/parameter_updates
/camera/rgb/image_rect_color/theora
/camera/rgb/image_rect_color/theora/parameter_descriptions
/camera/rgb/image_rect_color/theora/parameter_updates
/camera/rgb/image_rect_mono
/camera/rgb/image_rect_mono/compressed
/camera/rgb/image_rect_mono/compressed/parameter_descriptions
/camera/rgb/image_rect_mono/compressed/parameter_updates
/camera/rgb/image_rect_mono/compressedDepth
/camera/rgb/image_rect_mono/compressedDepth/parameter_descriptions
/camera/rgb/image_rect_mono/compressedDepth/parameter_updates
/camera/rgb/image_rect_mono/theora
/camera/rgb/image_rect_mono/theora/parameter_descriptions
/camera/rgb/image_rect_mono/theora/parameter_updates
/camera/rgb_debayer/parameter_descriptions
/camera/rgb_debayer/parameter_updates
/camera/rgb_rectify_color/parameter_descriptions
/camera/rgb_rectify_color/parameter_updates
/camera/rgb_rectify_mono/parameter_descriptions
/camera/rgb_rectify_mono/parameter_updates
/diagnostics
/rosout
/rosout_agg
/tf
/tf_static
```

We will subscribe to these topics later: `/camera/rgb/image_color` and `/camera/depth/image_raw`.

## <a name="section-3"></a> 3. Run applications through Virtual Machine

An easy way to install updated version of ROS that listen to our annoucer is by a virtual machine. Install Virtual Box. Get an Ubuntu 20 image. Then install ROS Noetic on it.

```shell
ub20@ub20-VM:~$ sudo wget -c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh && chmod +x ./ros_install_noetic.sh && ./ros_install_noetic.sh
```

Correct the environment variables of our VM:

```shell
ub20@ub20-VM:~$ echo "export ROS_IP=10.42.0.1" >> ~/.bashrc
ub20@ub20-VM:~$ echo "export ROS_MASTER_URI=http://10.42.0.1:11311" >> ~/.bashrc
ub20@ub20-VM:~$ source ~/.bashrc
ub20@ub20-VM:~$ env | grep ROS
ROS_VERSION=1
ROS_PYTHON_VERSION=3
ROS_PACKAGE_PATH=/home/ub20/ldlidar_ros_ws/src:/opt/ros/noetic/share
ROSLISP_PACKAGE_DIRECTORIES=/home/ub20/ldlidar_ros_ws/devel/share/common-lisp
ROS_IP=10.42.0.1
ROS_ETC_DIR=/opt/ros/noetic/etc/ros
ROS_MASTER_URI=http://10.42.0.1:11311
ROS_ROOT=/opt/ros/noetic/share/ros
ROS_DISTRO=noetic
```

Set bridge mode on the WIFI network adapter from host to VM, so it will act as another computer on the `RaspAP` network:

<img src="imgs/bridgeVM.png">

Going back to your VM, check if the IP is somewhere `10.42.0.X`, and ping Raspberry to check if it is reachable:

```shell
ub20@ub20-VM:~$ ip a | grep inet
...
    inet 10.42.0.83/24 brd 10.42.0.255 scope global dynamic noprefixroute enp0s3

ub20@ub20-VM:~$ ping 10.42.0.1
PING 10.42.0.1 (10.42.0.1) 56(84) bytes of data.
64 bytes de 10.42.0.1: icmp_seq=1 ttl=64 tempo=21.6 ms
64 bytes de 10.42.0.1: icmp_seq=2 ttl=64 tempo=10.9 ms
...
```

Now we are good to go, 1st check the Raspberry data transmission and vizualize some raw data:

```shell
ub20@ub20-VM:~$ rostopic list
# should return all topics
ub20@ub20-VM:~$ rostopic echo /camera/depth/image_raw
...
 [178, 63, 134, 235, 177, 63, 134, 235, 177, 63, 33, 176, 178, 63, 134, 235, 177, 63, 33, 176, 178, 63, 189, 116, 179, 63, 148, 24, 180, 63, 48, 221, 180, 63, 48, 221, 180, 63, 148, 24, 180, 63, 148, 24, 180, 63, 148, 24, 180, 63, 148, 24, 180, 63, 148, 24, 180, 63, 148, 24, 180, 63, 148, 24, 180, 63, 48, 221, 180, 63, 48, 221, 180, 63, 203, 161, 181, 63, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127
---
```

Press Ctrl+c to stop the message.

2nd vizualize some data:

```sehll
ub20@ub20-VM:~$ rqt_image_view
```

<img src="imgs/kinectRGB.png">

<img src="imgs/kinectDepth.png">

The depth topic send Image data type messages that has this structure (you can experiment check the authors page for more details):

```
Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)
```
To convert the data vector into an actual matrix would be painfull, but OpenCV already has a nice implementaiton of it, so we can got to our last step:

3rd run a subscriber to change data on the fly:

```python
#!/usr/bin/python3
import cv2
import rospy
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
bridge = CvBridge()

def callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    new_image = cv2.convertScaleAbs(cv_image, alpha=1, beta=10)
    cv2.imshow("Image from ROS", new_image)
    cv2.waitKey(1)  # Wait for a key press to update the image

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/camera/depth/image_raw" , Image, callback)
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    listener()
```

Save it as `subscriber.py` give permissions and run it:

```shell
ub20@ub20-VM:~$ chmod +x subscriber.py
ub20@ub20-VM:~$ ./subscriber.py
```

<img src="imgs/kinectsubscriber.png">


<!-- 
git init
git remote add origin https://github.com/HumbertoDiego/lidar-experiments
git pull origin main
#Do changes
git add * ; git commit -m "update Kinect files"; git push -u origin main
 -->
