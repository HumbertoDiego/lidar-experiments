# LiDAR-experiments
Learning steps for LiDAR usage and its possibilities in conjunction with another sensors

# Kinect

## Summary

* [1. Software setup](#section-1)
    * [1.1 Raspberry Access Point](#section-11)
    * [1.2 Install ROS on Ubuntu Xenial](#section-12)
    * [1.3 Install ROS freenect package](#section-13)

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

To make the access point persistent at every boot add to `etc/rc.local` but before `exit 0`:

```shell
sudo nano /etc/rc.local

    ##
    sudo nmcli d wifi hotspot ifname wlan0 ssid raspAP password raspberry

    exit 0
```

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


<!-- 
git init
git remote add origin https://github.com/HumbertoDiego/lidar-experiments
git pull origin main
#Do changes
git add * ; git commit -m "update Kinect files"; git push -u origin main
 -->
