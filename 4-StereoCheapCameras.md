# LiDAR-experiments
Learning steps for LiDAR usage and its possibilities in conjunction with another sensors

# Stereo Cheap Cameras

## Summary

* [1. Cameras Arrangement](#section-1)
* [2. Cheap Camera on left](#section-2)
* [3. Cheap Camera on right](#section-3)
* [4. Cheap Camera on center](#section-4)
* [5. Cheap Camera spare 1](#section-5)
* [6. Cheap Camera spare 2](#section-6)
* [7. Cameras Calibration](#section-7)
* [8. Cameras Rearrangement](#section-8)

## <a name="section-1"></a> 1. Cameras Arrangement

There are a lot o ~~shit~~ cheap cameras out there, how far with stereo pair quality images we can go with them? The key factor is dataset quality, in math terms, we want a lot of good keypoints found between images stereo pairs and it's next frames. Making videos with all cameras at fixed positions in relation to one another is our goal here. We start presenting the cameras arragement:

<p float="left">
<img src="imgs/arrangement.png" height="300">
<img src="imgs/setup-04.png"  height="300">
</p>

Were y-axis is the vertical, x-axis is the horizontal orthogonal to the direction of motion and z-axis is the horizontal in the direction of motion.

The initial setup is on top of a metal frame of a Brazilian Army Campaign Backpack, the computer is a [Raspberry Pi 3B](https://www.raspberrypi.com/products/). It's main purpose is to collect the images and share it's folder over the network.

Requirements:

* Raspiberry Pi model 3b
* 2 Cheap Cameras over USB
* 1 Cheap Camera over CSI
* Power Bank
* Sd card with [2023-02-09-ubiquity-base-focal-raspberry-pi.img](https://learn.ubiquityrobotics.com/noetic_pi_image_downloads) installed
* Internet connection plugged in via Ethernet cable (only initial steps)

### SSH connection to Raspberry pi

With this Ubiquity image, the wifi adapter of the Raspberry Pi acts as an Access Point, so we just need to enter `ubiquityrobotXXXX` network, where XXXX is part of the MAC address. The password is `robotseverywhere`. The sudo username is `ubuntu` with password `ubuntu`. 

In this network, just run on a shell:

```shell
> ssh ubuntu@10.42.0.1 # enter password
# Check your Raspberry IP over all networks
ubuntu@ubiquityrobot:~$ ip a | grep "inet "
    inet 127.0.0.1/8 scope host lo
    inet 192.168.1.20/24 brd 192.168.1.255 scope global enxb827ebe94c24
    inet 10.42.0.1/24 brd 10.42.0.255 scope global noprefixroute wlan0
```

Create a folder to put your images and share it 
via smb protocol:

```shell
ubuntu@ubiquityrobot:~$ cd ~ && mkdir raspfotos/ && chmod 777 -R raspfotos/
ubuntu@ubiquityrobot:~$ sudo apt update && sudo apt install samba
ubuntu@ubiquityrobot:~$ sudo nano /etc/samba/smb.conf
# Add to the bottom
[raspfotos]
    comment = Samba on Ubuntu
    path = /home/ubuntu/raspfotos
    read only = no
    browsable = yes
ubuntu@ubiquityrobot:~$ sudo service smbd restart
```

## <a name="section-2"></a> 2. Cheap Camera on left

## <a name="section-3"></a> 3. Cheap Camera on right

## <a name="section-4"></a> 4. Cheap Camera on center

## <a name="section-5"></a> 5. Cheap Camera spare 1

## <a name="section-6"></a> 6. Cheap Camera spare 2

## <a name="section-7"></a> 7. Cameras Calibration

## <a name="section-8"></a> 8. Cameras Rearrangement
