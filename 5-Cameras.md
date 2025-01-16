# LiDAR-experiments
Learning steps for LiDAR usage and its possibilities in conjunction with another sensors

# General Cameras

## Summary

* [1. Cameras arrangement and setup](#section-1)
* [2. Take pictures](#section-2)
    * [21. ffmpeg](#section-21)
    * [22. fswebcam](#section-22)
    * [23. raspistill](#section-23)
* [3. Video Stream](#section-3)
    * [31. v4l2rtspserver](#section-31)
    * [32. raspivid_mjpeg_server](#section-32)
* [4. More about multi cameras usage](#section-4)
    * [4.1 Check device path of your cameras](#section-41)
    * [4.2 Run photo captures in loop](#section-42)
    * [4.3 Run concurrent video captures](#section-43)
* [5. ROS publisher for camera](#section-5)
* [6. ROS subscriber through Virtual Machine](#section-6)
* [7. Cameras calibration](#section-7)

## <a name="section-1"></a> 1. Cameras arrangement and setup

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

### SMB folder Share

Create a folder to put your images and share it via smb protocol:

```shell
ubuntu@ubiquityrobot:~$ cd ~ && mkdir raspfotos/ 
ubuntu@ubiquityrobot:~$ sudo chmod 777 -R raspfotos/ && sudo chown -R nobody.nogroup raspfotos 
ubuntu@ubiquityrobot:~$ sudo apt update && sudo apt install samba samba-common-bin
ubuntu@ubiquityrobot:~$ sudo nano /etc/samba/smb.conf
# search for and edit
workgroup = RASPFOTOS
# Add to the bottom
[raspfotos]
    comment = Ubuntu File Server Share
    path = /home/ubuntu/raspfotos
    browsable = yes
    writeable = yes
    guest ok = yes
    read only = no
    create mask = 0777
    directory mask = 0777
    public = yes
ubuntu@ubiquityrobot:~$ sudo service smbd restart
```

Now access remote folder typing `\\10.42.0.1` on the file path bar of the Windows Explorer or whatever IP address you are accessing your Pi. If you are on Linux, type `smb://10.42.0.1` on the File Explore filepath bar.

<img src="imgs/Explorer.png">

## <a name="section-2"></a> 2. Take pictures

### <a name="section-21"></a> `fswebcam`

```shell
ubuntu@ubiquityrobot:~$ sudo apt update && sudo apt install fswebcam
ubuntu@ubiquityrobot:~$ fswebcam --no-banner imageX.jpg
fswebcam --no-banner imageX.jpg
--- Opening /dev/video0...
Trying source module v4l2...
/dev/video0 opened.
No input was specified, using the first.
--- Capturing frame...
Captured frame in 0.00 seconds.
--- Processing captured image...
Disabling banner.
Writing JPEG image to 'imageX.jpg'.
```

Notice the `/dev/video0`, you can choose the camera by change the device name with:

```shell
ubuntu@ubiquityrobot:~$ fswebcam --no-banner --device /dev/video0 image0.jpg
```

### <a name="section-22"></a> `ffmeg`

```shell
ubuntu@ubiquityrobot:~$ sudo apt update && sudo apt install ffmpeg
ubuntu@ubiquityrobot:~$ ffmpeg -y -video_size 1280x720 -i /dev/video0 -update 1 ~/webcam.jpg
```

`-update 1`: Enable in place update of image file for each video output frame, the ffmpeg command will run until interrupted with Ctrl+c or killed, this option generates better images, instead you can use `-frames:v 1` to generate a single frame file, but the result is more darker in low lights environments. 

To generate a video try:

```shell
ubuntu@ubiquityrobot:~$ ffmpeg -f v4l2 -framerate 10 -i /dev/video0 -c:v libx264 -t 20 -r 10 -pix_fmt yuv420p -s 640x480 raspfotos/output.mp4
...
frame=74 fps=13 q=25.0 size=0kB time=00:00:02.10 bitrate=0.2kbits/s dup=0 drop=98 speed=0.359x
...
frame=125 fps=11 q=25.0 size=256kB time=00:00:07.20 bitrate=291.3kbits/s dup=0 drop=172 speed=0.659x
...
frame=200 fps=9.6 q=-1.0 Lsize=1135kB time=00:00:19.70 bitrate=472.0kbits/s dup=0 drop=282 speed=0.947x
...
```

The options are:

- `-f v4l2` specifies the video input format. `v4l2` is the standard for video capture devices on Linux.
- `-t 20` limits the recording duration to 20 seconds.
- `-i /dev/video0` sets the video input source. `video0` is usually the default webcam device. Replace this with the correct device path if different.
- `-c:v libx264` uses the H.264 codec for video encoding.
- `-pix_fmt yuv420p` ensures compatibility with a wide range of players.
- `output.mp4` is the name of the output file.
- `-framerate 10` before the input device specifies the frame rate of the input device to 10 frames per second (fps). Adjust this to match your device’s frame rate capability (common frame rates are 15, 24, or 30 fps).
- `-r 10` after the input device sets the output video frame rate to 10 fps, which should match the input frame rate for smooth playback.
- `-s 640x480` sets the resolution to 640x480 pixels.

Check the relatively steady (desired) 10 fps at 640x480. Trying to increase resolution to 1920x1080 lead to near 3 fps and a lag video. It all depends on the underlying hardware.

### <a name="section-23"></a> `raspstill`

For Raspberry Pi cameras only:

```shell
ubuntu@ubiquityrobot:~$ raspistill -rot 180 -w 1920 -h 1080 -o cam.jpg
```

- `-rot 180` to rotate the image 180 degrees. Rhe same as `-vf -hf` (vertical flip and hosrinzotal flip)
- `-w 1920` is the photo width.
- `-h 1080` is the photo height.
- `-o cam.jpg` is the output filename.

```shell
ubuntu@ubiquityrobot:~$ raspistill -tl 1 -t 5000 -o cam_%04d.jpg
```

- `-t`: Time (in ms) before takes picture and shuts down (if not specified, set to 5000ms)
- `-tl`: Timelapse mode. Takes a picture every <t>ms. %d == frame number.
 

More options with: `raspistill --help`.

## <a name="section-3"></a> Stream video

### <a name="section-31"></a> `v4l2rtspserver`

We will setup here a Real Time Streaming Protocol (RTSP) server for our cameras, let's start by installing the [v4l2rtspserver](https://github.com/mpromonet/v4l2rtspserver), we will have to go back to version 0.2.4 due to compatibility issues with our Ubuntu 20.04:

```shell
ubuntu@ubiquityrobot:~$ wget https://github.com/mpromonet/v4l2rtspserver/releases/download/v0.2.4/v4l2rtspserver-0.2.4-Linux-armv7.deb
ubuntu@ubiquityrobot:~$ apt install ./v4l2rtspserver-0.2.4-Linux-armv7.deb
# run the server in background with
ubuntu@ubiquityrobot:~$ v4l2rtspserver &
```

The default options will generate a stream of `/dev/video0` at `rtsp://192.168.1.20:8554/unicast`. You can access it with any RTSP client (i.e [VLC](https://www.videolan.org/vlc/)). Also, you can try RTSP clients on [Android](https://play.google.com/store/apps/details?id=pl.huczeq.rtspplayer) or iOS.

Kill RTSP server with:

```shell
ubuntu@ubiquityrobot:~$  ps -aux | grep v4l2rtspserver
ubuntu   14704  ...  v4l2rtspserver
ubuntu@ubiquityrobot:~$ sudo kill 14704
```
### <a name="section-32"></a> `raspivid_mjpeg_server`

A better way to run a web server for our cameras and with low lattency is [raspivid_mjpeg_server](https://github.com/kig/raspivid_mjpeg_server), install and run the package with:

```shell
# Install Rust if needed
ubuntu@ubiquityrobot:~$ curl https://sh.rustup.rs -sSf | sh
ubuntu@ubiquityrobot:~$ . ~/.cargo/env 

# Clone the repo and start the server (Rust will build automatically ~12min)
ubuntu@ubiquityrobot:~$ git clone https://github.com/kig/raspivid_mjpeg_server
ubuntu@ubiquityrobot:~$ cd raspivid_mjpeg_server
ubuntu@ubiquityrobot:~/raspivid_mjpeg_server$ $ raspivid -ISO 0 -t 0 -n -o - -w 1280 -h 720 -fps 25 -b 25000000 -cd MJPEG | cargo run --release
...
   Compiling hyper v0.13.5
   Compiling raspivid_mjpeg_server v0.2.0 (/home/ubuntu/raspivid_mjpeg_server)
    Finished `release` profile [optimized] target(s) in 5m 42s
     Running `target/release/raspivid_mjpeg_server`
Listening on http://0.0.0.0:8554
```

Open [http://10.42.0.1:8554/video.mjpg](http://10.42.0.1:8554/video.mjpg). Kill with Ctrl+c.

## <a name="section-4"></a> 4. More about multi cameras usage

### <a name="section-41"></a> 4.1 Check device path of your cameras

On Linux, every peripheral shows up at `/dev` folder choosed by OS at start up, sometimes they can change if you unplug and plug them back.

```shell
# Device unplugged
ubuntu@ubiquityrobot:~$ ls /dev > nousb
# Device plugged
ubuntu@ubiquityrobot:~$ ls /dev > withusb
# Check differences
ubuntu@ubiquityrobot:~$ diff nousb withusb
35a36,37
> media0
> media1
173a176,177
> video0
> video1
```

We have the path for our 2 USB camera, `/dev/video0` and `/dev/video1`. The CSI camera is more trick to find.

```shell
ubuntu@ubiquityrobot:~$ echo $(ls /dev | grep video)
video0 video1 video10 video11 video12 video13 video14 video15 video16 video4
```

Other great option is `v4l2-ctl`:

```shell
ubuntu@ubiquityrobot:~$ sudo apt install v4l-utils
ubuntu@ubiquityrobot:~$ v4l2-ctl --list-devices
```

When you find it, remember these paths. They are useful in programs like `fswebcam`, `ffmpeg` or `opencv` to take photos from a specific camera or all of them.

### <a name="section-42"></a> 4.2 Run photo captures in loop

```shell
for device in $(ls /dev | grep video)
do
fswebcam --no-banner --device /dev/$device raspfotos/image-$device.jpg
done
```

Check the program above. We are saving the photos names with the device path. Now our missing camera showed up, in our case, `/dev/video3`. You can save it, set the execute permission and run it or just do this single line command:

```shell
ubuntu@ubiquityrobot:~$ for device in $(ls /dev | grep video); do fswebcam --no-banner --device /dev/$device raspfotos/image-$device.jpg; done
```

### <a name="section-43"></a> 4.3 Run concurrent video captures

Write this code into `capturevideos.sh` file:

```shell
#!/bin/bash
FPS=10
TIME=20
INOPTIONS="-hide_banner -f v4l2 -framerate $FPS"
OUTOPTIONS="-c:v libx264 -t $TIME -r $FPS -pix_fmt yuv420p -s 640x480"
# Capture videos from multiple devices in parallel
ffmpeg $INOPTIONS -i /dev/video0 $OUTOPTIONS -loglevel quiet raspfotos/output0.mp4 & 
ffmpeg $INOPTIONS -i /dev/video1 $OUTOPTIONS -loglevel quiet raspfotos/output1.mp4 & 
ffmpeg $INOPTIONS -i /dev/video3 $OUTOPTIONS -loglevel quiet raspfotos/output3.mp4 & 
ffmpeg $INOPTIONS -i /dev/video5 $OUTOPTIONS -loglevel info raspfotos/output5.mp4
# Wait for all background processes to finish
wait
echo "Video capture completed!"
```

Give execution permissions and run it:

```shell
ubuntu@ubiquityrobot:~$ chmod +x capture_videos.sh
ubuntu@ubiquityrobot:~$ ./capture_videos.sh
```

Here, we stdout only the log of the last device. Check the decrease in nominal fps at the end when all cameras are on and buggy 1 sec files generated at random runs. In general, Raspberry Pi 3B had better generated files when handling only 2 cameras.

## <a name="section-5"></a> 5. Get cameras specs

```shell
# Check possible controls with:
ubuntu@ubiquityrobot:~$ v4l2-ctl --device /dev/video0 -L
```

|                 |PC_CAM_OV1320_V1.1| DH-0918B        |Pi camera NOIR 2.1 | FH8852         |
| -------------   | -------------    | -------------   | -------------     |-------------   |
| Position        |  Upper Right     | Upper Left      |  Center Right     |  Center Left   |
| Name            |  webcam: Webcam  | USB2.0 PC CAMERA| Pi camera NOIR 2.1|Full HD webcam  |
| Driver name     |  uvcvideo        | uvcvideo        | bm2835 mmal       | uvcvideo       |
| Image Res       |  2048x1536       |  1920x1080      | 3280 x 2464       | 1920x1080      |
|Possible controls <br> at capture time|  brightness (int): 1 -255<br> contrast (int): 1 -255<br> saturation (int): 1 -255<br> white_balance_temperature_auto (bool):<br> gain (int): 1 -100<br> power_line_frequency (menu): 0 -2<br> white_balance_temperature (int): 2800 -6500<br> sharpness (int): 1 -255<br> exposure_auto (menu): 0 -3<br> exposure_absolute (int): 5 -2500<br> exposure_auto_priority (bool)| brightness  (int) : 0 -255 <br> contrast  (int) : 0 -255 <br> saturation  (int): 0 -255 <br> hue  (int) : -127 -127 <br>gamma (int) : 1 -8 <br>power_line_frequency  (menu): 0 -2<br>sharpness  (int) : 0 -15 <br>backlight_compensation  (int) : 1 -5  |brightness (int) : 0 -100 <br> contrast (int) : -100 -100<br> saturation (int) : -100 -100 <br> red_balance (int) : 1 -7999 <br> blue_balance (int) : 1 -7999<br> horizontal_flip (bool) : <br> vertical_flip (bool) :<br> power_line_frequency (menu) : 0 -3 <br> sharpness (int) : -100 -100  r<br> color_effects (menu) : 0 -15<br> rotate (int) : 0 -360 <br> color_effects_cbcr (int) : 0 -65535 <br>Codec Controls:<br> video_bitrate_mode (menu): 0 -1 <br> video_bitrate (int): 25000 -25000000 step=25000<br> repeat_sequence_header (bool): <br> h264_i_frame_period (int): 0 -2147483647 <br> h264_level (menu): 0 -11 <br> h264_profile (menu): 0 -4 <br><br>Camera Controls:<br>auto_exposure (menu): 0 -3 <br> exposure_time_absolute (int): 1 -10000<br> exposure_dynamic_framerate (bool): <br> auto_exposure_bias (intmenu): 0 -24 <br> white_balance_auto_preset (menu): 0 -10 <br> image_stabilization (bool): <br> iso_sensitivity (intmenu): 0 -4 <br> iso_sensitivity_auto (menu): 0 -1 <br> exposure_metering_mode (menu): 0 -2 <br> scene_mode (menu): 0 -13 <br><br> JPEG Compression Controls:<br> compression_quality (int): 1 -100 | brightness 0x00980900 (int) : 0 -255 step=1 default=128 value=153<br> contrast (int) : 0 -255<br> saturation (int) : 0 -255<br> hue (int) : 0 -255<br> white_balance_temperature_auto (bool) : <br> gamma (int) : 0 -255<br> gain (int) : 0 -255<br> power_line_frequency (menu) : 0 -2<br> white_balance_temperature (int) : 2800 -6500<br> sharpness (int) : 0 -255<br> backlight_compensation (int) : 0 -2<br> exposure_auto (menu) : 0 -3<br> exposure_absolute (int) : 3 max=2047 |

Notice that one of our cameras is a Raspberry Cam 2.1. The published specs are ([source](https://www.raspberrypi.com/documentation/accessories/camera.html)):

|                     |Camera Module v2| 
| -------------       | -------------    | 
| Weight              | 3g               | 
| Still resolution    | 8 MP             | 
| Video modes         | 1080p47, 1640 × 1232p41 and 640 × 480p206    | 
| Sensor              |  Sony IMX219     | 
| Sensor resolution   | 3280 × 2464 pixels    | 
| Sensor image area   |  3.68 × 2.76 mm (4.6 mm diagonal)    | 
| Sensor width        |  3.68 mm         | 
| Pixel size          |  1.12 µm × 1.12 µm    | 
| Optical size        | 1/4"             | 
| Focus               | Adjustable       | 
| Depth of field      | Approx 10 cm to ∞| 
| Focal length        | 3.04 mm          | 
| Horizontal Field of View (FoV)   | 62.2 degrees    | 
| Vertical Field of View (FoV)     | 48.8 degrees    | 
| Focal ratio (F-Stop)             | F2.0            | 
| Maximum exposure time (seconds)  | 11.76           | 

One can calculate sensor width with this formula:

$S_{width} = SensorResolution_x * PixelSize_x= 3280 * 1.12 \approx 3680 µm = 3.68 mm $

This value can be achieved knowing the Dots Per Inch (`dpi`), using `exiftool` one can extract this information from a photo:

```shell
ubuntu@ubiquityrobot:~$ exiftool cam1.jpg | grep Resolution
X Resolution                    : 72
Y Resolution                    : 72
Resolution Unit                 : inches
```

Therefore 72dpi is: 

$ 72 pixels/inch = 72 pixels/0.0254 µm = 2834 pixels/µm \rightarrow 0.00035 µm /pixel $

## <a name="section-6"></a> 6. ROS publisher for camera

Create a ROS Package dir `~/camera_ws`:

```shell
ubuntu@ubiquityrobot:~$ mkdir -p ~/camera_ws/src/camera_publisher/scripts
ubuntu@ubiquityrobot:~$ cd ~/camera_ws/src
ubuntu@ubiquityrobot:~$ catkin_create_pkg camera_publisher rospy std_msgs sensor_msgs cv_bridge
```

Write the Camera Publisher Script:

```shell
ubuntu@ubiquityrobot:~$ cd ~/camera_ws/src/camera_publisher/scripts
ubuntu@ubiquityrobot:~$ touch camera_publisher.py
ubuntu@ubiquityrobot:~$ chmod +x camera_publisher.py
```

Edit the script (`camera_publisher.py`) with the following content:

```Python
#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def camera_publisher():
    # Initialize the ROS node
    rospy.init_node('camera_publisher', anonymous=True)
    # Create a publisher for the Image topic
    image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
    # Create a CvBridge object
    bridge = CvBridge()

    # Open the default camera
    cap = cv2.VideoCapture(0)

    # Check if the camera opened successfully
    if not cap.isOpened():
        rospy.logerr("Failed to open the camera!")
        return

    rospy.loginfo("Camera publisher started.")

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        # Capture a frame from the camera
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn("Failed to capture a frame!")
            continue

        # Convert the OpenCV image to a ROS Image message
        image_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")

        # Publish the image message
        image_pub.publish(image_msg)

        # Sleep to maintain the rate
        rate.sleep()

    # Release the camera when done
    cap.release()

if __name__ == '__main__':
    try:
        camera_publisher()
    except rospy.ROSInterruptException:
        pass
```

Build your catkin workspace:

```shell
ubuntu@ubiquityrobot:~$ cd ~/camera_ws && catkin_make
ubuntu@ubiquityrobot:~$ cd ~/camera_ws && source devel/setup.bash
ubuntu@ubiquityrobot:~$ echo "source ~/camera_ws/devel/setup.bash" >> ~/.bashrc
```

Run the camera publisher node:

```shell
ubuntu@ubiquityrobot:~$ rosrun camera_publisher camera_publisher.py
[INFO] [1731688405.271017]: Camera publisher started.
```

Verify the topic is being published:

```shell
ubuntu@ubiquityrobot:~$ rostopic list
/camera/image_raw
...
```

## <a name="section-7"></a> 7. ROS subscriber through Virtual Machine

An easy way to install updated version of ROS that listen to our publisher is by a virtual machine. Install Virtual Box. Get an [Ubuntu](https://ubuntu.com/download/desktop) image. Then install ROS Noetic on it.

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
/camera/image_raw
...
ub20@ub20-VM:~$ rostopic echo /camera/image_raw
...
53, 160, 161, 154, 161, 162, 154, 158, 160, 155, 159, 161, 154, 158, 160, 153, 157, 159, 151, 161, 160, 152, 162, 161, 156, 163, 163, 154, 161, 161, 154, 161, 162, 156, 163, 164, 156, 160, 162, 156, 160, 162, 155, 162, 162, 153, 160, 160, 154, 161, 161, 154, 161, 161, 156, 161, 161, 156, 161, 161, 158, 163, 163, 159, 164, 164, 157, 163, 161, 157, 163, 161, 156, 161, 161, 156, 161, 161, 153, 160, 160, 152, 159, 159, 152, 159, 159, 152, 159, 159]
---
```

Press Ctrl+c to stop the message. 2nd vizualize some data:

```shell
ub20@ub20-VM:~$ rqt_image_view
```

<img src="imgs\CameraPublisher.png">

## <a name="section-6"></a> 6. Cameras calibration

The theory can be found at [Simple stereo model and camera calibration process](Theory-SimpleStereo.ipynb). Here we present the camera intrinsinc parameters of each one used:

|        |PC_CAM_OV1320_V1.1| DH-0918B |Pi camera NOIR 2.1 | FH8852   |
| -------| -------------    | ---------| -------------     |----------|
| $f_x$  |  2100            | 2175     | 1551              |  1494    |
| $f_y$  |  2043            | 2164     | 1486              |  1502    |
| $c_x$  |  636             | 551      | 811               |  944     |
| $c_y$  |  426             | 765      | 354               |  404     |

<!-- 
git init
git remote add origin https://github.com/HumbertoDiego/lidar-experiments
git pull origin main
#Do changes
git add * ; git commit -m "update cameras files"; git push -u origin main
 -->
