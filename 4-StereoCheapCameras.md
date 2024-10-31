# LiDAR-experiments
Learning steps for LiDAR usage and its possibilities in conjunction with another sensors

# Stereo Cheap Cameras

## Summary

* [1. Cameras arrangement and setup](#section-1)
* [2. Take pictures by command line](#section-2)
* [3. Cheap cameras specs](#section-3)
* [4. Cameras calibration](#section-4)

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

## <a name="section-2"></a> 2. Take pictures by command line

### `fswebcam`

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



### `ffmeg`

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

### Stream the camera over network with `v4l2rtspserver`

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

### Check device path of your cameras

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

### Run photo captures in loop

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

### Run video captures in parallel

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

## <a name="section-3"></a> 3. Cheap cameras specs

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

## <a name="section-4"></a> 4. Cameras calibration

The theory can be found at [Simple stereo model and camera calibration process](Theory-SimpleStereo.ipynb). Here we present the camera intrinsinc parameters of each one used:

|        |PC_CAM_OV1320_V1.1| DH-0918B |Pi camera NOIR 2.1 | FH8852   |
| -------| -------------    | ---------| -------------     |----------|
| $f_x$  |  2100            | 2175     | 1551              |  1494    |
| $f_y$  |  2043            | 2164     | 1486              |  1502    |
| $c_x$  |  636             | 551      | 811               |  944     |
| $c_y$  |  426             | 765      | 354               |  404     |



