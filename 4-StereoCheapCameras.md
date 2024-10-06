# LiDAR-experiments
Learning steps for LiDAR usage and its possibilities in conjunction with another sensors

# Stereo Cheap Cameras

## Summary

* [1. Cameras arrangement and setup](#section-1)
* [2. Simple Stereo pair theory](#section-2)
* [3. Cheap cameras specs](#section-3)
* [4. Cameras calibration](#section-4)
* [5. Run OpenCV photo caputres in loop](#section-5)
* [6. Cameras rearrangement](#section-6)

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

### Take 1st picture with `fswebcam`

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

### Take 1st picture with `ffmeg`

```shell
ubuntu@ubiquityrobot:~$ sudo apt update && sudo apt install ffmpeg
ubuntu@ubiquityrobot:~$ ffmpeg -y -f v4l2 -video_size 1280x720 -i /dev/video0 -r 0.2 -qscale:v 2 -update 1 ~/webcam.jpg
```

The non trivial options of this command are `-r 0.2`: set FPS to 0.2 or 1 frame per 5 seconds,`-qscale:v 2`: set video quality [JPEG quality in this case], 2 is highest quality and `-update 1`: Enable in place update of image file for each video output frame, the ffmpeg command will run until interrupted with Ctrl+c or killed, this option generates better images, instead you can use `-frames:v 1` to generate a single frame file, but the result is more darker in low lights environments. 

### Generate 1st video with `ffmeg`

```shell
ubuntu@ubiquityrobot:~$ ffmpeg -y -f v4l2 -r 25 -video_size 640x480 -i /dev/video0 output.mkv
```

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

You will notice here, the long time to take bad resolution pictures. Next work is running these commands in parallel and set de resolution to a commom denominator of the cameras. Let's say 640x480.

```shell
# sequential photos
ubuntu@ubiquityrobot:~$ for device in video0 video1 video3; do fswebcam -r 640x480 --no-banner --device /dev/$device raspfotos/image-$device.jpg ; done
# parallel photos
ubuntu@ubiquityrobot:~$ fswebcam -r 640x480 --no-banner --device /dev/video0 raspfotos/image-video0.jpg & \
fswebcam -r 640x480 --no-banner --device /dev/video1 raspfotos/image-video1.jpg & \
fswebcam -r 640x480 --no-banner --device /dev/video3 raspfotos/image-video3.jpg 
```

The total time is almost the same but check the time delta between the `Writing JPEG image to 'raspfotos/image-video0.jpg'.` and `Writing JPEG image to 'raspfotos/image-video3.jpg'.`, they are much closer now. 

## <a name="section-2"></a> 2. Simple Stereo pair theory

### Linear Camera Model

<img src="imgs/pinhole_camera_model.png" style="background : white">

By similarity of triangles we can make the perspective projection between the <u>Camera reference</u> system and the <u>Image Plane</u>:

$$
\frac{x}{X_C}=\frac{f}{Z_c} \rightarrow x = \frac{X_cf}{Z_c} \\
\frac{y}{Y_C}=\frac{f}{Z_c} \rightarrow y = \frac{Y_cf}{Z_c} 
$$

Then, from <u>Image Plane</u> in mm to <u>Sensor Plane</u> in pixels

$$
u = f_x\frac{X_c}{Z_c} + c_x\\
v = f_y\frac{Y_c}{Z_c} + c_y\\
$$

Which in homegeneus matrix is given by:

$$
\begin{equation}
    \begin{bmatrix}
    u \\
    v \\
    1
    \end{bmatrix} \equiv
    Z_c\begin{bmatrix}
    u \\
    v \\
    1
    \end{bmatrix} =
    \begin{bmatrix}
    f_xX_c + c_xZ_c \\
    f_yY_c + c_yZ_c \\
    1
    \end{bmatrix} =
    \underbrace{
        \begin{bmatrix}
            f_x & 0 & c_x & 0 \\
            0 & f_y & c_y & 0 \\
            0 & 0 & 1 & 0
        \end{bmatrix}
    }_{Intrinsic \ parameters \ [K_{3\times 3}|0]}
    \begin{bmatrix}
    X_c \\
    Y_c \\
    Z_c \\
    1
    \end{bmatrix} 
\end{equation} 
$$

The transformation between the <u>World reference</u> system and the <u>Camera reference</u> is given by:

$$
\begin{bmatrix}
X_c \\
Y_c \\
Z_c
\end{bmatrix} = R_{3\times 3}
\begin{bmatrix}
X_w \\
Y_w \\
Z_w
\end{bmatrix} +
\begin{bmatrix}
t_x \\
t_y \\
t_z
\end{bmatrix}
$$

Which in homegeneus matrix is given by:

$$
\begin{equation}
    \begin{bmatrix}
    X_c \\
    Y_c \\
    Z_c \\
    1
    \end{bmatrix} = 
    \underbrace{
        \begin{bmatrix}
            R_{3 \times 3} & t_{3\times 1} \\
            0_{1\times 3} & 1 
        \end{bmatrix}
    }_{Extrinsinc \ parameters \ T_{4\times 4}}
    \begin{bmatrix}
    X_w \\
    Y_w \\
    Z_w \\
    1
    \end{bmatrix} 
\end{equation} 
$$

The coordinate of a point in the world P on the Sensor plane can be achieved by combining equations (1) and (2):

$$
\begin{equation}
    \begin{bmatrix}
    u \\
    v \\
    1
    \end{bmatrix}=
    \underbrace{
            \underbrace{
                \begin{bmatrix}
                    f_x & 0 & c_x & 0 \\
                    0 & f_y & c_y & 0 \\
                    0 & 0 & 1 & 0
                \end{bmatrix}
            }_{Intrinsinc \ parameters \ [K_{3\times 3}|0]}
            \underbrace{
                \begin{bmatrix}
                    R_{3 \times 3} & t_{3\times 1} \\
                    0_{1 \times 3} & 1 
                \end{bmatrix}
            }_{Extrinsinc \ parameters \ T_{4\times 4}}
    }_{P_{3\times 4}}
    \begin{bmatrix}
    X_w \\
    Y_w \\
    Z_w \\
    1
    \end{bmatrix} 
\end{equation} 
$$

The intrinsic parameters $(f_x,f_y,c_x,c_y)$ can be found after the calibration process and are assumed to be known.

### Calibration process

The calibration consist in take pictures of an well known pattern, easy find a set of $[u^{(i)} v^{(i)} X_w^{(i)} Y_w^{(i)} Z_w^{(i)}], i=1,2,...,n$ homologue points between these pictures and solve the equation (3) for $P$. Then, factorize $P$ into $K$ and $T$. 

#### Solving for P

$$
Z_w\begin{bmatrix}
    u \\
    v \\
    1
\end{bmatrix} = 
\begin{bmatrix}
    p_{11} & p_{12} & p_{13} & p_{14} \\
    p_{21} & p_{22} & p_{23} & p_{24} \\
    p_{31} & p_{32} & p_{33} & p_{34}
\end{bmatrix}
\begin{bmatrix}
    X_w \\
    Y_w \\
    Z_w \\
    1
\end{bmatrix} =
\begin{bmatrix}
    p_{11}X_w + p_{12}Y_w + p_{13}Z_w + p_{14} \\
    p_{21}X_w + p_{22}Y_w + p_{23}Z_w + p_{24} \\
    p_{31}X_w + p_{32}Y_w + p_{33}Z_w + p_{34}
\end{bmatrix} \\

u = \frac{p_{11}X_w + p_{12}Y_w + p_{13}Z_w + p_{14}}{p_{31}X_w + p_{32}Y_w + p_{33}Z_w + p_{34}} \\

v = \frac{p_{21}X_w + p_{22}Y_w + p_{23}Z_w + p_{24}}{p_{31}X_w + p_{32}Y_w + p_{33}Z_w + p_{34}} \\

p_{11}X_w + p_{12}Y_w + p_{13}Z_w + p_{14} - up_{31}X_w - u p_{32}Y_w - u p_{33}Z_w - u p_{34} = 0 \\

p_{21}X_w + p_{22}Y_w + p_{23}Z_w + p_{24} - vp_{31}X_w - v p_{32}Y_w - v p_{33}Z_w - v p_{34} = 0
$$

Making this as a vector multiplication we have:

$$
\begin{bmatrix}
X_w & Y_w & Z_w & 1 & 0   & 0   & 0   & 0 & -uX_w & -uY_w & -uZ_w & -u \\
0   &  0  & 0   & 0 & X_w & Y_w & Z_w & 1 & -vX_w & -vY_w & -vZ_w & -v 
\end{bmatrix}
\begin{bmatrix}
 p_{11} \\
 p_{12} \\
 p_{13} \\
 p_{14} \\
 p_{21} \\
 p_{22} \\
 p_{23} \\
 p_{24} \\
 p_{31} \\
 p_{32} \\
 p_{33} \\
 p_{34}
\end{bmatrix} = 0
$$

And for our found set we have:

$$

\begin{bmatrix}
X_w^{(1)} & Y_w^{(1)} & Z_w^{(1)} & 1 & 0   & 0   & 0   & 0 & -u^{(1)}X_w^{(1)} & -u^{(1)}Y_w^{(1)} & -u^{(1)}Z_w^{(1)} & -u^{(1)} \\
0   &  0  & 0   & 0 & X_w^{(1)} & Y_w^{(1)} & Z_w^{(1)} & 1 & -v^{(1)}X_w^{(1)} & -v^{(1)}Y_w^{(1)} & -v^{(1)}Z_w^{(1)} & -v^{(1)} \\
\vdots \\
X_w^{(n)} & Y_w^{(n)} & Z_w^{(n)} & 1 & 0   & 0   & 0   & 0 & -u^{(n)}X_w^{(n)} & -u^{(n)}Y_w^{(n)} & -u^{(n)}Z_w^{(n)} & -u^{(n)} \\
0   &  0  & 0   & 0 & X_w^{(n)} & Y_w^{(n)} & Z_w^{(n)} & 1 & -v^{(n)}X_w^{(n)} & -v^{(n)}Y_w^{(n)} & -v^{(n)}Z_w^{(n)} & -v^{(n)} \\
\end{bmatrix}
\begin{bmatrix}
 p_{11} \\
 p_{12} \\
 p_{13} \\
 p_{14} \\
 p_{21} \\
 p_{22} \\
 p_{23} \\
 p_{24} \\
 p_{31} \\
 p_{32} \\
 p_{33} \\
 p_{34}
\end{bmatrix} = 0 
$$

$$
\begin{bmatrix}A\end{bmatrix}_{n \ \times \ 12}\ \begin{bmatrix}p\end{bmatrix}_{12 \ \times\ 1}=0
$$

Which is solveable trough constrained least squares parameter estimation method. The scale of $p$ is set and the problem is transformed into <u>[A][p] tending to [0]</u> such that <u>||p||=1</u>:

$$
min (||Ap||^2) \ subject \ to \ ||p||=1
$$

$$
min (p^TA^TAp) \ subject \ to \ p^Tp=1
$$

We can define a convenient loss function we want to minimize:

$$
min = p^TA^TAp - \lambda(p^Tp -1)
$$

Taking the derivative with respect to $p$, we have:

$$
0 = 2A^TAp - 2\lambda p
$$

That is equivalent to solving the eigenvalue problem:

$$
A^TAp = \lambda p
$$

$p$ is the eigenvector corresponding to smallest eigenvalue of $A^TA$.

#### Factorizing P

$$
\begin{bmatrix}
    p_{11} & p_{12} & p_{13} & p_{14} \\
    p_{21} & p_{22} & p_{23} & p_{24} \\
    p_{31} & p_{32} & p_{33} & p_{34}
\end{bmatrix} = 
\begin{bmatrix}
    f_x & 0 & c_x & 0 \\
    0 & f_y & c_y & 0 \\
    0 & 0 & 1 & 0
\end{bmatrix}
\begin{bmatrix}
    R_{3 \times 3} & t_{3\times 1} \\
    0_{1 \times 3} & 1 
\end{bmatrix}
$$

The first $3 \times 3$ block of $P$ is:

$$
\begin{bmatrix}
    p_{11} & p_{12} & p_{13} \\
    p_{21} & p_{22} & p_{23} \\
    p_{31} & p_{32} & p_{33} 
\end{bmatrix} = 
\begin{bmatrix}
    f_x & 0 & c_x \\
    0 & f_y & c_y \\
    0 &  0  & 1 
\end{bmatrix}
\begin{bmatrix}
    R\\
\end{bmatrix} = KR
$$

Where $K$ (the camera matrix) happen to be an upper triangular matrix and $R$ is a rotation matrix, thereby, orthonormal. With this, we can use *QR factorization* to compute $K$ and $R$.

The last $3 \times 1$ block of $P$ is:

$$
\begin{bmatrix}
    p_{14} \\
    p_{24} \\
    p_{34} 
\end{bmatrix} = 
\begin{bmatrix}
    f_x & 0 & c_x \\
    0 & f_y & c_y \\
    0 &  0  & 1 
\end{bmatrix}
\begin{bmatrix}
    t_x \\
    t_y \\
    t_z
\end{bmatrix} = Kt \rightarrow t = K^{-1}\begin{bmatrix}
    p_{14} \\
    p_{24} \\
    p_{34} 
\end{bmatrix}
$$

### Simple Stereo Camera Model

<img src="imgs/SimpleStereo.png">

With a calibrated pair of cameras ($f_x,f_y,c_x,c_y$ assumed to be known), for the camera on the left and the camera on the right we have a valid equation (1):

$$
Z_l
\begin{bmatrix}
u_l \\
v_l \\
1
\end{bmatrix} =
\begin{bmatrix}
    f_x & 0 & c_x & 0 \\
    0 & f_y & c_y & 0 \\
    0 & 0 & 1 & 0
\end{bmatrix}
\begin{bmatrix}
X_l \\
Y_l \\
Z_l \\
1
\end{bmatrix} \\

Z_l
\begin{bmatrix}
u_r \\
v_r \\
1
\end{bmatrix} =
\begin{bmatrix}
    f_x & 0 & c_x & 0 \\
    0 & f_y & c_y & 0 \\
    0 & 0 & 1 & 0
\end{bmatrix}
\begin{bmatrix}
X_l-b \\
Y_l \\
Z_l \\
1
\end{bmatrix}
$$

Which solving for the left camera reference ($X_l,Y_l,Z_l$) gives us:

$$
X_l = \frac{b(u_l-c_x)}{u_l-u_r} , 
Y_l = \frac{bf_x(v_l-c_y)}{f_y(u_l-u_r)} , 
Z_l = \frac{bf_x}{u_l-u_r} , 
$$

Note the commom denominator $(u_l-u_r)$, this term is also known as disparity. Higher the horizontal diference between two homologue points (disparity), lower the depth $(z)$. The opposite is also true, if the disparity is small, the depth is higher. Imagine a point in the infinity, the disparity would be closer to zero.

Also, if the baseline is zero, the two images would be equal and disparity would be zero. As the baseline increases, the disparity increases.

Higher baselines provide more precise measure in $X_l,Y_l,Z_l$ coordinates. But too high, can decrease the number of homologue points.

### Simple Stereo Example

Consider the scenario on [1. Cameras arrangement and setup](#section-1). Three pictures taken at the same time from the left, rigth and center cameras:

<img src="imgs/L_0000.jpg" height=200><img src="imgs/R_0000.jpg" height=200><img src="imgs/C_0000.jpg" height=200>

## <a name="section-3"></a> 3. Cheap cameras specs

```shell
# sudo apt install v4l-utils

ubuntu@ubiquityrobot:~$ v4l2-ctl --device /dev/video0 -L
```

|                 |left               | right           | center               | spare 1      | spare 2         |
| -------------   | -------------     | -------------   |-------------         |------------- |-------------    |
| Name            | USB2.0 PC CAMERA  |  webcam: Webcam | Pi camera NOIR 2.1   |GENERAL WEBCAM|Full HD webcam   |
| Driver name     | uvcvideo          |  uvcvideo       | bm2835 mmal          | uvcvideo     | uvcvideo        |
| Image Res       | ?????????????     |  2048x1536      | 3280 x 2464          | 1920x1080    | 1920x1080       |
|Possible controls|  brightness  (int) : 0 -255 <br> contrast  (int) : 0 -255 <br> saturation  (int): 0 -255 <br> hue  (int) : -127 -127 <br>gamma (int) : 1 -8 <br>power_line_frequency  (menu): 0 -2<br>sharpness  (int) : 0 -15 <br>backlight_compensation  (int) : 1 -5 | brightness (int): 1 -255<br> contrast (int): 1 -255<br> saturation (int): 1 -255<br> white_balance_temperature_auto (bool):<br> gain (int): 1 -100<br> power_line_frequency (menu): 0 -2<br> white_balance_temperature (int): 2800 -6500<br> sharpness (int): 1 -255<br> exposure_auto (menu): 0 -3<br> exposure_absolute (int): 5 -2500<br> exposure_auto_priority (bool)|brightness (int) : 0 -100 <br> contrast (int) : -100 -100<br> saturation (int) : -100 -100 <br> red_balance (int) : 1 -7999 <br> blue_balance (int) : 1 -7999<br> horizontal_flip (bool) : <br> vertical_flip (bool) :<br> power_line_frequency (menu) : 0 -3 <br> sharpness (int) : -100 -100  r<br> color_effects (menu) : 0 -15<br> rotate (int) : 0 -360 <br> color_effects_cbcr (int) : 0 -65535 <br>Codec Controls:<br> video_bitrate_mode (menu): 0 -1 <br> video_bitrate (int): 25000 -25000000 step=25000<br> repeat_sequence_header (bool): <br> h264_i_frame_period (int): 0 -2147483647 <br> h264_level (menu): 0 -11 <br> h264_profile (menu): 0 -4 <br><br>Camera Controls:<br>auto_exposure (menu): 0 -3 <br> exposure_time_absolute (int): 1 -10000<br> exposure_dynamic_framerate (bool): <br> auto_exposure_bias (intmenu): 0 -24 <br> white_balance_auto_preset (menu): 0 -10 <br> image_stabilization (bool): <br> iso_sensitivity (intmenu): 0 -4 <br> iso_sensitivity_auto (menu): 0 -1 <br> exposure_metering_mode (menu): 0 -2 <br> scene_mode (menu): 0 -13 <br><br> JPEG Compression Controls:<br> compression_quality (int): 1 -100 |brightness (int) : 1 -255| brightness 0x00980900 (int) : 0 -255 step=1 default=128 value=153<br> contrast (int) : 0 -255<br> saturation (int) : 0 -255<br> hue (int) : 0 -255<br> white_balance_temperature_auto (bool) : <br> gamma (int) : 0 -255<br> gain (int) : 0 -255<br> power_line_frequency (menu) : 0 -2<br> white_balance_temperature (int) : 2800 -6500<br> sharpness (int) : 0 -255<br> backlight_compensation (int) : 0 -2<br> exposure_auto (menu) : 0 -3<br> exposure_absolute (int) : 3 max=2047 |



## <a name="section-4"></a> 4. Cameras calibration

## <a name="section-5"></a> 5. Run OpenCV photo captures in loop

```python
#!/usr/bin/python3
import cv2
import time
import argparse
import os

BASE_PHOTO_PATH = '/home/ubuntu/raspfotos'

def list_avaiable_cameras():
    a = []
    for port in range(5):
        cam = cv2.VideoCapture(port)
        if not cam.isOpened():
            print(f"Camera {port} is not opened.")
        else:
            a.append(port)
        cam.release()
    return a

def main(t=60):
    folder_name = len(os.listdir(BASE_PHOTO_PATH))+1
    path = f"{BASE_PHOTO_PATH}/{folder_name}"
    os.mkdir(path)

    cam = []
    avaiable_cameras = list_avaiable_cameras()
    for k in avaiable_cameras:
        cam.append(cv2.VideoCapture(k))

    count = 0
    t_end = time.time() + t
    while time.time() < t_end:
        for i in range(len(cam)):
            ret, image = cam[i].read()
            if ret:
                pic_name = f'{path}/cam{i}_frame{count}.jpg'
                print(pic_name)
                cv2.imwrite(pic_name, image)
        count+=1
    
    for i in range(len(cam)):
        cam[i].release()
    
    cv2.destroyAllWindows()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-s','--segundos',
                        default=10,
                        type=int,
                        help='Add time in seconds')
    args = parser.parse_args()
    print(f"Starting taking {args.segundos} seconds of photos.")
    main(args.segundos)
```

## <a name="section-6"></a> 6. Cameras rearrangement