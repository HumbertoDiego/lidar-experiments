FROM ros:noetic

RUN apt update && apt install git -y

RUN cd ~ && mkdir -p ldlidar_ros_ws/src

RUN cd ldlidar_ros_ws/src && git clone  https://github.com/ldrobotSensorTeam/ldlidar_sl_ros.git

RUN chmod 777 /dev/ttyACM0

RUN cd ~/ldlidar_ros_ws && sed -i 's/ttyUSB0/ttyACM0/' src/ldlidar_sl_ros/launch/ld14p.launch 

RUN cd ~/ldlidar_ros_ws && rosdep install --from-paths src --ignore-src -r -y

RUN cd ~/ldlidar_ros_ws && catkin_make

RUN cd ~/ldlidar_ros_ws && source devel/setup.bash
RUN echo "source ~/ldlidar_ros_ws/devel/setup.bash" >> ~/.bashrc
# launch ros package roslaunch ldlidar_sl_ros ld14.launch
CMD ["roslaunch", "ldlidar_sl_ros", "ld14p.launch"]
