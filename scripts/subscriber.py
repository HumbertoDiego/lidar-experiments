#!/usr/bin/python3
import rospy
from sensor_msgs.msg import PointCloud

def add_increment_to_timestamp(timestamp, increment, size):
    return [timestamp+i*increment for i in range(size)]

def put_together(timestamps, points):
    return f"{timestamps},{points.x},{points.y},{points.z}\n"

def callback(data):
    # Get x,y,z
    points = data.points
    # Get timestamps
    initial_timestamp = data.header.stamp.secs + data.header.stamp.nsecs/1000000000
    timeincrement = data.channels[1].values[0]
    timestamps = add_increment_to_timestamp(initial_timestamp, timeincrement, len(points))
    # put together
    msg = "".join(list(map(put_together,timestamps,points)))
    print("-->",data.header.seq)
    # save
    with open("t_x_y_z.csv", "a") as f:
        f.write(msg)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/pointcloud2d", PointCloud, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()