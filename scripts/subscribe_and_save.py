#!/usr/bin/python3
import rospy
from sensor_msgs.msg import Image
import json
import base64
import os

file_path = "result.json"

"""
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
string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h
uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)
"""

def get_dic_from_file():
    if os.path.exists(file_path):
        json_file = open(file_path, "r+")
    else:
        with open(file_path, "w") as f:
            f.write("[]")
        json_file = open(file_path, "r+")

    json_str = json_file.read()
    json_rows = json.loads(json_str)
    return json_rows

def callback(image_msg):
    json_rows = get_dic_from_file()
    # Convert the image data to a base64 string to include in JSON
    # image_data_base64 = base64.b64encode(image_msg.data).decode('utf-8')
    
    # Create a JSON-compatible dictionary
    image_json = {
        "header": {
            "seq": image_msg.header.seq,
            "stamp": {
                "secs": image_msg.header.stamp.secs,
                "nsecs": image_msg.header.stamp.nsecs
            },
            "frame_id": image_msg.header.frame_id
        },
        "height": image_msg.height,
        "width": image_msg.width,
        "encoding": image_msg.encoding,
        "is_bigendian": image_msg.is_bigendian,
        "step": image_msg.step,
        "data": image_msg.data  # encoded data as base64 or image_data_base64
    }

    json_rows.append(image_json)
    json_str = json.dumps(json_rows)  # Serialize dictionary to JSON string

    # save
    with open(file_path, "w") as f:
        f.write(json_str)


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/depth_registered/image_raw", Image, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()