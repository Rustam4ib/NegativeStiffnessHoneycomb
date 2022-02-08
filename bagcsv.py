import bagpy
from bagpy import bagreader
import os


b = bagreader('/home/zhan/rustamch/src/calibrate_rgb_sensor_rustam/src/rosbag_data/myrosbagfile.bag')

print(b.topic_table)

csvfiles = []

for t in b.topics:
    data = b.message_by_topic(t)
    print(type(data))
    print(data)
    csvfiles.append(data)
