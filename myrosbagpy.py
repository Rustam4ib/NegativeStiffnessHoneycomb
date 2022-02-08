from geometry_msgs.msg import WrenchStamped
import rospy
import os
import sys
import rosbag

name = '/home/zhan/rustamch/src/calibrate_rgb_sensor_rustam/src/rosbag_data/myrosbagfile.bag'

i = 0
bag=rosbag.Bag(name,'w')
def mybag():
    sub = rospy.Subscriber('/weiss_wrench', WrenchStamped, weiss_callback)
    rospy.spin()

def weiss_callback(msg):
    weiss_force = msg.wrench.force.z
    bag.write('/weiss_wrench', weiss_force)
    i += 1
    if (i>10):
        bag.close()
        rospy.signal_shutdown("reason to close")

if __name__ == '__main__':
    rospy.init_node('data_node')
    mybag()
