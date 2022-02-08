#include <ros/ros.h>
#include <rosbag/bag.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include <geometry_msgs/WrenchStamped.h>
#include "my_msgs/my_message.h"
#include <stdio.h>

my_msgs::my_message global_msg;
void force_z_Callback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
rosbag::Bag bag;

void force_z_Callback(const geometry_msgs::WrenchStamped::ConstPtr& msg) // callback message
{
  global_msg.fz = msg->wrench.force.z;
  ROS_INFO_STREAM(global_msg);

}

int main(int argc, char **argv){
  ros::init(argc, argv, "ros_bag");

  ros::NodeHandle nh; //NodeHandle is the main access point to communications with the ROS system

  bag.open("/home/zhan/rustamch/src/calibrate_rgb_sensor_rustam/src/rosbag_data/myrosbagfile.bag", rosbag::bagmode::Write);
  ros::Rate loop_rate(140); // 140 Hz: frequency of bag.write()

  ros::Subscriber sub_force_z = nh.subscribe("/weiss_wrench", 100, force_z_Callback);
  ros::Duration(0.2).sleep(); // wait this amount of time while sub_force_z is subscribed
  int i = 1;
  while (ros::ok() && i < 30){
  //ros::Publisher all_pub = nh.advertise<my_msgs::my_message>("/all_data", 1);
  //all_pub.publish(global_msg);
  ROS_INFO_STREAM("Written message is above");
  bag.write("fignya13", ros::Time::now(), global_msg);
  ros::spinOnce(); // call force_z_Callback
  loop_rate.sleep();
  i++;
}
  bag.close();
  return 0;
}
