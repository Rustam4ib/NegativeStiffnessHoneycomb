#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TwistStamped.h>
#include "geometry_msgs/WrenchStamped.h"
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#define PI 3.1415926

// for transformations
#include <tf/tf.h>
#include <tf/transform_listener.h>
//frame broadcaster
#include <tf/transform_broadcaster.h>

#include <Eigen/Core>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/TwistStamped.h>

#include "tactile_servo_msgs/ContsFeats.h"
#include "tactile_servo_msgs/OneContFeats.h"
#include <std_msgs/Float64.h>
#include "ros/time.h"
#include<time.h>

double Z_INIT;
double Z_DOWN;
double Z_NOW;
double X_NOW;
double Y_NOW;
double WZ_NOW;

int STATE_NUMBER;

double X_INIT;
double X_2;
double X_3;
double X_4;

double Y_TEMP;
double X_TEMP;
double Z_TEMP;
double WZ_TEMP;
double WZ_INIT;


double Y_INIT;
double Y_2;
double Y_3;
double Y_4;

double X_5;
double X_6;
double X_7;

double Y_5;
double Y_6;
double Y_7;

typedef Eigen::Matrix<double, 6, 1>  VECTOR6_1;

std::string JOINT_NAMES[6]={"shoulder_pan_joint","shoulder_lift_joint",
			    "elbow_joint",  	 "wrist_1_joint",
			    "wrist_2_joint",	 "wrist_3_joint"};

//Q1 = [0,-1.57,0,-1.57,0,0];

class PoseShow
{
  public:
  PoseShow();
  ~PoseShow();
  bool not_first_time;
  void pose_init_create();
  tf::TransformListener listener;
  tf::StampedTransform stransform;
  bool is_transform_tool0_;
  tf::Vector3 init_position;
  tf::Transform init_transform;
  void br_init_pose();
  tf::TransformBroadcaster br_init_frame_;

  void pose_des_create();
  tf::Transform des_transform;
  tf::TransformBroadcaster br_goal_frame_;
  void br_des_pose();

  tf::TransformListener listener_pose_now;
  void pose_now();
  tf::StampedTransform  nowtransform;

  geometry_msgs::PoseStamped X_des_test_;
  geometry_msgs::PoseStamped X_now_test_;

  private:
  ros::NodeHandle nh;

};

PoseShow::PoseShow()
{
  is_transform_tool0_ = false;
  not_first_time = true;
}

PoseShow::~PoseShow(){}

void PoseShow::pose_init_create(){
	if (is_transform_tool0_== false)
	{
	listener.waitForTransform("tool0", ros::Time(0), "robot_base",ros::Time(0), "robot_base", ros::Duration(10));
	try
	{
//	ROS_INFO("in try");
		listener.lookupTransform("robot_base", "tool0", ros::Time(0), stransform);
		is_transform_tool0_ = true;
	}
	catch (tf::TransformException &ex)
	{
// 	ROS_INFO("in error 1");
	ROS_ERROR("%s", ex.what());
	}

  init_position = stransform.getOrigin();
  //ROS_INFO("%lf,%lf,%lf",init_position[0],init_position[1],init_position[2]);
  X_now_test_.header.frame_id = stransform.frame_id_.c_str();
  X_now_test_.pose.position.x = init_position[0];
  X_now_test_.pose.position.y = init_position[1];
  X_now_test_.pose.position.z = init_position[2];

  Z_INIT = init_position[2];
  X_INIT = init_position[0];
  Y_INIT = init_position[1];

  Z_TEMP = init_position[2];
  X_TEMP = init_position[0];
  Y_TEMP = init_position[1];

  not_first_time = false;
  // ROS_INFO("in init ZXY 1stime%lf,%lf,%lf",Z_INIT,X_INIT,Y_TEMP);
  }
  tf::Quaternion q_init_pose = stransform.getRotation();
  geometry_msgs::Quaternion quat;
  quat.w = q_init_pose.w();
  quat.w = q_init_pose.x();
  quat.w = q_init_pose.y();
  quat.w = q_init_pose.z();
  X_now_test_.pose.orientation = quat;
}

void PoseShow::pose_now(){

	listener_pose_now.waitForTransform("tool0", ros::Time(0), "robot_base",ros::Time(0), "robot_base", ros::Duration(10));
	try
	{
// 	  ROS_IFNO("in try");
		listener_pose_now.lookupTransform("robot_base", "tool0", ros::Time(0), nowtransform);
	}
	catch (tf::TransformException &ex)
	{
		ROS_ERROR("%s", ex.what());
	}
	Z_NOW = nowtransform.getOrigin().getZ();
	X_NOW = nowtransform.getOrigin().getX();
	Y_NOW = nowtransform.getOrigin().getY();
	//WZ_NOW = nowtransform.getRotation().getAngle();

	double roll, pitch, yaw;
	nowtransform.getBasis().getRPY(roll, pitch, yaw);
	WZ_NOW = yaw * 180.0 / PI;
	//ROS_ERROR("Bot Z: %f", WZ_NOW);


	//ROS_INFO("in pose now func zinit %lf,znow =  %lf, xnow%lf, ynow%lf",Z_NOW,X_NOW,Y_NOW);
// 	double WX_NOW = nowtransform.getRotation().getX();

}

void PoseShow::br_init_pose()
{
  if (is_transform_tool0_ == true)
  {
  init_transform.setOrigin(init_position);
  init_transform.setRotation(stransform.getRotation());
  br_init_frame_.sendTransform(tf::StampedTransform(init_transform, ros::Time::now(), "robot_base", "init_pose"));
//   br_frame_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), "world", "palm desired"));
  }
}

void PoseShow::pose_des_create()
{
  if (is_transform_tool0_ == true)
  {
    X_des_test_ = X_now_test_;
    X_des_test_.header.frame_id = "desired" ;
    X_des_test_.pose.position.z = Z_DOWN;// X_des_test_.pose.position.z - 0.100;

  }
  else
  {
    ROS_FATAL_STREAM("viz des pose no");
  }
}

void PoseShow::br_des_pose()
{
  des_transform = init_transform;
  des_transform.setOrigin(tf::Vector3(X_des_test_.pose.position.x, X_des_test_.pose.position.y,
			 X_des_test_.pose.position.z));//0.5 is the motion scalar
//   tf::Quaternion orien;
//   geometry_msgs::Quaternion quat;
//   quat = X_des_test_.pose.orientation;
//   orien.setW(quat.w);
//   orien.setZ(quat.z);
//   orien.setY(quat.y);
//   orien.setX(quat.x);
//   des_transform.setRotation(orien);
  br_goal_frame_.sendTransform(tf::StampedTransform(des_transform, ros::Time::now(), "robot_base", "goal"));
//   ROS_INFO("in broadcast des  1stime%lf,%lf,%lf",Z_INIT,X_INIT,Y_TEMP);

}

//class with subscribers
class URutil
{
public:
  URutil();
  ~URutil();

  ros::NodeHandle n;
  ros::Subscriber sub_opto;
  /*for command send*/
  ros::Publisher pub_joint_speed_to_real_driver;
  ros::Publisher pub_action_states;
  trajectory_msgs::JointTrajectory trj_;
  trajectory_msgs::JointTrajectoryPoint trjp_;
  std::vector<double> joint_init_velocities_;
  bool boollet_;
  int couner_;
  bool recieved_joint_state_;
  // current joint joint_values
  ros::Subscriber sub_joint_states;
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
  std::vector<double> joint_values_;
  std::vector<double> joint_velocities_;

  //for ur init
  bool is_init_exper_;
  bool is_at_home_;

  //get pose from /palm/pose
//   geometry_msgs::PoseStamped get_pose();
  //send pose to /move_arm_6D
//   void send_pose(geometry_msgs::PoseStamped msg_sendpose);
//   void callback_current_pose(const geometry_msgs::PoseStampedPtr& msg);
//   bool is_pose_rec_;

    /**
   * Subscribes to the palm pose published by
   * tools/kinematics_tools/scripts/palm_pose_publisher.py
   */
//   geometry_msgs::PoseStamped palm_pose_now_;
//   geometry_msgs::PoseStamped palm_pose_send_;
//   ros::Subscriber sub_current_pose;
  ros::Publisher pub_velocities;
  void send_velocity(geometry_msgs::Twist in_vel);

  ros::Subscriber fb_feats_sub;
  void cb_fb_feats_sub(const tactile_servo_msgs::ContsFeatsConstPtr& msg_);
  void Callback_optoforce(const geometry_msgs::WrenchStamped msg);

  int state_number_;
  int action_type_;

  bool is_in_contact_ur_;
  int  pixels_in_contact_ur_;
  float highest_force_cell_ur_;
  float real_total_force_ur_ ;
  bool is_new_msg_;
  geometry_msgs::Twist in_vel_;

  ros::Publisher pub_point;
  ros::Publisher pub_mode;
  ros::Publisher pub_state;
  ros::Publisher pub_angleZ;

  void send_pointCoor(geometry_msgs::PointStamped pointCoor);
  void send_angleZ(float angleZ);
  void send_mode(float mode);
  void send_state(float state);


  private:

};


URutil::URutil()
{
  is_new_msg_ = false;

  pub_velocities = n.advertise<geometry_msgs::Twist>("/desired_Vx_Vy_Vz", 1000);

  pub_point = n.advertise<geometry_msgs::PointStamped>("/topic_point", 100);

  pub_mode = n.advertise<std_msgs::Float64>("/topic_mode", 100);

  pub_state = n.advertise<std_msgs::Float64>("/topic_state", 100);

  pub_angleZ = n.advertise<std_msgs::Float64>("/topic_angleZ", 100);


  fb_feats_sub = n.subscribe("/fb_feats", 1,  &URutil::cb_fb_feats_sub,this);
  sub_opto = n.subscribe("/weiss_wrench", 100,  &URutil::Callback_optoforce, this);

  is_init_exper_ = false;
  is_at_home_ = false;
  boollet_ = false;
  couner_ = 0;
  recieved_joint_state_ = false;
  trjp_.velocities.resize(6);
  trj_.points.push_back(trjp_);
  trj_.points.push_back(trjp_);

//   	ROS_INFO("initializing joint names");
	trj_.joint_names.push_back("shoulder_pan_joint");
	trj_.joint_names.push_back("shoulder_lift_joint");
	trj_.joint_names.push_back("elbow_joint");
	trj_.joint_names.push_back("wrist_1_joint");
	trj_.joint_names.push_back("wrist_2_joint");
	trj_.joint_names.push_back("wrist_3_joint");

//   contact_points_sub = n.subscribe("/contact_points_weiss_plot", 1,  &markersShow::cb_contact_points_sub,this);
// contact_points_hist= n.advertise<visualization_msgs::MarkerArray>("mark_contacts_hist", 1 );

}
URutil::~URutil(){}
void URutil::cb_fb_feats_sub(const tactile_servo_msgs::ContsFeatsConstPtr& msg_){
    if (msg_->control_features.size() == 1){
        is_in_contact_ur_ = msg_->control_features[0].is_contact;
        pixels_in_contact_ur_ = msg_->control_features[0].num_pixels_contact;
        highest_force_cell_ur_ = msg_->control_features[0].highest_force_cell;
        real_total_force_ur_ = msg_->control_features[0].real_total_force;

	is_new_msg_ = true;
        /* coordinates of COP/COC in world =
        / sequence number of the  COP/COC pixel in x or y
        / *
        / physical size of a sensor in x or y
        / /
        / number of cells in x(y)*/
//        copx_fb = copx_fb*size_x / cells_x;
//        copy_fb = copy_fb*size_y / cells_y;
//        cocx_fb = cocx_fb*size_x / cells_x;
//        cocy_fb = cocy_fb*size_y / cells_y;
    }
}

// --------------------------------------------------------


void URutil::send_velocity(geometry_msgs::Twist in_vel)
{
  in_vel_.angular = in_vel.angular;
  in_vel_.linear = in_vel.linear;
  pub_velocities.publish(in_vel_);
}

geometry_msgs::PointStamped CURRENT_point;

void URutil::send_pointCoor(geometry_msgs::PointStamped pointCoor)
  {
    geometry_msgs::PointStamped in;
    in.point.x = pointCoor.point.x;
    in.point.y = pointCoor.point.y;
    in.point.z = pointCoor.point.z;
    pub_point.publish(in);
 }

float CURRENT_angleZ;
void URutil::send_angleZ(float angleZ)
  {
    std_msgs::Float64 in;
    in.data = angleZ;
    pub_angleZ.publish(in);
  }

float CURRENT_mode;
void URutil::send_mode(float mode)
  {
    std_msgs::Float64 in;
    in.data = mode;
    pub_mode.publish(in);
  }

float CURRENT_state;
void URutil::send_state(float state)
  {
    std_msgs::Float64 in;
    in.data = state;
    pub_state.publish(in);
  }

// void Callback_optoforce(const geometry_msgs::WrenchStamped msg);
 geometry_msgs::WrenchStamped global_msg_image_state_force;

  void URutil::Callback_optoforce(const geometry_msgs::WrenchStamped msg)
  {
    global_msg_image_state_force.wrench.force.x = msg.wrench.force.x;
    global_msg_image_state_force.wrench.force.y = msg.wrench.force.y;
    global_msg_image_state_force.wrench.force.z  = msg.wrench.force.z;
    global_msg_image_state_force.wrench.torque.x = msg.wrench.torque.x;
    global_msg_image_state_force.wrench.torque.y = msg.wrench.torque.y;
    global_msg_image_state_force.wrench.torque.z  = msg.wrench.torque.z;

    //Book.o_temp = msg;
    geometry_msgs::WrenchStamped temp = msg;
    //uint32_t temp2 = msg.header.seq;
    //geometry_msgs::WrenchStamped temp3 = msg.header.stamp;
    //ROS_INFO("I heard: %lf", temp.wrench.force.z);
  // 	ROS_INFO_STREAM(data_mine::o_temp);

  }


 int main(int argc, char** argv)

 {
  // ros::NodeHandle n;
  //

  ROS_INFO("pub_poses_exper");


  ros::Time::init();
  int OVERAL;
  OVERAL = 0;

  geometry_msgs::Twist in_velocityzero;
  in_velocityzero.linear.x = 0;
  in_velocityzero.linear.y = 0;
  in_velocityzero.linear.z = 0.0;
  in_velocityzero.angular.x = 0;
  in_velocityzero.angular.y = 0;
  in_velocityzero.angular.z = 0;

  geometry_msgs::Twist in_velocityup;
  in_velocityup.linear.x = 0;
  in_velocityup.linear.y = 0;
  in_velocityup.linear.z = 0.0047;
  in_velocityup.angular.x = 0;
  in_velocityup.angular.y = 0;
  in_velocityup.angular.z = 0;

  geometry_msgs::Twist in_velocitydown;
  in_velocitydown.linear.x = 0;
  in_velocitydown.linear.y = 0;
  in_velocitydown.linear.z = -0.001;
  in_velocitydown.angular.x = 0;
  in_velocitydown.angular.y = 0;
  in_velocitydown.angular.z = 0;

  geometry_msgs::Twist in_velocitydown_slow;
  in_velocitydown_slow.linear.x = 0;
  in_velocitydown_slow.linear.y = 0;
  in_velocitydown_slow.linear.z = -0.001;
  in_velocitydown_slow.angular.x = 0;
  in_velocitydown_slow.angular.y = 0;
  in_velocitydown_slow.angular.z = 0;

  geometry_msgs::Twist in_velocityXL;
  in_velocityXL.linear.x = -0.005;
  in_velocityXL.linear.y = 0;
  in_velocityXL.linear.z = 0.000;
  in_velocityXL.angular.x = 0;
  in_velocityXL.angular.y = 0;
  in_velocityXL.angular.z = 0;

  geometry_msgs::Twist in_velocityXR;
  in_velocityXR.linear.x = 0.005;
  in_velocityXR.linear.y = 0;
  in_velocityXR.linear.z = 0.00;
  in_velocityXR.angular.x = 0;
  in_velocityXR.angular.y = 0;
  in_velocityXR.angular.z = 0;

  geometry_msgs::Twist in_velocityYL;
  in_velocityYL.linear.x = 0;
  in_velocityYL.linear.y = -0.005; // normal 0.005; // shear 0.001
  in_velocityYL.linear.z = 0.000;
  in_velocityYL.angular.x = 0;
  in_velocityYL.angular.y = 0;
  in_velocityYL.angular.z = 0;

  geometry_msgs::Twist in_velocityYR;
  in_velocityYR.linear.x = 0;
  in_velocityYR.linear.y = 0.005;
  in_velocityYR.linear.z = 0.000;
  in_velocityYR.angular.x = 0;
  in_velocityYR.angular.y = 0;
  in_velocityYR.angular.z = 0;

  geometry_msgs::Twist in_velocityZL;
  in_velocityZL.linear.x = 0;
  in_velocityZL.linear.y = 0.00;
  in_velocityZL.linear.z = 0.00;
  in_velocityZL.angular.x = 0;
  in_velocityZL.angular.y = 0;
  in_velocityZL.angular.z = -0.12; //

  geometry_msgs::Twist in_velocityZR;
  in_velocityZR.linear.x = 0;
  in_velocityZR.linear.y = 0.00;
  in_velocityZR.linear.z = 0.00;
  in_velocityZR.angular.x = 0;
  in_velocityZR.angular.y = 0;
  in_velocityZR.angular.z = 0.03;

  geometry_msgs::Twist in_velocityRandXY;
  in_velocityRandXY.linear.x = 0.00;
  in_velocityRandXY.linear.y = 0.00;
  in_velocityRandXY.linear.z = 0.00;
  in_velocityRandXY.angular.x = 0.00;
  in_velocityRandXY.angular.y = 0.00;
  in_velocityRandXY.angular.z = 0.00;

  float v_z;
  float v_x;
  float v_y;
  bool is_touch = false;
  float penetration;

  ros::init(argc, argv, "init_before_any_exploration_node");
  ros::NodeHandle nh;
  ros::Rate loop_rate(125);
  ROS_INFO("pub_poses_exper");
  PoseShow PoseShow_test;
  URutil Experiment;

  // URutil  move_joint_wrist_1_joint;
  // to init subscribers

  ros::spinOnce();
  PoseShow_test.pose_init_create();
  ros::spinOnce();
  PoseShow_test.br_init_pose();
  ros::spinOnce();
  Z_DOWN = Z_INIT - 0.127;
  PoseShow_test.pose_des_create();

  //ROS_INFO("in init ZXY 1stime%lf,%lf,%lf",Z_INIT,X_INIT,Y_TEMP);

  ros::spinOnce();
  PoseShow_test.br_des_pose();
  ros::spinOnce();
  ROS_INFO("before twist init");

  //initalize the values
  ros::Time start_time = ros::Time::now();
  ros::Duration timeout(0.2);

  CURRENT_point.point.x = -1.0;
  CURRENT_point.point.y = -1.0;
  CURRENT_point.point.z = -1.0;

  CURRENT_angleZ = 0.0;
  CURRENT_state = 0.0;


  Experiment.send_pointCoor(CURRENT_point);
  Experiment.send_angleZ(CURRENT_angleZ);
  Experiment.send_mode(CURRENT_mode);

  ros::spinOnce();
  PoseShow_test.pose_now();
  WZ_INIT = WZ_NOW;
  WZ_TEMP = WZ_NOW;



  //*******************************************************
  // SELECT THE EXPERIMENT
  // operation modes : Normal - 1; Shear - 2; Torsion - 3;
  CURRENT_mode = 1;
  //*******************************************************



  // ::: RUN THE EXPERIMENTS :::

  //
  // MODE - 1: Normal Pressing

   if (CURRENT_mode == 1){

     ROS_ERROR("Start Mode 1: Pressing");

     // Normal pressing

     /*
      * Algorithm :
      Repeat:
	Take random theta, length, depth ->> convert to required X, Y, Z
	Press
	Read : force, CURRENT_point, CURRENT_mode, CURRENT_angleZ
	Subscibe : force

	Publish : CURRENT_point, CURRENT_angleZ, CURRENT_mode
      */

    int count = 0;
    while (ros::ok() && (count < 10)){

      ROS_ERROR("count  %i", count);

      count = count + 1;

      srand(ros::Time::now().toNSec());

      // convert to X,Y,Z



      //ROS_ERROR("rand_angle  %f", rand_angle);
      //ROS_ERROR("rand_length %f", rand_length);
      //ROS_ERROR("rand_depth  %f ",rand_depth);

      //ROS_ERROR("check cos rad or degrees %f", cos(rand_angle*3.14/180));
      //ROS_ERROR("check sin rad or degrees %f", sin(rand_angle*3.14/180));


      double increment_z = 5/10000.0;     // depth interval = 0.5 mm


      //ROS_ERROR("rand_x in mm %f", rand_x*1000);
      //ROS_ERROR("rand_y in mm %f", rand_y*1000);
      //ROS_ERROR("rand_z in mm %f ",rand_z*1000);

      geometry_msgs::Twist vel_x;
      geometry_msgs::Twist vel_y;
      geometry_msgs::Twist vel_z;

      // Move to rand_x -> then to rand_y -> then to rand_z
      //ROS_ERROR("START EXPERIMENTS: ");
      //ROS_ERROR("X_NOW %f", X_NOW);
      //ROS_ERROR("Y_NOW %f", Y_NOW);
      //ROS_ERROR("Z_NOW %f ",Z_NOW);
for (int iDepth = 1; iDepth <= 10; iDepth++  ) {

			while( (ros::ok())&&(Z_NOW>(Z_TEMP)) )
			{
			  PoseShow_test.pose_now();
			  Experiment.send_velocity(in_velocitydown);
			  ros::spinOnce();
			  loop_rate.sleep();
			}
			Z_INIT = Z_TEMP;


			CURRENT_point.point.z = increment_z*iDepth;

			//***********************************************
			// stop for some time and send the values, publish angle, mode, point pointCoor
			start_time = ros::Time::now();
			ROS_INFO("STOP and SEND the values");
			//ROS_ERROR("X_NOW %f", X_NOW);
			//ROS_ERROR("Y_NOW %f", Y_NOW);
			//ROS_ERROR("Z_NOW %f ",Z_NOW);

			CURRENT_state = 1.0;
			while((ros::ok()) && (ros::Time::now() - start_time < timeout)) {
			  Experiment.send_velocity(in_velocityzero);
			  Experiment.send_mode(CURRENT_mode);
			  Experiment.send_state(CURRENT_state);
			  Experiment.send_angleZ(CURRENT_angleZ);
			  Experiment.send_pointCoor(CURRENT_point);
			}

			// Finish the sending the signals and change the state to 0
			CURRENT_state = 0.0;
			Experiment.send_state(CURRENT_state);

      }

      double depth_offset = 0/1000.0;

      Z_TEMP = Z_INIT + increment_z*10.0 + depth_offset;
      ROS_INFO("Move to rand_z up");
      while( (ros::ok())&&(Z_NOW<(Z_TEMP)) )
      {
				PoseShow_test.pose_now();
				Experiment.send_velocity(in_velocityup);
				ros::spinOnce();
				loop_rate.sleep();
      }
      Z_INIT = Z_TEMP;

      while( (ros::ok()) && (ros::Time::now() - start_time < timeout) ){
	Experiment.send_velocity(in_velocityzero);
	ros::spinOnce();
	loop_rate.sleep();
      }
    }

  // terminate the whole experiment
  // && (ros::Time::now() - start_time < timeout*2)

  ROS_ERROR("Experiment ENDED !!! ");
  ROS_ERROR("X_NOW %f", X_NOW);
  ROS_ERROR("Y_NOW %f", Y_NOW);
  ROS_ERROR("Z_NOW %f ",Z_NOW);
  ROS_ERROR("WZ_NOW %f ",WZ_NOW);

  while( (ros::ok())){
      Experiment.send_velocity(in_velocityzero);
      ros::spinOnce();
      loop_rate.sleep();
   }
  // end
  return 0;
}
}
