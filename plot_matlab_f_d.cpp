#include <ros/ros.h>
#include "tactile_servo_msgs/ContsFeats.h"
#include "tactile_servo_msgs/OneContFeats.h"

#include "tactile_servo_msgs/CalibWeissNano.h"

#include <geometry_msgs/WrenchStamped.h>

#include <geometry_msgs/PoseStamped.h>

#include "tactile_servo_msgs/PlanFeats.h"

#include "tactile_servo_msgs/PlotMatlabImg.h"

#include <geometry_msgs/PointStamped.h>

#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

class PlotMatlab

{
public:
  PlotMatlab();
  ~PlotMatlab();
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);


  ros::Subscriber force;
  void callback_force(const geometry_msgs::WrenchStamped& msg);
  float fx, fy, fz, mx, my, mz;

  ros::Subscriber sub_feats_fb;
  void callback_fbfeats(const tactile_servo_msgs::ContsFeats& msg);
  
  long double t_now_sec, t_begin_sec;
  long double t_now_nsec;
  ros::Duration dt, test_time;

  ros::Time t_old_sec_total, test_time_begin;
  long double n_cycle;
  long double test_time2;

  bool init_time;

  ros::NodeHandle nh;
  ros::Publisher pub_plot_matlab;

  bool is_feat_fb_rec_, is_feat_des_rec_,
  is_feat_plan_rec_, is_ati_rec_, is_pose_now_rec_,
  is_pose_des_rec_, is_cont_hist_rec_;

  void send_plot();

  ros::Time the_time;

  image_transport::ImageTransport it_viz;
  image_transport::Subscriber  img_now;
  void callback_img_now(const sensor_msgs::ImageConstPtr& msg);
  std::vector<float> img;
bool is_img_rec;
bool is_img_sent;
};

PlotMatlab::PlotMatlab():it_viz(nh)
{

  force = nh.subscribe("/wrench", 10, &PlotMatlab::callback_force, this);

  pub_plot_matlab = nh.advertise<tactile_servo_msgs::PlotMatlabImg>("/plot_matlab", 10);

  img_now = it_viz.subscribe("/ros_tactile_image", 1, &PlotMatlab::callback_img_now, this);
  is_img_rec = 0;
  is_img_sent = 0;
}
PlotMatlab::~PlotMatlab()
{}



 void PlotMatlab::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
 joint_values_.clear();
joint_efforts_.clear();

    if(msg->velocity.size() == 6)
    {
        for(int i = 0; i < msg->velocity.size(); i++)
        {
//     	    ROS_INFO("%d joint_name: %s", i, msg->name[i].c_str());
// 	    ROS_INFO("actual velocities: %lf", msg->velocity[i]);
            joint_values_.push_back(msg->position[i]);
	    joint_efforts_.push_back(msg->effort[i]);
// // 	    ROS_INFO("actual positions: %lf", msg->position[i]);
        }
            is_recieved_new_joint_state_ = true;

    }
}

}

void PlotMatlab::callback_img_now(const sensor_msgs::ImageConstPtr& msg)
{


  int rows_ = 4;
  int cols_ = 4;
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1);//TYPE_8UC1
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  if (is_img_sent == 1)
  {
  int c = 0;
  for(int i=0; i<rows_; ++i) {
    for(int j=0; j<cols_; j++) {
      // make it visible
      img.push_back(cv_ptr->image.data[c]);
      c = c + 1;
    }
  }
  is_img_rec = 1;
  is_img_sent = 0;
  }
}



void PlotMatlab::callback_fbfeats(const tactile_servo_msgs::ContsFeats& msg)
{
//  ROS_FATAL_STREAM("fb feat size = " << msg->control_features.size());
  if (msg.control_features.size() == 1)
  {
        is_in_contact_ur_ = msg.control_features[0].is_contact;
        pixels_in_contact_ur_ = msg.control_features[0].num_pixels_contact;
        highest_force_cell_ur_ = msg.control_features[0].highest_force_cell;
        real_total_force_ur_ = msg.control_features[0].real_total_force;
    fx_weiss = msg.control_features[0].contactForce;
    mx_weiss = msg.control_features[0].zmp_x;
    my_weiss = msg.control_features[0].zmp_y;
    copx_weiss =msg.control_features[0].centerpressure_x;
    copy_weiss=msg.control_features[0].centerpressure_y;
    orient_weiss=msg.control_features[0].contactOrientation;
    cocx_weiss=msg.control_features[0].centerContact_x;
    cocy_weiss=msg.control_features[0].centerContact_y;
    is_feat_fb_rec_ = true;
  }
  else
  {
    is_feat_fb_rec_ = false;
    ROS_DEBUG_STREAM("fb feat is not received");
  }
}



void PlotMatlab::callback_force(const geometry_msgs::WrenchStamped& msg)
{
//  ROS_FATAL_STREAM("fb feat size = " << msg->control_features.size());
  fx = msg.wrench.force.x;
  fy = msg.wrench.force.y;
  fz = msg.wrench.force.z;
  mx = msg.wrench.torque.x;
  my = msg.wrench.torque.y;
  mz = msg.wrench.torque.z;

  is_ati_rec_ = true;
}


void PlotMatlab::send_plot()
{
   if ( is_feat_fb_rec_  )
  {
    tactile_servo_msgs::PlotMatlabImg plot_matl;
    plot_matl.header.frame_id = "1";
    plot_matl.header.stamp = ros::Time::now();

    plot_matl.fx = fx;
    plot_matl.fy = fy;
    plot_matl.fz = fz;
    plot_matl.fwx = mx;
    plot_matl.fwy = my;
    plot_matl.fwz = mz;

    plot_matl.coc_x = cocx_weiss;
    plot_matl.coc_y = cocy_weiss;
    plot_matl.f = fx_weiss;
    plot_matl.zmp_x = mx_weiss;
    plot_matl.zmp_y = my_weiss;
    plot_matl.orient_z = orient_weiss;
    plot_matl.cop_x = copx_weiss;
    plot_matl.cop_y = copy_weiss;

    plot_matl.coc_xd = cocx_weissd;
    plot_matl.coc_yd = cocy_weissd;
    plot_matl.fd = fx_weissd;
    plot_matl.zmp_xd = mx_weissd;
    plot_matl.zmp_yd = my_weissd;
    plot_matl.orient_zd = orient_weissd;
    plot_matl.cop_xd = copx_weissd;
    plot_matl.cop_yd = copy_weissd;

    plot_matl.num_contours = num_contours;

    plot_matl.x_now = x;
    plot_matl.y_now = y;
    plot_matl.z_now = z;
    plot_matl.wx_now = wx;
    plot_matl.wy_now = wy;
    plot_matl.wz_now = wz;
    plot_matl.ww_now = ww;

    plot_matl.x_des = xd;
    plot_matl.y_des = yd;
    plot_matl.z_des = zd;
    plot_matl.wx_des = wxd;
    plot_matl.wy_des = wyd;
    plot_matl.wz_des = wzd;
    plot_matl.ww_des = wwd;

    plot_matl.x_c = xc;
    plot_matl.y_c = yc;
    plot_matl.z_c = zc;

    plot_matl.x_c1 = xc1;
    plot_matl.y_c1 = yc1;
    plot_matl.z_c1 = zc1;

    plot_matl.x_c2 = xc2;
    plot_matl.y_c2 = yc2;
    plot_matl.z_c2 = zc2;

    plot_matl.x_c3 = xc3;
    plot_matl.y_c3 = yc3;
    plot_matl.z_c3 = zc3;

    plot_matl.x_c4 = xc4;
    plot_matl.y_c4 = yc4;
    plot_matl.z_c4 = zc4;

    plot_matl.x_c5 = xc5;
    plot_matl.y_c5 = yc5;
    plot_matl.z_c5 = zc5;

    plot_matl.x_c6 = xc6;
    plot_matl.y_c6 = yc6;
    plot_matl.z_c6 = zc6;

    plot_matl.x_c7 = xc7;
    plot_matl.y_c7 = yc7;
    plot_matl.z_c7 = zc7;

    plot_matl.x_c8 = xc8;
    plot_matl.y_c8 = yc8;
    plot_matl.z_c8 = zc8;

    plot_matl.x_c9 = xc9;
    plot_matl.y_c9 = yc9;
    plot_matl.z_c9 = zc9;

    plot_matl.x_c10 = xc10;
    plot_matl.y_c10 = yc10;
    plot_matl.z_c10 = zc10;
    plot_matl.x_c11 = xc11;
    plot_matl.y_c11 = yc11;
    plot_matl.z_c11 = zc11;

    plot_matl.x_c12 = xc12;
    plot_matl.y_c12 = yc12;
    plot_matl.z_c12 = zc12;

    plot_matl.x_c13 = xc13;
    plot_matl.y_c13 = yc13;
    plot_matl.z_c13 = zc13;

    plot_matl.x_c14 = xc14;
    plot_matl.y_c14 = yc14;
    plot_matl.z_c14 = zc14;

    plot_matl.x_c15 = xc15;
    plot_matl.y_c15 = yc15;
    plot_matl.z_c15 = zc15;



    //NOc
      plot_matl.xNO_c = xNOc;
    plot_matl.yNO_c = yNOc;
    plot_matl.zNO_c = zNOc;

    plot_matl.xNO_c1 = xNOc1;
    plot_matl.yNO_c1 = yNOc1;
    plot_matl.zNO_c1 = zNOc1;

    plot_matl.xNO_c2 = xNOc2;
    plot_matl.yNO_c2 = yNOc2;
    plot_matl.zNO_c2 = zNOc2;

    plot_matl.xNO_c3 = xNOc3;
    plot_matl.yNO_c3 = yNOc3;
    plot_matl.zNO_c3 = zNOc3;

    plot_matl.xNO_c4 = xNOc4;
    plot_matl.yNO_c4 = yNOc4;
    plot_matl.zNO_c4 = zNOc4;

    plot_matl.xNO_c5 = xNOc5;
    plot_matl.yNO_c5 = yNOc5;
    plot_matl.zNO_c5 = zNOc5;

    plot_matl.xNO_c6 = xNOc6;
    plot_matl.yNO_c6 = yNOc6;
    plot_matl.zNO_c6 = zNOc6;

    plot_matl.xNO_c7 = xNOc7;
    plot_matl.yNO_c7 = yNOc7;
    plot_matl.zNO_c7 = zNOc7;

    plot_matl.xNO_c8 = xNOc8;
    plot_matl.yNO_c8 = yNOc8;
    plot_matl.zNO_c8 = zNOc8;

    plot_matl.xNO_c9 = xNOc9;
    plot_matl.yNO_c9 = yNOc9;
    plot_matl.zNO_c9 = zNOc9;

    plot_matl.xNO_c10 = xNOc10;
    plot_matl.yNO_c10 = yNOc10;
    plot_matl.zNO_c10 = zNOc10;
    plot_matl.xNO_c11 = xNOc11;
    plot_matl.yNO_c11 = yNOc11;
    plot_matl.zNO_c11 = zNOc11;

    plot_matl.xNO_c12 = xNOc12;
    plot_matl.yNO_c12 = yNOc12;
    plot_matl.zNO_c12 = zNOc12;

    plot_matl.xNO_c13 = xNOc13;
    plot_matl.yNO_c13 = yNOc13;
    plot_matl.zNO_c13 = zNOc13;

    plot_matl.xNO_c14 = xNOc14;
    plot_matl.yNO_c14 = yNOc14;
    plot_matl.zNO_c14 = zNOc14;

    plot_matl.xNO_c15 = xNOc15;
    plot_matl.yNO_c15 = yNOc15;
    plot_matl.zNO_c15 = zNOc15;
    plot_matl.is_in_contact_ur = is_in_contact_ur_;
    plot_matl.pixels_in_contact_ur = pixels_in_contact_ur_;
    plot_matl.highest_force_cell_ur = highest_force_cell_ur_;
    plot_matl.real_total_force_ur = real_total_force_ur_;

    plot_matl.current_state_ur = current_state_;

    if (is_recieved_new_joint_state_)
    {
      for (int ind = 0; ind < joint_values_.size(); ind++)
      {
	plot_matl.joint_angles.push_back(joint_values_.at(ind));
	plot_matl.joint_efforts.push_back(joint_efforts_.at(ind));

      }
      is_recieved_new_joint_state_ = 0;
      joint_values_.clear();
      joint_efforts_.clear();
    }


    if (is_img_rec)
    {
      for (int ind = 0; ind < img.size(); ind++)
      {
	plot_matl.img.push_back(img.at(ind));
      }
      is_img_rec = 0;
      img.clear();
    }

    if (!init_time)
    {
      test_time_begin = ros::Time::now();
    }
    // correct time
    test_time = ros::Time::now() - test_time_begin;

    if(init_time)
    {
      dt = ros::Time::now() - t_old_sec_total;
    }

    t_old_sec_total = ros::Time::now();

    if(init_time)
    {
      n_cycle = n_cycle + 1;
      plot_matl.dt = dt.toSec();
      test_time2 = dt.toSec() * n_cycle;
      // not correct time as the dt is not stable
      plot_matl.test_time2 = test_time2;
    }

    init_time = true;

    plot_matl.time = test_time.toSec();

    ROS_DEBUG_STREAM("all_recieved");
    pub_plot_matlab.publish(plot_matl);
    is_feat_des_rec_ = false;
    is_feat_fb_rec_ = false;
    is_pose_now_rec_ = false;
    is_pose_des_rec_ = false;

    img.clear();
    plot_matl.img.clear();
    plot_matl.joint_angles.clear();
    plot_matl.joint_efforts.clear();

    is_img_sent = 1;
  }

}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "plot_matlab_image");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);
    ROS_INFO("plot_matlab_image");
    PlotMatlab plot_matlab;
    while( ros::ok() )
    {
      plot_matlab.send_plot();
      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
}
