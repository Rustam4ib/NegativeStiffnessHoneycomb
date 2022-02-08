class PoseShow
{
  public:
  PoseShow();
  ~PoseShow();
  bool not_first_time;
  void pose_init_create();
  tf::TransformListener listener; // listener object
  tf::StampedTransform stransform;
  bool is_transform_tool0_;
  tf::Vector3 init_position;
  tf::Transform init_transform;
  void br_init_pose();
  tf::TransformBroadcaster br_init_frame_; // object to send the transformations (publishing)

  void pose_des_create();
  tf::Transform des_transform;
  tf::TransformBroadcaster br_goal_frame_;
  void br_des_pose();

  tf::TransformListener listener_pose_now;
  void pose_now();
  tf::StampedTransform  nowtransform; // coordinate transformation: we store our resulting transform

  geometry_msgs::PoseStamped X_des_test_;
  geometry_msgs::PoseStamped X_now_test_;

  private:
  ros::NodeHandle nh;

};

//class with subscribers
class URutil
{
public:
  URutil();
  ~URutil();

  ros::NodeHandle n;
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

  ros::Publisher pub_velocities;
  void send_velocity(geometry_msgs::Twist in_vel);

  ros::Subscriber fb_feats_sub;
  void cb_fb_feats_sub(const tactile_servo_msgs::ContsFeatsConstPtr& msg_);
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
