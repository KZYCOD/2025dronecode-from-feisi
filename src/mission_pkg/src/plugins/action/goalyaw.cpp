#include "plugins/action/goalyaw.h"

GoalYaw::GoalYaw(const std::string &name, const NodeConfig &config)
    : SyncActionNode(name, config)
{
  mct = MavRosConnect::getInstance();
  nh = mct->getROSHandle();
  fcu_pose_ptr = mct->getFcuPose();
  fcu_state_ptr = mct->getFcuState();
  tgt_pose_pub_ptr = mct->getPosTgtPublier();
  cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  cmd.type_mask = 0xfff;
  cmd.type_mask &=
      ~mavros_msgs::PositionTarget::FORCE;                   // px4 不响应力的控制方式
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_YAW; // 放开yaw控制
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_YAW_RATE; //因为默认使用飞控内部的控制器，飞机旋转的速度很快，会导致位置偏移
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PX;
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PY;
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PZ;
  
  cmd.header.frame_id = "goal_yaw";
}

PortsList GoalYaw::providedPorts()
{
  return {InputPort("goal_yaw", "we need to control the fcu yaw"),
          InputPort("is_set_point","if we want set point"),
          InputPort("point","set we need point,but the is_set_point is true"),
          InputPort("yaw_rate","use we want yaw_rate")
  };
}

NodeStatus GoalYaw::tick()
{
  // if (!fcu_state_ptr->armed || fcu_state_ptr->mode != "OFFBOARD")
  // {
  //   throw RuntimeError(
  //       "to control fcu yaw fail, the dorne's state is node [in GoalYaw] "
  //       "offboard or not armed");
  //   return NodeStatus::FAILURE;
  // };
  ROS_INFO("in GoalYaw node");
  auto yaw_port = getInput<double>("goal_yaw");
 
  if (!yaw_port)
  {
    throw RuntimeError("error reading prot [goal_yaw]", yaw_port.error());
  }

  auto is_set_point_port = getInput<bool>("is_set_point");
  if(!is_set_point_port)
  {
    throw RuntimeError("error reading prot [is_set_point] in GoalYaw", is_set_point_port.error());
  }
  
  auto yaw_rate_port = getInput<double>("yaw_rate");
  if(!yaw_rate_port)
  {
    throw RuntimeError("error reading prot [yaw_rate] in GoalYaw", yaw_rate_port.error());
  }
  double yaw_rate = yaw_rate_port.value();
  bool is_set_point  = is_set_point_port.value();
  double yaw = yaw_port.value();
  if(is_set_point)
  {
    auto point_port = getInput<BT::Position3D>("point");
    if(!point_port)
    {
      throw RuntimeError("error reading prot [point] in GoalYaw", point_port.error());
    }
    auto point = point_port.value();
    ROS_INFO("in GoalYaw node ,the point is (%f,%f,%f)",point.x ,point.y , point.z);
    cmd.position.x = point.x;
    cmd.position.y = point.y;
    cmd.position.z = point.z;
  }
  else
  {
    cmd.position = fcu_pose_ptr->pose.position;
  }
  ros::Rate loop(20);
  while (ros::ok())
  {
    cmd.yaw = yaw * M_PI / 180;
    tgt_pose_pub_ptr->publish(cmd);
    ros::spinOnce();

    tf2::Quaternion q;
    tf2::fromMsg(fcu_pose_ptr->pose.orientation, q);
    tf2::Matrix3x3 rot(q);
    double yaw_, pitch_, roll_;
    rot.getRPY(roll_, pitch_, yaw_);
    ROS_INFO("fcu yaw: %f", yaw_);
    cmd.yaw_rate =yaw_rate; 
    if(yaw_ < 0)
    {
      cmd.yaw_rate = -yaw_rate;
    }
    auto det = abs(cmd.yaw) - abs(yaw_);
    ROS_INFO("fcu yaw: %f, det:%f", yaw_, det);
    if(abs(det) < 0.1)
    { 
      if(is_set_point && abs(fcu_pose_ptr->pose.position.x-cmd.position.x) < 0.2 
      && abs(fcu_pose_ptr->pose.position.y-cmd.position.y) < 0.2 
      && abs(fcu_pose_ptr->pose.position.z-cmd.position.z) < 0.2 )
      {
         ROS_INFO("current yaw(%f) contrl success", yaw);
        return NodeStatus::SUCCESS;
      }
      else if(!is_set_point)
      {
        ROS_INFO("current yaw(%f) contrl success", yaw);
        return NodeStatus::SUCCESS;
      }
      else{
        ROS_ERROR("happen erorr in GoalYaw");
      }
    }
    loop.sleep();
  }
}

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<GoalYaw>("GoalYaw");
}
