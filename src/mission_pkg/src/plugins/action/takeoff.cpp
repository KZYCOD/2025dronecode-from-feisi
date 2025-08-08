#include "plugins/action/takeoff.h"

Takeoff::Takeoff(const std::string &name, const NodeConfig &config)
    : SyncActionNode(name, config)
{
  mct = MavRosConnect::getInstance();
  fcu_pose_ptr = mct->getFcuPose();
  fcu_state_ptr = mct->getFcuState();
  off_client_ptr = mct->getSetModeClient();
  arm_client_ptr = mct->getArmClient();
  tgt_pose_pub_ptr = std::make_shared<ros::Publisher>(*mct->getPosTgtPublier());
}

PortsList Takeoff::providedPorts()
{
  const char *description = "Takeoff position ... ";
  return {InputPort<Position3D>("goal", description),
          InputPort("is_rc", "is remote controller operator")};
}

NodeStatus Takeoff::tick()
{
  auto goal_ = getInput<Position3D>("goal");
  if (!goal_)
  {
    throw RuntimeError("error reading port [goal]:", goal_.error());
  }
  auto goal = goal_.value();

  auto is_rc_ = getInput<int>("is_rc");
  if (!is_rc_)
  {
    throw RuntimeError("error reading port [is_rc]:", is_rc_.error());
  }
  ROS_INFO("takeoff goal = %f, %f, %f", goal.x, goal.y, goal.z);
  // 判断是不是通过遥控进行解锁和切offboard
  bool is_rc = is_rc_.value() == 0 ? false : true;

  mavros_msgs::PositionTarget cmd;
  cmd.coordinate_frame = mavros_msgs::PositionTarget::
      FRAME_LOCAL_NED; // 选择控制坐标系，位置，速度，加速度使用local坐标系，姿态使用的是body坐标系
  cmd.type_mask = 0xfff;
  cmd.type_mask &=
      ~mavros_msgs::PositionTarget::FORCE; // px4 不响应力的控制方式
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PX;
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PY;
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PZ;
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_YAW; 
  cmd.header.frame_id = "takeoff";
  cmd.position.x = goal.x;
  cmd.position.y = goal.y;
  cmd.position.z = goal.z;
  cmd.yaw = 0.0;
  cmd.header.stamp = ros::Time::now();

  mavros_msgs::SetMode mode;
  mode.request.custom_mode = "OFFBOARD";

  mavros_msgs::CommandBool armed;
  armed.request.value = true;
  ros::Time last_request = ros::Time::now();
  ros::Rate loop(20);
  for (int i = 0; i < 100; ++i)
  {

    tgt_pose_pub_ptr->publish(cmd);
    ros::spinOnce();
    loop.sleep();
  }
  while (ros::ok() && !ros::isShuttingDown())
  {

    tgt_pose_pub_ptr->publish(cmd);
    ros::spinOnce();
    if (abs(fcu_pose_ptr->pose.position.x - goal.x)<0.15 
    && abs(fcu_pose_ptr->pose.position.y - goal.y)<0.15 
    && abs(fcu_pose_ptr->pose.position.z - goal.z) < 0.15 
    && fcu_state_ptr->armed && fcu_state_ptr->mode == "OFFBOARD")
    {
      ROS_INFO(" takeoff finished ..... ");
      return NodeStatus::SUCCESS;
    }
    if (!is_rc)
    { // 如果不使用遥控，直接程序切offboard 解锁
      if (fcu_state_ptr->mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
      {
        if (off_client_ptr->call(mode) && mode.response.mode_sent)
        {
          ROS_INFO("Offboard enabled");
        }
        last_request = ros::Time::now();
      }
      else if (!fcu_state_ptr->armed && ros::Time::now() - last_request > ros::Duration(5.0))
      {
        // 解锁飞控
        ROS_INFO("to arming");
        if (arm_client_ptr->call(armed) && armed.response.success)
        {
          ROS_INFO("Vehicle armed");
        }
        last_request = ros::Time::now();
      }
    }
    loop.sleep();
  }
}
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<Takeoff>("Takeoff");
}
