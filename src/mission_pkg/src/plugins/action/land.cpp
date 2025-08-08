#include "plugins/action/land.h"

Land::Land(const std::string &name, const NodeConfig &config)
    : SyncActionNode(name, config)
{
  mct = MavRosConnect::getInstance();
  fcu_pose_ptr = mct->getFcuPose();
  fcu_state_ptr = mct->getFcuState();
  off_client_ptr = mct->getSetModeClient();
  arm_client_ptr = mct->getArmClient();
  tgt_pose_pub_ptr = mct->getPosTgtPublier();
  ;
}

PortsList Land::providedPorts()
{
  const char *description =
      "Land ... You can use the speed to control, or use AUTO.LAND mode";
  return {InputPort("use_speed", description),
          InputPort("speed_z", "land speed")};
}

NodeStatus Land::tick()
{
  ROS_INFO("to land ...");
  auto use_speed = getInput<int>("use_speed");
  if (!use_speed)
  {
    throw RuntimeError("error reading port [use_speed]: ", use_speed.error());
  }
  bool is_speed = use_speed.value() == 0 ? false : true;
  double speed_z = 0;
  if (is_speed)
  {
    auto speed_port = getInput<double>("speed_z");
    if (!speed_port)
    {
      throw RuntimeError("You choose using speed to control land，error "
                         "reading port [speed_z]",
                         speed_port.error());
    }
    speed_z = speed_port.value();
  }

  mavros_msgs::PositionTarget cmd;
  cmd.coordinate_frame = mavros_msgs::PositionTarget::
      FRAME_LOCAL_NED; // 选择控制坐标系，位置，速度，加速度使用local坐标系，姿态使用的是body坐标系
  cmd.type_mask = 0xfff;
  cmd.type_mask &=
      ~mavros_msgs::PositionTarget::FORCE; // px4 不响应力的控制方式
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_VX;
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_VY;
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_VZ;
  cmd.header.frame_id = "land";
  cmd.velocity.x = 0.0;
  cmd.velocity.y = 0.0;
  cmd.velocity.z = speed_z;
  mavros_msgs::CommandBool disArm;
  disArm.request.value = false;
  mavros_msgs::SetMode land;
  land.request.custom_mode = "AUTO.LAND";
  ros::Rate loop(20.0);

  if (fcu_state_ptr->mode != "OFFBOARD" || !fcu_state_ptr->armed)
  {
    ROS_ERROR("current mode of fcu not's OFFBOARD or fcu is disarm state");
    return NodeStatus::FAILURE;
  }
  ros::Time last_req = ros::Time::now();
  while (ros::ok && !ros::isShuttingDown())
  {
    if (!is_speed)
    {
      if (fcu_state_ptr->mode == "OFFBOARD" && (ros::Time::now() - last_req > ros::Duration(5.0)))
      {
        if (off_client_ptr->call(land) && land.response.mode_sent)
        {
          ROS_INFO("change mode (AUTO.LAND)");
          return NodeStatus::SUCCESS;
        }
        last_req = ros::Time::now();
      }
    }
    else if (is_speed)
    { // 速度控制降落
      static double last_z = fcu_pose_ptr->pose.position.z;
      if (abs(fcu_pose_ptr->pose.position.z - last_z) < 0.1 && ros::Time::now() - last_req > ros::Duration(3))
      { // 如果一段时间坐标没有发送变化了，飞机已经降落了，这时候可以上锁了，最好的方式更改is_speed的值
        is_speed = false;
      }
      else if (ros::Time::now() - last_req > ros::Duration(3))
      { // 3秒钟更新一下位置，和时间点
        last_z = fcu_pose_ptr->pose.position.z;
        last_req = ros::Time::now();
      }
      tgt_pose_pub_ptr->publish(cmd);
      ros::spinOnce();
    }

    loop.sleep();
  }
}
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<Land>("Land");
}
