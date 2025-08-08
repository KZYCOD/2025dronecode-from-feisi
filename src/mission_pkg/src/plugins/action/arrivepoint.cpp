#include "plugins/action/arrivepoint.h"

ArrivePoint::ArrivePoint(const std::string &name, const NodeConfig &config)
  : SyncActionNode(name, config)
{
  mct          = MavRosConnect::getInstance();
  nh           = mct->getROSHandle();
  fcu_pose_ptr = mct->getFcuPose();
  tgt_pub_ptr  = std::make_shared<ros::Publisher>(*mct->getPosTgtPublier());

  goal_pub =
    nh->advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
}

PortsList ArrivePoint::providedPorts()
{
  return { InputPort<BT::Position3D>("goal", "drone fly to goal"),
           InputPort<int>("ctrl_type", "0:PosCtrl,1:Velocity,2:Acc,3:Planner"),
           InputPort<BT::Position3D>("velocity", "This is expect speed"),
           InputPort<BT::Position3D>("acc", "This is expect accelerate"),
           InputPort<double>("tolerance_d", "less than the value,it arrvied") };
}

NodeStatus ArrivePoint::tick()
{

  //获取目标点参数
  auto goal_port = getInput<Position3D>("goal");
  if(!goal_port)
  {
    throw RuntimeError("error reading port [goal]:", goal_port.error());
  }
  auto goal = goal_port.value();

  ROS_INFO("=========================goal: %f,%f,%f",goal.x,goal.y,goal.z);

  auto tol_port = getInput<double>("tolerance_d");
  if(!tol_port)
  {
    throw RuntimeError("error reading port[tolerance_d]: ", tol_port.error());
  }
  auto tolerance_d = tol_port.value();

  auto ctrlType_port = getInput<int>("ctrl_type");
  if(!ctrlType_port)
  {
    throw RuntimeError("error reading port [ctrl_type]:",
                       ctrlType_port.error());
  }    
  int type             = ctrlType_port.value();
  cmd.coordinate_frame = mavros_msgs::PositionTarget::
    FRAME_LOCAL_NED;  // 选择控制坐标系，位置，速度，加速度使用local坐标系，姿态使用的是body坐标系
  cmd.type_mask = 0xfff;
  cmd.type_mask &=
    ~mavros_msgs::PositionTarget::FORCE;  // px4 不响应力的控制方式

  if(type == 3)  //使用避障的方式，直接把目标点发给规划器
  {
    geometry_msgs::PoseStamped goal_plan;
    goal_plan.pose.position.x = goal.x;
    goal_plan.pose.position.y = goal.y;
    goal_plan.pose.position.z = goal.z;
    goal_plan.header.stamp    = ros::Time::now();
    goal_pub.publish(goal_plan);
    ros::spinOnce();
    return NodeStatus::SUCCESS;
  }
  else if(type == 0)
  {
    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PX;
    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PY;
    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PZ;
    cmd.position.x = goal.x;
    cmd.position.y = goal.y;
    cmd.position.z = goal.z;
  }
  else if(type == 1)
  {
    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_VX;
    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_VY;
    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_VZ;
    auto vel_prot = getInput<Position3D>("velocity");
    if(!vel_prot)
    {
      throw RuntimeError("error reading port [velocity]: ", vel_prot.error());
    }
    auto vel       = vel_prot.value();
    cmd.velocity.x = vel.x;
    cmd.velocity.y = vel.y;
    cmd.velocity.z = vel.z;
  }
  else if(type == 2)
  {
    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_AFX;
    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_AFY;
    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_AFZ;
    auto acc_prot = getInput<Position3D>("velocity");
    if(!acc_prot)
    {
      throw RuntimeError("error reading port [accelerate]: ", acc_prot.error());
    }
    auto acc       = acc_prot.value();
    cmd.velocity.x = acc.x;
    cmd.velocity.y = acc.y;
    cmd.velocity.z = acc.z;
  }

  ros::Rate loop(20);
  while(ros::ok() && !ros::isShuttingDown())
  {
    if(abs(fcu_pose_ptr->pose.position.x - goal.x) < tolerance_d
       && abs(fcu_pose_ptr->pose.position.y - goal.y) < tolerance_d
       && abs(fcu_pose_ptr->pose.position.z - goal.z) < tolerance_d)
    {
      return NodeStatus::SUCCESS;
    }
    tgt_pub_ptr->publish(cmd);
    ros::spinOnce();
    loop.sleep();
  }
}

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ArrivePoint>("ArriveGoal");
}
