#include "plugins/action/plannode_rviz.h"

PlanNodeRviz::PlanNodeRviz(const std::string name, const NodeConfiguration &config)
  : StatefulActionNode(name, config)
{
  mct                  = MavRosConnect::getInstance();
  nh                   = mct->getROSHandle();
  fcu_state_ptr        = mct->getFcuState();
  fcu_pose_ptr         = mct->getFcuPose();
  tgt_pose_pub_ptr     = mct->getPosTgtPublier();
  is_enable_planner    = false;
  has_recv_cmd         = false;
  cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  cmd.type_mask        = 0xfff;
  cmd.type_mask &=
    ~mavros_msgs::PositionTarget::FORCE;  // px4 不响应力的控制方式
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PX;
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PY;
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PZ;
  // cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
  // cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_YAW;
  last_time = ros::Time::now();

  std::string goal_topic;
  nh->param<std::string>("goal_topic", goal_topic, "/planner/goal");
  
  std::string rviz_topic;
  nh->param<std::string>("rviz_topic", rviz_topic, "/move_base_simple/goal");
  
  std::string cmd_topic;
  nh->param<std::string>("planner_cmd_topic", cmd_topic, "/pose_cmd");
  nh->param<bool>("enable_planner_port",is_enable_planner,false);

  planer_goal_recv = nh->subscribe<geometry_msgs::PoseStamped>(rviz_topic, 1,bind(&PlanNodeRviz::GoalRecv,this,_1));
  planner_goal_pub = nh->advertise<geometry_msgs::PoseStamped>(goal_topic,1);

  pos_cmd_sub = nh->subscribe<quadrotor_msgs::PositionCommand>(
    cmd_topic, 10, bind(&PlanNodeRviz::PositionCmdCB, this, _1));
}

PortsList PlanNodeRviz::providedPorts()
{
  return { InputPort("enable_planner", "open or close planner"),
           InputPort("planner_ctrl_type",
                     "choose the control type,0:pose,1:volicty,2:accelerate") };
}

void PlanNodeRviz::PositionCmdCB(
  const quadrotor_msgs::PositionCommand::ConstPtr &msg)
{
  if(!is_enable_planner)
    return;  //如果不开启路径规划，直接返回
  cmd.position.x              = -msg->position.y;
  cmd.position.y              = msg->position.x;
  cmd.position.z              = msg->position.z;
  cmd.velocity.x              = -msg->velocity.y;
  cmd.velocity.y              = msg->velocity.x;
  cmd.velocity.z            = msg->velocity.z;
  cmd.acceleration_or_force.x = -msg->acceleration.y;
  cmd.acceleration_or_force.y = msg->acceleration.x;
  cmd.acceleration_or_force.z = msg->acceleration.z;
  // cmd.position = msg->position;
  cmd.yaw                   = msg->yaw - M_PI_2;
  // cmd.yaw                   = msg->yaw;
  cmd.yaw_rate              = msg->yaw_dot;
  last_time                 = ros::Time::now();
  if(!has_recv_cmd)
    has_recv_cmd = true;
}

void PlanNodeRviz::GoalRecv(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  goal = *msg;
  goal.header.stamp = ros::Time::now();
  goal.pose.position.z = 1;
  planner_goal_pub.publish(goal);
  ros::spinOnce();
  ROS_INFO("RECV GOAL x:%f,y:%f,z:%f",goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
}

NodeStatus PlanNodeRviz::onStart()
{
  if(fcu_state_ptr->mode != "OFFBOARD" || !fcu_state_ptr->armed)
    return NodeStatus::FAILURE;

  auto ret = getInput<bool>("enable_planner");

  if(!ret)
  {
    throw RuntimeError("error reading prot [enable_planner]", ret.error());
  }

  is_enable_planner = ret.value();
  if(!is_enable_planner)
  {
    ROS_WARN("is not open planner");
  }
  else
  {
    ROS_INFO("is enbale planner "); 
  }
  auto ret_ = getInput<int>("planner_ctrl_type");
  if(!ret_)
  {
    throw RuntimeError("error reading prot [enable_planner]", ret_.error());
  }

  ROS_INFO("planner ctrl type is %d", ret_.value());
  

  int type = ret_.value();
  if(type == 1)
  {  //速度控制
    cmd.type_mask = 0xfff;
    cmd.type_mask &=
      ~mavros_msgs::PositionTarget::FORCE;  // px4 不响应力的控制方式
    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_VX;
    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_VY;
    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_VZ;
    ROS_INFO("use velocity control");
  }
  if(type == 2)
  {  //加速度控制
    cmd.type_mask = 0xfff;
    cmd.type_mask &=
      ~mavros_msgs::PositionTarget::FORCE;  // px4 不响应力的控制方式
    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_AFX;
    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_AFY;
    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_AFZ;
    ROS_INFO("use accelerate control");
  }
  else{
    ROS_INFO("use position control");
  }
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_YAW;

  return NodeStatus::RUNNING;

}

NodeStatus PlanNodeRviz::onRunning()
{  // 监听开关

  auto ret = getInput<bool>("enable_planner");

  if(!ret)
  {
    throw RuntimeError("error reading prot [enable_planner]", ret.error());
  }
  
  if(!has_recv_cmd)
  {
    cmd.position.x = -fcu_pose_ptr->pose.position.y;
    cmd.position.y = fcu_pose_ptr->pose.position.x;
    cmd.position.z = fcu_pose_ptr->pose.position.z;
    cmd.position = fcu_pose_ptr->pose.position;
    cmd.yaw = -M_PI_2;
    tgt_pose_pub_ptr->publish(cmd);
    ros::spinOnce();
    // ROS_INFO("keep hover");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    return NodeStatus::RUNNING;
  }

  is_enable_planner = ret.value();
  // if(is_enable_planner && ros::Time::now() - last_time < ros::Duration(3))
  {  
    cmd.header.frame_id="plann_node";
    tgt_pose_pub_ptr->publish(cmd);
    ros::spinOnce();
  }

  // if( (abs(fcu_pose_ptr->pose.position.x - goal.pose.position.x) < 0.15 
  // && abs(fcu_pose_ptr->pose.position.y - goal.pose.position.y) < 0.15 
  // && abs(fcu_pose_ptr->pose.position.z - goal.pose.position.z) < 0.15)
  // || ros::Time::now() - last_time > ros::Duration(5))
  // { //有些路径规划节点可能到了终点了也会一直发终点的话题，因此这里不能仅仅用时间去判断是否以及到终点
  //   ROS_INFO("has over 5 seconds not update cmd");
  //   // return NodeStatus::SUCCESS;
  // }

  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  return NodeStatus::RUNNING;
  ;
}

void PlanNodeRviz::onHalted()
{
  ROS_WARN(
    "exit planner node ,maybe the planner has too long time not update cmd!");
  ;
}
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<PlanNodeRviz>("PlanNodeRviz");
}
