#include "plugins/action/plannode.h"

PlanNode::PlanNode(const std::string name, const NodeConfiguration &config)
    : StatefulActionNode(name, config)
{
  mct = MavRosConnect::getInstance();
  nh = mct->getROSHandle();
  fcu_state_ptr = mct->getFcuState();
  fcu_pose_ptr = mct->getFcuPose();
  tgt_pose_pub_ptr = mct->getPosTgtPublier();
  is_enable_planner = false;
  is_pause_planner = false;
  is_stop_planner = false;
  has_recv_cmd = false;
  nh->param<bool>("is_stop_planner", is_stop_planner, false);
  nh->param<bool>("is_pause_planner",is_pause_planner,false);
  cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  cmd.type_mask = 0xfff;
  cmd.type_mask &=
      ~mavros_msgs::PositionTarget::FORCE; // px4 不响应力的控制方式
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PX;
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PY;
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PZ;
  // cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_YAW_RATE; //角速度，角度只能控制一个
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_YAW;
  last_time = ros::Time::now();
  cmd.header.frame_id = "planner_node";
  ROS_INFO("Call Planner Construct");
}

PortsList PlanNode::providedPorts()
{
  return {InputPort("goal_position", "the goal position"),
          InputPort("goal_ori", "the goal orientation, [roll, pitch, yaw]"),
          InputPort("enabel_planner", "open or close planner"),
          InputPort("planner_ctrl_type",
                    "choose the control type,0:pose,1:volicty,2:accelerate"),
          InputPort("goal_src", "0: xml set; 1:program inner;3: rviz"),
          InputPort("enabel_yaw", "use trajectory yaw"),
          InputPort("enabel_yaw_rate", "use trajecotry yaw_rate")};      
}

void PlanNode::PositionCmdCB(
    const quadrotor_msgs::PositionCommand::ConstPtr &msg)
{
  last_time = ros::Time::now();
  if (!is_enable_planner)
    return; // 如果不开启路径规划，直接返回
  
  goal_pos = msg->goal_pos;
  cmd.position = msg->position;
  cmd.velocity = msg->velocity;
  cmd.acceleration_or_force = msg->acceleration;
  cmd.yaw = msg->yaw;
  cmd.yaw_rate = msg->yaw_dot;
  if (is_innode && !has_recv_cmd )  //只有当使用这个节点的时候才放开
    has_recv_cmd = true;
  if(!is_innode)
  { //因为回调函数会一直挂起，所以这里再一次确保没有使用plann节点时，保证状态正确
    has_recv_cmd = false;
  }
}

void PlanNode::GoalCB(const geometry_msgs::PoseStamped::ConstPtr &msg)
{ //可以订阅当前节点发的目标点，也可以订阅其他节点发的目标点
  goal_recv = *msg;
}

NodeStatus PlanNode::onStart()
{
  if (fcu_state_ptr->mode != "OFFBOARD" || !fcu_state_ptr->armed)
    return NodeStatus::FAILURE;
  ROS_INFO("in planner node");
  std::string goal_topic;
  nh->param<std::string>("goal_topic", goal_topic, "/move_base_simple/goal");
  std::string cmd_topic;
  nh->param<std::string>("planner_cmd_topic", cmd_topic, "/pose_cmd");
  goal_recv.pose.position.x = 0;
  goal_recv.pose.position.y = 0;  
  goal_recv.pose.position.z = 0;
  planer_goal_pub = nh->advertise<geometry_msgs::PoseStamped>(goal_topic, 10);
  pos_cmd_sub = nh->subscribe<quadrotor_msgs::PositionCommand>(
      cmd_topic, 10, bind(&PlanNode::PositionCmdCB, this, _1));
  goal_sub = nh->subscribe<geometry_msgs::PoseStamped>(
      "/move_base_simple/goal", 10, bind(&PlanNode::GoalCB, this, _1));
  auto ret_ = getInput<int>("planner_ctrl_type");
  is_innode = true;
  if (!ret_)
  {
    throw RuntimeError("error reading prot [enable_planner]", ret_.error());
  }

  ROS_INFO("planner ctrl type is %d", ret_.value());
  has_recv_cmd = false;

  nh->getParam("is_stop_planner", is_stop_planner);
  if (is_stop_planner)
  { // 如果别的节点结束了当前节点，直接退出
    ROS_WARN("planner node has recv other request , exit");
    // is_stop_planner = false;
    return NodeStatus::SUCCESS;
  }

  nh->setParam("is_pause_planner",false);
  auto goal_src_port = getInput<int>("goal_src"); // 路径规划节点来源
  // 如果目标源来自程序内，这里就不需要接受xml里面传过来的参数
  if (!goal_src_port)
  {
    throw RuntimeError("error reading prot [goal_src]", goal_src_port.error());
  }
  goal_src = goal_src_port.value();
  goal.header.frame_id = "PlannnNode";
  // 监听开关
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  auto ret = getInput<bool>("enabel_planner");
  is_enable_planner = ret.value();
  if (!ret)
  {
    throw RuntimeError("error reading prot [enabel_planner]", ret.error());
  }
  if (!is_enable_planner)
  {
    ROS_WARN("is not open planner");
    return NodeStatus::FAILURE;
  }
  int type = ret_.value();
  if (type == 1)
  { // 速度控制
    cmd.type_mask = 0xfff;
    cmd.type_mask &=
        ~mavros_msgs::PositionTarget::FORCE; // px4 不响应力的控制方式
    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_VX;
    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_VY;
    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_VZ;
  }
  if (type == 2)
  { // 加速度控制
    cmd.type_mask = 0xfff;
    cmd.type_mask &=
        ~mavros_msgs::PositionTarget::FORCE; // px4 不响应力的控制方式
    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_AFX;
    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_AFY;
    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_AFZ;
  }
  if (goal_src == 1)
  {
    // cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    cmd.header.frame_id = "plann_inner_node";
    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_YAW;
  }
  
  if (!has_recv_cmd)
  {
    if (goal_src == 0)
    { // 如果是使用外部给的目标点
      auto ret = getInput<Position3D>("goal_position");
      if (!ret)
      {
        throw RuntimeError("reading goal_position fail", ret.error());
      }
      Position3D pos = ret.value();
      goal.pose.position.x = pos.x;
      goal.pose.position.y = pos.y;
      goal.pose.position.z = pos.z;

      auto ret_ = getInput<Position3D>("goal_ori");
      if (!ret_)
      {
        throw RuntimeError("error reading port [goal_ori]:", ret_.error());
      }
      Position3D ori = ret_.value();

      tf2::Quaternion q;
      q.setRPY(ori.x, ori.y, ori.z);
      goal.pose.orientation.x = q.x();
      goal.pose.orientation.y = q.y();
      goal.pose.orientation.z = q.z();
      goal.pose.orientation.w = q.w();
      ROS_INFO("in planner node goal position is (%f,%f,%f)",
           goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
      ROS_INFO("in planner node goal orientation is (%f,%f,%f)",
           goal.pose.orientation.x, goal.pose.orientation.y,
           goal.pose.orientation.z);
      planer_goal_pub.publish(goal);
      // ROS_INFO("Planner OnStart current fcu pose is (%f,%f,%f)",
      //          fcu_pose_ptr->pose.position.x, fcu_pose_ptr->pose.position.y,
      //          fcu_pose_ptr->pose.position.z);
      ros::spinOnce();
    }
    else if (goal_src == 1)
    { // 使用别的节点给目标点
      ROS_INFO("use inner goal postion"); 
      return NodeStatus::RUNNING;
    }
    else if (goal_src == 2)
    {                             // 使用rviz
      return NodeStatus::RUNNING; // 如果在onStart()里面返回SUCCESS 节点将会结束
    }
  }
  ROS_INFO("current fcu pose is (%f,%f,%f)",
           fcu_pose_ptr->pose.position.x, fcu_pose_ptr->pose.position.y,
           fcu_pose_ptr->pose.position.z);
  return NodeStatus::RUNNING; // 如果在onStart()里面返回SUCCESS 节点将会结束
}

NodeStatus PlanNode::onRunning()
{
  // planer_goal_pub.publish(goal);
  // ros::spinOnce();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  nh->getParam("is_stop_planner", is_stop_planner);
  nh->getParam("is_pause_planner",is_pause_planner);
  if(is_stop_planner)
  {//如果接收到结束路径规划
    ROS_INFO("recv stop planner cmd");
    has_recv_cmd = false;  //重置收到命令状态
    CleanValue();
    return NodeStatus::SUCCESS;
  }

  bool stop_hover = false;
  // bool hover_is_end = false;

  if (has_recv_cmd)
  {
    // ROS_INFO("last_tra_id: %d, tra_id: %d",last_tra_id,tra_id);

    // if(last_tra_id != 0  && last_tra_id == tra_id)
    // {//说明节点重新启动后,发送的仍然时上次的轨迹数据，依然保持悬停,等待轨迹更新
      
    //   ROS_WARN("has new goal ,but the trajectroy is last time");
    //   return NodeStatus::RUNNING;
    // }
    //下面是一个补丁，@todo youself
    if(goal_src == 1)
    {//因为程序异步执行，这里接受到轨迹不一定使是我们想要的轨迹
      // if(abs(cmd.yaw) < 1)
      // { 这里是一个补丁
      //   return NodeStatus::RUNNING;
      // }

      if(abs(goal_pos.x - goal_recv.pose.position.x) > 0.5 || abs(goal_pos.y - goal_recv.pose.position.y) > 0.5 || abs(goal_pos.z - goal_recv.pose.position.z) > 0.5)
      { //如果接收到的轨迹点和上次的轨迹点差别大，说明是还是上一次的轨迹数据，保持悬停，不响应动作
        ROS_WARN("has new goal, but the goal is same as last time");
        return NodeStatus::RUNNING;
      }

    }
    if (!stop_hover)
    {                                          // 接受到规划的轨迹了，就不需要保持悬停了。
      nh->setParam("stop_hover_port", true);   // 结束悬停，该按照轨迹来飞了
      // nh->setParam("hover_is_end_port", true); // 悬停节点退出信号
      // nh->getParam("hover_is_end_port", hover_is_end);
      nh->getParam("stop_hover_port", stop_hover);
      // return NodeStatus::RUNNING;
    }
    //  setOutput("stop_hover", false);
    if (!stop_hover) // 存在一种可能，就是有轨迹点了，但是hover状态还没切换
    {                                 // 如果悬停节点还没退出不发送轨迹指令
      ROS_WARN("has recv trajectory, but the hover node is not exit");
      return NodeStatus::RUNNING;
    }
    if (goal_src == 0)
    { // 使用xml内节点指令，可以继续发
      //  没收到轨迹指令，可以继续发目标点
      planer_goal_pub.publish(goal);
      ros::spinOnce();
    }
  }
  else
  {
    ROS_INFO("not recv tra cmd");
    if (goal_src == 0)
    { // 使用xml内节点指令，可以继续发
      //  没收到轨迹指令，可以继续发目标点
      planer_goal_pub.publish(goal);
      ros::spinOnce();
    }
    else if (goal_src == 1)
    { // 等待其他节点程序发送gaol
      ;
    }
    return NodeStatus::RUNNING;
  }

  if (is_enable_planner /*&& ros::Time::now() - last_time < ros::Duration(3)*/)
  {   
    auto yaw_port = getInput<bool>("enabel_yaw");
    auto yaw_rate_port = getInput<bool>("enabel_yaw_rate");

    if(!yaw_port)
    {
      throw RuntimeError("read yaw_port fial",yaw_port.error());
    }
    if(!yaw_rate_port)
    {
      throw RuntimeError("read  yaw_rate_port fial",yaw_rate_port.error());
    }

    auto yaw_en = yaw_port.value();
    auto yaw_rate_en = yaw_rate_port.value();
    if(!yaw_en)
    {
      cmd.yaw = 0.0;
    }
    if(!yaw_rate_en)
    { //只能控制其一
      cmd.yaw_rate = 0.0;
    }
    if(is_pause_planner)
    { //如果暂停轨迹控制，控制权移交给其他节点
      ROS_INFO("planner is pause");
      return NodeStatus::RUNNING;
    }
    if(goal_src == 1)
    {
      ROS_INFO("cmd value: pose:(%f,%f,%f), vel:(%f,%f,%f) yaw:%f",cmd.position.x,cmd.position.y,cmd.position.z,
      cmd.velocity.x,cmd.velocity.y,cmd.velocity.z,cmd.yaw);
    }
    tgt_pose_pub_ptr->publish(cmd);
    ros::spinOnce();
  }

  if ((abs(fcu_pose_ptr->pose.position.x - goal.pose.position.x) < 0.2 && abs(fcu_pose_ptr->pose.position.y - goal.pose.position.y) < 0.2 && abs(fcu_pose_ptr->pose.position.z - goal.pose.position.z) < 0.2) /*|| ros::Time::now() - last_time > ros::Duration(5)*/)
  { // 有些路径规划节点可能到了终点了也会一直发终点的话题，因此这里不能仅仅用时间去判断是否以及到终点
    ROS_INFO("fcu_pose: %f,%f,%f", fcu_pose_ptr->pose.position.x, fcu_pose_ptr->pose.position.y, fcu_pose_ptr->pose.position.z);
    ROS_INFO("fcu_pose: %f,%f,%f", goal.pose.position.x, goal.pose.position.x, goal.pose.position.x);
    ROS_INFO("now time %f, last_time: %f", ros::Time::now().toSec(), last_time.toSec());
    ROS_INFO("planner node return success");
    CleanValue();
    return NodeStatus::SUCCESS;
  }
  return NodeStatus::RUNNING;
  ;
}
void PlanNode::CleanValue()
{
  is_innode = false;
  has_recv_cmd = false;
  pos_cmd_sub.shutdown();
  goal_sub.shutdown();
  planer_goal_pub.shutdown();
  nh->setParam("is_stop_planner",false);
  nh->setParam("is_stop_planner",false);
  nh->setParam("is_pause_planner", false);
}

void PlanNode::onHalted()
{

  ROS_WARN(
      "exit planner node, happend error");
}
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<PlanNode>("PlanNode");
}
