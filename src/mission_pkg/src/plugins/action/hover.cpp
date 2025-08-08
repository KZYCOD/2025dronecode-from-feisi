#include "plugins/action/hover.h"

Hover::Hover(const std::string &name, const NodeConfig &config)
  : StatefulActionNode(name, config)
{
  mct              = MavRosConnect::getInstance();
  nh               = mct->getROSHandle();
  fcu_pose_ptr     = mct->getFcuPose();
  fcu_state_ptr    = mct->getFcuState();
  tgt_pose_pub_ptr = std::make_shared<ros::Publisher>(*mct->getPosTgtPublier());
  cmd.coordinate_frame = mavros_msgs::PositionTarget::
    FRAME_LOCAL_NED;  // 选择控制坐标系，位置，速度，加速度使用local坐标系，姿态使用的是body坐标系
  cmd.type_mask = 0xfff;
  cmd.type_mask &=
    ~mavros_msgs::PositionTarget::FORCE;  // px4 不响应力的控制方式
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PX;
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PY;
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PZ;
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_YAW;
  is_innode = false;
  keep_time = 0;
  ROS_INFO("Call Hover Construct");
}

PortsList Hover::providedPorts()
{
  return { InputPort<bool>("stop_hover", "emit hover event"),
           InputPort<bool>("hover_is_end",
                           "if exit current node,be set ture"),
          InputPort<double>("is_time_ctrl",
                           "keep hover time")
                            };
}

NodeStatus Hover::onStart()
{
  /*这里参数本可以在行为树里面设置，但是目前黑板参数通过别的节点改不了，需后面排查，这里暂且使用ros
   * 参数服务器*/

  //可能在别的节点或者流程里面把下面的参数改变了，因此再次启动这个节点时，需要回复初始值
  ROS_INFO("in hover node ===================");
  is_innode = true;
  nh->setParam("hover_is_end_port", false);  //退出悬停节点状态
  nh->setParam("stop_hover_port", false);  //停止悬停的状态

  auto is_time_ctrl_port = getInput<double>("is_time_ctrl"); //悬停一个多少长时间
  if(!is_time_ctrl_port)
  {
    throw RuntimeError("error reading port [is_time_ctrl]:",
                       is_time_ctrl_port.error());
  }

  keep_time = is_time_ctrl_port.value();

  if(keep_time > 0)
  {
    flag_time = ros::Time::now();
  }

  if(fcu_state_ptr->mode != "OFFBOARD" || !fcu_state_ptr->armed)
  {
    ROS_ERROR("current fcu state has error");
    return NodeStatus::FAILURE;
  }
  else
  {
    cmd.position.x = fcu_pose_ptr->pose.position.x;
    cmd.position.y = fcu_pose_ptr->pose.position.y;
    cmd.position.z = fcu_pose_ptr->pose.position.z;
    
    tf2::Quaternion q(fcu_pose_ptr->pose.orientation.x, fcu_pose_ptr->pose.orientation.y, fcu_pose_ptr->pose.orientation.z, fcu_pose_ptr->pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    cmd.yaw = yaw;
    return NodeStatus::RUNNING;
  }
}

NodeStatus Hover::onRunning()
{
  //悬停节点除了开启或关闭节点，还需要一个退出节点的信号。
  //  auto stop_hover_port = getInput<bool>("stop_hover");
  //  auto is_end_port     = getInput<bool>("hover_is_end");
  std::this_thread::sleep_for(std::chrono::milliseconds(50));  // 
  bool is_end = false;
  nh->getParam("hover_is_end_port", is_end);


  if(is_end)
  {  //如果某个节点触发了结束当前节点，停止运行
    return NodeStatus::SUCCESS;
  }


  bool stop_hover = false;
  nh->getParam("stop_hover_port", stop_hover);

  if(!stop_hover && keep_time < 0)
  {
    ROS_INFO("keep hover ...");
    //cmd.position     = fcu_pose_ptr->pose.position;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = "hover_debug";
    tgt_pose_pub_ptr->publish(cmd);
    ros::spinOnce();
    return BT::NodeStatus::RUNNING; //发送完后返回
  }
  else
  {
    ROS_INFO("recv stop hover command"); //只是结束悬停，不是退出悬停节点，相当于挂起状态
  }
  


  if(keep_time > 0 )
  {
    ROS_INFO("keep_time:%f",keep_time);
    ROS_INFO("current_time: %f",ros::Time::now().toSec());
    ROS_INFO("flag_time: %f",flag_time.toSec());
    if(ros::Time::now() - flag_time < ros::Duration(keep_time))
    {
      // cmd.position     = fcu_pose_ptr->pose.position; 
      cmd.header.stamp = ros::Time::now();
      cmd.header.frame_id = "hover_debug";
      tgt_pose_pub_ptr->publish(cmd);
      ros::spinOnce();
        return BT::NodeStatus::RUNNING; //发送完后返回
    }
    else{
      ROS_INFO("use keep time is end");
      nh->setParam("stop_hover_port", true);
      return NodeStatus::SUCCESS;
    }
  }
  //如果时挂起时更新位置信息
    cmd.position.x = fcu_pose_ptr->pose.position.x;
    cmd.position.y = fcu_pose_ptr->pose.position.y;
    cmd.position.z = fcu_pose_ptr->pose.position.z;
    
    tf2::Quaternion q(fcu_pose_ptr->pose.orientation.x, fcu_pose_ptr->pose.orientation.y, fcu_pose_ptr->pose.orientation.z, fcu_pose_ptr->pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    cmd.yaw = yaw;
  return NodeStatus::RUNNING;
}

void Hover::onHalted()
{
  //  ROS_INFO("The havor node is end");
  ;
}

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<Hover>("Hover");
}
