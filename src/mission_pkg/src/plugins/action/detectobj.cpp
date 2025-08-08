#include "plugins/action/detectobj.h"

DetectObj::DetectObj(const std::string &name, const NodeConfig &config)
  : StatefulActionNode(name, config)
{
  mct = MavRosConnect::getInstance();
  //需要根据当前飞机位姿，计算目标在全局坐标系下的位置
  fcu_pose_ptr     = mct->getFcuPose();
  tgt_pose_pub_ptr = mct->getPosTgtPublier();
  nh               = mct->getROSHandle();

  nh->param<std::string>("det_topic", this->det_topic, "/objects");

  det_sub = nh->subscribe<common_msgs::Objects>(
    det_topic, 10, bind(&DetectObj::DetObjCallBack, this, _1));
  std::string detect_ctrl_topic;
  nh->param<std::string>("detect_ctrl_topic", detect_ctrl_topic,
                         "/detect_ctrl");
  detect_cmd_pub = nh->advertise<common_msgs::DetEnable>(detect_ctrl_topic, 10);
}

PortsList DetectObj::providedPorts()
{
  return { InputPort("class_name", "input class name"),
           InputPort<bool>("stop_detect"), OutputPort<bool>("stop_hover") };
}

NodeStatus DetectObj::onStart()
{
  //可能在别的节点或者流程里面把下面的参数改变了，因此再次启动这个节点时，需要回复初始值
  nh->setParam("stop_detect_port", false);

  //开启检测功能,
  std::cout << "start object detect " << std::endl;
  auto str = getInput<std::string>("class_name");

  if(!str)
  {
    throw RuntimeError("read class name faile [class name]: ", str.error());
  }

  std::stringstream ss(str.value());

  det_.header.stamp = ros::Time::now();

  std::string item;
  while(std::getline(ss, item, ','))
  {
    det_.class_name.push_back(item);
  }
  det_.is_open = true;
  detect_cmd_pub.publish(det_);
  ros::spinOnce();
  //  setOutput("stop_hover", false);  // 让飞机悬停
  return NodeStatus::RUNNING;
}

NodeStatus DetectObj::onRunning()
{
  // 检测过程中 ,遇到stop detect 信号，发布关闭检测功能，并成功返回
  //  auto stop_detect_port = getInput<bool>("stop_detect");

  //  if(!stop_detect_port)
  //  {
  //    throw BT::RuntimeError("Missing required input: stop_detect_port");
  //  }
  bool stop_detect;
  nh->getParam("stop_detect_port", stop_detect);

  if(stop_detect)
  {
    //在别的节点触发停止检测信号，让检测模块停止工作
    std::cout << "recv stop detect signal " << std::endl;

    det_.is_open = false;
    detect_cmd_pub.publish(det_);
    ros::spinOnce();
    return BT::NodeStatus::SUCCESS;  // 结束执行
  }
  else
  {
    detect_cmd_pub.publish(det_);
    ros::spinOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return BT::NodeStatus::RUNNING;
  }
}

void DetectObj::onHalted()
{

  ROS_INFO("close detect ");
}

void DetectObj::DetObjCallBack(const common_msgs::Objects::ConstPtr &msg)
{  //这个目标在这里没有，如果后期需要可以反馈到大模型那边
  objs = *msg;
}
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<DetectObj>("DetectObj");
}
