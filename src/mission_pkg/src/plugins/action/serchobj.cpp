#include "plugins/action/serchobj.h"

SerchObj::SerchObj(const std::string &name, const NodeConfig &config)
  : StatefulActionNode(name, config)
{
}

PortsList SerchObj::providedPorts()
{  //需要指定class name
  return { InputPort<int>("class_id", "need tu find object") };
}

NodeStatus SerchObj::onStart()
{
  return NodeStatus::SUCCESS;
  ;
}

NodeStatus SerchObj::onRunning()
{
  return NodeStatus::SUCCESS;
}

void SerchObj::onHalted()
{
  ;
}
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<SerchObj>("SerchObj");
}
