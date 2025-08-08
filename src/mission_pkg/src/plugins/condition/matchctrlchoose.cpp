#include "plugins/condition/matchctrlchoose.h"

MatchCtrlChoose::MatchCtrlChoose(const std::string &name,
                                 const NodeConfig & config)
  : ConditionNode(name, config)
{
  ;
}

PortsList MatchCtrlChoose::providedPorts()
{
  return { InputPort<int>("config_param"), InputPort<int>("expected_value") };
}

NodeStatus MatchCtrlChoose::tick()
{
  int config_param, expected_value;

  // 获取输入端口的值
  if(!getInput("config_param", config_param))
  {
    throw BT::RuntimeError("Missing required input [config_param]");
  }
  if(!getInput("expected_value", expected_value))
  {
    throw BT::RuntimeError("Missing required input [expected_value]");
  }

  // 比较输入值
  if(config_param == expected_value)
  {
    return BT::NodeStatus::SUCCESS;  // 条件满足
  }
  else
  {
    return BT::NodeStatus::FAILURE;  // 条件不满足
  };
}
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<MatchCtrlChoose>("MatchCtrlChoose");
}
