/******************************************************************************
 * @file       matchctrlchoose.h
 * @brief      这个主要是根据选择行为控制飞机飞行的方式
 *
 * @author     戴开世<daivesi@sina.com>
 * @date       2025/03/27
 * @history
 *****************************************************************************/

#ifndef MATCHCTRLCHOOSE_H
#define MATCHCTRLCHOOSE_H
#include "plugins/common.hpp"

using namespace BT;
class MatchCtrlChoose : public ConditionNode
{
public:
  MatchCtrlChoose(const std::string &name, const NodeConfig &config);

  static PortsList providedPorts();

  NodeStatus tick() override;
};

#endif  // MATCHCTRLCHOOSE_H
