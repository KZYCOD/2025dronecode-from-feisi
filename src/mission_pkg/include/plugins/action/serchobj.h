/******************************************************************************
 * @file       serchobj.h
 * @brief
 *搜索目标，这里是气球。撞击气球先搜索，当然在控制撞击气球的是可能会再一次丢失，
 * 需要继续搜索，因此这个节点结束的条件是目标被摧毁，目标摧毁后，有撞击节点发出结束搜索指令，当前节点完成任务
 *
 * @author     戴开世<daivesi@sina.com>
 * @date       2025/03/07
 * @history
 *****************************************************************************/

#ifndef SERCHOBJ_H
#define SERCHOBJ_H
#include "plugins/common.hpp"
#include "mavros_cnt.h"

using namespace BT;

class SerchObj : public StatefulActionNode
{
public:
  SerchObj(const std::string &name, const NodeConfig &config);

  static PortsList providedPorts();

  NodeStatus onStart() override;
  NodeStatus onRunning() override;
  void       onHalted() override;

private:
};

#endif  // SERCHOBJ_H
