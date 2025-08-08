/******************************************************************************
 * @file       hover.h
 * @brief 悬停节点，仅仅是在offbaord模式悬停，不包括起飞，一般作为过度节点使用
 * 比如，起飞后需要做一个动作，而这个动作的执行前需要一个较长的等待，那么在这段时间内飞机需要保持悬停
 * 需要注意的，一般使用悬停节点往往往是并行执行，然后通过一个信号触发结束悬停
 * 悬停节点有三种状态，悬停，停止悬停，退出悬停节点，如果需要并行执行，悬停节点的状态需要谨慎
 * @author     戴开世<daivesi@sina.com>
 * @date       2025/03/06
 * @history
 *****************************************************************************/
#ifndef HOVER_H
#define HOVER_H

#include "plugins/common.hpp"
#include "mavros_cnt.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>

using namespace BT;
class Hover : public StatefulActionNode
{
public:
  Hover(const std::string &name, const NodeConfig &config);

  static PortsList providedPorts();

  //节点开始时调用
  NodeStatus onStart() override;

  //节点运行时调用
  NodeStatus onRunning() override;

  //节点结束时调用
  void onHalted() override;
  void CleanValue();

private:
  std::shared_ptr<MavRosConnect>    mct;
  const geometry_msgs::PoseStamped *fcu_pose_ptr;
  const mavros_msgs::State *        fcu_state_ptr;
  ros::ServiceClient *              off_client_ptr;
  ros::ServiceClient *              arm_client_ptr;
  std::shared_ptr<ros::Publisher>   tgt_pose_pub_ptr;
  mavros_msgs::PositionTarget       cmd;
  std::shared_ptr<ros::NodeHandle>  nh;
  bool is_innode;
  double keep_time;
  ros::Time flag_time;
};

#endif  // HOVER_H
