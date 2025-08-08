/******************************************************************************
 * @file       detectobj.h
 * @brief
 * 这个节点就是一个开关，开启，关闭和过滤目标检测功能,至于具体的检测模块，
 * 或者目标结算都不应该在这个类里面执行
 *
 * @author     戴开世<daivesi@sina.com>
 * @date       2025/03/06
 * @history
 *****************************************************************************/

#ifndef DETECTOBJ_H
#define DETECTOBJ_H
#include "plugins/common.hpp"
#include "mavros_cnt.h"
#include "common_msgs/Objects.h"
#include "common_msgs/Obj.h"
#include "common_msgs/DetEnable.h"

using namespace BT;

class DetectObj : public StatefulActionNode
{
public:
  DetectObj(const std::string &name, const NodeConfig &config);

  void DetObjCallBack(const common_msgs::Objects::ConstPtr &msg);

  static PortsList providedPorts();

  //  NodeStatus tick() override;

  NodeStatus onStart() override;

  NodeStatus onRunning() override;

  void onHalted() override;

private:
  std::shared_ptr<MavRosConnect>    mct;
  std::shared_ptr<ros::NodeHandle>  nh;
  const geometry_msgs::PoseStamped *fcu_pose_ptr;
  const mavros_msgs::State *        fcu_state_ptr;
  const ros::Publisher *            tgt_pose_pub_ptr;
  std::string                       det_topic;
  common_msgs::Objects              objs;
  ros::Subscriber                   det_sub;
  ros::Publisher                    detect_cmd_pub;

  common_msgs::DetEnable det_;  //通过发型需要检测的目标给目标检测模块过滤
};

#endif  // DETECTOBJ_H
