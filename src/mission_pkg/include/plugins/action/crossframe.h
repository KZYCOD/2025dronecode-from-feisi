/******************************************************************************
 * @file       crossframe.h
 * @brief      XXXX Function
 * * 穿框节点，包含连个子节点，1.是否识别检测到框 2.执行穿动作
 * 需要注意，这里制作穿框动作，至于飞机还有前面的起飞动作啥的，这个类不关心
 * @author     戴开世<daivesi@sina.com>
 * @date       2025/03/06
 * @history
 *****************************************************************************/
#ifndef CROSSFRAME_H
#define CROSSFRAME_H
#include "plugins/common.hpp"
#include "mavros_cnt.h"
#include "common_msgs/Objects.h"
#include "common_msgs/Obj.h"
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mutex>

using namespace BT;

class CrossFrame : public StatefulActionNode
{
public:
  CrossFrame(const std::string &name, const NodeConfiguration &config);

  void RecvObj(const common_msgs::Objects::ConstPtr &msg);

  static PortsList providedPorts();

  //  NodeStatus tick() override;

  NodeStatus onStart() override;
  NodeStatus onRunning() override;
  void onHalted() override;

private:
  /**
   * @brief   函数功能说明,计算目标的中心点位置，然后通过tick函数发给规划器
   * @param   参数说明
   * @return 返回值描述
   */
  BT::Position3D ObjConterRGB(const common_msgs::Obj &obj);

private:
  std::shared_ptr<MavRosConnect> mct;
  std::shared_ptr<ros::NodeHandle> nh;
  const geometry_msgs::PoseStamped *fcu_pose_ptr;
  const mavros_msgs::State *fcu_state_ptr;
  const ros::Publisher *tgt_pose_pub_ptr;
  ros::Publisher planer_goal_pub;
  common_msgs::Objects objs;
  common_msgs::Obj obj;

  std::mutex mtx;
  double fx; // 相机焦距
  double fy; // 相机焦距
  int rgb_ppx;
  int rgb_ppy;
  double rgb_fov_h;
  double rgb_fov_v;
  int rgb_image_w;
  int rgb_image_h;
  mavros_msgs::PositionTarget cmd;

  bool has_recv_obj;
  std::vector<double> cam2body_R;
  std::vector<double> cam2body_T;

  double min_score;
  std::string obj_name;

  double kx, ky, kz, v_max, hight_min, hight_max;

  ros::Subscriber detect_sub;
};

#endif // CROSSFRAME_H
