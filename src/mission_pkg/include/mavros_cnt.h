/******************************************************************************
 * @file       mavros_cnt.h
 * @brief      因为多个行为树节点类里面都需要用到mavros消息，故此创建该类
 *
 * @author     戴开世<daivesi@sina.com>
 * @date       2025/03/06
 * @history
 *****************************************************************************/
#ifndef MAVROS_CNT_H
#define MAVROS_CNT_H
#include <ros/ros.h>
#include <plugins/common.hpp>
#include <mutex>
#include <memory>

class MavRosConnect
{

  typedef struct param_
  {
    //下面的变量都是只读
    std::string fcu_state_topic;
    std::string fcu_pose_topic;
    std::string pos_tgt_topic;
    std::string set_mode_service;
    std::string arm_service;

    std::string fcu_odom_topic;
    std::string att_tgt_topic;

    param_() {}
  } Param;

public:
  static std::shared_ptr<MavRosConnect>
               getInstance(std::shared_ptr<ros::NodeHandle> nh = nullptr);
  void         InitMavRosConnect();
  const Param *getParam();

  const geometry_msgs::PoseStamped *getFcuPose();
  const mavros_msgs::State *        getFcuState();
  std::mutex &GetPoseMtx();
  ros::ServiceClient *              getSetModeClient();
  ros::ServiceClient *              getArmClient();
  ros::Publisher *                  getPosTgtPublier();
  ros::Publisher *                  getAttTgtPublier();
  std::shared_ptr<ros::NodeHandle>  getROSHandle();

  //订阅飞机位置的回调函数
  void FcuPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

  //订阅飞控状态的回调函数
  void FcuStateCallback(const mavros_msgs::State::ConstPtr &msg);

  //订阅飞控里程计数据,里面包含姿态，速度，加速度信息。可以方便提供给行为树应用
  void FcuOdomCallback(const nav_msgs::Odometry::ConstPtr &msg);

  static int count_;
  static std::mutex pose_mtx;
private:
  MavRosConnect(std::shared_ptr<ros::NodeHandle> nh);
  MavRosConnect(const MavRosConnect &) = delete;
  MavRosConnect &operator=(const MavRosConnect &) = delete;

  static std::once_flag initFlag;

  static std::shared_ptr<MavRosConnect> mavros;
  //  static std::mutex                     mtx;
  
  Param                      param;
  mavros_msgs::State         fcu_state;
  geometry_msgs::PoseStamped fcu_pose;

  nav_msgs::Odometry fcu_odom;

  std::shared_ptr<ros::NodeHandle> nh_;
  ros::Subscriber                  fcu_pos_sub;
  ros::Subscriber                  fcu_odom_sub;
  ros::Subscriber                  fcu_state_sub;
  ros::Publisher                   tgt_pose_pub;
  ros::Publisher                   att_pub;
  ros::ServiceClient               set_mode_client;
  ros::ServiceClient               arming_client;
};

std::shared_ptr<MavRosConnect> MavRosConnect::mavros = nullptr;
std::once_flag                 MavRosConnect::initFlag;
int                            MavRosConnect::count_ = 0;
std::mutex MavRosConnect::pose_mtx;
#endif  // MAVROS_H
