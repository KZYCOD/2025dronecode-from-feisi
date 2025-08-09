/******************************************************************************
 * @file       calobjpos.h
 * @brief
 *计算目标的位置，这里或需要结合深度图计算。计算的方法就是目标检测体统目标中心位置和区域，简单的方法就是以目标中心位置的的像素坐标的深度值计算
 *从深度值里面计算获得目标点位姿
 * @author     戴开世<daivesi@sina.com>
 * @date       2025/03/25
 * @history
 *****************************************************************************/

#ifndef CALOBJPOS_H
#define CALOBJPOS_H

#include <common_msgs/Objects.h>
#include <common_msgs/Obj.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
//#include <message_filters/subscriber.h>
//#include <message_filters/synchronizer.h>
//#include <message_filters/sync_policies/approximate_time.h>
//#include <message_filters/sync_policies/exact_time.h>
#include <geometry_msgs/PoseArray.h>

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <queue>

#include <plugins/common.hpp>
#include "mavros_cnt.h"

using namespace BT;

// typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
//                                                        common_msgs::Objects>
//  SyncPolicy;
class CalObjPos : public StatefulActionNode
{
public:
  CalObjPos(const std::string name, const NodeConfig &config);
  ~CalObjPos();
  void ObjsCallback(const common_msgs::Objects::ConstPtr &objs);
  void DepthImgCB(const sensor_msgs::Image::ConstPtr &img);

  void DepthInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &info);
  static PortsList providedPorts();

  NodeStatus onStart() override;
  NodeStatus onRunning() override;
  void       onHalted() override;

private:
  common_msgs::Objects              objs;
  std::string                       class_name;
  const geometry_msgs::PoseStamped *fcu_pose_ptr;
  std::shared_ptr<MavRosConnect>    mct;
  std::shared_ptr<ros::NodeHandle>  nh;
  ros::Publisher                    obj_pos_pub;
  sensor_msgs::CameraInfo           depth_info;  // 相机内参，
  std::vector<double> cam2body_R;  // 相机外参数，从相机系到body系
  std::vector<double> cam2body_T;  // 相机外参数，从相机系到body系
  ros::Subscriber     depth_img_sub;
  ros::Subscriber     objs_sub;

  //  message_filters::Subscriber<sensor_msgs::Image>   img_sub_;
  //  message_filters::Subscriber<common_msgs::Objects> objs_sub_;
  //  typedef
  //  message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
  //                                                          common_msgs::Objects>
  //                                            SyncPolicy;
  //  message_filters::Synchronizer<SyncPolicy> sync_;
  std::queue<sensor_msgs::Image> depth_queue;
  std::mutex                     depth_mtx;
};

#endif  // CALOBJPOS_H
