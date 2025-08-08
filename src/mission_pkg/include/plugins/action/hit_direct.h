/******************************************************************************
 * @file       hit.h
 * @brief
 * 撞击绑在车上的气球，提供两种方式。1.视觉伺服控制；2.视觉伺服控制的同时避障；
 * 第一种是理想情况，如果想要飞机不撞击障碍物，需要对小车的路径规划障碍物的膨胀距离设置的大一点，
 * 这样可以保证飞机控制撞击气球的同时不撞击障碍物；
 * 第二种，就是飞机通过激光雷达获取障碍物深度进行避障，当然也需要小车保持较大的障碍物膨胀距离；
 * 因此本类，优先使用第一种方案，也就是飞机的避障行为交给小车，第二作为备用方案，读者可以自行实现
 *
 * 为了提高打击率，需要做目标分割，读者可以使用一些高级开源的目标分割算法，直接提出目标的像素
 * 但是为了降低计算消耗，使用最简单方法分割，考虑到比赛用到的是红色气球，
 * 我们可以在目标框内提取红色通道像素，然后提取这些像素的中心点，即为气球中心位置，
 * 当然如果不使用深度学习提出目标框，直接使用颜色通道的话，误识别可能性增大，降低打击效率，
 * 需要注意的，程序不能自己判断气球爆炸的效果，气球有没有刺破仅仅只是靠目标从当前视野里消失，
 * 但是消失也有可能是目标检测漏检情况，只能通过目标消失一段时间来判断，因此当气球刺破后需要人接管飞机；
 *
 * 根据比赛要求，只有小车从P1->P2->P1点后 才能攻击,因此需要知道可攻击的条件，当然最好的方式是通过下视相机去判断，但是免费版本平台仅支持3个传感器，因此还是使用前视去做判断。
 * 另外考虑到如果小车抛锚了，为了尽可能多的得分，飞机不必一直等待小车绕过第二根柱子，直接攻击气球，
 * 所以需要设置一个等待时间，如果小车不动了或者未到第二根柱子就往回走了。
 * @author     戴开世<daivesi@sina.com>
 * @date       2025/03/07
 * @history
 *****************************************************************************/
#ifndef HIT_DIRECT_H
#define HIT_DIRECT_H
#include "plugins/common.hpp"
#include <mavros_cnt.h>
#include <common_msgs/Objects.h>
#include <common_msgs/Obj.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <mutex>
#include <pthread.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <livox_ros_driver2/CustomPoint.h>
#include <omp.h>
#include <queue>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Time.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Geometry>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h> 


using namespace BT;
typedef message_filters::sync_policies::ApproximateTime<common_msgs::Objects, sensor_msgs::Image> SyncPolicy;
class HitDirect : public StatefulActionNode
{
  enum Method
  {
    direct = 1, // 直接使用视觉伺服的方式去控制
    other = 2
  };

public:


  HitDirect(const std::string &name, const NodeConfig &config);

  static PortsList providedPorts();

  NodeStatus onStart() override;
  NodeStatus onRunning() override;
  void onHalted() override;

  inline bool  getObjPosition(geometry_msgs::PoseStamped &goal);
  void toEulerAngle(const Eigen::Quaterniond &q, Eigen::Vector3d &rotat);
  void RecvObj(const common_msgs::Objects::ConstPtr &msg);

  // 控制俯冲式打击目标，为伺服方式,筛选出一个全飞行的区域
  void RecvPointCloud(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void DepthCallback(const sensor_msgs::Image::ConstPtr &msg);
  void DepthObjCallback(const common_msgs::Objects::ConstPtr &objs,const sensor_msgs::Image::ConstPtr &depth);
  void timerCallback(const ros::TimerEvent& event);
  inline bool  TimeSync(const std_msgs::Time &t1, const std_msgs::Time &t2)
  { //硬同步，时间戳相等
    if(t1.data.sec == t2.data.sec && t1.data.nsec == t2.data.nsec)
      return true;
    return false;
  }
private:
  bool CalFlyOri(double &vx, double &vy, double &vz, double &yaw_rate);
  // 判断是否可以进入攻击模式
  void JudgeHitEnable();
  double PredictObjOri();
  void VisualTest(const geometry_msgs::PoseStamped &point);

private:
  std::shared_ptr<MavRosConnect> mct;
  std::shared_ptr<ros::NodeHandle> nh; // 需要订阅来自目标检测输出
  const geometry_msgs::PoseStamped *fcu_pose_ptr;
  const mavros_msgs::State *fcu_state_ptr;
  const ros::Publisher *tgt_pose_pub_ptr;
  std::string det_topic;
  std::mutex *pose_mtx;
  std::mutex mtx;
  common_msgs::Objects objs;
  common_msgs::Obj obj;
  ros::Time last_obj_time;
  bool isDetect;
  std::mutex pc_mtx;
  ros::Publisher obj_vis_pub;

  double hight_max, hight_min; // 约束飞机飞行高度

  int rgb_image_w;
  int rgb_image_h;
  int rgb_cnt_x;
  int rgb_cnt_y;
  double kx, ky, kz, k_yaw, v_max, v_min, obs_dist, yaw_err;
  double fx, fy, cx, cy;
  std::string obj_name;
  // std::vector<livox_ros_driver2::CustomPoint> obs_ps;
  ros::Subscriber odom_sub;
  ros::Subscriber obs_sub;
  ros::Publisher goal_pub;
  ros::Publisher test_pose_pub;
  ros::Subscriber depth_sub;
  geometry_msgs::PoseStamped goal;
  sensor_msgs::PointCloud2 obs_ps;
  nav_msgs::Odometry odom;

  //当刺目标时，目标的行径方向与飞机的朝向一致时，或者目标静止时再做hit动作
  //需要做时间戳对齐，因为realsense d435i相机可以做到硬件同步

  std::mutex detph_mtx;
  int depth_len;
  int objs_len;
  int pose_len;
  std::queue<common_msgs::Obj> objs_queue; 
  std::deque<sensor_msgs::Image::ConstPtr> depth_buffer_;
  std::deque<geometry_msgs::PoseStamped> pose_buffer_;
  std::deque<geometry_msgs::PoseStamped> obj_tra;
  std::mutex obj_tra_mtx;
  ros::Timer  pose_timer;

  int len_sequence_objs; // 用来判断目标的行进的方向
  bool isHit_enable;
  Method hit_method;
  ros::Subscriber det_sub;
  mavros_msgs::PositionTarget cmd;
  double hit_dist,goal_tolerance,needle_adapt;
  int left_line, right_line;
};

#endif // HIT_H
