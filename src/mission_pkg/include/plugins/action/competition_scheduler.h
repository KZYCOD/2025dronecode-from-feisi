/******************************************************************************
 * @file       competition_scheduler.h
 * @brief      竞赛任务调度器 - 高层任务调度和状态管理
 * @author     AI Assistant  
 * @date       2025/01/01
 * @history    竞赛任务的高层调度控制
 *****************************************************************************/

#ifndef COMPETITION_SCHEDULER_H
#define COMPETITION_SCHEDULER_H

#include "plugins/common.hpp"
#include <ros/ros.h>
#include <mavros_cnt.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>

using namespace BT;

class CompetitionScheduler : public StatefulActionNode
{
public:
  CompetitionScheduler(const std::string &name, const NodeConfig &config);

  static PortsList providedPorts();

  NodeStatus onStart() override;
  NodeStatus onRunning() override;
  void onHalted() override;

private:
  // 任务状态枚举
  enum MissionState {
    INITIALIZING,
    TAKING_OFF,
    OBSTACLE_1_SQUARE,
    OBSTACLE_2_CIRCLE, 
    OBSTACLE_3_DOUBLE,
    OBSTACLE_4_SOMERSAULT,
    OBSTACLE_5_FLAG,
    LANDING,
    MISSION_COMPLETE,
    MISSION_FAILED
  };

  std::shared_ptr<MavRosConnect> mct;
  const geometry_msgs::PoseStamped *fcu_pose_ptr;
  const mavros_msgs::State *fcu_state_ptr;

  // 状态管理
  MissionState current_state;
  ros::Time state_start_time;
  ros::Time mission_start_time;
  
  // 任务参数
  struct ObstacleParams {
    BT::Position3D position;
    double timeout;
    bool completed;
  };
  
  std::vector<ObstacleParams> obstacles;
  int current_obstacle;
  
  // 发布者和订阅者
  ros::Publisher state_pub;
  ros::Publisher mission_status_pub;
  ros::Subscriber obstacle_complete_sub;
  
  // 安全和监控
  double field_width, field_length, field_height_limit;
  bool emergency_stop;
  bool mission_abort;
  
  // 方法
  void initializeObstacles();
  void publishMissionState();
  void obstacleCompleteCallback(const std_msgs::Bool::ConstPtr& msg);
  bool checkSafetyConstraints();
  bool isObstacleComplete(int obstacle_id);
  void handleEmergency();
  std::string stateToString(MissionState state);
  double getStateTimeout(MissionState state);
};

#endif // COMPETITION_SCHEDULER_H