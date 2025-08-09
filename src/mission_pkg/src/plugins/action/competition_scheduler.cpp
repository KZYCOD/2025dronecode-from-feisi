#include "plugins/action/competition_scheduler.h"
#include <string>

CompetitionScheduler::CompetitionScheduler(const std::string &name, const NodeConfig &config)
    : StatefulActionNode(name, config), current_state(INITIALIZING), 
      current_obstacle(0), emergency_stop(false), mission_abort(false)
{
  mct = MavRosConnect::getInstance();
  fcu_pose_ptr = mct->getFcuPose();
  fcu_state_ptr = mct->getFcuState();

  auto nh = mct->getROSHandle();
  
  // 初始化发布者和订阅者
  state_pub = nh->advertise<std_msgs::String>("/competition/state", 10);
  mission_status_pub = nh->advertise<std_msgs::String>("/competition/status", 10);
  obstacle_complete_sub = nh->subscribe("/competition/obstacle_complete", 10, 
                                       &CompetitionScheduler::obstacleCompleteCallback, this);

  // 获取场地参数
  nh->param("field_width", field_width, 10.0);
  nh->param("field_length", field_length, 10.0);
  nh->param("field_height_limit", field_height_limit, 4.0);

  initializeObstacles();
}

PortsList CompetitionScheduler::providedPorts()
{
  return {
    InputPort<bool>("enable_emergency_stop", "启用紧急停止", false),
    OutputPort<std::string>("current_mission_state", "当前任务状态"),
    OutputPort<int>("current_obstacle_id", "当前障碍编号"),
    OutputPort<bool>("mission_complete", "任务完成标志")
  };
}

NodeStatus CompetitionScheduler::onStart()
{
  ROS_INFO("CompetitionScheduler: Starting competition mission");
  
  current_state = INITIALIZING;
  mission_start_time = ros::Time::now();
  state_start_time = mission_start_time;
  current_obstacle = 0;
  emergency_stop = false;
  mission_abort = false;

  // 重置所有障碍完成状态
  for (auto& obstacle : obstacles) {
    obstacle.completed = false;
  }

  publishMissionState();
  return NodeStatus::RUNNING;
}

NodeStatus CompetitionScheduler::onRunning()
{
  ros::Time current_time = ros::Time::now();
  double state_duration = (current_time - state_start_time).toSec();
  double mission_duration = (current_time - mission_start_time).toSec();

  // 安全检查
  if (!checkSafetyConstraints()) {
    ROS_ERROR("CompetitionScheduler: Safety constraints violated");
    handleEmergency();
    return NodeStatus::FAILURE;
  }

  // 检查紧急停止
  auto emergency_input = getInput<bool>("enable_emergency_stop");
  if (emergency_input && emergency_input.value()) {
    ROS_WARN("CompetitionScheduler: Emergency stop activated");
    emergency_stop = true;
    return NodeStatus::FAILURE;
  }

  // 状态超时检查
  double timeout = getStateTimeout(current_state);
  if (state_duration > timeout) {
    ROS_WARN("CompetitionScheduler: State %s timeout (%.2fs)", 
             stateToString(current_state).c_str(), state_duration);
    return NodeStatus::FAILURE;
  }

  // 状态机逻辑
  switch (current_state) {
    case INITIALIZING: {
      // 初始化检查：无人机状态、系统就绪等
      if (fcu_state_ptr && fcu_state_ptr->connected) {
        ROS_INFO("CompetitionScheduler: Initialization complete, ready for takeoff");
        current_state = TAKING_OFF;
        state_start_time = current_time;
      }
      break;
    }

    case TAKING_OFF: {
      // 等待起飞完成
      if (fcu_pose_ptr && fcu_pose_ptr->pose.position.z > 1.5) {
        ROS_INFO("CompetitionScheduler: Takeoff complete, starting obstacle course");
        current_state = OBSTACLE_1_SQUARE;
        current_obstacle = 0;
        state_start_time = current_time;
      }
      break;
    }

    case OBSTACLE_1_SQUARE: {
      if (isObstacleComplete(0)) {
        ROS_INFO("CompetitionScheduler: Obstacle 1 (Square Frame) completed");
        current_state = OBSTACLE_2_CIRCLE;
        current_obstacle = 1;
        state_start_time = current_time;
      }
      break;
    }

    case OBSTACLE_2_CIRCLE: {
      if (isObstacleComplete(1)) {
        ROS_INFO("CompetitionScheduler: Obstacle 2 (Circle Frame) completed");
        current_state = OBSTACLE_3_DOUBLE;
        current_obstacle = 2;
        state_start_time = current_time;
      }
      break;
    }

    case OBSTACLE_3_DOUBLE: {
      if (isObstacleComplete(2)) {
        ROS_INFO("CompetitionScheduler: Obstacle 3 (Double Frame) completed");
        current_state = OBSTACLE_4_SOMERSAULT;
        current_obstacle = 3;
        state_start_time = current_time;
      }
      break;
    }

    case OBSTACLE_4_SOMERSAULT: {
      if (isObstacleComplete(3)) {
        ROS_INFO("CompetitionScheduler: Obstacle 4 (Somersault) completed");
        current_state = OBSTACLE_5_FLAG;
        current_obstacle = 4;
        state_start_time = current_time;
      }
      break;
    }

    case OBSTACLE_5_FLAG: {
      if (isObstacleComplete(4)) {
        ROS_INFO("CompetitionScheduler: Obstacle 5 (Flag Circle) completed");
        current_state = LANDING;
        state_start_time = current_time;
      }
      break;
    }

    case LANDING: {
      // 检查是否成功降落
      if (fcu_pose_ptr && fcu_pose_ptr->pose.position.z < 0.3) {
        ROS_INFO("CompetitionScheduler: Landing complete - MISSION SUCCESS!");
        current_state = MISSION_COMPLETE;
        
        // 设置输出端口
        setOutput("mission_complete", true);
        setOutput("current_mission_state", stateToString(current_state));
        
        return NodeStatus::SUCCESS;
      }
      break;
    }

    case MISSION_COMPLETE:
      return NodeStatus::SUCCESS;

    case MISSION_FAILED:
      return NodeStatus::FAILURE;
  }

  // 更新输出端口
  setOutput("current_mission_state", stateToString(current_state));
  setOutput("current_obstacle_id", current_obstacle);
  setOutput("mission_complete", false);

  publishMissionState();
  return NodeStatus::RUNNING;
}

void CompetitionScheduler::onHalted()
{
  ROS_INFO("CompetitionScheduler: Mission halted");
  current_state = INITIALIZING;
  emergency_stop = false;
  mission_abort = false;
}

void CompetitionScheduler::initializeObstacles()
{
  obstacles.clear();
  
  // 初始化5个障碍的参数
  obstacles.push_back({BT::Position3D(2.0, 0.0, 2.0), 30.0, false}); // obs1
  obstacles.push_back({BT::Position3D(4.0, 0.0, 2.0), 30.0, false}); // obs2  
  obstacles.push_back({BT::Position3D(6.0, 0.0, 2.0), 45.0, false}); // obs3
  obstacles.push_back({BT::Position3D(8.0, 0.0, 2.0), 60.0, false}); // obs4
  obstacles.push_back({BT::Position3D(9.0, 0.0, 2.0), 45.0, false}); // obs5
}

void CompetitionScheduler::publishMissionState()
{
  std_msgs::String state_msg;
  state_msg.data = stateToString(current_state);
  state_pub.publish(state_msg);

  std_msgs::String status_msg;
  status_msg.data = "State: " + stateToString(current_state) + 
                    ", Obstacle: " + std::to_string(current_obstacle + 1) + "/5";
  mission_status_pub.publish(status_msg);
}

void CompetitionScheduler::obstacleCompleteCallback(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data && current_obstacle < obstacles.size()) {
    obstacles[current_obstacle].completed = true;
    ROS_INFO("CompetitionScheduler: Obstacle %d marked as complete", current_obstacle + 1);
  }
}

bool CompetitionScheduler::checkSafetyConstraints()
{
  if (!fcu_pose_ptr) return false;

  // 检查高度限制
  if (fcu_pose_ptr->pose.position.z > field_height_limit) {
    ROS_WARN("CompetitionScheduler: Height limit violated: %.2f > %.2f", 
             fcu_pose_ptr->pose.position.z, field_height_limit);
    return false;
  }

  // 检查场地边界
  if (fcu_pose_ptr->pose.position.x < 0 || fcu_pose_ptr->pose.position.x > field_length ||
      fcu_pose_ptr->pose.position.y < -field_width/2 || fcu_pose_ptr->pose.position.y > field_width/2) {
    ROS_WARN("CompetitionScheduler: Field boundary violated");
    return false;
  }

  return true;
}

bool CompetitionScheduler::isObstacleComplete(int obstacle_id)
{
  if (obstacle_id < 0 || obstacle_id >= obstacles.size()) return false;
  return obstacles[obstacle_id].completed;
}

void CompetitionScheduler::handleEmergency()
{
  ROS_ERROR("CompetitionScheduler: Emergency situation - stopping mission");
  current_state = MISSION_FAILED;
  emergency_stop = true;
  
  // 这里可以添加紧急停止逻辑，如悬停、紧急降落等
}

std::string CompetitionScheduler::stateToString(MissionState state)
{
  switch (state) {
    case INITIALIZING: return "INITIALIZING";
    case TAKING_OFF: return "TAKING_OFF";
    case OBSTACLE_1_SQUARE: return "OBSTACLE_1_SQUARE";
    case OBSTACLE_2_CIRCLE: return "OBSTACLE_2_CIRCLE";
    case OBSTACLE_3_DOUBLE: return "OBSTACLE_3_DOUBLE";
    case OBSTACLE_4_SOMERSAULT: return "OBSTACLE_4_SOMERSAULT";
    case OBSTACLE_5_FLAG: return "OBSTACLE_5_FLAG";
    case LANDING: return "LANDING";
    case MISSION_COMPLETE: return "MISSION_COMPLETE";
    case MISSION_FAILED: return "MISSION_FAILED";
    default: return "UNKNOWN";
  }
}

double CompetitionScheduler::getStateTimeout(MissionState state)
{
  switch (state) {
    case INITIALIZING: return 10.0;
    case TAKING_OFF: return 20.0;
    case OBSTACLE_1_SQUARE: return 30.0;
    case OBSTACLE_2_CIRCLE: return 30.0;
    case OBSTACLE_3_DOUBLE: return 45.0;
    case OBSTACLE_4_SOMERSAULT: return 60.0;
    case OBSTACLE_5_FLAG: return 45.0;
    case LANDING: return 30.0;
    default: return 30.0;
  }
}