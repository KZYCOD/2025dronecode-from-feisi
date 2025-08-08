#include "mavros_cnt.h"



std::shared_ptr<MavRosConnect>
MavRosConnect::getInstance(std::shared_ptr<ros::NodeHandle> nh)
{
  // 使用 std::call_once 确保线程安全
  std::call_once(initFlag, [&]() {
    mavros = std::shared_ptr<MavRosConnect>(new MavRosConnect(nh));
    mavros->InitMavRosConnect();
  });
  count_++;
  return mavros;  
}

void MavRosConnect::InitMavRosConnect()
{
  std::cout << "in initail ..." << std::endl;
  if(nh_ == nullptr)
  {
    ROS_ERROR("MavRosConnect node handle is empty");
    return;
  }
  nh_->param<std::string>("fcu_state_topic", param.fcu_state_topic,
                          "mavros/state");
  nh_->param<std::string>("fcu_pos_topic", param.fcu_pose_topic,
                          "mavros/local_position/pose");
  nh_->param<std::string>("pos_tgt_topic", param.pos_tgt_topic,
                          "mavros/setpoint_raw/local");
  nh_->param<std::string>("odom_topic", param.fcu_odom_topic,
                          "mavros/local_position/odom");
  nh_->param<std::string>("set_mode_service", param.set_mode_service,
                          "mavros/set_mode");
  nh_->param<std::string>("arm_service", param.arm_service,
                          "mavros/cmd/arming");

  fcu_state_sub = nh_->subscribe<mavros_msgs::State>(
    param.fcu_state_topic, 10,
    bind(&MavRosConnect::FcuStateCallback, mavros, _1));

  fcu_pos_sub = nh_->subscribe<geometry_msgs::PoseStamped>(
    param.fcu_pose_topic, 10,
    bind(&MavRosConnect::FcuPoseCallback, mavros, _1));

  fcu_odom_sub = nh_->subscribe<nav_msgs::Odometry>(
    param.fcu_odom_topic, 10,
    bind(&MavRosConnect::FcuOdomCallback, mavros, _1));

  tgt_pose_pub =
    nh_->advertise<mavros_msgs::PositionTarget>(param.pos_tgt_topic, 10);

  att_pub =
    nh_->advertise<mavros_msgs::AttitudeTarget>(param.att_tgt_topic, 10);

  set_mode_client =
    nh_->serviceClient<mavros_msgs::SetMode>(param.set_mode_service);
  arming_client =
    nh_->serviceClient<mavros_msgs::CommandBool>(param.arm_service);
}

const MavRosConnect::Param *MavRosConnect::getParam()
{
  return &param;
}

const geometry_msgs::PoseStamped *MavRosConnect::getFcuPose()
{
  return &fcu_pose;
}


const mavros_msgs::State *MavRosConnect::getFcuState()
{
  return &fcu_state;
}

ros::ServiceClient *MavRosConnect::getSetModeClient()
{
  return &set_mode_client;
}

ros::ServiceClient *MavRosConnect::getArmClient()
{
  return &arming_client;
}

ros::Publisher *MavRosConnect::getPosTgtPublier()
{
  return &tgt_pose_pub;
}

ros::Publisher *MavRosConnect::getAttTgtPublier()
{
  return &att_pub;
}

std::shared_ptr<ros::NodeHandle> MavRosConnect::getROSHandle()
{
  return nh_;
}

std::mutex &MavRosConnect::GetPoseMtx()
 {
    return pose_mtx;
 }
void MavRosConnect::FcuPoseCallback(
  const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  fcu_pose = *msg;
  ;
}

void MavRosConnect::FcuStateCallback(const mavros_msgs::State::ConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(pose_mtx);
  fcu_state = *msg;
}


void MavRosConnect::FcuOdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  fcu_odom = *msg;
}

MavRosConnect::MavRosConnect(std::shared_ptr<ros::NodeHandle> nh)
{
  nh_ = nh;
}
