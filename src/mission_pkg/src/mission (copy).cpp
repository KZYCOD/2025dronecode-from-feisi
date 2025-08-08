#include "ros/ros.h"
#include "std_msgs/String.h"
// #include "controller.h"
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

#include <behaviortree_cpp/bt_factory.h>
// * nodes
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/control_node.h>
#include <behaviortree_cpp/decorator_node.h>

// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "contorller");
//   ros::NodeHandle nh;
//   auto            ctrl = new Controller(nh);
//   ros::spin();

//   delete ctrl;
//   return 0;
// }

using namespace BT;

mavros_msgs::State          current_state;
geometry_msgs::PoseStamped  fcu_pose;
mavros_msgs::PositionTarget cmd;
ros::Time                   last_timestamp;
mavros_msgs::SetMode        offb_set_mode;
mavros_msgs::AttitudeTarget att_thrust;
mavros_msgs::CommandBool    arm_cmd;
// mavros_msgs::State current_state;
mavros_msgs::SetMode land;
// ros::Rate rate();
std::string bt_tree_path =
  "/mnt/e/linzejun01/BehaviorTree.CPP-master/8.RflySimVision-master/"
  "BT_BaseAction/Ubuntu/src/Challege_ROS/controller/config/mav_baseaction.xml";

ros::Subscriber    state_sub;
ros::Publisher     cmd_pub;  //
ros::Subscriber    pose_usb;
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;
ros::Publisher     att_pub;

ros::Subscriber velocity_sub;

double vx;
double vy;
double vz;

// 角速度 (angular velocity)
double wx;
double wy;
double wz;

struct Position3D
{
  double x;
  double y;
  double z;
};

struct Distance
{
  double x;
};

// Template specialization to converts a string to Position3D.
namespace BT
{
template <> inline Position3D convertFromString(StringView str)
{
  // We expect real numbers separated by semicolons
  auto parts = splitString(str, ';');
  if(parts.size() != 3)
  {
    throw RuntimeError("invalid input)");
  }
  else
  {
    Position3D output;
    output.x = convertFromString<double>(parts[0]);
    output.y = convertFromString<double>(parts[1]);
    output.z = convertFromString<double>(parts[2]);
    return output;
  }
}

// template <> inline Distance convertFromString(StringView str)
// {
//   // We expect real numbers separated by semicolons
//   auto parts = splitString(str, ';');
//   if (parts.size() != 1)
//   {
//       throw RuntimeError("invalid input)");
//   }
//   else
//   {
//       Distance output;
//       output.x     = convertFromString<double>(parts[0]);
//       return output;
//   }
// }

template <> inline Distance convertFromString(StringView str)
{
  // We expect real numbers separated by semicolons
  if(str.find(';') != StringView::npos)
  {
    throw RuntimeError(
      "Invalid input: expected a single number without semicolons");
  }
  else
  {
    Distance output;
    output.x = convertFromString<double>(str);
    return output;
  }
}

}  // end namespace BT

// void RecvLIO(const nav_msgs::Odometry::ConstPtr &odom)
// {
//   send_local_pose.pose.position.x = odom->pose.pose.position.y;
//   send_local_pose.pose.position.y = -odom->pose.pose.position.x;
//   send_local_pose.pose.position.z = odom->pose.pose.position.z;

//   tf2::Quaternion q;
//   tf2::fromMsg(odom->pose.pose.orientation, q);
//   tf2::Matrix3x3 att(q);
//   double         roll, pitch, yaw;
//   att.getRPY(roll, pitch, yaw);
//   yaw += 1.5707;
//   tf2::Quaternion q_;
//   q_.setRPY(roll, pitch, yaw);

//   // send_local_pose.pose.orientation = odom->pose.pose.orientation;
//   send_local_pose.pose.orientation = tf2::toMsg(q_);

//   send_local_pose.header.frame_id = "map";
//   send_local_pose.header.stamp    = ros::Time::now();

//   pose_pub.publish(send_local_pose);
//   ros::spinOnce();
// }

void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
  // 线速度 (linear velocity)
  vx = msg->twist.linear.x;
  vy = msg->twist.linear.y;
  vz = msg->twist.linear.z;

  // 角速度 (angular velocity)
  wx = msg->twist.angular.x;
  wy = msg->twist.angular.y;
  wz = msg->twist.angular.z;
}

void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
  current_state = *msg;
}

void PoseCB(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  fcu_pose = *msg;
}

class Takeoff : public SyncActionNode
{
public:
  Takeoff(const std::string &name, const NodeConfig &config)
    : SyncActionNode(name, config)
  {
  }

  static PortsList providedPorts()
  {
    // Optionally, a port can have a human readable description
    const char *description = "Takeoff position...";
    return { InputPort<Position3D>("takeoff_target", description) };
  }

  NodeStatus tick() override
  {
    auto res = getInput<Position3D>("takeoff_target");
    if(!res)
    {
      throw RuntimeError("error reading port [takeoff_target]:", res.error());
    }
    ros::Rate  rate(20.0);
    Position3D target = res.value();
    printf("takeoff_Target positions: [ %.1f, %.1f ,%.1f]\n", target.x,
           target.y, target.z);
    cmd.coordinate_frame = mavros_msgs::PositionTarget::
      FRAME_LOCAL_NED;  // 选择控制坐标系，位置，速度，加速度使用local坐标系，姿态使用的是body坐标系
    cmd.type_mask = ~uint16_t(0);
    cmd.type_mask &=
      ~mavros_msgs::PositionTarget::FORCE;  // px4 不响应力的控制方式
    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PX;
    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PY;
    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PZ;
    cmd.position.x   = target.x;
    cmd.position.y   = target.y;
    cmd.position.z   = target.z;
    cmd.header.stamp = ros::Time::now();

    // mavros_msgs::AttitudeTarget att_thrust;
    // att_thrust.type_mask = ~uint8_t(0);

    for(int i = 100; ros::ok() && i > 0; --i)
    {
      //发送目标指令，以便飞控切换offboard状态，这里可以发位置，速度，加速度，一般情况起飞，都是发送位置
      cmd_pub.publish(cmd);
      ros::spinOnce();
      rate.sleep();
    }

    // mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    // mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    bool      is_takeoff   = false;
    bool      is_attitude  = false;
    while(ros::ok())
    {
      //切换offboard模式
      if(current_state.mode != "OFFBOARD"
         && (ros::Time::now() - last_request > ros::Duration(5.0))
         && !is_takeoff)
      {
        if(set_mode_client.call(offb_set_mode)
           && offb_set_mode.response.mode_sent)
        {
          ROS_INFO("Offboard enabled");
        }
        last_request = ros::Time::now();
      }
      else if(!is_takeoff)
      {
        //解锁飞控
        if(!current_state.armed
           && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
          if(arming_client.call(arm_cmd) && arm_cmd.response.success)
          {
            ROS_INFO("Vehicle armed");
          }
          last_request = ros::Time::now();
        }
      }
      if(abs(fcu_pose.pose.position.z - target.z) < 0.1 && !is_takeoff)
      {
        ROS_INFO("takeoff finished");
        last_request = ros::Time::now();
        is_takeoff   = true;
        return NodeStatus::SUCCESS;
      }
      if(!is_takeoff)
      {
        //起飞就是位置控制
        cmd_pub.publish(cmd);
        ros::spinOnce();
        rate.sleep();
        continue;
      }
      ros::spinOnce();
      rate.sleep();
    }

    return NodeStatus::FAILURE;
  }
};

// class Takeoff : public BT::StatefulAsyncAction
// {
//   public:
//     // Any TreeNode with ports must have a constructor with this signature
//     Takeoff(const std::string& name, const BT::NodeConfig& config)
//       : StatefulAsyncAction(name, config)
//     {}

//     // It is mandatory to define this static method.
//     static BT::PortsList providedPorts()
//     {
//       // Optionally, a port can have a human readable description
//       const char*  description = "Takeoff position...";
//       return { InputPort<Position3D>("takeoff_target", description) };
//     }

//     // this function is invoked once at the beginning.
//     BT::NodeStatus onStart() override
//     {
//       auto res = getInput<Position3D>("takeoff_target");
//       if (!res)
//       {
//           throw RuntimeError("error reading port [takeoff_target]:",
//           res.error());
//       }
//       ros::Rate rate(20.0);
//       // Position3D target = res.value();
//       target = res.value();
//       printf("takeoff_Target positions: [ %.1f, %.1f ,%.1f]\n", target.x,
//       target.y, target.z ); cmd.coordinate_frame =
//       mavros_msgs::PositionTarget::FRAME_LOCAL_NED; //
//       选择控制坐标系，位置，速度，加速度使用local坐标系，姿态使用的是body坐标系
//       cmd.type_mask = ~uint16_t(0);
//       cmd.type_mask &= ~mavros_msgs::PositionTarget::FORCE; //px4
//       不响应力的控制方式 cmd.type_mask &=
//       ~mavros_msgs::PositionTarget::IGNORE_PX; cmd.type_mask &=
//       ~mavros_msgs::PositionTarget::IGNORE_PY; cmd.type_mask &=
//       ~mavros_msgs::PositionTarget::IGNORE_PZ; cmd.position.x = target.x;
//       cmd.position.y = target.y;
//       cmd.position.z = target.z;
//       cmd.header.stamp = ros::Time::now();

//       // mavros_msgs::AttitudeTarget att_thrust;
//       // att_thrust.type_mask = ~uint8_t(0);

//       for(int i = 100; ros::ok() && i > 0; --i){
//           //发送目标指令，以便飞控切换offboard状态，这里可以发位置，速度，加速度，一般情况起飞，都是发送位置
//           cmd_pub.publish(cmd);
//           ros::spinOnce();
//           rate.sleep();
//       }

//       // mavros_msgs::SetMode offb_set_mode;
//       offb_set_mode.request.custom_mode = "OFFBOARD";

//       // mavros_msgs::CommandBool arm_cmd;
//       arm_cmd.request.value = true;

//       ros::Time last_request = ros::Time::now();
//       bool is_takeoff= false;
//       bool is_attitude = false;

//       return BT::NodeStatus::RUNNING;
//     }
//     // If onStart() returned RUNNING, we will keep calling
//     // this method until it return something different from RUNNING
//     BT::NodeStatus onRunning() override
//     {
//       if (current_state.mode != "OFFBOARD" && (ros::Time::now() -
//       last_request > ros::Duration(5.0)) && !is_takeoff)
//       {
//           if (set_mode_client.call(offb_set_mode) &&
//           offb_set_mode.response.mode_sent)
//           {
//               ROS_INFO("Offboard enabled");
//           }
//           last_request = ros::Time::now();
//           return NodeStatus::RUNNING;
//       }

//       if (!current_state.armed && (ros::Time::now() - last_request >
//       ros::Duration(5.0)) && !is_takeoff)
//       {
//           if (arming_client.call(arm_cmd) && arm_cmd.response.success)
//           {
//               ROS_INFO("Vehicle armed");
//           }
//           last_request = ros::Time::now();
//           return NodeStatus::RUNNING;
//       }

//       if (abs(fcu_pose.pose.position.z - target.z) < 0.1 && !is_takeoff)
//       {
//           ROS_INFO("Takeoff finished");
//           is_takeoff = true;
//           return NodeStatus::SUCCESS;
//       }

//       // 继续发布起飞指令
//       // cmd.position.z = target.z;
//       cmd_pub.publish(cmd);
//       ros::spinOnce();
//       return NodeStatus::RUNNING;
//     }
//     // callback to execute if the action was aborted by another node
//     void onHalted() override
//     {
//       printf("[ MoveBase: ABORTED ]");
//     }

//   private:
//     Position3D target;
//     chr::system_clock::time_point _completion_time;
// }

class MoveForward : public SyncActionNode
{
public:
  MoveForward(const std::string &name, const NodeConfig &config)
    : SyncActionNode(name, config)
  {
  }

  static PortsList providedPorts()
  {
    // Optionally, a port can have a human readable description
    const char *description = "Simply print the goal on console...";
    return { InputPort<Distance>("MoveForward_distance", description) };
  }

  NodeStatus tick() override
  {
    auto res = getInput<Distance>("MoveForward_distance");
    if(!res)
    {
      throw RuntimeError("error reading port [MoveForward_distance]:",
                         res.error());
    }
    ros::Rate rate(20.0);
    Distance  target = res.value();
    printf("MoveForward positions: [ %.1f]\n", target.x);
    // float lzj_distance = fcu_pose.pose.position.x + target.x;
    float lzj_distance = fcu_pose.pose.position.y + target.x;
    // cmd.position.x   = lzj_distance;
    cmd.position.y         = lzj_distance;
    ros::Time last_request = ros::Time::now();
    bool      is_finished  = false;
    while(ros::ok())
    {
      if(abs(fcu_pose.pose.position.y - lzj_distance) < 0.1 && !is_finished)
      {
        ROS_INFO("MoveForward finished");
        last_request = ros::Time::now();
        is_finished  = true;
        return NodeStatus::SUCCESS;
      }
      if(!is_finished)
      {
        //位置控制
        cmd_pub.publish(cmd);
        ros::spinOnce();
        rate.sleep();
        continue;
      }
      ros::spinOnce();
      rate.sleep();
    }
    return NodeStatus::FAILURE;
  }
};

class MoveBack : public SyncActionNode
{
public:
  MoveBack(const std::string &name, const NodeConfig &config)
    : SyncActionNode(name, config)
  {
  }

  static PortsList providedPorts()
  {
    // Optionally, a port can have a human readable description
    const char *description = "Simply print the goal on console...";
    return { InputPort<Distance>("MoveBack_distance", description) };
  }

  NodeStatus tick() override
  {
    auto res = getInput<Distance>("MoveBack_distance");
    if(!res)
    {
      throw RuntimeError("error reading port [MoveBack_distance]:",
                         res.error());
    }
    ros::Rate rate(20.0);
    Distance  target = res.value();
    printf("MoveBack positions: [ %.1f]\n", target.x);
    // float lzj_distance = fcu_pose.pose.position.x - target.x;
    // cmd.position.x   = lzj_distance;
    float lzj_distance     = fcu_pose.pose.position.y - target.x;
    cmd.position.y         = lzj_distance;
    ros::Time last_request = ros::Time::now();
    bool      is_finished  = false;
    while(ros::ok())
    {
      if(abs(fcu_pose.pose.position.y - lzj_distance) < 0.1 && !is_finished)
      {
        ROS_INFO("MoveBack finished");
        last_request = ros::Time::now();
        is_finished  = true;
        return NodeStatus::SUCCESS;
      }
      if(!is_finished)
      {
        //位置控制
        cmd_pub.publish(cmd);
        ros::spinOnce();
        rate.sleep();
        continue;
      }
      ros::spinOnce();
      rate.sleep();
    }
    return NodeStatus::FAILURE;
  }
};

class MoveRight : public SyncActionNode
{
public:
  MoveRight(const std::string &name, const NodeConfig &config)
    : SyncActionNode(name, config)
  {
  }

  static PortsList providedPorts()
  {
    // Optionally, a port can have a human readable description
    const char *description = "Simply print the goal on console...";
    return { InputPort<Distance>("MoveRight_distance", description) };
  }

  NodeStatus tick() override
  {
    auto res = getInput<Distance>("MoveRight_distance");
    if(!res)
    {
      throw RuntimeError("error reading port [MoveRight_distance]:",
                         res.error());
    }
    ros::Rate rate(20.0);
    Distance  target = res.value();
    printf("MoveRight positions: [ %.1f]\n", target.x);
    // float lzj_distance = fcu_pose.pose.position.y + target.x;
    // cmd.position.y   = lzj_distance;
    float lzj_distance     = fcu_pose.pose.position.x + target.x;
    cmd.position.x         = lzj_distance;
    ros::Time last_request = ros::Time::now();
    bool      is_finished  = false;
    while(ros::ok())
    {
      if(abs(fcu_pose.pose.position.x - lzj_distance) < 0.1 && !is_finished)
      {
        ROS_INFO("MoveBack finished");
        last_request = ros::Time::now();
        is_finished  = true;
        return NodeStatus::SUCCESS;
      }
      if(!is_finished)
      {
        //位置控制
        cmd_pub.publish(cmd);
        ros::spinOnce();
        rate.sleep();
        continue;
      }
      ros::spinOnce();
      rate.sleep();
    }
    return NodeStatus::FAILURE;
  }
};

class MoveLeft : public SyncActionNode
{
public:
  MoveLeft(const std::string &name, const NodeConfig &config)
    : SyncActionNode(name, config)
  {
  }

  static PortsList providedPorts()
  {
    // Optionally, a port can have a human readable description
    const char *description = "Simply print the goal on console...";
    return { InputPort<Distance>("MoveLeft_distance", description) };
  }

  NodeStatus tick() override
  {
    auto res = getInput<Distance>("MoveLeft_distance");
    if(!res)
    {
      throw RuntimeError("error reading port [MoveLeft_distance]:",
                         res.error());
    }
    ros::Rate rate(20.0);
    Distance  target = res.value();
    printf("MoveLeft positions: [ %.1f]\n", target.x);
    // float lzj_distance = fcu_pose.pose.position.y - target.x;
    // cmd.position.y   = lzj_distance;
    float lzj_distance = fcu_pose.pose.position.x - target.x;
    cmd.position.x     = lzj_distance;

    ros::Time last_request = ros::Time::now();
    bool      is_finished  = false;
    while(ros::ok())
    {
      if(abs(fcu_pose.pose.position.x - lzj_distance) < 0.1 && !is_finished)
      {
        ROS_INFO("MoveLeft finished");
        last_request = ros::Time::now();
        is_finished  = true;
        return NodeStatus::SUCCESS;
      }
      if(!is_finished)
      {
        //位置控制
        cmd_pub.publish(cmd);
        ros::spinOnce();
        rate.sleep();
        continue;
      }
      ros::spinOnce();
      rate.sleep();
    }
    return NodeStatus::FAILURE;
  }
};

class MoveUp : public SyncActionNode
{
public:
  MoveUp(const std::string &name, const NodeConfig &config)
    : SyncActionNode(name, config)
  {
  }

  static PortsList providedPorts()
  {
    // Optionally, a port can have a human readable description
    const char *description = "Simply print the goal on console...";
    return { InputPort<Distance>("MoveUp_distance", description) };
  }

  NodeStatus tick() override
  {
    auto res = getInput<Distance>("MoveUp_distance");
    if(!res)
    {
      throw RuntimeError("error reading port [MoveUp_distance]:", res.error());
    }
    ros::Rate rate(20.0);
    Distance  target = res.value();
    printf("MoveUp positions: [ %.1f]\n", target.x);
    float lzj_distance     = fcu_pose.pose.position.z - target.x;
    cmd.position.z         = lzj_distance;
    ros::Time last_request = ros::Time::now();
    bool      is_finished  = false;
    while(ros::ok())
    {
      if(abs(fcu_pose.pose.position.z - lzj_distance) < 0.1 && !is_finished)
      {
        ROS_INFO("MoveUp finished");
        last_request = ros::Time::now();
        is_finished  = true;
        return NodeStatus::SUCCESS;
      }
      if(!is_finished)
      {
        //位置控制
        cmd_pub.publish(cmd);
        ros::spinOnce();
        rate.sleep();
        continue;
      }
      ros::spinOnce();
      rate.sleep();
    }
    return NodeStatus::FAILURE;
  }
};

class MoveDown : public SyncActionNode
{
public:
  MoveDown(const std::string &name, const NodeConfig &config)
    : SyncActionNode(name, config)
  {
  }

  static PortsList providedPorts()
  {
    // Optionally, a port can have a human readable description
    const char *description = "Simply print the goal on console...";
    return { InputPort<Distance>("MoveDown_distance", description) };
  }

  NodeStatus tick() override
  {
    auto res = getInput<Distance>("MoveDown_distance");
    if(!res)
    {
      throw RuntimeError("error reading port [MoveDown_distance]:",
                         res.error());
    }
    ros::Rate rate(20.0);
    Distance  target = res.value();
    printf("MoveDown positions: [ %.1f]\n", target.x);
    float lzj_distance     = fcu_pose.pose.position.z + target.x;
    cmd.position.z         = lzj_distance;
    ros::Time last_request = ros::Time::now();
    bool      is_finished  = false;
    while(ros::ok())
    {
      if(abs(fcu_pose.pose.position.z - lzj_distance) < 0.1 && !is_finished)
      {
        ROS_INFO("MoveDown finished");
        last_request = ros::Time::now();
        is_finished  = true;
        return NodeStatus::SUCCESS;
      }
      if(!is_finished)
      {
        //位置控制
        cmd_pub.publish(cmd);
        ros::spinOnce();
        rate.sleep();
        continue;
      }
      ros::spinOnce();
      rate.sleep();
    }
    return NodeStatus::FAILURE;
  }
};

class SendPosition : public SyncActionNode
{
public:
  SendPosition(const std::string &name, const NodeConfig &config)
    : SyncActionNode(name, config)
  {
  }

  static PortsList providedPorts()
  {
    // Optionally, a port can have a human readable description
    const char *description = "Simply print the goal on console...";
    return { InputPort<Position3D>("SendPosition", description) };
  }

  NodeStatus tick() override
  {
    auto res = getInput<Position3D>("SendPosition");
    if(!res)
    {
      throw RuntimeError("error reading port [SendPosition]:", res.error());
    }
    ros::Rate  rate(20.0);
    Position3D target = res.value();
    printf("Sendpositions: [ %.1f, %.1f ,%.1f]\n", target.x, target.y,
           target.z);
    cmd.position.x         = target.x;
    cmd.position.y         = target.y;
    cmd.position.z         = target.z;
    ros::Time last_request = ros::Time::now();
    bool      is_finished  = false;
    while(ros::ok())
    {
      if(abs(fcu_pose.pose.position.x - target.x) < 0.1
         && abs(fcu_pose.pose.position.y - target.y)
         && abs(fcu_pose.pose.position.z - target.z) < 0.1 && !is_finished)
      {
        ROS_INFO("SendPosition finished");
        last_request = ros::Time::now();
        is_finished  = true;
        return NodeStatus::SUCCESS;
      }
      if(!is_finished)
      {
        //位置控制
        cmd_pub.publish(cmd);
        ros::spinOnce();
        rate.sleep();
        continue;
      }
      ros::spinOnce();
      rate.sleep();
    }
    return NodeStatus::FAILURE;
  }
};

class SendVelocity : public SyncActionNode
{
public:
  SendVelocity(const std::string &name, const NodeConfig &config)
    : SyncActionNode(name, config)
  {
  }

  static PortsList providedPorts()
  {
    // Optionally, a port can have a human readable description
    const char *description = "Simply print the goal on console...";
    return { InputPort<Position3D>("SendVelocity", description) };
  }

  NodeStatus tick() override
  {
    auto res = getInput<Position3D>("SendVelocity");
    if(!res)
    {
      throw RuntimeError("error reading port [SendVelocity]:", res.error());
    }
    ros::Rate  rate(20.0);
    Position3D target = res.value();
    printf("SendVelocity: [ %.1f, %.1f ,%.1f]\n", target.x, target.y, target.z);
    cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    cmd.type_mask        = ~uint16_t(0);
    cmd.type_mask &= ~mavros_msgs::PositionTarget::FORCE;
    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_VX;
    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_VY;
    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_VZ;
    cmd.velocity.x         = target.x;
    cmd.velocity.y         = target.y;
    cmd.velocity.z         = target.z;
    bool      is_finished  = false;
    ros::Time last_request = ros::Time::now();
    while(ros::ok())
    {
      if(abs(vx - target.x) < 0.1 && abs(vy - target.y) < 0.1
         && abs(vz - target.z) < 0.1 && !is_finished)
      {
        ROS_INFO("SendVelocity finished");
        last_request = ros::Time::now();
        is_finished  = true;
        return NodeStatus::SUCCESS;
      }
      if(!is_finished)
      {
        //位置控制
        cmd_pub.publish(cmd);
        ros::spinOnce();
        rate.sleep();
        //   rate.sleep()
        continue;
      }
      ros::spinOnce();
      rate.sleep();
      // rate.sleep()
    }
    return NodeStatus::FAILURE;
  }
};

// class SendYaw: public SyncActionNode
// {
//   public:
//     SendYaw(const std::string& name, const NodeConfig& config):
//         SyncActionNode(name,config)
//     {}

//     static PortsList providedPorts()
//     {
//       // Optionally, a port can have a human readable description
//       const char*  description = "Simply print the goal on console...";
//       return { InputPort<Distance>("MoveBack_distance", description) };
//     }

//     NodeStatus tick() override
//     {
//       auto res = getInput<Distance>("MoveBack_distance");
//       if( !res )
//       {
//         throw RuntimeError("error reading port [MoveBack_distance]:",
//         res.error());
//       }
//       Distance target = res.value();
//       printf("Sendpositions: [ %.1f, %.1f ,%.1f]\n", target.x );
//       cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
//       cmd.type_mask = ~uint16_t(0);
//       cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_YAW;
//       cmd.yaw = target.x
//       bool is_finished= false;
//       ros::Time last_request = ros::Time::now();
//       // cmd_pub.publish(cmd);
//       while(ros::ok()){
//         if(abs(fcu_pose.yaw - target.x) < 0.1 && !is_finished)
//         {
//           ROS_INFO("SendYaw finished");
//           last_request = ros::Time::now();
//           is_finished = true;
//           return NodeStatus::SUCCESS;
//         }
//         if(!is_finished)
//         {
//           //位置控制
//           cmd_pub.publish(cmd);
//           ros::spinOnce();
//           rate.sleep()
//           continue;
//         }
//         ros::spinOnce();
//         rate.sleep()
//       }
//     return NodeStatus::FAILURE;
//     }
// };

class SendYaw : public SyncActionNode
{
public:
  SendYaw(const std::string &name, const NodeConfig &config)
    : SyncActionNode(name, config)
  {
  }

  static PortsList providedPorts()
  {
    // Optionally, a port can have a human readable description
    const char *description = "Simply print the goal on console...";
    return { InputPort<Distance>("SendYaw", description) };
  }

  NodeStatus tick() override
  {
    auto res = getInput<Distance>("SendYaw");
    if(!res)
    {
      throw RuntimeError("error reading port [SendYaw]:", res.error());
    }
    ros::Rate rate(20.0);
    Distance  target = res.value();
    printf("SendYaw: [ %.1f]\n", target.x);
    cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    cmd.type_mask        = ~uint16_t(0);
    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_YAW;
    cmd.yaw                = target.x;
    bool      is_finished  = false;
    ros::Time last_request = ros::Time::now();
    double    x            = fcu_pose.pose.orientation.x;
    double    y            = fcu_pose.pose.orientation.y;
    double    z            = fcu_pose.pose.orientation.z;
    double    w            = fcu_pose.pose.orientation.w;
    double    yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    // cmd_pub.publish(cmd);
    while(ros::ok())
    {
      if(abs(yaw - target.x) < 0.1 && !is_finished)
      {
        ROS_INFO("SendYaw finished");
        last_request = ros::Time::now();
        is_finished  = true;
        return NodeStatus::SUCCESS;
      }
      if(!is_finished)
      {
        //位置控制
        cmd_pub.publish(cmd);
        ros::spinOnce();
        rate.sleep();
        //   rate.sleep()
        continue;
      }
      ros::spinOnce();
      rate.sleep();
      // rate.sleep()
    }
    return NodeStatus::FAILURE;
  }
};

class Land : public SyncActionNode
{
public:
  Land(const std::string &name) : SyncActionNode(name, {}) {}

  NodeStatus tick() override
  {
    //降落模式 切换到伺服控制
    cmd.coordinate_frame = mavros_msgs::PositionTarget::
      FRAME_BODY_NED;  // 速度控制基于body坐标系控制
    land.request.custom_mode = "AUTO.LAND";
    if(ros::Time::now().toSec() - cmd.header.stamp.toSec() > 10)
    {  //如果超过两秒中没有更新指令，直接降落

      set_mode_client.call(land);
    }
    cmd_pub.publish(cmd);
    return BT::NodeStatus::SUCCESS;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "offb_node");
  ros::NodeHandle nh;

  // ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
  //         ("/mavros/state", 10, state_cb);
  // ros::Publisher cmd_pub = nh.advertise<mavros_msgs::PositionTarget>
  //         ("/mavros/setpoint_raw/local", 10); //
  // ros::Subscriber pose_usb =
  // nh.subscribe("/mavros/local_position/pose",10,PoseCB); ros::ServiceClient
  // arming_client = nh.serviceClient<mavros_msgs::CommandBool>
  //         ("/mavros/cmd/arming");
  // ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
  //         ("/mavros/set_mode");
  // ros::Publisher att_pub =
  // nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude",10);

  state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
  cmd_pub   = nh.advertise<mavros_msgs::PositionTarget>(
    "/mavros/setpoint_raw/local", 10);  //
  pose_usb = nh.subscribe("/mavros/local_position/pose", 10, PoseCB);
  arming_client =
    nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  att_pub         = nh.advertise<mavros_msgs::AttitudeTarget>(
    "/mavros/setpoint_raw/attitude", 10);

  velocity_sub =
    nh.subscribe("/mavros/local_position/velocity_local", 10, velocityCallback);

  // the setpoint publishing rate MUST be faster than 2Hz
  // rate(20.0);
  ros::Rate rate(20.0);

  BT::BehaviorTreeFactory factory;

  // wait for FCU connection
  ROS_INFO("11111111111111111111111");
  while(ros::ok() && !current_state.connected)
  {
    //判断mavros`是否连接上飞控
    ros::spinOnce();
    ROS_INFO("22222222222222222222222");
    // rate.sleep()
    rate.sleep();
  }

  ROS_INFO("333333333333333333333333");
  ROS_INFO("BT START");

  factory.registerNodeType<Takeoff>("Takeoff");
  factory.registerNodeType<MoveForward>("MoveForward");
  factory.registerNodeType<MoveBack>("MoveBack");
  factory.registerNodeType<MoveRight>("MoveRight");
  factory.registerNodeType<MoveLeft>("MoveLeft");
  factory.registerNodeType<MoveUp>("MoveUp");
  factory.registerNodeType<MoveDown>("MoveDown");
  factory.registerNodeType<SendPosition>("SendPosition");
  factory.registerNodeType<SendVelocity>("SendVelocity");
  factory.registerNodeType<SendYaw>("SendYaw");
  factory.registerNodeType<Land>("Land");

  auto tree = factory.createTreeFromFile(bt_tree_path);
  tree.tickWhileRunning();

  // mavros_msgs::PositionTarget cmd;
  //是否需要控制角度与角速度，如果需要放开注释
  //    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_YAW;
  //    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
  return 0;
}
