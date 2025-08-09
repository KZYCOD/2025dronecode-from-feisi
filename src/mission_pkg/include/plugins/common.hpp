#ifndef COMMON_H
#define COMMON_H

#include <ros/ros.h>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/control_node.h>
#include <behaviortree_cpp/decorator_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
// #include <pugixml.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>

namespace BT
{

struct Position3D
{
  double x;
  double y;
  double z;
  Position3D(const double &_x, const double &_y, const double &_z)
  {
    x = _x;
    y = _y;
    z = _z;
  }
  Position3D() {}
  Position3D &operator=(const Position3D &tmp)
  {
    this->x = tmp.x;
    this->y = tmp.y;
    this->z = tmp.z;
    return *this;
  }
  friend std::ostream &operator<<(std::ostream &out, Position3D pos);
};
std::ostream &operator<<(std::ostream &out, Position3D pos)
{
  out << "x: " << pos.x << ", y: " << pos.y << ", z: " << pos.z;
  return out;
}
/**
 * @brief
 * 把参数字符穿分割成Position3D结构，在使用getInput解析参数时，BT内会自动调用这个内联函数
 * @param
 * @return 返回值描述
 */
template <> inline Position3D convertFromString(StringView str)
{
  auto parts = splitString(str, ';');
  if(parts.size() != 3)
  {
    throw RuntimeError("invalid input");
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

}  // namespace BT
#endif  // COMMON_H
