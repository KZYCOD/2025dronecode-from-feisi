#include "ros/ros.h"
#include "std_msgs/String.h"
#include <common_msgs/Objects.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <nav_msgs/Odometry.h>

void chatterCallback(const std_msgs::String::ConstPtr &msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

class Test
{
public:
  Test(std::shared_ptr<ros::NodeHandle> ptr)
  {
    nh_ptr = ptr;

    std::shared_ptr<ros::NodeHandle> nh_ptr = ptr;

    message_filters::Subscriber<sensor_msgs::Image>   img_sub(*nh_ptr, "/rflysim/sensor2/img_depth",
                                                            10);
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(*nh_ptr,
                                                              "/mavros/local_position/odom", 10);

    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image,nav_msgs::Odometry>
      SyncPolicy;

    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(100), img_sub,
                                                   odom_sub);
    sync.registerCallback(boost::bind(&Test::message_cb, this, _1, _2));
  }

  void message_cb(const sensor_msgs::Image::ConstPtr &  img,
                  const nav_msgs::Odometry::ConstPtr &odom)
  {
    ROS_INFO("recv depth and odom");
    return;
  }

private:
  std::shared_ptr<ros::NodeHandle> nh_ptr;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh("~");

  Test test(std::make_shared<ros::NodeHandle>(nh));


  ros::spin();

  return 0;
}
