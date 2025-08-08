#include "plugins/action/calobjpos.h"

CalObjPos::CalObjPos(const std::string name, const NodeConfig &config)
  : StatefulActionNode(name, config)
{
  mct          = MavRosConnect::getInstance();
  nh           = mct->getROSHandle();
  fcu_pose_ptr = mct->getFcuPose();
  std::string det_topic;
  nh->param<std::string>("detect_resutl_topic", det_topic, "/objects");

  std::string align_depth_topic;
  nh->param<std::string>("align_depth", align_depth_topic, "/depth");

  std::string obj_pose_topic;
  nh->param<std::string>("obj_pose_topic", obj_pose_topic, "/obj_pos");

  nh->param<std::vector<double>>("cam2body_R", cam2body_R,
                                 std::vector<double>());
  nh->param<std::vector<double>>("cam2body_T", cam2body_T,
                                 std::vector<double>());
  int depth_w;
  int depth_h;
  nh->param("depth_w", depth_w, 0);
  nh->param("depth_h", depth_h, 0);

  if(depth_w < 0 || depth_h < 0)
  {
    ROS_ERROR("the depth param set fial");
    return;
  }
  depth_info.width  = depth_w;
  depth_info.height = depth_h;

  nh->param("fx", depth_info.K[0], 0.);
  nh->param("fy", depth_info.K[4], 0.);
  nh->param("cx", depth_info.K[2], 0.);
  nh->param("cy", depth_info.K[5], 0.);

  depth_img_sub = nh->subscribe<sensor_msgs::Image>(
    align_depth_topic, 10, boost::bind(&CalObjPos::DepthImgCB, this, _1));
  objs_sub = nh->subscribe<common_msgs::Objects>(
    det_topic, 10, boost::bind(&CalObjPos::ObjsCallback, this, _1));

  obj_pos_pub = nh->advertise<geometry_msgs::PoseArray>(obj_pose_topic, 10);
}

CalObjPos::~CalObjPos()
{
  ;
}

void CalObjPos::ObjsCallback(const common_msgs::Objects::ConstPtr &objs)
{
  if(class_name.empty(), depth_queue.size() == 0)
    return;

  sensor_msgs::Image depth;
  {
    std::unique_lock<std::mutex> lock(depth_mtx);
    while(!depth_queue.empty())
    {
      if(abs(depth_queue.front().header.stamp.toSec()
             - objs->header.stamp.toSec())
         < 0.03)  //这里如果是做了同步的话，
      {
        depth = depth_queue.front();
        depth_queue.pop();
        break;
      }
      depth_queue.pop();
    }
  }

  if(depth_info.width < INT_MIN || depth_info.height < INT_MIN
     || depth_info.K[0] < DBL_MIN || depth_info.K[2] < DBL_MIN
     || depth_info.K[4] < DBL_MIN || depth_info.K[5] < DBL_MIN)
  {
    ROS_ERROR("haven't to set depth camera Intrinsics, or not subscrib cmaera "
              "infomation topic ");
    return;
  }

  if(depth.encoding != "16UC1" && depth.encoding != "32FC1")
  {
    ROS_ERROR("Unsupported depth encoding: %s", depth.encoding.c_str());
    return;
  }
  int pixel_size =
    (depth.encoding == "16UC1") ? 2 : 4;  // 16位=2字节，32位=4字节

  geometry_msgs::PoseArray objs_pose;
  for(size_t i = 0; i < objs->objects.size(); ++i)
  {
    if(objs->objects[i].class_name != class_name)
      continue;  //如果不是制定目标，不做位置计算
    auto   obj_x = objs->objects[i].center_x;
    auto   obj_y = objs->objects[i].center_y;
    int    index = obj_y * depth.step + obj_x * pixel_size;
    double z     = 0;
    // 读取深度值
    if(depth.encoding == "16UC1")
    {
      // 16位无符号整数（单位：毫米）
      uint16_t depth_mm =
        *reinterpret_cast<const uint16_t *>(depth.data[index]);
      ROS_INFO("Center depth (16UC1): %u mm", depth_mm);
      z = depth_mm / 100.0;
    }
    else if(depth.encoding == "32FC1")
    {
      // 32位浮点数（单位：米）
      float depth_m = *reinterpret_cast<const float *>(depth.data[index]);
      ROS_INFO("Center depth (32FC1): %.3f m", depth_m);
      z = depth_m;
    }

    if(z < DBL_MIN)
    {
      ROS_WARN("current objs (%s), not get dpeth", objs->objects[i].class_name);
      continue;
    }
    auto x = (obj_x - depth_info.K[2]) * z / depth_info.K[0];
    auto y = (obj_y - depth_info.K[5]) * z / depth_info.K[4];
    ROS_INFO("current obj pos: (%f,%f,%f)", x, y, z);
    double x_ =
      x * cam2body_R[0] + y * cam2body_R[1] + z * cam2body_R[2] + cam2body_T[0];
    double y_ =
      x * cam2body_R[3] + y * cam2body_R[4] + z * cam2body_R[5] + cam2body_T[1];
    double z_ =
      x * cam2body_R[6] + y * cam2body_R[7] + z * cam2body_R[8] + cam2body_T[2];
    tf2::Quaternion q;
    tf2::convert(fcu_pose_ptr->pose.orientation, q);
    tf2::Vector3 t;
    tf2::convert(fcu_pose_ptr->pose.position, t);

    tf2::Transform trans;
    trans.setOrigin(t);
    trans.setRotation(q);

    tf2::Vector3        b_p(x_, y_, z_);
    tf2::Vector3        w_p = trans * b_p;
    geometry_msgs::Pose obj_p;
    obj_p.position.x = w_p.x();
    obj_p.position.y = w_p.y();
    obj_p.position.z = w_p.z();
    objs_pose.poses.push_back(obj_p);
  }
  objs_pose.header = objs->header;
  obj_pos_pub.publish(objs_pose);
  ros::spinOnce();
}

void CalObjPos::DepthImgCB(const sensor_msgs::Image::ConstPtr &img)
{
  {
    std::unique_lock<std::mutex> lock(depth_mtx);
    depth_queue.push(*img);
    if(depth_queue.size()
       > 30)  //深度相机30hz，超过10hz的数据近1s, 目标检测时间不能超过这个值
    {
      depth_queue.pop();
    }
  }
}

void CalObjPos::DepthInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &info)
{
  depth_info = *info;
}

PortsList CalObjPos::providedPorts()
{
  return { InputPort("class_name", "we need to set object name") };
}

NodeStatus CalObjPos::onStart()
{
  //  sync_->connectInput(*img_sub_, *objs_sub_);
  //  sync_->registerCallback(boost::bind(&CalObjPos::SyncCallback, this, _1,
  //  _2));
  auto ret = getInput<std::string>("class_name");
  if(!ret)
  {
    throw RuntimeError("error reading prot [class_name]", ret.error());
  };
  class_name = ret.value();
  return NodeStatus::RUNNING;
}

NodeStatus CalObjPos::onRunning()
{
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  return NodeStatus::RUNNING;
  ;
}

void CalObjPos::onHalted()
{
  ;
}
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<CalObjPos>("CalObjPos");
}
