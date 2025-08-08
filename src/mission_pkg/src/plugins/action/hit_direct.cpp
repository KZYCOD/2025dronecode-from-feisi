#include "plugins/action/hit_direct.h"

HitDirect::HitDirect(const std::string &name, const NodeConfig &config)
    : StatefulActionNode(name, config)
{
  mct = MavRosConnect::getInstance();
  // 需要根据当前飞机位姿，计算目标在全局坐标系下的位置
  fcu_pose_ptr = mct->getFcuPose();
  fcu_state_ptr = mct->getFcuState();
  tgt_pose_pub_ptr = mct->getPosTgtPublier();
  nh = mct->getROSHandle();

  nh->param<std::string>("det_topic", det_topic, "/objects");
  nh->param<int>("img_w", rgb_image_w, 640);
  nh->param<int>("img_h", rgb_image_h, 480);
  rgb_cnt_x = int(rgb_image_w /2);
  rgb_cnt_y = int(rgb_image_h /2);
  nh->param<double>("hight_min", hight_min, 0.5);
  nh->param<double>("hight_max", hight_max, 1.0);
  
  nh->param<double>("obs_dist", obs_dist, 0.5);
  nh->param<double>("v_max", v_max, 2);
  nh->param<double>("fx", fx, 320.0);
  nh->param<double>("fy", fy, 320.0);
  nh->param<double>("cx", cx, 240.0);
  nh->param<double>("cy", cy, 240.0);
  nh->param<int>("depth_len",depth_len,20);
  nh->param<int>("obj_len", objs_len,2);  //尽管队列存储多个目标，但是用作计算坐标的只能用作最后一个
  nh->param("len_sequence_objs", len_sequence_objs, 10);
  // std::string odom_topic;
  std::string obs_cloud_topic;
  std::string depth_topic;
  std::string pos_cmd_topic;
  nh->param<std::string>("depth_topic", depth_topic, "/rflysim/sensor2/img_depth");
  // nh->param("odom_topic",odom_topic,"/mavros/local_position/odom");
  nh->param<std::string>("obs_cloud_topic", obs_cloud_topic, "/grid_map/occupancy_inflate");
  nh->param<std::string>("planner_cmd_topic", pos_cmd_topic, "/planning/pos_cmd");

  nh->param("kx", kx, 0.0);
  nh->param("ky", ky, 0.0); 
  nh->param("kz", kz, 0.0);
  nh->param("kyaw_rate", k_yaw, 0.0);
  nh->param("pose_len",pose_len,100);

  obj_vis_pub = nh->advertise<visualization_msgs::Marker>("object_vis", 10);
  det_sub = nh->subscribe<common_msgs::Objects>(det_topic, 10,
                                                bind(&HitDirect::RecvObj, this, _1));

  // obs_sub = nh->subscribe<sensor_msgs::PointCloud2>(obs_cloud_topic, 10, boost::bind(&Hit::RecvPointCloud, this, _1));
  depth_sub = nh->subscribe<sensor_msgs::Image>(depth_topic, 1, boost::bind(&HitDirect::DepthCallback, this, _1));

  goal_pub = nh->advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
  test_pose_pub = nh->advertise<visualization_msgs::Marker>("/test_pose_maker",10);
  //创建一个定时去存储一段时间的位姿数据
  pose_timer = nh->createTimer(ros::Duration(0.03), boost::bind(&HitDirect::timerCallback,this,_1));
  
  //行为树里面不支持ROS 的这种同步的方式
  // detection_sub_.subscribe(*nh, det_topic , 10);
  // depth_sub_.subscribe(*nh, depth_topic, 10);
  // message_filters::Synchronizer<ExactPolicy> sync(ExactPolicy(30), detection_sub_, depth_sub_);
  // sync.registerCallback(boost::bind(&Hit::DepthObjCallback, this, _1, _2));

  // message_filters::Subscriber<sensor_msgs::Image> image_sub(*nh, depth_topic, 1);
  // message_filters::Subscriber<common_msgs::Objects> det_sub(*nh, det_topic, 1);
    
  // // message_filters::TimeSynchronizer<sensor_msgs::Image, common_msgs::Objects> sync(image_sub, det_sub, 10);
  // // sync.registerCallback(boost::bind(&callback, _1, _2));

 
  // message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), det_sub, image_sub);
  // // sync.setSlop(0.05);
  // sync.registerCallback(boost::bind(&Hit::DepthObjCallback,this, _1, _2));

  isDetect = false;
  isHit_enable = false;
  yaw_err = 0.0;

  cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
  cmd.type_mask = 0xfff;
  cmd.type_mask &=
      ~mavros_msgs::PositionTarget::FORCE; // px4 不响应力的控制方式
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PX;
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PY;
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PZ;
  // cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
  //cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_YAW;
  cmd.header.frame_id = "Hit";
  ;
}

PortsList HitDirect::providedPorts()
{

  return {InputPort("object_name", "input hitted object"),
          InputPort("method", "hit object method,direct or other"),
          InputPort("hit_dist","when the to object distane less the value"),
          InputPort("goal_tolerance","the value greate than obj_inflate ande less than his_dist"),
          InputPort("needle_ang_adapt","the value to adapt the needle fixed angle"),
          InputPort("left_line","no discript"),
          InputPort("right_line","no discript"),
          InputPort("yaw_err","when the object's yaw subtract fcu's yaw lees than value ,considered equ")           
          };
}

NodeStatus HitDirect::onStart()
{
  ROS_INFO("in hit node");
  // return NodeStatus::RUNNING;
   
  if (fcu_state_ptr->mode != "OFFBOARD" || !fcu_state_ptr->armed)
  {
    ROS_ERROR("current fcu state cann't to fly");
    return NodeStatus::FAILURE;
  }
  std::deque<geometry_msgs::PoseStamped>().swap(obj_tra);
  std::queue<common_msgs::Obj>().swap(objs_queue);
  auto object_name_port = getInput<std::string>("object_name");
  if (!object_name_port)
  {
    throw RuntimeError("error reading prot[balloon_name]", object_name_port.error());
  }
  obj_name = object_name_port.value();
  ROS_INFO("we will hited %s", obj_name.c_str());

  auto hit_method_port = getInput<int>("method");
  if (!hit_method_port)
  {
    throw RuntimeError("error reading port[method in Hit]", hit_method_port.error());
  }
  hit_method = static_cast<HitDirect::Method>(hit_method_port.value());

  if (hit_method == HitDirect::Method::direct)
  { // 如是直接撞击目标，直接对isHit_enable赋值
    isHit_enable = true;
  }
  auto hit_dist_port = getInput<double>("hit_dist");
  if(!hit_dist_port)
  {
    throw RuntimeError("error reading port[hit_dist in Hit]", hit_dist_port.error());
  }
  hit_dist = hit_dist_port.value();

  auto goal_toler_port = getInput<double>("goal_tolerance");
  if(!goal_toler_port)
  {
    throw RuntimeError("error rading port[goal_tolerance_port] in Hit",goal_toler_port.error());
  }
  goal_tolerance = goal_toler_port.value();
  auto needle_adapt_port = getInput<double>("needle_ang_adapt");
  if(!needle_adapt_port)
  {
    throw RuntimeError("error rading port[needle_adapt_port] in Hit",needle_adapt_port.error());
  }
  needle_adapt = needle_adapt_port.value();

  auto left_line_port = getInput<int>("left_line");
  if(!left_line_port)
  {
    throw RuntimeError("error rading port[left_line] in Hit",left_line_port.error());
  }
  left_line = left_line_port.value();
  auto right_line_port = getInput<int>("right_line");
  if(!right_line_port)
  {
     throw RuntimeError("error rading port[left_line] in Hit",right_line_port.error());
  }
  right_line = right_line_port.value();

  auto yaw_err_port = getInput<double>("yaw_err");
  if(!yaw_err_port)
  {
    throw RuntimeError("error reading port[yaw_err] in Hit",yaw_err_port.error());
  }
  yaw_err = yaw_err_port.value();

  return NodeStatus::RUNNING; // 如果在onStart()里面返回SUCCESS 节点将会结束
  ;
}

NodeStatus HitDirect::onRunning()
{ 
  
  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 可适当降低执行频率，来降低计算量
   
  if (fcu_state_ptr->mode != "OFFBOARD" || !fcu_state_ptr->armed)
  {
    ROS_ERROR("current fcu state cann't to fly");
    return NodeStatus::FAILURE;
  }
  static bool isDirect = false;
  if(!isDetect)
  {//没有检测到目标不控制飞机
    ROS_INFO("Not recv object data");
    //保持悬停
    nh->setParam("stop_hover_port", false); // 结束悬停
    return NodeStatus::RUNNING;
  }
  if(isDetect && ros::Time::now() - last_obj_time > ros::Duration(1) && !isDirect)
  {
    ROS_INFO("lost object detect data");
    //保持悬停
    nh->setParam("stop_hover_port", false); // 结束悬停
    return NodeStatus::RUNNING;
    ;
  }
  //结束悬停，直接使用视觉伺服控制
  //直接使用速度控制
  ROS_INFO("have recv obje cnt:%d, %d",obj.center_x,obj.center_y);
  bool stop_hover = false;
  nh->getParam("stop_hover_port", stop_hover);
  if(!stop_hover)
  {
    nh->setParam("stop_hover_port", true); // 结束悬停
    ROS_INFO("to stop hover");
    return NodeStatus::RUNNING;
  }
  int dx =  rgb_cnt_x - obj.center_x ;
  int dy =  rgb_cnt_y - obj.center_y;
  float vx = 0;
  ROS_INFO("dx:%d, dy:%d",dx,dy);
  //计算目标占用飞机图像大小
  static int img_area = rgb_image_h * rgb_image_w;
  auto scale = (obj.right_bottom_x - obj.left_top_x)*(obj.right_bottom_y - obj.left_top_y) / double(img_area);
  ROS_INFO("scale : %f",scale);


  auto vy = kx * dx;
  auto vz = kz * dy;
  vx = v_max-abs(vy+vz);
  if(vx < 0)
  {
    vx = 0.01;
  }
  if(vx < 0)
  if(scale > 0.1 )
  { //大于这个值的时候，看看目标中心是否满足直接刺的条件，不满足就调整
    ROS_WARN("current scale greate than scale_param");
    if(std::abs(dx) > 30)  //微调位置
      vy = kx * dx;
    if(std::abs(dy) > 20) //微  调位置
      vz= kz* dy;
    else
      isDirect = true; //已经近距离锁定目标，直接刺破
  }

  if(isDirect)
  {
    vx = v_max;
    vy = 0;
    vz = 0; 
  }
  if(abs(vy) > v_max)
  {
    vy = vy/abs(vy) * v_max;
  }
  if(abs(vz) > v_max)
  {
    vz = vz/abs(vz)*v_max;
  }

  ROS_INFO("vx:%f vy:%f vz:%f ",vx,vy,vz);
  cmd.velocity.x = vx;
  cmd.velocity.y = vy;
  cmd.velocity.z = vz;

  tgt_pose_pub_ptr->publish(cmd);


  return NodeStatus::RUNNING;
}

void HitDirect::onHalted()
  {
    ;
  }


bool HitDirect::getObjPosition(geometry_msgs::PoseStamped &goal)
{
  ROS_INFO("objs_queue size: %d,depth_buffer_.size: %d",objs_queue.size(),depth_buffer_.size());
  if(!isDetect  || depth_buffer_.size() == 0)
  { 
    return false;
  }
  // common_msgs::Obj  obj;
  {
    // std::unique_lock<std::mutex> lock(mtx, std::try_to_lock);
    // if (lock.owns_lock())
    {
      // obj = objs_queue.back();
      // objs_queue.pop(); //动态更新
    }
  }

  if (obj.center_x < 1 || obj.center_y < 1)
  { // 没有检测到目标，不需要计算
    ROS_WARN("get obj data error ,it center not set");
    return false;
  }

  ros::Time target_time = obj.header.stamp;
  auto it = std::lower_bound(
    depth_buffer_.begin(),
    depth_buffer_.end(),
    target_time,
    [](const auto& a, ros::Time t) { return a->header.stamp < t; }
    );

  sensor_msgs::Image::ConstPtr best_match;
  float depth = 0.0;
  {
      // std::lock_guard<std::mutex> lock(detph_mtx);
      // std::unique_lock<std::mutex> lock(detph_mtx);
      if (it == depth_buffer_.begin())
      {
        best_match = *it;
      }
      else if (it == depth_buffer_.end())
      {
        best_match = depth_buffer_.back();
      }
      else
      {
        auto prev_it = std::prev(it);
        double diff1 = fabs(((*it)->header.stamp - target_time).toSec());
        double diff2 = fabs(((*prev_it)->header.stamp - target_time).toSec());
        best_match = (diff1 < diff2) ? *it : *prev_it;
      }

      ROS_INFO("diff time %f",fabs((best_match->header.stamp - obj.header.stamp).toSec()));
    
      
      const int pixel_idx = obj.center_y * best_match->width + obj.center_x;
      static int bytes = best_match->data.size() / (best_match->width * best_match->height);
      const int byte_offset = pixel_idx * (best_match->step / best_match->width);
      if (bytes == 2)
      { // 表明是一个像素占两个字节
        uint16_t raw_depth;
        memcpy(&raw_depth, &best_match->data[byte_offset], sizeof(uint16_t));
        depth = raw_depth / 1000.0f; // 转换为米
      }
      else if (bytes == 4)
      {
        memcpy(&depth, &best_match->data[byte_offset], sizeof(float));
      }
      if (depth <= 0.0 || std::isnan(depth))
      { // 无效点
        ROS_INFO("the depth image not aviable");
        return false;
      }
  }
    
    // 直接拿球的RGB中心位置，再深度图里面做位置解算（当然前提是，使用深度与RGB对齐后的数据）
    float cam_x = (obj.center_x - cx) / fx;
    float cam_y = (obj.center_y - cy) / fy;
    float cam_z = depth;
    // 将点转换到map坐标系下
    // x = cam_z; y =-cam_x; z = -cam_y;
    ROS_INFO("cam_x:%f, cma_y:%f,cam_z:%f",cam_x,cam_y,cam_z);
    geometry_msgs::PoseStamped body_p;
    body_p.header.stamp =  best_match->header.stamp;
    body_p.pose.position.x = cam_z;
    body_p.header.frame_id = "body";
    body_p.pose.position.y = -cam_x;
    body_p.pose.position.z = -cam_y;

    //获取与深度图最佳匹配的飞机位姿数据
    {
      ros::Time target_time = obj.header.stamp;
      auto it = std::lower_bound(
      pose_buffer_.begin(),
      pose_buffer_.end(),
      target_time,
      [](const auto& a, ros::Time t) { return a.header.stamp < t; }
      );
      geometry_msgs::PoseStamped best_pose;
      std::unique_lock<std::mutex> lock(detph_mtx);
      if (it == pose_buffer_.begin())
      {
         best_pose = *it;
      }
      else if (it == pose_buffer_.end())
      {
        best_pose = pose_buffer_.back();
      }
      else
      {
        auto prev_it = std::prev(it);
        double diff1 = fabs(((*it).header.stamp - target_time).toSec());
        double diff2 = fabs(((*prev_it).header.stamp - target_time).toSec());
        best_pose = (diff1 < diff2) ? *it : *prev_it;
      }

      
      //转换目标到飞机的local系下
  Eigen::Quaterniond q(
        best_pose.pose.orientation.w,
        best_pose.pose.orientation.x,
        best_pose.pose.orientation.y,
        best_pose.pose.orientation.z);
	// Eigen::Matrix3d rotation_matrix = q.toRotationMatrix();




	// 从旋转矩阵中提取欧拉角
	// Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0);
    	// ROS_INFO("roll:%f,pitch:%f,yaw:%f",euler_angles[2],euler_angles[1],euler_angles[0]);
      Eigen::Vector3d t(
        best_pose.pose.position.x,
        best_pose.pose.position.y,
        best_pose.pose.position.z);
      Eigen::Vector3d p_body(
        body_p.pose.position.x,
         body_p.pose.position.y,
         body_p.pose.position.z);
    Eigen::Vector4f p_body2(
         body_p.pose.position.x,
         body_p.pose.position.y,
         body_p.pose.position.z,
         1
         );

    Eigen::Vector3d trans_ = t;
    Eigen::Vector3d rotat_;
    Eigen::Matrix4f SE3_matrix_;
    toEulerAngle(q,rotat_);

    float sin_roll = sin(rotat_.x());
    float cos_roll = cos(rotat_.x());
    float cos_pitch = cos(rotat_.y());
    float sin_pitch = sin(rotat_.y());
    float sin_yaw = sin(rotat_.z());
    float cos_yaw = cos(rotat_.z());

    SE3_matrix_(0, 0) = cos_pitch * cos_yaw;
    SE3_matrix_(1, 0) = cos_pitch * sin_yaw;
    SE3_matrix_(2, 0) = -sin_pitch;
    SE3_matrix_(3, 0) = 0;

    SE3_matrix_(0, 1) = sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw;
    SE3_matrix_(1, 1) = sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw;
    SE3_matrix_(2, 1) = sin_roll * cos_pitch;
    SE3_matrix_(3, 1) = 0;

    SE3_matrix_(0, 2) = cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw;
    SE3_matrix_(1, 2) = cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw;
    SE3_matrix_(2, 2) = cos_roll * cos_pitch;
    SE3_matrix_(3, 2) = 0;

    SE3_matrix_(0, 3) = trans_.x();
    SE3_matrix_(1, 3) = trans_.y();
    SE3_matrix_(2, 3) = trans_.z();
    SE3_matrix_(3, 3) = 1;


    // 执行变换: R*p_body + t
    // Eigen::Vector3d p_local1 = rotation_matrix * p_body + t;
    Eigen::Vector4f p_local1 = SE3_matrix_ * p_body2;
    
    // Eigen::Vector4f test(1.0,0.0,0.0,1.0);
    // Eigen::Vector4f test_pose = SE3_matrix_ * test;
    // geometry_msgs::PoseStamped test_p;
    // test_p.pose.position.x = test_pose[0];
    // test_p.pose.position.y = test_pose[1];
    // test_p.pose.position.z = test_pose[2];
    // goal = test_p;
    // return true;
    // VisualTest(test_p);
    // ROS_INFO("XXXX======================");
    /*
    visualization_msgs::Marker marker;
    // 设置marker的基本属性
    marker.header.frame_id = "map";  // 参考坐标系
    marker.ns = "test";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    
    // 设置圆点位置
    marker.pose.position.x = test_pose[0];
    marker.pose.position.y = test_pose[1];
    marker.pose.position.z = test_pose[2];
    // 设置圆点大小
    marker.scale.x = 0.5;  // 直径
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    
    // 设置圆点颜色 (红色)
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;  // 不透明度
    
    marker.lifetime = ros::Duration();  // 0表示永远存在

    
    marker.header.stamp = ros::Time::now();
    test_pose_pub.publish(marker);
    ros::spinOnce();
    */
    ROS_INFO("P_L1 x:%f,y:%f,z:%f",p_local1[0],p_local1[1],p_local1[2]);
    Eigen::Vector3d p_local3 = q * p_body + t;
    ROS_INFO("P_L3 x:%f,y:%f,z:%f",p_local3[0],p_local3[1],p_local3[2]);
    Eigen::Vector3d p_local = p_body + t;
    ROS_INFO("P_L2 x:%f,y:%f,z:%f",p_local[0],p_local[1],p_local[2]);
    goal.pose.position.x = p_local1[0];
    goal.pose.position.y = p_local1[1];
    goal.pose.position.z = p_local1[2];
    ROS_INFO("xxxxgoal: x:%f,y:%f,z:%f",goal.pose.position.x,goal.pose.position.y,goal.pose.position.z);
    }
    
    
    return true;
  
}

void HitDirect::toEulerAngle(const Eigen::Quaterniond &q, Eigen::Vector3d &rotat)
{
    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    rotat.x() = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
    if (fabs(sinp) >= 1)
        rotat.y() = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        rotat.y() = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    rotat.z() = atan2(siny_cosp, cosy_cosp);
}

void HitDirect::DepthObjCallback(const common_msgs::Objects::ConstPtr &objs,const sensor_msgs::Image::ConstPtr &depth)
{
      // 检查时间差是否在允许范围内
    // ROS_ASSERT(image->header.stamp == detection->header.stamp);
  ROS_INFO("obj stmp:%f, depth stmp: %f",objs->header.stamp.toSec(), depth->header.stamp.toSec());

}

void HitDirect::timerCallback(const ros::TimerEvent& event)
{
    {
    // ROS_INFO("detph time %d.%09d",msg->header.stamp.sec,msg->header.stamp.nsec);
    std::lock_guard<std::mutex> lock(mct->pose_mtx);
    //auto text_ptr = mct->getFcuPose();
    auto it = std::lower_bound(
            pose_buffer_.begin(), 
            pose_buffer_.end(), 
            *fcu_pose_ptr,
            [](const auto& a, const auto& b) { return a.header.stamp < b.header.stamp; }
    );
    pose_buffer_.insert(it, *fcu_pose_ptr);
        // 限制队列大小
    if (pose_buffer_.size() > pose_len)
    {
        // auto first_msg = depth_buffer_.front();
        // ROS_INFO("detph time %d.%09d",first_msg->header.stamp.sec,first_msg->header.stamp.nsec);
        pose_buffer_.pop_front();
    }
    return;
  }
}

void HitDirect::DepthCallback(const sensor_msgs::Image::ConstPtr &msg)
{ 
  {
    // ROS_INFO("detph time %d.%09d",msg->header.stamp.sec,msg->header.stamp.nsec);
    std::unique_lock<std::mutex> lock(detph_mtx);
    auto it = std::lower_bound(
            depth_buffer_.begin(), 
            depth_buffer_.end(), 
            msg,
            [](const auto& a, const auto& b) { return a->header.stamp < b->header.stamp; }
    );
    depth_buffer_.insert(it, msg);
        // 限制队列大小
    if (depth_buffer_.size() > depth_len)
    {
        // auto first_msg = depth_buffer_.front();
        // ROS_INFO("detph time %d.%09d",first_msg->header.stamp.sec,first_msg->header.stamp.nsec);
        depth_buffer_.pop_front();
    }
    return;
  }

    // // 直接拿球的RGB中心位置，再深度图里面做位置解算（当然前提是，使用深度与RGB对齐后的数据）
    // if (obj.center_x < 1 || obj.center_y < 1)
    // { // 没有检测到目标，不需要计算
    //   return;
    // }
    // float depth = 0.0;
    // const int pixel_idx = obj.center_y * msg->width + obj.center_x;
    // static int bytes = msg->data.size() / (msg->width * msg->height);
    // const int byte_offset = pixel_idx * (msg->step / msg->width);
    // if (bytes == 2)
    // { // 表明是一个像素占两个字节
    //   uint16_t raw_depth;
    //   memcpy(&raw_depth, &msg->data[byte_offset], sizeof(uint16_t));
    //   depth = raw_depth / 1000.0f; // 转换为米
    // }
    // else if (bytes == 4)
    // {
    //   memcpy(&depth, &msg->data[byte_offset], sizeof(float));
    // }

    // if (depth <= 0.0 || std::isnan(depth))
    // { // 无效点
    //   return;
    // }

    // float cam_x = (obj.center_x - cx) / fx;
    // float cam_y = (obj.center_y - cy) / fy;
    // float cam_z = depth;
    // // 将点转换到map坐标系下
    // // x = cam_z; y =-cam_x; z = -cam_y;
    // geometry_msgs::PoseStamped body_p;
    // body_p.pose.position.x = cam_z;
    // body_p.header.frame_id = "body";
    // body_p.pose.position.y = -cam_x;
    // body_p.pose.position.z = -cam_y;
    // geometry_msgs::PoseStamped ball_p;
    // static tf2_ros::Buffer tf_buffer;
    // static tf2_ros::TransformListener tf_listener(tf_buffer);
    // try
    // { // 这里需要查看TF树
    //   goal = tf_buffer.transform(body_p, "camera_init");
    //   // ROS_INFO("goal in map: %f,%f,%f", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);

    // }
    // catch (tf2::TransformException &ex)
    // {
    //   ROS_WARN("Transform failure: %s", ex.what());
    // }
    // 发送目标点给路径规划节点
}


void HitDirect::RecvObj(const common_msgs::Objects::ConstPtr &msg)
  { // 这里需要的目标是目标分割后的中心位置，而非检测框的中心位置；

    // ROS_INFO("recv objsize in hit: %d", msg->objects.size());
    obj_name = "balloon";

    if (msg->objects.empty() || obj_name.empty())
      return;
    
    static int idx = -1;
    static double score = -1.0;

    for (size_t i = 0; i < msg->objects.size(); ++i)
    {
      if (msg->objects[i].class_name != obj_name)
        continue;
      // ROS_INFO("Recv ball object");
      if (msg->objects[i].score > score)
      {
        idx = i;
        score = msg->objects[i].score;
      }
    };
    if (msg->objects[idx].score < 0.5 || msg->objects[idx].class_name != obj_name)
    { // 置信度太低了，
      idx = -1;
      score = -1;
      // std::unique_lock<std::mutex> lock(mtx, std::try_to_lock);
      // if (lock.owns_lock())
      {
        obj.center_x = 0;
        obj.center_y = 0;
      }
      return;
      // ROS_WARN("The object score is %f,or havn't need to hit object", msg->objects[idx].score);
    }
    
    // std::unique_lock<std::mutex> lock(mtx, std::try_to_lock);
    // if (lock.owns_lock())
    {
      obj = msg->objects[idx];
      last_obj_time = ros::Time::now();
      ROS_INFO("recv obj time: %d.%09d", obj.header.stamp.sec, obj.header.stamp.nsec);
      // objs_queue.push(obj);
      isDetect = true;
      // if(objs_queue.size() > objs_len)
      // {
      //   auto f = objs_queue.front();
      //   // ROS_INFO("first obj time: secs: %d.%09d ", f.header.stamp.sec, f.header.stamp.nsec);
      //   objs_queue.pop();
      // }
      

    }
    idx = 0;
    score = 0;
  }

  /*
  void Hit::RecvPointCloud(const livox_ros_driver2::CustomMsg::ConstPtr &msg)
  {
    // 需要过滤些点云，如雷达罩，远处点，为了减少计算这里过滤出一个正方体的点集
    std::vector<livox_ros_driver2::CustomPoint> tmp;
  #pragma omp parallel shared(obs_ps)
    {
      for (size_t i = 0; i < msg->points.size(); ++i)
      {
        if (abs(msg->points[i].x) < obs_dist && abs(msg->points[i].y) < obs_dist && abs(msg->points[i].z) < obs_dist)
        {
          tmp.push_back(msg->points[i]);
          ;
        }
      }
  #pragma omp barrier
  #pragma omp single
      {   // 只有在需要的时候才会用上这个，也就是当飞机需要在x,y方向的值后开始判断
        ; // 只有一个线程执行打印
        std::unique_lock<std::mutex> lock(pc_mtx);
        obs_ps.swap(tmp);
      }
    }
  }
  */

  void HitDirect::RecvPointCloud(const sensor_msgs::PointCloud2::ConstPtr &msg)
  {
    /*
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    try
    {
      // 这里直接从tf树里面获取到飞机当前的位置，这几从避障模块获得到的点云，使用最新的odom计算是不准确的，准确的来讲应该直接获取到雷达获得深度点云，但是那样会带来更多的计算量
      geometry_msgs::TransformStamped transform =
          tf_buffer.lookupTransform("base_link", "odom", ros::Time(0));

      pcl_ros::transformPointCloud("base_link", transform, *msg, obs_ps);
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
    }
    */
  }

  bool HitDirect::CalFlyOri(double &vx, double &vy, double &vz, double &yaw_rate)
  {
    /*
    if (obs_ps.empty()) // 如果没有障碍物阻挡，可直接使用 速度 + 偏航控制
      return false;

    // 说明安全范围内有障碍物点，应该避让
    std::vector<livox_ros_driver2::CustomPoint> tmp;
    {
      std::unique_lock<std::mutex> lock(pc_mtx);
      tmp.swap(obs_ps);
    }

    if (vx > 0)
    { // 飞机往前飞，扫描前方安全飞行范围内是否有障碍物
      // 仅仅使用yaw_rate 和 vx
      // 已经不能规避障碍物了，需要控制vy方向,加单把点云分成四个象限
      // ，但是点肯定有重叠的地方
      std::vector<livox_ros_driver2::CustomPoint> fp;
      std::vector<livox_ros_driver2::CustomPoint> bp;
      std::vector<livox_ros_driver2::CustomPoint> lp;
      std::vector<livox_ros_driver2::CustomPoint> rp;
      for (size_t i = 0; i < tmp.size(); ++i)
      {
        if (tmp[i].x > 0)
        {
          fp.push_back(tmp[i]);
        }
        else
        {
          bp.push_back(tmp[i]);
        }
        if (tmp[i].y > 0)
        {
          lp.push_back(tmp[i]);
        }
        else
        {
          rp.push_back(tmp[i]);
        }
      }

      if (yaw_rate > 0 && lp.size())
      {             // 飞机向左转向，转为飞机向左飞，转向停止
        vy = v_max; // 这里使用最粗暴的方式，后续根据情况再改
        vx = 0;     // 停止向前飞
        yaw_rate = 0;
      }
    }
    else
    {
      ;
    }
    */
  }

double HitDirect::PredictObjOri()
{//预测目标的行径方向，这个是一个不那么确定的操作，只能使用先验知识预测，涉及到多个方面，如目标速度，飞机的观测频率，计算性能等。
//只有实时性越高预测的越准，但是预测的越准，要想打击目标控制响应也要越快才能命中目标
//因此这个函数未必用的上，但是如果目标速度慢，预测还有是有效的。
  if(obj_tra.size() == 0)
  {
    ROS_WARN("obj_tra empty in PredictObjOri");
    return FLT_MAX;
  }

  //这里预测也简单，就是通过目标
  geometry_msgs::PoseStamped start;
  geometry_msgs::PoseStamped middle;
  geometry_msgs::PoseStamped end;
  {
    std::lock_guard<std::mutex> lock(obj_tra_mtx);
    auto len = obj_tra.size();
    start = obj_tra[0];
    middle = obj_tra[int(len/2)];
    end = obj_tra.back();
  }
  ROS_INFO("start P(%f,%f),end(%f,%f)",
  start.pose.position.x,start.pose.position.y,end.pose.position.x,end.pose.position.y
  );
  auto dx = end.pose.position.x - start.pose.position.x;
  auto dy = end.pose.position.y - start.pose.position.y;
  double begein_end_d = std::sqrt(dx*dx+dy*dy);
  ROS_INFO("begein_end_d: %f",begein_end_d);
  if(begein_end_d < 0.05)
  {//目标没动
    return 2*M_PI;
  }

  double start_end_yaw = atan2(dy, dx); //先粗略判断目标移动方向
  double middle_end_yaw = atan2(end.pose.position.y - middle.pose.position.y, end.pose.position.x - middle.pose.position.x);
  auto ret = fabs(start_end_yaw) - fabs(middle_end_yaw);
  ROS_INFO("start_end_yaw :%f ,  middle_end_yaw: %f",start_end_yaw, middle_end_yaw);
  if(fabs(ret) < 0.3)
  {//如果差值很小, 表示，目标直线行驶
    ROS_INFO("=============: %f",start_end_yaw);
    return start_end_yaw;
  }

  // if (hit_method != Hit::Method::direct)
  // { // 当单独撞击目标时，不用进入这里，这里仅仅是满足二十七届机器人比赛需求，要满足小车从P1->P2->P1这样一个流程后才能去刺
  //       // if (!isHit_enable)
  //       // {
  //       //   // JudgeHitEnable();
  //       // }
  //       // ROS_INFO("end hover");
  //   if(*yaw < 0)
  //   {//目标从P1到P2，认为小车已经做完一个流程,所以如果使用比赛规则，需要确保飞机小车已经走完一个流程
  //       if (isHit_enable)
  //       {
  //         // 当可以攻击的时候，才结束悬停, 如果使用路径规划去避障，需从planner node 里面去控制
  //         nh->setParam("stop_hover_port", true); // 结束悬停
  //         ROS_INFO("end hover");
  //       }
  //       isDetect = true;
  //   }
  // }
  ROS_INFO("the object turn ");
 return FLT_MAX;
}

void HitDirect::VisualTest(const geometry_msgs::PoseStamped &point)
{

    visualization_msgs::Marker marker;
    // 设置marker的基本属性
    marker.header.frame_id = "map";  // 参考坐标系
    marker.ns = "dot";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    
    // 设置圆点位置
    marker.pose= point.pose;
    
    // 设置圆点大小
    marker.scale.x = 0.1;  // 直径
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    
    // 设置圆点颜色 (红色)
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;  // 不透明度
    
    marker.lifetime = ros::Duration();  // 0表示永远存在

    
  marker.header.stamp = ros::Time::now();
  obj_vis_pub.publish(marker);
  ros::spinOnce();
}

void HitDirect::JudgeHitEnable()
{                                               // 当小车在图像的轨迹从图像底部到顶部的行径方向的时候，任务可以攻击了。因此当飞机飞到P1的，小车应该能走完半段路程，因此，这里需要保证小车有一定的速度
    static std::queue<common_msgs::Obj> pre_objs; // 用来存储一段时间目标
    if (pre_objs.size() < len_sequence_objs)
    {//因为这个频率执行的很快，满足一段时间，或者移动一段距离的目标才能被存储
      pre_objs.push(obj);
    }
    else
    { // 这里判断简单，就拿第一个和最后一个做差值即可,如果出现误判，适当增大len_sequenece_objs的值
      pre_objs.push(obj);
      common_msgs::Obj f = pre_objs.front();
      common_msgs::Obj b = pre_objs.back();
      if (f.center_y - b.center_y > 0)
      {//在视觉上，目标向反向飞机的方向行驶
        isHit_enable = true;
        ROS_INFO("enable Hit");
        return;
      }
      pre_objs.pop();
    }
}

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<HitDirect>("HitDirect");
}
