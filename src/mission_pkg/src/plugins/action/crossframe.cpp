#include "plugins/action/crossframe.h"

CrossFrame::CrossFrame(const std::string &name,
                       const BT::NodeConfiguration &config)
    : StatefulActionNode(name, config)
{
  mct = MavRosConnect::getInstance();
  nh = mct->getROSHandle();



  std::string planer_goal_topic;
  nh->param<std::string>("planer_goal_topic", planer_goal_topic,
                         "/move_base_goal");
  planer_goal_pub =
      nh->advertise<geometry_msgs::PoseStamped>(planer_goal_topic, 10);

  nh->param("kx", kx, 0.0);
  nh->param("ky", ky, 0.0);
  nh->param("kz", kz, 0.0);
  nh->param("v_max", v_max, 0.0); // 只是用最大速度，然后分解到x,y，z方向
  //  nh->param("vy_max", vy_max, 0.0);
  //  nh->param("vz_max", vz_max, 0.0);
  //  nh->param("f_rgb", f_rgb, 0.0);
  nh->param("img_w", rgb_image_w, 640);
  nh->param("img_h", rgb_image_h, 480);

  nh->param("fx", fx, 0.0);
  nh->param("fy", fy, 0.0);
  nh->param("rgb_ppx", rgb_ppx, 0);       // 即我们标定的cx
  nh->param("rgb_ppy", rgb_ppy, 0);       // 即我们标定cy 参数
  nh->param("rgb_fov_h", rgb_fov_h, 0.0); //
  nh->param("rgb_fov_v", rgb_fov_v, 0.0); //
  nh->param("min_score", min_score, 0.5);

  nh->param<std::vector<double>>("cam2body_R", cam2body_R,
                                 std::vector<double>());
  nh->param<std::vector<double>>("cam2body_T", cam2body_T,
                                 std::vector<double>());

  fcu_pose_ptr = mct->getFcuPose();
  fcu_state_ptr = mct->getFcuState();
  tgt_pose_pub_ptr = mct->getPosTgtPublier();

  cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  cmd.type_mask = 0xfff;
  cmd.type_mask &=
      ~mavros_msgs::PositionTarget::FORCE; // px4 不响应力的控制方式
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_VX;
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_VY;
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_VZ;
  //  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNOREYAW;
  cmd.header.frame_id = "corssframe";
  obj_name = "";
  has_recv_obj = false;
}

void CrossFrame::RecvObj(const common_msgs::Objects::ConstPtr &msg)
{ // 看具体的控制方式，如果是视觉伺服控制，直接拿结果做计算，如果是使用规划器，则需计算目标的世界坐标系坐标
  //   objs = *msg;
  if (msg->objects.size() == 0)
  {
    ROS_INFO("obj size: %d", msg->objects.size());
    ROS_WARN("net get object or not init,please wait ....[in cross frame]");
    return;
  }
  size_t idx = -1;
  double score = -1;
  for (size_t i = 0; i < msg->objects.size(); ++i)
  {
    if (msg->objects[i].class_name != obj_name)
    {
      continue;
    }
    else
    {
      if (msg->objects[i].score > score)
      {
        idx = i;
        score = msg->objects[i].score;
      }
    }
  }

  if (idx < 0 || score < min_score)
  {
    idx = -1;
    return;
  }

  auto w = msg->objects[idx].right_bottom_x - msg->objects[idx].left_top_x;
  auto h = msg->objects[idx].right_bottom_y - msg->objects[idx].left_top_y;
  auto scale = float(w) / h;

  if (scale < 0.7)
  { // 没有识别到全框，
    ROS_WARN("has detect objects, but not complete");
    return;
  }
  std::unique_lock<std::mutex> lock(mtx, std::try_to_lock);
  if (lock.owns_lock())
  {
    objs.objects.push_back(msg->objects[idx]);
    obj = msg->objects[idx];
    has_recv_obj = true;
  }
}

PortsList CrossFrame::providedPorts()
{
  return {InputPort("object_name", "we need cross frame name"),
          InputPort("ctrl_type", "0: just use Visual servo; 1: use planner"),
          InputPort("ctrl_speed", "set corss frame speed")

  };
}

// NodeStatus CrossFrame::tick()
//{

//  if(fcu_state_ptra->mode != "OFFBOARD" || !fcu_state_ptr->armed)
//  {  //如果飞机状态不对，直接结束节点并返回失败
//    return NodeStatus::FAILURE;
//  }

//  ;
//}

NodeStatus CrossFrame::onStart()
{ // 首先确保飞机状态是真确的
  std::cout << "============cross frame ==========" << std::endl;
  if (fcu_state_ptr->mode != "OFFBOARD" || !fcu_state_ptr->armed)
    return NodeStatus::FAILURE;
  auto object_name_port = getInput<std::string>("object_name");
  if (!object_name_port)
  {
    throw RuntimeError("error reading prot[object_name]", object_name_port.error());
  }
  obj_name = object_name_port.value();
  ROS_INFO("we need to cross %s", obj_name.c_str());
  std::string det_topic;
  nh->param<std::string>("detect_resutl_topic", det_topic, "/objects");
  detect_sub = nh->subscribe<common_msgs::Objects>(
      det_topic, 1, bind(&CrossFrame::RecvObj, this, _1));
  return NodeStatus::RUNNING;

  ;
}

NodeStatus CrossFrame::onRunning()
{
  auto ctrlType_port = getInput<int>("ctrl_type");
  if (!ctrlType_port)
  {
    throw RuntimeError("error reading port [ctrl_type]:",
                       ctrlType_port.error());
  }

  int ctrl_type = ctrlType_port.value();

  if (!has_recv_obj)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    return NodeStatus::RUNNING;
  }

  bool stop_hover = false;
  nh->getParam("stop_hover_port", stop_hover);
  if (!stop_hover)
  {
    nh->setParam("stop_hover_port", true);
    return NodeStatus::RUNNING;
  }
  setOutput("velocity_stop",
            true); // 通过黑板设置参数,这个参数在速度控制飞机节点里面使用
  //  setOutput("stop_hover", true);
  //  std::cout << "emit stop hover" << std::endl;

  static common_msgs::Obj obj_;
  {
    std::unique_lock<std::mutex> lock(mtx, std::try_to_lock);
    obj_ = obj;
  }
  if (ctrl_type == 0)
  {
    // 直接根据视觉伺服控制， 需要获得PID控制参数
    /*因为涉及到，x,z两个方向的速度，而且这两个速度不是独立，
  因此需要使用合成速度，当z方向速度增大时，x方向速度就减小，反之亦然*/

    // 当目标框占据图像大小一定比例时，这个时候飞机保持保持飞一段距离
    static double img_area = rgb_image_h * rgb_image_w;
    auto obj_area = double(obj_.right_bottom_x - obj_.left_top_x) * (obj_.right_bottom_y - obj_.left_top_y);
    auto scale = obj_area / img_area;
    static bool direct_fly = false;
    static Position3D _goal = Position3D(0, 0, 0);
    std::cout << "aream scale " << scale << std::endl;
    if (scale > 0.6 && !direct_fly)
    {
      // 当目标框占据图像大小超过该值是，不响应目标检测，以设置的最大速度向前飞，然后计算穿过获取的目标点
      direct_fly = true;
      _goal.x = fcu_pose_ptr->pose.position.x + 1.3 * fx / rgb_image_h + 0.2;
      ROS_DEBUG("frame_goal: %f", _goal.x);
      // 此时的目标的位置不应再更新了
    }
    else if (!direct_fly)
    { // 小于改值时，才会更新目标位置
      _goal = ObjConterRGB(obj_);
    }
    // 仅采用P控制
    //       Position3D cnt;
    //       cnt.x   = (obj_.right_bottom_x + obj_.left_top_x) / 2;
    //       cnt.y   = (obj_.right_bottom_y + obj_.left_top_y) / 2;
    std::cout << "img_w: " << rgb_image_w << " obj cnt.x: " << obj_.center_x
              << std::endl;
    std::cout << "img_h: " << rgb_image_h << " obj cnt.y: " << obj_.center_y
              << std::endl;
    auto ex = rgb_image_w / 2. - obj_.center_x;
    auto ey = rgb_image_h / 2. - obj_.center_y;

    auto vy = ky * ex;
    auto vz = kz * ey;
    auto vx = v_max - abs(std::cos(M_PI_4) * vy) - abs(std::cos(M_PI_4) * vz);
    if (vx < 0)
      vx = 0;

    if (direct_fly)
    {
      vx = v_max;
      vy = 0;
      vz = 0;
    }

    if (fcu_pose_ptr->pose.position.x > _goal.x)
    { // 当前节点的使命已经完成
      std::cout << "goal: " << _goal << std::endl;
      std::cout << "fcu_pose: x " << fcu_pose_ptr->pose.position.x << std::endl;
      //      setOutput("stop_detect", true);
      nh->setParam("stop_detect_port", true);
      nh->setParam("hover_is_end_port", true); // 悬停节点退出信号
      return NodeStatus::SUCCESS;
    }

    cmd.velocity.x = vx;
    cmd.velocity.y = vy;
    cmd.velocity.z = vz;
    cmd.yaw = 0; // 保证机头一致朝前
    std::cout << "dx: " << ex << ", dy: " << ey << std::endl;
    ROS_INFO("vx: %f, vy: %f, vz: %f", vx, vy, vz);
    ROS_INFO("goal: %f, %f, %f", _goal.x, _goal.y, _goal.z);
    ROS_INFO("fcu_pose: %f, %f, %f", fcu_pose_ptr->pose.position.x,
             fcu_pose_ptr->pose.position.y, fcu_pose_ptr->pose.position.z);
    tgt_pose_pub_ptr->publish(cmd);
    ros::spinOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    return NodeStatus::RUNNING;
    // 考虑可能因为气流的干扰导致偏航角出现偏转，导致ey数值偏大，不是位置偏离而是偏航角的问题
  }
  else if (ctrl_type == 1)
  {
    
    cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    cmd.type_mask = 0xfff;
    cmd.type_mask &= ~mavros_msgs::PositionTarget::FORCE; // px4 不响应力的控制方式
    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PX;
    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PY;
    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PZ;
    // 把目标点发给规划器去控制，需要计算三维世界目标点

    Position3D obj_goal = ObjConterRGB(obj_);
    if (abs(obj_goal.x) < FLT_MIN && abs(obj_goal.y) < FLT_MIN && abs(obj_goal.z) < FLT_MIN)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      NodeStatus::RUNNING;
    }
    if (abs(fcu_pose_ptr->pose.position.x - obj_goal.x) < 0.2 && abs(fcu_pose_ptr->pose.position.y - obj_goal.y) < 0.2 && abs(fcu_pose_ptr->pose.position.z - obj_goal.z) < 0.2)
    {
      //      setOutput("stop_detect", true);  //已经穿过框，发送停止检测信号
      nh->setParam("stop_detect_port", true);
      nh->setParam("hover_is_end_port", true); // 悬停节点退出信号
      detect_sub.shutdown();
      return NodeStatus::SUCCESS;
    }
    static bool send_flag = false;
    if (!send_flag)
    {
      ROS_DEBUG("object goal(%f,%f,%f)", obj_goal.x, obj_goal.y, obj_goal.z);
      geometry_msgs::PoseStamped msg;
      msg.header.stamp = ros::Time::now();
      msg.pose.position.x = obj_goal.x;
      msg.pose.position.y = obj_goal.y;
      msg.pose.position.z = obj_goal.z;
      planer_goal_pub.publish(msg);
      ros::spinOnce();
      setOutput("stop_hover", true); // 取消飞机悬停动作
    }
    return NodeStatus::
        RUNNING; // 此时还不能返回成功，只有等完全穿过去之后才能返回成功，但是这里的目标点也只能发一次，
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  return NodeStatus::RUNNING;
  ;
}

void CrossFrame::onHalted()
{
  std::cout << "the cross frame node is end" << std::endl;
}

Position3D CrossFrame::ObjConterRGB(const common_msgs::Obj &obj)
{
  // 该接口目标中心计算比较粗糙，直接使用目标检测框（这就要求标注的时候尽可能的贴着边缘标定），严格上来讲，应该对目标款内图像做角点检测，然后筛选出目标的实际目标点位置

  static double scale_ = 1;

  if (obj.score < min_score)
    return Position3D(0, 0, 0);
  auto width = obj.right_bottom_x - obj.left_top_x;
  auto height = obj.right_bottom_y - obj.left_top_y;

  // 计算传感器的宽度和高度，unit:mm
  static double s_w = 2 * fx * std::tan(rgb_fov_h / 2);
  static double s_h = 2 * fy * std::tan(rgb_fov_v / 2);
  static double pw = s_w / rgb_image_w; // 像素的宽度
  static double ph = s_h / rgb_image_h; // 像素的高度
  static double wf = 1.3 * fx;

  double z = wf / width;

  double cx = (obj.right_bottom_x + obj.left_top_x) / 2;
  double cy = (obj.right_bottom_y + obj.left_top_y) / 2;
  double xc = (cx - int(rgb_image_h / 2)) * pw;
  double yc = (cy - int(rgb_image_w / 2)) * ph;
  double x = xc * z / fx;
  double y = yc * z / fy;
  // 至此，一致计算得到目标宽中心位置在相机坐标系里的位置了；
  // 需要这是相机系下坐标到机体坐标系的坐标
  double x_ =
      x * cam2body_R[0] + y * cam2body_R[1] + z * cam2body_R[2] + cam2body_T[0];
  double y_ =
      x * cam2body_R[3] + y * cam2body_R[4] + z * cam2body_R[5] + cam2body_T[1];
  double z_ =
      x * cam2body_R[6] + y * cam2body_R[7] + z * cam2body_R[8] + cam2body_T[2];
  tf2::Quaternion q;
  //@ todo
  //  return Position3D(z, -y, x);

  // 这里获取到坐标系如果有差异就会出问题，
  tf2::convert(fcu_pose_ptr->pose.orientation, q);
  tf2::Vector3 t;
  tf2::convert(fcu_pose_ptr->pose.position, t);

  tf2::Transform trans;
  trans.setOrigin(t);
  trans.setRotation(q);

  tf2::Vector3 b_p(x_, y_, z_);
  tf2::Vector3 w_p = trans * b_p;
  return Position3D(w_p.x(), w_p.y(), w_p.z());
}

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<CrossFrame>("CrossFrame");
}
