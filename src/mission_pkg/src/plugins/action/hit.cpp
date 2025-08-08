#include "plugins/action/hit.h"

Hit::Hit(const std::string &name, const NodeConfig &config)
    : StatefulActionNode(name, config)
{
  mct = MavRosConnect::getInstance();
  // 需要根据当前飞机位姿，计算目标在全局坐标系下的位置
  fcu_pose_ptr = mct->getFcuPose();
  fcu_state_ptr = mct->getFcuState();
  tgt_pose_pub_ptr = mct->getPosTgtPublier();
  nh = mct->getROSHandle();

  // nh->param<std::string>("det_topic", det_topic, "/objects");
  nh->param<int>("img_w", rgb_image_w, 640);
  nh->param<int>("img_h", rgb_image_h, 480);
  nh->param<double>("hight_min", hight_min, 0.5);
  nh->param<double>("hight_max", hight_max, 1.0);
  nh->param<double>("obs_dist", obs_dist, 0.5);
  nh->param<double>("v_max", v_max, 2);
  nh->param<double>("v_min",v_min,0);
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
  is_innode = false;
  nh->param<std::string>("depth_topic", depth_topic, "/rflysim/sensor2/img_depth");
  // nh->param("odom_topic",odom_topic,"/mavros/local_position/odom");
  nh->param<std::string>("obs_cloud_topic", obs_cloud_topic, "/grid_map/occupancy_inflate");
  nh->param<std::string>("planner_cmd_topic", pos_cmd_topic, "/planning/pos_cmd");


  nh->param("kx", kx, 0.0);
  nh->param("ky", ky, 0.0); 
  nh->param("kz", kz, 0.0);
  nh->param("kyaw_rate", k_yaw, 0.0);
  nh->param("pose_len",pose_len,100);


  
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

  cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  cmd.type_mask = 0xfff;
  cmd.type_mask &=
      ~mavros_msgs::PositionTarget::FORCE; // px4 不响应力的控制方式
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PX;
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PY;
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PZ;
  // cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_YAW;
  cmd.header.frame_id = "Hit";
  ;
}

PortsList Hit::providedPorts()
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

NodeStatus Hit::onStart()
{
  ROS_INFO("in hit node");
  // return NodeStatus::RUNNING;
   
  if (fcu_state_ptr->mode != "OFFBOARD" || !fcu_state_ptr->armed)
  {
    ROS_ERROR("current fcu state cann't to fly");
    return NodeStatus::FAILURE;
  }
  nh->param<std::string>("det_topic", det_topic, "/objects");
  det_sub = nh->subscribe<common_msgs::Objects>(det_topic, 10,
                                                bind(&Hit::RecvObj, this, _1));
  std::string depth_topic;
  is_vis_servo_end = false;
  nh->param<std::string>("depth_topic", depth_topic, "/rflysim/sensor2/img_depth");
  // obs_sub = nh->subscribe<sensor_msgs::PointCloud2>(obs_cloud_topic, 10, boost::bind(&Hit::RecvPointCloud, this, _1));
  depth_sub = nh->subscribe<sensor_msgs::Image>(depth_topic, 1, boost::bind(&Hit::DepthCallback, this, _1));

  obj_vis_pub = nh->advertise<visualization_msgs::Marker>("object_vis", 10);

  goal_pub = nh->advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
  //创建一个定时去存储一段时间的位姿数据
  pose_timer = nh->createTimer(ros::Duration(0.03), boost::bind(&Hit::timerCallback,this,_1));

  is_innode  = true;
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
  hit_method = static_cast<Hit::Method>(hit_method_port.value());

  if (hit_method == Hit::Method::direct)
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

NodeStatus Hit::onRunning()
{ // 这个节点结束条件是气球破裂，但是在程序里面不判断气球破裂的条件，
  // 因此这里不会返回成功，如果刺破了气球，人接手飞机控制权,否则任务目标丢失，等待再一次刺破动作
  // 首先确保飞机状态是真确的、

  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 可适当降低执行频率，来降低计算量
  // if (fcu_state_ptr->mode != "OFFBOARD" || !fcu_state_ptr->armed)
  // {
  //   ROS_ERROR("current fcu state cann't to fly");
  //   return NodeStatus::FAILURE;
  // }
  //这里同时用到RGB图和深度图，可能因为深度图噪声的原因，导致目标位置跳动较大，因此，这里需要一个简单的距离阈值滤波使用3到5帧过滤跳变较大的位置，这个代码读者自行优化
  auto ret = getObjPosition(goal);
  ROS_INFO("goal at pub: %f,%f,%f", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
  if(!ret)
  {
    
  }

  VisualTest(goal);
  if(hit_method != Hit::Method::direct)
  {//这里按照比赛的需求来，就是等到小车从P1点往P2点走时认为小车已经走完一个来回，再去刺小球,这里使用的方法是通过一段小车的行径轨迹来判断，但是如小车行走的特别慢，
  //这里判断就有可能任务小车是静止状态，因此需要读者自行去优化。
    auto obj_yaw = PredictObjOri(); //获得目标行径方向

    //@todo youself
    ROS_WARN("you must to implement code ,that to judge the drone can to hit");
    return NodeStatus::RUNNING;
  }
  
  if(!ret && !is_vis_servo_end)
  {//如果没有获取到目标位置,也不是在刺破的末端
    ROS_WARN("In Hit Node, not get object goal - isDetect: %s, depth_buffer size: %lu, obj center: (%d,%d)", 
             isDetect ? "true" : "false", depth_buffer_.size(), obj.center_x, obj.center_y);
    return NodeStatus::RUNNING;
  }
  ROS_INFO("Get object coord: (%f,%f,%f)",goal.pose.position.x,goal.pose.position.y,goal.pose.position.z);
  if((!isDetect  || ros::Time::now() - last_obj_time > ros::Duration(2)) && !is_vis_servo_end)
  { //如果一段时间没有检测到目标
      double time_since_last_obj = (ros::Time::now() - last_obj_time).toSec();
      if(ros::Time::now() - last_obj_time > ros::Duration(2))
      {//较长时间没有检测到目标，清空队列
        ROS_WARN("haven't recv object for %.1f seconds, last_obj_time: %d.%09d", 
                 time_since_last_obj, last_obj_time.sec, last_obj_time.nsec);
        std::deque<geometry_msgs::PoseStamped>().swap(obj_tra);
        std::queue<common_msgs::Obj>().swap(objs_queue);
        isDetect = false;
      }
      bool stop_hover;
      nh->getParam("stop_hover_port", stop_hover);;
      ROS_WARN("lost object for %.1f seconds, keep hover [in hit.cpp at Running]", time_since_last_obj);
      nh->setParam("is_pause_planner", true); //暂停规划器节点
      nh->setParam("stop_hover_port", false);
      // isHit_enable = false;
      return NodeStatus::RUNNING;
  }


  /*使用避障的方式逼近，然后近距离是使用视觉伺服控制*/
  if (abs(goal.pose.position.x) < FLT_MIN || abs(goal.pose.position.y) < FLT_MIN || abs(goal.pose.position.z) < FLT_MIN)
  { // 如果能计算到目标的全局系下的坐标，那就表明目标以及被检查了
    ROS_WARN("the goal is not settings");
    nh->setParam("stop_hover_port", false);
    nh->setParam("is_pause_planner", true); //暂停规划器节点
    return NodeStatus::RUNNING;
  }
  
  double dist = std::sqrt(std::pow(goal.pose.position.x - fcu_pose_ptr->pose.position.x, 2) + std::pow(goal.pose.position.y - fcu_pose_ptr->pose.position.y, 2) + std::pow(goal.pose.position.z - fcu_pose_ptr->pose.position.z, 2));
  ROS_INFO("current ball distance %f", dist);
  if(hit_method == Hit::Method::direct)
  {
    double dx =  goal.pose.position.x - fcu_pose_ptr->pose.position.x;
    double dy = goal.pose.position.y - fcu_pose_ptr->pose.position.y;
    double dz = goal.pose.position.z - fcu_pose_ptr->pose.position.z;
    
    double length = sqrt(dx * dx + dy * dy + dz * dz);
    
    double unit_x = dx / length;
    double unit_y = dy / length;
    double unit_z = dz / length;

    if(dist > hit_dist && !is_vis_servo_end)
    {
      ROS_INFO("use planner to hit the goal");
      //如果距离目标较远，使用规划器去做避障逼近
      nh->setParam("is_pause_planner",false); //使用规划器去做避障逼近
      geometry_msgs::PoseStamped tmp;
      tmp.pose.position.x = goal.pose.position.x - unit_x * goal_tolerance;
      tmp.pose.position.y = goal.pose.position.y - unit_y * goal_tolerance;
      tmp.header.frame_id = "hit_goal";
      // tmp.pose.position.z = goal.pose.position.z - unit_z * goal_tolerance;
      //考虑到可能靠近目标的时候还未降低高度，会丢失目标；
      tmp.pose.position.z  = goal.pose.position.z;
      
      if(tmp.pose.position.z < 0.2)
      {
        tmp.pose.position.z = 0.2;
      }
          // goal_pub.publish(goal);//先测试能不能实时规划轨迹；
      ROS_INFO("tmp at pub: %f,%f,%f", tmp.pose.position.x, tmp.pose.position.y, tmp.pose.position.z);
      goal_pub.publish(tmp);//先测试能不能实时规划轨迹；
      ros::spinOnce();
    }
    else
    { 
      //计算飞机航向角
      geometry_msgs::Quaternion quat_msg = fcu_pose_ptr->pose.orientation;
      tf2::Quaternion tf_quat;
      tf_quat.setX(quat_msg.x);
      tf_quat.setY(quat_msg.y);
      tf_quat.setZ(quat_msg.z);
      tf_quat.setW(quat_msg.w);
      double roll, pitch, fcu_yaw;
      tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, fcu_yaw);
      nh->setParam("is_pause_planner",true); //将控制权从Planner 节点转接到Hit节点上来
      //计算飞机
      nh->setParam("stop_hover_port",true); //，因为控制权从planner 节点移交过来，所以同时结束悬停
      double tgt_yaw = atan2(dy, dx);
      ROS_INFO("fcu_yaw: %f, tgt_yaw:%f",fcu_yaw,tgt_yaw);
      if(abs(fcu_yaw - tgt_yaw) > 0.1 && !is_vis_servo_end)
      {//如果目标飞机没有指向目标，先调整飞机朝向，有可能会在视觉伺服控制的情况下朝向偏了情况，需要重新调整
        cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        cmd.type_mask = 0xfff;
        cmd.type_mask &= ~mavros_msgs::PositionTarget::FORCE; // px4 不响应力的控制方式
        cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_YAW;
        cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PX;
        cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PY;
        cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PZ;
        cmd.position = fcu_pose_ptr->pose.position;
        cmd.yaw = tgt_yaw;
      }
      else
      {
        ROS_INFO("se visual servo control,just use velocity control");
        cmd.type_mask = 0xfff;
        cmd.type_mask &= ~mavros_msgs::PositionTarget::FORCE; // px4 不响应力的控制方式
        cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_VX;
        cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_VY;
        cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_VZ;
        cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
      // cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

        // cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_YAW;
        // cmd.position.x = goal.pose.position.x + unit_x * goal_tolerance;
        // cmd.position.y = goal.pose.position.y + unit_y * goal_tolerance;
        // //给刺针安装角度等一个适配
        // cmd.position.z = goal.pose.position.z + unit_z * goal_tolerance + needle_adapt;
        // cmd.yaw = tgt_yaw;
        double vx = 0;
        double vy = 0;
        double vz = 0;
        double yaw_rate = 0;
        getVel(vx,vy,vz,yaw_rate); //默认目标检测正常
        //使用视觉伺服去控
        cmd.velocity.x = vx;
        cmd.velocity.y = vy;
        cmd.velocity.z = vz;
      } 
      tgt_pose_pub_ptr->publish(cmd);
      ros::spinOnce();
    }
    return NodeStatus::RUNNING;
  }
  else
  {//@todo youself
    //如不是发现目标就刺的方式，应该在这里处理。比如：小车满足昨晚一个来回再刺；

  }

  return NodeStatus::RUNNING; //如果是直接刺破的方式，直接返回运行状态 , 下面的代码不会运行，仅供优化参考
/*
//如果采取直接刺破的方式不考虑飞机的行径方向，也就是说不管飞机是静止，还是运动，或已经走完一个来回
  ROS_INFO("current_obj yaw: %f",obj_yaw);
  if(abs(obj_yaw - 2*M_PI) < FLT_MIN && hit_method != Hit::Method::direct )
  {//目标静止不动,因为使用的是比赛流程，小车静止时，飞机悬停
    ROS_WARN("current the object is static");
    bool plan_flag = false;
    nh->getParam("is_pause_planner",plan_flag);
    if(!plan_flag)
    {
      nh->setParam("is_pause_planner", true);  //暂停planner node 去控制
      return NodeStatus::RUNNING;  //当次不控制
    }
    else
    {
      ROS_WARN("puase planner node,but Planner Node state not switch");
    }
    return NodeStatus::RUNNING;
  }
  if(abs(obj_yaw) > M_PI && hit_method != Hit::Method::direct)
  { //如果是直接控制的，目标静态下不能进入到这里
    ROS_WARN("not get object heading ,maybe not get obj data");
    return NodeStatus::RUNNING;
  }
*/ 
    //计算飞机朝向，这里为保证刺破的成功率，先获取飞机的朝向，再与获得目标的行径方向比较


    //-pi 与 pi 是同一方向
    // if(abs(fcu_yaw) - abs(obj_yaw) < yaw_err)
    // if(!isHit_enable)
    // {
    //   nh->setParam("is_pause_planner", true);  //暂停planner node 去控制
    //   nh->setParam("stop_hover_port", false); // 保持悬停
    //   return NodeStatus::RUNNING;  
    // }

    // nh->setParam("stop_hover_port", true); // 结束悬停
    // if(abs(obj_yaw) > M_PI_2)
    // { //目标行径方向与飞机朝向一致
    //   if(!isHit_enable)
    //   {
    //     nh->setParam("stop_hover_port", true); // 结束悬停
    //     ROS_INFO("end hover");
    //     return NodeStatus::RUNNING; //当前帧不执行，等待hover节点能读到参数再执行
    //   }
    //   isHit_enable = true;

    // }

  /*下面的逻辑是使用轨迹规划避障，当近距离且目标中心在图像中心区域时，采用直接刺破地方方式*/
  // double dist = std::sqrt(std::pow(goal.pose.position.x - fcu_pose_ptr->pose.position.x, 2) + std::pow(goal.pose.position.y - fcu_pose_ptr->pose.position.y, 2) + std::pow(goal.pose.position.z - fcu_pose_ptr->pose.position.z, 2));
  
  ROS_INFO("current ball distance %f", dist);

  //要预测目标的行径方向，通过一个队列保存目标的轨迹点，然后估算目标的行径方向
  //需要有效轨迹点，也就是说有一段移动距离，有一段时间(静止状态也行)
  if( dist > hit_dist)
  {//这种情况，虽然能够获得气球的位置，但是不能直接取刺破,需要使用路径规划去做避障
    bool plan_flag = true;
    nh->getParam("is_pause_planner",plan_flag);
    if(plan_flag)
    {
      nh->setParam("is_pause_planner", false);  //planner node 去控制
      return NodeStatus::RUNNING;
    }
   
    goal.header.frame_id = "Hit_planner";
    geometry_msgs::PoseStamped tmp;
    
    double dx =  goal.pose.position.x - fcu_pose_ptr->pose.position.x;
    double dy = goal.pose.position.y - fcu_pose_ptr->pose.position.y;
    double dz = goal.pose.position.z - fcu_pose_ptr->pose.position.z;
    
    double length = sqrt(dx * dx + dy * dy + dz * dz);
    
    double unit_x = dx / length;
    double unit_y = dy / length;
    double unit_z = dz / length;
    
    // double cx, cy, cz;

    tmp.pose.position.x = goal.pose.position.x - unit_x * goal_tolerance;
    tmp.pose.position.y = goal.pose.position.y - unit_y * goal_tolerance;
    tmp.pose.position.z = goal.pose.position.z - unit_z * goal_tolerance;
    if(tmp.pose.position.z < needle_adapt)
    {
      tmp.pose.position.z = needle_adapt;
    }

    goal_pub.publish(tmp); //这个发给路径规划的goal需要改一下，因为此时目标点在目标上，不能正常规划轨迹
   
    // goal_pub.publish(goal); 

    ros::spinOnce();
    ROS_INFO("goal at pub: %f,%f,%f", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
    ROS_INFO("current_pose: %f,%f,%f", fcu_pose_ptr->pose.position.x, fcu_pose_ptr->pose.position.y, fcu_pose_ptr->pose.position.z);
    // ROS_INFO("tmp goal at pub: %f,%f,%f", tmp.pose.position.x, tmp.pose.position.y, tmp.pose.position.z);
    return NodeStatus::RUNNING;
    
  }
  else
  { //如果气球位置离得近，且正对飞机刺针，直接飞过去刺破就行
    ROS_INFO("obj_center(%d,%d)",obj.center_x,obj.center_y);
    ROS_INFO("to object distance: %f",dist);
    // if(obj.center_x < left_line || obj.center_x > right_line)
    bool plan_flag = false;
    nh->getParam("is_pause_planner",plan_flag);
    if(!plan_flag)
    {
      nh->setParam("is_pause_planner", true);  //暂停planner node 去控制
      return NodeStatus::RUNNING;  //当次不控制
    }
    else{
      ROS_WARN("use point control in Hit Node,but Planner Node state not switch");
    }
    cmd.header.frame_id = "hit_cmd";
    //此时的目标坐标是气球表面的，因该往里刺
    // cmd.position.x = goal.pose.position.x;
    // cmd.position.y = goal.pose.position.y;
    // cmd.position.z = goal.pose.position.z;
    //为了保证刺破气球，需要尽可能的确保戳点切面与针垂直
    //计算到目标点yaw角。
    // 计算飞机当前飞机的yaw角度
    
    double dx = goal.pose.position.x - fcu_pose_ptr->pose.position.x;
    double dy = goal.pose.position.y - fcu_pose_ptr->pose.position.y;
    double dz = goal.pose.position.z - fcu_pose_ptr->pose.position.z;
    double length = sqrt(dx * dx + dy * dy + dz * dz);
    
    double unit_x = dx / length;
    double unit_y = dy / length;
    double unit_z = dz / length;
    
    // double cx, cy, cz;

    cmd.position.x = goal.pose.position.x + unit_x * goal_tolerance;
    cmd.position.y = goal.pose.position.y + unit_y * goal_tolerance;
    cmd.position.z = goal.pose.position.z + unit_z * goal_tolerance;

   

    // 使用atan2计算yaw角度（弧度）
    double tgt_yaw = atan2(dy, dx); //这是飞机与目标连线的yaw角度，如果这个角度与目标行径方向的角度相差大，就有可能出现飞机刺空的现象，
    /*优化建议
      保持飞机刺球的方向与气球的行径方向在同一直线上(保证刺针正对球心刺)，即可保证刺破，但是要保证避障的前提下，做到这个点并不容易(有可能在调整飞机姿态的时候就发生碰撞) */
      // double tol_yaw  = 0.2; //允许的角度偏差
      // if(abs(obj_yaw) < M_PI && fabs(tgt_yaw - obj_yaw) > tol_yaw)
      // { //这个可以根据目标速度预测目标位置，直接朝预测目标位置刺破，
      //   //这种情况需要计算目标移动速度，同时也需要计算飞机位置控制的速度，因为这时候飞机离目标已经很近了，使用别的方法可能效率更高
         //因此最简单的方法就是通过减小两个角度差值，旋转飞机角度，控制在（obj_yaw 与 tgt_yaw）的差值很小时，等目标行驶到obj_yaw，直接刺
          //保持飞机位置不变，仅仅控制飞机的yaw角度
      // }
    // if(obj_yaw > 0 && hit_method != Hit::Method::direct )
    { //这个时候目标可能还没走完比赛规则里的一个来回

    }
    // if(abs(obj_yaw) < M_PI && fabs(tgt_yaw - obj_yaw) > tol_yaw )
    // {

    // }


    // ROS_INFO("fcu_y  aw:%f, tgt_yaw:%f",fcu_yaw,tgt_yaw);
    cmd.yaw = tgt_yaw;
  
    if(cmd.position.z < needle_adapt)
    {
      cmd.position.z = needle_adapt;
    }
    tgt_pose_pub_ptr->publish(cmd);
  }

  return NodeStatus::RUNNING;
}

void Hit::onHalted()
  {
;
  }
void Hit::CleanValue()
{
    is_innode = false;
    isDetect = false;
}

bool Hit::getObjPosition(geometry_msgs::PoseStamped &goal)
{
  // ROS_INFO("objs_queue size: %d,depth_buffer_.size: %d",objs_queue.size(),depth_buffer_.size());
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
    float cam_x = (obj.center_x - cx)*depth / fx;
    float cam_y = (obj.center_y - cy)*depth / fy;
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
      Eigen::Vector3d t(
        best_pose.pose.position.x,
        best_pose.pose.position.y,
        best_pose.pose.position.z);
      Eigen::Vector3d p_body(
        body_p.pose.position.x,
         body_p.pose.position.y,
         body_p.pose.position.z);

    // 执行变换: R*p_body + t
    Eigen::Vector3d p_local = q * p_body + t;
    
    goal.pose.position.x = p_local[0];
    goal.pose.position.y = p_local[1];
    goal.pose.position.z = p_local[2];
    // ROS_INFO("xxxxgoal: x:%f,y:%f,z:%f",goal.pose.position.x,goal.pose.position.y,goal.pose.position.z);
    }
    return true;
}

void Hit::toEulerAngle(const Eigen::Quaterniond &q, Eigen::Vector3d &rotat)
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

void Hit::DepthObjCallback(const common_msgs::Objects::ConstPtr &objs,const sensor_msgs::Image::ConstPtr &depth)
{
      // 检查时间差是否在允许范围内
    // ROS_ASSERT(image->header.stamp == detection->header.stamp);
  ROS_INFO("obj stmp:%f, depth stmp: %f",objs->header.stamp.toSec(), depth->header.stamp.toSec());

}

void HitgetObjPosition()
{

}

void Hit::timerCallback(const ros::TimerEvent& event)
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

void Hit::DepthCallback(const sensor_msgs::Image::ConstPtr &msg)
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


void Hit::RecvObj(const common_msgs::Objects::ConstPtr &msg)
  { // 这里需要的目标是目标分割后的中心位置，而非检测框的中心位置；

    ROS_DEBUG("recv objsize in hit: %d", msg->objects.size());
    obj_name = "balloon";

    if (msg->objects.empty() || obj_name.empty())
    {
      ROS_DEBUG("No objects received or obj_name empty");
      return;
    }
    
    static int idx = -1;
    static double score = -1.0;

    for (size_t i = 0; i < msg->objects.size(); ++i)
    {
      if (msg->objects[i].class_name != obj_name)
        continue;
      ROS_DEBUG("Found balloon object with score: %f, center: (%d,%d)", 
                msg->objects[i].score, msg->objects[i].center_x, msg->objects[i].center_y);
      if (msg->objects[i].score > score)
      {
        idx = i;
        score = msg->objects[i].score;
      }
    };
    
    if (idx < 0 || msg->objects[idx].score < 0.5 || msg->objects[idx].class_name != obj_name)
    { // 置信度太低了，
      ROS_WARN("Object score too low (%f) or no valid balloon detected", 
               idx >= 0 ? msg->objects[idx].score : -1.0);
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
      ROS_INFO("Successfully received balloon object: score=%.2f, center=(%d,%d), time=%d.%09d", 
               obj.score, obj.center_x, obj.center_y, obj.header.stamp.sec, obj.header.stamp.nsec);
      // objs_queue.push(obj);
      if(is_innode && !isDetect)
      {
        isDetect = true;
        ROS_INFO("First balloon detection in Hit node");
      }
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

  void Hit::RecvPointCloud(const sensor_msgs::PointCloud2::ConstPtr &msg)
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

  bool Hit::CalFlyOri(double &vx, double &vy, double &vz, double &yaw_rate)
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

double Hit::PredictObjOri()
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

void Hit::VisualTest(const geometry_msgs::PoseStamped &point)
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
    marker.scale.x = 0.8;  // 直径
    marker.scale.y = 0.8;
    marker.scale.z = 0.8;
    
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

bool Hit::getVel(double &vx, double &vy, double &vz,double &yaw_rate)
{
  if(is_vis_servo_end) //以及在刺破的末端
  { // 直接飞过去
    vx = v_max;
    vy = 0;
    vz = 0;
    yaw_rate = 0;
    return true;
  }
  double area_rate = 0.0; //识别面积占比
  static double img_size = rgb_image_w * rgb_image_h; //图像面积
  area_rate = (obj.right_bottom_x - obj.left_top_x) * (obj.right_bottom_y - obj.left_top_y) / img_size; // 目标面积占图像面积的比例
  ROS_INFO("area_rate: %f", area_rate);
  if(area_rate < 0.01)
  { // 目标面积太小，认为目标不在图像中
    ROS_WARN("the object area is too small,area_rate: %f",area_rate);
    return false;
  }
  // if(area_rate > 0.1) // 目标面积占图像面积的比例大于这个值，直接刺
  // { // 目标面积太大，认为目标不在图像中
  //   vx = v_max;
  //   vy = 0;
  //   vz = 0;
  //   yaw_rate = 0;
  //   ROS_INFO("the object area is too large,area_rate: %f",area_rate);
  // }
  // 用与P控制
  if(obj.center_x < 1 || obj.center_y < 1)
  {
    return false;
  }
  int ex = 0;
  int ey = 0;
  ex = obj.center_x - int(rgb_image_w / 2); // 控制y方向速度
  ey = obj.center_y - int(rgb_image_h / 2); // 控制z方向速度
  std::cout << "ex: " << ex << " ey: " << ey << " ar " << area_rate
                << std::endl;

    // 因为随着目标的靠近，目标中心在图像
    //   auto vy = ky * ex;  //y方向不控制速度，使用 控制偏航角
    vz = -kz * ey;
    vy = ky * ex;
    vx = v_max - abs(vy) -abs(vz);
    if (vx < 0)
    {
      vx = v_min;
    }
    vz = abs(vz) > v_max ? vz/abs(vz) * v_max : vz;
    vy = abs(vy) > v_max ? vy/abs(vy) * v_max : vy;
    vx = vx > v_max ? v_max : vx;

    if(ex < 20 && ey < 20 )
    {
      vz = 0;
      vy = 0;
      vx = v_max;
      if(area_rate > 0.1)
      { // 目标面积占图像面积的比例大于这个值，同时也正对目标了
        is_vis_servo_end = true; //这是时候已经在刺破的末端
        ROS_INFO("the object area is too large,area_rate: %f",area_rate);
      }
    }
    ROS_INFO("ex:%d,ey:%d", ex,ey);
    ROS_INFO("vx:%f,vy:%f,vz:%f,yaw_rate:%f", vx,vy, vz, yaw_rate);
    return true;

}

void Hit::JudgeHitEnable()
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
  factory.registerNodeType<Hit>("Hit");
}
