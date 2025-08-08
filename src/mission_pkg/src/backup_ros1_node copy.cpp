/*
 *
 *
 *mavros 控制需要注意，仿真与当前mavros存在坐标系差异，详细请阅读视觉API文档，关于MAVROS使用问题；
 *
 * 该例程使用例位置，速度，加速度，姿态控制，每个单独控制。位置，速度，加速这三者可以单独控制，
 * 也可以组合控制，组合控制时，使用的是矢量叠加，比如选择位置和速度控制，会按照位置对时间（t）求导，然后再与速度控制部分做矢量和
 *
 *
 *特别注意，如果使用姿态控制时，不要再与位置、速度、加速度混合控制
 *
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/PointStamped.h>

mavros_msgs::State current_state;
geometry_msgs::PoseStamped fcu_pose;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void PoseCB(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  fcu_pose = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/mavros/state", 10, state_cb);
    ros::Publisher cmd_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("/mavros/setpoint_raw/local", 10); //
    ros::Subscriber pose_usb = nh.subscribe("/mavros/local_position/pose",10,PoseCB);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/mavros/set_mode");
    ros::Publisher att_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude",10);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        //判断mavros`是否连接上飞控
        ros::spinOnce();
        rate.sleep();
    }


    mavros_msgs::PositionTarget cmd;
    //是否需要控制角度与角速度，如果需要放开注释
//    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_YAW;
//    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED; // 选择控制坐标系，位置，速度，加速度使用local坐标系，姿态使用的是body坐标系
    cmd.type_mask = ~uint16_t(0);
    cmd.type_mask &= ~mavros_msgs::PositionTarget::FORCE; //px4 不响应力的控制方式
    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PX;
    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PY;
    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PZ;
    cmd.position.x = 0;
    cmd.position.y = 0;
    cmd.position.z = 20;
    cmd.header.stamp = ros::Time::now();

    mavros_msgs::AttitudeTarget att_thrust;
    att_thrust.type_mask = ~uint8_t(0);

    for(int i = 100; ros::ok() && i > 0; --i){
        //发送目标指令，以便飞控切换offboard状态，这里可以发位置，速度，加速度，一般情况起飞，都是发送位置
        cmd_pub.publish(cmd);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    bool is_takeoff= false;
    bool is_attitude = false;
    while(ros::ok()){
        //切换offboard模式
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)) && !is_takeoff){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else if(!is_takeoff) {
            //解锁飞控
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0)) ){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        if(abs(fcu_pose.pose.position.z - 10.) < 0.1 && !is_takeoff)
        {
            ROS_INFO("takeoff finished");
            last_request = ros::Time::now();
            is_takeoff = true;
        }
        if(!is_takeoff)
        {//起飞就是位置控制
            cmd_pub.publish(cmd);
            ros::spinOnce();
            rate.sleep();
            continue;
        }
        if(ros::Time::now() - last_request < ros::Duration(10))
        {//速度控制，
            ROS_INFO("velocity control.....");
            cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            cmd.type_mask = ~uint16_t(0);
            cmd.type_mask &= ~mavros_msgs::PositionTarget::FORCE;
            cmd.velocity.x = 10;
            cmd.velocity.y = 0;
            cmd.velocity.z = 0;
            cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_VX;
            cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_VY;
            cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_VZ;
            cmd_pub.publish(cmd);
        }
        else if(ros::Time::now() - last_request < ros::Duration(20))
        { //加速度控制
            ROS_INFO("accelerate control.....");
            cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            cmd.type_mask = ~uint16_t(0);
            cmd.type_mask &= ~mavros_msgs::PositionTarget::FORCE;
            cmd.acceleration_or_force.x = -4;
            cmd.acceleration_or_force.y = 0; //@
            cmd.acceleration_or_force.z = 10;
            cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_AFX;
            cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_AFY;
            cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_AFZ;
            cmd_pub.publish(cmd);

        }
//        else if (ros::Time::now() - last_request < ros::Duration(30))
        else
        {//通常情况下，实际上姿态控制是不会和加速度，速度，位置控制混合使用的。
            static auto time_r = ros::Time::now();
            if(ros::Time::now() - time_r < ros::Duration(5))
            { //这里的操作仅仅是因为从角速度控制突然切换到姿态控制，会导致不可预测的问题，所以得先让飞机悬停后，再去做姿态控制。
                ROS_INFO("change attitude control.....");
                cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
                cmd.type_mask = ~uint16_t(0);
                cmd.type_mask &= ~mavros_msgs::PositionTarget::FORCE;
                cmd.acceleration_or_force.x = 0;
                cmd.acceleration_or_force.y = 0;
                cmd.acceleration_or_force.z = 0;
                cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_AFX;
                cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_AFY;
                cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_AFZ;
                cmd_pub.publish(cmd);
            }
            ROS_INFO("attitude control.....");
             att_thrust.type_mask = 0;
//             att_thrust.type_mask &= ~mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
//             att_thrust.type_mask &= ~mavros_msgs::AttitudeTarget::IGNORE_THRUST;
             att_thrust.type_mask |= mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE;
             att_thrust.type_mask |= mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE;
             att_thrust.type_mask |= mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
             att_thrust.thrust = 0.8f;  //一般情况，0.6的油门值，飞机悬停状态
             att_thrust.orientation.w = 0.991;
             att_thrust.orientation.x = 0.131;
             att_thrust.orientation.y = 0.;
             att_thrust.orientation.z = 0.;
             att_pub.publish(att_thrust);

        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
