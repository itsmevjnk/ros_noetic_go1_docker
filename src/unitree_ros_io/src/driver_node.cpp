#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include <unitree_legged_sdk/unitree_legged_sdk.h>
#include "convert.h"
#include <chrono>
#include <pthread.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

using namespace UNITREE_LEGGED_SDK;
class Custom
{
public:
    UDP low_udp;
    UDP high_udp;

    HighCmd high_cmd = {0};
    HighState high_state = {0};

    LowCmd low_cmd = {0};
    LowState low_state = {0};

public:
    Custom()
        : 
        // low_udp(LOWLEVEL),
        low_udp(LOWLEVEL, 8091, "192.168.123.10", 8007),
        high_udp(8090, "192.168.12.1", 8082, sizeof(HighCmd), sizeof(HighState)) // 192.168.123.161 if running on robot, 192.168.12.1 otherwise
    {
        high_udp.InitCmdData(high_cmd);
        low_udp.InitCmdData(low_cmd);

        /* make sure robot stands up */
        high_cmd.mode = 6;
        high_cmd.gaitType = 1;
    }

    void highUdpSend()
    {
        // printf("high udp send is running\n");

        high_udp.SetSend(high_cmd);
        high_udp.Send();
    }

    void lowUdpSend()
    {

        low_udp.SetSend(low_cmd);
        low_udp.Send();
    }

    void lowUdpRecv()
    {
        low_udp.Recv();
        low_udp.GetRecv(low_state);
    }

    void highUdpRecv()
    {
        // printf("high udp recv is running\n");

        high_udp.Recv();
        high_udp.GetRecv(high_state);
    }
};

Custom custom;

ros::Subscriber sub_cmd_vel;
ros::Publisher pub_high;

long cmd_vel_count = 0;

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    // printf("cmdVelCallback is running!\t%ld\n", cmd_vel_count);

    custom.high_cmd = rosMsg2Cmd(msg);

    ROS_INFO("vx=%f, vy=%f, rot=%f", custom.high_cmd.velocity[0], custom.high_cmd.velocity[1], custom.high_cmd.yawSpeed);

    // printf("cmd_x_vel = %f\n", custom.high_cmd.velocity[0]);
    // printf("cmd_y_vel = %f\n", custom.high_cmd.velocity[1]);
    // printf("cmd_yaw_vel = %f\n", custom.high_cmd.yawSpeed);

    // printf("cmdVelCallback ending!\t%ld\n\n", cmd_vel_count++);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "twist_sub");

    ros::NodeHandle nh;

    sub_cmd_vel = nh.subscribe("cmd_vel", 1, cmdVelCallback);

    LoopFunc loop_udpSend("high_udp_send", 0.002, 3, boost::bind(&Custom::highUdpSend, &custom));
    LoopFunc loop_udpRecv("high_udp_recv", 0.002, 3, boost::bind(&Custom::highUdpRecv, &custom));

    loop_udpSend.start();
    loop_udpRecv.start();

	ros::Publisher pub_imu = nh.advertise<sensor_msgs::Imu>("imu", 1);
	ros::Publisher pub_odom = nh.advertise<nav_msgs::Odometry>("odom", 1);
	tf::TransformBroadcaster odom_broadcaster;

    ROS_INFO("node is up");

    /* adapted from https://github.com/aatb-ch/go1_republisher/blob/main/src/imu_odom.cpp - MIT licensed */
    ros::Rate loop_rate(100);
    long count = 0;
    while (ros::ok())
    {
        auto current_time = ros::Time::now();

        sensor_msgs::Imu msg_imu;
        msg_imu.header.seq = count;
        msg_imu.header.stamp = current_time;
        msg_imu.header.frame_id = "odom";
        msg_imu.orientation.w = custom.high_state.imu.quaternion[0];
        msg_imu.orientation.x = custom.high_state.imu.quaternion[1];
        msg_imu.orientation.y = custom.high_state.imu.quaternion[2];
        msg_imu.orientation.z = custom.high_state.imu.quaternion[3];
        msg_imu.angular_velocity.x = custom.high_state.imu.gyroscope[0];
        msg_imu.angular_velocity.y = custom.high_state.imu.gyroscope[1];
        msg_imu.angular_velocity.z = custom.high_state.imu.gyroscope[2];
        msg_imu.linear_acceleration.x = custom.high_state.imu.accelerometer[0];
        msg_imu.linear_acceleration.y = custom.high_state.imu.accelerometer[1];
        msg_imu.linear_acceleration.z = custom.high_state.imu.accelerometer[2];
        ROS_INFO("publishing /imu");
        pub_imu.publish(msg_imu);

        nav_msgs::Odometry msg_odom;
        msg_odom.header.seq = count;
        msg_odom.header.stamp = current_time;
        msg_odom.header.frame_id = "odom";
        msg_odom.child_frame_id = "base_link";
        msg_odom.pose.pose.position.x = custom.high_state.position[0];
        msg_odom.pose.pose.position.y = custom.high_state.position[1];
        msg_odom.pose.pose.position.z = custom.high_state.position[2];
        msg_odom.pose.pose.orientation.w = custom.high_state.imu.quaternion[0];
        msg_odom.pose.pose.orientation.x = custom.high_state.imu.quaternion[1];
        msg_odom.pose.pose.orientation.y = custom.high_state.imu.quaternion[2];
        msg_odom.pose.pose.orientation.z = custom.high_state.imu.quaternion[3];
        msg_odom.twist.twist.linear.x = custom.high_state.velocity[0];
        msg_odom.twist.twist.linear.y = custom.high_state.velocity[1];
        msg_odom.twist.twist.angular.z = custom.high_state.velocity[2];
        ROS_INFO("publishing /odom");
        pub_odom.publish(msg_odom);
        
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";
		odom_trans.transform.translation.x = custom.high_state.position[0];
		odom_trans.transform.translation.y = custom.high_state.position[1];
		odom_trans.transform.translation.z = custom.high_state.position[2];
		odom_trans.transform.rotation.w = custom.high_state.imu.quaternion[0];
		odom_trans.transform.rotation.x = custom.high_state.imu.quaternion[1];
		odom_trans.transform.rotation.y = custom.high_state.imu.quaternion[2];
		odom_trans.transform.rotation.z = custom.high_state.imu.quaternion[3];
        ROS_INFO("publishing odom -> base_link transform");
		odom_broadcaster.sendTransform(odom_trans);

        ros::spinOnce(); loop_rate.sleep();
    }

    return 0;
}
