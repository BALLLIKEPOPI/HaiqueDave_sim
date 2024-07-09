#include "mpc_control/controlPub.h"
#include "../../MPC_control/include/mpc_ctl.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "ros/subscriber.h"
#include "uuv_gazebo_ros_plugins_msgs/FloatStamped.h"
#include <sensor_msgs/JointState.h>

ros::Publisher thrusterPub0;
ros::Publisher thrusterPub1;
ros::Publisher thrusterPub2;
ros::Publisher thrusterPub3;
ros::Publisher thrusterPub4;
ros::Publisher thrusterPub5;
ros::Publisher thrusterPub6;
ros::Publisher thrusterPub7;
ros::Publisher joint_pub;
int thrust_gain;

void conSub_cb(const mpc_control::controlPub::ConstPtr &msg){
    mpc_control::controlPub conSub_msg = *msg;

    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust0;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust1;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust2;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust3;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust4;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust5;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust6;
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust7;
    sensor_msgs::JointState joint_state_left;
    sensor_msgs::JointState joint_state_right;

    thrust0.data = conSub_msg.thrust1/2*thrust_gain;
    thrust1.data = conSub_msg.thrust2/2*thrust_gain;
    thrust2.data = conSub_msg.thrust3/2*thrust_gain;
    thrust3.data = conSub_msg.thrust4/2*thrust_gain;
    thrust4.data = conSub_msg.thrust5/2*thrust_gain;
    thrust5.data = conSub_msg.thrust6/2*thrust_gain;
    thrust6.data = conSub_msg.thrust7/2*thrust_gain;
    thrust7.data = conSub_msg.thrust8/2*thrust_gain;
    joint_state_left.name.push_back("zuoqingzhuan_joint");
    joint_state_right.name.push_back("youqingzhuan_joint");

    // thrusterPub0.publish(thrust0);
    // thrusterPub1.publish(thrust1);
    // thrusterPub2.publish(thrust2);
    // thrusterPub3.publish(thrust3);
    // thrusterPub4.publish(thrust4);
    // thrusterPub5.publish(thrust5);
    // thrusterPub6.publish(thrust6);
    // thrusterPub7.publish(thrust7);
    joint_state_left.header.stamp = ros::Time::now();
    joint_state_left.position.resize(1);
    joint_state_left.position[0] = sin(30);
    joint_pub.publish(joint_state_left);

    joint_state_right.header.stamp = ros::Time::now();
    joint_state_right.position.resize(1);
    joint_state_right.position[0] = sin(60);
    joint_pub.publish(joint_state_right);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "thrustPub");
    ros::NodeHandle nh;

    nh.param("thrust_gain", thrust_gain, 100);

    thrusterPub0 = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/haique/thrusters/0/input", 10);
    thrusterPub1 = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/haique/thrusters/1/input", 10);
    thrusterPub2 = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/haique/thrusters/2/input", 10);
    thrusterPub3 = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/haique/thrusters/3/input", 10);
    thrusterPub4 = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/haique/thrusters/4/input", 10);
    thrusterPub5 = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/haique/thrusters/5/input", 10);
    thrusterPub6 = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/haique/thrusters/6/input", 10);
    thrusterPub7 = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/haique/thrusters/7/input", 10);
    joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
    ros::Subscriber conSub = nh.subscribe<mpc_control::controlPub>("/mpc_ctl", 10, conSub_cb);
    ros::Rate rate(10.0); 
    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
}