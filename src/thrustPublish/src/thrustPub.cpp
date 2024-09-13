#include "mpc_control/controlPub.h"
#include "../../MPC_control/include/mpc_ctl.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "ros/subscriber.h"
#include "uuv_gazebo_ros_plugins_msgs/FloatStamped.h"
#include <casadi/core/calculus.hpp>
#include <cmath>
#include <iostream>
#include <ostream>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <vector>

ros::Publisher thrusterPub0;
ros::Publisher thrusterPub1;
ros::Publisher thrusterPub2;
ros::Publisher thrusterPub3;
ros::Publisher thrusterPub4;
ros::Publisher thrusterPub5;
ros::Publisher thrusterPub6;
ros::Publisher thrusterPub7;
ros::Publisher joint1_pub;
ros::Publisher joint2_pub;
int thrust_gain;
vector<uuv_gazebo_ros_plugins_msgs::FloatStamped> thrust(8);
vector<uuv_gazebo_ros_plugins_msgs::FloatStamped> last_thrust(8);
vector<std_msgs::Float64> joint_angle(2);
vector<std_msgs::Float64> last_joint_angle(2);

void conSub_cb(const mpc_control::controlPub::ConstPtr &msg){
    mpc_control::controlPub conSub_msg = *msg;
    
    thrust[0].data = copysign(1.0, conSub_msg.thrust1)*sqrt(abs(conSub_msg.thrust1)/2)*thrust_gain;
    thrust[1].data = copysign(1.0, conSub_msg.thrust2)*sqrt(abs(conSub_msg.thrust2)/2)*thrust_gain;
    thrust[2].data = copysign(1.0, conSub_msg.thrust3)*sqrt(abs(conSub_msg.thrust3)/2)*thrust_gain;
    thrust[3].data = -copysign(1.0, conSub_msg.thrust4)*sqrt(abs(conSub_msg.thrust4)/2)*thrust_gain;
    thrust[4].data = copysign(1.0, conSub_msg.thrust5)*sqrt(abs(conSub_msg.thrust5)/2)*thrust_gain;
    thrust[5].data = copysign(1.0, conSub_msg.thrust6)*sqrt(abs(conSub_msg.thrust6)/2)*thrust_gain;
    thrust[6].data = copysign(1.0, conSub_msg.thrust7)*sqrt(abs(conSub_msg.thrust7)/2)*thrust_gain;
    thrust[7].data = -copysign(1.0, conSub_msg.thrust8)*sqrt(abs(conSub_msg.thrust8)/2)*thrust_gain;
    joint_angle[1].data = conSub_msg.alpha;
    joint_angle[0].data = conSub_msg.beta;

    // for(int i = 0; i < 8; i++){
    //     if(std::isnan(thrust[i].data)){
    //         thrust[i].data = last_thrust[i].data;
    //     }
    // }
    // for(int i = 0; i < 2; i++){
    //     if(std::isnan(joint_angle[i].data)){
    //         joint_angle[i].data = last_joint_angle[i].data;
    //     }
    // // }    
    // last_thrust.insert(last_thrust.begin(), thrust.begin(), thrust.end());
    // last_joint_angle.insert(last_joint_angle.begin(), joint_angle.begin(), joint_angle.end());
}

int main(int argc, char **argv){
    ros::init(argc, argv, "thrustPub");
    ros::NodeHandle nh;

    nh.param("thrust_gain/value", thrust_gain, 1000);

    thrusterPub0 = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/haique/thrusters/0/input", 10);
    thrusterPub1 = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/haique/thrusters/1/input", 10);
    thrusterPub2 = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/haique/thrusters/2/input", 10);
    thrusterPub3 = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/haique/thrusters/3/input", 10);
    thrusterPub4 = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/haique/thrusters/4/input", 10);
    thrusterPub5 = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/haique/thrusters/5/input", 10);
    thrusterPub6 = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/haique/thrusters/6/input", 10);
    thrusterPub7 = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/haique/thrusters/7/input", 10);
    joint1_pub = nh.advertise<std_msgs::Float64>("/haique/haique_joint1_position_controller/command", 10);
    joint2_pub = nh.advertise<std_msgs::Float64>("/haique/haique_joint2_position_controller/command", 10);
    ros::Subscriber conSub = nh.subscribe<mpc_control::controlPub>("/mpc_ctl", 10, conSub_cb);
    ros::Rate rate(50.0); 
    while(ros::ok()) {
        thrusterPub0.publish(thrust[0]);  // motor0
        thrusterPub1.publish(thrust[1]);  // motor1
        thrusterPub2.publish(thrust[2]);  // motor2
        thrusterPub3.publish(thrust[3]);  // motor3
        thrusterPub4.publish(thrust[4]);  // motor4
        thrusterPub5.publish(thrust[5]);  // motor5
        thrusterPub6.publish(thrust[6]);  // motor6
        thrusterPub7.publish(thrust[7]);  // motor7
        joint1_pub.publish(joint_angle[0]);
        joint2_pub.publish(joint_angle[1]);
        ros::spinOnce();
        rate.sleep();
    }
}