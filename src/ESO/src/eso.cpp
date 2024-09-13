#include "../include/eso.h"
#include "eso/eso.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "ros/subscriber.h"
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <iostream>

using namespace std;

float I1 = 0.103; float I2 = 0.104; float I3 = 0.161; 
float m = 4.8; // kg
float V = 0.00285; // m3
float rou = 1000; // kg/m3
float G = 9.8; // m/s2
float l = 0.6; // m
float c1 = 0.01; float c2 = 0.01; float c3 = 0.01;
float k = 0.00031; // 推力系数
float c = 0.000015; // 反扭系数

float Mx = 0.0;
float My = 0.0;
float Mz = 0.0;
float Tx = 0.0;
float Ty = 0.0;
float Tz = 0.0;

float input[6] = {0};
float state[6] = {0};
float drag[6] = {0};
ros::Publisher eso_Pub;

vector<ESO> ESO_vec(6);

void conSub_cb(const mpc_control::controlPub::ConstPtr &msg){
    mpc_control::controlPub conSub_msg = *msg;
    // cout << conSub_msg << endl;

    float M1 = c*conSub_msg.thrust1/k;     float M2 = c*conSub_msg.thrust2/k;     
    float M3 = c*conSub_msg.thrust3/k;     float M4 = c*conSub_msg.thrust4/k; 
    float M5 = c*conSub_msg.thrust5/k;     float M6 = c*conSub_msg.thrust6/k; 
    float M7 = c*conSub_msg.thrust7/k;     float M8 = c*conSub_msg.thrust8/k; 

    Mx = l*((conSub_msg.thrust2+conSub_msg.thrust6)*sin(conSub_msg.beta)-(conSub_msg.thrust4+conSub_msg.thrust8)*sin(conSub_msg.alpha));
    My = l*(-conSub_msg.thrust1-conSub_msg.thrust5+conSub_msg.thrust3+conSub_msg.thrust7);
    Mz = M1+M3-M5-M7+l*((conSub_msg.thrust2+conSub_msg.thrust6)*cos(conSub_msg.beta)-
                (conSub_msg.thrust4+conSub_msg.thrust8)*cos(conSub_msg.alpha));

    Tx = (conSub_msg.thrust2+conSub_msg.thrust6)*cos(conSub_msg.beta)+(conSub_msg.thrust4+conSub_msg.thrust8)*cos(conSub_msg.alpha);
    Ty = 0;
    Tz = conSub_msg.thrust1+conSub_msg.thrust3+conSub_msg.thrust5+conSub_msg.thrust7+
                (conSub_msg.thrust2+conSub_msg.thrust6)*sin(conSub_msg.beta)+(conSub_msg.thrust4+conSub_msg.thrust8)*sin(conSub_msg.alpha);
    
    input[0] = Mx; input[1] = My; input[2] = Mz;
    input[3] = Tx; input[4] = Ty; input[5] = Tz;
    // for(int i = 0; i < 6; i++){
    //     cout << "input" << i << ": " << input[i] << endl;
    // }
}

// for sim
void local_pose_cb(const gazebo_msgs::ModelStates::ConstPtr& msg){
    double roll, pitch, yaw;
    // for realfly
    // geometry_msgs::PoseStamped current_odom = *msg;
    // for sim
    std::string model_name = msg->name[3];
    geometry_msgs::Pose model_pose = msg->pose[3];
    geometry_msgs::Twist model_twist = msg->twist[3];
    eso::eso eso_msg;

    // for realfly
    // double w = current_odom.pose.orientation.w;
    // double x = current_odom.pose.orientation.x;
    // double y = current_odom.pose.orientation.y;
    // double z = current_odom.pose.orientation.z;
    // for sim
    double w = model_pose.orientation.w;
    double x = model_pose.orientation.x;
    double y = model_pose.orientation.y;
    double z = model_pose.orientation.z;
    double x_ = model_pose.position.x;
    double y_ = model_pose.position.y;
    double z_ = model_pose.position.z;
    double v_x = model_twist.linear.x;
    double v_y = model_twist.linear.y;
    double v_z = model_twist.linear.z;
    double w_x = model_twist.angular.x;
    double w_y = model_twist.angular.y;
    double w_z = model_twist.angular.z;

    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    roll = atan2(sinr_cosp, cosr_cosp);

    double sinp = 2 * (w * y - z * x);
    if (abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp); // 使用 90 度或 -90 度
    else
        pitch = asin(sinp);

    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw = atan2(siny_cosp, cosy_cosp);
    state[0] = w_x; state[1] = w_y; state[2] = w_z; 
    state[3] = v_x; state[4] = v_y; state[5] = v_z; 

    for(int i = 0; i < 6; i++){
        cout << i << endl;
        ESO_vec[i].SetInput(input[i], state[i]);
        ESO_vec[i].ESO_ADRC();
        drag[i] = ESO_vec[i].GetEState();
        if(i == 5){
            drag[i] = drag[i] - (rou*V-m)*G/m;
        }
        // cout << "drag" << i << ": " << drag[i] << endl;
        // cout << "state" << i << ": " << state[i] << endl;
    }
    eso_msg.torque1 = drag[0];
    eso_msg.torque2 = drag[1];
    eso_msg.torque3 = drag[2];
    eso_msg.force1 = drag[3];
    eso_msg.force2 = drag[4];
    eso_msg.force3 = drag[5];
    eso_Pub.publish(eso_msg);
    cout << "roll: " << pitch << "  pitch: " << -roll << "  yaw: " << yaw << endl;
}

void paraInit(double para[]){
    para[0] = 1/I1;
    para[1] = 1/I2;
    para[2] = 1/I3;
    para[3] = 1/m;
    para[4] = 1/m;
    para[5] = 1/m;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "eso_node");
    ros::NodeHandle nh;

    // for sim
    ros::Subscriber sub = nh.subscribe("/gazebo/model_states", 10, local_pose_cb);
    ros::Subscriber conSub = nh.subscribe<mpc_control::controlPub>("/mpc_ctl", 10, conSub_cb);
    eso_Pub = nh.advertise<eso::eso>("/eso_pub", 10);
    double para[6];
    paraInit(para);
    for(int i = 0; i < 6; i++){
        ESO_vec[i].ESOInit(para[i]);
    }
    ros::Rate rate(50.0); 
    

    while(ros::ok()) {
        // for(int i = 0; i < 6; i++){
        //     cout << i << endl;
        //     ESO_vec[i].SetInput(input[i], state[i]);
        //     ESO_vec[i].ESO_ADRC();
        //     drag[i] = ESO_vec[i].GetEState();
        //     if(i == 5){
        //         drag[i] = drag[i] - (rou*V-m)*G/m;
        //     }
        //     cout << "drag" << i << ": " << drag[i] << endl;
        // }
        ros::spinOnce();
        rate.sleep();
    }
}