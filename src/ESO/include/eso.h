#ifndef __ESO_H__
#define __ESO_H__

#include "eigen3/Eigen/Eigen"
#include <iostream>
#include <ostream>
#include <vector>
#include <ros/ros.h>
#include "ros/publisher.h"
#include "mpc_control/controlPub.h"

using namespace std;

class ESO{
    public:
    ESO(){
        h = 0.02;
        z1 = 0.0;
        z2 = 0.0;
        e = 0.0; //系统状态误差
        y = 0.0; //系统输出量
        fe = 0.0;
        fe1 = 0.0;
        beta_01 = 0.4; // 2w
        beta_02 = 0.04; // w^2
        u = 0;
    }
    ~ESO(){};

    int Sign_ADRC(float Input){
        int output=0;
        if(Input>1E-6) output=1;
        else if(Input<-1E-6) output=-1;
        else output=0;
        return output;
    }

    float Fal_ADRC(float e,float alpha,float zeta){
        int s=0;
        float fal_output=0;
        s=(Sign_ADRC(e+zeta)-Sign_ADRC(e-zeta))/2;
        fal_output=e*s/(powf(zeta,1-alpha))+powf(fabs(e),alpha)*Sign_ADRC(e)*(1-s);
        return fal_output;
    }

    void ESO_ADRC(){
        e=z1-y; //状态误差
        cout << "e:" << e << endl;
        fe=Fal_ADRC(e,0.5, h); //非线性函数，提取跟踪状态与当前状态误差
        fe1=Fal_ADRC(e,0.25,h);
        
        /*************扩展状态量更新**********/
        z1+=h*(z2-beta_01*fe+b0*u);
        z2+=h*(-beta_02*fe1);
        // cout << "z1:" << z1 << " z2:" << z2 << endl;
    }

    void ESOInit(float para1_){
        b0 = para1_;
    }

    void SetInput(float input, float state){
        u = input;
        y = state;
    }

    float GetEState(){
        return z2;
    }

    private:
    float h;
    /*****扩张状态观测器*******/
    /******已系统输出y和输入u来跟踪估计系统状态和扰动*****/
    float z1;
    float z2;
    float e; //系统状态误差
    float y; //系统输出量
    float fe;
    float fe1;
    float beta_01;
    float beta_02;
    float u;
    float b0;
    float d0;
};

#endif