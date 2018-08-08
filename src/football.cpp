//
//  football.c
//  
//
//  Created by huying on 2018/7/23.
//

#include <ros/ros.h>
#include "math.h"
Vector3D //位置向量定义
typedef struct
{
    double x,y,z;
}Vector3D;

Bounds //区域范围定义
typedef struct
{
    long left,right,top,bottom;
}Bounds;

Robot //我方机器人的信息定义
typedef struct
{
    Vector3D pos;//机器人的坐标位置
    double rotations;//机器人当前角度
    double velocityLeft,velocityRight;//机器人的左右轮速
}Robot;

OpponentRobot   //对方机器人信息的定义
typedef struct
{
    Vector3D pos;//机器人的坐标位置
    double rotation;//机器人当前的角度
}OpponentRobot;

Ball    //小球信息的定义
typedef struct
{
    Vector3D pos;   //小球的坐标位置
}

Environment //最重要的数据定义，包含所有运行时的信息，由系统刷新
typedef struct
{
    Robot home[PLAYERS_PER_SIDE]; //我方机器人数组
    OpponentRobot opponent[PLAYERS_PER_SIDE];//敌方机器人数组
    Ball currentBall,   //当前小球的位置
        lastBall;//上一次小球的位置
        predictedBall;//调用Predict()函数后可获得数据
    Bounds fieldBounds; //场地范围
           goalBounds;//球门的位置与范围
    long gameState;//当前比赛状态（各种定位球的标志变量）
    long whosBall;//由谁控制球
    void *userData;//用户自定义信息
}Environment;


void Angle(Robot *robot,int desired_angle)//该动作可以让机器人转到任意指定的角度
{
    int theta_e,vl,vr;
    theta_e=desired_angle-(int)robot->rotation;
    while(theta_e>180)theta_e -= 360;
    while(theta_e<-180)theta_e+=360;
    
    if(theta_e <-90) theta_e += 180;
    else if(theta_e>90) theta_e-=180;

if(abs(theta_e)>50)
    {
        v1 = (int)(-9.0/90.0*(doule)theta_e);
        vr = (int)(9.0/90.0*(doule)theta_e);
    }
    else if(abs(theta_e)>20)
    {
        v1 = (int)(-11.0/90.0*(double)theta_e);
        vr = (int)(11.0/90.0*(double)theta_e);
    }
    Velocity(robot,v1,vr);
}

void Attack(Robot *robot,Environment *env)
//机器人向小球的位置移动的方法
{
    predictedBall(env); //估计小球的方法
    Position(robot,env->predictedBall.pos.x,env->predictedBall.pos.y);
}//移动机器人到小球的位置

void predictedBall(Environment *env)//预估小球的位置
{
    double dx = env->currentBall.pos.x->lastBall.pos.x;
    double dy = env->currentBall.pos.y->env->lastBall.pos.y;
    env->predictBall.pos.x= env->currentBall.pos.x + dx;
    env->predictedBall.pos.y = env->currentBall.pos.y + dy;
}

void Position(Robot *robot,double x,double y)
//该动作使移动小车到指定位置
{
    int desired_angle=0,theta_e=0,d_angle=0,vl,vr,vc=70;
    
    double dx,dy,d_e,Ka = 10.0/90.0;
    dx = x-robot->pos.x; //计算当前位置与目标的相对位移
    dy = y-robot->pos.y;
    
    d_e = sqrt(dx * dx +dy * dy);//计算机器人到目标位置的直线距离
    if(dx==0&&dy==0)//计算当前位置到目标点的角度
        desired_angle=90;
    else
        desired_angle=(int)(180./PI*atan2((double)(dy),(double)(dx)));
    theta_e=desired_angle-(int)robot->rotation;
    //当前机器人的角度与机器人到目标连线角度的夹角
    while(theta_e > 180)theta_e-=360;
    while (theta_e< -180)theta_e+=360;
    
    if(d_e > 100.)
        Ka = 17./90.;
    else if(d_e > 50)
        Ka = 19./90.;
    else if(d_e > 30)
        Ka = 21./90
    else if(d_e>20)
        Ka = 23./90.;
    else
        Ka = 25./90.;
    
    if(theta_e>95 || theta_e<-95)
    {
        theta_e+= 180;
        
        if(theta_e>180)
            theta_e-=360;
        if(theta_e>80)
            theta_e=80;
        if(theta_e<-80)
            theta_e= -80;
        if(d_e<5.0&&abs(theta_e)<40)
            Ka = 0.1;
        vr = (int)(-vc*(1.0/(1.0+exp(-3.0*d_e))-0.3)+Ka*theta);
        v1 = (int)(-vc*(1.0/(1.0+exp(-3.0*d_e))-0.3)+Ka*theta);
    }
    
    else if(theta_e < 85&& theta_e>-85)
    {
       if(d_e<50 && abs(theta_e)< 40)
           Ka = 0.1;
        vr = (int)(vc*(1.0/(1.0+exp(-3.0*d_e))-0.3)+Ka*theta);
        v1 = (int)(vc*(1.0/(1.0+exp(-3.0*d_e))-0.3)+Ka*theta);
    }
    
    else
    {
        vr=(int)(+.17*theta_e);
        vl=(int)(-.17*theta_e);
    }
    Velocity(robot,vl,vr);
}


