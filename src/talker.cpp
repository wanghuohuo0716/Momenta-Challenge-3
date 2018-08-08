#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Imu.h>
#include "turtlebot3_wanghuohuo/Num.h"
#include "geometry_msgs/Twist.h"
int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<turtlebot3_wanghuohuo::Num>("ballpose", 1000);
  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Rate loop_rate(20);
  int count = 0;
  turtlebot3_wanghuohuo::Num msg;
  geometry_msgs::Twist vel;
  vel.angular.z=0.1;
  vel.linear.x=0.1;
  if(ros::param::get("talker/ball_x",msg.x_ball))
  {
    ros::param::get("talker/ball_y",msg.y_ball);
  }
  else
  {
    msg.x_ball=1.2;
    msg.y_ball=0.8695;
  }
  while (ros::ok())
  {
    chatter_pub.publish(msg);
    cmd_vel_pub.publish(vel);
    ROS_INFO_STREAM("ballpose x="<<msg.x_ball<<"ballpose y="<<msg.y_ball);
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
