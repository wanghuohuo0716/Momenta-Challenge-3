#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <nav_msgs/Odometry.h>
#include <turtlebot3_wanghuohuo/Num.h>
#include <turtlebot3_wanghuohuo/Num2.h>

typedef struct
{
	double x;
	double y;
}Point;

class Robot
{
	public:
		Robot();
		~Robot();
		//void turn(bool direction,float desired_angle);
		//void go_front(double currentpos,double length);
		void imuMessageReceived(const sensor_msgs::Imu & msg);
		void odomMessageReceived(const nav_msgs::Odometry & msg);
		void mymsgReceived(const turtlebot3_wanghuohuo::Num &msg);
		void mymsg2Received(const turtlebot3_wanghuohuo::Num2 &msg);
        void convertPose(void);
		void Attack(void);
		void Rotate(void);
		void k_turn(double desired_angle);
		void go(void);
		void go_back(void);
		void control1(void);
		void control2(void);
		void Is_in_fence(void);

		turtlebot3_wanghuohuo::Num mymsg;
		ros::NodeHandle nh_;
		ros::Publisher vel_;
		ros::Publisher pub_cmd_vel_ ;
		ros::Subscriber sub_imu_;
		ros::Subscriber sub_odom_;
		ros::Subscriber sub_mymsg_;
		ros::Subscriber sub_mymsg2_;
		geometry_msgs::Twist vel_msg_;
		nav_msgs::Odometry odom_;

		//double pos_x;
		//double pos_y;
		int two_ball_near;
		int door_attack;//which door to attack
		int go_flag;
		int turn1_flag;
		int turn2_flag;
		int attack_flag;
		int current_state;
		int last_state;
		int last_last_state;
		int state_flag;
		int ball_lost_count;
		int ball_near_count;
		int state_count;
		int state_count2;
		int go_circle_count;
		int attack_count;

		double kp;
		double ki;
		double kd;
		double error;
		double last_error;
		double output;
		double integrate;


		double max_attack_angle;
		double min_attack_angle;

		double fence_left_y;
		double fence_right_y;
		double fence_up_x;
		double fence_down_x;

		double door_x;//x position of door
		double door_y;//y position of door
		double middle_x;
		double middle_y;
		double angle_z;// really angle of robot
		double last_angle_z;
		double current_x;
		double current_y;

		double ball_direct_distance;

		double desired_angle;
		double target_x;//target position to attack
		double target_y;
		double target_angle;//the angle of going to the postion
		double target_angle2;
		double attack_angle;//the angle of attack the ball
		double last_ball_x;
		double last_ball_y;
		double last_camera_x;

		double calibrate_x;
		double calibrate_y;
		double turn_radius;
		double turn_vel;

};
//类中变量的分为两类，一类是类的变量，另一类的成员对象的变量。类的变量一般都是static，所以成员函数或者数据类型除非名字有异议，否则很少用this,其他就是正常的。
