#include "motion.h"
#define left 0
#define right 1
#define PI 3.141592657
//该动作可以让机器人原地任意指定的角度
//开机0度，左转为正，右转为负，正负180度
//angular_vel为正，则左转；为负，则右转
//atan2 operator is -180~+180
Robot::Robot()
{
	sub_imu_= nh_.subscribe("/imu",30,&Robot::imuMessageReceived,this);
	sub_odom_= nh_.subscribe("/odom",30,&Robot::odomMessageReceived,this);
    pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel",1,this);
	sub_mymsg_= nh_.subscribe("/output/odom/football",30,&Robot::mymsgReceived,this);
	sub_mymsg2_ = nh_.subscribe("/output/odom_car",30,&Robot::mymsg2Received,this);
	two_ball_near=0;
	go_flag=0;//the initial value is 0
	turn1_flag=0;
	turn2_flag=0;
	attack_flag=0;

	turn_radius=0.42;//0.358 0.476 0.37 0.362
	turn_vel=1;

	state_flag=-3;

	state_count=0;
	state_count2=0;
	ball_lost_count=0;
	ball_near_count=0;
	go_circle_count=0;

	angle_z=0;//initial the angle

	//the fence filed

	fence_left_y=1.25;
	fence_right_y=0;
	fence_up_x=2.82;
	fence_down_x=0.5;

	
	door_attack=1;//the door to attack:1 or 2

	middle_x=1.3;//middle place
	middle_y=0.86;

	if(door_attack==1)
	{
		door_x=2.626;//Door 1
		door_y=0.8695;
	}
	else if(door_attack==2)
	{
		door_x=0.299;// Door 2 
		door_y=0.863;	
	}
		else
		{
			while(1)
			ROS_INFO_STREAM("!!!!!!!!!PARAMETER OF DOOR_ATTACK IS WRONG!!!!!!!!!");
		}
	if(ros::param::get("motion/kp",kp))
	{
		ros::param::get("motion/ki",ki);
		ros::param::get("motion/kd",kd);
	}
	else
	{
		while(1)
		ROS_INFO_STREAM("!!!!!!!!!PARAMETER OF PID LOST!!!!!!!!!");
	}
}

Robot::~Robot()
{

}


/* void Robot::turn(bool direction,float desired_angle)
{
	double target_angle;
	ros::Rate rate(10);
	if(direction == left)//turn left
	{
		desired_angle=target_angle;//!!!!!!!buff the target_angle,the method is very!!!!!!
		if(target_angle>180) target_angle-=360;
		while((angle_z-target_angle)>0.1)
		{
			vel_msg_.linear.x=0;//max:0.22
			vel_msg_.angular.z=0.5;//max:2.48
			pub_cmd_vel_.publish(vel_msg_);
			rate.sleep();
		}
		vel_msg_.linear.x=0;//max:0.22
		vel_msg_.angular.z=0;//max:2.48
		pub_cmd_vel_.publish(vel_msg_);
	}
	else//turn right
	{
		target_angle=angle_z-desired_angle;
		if(target_angle>180) target_angle+=360;
		while((angle_z-target_angle)>0.1)
		{
			vel_msg_.linear.x=0;//max:0.22
			vel_msg_.angular.z=-0.5;//max:2.48
			pub_cmd_vel_.publish(vel_msg_);
			rate.sleep();
		}
	//stop after turning over
	vel_msg_.linear.x=0;//max:0.22
	vel_msg_.angular.z=0;//max:2.48
	pub_cmd_vel_.publish(vel_msg_);
	}
} */

/* void Robot::go_front(double currentpos, double length)
{
    while(fabs(odom_.pose.pose.position.x-(currentpos+length)<0.01))
    {
        vel_msg_.linear.x=0.2;
        vel_msg_.angular.z=0;
        pub_cmd_vel_.publish(vel_msg_);
    }
    vel_msg_.linear.x=0;//max:0.22
    vel_msg_.angular.z=0;//max:2.48
    pub_cmd_vel_.publish(vel_msg_);
} */

/*
//convert the coordinate frame
void Robot::convertPose(void)
{
    double delta_x =(odom_.pose.pose.position.x-old_pos_)*sin(angle_z);
	double delta_y =(odom_.pose.pose.position.x-old_pos_)*cos(angle_z);
	pos_x+=delta_x;
	pos_y+=delta_y;
	old_pos_=odom_.pose.pose.position.x;
}*/

//subscribe
void Robot::imuMessageReceived(const sensor_msgs::Imu & msg)//77Hz 
{
/* 	angle_z=atan2(2*(msg.orientation.z*msg.orientation.w+msg.orientation.x*msg.orientation.y),
                  1-2*(msg.orientation.y*msg.orientation.z+msg.orientation.z*msg.orientation.z));// rad
	angle_z=angle_z / (2 * 3.141592657) * 360; */
	//ROS_INFO_STREAM("imu");
	//ROS_INFO_STREAM("angle_z="<<angle_z);
	//ROS_INFO_STREAM("angle_z="<<angle_z);
	//ROS_INFO_STREAM("imu_data:orientation."<<msg.orientation.z);
}

//订阅odom信息
void Robot::odomMessageReceived(const nav_msgs::Odometry & msg)//25Hz 
{
	//ROS_INFO_STREAM("pos.x="<<msg.pose.pose.position.x<<" pos.y="<<msg.pose.pose.position.y);
    odom_=msg;

	angle_z=atan2(2*(msg.pose.pose.orientation.z*msg.pose.pose.orientation.w+msg.pose.pose.orientation.x*msg.pose.pose.orientation.y),
                  1-2*(msg.pose.pose.orientation.y*msg.pose.pose.orientation.z+msg.pose.pose.orientation.z*msg.pose.pose.orientation.z));// rad
	angle_z=angle_z / (2 * 3.141592657) * 360;

	max_attack_angle=atan2(0.2,door_x-odom_.pose.pose.position.x);
	min_attack_angle=atan2(0.1,door_x-odom_.pose.pose.position.x);
	//ROS_INFO_STREAM("odom");
	//ROS_INFO_STREAM("pos_x="<<odom_.pose.pose.position.x);
	//ROS_INFO_STREAM("pos_y="<<odom_.pose.pose.position.y);
    //convertPose();
}

//Attack Ball!!!
void Robot::mymsgReceived(const turtlebot3_wanghuohuo::Num &msg)//20Hz
{
	//ROS_INFO_STREAM("detected="<<msg.ball_detected);
	mymsg=msg;
	//ROS_INFO_STREAM("mymsg");
	//transfer the unit 

	//trasfer it to the absolute coordinate frame
	//mymsg.x_ball=sin((atan2(mymsg.x_ball,mymsg.y_ball)+angle_z/PI))*sqrt(mymsg.x_ball*mymsg.x_ball+mymsg.y_ball*mymsg.y_ball)+ odom_.pose.pose.position.x;
	//mymsg.x_ball=cos((atan2(mymsg.x_ball,mymsg.y_ball)+angle_z/PI))*sqrt(mymsg.x_ball*mymsg.x_ball+mymsg.y_ball*mymsg.y_ball)+ odom_.pose.pose.position.y;

	ball_direct_distance=mymsg.x_ball/1000.0;

	mymsg.x_ball=odom_.pose.pose.position.x+msg.x_ball/1000.0*cos(angle_z/180.0*PI)-msg.y_ball/1000.0*sin(angle_z/180.0*PI);
	mymsg.y_ball=odom_.pose.pose.position.y+msg.y_ball/1000.0*cos(angle_z/180.0*PI)+msg.x_ball/1000.0*sin(angle_z/180.0*PI);
	


	if(mymsg.ball_detected==1)//msg is valid when ball is detected
	{
		double k=(mymsg.y_ball - door_y)/(mymsg.x_ball-door_x);//calc the line of k
		double b=door_y-door_x*k;//calc the line of b
		if(door_attack==1)
		{
			target_x=mymsg.x_ball+0.4;//attack the door 1,so the operator is "-":decrease,the robot is 20cm wide
		}
		else if(door_attack==2)
		{
			target_x=mymsg.x_ball+0.4;//attack the door 2,so the operator is "+":increase,the robot is 20cm wide
		}
			else
			{
				while(1)
				{
					ROS_INFO_STREAM("!!!!!!!!!PARAMETER OF DOOR_ATTACK IS WRONG!!!!!!!!!");
				}
			}
/*		ROS_INFO_STREAM("detected="<<msg.ball_detected);
 		ROS_INFO_STREAM("x_ball="<<mymsg.x_ball);
		ROS_INFO_STREAM("y_ball="<<mymsg.y_ball);
		ROS_INFO_STREAM("target_x="<<target_x);
		ROS_INFO_STREAM("target_y="<<target_y);
		ROS_INFO_STREAM("target_agnle="<<target_angle);
		ROS_INFO_STREAM("attack_angle="<<attack_angle);		 */

		//target_y=target_x*k+b;
		//target_angle=atan2(mymsg.y_ball-odom_.pose.pose.position.y,mymsg.x_ball-odom_.pose.pose.position.x);//calcu the target_angle
		//attack_angle=atan2(mymsg.y_ball-door_y,mymsg.x_ball-door_x);//calcu the attack_angle
		
		//the type of return in fabs() is 'double' 
		if(fabs(mymsg.x_ball-last_ball_x)<=0.1)
		{
			two_ball_near=1;
		}
		else if(fabs(mymsg.x_ball-last_ball_x)>0.1)
		{
			two_ball_near=0;
		}
		last_ball_y=mymsg.y_ball;
		last_ball_x=mymsg.x_ball;
		last_camera_x=mymsg.camera_x_ball;
		//ROS_INFO_STREAM("target_x="<<target_x<<" target_y="<<target_y);
		//ROS_INFO_STREAM("k="<<k<<" b="<<b);
	}
}

void Robot::mymsg2Received(const turtlebot3_wanghuohuo::Num2 &msg)
{
	ROS_INFO_STREAM("cali");
	calibrate_x=msg.x_car/1000.0;
	calibrate_y=msg.y_car/1000.0;
}

//the turn angle is the atan2(k),k is the slope of the line
void Robot::k_turn(double desired_angle)
{
	double delta_z;
	ROS_INFO_STREAM("turn");
	delta_z=desired_angle-angle_z;
	if(delta_z<0)
	{
		delta_z+=360;
	}

	if(fabs(desired_angle-angle_z)>=5)
	{
		ROS_INFO_STREAM("fabs(desired_angle-angle_z)"<<fabs(desired_angle-angle_z));
		ROS_INFO_STREAM("turn1");
		vel_msg_.linear.x=0;
		if(delta_z>=0 && delta_z<=180)//turn left
		{
			vel_msg_.angular.z=turn_vel;
		}
		else	
		{
			vel_msg_.angular.z=-1*turn_vel;//turn right
		}

		pub_cmd_vel_.publish(vel_msg_);
	}
	
	//important code the mind is very important!to set a buffer,when origin has changed,the buffer won't change
	//desired_angle=target_angle;
/* 	if(desired_angle-angle_z>0)//turn left
	{
		while(fabs(desired_angle-angle_z)>5)
		{
			ROS_INFO_STREAM("fabs(desired_angle-angle_z)"<<fabs(desired_angle-angle_z));
			ROS_INFO_STREAM("turn1");
			vel_msg_.linear.x=0;
        	vel_msg_.angular.z=0.5;		
			pub_cmd_vel_.publish(vel_msg_);
			ros::spinOnce();//first sub the msg
		}
	}
	if(desired_angle-angle_z<0)//turn right
	{
		while(fabs(desired_angle-angle_z)>5)	
		{
			ROS_INFO_STREAM("fabs(desired_angle-angle_z)"<<fabs(desired_angle-angle_z));
			ROS_INFO_STREAM("turn2");
			vel_msg_.linear.x=0;
			vel_msg_.angular.z=-0.5;		
			pub_cmd_vel_.publish(vel_msg_);
			ros::spinOnce();//first sub the msg
		}
	} */
}

void Robot::go(void)//x,y value of the target position
{
	//it's no matter x or y.it is not necessary the to set the both are in the filed.actually x value is better
/* 	while(fabs(odom_.pose.pose.position.x-mymsg.x_ball)>0.1)
	{
		//calc current ball position whether it is close to last position,while(if close or no detected)
		if(two_ball_near==1 || mymsg.ball_detected==0)			
		{
			ROS_INFO_STREAM("fabs(odom_.pose.pose.position.x-mymsg.x_ball)="<<fabs(odom_.pose.pose.position.x-mymsg.x_ball));
			ROS_INFO_STREAM("go");
			vel_msg_.linear.x=0.2;
			vel_msg_.angular.z=0;												
			pub_cmd_vel_.publish(vel_msg_);//just go ,set speed 
		}
		else if(two_ball_near==0)
		{
			break;//jump out the while
		}
		ros::spinOnce();//first sub the msg
	} */
	if(fabs(odom_.pose.pose.position.x-target_x)>0.1)
	{
		//calc current ball position whether it is close to last position,while(if close or no detected)
		//if(two_ball_near==1 || mymsg.ball_detected==0)			
		//{
				ROS_INFO_STREAM("fabs(odom_.pose.pose.position.x-mymsg.x_ball)="<<fabs(odom_.pose.pose.position.x-mymsg.x_ball));
				ROS_INFO_STREAM("go");
				vel_msg_.linear.x=0.2;
				vel_msg_.angular.z=0;												
				pub_cmd_vel_.publish(vel_msg_);//just go ,set speed 
		//}
	}
}

void Robot::Attack(void)
{
	//x*y=320*240. the middle value of x is 160
	error=160-mymsg.camera_x_ball;
	output=kp*error + ki*integrate + kd*(last_error-error);
	last_error=error;

	if(output>2.48) output=2.48;
	if(output<-2.48) output=-2.48;
	//ROS_INFO_STREAM("Attack"<<" error="<<error<<" angle_z="<<angle_z);
	//ROS_INFO_STREAM("error="<<error<<" output="<<output);
	vel_msg_.linear.x=0.22;
	vel_msg_.angular.z=output;
	pub_cmd_vel_.publish(vel_msg_);//just go ,set speed 
	//ROS_INFO_STREAM("kp="<<kp<<" ki="<<ki<<" kd="<<kd);
	//ROS_INFO_STREAM("error="<<error<<" integrate="<<integrate<<"last_error-error="<<last_error-error);
	//ROS_INFO_STREAM("output="<<output);
}

void Robot::Rotate(void)
{
	//x*y=320*240. the middle value of x is 160
	error=160-mymsg.camera_x_ball;
	output=kp*error + ki*integrate + kd*(last_error-error);
	last_error=error;

	if(output>2.48) output=2.48;
	if(output<-2.48) output=-2.48;
	//ROS_INFO_STREAM("Attack"<<" error="<<error<<" angle_z="<<angle_z);
	//ROS_INFO_STREAM("error="<<error<<" output="<<output);
	vel_msg_.linear.x=0;
	vel_msg_.angular.z=output;
	pub_cmd_vel_.publish(vel_msg_);//just go ,set speed 
	//ROS_INFO_STREAM("kp="<<kp<<" ki="<<ki<<" kd="<<kd);
	//ROS_INFO_STREAM("error="<<error<<" integrate="<<integrate<<"last_error-error="<<last_error-error);
	//ROS_INFO_STREAM("output="<<output);
}

void Robot::control1(void)
{
/* 	switch(state_flag)
	{
		case 0:
			vel_msg_.linear.x=0;
			vel_msg_.angular.z=0.2;		
			pub_cmd_vel_.publish(vel_msg_);
			if(mymsg.ball_detected==1) state_flag=1;
		break;

		case 1:
			k_turn(target_angle);//turn to target_angle
			if(fabs(desired_angle-angle_z)<=6) 
			{
				state_flag=2;
			}
		break;

		case 2:
			go();//go to the target position
			if(fabs(odom_.pose.pose.position.x-target_x)<=0.2)
			{
				state_flag=3;
			}
		break;

		case 3:
			k_turn(attack_angle);//turn to attack_angle	
			if(fabs(desired_angle-angle_z)<=6)
			{
				state_flag=4;
			}	
		break;

		case 4:
			Attack();
			if(mymsg.ball_detected==0)
			{
				ball_lost_count++;
				if(ball_lost_count>40)
				{
					ball_lost_count=0;
					state_flag=0;
				}
			}
		break;
	} */

	//ROS_INFO_STREAM("ball_detected="<<mymsg.ball_detected);
/* 	if(mymsg.ball_detected==1)//detected the ball
	{ 	
		
	}
	else//rotate,pub frequency is 10Hz
	{
		vel_msg_.linear.x=0;
		if(last_camera_x<=160)
		{
			vel_msg_.angular.z=0.2;
		}
		else
		{
			vel_msg_.angular.z=-0.2;
		}		
		pub_cmd_vel_.publish(vel_msg_);//just go ,set speed 
	} */
}

void Robot::Is_in_fence(void)
{
	if(odom_.pose.pose.position.x<fence_down_x | odom_.pose.pose.position.x>fence_up_x | 
	   odom_.pose.pose.position.y<fence_right_y | odom_.pose.pose.position.y>fence_left_y)
	{
		state_flag=6;
	}
}

void Robot::go_back(void)
{
	//y=0.43,x=1.3
	vel_msg_.linear.x=-0.22;
	vel_msg_.angular.z=0;		
	pub_cmd_vel_.publish(vel_msg_);
}

void go_middle(void)
{}

void Robot::control2(void)
{
	switch(state_flag)
	{
		case -3://go straight
			ROS_INFO_STREAM("state_flag<-3-3-3-3-3-3-3-3-3 go straight -3-3-3-3-3-3-3-3-3-3-3>");
			ROS_INFO_STREAM("ball_detected="<<mymsg.ball_detected);
			ROS_INFO_STREAM("last_state="<<last_state<<"last_last_state"<<last_last_state);
			ROS_INFO_STREAM("odom_.pose.pose.position.x="<<odom_.pose.pose.position.x);
			ROS_INFO_STREAM("mymsg.ball_detected="<<mymsg.ball_detected);
			//Attack();
			vel_msg_.linear.x=0.22;
			vel_msg_.angular.z=0;		
			pub_cmd_vel_.publish(vel_msg_); 

			if(odom_.pose.pose.position.x>=0.2) state_flag=-4;
		current_state=-3;
		last_state=-3;
		break;

		case -4://turn right
			if(current_state!=-4)
			{
				last_last_state=last_state;
				last_state=current_state;
			}
			vel_msg_.linear.x=0;
			vel_msg_.angular.z=-1;		
			pub_cmd_vel_.publish(vel_msg_);
			ROS_INFO_STREAM("last_state="<<last_state<<" last_last_state"<<last_last_state);
			ROS_INFO_STREAM("state_flag<-4-4-4-4-4-4-4-4-4 turn left  -4-4-4-4-4-4-4-4-4-4-4-4>");
			ROS_INFO_STREAM("angle_z="<<angle_z);
			if(fabs(angle_z+90)<5) state_flag=-5;
		current_state=-4;
		break;

		case -5://go straight
			if(current_state!=-5)
			{
				last_last_state=last_state;
				last_state=current_state;
			}
			vel_msg_.linear.x=0.22;
			vel_msg_.angular.z=0;		
			pub_cmd_vel_.publish(vel_msg_);
			ROS_INFO_STREAM("last_state="<<last_state<<" last_last_state"<<last_last_state);
			ROS_INFO_STREAM("state_flag<-5-5-5-5-5-5-5-5-5  go straight  -5-5-5-5-5-5-5-5-5-5-5-5>");
			ROS_INFO_STREAM("odom_.pose.pose.position.y="<<odom_.pose.pose.position.y);
			if(odom_.pose.pose.position.y<=-0.6) state_flag=-7;
		current_state=-5;
		break;

		case -6://rotate toward the ball
			if(current_state!=-6)
			{
				last_last_state=last_state;
				last_state=current_state;
			}
		    if(mymsg.ball_detected==1)
			{
				ball_lost_count=0;
				Rotate();
				/* if(mymsg.camera_x_ball<100)
				{
                    vel_msg_.linear.x=0;
					vel_msg_.angular.z=-1;
				}
				else if(mymsg.camera_x_ball>=100&&mymsg.camera_x_ball<160)
				{
  					vel_msg_.linear.x=0;
					vel_msg_.angular.z=-0.5;
				}
				else if(mymsg.camera_x_ball>=160&&mymsg.camera_x_ball<220)
				{
  					vel_msg_.linear.x=0;
					vel_msg_.angular.z=0.5;
				}
				else
				{
  					vel_msg_.linear.x=0;
					vel_msg_.angular.z=1;
				} */
				if(ball_direct_distance<=0.6) 
				{
					ball_near_count++;
				}
				else
				{
					ball_near_count=0;				
				}
			}
			else
			{
				ball_lost_count++;
				if(ball_lost_count>40)
				{
					ball_lost_count=0;
					state_flag=-7;
				}				
			}
			ROS_INFO_STREAM("last_state="<<last_state<<" last_last_state"<<last_last_state);
			ROS_INFO_STREAM("state_flag<-6-6-6-6-6-6-6-6-6-6  go straight  -6-6-6-6-6-6-6-6-6-6>");
			ROS_INFO_STREAM("ball_direct_distance="<<ball_direct_distance);
			if(ball_near_count>20)	state_flag=0;
		current_state=-6;
		break;

		case -7://turn left
			if(current_state!=-7)
			{
				last_last_state=last_state;
				last_state=current_state;
			}
			vel_msg_.linear.x=0;
			vel_msg_.angular.z=1;		
			pub_cmd_vel_.publish(vel_msg_);
			ROS_INFO_STREAM("last_state="<<last_state<<" last_last_state"<<last_last_state);
			ROS_INFO_STREAM("state_flag<-7-7-7-7-7-7-7-7-7 turn left  -7-7-7-7-7-7-7-7-7-7-7>");
			ROS_INFO_STREAM("angle_z="<<angle_z);
			if(angle_z>60) state_flag=-8;
			if(mymsg.ball_detected==1) state_flag=-6;
		current_state=-7;
		break;

		case -8://turn right
			if(current_state!=-8)
			{
				last_last_state=last_state;
				last_state=current_state;
			}
			vel_msg_.linear.x=0;
			vel_msg_.angular.z=-1;		
			pub_cmd_vel_.publish(vel_msg_);
			ROS_INFO_STREAM("last_state="<<last_state<<" last_last_state"<<last_last_state);
			ROS_INFO_STREAM("state_flag<-8-8-8-8-8-8-8-8-8-8 turn right  -8-8-8-8-8-8-8-8-8-8-8>");
			ROS_INFO_STREAM("angle_z="<<angle_z);
			if(angle_z<-60) state_flag=-7;
			if(mymsg.ball_detected==1) state_flag=-6;
		current_state=-8;
		break;

		case -2://Attack the ball,put the ball
			ROS_INFO_STREAM("state_flag<-2-2-2-2-2-2-2-2  Attack  -2-2-2-2-2-2-2-2>");
			ROS_INFO_STREAM("mymsg.ball_detected="<<mymsg.ball_detected);
			ROS_INFO_STREAM("last_state="<<last_state<<"last_last_state"<<last_last_state);
			ROS_INFO_STREAM("odom_.pose.pose.position.x="<<odom_.pose.pose.position.x);
			ROS_INFO_STREAM("mymsg.ball_detected="<<mymsg.ball_detected);
			Attack();
/* 			vel_msg_.linear.x=0.22;
			vel_msg_.angular.z=0;		
			pub_cmd_vel_.publish(vel_msg_); */

			if(odom_.pose.pose.position.x>=1.2) state_flag=0;
		current_state=-2;
		last_state=-2;
		break;

		case -1://initial state
			vel_msg_.linear.x=0.22;
			vel_msg_.angular.z=0;		
			pub_cmd_vel_.publish(vel_msg_);
			ROS_INFO_STREAM("last_state="<<last_state<<" last_last_state"<<last_last_state);
			ROS_INFO_STREAM("state_flag<-1-1-1-1-1-1-1  go straight  -1-1-1-1-1-1-1-1>");
			ROS_INFO_STREAM("odom_.pose.pose.position.x="<<odom_.pose.pose.position.x);
			if(odom_.pose.pose.position.x>=0.1) state_flag=0;
		current_state=-1;
		last_state=-1;
		break;
			
		case 0://rotate
			if(current_state!=0)
			{
				last_last_state=last_state;
				last_state=current_state;
			}
			ROS_INFO_STREAM("last_state="<<last_state<<" last_last_state"<<last_last_state);
			ROS_INFO_STREAM("state_flag<00000000000000     rotate      0000000000000>");
			ROS_INFO_STREAM("mymsg.ball_detected="<<mymsg.ball_detected);
			ROS_INFO_STREAM("turn_radius*2*sin(angle_z/180*PI)="<<fabs(turn_radius*2*sin(angle_z/180*PI)));
			ROS_INFO_STREAM("ball_direct_distance="<<ball_direct_distance);
			vel_msg_.linear.x=0;
			vel_msg_.angular.z=-1;// put on the left(-1) or right(1) 		
			pub_cmd_vel_.publish(vel_msg_);
			if(mymsg.ball_detected==1)
			{
				if(angle_z<90 && angle_z>-90)
				{
					if(ball_direct_distance<=fabs(turn_radius*2*sin(angle_z/180*PI)))
					{
						state_flag=5;//go back
						current_x=odom_.pose.pose.position.x;
						current_y=odom_.pose.pose.position.y;
					}
					else
					{
						state_flag=1;
					}
				}
				else
				{
					state_flag=6;//defence state
				}				
			}
		current_state=0;
		break;

		case 1://attack
			if(current_state!=1)
			{
				last_last_state=last_state;
				last_state=current_state;
			}
			Attack();
			ROS_INFO_STREAM("last_state="<<last_state<<" last_last_state"<<last_last_state);
			ROS_INFO_STREAM("state_flag<11111111111111 Attack  1111111111111111>");
			ROS_INFO_STREAM("ball_detected="<<mymsg.ball_detected);
			ROS_INFO_STREAM("turn_radius*2*sin(angle_z/180*PI)="<<fabs(turn_radius*2*sin(angle_z/180*PI)));
			ROS_INFO_STREAM("ball_direct_distance="<<ball_direct_distance);
			ROS_INFO_STREAM("ball_lost_count="<<ball_lost_count);

			if(ball_direct_distance<=0.2 && ball_direct_distance>=0.05)//first 
			{
				current_x=odom_.pose.pose.position.x;
				current_y=odom_.pose.pose.position.y;
				state_flag=5;
			}

			if(ball_direct_distance<=fabs(turn_radius*2*sin(angle_z/180*PI)) && mymsg.ball_detected==1)//second
			{
				target_angle=2*angle_z;
				if(target_angle>180) target_angle-=360;
				if(target_angle<-180) target_angle+=360;
				state_flag=2;
			}

			//if lose the ball change state
			if(mymsg.ball_detected==0)
			{
				ball_lost_count++;
				if(ball_lost_count>40)
				{
					ball_lost_count=0;
					state_flag=0;
				}
			}
			else 
			{
				ball_lost_count=0;
			}
		current_state=1;		
		break;

		case 2://turn double angle_z
			if(current_state!=2)
			{
				last_last_state=last_state;
				last_state=current_state;
			}
			k_turn(target_angle);
			ROS_INFO_STREAM("last_state="<<last_state<<" last_last_state"<<last_last_state);
			ROS_INFO_STREAM("state_flag<22222222222222222222 turn 2222222222222222222222>");
			ROS_INFO_STREAM("angle_z="<<angle_z);
			ROS_INFO_STREAM("target_angle="<<target_angle);
			ROS_INFO_STREAM("fabs(target_angle-angle_z)="<<fabs(target_angle-angle_z));
			if(fabs(target_angle-angle_z)<=5)
			{
				state_flag=3;
				if(angle_z>=0)
					vel_msg_.angular.z=-1;
				else	
					vel_msg_.angular.z=1;
			}
				
		current_state=2;
		break;

		case 3://go circle
			if(current_state!=3)
			{
				last_last_state=last_state;
				last_state=current_state;
			}
			vel_msg_.linear.x=0.22;
			pub_cmd_vel_.publish(vel_msg_);
			ROS_INFO_STREAM("last_state="<<last_state<<" last_last_state"<<last_last_state);
			ROS_INFO_STREAM("state_flag<33333333333333333 go circle 3333333333333333>");
			ROS_INFO_STREAM("ball_detected="<<mymsg.ball_detected<<" angle_z="<<angle_z<<" camera_x_ball="<<mymsg.camera_x_ball);
			ROS_INFO_STREAM("ball_lost_count="<<ball_lost_count);
			
			if(angle_z>=-3 && angle_z<=3)
			{
				if(mymsg.camera_x_ball>100 && mymsg.camera_x_ball<=220) 
				{
					state_flag=4;
				}
				else
				{
					current_x=odom_.pose.pose.position.x;
					current_y=odom_.pose.pose.position.y;
					state_flag=5;
				}

				if(mymsg.ball_detected==0)
				{
					ball_lost_count++;
					if(ball_lost_count>10)
					{
						ball_lost_count=0;
						current_x=odom_.pose.pose.position.x;
						current_y=odom_.pose.pose.position.y;
						state_flag=5;
					}
				}
				else 
				{
					ball_lost_count=0;
				}
			}
			else
			{
				if(fabs(angle_z-last_angle_z)<1)
				{
					go_circle_count++;
					if(go_circle_count>40)
					{
						current_x=odom_.pose.pose.position.x;
						current_y=odom_.pose.pose.position.y;
						state_flag=5;
						go_circle_count=0;
					}
				}
				else
				{
					go_circle_count=0;
				}
			}
		last_angle_z=angle_z;
		current_state=3;
		break;

		case 4://push
			if(current_state!=4)
			{
				last_last_state=last_state;
				last_state=current_state;
			}	
			Attack();
			ROS_INFO_STREAM("last_state="<<last_state<<" last_last_state"<<last_last_state);
			ROS_INFO_STREAM("state_flag<44444444444444444 Push 44444444444444444444>");
			ROS_INFO_STREAM("ball_detected="<<mymsg.ball_detected);
			ROS_INFO_STREAM("ball_lost_count="<<ball_lost_count);
			if(mymsg.ball_detected==0)
			{
				ball_lost_count++;
				if(ball_lost_count>40)
				{
					ball_lost_count=0;
					current_x=odom_.pose.pose.position.x;
					current_y=odom_.pose.pose.position.y;
					state_flag=5;
				}
			}
			else 
			{
				if(fabs(angle_z-last_angle_z)<1 && fabs(angle_z)>60)
				{
					attack_count++;
					if(attack_count>20)
					{
						current_x=odom_.pose.pose.position.x;
						current_y=odom_.pose.pose.position.y;
						state_flag=5;
						attack_count=0;
					}
				}
				else
				{
					attack_count=0;
				}
				ball_lost_count=0;
			}
		last_angle_z=angle_z;
		current_state=4;	
		break;

		case 5://lose the ball,or two close but angle is error,so go back
			if(current_state!=5)
			{
				last_last_state=last_state;
				last_state=current_state;
			}
			ROS_INFO_STREAM("last_state="<<last_state<<" last_last_state"<<last_last_state);
			ROS_INFO_STREAM("state_flag<555555555555555555 go back 55555555555555555555>");
			ROS_INFO_STREAM("state_count="<<state_count);
			ROS_INFO_STREAM("state_count2="<<state_count2);
			ROS_INFO_STREAM("go distance ="<<fabs(sqrt(pow(odom_.pose.pose.position.x-current_x,2) + pow(odom_.pose.pose.position.y-current_y,2))));
			go_back();
			if(fabs(sqrt(pow(odom_.pose.pose.position.x-current_x,2) + pow(odom_.pose.pose.position.y-current_y,2)))>0.2)
			{
				state_flag=0;
			}
			if(mymsg.ball_detected==1)
			{
				if(last_state==4 && angle_z<10 && angle_z>-10)
					state_flag=4;
			}

			//avoid the dead circle:'0->1->5->0'
			if(last_state==1 && last_last_state==0)
			{
				state_count++;
				if(state_count>=2)
				{
					state_flag=4;
					state_count=0;
				}					
			}
			else
			{
				state_count=0;
			}

			//avoid the dead circle:'0->5'
			if(last_state==0)
			{
				state_count2++;
				if(state_count2>=2)
				{
					state_count2=0;
					state_flag=4;
				}
			}
			else
			{
				state_count2=0;
			}
			
		current_state=5;
		break;

		case 6://defence attack
			if(current_state!=6)
			{
				last_last_state=last_state;
				last_state=current_state;
			}
			ROS_INFO_STREAM("last_state="<<last_state<<" last_last_state"<<last_last_state);
			ROS_INFO_STREAM("state_flag<66666666666666 Defence-Attack 666666666666666>");
			ROS_INFO_STREAM("ball_direct_distance="<<ball_direct_distance);
			if(ball_direct_distance<=0.476)
			{
				if(angle_z<0)
				{
					target_angle2=angle_z+270;
					if(target_angle2>180)
					target_angle2=target_angle2-360;
				}
				else
				{
					target_angle2=angle_z-270;
					if(target_angle2<-180)
					target_angle2=target_angle2+360;
				}
				state_flag=7;
			}
			Attack();
		current_state=6;
		break;

		case 7://Defence-turn
			if(current_state!=7)
			{
				last_last_state=last_state;
				last_state=current_state;
			}
			k_turn(target_angle2);
			ROS_INFO_STREAM("last_state="<<last_state<<" last_last_state"<<last_last_state);
			ROS_INFO_STREAM("state_flag<77777777777777777 Defence-turn 777777777777777>");
			ROS_INFO_STREAM("angle_z="<<angle_z);
			ROS_INFO_STREAM("target_angle2="<<target_angle2);
			ROS_INFO_STREAM("fabs(target_angle2-angle_z)="<<fabs(target_angle2-angle_z));
			if(fabs(target_angle2-angle_z)<=5) 
			{
				state_flag=8;
				current_x=odom_.pose.pose.position.x;
				if(angle_z>=0)
					vel_msg_.angular.z=1.3;//1.3
				else	
					vel_msg_.angular.z=-1.3;
			}	
		current_state=7;
		break;

		case 8://Defence-go circle
			if(current_state!=8)
			{
				last_last_state=last_state;
				last_state=current_state;
			}
			ROS_INFO_STREAM("last_state="<<last_state<<" last_last_state"<<last_last_state);
			ROS_INFO_STREAM("state_flag<888888888888 Defence-go circle 88888888888888888>");
			ROS_INFO_STREAM("fabs(angle_z-90)="<<fabs(angle_z-90)<<" fabs(angle_z+90)="<<fabs(angle_z+90));
			if(current_x-odom_.pose.pose.position.x>0.1)
			{
				if(fabs(angle_z-90)<3 || fabs(angle_z+90)<3)
					state_flag=0;
			}
			vel_msg_.linear.x=0.22;
			pub_cmd_vel_.publish(vel_msg_);

			if(fabs(angle_z-last_angle_z)<1)
			{
				go_circle_count++;
				if(go_circle_count>20)
				{
					current_x=odom_.pose.pose.position.x;
					current_y=odom_.pose.pose.position.y;
					state_flag=5;
					go_circle_count=0;
				}
			}
			else
			{
				go_circle_count=0;
			}
		last_angle_z=angle_z;
		current_state=8;
		break;
	}
}

int main(int argc,char ** argv)
{
	ros::init(argc,argv,"motion");
	Robot robot;
	ros::Rate rate(20);//control rate is 10Hz
	while(ros::ok())
	{
		ros::spinOnce();//first sub the msg
/* 		ROS_INFO_STREAM("x="<<robot.mymsg.x_ball);
		ROS_INFO_STREAM("y="<<robot.mymsg.y_ball);

		ROS_INFO_STREAM("sin="<<sin(robot.angle_z/180.0*PI));
		robot.Attack();
		robot.control1();//then control */
		robot.control2();//then control
		//robot.vel_msg_.linear.x=0.22;
		//robot.vel_msg_.angular.z=1;
/* 		robot.pub_cmd_vel_.publish(robot.vel_msg_);

		ROS_INFO_STREAM("state_flag---------------------------->"<<robot.state_flag);
		ROS_INFO_STREAM("angle_z="<<robot.angle_z<<" camera_x_ball="<<robot.mymsg.camera_x_ball);
		ROS_INFO_STREAM("atan2="<<atan2(-1,1)/PI*360); */
		rate.sleep();	
	}
}
