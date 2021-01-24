#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Joy.h>
#include <math.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <linux/input-event-codes.h>

#include "origarm_uw/Command_ABL.h"
#include "origarm_uw/Seg_ABL.h"
#include "origarm_uw/Command_Position.h"
#include "origarm_uw/keynumber.h"
#include "origarm_uw/modenumber.h"
#include "origarm_uw/segnumber.h"

#include "myData.h"

using namespace std;

//save into files
static int save_flag = 0;
string trajPointFileName = "";
ofstream savedata;

//joystick mapping
static float joyLx;
static float joyLy;
static float joyRx;
static float joyRy;
static float joyLT;
static float joyRT;
static int joyLB; 
static int joyRB; 
static int joyX;  
static int joyY;  //mode[2]
static int joyA;  //mode[0]
static int joyB;  //mode[1]
static int joyCrossY;
static int joyCrossX;
static int joyUp;
static int joyDown;
static int joyLeft;
static int joyRight;

static int last_joyRB;
static int last_joyLB;
static int last_joyX;
static int last_joyY;
static int last_joyA;
static int last_joyB;
static int last_joyUp;
static int last_joyDown;
static int last_joyLeft;
static int last_joyRight;

static int enable;
static int disable;
static int last_enable;
static int last_disable;
static int status;

//Write ABL
int segNumber;
float alpha;
float beta;
float length = length0;

float seg_alpha[2];
float seg_beta[2];
float seg_length[2] = {length0, length0};

float seg_alpha_[4];
float seg_beta_[4];
float seg_length_[4] = {length0, length0, length0, length0};

static float a_scale = 0.003;
static float b_scale = 0.002;
static float l_scale = 0.00003;

//control mode
//mode[0]: 1 abl; mode[1]: 2 abl; mode[2]: 4 abl; 
//mode[0]: joyA;  mode[1]: joyB;  mode[2]: joyY;  
int mode;

std::string getTimeString()
{
    time_t rawtime;
    struct tm *timeinfo;
    char buffer[100];

    time(&rawtime);
    timeinfo = localtime(&rawtime);
	struct timeval time_now{};
	gettimeofday(&time_now,nullptr);
	time_t msecs_time=time_now.tv_usec;
	time_t msec=(msecs_time/1000)%1000;

    strftime(buffer, 100, "%G_%h_%d_%H_%M_%S", timeinfo);
    std::string ret = buffer;
	//std::string ret=ret1+"_"+std::to_string(msec);
    return ret;
}

//keyboard callback
void keyCallback(const origarm_uw::keynumber &key)
{
	int i = 0;
	switch (key.keycodePressed)
	{
		case KEY_0:
			segNumber = 0;
			break;
		case KEY_1:
			segNumber = 1;
			break;
		case KEY_2:
			segNumber = 2;
			break;
		case KEY_3:
			segNumber = 3;
			break;
		case KEY_4:
			segNumber = 4;
			break;
		case KEY_5:
			segNumber = 5;
			break;
		case KEY_6:
			segNumber = 6;
			break;
		case KEY_7:
			segNumber = 7;
			break;
		case KEY_8:
			segNumber = 8;
			break;
		case KEY_9:
			segNumber = 9;
			break;
		}
}

//joystick callback
void joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
	//joystick mapping
	joyLx = joy->axes[0];
	joyLy = joy->axes[1];
	joyRx = joy->axes[3];
	joyRy = joy->axes[4];
	joyLT = joy->axes[2];
	joyRT = joy->axes[5];

	//enable = joy->buttons[0];
	disable = joy->buttons[6]; //back

	joyA = joy->buttons[0];
	joyB = joy->buttons[1];
	joyX = joy->buttons[2];
	joyY = joy->buttons[3];
	joyLB = joy->buttons[4];
	joyRB = joy->buttons[5];

	joyCrossX = joy->axes[6];
	joyCrossY = joy->axes[7];

	if (joyCrossX == 1)
	{
		joyLeft = 1;
	}
	else if (joyCrossX == -1)
	{
		joyRight = 1;
	}
	else
	{
		joyLeft = 0;
		joyRight = 0;
	}

	if (joyCrossY == 1)
	{
		joyUp = 1;
	}
	else if (joyCrossY == -1)
	{
		joyDown = 1;
	}
	else
	{
		joyUp = 0;
		joyDown = 0;
	}

	if (joyA == 1 && last_joyA == 0)
	{
		mode = 0;
	}
	else if (joyB == 1 && last_joyB == 0)
	{
		mode = 1;
	}
	else if (joyY == 1 && last_joyY == 0)
	{
		mode = 2;
	}

	//save into files
	if (joyLB == 1 && last_joyLB == 0)
	{
		save_flag = 1;		
	}
	else
	{
		save_flag = 0;
	}

	//only when joyRT && joyLT pressed together, joystick starts to control
	if (joyLT == -1 && joyRT == -1)
	{
		enable = 1;
	}
	else
	{
		enable = 0;
	}

	if (enable == 1 && last_enable == 0)
	{
		status = 1;
	}
	else if (disable == 1 && last_disable == 0)
	{
		status = 0;
	}

	last_enable = enable;
	last_disable = disable;

	last_joyA = joyA;
	last_joyB = joyB;
	last_joyX = joyX;
	last_joyY = joyY;
	last_joyUp = joyUp;
	last_joyDown = joyDown;
	last_joyLeft = joyLeft;
	last_joyRight = joyRight;
	last_joyRB = joyRB;
	last_joyLB = joyLB;
}

float constrain2PI(float s)
{
	if (s > M_PI)
	{
		s = s - 2 * M_PI;
	}
	else if (s < -M_PI)
	{
		s = s + 2 * M_PI;
	}

	return s;
}

//Joystick->ABL (joyLy->alpha, joyRx->beta, joyLT & joyRT->length), for each mode, the joystick only controls one segment
void writeABL()
{
	if (mode == 0)
	{
		if (joyLy > 0.05)
		{
			alpha = alpha + a_scale;
		}
		else if (joyLy < -0.05)
		{
			alpha = alpha - a_scale;
		}

		if (joyRx > 0.05)
		{
			if (joyRx > 0.5)
			{
				beta = beta + 2 * b_scale;
			}
			else
			{
				beta = beta + b_scale;
			}
		}
		else if (joyRx < -0.05)
		{
			if (joyRx < -0.5)
			{
				beta = beta - 2 * b_scale;
			}
			else
			{
				beta = beta - b_scale;
			}
		}

		if (joyLT != 1 && joyRT != 1)
		{
		}
		else if (abs(joyLT - 1) > 0.05)
		{
			if (abs(joyLT - 1) > 1)
			{
				length = length + 2 * l_scale;
			}
			else
			{
				length = length + l_scale;
			}
		}
		else if (abs(joyRT - 1) > 0.05)
		{
			if (abs(joyRT - 1) > 0.05)
			{
				if (abs(joyRT - 1) > 1)
				{
					length = length - 2 * l_scale;
				}
				else
				{
					length = length - l_scale;
				}
			}
		}

		beta = constrain2PI(beta);
		alpha = CONSTRAIN(alpha, alphamin, alphamax);
		length = CONSTRAIN(length, lengthmin, lengthmax);
	}
	else if (mode == 1)
	{			
		if (joyLy > 0.05)
		{
			seg_alpha[segNumber] = seg_alpha[segNumber] + a_scale;
		}
		else if (joyLy < -0.05)
		{
			seg_alpha[segNumber] = seg_alpha[segNumber] - a_scale;
		}

		if (joyRx > 0.05)
		{
			if (joyRx > 0.5)
			{
				seg_beta[segNumber] = seg_beta[segNumber] + 2 * b_scale;
			}
			else
			{
				seg_beta[segNumber] = seg_beta[segNumber] + b_scale;
			}
		}
		else if (joyRx < -0.05)
		{
			if (joyRx < -0.5)
			{
				seg_beta[segNumber] = seg_beta[segNumber] - 2 * b_scale;
			}
			else
			{
				seg_beta[segNumber] = seg_beta[segNumber] - b_scale;
			}
		}

		if (joyLT != 1 && joyRT != 1)
		{
		}
		else if (abs(joyLT - 1) > 0.05)
		{
			if (abs(joyLT - 1) > 1)
			{
				seg_length[segNumber] = seg_length[segNumber] + 2 * l_scale;
			}
			else
			{
				seg_length[segNumber] = seg_length[segNumber] + l_scale;
			}
		}
		else if (abs(joyRT - 1) > 0.05)
		{
			if (abs(joyRT - 1) > 0.05)
			{
				if (abs(joyRT - 1) > 1)
				{
					seg_length[segNumber] = seg_length[segNumber] - 2 * l_scale;
				}
				else
				{
					seg_length[segNumber] = seg_length[segNumber] - l_scale;
				}
			}
		}
	
		for (int i = 0; i < 2; i++)
		{
			seg_beta[i] = constrain2PI(seg_beta[i]);
			seg_alpha[i] = CONSTRAIN(seg_alpha[i], alphamin, alphamax);
			seg_length[i] = CONSTRAIN(seg_length[i], lengthmin, lengthmax);
		}
	}			
	else if (mode == 2)
	{
		if (joyLy > 0.05)
		{
			seg_alpha_[segNumber] = seg_alpha_[segNumber] + a_scale;
		}
		else if (joyLy < -0.05)
		{
			seg_alpha_[segNumber] = seg_alpha_[segNumber] - a_scale;
		}

		if (joyRx > 0.05)
		{
			if (joyRx > 0.5)
			{
				seg_beta_[segNumber] = seg_beta_[segNumber] + 2 * b_scale;
			}
			else
			{
				seg_beta_[segNumber] = seg_beta_[segNumber] + b_scale;
			}
		}
		else if (joyRx < -0.05)
		{
			if (joyRx < -0.5)
			{
				seg_beta_[segNumber] = seg_beta_[segNumber] - 2 * b_scale;
			}
			else
			{
				seg_beta_[segNumber] = seg_beta_[segNumber] - b_scale;
			}
		}

		if (joyLT != 1 && joyRT != 1)
		{
		}
		else if (abs(joyLT - 1) > 0.05)
		{
			if (abs(joyLT - 1) > 1)
			{
				seg_length_[segNumber] = seg_length_[segNumber] + 2 * l_scale;
			}
			else
			{
				seg_length_[segNumber] = seg_length_[segNumber] + l_scale;
			}
		}
		else if (abs(joyRT - 1) > 0.05)
		{
			if (abs(joyRT - 1) > 0.05)
			{
				if (abs(joyRT - 1) > 1)
				{
					seg_length_[segNumber] = seg_length_[segNumber] - 2 * l_scale;
				}
				else
				{
					seg_length_[segNumber] = seg_length_[segNumber] - l_scale;
				}
			}
		}
	
		for (int i = 0; i < 4; i++)
		{
			seg_beta_[i] = constrain2PI(seg_beta_[i]);
			seg_alpha_[i] = CONSTRAIN(seg_alpha_[i], alphamin, alphamax);
			seg_length_[i] = CONSTRAIN(seg_length_[i], lengthmin, lengthmax);
		}
	}	
}

void Init_parameter()
{
	alpha  = 0;
	beta   = 0;
	length = length0;

	for (int i = 0; i < 2; i++)
	{
		seg_alpha[i]  = 0;
		seg_beta[i]   = 0;
		seg_length[i] = length0;
	}

	for (int i = 0; i < 4; i++)
	{
		seg_alpha_[i]  = 0;
		seg_beta_[i]   = 0;
		seg_length_[i] = length0;
	}
}


void help_menu()
{
	cout << "===============================================================================" << endl;
	cout << "                              JOYSCTIK USER MENU                               " << endl;
	cout << "-------------------------------------------------------------------------------" << endl;
	cout << "           MODE         |"
		 << " KEY |                    KEYCODE                 " << endl;
	cout << " 0: 1 segment abl       |"
		 << " A   |"
		 << "  alpha  +: joyLy +, alpha  -: joyLy -; " << endl;
	cout << " 1: 2 segment abl       |"
		 << " B   |"
		 << "  beta   +: joyRx +, beta   -: joyRx -; " << endl;
	cout << " 2: 4 segment abl       |"
		 << " Y   |"
		 << "  length +: joyLT  , length -: joyRT  ; " << endl;
	cout << "                                                                               " << endl;
	cout << " save data into file    |"
		 << " LB  |"
		 << "                                        " << endl;
	cout << "                                                                               " << endl;
	cout << "===============================================================================" << endl;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "joystick");
	ros::NodeHandle nh;
	ros::Rate r(100); //Hz
	
	ros::Subscriber sub1 = nh.subscribe("joy", 1, joyCallback);
	ros::Subscriber sub2 = nh.subscribe("key_number", 1, keyCallback);
	ros::Publisher pub1 = nh.advertise<origarm_uw::Command_ABL>("Cmd_ABL_joy", 100);
	ros::Publisher pub2 = nh.advertise<origarm_uw::modenumber>("modenumber", 100);
	ros::Publisher pub3 = nh.advertise<origarm_uw::segnumber>("segnumber", 100);

	trajPointFileName = "trajp_" + getTimeString();
	savedata.open("/home/ubuntu/catkin_ws/src/origarm_uw/data/" + trajPointFileName + ".txt", ios::trunc);

	Init_parameter();

	while (ros::ok())
	{
		//print help menu
		help_menu();

		//check whether joystick is available
		if (status == 1)
		{
			//control mode
			//mode[0]: 1 abl; mode[1]: 2 abl; mode[2]: 4 abl; 
			//mode[0]: joyA;  mode[1]: joyB;  mode[2]: joyY; 
			writeABL();			
		}
		else if (status == 0)
		{
			//Reset
			Init_parameter();
			//ROS_INFO("status: %d", status);
		}

		origarm_uw::Command_ABL Cmd_ABL;
		origarm_uw::modenumber modenumber;
		modenumber.modeNumber = mode;
		modenumber.status = status;

		origarm_uw::segnumber segnumber;
		segnumber.segmentNumber = segNumber;

		if (mode == 0)
		{
			for (int i = 0; i < 4; i++)
			{
				Cmd_ABL.segment[i].A = alpha*4;
				Cmd_ABL.segment[i].B = beta;
				Cmd_ABL.segment[i].L = length*4;
			}

			printf("ABL1: alpha: %f, beta: %f, length: %f\r\n", alpha, beta, length);

			pub1.publish(Cmd_ABL);
		}
		else if (mode == 1)
		{
			for (int i = 0; i < 2; i++)
			{
				Cmd_ABL.segment[i].A = seg_alpha[0]*2;
				Cmd_ABL.segment[i].B = seg_beta[0];
				Cmd_ABL.segment[i].L = seg_length[0]*2;				
			}
			for (int i = 2; i < 4; i++)
			{
				Cmd_ABL.segment[i].A = seg_alpha[1]*2;
				Cmd_ABL.segment[i].B = seg_beta[1];
				Cmd_ABL.segment[i].L = seg_length[1]*2;	
			}

			for (int i = 0; i < 2; i++)
			{
				printf("ABL2: alpha[%d]: %f, beta[%d]: %f, length[%d]: %f\r\n", i, seg_alpha[i], i, seg_beta[i], i, seg_length[i]);
			}
			
			pub1.publish(Cmd_ABL);
		}
		else if (mode == 2)
		{
			for (int i = 0; i < 4; i++)
			{
				Cmd_ABL.segment[i].A = seg_alpha_[i];
				Cmd_ABL.segment[i].B = seg_beta_[i];
				Cmd_ABL.segment[i].L = seg_length_[i];

				printf("ABL4: alpha[%d]: %f, beta[%d]: %f, length[%d]: %f\r\n", i, seg_alpha_[i], i, seg_beta_[i], i, seg_length_[i]);
			}

			pub1.publish(Cmd_ABL);
		}		

		//write into files
		if (save_flag == 1)
		{			
			if (mode == 0)
			{
				savedata << 0 << " " << 10000 << " " << 10000 << " " << 10000 
				         << " " << alpha << " " << beta << " " << length
						 << " " << alpha << " " << beta << " " << length
						 << " " << alpha << " " << beta << " " << length
						 << " " << alpha << " " << beta << " " << length
						 << endl;
				printf("%s\n", "write mode 0 data, 1 segment");
			}
			else if (mode == 1)
			{
				savedata << 1 << " " << 10000 << " " << 10000 << " " << 10000 
						 << " " << seg_alpha[0] << " " << seg_beta[0] << " " << seg_length[0]
						 << " " << seg_alpha[0] << " " << seg_beta[0] << " " << seg_length[0]
						 << " " << seg_alpha[1] << " " << seg_beta[1] << " " << seg_length[1]
						 << " " << seg_alpha[1] << " " << seg_beta[1] << " " << seg_length[1]					
						 << endl;
				printf("%s\n", "write mode 1 data, 2 segments");
			}
			else if (mode == 2)
			{
				savedata << 2 << " " << 10000 << " " << 10000 << " " << 10000 
						 << " " << seg_alpha_[0] << " " << seg_beta_[0] << " " << seg_length_[0]
						 << " " << seg_alpha_[1] << " " << seg_beta_[1] << " " << seg_length_[1]
						 << " " << seg_alpha_[2] << " " << seg_beta_[2] << " " << seg_length_[2]
						 << " " << seg_alpha_[3] << " " << seg_beta_[3] << " " << seg_length_[3]						
						 << endl;

				printf("%s\n", "write mode 2 data, 4 segments");
			}
		}

		pub2.publish(modenumber);
		pub3.publish(segnumber);

		ros::spinOnce();
		r.sleep(); //sleep for 1/r sec
			
	}

	savedata.close();
	return 0;
}
