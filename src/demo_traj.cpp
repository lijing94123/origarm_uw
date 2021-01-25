#include <ros/ros.h>
#include <geometry_msgs/Pose.h> 
#include <math.h>
#include <time.h>
#include <fstream>
#include <iostream>
#include <linux/input-event-codes.h>

#include "origarm_uw/Command_Position_Arm.h"
#include "origarm_uw/Command_ABL_Arm.h"
#include "origarm_uw/modenumber.h"
#include "origarm_uw/segnumber.h"
#include "origarm_uw/keynumber.h"
#include <ros/time.h>
#include "ros/package.h"
#include "myData.h"

using namespace std;

int ts = 2000000;     //time sleep at each step
const int mt = 1000; //1ms
int tstep[]={1000*mt, 5000*mt, 5000*mt, 5000*mt, 5000*mt, 5000*mt, 5000*mt, 5000*mt};
int k = 0;
int repeat = 1;

int mode_in;
float x_in, y_in, z_in;
float a_in[4], b_in[4], l_in[4];

ifstream inFile;
string trajFilePath = "";
string trajFileName = "";

int flag_start = 0;

void keyCallback(const origarm_uw::keynumber &key)
{
	if (key.keycodePressed == KEY_B) // break and reset all command pressure to 0
	{		
		printf("KEY_B pressed!\r\n");
		flag_start = 0;
	}
	else if (key.keycodePressed == KEY_J) // same as saving button
	{
		printf("KEY_J pressed!\r\n");
		flag_start = 1;
	}	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "demo_traj");
	ros::NodeHandle nh;	
	ros::Rate r(100);     //Hz
		
	trajFilePath = ros::package::getPath("origarm_uw") + "/data/";
	trajFileName = "trajp_1.txt";
	inFile.open(trajFilePath + trajFileName, ios::in);

	if (!inFile)
	{
		printf("%s\n", "unable to open the file.");
		exit(1);
	}

	if (nh.getParam("repeat", repeat))
	{
		ROS_INFO("repeat is set to %d\r\n", repeat);
	}
	else
	{
		repeat = 1;
	}

	ros::Publisher  pub1 = nh.advertise<origarm_uw::Command_ABL_Arm>("Cmd_ABL_joy", 100);
	ros::Publisher  pub2 = nh.advertise<origarm_uw::modenumber>("modenumber", 100);	
	ros::Subscriber  sub1 = nh.subscribe("key_number", 1, keyCallback);

	while (ros::ok())
	{
		origarm_uw::modenumber moden;
		origarm_uw::Command_ABL_Arm Command_ABL_demo;
		
		if (flag_start)
		{
			if (!inFile.eof() && repeat > 0)
			{				
				inFile>>mode_in>>x_in>>y_in>>z_in>>a_in[0]>>b_in[0]>>l_in[0]>>a_in[1]>>b_in[1]>>l_in[1]>>a_in[2]>>b_in[2]>>l_in[2]>>a_in[3]>>b_in[3]>>l_in[3];

				moden.modeNumber = 0;

				for (int i = 0; i < SEGMENTNUM; i++)
				{
					Command_ABL_demo.segment[i].A = a_in[i];
					Command_ABL_demo.segment[i].B = b_in[i];
					Command_ABL_demo.segment[i].L = l_in[i];
				}
				
				pub1.publish(Command_ABL_demo);
				pub2.publish(moden);
				
				printf("Receiving: %d, %f, %f, %f, %f, %f, %f, %f, %f, %f\r\n", mode_in, a_in[0], b_in[0], l_in[0], a_in[2], b_in[2], l_in[2], a_in[3], b_in[3], l_in[3]);
				usleep(tstep[k]);	
				k++;							
			}
			else if (repeat > 0)
			{
				inFile.clear();
				printf("eof:%d\n", inFile.eof());
				inFile.seekg(0, ios::beg);
				repeat = repeat - 1;
				k = 0;
			}
		}		
		else
		{
			moden.modeNumber = 0;

			for (int i = 0; i < SEGMENTNUM; i++)
			{
				Command_ABL_demo.segment[i].A = 0;
				Command_ABL_demo.segment[i].B = 0;
				Command_ABL_demo.segment[i].L = length0;
			}
			
			pub1.publish(Command_ABL_demo);
			pub2.publish(moden);
		}

		ros::spinOnce();
		r.sleep();
	}

	inFile.close();	
	return 0;
}
