#include <iostream>
#include <robot_link.h>
#include <robot_instr.h>
#include <stopwatch.h>
#include <string>
#include <stdio.h>
#include <delay.h>

#include "global.h"

using namespace std;


void move_robot(int speed_m1, int speed_m2, int ramp_time = 0)
{
	//ramp time connot be negative or greater than 255

	if(ramp_time < 0)
	{
		ramp_time = 0;
	}
	else if(ramp_time > 255)
	{
		ramp_time = 254; //255 is the default so if we pass anything greater use the greatest 
	}					 //user defined ramp time

	if(speed_m1 < 0)	//negative speed are from 128-255 which is reverse
	{
		speed_m1 = -speed_m1 + 128;
		
		if(speed_m1 > 255)
		{
			speed_m1 = 255;
			cout << "invalid speed_m1 entered in move_robot(>255)" << endl;
		}
		else if(speed_m1 < 0)
		{
			speed_m1 = 0;
			cout << "invalid speed_m1 entered in move_robot (<<0)" << endl;
		}
	}

	if(speed_m2 < 0)
	{
		speed_m2 = -speed_m2 + 128;
		
		if(speed_m2 > 255)
		{
			speed_m2 = 255;
			cout << "invalid speed_m1 entered in move_robot(>255)" << endl;
		}
		else if(speed_m2 < 0)
		{
			speed_m2 = 0;
			cout << "invalid speed_m1 entered in move_robot (<<0)" << endl;
		}
	}
	

	rlink.command(RAMP_TIME, ramp_time);
	rlink.command(MOTOR_1_GO, speed_m1);
	rlink.command(MOTOR_2_GO, speed_m2);
} 

bool turn_robot(int angle);

bool stop_robot();
