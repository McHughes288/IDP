#include <iostream>
#include <robot_link.h>
#include <robot_instr.h>
#include <stopwatch.h>
#include <string>
#include <stdio.h>
#include <delay.h>

#include "global.h"
#include "movement.h"

using namespace std;

extern robot_link rlink;
extern stopwatch watch;

extern int current_bearing;
extern int current_location[2];
extern int next_location[2];
extern int pallets_picked_up;
extern int mode;
extern int pallets_delivered; 
extern int current_pallet_colour;
extern int order_of_pallets_on_conveyor[6];

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

bool turn_robot(int angle) {
	
	int speed;
	int angle_difference = angle - current_bearing;
	
	if (angle_difference < 0) {
		speed = -100;
	} else if (angle_difference > 0) {
		speed = 100;
	} else {
		speed = 0;
	}
	
	return true;
	
}

bool stop_robot();

