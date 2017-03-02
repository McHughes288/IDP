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

extern const int bit0;
extern const int bit1;
extern const int bit2;
extern const int bit3;
extern const int bit4;
extern const int bit5;
extern const int bit6;
extern const int bit7;

void move_robot(int speed_m1, int speed_m2, int ramp_time = 0)
{
	speed_m1 = -speed_m1; // due to orientation of motors speed_m1 must be negative to go forward
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
	

	//motor 1 -> left
	//motor 2 -> right
	rlink.command(RAMP_TIME, ramp_time);
	rlink.command(MOTOR_1_GO, speed_m1);
	rlink.command(MOTOR_2_GO, speed_m2);
} 

bool turn_robot(int angle) {
	
	int speed_m1;
	int speed_m2;
	int angle_difference = angle - current_bearing;
	int port_value;
	const int s_rear = bit3;
	int state = 0; // 0 is when rear sensor is active before turning
					// 1 is when the rear sensor isnt active as the robot is turning
					// 2 is when the rear sensor is active again and the robot has completed a 90 degree turn
	
	// Set motor speeds to turn right if angle difference is positive
	if (angle_difference > 0) {
		speed_m1 = 100;
		speed_m2 = -100;
		
	// Set motor speeds to turn left if angle difference is negative
	} else if (angle_difference < 0) {
		speed_m1 = -100;
		speed_m2 = 100;
		
	} else {
		speed_m1 = 0;
		speed_m2 = 0;
	}
	
	// Update current bearing
	current_bearing = current_bearing + angle_difference;
	
	
	port_value = rlink.request(READ_PORT_4);
	
	move_robot(speed_m1, speed_m2);
	
	while(state != 2) {
	
		//robot is over the junction
		if(port_value bitand s_rear && state == 0) {
			cout << "Still on junction" << endl;
		
			
		} else if (!(port_value bitand s_rear)) {
			cout << "Off junction, turning..." << endl;
			state = 1;
			
		} else if (port_value bitand s_rear && state == 1) {
			stop_robot();
			cout << "Stopping Turning" << endl;
			state = 2;
		}
	}
	
	return true;
	
}

bool stop_robot() {
	move_robot(0, 0);
	return true;
}

