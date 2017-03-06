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
	int speed = 50;
	int angle_difference = angle - current_bearing;
	if(angle_difference > 180)
	{
		angle_difference -= 360;
	}
	if(angle_difference < -180)
	{
		angle_difference += 360;
	}
	int counter;
	int times;
	bool flag_180 = false;
	
	
	int port_value;
	//const int s_rear = bit3;	// rear sensor bit
	const int s_left = bit2;	//left sensor bit
	const int s_middle = bit1;	//middle sensor bit 
	const int s_right = bit0;	// right sensor bit
	
	
	int state = 0; // 0 is when rear sensor is active before turning
					// 1 is when the rear sensor isnt active as the robot is turning
					// 2 is when the rear sensor is active again and the robot has completed a 90 degree turn
					

	// Set motor speeds to turn right if angle difference is positive
	if (angle_difference > 0) {
		cout << "turning left" << endl;
		speed_m1 = speed;
		speed_m2 = -speed;
		
	// Set motor speeds to turn left if angle difference is negative
	} else if (angle_difference < 0) {
		cout << "turning right" << endl;
		speed_m1 = -speed;
		speed_m2 = speed;
		
	} else {
		speed_m1 = 0;
		speed_m2 = 0;
	}
	
	if(angle_difference == 180 || angle_difference == -180)
	{
		times = 2;
		counter = 1;
		flag_180 = true;
	}
	else
	{
		times = 1;
		counter = 1;
	}
	
	// Update current bearing
	current_bearing = current_bearing + angle_difference;
	/*if(current_bearing >= 360)
	{
		current_bearing = current_bearing - 360;
	}
	if(current_bearing < 0)
	{
		current_bearing += 360;
	}*/
	
	port_value = rlink.request(READ_PORT_4);
	
	move_robot(speed_m1, speed_m2);
	
	cout << "Starting the turn" << endl;
	while(state != 2 && (times != counter)&flag_180) {
		
		port_value = rlink.request(READ_PORT_4);

		//robot is no longer over the junction
		if(port_value bitand s_middle)
		{
			continue;
		} else {
			state = 1;
			continue;
		}
		
		
		
		if (angle_difference > 0) {
			// robot has completed the RIGHT turn (checks left sensor)
			if (port_value bitand s_left && state == 1) {
				
				
				if(counter < times)
				{
					counter ++;
					state = 0;
				}
				else
				{
					stop_robot();
					state = 2;
					break;
				}
			}
		} else if (angle_difference < 0) {
			// robot has completed the LEFT turn (checks right sensor)
			if (port_value bitand s_right && state == 1) {

				
				if(counter < times)
				{
					
					counter ++;
					state = 0;
				}
				else
				{
					stop_robot();
					state = 2;
					break;
				}
		
			}
		} else {
			break;
		}
		
	}
	cout << "Stopped Turn" << endl;
	//stop_robot();
	return true;
	
}

bool stop_robot() {
	move_robot(0, 0);
	return true;
}

