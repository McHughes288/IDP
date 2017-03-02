#include <iostream>
#include <robot_link.h>
#include <robot_instr.h>
#include <stopwatch.h>
#include <string>
#include <stdio.h>
#include <delay.h>

#include "global.h"
#include "movement.h"
#include "navigation.h"

using namespace std;

extern robot_link rlink;
extern stopwatch watch;

//Global variables

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


bool follow_line()
{
	// Sensor values as bits
	const int s_rear = bit3;	// rear sensor bit
	const int s_left = bit2;	//left sensor bit
	const int s_middle = bit1;	//middle sensor bit 
	const int s_right = bit0;	// right sensor bit
	
	int speed_m1; //motor 1 -> left
	int speed_m2; //motor 2 -> right
	int port_value;
	
	const int k1 = 15;	//constants to be added or subtracted to motor speed to change direction
	const int k2 = 20;
	
	bool junction_flag = false; 

	speed_m1 = 100;
	speed_m2 = 100;
	
	move_robot(speed_m1,speed_m2,255);
	
	stopwatch line_following_watch;	// make a new stopwatch class to keep previous one working;
	line_following_watch.start();
	
	while(1) // execute until junction, junction check not yet implemented
	{
		if(line_following_watch.read() > 60000)
		{
			cout << " Follow line routine active for 60 seconds" << endl;
			return false; //stop after 20 seconds for now
		}
		/*if(speed_m2 > 127)
		{
			speed_m2  = 100;
			speed_m1 = 100;
			cerr << "The speed of motor 2 was greater than 127" << endl;
		}
		if(speed_m2 > 127)
		{
			speed_m2 = 127;
		}

		if(speed_m1 > 127)
		{
			speed_m1 = 127;
		}
		
		if(speed_m1 > 127)
		{
			speed_m2  = 100;
			speed_m1 = 100;
			cerr << "The speed of motor 1 went positive" << endl;;			
		}*/
		cout << "Port value: ";
		print_binary_8_bit(port_value);
		
		cout << "Speed_m1 = " << speed_m1 << " , and speed_m2 = " << speed_m2 << endl << endl;
		move_robot(speed_m1,speed_m2,0);
		delay(5);
		
		speed_m1 = 100;
		speed_m2 = 100;
		
		port_value = rlink.request(READ_PORT_4);
		
		if(port_value bitand s_rear && junction_flag)
		{
			break;
		}
		
		//1 1 1
		else if(port_value bitand s_left && port_value bitand s_right && port_value bitand s_middle) // all sensors actice -> junction
		{
			junction_flag = true;	//if a junction is detected set the flag as true;
			continue;
		}
		
		// 1 1 0
		else if(port_value bitand s_middle && port_value bitand s_left) //robot is leaning to the right 
		{
			speed_m1  -= k1; //decrease speed of left motor
			speed_m2  += k1; //increase speed of right motor
			cout << "changed speed" << endl;

 			continue; 
		}
		
		// 0 1 1
		else if(port_value bitand s_middle && port_value bitand s_right) //robot is leaning to the left
		{
			speed_m1 += k1; //increase speed of left motor
			speed_m2 -= k1; //decrease speed of right motor
			cout << "changed speed" << endl;
			continue;
		}
		
		// 0 1 0
		else if(port_value bitand s_middle)
		{
			cout << "on line" << endl;
			//normal operation, continue normally
			continue;
		}
		
		 // 1 0 0
		else if(port_value bitand s_left) //robot is too far right
		{
			speed_m1 -= k2;
			speed_m2 += k2;
			continue;
		}
		
		// 0 0 1
		else if(port_value bitand s_right) //robot is too far left
		{
			speed_m1 += k2;
			speed_m2 -= k2;
			continue;
		}
	}
	move_robot(0,0,0);
	return true; // junction detected, returns true
}

//returns next junction
int calcuate_path(int start_location[2], int finish_location[2]);

//determine whether the robot should go straight,left or right at each junction
bool navigation(int current_location[2],int next_location[2], int bearing);
