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
	int speed_m1; //motor 1 -> left
	int speed_m2; //motor 2 -> right
	int port_value;
	
	const int k1 = 3;
	const int k2 = 5;

	speed_m1 = -100;
	speed_m2 = 100;
	
	move_robot(speed_m1,speed_m2,255);
	watch.start();
	
	while(1) // execute until junction, junction check not yet implemented
	{
		if(watch.read() > 20000)
		{
			break; //stop after 20 seconds for now
		}
		if(speed_m2 < 0)
		{
			speed_m2  = 50;
			speed_m1 = -50;
			cerr << "The speed of motor 2 went negative" << endl;
		}
		if(speed_m2 > 127)
		{
			speed_m2 = 127;
		}

		if(speed_m1 < -127)
		{
			speed_m1 = -127;
		}
		
		if(speed_m1 > 0)
		{
			speed_m2  = 50;
			speed_m1 = -50;
			cerr << "The speed of motor 1 went positive" << endl;;			
		}
		cout << "Port value: ";
		print_binary_8_bit(port_value);
		
		cout << "Speed_m1 = " << speed_m1 << " , and speed_m2 = " << speed_m2 << endl << endl;
		move_robot(speed_m1,speed_m2,0);
		delay(50);
		
		speed_m1 = -100;
		speed_m2 = 100;
		
		port_value = rlink.request(READ_PORT_4);
		
		if(port_value bitand bit1)
		{
			cout << "on line" << endl;
			//normal operation, continue normally
			continue;
		}
		
		else if(port_value bitand bit1 && port_value bitand bit2) //robot is leaning to the right 
		{
			speed_m1  -= k1; //decrease speed of left motor, since speed_m1 is negative add k1
			speed_m2  -= k1; //increase speed of right motor

 			continue; 
		}
		
		else if(port_value bitand bit1 && port_value bitand bit0) //robot is leaning to the left
		{
			speed_m2 += k1; //decrease speed of left motor, since speed_m1 is negative add k1+
			speed_m1 += k1; //increase speed of right mtotor
			continue;
		}
		
		else if(port_value bitand bit0) //robot is too far left
		{
			speed_m1 -= k2;
			speed_m2 -= k2;
			continue;
		}
		
		else if(port_value bitand bit2) //robot is too far right
		{
			speed_m2 += k2;
			speed_m1 += k2;
			continue;
		}
	}
	
	return true;
}

//returns next junction
int calcuate_path(int start_location[2], int finish_location[2]);

//determine whether the robot should go straight,left or right at each junction
bool navigation(int current_location[2],int next_location[2], int bearing);
