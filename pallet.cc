#include <iostream>
#include <robot_link.h>
#include <robot_instr.h>
#include <stopwatch.h>
#include <string>
#include <stdio.h>
#include <delay.h>
#include <fstream>
#include <string>
#include <cmath>
#include <vector>
#include <iomanip>

#include "global.h"
#include "movement.h"
#include "navigation.h"
#include "pallet.h"

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

extern int order_of_pallets_on_conveyor_counter;
extern int fork_height;

extern const int bit0;
extern const int bit1;
extern const int bit2;
extern const int bit3;
extern const int bit4;
extern const int bit5;
extern const int bit6;
extern const int bit7;


int identify_pallet()
{
	int red,green,blue;
	red = green = blue = 0;

	
	const int red_led = 0b01111111;
	const int green_led = 0b10111111;
	const int blue_led = 0b11011111;
	
	const int r_d1_led = 0b01111111; //bit 7 0
	const int gw_d2_led = 0b10111111; // bit 6 0
	const int b_d3_led = 0b11011111; // bit 7 0
	const int white_leds = 0b00011111;
	
	
	
	
	rlink.command(WRITE_PORT_1, red_led);
	delay(500);
	red = rlink.request(ADC0);
	
	rlink.command(WRITE_PORT_1, green_led);
	delay(500);
	green = rlink.request(ADC0);
	
	rlink.command(WRITE_PORT_1, blue_led);
	delay(500);
	blue = rlink.request(ADC0);
	
	cout << "Red: " << red << "  Green: " << green << "   Blue: " << blue << endl;
	
	
	rlink.command(WRITE_PORT_1, 0xFF); // make everythin high -> off
	
	// WHITE
	if((red >= 50 && red <= 99) && (blue >= 50 && blue <= 100) && (green >= 60 && green <= 100))
	{
		cout << "White Pallet" << endl;
		rlink.command(WRITE_PORT_4, white_leds);
		return WHITE;
	}
	
	//RED
	else if((red >= 100 && red <= 130) && (blue >= 150 && blue <= 167) && (green >= 140 && green <= 165))
	{
		cout << "Red Pallet" << endl;
		rlink.command(WRITE_PORT_4, r_d1_led);
		return RED;
	}
	
	
	//GREEN
	else if((red >= 145 && red <= 160) && (blue >= 145 && blue <= 165) && (green >= 165 && green <= 180))
	{
		cout << "Green Pallet" << endl;
		rlink.command(WRITE_PORT_4, gw_d2_led);
		return GREEN;
	}
	
	//BLACK
	else if((red >= 161 && red <= 180) && (blue >= 161 && blue <= 200) && (green >= 149 && green <= 200))
	{
		cout << "Black Pallet" << endl;
		rlink.command(WRITE_PORT_4, b_d3_led);
		return BLACK;
	}
	
	else
	{
		return -1;
	}
	
}


bool operate_lift(int speed_m3) {
	
	speed_m3 = -speed_m3; //negative is up

	if(speed_m3 < 0)	//negative speed are from 128-255 which is reverse
	{
		speed_m3 = -speed_m3 + 128;
		
		if(speed_m3 > 255)
		{
			speed_m3 = 255;
			cout << "invalid speed_m3 entered in move_robot(>255)" << endl;
		}
		else if(speed_m3 < 0)
		{
			speed_m3 = 0;
			cout << "invalid speed_m3 entered in move_robot (<<0)" << endl;
		}
	}

	rlink.command(MOTOR_3_GO, speed_m3);
	
	return true;
	
}



bool move_forks_one_switch(int position)
{
	int value;
	bool pallet_on_flag;
	//switch on mechanism that indicates position
	const int mechanism_switch = 0b00000000; //to be set after testing!!
	cout << " mechanism switch not set, returning false" << endl;
	return false;

	const int fork_switch = 0b00000000; //to be set after testing
	cout << "fork  switch not set, returning false" << endl;
	return false;

	if(fork_height == position)
	{
		cout << "No change in fork height needed" << endl;
		return true;
	}

	//to stop the lowering ifpallet no longer on forks
	if(value bitand fork_switch)	//check if there is a pallet, needed for lowering
	{
		pallet_on_flag = true;//no pallet on fork
	}
	else
	{
		pallet_on_flag = false;//pallet on fork
	}


	if(fork_height < position)
	{
		//the mechanism will be above the given mark so it will have to go up
		//one less click than normally -> hence the following line
		fork_height--;
		while(fork_height < position)
		{
			operate_lift(100);
			value = rlink.request(READ_PORT_1);
			
			if(value bitand mechanism_switch)
			{
				fork_height++;	
			}

			delay(5);
		}

		delay(25);	//let mechanism move slighlty higher than the position of triggering
		operate_lift(0); //stop the lift;
		return true;
	}

	else if(fork_height > position)
	{
		//the mechanism will be above the given mark so it will have to go down
		//one more click than normally -> hence the following line
		fork_height++;
		while(fork_height > position)
		{
			operate_lift(-50);		//move down slower
			value = rlink.request(READ_PORT_1);
			
			if(value bitand mechanism_switch)
			{
				fork_height--;	
			}

			else if((value bitand fork_switch)&& pallet_on_flag) //pallet no longer restning on fork
			{
				delay(10);
				operate_lift(0);
				move_robot(-100,-100,100);	// move back and then turn
				delay(2000);
				stop_robot();
			}

			delay(5);
		}

		//move mechanism a bit up to be higher than the position of triggering

		operate_lift(100);
		while(1)
		{
			value = rlink.request(READ_PORT_1);
			if(value bitand mechanism_switch)
			{
				break;
			}
		}
		delay(25);
		operate_lift(0);
		return true;
	}
}

bool move_forks(int position)	//implementation for 3 switches
{
	
	cout << "Moving lift" << endl;
	
	int value;
	bool pallet_on_flag;
	const int fork_switch = 0b00010000;
		
	value = rlink.request(READ_PORT_1);
	
	//to stop the lowering ifpallet no longer on forks
	if((value bitand fork_switch) == 0)	//check if there is a pallet, needed for lowering
	{
		pallet_on_flag = true;//no pallet on fork
	}
	else
	{
		pallet_on_flag = false;//pallet on fork
	}
  
	const int bottom_switch = 0b00000100;
	const int middle_switch = 0b00001000;
	const int top_switch = 0b00000010;

	if(position == fork_height)
	{
		cout << "height at the desired level" << endl;
		return true;
	}

	else if (position > fork_height)
	{
		operate_lift(127);

		while(1)
		{

			value = rlink.request(READ_PORT_1);		

			if(position == BOTTOM)
			{
				
				if((value bitand bottom_switch) == 0)
				{
					cout << "bottom mechnism switch pressed" << endl;
					operate_lift(0);
					fork_height = position;
					return true;
					break;
				}
			}

			if(position == MIDDLE)
			{
				
				if((value bitand middle_switch) == 0)
				{
					cout << "middle mechnism switch pressed" << endl;
					operate_lift(0);
					fork_height = position;
					return true;
					break;
				}
			}

			if(position == TOP)
			{
				
				if((value bitand top_switch) == 0)
				{
					cout << "top mechnism switch pressed" << endl;
					operate_lift(0);
					fork_height = position;
					return true;
					break;
				}
			}
			
			
		}
		
		operate_lift(0);

		
	}

	else if (position < fork_height)
	{
		operate_lift(-110);

		while(1)
		{
			value = rlink.request(READ_PORT_1);		

			if((value bitand fork_switch) != 0 && pallet_on_flag)	//if pallet dropped and  at position wanted quit returning false
			{
				cout << "Switch on fork released" << endl;
				delay(100);
				operate_lift(0);
				fork_height = position;
				return false;
			}

			if(position == BOTTOM)
			{
				
				if((value bitand bottom_switch) == 0)
				{
					cout << "bottom mechnism switch pressed" << endl;
					operate_lift(0);
					fork_height = position;
					return true;
					break;
				}
			}

			if(position == MIDDLE)
			{
				
				if((value bitand middle_switch) == 0)
				{
					cout << "middle mechnism switch pressed" << endl;
					operate_lift(0);
					fork_height = position;
					return true;
					break;
				}
			}

			if(position == TOP)
			{
				
				if((value bitand top_switch) == 0)
				{
					cout << "top mechnism switch pressed" << endl;
					operate_lift(0);
					position = fork_height;
					return true;
					break;
				}
			}
		
		}

		operate_lift(0);
	}

	fork_height = position;
	return true;
}

void new_load_led() {
	const int new_load_led = 0b11101111;
	rlink.command(WRITE_PORT_4, new_load_led);
	delay(2000);
	rlink.command(WRITE_PORT_4, 0xFF);
	delay(2000);
	rlink.command(WRITE_PORT_4, new_load_led);
	delay(2000);
	rlink.command(WRITE_PORT_4, 0xFF);
	delay(4000);
}
