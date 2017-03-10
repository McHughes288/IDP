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
	int all_on;
	
	const int red_led = 0b01111111;
	const int green_led = 0b10111111;
	const int blue_led = 0b11011111;
	const int all = 0b00011111;
	
	const int r_d1_led = 0b01111111; //bit 7 0
	const int gw_d2_led = 0b10111111; // bit 6 0
	const int b_d3_led = 0b11011111; // bit 7 0
	
	
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
	
	
	rlink.command(WRITE_PORT_1,all);
	delay(500);
	all_on = rlink.request(ADC0);
	cout << "3 white on: " << all_on << endl << endl;
	
	rlink.command(WRITE_PORT_1, 0xFF); // make everythin high -> off
	
	if((red >= 75 && red <= 90) && (blue >= 85 && blue <= 110) && (green >= 40 && green <= 75))
	{
		cout << "White Pallet" << endl;
		rlink.command(WRITE_PORT_4, gw_d2_led);
		return WHITE;
	}
	
	else if((red >= 95 && red <= 125) && (blue >= 170 && blue <= 200) && (green >= 105 && green <= 125))
	{
		cout << "Red Pallet" << endl;
		rlink.command(WRITE_PORT_4, r_d1_led);
		return RED;
	}
	
	else if((red >= 126 && red <= 157) && (blue >= 140 && blue <= 190) && (green >= 105 && green <= 125))
	{
		cout << "Green Pallet" << endl;
		rlink.command(WRITE_PORT_4, gw_d2_led);
		return GREEN;
	}
	
	else if((red >= 150 && red <= 165) && (blue >= 190 && blue <= 210) && (green >= 130 && green <= 145))
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
	
	speed_m3 = -speed_m3;
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
