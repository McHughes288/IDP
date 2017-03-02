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
//#include "pallet.h"
//#include "file.h"

using namespace std;

robot_link rlink;
stopwatch watch;

int current_bearing;
int current_location[2];
int next_location[2];
int pallets_picked_up;
int mode;
int pallets_delivered; 
int current_pallet_colour;
int order_of_pallets_on_conveyor[6];

//used for bitwise checks
const int bit0 = 0b00000001;
const int bit1 = 0b00000010;
const int bit2 = 0b00000100;
const int bit3 = 0b00001000;
const int bit4 = 0b00010000;
const int bit5 = 0b00100000;
const int bit6 = 0b01000000;
const int bit7 = 0b10000000;

int main()
{
	
#ifdef __arm__
	if(!rlink.initialise())
	{
		cout << "Cannot initialise link" <<endl;
		rlink.print_errs(" ");
		return -1;
	}
#else
	if(!rlink.initialise(ROBOT_NUM))
		{
			cout << "Cannot initialise link" <<endl;
			rlink.print_errs(" ");
			return -1;
		}
#endif



	/*
	 
	int val;
	char port[8];
	int time;
	watch.start();
	for(int i = 0; i < 100; i++)
	{
		val = rlink.request(READ_PORT_0);
	}
	time = watch.read() / 100;
	watch.stop();
	
	for(int i=7; i>=0; i--)
	{
   		cout<<((val>>i)&1);
	}
	cout << endl;

	sprintf(port, "%x", val);
	
	cout << "Using sprintf(hex): 0x" << port << endl;
	
	cout << "Time to read: " << time << endl;*/
	
	//Left Motor = Red, Right Motor = Green, Forward --> -ve, +ve
	//cout << "Moving Forwad, hopefully..." << endl;
	//move_robot(-100,100,135);
	//delay(5000);
	
	//move_robot(0,0,0);
	
	/*
	cout << "Reading Port 0: ";
	val = rlink.request(READ_PORT_0);
	print_binary_8_bit(val);
	
	cout << "Reading Port 1: ";
	val = rlink.request(READ_PORT_1);
	print_binary_8_bit(val);
	
	cout << "Reading Port 2: ";
	val = rlink.request(READ_PORT_2);
	print_binary_8_bit(val);
	
	cout << "Reading Port 3: ";
	val = rlink.request(READ_PORT_3);
	print_binary_8_bit(val);
	
	cout << "Reading Port 4: ";
	val = rlink.request(READ_PORT_4);
	print_binary_8_bit(val);
	
	cout << "Reading Port 5: ";
	val = rlink.request(READ_PORT_5);
	print_binary_8_bit(val);
	
	cout << "Reading Port 6: ";
	val = rlink.request(READ_PORT_6);
	print_binary_8_bit(val);
	
	cout << "Reading Port 7: ";
	val = rlink.request(READ_PORT_7);
	print_binary_8_bit(val);
	* */
	
	//move_robot(-100,100*1.1,135);
	//delay(5000);
	
	//move_robot(0,0,0);
	
	cout << "Starting follow line routine" << endl;
	follow_line();
	cout << "Stoping operation" << endl;
	move_robot(0,0,0);
	int speed1,speed2;
	speed1 = 100;
	speed2 = 100;	
	
	while(1)
	{
		char choice;
		cout << "Change Motor speeds(y/n):";
		cin >> choice;
		if(choice == 'n' || choice == 'N');
		{
			break;
		}
		
		cout << "Enter speed 1,left (normal -100), previous(" << speed1 << "): ";
		cin >> speed1;
		cout << "Enter speed 2, right(normal 100), previous(" << speed2 << "): ";
		cin >> speed2;
		move_robot(speed1,speed2,255);
		delay(3000);
		move_robot(0,0,0);	
	}
	
	return 1;
}
