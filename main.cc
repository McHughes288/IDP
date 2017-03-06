#include <iostream>
#include <robot_link.h>
#include <robot_instr.h>
#include <stopwatch.h>
#include <string>
#include <stdio.h>
#include <delay.h>
#include <fstream>
#include <string>


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

int main(int argc, const char **argv)
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
	//print_binary_8_bit(6);
	if(argc > 1 && std::string(argv[1]) == "r")
	{
		cout << "restart" << endl;
		cout << "Loading Variables" << endl;
		//read_and_update();
		current_bearing = EAST;
	}
	else
	{
		current_bearing = EAST;
		pallets_picked_up = 0;
		pallets_delivered = 0;
	}

	cout << "Starting in 7 seconds" << endl;
	delay(7000);
	
	cout << "Start Turning test" << endl;
	
	cout << "Current bearing: " << current_bearing << endl;
	watch.start();
	turn_robot(NORTH);
	cout << "Time needed: " << watch.read()/1000 << endl;
	//stop_robot();
	delay(5000);
	
	/*cout << "Current bearing: " << current_bearing << endl;
	turn_robot(WEST);
	//stop_robot();
	delay(5000);
	
	cout << "Current bearing: " << current_bearing << endl;
	turn_robot(SOUTH);
	//stop_robot();
	delay(5000);

	cout << "Current bearing: " << current_bearing << endl;
	turn_robot(EAST);*/
	
	//stop_robot();
	return 1;
	
	cout << "Following first line" << endl;
	follow_line();
	cout << "Following Second line" << endl;
	follow_line();
	cout << "Followig third line" << endl;
	follow_line();
	


	
	move_robot(100,100,0);
	delay(2000);
	stop_robot();
	delay(10000);
	
	
	cout << "Place robot on start" << endl;
	delay(10000);
	
	cout << "Following first line" << endl;
	follow_line();
	cout << "Following Second line" << endl;
	follow_line();
	cout << "Followig third line" << endl;
	follow_line();
	cout << "Turning on P1" << endl;
	turn_robot(SOUTH);
	
	
	move_robot(0,0,0);
	return 1;
}
