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


	// Initialise Parameters
	current_bearing = EAST;
	pallets_picked_up = 0;
	pallets_delivered = 0;
	
	cout << "Start Turning test" << endl;
	turn_robot(NORTH);
	turn_robot(EAST);
	turn_robot(NORTH);
	turn_robot(SOUTH);
	
	
	cout << "Starting follow line routine" << endl;
	follow_line();
	cout << "Stoping operation" << endl;
	move_robot(0,0,0);
	
	
	return 1;
}
