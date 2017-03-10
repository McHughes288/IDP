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

extern vector<int> path_x,path_y, path_n; // path[x][y][#]

extern vector<int> final_x,final_y,final_n;
extern vector<int> matrix_row,matrix_column;

extern vector<char> directions;
extern vector<int> bearing_vector;

int main(int argc, const char **argv)
{
	
    // ===== INITIALISE LINK =====
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
	
    
    
    
    
    // ===== LOAD VARIABLES =====
    
    // load parameters from file if main started with argument r
    if(argc > 1 && std::string(argv[1]) == "r")
	{
		cout << "restart" << endl;
		cout << "Loading Variables" << endl;
		//read_and_update();
		current_bearing = EAST;
	}
    // load default parameters
	else
	{
		current_bearing = EAST;
		pallets_picked_up = 0;
		pallets_delivered = 0;
	}

    
    //operate_lift(100);
	//delay(2000);
	//operate_lift(0);
    
    
    
    // ===== TESTS =====
	
	
	
	
	// *** DEMO TEST 1 - Follow a line for over 0.5m
	
	/*
	cout << "Starting in 7 seconds" << endl;
	delay(7000);
	
	// Follow 3 lines and stop at the third junction
	for(int i = 0; i < 3; i++)
	{
		follow_line();
		delay(500);
	}*/
	
	
	
	
	
	
	// *** DEMO TEST 2 - Naviagte to P1 from start area, turn to face the lorry and operate the lift motor
	
	cout << "Starting in 10 seconds" << endl;
	delay(10000);
	
	cout << "Starting navigation routine demo" << endl;
	cout << "Bearing to Follow: " << endl;
	
	current_bearing = EAST; // Start bearing
	shortest_path(14,4,14,10); // Calculate path to P1
	make_directions(current_bearing); // Calculate turns at each junction and then end up facing south
	
	// Print the bearing vector and direction vectors
	for(int i = 0; i < (int)bearing_vector.size(); i++)
	{
		cout << bearing_vector[i] << setw(10) << directions[i] << endl;
	}
	
	// Loop through the turns required and follow the line between each one
	for(int i = 0; i < (int)bearing_vector.size(); i++)
	{
		turn_robot(bearing_vector[i]);
		follow_line();		
	}
	
	turn_robot(SOUTH);
	
	// Operate the lift motor for 4s
	//operate_lift(50);
	//delay(4000);
	//operate_lift(0);
	
	shortest_path(14,10,3,12); // Calculate path to P1
	make_directions(current_bearing); // Calculate turns at each junction and then end up facing south
	
	// Print the bearing vector and direction vectors
	for(int i = 0; i < (int)bearing_vector.size(); i++)
	{
		cout << bearing_vector[i] << setw(10) << directions[i] << endl;
	}
	
	// Loop through the turns required and follow the line between each one
	for(int i = 0; i < (int)bearing_vector.size(); i++)
	{
		turn_robot(bearing_vector[i]);
		follow_line();
	
		
	}
	
	
	
	
	// *** DEMO TEST 3 - Identify the pallet with the LDR circuit
	/*
	cout << "Starting in 3 seconds" << endl;
	delay(3000);
	
	
	const int r_d1_led = 0b01111111; //bit 7 0
	const int gw_d2_led = 0b10111111; // bit 6 0
	const int b_d3_led = 0b11011111; // bit 7 0
	
	rlink.command(WRITE_PORT_4, r_d1_led);
	delay(500);
	rlink.command(WRITE_PORT_4, gw_d2_led);
	delay(500);
	rlink.command(WRITE_PORT_4, b_d3_led);
	delay(500);
	
	
	for(int i = 0; i < 40; i++)
	{
		identify_pallet();
		delay(1000);
	}
	
	*/
	
	return 1;

	
	
	
	
	
	
	
	
	
	
	
	/*
	const int all = 0b00011111;
	rlink.command(WRITE_PORT_1,all);
	delay(500);
	
	rlink.command(WRITE_PORT_1,0);
	delay(500);
	
	rlink.command(WRITE_PORT_1,0b01011111);
	delay(500);
	*/
	
	//const int s_red = bit7;
	//const int s_green = bit5;
	//const int s_blue = bit6;
	
	//const int r_d1_led = 0b01111111; //bit 7 0
	//const int gw_d2_led = 0b10111111 // bit 6 0
	//const int b_d3_led = 0b11011111 // bit 7 0
	
	//rlink.command(WRITE_PORT_4, r_d1_led);
	
	/*
	
	
	
	return 1;
	
	cout << "Start turning test" << endl;
	
    // Turn anti-clockwise
    turn_robot(NORTH);
    turn_robot(WEST);
    turn_robot(SOUTH);
    turn_robot(EAST);
    
    // Turn clockwise
    turn_robot(SOUTH);
    turn_robot(WEST);
    turn_robot(NORTH);
    turn_robot(EAST);
    
    // 180 test
    turn_robot(WEST);
    turn_robot(EAST);
    
    */
    
}
