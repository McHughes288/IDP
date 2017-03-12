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

//following 3 libraries used for exit handler of the program
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>

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
//new globals
int order_of_pallets_on_conveyor_counter; 
int fork_height;

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


struct location
{
	int row;
	int column;
} start, J_C1, J_P1, P1, J_P2, P2, J_D31, D31, J_D32, D32, J_C2, J_D, D1, D2, current_coordinates;


void my_handler(int s)
{
	//note that SIGINT(interrupt from keyboard -> ctrl-c) == 2
	// and the SIGQUIT(interrupt from keyboard again) == 3
	printf("Caught signal: %d\n", s);
	printf("Assuming it is ctrl-c signal the variables will be written to the file\n");

	current_location[0] = current_coordinates.row;
	current_location[1] = current_coordinates.column;
	write_to_file();
	exit(1);
}


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


	// ==== Create sigaction structure and link my_handler to the interrupt caused by the signal	
	struct sigaction sigIntHandler;

	sigIntHandler.sa_handler = my_handler;	//specify routine that will excecute on signal detection
	//The sigemptyset() function initialises the signal set pointed to by set, 
	//such that all signals defined in this document are excluded. 
	sigemptyset(&sigIntHandler.sa_mask);// sa_mask specifies a mask of signals which should be blocked 
	sigIntHandler.sa_flags = 0; // sa_flags specifies a set of flags which modify the behavior of the
       							// signal.

	sigaction(SIGINT, &sigIntHandler, NULL);

    // ===== LOAD VARIABLES =====
    
    // load parameters from file if main started with argument r
    if(argc > 1 && std::string(argv[1]) == "r")
	{
		cout << "restart" << endl;
		cout << "Loading Variables" << endl;
		read_and_update();
		current_bearing = EAST;
		current_location[0] = start.row;
		current_location[1] = start.column;
	}
    // load default parameters
	else
	{
		current_bearing = EAST;
		pallets_picked_up = 0;
		pallets_delivered = 0;
		current_location[0] = start.row;
		current_location[1] = start.column;
		pallets_picked_up = 0;
		pallets_delivered = 0;
		current_pallet_colour = -1; //no current colour
		mode = -1; //not yet defined

		next_location[0] = P1.row;	//to move to P1
		next_location[1] = P1.column;

		for(int i = 0; i < 6; i++)
		{
			order_of_pallets_on_conveyor[i] = -1; //ie no pallet on the conveyor
		}

		order_of_pallets_on_conveyor_counter = 0; //to be used to add the colour at the correct position

		


		//  ==== 	INITIAL CALIBRATION OF FORK MECHANISM ====
		
		/*
		* 	An initial calibration of the mechanism is needed since only one switch is used
		*	and there is no possibility of knowing where the mechanism has started from
		*	we require that the mechanism is bellow the bottom position and then we start 
		* 	lifting until we reach the bottom position. From there a global variable will
		* 	be used to monitor the current height of the mechanism 
		*/
		/*

		int val;
		int mech_switch = 0b00000000; // to be set
		while(1)
		{

			val = rlink.request(READ_PORT_1);//If on another port change
			if(val bitand mech_switch)
			{
				operate_lift(0);	//stop lift motor
				break;	//exit while loop
			}

			operate_lift(50);	// move lift up
		}

		*/
		fork_height = BOTTOM;


		write_to_file();

		current_coordinates.row = 14;
		current_coordinates.column = 4;
	}






	    // ==== CREATE LOCATION COORDINATES ==== 
    /*
    * All the columns and rows correspond to the rows and 
    * columns of the  matrix found in navigation.cc and 
    * each pair corresponds to a location
    *
    * Mostly the junctions just before each point will be 
    * used due to the insufficient room to manoeuvre 
    * freely around the arena
    */


    start.column = 14;			//Start coordinate
	start.row = 4;
	
	J_C1.row = 14;				// Junction just before C1
	J_C1.column = 10;	
	
	J_P1.row = 14;				//Junction just before P1
	J_P1.column = 10;

	P1.row = 16;				//Location of P1
	P1.column = 10;

	J_P2.row = 14;				//Junction just before P2
	J_P2.column = 8;

	P2.row = 14;				//Location of P2
	P2.column = 10;

	J_D31.row = 11;				//Junction before D3, on the 
	J_D32.column = 8;			//lower part of the arena

	D31.row = 9;				//Location of D3 on the lower
	D31.column = 8;				//part of the arena

	J_D32.row = 5;				//junction before D3, on the 
	J_D32.column = 8;			//uper part of the arena

	D32.row = 7;				//upper location of d3
	D32.column = 8;

	J_C2.row = 3;				//junction before C2
	J_C2.column = 12; 
    
    J_D.row = 3;				//junction before D1 and D2
	J_D.column = 12;			//same as previous

	D1.row = 1;					//location of D1
	D1.column = 12;

	D2.row = 5;					//location of D2
	D2.column = 12;

    
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
	
	navigation();
	
	
	
	
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

	
	
	
	
	
	// ===== MAIN PROGRAMM =====



	int colour;
	int counter;
	bool lift_flag = true; // to be used to check whether the lift managed to go to it's position

	//pickup a pallet from P1 which is closer

	shortest_path(current_coordinates.row,current_coordinates.column, J_P1.row, J_P1.column);	
	make_directions();
	navigation();			//robot will move to junction before p1
	approach_pickup();		//routine to approach truck
	current_coordinates.row = P1.row;	//update current coordinates
	current_coordinates.column = P1.column

	//must flash LEDs not yet implemented
	cout << "implement flash LEDs routine"

	delay(10000);
	move_forks(MIDDLE);	//pick palle up and move it to the middle position
	colour = identify_pallet();
	pallets_picked_up++;

	follow_line_reverse();
	current_coordinates.row = J_P1.row;
	current_coordinates.column = J_P1.column;

	while(colour == -1 && counter < 5)	//if colour == -1 the identify pallet cannot determine colour
	{
		colour = identify_pallet;
		counter++;
	}

	if(counter == 5)	//after five consecutive errors
	{
		cout << "Could not Identify pallet. Assuming green" >> endl;
		colour = GREEN;
	}

	if(colour == BLACK)	//if black deliver directly
	{
		shortest_path(current_coordinates.row, current_coordinates.column, J_D31.row, J_D31.column);
		make_directions();
		navigation(); // Will move up to the junction before D3 on the low side
		move_forks(TOP);
		follow_line();
		move_forks(MIDDLE);
		follow_line_reverse(); // reverseuntil junction
		move_forks(BOTTOM);
		current_coordinates.row = J_D31.row;
		current_coordinates.column = J_D31.column;
	}

	else
	{
		shorthes_path(current_coordinates.row, current_coordinates.column, J_C1.row, J_C1.column);
		//calibrate the follow line time
		cout << "Follow line time not calibrated for approaching C1" << endl;
		turn_robot(SOUTH);
		follow_line(200);//follow line for another 0.2 seconds

		lift_flag = move_forks(BOTTOM);

		follow_line_reverse(); //reverse to be able to turn without throwing pallet of the conveyor
		current_coordinates.row = J_C1.row;
		current_coordinates.column = J_C1.column;

		if(lift_flag == false)
		{
			move_forks(BOTTOM);
		}

		order_of_pallets_on_conveyor[order_of_pallets_on_conveyor_counter] = colour;
		order_of_pallets_on_conveyor_counter++;
	}

	//pick the next 4 pallets if possible from p2

	while(pallets_picked_up < 5)
	{
		shortest_path(current_coordinates.row,current_coordinates.column, J_P2.row, J_P2.column);	
		make_directions();
		navigation();			//robot will move to junction before p1
		approach_pickup();		//routine to approach truck
		current_coordinates.row = P2.row;	//update current coordinates
		current_coordinates.column = P2.column

		//must flash LEDs not yet implemented
		cout << "implement flash LEDs routine"

		delay(10000);
		move_forks(MIDDLE);	//pick palle up and move it to the middle position
		colour = identify_pallet();
		pallets_picked_up++;

		follow_line_reverse();
		current_coordinates.row = J_P2.row;
		current_coordinates.column = J_P2.column;



		while(colour == -1 && counter < 5)	//if colour == -1 the identify pallet cannot determine colour
		{
			colour = identify_pallet;
			counter++;
		}

		if(counter == 5)	//after five consecutive errors
		{
			cout << "Could not Identify pallet. Assuming green" >> endl;
			colour = GREEN;
		}

		if(colour == BLACK)	//if black deliver directly
		{
			shortest_path(current_coordinates.row, current_coordinates.column, J_D31.row, J_D31.column);
			make_directions();
			navigation(); // Will move up to the junction before D3 on the low side
			move_forks(TOP);
			follow_line();
			move_forks(MIDDLE);
			follow_line_reverse(); // reverse for 1.5 seconds
			move_forks(BOTTOM);
			current_coordinates.row = J_D31.row;
			current_coordinates.column = J_D31.column;
		}

		else
		{
			shorthes_path(current_coordinates.row, current_coordinates.column, J_C1.row, J_C1.column);
			//calibrate the follow line time
			cout << "Follow line time not calibrated for approaching C1" << endl;
			turn_robot(SOUTH);
			follow_line(200);//follow line for another 0.2 seconds

			move_flag = move_forks(BOTTOM);


			follow_line_reverse(); //reverse to be able to turn without throwing pallet of the conveyor
			current_coordinates.row = J_C1.row;
			current_coordinates.column = J_C1.column;

			if(lift_flag == false)
			{
				move_forks(BOTTOM);
			}

			order_of_pallets_on_conveyor[order_of_pallets_on_conveyor_counter] = colour;
			order_of_pallets_on_conveyor_counter++;
		}
	}
	

	//get the last pallet from P1 a

	shortest_path(current_coordinates.row,current_coordinates.column, J_P1.row, J_P1.column);	
	make_directions();
	navigation();			//robot will move to junction before p1
	approach_pickup();		//routine to approach truck
	current_coordinates.row = P1.row;	//update current coordinates
	current_coordinates.column = P1.column

	//must flash LEDs not yet implemented
	cout << "implement flash LEDs routine"

	delay(10000);
	move_forks(MIDDLE);	//pick palle up and move it to the middle position
	colour = identify_pallet();
	pallets_picked_up++;

	follow_line_reverse();
	current_location.row = J_P1.row;
	current_location.column = J_P1.column;

	while(colour == -1 && counter < 5)	//if colour == -1 the identify pallet cannot determine colour
	{
		colour = identify_pallet;
		counter++;
	}

	if(counter == 5)	//after five consecutive errors
	{
		cout << "Could not Identify pallet. Assuming green" >> endl;
		colour = GREEN;
	}

	if(colour == BLACK)	//if black deliver directly
	{
		shortest_path(current_coordinates.row, current_coordinates.column, J_D31.row, J_D31.column);
		make_directions();
		navigation(); // Will move up to the junction before D3 on the low side
		move_forks(TOP);
		follow_line();
		move_forks(MIDDLE);
		follow_line_reverse(); // reverse for 1.5 seconds
		move_forks(BOTTOM);
		current_coordinates.row = J_D31.row;
		current_coordinates.column = J_D31.column;
	}

	else 
	{
		shortest_path(current_coordinates.row, current_coordinates.column, J_D.row, J_D.column);	//move on the junction between  D1 and D2
		make_directions();
		navigation(); 

		if(colour == RED)
		{
			turn_robot(NORTH);
			follow_line(0);
			lift_flag = move_forks(BOTTOM);

			follow_line_reverse();
			if(lift_flag == false)
			{
				move_forks(BOTTOM);
			}
			current_coordinates.row = J_D.row;
			current_coordinates.column = J_D.column;
		}

		else
		{
			turn_robot(SOUTH);
			follow_line(0);
			lift_flag = move_forks(BOTTOM);

			follow_line_reverse();
			if(lift_flag == false)
			{
				move_forks(BOTTOM);
			}
			current_coordinates.row = J_D.row;
			current_coordinates.column = J_D.column;
	}


	int counter = 0;
	while(counter < order_of_pallets_on_conveyor_counter)
	{

		if(order_of_pallets_on_conveyor[counter] == -1)
		{
			cout << "Trying to get a pallet where there is none" << endl;
			break;
		}

		shortest_path(current_coordinates.row, current_coordinates.column, J_C2.row, J_C2.column);
		make_directions();
		navigate();
		turn_robot(EAST); //if not facing EAST turn to face the conveyor 
		cout << "Need to callibrate follow line time from junction to C2"
		follow_line(600); //uncalibrated !!!!!

		move_forks(TOP);

		follow_line_reverse();

		current_coordinates.row = J_C2.row;
		current_coordinates.column = J_C2.column;

		if(order_of_pallets_on_conveyor[counter] == RED);
		{
			shortest_path(current_coordinates.row, current_coordinates.column, J_D.row, J_D.column);
			make_directions();
			navigate();

			turn_robot(NORTH);
			follow_line();
			lift_flag = move_forks(BOTTOM);
			follow_line_reverse(); 

			if(lift_flag == false)
			{
				move_forks(BOTTOM);
			}
			
			current_coordinates.row = J_D.row;
			current_coordinates.column = J_D.column;
			counter++;
		}

		else;
		{
			shortest_path(current_coordinates.row, current_coordinates.column, J_D.row, J_D.column);
			make_directions();
			navigate();

			turn_robot(SOUTH);
			follow_line();
			lift_flag = move_forks(BOTTOM);
			follow_line_reverse(); 

			if(lift_flag == false)
			{
				move_forks(BOTTOM);
			}
			
			current_coordinates.row = J_D.row;
			current_coordinates.column = J_D.column;
			counter++;
		}


	}


	shortest_path(current_coordinates.row, current_coordinates.column, start.row, start.column);
	make_directions();
	navigate();

	for(int i = 0; i < 8; i++)
	{
		turn_left_90();
	}

	return 1;
	//by this point all 6 pallets should have been picked up
	
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
