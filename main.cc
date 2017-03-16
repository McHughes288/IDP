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

//try multithreading
//#include <thread>  

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
int pallets_picked_up = 0;
int mode;
int pallets_delivered = 0; 
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
} start, end, J_C1, J_P1, P1, J_P2, P2, J_D31, D31, J_D32, D32, J_C2, J_D, D1, D2, current_coordinates;


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
		
		
		
		const int bottom_switch = 0b00000100;
		int value;
		
		value = rlink.request(READ_PORT_1);	
		cout << "Calibrating Mechanism" << endl;
		while(1)
		{
			operate_lift(-50);
			value = rlink.request(READ_PORT_1);	
			if((value bitand bottom_switch) == 0)
			{
				break;
			}
		}
		
		operate_lift(0);
		
		
		fork_height = BOTTOM;
		current_coordinates.row = 14;
		current_coordinates.column = 4;
		
	}
    // load default parameters
	else
	{
		current_bearing = EAST;
		pallets_picked_up = 0;
		pallets_delivered = 0;
		current_location[0] = start.row;
		current_location[1] = start.column;
		current_pallet_colour = -1; //no current colour
		mode = -1; //not yet defined

		next_location[0] = P1.row;	//to move to P1
		next_location[1] = P1.column;

		for(int i = 0; i < 4; i++)
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
		const int bottom_switch = 0b00000100;
		int value;
		
		value = rlink.request(READ_PORT_1);	
		
		
		cout << "Calibrating Mechanism" << endl;
		
		while(1)
		{
			operate_lift(-50);
			value = rlink.request(READ_PORT_1);	
			if((value bitand bottom_switch) == 0)
			{
				break;
			}
			
		}
		
		operate_lift(0);
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
	
	end.row = 3;
	end.column = 8; 
	
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

	J_D31.row = 9;				//Junction before D3, on the 
	J_D31.column = 8;			//lower part of the arena

	D31.row = 9;				//Location of D3 on the lower
	D31.column = 8;				//part of the arena

	J_D32.row = 7;				//junction before D3, on the 
	J_D32.column = 8;			//uper part of the arena

	D32.row = 9;				//upper location of d3
	D32.column = 8;

	J_C2.row = 3;				//junction before C2
	J_C2.column = 12; 
    
    J_D.row = 3;				//junction before D1 and D2
	J_D.column = 12;			//same as previous

	D1.row = 1;					//location of D1
	D1.column = 12;

	D2.row = 5;					//location of D2
	D2.column = 12;

    
    
	
	// ===== MAIN PROGRAMM =====
	
	
	cout << "Starting in 5 seconds" << endl;
	delay(5000);
	
	cout << "Starting main programm" << endl;
	
	//pallets_picked_up = 4;
	//order_of_pallets_on_conveyor[0] = RED;
	//order_of_pallets_on_conveyor[1] = GREEN;
	//order_of_pallets_on_conveyor[2] = WHITE;
	//order_of_pallets_on_conveyor_counter = 2;

	int colour;
	int last_pallet;
	int counter;
	//bool lift_flag = true; // to be used to check whether the lift managed to go to it's position
	//bool reversed_flag = false; // A flag used to see whether the robot reversed in the drop pallet routine or not

	
	
	
	// FIRST PALLET
	// Pick up from P2
	
	if (pallets_picked_up < 1) {
	
		cout << "Getting first pallet from P2" << endl;

		shortest_path(current_coordinates.row,current_coordinates.column, J_P2.row, J_P2.column);	
		make_directions(current_bearing);
		navigation();			//robot will move to junction before p2
		approach_pickup();		//routine to approach truck
		current_coordinates.row = P1.row;	//update current coordinates
		current_coordinates.column = P1.column;

		move_forks(TOP);	//pick pallet up and move it to the top position
		
		
		cout << "Pallet picked up from P2" << endl;
		colour = identify_pallet();
		pallets_picked_up++;

		back_to_junction(false);
		current_coordinates.row = J_P2.row;
		current_coordinates.column = J_P2.column;

		while(colour == -1 && counter < 5)	//if colour == -1 the identify pallet cannot determine colour
		{
			colour = identify_pallet();
			counter++;
		}

		if(counter == 5)	//after five consecutive errors
		{
			cout << "Could not Identify pallet. Assuming green" << endl;
			colour = GREEN;
		}

		if(colour == BLACK)	//if black deliver directly
		{
			cout << "Colour Black -> Delivering to D31" << endl;
			
			shortest_path(current_coordinates.row, current_coordinates.column, J_D31.row, J_D31.column);
			make_directions(current_bearing);
			navigation(); // Will move up to the junction before D3 on the low side
			move_forks(TOP);
			turn_robot(NORTH);
			follow_line(0);
			
			cout << "Dropping off black pallet at D3" << endl;
			move_forks(MIDDLE);
			last_pallet = BLACK;
			
			move_robot(-100,-100,100);
			delay(1350);
			move_robot(0,0,100);
			turn_left_90();
			current_bearing = SOUTH;
			follow_line(0);
			
			move_forks(BOTTOM);
			
			current_coordinates.row = 12;
			current_coordinates.column = 8;
		}

		else
		{
			cout << "Not black -> to conveyor" << endl;
			
			shortest_path(current_coordinates.row, current_coordinates.column, J_C1.row, J_C1.column);
			make_directions(current_bearing);
			navigation();
			
			turn_robot(EAST); 
			follow_line(1000);//follow line for another 0.2 seconds
		
			cout << "Dropping first pallet off at conveyor" << endl;
			move_forks(BOTTOM);

			//cout << "moving back to junction" << endl;
			back_to_junction(false); //reverse to be able to turn without throwing pallet of the conveyor
			
			move_forks(TOP);
			current_coordinates.row = J_C1.row;
			current_coordinates.column = J_C1.column;

		
			order_of_pallets_on_conveyor[order_of_pallets_on_conveyor_counter] = colour;
			order_of_pallets_on_conveyor_counter++;
			last_pallet = colour;
		}
	}
	
	
	
	
	
	
	// NEXT 2 PALLETS
	// Pick up from P1

	while(pallets_picked_up < 3)
	{
		cout << endl << "Getting pallet: " << pallets_picked_up + 1 << endl;
		
		// Show new LED after 2 have been picked up
		if (pallets_picked_up == 2) {
			cout << "Showing new LED" << endl;
			new_load_led(); // only need to flash once since the pick up trucks will be loaded in the beggining
		}

	
		if(last_pallet != BLACK)//go to p1
		{
			//move_robot(-100,-100,100);
			//delay(700);
			//stop_robot();
			//move_forks(TOP);
			//follow_line(0);
			
			cout << "Approach P1 from conveyor" << endl;
			approach_pickup();
		}
		
		else
		{
			//shortest_path(current_coordinates.row, current_coordinates.column, J_P1.row, J_P1.column);
			
			//shorthest and easiest path from D3 to P1
			cout << "Approach P1 from D3" << endl;
			move_forks(TOP);
			turn_left_90();
			follow_line(0);
			turn_right_90();
			move_forks(BOTTOM);
			follow_line(0);
			current_bearing = SOUTH;

			
			if(current_bearing != SOUTH)
			{
				approach_pickup();
			}
			
		}
		
		
		cout << "Picking up pallet from P1" << endl;
		follow_line(0);		//routine to approach truck
		current_coordinates.row = P1.row;	//update current coordinates
		current_coordinates.column = P1.column;

		move_forks(TOP);	//pick pallet up and move it to the top position
		colour = identify_pallet();
		pallets_picked_up++;

		back_to_junction(false);
		current_coordinates.row = J_P1.row;
		current_coordinates.column = J_P1.column;


		counter = 0;
		
		while(colour == -1 && counter < 5)	//if colour == -1 the identify pallet cannot determine colour
		{
			colour = identify_pallet();
			counter++;
		}

		if(counter == 5)	//after five consecutive errors
		{
			cout << "Could not Identify pallet. Assuming green" << endl;
			colour = GREEN;
		}

		if(colour == BLACK)	//if black deliver directly
		{
			cout << "Black Pallet -> D31 low" << endl;
			shortest_path(current_coordinates.row, current_coordinates.column, J_D31.row, J_D31.column);
			make_directions(current_bearing);
			navigation(); // Will move up to the junction before D3 on the low side
			move_forks(TOP);
			follow_line(0);
			cout << "Dropping off black pallet at D3" << endl;
			move_forks(MIDDLE);
			move_robot(-100,-100,100);
			delay(1350);
			move_robot(0,0,100);
			turn_left_90();
			current_bearing = SOUTH;
			follow_line(0);
		
			current_coordinates.row = 12;
			current_coordinates.column = 8;
			
			//move_forks(BOTTOM);
			
			last_pallet = BLACK;
		}

		else
		{
			cout << "Not black -> to conveyor" << endl;
		
			
			turn_robot(EAST); 
			follow_line(0);
			
			cout << "Dropping pallet off at conveyor" << endl;
			move_forks(BOTTOM);

			//cout << "moving back to junction" << endl;
			back_to_junction(false); //reverse to be able to turn without throwing pallet of the conveyor
			
			move_forks(TOP);
			current_coordinates.row = J_C1.row;
			current_coordinates.column = J_C1.column;

		
			order_of_pallets_on_conveyor[order_of_pallets_on_conveyor_counter] = colour;
			order_of_pallets_on_conveyor_counter++;
			last_pallet = colour;
		}
	}
	
	
	
	
	
	
	
	// LAST PALLET
	// Pick up from P2
	
	if (pallets_picked_up < 4) {
		cout << endl << "Getting Last Pallet" << endl;
		
			
			if(last_pallet != BLACK)
			{
				cout << "Approaching P2 from conveyor" << endl;
				JC1_to_JP2();
			}
			
			else
			{
				cout << "Approaching P2 from D3" << endl;
				shortest_path(current_coordinates.row, current_coordinates.column, J_P2.row, J_P2.column);
				make_directions(current_bearing);
				move_forks(BOTTOM);	//swapped with next line
				navigation();
					
			}
			
		
			cout << "Picking up last pallet from P2" << endl;
			follow_line(0);		//routine to approach truck
			current_coordinates.row = P2.row;	//update current coordinates
			current_coordinates.column = P2.column;

			move_forks(TOP);	//pick palle up and move it to the middle position
			colour = identify_pallet();
			pallets_picked_up++;

			back_to_junction(false);
			current_coordinates.row = J_P2.row;
			current_coordinates.column = J_P2.column;


			counter = 0;
			
			while(colour == -1 && counter < 5)	//if colour == -1 the identify pallet cannot determine colour
			{
				colour = identify_pallet();
				counter++;
			}

			if(counter == 5)	//after five consecutive errors
			{
				cout << "Could not Identify pallet. Assuming green" << endl;
				colour = GREEN;
			}

			if(colour == BLACK)	//if black deliver directly
			{
				cout << "Black Pallet -> D31 low" << endl;
				shortest_path(current_coordinates.row, current_coordinates.column, J_D31.row, J_D31.column);
				make_directions(current_bearing);
				navigation(); // Will move up to the junction before D3 on the low side
				move_forks(TOP);
				follow_line(0);
				cout << "Dropping off black pallet at D3" << endl;
				move_forks(MIDDLE);
				move_robot(-100,-100,100);
				delay(1350);
				move_robot(0,0,100);
				turn_left_90();
				current_bearing = SOUTH;
				follow_line(0);
			
				current_coordinates.row = 12;
				current_coordinates.column = 8;
				
				move_forks(BOTTOM);
				last_pallet = BLACK;
				
				shortest_path(current_coordinates.row, current_coordinates.column, J_D.row, J_D.column);	//move on the junction between  D1 and D2
				make_directions(current_bearing);
				navigation(); 
			}
				
			else 
			{
				cout << "Moving with pallet to the top" << endl;
				shortest_path(current_coordinates.row, current_coordinates.column, J_D.row, J_D.column);	//move on the junction between  D1 and D2
				make_directions(current_bearing);
				navigation(); 

				if(colour == RED)
				{
					cout << " Delivering to D1 for Red pallets" << endl;
					turn_robot(NORTH);
					//move_robot(-100,-100,100);
					//delay(500);
					//stop_robot();
					//follow_line(0);
					follow_line(0);
					move_forks(BOTTOM);
					
					move_robot(-100,-100,100);
					delay(1350); // calibrate this
					stop_robot();
					move_forks(TOP);
					follow_line(0);
					
					current_coordinates.row = J_D.row;
					current_coordinates.column = J_D.column;
				}

				else
				{
					cout << " Delivering to D2 for white and green pallets" << endl;
					turn_robot(SOUTH);
					//move_robot(-100,-100,100);
					//delay(500);
					//stop_robot();
					//follow_line(0);
					follow_line(0);
					move_forks(BOTTOM);

					move_robot(-100,-100,100);
					delay(1000); // calibrate this
					stop_robot();
					move_forks(TOP);
					follow_line(0);
					current_coordinates.row = J_D.row;
					current_coordinates.column = J_D.column;
				}
		}
	}
		
	
	
	
	// UNLOADING THE CONVEYOR
	counter = 0;
	
	//current_bearing = SOUTH;
	//fork_height = TOP;
	//order_of_pallets_on_conveyor_counter = 3;
	
	while(counter < (int)order_of_pallets_on_conveyor_counter)
	{
	
		if(current_bearing == NORTH || current_bearing == SOUTH)
		{
			turn_robot(WEST);
			follow_line(0);
			move_forks(BOTTOM);
			turn_robot(EAST);
			follow_line(0);
			follow_line(0);
			move_forks(TOP);
		}
		
		else if(current_bearing == EAST)
		{
			follow_line(0);
			move_forks(TOP);
		}
		if(order_of_pallets_on_conveyor[counter] == RED)
		{
			cout << "Delivering red pallet" << endl;
			
			//shortest_path(current_coordinates.row, current_coordinates.column, J_D.row, J_D.column);
			//make_directions(current_bearing);
			//navigation();
			
			back_to_junction(false);

			turn_robot(NORTH);
			//move_robot(-100,-100,100);
			//delay(500);
			//stop_robot();
			//follow_line(0);
			follow_line(300);
			move_forks(BOTTOM);
			
			move_robot(-100,-100,100);
			cout << "going back" << endl;
			delay(1250); // calibrate this
			stop_robot();
			move_forks(TOP);
			follow_line(0);
			current_coordinates.row = J_D.row;
			current_coordinates.column = J_D.column;
			counter++;
		}
		else
		{	
			cout << "Delivering white or green pallet" << endl;
			//shortest_path(current_coordinates.row, current_coordinates.column, J_D.row, J_D.column);
			//make_directions(current_bearing);
			//navigation();

			back_to_junction(false);

			turn_robot(SOUTH);
			//move_robot(-100,-100,100);
			//delay(500);
			//stop_robot();
			//follow_line(0);
			follow_line(300);
			move_forks(BOTTOM);
			
			move_robot(-100,-100,100);
			delay(1250); // calibrate this
			stop_robot();
			move_forks(TOP);
			follow_line(0);
			current_coordinates.row = J_D.row;
			current_coordinates.column = J_D.column;
			counter++;
		}
		
	}
	
	cout << "going back to start area" << endl;
	shortest_path(current_coordinates.row, current_coordinates.column, end.row, end.column);
	make_directions(current_bearing);
	navigation();
	
	move_robot(-100,-100,0);
	delay(200);
	stop_robot();
	return 1;
	
	/*
	while(counter <= (int)order_of_pallets_on_conveyor_counter)
	{

		if(order_of_pallets_on_conveyor[counter] == -1)
		{
			cout << "Trying to get a pallet where there is none" << endl;
			cout << "Delivered them all" << endl;
			break;
		}

		cout << endl << "Approaching conveyor" << endl;
		turn_robot(EAST);
		move_robot(-100,-100,100);
		delay(1000);
		stop_robot();
		move_forks(BOTTOM);
		follow_line(0);
		follow_line(0);
		move_forks(TOP);

		if(order_of_pallets_on_conveyor[counter] == RED)
		{
			cout << "Delivering red pallet" << endl;
			
			//shortest_path(current_coordinates.row, current_coordinates.column, J_D.row, J_D.column);
			//make_directions(current_bearing);
			//navigation();
			
			back_to_junction(false);

			turn_robot(NORTH);
			move_robot(-100,-100,100);
			delay(500);
			stop_robot();
			follow_line(0);
			follow_line(0);
			move_forks(BOTTOM);
			
			move_robot(-100,-100,100);
			delay(1000); // calibrate this
			stop_robot();
			move_forks(TOP);
			follow_line(200);
			current_coordinates.row = J_D.row;
			current_coordinates.column = J_D.column;
			counter++;
		}
		else
		{	
			cout << "Delivering white or green pallet" << endl;
			//shortest_path(current_coordinates.row, current_coordinates.column, J_D.row, J_D.column);
			//make_directions(current_bearing);
			//navigation();

			back_to_junction(false);

			turn_robot(SOUTH);
			//move_robot(-100,-100,100);
			//delay(500);
			//stop_robot();
			//follow_line(0);
			follow_line(0);
			move_forks(BOTTOM);
			
			move_robot(-100,-100,100);
			delay(1000); // calibrate this
			stop_robot();
			move_forks(TOP);
			follow_line(200);
			current_coordinates.row = J_D.row;
			current_coordinates.column = J_D.column;
			counter++;
		}


	}

	cout << "going back to start area" << endl;
	shortest_path(current_coordinates.row, current_coordinates.column, end.row, end.column);
	make_directions(current_bearing);
	navigation();

	for(int i = 0; i < 8; i++)
	{
		turn_left_90();
	}

	return 1;
	*/
	
	//by this point all 4 pallets should have been picked up and delivered
	
	
	
	
	
	
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
	/*
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
		follow_line(0);		
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
	//
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
	
	//return 1;
	
	/*

	cout << "STarting in 5 seconds" << endl;
	delay(5000);
	follow_line(0);
	move_robot(-100,-100,100);
	delay(2000);
	stop_robot();
	
	back_to_junction(false);
	return 1;
	*/
	
	//follow_line(0);
	//return 1;
	
	
	/*
	cout << "Testing Mechanism" << endl;
	
	cout << "Moving to top" << endl;
	move_forks(TOP);
	delay(1000);
	
	cout << "Move to bottop" << endl;
	move_forks(BOTTOM);
	delay(1000);
	
	cout << "Move to middle" << endl;
	move_forks(MIDDLE);
	delay(1000);	
	
	return 1;
	
	cout << "Starting in 10 seconds" << endl;
	delay(10000);
	
	fork_height = BOTTOM;
	
	follow_line(0);
	approach_pickup();
	operate_lift(127);
	delay(3000);
	operate_lift(0);
	back_to_junction(false);
	
	current_coordinates.column = J_P1.column;
	current_coordinates.row = J_P1.row;
	
	shortest_path(current_coordinates.row, current_coordinates.column, J_C1.row, J_C2.column);
	make_directions(current_bearing);
	navigation();
	follow_line(500);
	stop_robot();
	move_forks(BOTTOM);
	
	back_to_junction(false);
	
	JC1_to_JP2();
	
	return 1;
	
	cout << "Reading Port 1" << endl;
	
	int val;
	int i;
	
	while(i < 20)
	{
		val = rlink.request(READ_PORT_1);
		print_binary_8_bit(val);
		delay(1000);
		i++;
		
	}
	
	return 1;
	
	*/
	
	/*
	 move_forks(BOTTOM);
	move_forks(TOP);
	delay(500);
	move_forks(MIDDLE);
	delay(500);
	move_forks(BOTTOM);
	cout << "connected" << endl;
	while(1)
	{
		identify_pallet();
		delay(1000);
	}
	return 0;
	 */
	
	
	
	
	
	
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
