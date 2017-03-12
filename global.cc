#include <iostream>
#include <robot_link.h>
#include <robot_instr.h>
#include <stopwatch.h>
#include <string>
#include <stdio.h>
#include <delay.h>
#include <fstream>

#include "global.h"
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

void print_binary_8_bit(int input)
{
	for(int i=7; i>=0; i--)
	{
   		cout<<((input>>i)&1);
	}
	cout << endl;	
}

void print_binary_16_bit(int input)
{
	for(int i=15; i>=0; i--)
	{
   		cout<<((input>>i)&1);
	}
	cout << endl;	
}

void print_hex(int input)
{
	char output[8];
	sprintf(output, "%x-", input);
	cout << output;
}

bool write_to_file() // writes global variables to file
{
	ofstream myfile;							// create output stream object
	myfile.open("variables.txt", ios::out);		//open file for writing
	myfile << current_bearing << endl;			//write all global variables to file 
	myfile << current_location[1] << endl;
	myfile << current_location[2] << endl;
	myfile << next_location[1] << endl;
	myfile << next_location[2] <<endl;
	myfile << pallets_picked_up << endl;
	myfile << mode << endl;
	myfile << pallets_delivered << endl;
	myfile << current_pallet_colour << endl;
	
	for(int i; i < 6; i++)
	{
		myfile << order_of_pallets_on_conveyor[i] << endl;
	}
	myfile << order_of_pallets_on_conveyor_counter;
	myfile << fork_height;

	myfile.close();
	return true;
}

bool read_and_update() //reads global variables from file and updates the current 
{
	ifstream myfile;							// create output stream object
	myfile.open("variables.txt", ios::in);		//open file for writing
	myfile >> current_bearing;			//write all global variables to file 
	myfile >> current_location[1];
	myfile >> current_location[2];
	myfile >> next_location[1];
	myfile >> next_location[2];
	myfile >> pallets_picked_up;
	myfile >> mode;
	myfile >> pallets_delivered;
	myfile >> current_pallet_colour;
	
	for(int i; i < 6; i++)
	{
		myfile >> order_of_pallets_on_conveyor[i];
	}
	myfile >> order_of_pallets_on_conveyor_counter;
	myfile >> fork_height;
	myfile.close();
	
	return true;
}
