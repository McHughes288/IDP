#include <iostream>
#include <robot_link.h>
#include <robot_instr.h>
#include <stopwatch.h>
#include <string>
#include <stdio.h>
#include <delay.h>

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
	sprintf(output, "%x", input);
	cout << output;
}
