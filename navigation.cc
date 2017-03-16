#include <iostream>
#include <robot_link.h>
#include <robot_instr.h>
#include <stopwatch.h>
#include <string>
#include <stdio.h>
#include <delay.h>

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

string string_matrix[18][15] = {
{"x",   "x",   "x",   "x",   "x",   "x",   "x",   "x",   "x",   "x",   "x",   "x",   "x",   "x",   "x"},
{"x",   "x",   "x",   "x",   "x",   "x",   "T6",  "0",   "T5",  "0",   "T7",  "x",   "D1",  "x",   "x"},
{"x",   "x",   "x",   "x",   "0",   "x",   "0",   "x",   "0",   "x",   "0",   "x",   "0",   "x",   "x"},
{"x",   "x",   "x6",  "0",   "J10", "0",   "J11", "0",   "j12", "0",   "j13", "0",   "j16", "0",   "c2"},
{"x",   "x",   "x",   "x",   "0",   "x",   "0",   "x",   "0",   "x",   "0",   "x",   "0",   "x",   "x"},
{"x",   "x",   "x",   "x",   "0",   "x",   "T8",  "0",   "J14", "0",   "T9",  "x",   "D2",  "x",   "x"},  
{"x",   "x",   "x",   "x",   "0",   "x",   "x",   "x",   "0",   "x",   "x",   "x",   "x",   "x",   "x"},
{"x",   "x",   "x",   "x",   "0",   "x",   "x",   "x",   "D32", "x",   "x",   "x",   "x",   "x",   "x"},
{"x",   "x",   "x",   "x",   "0",   "x",   "x",   "x",   "x",   "x",   "x",   "x",   "x",   "x",   "x"},
{"x",   "x",   "x",   "x",   "0",   "x",   "x",   "x",   "D31", "x",   "x",   "x",   "x",   "x",   "x"},
{"x",   "x",   "x",   "x",   "0",   "x",   "x",   "x",   "0",   "x",   "x",   "x",   "x",   "x",   "x"},
{"x",   "x",   "x",   "x",   "0",   "x",   "x5",  "0",   "J3",  "x6",  "T2",  "x",   "x",   "x",   "x"},
{"x",   "x",   "T5",  "0",   "J7",  "0",   "T4",  "x4",  "J17", "0",   "J2",  "x3",  "x",   "x",   "x"},
{"x",   "x",   "0",   "x",   "0",   "x",   "0",   "x",   "0",   "x",   "0",   "x",   "x",   "x",   "x"},
{"x2",  "0",   "J8",  "0",   "J6",  "0",   "J5",  "0",   "J4",  "0",   "J1",  "0",   "C1",  "x",   "x"},
{"x",   "x",   "0",   "x",   "0",   "x",   "0",   "x",   "0",   "x",   "0",   "x",   "x",   "x",   "x"},
{"x",   "x",   "T3",  "0",   "J9",  "0",   "T2",  "x",   "P2",  "x",   "P1",  "x",   "x",   "x",   "x"},
{"x",   "x",   "x",   "x",   "x1",  "x",   "x",   "x",   "x",   "x",   "x",   "x",   "x",   "x",   "x"},
};

int int_matrix[18][15] = {
{-1,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
{-1,  -1, -1, -1, -1, -1, -2,  0,  0,  0, -2, -1,  0, -1, -1},
{-1,  -1, -1, -1,  0, -1,  0, -1,  0, -1,  0, -1,  0, -1, -1},
{-1,  -1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
{-1,  -1, -1, -1,  0, -1,  0, -1,  0, -1,  0, -1,  0, -1, -1},
{-1,  -1, -1, -1,  0, -1, -2,  0,  0,  0, -2, -1,  0, -1, -1},
{-1,  -1, -1, -1,  0, -1, -1, -1,  0, -1, -1, -1, -1, -1, -1},
{-1,  -1, -1, -1,  0, -1, -1, -1,  0, -1, -1, -1, -1, -1, -1},
{-1,  -1, -1, -1,  0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
{-1,  -1, -1, -1,  0, -1, -1, -1,  0, -1, -1, -1, -1, -1, -1},
{-1,  -1, -1, -1,  0, -1, -1, -1,  0, -1, -1, -1, -1, -1, -1},
{-1,  -1, -1, -1,  0, -1, -1,  0,  0,  -1, 0, -1, -1, -1, -1},   //x6 is set as -1 to not take thar root as there is insufficient space
{-1,  -1, -2,  0,  0,  0, -2, -1,  0, -0,  0, -2, -1, -1, -1}, 	 //change -2 in this and the above line to -1 to prevent bugs, since with the
{-1,  -1,  0, -1,  0, -1,  0, -1,  0, -1,  0, -1, -1, -1, -1},	 //current logic it is possible to go from the -2 to the next zero
{-2,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, -1, -1},
{-1,  -1,  0, -1,  0, -1,  0, -1,  0, -1,  0, -1, -1, -1, -1},
{-1,  -1, -2,  0,  0,  0, -2, -1,  0, -1,  0, -1, -1, -1, -1},
{-1,  -1, -1,-1   -2, -1, -1, -1, -1,  -1, -1, -1, -1, -1, -1},
};


/*
 * Convention Used:
 * -2 - Dead ends 
 * -1 - no road
 *  0 - lines
 *  1 - junctions
 *  2 corners
 * 	5 - D1
 *  6 - D2 
 *  7 - D3
 * 	8 - P1
 *  9 - P2
 * 10 - C1
 * 11 - C2  
 */
int coded_matrix[18][15] = {
{-1, -1, -1, -1, -1, -1, -1,  -1, -2, -1, -1, -1, -1,  -1, -1},
{-1, -1, -1, -1, -2, -1,  2,   0,  1,  0,  2, -1,  5   -1, -1},
{-1, -1, -1, -1,  0, -1,  0,  -1,  0, -1,  0, -1,  0,  -1, -1},
{-1, -1,  0,  0,  1,  0,  1,   0,  1,  0,  1,  0,  1,   0,  11},
{-1, -1, -1, -1,  0, -1,  0,  -1,  0, -1,  0, -1,  0,  -1, -1},
{-1, -1, -1, -1,  0, -1,  2,   0,  1,  0,  2, -1,  6,  -1, -1},
{-1, -1, -1, -1,  0, -1, -1,  -1,  0, -1, -1, -1, -1,  -1, -1},
{-1, -1, -1, -1,  0, -1, -1,  -1,  7, -1, -1, -1, -1,  -1, -1},
{-1, -1, -1, -1,  0, -1, -1,  -1, -1, -1, -1, -1, -1,  -1, -1},
{-1, -1, -1, -1,  0, -1, -1,  -1,  7, -1, -1, -1, -1,  -1, -1},
{-1, -1, -1, -1,  0, -1, -1,  -1,  0, -1, -1, -1, -1,  -1, -1},
{-1, -1, -1, -1,  0, -1, -2,   0,  1,  -2,  2, -1, -1,  -1, -1},
{-1, -1,  2,  0,  0,  0,  2,  -2,  1,  0,  0, -2, -1,  -1, -1},
{-1, -1,  0, -1,  0, -1,  0,  -1,  0, -1,  0, -1, -1,  -1, -1},
{-2,  0,  1,  0,  1,  0,  1,   0,  1,  0,  1,  0,  10, -1, -1},
{-1, -1,  0, -1,  0, -1,  0,  -1,  0, -1,  0, -1, -1,  -1, -1},
{-1, -1,  2,  0,  0,  0,  2,  -1,  9, -1,  8, -1, -1,  -1, -1},
{-1, -1, -1, -1, -2, -1, -1,  -1, -1, -1, -1, -1, -1,  -1, -1},
};

vector<int> path_x,path_y, path_n; // path[x][y][#]

vector<int> final_x,final_y,final_n;
vector<int> matrix_row,matrix_column;

vector<char> directions;
vector<int> bearing_vector;



bool follow_line(int time)
{
	int switch_value;
	
	// Sensor values as bits
	const int s_rear = bit3;	// rear sensor bit
	const int s_left = bit2;	//left sensor bit
	const int s_middle = bit1;	//middle sensor bit 
	const int s_right = bit0;	// right sensor bit
	
	const int front_microswitch = 0b00000001;
	//cout << "front switch value not set" << endl;

	int speed_m1; //motor 1 -> left
	int speed_m2; //motor 2 -> right
	int port_value;
	
	const int k1 = 30;	//constants to be added or subtracted to motor speed to change direction
	const int k2 = 50;
	
	bool junction_flag = false; //flag set to true when the fron 3 sensors detect a junction
								//used afterwords in conjucntion with the rear sensor to stop
								//at a junction

	
	int previous_speed_m1 = 127;	//hold previous values for motors to use after turning 
	int previous_speed_m2 = 127;	//when the line is lost
	int counter; 					//counter in order to enter retrace routine
	
	vector<int> past_m1;
	vector<int> past_m2;

	speed_m1 = 127;
	speed_m2 = 127;
	
	move_robot(speed_m1,speed_m2,255);
	
	stopwatch line_following_watch;	// make a new stopwatch class to keep previous one working;
	line_following_watch.start();
	
	while(1) // execute until junction, junction check not yet implemented
	{
		switch_value = rlink.request(READ_PORT_1); //check if this is the correct port !!!!!!!!!!!!!!!
		//for debugging purposes 
		if(line_following_watch.read() > 60000)
		{
			cout << " Follow line routine active for 60 seconds" << endl;
			return false; //stop after 20 seconds for now
		}

		if(time != 0 && line_following_watch.read() > time)
		{
			cout << "Folloed line for: " << time << " seconds, now returning" << endl;
			return false; // return false to know that it was the timer and not a junction
		}

		if((switch_value bitand front_microswitch) == 0b00000000)
		{
			cout << "fron_microswitch pressed" << endl;
			stop_robot();
			return false; //indicate that we hit something
		}
		
		move_robot(speed_m1,speed_m2,0);
		delay(5);
		
		previous_speed_m1 = speed_m1;
		previous_speed_m2 = speed_m2; 
		
		past_m1.insert(past_m1.begin(), speed_m1); //store at start of vector, ie at position zero
		past_m2.insert(past_m2.begin(), speed_m2);
		
		speed_m1 = 127;
		speed_m2 = 127;
		
		port_value = rlink.request(READ_PORT_4);
		
		if(port_value bitand s_rear && junction_flag)
		{
			break;
		}
		
		//1 1 1
		else if(port_value bitand s_left && port_value bitand s_right && port_value bitand s_middle) // all sensors actice -> junction
		{
			junction_flag = true;	//if a junction is detected set the flag as true;
			counter = 0; // zero counter
			continue;
		}
		
		// 1 1 0
		else if(port_value bitand s_middle && port_value bitand s_left) //robot is leaning to the right 
		{
			speed_m1  -= k1; //decrease speed of left motor
			//speed_m2  += k1; //increase speed of right motor
			counter = 0;

 			continue; 
		}
		
		// 0 1 1
		else if(port_value bitand s_middle && port_value bitand s_right) //robot is leaning to the left
		{
			//speed_m1 += k1; //increase speed of left motor
			speed_m2 -= k1; //decrease speed of right motor
			counter = 0; // zero counter
			
			continue;
		}
		
		// 0 1 0
		else if(port_value bitand s_middle)
		{
			counter = 0; // zero counter
			//normal operation, continue normally
			continue;
		}
		
		 // 1 0 0
		else if(port_value bitand s_left) //robot is too far right
		{
			counter = 0; // zero counter
			speed_m1 -= k2;
			//speed_m2 += k2;
			continue;
		}
		
		// 0 0 1
		else if(port_value bitand s_right) //robot is too far left
		{
			counter = 0; // zero counter
			//speed_m1 += k2;
			speed_m2 -= k2;
			continue;
		}
		
		// 0 0 0
		else if(((port_value bitand s_left) == 0) && ((port_value bitand s_middle) == 0) && ((port_value bitand s_right) == 0))
		{
			speed_m1 = previous_speed_m1;
			speed_m2 = previous_speed_m2;
			counter++;
			
			if(counter > 500) // if after 2.5 seconds it hasn't found the line
			{
				int i = 0;
				cout << "retracing..." << endl;	
				//retrace until a sensor finds the line
				while( (port_value bitand s_left) == 0    && (port_value bitand s_middle) == 0 && (port_value bitand s_right) == 0)
				{
					
					move_robot(-past_m1[i], -past_m2[i], 0);
					delay(6);
					
					i++;
					
					//retraced max number of steps
					if(i == (int)past_m1.size())
					{
						break;
					}
				}
				
				if(port_value bitand s_left || port_value bitand s_middle || port_value bitand s_right)
				{
					cout << "Found the Line using retracing!" << endl;
				}
				else
				{
					cout << "Lost in space...."  << endl;
					return false;
				}
			}
			continue;
		}
		
	}
	move_robot(0,0,0);
	return true; // junction detected, returns true
}

//returns next junction
int calcuate_path(int start_location[2], int finish_location[2]);

//follow directions to point
bool navigation()
{
	// Loop through the turns required and follow the line between each one
	for(int i = 0; i < (int)bearing_vector.size(); i++)
	{
		turn_robot(bearing_vector[i]);
		follow_line(0);	
	}
	return true;
}




// call for shortest path(call using row,column ,row, column)
void shortest_path(int start_row, int start_column, int f_row, int f_column)
{
	int k = 0;
	vector<int> temp_x, temp_y, temp_n, near_x, near_y, near_n;
	temp_y.push_back(f_column);
	temp_x.push_back(f_row);

	
	#ifdef DEBUG
	int counter = 1;
	#endif
	
	//int copy_matrix[18][15];
	
	if(start_row < 0 || start_column < 0 || f_row < 0 || f_column < 0 )
	{
		cerr << "Invalid start or finish coordinate" << endl;
		return;
	}
	
	else if(start_row >18 || start_column > 15 || f_row >18 || f_column > 15 )
	{
		cerr << "Invalid start or finish coordinate" << endl;
		return;
	}
	
	else if(int_matrix[start_row][start_column] == -1 || int_matrix[f_row][f_column] == -1)
	{
		cerr << "Coordinate entered is a wall" << endl;
		return;
	}
	
	
	if(!path_x.empty() || !path_y.empty() || !path_n.empty())
	{
		cout << "Erased vectors" << endl;
		path_x.erase(path_x.begin(), path_x.end());
		path_y.erase(path_y.begin(), path_y.end());
		path_n.erase(path_n.begin(), path_n.end());
	}
	
	/*for(int i = 0; i < 18; i++)
	{
		for(int j = 0; j < 15; j++)
		{
			copy_matrix[i][j] = int_matrix[i][j];
		}
	}*/
	
	path_x.push_back(start_row);
	path_y.push_back(start_column);
	path_n.push_back(0);
	k++;
	
	while(1)
	{
		#ifdef DEBUG
		cout << "Entered while loop" << endl;
		cout << "Size of path vector = " << path_x.size() << endl;
		#endif
		int size = path_x.size();
		
		for(int i = 0; i < size; i++) //add the four adjacent cells of each point of the path to the list
		{
			path_x.push_back(path_x[i] + 1);
			path_y.push_back(path_y[i]);
			path_n.push_back(k);
			
			path_x.push_back(path_x[i] - 1);
			path_y.push_back(path_y[i]);
			path_n.push_back(k);
			
			path_x.push_back(path_x[i]);
			path_y.push_back(path_y[i] + 1);
			path_n.push_back(k);
			
			path_x.push_back(path_x[i]);
			path_y.push_back(path_y[i] - 1);
			path_n.push_back(k);
			
			#ifdef DEBUG
			cout << "Added 4 new elements" << endl;
			#endif
			
		}
		k++;
		
		#ifdef DEBUG
		cout << "Created adjacent points" << endl;
		for(int i= 0; i<(int)path_x.size(); i++)
		{
			cout << endl;
			cout  << "(" << path_x[i] << "," << path_y[i] << "," << path_n[i] << ")" << endl;
		}
		#endif
		
		for(int i = 0; i < (int)path_x.size(); i++)
		{
			
			
			if(int_matrix[path_x[i]][path_y[i]] < 0) //remove any walls and unwanted elements(corners,dead ends)from the list
			{
				#ifdef DEBUG 
				cout << "Deleting: " << "(" << path_x[i] << "," << path_y[i] << "," << path_n[i] << ")" << endl;
				#endif
				
				path_x.erase(path_x.begin() + i ); //remove the element that corresponds to a wall
				path_y.erase(path_y.begin() + i );
				path_n.erase(path_n.begin() + i );
				i = 0; // to avoid elements being transfered to the previous element and thus not being checked
			}
			
		}
		
		#ifdef DEBUG
		cout << "Removed walls, current length of vector = " << path_x.size() << endl;
		#endif
		
		for(int i = 0; i < (int)path_x.size(); i++) //remove duplicate element with the highest number
		{
			for(int j = 0; j < (int)path_x.size(); j++)
			{
				if(path_x[i] == path_x[j] && path_y[i] == path_y[j] && i != j) // if x' = x, y' = y and they correspond to different elements 
				{
					if(path_n[i] < path_n[j])	//remove the element that has the smallest number required to reach that point
					{
						#ifdef DEBUG 
						cout << "Deleting: " << "(" << path_x[j] << "," << path_y[j] << "," << path_n[j] << ")" << endl;
						#endif
						path_x.erase(path_x.begin() + j );
						path_y.erase(path_y.begin() + j );
						path_n.erase(path_n.begin() + j );
					}
					else
					{
						#ifdef DEBUG 
						cout << "Deleting: " << "(" << path_x[i] << "," << path_y[i] << "," << path_n[i] << ")" << endl;
						#endif
						path_x.erase(path_x.begin() + i );
						path_y.erase(path_y.begin() + i );
						path_n.erase(path_n.begin() + i );
					}
					i = j = 0; //to avoid elements not being checked
				}
			}
		}
		
		
		#ifdef DEBUG
		cout << "Removed duplicates current length of vector = " << path_x.size()  << endl;
		
		counter++;
		
		if(counter == -1) //ie don't break or set a number to break after a number of repetitions
		{
			break;
		}
		#endif
		
		#ifdef DEBUG
		cout << "Remaining points: " << endl;
		for(int i= 0; i<(int)path_x.size(); i++)
		{
			cout << endl;
			cout  << "(" << path_x[i] << "," << path_y[i] << "," << path_n[i] << ")" << endl;
		}
		#endif
		
		
		for(int i = 0; i < (int)path_x.size(); i++)
		{
			if(path_x[i] == f_row && path_y[i] == f_column)
			{
				cout << "path found with " << path_n[i] << " steps" << endl;
				temp_n.push_back(path_n[i]);
				int counter = 0;
				int steps = path_n[i];
				
				for(int j = 0; j <= path_n[i];j++)//work backward from the final element to the start by considering
				{				  //the next element with a number corresponding to one less than the finsish
					counter++;
					
					near_x.erase(near_x.begin(),near_x.end());//clear the near vectors
					near_y.erase(near_y.begin(),near_y.end());
					near_n.erase(near_n.begin(),near_n.end());
			
					near_x.push_back(temp_x[j] + 1);
					near_y.push_back(temp_y[j]);
					near_n.push_back(steps - counter);
			
					near_x.push_back(temp_x[j] - 1);
					near_y.push_back(temp_y[j]);
					near_n.push_back(steps - counter);
			
					near_x.push_back(temp_x[j]);
					near_y.push_back(temp_y[j] + 1);
					near_n.push_back(steps - counter);
		
					near_x.push_back(temp_x[j]);
					near_y.push_back(temp_y[j] - 1);
					near_n.push_back(steps - counter);
					
			
			
					for( int k = 0; k < (int)path_x.size(); k++)
					{
						bool flag = false;
						
						for(int n = 0; n < (int)near_x.size(); n++)
						{
							if(near_x[n] == path_x[k] && near_y[n] == path_y[k] && near_n[n] == path_n[k])
							{
								temp_x.push_back(path_x[k]); //push the element at the end if the temporary vectors
								temp_y.push_back(path_y[k]);
								temp_n.push_back(path_n[k]);
								flag = true;
							}
							if(flag)
							{
								break;
							}
						}
						if(flag)
						{
							break;
						}
					}
				}
				
				final_x.erase(final_x.begin(),final_x.end());//clear final path
				final_y.erase(final_y.begin(),final_y.end());
				final_n.erase(final_n.begin(),final_n.end());
				
				matrix_row.erase(matrix_row.begin(), matrix_row.end());
				matrix_column.erase(matrix_column.begin(),matrix_column.end());
				
				for(int k = 0; k < (int)temp_x.size(); k++)
				{
					final_x.insert(final_x.begin(), temp_x[k]);	//copy path to the final path in the correct order
					final_y.insert(final_y.begin(), temp_y[k]);
					final_n.insert(final_n.begin(), temp_n[k]);
					
					matrix_column.insert(matrix_column.begin(), temp_y[k]);//store point in matrix form as well
					matrix_row.insert(matrix_row.begin(), temp_x[k]);
				}
				return;
			}
		}
		
		
	}
	
	
}


bool make_directions(int bearing)
{
	directions.erase(directions.begin(), directions.end()); //clear directions
	bearing_vector.erase(bearing_vector.begin(), bearing_vector.end()); //cleal bearing vector;
	
	int change_row, change_column; //change in rows and columns from one element to the next
	int temporary_bearing = bearing; // temporary 
	int direction_to_follow; // store direction to follow
	int angle_difference;	//change in angle betweem [pomts
	
	
	for(int i = 0; i < (int)matrix_column.size() - 2; i++)
	{
		change_column = matrix_column[i] - matrix_column[i+1];
		change_row = matrix_row[i] - matrix_row[i+1];
		  

		//case where the current and next point are on the same line section, hence continue straight
		if(coded_matrix[matrix_row[i]][matrix_column[i]] == 0 && coded_matrix[matrix_row[i+1]][matrix_column[i+1]] == 0)
		{
			continue;
		}
		
		//previous and next junction are a junction -> on section no action needed
		else if(i >= 1 && coded_matrix[matrix_row[i-1]][matrix_column[i-1]] == 1 && coded_matrix[matrix_row[i+1]][matrix_column[i+1]] == 1)
		{
			continue;
		}
		
		
		//Create a vactor holding all the bearings that are to be used in the
		//turn robot subroutine
		else if (change_column ==-1 && change_row == 0)
		{
			bearing_vector.insert(bearing_vector.end(), EAST);
			direction_to_follow = EAST;
			
		}
		
		else if(change_column == 1 && change_row == 0)
		{
			bearing_vector.insert(bearing_vector.end(), WEST);
			direction_to_follow = WEST;

		}
		
		else if(change_column == 0 && change_row == -1)
		{
			bearing_vector.insert(bearing_vector.end(), SOUTH);
			direction_to_follow = SOUTH;
		}
		
		else if(change_column == 0 && change_row == 1)
		{
			bearing_vector.insert(bearing_vector.end(), NORTH);
			direction_to_follow = NORTH;	
		}
		
		else
		{
			cerr << "\n\nDirections could not be made in step: " << i << endl;
			return false;
		}

		angle_difference = temporary_bearing - direction_to_follow;
		
		//No 270 degree turns needed
		if(angle_difference > 180)
		{
			angle_difference -=360;
		}
		
		else if(angle_difference < -180)
		{
			angle_difference +=360;
		}
		
		//create a directions vector made up of verbal directions
		//L for left, R for Right, U for 180 degree turn(U-turn in a way)
		//S for straight
		if(angle_difference == 90)
		{
			directions.insert(directions.end(), 'L');
		}
		else if(angle_difference == -90)
		{
			directions.insert(directions.end(), 'R');
		}
		else if(angle_difference == 180 || angle_difference == -180)
		{
			directions.insert(directions.end(), 'U');
		}
		
		else if(angle_difference == 0)
		{
			directions.insert(directions.end(),'S');
		}
		else
		{
			cerr << "Not specified angle difference encountered! (" << angle_difference <<")" << endl;
			directions.insert(directions.end(),'X');
		}
		
		temporary_bearing = direction_to_follow;
	}
	
	return true;
}




bool approach_pickup()
{

	bool val;

	if(current_bearing != SOUTH)
	{
		turn_robot(NORTH);
		val = follow_line(0); //follow line north for 4 seconds;

		if(val)	//val is true if the follow line reaches a line and returns
		{
			turn_left_90();
			turn_left_90();
			current_bearing = SOUTH;
		}

		else
		{
			turn_left_90();				//a single turn will happen since the robot is on
										//a straight section of the line
			current_bearing = SOUTH;	//the current bearing has to be set independently 
		}
	}
	
	move_forks(BOTTOM);
	follow_line(0);//will follow line up to the junction before the pickup point
	val = follow_line(0);//will follow the line up to the truck

	if(val == false)	//if val = false obstacle detected -> pick up truck
	{	
		return true;
	}

	else
	{
		cout << "Could not reach pick up truck" << endl;
		return false;
	}
}


bool back_to_junction(bool already_reversed)
{
	if(already_reversed == false)
	{
		move_robot(-100,-100,100);		//back up to beclear of obstacles
		delay(1500);
		move_robot(0,0,100);
	}
		
	// == move lift up to avoid problems!!
	move_forks(TOP);
     follow_line(0); // goes back to junction and stops
     return true;
}


bool JC1_to_JP2()
{
	turn_robot(NORTH);
	follow_line(0);
	turn_robot(WEST);
	follow_line(0);
	turn_robot(SOUTH);
	
	
	// ==== FLASH LED HERE ====
	move_forks(BOTTOM);
	
	/*
	const int new_load_led = 0b11101111;
	rlink.command(WRITE_PORT_4, new_load_led);
	delay(2000);
	rlink.command(WRITE_PORT_4, 0xFF);
	delay(2000);
	rlink.command(WRITE_PORT_4, new_load_led);
	delay(2000);
	rlink.command(WRITE_PORT_4, 0xFF);
	delay(4000);
	*/
	follow_line(0);
	
	return true;
}
