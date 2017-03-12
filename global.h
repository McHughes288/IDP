#ifndef GLOBAL_H
#define GLOBAL_H

#define ROBOT_NUM 17 

//define colours
#define WHITE 1
#define RED 2
#define GREEN 3
#define BLACK 4

//define directions
#define NORTH 0
#define EAST 90
#define SOUTH 180
#define WEST 270

//define for heights
#define BOTTOM 0
#define MIDDLE 1
#define TOP 2


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

extern const int bit0;
extern const int bit1;
extern const int bit2;
extern const int bit3;
extern const int bit4;
extern const int bit5;
extern const int bit6;
extern const int bit7;




//enum colour {white, red, green, black}; // white = 0, red = 1 ...
//enum bearing {north = 0, east = 90, south = 180, west = 270};

void print_binary_8_bit(int input);
void print_binary_16_bit(int input);
void print_hex(int input);

// ** FILE **
bool write_to_file(); // writes global variables to file
bool read_and_update(); //reads global variables from file and updates the current 				                      //global variables
/*

//function prototypes

//function to move robot forward with speed_m1 for motor 1, speed m2 for motor 2
//and ramp_time, time to reach those speeds

// ** MOVEMENT **
void move_robot(int speed_m1, int speed_m2, int ramp_time); 
bool turn_robot(int angle);
bool stop_robot();



// ** NAVIGATION **
bool follow_line();
//returns next junction
int calcuate_path(int start_location[2], int finish_location[2]);
//determine whether the robot should go straight,left or right at each junction
bool navigation(int current_location[2],int next_location[2], int bearing);
bool follow_line();
//returns next junction
int calcuate_path(int start_location[2], int finish_location[2]);
//determine whether the robot should go straight,left or right at each junction
bool navigation(int current_location[2],int next_location[2], int bearing);



// ** PALLET **
//the height will be worked out within this function using the current location
//which will either be a pick up position or a drop down position
bool operate_lift(int current_location[2]);
bool pick_up_pallet();
bool drop_off_pallet();
int identify_pallet();	//returns color

//void new_load();



*/

#endif
