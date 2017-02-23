//Global variables

extern int current_bearing;
extern int current_location[2];
extern int next_location[2];
extern int pallets_picked_up;
extern int mode;
extern int pallets_delivered; 
extern int current_pallet_colour;
extern int order_of_pallets_on_conveyor[6];

enum colour {white, red, green, black}; // white = 0, red = 1 ...
enum bearing {north = 0, east = 90, south = 180, west = 270};


//function prototypes

//function to move robot forward with speed_m1 for motor 1, speed m2 for motor 2
//and ramp_time, time to reach those speeds
void move_robot(int speed_m1, int speed_m2, int ramp_time); 

bool turn_robot(int angle);
bool stop_robot();
bool follow_line();

//returns next junction
int calcuate_path(int start_location[2], int finish_location[2]);

//determine whether the robot should go straight,left or right at each junction
bool navigation(int current_location[2],int next_location[2], int bearing);

//the height will be worked out within this function using the current location
//which will either be a pick up position or a drop down position
bool operate_lift(int current_location[2]);

int identify_pallet();	//returns color

bool pick_up_pallet();
bool drop_off_pallet();

void new_load();

bool write_to_file(); // writes global variables to file
bool read_and_update(); //reads global variables from file and updates the current 				//global variables
