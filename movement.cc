#include <iostream>
#include <robot_link.h>
#include <robot_instr.h>
#include <stopwatch.h>
#include <string>
#include <stdio.h>
#include <delay.h>

#include "global.h"
#include "movement.h"

using namespace std;

extern robot_link rlink;
extern stopwatch watch;

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

// ===== MOVE =====

void move_robot(int speed_m1, int speed_m2, int ramp_time = 0)
{
	speed_m1 = -speed_m1; // due to orientation of motors speed_m1 must be negative to go forward
	
    //ramp time connot be negative or greater than 255
	if(ramp_time < 0)
	{
		ramp_time = 0;
	}
	else if(ramp_time > 255)
	{
		ramp_time = 254; //255 is the default so if we pass anything greater use the greatest 
	}					 //user defined ramp time

	if(speed_m1 < 0)	//negative speed are from 128-255 which is reverse
	{
		speed_m1 = -speed_m1 + 128;
		
		if(speed_m1 > 255)
		{
			speed_m1 = 255;
			cout << "invalid speed_m1 entered in move_robot(>255)" << endl;
		}
		else if(speed_m1 < 0)
		{
			speed_m1 = 0;
			cout << "invalid speed_m1 entered in move_robot (<<0)" << endl;
		}
	}

	if(speed_m2 < 0)
	{
		speed_m2 = -speed_m2 + 128;
		
		if(speed_m2 > 255)
		{
			speed_m2 = 255;
			cout << "invalid speed_m1 entered in move_robot(>255)" << endl;
		}
		else if(speed_m2 < 0)
		{
			speed_m2 = 0;
			cout << "invalid speed_m1 entered in move_robot (<<0)" << endl;
		}
	}
	

	//motor 1 -> left
	//motor 2 -> right
	rlink.command(RAMP_TIME, ramp_time);
	rlink.command(MOTOR_1_GO, speed_m1);
	rlink.command(MOTOR_2_GO, speed_m2);
} 



// ===== TURN =====

bool turn_robot(int angle) {
    
    
    int angle_difference = angle - current_bearing;
    
    // Makes sure angle difference is between -180 and 180 which correspond to left and right
    if(angle_difference > 180)
    {
        angle_difference -= 360;
    }
    if(angle_difference < -180)
    {
        angle_difference += 360;
    }
    
    //cout << endl << "Start Bearing: " << current_bearing << endl;
    //cout << "Finish Bearing: " << angle << endl;
    //cout << "Angle Difference: " << angle_difference << endl;
    
    // Turn right if angle difference is positive
    if (angle_difference > 0) {
        //cout << "Turning right..." << endl;
        turn_right_90();
        
        // Turn again for full 180
        if (angle_difference > 90) {
            //cout << "Turning right again..." << endl;
            turn_right_90();
        }
        
        
    // Turn left if angle difference is negative
    } else if (angle_difference < 0) {
        //cout << "Turning left..." << endl;
        turn_left_90();
        
        // Turn again for full 180
        if (angle_difference <- 90) {
            //cout << "Turning left again..." << endl;
            turn_left_90();
        }
        
    // If angle difference is 0 do nothing
    } else {
        //cout << "Don't need to turn" << endl;
    }
    
    // NOW DONE WITHIN OTHER FUNCTIONS
    //current_bearing = current_bearing + angle_difference;
    
    
    // Check to see if the robot is in the position it should be
    if (current_bearing == angle) {
        //cout << "Now at correct bearing" << endl;
        return true;
    } else {
       //cout << "Something went wrong, not at correct bearing" << endl;
        return false;
    }

}

bool turn_left_90() {
    int speed_m1 = -126;
    int speed_m2 = 126;
    
    int state = 0; // 0 is when rear sensor is active before turning
    // 1 is when the rear sensor isnt active as the robot is turning
    // 2 is when the rear sensor is active again and the robot has completed a 90 degree turn
    
    int port_value;
    const int s_middle = bit1;	//middle sensor bit
    //const int s_right = bit0;	//right sensor bit
    //const int s_left = bit2;	//left sensor bit
    //const int s_rear = bit3;	// rear sensor bit
    
    
    move_robot(speed_m1, speed_m2);
    
    while(state != 2){
        
        port_value = rlink.request(READ_PORT_4);
        
        // robot is no longer over the junction
        if(port_value bitand s_middle)
        {
        } else {
            state = 1;
            //continue;
        }
        
        // robot has completed the LEFT turn (checks right sensor)
        if (port_value bitand s_middle && state == 1) {
			delay(100); //was 250
            stop_robot();
            state = 2;
            break;
        }
        
    }
    
    // Update bearing
    current_bearing = current_bearing - 90;
    if (current_bearing < 0) {
        current_bearing += 360;
    }
    
    //cout << "Stopped left turn. Current Bearing: " << current_bearing << endl;
    
    return true;
    
}



bool turn_right_90() {
    int speed_m1 = 126;
    int speed_m2 = -126;
    
    int state = 0; // 0 is when rear sensor is active before turning
    // 1 is when the rear sensor isnt active as the robot is turning
    // 2 is when the rear sensor is active again and the robot has completed a 90 degree turn
    
    int port_value;
   
    const int s_middle = bit1;	//middle sensor bit
	//const int s_right = bit0;	//right sensor bit
	//const int s_left = bit2;	//left sensor bit
	//const int s_rear = bit3;	// rear sensor bit
	
    move_robot(speed_m1, speed_m2);

    while(state != 2){
        
        port_value = rlink.request(READ_PORT_4);
        
        // robot is no longer over the junction
        if(port_value bitand s_middle)
        {
        } else {
            state = 1;
            //continue;
        }
        
        // robot has completed the RIGHT turn (checks left sensor)
        if (port_value bitand s_middle && state == 1) {
			delay(100); // was 250 
            stop_robot();
            state = 2;
            break;
        }
        
    }
    
    // Update bearing
    current_bearing = current_bearing + 90;
    if (current_bearing >= 360) {
        current_bearing -= 360;
    }
    
    //cout << "Stopped right turn. Current Bearing: " << current_bearing << endl;

    
    return true;

}



// ===== STOP =====

bool stop_robot() {
	move_robot(0, 0);
	return true;
}



