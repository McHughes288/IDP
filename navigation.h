#ifndef NAVIGATION_H
#define NAVIGATION_H

bool follow_line();

//returns next junction
int calcuate_path(int start_location[2], int finish_location[2]);

//determine whether the robot should go straight,left or right at each junction
bool navigation(int current_location[2],int next_location[2], int bearing);


#endif
