#ifndef NAVIGATION_H
#define NAVIGATION_H

bool follow_line(int time);

//returns next junction
int calcuate_path(int start_location[2], int finish_location[2]);

bool navigation();

bool JC1_to_JP2();

void shortest_path(int start_x, int start_y, int f_x, int f_y);//calculates path

bool make_directions(int bearing);//make directions for junctions

bool approach_pickup_point();

bool back_to_junction(bool alread_reversed); //gos from a drop off point or a pick up point back to the junction
#endif
