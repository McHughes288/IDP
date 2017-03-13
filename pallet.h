#ifndef PALLET_H
#define PALLET_H

//define colours
#define WHITE 1
#define RED 2
#define GREEN 3
#define BLACK 4

//the height will be worked out within this function using the current location
//which will either be a pick up position or a drop down position
bool operate_lift(int speed_m3);
bool pick_up_pallet();
bool move_forks(int position); //moves fork to one of the given positions: top,middle,bottom
bool move_forks_one_switch(int position);//moves for to one of the above positions, implementation with only one switch
int identify_pallet();	//returns color

#endif
