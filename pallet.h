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
bool drop_off_pallet();
int identify_pallet();	//returns color

#endif
