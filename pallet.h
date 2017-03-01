#ifndef PALLET_H
#define PALLET_H

//the height will be worked out within this function using the current location
//which will either be a pick up position or a drop down position
bool operate_lift(int current_location[2]);
bool pick_up_pallet();
bool drop_off_pallet();
int identify_pallet();	//returns color

#endif
