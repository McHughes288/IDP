#ifndef MOVEMENT_H
#define MOVEMENT_H


void move_robot(int speed_m1, int speed_m2, int ramp_time );

bool turn_robot(int angle);

bool turn_right_90();

bool turn_left_90();

bool stop_robot();

bool approach_pickup();

bool move_to_junction(bool already_reversed);

#endif
