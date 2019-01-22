#ifndef MOVEMENT_H_  
#define MOVEMENT_H_ 

float init_value_compass;
void curved_turn(float left_factor, float right_factor, int runtime);
void go_to_corner(int to_right); // Go to right corner if to_right = 1, otherwise go to left corner
void shoot_from_stage1();
void shoot_from_stage2();
void shoot_from_stage3();
void shoot_from_stage4();
void shoot_from_stage5();

#endif
