#ifndef DEAD_RECKONING_H_ 
#define DEAD_RECKONING_H_ 

struct position
{ 
  float x;        // in meter
  float y;        // in meter
  float theta;    // in radian (counterclockwise from x-axis)
};

void init_pos();

void dead_reckoning(int COUNT_PER_METER, int left_speed, int right_speed, float dist);

#endif
