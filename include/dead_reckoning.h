#ifndef DEAD_RECKONING_H_ 
#define DEAD_RECKONING_H_ 

struct position
{
  char * state;   // TACHO_STOP, TACHO_RUN_FOREVER, etc
  float x;        // in centimeter
  float y;        // in centimeter
  float theta;    // in degree (counterclockwise from x-axis)
};

void init_pos();
void rewrite_state();
float get_theta();
float get_x();
float get_y();
void *dead_reckoning(void * unused);
int turn(float angle);
void init_orientation();
int move_to_xy(float x, float y);
void move_forward(int dist);

#endif
