/***********************************************************/
/*************  DEAD RECKONING IMPLEMENTATION  *************/
/*************    Written by Yasmine Bennani   *************/
/***********************************************************/



/* User defined constants */
#define WHEEL_DIAMETER 0.069
#define PULSES_PER_REVOLUTION 48.0
#define AXLE_LENGTH 0.125

/* Fixed constants */
#define PI 3.14159


struct position
{
  float x;        // in meter
  float y;        // in meter
  float theta;    // in radian (counterclockwise from x-axis)
};


struct position current_position;


void init_pos()
{
  current_position.x = 0.0;
  current_position.y = 0.0;
  current_position.theta = PI/2;
}


void dead_reckoning()  //Thread, to know the position at any time
{
  float dist_left;
  float dist_right;
  int left_ticks;
  int right_ticks;
  float expr1;
  float cos_current;
  float sin_current;
  float right_minus_left;
  float MUL_COUNT;

  MUL_COUNT  = PI * WHEEL_DIAMETER / PULSES_PER_REVOLUTION;

  while(1)
  {
    left_ticks = left_count;
    right_ticks = right_count;
    left_count = 0;
    right_count = 0;

    dist_left = (float)left_ticks * MUL_COUNT;
    dist_right = (float)right_ticks * MUL_COUNT;

    cos_current = cos(current_position.theta);
    sin_current = sin(current_position.theta);

    if (left_ticks == right_ticks)
    {
      /* Moving in a straight line */
      current_position.x += dist_left * cos_current;
      current_position.y += dist_left * sin_current;
    }
    else
    {
      /* Moving in an arc */
      expr1 = AXLE_LENGTH * (dist_right + dist_left)
              / 2.0 / (dist_right - dist_left);

      right_minus_left = dist_right - dist_left;

      current_position.x += expr1 * (sin(right_minus_left /
                            AXLE_LENGTH + current_position.theta) - sin_current);

      current_position.y -= expr1 * (cos(right_minus_left /
                            AXLE_LENGTH + current_position.theta) - cos_current);

      /* Calculate new orientation */
      current_position.theta += right_minus_left / AXLE_LENGTH;

      /* Keep in the range -PI to +PI */
      while(current_position.theta > PI)
        current_position.theta -= (2.0*PI);
      while(current_position.theta < -PI)
        current_position.theta += (2.0*PI);
    }

    sleep(0.1);
  }
}

