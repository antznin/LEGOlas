/***********************************************************/
/*************  DEAD RECKONING IMPLEMENTATION  *************/
/*************    Written by Yasmine Bennani   *************/
/***********************************************************/

#include "dead_reckoning.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"

#define FIELD_WIDTH 1.20 //Width of the field in meter

#define FIELD_LENGTH 1.0 //Length of the field

#define PI 3.14159

struct position current_position;

void init_pos()
{
  current_position.x = 0.0;
  current_position.y = 0.0;
  current_position.theta = PI/2;
}


void dead_reckoning(int COUNT_PER_METER, int left_speed, int right_speed, float dist)  //Thread, to know the position at any time
{
    uint8_t sn;
    uint8_t sn_compass;
    float theta; //In radian
    float value; //In degree, compass output
    float dist_left;
    float dist_right;
    float expr1;
    float cos_current;
    float sin_current;
    float right_minus_left;
    
    
    while(1)
    {
        if (ev3_search_sensor(HT_NXT_COMPASS, &sn_compass,0)){
            printf("COMPASS found, reading compass...\n");
            if ( !get_sensor_value0(sn_compass, &value )) {
                value = 0;
            }
            printf( "\r(%f) \n", value);
            fflush( stdout );
        }
        
        theta = value * (PI / 180);
        current_position.theta = theta;
        cos_current = cos(current_position.theta);
        sin_current = sin(current_position.theta);

        if (left_speed == right_speed)
        {
            /* Moving in a straight line */
            current_position.x += dist * cos_current;
            current_position.y += dist * sin_current;
        }
        else
        {
            /* The robot is just turning around */
            current_position.x = current_position.x;
            current_position.y = current_position.y;

            /* Get new orientation from sensor */
            current_position.theta = theta;

            /* Keep in the range -PI to +PI */
            while(current_position.theta > PI)
                current_position.theta -= (2.0*PI);
            while(current_position.theta < -PI)
                current_position.theta += (2.0*PI);
        }

    sleep(0.1);
    }
}

