/***********************************************************/
/*************  DEAD RECKONING IMPLEMENTATION  *************/
/*************    Written by Yasmine Bennani   *************/
/***********************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <pthread.h>
#include "/src/ev3dev-c/source/ev3/ev3.h"
#include "/src/ev3dev-c/source/ev3/ev3_port.h"
#include "/src/ev3dev-c/source/ev3/ev3_tacho.h"
#include "/src/ev3dev-c/source/ev3/ev3_sensor.h"
#include "dead_reckoning.h"

#define Sleep( msec ) usleep(( msec ) * 1000 )
#define LEFT_BORDER -80 //Width of the field in centimeter
#define RIGHT_BORDER 50 //Length of the field
#define UP_BORDER 40
#define BOTTOM_BORDER -50
#define CLOCK_PER 0.12

#define PI 3.14159
#define WHEEL_DIAM 5.5 //Wheels' diameter is 5.5 cm

struct position current_position;

void init_pos()
{
    current_position.state = malloc(32*sizeof(char));
    current_position.state = "TACHO_STOP";
    current_position.x = 0.0;
    current_position.y = 0.0;
    current_position.theta = 90;
}

void rewrite_state(char * state){
    current_position.state = state;
}

float get_theta(){
    return current_position.theta;
}

float get_x(){
    return current_position.x;
}

float get_y(){
    return current_position.y;
}

/* Thread, to know the position and the orientation of the robot at any time */
void *dead_reckoning(void* unused)
{
    uint8_t sn_right, sn_left;
    int speed_left, speed_right, count_per_rot;
    int port_left = 66;
    int port_right = 67;
    float theta;
    float dist;
    char * state = malloc(32*sizeof(char));

    if( ev3_search_tacho_plugged_in(port_left, 0, &sn_left, 0) &&  ev3_search_tacho_plugged_in(port_right,0, &sn_right, 0 )) {
        get_tacho_count_per_rot(sn_left, &count_per_rot); // Set count_per_rots
        while(1)
        {
            get_tacho_speed_sp(sn_left, &speed_left); // Get left_speed
            get_tacho_speed_sp(sn_right, &speed_right); // Get right_speed
            state = current_position.state;
            
           // printf("State: %s      x: %f    y: %f     theta: %f\n", state, current_position.x, current_position.y, current_position.theta);
            
            if(speed_left == (-speed_right))
            {
                /* The robot is just turning around */
                current_position.x = current_position.x;
                current_position.y = current_position.y;
                /* Theta is updated at the end of the turn() function */
            }
            else{
                while (speed_left == speed_right && strcmp((const char *) "TACHO_STOP", (const char *) state) != 0)
                {
                    /* Moving in a straight line */
                    theta = current_position.theta;
                    /* Dist is neg if speed_left is neg */
                    dist = (float) ((CLOCK_PER * (float)(speed_left) * PI * WHEEL_DIAM ) / (float)count_per_rot) ; // Distance runned in CLOCK_PER seconds
                    
                    /* Update current_position.x and current_position.y */
                    if(theta <= 90)
                    {
                        current_position.x += dist * cos((theta*PI)/180);
                        current_position.y += dist * sin((theta*PI)/180);
                    }
                    else if ( 90 <= theta && theta <= 180){
                        current_position.x -= dist * sin((theta*PI)/180 - PI/2);
                        current_position.y += dist * cos((theta*PI)/180 - PI/2);
                    }
                    else if ( 180 <= theta && theta <= 270){
                        current_position.x -= dist * cos((theta*PI)/180 - PI);
                        current_position.y -= dist * sin((theta*PI)/180 - PI);
                    }
                    else if(theta > 270){
                        current_position.x += dist * cos(2*PI - (theta*PI)/180);
                        current_position.y += dist * sin(2*PI - (theta*PI)/180);
                    }
    
                    /* ****************************************** */
                    
                    Sleep(CLOCK_PER * 500); // Less than 1000 because otherwise it sleeps too much and the robot travels to much distance
                    get_tacho_speed(sn_left, &speed_left); // Get left_speed
                    get_tacho_speed(sn_right, &speed_right); // Get right_speed
                    state = current_position.state; // Update state
                    
                    /* Check if the robot does not cross the limits of the field */
                    if(current_position.x > RIGHT_BORDER || current_position.x < LEFT_BORDER || current_position.y > UP_BORDER || current_position.y < BOTTOM_BORDER){
                        set_tacho_command_inx( sn_left, TACHO_STOP);
                        set_tacho_command_inx( sn_right, TACHO_STOP);
                        rewrite_state((char *)"TACHO_STOP");
                    }
                }
            }
            
        Sleep(10);
        }
    }
    else {
        printf( "LEGO_EV3_M_MOTOR NOT found\n");
    }
    
    pthread_exit(NULL);
}
    

int turn(float angle) {
    uint8_t sn_left, sn_right, sn_compass;
    int max_speed_left, max_speed_right, max_speed;
    int port_left = 66; /* Left wheel */
    int port_right = 67; /* Right wheel */
    float value, init, sign_angle;
    
    if (ev3_search_sensor(LEGO_EV3_GYRO, &sn_compass,0)){
        get_sensor_value0(sn_compass, &init );
    }
    else {
        return 0;
    }
    
    if( ev3_search_tacho_plugged_in(port_left, 0, &sn_left, 0) &&  ev3_search_tacho_plugged_in(port_right,0, &sn_right, 0 ) && angle != 0) {
        get_tacho_max_speed(sn_left, &max_speed_left);
        get_tacho_max_speed(sn_left, &max_speed_right);

        set_tacho_stop_action_inx( sn_left, TACHO_BRAKE );
        set_tacho_stop_action_inx( sn_right, TACHO_BRAKE );
        
        if(max_speed_left == max_speed_right || max_speed_left < max_speed_right)  /* Set max_speed to the lowest max_speed (left or right) */
            max_speed = max_speed_left;
        else
            max_speed = max_speed_right;
        
        /* Get the sign of angle */
        sign_angle = angle / abs(angle);
        /* Set tacho speed to (1/30) * max_speed */
        set_tacho_speed_sp( sn_left, sign_angle * max_speed * 1/40);
        set_tacho_speed_sp( sn_right, - sign_angle * max_speed * 1/40);
        /* Get sensor compass value */
        get_sensor_value0(sn_compass, &value);
        
        /* Run the tachos */
        set_tacho_command_inx(sn_left, TACHO_RUN_FOREVER);
        set_tacho_command_inx(sn_right, TACHO_RUN_FOREVER);
        
        /* Turn until it has turned enough (i.e. until it has reached its goal */
        while(sign_angle * (float)(((int)value - ((int)angle  + (int)init)) % 360) != 0.0){
            get_sensor_value0(sn_compass, &value);
        }
        
        /* Stop the tachos and update current_position.theta */
        set_tacho_command_inx( sn_left, TACHO_STOP);
        set_tacho_command_inx( sn_right, TACHO_STOP);
        current_position.theta -= angle;
        current_position.theta = (float) ((int)current_position.theta % 360);
        return 1;
    }
    else {
        printf( "Exit turn function (motor not found or angle = 0)\n");
        return 0;
    }
}

/* Turn the robot so its orientation is the smae as its beginning, i.e. so that current_position.theta = 90 degrees */
void init_orientation(){
    float angle = current_position.theta;
    turn(-(90-angle));
}

/* Move to the position with the coordinates x, y */
int move_to_xy(float x, float y){
    uint8_t sn_right, sn_left;
    int port_left = 66;
    int port_right = 67;
    int max_speed, max_speed_left, max_speed_right;
    float x0, y0, cos_theta, sin_theta, dist, theta, angle;
    char * state = malloc(32*sizeof(char));

    if( ev3_search_tacho_plugged_in(port_left, 0, &sn_left, 0) &&  ev3_search_tacho_plugged_in(port_right,0, &sn_right, 0 )) {
        get_tacho_max_speed( sn_left, &max_speed_left);
        get_tacho_max_speed( sn_right, &max_speed_right);
        
        set_tacho_stop_action_inx( sn_left, TACHO_BRAKE );
        set_tacho_stop_action_inx( sn_right, TACHO_BRAKE );

        /* Set max_speed to the lowest max_speed (left or right) */
        if(max_speed_left == max_speed_right || max_speed_left < max_speed_right)
            max_speed = max_speed_left;
        else
            max_speed = max_speed_right;
        
        x0 = current_position.x; // Get current position (x)
        y0 = current_position.y; // Get current position (y)
        
        /* Calculate the distance to be travelled */
        dist = sqrt(pow((x-x0),2) + pow((y-y0),2));
        
        /* Calculate the angle to be turned */
        if( x > x0 && y > y0){
            cos_theta = (y - y0)/dist;
            sin_theta = (x - x0)/dist;
            theta = atan(sin_theta/cos_theta);
        }
        else if( x < x0 && y < y0){
            cos_theta = (x0 - x)/dist;
            sin_theta = (y0 - y)/dist;
            theta = atan(sin_theta/cos_theta) + PI/2;
        }
        else if( x < x0 && y > y0){
            cos_theta = (y - y0)/dist;
            sin_theta = (x0 - x)/dist;
            theta = atan(sin_theta/cos_theta);
        }
        else if (x > x0 && y < y0){
            cos_theta = (x - x0)/dist;
            sin_theta = (y0 - y)/dist;
            theta = atan(sin_theta/cos_theta) + PI/2;
        }
        else {return 0; } //Already here
        
        /* Turn to be in the right direction */
        angle = current_position.theta - 90; // The diff angle from the beginning direction
        printf("Angle that need to be turned: %f \n", round(((((180*theta)/PI)-angle)/10)*10));
        /* Need to turn the other way around under some conditions */
        if(angle > 180){
            angle = 180 - angle;
        }
        if(x0 < x)
            turn(round((((180*theta)/PI)-angle)/10)*10);
        else
            turn(-(round(((180*theta)/PI)-angle)/10)*10);
        
        /* Set tacho speed to (1/3) * max_speed */
        set_tacho_speed_sp( sn_left, max_speed * 1/4);
        set_tacho_speed_sp( sn_right, max_speed * 1/4);
        set_tacho_command_inx( sn_left, TACHO_RUN_FOREVER );
        set_tacho_command_inx( sn_right, TACHO_RUN_FOREVER );
        rewrite_state((char *)"TACHO_RUN_FOREVER");
        state = current_position.state;
        
        /* Move until the robot has reached his destination */
        while((round(x0) < round(x)-2 || round(x0) > round(x)+2 || round(y0) < round(y)-2 || round(y0) > round(y)+2) && strcmp((const char *) "TACHO_STOP", (const char *) state) != 0)
        {
            //printf("round(x0): %f, round(x): %f, round(y0): %f, round(y): %f \n", round(x0), round(x), round(y0), round(y));
            x0 = current_position.x;
            y0 = current_position.y;
            state = current_position.state;
        }
        
        /* Stop the tachos and update current_position.state */
        set_tacho_command_inx( sn_left, TACHO_STOP);
        set_tacho_command_inx( sn_right, TACHO_STOP);
        rewrite_state((char *)"TACHO_STOP");
        printf("State in move_to_xy: %s\n", current_position.state);
    }
    else {
        printf( "LEGO_EV3_M_MOTOR NOT found\n");
        return 1;
    }
    
    return 0;
}


/* Makes the robot move forward for dist cm (if dist is neg, move back) */
void move_forward(int dist){
    uint8_t sn_left, sn_right;
    int port_left = 66; /* Left wheel */
    int port_right = 67; /* Right wheel */
    int max_speed, max_speed_left, max_speed_right, count_per_rot;
    float t=0; //t in sec
    float  theta;
    
    if ( ev3_search_tacho_plugged_in(port_left,0, &sn_left, 0 ) &&  ev3_search_tacho_plugged_in(port_right,0, &sn_right, 0 )) {
        /* Get max_speed and get count_per_rot */
        get_tacho_count_per_rot(sn_left, &count_per_rot);
        get_tacho_max_speed( sn_left, &max_speed_left);
        get_tacho_max_speed( sn_right, &max_speed_right);
        
        set_tacho_stop_action_inx( sn_left, TACHO_BRAKE );
        set_tacho_stop_action_inx( sn_right, TACHO_BRAKE );
        
        if(max_speed_left == max_speed_right || max_speed_left < max_speed_right) /* Set max_speed to the lowest max_speed (left or right) */
            max_speed = max_speed_left;
        else
            max_speed = max_speed_right;
        
        /* Set tacho speed to (1/3) * max_speed */
        if(dist > 0) {
            set_tacho_speed_sp( sn_left, max_speed * 1/4);
            set_tacho_speed_sp( sn_right, max_speed * 1/4);
        }
        else {
            set_tacho_speed_sp( sn_left, - max_speed * 1/4);
            set_tacho_speed_sp( sn_right, - max_speed * 1/4);
        }
        
        /* Calculate time the tacho will run */
        t = (float) (count_per_rot * abs(dist) )/(max_speed * 1/4 * PI * WHEEL_DIAM) ;
        printf("%d %d %d\n", count_per_rot, abs(dist), max_speed);
        printf("Tacho will run for %f seconds \n", t);
        
        /* Set tachos to run for t seconds */
        set_tacho_time_sp( sn_left, t*1000 );
        set_tacho_time_sp( sn_right, t*1000 );
        set_tacho_command_inx( sn_left, TACHO_RUN_TIMED );
        set_tacho_command_inx( sn_right, TACHO_RUN_TIMED );
        
        Sleep(t*1000);
        theta = current_position.theta;
        current_position.x += dist * cos((theta*PI)/180);
        current_position.y += dist * sin((theta*PI)/180);
        
    } else {
        printf( "LEGO_EV3_M_MOTOR NOT found\n");
    }
    
    /* Waiting for the tacho to stop */
    Sleep( t*1000 );
}


/****************************************************************/
/*                      Main function                           */
/****************************************************************/

/*
int main() {
    init_robot();
    init_pos();
    pthread_t reckoning_thread;
    pthread_create(&reckoning_thread, NULL, dead_reckoning, NULL);

    move_to_xy(30,15);
    move_to_xy(0,15);
    move_to_xy(0,0);
    init_orientation();
    
    exit_robot();
    
    return 0;
}
*/
