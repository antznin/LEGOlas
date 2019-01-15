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

#define Sleep( msec ) usleep(( msec ) * 1000 )
#define FIELD_WIDTH 120 //Width of the field in centimeter
#define FIELD_LENGTH 100 //Length of the field
#define CLOCK_PER 0.12

#define PI 3.14159
#define WHEEL_DIAM 5.5 //Wheels' diameter is 5.5 cm

struct position
{
    char * state;   // TACHO_STOP, TACHO_RUN_FOREVER, etc
    float x;        // in centimeter
    float y;        // in centimeter
    float theta;    // in degree (counterclockwise from x-axis)
};

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

void *dead_reckoning(void* unused)  //Thread, to know the position at any time
{
    uint8_t sn_right, sn_left, sn_compass;
    int speed_left, speed_right, sign_speed, count_per_rot;
    int port_left = 66;
    int port_right = 67;
    float theta; //In radian
    float value; //In degree, compass output
    float dist;
    char * state = malloc(32*sizeof(char));

    if (ev3_search_sensor(LEGO_EV3_GYRO, &sn_compass,0)){
        printf("COMPASS found in DEAD_RECK, reading compass...\n");
        if ( !get_sensor_value0(sn_compass, &value )) {
            value = 0;
        }
    }
    else {
        printf( "COMPASS NOT in DEAD_RECK found\n");
    }
    
    if( ev3_search_tacho_plugged_in(port_left, 0, &sn_left, 0) &&  ev3_search_tacho_plugged_in(port_right,0, &sn_right, 0 )) {
        get_tacho_count_per_rot(sn_left, &count_per_rot); // Set count_per_rots
        while(1)
        {
            get_tacho_speed_sp(sn_left, &speed_left); // Get left_speed
            get_tacho_speed_sp(sn_right, &speed_right); // Get right_speed
            state = current_position.state;
            
            printf("State: %s      x: %f    y: %f     theta: %f\n", state, current_position.x, current_position.y, current_position.theta);
            
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
                    sign_speed = speed_left / abs(speed_left);
                    theta = current_position.theta;
                    dist = (float)sign_speed * ((CLOCK_PER * (float)(speed_left) * PI * WHEEL_DIAM ) / (float)count_per_rot) ; // Distance runned in CLOCK_PER seconds
                    printf("Distance parcourue: %f \n", dist);
                    printf("theta: %f\n", theta);
                    
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
                        current_position.y -= dist * sin(2*PI - (theta*PI)/180);
                    }
                    printf("x: %f, y:%f \n", current_position.x, current_position.y);
                    /* ****************************************** */
                    
                    Sleep(CLOCK_PER * 500);
                    get_tacho_speed(sn_left, &speed_left); // Get left_speed
                    get_tacho_speed(sn_right, &speed_right); // Get right_speed
                    state = current_position.state;
                    
                    /* Check if the robot does not cross the limits of the field */
                    if(abs(current_position.x) > FIELD_WIDTH / 2 || abs(current_position.y) > FIELD_LENGTH / 2){
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
    

void turn(float angle) {
    uint8_t sn_left, sn_right, sn_compass;
    int max_speed_left, max_speed_right, max_speed;
    int port_left = 66; /* Left wheel */
    int port_right = 67; /* Right wheel */
    float value, init, sign_angle;
    
    if (ev3_search_sensor(LEGO_EV3_GYRO, &sn_compass,0)){
        get_sensor_value0(sn_compass, &init );
    }
    
    if( ev3_search_tacho_plugged_in(port_left, 0, &sn_left, 0) &&  ev3_search_tacho_plugged_in(port_right,0, &sn_right, 0 ) && angle != 0) {
        set_tacho_stop_action_inx( sn_left, TACHO_STOP );
        set_tacho_stop_action_inx( sn_right, TACHO_STOP );
        
        get_tacho_max_speed(sn_left, &max_speed_left);
        get_tacho_max_speed(sn_left, &max_speed_right);
        
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
        
        while(sign_angle * (float)(((int)value - ((int)angle  + (int)init)) % 360) != 0.0){
            get_sensor_value0(sn_compass, &value);
            printf("Value - (angle + init ) = %f \n ", value - (angle + init));
        }
        
        set_tacho_command_inx( sn_left, TACHO_STOP);
        set_tacho_command_inx( sn_right, TACHO_STOP);
        current_position.theta -= angle;
        //current_position.theta = (float) ((int)current_position.theta % 360);
        printf("Theta at the end of turn function: %f \n", current_position.theta);
    }
    else {
        printf( "Exit turn function (motor not found or angle = 0)\n");
    }
}


void init_orientation(){
    float angle = current_position.theta;
    turn(-(90-angle));
}

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
        
        if(max_speed_left == max_speed_right || max_speed_left < max_speed_right) /* Set max_speed to the lowest max_speed (left or right) */
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
        else { return 0; } //Already here
        
        /* Turn to be in the right direction */
        angle = current_position.theta - 90; // The diff angle from the beginning direction
        printf("Angle that need to be turned: %f \n", abs((180*theta)/PI)+angle);
        /* Need to turn the other way around under some conditions */
        if(angle > 180){
            angle = 180 - angle;
        }
        if(x0 < x)
            turn(abs((180*theta)/PI)-angle);
        else
            turn(-(abs((180*theta)/PI)-angle));
        
        /* Set tacho speed to (1/3) * max_speed */
        set_tacho_speed_sp( sn_left, max_speed * 1/4);
        set_tacho_speed_sp( sn_right, max_speed * 1/4);
        set_tacho_command_inx( sn_left, TACHO_RUN_FOREVER );
        set_tacho_command_inx( sn_right, TACHO_RUN_FOREVER );
        rewrite_state((char *)"TACHO_RUN_FOREVER");
        state = current_position.state;
        
        /* Move until the robot has reached his destination */
        
        while((round(x0) < round(x)-1 || round(x0) > round(x)+1 || round(y0) < round(y)-1 || round(y0) > round(y)+1) && strcmp((const char *) "TACHO_STOP", (const char *) state) != 0)
        {
            printf("round(x0): %f, round(x): %f, round(y0): %f, round(y): %f \n", round(x0), round(x), round(y0), round(y));
            x0 = current_position.x;
            y0 = current_position.y;
            state = current_position.state;
        }
        
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


/****************************************************************/
/*                  Init and exit functions                     */
/****************************************************************/

int exit_robot(void){ //Exit the ev3
    ev3_uninit();
    printf( "*** ( EV3 ) Bye! ***\n" );
    return ( 0 );
}

int init_robot( void ) // Find the tachos
{
    uint8_t sn_compass;
    float init_value_compass;
    int i;
    char s[256];
    
    if(ev3_tacho_init() == -1)
        return 1;
    
    while ( ev3_tacho_init() < 1 ) Sleep( 1000 );
    
    printf( "Found tacho motors:\n" );
    for ( i = 0; i < DESC_LIMIT; i++ ) {
        if ( ev3_tacho[ i ].type_inx != TACHO_TYPE__NONE_ ) {
            printf( "  type = %s\n", ev3_tacho_type( ev3_tacho[ i ].type_inx ));
            printf( "  port = %s\n", ev3_tacho_port_name( i, s ));
            printf("  port = %d %d\n", ev3_tacho_desc_port(i), ev3_tacho_desc_extport(i));
        }
    }
    
    //Run all sensors
    ev3_sensor_init();
    if (ev3_search_sensor(LEGO_EV3_GYRO, &sn_compass,0)){
        printf("COMPASS found IN MAIN, reading compass...\n");
        if ( !get_sensor_value0(sn_compass, &init_value_compass )) {
            init_value_compass = 0;
        }
        get_sensor_value0(sn_compass, &init_value_compass);
    }
    
    return 0;
}


int main() {
    init_robot();
    init_pos();
    pthread_t reckoning_thread;
    pthread_create(&reckoning_thread, NULL, dead_reckoning, NULL);
    
    move_to_xy(30,15);
    move_to_xy(0,0);
    /*
    move_to_xy(-30, 30);
    //init_orientation();
    move_to_xy(-30, -30);
    //init_orientation();
    move_to_xy(30, -30);
    //init_orientation();
    move_to_xy(0,0);*/
    
    exit_robot();
    
    return 0;
}

