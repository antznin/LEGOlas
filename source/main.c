/**************************************************************************************************/
/************************      MAIN FUNCTION FOR THE 22th JANUARY     *****************************/
/**************************************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"
#include "movement.h"
#include "ballDetection.h"
#include "ballaunch.h"
#include "dead_reckoning.h"
#include "client.h"

#define Sleep( msec ) usleep(( msec ) * 1000 )
#define DELAY 5000
#define ARM_LENGTH 15 /* Length of the robot's arm that will catch the ball */
#define PI 3.14159

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


/***********************      STRATEGY 1      **********************/
/*   Throw the first two balls and then go to the opponent's field */
/*******************************************************************/

void strategy1(){
    /* Go to the opponent's field */
    turn(45);
    curved_turn(0.62, 1, 4500);
    release_catapult();
    move_forward(15);
    hold_catapult_for_movement();
    Sleep(120 * 1000); // Sleep for 2 minutes
}

/***********************      STRATEGY 2      **********************/
/*   Throw the first two balls and then scan to look for the ball  */
/*******************************************************************/

void strategy2(){
    struct values first_scan_values;
    struct values second_scan_values;
    struct values ball_position;
    int x,y;
    int found = 0;
    int count = 0;
    float theta;
    
    /* Then scan, find a ball, go to shoot position, shoot and scan again */
    while(1){
        count += 1;
        
        /* Move to the three different scan positions */
        if((count % 4) == 1){
            move_to_xy(0,8);
            init_orientation();
        }
        else if((count % 4) == 2){
            move_to_xy(-8, 8);
            init_orientation();
        }
        else if((count % 4) == 3){
            move_to_xy(-8, -8);
            init_orientation();
        }
        else if((count % 4) == 0){
            move_to_xy(8, -8);
            init_orientation();
        }
        
        first_scan_values = single_scan(1000,1,1,360);
        printf("first scan %f %f \n",first_scan_values.radius,first_scan_values.angle);
        second_scan_values = single_scan(1000,2,-1,first_scan_values.angle);
        printf("second scan %f %f \n",second_scan_values.radius,first_scan_values.angle);
        
        if (are_close(first_scan_values,second_scan_values)){
            ball_position.radius = (first_scan_values.radius + second_scan_values.radius)/2;
            ball_position.angle = (first_scan_values.angle + second_scan_values.angle)/2;
            printf("Ball is at radius %f , angle %f ", ball_position.radius,ball_position.angle);
            theta = get_theta();
            turn((180*theta)/PI - ball_position.angle);
            turn(ball_position.angle);
            dismiss_hand_nc();
            release_catapult();
            move_forward((int)(ball_position.radius/10) - ARM_LENGTH);
            catch_ball(1600);
            /* Go to shoot position */
            move_to_xy(0,0);
            init_orientation();
            move_forward(-8);
            dismiss_hand_wc();
            throw_ball();
            send_score(3);
            hold_catapult_for_movement();
            place_hand_for_movement();
        }
    }
}


int main(int argc, char * argv []){
    int is_init;
    /* Check for the right initialization of the robot */
    is_init = init_robot();
    if(is_init == -1){
        printf("Failed to init \n");
        return -1;
    }
    
    /* Inits the position of the robot and starts the thread */
    init_pos();
    pthread_t reckoning_thread;
    pthread_create(&reckoning_thread, NULL, dead_reckoning, NULL);

    connect_bt();
    while (hasStarted != 1) {};

    /* Throw the first two balls */
    dismiss_hand_wc();
    move_forward(-8);
    throw_ball();
    send_score(3);
    catch_ball(1400);
    dismiss_hand_wc();
    throw_ball();
    send_score(3);
    hold_catapult_for_movement();
    place_hand_for_movement();
    
    if(strcmp((const char *) argv[1], (const char *) "1") == 0){
        strategy1();
    }
    else if(strcmp((const char *) argv[1], (const char *) "2") == 0){
        strategy2();
    }

    exit_robot();
}


