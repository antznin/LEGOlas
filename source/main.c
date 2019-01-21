/**************************************************************************************************/
/************************      MAIN FUNCTION FOR THE 22th JANUARY     *****************************/

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
#define ARM_LENGTH 8 /* Length of the robot's arm that will catch the ball */


/***********************      STRATEGY 1      **********************/
/*   Throw the first two balls and then go to the opponent's field */
/*******************************************************************/

void strategy1(){
    
    /* Go to the opponent's field */
    turn(30);
    curved_turn(0.62, 1, 3000);
    release_catapult();
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
    
    /* Move to scan position */
    printf("HEYYYYYYYYY\n");
    move_to_xy(-6,9);
    init_orientation();
    Sleep(1000); 
    printf("after move_to_xy and init_orientation\n");
    /* Then scan, find a ball, go to shoot position, shoot and scan again */
    while(1){
        first_scan_values = single_scan(DELAY,1,45,1, 360);
        second_scan_values = single_scan(DELAY,2,45,-1, first_scan_values.angle);
        count += 1;
        
        if (are_close(first_scan_values,second_scan_values)){
            ball_position.radius = (first_scan_values.radius + second_scan_values.radius)/2;
            ball_position.angle = (first_scan_values.angle + second_scan_values.angle)/2;
            printf("Ball is at radius %f , angle %f ", ball_position.radius,ball_position.angle);
            turn(ball_position.angle);
            release_catapult();
            move_forward((int)(ball_position.radius/10) - ARM_LENGTH);
            catch_ball(1600);
            /* Go to shoot position */
            move_to_xy(0,0);
            move_forward(-8);
            init_orientation();
            dismiss_hand_wc();
            throw_ball();
        }
        
	printf("%d\n", count%3);
        if((count % 3) == 1){
            move_to_xy(-6,9);
            init_orientation();
        }
        else if((count % 3) == 2){
            move_to_xy(-10, -9);
            init_orientation();
        }
        else if((count % 3) == 0){
            move_to_xy(10, -9);
            init_orientation();
        }
    }
}


int main(){
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
    move_forward(-8);
    throw_ball();
    dismiss_hand_wc();
    catch_ball(1600);
    dismiss_hand_wc();
    throw_ball();
    hold_catapult_for_movement();
    move_forward(8); 
    move_to_xy(-6,9);
    //strategy2();

    exit_robot();
}


