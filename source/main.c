/**************************************************************************************************/
/************************      MAIN FUNCTION FOR THE 22th JANUARY     *****************************/
/************************         Written by Yasmine Bennani          *****************************/
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

#define Sleep( msec ) usleep(( msec ) * 1000 )
#define ARM_LENGTH 8 /* Length of the robot's arm that will catch the ball */


/***********************      STRATEGY 1      **********************/
/*   Throw the first two balls and then go to the opponent's field */
/*******************************************************************/

void strategy1(){
    /* Throw the first two balls */
    throw_ball();
    Sleep(500);
    catch_ball();
    dismiss_hand_nc();
    Sleep(500);
    throw_ball();
    
    /* Go to the opponent's field */
    curved_turn(0.5, 1, 1000);
    Sleep(120 * 1000); // Sleep for 2 minutes
}

/***********************      STRATEGY 2      **********************/
/*   Throw the first two balls and then scan to look for the ball  */
/*******************************************************************/

void stratey2(){
    /* Throw the first two balls */
    throw_ball();
    Sleep(500);
    catch_ball();
    dismiss_hand_nc();
    Sleep(500);
    throw_ball();
    
    /* Then scan, find a ball, go to shoot position, shoot and scan again */
    
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
    
    strategy1();

    exit_robot();
}


