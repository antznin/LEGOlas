/**************************************************************************************************/
/************************     MAIN FUNCTION FOR THE 18th DECEMBER     *****************************/
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

#define Sleep( msec ) usleep(( msec ) * 1000 )
#define ARM_LENGTH 10 /* Length of the robot's arm that will catch the ball */


/* Scan the entire field in 5 zones to find the ball and move to it */
/*            Returns 0 if no ball found, 1 otherwise               */

int explore(){
    struct values scan0;
    struct values scan1;
    struct values scan2;
    struct values scan3;
    struct values scan4;
    struct values scan5;
    struct values scan6;
    struct values scan7;
    
    /* Fisrt scans : see if the ball is somewhere around */
    scan0 = scan(55, 20);
    if(scan0.angle != 0.0 && scan0.radius != 0.0){
        move_forward((scan0.radius - ARM_LENGTH)); //To be able to catch the ball the robot must be 5cm away from it
        printf("Ball found! \n");
        return 1;
    }
    scan1 = scan(250, 50);
    if(scan1.angle != 0.0 && scan1.radius != 0.0){
        move_forward((scan1.radius - ARM_LENGTH));
        printf("Ball found! \n");
        return 1;
    }
    scan2 = scan(55, 20);
    if(scan2.angle != 0.0 && scan2.radius != 0.0){
        move_forward((scan2.radius - ARM_LENGTH)); //To be able to catch the ball the robot must be 5cm away from it
        printf("Ball found! \n");
        return 1;
    }
    
    move_forward(15);
    turn(-90);
    
    /* Fourth scan to know if it can move forward without kicking the ball */
    turn(-10);
    scan3 = scan(10, 50);
    if(scan3.angle != 0.0 && scan3.radius != 0.0){
        move_forward((scan3.radius - ARM_LENGTH));
        printf("Ball found! \n");
        return 1;
    }
    
    /* Go to stage2 and scan */
    move_forward(38);
    turn(-90);
    scan4 = scan(-80, 40);
    if(scan4.angle != 0.0 && scan4.radius != 0.0){
        move_forward((scan4.radius - ARM_LENGTH));
        printf("Ball found! \n");
        return 1;
    }
    turn(80);
    
    /* Go to stage3 and scan */
    move_forward(80);
    turn(90);
    scan5 = scan(-80, 40);
    if(scan5.angle != 0.0 && scan5.radius != 0.0){
        move_forward((scan5.radius - ARM_LENGTH));
        printf("Ball found! \n");
        return 1;
    }
    turn(80);
    
    /* Go to stage4 and scan */
    move_forward(80);
    turn(-90);
    scan6 = scan(-80, 60);
    if(scan5.angle != 0.0 && scan5.radius != 0.0){
        move_forward((scan5.radius - ARM_LENGTH));
        printf("Ball found! \n");
        return 1;
    }
    turn(80);
    
    /* Go to stage5 and scan */
    move_forward(80);
    turn(-90);
    scan7 = scan(-80, 60);
    if(scan7.angle != 0.0 && scan7.radius != 0.0){
        move_forward((scan7.radius - ARM_LENGTH));
        printf("Ball found! \n");
        return 1;
    }
    
    printf("Didn't find any ball!\n");
    return 0;
}

int main(){
    int is_init;
    /* Check for the right initialization of the robot */
    is_init = init_robot();
    if(is_init == -1){
        printf("Failed to init \n");
        return -1;
    }
    
    int ball_found = explore();
    if(ball_found){
        
    }
    else{
        
    }
    exit_robot();
}


