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

int total_scan(int angle, int radius, int step_id){
    struct values scan_values;
    int sign, ball_found;

    if (angle > 0) {
        sign = 1;
    } else {
        sign = -1;
    }

    scan_values = scan(angle, radius);
    if(scan_values.angle != 0.0 && scan_values.radius != 0.0){
        ball_found = move_to_ball((int)scan_values.radius); //To be able to catch the ball the robot must be 5cm away from it
        if(ball_found) {
           // TO ADD : Sami's functions
            move_forward( - (int)(scan_values.radius - ARM_LENGTH));
            switch (step_id){
                case 1: turn(-(sign * scan_values.angle));
                case 2: turn(- (55.0 + scan_values.angle));
                case 3: turn(55 - scan_values.angle);
            }
            printf("Ball found! \n");
            return 1;
        }
        else {
            printf("No ball to catch \n");
            move_forward(-(int)(scan_values.radius + 5));
            turn(sign * (angle - scan_values.angle));
            return 0;
        }
    }
    return 0;
}

/* Scan the entire field in 5 zones to find the ball and move to it */
/*            Returns 0 if no ball found, 1 otherwise               */

int explore(){
    int res1;
    int res2;
    int res3;
    int res4;
    int res5;
    int res6;
    int res7;
    int res8;

    /* Fisrt scans : see if the ball is somewhere around (STAGE 1)*/
    res1 = total_scan(55, 20, 1);
    if(res1)
        return 1;

    res2 = total_scan(250, 30, 2);
    if(res2)
        return 1;

    res3 = total_scan(55, 20, 3);
    if(res3)
        return 1;

    move_forward(15);
    turn(-90);

    /* Fourth scan to know if it can move forward without kicking the ball */
    turn(-10);
    res4 = total_scan(10, 50, 1);
    if(res4)
        return 1;

    /* Go to STAGE 2 and scan */
    move_forward(38);
    turn(-90);
    res5 = total_scan(-80, 50, 1);
    if(res5)
        return 2;
    turn(80);

    /* Go to STAGE 3 and scan */
    move_forward(80);
    turn(90);
    res6 = total_scan(-80, 50, 1);
    if(res6)
        return 3;
    turn(80);

    /* Go to STAGE 4 and scan */
    move_forward(80);
    turn(-90);
    res7 = total_scan(-80, 50, 1);
    if(res7)
        return 4;
    turn(80);

    /* Go to STAGE 5 and scan */
    move_forward(80);
    turn(-90);
    res8 = total_scan(-80, 50, 1);
    if(res8)
        return 5;

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
        switch (ball_found){
            case 1: shoot_from_stage1();
            case 2: shoot_from_stage2();
            case 3: shoot_from_stage3();
            case 4: shoot_from_stage4();
            case 5: shoot_from_stage5();
        }
    }
    else{
        printf("NO BALL FIND: EXITED \n");
    }
    exit_robot();
}


