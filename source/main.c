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

int main(){
    int is_init;
    struct values scan1;
    struct values scan2;
    struct values scan3;
    struct values scan4;
    struct values scan5;
    struct values scan6;
    struct values scan7;

    /* Check for the right initialization of the robot */
    is_init = init_robot();
    if(is_init == -1){
        printf("Failed to init \n");
        return -1;
    }

    /* Fisrt scan : the limit is the opponent field separation */
    turn(-45);
    scan1 = scan(45, 10);
    if(scan1.angle != 0.0 && scan1.radius != 0.0){
        move_forward((scan1.radius - 5)); //To be able to catch the ball the robot must be 5cm away from it
        printf("Ball found! \n");
        return 1;
    }

    /* Second scan : see if the ball is somewhere around and then go to stage1 */
    scan2 = scan(270, 50);
    if(scan2.angle != 0.0 && scan2.radius != 0.0){
        move_forward((scan2.radius - 5));
        printf("Ball found! \n");
        return 1;
    }
    turn(45);
    move_forward(10);
    turn(-90);

    /* Third scan to know if it can move forward without kicking the ball */
    turn(-10);
    scan3 = scan(20, 50);
    if(scan3.angle != 0.0 && scan3.radius != 0.0){
        move_forward((scan3.radius - 5));
        printf("Ball found! \n");
        return 1;
    }
    turn(-10);

     /* Go to stage2 and scan */
    move_forward(50);
    turn(-90);
    scan4 = scan(-90, 60);
    if(scan4.angle != 0.0 && scan4.radius != 0.0){
        move_forward((scan4.radius - 5));
        printf("Ball found! \n");
        return 1;
    }
    turn(90);

    /* Go to stage3 and scan */
    move_forward(90);
    turn(90);
    scan5 = scan(-90, 60);
    if(scan5.angle != 0.0 && scan5.radius != 0.0){
        move_forward((scan5.radius - 5));
        printf("Ball found! \n");
        return 1;
    }
    turn(90);

    /* Go to stage4 and scan */
    move_forward(90);
    turn(-90);
    scan6 = scan(-90, 60);
    if(scan5.angle != 0.0 && scan5.radius != 0.0){
        move_forward((scan5.radius - 5));
        printf("Ball found! \n");
        return 1;
    }
    turn(90);

    /* Go to stage5 and scan */
    move_forward(90);
    turn(-90);
    scan7 = scan(-90, 60);
    if(scan7.angle != 0.0 && scan7.radius != 0.0){
        move_forward((scan7.radius - 5));
        printf("Ball found! \n");
        return 1;
    }

    printf("Didn't find any ball!\n");
    exit_robot();
    return 0;
}


