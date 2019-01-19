/**************************************************************************************************/
/************************     MAIN FUNCTION FOR THE 18th DECEMBER     *****************************/
/************************         Written by                 *****************************/
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
#define ARM_LENGTH 20 /* Length of the robot's arm that will catch the ball */

#define DELAY 1000

int main(){
	int is_init;
    	/* Check for the right initialization of the robot */
    	is_init = init_robot();
    	if(is_init == -1){
    	    printf("Failed to init \n");
    	    return -1;
    	}
    	
    	// Arm initial movement
	move_forward(-9);
	Sleep(1500);
    	throw_ball();
    	Sleep(1000);
    	catch_ball(600);
    	dismiss_hand_wc();
    	throw_ball();
    	Sleep(1000);
    	hold_catapult_for_movement();


    	/* execute 2 scans and get relevant values i.e values that are close from each other */
	struct values first_scan_values;
	struct values second_scan_values;
	first_scan_values = single_scan(DELAY,1,45,1, 360);
	second_scan_values = single_scan(DELAY,2,45,-1, first_scan_values.angle);
	struct values ball_position;
	int x,y;
	int found = 0;
	



	// for (x=0 ; x<5 && not_found; x++){
	// 	for (y=0; y<5 ; y++){
	if (are_close(first_scan_values,second_scan_values)){
		ball_position.radius = (first_scan_values.radius + second_scan_values.radius)/2;
		ball_position.angle = (first_scan_values.angle + second_scan_values.angle)/2;
		printf("Ball is at radius %f , angle %f ", ball_position.radius,ball_position.angle);	
		turn(ball_position.angle);
		release_catapult();
		move_forward((int)(ball_position.radius/10) - ARM_LENGTH);
		catch_ball(1600);
		dismiss_hand_wc();
		throw_ball();
	}
	release_catapult();
	// 	}
	// }

    exit_robot();
}


