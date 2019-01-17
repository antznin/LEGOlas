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
#define ARM_LENGTH 10 /* Length of the robot's arm that will catch the ball */

int main(){
    int is_init;
    /* Check for the right initialization of the robot */
    is_init = init_robot();
    if(is_init == -1){
        printf("Failed to init \n");
        return -1;
    }

    /* execute 2 scans and get relevant values i.e values that are close from each other */
	struct values * first_scan_values,second_scan_values;
	first_scan_values = single_scan(500,1),20;
	Sleep(3000);
	second_scan_values = single_scan(500,2,20);
	struct values ball_position;
	first_scan_values = (struct values *)(malloc(5 * sizeof(struct values));
	second_scan_values= (struct values *)(malloc(5 * sizeof(struct values));
	int x,y;
	int not_found = 1;
	for (x=0 ; x<6 && not_found; x++){
		for (y=0; y<6 ; y++){
			int is_relevant = are_close(first_scan_values[x],second_scan_values[y]);
			if (is_relevant == 1){
				ball_position.radius = (first_scan_values[x].radius + second_scan_values[y].radius)/2;
				ball_position.angle = (first_scan_values.angle + second_scan_values.angle)/2;
				not_found = 0;
				break;
			}

		}
	}
	printf("Ball is at radius %f , angle %f ", ball_position.radius,ball_position.angle);	

    exit_robot();
}


