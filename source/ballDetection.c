/***************************************************************************************************/
/*************************         TACHOS MOVEMENTS FUNCTIONS        *******************************/
/*************************         Written by Antonin Godard         *******************************/
/***************************************************************************************************/

#include "ballDetection.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"

#define Sleep( msec ) usleep(( msec ) * 1000 )

struct values scan(float angle, float radius) {
	/*
	 * This functions makes the robot turn on it self until it detects the ball.
	 * It returns the angle and makes the robot turn back to its initial position.
	 */
		
	printf("[.] Making the robot turn :\n");	
	uint8_t sn1, sn2, sncompass, snsonar;
	int port1, port2;
	port1 = 66; port2 = 67; // left motor, right motor
	int max_speed;
	float initial_angle, initial_dist;
	struct values myvalues;
	int sign;

	if (angle > 0) {
		sign = 1;
	} else {
		sign = -1;
	}

	// int bool_tacho1, bool_tacho2;
	if ( ev3_search_tacho_plugged_in(port1,0, &sn1, 0 ) 
			&& ev3_search_tacho_plugged_in(port2,0, &sn2, 0 )) {
		
		printf("[v] Tachos %d and %d found.\n", port1 - 64, port2 - 64);

		get_tacho_max_speed( sn1, &max_speed );
		printf("[.] TACHO MAX SPEED : %d\n", max_speed);

		set_tacho_stop_action_inx( sn1, TACHO_COAST );
		set_tacho_stop_action_inx( sn2, TACHO_COAST );

		set_tacho_speed_sp( sn1, max_speed * 1/40 * sign ); // set the tachos speed
		set_tacho_speed_sp( sn2,-max_speed * 1/40 * sign );

		//set_tacho_ramp_up_sp( sn1,0 );
		//set_tacho_ramp_down_sp( sn1,0 );
		//set_tacho_ramp_up_sp( sn2,0 );
		//set_tacho_ramp_down_sp( sn2,0 );

		float sonar_value;
		get_sensor_value0(snsonar, &sonar_value);
		float compass_value;
		get_sensor_value0(sncompass, &compass_value);
		printf("[.] COMPASS INITIAL VALUE : %f\n", initial_angle);
		printf("[.] SONAR INITIAL VALUE : %f\n", initial_dist);

		set_tacho_command_inx(sn1, TACHO_RUN_FOREVER);
            	set_tacho_command_inx(sn2, TACHO_RUN_FOREVER);

		//while (abs(sonar_value - prev_sonar_value) < 50) {
		while (((int)compass_value - ((int)angle  + (int)initial_angle)) % 360 != 0
				&& sonar_value > radius) {
			get_sensor_value0(snsonar, &sonar_value);
			printf("Distance : /%f/\n", sonar_value);
			printf("Value - (angle + init ) = %f \n ", compass_value - (angle + initial_angle));
			//set_tacho_command_inx( sn1, TACHO_RUN_TIMED );
			//set_tacho_command_inx( sn2, TACHO_RUN_TIMED );
		}

		set_tacho_command_inx( sn1, TACHO_STOP );
		set_tacho_command_inx( sn2, TACHO_STOP );

		myvalues.angle = compass_value - (angle + initial_angle);
		myvalues.radius = round(sonar_value * 0.1);

		return myvalues;
	} else {
		printf("[x] Tachos not found.");
		// if (!bool_tacho1) {
		// 	printf("[x] Tacho %d not found.\n", port1 - 64);
		// } else if (!bool_tacho1) {
		// 	printf("[x] Tacho %d not found.\n", port2 - 64);
		// } else {
		// 	printf("[x] Tacho %d and %d not found.\n",port1 - 64,  port2 - 64);
		// }
	}
	myvalues.angle = 0.0;
	myvalues.radius = 0.0;
	return myvalues;
}
