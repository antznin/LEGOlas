/***************************************************************************************************/
/*************************         TACHOS MOVEMENTS FUNCTIONS         ******************************/
/*************************         Written by Antonin Godard         ******************************/
/***************************************************************************************************/


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"
#define Sleep( msec ) usleep(( msec ) * 1000 )

float scan(void);

int main(void) {

	uint8_t sn_sonar;
	float value;
	
	ev3_sensor_init();
	printf("Init");

	value = scan();
	
	printf("Angle : %f\n", value);

	// while(true) {
	// 	if (ev3_search_sensor(LEGO_EV3_US, &sn_sonar,0)){
     	// 		printf("SONAR found, reading sonar...\n");
     	// 		if ( !get_sensor_value0(sn_sonar, &value )) {
     	// 			value = 0;
	// 		}
     	// 		printf( "\r(%f) \n", value);
     	// 		fflush( stdout );
	// 		Sleep(200);
	// 	}
	// }
}

float scan(void) {
	/*
	 * This functions makes the robot turn on it self until it detects the ball.
	 * It returns the angle and makes the robot turn back to its initial position.
	 */
		
	printf("[.] Making the robot turn :\n");	
	uint8_t sn1, sn2, sncompass, snsonar;
	int port1, port2;
	port1 = 66; port2 = 67; // left motor, right motor
	int max_speed;
	float initial_angle, initial_dist, min_distance, speed_factor;
	speed_factor = 5/6;
	min_distance = 00;
	
	while ( ev3_tacho_init() < 1 ) Sleep( 1000 ); // robot tachos init
	ev3_sensor_init(); // robot sensors init

	if (ev3_search_sensor(LEGO_EV3_GYRO, &sncompass, 0)){
		printf("[v] Compass sensor found.\n");
		get_sensor_value0(sncompass, &initial_angle ); // compass sensor
	} else {
		printf("[x] Compass sensor not found.\n");
	}

	if (ev3_search_sensor(LEGO_EV3_US, &snsonar, 0)){
		printf("[v] Sonar sensor found.\n");
		get_sensor_value0(snsonar, &initial_dist ); // compass sensor
      		//fflush( stdout );
	} else {
		printf("[x] Sonar sensor not found\n");
	}

	// int bool_tacho1, bool_tacho2;
	if ( ev3_search_tacho_plugged_in(port1,0, &sn1, 0 ) 
			&& ev3_search_tacho_plugged_in(port2,0, &sn2, 0 )) {
		
		printf("[v] Tachos %d and %d found.\n", port1 - 64, port2 - 64);

		get_tacho_max_speed( sn1, &max_speed );

		set_tacho_speed_sp( sn1, max_speed * speed_factor ); // set the tachos speed
		set_tacho_speed_sp( sn2,-max_speed * speed_factor );

		set_tacho_stop_action_inx( sn1, TACHO_COAST );
		set_tacho_stop_action_inx( sn2, TACHO_COAST );

		set_tacho_ramp_up_sp( sn1,0 );
		set_tacho_ramp_down_sp( sn1,0 );
		set_tacho_ramp_up_sp( sn2,0 );
		set_tacho_ramp_down_sp( sn2,0 );

		float sonar_value;
		get_sensor_value0(snsonar, &sonar_value);
		printf("[.] SONAR INITIAL VALUE : %f\n", initial_dist);
		while (sonar_value > min_distance) {
			get_sensor_value0(snsonar, &sonar_value);
			set_tacho_command_inx( sn1, TACHO_RUN_FOREVER );
			set_tacho_command_inx( sn2, TACHO_RUN_FOREVER );
		}

		set_tacho_stop_action_inx( sn1, TACHO_COAST );
		set_tacho_stop_action_inx( sn2, TACHO_COAST );

		return abs(initial_angle - sonar_value);

	} else {
		// if (!bool_tacho1) {
		// 	printf("[x] Tacho %d not found.\n", port1 - 64);
		// } else if (!bool_tacho1) {
		// 	printf("[x] Tacho %d not found.\n", port2 - 64);
		// } else {
		// 	printf("[x] Tacho %d and %d not found.\n",port1 - 64,  port2 - 64);
		// }
	}
	return 0.0;
}
