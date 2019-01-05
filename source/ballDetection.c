/***************************************************************************************************/
/*************************         TACHOS MOVEMENTS FUNCTIONS        *******************************/
/*************************         Written by Antonin Godard         *******************************/
/***************************************************************************************************/

#include "ballDetection.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <pthread.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"


#define Sleep( msec ) usleep(( msec ) * 1000 )
#define SPEED_FACTOR 1/15

int found;
struct values *valuestopass;

static void * get_sensor_values(void * initvalues) {

	uint8_t snsonar, sncompass;

	struct values *myvalues = (struct values *)initvalues;
	printf("VALUES AT BEGINNING OF THREAD : %f, %f\n", (*myvalues).angle, (*myvalues).radius);
	float angle = (*myvalues).angle;
	float radius = (*myvalues).radius;

	//float * values  = (float*)initvalues;
	//float angle = *values;
	//float radius = *(values + sizeof(float));
	//printf("VALUES AT BEGINNING OF THREAD : %f, %f\n", values[0],values[1]);

	float initial_angle, initial_dist;
	float sign;

	if (angle > 0.0) {
		sign = 1.0;
	} else {
		sign = -1.0;
	}

	if (ev3_search_sensor(LEGO_EV3_GYRO, &sncompass, 0)){
		get_sensor_value0(sncompass, &initial_angle ); // compass sensor
	} else {
		printf("[x] Compass sensor in scan not found.\n");
	}

	if (ev3_search_sensor(LEGO_EV3_US, &snsonar, 0)){
		get_sensor_value0(snsonar, &initial_dist ); // sonar sensor
	} else {
		printf("[x] Sonar sensor in scan not found\n");
	}

	float sonar_value;
	get_sensor_value0(snsonar, &sonar_value);
	float compass_value;
	get_sensor_value0(sncompass, &compass_value);

	int val;
	val = ((int)sign * ( (int)compass_value - (int)angle  - (int)initial_angle ) % 360);

	printf("[.] COMPASS INITIAL VALUE : %f\n", initial_angle); 
	printf("[.] SONAR INITIAL VALUE : %f\n", initial_dist);

	while (abs(val) > 0 && sonar_value > radius * 10) {
		get_sensor_value0(snsonar, &sonar_value);
		get_sensor_value0(sncompass, &compass_value);
		val = ((int)sign * ( (int)compass_value - (int)angle  - (int)initial_angle ) % 360);
		printf("Angle in scan : %d\n", val);
		printf("Sonar value : %f\n", sonar_value);
	}

	printf("Out of the loop.");
	found = 1;
	printf("Found IN THREAD : %d\n", found);

	if (abs(val) <  0) {
		(*myvalues).angle = 0.0;
		(*myvalues).radius = 0.0;	
		
	} else {
		(*myvalues).angle = (float)((int)sign * ( (int)compass_value  - (int)initial_angle ) % 360);
		(*myvalues).radius = round(sonar_value * 0.1);
	}
	pthread_exit ((void *)myvalues);
}

struct values scan(float angle, float radius) {
	/*
	 * This functions makes the robot turn on it self until it detects the ball.
	 * It returns the angle and makes the robot turn back to its initial position.
	 *
	 * The main thread makes the robot turn, and a thread takes the
	 * sensor values continuously.
	 */
		
	printf("[.] Making the robot turn :\n");	
	uint8_t sn1, sn2;
	int port1, port2;
	port1 = 66; port2 = 67; // left motor, right motor
	int max_speed;
	int ret;
	int sign;
	struct values myvalues;
	pthread_t sensorvalues_thread;
	found = 0;

	if (angle > 0.0) {
		sign = 1.0;
	} else {
		sign = -1.0;
	}

	// array to pass values to sensor thread
	// float * values = (float *)malloc(2 * sizeof(float));
	// *values = angle;
	// *(values + sizeof(float)) = radius;
	// values = (void *)values;
	// printf("VALUES AT BEGINNING OF SCAN : %f, %f\n", *values, *(values + sizeof(float)));
	valuestopass = malloc(sizeof(struct values));
	(*valuestopass).angle = angle;
	(*valuestopass).radius = radius;

	ret = pthread_create(&sensorvalues_thread,
			NULL,
			get_sensor_values,
			(void *)valuestopass
			);

	if (!ret) {	
		if ( ev3_search_tacho_plugged_in(port1,0, &sn1, 0 ) 
				&& ev3_search_tacho_plugged_in(port2,0, &sn2, 0 )) {
			
			printf("[v] Tachos %d and %d found.\n", port1 - 64, port2 - 64);

			get_tacho_max_speed( sn1, &max_speed );
			printf("[.] TACHO MAX SPEED : %d\n", max_speed);

			set_tacho_stop_action_inx( sn1, TACHO_COAST );
			set_tacho_stop_action_inx( sn2, TACHO_COAST );

			set_tacho_speed_sp( sn1, max_speed * SPEED_FACTOR * sign ); // set the tachos speed
			set_tacho_speed_sp( sn2,-max_speed * SPEED_FACTOR * sign );

			while (found == 0) {
				// set_tacho_time_sp(sn1, 200);
				// set_tacho_time_sp(sn2, 200);
				// set_tacho_command_inx(sn1, TACHO_RUN_TIMED);
				// set_tacho_command_inx(sn2, TACHO_RUN_TIMED);
				// Sleep(400);
				set_tacho_command_inx(sn1, TACHO_RUN_FOREVER);
				set_tacho_command_inx(sn2, TACHO_RUN_FOREVER);
				printf("Found : %d\n", found);
			}

			set_tacho_command_inx( sn1, TACHO_STOP );
			set_tacho_command_inx( sn2, TACHO_STOP );
			set_tacho_time_sp(sn1, 800);
			set_tacho_time_sp(sn2, 800);
			set_tacho_command_inx(sn1, TACHO_RUN_TIMED);
			set_tacho_command_inx(sn2, TACHO_RUN_TIMED);
			set_tacho_command_inx( sn1, TACHO_STOP );
			set_tacho_command_inx( sn2, TACHO_STOP );

			void * tmp_finalvalues;
			pthread_join (sensorvalues_thread, &tmp_finalvalues);
			struct values *finalvalues = (struct values *)tmp_finalvalues;

			myvalues.angle = (*finalvalues).angle;
			myvalues.radius = (*finalvalues).radius;

			printf("Angle : %f\n", myvalues.angle);
			printf("Radius : %f\n", myvalues.radius);

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
}
