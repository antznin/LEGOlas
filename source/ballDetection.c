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

/* This variable is used to communicate between the main thread and
 * the sensor thread (used for permanently retrieving the sensor values)
 */
int found;

/* This variables are the values to pass to the sensor thread
 */
struct values *valuestopass;

struct values *tab_scan1;

/* Sensor values routine
 * This thread routine takes care of permanently retrieving the values from the
 * sensors asynchronously from the robot movements. When ending, pthread_cancel
 * sends the values back to the main thread at pthread_join (See scan function).
 *
 * @param {void *} initvalues : pointer on struct values (to be casted) passed
 * 	by the main thread
 */
 
static void * get_sensor_values(void * initvalues) {

	uint8_t snsonar, sncompass;

	/* Getting the parameters from the scan function */
	struct values *myvalues = (struct values *)initvalues;
	float angle = (*myvalues).angle;
	float radius = (*myvalues).radius;
	/* Initial values */
	float initial_angle, initial_dist;

	/* Sign used for the while condition below. Takes account of the
	 * sign of the angle. */
	float sign;
	if (angle > 0.0) {
		sign = 1.0;
	} else {
		sign = -1.0;
	}

	/* Looking for the sensor and storing initial values */
	if (ev3_search_sensor(LEGO_EV3_GYRO, &sncompass, 0)){
		get_sensor_value0(sncompass, &initial_angle );
	} else {
		fprintf(stderr, "Compass sensor in scan not found.\n");
	}

	if (ev3_search_sensor(LEGO_EV3_US, &snsonar, 0)){
		get_sensor_value0(snsonar, &initial_dist );
	} else {
		fprintf(stderr, "Sonar sensor in scan not found\n");
	}

	/* Initialize sensor values */
	float sonar_value;
	get_sensor_value0(snsonar, &sonar_value);
	float compass_value;
	get_sensor_value0(sncompass, &compass_value);

	/* Val represent the current angle which tends to zero in every case */
	int val;
	val = ((int)sign * ((int)compass_value - (int)angle - (int)initial_angle)) % 360;

	printf("COMPASS INITIAL VALUE : %f\n", initial_angle);
	printf("SONAR INITIAL VALUE : %f\n", initial_dist);

	while (abs(val) > 0 && sonar_value > radius * 10  ) {
		get_sensor_value0(snsonar, &sonar_value);
		get_sensor_value0(sncompass, &compass_value);
		val = ((int)sign * ( (int)compass_value - (int)angle  - (int)initial_angle ) % 360);
		printf("Angle in scan : %d\n", val);
		printf("Sonar value : %f\n", sonar_value);
	}
	found = 1;

	if (abs(val) <  0) { // The robot has reaches maximum angle
		(*myvalues).angle = 0.0;
		(*myvalues).radius = 0.0;

	} else { // The robot has found the ball
		(*myvalues).angle = (float)val;
		(*myvalues).radius = round(sonar_value * 0.1);
	}
	/* Exiting and passing values */
	pthread_exit ((void *)myvalues);
}

struct values partial_scan(float angle, float radius) {
	/*
	 * This functions makes the robot turn on it self until it detects the ball.
	 * It returns the angle and makes the robot turn back to its initial position.
	 *
	 * The main thread makes the robot turn, and a thread takes the
	 * sensor values continuously.
	 *
	 * @param {float} angle : turn for angle
	 * @param {float} radius : maximum radius distance (threshold)
	 * @return {struct values} myvalues : structure composed of an angle and a radius
	 */

	printf("[.] Making the robot turn :\n");
	uint8_t sn1, sn2;
	int port1, port2;
	port1 = 66; port2 = 67; // left motor, right motor
	int max_speed;
	int sign;
	struct values myvalues; // values to be returned
	pthread_t sensorvalues_thread;
	found = 0;

	if (angle > 0.0) {
		sign = 1.0;
	} else {
		sign = -1.0;
	}

	valuestopass = malloc(sizeof(struct values));
	(*valuestopass).angle = angle;
	(*valuestopass).radius = radius;

	/* Creating the thread */
	if (pthread_create(&sensorvalues_thread, NULL,
			get_sensor_values, (void *)valuestopass) != 0) {
		fprintf(stderr, "Sensor scan thread failed to initialize");
		exit(EXIT_FAILURE);
	}

	/* Robot motions functions */
	if ( ev3_search_tacho_plugged_in(port1,0, &sn1, 0 )
			&& ev3_search_tacho_plugged_in(port2,0, &sn2, 0 )) {

		printf("Tachos %d and %d found.\n", port1 - 64, port2 - 64);

		get_tacho_max_speed( sn1, &max_speed );

		set_tacho_stop_action_inx( sn1, TACHO_COAST );
		set_tacho_stop_action_inx( sn2, TACHO_COAST );

		set_tacho_speed_sp( sn1, max_speed * SPEED_FACTOR * sign );
		set_tacho_speed_sp( sn2,-max_speed * SPEED_FACTOR * sign );

		/* Robot will stop when ball is found OR robot has reached angle */
		while (found == 0) {
			set_tacho_time_sp(sn1, 200);
			set_tacho_time_sp(sn2, 200);
			set_tacho_command_inx(sn1, TACHO_RUN_TIMED);
			set_tacho_command_inx(sn2, TACHO_RUN_TIMED);
			Sleep(400);
		}

		set_tacho_command_inx( sn1, TACHO_STOP );
		set_tacho_command_inx( sn2, TACHO_STOP );

		/* Retrieving values returned by pthread_cancel */
		void * tmp_finalvalues;
		pthread_join (sensorvalues_thread, &tmp_finalvalues);
		struct values *finalvalues = (struct values *)tmp_finalvalues;

		/* Returning values */
		myvalues.angle = (*finalvalues).angle;
		myvalues.radius = (*finalvalues).radius;

		printf("Scan return angle : %f\n", myvalues.angle);
		printf("Scan returned radius : %f\n", myvalues.radius);

		return myvalues;
	} else {
		fprintf(stderr, "Tachos not found.");
	}
	// Tachos couldn't be found, returning 0.0s
	myvalues.angle = 0.0;
	myvalues.radius = 0.0;
	return myvalues;
}

struct values * single_scan(int sleep_value, int scan_id, float radius_value) {
	/*
	 * This functions makes the robot turn on it self until it detects the ball.
	 * It returns the angle and makes the robot turn back to its initial position.
	 *
	 * The main thread makes the robot turn, and a thread takes the
	 * sensor values continuously.
	 *
	 * @param {float} angle : turn for angle
	 * @param {float} radius : maximum radius distance (threshold)
	 * @return {struct values} myvalues : structure composed of an angle and a radius
	 */

	uint8_t sn1, sn2;
	int port1, port2;
	port1 = 66; port2 = 67; // left motor, right motor
	int max_speed;
	struct values * myvalues; // values to be returned
	myvalues = (struct values *)(malloc(5 * sizeof(struct values)));
	
	/* initialization of table values*/
	
	int iterator;
	for (iterator = 0; iterator < 6; iterator++){
		myvalues[iterator].radius = scan_id * 6000 + iterator*1000;
		myvalues[iterator].angle = scan_id * 6000 + iterator*1000;
	}
	/* Initial values */
	float initial_angle, initial_dist;

	/* Looking for the sensor and storing initial values */
	if (ev3_search_sensor(LEGO_EV3_GYRO, &sncompass, 0)){
		get_sensor_value0(sncompass, &initial_angle );
	} else {
		fprintf(stderr, "Compass sensor in scan not found.\n");
	}

	if (ev3_search_sensor(LEGO_EV3_US, &snsonar, 0)){
		get_sensor_value0(snsonar, &initial_dist );
	} else {
		fprintf(stderr, "Sonar sensor in scan not found\n");
	}

	/* Sign used for the while condition below. Takes account of the
	 * sign of the angle. */
	float sign;
	if (angle > 0.0) {
		sign = 1.0;
	} else {
		sign = -1.0;
	}

	/* Initialize sensor values */
	float sonar_value;
	get_sensor_value0(snsonar, &sonar_value);
	float compass_value;
	get_sensor_value0(sncompass, &compass_value);
	float current_angle = compass_value - initial_angle;
	/* Robot motion functions */
	if ( ev3_search_tacho_plugged_in(port1,0, &sn1, 0 )
			&& ev3_search_tacho_plugged_in(port2,0, &sn2, 0 )) {

		printf("Tachos %d and %d found.\n", port1 - 64, port2 - 64);

		get_tacho_max_speed( sn1, &max_speed );

		set_tacho_stop_action_inx( sn1, TACHO_COAST );
		set_tacho_stop_action_inx( sn2, TACHO_COAST );

		set_tacho_speed_sp( sn1, max_speed * SPEED_FACTOR * sign );
		set_tacho_speed_sp( sn2,-max_speed * SPEED_FACTOR * sign );

		set_tacho_command_inx( sn1, TACHO_RUN_FOREVER );
		set_tacho_command_inx( sn2, TACHO_RUN_FOREVER );
		int count = 0;
		while ( fabs(initial_angle - compass_value) < 360.0 ) {
				get_sensor_value0(snsonar, &sonar_value);
				get_sensor_value0(sncompass, &compass_value);
				printf("Angle in single_scan : %d\n", current_angle);
				printf("Sonar value : %f\n", sonar_value);
				if (sonar_value < radius_value * 9.9  && count < 6)
				{
						myvalues[count].radius = sonar_value;
						myvalues[count].angle = current_angle;
						count++;
						Sleep(sleep_value);
				}
		}

		set_tacho_command_inx( sn1, TACHO_STOP );
		set_tacho_command_inx( sn2, TACHO_STOP );

		/* Retrieving values returned by pthread_cancel */
		/*void * tmp_finalvalues;
		pthread_join (sensorvalues_thread, &tmp_finalvalues);
		struct values *finalvalues = (struct values *)tmp_finalvalues;*/

		/* Returning values */
		/*myvalues.angle = (*finalvalues).angle;
		myvalues.radius = (*finalvalues).radius;*/
		/*
		printf("Scan return angle : %f\n", myvalues.angle);
		printf("Scan returned radius : %f\n", myvalues.radius);*/

		return myvalues;
	} else {
		fprintf(stderr, "Tachos not found.");
	}

	return myvalues;
}

int are_close(struct values coordinates_1, struct values coordinates_2){
	if (fabs(coordinates_1.radius - coordinates_2.radius) < 20 && fabs(coordinates_1.angle - coordinates_2.angle) < 15) {
		return 1;
	}
	else return 0;
}


