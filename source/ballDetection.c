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
#define SPEED_FACTOR 1/20
#define PI 3.1415926

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

struct values single_scan(int sleep_value, 
		int scan_id, 
		float radius_value, 
		int sign, 
		float ignore_angle) {

	uint8_t sn1, sn2, sncompass, snsonar;
	int port1, port2;
	port1 = 66; port2 = 67; // left motor, right motor
	int max_speed;
	struct values myvalues; // values to be returned
	// myvalues = (struct values *)(malloc(5 * sizeof(struct values)));
	
	/* initialization of table values*/
	
	// int iterator;
	// for (iterator = 0; iterator < 6; iterator++){
	// 	myvalues[iterator].radius = scan_id * 6000 + iterator*1000;
	// 	myvalues[iterator].angle = scan_id * 6000 + iterator*1000;
	// }
	/* Initial values */

	// Initial large values
	myvalues.angle = scan_id * 6000;
	myvalues.radius = scan_id * 6000;

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

	/* Initialize sensor values */
	float sonar_value;
	get_sensor_value0(snsonar, &sonar_value);
	float compass_value;
	get_sensor_value0(sncompass, &compass_value);
	float current_angle = compass_value - initial_angle;
	int found = 0;
	/* Robot motion functions */
	if ( ev3_search_tacho_plugged_in(port1,0, &sn1, 0 )
			&& ev3_search_tacho_plugged_in(port2,0, &sn2, 0 )) {

		printf("Tachos %d and %d found.\n", port1 - 64, port2 - 64);

		get_tacho_max_speed( sn1, &max_speed );

		set_tacho_stop_action_inx( sn1, TACHO_COAST );
		set_tacho_stop_action_inx( sn2, TACHO_COAST );

		set_tacho_speed_sp( sn1, sign * max_speed * SPEED_FACTOR );
		set_tacho_speed_sp( sn2,-sign * max_speed * SPEED_FACTOR );

		set_tacho_command_inx( sn1, TACHO_RUN_FOREVER );
		set_tacho_command_inx( sn2, TACHO_RUN_FOREVER );

		printf("For angle %f, radius %f\n", 140.9, theoretical_radius(140.9));
		printf("For angle %f, radius %f\n", 158.0, theoretical_radius(158.0));

		printf("Ignore : %f\n", ignore_angle);
		while ( fabs(initial_angle - compass_value) < 360.0 ) {
				get_sensor_value0(snsonar, &sonar_value);
				get_sensor_value0(sncompass, &compass_value);
				current_angle = compass_value - initial_angle;
				// printf("Angle in single_scan : %f\n", sign * current_angle);
				// printf("Sonar value : %f\n", sonar_value);
				printf("%f %f\n",
						theoretical_radius(fabs(current_angle)),
						fabs(current_angle));
				printf("%f %f\n", sonar_value, current_angle);
				if (sonar_value < theoretical_radius(fabs(current_angle)) 
						&& found == 0 
						&& fabs(current_angle) > ( 360 - ignore_angle - 40)) 
				{ 
						// myvalues[count].radius = sonar_value;
						// myvalues[count].angle = current_angle;
						myvalues.angle = fmod(360.0 + current_angle, 360.0);
						myvalues.radius = sonar_value;
						 printf("======== FOUND =======\nSonar : %f, Angle : %f\n======================\n",
							sonar_value, fmod(360.0 + current_angle, 360.0));
						found = 1;
				}
		}

		set_tacho_command_inx( sn1, TACHO_STOP );
		set_tacho_command_inx( sn2, TACHO_STOP );

		return myvalues;
	} else {
		fprintf(stderr, "Tachos not found.");
	}

	return myvalues;
}

int are_close(struct values coordinates_1, struct values coordinates_2){
	if (fabs(coordinates_1.radius - coordinates_2.radius) < 20) {
		return 1;
	}
	else return 0;
}

//------------------------------------------------------------------------------
//----------------------------Mathematical scan functions-----------------------
//------------------------------------------------------------------------------

/**
 *   Defines a function that represents the parameterized curve of the terrain
 *  considering the intersection of the initial rotation axis and the surface
 *  of the terrain as its origin and its axes are parallel to the walls
 *  @param {float} angle
 *  @return {float} theoretical_radius
 */
  /*
                      Y-axis
   _____________________^_______________________
  |               1     |       8              |
  |________2____________|_____________7_______>|        X-axis
  |        3            |           6          |
  |                     |                      |
  |____________4________|_________5____________|

  Numbers represent the number of the quadrant



  */

 float theoretical_radius(float angle){
    float radius;
    float maxY = 200;
    float maxX = 435;
    float minY = 465;
    float minX = 435;
    float degToRad = PI/180;
    float reduced_angle = degToRad * angle ;
    float q1 = 65.0 ; //66.57
    float q2 = 90.0;
    float q3 = 137.0 ; //140.2
    float q4 = 180.0;
    float q5 = 222.4; //219.8
    float q6 = 270.0;
    float q7 = 294.43; //293.43
    float q8 = 360.0;
      if (angle >= 0 && angle <= q1 ){
        radius = abs(maxY/cos(reduced_angle));
        return radius;}
      else if (angle > q1 && angle <= q2 ){
        radius = abs(minX/sin(reduced_angle));
        return radius;}
      else if (angle > q2 && angle <= q3 ){
        radius = abs(minX/sin(reduced_angle));
        return radius;}
      else if (angle > q3 && angle <= q4 ){
        radius = abs(minY/cos(reduced_angle));
        return radius;}
      else if (angle > q4 && angle <= q5 ){
        radius = abs(minY/cos(reduced_angle));
        return radius;}
      else if (angle > q5 && angle <= q6 ){
        radius = abs(maxX/sin(reduced_angle));
        return radius;}
      else if (angle > q6 && angle <= q7 ){
        radius = abs(maxX/sin(reduced_angle));
        return radius;}
      else if (angle > q7 && angle <= q8 ){
        radius = abs(maxY/cos(reduced_angle));
        return radius;}

 }

//------------------------------------------------------------------------------
