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

	scan();

	while(true) {
		if (ev3_search_sensor(LEGO_EV3_US, &sn_sonar,0)){
     			printf("SONAR found, reading sonar...\n");
     			if ( !get_sensor_value0(sn_sonar, &value )) {
     				value = 0;
     			}
     			printf( "\r(%f) \n", value);
     			fflush( stdout );
			Sleep(200);
		}
	}
}

float scan(void) {
	
	printf("Making the robot turn :\n");	
	uint8_t sn1, sn2;
	int port1, port2;
	port1 = 66; port2 = 67;
	
	while ( ev3_tacho_init() < 1 ) Sleep( 1000 );

	if ( ev3_search_tacho_plugged_in(port1,0, &sn1, 0 ) && ev3_search_tacho_plugged_in(port2,0, &sn2, 0 )) {
		
		printf("Tachos found\n");
		int max_speed;
		get_tacho_max_speed( sn1, &max_speed );

		set_tacho_stop_action_inx( sn1, TACHO_COAST );
		set_tacho_speed_sp( sn1, max_speed * 2 / 3 );
		set_tacho_time_sp( sn1, 300 );
		set_tacho_ramp_up_sp( sn1,0 );
		set_tacho_ramp_down_sp( sn1,0 );
		set_tacho_command_inx( sn1, TACHO_RUN_TIMED );
		/* Waiting for the tacho to stop */
		set_tacho_stop_action_inx( sn2, TACHO_COAST );
		set_tacho_speed_sp( sn2,-max_speed * 2 / 3 );
		set_tacho_time_sp( sn2, 300 );
		set_tacho_ramp_up_sp( sn2,0 );
		set_tacho_ramp_down_sp( sn2,0 );
		set_tacho_command_inx( sn2, TACHO_RUN_TIMED );
		/* Waiting for the tacho to stop */
		Sleep( 100 );

	}
}
