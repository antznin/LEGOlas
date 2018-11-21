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

int main(void) {

	uint8_t sn_sonar;
	float value;
	
	ev3_sensor_init();

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
