/***************************************************************************************************/
/*************************         TACHOS MOVEMENTS FUNCTIONS         ******************************/
/*************************         Written by Yasmine Bennani         ******************************/
/***************************************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include "/src/ev3dev-c/source/ev3/ev3.h"
#include "/src/ev3dev-c/source/ev3/ev3_port.h"
#include "/src/ev3dev-c/source/ev3/ev3_tacho.h"
#include "/src/ev3dev-c/source/ev3/ev3_sensor.h"
#include <unistd.h>

#define Sleep( msec ) usleep(( msec ) * 1000 )

//Move one tacho for 5sec and cadencé
void move_forward(int dist){ //Makes the robot move forward for dist cm
    uint8_t sn;
    int port=65;
    FLAGS_T state;

    for (port=65; port<67; port++){
        if ( ev3_search_tacho_plugged_in(port,0, &sn, 0 )) {
            int max_speed;
            get_tacho_max_speed( sn, &max_speed );
            printf("  max speed = %d\n", max_speed );
            set_tacho_stop_action_inx( sn, TACHO_COAST );
            //set_tacho_speed_sp( sn, max_speed * 2 / 3 );
            //set_tacho_time_sp( sn, 5000 );
            //set_tacho_ramp_up_sp( sn, 2000 );
            //set_tacho_ramp_down_sp( sn, 2000 );
            set_tacho_command_inx( sn, TACHO_RUN_TIMED );
            /* Waiting for the tacho to stop */
            Sleep( 100 );

            do {
                get_tacho_state_flags( sn, &state );
            } while ( state );
            printf( "run to relative position...\n" );
            set_tacho_speed_sp( sn, max_speed / 2 );
            set_tacho_ramp_up_sp( sn, 0 );
            set_tacho_ramp_down_sp( sn, 0 );
            set_tacho_position_sp( sn, 90 );
            set_tacho_command_inx( sn, TACHO_RUN_TO_REL_POS );

        } else {
            printf( "LEGO_EV3_M_MOTOR %d is NOT found\n", (port-64));
        }
    }
}

int init_robot( void ) // Find the tachos
{
    int i;
    char s[256];

#ifndef __ARM_ARCH_4T__
    /* Disable auto-detection of the brick */
    ev3_brick_addr = "192.168.0.204";

#endif
    if ( ev3_init() == -1 ) return ( 1 );

#ifndef __ARM_ARCH_4T__
    printf( "The EV3 brick auto-detection is DISABLED,\nwaiting %s online with plugged tacho...\n", ev3_brick_addr );

#else
    printf( "Waiting tacho is plugged...\n" );

#endif
    while ( ev3_tacho_init() < 1 ) Sleep( 1000 );

    printf( "Found tacho motors:\n" );
    for ( i = 0; i < DESC_LIMIT; i++ ) {
        if ( ev3_tacho[ i ].type_inx != TACHO_TYPE__NONE_ ) {
            printf( "  type = %s\n", ev3_tacho_type( ev3_tacho[ i ].type_inx ));
            printf( "  port = %s\n", ev3_tacho_port_name( i, s ));
            printf("  port = %d %d\n", ev3_tacho_desc_port(i), ev3_tacho_desc_extport(i));
        }
    }

    return(0);
}

int exit_robot(void){ //Exit the ev3
    ev3_uninit();
    printf( "*** ( EV3 ) Bye! ***\n" );
    return ( 0 );
}




int main(){
  init_robot();
  move_forward(10);
  exit_robot();
  printf("Hello world");
  return 1;
}
