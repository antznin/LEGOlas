//
//  test.c
//  
//
//  Created by Yasmine Bennani on 08/01/2019.
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include "/src/ev3dev-c/source/ev3/ev3.h"
#include "/src/ev3dev-c/source/ev3/ev3_port.h"
#include "/src/ev3dev-c/source/ev3/ev3_tacho.h"
#include "/src/ev3dev-c/source/ev3/ev3_sensor.h"

#define Sleep( msec ) usleep(( msec ) * 1000 )

struct position
{
    char * state;   // TACHO_STOP, TACHO_RUN_FOREVER, etc
    float x;        // in centimeter
    float y;        // in centimeter
    float theta;    // in radian (counterclockwise from x-axis)
};

struct position current_position;

void init_pos()
{
    current_position.state = "TACHO_STOP";
    current_position.x = 0.0;
    current_position.y = 0.0;
    current_position.theta = 0;
}

int init_robot( void ) // Find the tachos
{
    int i;
    char s[256];
    
    if(ev3_tacho_init() == -1)
        return 1;
    
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
    uint8_t sn_right, sn_left;
    int max_speed;
    int port_left = 66; /* Left wheel */
    int port_right = 67; /* Right wheel */
    
    init_robot();
    
    if( ev3_search_tacho_plugged_in(port_left, 0, &sn_left, 0) &&  ev3_search_tacho_plugged_in(port_right,0, &sn_right, 0 )) {
        printf("HEY\n");
        get_tacho_max_speed(sn_left, &max_speed);
        /* Set tacho speed to (1/30) * max_speed */
        set_tacho_speed_sp( sn_left, max_speed * 1/30);
        set_tacho_speed_sp( sn_right, (- max_speed * 1/30));
        
        set_tacho_command_inx( sn_left, TACHO_STOP);
        set_tacho_command_inx( sn_right, TACHO_STOP);
        
        current_position.state = "TACHO_STOP";
        
        printf("Sate:%s\n", current_position.state);
        
        set_tacho_command_inx( sn_left, TACHO_RUN_FOREVER );
        set_tacho_command_inx( sn_right, TACHO_RUN_FOREVER );
        current_position.state = "TACHO_RUN_FOREVER";
        
        Sleep(1000);

        printf("Sate:%s\n", current_position.state);
        
        set_tacho_command_inx( sn_left, TACHO_STOP);
        set_tacho_command_inx( sn_right, TACHO_STOP);
    }
}



