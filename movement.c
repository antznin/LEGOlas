
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
#include <math.h>

#define Sleep( msec ) usleep(( msec ) * 1000 )
#define PI 3.1415
#define WHEEL_DIAM 5.5 //Wheels' diameter is 5.5 cm

void turn(int angle) {
    uint8_t sn, sn_compass;
    int max_speed;
    float init_value, value;

    if (ev3_search_sensor(HT_NXT_COMPASS, &sn_compass,0)){
      get_sensor_value0(sn_compass, &init_value );
      //fflush( stdout );
    }

    if(angle > 0) {
        if( ev3_search_tacho_plugged_in(67, 0, &sn, 0)) {
            get_tacho_max_speed(sn, &max_speed);
            printf("max_speed = %d\n", max_speed);
            set_tacho_stop_action_inx( sn, TACHO_COAST );
            set_tacho_speed_sp( sn, max_speed * 2/3); //set tacho speed to (2/3)*max_speed
            get_sensor_value0(sn_compass, &value);
            while(init_value - value != angle){
                //set_tacho_command_inx(sn, TACHO_RUN_FOREVER);
                get_sensor_value0(sn_compass, &value);
                printf("Compass value : %f \n", value);
            }
            set_tacho_stop_action_inx( sn, TACHO_COAST);
        }
        else {
            printf( "LEGO_EV3_M_MOTOR %d is NOT found\n", 67);
        }

    }
    else {
        if( ev3_search_tacho_plugged_in(66, 0, &sn, 0)) {
            get_tacho_max_speed(sn, &max_speed);
            set_tacho_stop_action_inx( sn, TACHO_COAST );
            set_tacho_speed_sp( sn, max_speed * 2/3); //set tacho speed to (2/3)*max_speed
            set_tacho_time_sp( sn, 2000 ); //Tacho will run or t sec
            set_tacho_command_inx( sn, TACHO_RUN_TIMED );
        }
        else {
            printf( "LEGO_EV3_M_MOTOR %d is NOT found\n", 66);
        }
    }
}

void move_forward(int dist){ //Makes the robot move forward for dist cm
    uint8_t sn;
    int port;
    float t; //t in sec

    for (port=66; port<68; port++){
        if ( ev3_search_tacho_plugged_in(port,0, &sn, 0 )) {
            int max_speed, count_per_rot;
            get_tacho_count_per_rot(sn, &count_per_rot);
            get_tacho_max_speed( sn, &max_speed );
            printf("  count per rot = %d\n", count_per_rot);
            printf("  max speed = %d\n", max_speed );
            set_tacho_stop_action_inx( sn, TACHO_COAST );
            set_tacho_speed_sp( sn, max_speed * 2/3); //set tacho speed to (2/3)*max_speed
            t = (float) (count_per_rot * dist )/(max_speed * 2/3 * PI * WHEEL_DIAM) ;
            printf("Tacho will run for %f seconds \n", t);
            set_tacho_time_sp( sn, t*1000 ); //Tacho will run or t sec
            if(port == 66) {
                Sleep(100);
                set_tacho_command_inx( sn, TACHO_RUN_TIMED );
            }
            else {
                float t_left; // = t + 200 ms because otherwise turns too much
                t_left = (t*1000) + 150;
                set_tacho_time_sp(sn, t_left);
                set_tacho_command_inx( sn, TACHO_RUN_TIMED );
           }

        } else {
            printf( "LEGO_EV3_M_MOTOR %d is NOT found\n", (port-64));
            t = 0;
        }
    }
    /* Waiting for the tacho to stop */
    Sleep( t*1000 );
}

int init_robot( void ) // Find the tachos
{
    uint8_t sn_compass;
    int i;
    char s[256];
    float value;

    if(ev3_tacho_init() == -1)
       return 1;

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

    //Run all sensors
    ev3_sensor_init();
    if (ev3_search_sensor(HT_NXT_COMPASS, &sn_compass,0)){
      printf("COMPASS found, reading compass...\n");
      if ( !get_sensor_value0(sn_compass, &value )) {
        value = 0;
      }
      printf( "\r(%f) \n", value);
      fflush( stdout );
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
//    move_forward(20);
    turn(90);
    exit_robot();
    return 1;
}
