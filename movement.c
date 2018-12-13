
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

float init_value_compass;
void move_forward(int dist);
void turn(float angle);
void go_to_corner(int to_right); // Go to right corner if to_right = 1, otherwise go to left corner
void shoot_from_stage1();
void shoot_from_stage2();
void shoot_from_stage3();
void shoot_from_stage4();
void shoot_from_stage5();

/******************************* SHOOT FROM STAGE FUNCTIONS **************************************/
/*************** The robot will scan the field from 5 different stages, **************************/
/************ and if he has found the ball he has to go to the shoot position ********************/
/*************************************************************************************************/

/* Go to shoot position from stage 1 */
void shoot_from_stage1(){
    turn(-30);
    move_forward(-20);
    turn(30);
}

/* Go to shoot position from stage 2 */
void shoot_from_stage2(){
    turn(-60);
    move_forward(60);
    turn(-120);
}

/* Go to shoot position from stage 3 */
void shoot_from_stage3(){
    turn(-30);
    move_forward(60);
    turn(-60);
}

/* Go to shoot position from stage 4 */
void shoot_from_stage4(){
    turn(-45);
    move_forward(50);
    turn(45);
}

/* Go to shoot position from stage 5 */
void shoot_from_stage5(){
    turn(-60);
    move_forward(50);
    turn(-30);
}


/* Go to left corner from its beginning position */
void go_to_corner(int to_right){
    move_forward(53); /* Move to the wall */

    if(to_right) {
        turn(90);
        move_forward(40);
        turn(90);
    }
    else {
        turn(-90);
        move_forward(40);
        turn(-90);
    }
}


void turn(float angle) {
    uint8_t sn_left, sn_right, sn_compass;
    int max_speed_left, max_speed_right, max_speed;
    int port_left = 66; /* Left wheel */
    int port_right = 67; /* Right wheel */
    float value, init;

    if (ev3_search_sensor(LEGO_EV3_GYRO, &sn_compass,0)){
      get_sensor_value0(sn_compass, &init );
    }


    if(angle > 0) {
        if( ev3_search_tacho_plugged_in(port_left, 0, &sn_left, 0) &&  ev3_search_tacho_plugged_in(port_right,0, &sn_right, 0 )) {
            get_tacho_max_speed(sn_left, &max_speed_left);
            get_tacho_max_speed(sn_left, &max_speed_right);

            set_tacho_stop_action_inx( sn_left, TACHO_COAST );
            set_tacho_stop_action_inx( sn_right, TACHO_COAST );

            if(max_speed_left == max_speed_right || max_speed_left < max_speed_right)  /* Set max_speed to the lowest max_speed (left or right) */
                max_speed = max_speed_left;
            else
                max_speed = max_speed_right;

            /* Set tacho speed to (1/30) * max_speed */
            set_tacho_speed_sp( sn_left, max_speed * 1/30);
            set_tacho_speed_sp( sn_right, (- max_speed * 1/30));

            /* Get sensor compass value */
            get_sensor_value0(sn_compass, &value);

            /* Run the tachos */
            set_tacho_command_inx(sn_left, TACHO_RUN_FOREVER);
            set_tacho_command_inx(sn_right, TACHO_RUN_FOREVER);

            while(((int)value - ((int)angle  + (int)init)) % 360 != 0.0){
                    get_sensor_value0(sn_compass, &value);
                    printf("Value - (angle + init ) = %f \n ", value - (angle + init));
                }

            set_tacho_command_inx( sn_left, TACHO_STOP);
            set_tacho_command_inx( sn_right, TACHO_STOP);
        }
        else {
           printf( "LEGO_EV3_M_MOTOR NOT found\n");
        }
    }

    else {
        if( ev3_search_tacho_plugged_in(port_left, 0, &sn_left, 0) &&  ev3_search_tacho_plugged_in(port_right,0, &sn_right, 0 )) {
            get_tacho_max_speed(sn_left, &max_speed_left);
            get_tacho_max_speed(sn_left, &max_speed_right);

            set_tacho_stop_action_inx( sn_left, TACHO_COAST );
            set_tacho_stop_action_inx( sn_right, TACHO_COAST );

            if(max_speed_left == max_speed_right || max_speed_left < max_speed_right)  /* Set max_speed to the lowest max_speed (left or right) */
                max_speed = max_speed_left;
            else
                max_speed = max_speed_right;

            /* Set tacho speed to (1/30) * max_speed */
            set_tacho_speed_sp( sn_left, (- max_speed * 1/30));
            set_tacho_speed_sp( sn_right, ( max_speed * 1/30));

            /* Get sensor compass value */
            get_sensor_value0(sn_compass, &value);

            /* Run the tachos */
            set_tacho_command_inx(sn_left, TACHO_RUN_FOREVER);
            set_tacho_command_inx(sn_right, TACHO_RUN_FOREVER);

            while(((int)init - ((int)value  - (int)angle)) % 360 != 0.0){
                    get_sensor_value0(sn_compass, &value);
                    printf("Value - (angle + init ) = %f \n ", value - (angle + init));
                }

            set_tacho_command_inx( sn_left, TACHO_STOP);
            set_tacho_command_inx( sn_right, TACHO_STOP);
        }

        else {
            printf( "LEGO_EV3_M_MOTOR NOT found\n");
        }
    }
}

void move_forward(int dist){ //Makes the robot move forward for dist cm (if dist is neg, move back)
    uint8_t sn_left, sn_right;
    int port_left = 66; /* Left wheel */
    int port_right = 67; /* Right wheel */
    int max_speed, max_speed_left, max_speed_right, count_per_rot;
    float t=0; //t in sec

    if ( ev3_search_tacho_plugged_in(port_left,0, &sn_left, 0 ) &&  ev3_search_tacho_plugged_in(port_right,0, &sn_right, 0 )) {
        /* Get max_speed and get count_per_rot */
        get_tacho_count_per_rot(sn_left, &count_per_rot);
        get_tacho_max_speed( sn_left, &max_speed_left);
        get_tacho_max_speed( sn_right, &max_speed_right);

        set_tacho_stop_action_inx( sn_left, TACHO_COAST );
        set_tacho_stop_action_inx( sn_right, TACHO_COAST );

        if(max_speed_left == max_speed_right || max_speed_left < max_speed_right) /* Set max_speed to the lowest max_speed (left or right) */
            max_speed = max_speed_left;
        else
            max_speed = max_speed_right;

        /* Set tacho speed to (1/3) * max_speed */
        if(dist > 0) {
            set_tacho_speed_sp( sn_left, max_speed * 1/3);
            set_tacho_speed_sp( sn_right, max_speed * 1/3);
        }
        else {
            set_tacho_speed_sp( sn_left, - max_speed * 1/3);
            set_tacho_speed_sp( sn_right, - max_speed * 1/3);
        }

        /* Calculate time the tacho will run */
        t = (float) (count_per_rot * dist )/(max_speed * 1/3 * PI * WHEEL_DIAM) ;
        printf("Tacho will run for %f seconds \n", t);

        /* Set tachos to run for t seconds */
        set_tacho_time_sp( sn_left, t*1000 );
        set_tacho_time_sp( sn_right, t*1000 );
        set_tacho_command_inx( sn_left, TACHO_RUN_TIMED );
        set_tacho_command_inx( sn_right, TACHO_RUN_TIMED );

    } else {
        printf( "LEGO_EV3_M_MOTOR NOT found\n");
    }

    /* Waiting for the tacho to stop */
    Sleep( t*1000 );
}


int init_robot( void ) // Find the tachos
{
    uint8_t sn_compass;
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

    //Run all sensors
    ev3_sensor_init();
    if (ev3_search_sensor(LEGO_EV3_GYRO, &sn_compass,0)){
      printf("COMPASS found, reading compass...\n");
      if ( !get_sensor_value0(sn_compass, &init_value_compass )) {
        init_value_compass = 0;
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
    int is_init;
    is_init = init_robot();

    if(is_init == -1)
        return -1;

    shoot_from_stage1();
    Sleep(10000);
    shoot_from_stage2();
    Sleep(10000);
    shoot_from_stage3();
    Sleep(10000);
    shoot_from_stage4();
    Sleep(10000);
    shoot_from_stage5();
    return 0;
}
