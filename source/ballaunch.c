#include <stdio.h>
#include <stdlib.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"
#include <unistd.h>
#define Sleep( msec ) usleep(( msec ) * 1000 )


void catch_ball(int msDuration){
    uint8_t sn;
    int portNb = 65;
    float speedFactor = -0.1;
    // int msDuration = 1600;
    if (ev3_search_tacho_plugged_in(portNb,0,&sn,0))
    {
        int max_speed;
        get_tacho_max_speed(sn, &max_speed);
        printf("  max speed = %d\n", max_speed );
        set_tacho_stop_action_inx(sn, TACHO_COAST);
        set_tacho_speed_sp(sn,speedFactor*max_speed);
        set_tacho_time_sp(sn, msDuration);
        set_tacho_command_inx(sn, TACHO_RUN_TIMED);
	Sleep(msDuration + 200);
    }
    else {
            printf( "LEGO_EV3_M_MOTOR %d is NOT found\n", (portNb-64));
        }

}

void place_hand_for_movement(){
    uint8_t sn;
    int portNb = 65;
    float speedFactor = -0.1;
    int msDuration = 1950;
    if (ev3_search_tacho_plugged_in(portNb,0,&sn,0))
    {
        int max_speed;
        get_tacho_max_speed(sn, &max_speed);
        printf("  max speed = %d\n", max_speed );
        set_tacho_stop_action_inx(sn, TACHO_COAST);
        set_tacho_speed_sp(sn,speedFactor*max_speed);
        set_tacho_time_sp(sn, msDuration);
        set_tacho_command_inx(sn, TACHO_RUN_TIMED);
	Sleep(msDuration + 200);
    }
    else {
            printf( "LEGO_EV3_M_MOTOR %d is NOT found\n", (portNb-64));
        }

}

void dismiss_hand_nc(){ //nc = no catapult
    uint8_t sn;
    int portNb = 65;
    float speedFactor = 0.1;
    int msDuration = 1850;
    if (ev3_search_tacho_plugged_in(portNb,0,&sn,0))
    {
        int max_speed;
        get_tacho_max_speed(sn, &max_speed);
        printf("  max speed = %d\n", max_speed );
        set_tacho_stop_action_inx(sn, TACHO_COAST);
        set_tacho_speed_sp(sn,speedFactor*max_speed);
        set_tacho_time_sp(sn, msDuration);
        set_tacho_command_inx(sn, TACHO_RUN_TIMED);
	Sleep(msDuration + 200);
    }
    else {
            printf( "LEGO_EV3_M_MOTOR %d is NOT found\n", (portNb-64));
        }

}

void dismiss_hand_wc(){ // wc= with catapult
    uint8_t sn;
    int portNb = 65;
    float speedFactor = 0.1;
    int msDuration = 1600;
    if (ev3_search_tacho_plugged_in(portNb,0,&sn,0))
    {
        int max_speed;
        get_tacho_max_speed(sn, &max_speed);
        printf("  max speed = %d\n", max_speed );
        set_tacho_stop_action_inx(sn, TACHO_COAST);
        set_tacho_speed_sp(sn,speedFactor*max_speed);
        set_tacho_time_sp(sn, msDuration);
        set_tacho_command_inx(sn, TACHO_RUN_TIMED);
	Sleep(msDuration + 200);
    }
    else {
            printf( "LEGO_EV3_M_MOTOR %d is NOT found\n", (portNb-64));
        }

}

void throw_ball(){
    uint8_t sn;
    int portNb = 68;
    float speedFactor = -1.0;
    int msDuration = 260;
    if (ev3_search_tacho_plugged_in(portNb,0,&sn,0))
    {
        int max_speed;
        get_tacho_max_speed(sn, &max_speed);
        printf("  max speed = %d\n", max_speed );
        set_tacho_stop_action_inx(sn, TACHO_BRAKE);
        set_tacho_speed_sp(sn,speedFactor*max_speed);
        set_tacho_time_sp(sn, msDuration);
        set_tacho_command_inx(sn, TACHO_RUN_TIMED);
	Sleep(msDuration + 200);
    }
    else {
            printf( "LEGO_EV3_M_MOTOR %d is NOT found\n", (portNb-64));
        }

}

void hold_catapult_for_movement(){
    uint8_t sn;
    int portNb = 68;
    float speedFactor = -0.5;
    int msDuration = 400;
    if (ev3_search_tacho_plugged_in(portNb,0,&sn,0))
    {
        int max_speed;
        get_tacho_max_speed(sn, &max_speed);
        printf("  max speed = %d\n", max_speed );
        set_tacho_stop_action_inx(sn, TACHO_HOLD);
        set_tacho_speed_sp(sn,speedFactor*max_speed);
        set_tacho_time_sp(sn, msDuration);
        set_tacho_command_inx(sn, TACHO_RUN_TIMED);
	Sleep(msDuration + 200);
    }
    else {
            printf( "LEGO_EV3_M_MOTOR %d is NOT found\n", (portNb-64));
        }

}
void release_catapult(){
    uint8_t sn;
    int portNb = 68;
    float speedFactor = 0;
    int msDuration = 100;
    if (ev3_search_tacho_plugged_in(portNb,0,&sn,0))
    {
        int max_speed;
        get_tacho_max_speed(sn, &max_speed);
        printf("  max speed = %d\n", max_speed );
        set_tacho_stop_action_inx(sn, TACHO_COAST);
        set_tacho_speed_sp(sn,speedFactor*max_speed);
        set_tacho_time_sp(sn, msDuration);
        set_tacho_command_inx(sn, TACHO_RUN_TIMED);
	Sleep(msDuration + 200);
    }
    else {
            printf( "LEGO_EV3_M_MOTOR %d is NOT found\n", (portNb-64));
        }

}




/*
int main() {
    printf( "*** ( EV3 ) Hello! ***\n" );

    ev3_tacho_init();

    //------------------------testing area-----------------
        
catch_ball();
Sleep(2600);
dismiss_hand_wc();
Sleep(2600);
throw_ball();
Sleep(1000);
hold_catapult_for_movement();
Sleep(3000);
release_catapult();
Sleep(1500);
hold_catapult_for_movement();
Sleep(3000);
place_hand_for_movement();
Sleep(3000);
dismiss_hand_nc();
Sleep(3000);
release_catapult();

    //-----------------------------------------------------

    ev3_uninit();

    printf( "*** ( EV3 ) Bye! ***\n" );
    return 0;
}*/
