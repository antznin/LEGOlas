#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"
#include "movement.h"
#include "ballDetection.h"

#define Sleep( msec ) usleep(( msec ) * 1000 )

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
} 
