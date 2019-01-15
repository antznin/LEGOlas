

#To run the docker
docker run --rm -it -h ev3 -v /Users/yasminebennani/Google\ Drive/Cours/Eurecom/FALL/OS_Group1_ev3Project/:/src -w /src ev3cc /bin/bash


#To compile a script .c
arm-linux-gnueabi-gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c movement.c -o movement.o
arm-linux-gnueabi-gcc movement.o -Wall -L./libraries -lrt -lm -lev3dev-c -o movement

arm-linux-gnueabi-gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c dead_reckoning.c -o dead_reckoning.o
arm-linux-gnueabi-gcc dead_reckoning.o -Wall -L./libraries -lrt -lpthread -lm -lev3dev-c -o dead_reckoning

