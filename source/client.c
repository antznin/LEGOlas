#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdarg.h>
#include <time.h>
#include <sys/socket.h>
#include <math.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <pthread.h>
#include "client.h"
#include "dead_reckoning.h"
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"

#define SERV_ADDR	"18:5e:0f:9d:7c:99"     /* Server's MAC address */
#define TEAM_ID 	1                       /* Team ID */
#define MSG_ACK 	0
#define MSG_START 	1
#define MSG_STOP 	2
#define MSG_KICK 	3
#define MSG_SCORE 	4
#define MSG_CUSTOM 	8

#define SIG_SCORE3 	3
#define SIG_SCORE1 	1

#define Sleep( msec ) usleep(( msec ) * 1000 )

int s; // socket
uint16_t msgId = 0;

/* score represents the score to be sent to the server */
int score;

/* hasEnded represents the state of the bluetooth connection 
 *  - 1 means the connection is still running
 *  - 0 means the connection must end, i.e. the robot 
 *  	has finished his job */
int hasEnded = 1;

int hasStarted = 0;

char score_str[58];

pthread_mutex_t client_mutex;
pthread_cond_t cond;

/* The client thread routine.
 * Checks for mutex and condtion variable then proceeds to
 * message sending.
 */
static void * client_thread_routine(void * data) {
	
	if (pthread_mutex_lock(&client_mutex) != 0) {
		fprintf(stderr, "Mutex lock error, exiting\n");
		exit(EXIT_FAILURE);
	}
	
	printf("Client is entering sleep mode...\n");

	while (hasEnded != 0) {
		printf("Before waiting\n");
		if (pthread_cond_wait(&cond, &client_mutex) != 0) {
			fprintf(stderr, "Cond wait error, exiting\n");
			exit(EXIT_FAILURE);
		}
		printf("Client has woken up\n");

		if (hasEnded != 0) {
			build_score_msg();
		}

		printf("Wait over, sending score %d\nMessage id : %d\n",
						score, msgId);
		
		write(s, score_str, 6);
		printf("Finished sending message\n");
	}
}


static void * waiting_thread_routine(void * data) {

	char string[58];
	uint8_t sn1, sn2;
	int port1, port2;
	port1 = 66; port2 = 67; // left motor, right motor

	while (1) {
		read_from_server (s, string, 9);
		printf("Reading from server...\n");
		if (string[4] == MSG_START) {
			hasStarted = 1;
		} else if (string[4] == MSG_STOP || string[4] == MSG_KICK) {
			hasEnded = 1;
			if ( ev3_search_tacho_plugged_in(port1,0, &sn1, 0 )
					&& ev3_search_tacho_plugged_in(port2,0, &sn2, 0 )) {
				set_tacho_command_inx(sn1, TACHO_STOP);
				set_tacho_command_inx(sn2, TACHO_STOP);
			}
			exit_robot();
			exit(0);
		}
		Sleep(10);
	}
	close_bt();
}

/* Function to build the score message with the 
 * current score according to the given description on
 * https://gitlab.eurecom.fr/ludovic.apvrille/OS_Robot_Project_Fall2018
 */
void build_score_msg() {
	*((uint16_t *) score_str) = msgId++;
	score_str[2] = TEAM_ID;
	score_str[3] = 0xFF;
	score_str[4] = MSG_SCORE;
	score_str[5] = score;
}

/* Function used by main thread to wake up
 * the client thread and send a specified
 * score.
 * @param : score
 */
void send_score(int sent_score) {
	score = sent_score;
	/* Wakes up the client thread */
	pthread_cond_signal(&cond);
}

/* Credits to Matteo Bertolino 
 * https://gitlab.eurecom.fr/matteo.bertolino */
static int read_from_server (int sock, char *buffer, size_t maxSize) {
	int bytes_read = read (sock, buffer, maxSize);
	if (bytes_read <= 0) {
		fprintf (stderr, "Server unexpectedly closed connection...\n");
		close (s);
		exit (EXIT_FAILURE);
	}
	printf ("[DEBUG] received %d bytes\n", bytes_read);
	return bytes_read;
}

/* Credits to Matteo Bertolino 
 * https://gitlab.eurecom.fr/matteo.bertolino */
void connect_bt() {
	struct sockaddr_rc addr = { 0 };
	int status;
	int thread_ret;
	pthread_t client_thread;
	pthread_t waiting_thread;

	/* allocate a socket */
	s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
	
	/* set the connection parameters (who to connect to) */
	addr.rc_family = AF_BLUETOOTH;
	addr.rc_channel = (uint8_t) 1;
	str2ba (SERV_ADDR, &addr.rc_bdaddr);
	
	/* connect to server */
	status = connect(s, (struct sockaddr *)&addr, sizeof(addr));
	
	/* if connected */
	if( status == 0 ) {
		/* CLIENT THREAD INITIALIZATION */
		if (pthread_mutex_init(&client_mutex, NULL) != 0) {
			fprintf(stderr, "Client mutex init error, exiting\n");
			exit(EXIT_FAILURE);
		}
		if (pthread_cond_init(&cond, NULL) != 0) {
			fprintf(stderr, "Client cond init error, exiting\n");
			exit(EXIT_FAILURE);
		}

		if ( pthread_create(&client_thread, NULL,
					client_thread_routine, NULL) != 0) {
			fprintf(stderr, "Client thread init error, exiting\n");
			exit(EXIT_FAILURE);
		}

		if (pthread_create(&waiting_thread, NULL,
					waiting_thread_routine, NULL) != 0) {
			fprintf(stderr, "Client waiting thread init error, exiting\n");
			exit(EXIT_FAILURE);
		}

	} else {
		fprintf (stderr, "Failed to connect to server...\n");
		sleep (2);
		exit (EXIT_FAILURE);
	}
	return;
}

/* Closes the bluetooth connection
 */
void close_bt(void) {
	hasEnded = 0;
	pthread_cond_signal(&cond);
	close (s);
}
