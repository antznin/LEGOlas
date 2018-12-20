#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdarg.h>
#include <time.h>
#include <sys/socket.h>
#include <math.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>

#define SERV_ADDR	"dc:53:60:ad:61:90"     /* Whatever the address of the server is */
#define TEAM_ID 	1                       /* Your team ID */

#define MSG_ACK 	0
#define MSG_START 	1
#define MSG_STOP 	2
#define MSG_KICK 	3
#define MSG_SCORE 	4
#define MSG_CUSTOM 	8

#define Sleep( msec ) usleep(( msec ) * 1000 )

int s;
uint16_t msgId = 0;

void send_score(int score) {
	
}

/* Credits to Matteo Bertolino 
 * https://gitlab.eurecom.fr/matteo.bertolino */
void connect_bt(void) {
	struct sockaddr_rc addr = { 0 };
	int status;

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
		char string[58];
		/* Wait for START message */
		read_from_server (s, string, 9);
		if (string[4] == MSG_START) {
			printf ("Received start message!\n");
		}
	} else {
		fprintf (stderr, "Failed to connect to server...\n");
		sleep (2);
		exit (EXIT_FAILURE);
	}
	
	close_bt();
	return 0;
}

void close_bt(void) {
	close (s);
}
