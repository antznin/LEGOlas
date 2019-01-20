#ifndef CLIENT_H_
#define CLIENT_H_

void connect_bt();
void close_bt(void);
void send_score(int sent_score);
void build_score_msg();
static int read_from_server (int sock, char *buffer, size_t maxSize);

extern int hasEnded;
extern int hasStarted;

#endif
