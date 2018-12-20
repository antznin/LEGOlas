#ifndef CLIENT_H_
#define CLIENT_H_

void connect_bt(pthread_t client_th);
void close_bt(void);
void send_score(int sent_score);
void build_score_msg();

#endif
