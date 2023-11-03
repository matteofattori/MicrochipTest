/* 
 * File:   consolle.h
 * Author: marco.landoni
 *
 * Created on 14 luglio 2017, 9.28
 */

void send_val(void);
void send_limit(void);
void send_text(char *);
void send_text_stam(char * text, int size);
void send_coeff(int *);
int data_in(void);
int or_data_in(void);
int serial_set(void);
int man_set(void);
int send_brd_data(void);
void send_B_data(void);
void send_Counters(void);
void send_State(void);
void send_Temp(void);
void send_Vline(void);
void send_Scala(void);

