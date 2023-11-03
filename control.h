/* 
 * File:   control.h
 * Author: marco.landoni
 *
 * Created on 6 ottobre 2017, 11.53
 */

int process_packet(void);
void __attribute__((__interrupt__, auto_psv)) _U2RXInterrupt(void);
