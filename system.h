/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// functions declarations ------------------------------------------------------
void cfg_system(void);
void cfg_PWM(unsigned char set);
void cfg_PID(void);
void cfg_3P3Z(void);
void cfg_dspPID(void);
int offset_cal(void);
int offset_cal_blocking(void);
int load_parameters(void);
int save_m_counters(void);
void init_ic2_400khz(void);
void init_ic2_100khz(void);
void presettings(void);

void __attribute__((__interrupt__, auto_psv)) _PWM2Interrupt(void);
void __attribute__((__interrupt__, auto_psv)) _T1Interrupt(void);
void __attribute__((__interrupt__, auto_psv)) _DMA0Interrupt(void);
void __attribute__((__interrupt__, auto_psv)) _SPI1Interrupt(void);
void __attribute__((__interrupt__, auto_psv)) _U1RXInterrupt(void);

void __attribute__((__interrupt__, auto_psv)) _CM1Interrupt(void);
