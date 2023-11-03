
/******************************************************************************/
// This file contains UART consolle routines for control the HVDriver ----------
/******************************************************************************/

#include "main.h"
#include "consolle.h"
#include "math_u.h"
#include "stdio.h"
#include "stdlib.h"
#include "dsp.h"
#include "I2C_driver.h"
#include "system.h"
#include "i2c_stam.h"
#include <string.h>

extern long Vsum, Isum;
extern int max_v, max_i;
extern unsigned int TIMER_0;

extern struct  {   int V;      // + voltage limit
                   int nV;     // - voltage limit
                   int I;      // + current limit
                   int nI;     // - current limit
                   int VS;     // voltage shutdown limit (Vl + 50%)
                   int IS;     // current shutdown limit (Il + 50%)
                }limit;

// send rms values on UART -----------------------------------------------------
void send_val(void)
{   // send actual V and I through UART1
    extern char string[c_str_l];
    int v, i;
    int kv = _kv;
    int ki = _ki;
        
    if(!DMA1CONbits.CHEN)
    {   // only if not already busy
        U1_TX_IRQ_CLEAR;

        if(FLAGS.RMS)
        {   // send rms values
            v = s_sqrt(Vsum);
            v = (int)(__builtin_mulss((int)v, (int)kv) >> 15);
            i = s_sqrt(Isum);
            i = (int)(__builtin_mulss((int)i, (int)ki) >> 14);
            sprintf(string, " Vout = %5i Vrms   Iout = %5i mArms\r", v, i);
        }
        else
        {   // send instantaneous values
            v = (int)(__builtin_mulss((int)max_v, (int)kv) >> 15);
            i = (int)(__builtin_mulss((int)max_i, (int)ki) >> 14);
            sprintf(string, " Vout = %5i Vpk    Iout = %5i mApk \r", v, i);
        }

        DMA1CONbits.CHEN = 1;                       // enable DMA: DMA transfer will be
                                                    // automatically triggered by peripheral
        U1STAbits.UTXEN = 1;                        // enable TX
    }
}

// send string on UART ---------------------------------------------------------
void send_text(char * text)
{   // send text, array *text must be 40 chars
    extern char string[c_str_l];
    
    int i;
    
    U1_TX_IRQ_CLEAR;
    
    for(i = 0; i < c_str_l; i++)
    {
        string[i] = *text;                          // 40 bytes will be moved!!!!
        text++;                                     // source string MUST be c_str_l char length!!!!
    }
    
    DMA1CONbits.CHEN = 1;                           // enable DMA: DMA transfer will be
                                                    // automatically triggered by peripheral
    U1STAbits.UTXEN = 1;                            // enable TX
}

// send string on UART ---------------------------------------------------------
void send_text_stam(char * text, int size)
{   // send text, array *text must be 40 chars
    extern char string[c_str_l];
 
    U1_TX_IRQ_CLEAR;
    //memset(string, '\0', sizeof(string));
    memcpy(( char *) &string, text, size);
    
    DMA1CONbits.CHEN = 1;                           // enable DMA: DMA transfer will be
                                                    // automatically triggered by peripheral
    U1STAbits.UTXEN = 1;                            // enable TX
}

// send PID coefficients on UART -----------------------------------------------
#ifdef _PID
void send_coeff(int * coeff)
{   // print Kp, Ki, Kd
    // int format
    extern char string[c_str_l];
    
    U1_TX_IRQ_CLEAR;
    
    sprintf(string, " Kp = %5i, Ki = %5i, Kd = %5i   \n\r", coeff[0], coeff[1], coeff[2]);
    
    DMA1CONbits.CHEN = 1;                           // enable DMA: DMA transfer will be
                                                    // automatically triggered by peripheral
    U1STAbits.UTXEN = 1;                            // enable TX
}
#endif

// send limit values on UART ---------------------------------------------------
void send_limit(void)
{   // print lmit.V, limit.I
    // int format
    extern char string[c_str_l];
    
    U1_TX_IRQ_CLEAR;
    
    sprintf(string, " V_limit = %5i, I_limit = %5i    \n\r", limit.V, limit.I);
    
    DMA1CONbits.CHEN = 1;                           // enable DMA: DMA transfer will be
                                                    // automatically triggered by peripheral
    U1STAbits.UTXEN = 1;                            // enable TX
}

// read Kx coefficient ---------------------------------------------------------
#ifdef _PID
int data_in(void)
{   // select coefficient to be set, read and update it
    extern char string[c_str_l];
    extern char char_buffer;
    extern int pid_coeff[3];
    extern int pid_V_coeff[3],pid_V_18Vcoeff [3], pid_I_coeff[3];
    extern tPID PID_DATA;
    
    static int  STATE_FUNC = 0;
    static int  ptr, i;
    
    switch(STATE_FUNC)
    {
        case 0:           
            send_text(" Select PID coeff K[p], [i], [d]      \n\r");
            STATE_FUNC++;
            break;
        case 1:
            if(FLAGS.CHR)
            {   // char received
                FLAGS.CHR = DISABLE;
                switch(char_buffer)
                {
                    case 'p':
                        send_text(" Kp: (ddddd format)                   \n\r");
                        ptr = 0;
                        STATE_FUNC++;
                        TIMER_0 = _100ms;
                        break;
                    case 'i':
                        send_text(" Ki: (ddddd format)                   \n\r");
                        ptr = 1;
                        STATE_FUNC++;
                        TIMER_0 = _100ms;
                        break;
                    case 'd':
                        send_text(" Kd: (ddddd format)                   \n\r");
                        ptr = 2;
                        STATE_FUNC++;
                        TIMER_0 = _100ms;
                        break;
                    case _ESC:
                        send_text("                                      \n\r");
                        STATE_FUNC = END_FUNC;
                        break;
                    default:
                        break;            
                }
            }
            break;
        case 2:
            if(!TIMER_0)
            {   // after string has been printed...
                for(i = 1; i < 40; i++)
                    string[i] = 0x00;

                i = 1;
                string[0] = '\r';
                STATE_FUNC++;
            }
            break;
        case 3:
            if(FLAGS.CHR)
            {
                FLAGS.CHR = DISABLE;
                switch(char_buffer)
                {
                    case _BKS:  // backspace
                        if(i > 1)
                            i--;
                        string[i] = 0x00;
                        break;
                    case _CR:
                    case _ENT:  // enter
                        STATE_FUNC++;
                        break;
                    case _ESC:
                        string[0] = '\r';
                        string[1] = '\n';
                        STATE_FUNC = END_FUNC;
                        break;
                    default:    // load char
                        string[i++] = char_buffer;
                        break;
                }
                send_text(string);
            }
            break;
        case 4:
            if(!FLAGS.FRM && !FLAGS.TMP)
            {   // wait I2C bus to be free
                FLAGS.FRM = ENABLE;
                FLAGS.TMP = ENABLE;
                STATE_FUNC++;
            }
            break;
        case 5:
            pid_coeff[ptr] = atoi(string);              // convert string
            if(FLAGS.I_L)
            {   // save current mode parameter
                pid_I_coeff[ptr] = pid_coeff[ptr];
                STATE_FUNC = STATE_FUNC + 2;
            }
            else if (FLAGS.V_L && FLAGS.SCALE)          //aggiunto per avere avere i parametri del pid per la scala 18V
            {   // save voltage mode parameter scala 18V
                pid_V_18Vcoeff[ptr] = pid_coeff[ptr];
                STATE_FUNC++;
            }
            else
            {   // save voltage mode parameter scala 180V
                pid_V_coeff[ptr] = pid_coeff[ptr];
                STATE_FUNC++;
            }                        
            break;
        case 6:     // now save data to FRAM
            if (FLAGS.SCALE) //se la scala è 18V cambio puntatore
            {
                if(!FRAM_write((PID_V_18VADD + (ptr << 1)), (int)pid_V_18Vcoeff[ptr]))
                STATE_FUNC = STATE_FUNC + 2;
            }
            else // scala 180V
            {
                if(!FRAM_write((PID_V_ADD    + (ptr << 1)), (int)pid_V_coeff[ptr]))
                STATE_FUNC = STATE_FUNC + 2;    
            }    
            
            break;
        case 7:
            if(!FRAM_write((PID_I_ADD + (ptr << 1)), (int)pid_I_coeff[ptr]))
                STATE_FUNC++;
            break;
        case 8:
            send_text("\n\r                                    \n\r");
            FLAGS.TMP = DISABLE;                        // release I2C bus
            FLAGS.FRM = DISABLE;
            TIMER_0 = _100ms;
            STATE_FUNC++;
            break;
        case 9:
            if(!TIMER_0)
            {
                PIDInit(&PID_DATA);                     // clear data structure
                PIDCoeffCalc(&pid_coeff[0], &PID_DATA);     // initialize struct with a, b, c coeff.
           
                send_text(" OK!                                  \n\r");
                STATE_FUNC = END_FUNC;
            }
            break;
    }
    
    return (STATE_FUNC);
}
#endif

// read V, I limit value -------------------------------------------------------
int or_data_in(void)
{   // select coefficient to be set, read and update it
    extern char string[c_str_l];
    extern char char_buffer;
    
    static int  STATE_FUNC = 0;
    static int  ptr, i;
    
    switch(STATE_FUNC)
    {
        case 0:
            send_text(" Select limit: L[v], L[i]             \n\r");
            STATE_FUNC++;
            break;
        case 1:
            if(FLAGS.CHR)
            {   // char received
                FLAGS.CHR = DISABLE;
                switch(char_buffer)
                {
                    case 'v':
                        send_text(" Lv: (ddddd format)                   \n\r");
                        ptr = 0;
                        STATE_FUNC++;
                        TIMER_0 = _100ms;
                        break;
                    case 'i':
                        send_text(" Li: (ddddd format)                   \n\r");
                        ptr = 1;
                        STATE_FUNC++;
                        TIMER_0 = _100ms;
                        break;
                    case _ESC:
                        send_text("                                      \n\r");
                        STATE_FUNC = END_FUNC;
                        break;
                    default:
                        break;            
                }
            }
            break;
        case 2:
            if(!TIMER_0)
            {   // after string has been printed...
                for(i = 1; i < 40; i++)
                    string[i] = 0x00;

                i = 1;
                string[0] = '\r';
                STATE_FUNC++;
            }
            break;
        case 3:
            if(FLAGS.CHR)
            {
                FLAGS.CHR = DISABLE;
                switch(char_buffer)
                {
                    case _BKS:  // backspace
                        if(i > 1)
                            i--;
                        string[i] = 0x00;
                        break;
                    case _CR:
                    case _ENT:  // enter
                        STATE_FUNC++;
                        break;
                    case _ESC:
                        string[0] = '\r';
                        string[1] = '\n';
                        STATE_FUNC = END_FUNC;
                        break;
                    default:    // load char
                        string[i++] = char_buffer;
                        break;
                }
                send_text(string);
            }
            break;
        case 4:
            i = atoi(string);               // convert string
            if(!ptr)
            {   // load voltage limit
                limit.V = i;
#ifdef _zero
                limit.nV = 50;
#else
                limit.nV = i - (i >> 5);    // add 3% hysteresis
#endif
                if(limit.V > max_V_limit)
                    limit.VS = max_VS;                          // shutdown limit = max
                else
                    limit.VS = limit.V + (limit.V >> 1);        // shutdown limit = limit * 1.5
            }
            else
            {
                limit.I = i;
#ifdef _zero
                limit.nI = 50;
#else
                limit.nI = i - (i >> 5);    // add 3% hysteresis
#endif
                if(limit.I > max_I_limit)
                    limit.IS = max_IS;                          // shutdown limit = max
                else
                    limit.IS = limit.I + (limit.I >> 1);        // shutdown limit = limit * 1.5
            }
            STATE_FUNC++;
            break;
        case 5:
            send_text("\n\r                                    \n\r");
            TIMER_0 = _100ms;
            STATE_FUNC++;
            break;
        case 6:
            if(!TIMER_0)
            {
                send_text(" OK!                                  \n\r");
                STATE_FUNC = END_FUNC;
            }
            break;
    }
    
    return (STATE_FUNC);
}

// Set new serial number -------------------------------------------------------
// it will be saved on specifica FRAM address
int serial_set(void)
{   // input serial number and write it into FRAM
    extern char string[c_str_l];
    extern char char_buffer;
    extern char serial_n[6];
    
    static int  STATE_FUNC = 0;
    static int  i;
    int data;
    
    switch(STATE_FUNC)
    {
        case 0:
            send_text(" Input SERIAL Number: nnnnn           \n\r");
            STATE_FUNC++;
            TIMER_0 = _100ms;
            break;
        case 1:
            if(!TIMER_0)
            {   // after string has been printed...
                for(i = 1; i < 40; i++)
                    string[i] = 0x00;

                i = 1;
                string[0] = '\r';
                STATE_FUNC++;
            }
            break;
        case 2:
            if(FLAGS.CHR)
            {   // new char received
                FLAGS.CHR = DISABLE;
                switch(char_buffer)
                {
                    case _BKS:  // backspace
                        if(i > 1)
                            i--;
                        string[i] = 0x00;
                        break;
                    case _CR:
                    case _ENT:  // enter
                        STATE_FUNC++;
                        break;
                    case _ESC:
                        string[0] = '\r';
                        string[1] = '\n';
                        FLAGS.TMP = 0;              // release I2C bus
                        FLAGS.FRM = 0;
                        STATE_FUNC = END_FUNC;
                        break;
                    default:    // load char
                        string[i++] = char_buffer;
                        break;
                }
                send_text(string);
            }
            break;
        case 3:
            serial_n[5] = 0x00;                     // not used
            serial_n[4] = string[1];                // serial_n[] is a char like string[] is
            serial_n[3] = string[2];                // no conversion needed
            serial_n[2] = string[3];
            serial_n[1] = string[4];
            serial_n[0] = string[5];
            STATE_FUNC++;
            break;
        case 4:
            if(!FLAGS.TMP && !FLAGS.FRM)
            {   // I2C bus is free -> hold it and write data
                FLAGS.TMP = 1;
                FLAGS.FRM = 1;
                STATE_FUNC++;
            }
            break;
        case 5:
            data = (int)(serial_n[5] << 8 | serial_n[4]);
            if(!FRAM_write(SERIAL_H, (int)(data)))
                STATE_FUNC++;
            break;
        case 6:
            data = (int)(serial_n[3] << 8 | serial_n[2]);
            if(!FRAM_write(SERIAL_M, (int)(data)))
                STATE_FUNC++;
            break;
        case 7:
            data = (int)(serial_n[1] << 8 | serial_n[0]);
            if(!FRAM_write(SERIAL_L, (int)(data)))
                STATE_FUNC++;
            break;
        case 8:
            FLAGS.TMP = 0;                      // release I2C bus
            FLAGS.FRM = 0;
            send_text("\n\r                                    \n\r");
            TIMER_0 = _100ms;
            STATE_FUNC++;
            break;
        case 9:
            if(!TIMER_0)
            {
                send_text(" OK!                                  \n\r");
                STATE_FUNC = END_FUNC;
            }
            break;
    }
    
    return (STATE_FUNC);
}

// Set new manufacturer code ---------------------------------------------------
// it will be saved on specific FRAM address
int man_set(void)
{   // input manufacturer and write it into FRAM
    extern char string[c_str_l];
    extern char char_buffer;
    extern char man[4];
    
    static int  STATE_FUNC = 0;
    static int  i;
    int data;
    
    switch(STATE_FUNC)
    {
        case 0:
            send_text(" Input MANUFACTURER (lowercase): mmm  \n\r");
            STATE_FUNC++;
            TIMER_0 = _100ms;
            break;
        case 1:
            if(!TIMER_0)
            {   // after string has been printed...
                for(i = 1; i < 40; i++)
                    string[i] = 0x00;

                i = 1;
                string[0] = '\r';
                STATE_FUNC++;
            }
            break;
        case 2:
            if(FLAGS.CHR)
            {   // new char received
                FLAGS.CHR = DISABLE;
                switch(char_buffer)
                {
                    case _BKS:  // backspace
                        if(i > 1)
                            i--;
                        string[i] = 0x00;
                        break;
                    case _CR:
                    case _ENT:  // enter
                        STATE_FUNC++;
                        break;
                    case _ESC:
                        string[0] = '\r';
                        string[1] = '\n';
                        FLAGS.TMP = 0;              // release I2C bus
                        FLAGS.FRM = 0;
                        STATE_FUNC = END_FUNC;
                        break;
                    default:    // load char
                        string[i++] = char_buffer;
                        break;
                }
                send_text(string);
            }
            break;
        case 3:
            man[3] = 0x00;                          // not used
            man[2] = string[1];                     // man[] is a char like string[] is
            man[1] = string[2];                     // no conversion needed
            man[0] = string[3];
            STATE_FUNC++;
            break;
        case 4:
            if(!FLAGS.TMP && !FLAGS.FRM)
            {   // I2C bus is free -> hold it and write data
                FLAGS.TMP = 1;
                FLAGS.FRM = 1;
                STATE_FUNC++;
            }
            break;
        case 5:
            data = (int)(man[3] << 8 | man[2]);
            if(!FRAM_write(MAN_H, (int)(data)))
                STATE_FUNC++;
            break;
        case 6:
            data = (int)(man[1] << 8 | man[0]);
            if(!FRAM_write(MAN_L, (int)(data)))
                STATE_FUNC++;
            break;
        case 7:
            FLAGS.TMP = 0;                          // release I2C bus
            FLAGS.FRM = 0;
            send_text("\n\r                                    \n\r");
            TIMER_0 = _100ms;
            STATE_FUNC++;
            break;
        case 8:
            if(!TIMER_0)
            {
                send_text(" OK!                                  \n\r");
                STATE_FUNC = END_FUNC;
            }
            break;
    }
    
    return (STATE_FUNC);
}

// Send board ID and counters on consolle
int send_brd_data(void)
{
    static int  STATE_FUNC = 0;

    switch(STATE_FUNC)
    {
        case 0:
            send_text("\n\r ------ High Voltage Driver -------\n\r");
            TIMER_0 = _200ms;
            STATE_FUNC++;
            break;
        case 1:
            if(!TIMER_0)                                // wait end of uart transmission
            {
                send_B_data();                          // then transmit board ID
                TIMER_0 = _200ms;
                STATE_FUNC++;
            }
            break;
        case 2:
            if(!TIMER_0)                                // wait end of uart transmission
            {
                send_Counters();                        // then transmit counters
                TIMER_0 = _200ms;
                STATE_FUNC++;
            }
            break;
        case 3:
            if(!TIMER_0)                                // wait end of uart transmission
            {

                send_State();                           // then transmit state
                TIMER_0 = _200ms;
                STATE_FUNC++;
            }
            break;
        case 4:
            if(!TIMER_0)
            {
                if(FLAGS.DC)
                    send_text("\n\r DC MODE                            \n\r");
                else
                    send_text("\n\r AC MODE                            \n\r");
                
                TIMER_0 = _200ms;
                STATE_FUNC++;
            }
            break;
        case 5:
            if(!TIMER_0)                                // wait end of uart transmission
            {
                stam_rele_off();
                send_Scala();                           // then transmit state
                TIMER_0 = _1s;
                STATE_FUNC++;
            }
            break;
          case 6:
            if(!TIMER_0)                                // wait end of uart transmission
            {
                send_Temp();                            // then transmit temperatures
                TIMER_0 = _200ms;
                STATE_FUNC++;
            }
            break;
        case 7:
            if(!TIMER_0)                                // wait end of uart transmission
            {
                send_Vline();                           // then transmit state
                TIMER_0 = _200ms;
                STATE_FUNC++;
            }
            break;
        case 8:
            if(!TIMER_0)                                // wait end of uart transmission
            {
                //send_Vline();   
                //TIMER_0 = _200ms; // then transmit state
                STATE_FUNC++;
            }
            break;
        case 9:
            if(!TIMER_0)                                // wait end of uart transmission
            {
                //send_Vline();
               offset_cal_blocking();
               // TIMER_0 = _200ms;               // calibrating
                STATE_FUNC = END_FUNC;
            }
            break;
    }

    return (STATE_FUNC);
}

// send board data on UART -----------------------------------------------------
void send_B_data(void)
{
    extern char string[c_str_l];
    extern char man[4];
    extern char serial_n[6];
    
    U1_TX_IRQ_CLEAR;
    
    // fill string fields
    sprintf(string, " MAN:     S/N:      \n\r HW:  .  FW:  . \n\r");
    string[6] = man[2];
    string[7] = man[1];
    string[8] = man[0];
    
    string[15] = serial_n[4];
    string[16] = serial_n[3];
    string[17] = serial_n[2];
    string[18] = serial_n[1];
    string[19] = serial_n[0];
    
    string[27] = HW_VERa;
    string[29] = HW_VERb;
    
    string[35] = FW_VERa;
    string[37] = FW_VERb;
    
    DMA1CONbits.CHEN = 1;                           // enable DMA: DMA transfer will be
                                                    // automatically triggered by peripheral
    U1STAbits.UTXEN = 1;                            // enable TX
}

// send counters on UART -------------------------------------------------------
void send_Counters(void)
{
    extern char string[c_str_l];
    extern unsigned long min_counter;
    extern unsigned long min_on_counter;
    
    U1_TX_IRQ_CLEAR;
    
    if(ERR_FLAGS.MEM)
    {   // fill string with error message
        sprintf(string, " Backup memory ERROR                 \n\r\n");
    }
    else
    {   // fill string fields
        sprintf(string, " ON: %10lu m\n\r WAVE: %8lu m \n\r\n", min_counter, min_on_counter);
    }
    
    DMA1CONbits.CHEN = 1;                           // enable DMA: DMA transfer will be
                                                    // automatically triggered by peripheral
    U1STAbits.UTXEN = 1;                            // enable TX
}

// send temperatures on UART ---------------------------------------------------
void send_Temp(void)
{
    extern char string[c_str_l];
    extern struct {   
                            int     IGBT;               // IGBT temperature
                        int     BOARD;              // board temperature
                        int     TRAFO;              // trafo temperature
                    }T;
    
    int tdrv, tcpu;
    
    U1_TX_IRQ_CLEAR;
    
    tdrv = T.IGBT - 73;
    tcpu = T.BOARD - 73;
    
    if(ERR_FLAGS.TMP)
    {   // fill string fields with error message
        sprintf(string, " Temperature Sensor ERROR            \n\r\n");
    }
    else if((int)tdrv > (int)-10)
    {   // tdrv OK
        if((int)tcpu > (int)-10)
        {   // tcpu OK -> fill string fields
            sprintf(string, " Tdrv: %+3i 'C\n\r Tcpu: %+3i 'C         \n\r\n", tdrv, tcpu);
        }
        else
        {   // fill only tdrv
            sprintf(string, " Tdrv: %+3i 'C\n\r Tcpu: --- 'C          \n\r\n", tdrv);
        }
    }
    else
    {   // tdrv not OK
        if((int)tcpu > (int)-30)
        {   // tcpu OK -> fill only tcpu
            sprintf(string, " Tdrv: --- 'C\n\r Tcpu: %+3i 'C         \n\r\n", tcpu);
        }
        else
        {   // fill nothing
            sprintf(string, " Tdrv: --- 'C\n\r Tcpu: --- 'C         \n\r\n");
        }
    }
    
    DMA1CONbits.CHEN = 1;                           // enable DMA: DMA transfer will be
                                                    // automatically triggered by peripheral
    U1STAbits.UTXEN = 1;                            // enable TX
}

// send board state on UART ----------------------------------------------------
void send_State(void)
{
    extern char string[c_str_l];
    extern int N_CYCLES;
    
    U1_TX_IRQ_CLEAR;
    
    // fill string fields
    sprintf(string, " CAL -\n\r POWER -\n\r LOOP -\n\r CYCLES %2i \n\r", N_CYCLES);
    
    if(FLAGS.CAL)
        string[5] = 'X';
    if(!DRV_SHUTDOWN)
        string[15] = 'X';
    if(FLAGS.I_L)
        string[24] = 'I';
    if(FLAGS.V_L)
        string[24] = 'V';
    
    DMA1CONbits.CHEN = 1;                           // enable DMA: DMA transfer will be
                                                    // automatically triggered by peripheral
    U1STAbits.UTXEN = 1;                            // enable TX
}

// send Vline on UART ----------------------------------------------------------
void send_Vline(void)
{
    extern char string[c_str_l];
    extern unsigned int VlineV;
    
    U1_TX_IRQ_CLEAR;
    
    // fill string fields
    sprintf(string, " Supply Voltage: %3i V               \n\r\n", VlineV);  // printf(" U2STA=%d, RCON=%d               \n\r\n", U2STA,RCON); serviva per capire come si resettava
   
    DMA1CONbits.CHEN = 1;                           // enable DMA: DMA transfer will be
                                                    // automatically triggered by peripheral
    U1STAbits.UTXEN = 1;                            // enable TX
}

// send Vline on UART ----------------------------------------------------------
void send_Scala(void)
{
    extern char string[c_str_l];
    int res=0;
     init_ic2_100khz();
    res = GetVscale();
    if (res){
        sprintf(string, " scale active: HIGH                  \n\r\n"); 
    }else{
        sprintf(string, " scale active: STANDARD              \n\r\n");
    }
    U1_TX_IRQ_CLEAR;     
    DMA1CONbits.CHEN = 1;                           // enable DMA: DMA transfer will be
    // automatically triggered by peripheral
    U1STAbits.UTXEN = 1;                            // enable TX
}

