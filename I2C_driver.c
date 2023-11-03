
/******************************************************************************/
// This file is the I2C driver for both TMP431 and FRAM control
/******************************************************************************/

#include <p33EP256MC204.h>

#include "I2C_driver.h"
#include "system.h"
#include "main.h"

// PC8574 Remote I/O expander  XWB...
#define PCF8574_WR      0x40    // ref pdf datasheet pag 13
#define PCF8574_RD      0x41
#define PCF8574_P3      0xFF    // mi serve per abilitare il P3 
// TMP431 (Temperature sensor) --------------------------------------------------
#define TMP431A_C
//#define TMP431B_D

#ifdef TMP431A_C
#define TMP431_WR       0x98            // (0x4C << 1) | 0x00
#define TMP431_RD       0x99            // (0x4C << 1) | 0x01
#endif

#ifdef TMP431B_D
#define TMP431_WR       0x9A            // (0x4D << 1) | 0x00
#define TMP431_RD       0x9B            // (0x4D << 1) | 0x01
#endif

#define TMP431_ST_REG   0x02            // status register pointer address
#define TMP431_CFG1_REG 0x09            // config1 register write pointer address
#define TMP431_CFG2_REG 0x1A            // config2 register write pointer address
#define TMP431_CNV_REG  0x0A            // conversion rate register pointer address
#define TMP431_LOC_REG  0x00            // local temperature register pointer address
#define TMP431_REM_REG  0x01            // remote temperature register pointer address
#define TMP431_CONV     0x0F            // one-shot start of conversion
#define TMP431_BETA     0x25            // beta factor correction register pointer
#define TMP431_SC       0xAA            // dummy write to start conversion
#define TMP431_RESET    0xFC            // software reset

#define I2C_start       I2C1CONbits.SEN
#define I2C_stop        I2C1CONbits.PEN
#define I2C_Rstart      I2C1CONbits.RSEN
#define I2C_read        I2C1CONbits.RCEN
#define I2C_ACK         I2C1CONbits.ACKDT = 0
#define I2C_NACK        I2C1CONbits.ACKDT = 1
#define I2C_M_ACK       I2C1CONbits.ACKEN

#define I2C_busy        I2C1STATbits.TRSTAT
#define I2C_transmit    I2C1STATbits.TBF
#define I2C_ack         I2C1STATbits.TRSTAT
#define I2C_ACKST       I2C1STATbits.ACKSTAT
#define I2C_coll        I2C1STATbits.BCL

#define rem_sensor_open 0x04            // STATUS REGISTER.OPEN

// FRAM -------------------------------------------------------------------------
#define FRAM_WR         0xA8            // FRAM I2C address and write
#define FRAM_RD         0xA9            // FRAM I2C address and read


// CONSTANTS --------------------------------------------------------------------
const int TMP431_cfg[8] = {	TMP431_CFG1_REG, 0xE4,      // cfg1:        		ALERT masked, Shut Down, TERM mode, -55 ~ +150°C
                            TMP431_CFG2_REG, 0x1C,      // cfg2:                Enable ext and int sensor, enable R correction
                            TMP431_BETA, 0x0F,          // beta correction:		auto beta detect, diode connection
							TMP431_CONV, TMP431_SC};    // start with the first one-shot conversion

// VARIABLES --------------------------------------------------------------------
extern unsigned int TIMER_I2C;          // SW timer, 40us timebase
extern union ERR_FLAGS_type ERR_FLAGS;  // ERROR flags

extern struct  {  
    int     IGBT;       // remote sensor
    int     BOARD;      // local sensor
    int     TRAFO;      // trafo sensor
}T;

 // XWB  imposta sul PCF il P3 a 1 in modo da selezionare la scala in tensione con più sensibilità  , ho copiato la struttura di TMP432_drv            
int VscaleEN(char en)               
{
    char read=0;
    char write=0;
    // assert START, I2C address and write command
    I2C1STAT = 0x0000;              // clear status register         
   I2C_START();
    // issue read command
   if(I2C_WRITE(PCF8574_RD)) {       // read command
       //error
        I2C_STOP();                 // assert STOP
    }  else {   // !ACK received and no timeout
      // I2C_ACK;                   // prepare to issue a NACK
    }
    // read sequence and send NACK to stop transmission
    read = I2C_READ();
           // I2C_ACK;                  // issue NACK (previously loaded)
     I2C_STOP();                     // assert STOP
         // read =0;  
    if (en){
        write = read | 1 << 3;
    }else{
        write = read & ~(1 << 3);
    }
            I2C_START();
            if(I2C_WRITE(PCF8574_WR))     // leggo lo stato del registro così mantengo le impostazioni di prima e aggiungo un or con il P3 a 1
            {
        I2C_STOP();
    }

            if(I2C_WRITE(write))                 
            {
        I2C_STOP();
    }

     I2C_STOP();    
    // assert STOP          
          
    //init_ic2_400khz();
    return 0;  
}                

// controlla la scala impostata
// restituisce 0 se la scala è standard
// restituisce 1 se la scala è alta
int GetVscale(void)               
{
    char read = 0;
    int res = 0;
    // assert START, I2C address and write command
    I2C1STAT = 0x0000;              // clear status register         
    I2C_START();
    // issue read command
    if( I2C_WRITE(PCF8574_RD) ){        // read command
       // error
        I2C_STOP();                 // assert STOP
    } else {   // !ACK received and no timeout
        I2C_NACK;                   // prepare to issue a NACK
    }
    // read sequence and send NACK to stop transmission
    read = I2C_READ();
    I2C_M_ACK = 1;                  // issue NACK (previously loaded)
    I2C_STOP();                     // assert STOP        
    res = read & (1 << 3);                   
    //init_ic2_400khz();
    return res;  
}        


// Manage TMP431 ----------------------------------------------------------------
int TMP431_drv(void)
{   // this function manages TMP431, configure it at startup and read temp data
    // configuration is done at startup starting from step #100
    // normal loop reads status register, then reads data from previous conversion
    // at the end switch between internal or remote sensor and start new conversion
    
    // this function return 0 if the I2C bus is free or !=0 if the buffer is in hold
    
    // data are sampled every 500ms, one time internal, one time external, thus the net
    // sampling rate is 1s/s per each data
    // after sampling has done the function waits 500ms before reading data, during this time
    // vale returned is 0, releasing I2C buffer
       
    static int STATE_FUNC = 100;            // start from configuration
    static int i;
    int buffer;
    long acc;
    
    switch(STATE_FUNC)
    {   // READ STATUS REGISTER -------------------------------------------------
        case 0:
            if(!TIMER_I2C)
                STATE_FUNC++;
            break;
        case 1:
            // assert START, I2C address and write command
            I2C1STAT = 0x0000;              // clear status register
            I2C_START();
            STATE_FUNC++;
            break;
        case 2:
            if(I2C_WRITE(TMP431_WR))        // write command
            {   // error
                I2C_STOP();                 // assert STOP
                ERR_FLAGS.TMP = 1;
                STATE_FUNC = F_ERR;         // stop function execution
            }
            else
            {   // !ACK received and no timeout
                STATE_FUNC++;
                ERR_FLAGS.TMP = 0;
            }
            break;
        case 3:
            // Read STATUS REGISTER
            if(I2C_WRITE(TMP431_ST_REG))    // status register address
            {   // error
                I2C_STOP();                 // assert STOP
                ERR_FLAGS.TMP = 1;
                STATE_FUNC = F_ERR;         // stop function execution
            }
            else
            {   // !ACK received and no timeout
                I2C_RESTART();
                STATE_FUNC++;
                ERR_FLAGS.TMP = 0;
            }
            break;
        case 4:
            // issue read command
            if(I2C_WRITE(TMP431_RD))        // read command
            {   // error
                I2C_STOP();                 // assert STOP
                ERR_FLAGS.TMP = 1;
                STATE_FUNC = F_ERR;         // stop function execution
            }
            else
            {   // !ACK received and no timeout
                I2C_NACK;                   // prepare to issue a NACK
                STATE_FUNC++;
                ERR_FLAGS.TMP = 0;
            }
            break;
        case 5:
            // read sequence and send NACK to stop transmission
            buffer = I2C_READ();
            I2C_M_ACK = 1;                  // issue NACK (previously loaded)
            if(buffer & rem_sensor_open)
                ERR_FLAGS.REM = 1;          // signal error on remote sensor
            
            I2C_STOP();                     // assert STOP
            STATE_FUNC++;
            break;
        case 6:
            // READ TEMPERATURE DATA --------------------------------------------
            // prepare to read temperature data - value comes from previous conversion
            // assert START, I2C address and write command
            I2C_START();
            STATE_FUNC++;
            break;
        case 7:
            if(I2C_WRITE(TMP431_WR))        // write command
            {   // error
                I2C_STOP();                 // assert STOP
                ERR_FLAGS.TMP = 1;
                STATE_FUNC = F_ERR;         // stop function execution
            }
            else
            {   // !ACK received and no timeout
                STATE_FUNC++;
                ERR_FLAGS.TMP = 0;
            }
            break;
        case 8:
            // write address to be read: if i = 0 -> read internal, if i = 1 -> read external
            if(i)
                buffer = TMP431_REM_REG;    // external
            else
                buffer = TMP431_LOC_REG;    // internal
                
            if(I2C_WRITE(buffer))           // status register address
            {   // error
                I2C_STOP();                 // assert STOP
                ERR_FLAGS.TMP = 1;
                STATE_FUNC = F_ERR;         // stop function execution
            }
            else
            {   // !ACK received and no timeout
                I2C_RESTART();
                STATE_FUNC++;
                ERR_FLAGS.TMP = 0;
            }
            break;
        case 9:
            // issue read command
            if(I2C_WRITE(TMP431_RD))        // read command
            {   // error
                I2C_STOP();                 // assert STOP
                ERR_FLAGS.TMP = 1;
                STATE_FUNC = F_ERR;         // stop function execution
            }
            else
            {   // !ACK received and no timeout
                I2C_NACK;                   // prepare to issue a NACK
                STATE_FUNC++;
                ERR_FLAGS.TMP = 0;
            }
            break;
        case 10:
            // read sequence and send NACK to stop transmission
            // filter data and update temp structure
            buffer = I2C_READ() + 9;        // add offset in order to be compliant
                                            // with serial protocol representation
            I2C_M_ACK = 1;                  // issue NACK (previously loaded)
            if(i)
            {   // update remote T data
                acc = buffer + (T.IGBT * 3);
                T.IGBT = (int)(acc >> 2);
                i = 0;                      // next time update local data
            }
            else
            {   // update local T data
                acc = buffer + (T.BOARD * 3);
                T.BOARD = (int)(acc >> 2);
                i = 1;                      // next time update external data
            }
            
            I2C_STOP();                     // assert STOP
            STATE_FUNC++;
            break;
        case 11:
            // START A NEW CONVERSION -------------------------------------------
            // One-shot mode
            I2C_START();
            STATE_FUNC++;
            break;
        case 12:
            if(I2C_WRITE(TMP431_WR))        // write command
            {   // error
                I2C_STOP();                 // assert STOP
                ERR_FLAGS.TMP = 1;
                STATE_FUNC = F_ERR;         // stop function execution
            }
            else
            {   // !ACK received and no timeout
                STATE_FUNC++;
            }
            break;
        case 13:
            if(I2C_WRITE(TMP431_CONV))      // send pointer register
            {   // error
                I2C_STOP();                 // assert STOP
                ERR_FLAGS.TMP = 1;
                STATE_FUNC = F_ERR;         // stop function execution
            }
            else
            {   // !ACK received and no timeout
                STATE_FUNC++;
                ERR_FLAGS.TMP = 0;
            }
            break;
        case 14:
            if(I2C_WRITE(TMP431_SC))        // dummy data write to initiate a conversion
            {   // error
                I2C_STOP();                 // assert STOP
                ERR_FLAGS.TMP = 1;
                STATE_FUNC = F_ERR;         // stop function execution
            }
            else
            {   // !ACK received and no timeout
                I2C_STOP();                 // assert STOP
                TIMER_I2C = _500ms;         // free the buffer for 500ms
                STATE_FUNC = 0;             // close loop
                ERR_FLAGS.TMP = 0;
            }
            break;
        
        // INITIALIZE SENSOR (only at startup) ----------------------------------
        // SW RESET -------------------------------------------------------------
        case 100:
            // assert START, I2C address and write command
            I2C1STAT = 0x0000;              // clear status register
            I2C_START();
            STATE_FUNC++;
            break;
        case 101:
            if(I2C_WRITE(TMP431_WR))        // write command
            {   // error
                I2C_STOP();                 // assert STOP
                ERR_FLAGS.TMP = 1;
                STATE_FUNC = F_ERR;         // stop function execution
            }
            else
            {   // !ACK received and no timeout
                STATE_FUNC++;
                ERR_FLAGS.TMP = 0;
            }
            break;
        case 102:
            if(I2C_WRITE(TMP431_RESET))     // send pointer register
            {   // error
                I2C_STOP();                 // assert STOP
                ERR_FLAGS.TMP = 1;
                STATE_FUNC = F_ERR;         // stop function execution
            }
            else
            {   // !ACK received and no timeout
                STATE_FUNC++;
                ERR_FLAGS.TMP = 0;
            }
            break;
        case 103:
            if(I2C_WRITE(0x00))             // dummy data write to reset chip
            {   // error
                I2C_STOP();                 // assert STOP
                ERR_FLAGS.TMP = 1;
                STATE_FUNC = F_ERR;         // stop function execution
            }
            else
            {   // !ACK received and no timeout
                I2C_STOP();                 // assert STOP
                TIMER_I2C = _1ms;           // wait reset before continue
                i = 0;                      // reset pointer
                STATE_FUNC++;
                ERR_FLAGS.TMP = 0;
            }
            break;
        // SEND CHIP CONFIGURATION ----------------------------------------------
        case 104:
            if(!TIMER_I2C)
            {   // assert START, I2C address and write command
                I2C_START();
                STATE_FUNC++;
            }
            break;
        case 105:
            if(I2C_WRITE(TMP431_WR))        // write command
            {   // error
                I2C_STOP();                 // assert STOP
                ERR_FLAGS.TMP = 1;
                STATE_FUNC = F_ERR;         // stop function execution
            }
            else
            {   // !ACK received and no timeout
                STATE_FUNC++;
                ERR_FLAGS.TMP = 0;
            }
            break;
        case 106:
            // send register pointer
            if(I2C_WRITE(TMP431_cfg[i++]))
            {   // error
                I2C_STOP();                 // assert STOP
                ERR_FLAGS.TMP = 1;
                STATE_FUNC = F_ERR;         // stop function execution
            }
            else
            {   // !ACK received and no timeout
                STATE_FUNC++;
                ERR_FLAGS.TMP = 0;
            }
            break;
        case 107:
            // load data in the pointed cfg register
            if(I2C_WRITE(TMP431_cfg[i++]))
            {   // error
                I2C_STOP();                 // assert STOP
                ERR_FLAGS.TMP = 1;
                STATE_FUNC = F_ERR;         // stop function execution
            }
            else
            {   // !ACK received and no timeout
                STATE_FUNC++;
                ERR_FLAGS.TMP = 0;
            }
            break;
        case 108:
            I2C_STOP();                     // assert STOP
            if(i < 8)
            {   // last couple issued a conversion command, jump to data read
                TIMER_I2C = _500ms;         // free the buffer for 500ms
                STATE_FUNC = 0;
                i = 0;
            }
            else
            {   // output another couple pointer register - data
                STATE_FUNC = 104;
            }
            break;
            
        // ERROR STATE ----------------------------------------------------------
        case F_ERR:                         // do nothing...
            STATE_FUNC = 100;               // restart with sensor configuration
            break;
    }
    
    return STATE_FUNC;
}

// Write data on specific location ----------------------------------------------
// address must be inside the specific device space memory
int FRAM_write(unsigned int address, unsigned int data)
{
    static int STATE_FUNC = 0;
    static int j;
    int buffer;
    
    switch(STATE_FUNC)
    {
        case 0:
            // assert START, I2C address and write command
            I2C1STAT = 0x0000;              // clear status register
            I2C_START();
            STATE_FUNC++;
            break;
        case 1:
            if(I2C_WRITE(FRAM_WR))          // write command
            {   // error
                I2C_STOP();                 // assert STOP
                ERR_FLAGS.MEM = 1;
                STATE_FUNC = F_ERR;         // stop function execution
            }
            else
            {   // !ACK received and no timeout
                STATE_FUNC++;
            }
            break;
        case 2:
            // take the HIGH part of the address
            buffer = (unsigned int)(address & 0xFF00) >> 8;
            if(I2C_WRITE(buffer))           // write command
            {   // error
                I2C_STOP();                 // assert STOP
                ERR_FLAGS.MEM = 1;
                STATE_FUNC = F_ERR;         // stop function execution
            }
            else
            {   // !ACK received and no timeout
                STATE_FUNC++;
            }
            break;
        case 3:
            // take the LOW part of the address
            buffer = address & 0x00FF;
            if(I2C_WRITE(buffer))           // write command
            {   // error
                I2C_STOP();                 // assert STOP
                ERR_FLAGS.MEM = 1;
                STATE_FUNC = F_ERR;         // stop function execution
            }
            else
            {   // !ACK received and no timeout
                STATE_FUNC++;
                j = 0;                          // init number of bit to shift
            }
            break;
        case 4:
            // write 8bit of data at time starting from Least Significant Byte:
            // data 0xXXYY is written in FRAM in order 0x-YY-XX
            if(j < 16)
            {   // write single byte
                buffer = (unsigned int)(((unsigned int)data >> j) & 0x00FF);
                if(I2C_WRITE(buffer))           // write command (memory pointer is auto incremented)
                {   // error
                    I2C_STOP();                 // assert STOP
                    ERR_FLAGS.MEM = 1;
                    STATE_FUNC = F_ERR;         // stop function execution
                }
                else
                {   // continue with next byte
                    j = j + 8;
                }
            }
            else
            {   // assert STOP and exit
                I2C_STOP();                 // assert STOP
                STATE_FUNC = 0;
            }
            break;
            
        // ERROR STATE ----------------------------------------------------------
        case F_ERR:                         // do nothing...
            STATE_FUNC = 0;
            break;
    }
    
    return STATE_FUNC;
}

// Read data from specific location ---------------------------------------------
// address must be inside the specific device space memory
// at the end of read function variable at data address has been updated
int FRAM_read(unsigned int *data, unsigned int address)
{
    static int STATE_FUNC = 0;
    static long buffer;
    static int j;
    
    switch(STATE_FUNC)
    {
        case 0:
            // assert START, I2C address and write command
            I2C1STAT = 0x0000;              // clear status register
            I2C_START();
            STATE_FUNC++;
            break;
        case 1:
            if(I2C_WRITE(FRAM_WR))          // write command
            {   // error
                I2C_STOP();                 // assert STOP
                ERR_FLAGS.MEM = 1;
                STATE_FUNC = F_ERR;         // stop function execution
            }
            else
            {   // !ACK received and no timeout
                STATE_FUNC++;
            }
            break;
        case 2:
            // take the HIGH part of the address
            buffer = (unsigned int)(address & 0xFF00) >> 8;
            if(I2C_WRITE(buffer))           // write command
            {   // error
                I2C_STOP();                 // assert STOP
                ERR_FLAGS.MEM = 1;
                STATE_FUNC = F_ERR;         // stop function execution
            }
            else
            {   // !ACK received and no timeout
                STATE_FUNC++;
            }
            break;
        case 3:
            // take the LOW part of the address
            buffer = address & 0x00FF;
            if(I2C_WRITE(buffer))           // write command
            {   // error
                I2C_STOP();                 // assert STOP
                ERR_FLAGS.MEM = 1;
                STATE_FUNC = F_ERR;         // stop function execution
            }
            else
            {   // !ACK received and no timeout
                STATE_FUNC++;
            }
            break;
        case 4:
            // issue a restart and read command
            I2C_RESTART();
            STATE_FUNC++;
            break;
        case 5:
            if(I2C_WRITE(FRAM_RD))          // read command
            {   // error
                I2C_STOP();                 // assert STOP
                ERR_FLAGS.MEM = 1;
                STATE_FUNC = F_ERR;         // stop function execution
            }
            else
            {   // !ACK received and no timeout
                buffer = 0;
                j = 0;                      // init number of bit to shift
                STATE_FUNC++;
            }
            break;
        case 6:
            // read 8bit of data at time starting from Least Significant Byte:
            // data 0xXXYY is read from FRAM in order 0x-YY-XX
            // read single byte, FRAM memory pointer is auto incremented
            buffer = buffer | (unsigned int)((unsigned int)I2C_READ() << j);
            j = j + 8;

            if(j < 16)
            {   // issue an ACK
                I2C_ACK;
            }
            else
            {   // stop reading
                I2C_NACK;                   // prepare to issue a nACK
                STATE_FUNC++;
            }
            
            I2C_M_ACK = 1;                  // issue ACK or NACK
            break;
        case 7:
            I2C_STOP();                     // assert STOP
            *data = buffer;                 // update variable
            STATE_FUNC = 0;
            break;
            
        // ERROR STATE ----------------------------------------------------------
        case F_ERR:                         // do nothing...
            STATE_FUNC = 0;
            break;
    }
    
    return STATE_FUNC;
}

// I2C low level routines -------------------------------------------------------
// assert START -----------------------------------------------------------------
void I2C_START(void)
{   // assert start condition
    I2C_start = 1;                      // assert START
    while(I2C_start);    
}

// assert STOP ------------------------------------------------------------------
void I2C_STOP(void)
{   // assert stop condition
    I2C_stop = 1;                       // assert STOP
    while(I2C_stop);
}

// assert RESTART ---------------------------------------------------------------
void I2C_RESTART(void)
{   // assert RESTART condition
    I2C_Rstart = 1;                     // repeat START
    while(I2C_Rstart);
}

// write DATA to I2C ------------------------------------------------------------
int I2C_WRITE(int data)
{   // put data (8bits) to I2C bus
    int ack;
    
    I2C1TRN = (unsigned char)data;      // write data
    TIMER_I2C = _200us;                 // charge timeout
    while(TIMER_I2C | I2C_transmit);    // wait end transmission...
    while(TIMER_I2C | I2C_ack);         // wait ack reception...
    ack = I2C_ACKST | I2C_ack | I2C_coll;   // ack is 0 if !ACK was received AND transfer complete
                                            // ack is 1 if NACK was received OR transfer incomplete
    return ack;
}

// read DATA from I2C -----------------------------------------------------------
int I2C_READ(void)
{   // read I2C bus
    I2C_read = 1;                       // enable reading and start to send CK pulses
    TIMER_I2C = _200us;                 // charge timeout
    while(TIMER_I2C | I2C_read);        // wait...
    
    return (unsigned int)I2C1RCV;       // return data read
}
