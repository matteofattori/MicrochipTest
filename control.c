
/******************************************************************************/
// This file is the driver for the serial port control. It implements the
// communication protocol with the host processor.
/******************************************************************************/

#include <p33EP256MC204.h>

#include "control.h"
#include "main.h"
#include "math_u.h"


// Packet format is the following:
// STX | L2 - L1 - L0 | C1 - C0 | Dn - ... - D0 | CRC1 - CRC0 | ETX
//  |          |           |            |                |       |
// START    LENGTH      COMMAND     DATA (n bytes)      CRC     END

// LENGTH = n°bytes CMD + n°bytes DATA = n°bytes total - 7
// COMMAND starts from byte[4]
// DATA starts from byte[6]

// ACK, NACK, ENQ are single byte command

// DEFINITIONS -----------------------------------------------------------------
#define P_STX               0x02        // start byte
#define P_ETX               0x03        // stop byte
#define P_ENQ               0x05        // enquiry byte
#define P_ACK               0x06        // ACK
#define P_NACK              0x15        // NACK

#define P_SETHWGENERAL      0x3130      // set current configuration
#define P_GETHWCONF         0x3131      // return current configuration
#define P_SETVLIMIT         0x3132      // set voltage limit (overvoltage assertion)
#define P_SETILIMIT         0x3133      // set current limit (overcurrent assertion)
#define P_SETSUPPLYOUT      0x3134      // enable output
#define P_RESETSUPPLYOUT    0x3230      // reset output
#define P_RESETOFFSET       0x3235      // offset zeroing
#define P_RESETERR          0x3231      // reset selected errors
#define P_DISABLEERR        0x3232      // disable selected errors
#define P_ENABLEERR         0x3233      // enable selected errors
#define P_GETERRLIST        0x3234      // return alarm list
#define P_GETTEMPER         0x3431      // return temperatures
#define P_GETDIAGNOSTIC     0x3432      // return diagnostic P_SETSUPPLYOUTmeasures
#define P_RESETCOUNTERS     0x3435      // activity counters reset
#define P_RESETDIAGNOSTIC   0x3433      // diagnostic measure reset
#define P_NEWTEST            0x3530      // initiate a new test sequence
#define P_RESETCY                       0x3531      // reset current test sequence
#define P_GETCY                         0x3532      // send cycle n to the master
#define P_SETENDCY                   0x3536      // abort current cycle
#define P_SETCYCH                     0x3534      // set cycle n
#define P_BOOTENTRY                 0x5581      // boot entry command
#define P_GET_V_I_SUPPLY        0x3430      // measure v and i input and output (added to prevent a bug on   the ISA protocol)

#define BTX_message         199
#define TX_message          200

// VARIABLES -------------------------------------------------------------------
unsigned char RX_buffer[256];           // RX data buffer (all chars, header and footer included)
unsigned int b_index = 0;               // RX data buffer index
unsigned int RX_length = 0;             // RX packet length (built from the frame)
unsigned int RX_command = 0;            // RX command (built from the frame)
unsigned int RX_CRC = 0;                // RX CRC (built from the frame)

unsigned char TX_buffer[48];            // TX data buffer (all chars, header and footer included)
// this buffer is sent via DMA to UART
unsigned char TX_sec_buffer[48];        // TX buffer that contains answer for ENQ
unsigned int TX_length;
unsigned int TX_sec_length;

extern unsigned int RX_timeout;         // timeout reception
extern unsigned int STATE_MACHINE;      // state machine
extern struct CFG_type HW_CFG;          // HW configuration formatted as protocols wants
extern char man[4];                     // manufacturer name (stored in FRAM)
extern char serial_n[6];                // serial number (stored in FRAM)
// HW ver and FW ver are stored as #define

    int kv = _kv;
    int ki = _ki;

extern struct   {   int     IGBT;       // remote sensor
                    int     BOARD;      // local sensor
                    int     TRAFO;      // trafo sensor
                }T;

extern struct   {   int V;              // voltage limit (rising)
                    int nV;             // voltage limit (falling)
                    int I;              // current limit (rising)
                    int nI;             // current limit (falling)
                    int VS;             // voltage shutdown limit (Vlimit + 50%)
                    int IS;             // current shutdown limit (Ilimit + 50%)
                }limit;

extern struct   {   unsigned long M;    // phase accumulator increment
                    unsigned long PHA;  // phase accumulator
                }DDS;

extern int scale;

extern cycle_str_type CYCLES[15];       // cycles data structure
int N_CYCLES = 0;                       // number of cycles currently in memory
int EXP_N_CYCLES = 0;                   // number of cycles to be expected via serial protocol

extern int VlineV;                      // actual Vline value
extern int max_Vline;                   // max Vline represented in V
extern int min_Vline;                   // min Vline
extern int I_OUT;                       // max current represented in A/10 (ex. 26 = 2.6A)
extern unsigned int DEEP_SUPPLY;        // supply interruption counter (ms)
extern unsigned long min_counter;       // minute power ON counter
extern unsigned long min_on_counter;    // minute generation ON counter

// UART2 RX IRQ ----------------------------------------------------------------
void __attribute__((__interrupt__, auto_psv)) _U2RXInterrupt(void)
{   // this function, called at each character reception, builds the RX_buffer
    // and check the validity of the packet
    if (U2STAbits.FERR == 1)            // frame error aggiunto
    {
        U2_RX_IRQ_DISABLE;
        U2MODEbits.UARTEN = 0;          // disablito seriale
        U2STAbits.OERR = 0;             //resetto gli errori
        ERR_FLAGS.MEM = 1;              // metto alto errore di frame    
        U2MODEbits.UARTEN = 1;          //abilit seriale
    }
    
    if (U2STAbits.OERR == 1)            // overflow
    {
        U2_RX_IRQ_DISABLE;
        U2STAbits.OERR = 0;             //resetto gli errori
        ERR_FLAGS.MEM = 1;              // metto alto errore di frame    
        U2MODEbits.UARTEN = 1;
    }     
        unsigned int CRC, i;

    U2_RX_IRQ_CLEAR;                                // clear IF
    U2_RX_IRQ_DISABLE;                              // aggiunta per ridurre l'accesso agli interrupt durante l'elaborazione
    // check timeout and buffer level
    if(!RX_timeout && b_index)
    {   // b_index != 0 -> a reception is in progress
        // RX_timeout = 0 -> reception time elapsed
        // clear buffers and discard old packet for timeout
        if(!RX_timeout)
            ERR_FLAGS.TOT = 1;                      // timeout -> signal error

        b_index = 0;
        RX_CRC = 0;
        RX_length = 0;
        RX_command = 0;
    }
    
    RX_buffer[b_index] = U2RXREG;                   // read data

    switch(b_index)
    {
        case 0:     // check the START char
            if(RX_buffer[b_index] == P_STX)
            {   // is a start byte
                b_index++;                          // first char is a valid START
                RX_length = 0;                      // reset command and length buffers
                RX_command = 0;
                RX_timeout = _200ms;                // charge reception timeout
            }
            else if(RX_buffer[b_index] == P_ENQ)
            {   // is a single byte command (ENQ) - used to output answer from slave
                RX_command = P_ENQ;                 // force command and
                FLAGS.RX = ENABLE;                  // signal a valid frame
                FLAGS.ROK = ENABLE;
            }
            break;
        case 1:     // build number of bytes in the frame
        case 2:
        case 3:
            if(RX_buffer[b_index] & 0x40)
            {   // is a char (A ... F)
                RX_length = (RX_length << 4) + (RX_buffer[b_index] - 0x37);
            }
            else
            {   // is a number (0 ... 9)
                RX_length = (RX_length << 4) + (RX_buffer[b_index] - 0x30);
            }
            b_index++;
            break;
        case 4:     // build the command
        case 5:
            RX_command = (RX_command << 8) | RX_buffer[b_index];
            b_index++;
            break;
        default:    // from here the size is variable -> threat each char as a data
            // exit when the RX_length is reached
            if(b_index == (RX_length + 6))
            {   // check the STOP char -----------------------------------------
                if(RX_buffer[b_index] == P_ETX)
                {   // build received CRC --------------------------------------
                    RX_CRC = 0xFF00 & (RX_buffer[b_index - 2] << 8);
                    RX_CRC = RX_CRC | (0x00FF & (RX_buffer[b_index - 1]));
                    // calculate packet CRC
                    i = RX_length + 3;
                    CRC = 0;
                    while(i)
                    {
                        CRC = (unsigned int)CRC + (unsigned char)RX_buffer[i];
                        i--;
                    }
                    CRC = CRC ^ 0xFFFF;

                    // check CRC -----------------------------------------------
                    if(CRC == RX_CRC)
                    {   // CRC OK -> packet valid
                        FLAGS.RX = ENABLE;          // signal a valid frame
                        FLAGS.ROK = ENABLE;
                    }
                    else
                    {   // CRC not OK
                        FLAGS.RX = ENABLE;          // signal a non valid frame
                        FLAGS.ROK = DISABLE;
                    }

                    // reset counters for next reception -----------------------
                    b_index = 0;
                    RX_CRC = 0;
                }
            }
            else
            {   // increment RX_buffer[] pointer
                b_index++;
            }
            break;
    }
    U2_RX_IRQ_ENABLE;
}

// Process current packet ------------------------------------------------------
// the answer to the command from the host is built into the TX_buffer[], the 
// length of the packet is set to DMA2CNT and UART is enabled. This trigger
// the DMA2 channel and the data are automatically sent via UART2.
// At the end of transmission, the invoked IRQ routine stops DMA transfer
int process_packet(void)
{
    static int STATE_FUNC = 0;
    unsigned int CRC, i, data;
    int v_value, i_value;
    extern long Vsum, Isum;
   extern unsigned long   timer_cycle;        // length of cycle in [ms]
    
    switch(STATE_FUNC)
    {
        case 0:
            if(FLAGS.ROK)
            {   // packet received is formally correct
                switch(RX_command)
                {
                    case P_ENQ:
                        if(FLAGS.ENQ)
                        {   // send secondary data buffer as reply to previous command
                            FLAGS.ENQ = 0;

                            // copy data buffer and msg length
                            TX_length = TX_sec_length;
                            for(i = 0; i < 48; i++)
                                TX_buffer[i] = TX_sec_buffer[i];

                            STATE_FUNC = BTX_message;   // build and TX message
                        }
                        else
                        {   // send simple ACK
                            STATE_FUNC = TX_message;
                            TX_buffer[0] = P_ACK;
                            DMA2CNT = 0;            // singled byte answer
                        }
                        break;
                    case P_SETHWGENERAL:            // general configuration
                        if((RX_buffer[6] & 0x3F) || (RX_buffer[5] & 0xFF))
                        {   // data is consistent
                            TX_buffer[0] = P_ACK;

                            HW_CFG.LSB = RX_buffer[7];   // update configuration
                            HW_CFG.MSB = RX_buffer[6];
                            FLAGS.UHW = 1;              // update request
                        }
                        else
                        {
                            TX_buffer[4] = RX_buffer[4];    // copy current command
                            TX_buffer[5] = RX_buffer[5];

                            TX_buffer[0] = P_NACK;
                        }

                        STATE_FUNC = TX_message;
                        DMA2CNT = 0;                // singled byte answer
                        break;
                    case P_SETSUPPLYOUT:
                        if(HW_CFG.PEN)
                        {   // driver is on -> enable waveform
                            if(FLAGS.CYC)
                            {   // in cycle based mode
                                if(EXP_N_CYCLES == N_CYCLES)
                                {   // enable out only if cycles structure is complete
                                    FLAGS.WV = ENABLE;
                                    FLAGS.ZC = ENABLE;
                                }
                            }
                            else
                            {   // enable out starting form default frequency and amplitude
                                FLAGS.WV = ENABLE;
                                FLAGS.ZC = ENABLE;
                                DDS.PHA = 0;
                                DDS.M = _f50Hz;
                                scale = _min_scale;
                            }

                            TX_buffer[0] = P_ACK;
                           // FLAGS.UHW = 1;              // update request
                        }
                        else
                        {   // driver is not enable -> reply with nack
                            TX_buffer[0] = P_NACK;
                        }

                        STATE_FUNC = TX_message;
                        DMA2CNT = 0;            // singled byte answer
                        break;
                    case P_RESETSUPPLYOUT:          // fast output reset
                        if(!RX_buffer[6])           // se è a zero il bit inviato resetta
                        {   // reset power driver
                            DRV_SHUTDOWN = 1;
                            FLAGS.WV_D = 1;         // stop waveform
                            ERR_FLAGS.SDN = 1;      // signal fast shutdown request
                            HW_CFG.PEN = 0;         // signal power section disabled
                            TX_buffer[0] = P_ACK;
                        }
                        else
                        {
                             if(HW_CFG.PEN)
                            {   // driver is on -> enable waveform
                                if(FLAGS.CYC)
                                {   // in cycle based mode
                                    if(EXP_N_CYCLES == N_CYCLES)
                                    {   // enable out only if cycles structure is complete
                                        FLAGS.WV = ENABLE;
                                        FLAGS.ZC = ENABLE;
                                        
                                    }
                                }
                                else
                                {   // enable out starting form default frequency and amplitude
                                    FLAGS.WV = ENABLE;
                                    FLAGS.ZC = ENABLE;
                                    DDS.PHA = 0;
                                    DDS.M = _f50Hz;
                                    scale = _min_scale;
                                }

                                TX_buffer[0] = P_ACK;
                             } 
                            else
                            {   // driver is not enable -> reply with nack
                                TX_buffer[0] = P_NACK;
                            }
                        }
                        STATE_FUNC = TX_message;                     
                        DMA2CNT = 0;                // singled byte answer
                        break;
                    case P_RESETOFFSET:             // launch offset calibration
                        STATE_MACHINE = OFF_CAL;
                        STATE_FUNC = TX_message;
                        TX_buffer[0] = P_ACK;
                        DMA2CNT = 0;                // singled byte answer
                        break;
                    case P_GETHWCONF:               // get HW configuration
                        // prepare immediate answer to command (ACK)
                        STATE_FUNC = TX_message;
                        TX_buffer[0] = P_ACK;
                        DMA2CNT = 0;                // singled byte answer

                        // prepare answer to the next ENQ command
                        TX_sec_length = 20;                 // packet length (data + command)
                        FLAGS.ENQ = 1;                      // signal answer is ready

                        // command
                        TX_sec_buffer[4] = RX_buffer[4];    // copy current command
                        TX_sec_buffer[5] = RX_buffer[5];    // this is possible 'cause the buffer is not
                        // yet changed

                        // data
                        TX_sec_buffer[6] = man[2];          // manufacturer
                        TX_sec_buffer[7] = man[1];
                        TX_sec_buffer[8] = man[0];

                        TX_sec_buffer[9] = serial_n[4];     // serial number
                        TX_sec_buffer[10] = serial_n[3];
                        TX_sec_buffer[11] = serial_n[2];
                        TX_sec_buffer[12] = serial_n[1];
                        TX_sec_buffer[13] = serial_n[0];

                        TX_sec_buffer[14] = HW_VERa;        // HW version
                        TX_sec_buffer[15] = HW_VERb;

                        TX_sec_buffer[16] = FW_VERa;        // FW version
                        TX_sec_buffer[17] = FW_VERb;

                        TX_sec_buffer[18] = HW_CFG.MSB;     // HW configuration
                        TX_sec_buffer[19] = HW_CFG.LSB;

                        TX_sec_buffer[20] = 0;              // spare
                        TX_sec_buffer[21] = 0;
                        TX_sec_buffer[22] = 0;
                        TX_sec_buffer[23] = 0;
                        break;
                    case P_GETTEMPER:               // return temperatures
                        // prepare immediate answer to command (ACK)
                        STATE_FUNC = TX_message;
                        TX_buffer[0] = P_ACK;
                        DMA2CNT = 0;                // singled byte answer

                        // prepare answer to the next ENQ command
                        TX_sec_length = 6;                  // packet length (data + command)
                        FLAGS.ENQ = 1;                      // signal answer is ready

                        // command
                        TX_sec_buffer[4] = RX_buffer[4];    // copy current command
                        TX_sec_buffer[5] = RX_buffer[5];    // this is possible 'cause the buffer is not
                        // yet changed

                        // data
                        TX_sec_buffer[6] = (char)(T.IGBT);  // 4 byte for temperatures
                        TX_sec_buffer[7] = (char)(T.BOARD);
                        TX_sec_buffer[8] = (char)(T.TRAFO);
                        TX_sec_buffer[9] = 0;
                        break;
                    case P_GETDIAGNOSTIC:           // return board diagnostic
                        // prepare immediate answer to command (ACK)
                        STATE_FUNC = TX_message;
                        TX_buffer[0] = P_ACK;
                        DMA2CNT = 0;                // singled byte answer

                        // prepare answer to the next ENQ command
                        TX_sec_length = 20;                 // packet length (data + command)  
                        FLAGS.ENQ = 1;                      // signal answer is ready

                        // command
                        TX_sec_buffer[4] = RX_buffer[4];    // copy current command
                        TX_sec_buffer[5] = RX_buffer[5];    // this is possible 'cause the buffer is not
                        // yet changed

                        // data
                        TX_sec_buffer[6] = (char)(VlineV >> 8);  //modifcato adesso esporta valore non massimo ma attuale (char)(max_Vline >> 8)
                        TX_sec_buffer[7] = (char)(VlineV);       // (char)(maxVline);  
                        TX_sec_buffer[8] = (char)(min_Vline >> 8);
                        TX_sec_buffer[9] = (char)(min_Vline);
                        TX_sec_buffer[10] = (char)(I_OUT >> 8);
                        TX_sec_buffer[11] = (char)(I_OUT);
                        TX_sec_buffer[12] = (char)(DEEP_SUPPLY >> 8);
                        TX_sec_buffer[13] = (char)(DEEP_SUPPLY);
                        TX_sec_buffer[14] = 0;
                        TX_sec_buffer[15] = 0;
                        TX_sec_buffer[16] = (char)(min_counter >> 24);
                        TX_sec_buffer[17] = (char)(min_counter >> 16);
                        TX_sec_buffer[18] = (char)(min_counter >> 8);
                        TX_sec_buffer[19] = (char)(min_counter);
                        TX_sec_buffer[20] = (char)(min_on_counter >> 24);
                        TX_sec_buffer[21] = (char)(min_on_counter >> 16);
                        TX_sec_buffer[22] = (char)(min_on_counter >> 8);
                        TX_sec_buffer[23] = (char)(min_on_counter);
                        break;
                    case P_RESETCOUNTERS:           // reset minutes counters
                        min_on_counter = 0;
                        min_counter = 0;
                        FLAGS.SCM = 1;              // save request

                        STATE_FUNC = TX_message;
                        TX_buffer[0] = P_ACK;
                        DMA2CNT = 0;                // singled byte answer
                        break;
                    case P_RESETDIAGNOSTIC:         // reset power supply monitors
                        max_Vline = VlineV;
                        min_Vline = VlineV;
                        I_OUT = 0;
                        DEEP_SUPPLY = 0;

                        STATE_FUNC = TX_message;
                        TX_buffer[0] = P_ACK;
                        DMA2CNT = 0;                // singled byte answer
                        break;
                    case P_SETVLIMIT:               // set voltage threshold for overvoltage assertion
                        i = (RX_buffer[6] << 8) + RX_buffer[7];         // build data
                        // scale limit to raw value
                        limit.V = (__builtin_mulss((int)i, (int)_kv_raw)) >> 4;
#ifdef _zero
                        limit.nV = 50;
#else
                        limit.nV = limit.V - (limit.V >> 5);            // add 3% hysteresis
#endif
                        if(limit.V > max_V_limit)
                            limit.VS = max_VS;                          // shutdown limit = max
                        else
                            limit.VS = limit.V + (limit.V >> 1);        // shutdown limit = limit * 1.5

                        STATE_FUNC = TX_message;
                        TX_buffer[0] = P_ACK;
                        DMA2CNT = 0;                // singled byte answer
                        break;
                    case P_SETILIMIT:               // set current threshold for overcurrent assertion
                        i = (RX_buffer[6] << 8) + RX_buffer[7];         // build data
                        // scale limit to raw value
                        limit.I = (__builtin_mulss((int)i, (int)_ki_raw)) >> 4;
#ifdef _zero
                        limit.nI = 50;
#else
                        limit.nI = limit.I - (limit.I >> 5);            // add 3% hysteresis
#endif
                        if(limit.I > max_I_limit)
                            limit.IS = max_IS;                          // shutdown limit = max
                        else
                            limit.IS = limit.I + (limit.I >> 1);        // shutdown limit = limit * 1.5

                        STATE_FUNC = TX_message;
                        TX_buffer[0] = P_ACK;
                        DMA2CNT = 0;                // singled byte answer
                        break;
                    case P_RESETERR:                // reset selected error flags
                        // note:    RX buffer bits to 1 mean that these bits should be reset
                        ERR_FLAGS.B0 = ERR_FLAGS.B0 & (~RX_buffer[6]);
                        ERR_FLAGS.B1 = ERR_FLAGS.B1 & (~RX_buffer[7]);
                        ERR_FLAGS.B2 = ERR_FLAGS.B2 & (~RX_buffer[8]);
                        ERR_FLAGS.B3 = ERR_FLAGS.B3 & (~RX_buffer[9]);

                        STATE_FUNC = TX_message;
                        TX_buffer[0] = P_ACK;
                        DMA2CNT = 0;                // singled byte answer
                        break;
                    case P_DISABLEERR:              // select bits to be disabled
                        // note:    RX buffer bits to 1 mean that these bits should be masked
                        ERR_FLAGS_MASK.B0 = ERR_FLAGS_MASK.B0 & (~RX_buffer[6]);
                        ERR_FLAGS_MASK.B1 = ERR_FLAGS_MASK.B1 & (~RX_buffer[7]);
                        ERR_FLAGS_MASK.B2 = ERR_FLAGS_MASK.B2 & (~RX_buffer[8]);
                        ERR_FLAGS_MASK.B3 = ERR_FLAGS_MASK.B3 & (~RX_buffer[9]);

                        STATE_FUNC = TX_message;
                        TX_buffer[0] = P_ACK;
                        DMA2CNT = 0;                // singled byte answer
                        break;
                    case P_ENABLEERR:               // select bits to be enabled
                        // note:    RX buffer bits to 1 mean that these bits should be enabled
                        ERR_FLAGS_MASK.B0 = ERR_FLAGS_MASK.B0 | RX_buffer[6];
                        ERR_FLAGS_MASK.B1 = ERR_FLAGS_MASK.B1 | RX_buffer[7];
                        ERR_FLAGS_MASK.B2 = ERR_FLAGS_MASK.B2 | RX_buffer[8];
                        ERR_FLAGS_MASK.B3 = ERR_FLAGS_MASK.B3 | RX_buffer[9];

                        STATE_FUNC = TX_message;
                        TX_buffer[0] = P_ACK;
                        DMA2CNT = 0;                // singled byte answer
                        break;
                    case P_GETERRLIST:              // return board diagnostic
                        // prepare immediate answer to command (ACK)
                        STATE_FUNC = TX_message;
                        TX_buffer[0] = P_ACK;
                        DMA2CNT = 0;                // singled byte answer

                        // prepare answer to the next ENQ command
                        TX_sec_length = 10;                 // packet length (data + command)
                        FLAGS.ENQ = 1;                      // signal answer is ready

                        // command
                        TX_sec_buffer[4] = RX_buffer[4];    // copy current command
                        TX_sec_buffer[5] = RX_buffer[5];    // this is possible 'cause the buffer is not
                        // yet changed

                        // data
                        // out only errors not masked
                        TX_sec_buffer[6] = ERR_FLAGS_MASK.B0 & ERR_FLAGS.B0;
                        TX_sec_buffer[7] = ERR_FLAGS_MASK.B1 & ERR_FLAGS.B1;
                        TX_sec_buffer[8] = ERR_FLAGS_MASK.B2 & ERR_FLAGS.B2;
                        TX_sec_buffer[9] = ERR_FLAGS_MASK.B3 & ERR_FLAGS.B3;
                        TX_sec_buffer[10] = 0x00;
                        TX_sec_buffer[11] = 0x00;
                        TX_sec_buffer[12] = 0x00;
                        TX_sec_buffer[13] = 0x00;
                        break;

                        // CYCLE BASED WAVEFORM GENERATION ------------------------- 
                    case P_NEWTEST:                 // initiate a new test sequence
                        EXP_N_CYCLES = RX_buffer[6];        // this is the number of cycles
                        // to be expected
                       // FLAGS.CYC = ENABLE;         // signal cycle-based DDS generation

                        //memset(&CYCLES, 0, sizeof(CYCLES));
                        for(i = 0; i < 15; i++)
                        {
                            CYCLES[i].n = cycle_free;   // reset index -> will skip cycle execution*/
                        }
                         N_CYCLES = 0;               // reset actual number of stored cycles

                        STATE_FUNC = TX_message;
                        TX_buffer[0] = P_ACK;
                        DMA2CNT = 0;                // singled byte answer
                        break;
                    case P_RESETCY:                 // reset current cycle structure
                        for(i = 0; i < 15; i++)
                        {
                            CYCLES[i].n = cycle_free;   // reset index -> will skip cycle execution
                        }

                        N_CYCLES = 0;               // reset actual number of stored cycles
                        EXP_N_CYCLES = 0;
                        //FLAGS.CYC = DISABLE;        // return to direct DDS generation or data streaming 

                        STATE_FUNC = TX_message;
                        TX_buffer[0] = P_ACK;
                        DMA2CNT = 0;                // singled byte answer
                        break;
                    case P_GETCY:                   // send back cycle structure n
                        // prepare immediate answer to command (ACK)
                        STATE_FUNC = TX_message;
                        TX_buffer[0] = P_ACK;
                        DMA2CNT = 0;                // singled byte answer

                        // prepare answer to the next ENQ command
                        TX_sec_length = 20;                 // packet length (data + command)
                        FLAGS.ENQ = 1;                      // signal answer is ready

                        // command
                        TX_sec_buffer[4] = RX_buffer[4];    // copy current command
                        TX_sec_buffer[5] = RX_buffer[5];    // this is possible 'cause the buffer is not
                        // yet changed

                        i = (RX_buffer[6] << 8) + RX_buffer[7];     // extract n cycle required

                        TX_sec_buffer[6] = (char)(CYCLES[i].n >> 8);
                        TX_sec_buffer[7] = (char)CYCLES[i].n;

                        TX_sec_buffer[8] = (char)(CYCLES[i].quality_word >> 8);
                        TX_sec_buffer[9] = (char)CYCLES[i].quality_word;

                        TX_sec_buffer[10] = (char)(CYCLES[i].f_div >> 8);
                        TX_sec_buffer[11] = (char)CYCLES[i].f_div;

                        TX_sec_buffer[12] = 0;
                        TX_sec_buffer[13] = 0;

                        TX_sec_buffer[14] = (char)(CYCLES[i].amplitude >> 24);
                        TX_sec_buffer[15] = (char)(CYCLES[i].amplitude >> 16);
                        TX_sec_buffer[16] = (char)(CYCLES[i].amplitude >> 8);
                        TX_sec_buffer[17] = (char)CYCLES[i].amplitude;

                        TX_sec_buffer[18] = (char)(CYCLES[i].gradient >> 24);
                        TX_sec_buffer[19] = (char)(CYCLES[i].gradient >> 16);
                        TX_sec_buffer[20] = (char)(CYCLES[i].gradient >> 8);
                        TX_sec_buffer[21] = (char)CYCLES[i].gradient;

                        TX_sec_buffer[22] = (char)(CYCLES[i].time >> 24);
                        TX_sec_buffer[23] = (char)(CYCLES[i].time >> 16);
                        TX_sec_buffer[24] = (char)(CYCLES[i].time >> 8);
                        TX_sec_buffer[25] = (char)CYCLES[i].time;

                        break;
                    case P_SETENDCY:                // abort current cycle
                        FLAGS.WV_D = ENABLE;        // stop wave request
                        timer_cycle = 0;    //azzera il timer dei cicli, dopo aver dato il comando di setedncy si può riavviare il ciclo senza aspettare lo scadere del timer precedente.
                        STATE_FUNC = TX_message;
                        TX_buffer[0] = P_ACK;
                        DMA2CNT = 0;                // singled byte answer
                        break;
                      case P_GET_V_I_SUPPLY:           // STAM nuovo comando
              
                     v_value = s_sqrt(Vsum);
                     v_value = (int)(__builtin_mulss((int)v_value, (int)kv) >> 15);
                     i_value = s_sqrt(Isum);
                     i_value = (int)(__builtin_mulss((int)i_value, (int)ki) >> 14);
                          // prepare immediate answer to command (ACK)
                          STATE_FUNC = TX_message;
                          TX_buffer[0] = P_ACK;
                          DMA2CNT = 0;                // singled byte answer

                          // prepare answer to the next ENQ command
                          TX_sec_length = 18;                 // packet length (data + command)
                          FLAGS.ENQ = 1;                      // signal answer is ready

                          // command
                          TX_sec_buffer[4] = RX_buffer[4];    // copy current command
                          TX_sec_buffer[5] = RX_buffer[5];    // this is possible 'cause the buffer is not
                          // yet changed

                          // data
                          TX_sec_buffer[6] = (char)(VlineV >> 8);     // [1]  tensione del ponte [1]
                          TX_sec_buffer[7] = (char)(VlineV);            // [2]  tensione del ponte [2]
                          TX_sec_buffer[8] = (char)(0);                   //[3] tensione di uscita fissata a 121 (valore provvisorio chiesto da ISA) 
                          TX_sec_buffer[9] = (char)(121);               //[4] tensione di uscita fissata a 121 (valore provvisorio chiesto da ISA)       
                          TX_sec_buffer[10] = (char)(0);                   //[5] dummy
                          TX_sec_buffer[11] = (char)(200);       //       [6] dummy
                          TX_sec_buffer[12] = (char)(v_value >>8);    //   [4] 
                          TX_sec_buffer[13] = (char)(v_value );
                          TX_sec_buffer[14] = (char)(0);
                          TX_sec_buffer[15] = (char)(0);
                          TX_sec_buffer[16] = (char)(i_value >>8 );
                          TX_sec_buffer[17] = (char)(i_value);
                          TX_sec_buffer[18] = 0;
                          TX_sec_buffer[19] = 0;
                          TX_sec_buffer[20] = (char)(0);
                          TX_sec_buffer[21] = (char)(101);
                          break;
                          
                    case P_BOOTENTRY: //comando di upgrade fw
                        /*while the boot pin becomes high*/
                        while(!BOOT_ENTER);
                        
                          asm ("RESET");
                        break;
                        
                    case P_SETCYCH:                 // set cycle n structure
                        // current cycle
                        i = (RX_buffer[6] << 8) + RX_buffer[7];     // extract n cycle

                        if(CYCLES[i].n == cycle_free)
                            N_CYCLES++;                             // increment only if cycle is free, to avoid double increment
                        // when modify a single cycle
                        CYCLES[i].n = i;                            // set CYCLE[n]

                        CYCLES[i].quality.dur_type = 0x07 & (RX_buffer[8] >> 5);
                        CYCLES[i].quality.harmonic = 0x0F & (RX_buffer[8] >> 2);
                        CYCLES[i].quality.waveform = 0x03 & RX_buffer[8];
                        CYCLES[i].quality.type = 0x1F & RX_buffer[9];

                        // frequency divider: f_out = 40000 / f_div
                        // expected f_div = [100, 2667] -> f_out = [400, 15]Hz
                        CYCLES[i].f_div = (((unsigned int )RX_buffer[10]) << 8) + RX_buffer[11];

                        // amplitude in absolute raw value
                     
                        CYCLES[i].amplitude = (((unsigned long) RX_buffer[14]) << 24) + (((unsigned long) RX_buffer[15]) << 16);
                        CYCLES[i].amplitude = CYCLES[i].amplitude + (((unsigned long)RX_buffer[16]) << 8) + RX_buffer[17];
                        
                       
                        // gradient field in pts/s or mHz/s
                        // expected value: [1000, 32767]pts/s, [1000, 385000]mHz/s
                        CYCLES[i].gradient = (((unsigned long) RX_buffer[18]) << 24) + (((unsigned long) RX_buffer[19]) << 16);
                        CYCLES[i].gradient = CYCLES[i].gradient + (((unsigned long)RX_buffer[20]) << 8) + RX_buffer[21];

                        // time length field in ms
                        // expected value: [1, 2^32 - 1]ms
                        CYCLES[i].time = (((unsigned long) RX_buffer[22]) << 24) + (((unsigned long) RX_buffer[23]) << 16);
                        CYCLES[i].time = CYCLES[i].time + (((unsigned long)RX_buffer[24]) << 8) + RX_buffer[25];

                        // now compute M_val, the phase increment value
                        // M_val = 33333333 / f_div
                        // fout = 1.2mHz x M_val
                        // when f_div = 100 -> fout = 400Hz
                        // when f_div = 2667 -> fout = 15Hz
                        // to avoid divisions with 32bit result do the following:
                        // data = 6 x (33333333 / 6) / freq
                        data = (int) __builtin_divud((long) 5555555, (int) CYCLES[i].f_div);
                        CYCLES[i].M_val = (long) __builtin_mulss((int) data, (int) 6);

                        // compute freq sweep value if required...
                        if(CYCLES[i].quality.type == freq_gradient)
                        {
                            // M_sweep[n/ms] = gradient[mHz/s] / 1.2mHz / 1000 = gradient / 1024 x 0.853
                            // To avoid long x int multiplication, two int x int multiplication are done instead:
                            // M_sweep = ((gradient[MSWord] >> 10 x 0.853) << 16) + (gradient[LSWord] >> 10 x 0.853)
                            // M_sweep is the number to add each ms to DDS phase accumulator
                            data = (int)(CYCLES[i].gradient >> 26);     // take MSW / 1024
                            CYCLES[i].M_sweep = (long)(__builtin_mulss((int)0x6D2E, (int)data) << 1);   // (MSW / 1024 x 0.853) << 16
                            data = (int)(CYCLES[i].gradient >> 10);     // take LSW / 1024
                            CYCLES[i].M_sweep = CYCLES[i].M_sweep + (long)(__builtin_mulss((int)0x6D2E, (int)data) >> 15); // (LSW / 1024 x 0.853)

                            CYCLES[i].amp_sweep = 0;
                             CYCLES[i].amp_sweep_float = 0;
                        }
                        else if(CYCLES[i].quality.type == ampl_gradient)
                        {   // ... or amplitude sweep vale

                            // amp_sweep[pts/ms] = gradient[pts/s] / 1000 = gradient / 512 x 0.512
                            // To avoid long x int multiplication, two int x int multiplication are done instead:
                            // amp_sweep = ((gradient[MSWord] >> 9 x 0.512) << 16) + (gradient[LSWord] >> 9 x 0.512)
                            // amp_sweep is the number to add each ms to amplitude control
                           
                           /* data = (int)(CYCLES[i].gradient >> 25);     // take MSW / 512
                            CYCLES[i].amp_sweep = (long)(__builtin_mulss((int)0x4188, (int)data) << 1); // (MSW / 512 x 0.512) << 16
                            data = (int)(CYCLES[i].gradient >> 9);      // take LSW / 512
                            CYCLES[i].amp_sweep = CYCLES[i].amp_sweep + (long)(__builtin_mulss((int)0x4188, (int)data) >> 15);  // (LSW / 512 x 0.512)*/
                            //CYCLES[i].amp_sweep =   (int) __builtin_divud( CYCLES[i].gradient,  10000 ); //ms
                            if  (FLAGS.I_L){
                                CYCLES[i].amp_sweep_float =   CYCLES[i].gradient / CYC_CONVERSION_CURRENT_IRMS;    
                                CYCLES[i].amp_sweep_float =   CYCLES[i].amp_sweep_float / 1000;  
                                CYCLES[i].amp_sweep_float =   CYCLES[i].amp_sweep_float / 0.8;
                            }else{
                                CYCLES[i].amp_sweep_float =   CYCLES[i].gradient / CYC_CONVERSION_VOLTAGE_VRMS;    
                                CYCLES[i].amp_sweep_float =   CYCLES[i].amp_sweep_float / 1000;  
                                CYCLES[i].amp_sweep_float =   CYCLES[i].amp_sweep_float / 6.5;
                            }
                            
                            CYCLES[i].M_sweep = 0;
                        }
                        else
                        {   // clear amp and freq sweep fields
                            CYCLES[i].amp_sweep = 0;
                            CYCLES[i].M_sweep = 0;
                             CYCLES[i].amp_sweep_float = 0;
                        }

                        STATE_FUNC = TX_message;
                        TX_buffer[0] = P_ACK;
                        DMA2CNT = 0;                // singled byte answer
                        break;
                }
            }
            else
            {   // packet is not correct
                STATE_FUNC = TX_message;                // jump to message build and send
                TX_buffer[6] = P_NACK;
                DMA2CNT = 0;                            // singled byte answer
            }
            break;
           
        case BTX_message:   // build answer: data fields are already filled
            TX_buffer[0] = P_STX;                       // START char

            TX_buffer[1] = (char)(TX_length >> 8);      // fill packet length
            TX_buffer[2] = (char)(TX_length >> 4);
            TX_buffer[3] = (char) (TX_length & 0xF);

            // convert into ASCII char
            for(i = 1; i < 4; i++)
            {
                if(TX_buffer[i] > 0x09)
                    TX_buffer[i] = TX_buffer[i] + 0x37; // is a char (A ... F)
                else
                    TX_buffer[i] = TX_buffer[i] + 0x30; // is a number (0 ... 9)
            }

            // fill CRC
            i = TX_length + 3;
            CRC = 0;
            while(i)
            {
                CRC = (unsigned int)CRC + (unsigned char)TX_buffer[i];
                i--;
            }
            CRC = CRC ^ 0xFFFF;
            TX_buffer[TX_length + 5] = (char)CRC;
            TX_buffer[TX_length + 4] = (char)(CRC >> 8);

            TX_buffer[TX_length + 6] = P_ETX;

            DMA2CNT = TX_length + 6;                // load DMA counter for formatted messages

            STATE_FUNC++;
            break;
        case TX_message:    // just TX message
            U2STAbits.UTXEN = 0;                    // disable UART TX (this is needed in order to
            // trigger new transmission)
            U2_TX_IRQ_CLEAR;

            DMA2CONbits.CHEN = 1;                   // enable DMA: DMA transfer will be
            // automatically triggered by peripheral
            U2STAbits.UTXEN = 1;                    // enable TX
           
            STATE_FUNC = 0;                         // exit
            break;
    }

    return STATE_FUNC;
}
