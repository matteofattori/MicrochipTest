/*
 * File:   main.c
 * Author: marco.landoni
 *
 * Created on 15 giugno 2017, 12.46
 */


// DSPIC33EP256MC204 Configuration Bit Settings

// 'C' source line config statements

// FICD
#pragma config ICS = PGD2               // ICD Communication Channel Select bits (Communicate on PGEC2 and PGED2)
#pragma config JTAGEN = OFF             // JTAG Enable bit (JTAG is disabled)

// FPOR
#pragma config ALTI2C1 = OFF             // Alternate I2C1 pins (I2C1 mapped to ASDA1/ASCL1 pins)
#pragma config ALTI2C2 = OFF            // Alternate I2C2 pins (I2C2 mapped to SDA2/SCL2 pins)
#pragma config WDTWIN = WIN75           // Watchdog Window Select bits (WDT Window is 25% of WDT period)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler bits (1:32,768)
#pragma config WDTPRE = PR128           // Watchdog Timer Prescaler bit (1:128)
#pragma config PLLKEN = ON              // PLL Lock Enable bit (Clock switch to PLL source will wait until the PLL lock signal is valid.)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable bit (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable bit (Watchdog timer enabled/disabled by user software)

// FOSC
#pragma config POSCMD = HS              // Primary Oscillator Mode Select bits (HS Crystal Oscillator Mode)
#pragma config OSCIOFNC = ON            // OSC2 Pin Function bit (OSC2 is general purpose digital I/O pin)
#pragma config IOL1WAY = OFF            // Peripheral pin select configuration (Allow multiple reconfigurations)
#pragma config FCKSM = CSECME           // Clock Switching Mode bits (Both Clock switching and Fail-safe Clock Monitor are enabled)

// FOSCSEL
#pragma config FNOSC = FRC              // Oscillator Source Selection (Internal Fast RC (FRC))
#pragma config PWMLOCK = OFF            // PWM Lock Enable bit (PWM registers may be written without key sequence)
#pragma config IESO = ON                // Two-speed Oscillator Start-up Enable bit (Start up device with FRC, then switch to user-selected oscillator source)

// FGS
#pragma config GWRP = OFF               // General Segment Write-Protect bit (General Segment may be written)
#pragma config GCP = OFF                // General Segment Code-Protect bit (General Segment Code protect is Disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

#include "main.h"
#include "system.h"
#include "consolle.h"
#include "I2C_driver.h"
#include "control.h"
#include "libpic30.h"
#include <p33EP256MC204.h>

// VARIABLES -------------------------------------------------------------------
char man[4];                            // manufacturer name (stored in FRAM)
char serial_n[6];                       // serial number (stored in FRAM)
                                        // HW ver and FW ver are stored as #define
    
union FLAGS_type  FLAGS;                // FLAGS structure
union ERR_FLAGS_type ERR_FLAGS;         // ERROR flags
union ERR_FLAGS_type ERR_FLAGS_MASK;    // ERROR flags mask, used to enable/disable errors on communication bus

long Vsum, Isum;                        // partial rms value based on last 2048 samples (~80ms)
                                        // Xrms = 2047 * Xsum + X^2
int Vline;                              // line voltage (alimentazione ponte), raw number used for compensation
int VlineV;                             // line voltage (alimentazione ponte) in Volts
int max_Vline = 0;                      // max Vline ,var che memoriazza il picco positivo della tensione di alimentaz
int min_Vline = 300;                    // min Vline,var che memorizza il picco negativo della tensione di alimentazione
int ov_mon = 0;                         // overcurrent and overvoltage monostable counters
int oi_mon = 0;

int max_v = 0;                          // instantaneous max output values (raw data)
int max_i = 0;
int abs_max_i = 0;                      // max current registered (raw data)
int I_OUT = 0;                          // max current represented in A/10 (ex. 26 = 2.6A)
unsigned int DEEP_SUPPLY = 0;           // supply interruption counter (ms)

unsigned char   STATE_MACHINE = LOAD_PAR;   // initial machine state
struct CFG_type HW_CFG;                 // HW configuration formatted as protocols wants

unsigned int timer_ov = _500ms;         // used for over-range signalling

// Temperatures are in format K with -200K offset
// 0°C -> 273K -> 73
// 25°C -> 298K -> 98
struct  {   int     IGBT;               // IGBT temperature XWB lettura fatta con pnp 
            int     BOARD;              // board temperature
            int     TRAFO;              // trafo temperature XWB lettura fatta con chiamata i2c e adc a 14bit oppure dspic
        }T;

unsigned long min_counter = 0;          // minute power ON counter
unsigned long min_on_counter = 0;       // minute generation ON counter

// Generation cycles structures
cycle_str_type CYCLES[15];              // cycles data structure

unsigned char set_PWM;

// MAIN PROGRAM STARTS HERE ----------------------------------------------------
int main(void)
{
    // VARIABLES ---------------------------------------------------------------=    ERR_FLAGS_MASK.ALL = 0xFFFFFFFF;    // enable all errors as default
    
    // MAIN --------------------------------------------------------------------
    set_PWM=0;
    cfg_PWM(set_PWM);                    // configure uC and peripherals
    cfg_system();
    FLAGS.O_L = ENABLE;                 // set open loop mode as default
    HW_CFG.OL = ENABLE;                 //perchè due settaggi?
     
   while(1)
    {
        // STATE MACHINE
        switch(STATE_MACHINE)
        {
            case NORM:
                // start waveform generator ------------------------------------
                if(FLAGS.EN != FLAGS.ENL)
                {   // waveform on/off edge
                    if(FLAGS.EN && HW_CFG.PEN) //se è acceso e abilitato
                    {   // if driver is powered AND ON request
                        FLAGS.WV = ENABLE;      // enable waveform
                    }
                    else if(HW_CFG.PEN)
                    {   // driver powered AND OFF request
                        FLAGS.WV_D = ENABLE;    // disable waveform request
                    }
                }
                
                FLAGS.ENL = FLAGS.EN;           // update latch flag
                
                // overcurrent / overvoltage -----------------------------------
                // two levels: the lower (FLAGS.ORV or FLAGS.ORI are set), the upper
                // (ERR_FLAGS.SO is set)
                // When ERR_FLAGS.SO is set the waveform shuts down
                if(FLAGS.ORV)
                {   // overvoltage detected -> hold for 500ms
                    ov_mon = _500ms;
                    OV_OUT = ENABLE;        //accende il segnale per disabilitare
                }
                if(!ov_mon)
                    OV_OUT = DISABLE;
                
                if(FLAGS.ORI)
                {   // overcurrent detected -> hold for 500ms
                    oi_mon = _500ms;
                    OI_OUT = ENABLE;
                }
                if(!oi_mon)
                    OI_OUT = DISABLE;

                if(ERR_FLAGS.SO)
                {   // current/voltage is greatest than the upper limit:
                    // SHUTDOWN
                    FLAGS.ORI = DISABLE;
                    FLAGS.ORV = DISABLE;
                }
                
                // consolle message --------------------------------------------
                if(!timer_ov)
                {   // once every 500ms print a message
                    timer_ov = _500ms;
                    
                    if(ERR_FLAGS.SO)
                    {   // max overcurrent / overvoltage -> shutdown
                        send_text(" SHUTDOWN                             \r\n");
                        STATE_MACHINE = ERROR;
                    }
                    else if(ERR_FLAGS.DRV)
                    {   // fault on power driver -> shutdown
                        send_text(" POWER BRIDGE FAULT                   \r\n");
                        STATE_MACHINE = ERROR;
                    }
                    else if(ERR_FLAGS.MTI)
                    {   // maximum IGBT temperature reached -> stop waveform
                        send_text(" POWER BRIDGE OVERTEMPERATURE         \r\n");
                        FLAGS.WV_D = ENABLE;
                        STATE_MACHINE = ERROR;
                    }
                    else if(OV_OUT)
                    {   // overvoltage
                        send_text(" OVERVOLTAGE                          \r\n");
                    }
                    else if(OI_OUT)
                    {   // overcurrent
                        send_text(" OVERCURRENT                          \r\n");
                    }
                    else if(FLAGS.WV)
                    {   // print rms or peak values
                        send_val();
                    }
                }
                break;
            
            case LOAD_PAR:      // initialize parameters loading them from FRAM
                #ifdef _use_FRAM
                if(!load_parameters())
                {
                    STATE_MACHINE = BRD_DATA;
                }
                #else
                STATE_MACHINE = NORM;
                #endif
                
                 #ifdef _PID    
                cfg_PID();                          // configure PID controller
                #endif
    
                #ifdef _3P3Z
                cfg_3P3Z();                         // configure 3P3Z controller
                #endif

                #ifdef _dspPID
                cfg_dspPID();                       // configure dspPID controller
                #endif
                break;
                
            case OFF_CAL:       // calibrate offset
                if(!offset_cal())
                    STATE_MACHINE = NORM;
                break;
            
            case K_IN:          // acquire kp, ki, kd from consolle
#ifdef _PID
                if(!data_in())
                    STATE_MACHINE = NORM;
#endif
                break;
            
            case OR_IN:         // acquire overrange limits from consolle
                if(!or_data_in())
                    STATE_MACHINE = NORM;
                break;
                
            case ERROR:         // error state, stop to work until user operation
                if(!ERR_FLAGS.SO && !ERR_FLAGS.DRV && !ERR_FLAGS.MTI)
                    STATE_MACHINE = NORM;
                break;
                
            case SERIAL_SET:    // load and write to FRAM serial number
                if(!serial_set())
                    STATE_MACHINE = NORM;
                break;
                
            case MAN_SET:       // load and write to FRAM manufacturer code
                if(!man_set())
                    STATE_MACHINE = NORM;
                break;
            case BRD_DATA:      // print board data on consolle
                if(!send_brd_data())
                    STATE_MACHINE = NORM;
                break;
        }
        
        // Check Vline max and min values, salva i valori di picco alimenta------
        if(VlineV > max_Vline)
            max_Vline = VlineV;
       if(VlineV < min_Vline)
            min_Vline = VlineV;
        
        // Check if Vline > 320V ------------------------------------------------
        if(VlineV > Vline320)
            ERR_FLAGS.OV = 1;
        
        // Check if Vline < 30V -------------------------------------------------
        if(FLAGS.UV)
            ERR_FLAGS.UV = 1;
        
        // Check output voltage and current maximum values ----------------------
        
        if(FLAGS.SCALE) // se è abilitata la scala 18V per ottenere una lettura in volt devo dividere per il rapporto delle due scale 
        {
            if((max_v/Ratio180_18) > max_vout)
            ERR_FLAGS.MVO = 1;  //errore di over range
        }
        else // è abilitata la scala 180V
        {   
            if(max_v > max_vout)
            ERR_FLAGS.MVO = 1;  //errore di over range
        }
        
        if(max_i > max_iout)
            ERR_FLAGS.MIO = 1;  //errore di over current
        
        // Update max current value ---------------------------------------------
        if(max_i > abs_max_i)
        {   // update and convert to A/10
            abs_max_i = max_i;
            I_OUT = (int)(__builtin_mulss((int)max_i, (int)_ki_out) >> 14);
        }
        if(!ERR_FLAGS.TMP)
        {// Check temperatures limits --------------------------------------------
            if(T.BOARD > _max_TBOARD)
                ERR_FLAGS.MTB = 1;
            if(T.IGBT > _max_TIGBT)
                ERR_FLAGS.MTI = 1;
            if(T.TRAFO > _max_TTRAFO)
                ERR_FLAGS.MTT = 1;
        }
        // Manage I2C temperature sensor ----------------------------------------
#ifdef _use_TMP431
        if(!FLAGS.FRM)
        {   // FRAM doesn't hold I2C bus
            if(!ERR_FLAGS.TMP)
            {   // no problems with TMP431 I2C bus
                if(!TMP431_drv())
                    FLAGS.TMP = 0;              // I2C bus is not held by TMP431
                else
                    FLAGS.TMP = 1;              // I2C bus is busy
            }
        }
#endif
        
        // Update minutes counter -----------------------------------------------
#ifdef _use_FRAM
        if(!FLAGS.TMP)
        {   // TMP431 doesn't hold I2C bus
            if(FLAGS.SCM && !ERR_FLAGS.MEM)
            {   // minute counter updated -> save current count if no problem with I2C bus
                FLAGS.FRM = 1;              // I2C bus is busy
                if(!save_m_counters())
                {   // save done
                    FLAGS.FRM = 0;          // free I2C bus
                    FLAGS.SCM = 0;          // clear save request
                }
            }
        }
#endif
        
        // Control interface with DSP -------------------------------------------
        if(FLAGS.RX)
        {   // a packet has been received
            if(!process_packet())           // process current packet
                FLAGS.RX = DISABLE;
        }
        
        // HW configuration, change it when required ----------------------------
        if(FLAGS.UHW) // se è a unosignifica che è richiesto un aggiornamento dello stato
        {   // update HW configuration, when requested, from protocol
            if(HW_CFG.PEN) // è stata richiesta l'accensione     
            {
                PWM_FAULT_ENABLE;     
                DRV_SHUTDOWN = 0;//abilita l'accensione del ponte perchè sull'IR2110 lo SHWN è attivo alto
            }
            else
            {
                DRV_SHUTDOWN = 1;
            }
            
            if(!HW_CFG.SCALEI) // imposto la scala 25A
            {
            FLAGS.SCALEI = 0;
            }   
            
           
            if(HW_CFG.SCALEI) // imposto la scala 6A
            {
            FLAGS.SCALEI = 1;
            }    
            
            if(!HW_CFG.SCALE ) // imposto scala 180V sia in SPI e CICLI
            {   
                //init_ic2_100khz();
                VscaleEN(0);
                FLAGS.SCALE=0; //imposto la scala di tensione normale
                
            }    
            if(HW_CFG.SCALE) //imposto la scala 18V in SPI e CICLI 
            {  
                //init_ic2_100khz();
                VscaleEN(1);
                FLAGS.SCALE= 1; //imposto la scala più sensibile
            }
            
            
            if(HW_CFG.URT) // Impostazione CICLI   dalla specifica per avere il bitstream il bit 14 (HW_CFG.URT) deve essere "0"
            {   // disabilita la spi perchè il bit 14 è a uno significa che è funzionamento a cicli
                FLAGS.SPI = 0;
                FLAGS.CYC = 1; //XWB messo perchè non entrava nella modalità ciclo con il loop in corrente
                if(HW_CFG.OL)       // lo metto per prima perchè è meno prioritario
                {   // Open loop mode
                    FLAGS.V_L = 0;
                    FLAGS.I_L = 0;
                    FLAGS.O_L = 1;
                    FLAGS.DC = 0;                    
                }            
                if(HW_CFG.IAC || HW_CFG.IL) // current mode AC  con fondoscala 6AC
                {// Current mode
                    FLAGS.V_L = 0;
                    FLAGS.O_L = 0;
                    FLAGS.I_L = 1;
                    FLAGS.DC = 0;              
                }
                
                if(HW_CFG.VAC ) // voltage mode AC scala nornale, anche se è in spi mode
                {   // Voltage mode
                    FLAGS.O_L = 0;
                    FLAGS.I_L = 0;
                    FLAGS.V_L = 1;
                    FLAGS.DC = 0;
                    //init_ic2_100khz();
                    VscaleEN(0);
                    FLAGS.SCALE=0; //imposto la scala di tensione normale
                 
                }
                
                if(HW_CFG.VDC!=0)
                {   // DC signal
                    FLAGS.O_L = 0;
                    FLAGS.I_L = 0;
                    FLAGS.V_L = 1;
                    FLAGS.DC  = 1;
                }
                if(HW_CFG.IDC)
                {   // DC signal
                    FLAGS.O_L = 0;
                    FLAGS.I_L = 1;
                    FLAGS.V_L = 0;
                    FLAGS.DC  = 1; 
                }
            }    
            else // SPI oppure Console             
            {   // abilita la spi perchè il bt 14 è a 0
               // FLAGS.WV_D = 1;         // stop internal waveform generation (if active)
                FLAGS.SPI = 1;
                FLAGS.CYC = 0;
                if(HW_CFG.OL)       // lo metto per prima perchè è meno prioritario
                {   // Open loop mode
                    FLAGS.V_L = 0;
                    FLAGS.I_L = 0;
                    FLAGS.O_L = 1;   
                }       
                if(HW_CFG.VL) // loop in tensione
                {  
                    FLAGS.O_L = 0;
                    FLAGS.I_L = 0;
                    FLAGS.V_L = 1;
                    
                }
            
                if(HW_CFG.IL) // current mode AC  
                {// Current mode
                    FLAGS.V_L = 0;
                    FLAGS.O_L = 0;
                    FLAGS.I_L = 1;
                    
                }
                 
            }
  
            #ifdef _PID 
            cfg_PID();                          // configure PID controller
                                                // and load coefficients
            #endif
            #ifdef _3P3Z
            cfg_3P3Z();                         // configure 3P3Z controller
                                                // and load coefficients
            #endif
            #ifdef _dspPID
            cfg_dspPID();                       // configure dspPID controller
                                                // and load coefficients
            #endif
            
            FLAGS.UHW = 0;
        
        }
        // sleep until next IRQ -------------------------------------------------
        Idle();
   }
    
    return 0;
}


