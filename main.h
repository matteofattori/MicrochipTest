
/******************************************************************************/
// This file contains all defines and constants, FLAGS structures are also
// declared here
/******************************************************************************/

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef XC_HEADER_TEMPLATE_H
#define	XC_HEADER_TEMPLATE_H

#include <xc.h> // include processor files - each processor file is guarded.  

// user's declarations ---------------------------------------------------------
#include <p33EP256MC204.h>
#include "dsp_coeff.h"

// VERSION ---------------------------------------------------------------------
#define FW_VERa     '9'     // Firmware version
#define FW_VERb     '6'     // data will be exposed as "FW_vera FW_verb": "01" versione di matteo
#define HW_VERa     '0'     //Hardware version
#define HW_VERb     '3'

// controller ------------------------------------------------------------------
//#define 3_levels_PWM    0
//#define 50kHz_PWM       1
//#define 100kHz_PWM      2
//#define _3levels_PWM             // enable 50kHz 3 levels pwm: higher zero crossing distortion but less ripple
#define _50kHz_PWM             // enable complementary mode PWM @ 50kHz: lower distortion but more ripple
//#define _100kHz_PWM            // enable complementary mode PWM @ 100kHz: low ripple, low distortion, high power loss

//#define _use_dithering           // use dithering to increase resolution (higher out noise)
//#define _12bit
#define _10bit

#define _offset_compensation            // compensate output offset in voltage mode by measuring positive
                                // and negative I peak values: force them to be equal

#define _auto_overrange_ctrl            // clear automatically over-range signalling when
                                // V or I level falls down the hyst low level

//#define _zero                           // force hyst low level @ zero
                                // (assert over-range until zero crossing)

#define _use_TMP431                     // use I2C temperature sensor (T.IGBT and T.BOARD)
#define _use_FRAM                       // use external FRAM backup memory

// Vline loop gain compensation: decrease loop gain when Vline increases
#define _k_gain     98320000     // _k_gain = V_line * 32000
#define _def_gain   32000        // is the gain @ Vline = 30V
#define _Vl_min     2690         // XWB cambiato era 3050Vline, è il valore dell'ADC value at 30V
#define _Vl_scale   356          // XWB prima era 326 , fractional value to scale Vline in Volts cambiato prima era 328 e deriva da vref=3 * 108 attenuazione del partitore sulla lettura dell'aliment ponte
#define Vline320    320          // threshold value for assert Vline overvolage

#define LSB_180V    297         // LSB nella scala 180V,è riferito a 32768, per ottenere in V si divide per 32768    
#define LSB_18V     38          // LSB nella scala 18V,riferito a 32768, per ottenere in V si divide 32768 
#define Ratio180_18 7.74        // rapporto tra le scale

#define SPI_CONVERSION_VOLTAGE_180V  29200          // fattore di conversione scala 180V del dato che lega l'uscita in tensione con il dato fornito tramite protocollo SPI es. 7FFF deve corrispondere al massimo della tensione ai capi del ponte
#define SPI_CONVERSION_VOLTAGE_18V  1450            // richiesta di Puricelli, scala 18V 7FFF=18V+5% fattore conversione scala 18V è crica 7 ( il rapporto dei guadagni delle due scale)   
#define SPI_CONVERSION_VOLTAGE_OPEN_180V 5200      // conversione open loop        
#define SPI_CONVERSION_VOLTAGE_OPEN_18V 530        // l'ho introdotta perchè il significato del codice SPI cambia con la scala, 7FFF=18V + 5%
#define SPI_CONVERSION_CURRENT  8500                // 7FFF corrisponde a 6Arms x 1,41 = 8.46                                                                   // 7FFF x SPI_CONVERSION_VOLTAGE =  massima tensione in uscita che non sarà 300V ma 300V - il delta dato dalle perdite sugli IGBT
#define SPI_CONVERSION_CURRENT_6A  20500 
// 7FFF = dato fornito dalla sPI
#define CYC_CONVERSION_VOLTAGE_VPP  1.112 //STAM moltiplicato con scala su cyc da impostazione cyc sul campo amplitude Vpk in mV
#define CYC_CONVERSION_VOLTAGE_VRMS  1.55//STAM moltiplicato con scala su cyc da impostazione cyc sul campo amplitude Vrms in mV
#define CYC_CONVERSION_VOLTAGE_VRMS_HIGH_SENS 2.4
#define CYC_CONVERSION_CURRENT_IRMS  1.23 //STAM moltiplicato con scala su cyc da impostazione cyc sul campo amplitude Irms  in mA
#define CYC_CONVERSION_CURRENT_DC  0.9 //STAM moltiplicato con scala su cyc da impostazione cyc sul campo amplitude Irms  in mA

#define V_offsetFixed 0 //-57             //usato per compensare in maniera statica l'offset
#define I_offsetFixed 0
#define Ilow_offsetFixed 0


// scale NTC data
#define s_NTC       -136                // this value will be divided by 32768 giving -0.04150
#define o_NTC       231                 // offset value: is 158 + 73 in order to be compliant with
                             // protocol representation format
#define min_NTC     3000                // min ADC value, below this threshold assert an error

// gain factor for voltage / current conversion --------------------------------
// these values convert V, I raw data to V and mA
#define _kv         303       // raw to Volt scale factor 0.0155 prima era 505
#define _ki         14500     // shunt 17373 raw to mA scale factor 0.53 (x2)
#define _ki_out     173       // raw to A/10 scale factor 0.0053

#define _kv_raw     1166    //1027                // Volt to raw scale factor 64.516 (x16)
#define _ki_raw    905 //3017                // A/10 to raw scale factor 188.67 (x16)

// waveform, frequency and amplitude control
#define _min_freq   833                 // min freq = 1Hz
#define _max_freq   333333              // max freq = 400Hz
#define _freq_step  833                 // 1Hz freq step
#define _f50Hz      41666               // 50Hz

#define _min_dc_scale   -32100
#define _min_scale      0
#define _max_scale      32100
#define _scale_step     2
#define _scale_step_big 10

// clipping values for offset compensation
#define _max_i_diff 100
#define _min_i_diff -100

// LIMIT values for error assertion --------------------------------------------
// overvoltage/current limit (not in mA or V, raw values)
#define _v_limit    32000    //MAT prima era 10000 21000 perchè ci mette un x 1.5
#define _i_limit    21000    //MAT prima era 5000

// maximum V, I values for output (error assertion, raw values)
#define max_vout    32000   // se l'ADC sulla tensione supera questo valore si segnala over range
#define max_iout    28000   // se l'ADC sull corrente supera questo valore si segnala overcurrent

// clipping value for shutdown
#define max_V_limit 20000     // over this value clip the limit.XS threshold
#define max_VS      32000
#define max_I_limit 20000
#define max_IS      30000

// offset zeroing max values (raw data)
#define max_offset  800
#define max_noffset -800

// Temperatures over range
#define _max_TBOARD 125                 // 125 - 73 = 52°C
#define _max_TIGBT  143                 // 125 - 73 = 52°C cambiato  XWB
#define _max_TTRAFO 155                 // 155 - 73 = 82°C

// USEFUL DEFINES --------------------------------------------------------------
#define ZERO        0
#define ONE         1
#define DISABLE     0
#define ENABLE      1

// 40us SW timer ---------------------------------------------------------------
#define _40us      1
#define _200us      5
#define _1ms        25
#define _100ms      2500
#define _200ms      5000
#define _500ms      12500
#define _1s         25000
#define _2s         50000
#define _1min       1500000

// Consolle --------------------------------------------------------------------
#define c_str_l     41                     // string length
#define _rms_dec    8                       // samples decimation factor
#define _max_dec    255                     // peak decay decimation factor

#define _ESC        0x1B                    // escape char
#define _BKS        0x08                    // backspace
#define _ENT        0x0D                    // carridge return
#define _CR         0x0A                    // line feed

// State Machine ---------------------------------------------------------------
#define NORM        0
#define OFF_CAL     1
#define K_IN        2
#define OR_IN       3
#define LOAD_PAR    4
#define SERIAL_SET  5
#define MAN_SET     6
#define BRD_DATA    7

#define ERROR       99

#define END_FUNC    0

#define F_ERR       -1                      // function state machine error

// FRAM ADDRESSES --------------------------------------------------------------
// memory space is byte based, read/write routines are word (16bit) based
#define MAN_L       0x0000                  // manufacturer string (low part)
#define MAN_H       0x0002                  // manufacturer string (high part)

#define SERIAL_L    0x0004                  // serial number - low part
#define SERIAL_M    0x0006                  // serial number - mid part
#define SERIAL_H    0x0008                  // serial number - high part

#define M_CNT_ADD_L 0x000A                  // minute counter FRAM address - low part
#define M_CNT_ADD_H 0x000C                  // minute counter FRAM address - high part

#define M_ACT_ADD_L 0x000E                  // minute of activity counter FRAM address - low part
#define M_ACT_ADD_H 0x0010                  // minute of activity counter FRAM address - high part

#define PID_V_ADD   0x0012                  // PID Kx Voltage mode address (3 words)

#define PID_I_ADD   0x0018                  // PID Kx Voltage mode address (3 words)

#define PID_V_18VADD 0x001E               // XWB PID Kx Voltage scala 18V mode address (3 words)

// Debounce threshold ----------------------------------------------------------
#define B_thr_up    33268
#define B_thr_dn    32268

#define B_lim_up    33368
#define B_lim_dn    32168

#ifdef _3levels_PWM
// set PWM period to 50kHz in center aligned mode: PHASEx = fosc / (fPWM * PWM prescaler * 2)
// PWM_freq > (PWM_max - PWM_min) + PWM_dead_time
#define _PWM_freq           1200 //era 1200
#define _PWM_max            580//era540  520 va nello squadratore 540 lo regge solo se è caricata l'uscita 
#define _PWM_min           -580//era -520

#define _PWM_max_duty       1200
#define _PWM_mid_duty       600

// set PWM dead time: ALTDTRx = fosc * (dead_time / PWM prescaler)
#define _PWM_dead_time      100//100 prima era 50 35 15 XWB
#endif

#ifdef _50kHz_PWM
// set PWM period to 50kHz in complementary mode: PHASEx = fosc / (fPWM * PWM prescaler)
#define _PWM_freq           2400
#define _PWM_max            1100
#define _PWM_min            -1100

#define _PWM_max_duty       2400
#define _PWM_mid_duty       1200

// set PWM dead time: ALTDTRx = fosc * (dead_time / PWM prescaler)
#define _PWM_dead_time      50
#endif

#ifdef _100kHz_PWM
// set PWM period to 100kHz in complementary mode: PHASEx = fosc / (fPWM * PWM prescaler)
#define _PWM_freq           1200
#define _PWM_max            450
#define _PWM_min            -450

#define _PWM_max_duty       1200
#define _PWM_mid_duty       600

// set PWM dead time: ALTDTRx = fosc * (dead_time / PWM prescaler)
#define _PWM_dead_time      50
#endif

// INTERRUPTS ------------------------------------------------------------------
#define T1_IRQ_CLEAR        IFS0bits.T1IF = DISABLE
#define T1_IRQ_ENABLE       IEC0bits.T1IE = ENABLE
#define T1_IRQ_DISABLE      IEC0bits.T1IE = DISABLE

#define T3_IRQ_CLEAR        IFS0bits.T3IF = DISABLE
#define T3_IRQ_ENABLE       IEC0bits.T3IE = ENABLE
#define T3_IRQ_DISABLE      IEC0bits.T3IE = DISABLE

#define PWM_IRQ_CLEAR       IFS5bits.PWM2IF = DISABLE
#define PWM_IRQ_ENABLE      IEC5bits.PWM2IE = ENABLE
#define PWM_IRQ_DISABLE     IEC5bits.PWM2IE = DISABLE

#define PWM_FAULT_DISABLE   PWMCON2bits.FLTIEN = DISABLE
#define PWM_FAULT_ENABLE    PWMCON2bits.FLTIEN = ENABLE
#define PWM_FAULT_IRQ       PWMCON2bits.FLTSTAT

#define DMA0_IRQ_CLEAR      IFS0bits.DMA0IF = DISABLE
#define DMA0_IRQ_ENABLE     IEC0bits.DMA0IE = ENABLE
#define DMA0_IRQ_DISABLE    IEC0bits.DMA0IE = DISABLE

#define DMA1_IRQ_CLEAR      IFS0bits.DMA1IF = DISABLE
#define DMA1_IRQ_ENABLE     IEC0bits.DMA1IE = ENABLE
#define DMA1_IRQ_DISABLE    IEC0bits.DMA1IE = DISABLE

#define DMA2_IRQ_CLEAR      IFS1bits.DMA2IF = DISABLE
#define DMA2_IRQ_ENABLE     IEC1bits.DMA2IE = ENABLE
#define DMA2_IRQ_DISABLE    IEC1bits.DMA2IE = DISABLE

#define U1_TX_IRQ_CLEAR     IFS0bits.U1TXIF = DISABLE
#define U1_TX_IRQ_ENABLE    IEC0bits.U1TXIE = ENABLE
#define U1_TX_IRQ_DISABLE   IEC0bits.U1TXIE = DISABLE

#define U1_RX_IRQ_CLEAR     IFS0bits.U1RXIF = DISABLE
#define U1_RX_IRQ_ENABLE    IEC0bits.U1RXIE = ENABLE
#define U1_RX_IRQ_DISABLE   IEC0bits.U1RXIE = DISABLE

#define U2_TX_IRQ_CLEAR     IFS1bits.U2TXIF = DISABLE
#define U2_TX_IRQ_ENABLE    IEC1bits.U2TXIE = ENABLE
#define U2_TX_IRQ_DISABLE   IEC1bits.U2TXIE = DISABLE

#define U2_RX_IRQ_CLEAR     IFS1bits.U2RXIF = DISABLE
#define U2_RX_IRQ_ENABLE    IEC1bits.U2RXIE = ENABLE
#define U2_RX_IRQ_DISABLE   IEC1bits.U2RXIE = DISABLE

#define SPI_IRQ_CLEAR       IFS2bits.SPI2IF = DISABLE
#define SPI_IRQ_ENABLE      IEC2bits.SPI2IE = ENABLE
#define SPI_IRQ_DISABLE     IEC2bits.SPI2IE = DISABLE

// GPIOs -----------------------------------------------------------------------
 #define _idle               LATCbits.LATC7                      // LED 1 is IDLE indicator
 #define LED2                LATCbits.LATC6

#define CAL_V               LATCbits.LATC9
#define CAL_I               LATCbits.LATC8

#define ON_OFF              PORTCbits.RC2  
#define OV_OUT              LATAbits.LATA7
#define OI_OUT              LATBbits.LATB7

#define BOOT_ENTER          PORTBbits.RB15

#define DRV_SHUTDOWN        LATBbits.LATB8 //è l'accensione del ponte
#define DRV_FAULT           PORTCbits.RC1

// Generation cycle structure --------------------------------------------------
#define freq_gradient       0b00100
#define ampl_gradient       0b00011
#define hold                0b00010
#define shot                0b00001
#define cycle_free          0xFFFF

typedef struct  {   
                    unsigned int    f_div;                                      // f_div value, received from master
                    long            M_val;                                      // phase accumulator increment, computed from f_div
                    long            M_sweep;                                    // phase accumulator increment sweep, computed from gradient
                    unsigned long   amplitude;                                  // received from master
                    float             amp_sweep_float;                                  // amplitude sweep control, computed from gradient
                    int             amp_sweep;                                  // amplitude sweep control, computed from gradient
                    long            gradient;                                   // received from master
                    unsigned long   time;                                       // received from master
                    union   {   unsigned int quality_word;
                                struct  {   unsigned char   type        :5;     // this field specifies the waveform type
                                            unsigned char               :2;
                                            unsigned char   waveform    :2;
                                            unsigned char   harmonic    :4;
                                            unsigned char   dur_type    :3;
                                        } quality;
                            };
                    unsigned int    n;                                          // cycle index
                } cycle_str_type;


// FLAGS -----------------------------------------------------------------------
union   FLAGS_type {    unsigned long    ALL;
                        struct {    
                                    unsigned char   RMS :1;     // select between rms or istant. values
                                    unsigned char   WV  :1;     // waveform ON
                                    unsigned char   CAL :1;     // offset calibration performed
                                    unsigned char   WV_D:1;     // request wave OFF
                                    unsigned char   O_L :1;     // open loop mode
                                    unsigned char   V_L :1;     // voltage mode
                                    unsigned char   I_L :1;     // current mode
                                    unsigned char   CHR :1;     // char received
                                    unsigned char   ORV :1;     // overvoltage detected
                                    unsigned char   ORI :1;     // overcurrent detected
                                    unsigned char   SDV :1;     // shutdown for overvoltage
                                    unsigned char   SDI :1;     // shutdown for overcurrent
                                    unsigned char   UV  :1;     // Vline undervoltage
                                    unsigned char   UVL :1;     // Vline undervoltage latch
                                    unsigned char   TMP :1;     // TMP431 busies I2C bus
                                    unsigned char   FRM :1;     // FRAM busies I2C bus
                                    unsigned char   SCM :1;     // save minute counter
                                    unsigned char   RX  :1;     // a packet has been received
                                    unsigned char   ROK :1;     // packet was valid
                                    unsigned char   SPI :1;     // waveform by data streaming
                                    unsigned char   UHW :1;     // update HW configuration
                                    unsigned char   ENQ :1;     // reply at enquire command with data
                                    unsigned char   CYC :1;     // enable cycle-based waveform generation
                                    unsigned char   DS  :1;     // n DDS waveform sign
                                    unsigned char   ODS :1;     // n-1 DDS waveform sign
                                    unsigned char   MS  :1;     // ms flag for waveform update
                                    unsigned char   DC  :1;     // DC signal required
                                    unsigned char   ZC  :1;     // zero crossing detect
                                    unsigned char   EN  :1;     // enable/disable out (waveform on/off)
                                    unsigned char   ENL :1;     // enable/disable latch for toggle detection
                                    unsigned char   SCALE :1;   // scala in tensione 0=scala 180V , 1=scala 18V  
                                    unsigned char   SCALEI :1;  //  abilita la scala in corrente 0=scala 25A 1=scala 6A    
                                    unsigned char   CHANGE_DDS :1;     // 
                                    unsigned char   DDS_STATUS:3;// statodds 0=sine, 01=square 10=tria
                                };
                         };
extern union FLAGS_type FLAGS;

union   ERR_FLAGS_type {    unsigned long    ALL;
                            struct {    unsigned char   B0 :8;      // byte #0
                                        unsigned char   B1 :8;      // byte #1
                                        unsigned char   B2 :8;      // byte #2
                                        unsigned char   B3 :8;      // byte #3
                                    };

                            struct {    unsigned char   f0  :1;     // 
                                        unsigned char   f1  :1;     // 
                                        unsigned char   f2  :1;     // 
                                        unsigned char   DRV :1;     // fault on power bridge
                                        unsigned char   OV  :1;     // Vline overvoltage > 320V
                                        unsigned char   UV  :1;     // Vline undervoltage < 30V
                                        unsigned char   DIP :1;     // triggered a supply dip
                                        unsigned char   MIO :1;     // maximum output current exceeded
                                        
                                        unsigned char   NTC :1;     // short on NTC trafo sensor
                                        unsigned char   TOT :1;     // serial communication timeout
                                        unsigned char   MTB :1;     // board overtemperature
                                        unsigned char   MTT :1;     // TRAFO overtemperature
                                        unsigned char   MTI :1;     // IGBT overtemperature
                                        unsigned char   SDN :1;     // shutdown request from master
                                        unsigned char   SO  :1;     // output shutdown for overcurrent / overvoltage
                                        unsigned char   MVO :1;     // maximum output voltage exceeded
                                        
                                        unsigned char   f16 :1;     // 
                                        unsigned char   f17 :1;     // 
                                        unsigned char   f18 :1;     // 
                                        unsigned char   f19 :1;     // 
                                        unsigned char   f20 :1;     // 
                                        unsigned char   f21 :1;     // 
                                        unsigned char   f22 :1;     // 
                                        unsigned char   f23 :1;     // 
                                        
                                        unsigned char   PCF :1;     // PCF8574 bus error XWB
                                        unsigned char   OFF :1;     // larger offset revealed
                                        unsigned char   REM :1;     // TMP431 remote sensor error
                                        unsigned char   TMP :1;     // TMP431 bus error
                                        unsigned char   f28 :1;     // 
                                        unsigned char   f29 :1;     // 
                                        unsigned char   f30 :1;     // 
                                        unsigned char   MEM :1;     // FRAM bus error
                                    };
                        };
extern union ERR_FLAGS_type ERR_FLAGS;
extern union ERR_FLAGS_type ERR_FLAGS_MASK;

struct CFG_type     {   union   {   unsigned char   LSB;
                                    struct {    unsigned char   VDC :3;     // loop on voltage DC
                                                unsigned char   SCALEI :1;  // se è a 1 scala 6A , 0 scala 25A
                                                unsigned char   IAC :1;     // loop on current AC
                                                unsigned char   IDC :1;     // loop on current DC
                                                unsigned char   VAC :1;     // loop on voltage AC
                                                unsigned char   SCALE:1;    // se è a 1 si vuole la scala più sensibile
                                            };
                                };
                        union   {   unsigned char   MSB;
                                    struct {    unsigned char   VL  :1;     // voltage mode
                                                unsigned char   OL  :1;     // open loop mode
                                                unsigned char   IL  :1;     // current mode
                                                unsigned char   :3;
                                                unsigned char   URT :1;     // use serial commands or SPI stream 1= seriale ,0=spi
                                                unsigned char   PEN :1;     // BIT15 power section enable 1on 0 off 
                                            };
                                };
                    };
extern struct CFG_type HW_CFG;

// end on user's declarations --------------------------------------------------

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

