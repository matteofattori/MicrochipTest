
// CONTROLLER TYPE -------------------------------------------------------------
#define _PID                            // enable PID type controller
//#define _3P3Z                           // enable 3P3Z type controller
//#define _dspPID                         // enable dsp PID type controller

// PID parameters --------------------------------------------------------------
// PID parameters are stored in FRAM and are configurable by user. If FRAM is not
// available, these default are used instead
// VOLTAGE MODE ----------------------------------------------------------------
#define _kp_Vdef     200                // 
#define _ki_Vdef     1000               // values near to 0 increase Integral action
#define _kd_Vdef     0

// CURRENT MODE ----------------------------------------------------------------
#define _kp_Idef     400              // 
#define _ki_Idef     250                // values near to 0 increase Integral action
#define _kd_Idef     0

// 3P3Z parameters -------------------------------------------------------------
// VOLTAGE MODE ----------------------------------------------------------------
/**
  Compensator Type:  3P3Z
      Entry                Value  
    ---------            ---------
  Pole 0                 5.0000e+01 Hz
  Pole 2                 1.0000e+04 Hz
  Pole 3                 1.0000e+05 Hz
  Zero 1                 4.0000e+03 Hz
  Zero 2                 5.0000e+03 Hz
  Gain(Kdc)              100.000
  Warp                   false
  PWM Frequency          5.0000e+04
  PWM Sampling Ratio     2
  Sampling Frequency     2.5000e+04
  PWM Max Resolution     3.9000e-08
  Computational Delay    1.2000e-05
  Gate Drive Delay       1.5000e-07
  Control Output Min.    -400
  Control Output Max.    400
  Kuc Gain               1.1749e+01
  Use Kuc Gain           true


  PWM Calculations
      Name                Value  
    ---------           ---------
  Bits of Resolution    10.002
  Gain                  9.760e-04


  s-domain transfer function

               Wp0   Wp2(Wp3)(Wz1 + s)(Wz2 + s)
  H(s) = Kdc X --- X --------------------------
                s    Wz1(Wz2)(Wp2 + s)(Wp3 + s)

                 3.14e+02   6.28e+04(6.28e+05)(2.51e+04 + s)(3.14e+04 + s)
  H(s) = 100.000 X -------- X ----------------------------------------------
                    s       2.51e+04(3.14e+04)(6.28e+04 + s)(6.28e+05 + s)



  Digital Compensator Coefficients

  Name    Value      Normalized    Q15      Hex
  ----    -----      ----------    ---      ---
  a1      0.034      0.001         37       0x0025
  a2      0.869      0.029         965      0x03C5
  a3      0.097      0.003         107      0x006B
  b0      29.500     1.000         32764    0x7FFC
  b1      13.003     0.441         14441    0x3869
  b2      -14.269    -0.484        -15847   0xC219
  b3      2.229      0.076         2475     0x09AB


  z-domain transfer function

         u(z)  B0 + B1z^(-1) + B2z^(-2) + B3z^(-3)
  H(z) = --- = -----------------------------------
         e(z)  A0 - A1z^(-1) - A2z^(-2) - A3z^(-3)

          (2.511) + (1.107)z^(-1) + (-1.214)z^(-2) + (0.190)z^(-3)
  H(z) = -----------------------------------------------------------
          1 - (0.034)z^(-1) - (0.869)z^(-2) - (0.097)z^(-3)

**/
#define _Va0         37
#define _Va1         965
#define _Va2         107

#define _Vb0         32764
#define _Vb1         14441
#define _Vb2         -15847
#define _Vb3         2457

#define _VpScaler    0x7603             // postascaler gain
#define _VpreS       0x0000             // shift value
#define _VpostS      0xFFFB             // shift value

// CURRENT MODE ----------------------------------------------------------------
/**
       Entry                Value  
    ---------            ---------
  Pole 0                 1,0000e+02 Hz
  Pole 2                 1,5000e+04 Hz
  Pole 3                 1,0000e+05 Hz
  Zero 1                 1,0000e+00 Hz
  Zero 2                 2,5000e+03 Hz
  Gain(Kdc)              1,000
  Warp                   false
  PWM Frequency          5,0000e+04
  PWM Sampling Ratio     2
  Sampling Frequency     2,5000e+04
  PWM Max Resolution     1,6000e-05
  Computational Delay    0,0000e+00
  Gate Drive Delay       0,0000e+00
  Control Output Min.    -490
  Control Output Max.    490
  Kuc Gain               4,0323e-03
  Use Kuc Gain           true


  PWM Calculations
      Name                Value  
    ---------           ---------
  Bits of Resolution    1,322
  Gain                  6,667e-01


  s-domain transfer function

               Wp0   Wp2(Wp3)(Wz1 + s)(Wz2 + s)
  H(s) = Kdc X --- X --------------------------
                s    Wz1(Wz2)(Wp2 + s)(Wp3 + s)

                 6,28e+02   9,42e+04(6,28e+05)(6,28e+00 + s)(1,57e+04 + s)
  H(s) = 1,000 X -------- X ----------------------------------------------
                    s       6,28e+00(1,57e+04)(9,42e+04 + s)(6,28e+05 + s)



  Digital Compensator Coefficients

  Name    Value     Normalized    Q15      Hex
  ----    -----     ----------    ---      ---
  a1      -0,159    -0,156        -5113    0xEC07
  a2      0,898     0,879         28812    0x708C
  a3      0,262     0,256         8392     0x20C8
  b0      1,021     1,000         32764    0x7FFC
  b1      -0,533    -0,522        -17091   0xBD3D
  b2      -1,021    -1,000        -32760   0x8008
  b3      0,533     0,522         17095    0x42C7


  z-domain transfer function

         u(z)  B0 + B1z^(-1) + B2z^(-2) + B3z^(-3)
  H(z) = --- = -----------------------------------
         e(z)  A0 - A1z^(-1) - A2z^(-2) - A3z^(-3)

          (253,198) + (-132,077)z^(-1) + (-253,168)z^(-2) + (132,107)z^(-3)
  H(z) = -----------------------------------------------------------
          1 - (-0,159)z^(-1) - (0,898)z^(-2) - (0,262)z^(-3)
 */
#define _Ia0         -2272
#define _Ia1         23261
#define _Ia2         2640

#define _Ib0         32764
#define _Ib1         -6441
#define _Ib2         -27477
#define _Ib3         11728

#define _IpScaler    15000              // postascaler gain
#define _IpreS       0                  // shift value
#define _IpostS      -1                 // shift value

// dsp PID parameters ----------------------------------------------------------

// Voltage loop
#define _kav         3000
#define _kbv         -500
#define _kcv         0

#define _poSv        0x7FFF             // postascaler gain
#define _prShv       0x0000             // shift value
#define _poShv       0x0000             // shift value


// Current loop
#define _kai         10500
#define _kbi         -10000
#define _kci         0

#define _poSi        0x7FFF             // postascaler gain
#define _prShi       0                  // shift value
#define _poShi       -2                 // shift value


