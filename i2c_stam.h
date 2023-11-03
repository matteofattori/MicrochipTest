/* 
 * File:   i2c_stam.h
 * Author: Techimp
 *
 * Created on April 8, 2019, 1:46 PM
 */

#ifndef I2C_STAM_H
#define	I2C_STAM_H

#ifdef	__cplusplus
extern "C" {
#endif

//#include <xc.h>

#define I2C_ERROR           -1
#define I2C_OK              1

//void I2C_Init(void);
//int setBaudRate(void);
int I2C_WriteReg(char dev_addr, char reg_addr, char value);
int I2C_ReadReg(char dev_addr, char reg_addr, char *value);

void stam_rele_on(void);


void stam_rele_off(void);

void stam_rele_read(void);


#ifdef	__cplusplus
}
#endif

#endif	/* I2C_STAM_H */

