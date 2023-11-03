
#include <p33EP256MC204.h>
#include "i2c_stam.h"
#include "system.h"
#include "main.h"
/*
void I2C_Init(void)
{

    I2C1CONbits.I2CEN = 0;	
	I2C1CONbits.I2CSIDL 	= 0;
	I2C1CONbits.IPMIEN 	= 0;
	I2C1CONbits.A10M		= 0;
	I2C1CONbits.DISSLW 	= 1;
	I2C1CONbits.SMEN 	= 0;
    I2C1BRG              = setBaudRate();    
    __delay_ms(1);
    I2C1CONbits.I2CEN 	= 1;
}
*/
/*
int setBaudRate(){
    return (FCY/I2C_BAUDRATE - FCY/1111111) - 1;
}
*/
/* funzione diretta, non divisa in parti per ciclo macchina. viene eseguita in una chiamata*/
int I2C_WriteReg(char dev_addr, char reg_addr, char value)
{
    char wr_dev_addr = dev_addr << 1;
    // Send I2C start condition
	I2C1CONbits.SEN = 1;			
	while(I2C1CONbits.SEN == 1);
	// Send I2C device address on the bus for write operation
	I2C1TRN = wr_dev_addr;			
	while(I2C1STATbits.TRSTAT);			
	if (I2C1STATbits.ACKSTAT)				
	{								
		I2C1CONbits.PEN = 1;
		while(I2C1CONbits.PEN);			
		return I2C_ERROR;					
	}
    // Send register address on the bus
/*	I2C1TRN = reg_addr;
	while(I2C1STATbits.TRSTAT);
	if (I2C1STATbits.ACKSTAT)
	{
		I2C1CONbits.PEN = 1;
		while(I2C1CONbits.PEN);
		return I2C_ERROR;
	}*/
	// Send register value on the bus    
	I2C1TRN = value;
	while(I2C1STATbits.TRSTAT);
	if (I2C1STATbits.ACKSTAT)
	{
		I2C1CONbits.PEN = 1;
		while(I2C1CONbits.PEN);
		return I2C_ERROR;
	}
	/// Send I2C stop condition
	I2C1CONbits.PEN = 1;
	while(I2C1CONbits.PEN);
	return I2C_OK;
}
/* funzione diretta, non divisa in parti per ciclo macchina. viene eseguita in una chiamata*/
int I2C_ReadReg(char dev_addr, char reg_addr, char *value)
{
    //char wr_dev_addr = dev_addr << 1;
    char rd_dev_addr = (dev_addr << 1) | 0x01;
    // Send I2C start condition
	I2C1CONbits.SEN = 1;	
	while(I2C1CONbits.SEN == 1);
	// Send I2C device address on the bus for write operation
	I2C1TRN = rd_dev_addr;
	while(I2C1STATbits.TRSTAT);
	if (I2C1STATbits.ACKSTAT)
	{
		I2C1CONbits.PEN = 1;
		while(I2C1CONbits.PEN);
		return I2C_ERROR;
	}
    // Send I2C register address on the bus 
	/*I2C1TRN = reg_addr;
	while(I2C1STATbits.TRSTAT);
	if (I2C1STATbits.ACKSTAT)
	{	
		I2C1CONbits.PEN = 1;
		while(I2C1CONbits.PEN);
		return I2C_ERROR;
	}*/
    // Send I2C restart condition
 /*   I2C1CONbits.RSEN = 1;
        while(I2C1CONbits.RSEN == 1);	
    // Send I2C device address on the bus for read operation
        I2C1TRN = rd_dev_addr;
	while(I2C1STATbits.TRSTAT);
	if (I2C1STATbits.ACKSTAT)
	{
		I2C1CONbits.PEN = 1;
		while(I2C1CONbits.PEN);
		return I2C_ERROR;	
	}*/
    // Enable I2C clock for read operation
	I2C1CONbits.RCEN = 1;
        while(!I2C1STATbits.RBF);
    // Retrieve value from I2C register
	*value = I2C1RCV;	
	// Send I2C stop condition
	I2C1CONbits.PEN = 1;
	while(I2C1CONbits.PEN);
	return I2C_OK;
}

void stam_rele_on(void){
    char write=8; 
    //init_ic2_100khz();
    I2C_WriteReg(0x20, 0x00,  write );   
     //init_ic2_400khz();
}

void stam_rele_off(void){
    char write=0; 
   // init_ic2_100khz();
    I2C_WriteReg(0x20, 0x00,  write );    
   // init_ic2_400khz();
}

void stam_rele_read(void){
    char read=0;
   // char write=0; 
    I2C_ReadReg(0x20, 0x00,  &read );
}