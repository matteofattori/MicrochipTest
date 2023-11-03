/* 
 * File:   I2C_driver.h
 * Author: marco.landoni
 *
 * Created on 27 settembre 2017, 15.34
 */

void I2C_START(void);
void I2C_STOP(void);
void I2C_RESTART(void);
int I2C_WRITE(int);
int I2C_READ(void);
int TMP431_drv(void);
//int VscaleEN(void); //XWB chiamata I2C per abilitare la scala in tensione
int VscaleEN(char en);     
int GetVscale(void);     

int FRAM_write(unsigned int, unsigned int);
int FRAM_read(unsigned int *, unsigned int);
