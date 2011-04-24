// talk to tmp75/175/275 family of i2c temperature sensors
// eric volpe (epvuc@limpoc.com) 3/18/2009
// constants for the config register are included in tmp75.h
// for example, 
// tmp75_write_config(TMP75_12BITS);
// 
// modified by kevin
//

#include "../../inc/sensn/i2cmaster.h"
#include "../../inc/deRFaddon/uart.h"
#include <util/delay.h>

#define TMP75_ADDR 0x92

// read tmp75 config register. see tmp75.h for bit meanings
uint8_t tmp75_read_config(void)
{ 
	uint8_t conf;

	i2c_start_wait(TMP75_ADDR+I2C_WRITE);
	i2c_write(0x01); // address config register
	i2c_rep_start(TMP75_ADDR + I2C_READ);
	conf = i2c_readNak();
	i2c_stop();
	return(conf);
}

// write to tmp75 config register
void tmp75_write_config(uint8_t conf)
{ 
	i2c_start_wait(TMP75_ADDR+I2C_WRITE);
	i2c_write(0x01); // address config register
	i2c_write(conf);
	i2c_stop();
}

// read current temperature in degC * 16. 
int16_t tmp75_read_temp(void)
{ 

	uint8_t tmphi, tmplo;
	int16_t temp;
	i2c_start_wait(TMP75_ADDR+I2C_WRITE);
	i2c_write(0x00); // address Temperature register
	i2c_rep_start(TMP75_ADDR + I2C_READ);
	tmphi = i2c_readAck();
	tmplo = i2c_readNak();
	i2c_stop();
	temp = ((tmphi<<8)|tmplo)>>4;
	return(temp);
} 

// read low temp alarm threshold
int16_t tmp75_read_tlow(void)
{ 
	uint8_t tmphi, tmplo;
	uint16_t temp;
	i2c_start_wait(TMP75_ADDR+I2C_WRITE);
	i2c_write(0x02); // address TempLow register
	i2c_rep_start(TMP75_ADDR + I2C_READ);
	tmphi = i2c_readAck();
	tmplo = i2c_readNak();
	i2c_stop();
	temp = ((tmphi<<8)|tmplo)>>4;
	return(temp);
} 

// read high temp alarm threshold
int16_t tmp75_read_thi(void)
{ 
	uint8_t tmphi, tmplo;
	uint16_t temp;
	i2c_start_wait(TMP75_ADDR+I2C_WRITE);
	i2c_write(0x03); // address TempHigh register
	i2c_rep_start(TMP75_ADDR + I2C_READ);
	tmphi = i2c_readAck();
	tmplo = i2c_readNak();
	i2c_stop();
	temp = ((tmphi<<8)|tmplo)>>4;
	return(temp);
} 

// set temp alarm low threshold. (12 bit number)
void tmp75_write_tlow(int16_t tlow)
{ 
	tlow <<= 4;
	i2c_start_wait(TMP75_ADDR+I2C_WRITE);
	i2c_write(0x02); // address TempLow register
	i2c_write(tlow>>8);	// write high byte
	i2c_write(tlow&0x00ff); // write low byte
	i2c_stop();
} 

// set temp alarm high threshold. (12 bit number)
void tmp75_write_thi(int16_t thi)
{ 
	thi <<= 4;
	i2c_start_wait(TMP75_ADDR+I2C_WRITE);
	i2c_write(0x03); // address TempHigh register
	i2c_write(thi>>8);	// write high byte
	i2c_write(thi&0x00ff); // write low byte
	i2c_stop();
} 
