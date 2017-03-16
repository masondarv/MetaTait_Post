/*
 * Post_I2C.h
 *
 *  Created on: Mar 14, 2017
 *      Author: Mason
 */

#ifndef POST_I2C_H_
#define POST_I2C_H_

#include "F28x_Project.h"
#include "F2837xS_device.h"
#include "F2837xS_Examples.h"

// Values for I2C Communication
#define I2C_SLAVE_ADDR        0x50
#define I2C_NUMBYTES          3
#define I2C_EEPROM_HIGH_ADDR  0x00
#define I2C_EEPROM_LOW_ADDR   0x30
#define Post1_SLAVE_ADDR      0x25
#define Post2_SLAVE_ADDR      0x35
#define Post3_SLAVE_ADDR      0x45

// I2C Prototypes
void   I2C_Receive(void);
void   I2CA_Init(void);
Uint16 I2CA_WriteData(struct I2CMSG *msg);
Uint16 I2CA_ReadData(struct I2CMSG *msg);
void I2CA_SlaveRead(struct I2CMSG*msg);
__interrupt void i2c_int1a_isr(void);
void pass(void);
void fail(void);



#endif /* POST_I2C_H_ */
