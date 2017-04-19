/*
 * Post_SPI.h
 *
 *  Created on: Mar 14, 2017
 *      Author: Mason
 */

#ifndef POST_SPI_H_
#define POST_SPI_H_

#include "F28x_Project.h"
#include "F2837xS_device.h"
#include "F2837xS_Examples.h"


//High Speed SPI Prototypes
void spi_xmita(Uint16 a);		//16 bit transmit out of SPI Port A
void spi_xmitb(Uint16 a);		//16 bit transmit out of SPI Port B
void spi_xmitc(Uint16 a);		//16 bit transmit out of SPI Port C
void spi_fifo_init(void);		//Initialize FIFO Buffers


#endif /* POST_SPI_H_ */


