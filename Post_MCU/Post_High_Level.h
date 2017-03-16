/*
 * Post_High_Level.h
 *
 *  Created on: Mar 14, 2017
 *      Author: Mason
 */

#ifndef POST_HIGH_LEVEL_H_
#define POST_HIGH_LEVEL_H_

#include "F28x_Project.h"
#include "F2837xS_device.h"
#include "F2837xS_Examples.h"


//Values for LED Data Parsing
#define StreamSize           171072     //LED data stream size in bytes
#define ChunkSizeI            3168      // Size of a single chunk pulled from FRAM in 16 bit words
#define ChunkSizeB            6336      // Size of a single chunk pulled from FRAM in bytes

//LED Data Handling Prototypes
void getLEDChunk(Uint16 startL1L2, Uint16 endL1L2,Uint16 startL3L4, Uint16 endL3L4,Uint16 startL5L6, Uint16 endL5L6, Uint16* data);
void Runtime();
void switchDataChunk(int u);

//
//CPU Timer ISR
__interrupt void cpu_timer0_isr(void);



#endif /* POST_HIGH_LEVEL_H_ */
