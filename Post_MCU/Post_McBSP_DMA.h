/*
 * Post_McBSP_DMA.h
 *
 *  Created on: Mar 14, 2017
 *      Author: Mason
 */

#ifndef POST_MCBSP_DMA_H_
#define POST_MCBSP_DMA_H_



#include "F28x_Project.h"
#include "F2837xS_device.h"
#include "F2837xS_Examples.h"




// Choose a word size for MCBSP DMA.  Uncomment one of the following lines
//
//#define WORD_SIZE  8      // Run a loopback test in 8-bit mode
#define WORD_SIZE 16      // Run a loopback test in 16-bit mode
//#define WORD_SIZE 32      // Run a loopback test in 32-bit mode

//Values for McBSP Communication
#define CLKGDV_VAL   1

#define CPU_SPD              200E6
#define MCBSP_SRG_FREQ       CPU_SPD/4   // SRG input is LSPCLK (SYSCLKOUT/4)
                                         // for examples

// # of CPU cycles in 2 CLKG cycles-init delay
#define MCBSP_CLKG_DELAY     2*(CPU_SPD/(MCBSP_SRG_FREQ/(1+CLKGDV_VAL)))




//MCBSP DMA Prototypes
__interrupt void local_D_INTCH1_ISR(void);
__interrupt void local_D_INTCH2_ISR(void);
void mcbsp_init_dlb(void);
void init_mcbsp_spi(void);
void mcbsp_xmit(Uint16 a);
void init_dma(void);
void init_dma_32(void);
void start_dma(void);
void error(void);



#endif /* POST_MCBSP_DMA_H_ */
