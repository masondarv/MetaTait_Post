/*
 * Post_McBSP_DMA.c
 *
 *  Created on: Mar 14, 2017
 *      Author: Mason
 */

#include "F28x_Project.h"
#include "F2837xS_device.h"
#include "F2837xS_Examples.h"
#include "Post_McBSP_DMA.h"
#include "Post_High_Level.h"


extern unsigned long chunk1Addr;            //address of the first third of data in FRAM
extern unsigned long chunk2Addr;            //address of the first third of data in FRAM
extern unsigned long chunk3Addr;            //address of the third third of data in FRAM



//
// error - Function to handle errors and halt debugger
//
void error(void)
{
   __asm("     ESTOP0"); // Test failed!! Stop!
    for (;;);
}



//
// mcbsp_xmit - Transmit MCBSP data
//
void mcbsp_xmit(Uint16 a)
{
    McbspaRegs.DXR1.all = a;
}

//
// init_mcbsp_spi - Configure McBSP settings
//
void init_mcbsp_spi()
{
    //
    // McBSP-A register settings
    //
    McbspaRegs.SPCR2.all = 0x0000;       // Reset FS generator, sample rate
                                         // generator & transmitter
    McbspaRegs.SPCR1.all = 0x0000;       // Reset Receiver, Right justify word,

    McbspaRegs.PCR.all = 0x0F08;         //(CLKXM=CLKRM=FSXM=FSRM= 1, FSXP = 1)
    McbspaRegs.SPCR1.bit.DLB = 1;        // DLB mode for testing
    McbspaRegs.SPCR1.bit.CLKSTP = 2;     // Together with CLKXP/CLKRP
                                         // determines clocking scheme
    McbspaRegs.PCR.bit.CLKXP = 0;        // CPOL = 0, CPHA = 0 rising edge
                                         // no delay
    McbspaRegs.PCR.bit.CLKRP = 0;
    McbspaRegs.RCR2.bit.RDATDLY = 01;    // FSX setup time 1 in master mode.
                                         // 0 for slave mode (Receive)
    McbspaRegs.XCR2.bit.XDATDLY = 01;    // FSX setup time 1 in master mode.
                                         // 0 for slave mode (Transmit)

    McbspaRegs.RCR1.bit.RWDLEN1 = 2;     //16-bit word
    McbspaRegs.XCR1.bit.XWDLEN1 = 2;     //16-bit word

    McbspaRegs.SRGR2.all = 0x2000;       // CLKSM=1, FPER = 1 CLKG periods
    McbspaRegs.SRGR1.all = 0x000F;       // Frame Width = 1 CLKG period,
                                         // CLKGDV=16

    McbspaRegs.SPCR2.bit.GRST = 1;       // Enable the sample rate generator
    delay_loop();                        // Wait at least 2 SRG clock cycles
    McbspaRegs.SPCR2.bit.XRST = 1;       // Release TX from Reset
    McbspaRegs.SPCR1.bit.RRST = 1;       // Release RX from Reset
    McbspaRegs.SPCR2.bit.FRST = 1;       // Frame Sync Generator reset
}




//
// start_dma - Start the DMA channels' 1 and 2
//
void start_dma (void)
{
  EALLOW;
  DmaRegs.CH1.CONTROL.bit.RUN = 1;      // Start DMA Transmit from McBSP-A
  DmaRegs.CH2.CONTROL.bit.RUN = 1;      // Start DMA Receive from McBSP-A
  EDIS;
}

//
// local_D_INTCH1_ISR - DMA ISR for channel 1 (INT7.1)
//
__interrupt void local_D_INTCH1_ISR(void)
{
    EALLOW;      // NEED TO EXECUTE EALLOW INSIDE ISR !!!
    DmaRegs.CH1.CONTROL.bit.HALT = 1;

    //
    // To receive more interrupts from this PIE group, acknowledge
    // this interrupt
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
    EDIS;
    return;
}


//
// local_D_INTCH2_ISR - DMA ISR for channel 2 (INT7.2)
//
__interrupt void local_D_INTCH2_ISR(void)
{

    EALLOW;      // NEED TO EXECUTE EALLOW INSIDE ISR !!!
    DmaRegs.CH2.CONTROL.bit.HALT = 1;

    //
    // To receive more interrupts from this PIE group, acknowledge this
    // interrupt
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
    /*
     * Uint16 i;
    for (i=0; i<128; i++)
    {
        if(data_size == 8)
        {
            if((rdata[i]&0x00FF) != (sdata[i]&0x00FF))
            {
              error();  // STOP if there is an error !!
            }
        }
        else if(data_size == 16)
        {
            if (rdata[i] != sdata[i])
            {
              error();  // STOP if there is an error !!
            }
        }
        else if(data_size == 32)
        {
            if ((rdata[i])!=(sdata[i]))
            {
              error ();
            }
        }
   }
   */
   EDIS;

   return;

}



