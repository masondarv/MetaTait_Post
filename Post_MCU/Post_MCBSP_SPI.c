/*
 * Post_MCBSP_SPI.c
 *
 *  Created on: Apr 16, 2017
 *      Author: Mason
 */

#include "F28x_Project.h"
#include "Post_MCBSP_SPI.h"



void init_mcbsp_spi()
{
    //
    // McBSP-A register settings
    //
    McbspbRegs.SPCR2.all = 0x0000;       // Reset FS generator, sample rate
                                         // generator & transmitter
    McbspbRegs.SPCR1.all = 0x0000;       // Reset Receiver, Right justify word,
                                         // Digital loopback dis.
    McbspbRegs.PCR.all = 0x0F08;         //(CLKXM=CLKRM=FSXM=FSRM= 1, FSXP = 1)
    McbspbRegs.SPCR1.bit.DLB = 1;
    McbspbRegs.SPCR1.bit.CLKSTP = 2;     // Together with CLKXP/CLKRP
                                         // determines clocking scheme
    McbspbRegs.PCR.bit.CLKXP = 0;        // CPOL = 0, CPHA = 0 rising edge
                                         // no delay
    McbspbRegs.PCR.bit.CLKRP = 0;
    McbspbRegs.RCR2.bit.RDATDLY = 01;    // FSX setup time 1 in master mode.
                                         // 0 for slave mode (Receive)
    McbspbRegs.XCR2.bit.XDATDLY = 01;    // FSX setup time 1 in master mode.
                                         // 0 for slave mode (Transmit)

    McbspbRegs.RCR1.bit.RWDLEN1 = 2;     // 16-bit read
    McbspbRegs.XCR1.bit.XWDLEN1 = 2;     // 16-bit write

    McbspbRegs.SRGR2.all = 0x2000;       // CLKSM=1, FPER = 1 CLKG periods
    McbspbRegs.SRGR1.all = 0x000F;       // Frame Width = 1 CLKG period,
                                         // CLKGDV=16

    McbspbRegs.SPCR2.bit.GRST = 1;       // Enable the sample rate generator
    delay_loop();                        // Wait at least 2 SRG clock cycles
    McbspbRegs.SPCR2.bit.XRST = 1;       // Release TX from Reset
    McbspbRegs.SPCR1.bit.RRST = 1;       // Release RX from Reset
    McbspbRegs.SPCR2.bit.FRST = 1;       // Frame Sync Generator reset

    McbspbRegs.SPCR1.bit.DLB = 0;
}


void InitMcbspbGpio(void)
{
#ifdef CPU1
    EALLOW;

    //
    // This specifies which of the possible GPIO pins will be
    // McBSPB functional pins. Comment out unwanted connections.
    // Set qualification for selected input pins to asynchronous only
    // This will select asynchronous (no qualification) for the selected pins.
    //

    //
    // Select one of the following for MDXB
    // GPIO24
    // GPIO84
    //
    //GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 3;
   // GpioCtrlRegs.GPAGMUX1.bit.GPIO12 = 3;
    GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 3;

    //
    // MDRB
    // GPIO13 with asynchronous qualification
    // GPIO25 with asynchronous qualification
    // GPIO85 with asynchronous qualification
    //
    GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 3;
    GpioCtrlRegs.GPAQSEL1.bit.GPIO13 = 3;
    //GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 3;
    //GpioCtrlRegs.GPAQSEL2.bit.GPIO25 = 3;
    //GpioCtrlRegs.GPCGMUX2.bit.GPIO85 = 1;
    //GpioCtrlRegs.GPCMUX2.bit.GPIO85 = 2;
    //GpioCtrlRegs.GPCQSEL2.bit.GPIO85 = 3;

    //
    // MCLKXB
    // GPIO14 with asynchronous qualification
    // GPIO26 with asynchronous qualification
    // GPIO86 with asynchronous qualification
    //
    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 3;
    GpioCtrlRegs.GPAQSEL1.bit.GPIO14 = 3;
    //GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 3;
    //GpioCtrlRegs.GPAQSEL2.bit.GPIO26 = 3;
    //GpioCtrlRegs.GPCGMUX2.bit.GPIO86 = 1;
    //GpioCtrlRegs.GPCMUX2.bit.GPIO86 = 2;
    //GpioCtrlRegs.GPCQSEL2.bit.GPIO86= 3;

    //
    // MCLKRB
    // Select one of the following
    // GPIO3 with asynchronous qualification
    // GPIO60 with asynchronous qualification
    //
    //GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 3;
    //GpioCtrlRegs.GPAQSEL1.bit.GPIO3 = 3;
    GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 1;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO60 = 3;

    //
    // MFSXB
    // GPIO15 with asynchronous qualification
    // GPIO27 with asynchronous qualification
    // GPIO87 with asynchronous qualification
    //
    GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 3;
    GpioCtrlRegs.GPAQSEL1.bit.GPIO15 = 3;
    //GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 3;
    //GpioCtrlRegs.GPAQSEL2.bit.GPIO27 = 3;
    //GpioCtrlRegs.GPCGMUX2.bit.GPIO87 = 1;
    //GpioCtrlRegs.GPCMUX2.bit.GPIO87 = 2;
    //GpioCtrlRegs.GPCQSEL2.bit.GPIO87= 3;

    //
    // MFSRB
    // Select one of the following
    // GPIO1 with asynchronous qualification
    // GPIO61 with asynchronous qualification
    //
    //GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 3;
    //GpioCtrlRegs.GPAQSEL1.bit.GPIO1 = 3;
    GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 1;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO61 = 3;

    EDIS;

#endif
}

void delay_loop(void)
{
    long i;
    for (i = 0; i < MCBSP_INIT_DELAY; i++) {}
}

//
// clkg_delay_loop - Delay function (at least 2 CLKG cycles)
//                   Required in McBSP init
//
void clkg_delay_loop(void)
{
    long i;
    for (i = 0; i < MCBSP_CLKG_DELAY; i++) {}
}


void mcbsp_xmit(int a) /*int b);*/
{
    //McbspaRegs.DXR2.all = b;
    McbspbRegs.DXR1.all = a;
}


