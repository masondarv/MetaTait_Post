/*
 * Post_SPI.c
 *
 *  Created on: Mar 14, 2017
 *      Author: Mason
 */

#include "F28x_Project.h"
#include "F2837xS_device.h"
#include "F2837xS_Examples.h"
#include "Post_SPI.h"




//
// Calculate BRR: 7-bit baud rate register value
// SPI CLK freq = 500 kHz
// LSPCLK freq  = CPU freq / 4  (by default)
// BRR          = (LSPCLK freq / SPI CLK freq) - 1
//
#if CPU_FRQ_200MHZ
#define SPI_BRR     3   //((200E6 / 4) / 350E3) - 1
#endif

#if CPU_FRQ_150MHZ
#define SPI_BRR        ((150E6 / 4) / 500E3) - 1
#endif

#if CPU_FRQ_120MHZ
#define SPI_BRR        ((120E6 / 4) / 500E3) - 1
#endif

//
// InitSPI - This function initializes the SPI to a known state
//
void InitSpi(void)
{
    // Initialize High Speed SPI A, B, and C

	ClkCfgRegs.LOSPCP.bit.LSPCLKDIV = 0;  // Set LSPCLK equal to SYSCLK


    // Set reset low before configuration changes
    // Clock polarity (0 == rising, 1 == falling)
    // 16-bit character
    // Enable loop-back
	// Enable High Speed SPI Enhancements
    SpiaRegs.SPICCR.bit.SPISWRESET = 0;
    SpiaRegs.SPICCR.bit.CLKPOLARITY = 0;
    SpiaRegs.SPICCR.bit.SPICHAR = (16-1);
    SpiaRegs.SPICCR.bit.SPILBK = 1;
    SpiaRegs.SPICCR.bit.HS_MODE = 1;


    // Enable master (0 == slave, 1 == master)
    // Enable transmission (Talk)
    // Clock phase (0 == normal, 1 == delayed)
    // SPI interrupts are disabled
    SpiaRegs.SPICTL.bit.MASTER_SLAVE = 1;
    SpiaRegs.SPICTL.bit.TALK = 1;
    SpiaRegs.SPICTL.bit.CLK_PHASE = 0;
    SpiaRegs.SPICTL.bit.SPIINTENA = 0;

    // Set the baud rate
    SpiaRegs.SPIBRR.bit.SPI_BIT_RATE = SPI_BRR;

    // Set FREE bit
    // Halting on a breakpoint will not halt the SPI
    SpiaRegs.SPIPRI.bit.FREE = 1;

    // Release the SPI from reset
    SpiaRegs.SPICCR.bit.SPISWRESET = 1;
}

//
// InitSpiGpio - This function initializes GPIO pins to function as SPI pins.
//               Each GPIO pin can be configured as a GPIO pin or up to 3
//               different peripheral functional pins. By default all pins come
//               up as GPIO inputs after reset.
//
//               Caution:
//               For each SPI peripheral
//               Only one GPIO pin should be enabled for SPISOMO operation.
//               Only one GPIO pin should be enabled for SPISOMI operation.
//               Only one GPIO pin should be enabled for SPICLK  operation.
//               Only one GPIO pin should be enabled for SPISTE  operation.
//               Comment out other unwanted lines.
//


// Initialize GPIO pins for all 3 High Speed SPI outputs
void InitSpiGpio()
{
   EALLOW;

    //
    // Enable internal pull-up for the selected pins
    //
    // Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //

  /* GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;  // Enable pull-up on GPIO16 (SPISIMOA)
   GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;  // Enable pull-up on GPIO18 (SPICLKA)
   GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;
   GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;
  */
  // High speed SPI
    GpioCtrlRegs.GPBPUD.bit.GPIO58 = 0;  // Enable pull-up on GPIO58 (SPISIMOA)
    GpioCtrlRegs.GPBPUD.bit.GPIO60 = 0;   // Enable pull-up on GPIO60 (SPICLKA)
    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0;  // Enable pull-up on GPIO63 (SPISIMOB)
    GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;   // Enable pull-up on GPIO65 (SPICLKB)
    GpioCtrlRegs.GPCPUD.bit.GPIO69 = 0;  // Enable pull-up on GPIO18 (SPISIMOC)
    GpioCtrlRegs.GPCPUD.bit.GPIO71 = 0;  // Enable pull-up on GPIO19 (SPICLKC)

    //
    // Set qualification for selected pins to asynch only
    //
    // This will select asynch (no qualification) for the selected pins.
    // Comment out other unwanted lines.
    //

   // Regular SPI
/*   GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 3;  //Asynch input GPIO16 (SPISIMOA)
   GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = 3;  //Asynch input GPIO18 (SPICLKA)
   GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 3;
   GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = 3;
 */
   //  High Speed SPI
    GpioCtrlRegs.GPBQSEL2.bit.GPIO58 = 3; // Asynch input GPIO58 (SPISIMOA)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO60 = 3;  // Asynch input GPIO60 (SPICLKA)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3; // Asynch input GPIO63 (SPISIMOB)
    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3;  // Asynch input GPIO65 (SPICLKB)
    GpioCtrlRegs.GPCQSEL1.bit.GPIO69 = 3; // Asynch input GPIO69 (SPISIMOC)
    GpioCtrlRegs.GPCQSEL1.bit.GPIO71 = 3; // Asynch input GPIO71 (SPICLKC)


    //
    //Configure High Speed SPI-A, B and C pins using GPIO regs
    //
    // This specifies which of the possible GPIO pins will be SPI functional
    // pins.
    //

   // Regular SPI
 /*GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 1;  // Configure GPIO16 as SPISIMOA
   GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 1;  // Configure GPIO18 as SPICLKA
   GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 1;
   GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 1;
   */
  // High Speed SPI
    GpioCtrlRegs.GPBGMUX2.bit.GPIO58 = 3; // Configure GPIO58 as SPISIMOA
    GpioCtrlRegs.GPBGMUX2.bit.GPIO60 = 3; // Configure GPIO60 as SPICLKA
    GpioCtrlRegs.GPBGMUX2.bit.GPIO63 = 3; // Configure GPIO63 as SPISIMOB
    GpioCtrlRegs.GPCGMUX1.bit.GPIO65 = 3; // Configure GPIO65 as SPICLKB
    GpioCtrlRegs.GPCGMUX1.bit.GPIO69 = 3; // Configure GPIO69 as SPISOMOC
    GpioCtrlRegs.GPCGMUX1.bit.GPIO71 = 3; // Configure GPIO71 as SPICLKC

    GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 3; // Configure GPIO58 as SPISIMOA
    GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 3; // Configure GPIO60 as SPICLKA
    GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 3; // Configure GPIO63 as SPISIMOB
    GpioCtrlRegs.GPCMUX1.bit.GPIO65 = 3; // Configure GPIO65 as SPICLKB
    GpioCtrlRegs.GPCMUX1.bit.GPIO69 = 3; // Configure GPIO69 as SPISOMOC
    GpioCtrlRegs.GPCMUX1.bit.GPIO71 = 3; // Configure GPIO71 as SPICLKC

    EDIS;
}




void spi_xmita(Uint16 a)		//Transmit via SPIA
{
   SpiaRegs.SPITXBUF = a;
}

void spi_xmitb(Uint16 a)		//Transmit via SPIB
{
   SpiaRegs.SPITXBUF = a;
}

void spi_xmitc(Uint16 a)		//Transmit via SPIC
{
   SpiaRegs.SPITXBUF = a;
}

void spi_fifo_init()
{
   SpiaRegs.SPIFFTX.all = 0xE040;	//Initialize SPIA FIFO Registers
   SpiaRegs.SPIFFRX.all = 0x2044;
   SpiaRegs.SPIFFCT.all = 0x0;

   SpibRegs.SPIFFTX.all = 0xE040;	//Initialize SPIB FIFO Registers
   SpibRegs.SPIFFRX.all = 0x2044;
   SpibRegs.SPIFFCT.all = 0x0;

   SpicRegs.SPIFFTX.all = 0xE040;	//Initialize SPIC FIFO Registers
   SpicRegs.SPIFFRX.all = 0x2044;
   SpicRegs.SPIFFCT.all = 0x0;

   InitSpi();	//Initialize Core SPI Registers
}

//
// End of file
//
