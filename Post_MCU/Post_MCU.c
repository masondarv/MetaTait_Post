#include "F28x_Project.h"
#include "F2837xS_device.h"
#include "F2837xS_Examples.h"
#include "Post_SPI.h"
#include "Post_High_Level.h"
#include "Post_MCBSP_SPI.h"



Uint16 globalBrt= 0;                       // Global brightness value received from Base, user programmed
Uint16 Refresh = 0;                  //Refresh rate value sent from Base used to calculate timer interrupt value
Uint16 rdata = 0;                    // temp variable for storing recieved data from base
Uint16 intCounter = 0;              // keeps track of the number of 16-bit words that have been transferred from base to post
Uint16 imageIntCount = 60000;
Uint16 sdata =0xF0F0;



int main(void)
{

	//
	// Initialize System Control:
	// PLL, WatchDog, enable Peripheral Clocks
	// This example function is found in the F2837xS_SysCtrl.c file.
	//

	InitSysCtrl();//Initialize System Control, Where you can change PLL system clock

	InitGpio();

	//
	// Initialize GPIO:
	// These example functions are found in the F2837xS_Gpio.c file and
	// illustrates how to set the GPIO to it's default state.
	// Setup only the GP I/O only for McBSP-A functionality
	//

	InitSpiGpio();				//Initialize GPIO pins for SPI Communication

	InitMcbspbGpio();          // Initialize GPIO pins for MCBSP DMA


	//
	// Clear all interrupts and initialize PIE vector table:
	// Disable CPU interrupts
	//

	DINT;
	//Clear all interrupts

	//
	// Initialize PIE control registers to their default state.
	// The default state is all PIE interrupts disabled and flags
	// are cleared.
	// This function is found in the F2837xS_PieCtrl.c file.
	//

	InitPieCtrl();

	IER = 0x0000;				//Disable CPU interrupts
	IFR = 0x0000;				//Clear interrupt flags

	//
	// Initialize the PIE vector table with pointers to the shell Interrupt
	// Service Routines (ISR).
	// This will populate the entire table, even if the interrupt
	// is not used in this example.  This is useful for debug purposes.
	// The shell ISR routines are found in F2837xS_DefaultIsr.c.
	// This function is found in F2837xS_PieVect.c.
	//

	InitPieVectTable();
	spi_fifo_init();     		// Initialize the SPI FIFO,

	// Interrupts that are used in this example are re-mapped to
	// ISR functions found within this file.
	//  Write address of I2C ISR and DMA ISRs into PieVectTable register

	EALLOW;
	// This is needed to write to EALLOW protected registers
	PieVectTable.TIMER0_INT = &cpu_timer0_isr;
	EDIS;
	// This is needed to disable write to EALLOW protected registers


	init_mcbsp_spi();      //   (McBSP) from Reset.

	//
	// Enable interrupts required for this example
	//
	PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block

	 IER |= M_INT1;

	 PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

	 EINT;  // Enable Global interrupt INTM
	 ERTM;  // Enable Global realtime interrupt DBGM



while(1)
{

//	mcbsp_xmit(sdata);
//	DELAY_US(300);

	 while( McbspbRegs.SPCR1.bit.RRDY == 0 ) {}
	 rdata = McbspaRegs.DRR1.all;
	 if(intCounter<imageIntCount)
	 {
		 StoreImageData(rdata);
		 intCounter++;
	 }
	 if(intCounter==imageIntCount)
	 {
		 globalBrt=rdata;
		 intCounter++;
	 }
	 if(intCounter==imageIntCount+1)
	 {

		 Refresh=rdata;
		 intCounter++;
		 Runtime();

	 }
	 if(intCounter>imageIntCount+1)
	 {
		 Refresh = rdata;
		 DINT;
		ConfigCpuTimer(&CpuTimer0, 200, Refresh);
		CpuTimer1Regs.TCR.all = 0x4000;
		IER |= M_INT1;
		EINT;

	 }






}


}
















