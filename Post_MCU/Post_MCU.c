#include "F28x_Project.h"
#include "F2837xS_device.h"
#include "F2837xS_Examples.h"
#include "Post_SPI.h"
#include "Post_McBSP_DMA.h"
#include "Post_I2C.h"
#include "Post_High_Level.h"


int Run = 0;




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

	InitMcbspaGpio();          // Initialize GPIO pins for MCBSP DMA

	GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 1);     // For I2C communication
	GPIO_SetupPinMux(33, GPIO_MUX_CPU1, 1);     // For I2C communication

	GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 0);     // For timer interrupt
	GPIO_SetupPinOptions(65, GPIO_OUTPUT, GPIO_PUSHPULL); // For timer interrupt

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
	PieVectTable.I2CA_INT = &i2c_int1a_isr;
	PieVectTable.DMA_CH1_INT = &local_D_INTCH1_ISR;
	PieVectTable.DMA_CH2_INT = &local_D_INTCH2_ISR;
	EDIS;
	// This is needed to disable write to EALLOW protected registers

	//
	// Initialize I2CA
	//
	I2CA_Init();

	//
	// Ensure DMA is connected to PF2SEL bridge (EALLOW protected)
	//
	EALLOW;
	CpuSysRegs.SECMSEL.bit.PF2SEL = 1;
	EDIS;

	//For testing McBSP DMA writing and reading
/*	int i;
	for (i = 0; i < ChunkSizeI; i++) {
		ChunkDMAT[i] = 0x22FF;
		ChunkDMA[i] = 0;
	}
*/
	init_dma();        // 1. When using DMA, initialize DMA with
					   //    peripheral interrupts first.

	start_dma();       // not sure if start DMA should go first
	//mcbsp_init_dlb();      // 2. Then initialize and release peripheral
	init_mcbsp_spi();      //   (McBSP) from Reset.

	//
	// Enable interrupts required for this example
	//
	PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
	PieCtrlRegs.PIEIER7.bit.INTx1 = 1;   // Enable PIE Group 7, INT 1 (DMA CH1)
	PieCtrlRegs.PIEIER7.bit.INTx2 = 1;   // Enable PIE Group 7, INT 2 (DMA CH2)

	IER = 0x40;                            // Enable CPU INT groups 6 and 7

	if (Run > 0) {
		Runtime();

		while (1) {

		}
	}

}


















