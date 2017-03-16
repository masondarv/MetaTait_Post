/*
 * Post_High_Level.c
 *
 *  Created on: Mar 14, 2017
 *      Author: Mason
 */




#include "F28x_Project.h"
#include "F2837xS_device.h"
#include "F2837xS_Examples.h"
#include "Post_High_Level.h"
#include "Post_SPI.h"
#include "Post_McBSP_DMA.h"




#pragma DATA_SECTION(ChunkDMA, "ramgs0") //Place LED Chunkdata in DMA-accessible RAM
#pragma DATA_SECTION(ChunkDMAT, "ramgs1")
#pragma DATA_SECTION(ChunkTMP, "ramgs2")

Uint16 ChunkDMA[ChunkSizeI];                // data array which DMA stores FRAM data
Uint16 ChunkTMP[ChunkSizeI];             //data array which is used to send data out to strips, ChunkDMA will be copied into ChunkTMP
Uint16 ChunkDMAT[ChunkSizeI];            //array used to test MCBSP DMA code
//char   ChunkTMPB[6336];
unsigned long chunk1Addr;            //address of the first third of data in FRAM
unsigned long chunk2Addr;            //address of the first third of data in FRAM
unsigned long chunk3Addr;            //address of the third third of data in FRAM


Uint16 globalBrt= 0;                       // Global brightness value received from Base, user programmed
Uint16 Refresh = 0;                  //Refresh rate value sent from Base used to calculate timer interrupt value
Uint16 stframe = 0x0000;  	// 32 0 bits signal start of each LED frame, each stframe is 16 bits, must send 2
Uint16 startBlue =0;        //First 16 bits of an LED frame
Uint16 greenRed;            //Second 16 bits of an LED frame
char L1L2[264];
char L3L4[264];
char L5L6[264];


int rgbDim = 3;		//hardcode? for now.
int stripsPerPort = 2;
int numPorts = 3;
int numPosts = 3;
int LEDupdates = 216;
int  height = 44*2;
int pixelStart =0;
int DMAupdates =StreamSize/ChunkSizeB;

Uint16 updateLength = 44*2*3*3;  //total number of bytes sent out per update

int u = 0;


void Runtime() {

	//run initial DMA transfer

	pixelStart = 0b11100000 | globalBrt;   //define start byte for each LED frame

	//use Refresh value to configure timer interrupt
	ConfigCpuTimer(&CpuTimer0, 200, Refresh);
	CpuTimer0Regs.TCR.all = 0x4000;// set up timer interrupt

	IER |= M_INT1;  //Enable timer0 interrupt

	//
	// Enable TINT0 in the PIE: Group 1 interrupt 7
	//
	    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

	    EINT;  // Enable Global interrupt INTM
	    ERTM;  // Enable Global realtime interrupt DBGM

	while(u<LEDupdates) {

		if (u==0||(u+1) %4==0)
			// get new chunk and initiaite DMA transfer
			 switchDataChunk(u);  // may have to include specific size of chunk



	}

}


void switchDataChunk(int u)
{
				  //mcbsp_xmit to FRAM to determine which part of FRAM to pull from
	start_dma();  //pull chunk of data from FRAM into ChunkDMA
	DELAY_US(30);
	int i;
	int k = 0;
	for (i=0;i<ChunkSizeI;i++)
	{
		ChunkTMP[k]=ChunkDMAT[i] && 0x00FF;
		ChunkTMP[k+1] = ChunkDMAT[i] && 0xFF00;
		k = k+2;

	}



}

void getLEDChunk(Uint16 startL1L2, Uint16 endL1L2, Uint16 startL3L4, Uint16 endL3L4, Uint16 startL5L6, Uint16 endL5L6, Uint16* data)
{

	//char stripData[264];   //initialize array to hold LED strip data
	int i = 0;
	int k = 0;

	for (i=startL1L2; i<=endL1L2; i++)
	{
		L1L2[k] = data[i];
		k++;
	}
	k=0;
	for (i=startL3L4; i<=endL3L4; i++)
	{
			L3L4[k] = data[i];
			k++;
		}
	k=0;
	for (i=startL5L6; i<=endL5L6; i++)
	{
			L5L6[k] = data[i];
			k++;
		}
	k=0;

}

//
// cpu_timer0_isr - CPU Timer0 ISR with interrupt counter
//
__interrupt void cpu_timer0_isr(void)
{
   CpuTimer0.InterruptCount++;
	// Get
    //
    //
	Uint16 i;
		    Uint16 index = updateLength*u;
		    Uint16 startL1L2 = index;
			Uint16 endL1L2 = index+height*rgbDim-1;
			Uint16 startL3L4 = index + height*rgbDim;
			Uint16 endL3L4 = index + 2*height*rgbDim - 1;
			Uint16 startL5L6 = index + 2*height*rgbDim;
			Uint16 endL5L6 = index+3*height*rgbDim - 1;


		// extract corresponding data for LED strip pairs
		getLEDChunk(startL1L2, endL1L2,startL3L4, endL3L4, startL5L6, endL5L6, ChunkTMP);

		// transmit Strip Start Packets (32-bits)
	    spi_xmita(stframe);
	    spi_xmita(stframe);
	 	 DELAY_US(3);

	   	spi_xmitb(stframe);
	   	spi_xmitb(stframe);
	    DELAY_US(3);

	   	spi_xmitc(stframe);
	   	spi_xmitc(stframe);
	   	DELAY_US(3);

		// transmit pixel data
		for(i=0; i<height*rgbDim; i+=rgbDim)
		{
			 startBlue = ( pixelStart << 8 ) | L1L2[i];    //pixel start (MSB) and blue byte (LSB)
			 greenRed = ( L1L2[i+1] << 8 ) | L1L2[i+2];  //green byte (MSB) and red byte (LSB)
			spi_xmita(startBlue);
	    	spi_xmita(greenRed);
	    	DELAY_US(3);

			startBlue = ( pixelStart << 8 ) | L3L4[i];    //pixel start (MSB) and blue byte (LSB)
		    greenRed = ( L3L4[i+1] << 8 ) | L3L4[i+2]; //green byte (MSB) and red byte (LSB)
			spi_xmitb(startBlue);
			spi_xmitb(greenRed);
			DELAY_US(3);

			Uint16 startBlue = ( pixelStart << 8 ) | L5L6[i];    //pixel start (MSB) and blue byte (LSB)
			Uint16 greenRed = ( L5L6[i+1] << 8 ) | L5L6[i+2];  //green byte (MSB) and red byte (LSB)
			spi_xmitc(startBlue);
			spi_xmitc(greenRed);
			DELAY_US(3);
		}

		u=(u+1)%LEDupdates;
	////  END STUFF IN ISR

   //
   // Acknowledge this interrupt to receive more interrupts from group 1
   //
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//
// init_dma - DMA Initialization for data size <= 16-bit
//
void init_dma()
{
  EALLOW;
  DmaRegs.DMACTRL.bit.HARDRESET = 1;
  __asm(" NOP");                        // Only 1 NOP needed per Design

  DmaRegs.CH1.MODE.bit.CHINTE = 0;
  // Channel 1, McBSPA transmit
  DmaRegs.CH1.BURST_SIZE.all = 0;       // 1 word/burst
  DmaRegs.CH1.SRC_BURST_STEP = 0;       // no effect when using 1 word/burst
  DmaRegs.CH1.DST_BURST_STEP = 0;       // no effect when using 1 word/burst
  DmaRegs.CH1.TRANSFER_SIZE = ChunkSizeI -1;      // Interrupt every frame
                                        // (3167 bursts/transfer)
  DmaRegs.CH1.SRC_TRANSFER_STEP = 1;    // Move to next word in buffer after
                                        // each word in a burst
  DmaRegs.CH1.DST_TRANSFER_STEP = 0;    // Don't move destination address
  DmaRegs.CH1.SRC_ADDR_SHADOW = (Uint32) &ChunkDMAT[0];   // Start address = buffer
  DmaRegs.CH1.SRC_BEG_ADDR_SHADOW = (Uint32) &ChunkDMAT[0];  // Not needed unless
                                                         // using wrap function
  DmaRegs.CH1.DST_ADDR_SHADOW = (Uint32) &McbspaRegs.DXR1.all; // Start address
                                                               // = McBSPA DXR
  //
  // Not needed unless using wrap function
  //
  DmaRegs.CH1.DST_BEG_ADDR_SHADOW = (Uint32) &McbspaRegs.DXR1.all;
  DmaRegs.CH1.CONTROL.bit.PERINTCLR = 1;   // Clear peripheral interrupt event
                                           // flag.
  DmaRegs.CH1.CONTROL.bit.ERRCLR = 1;      // Clear sync error flag
  DmaRegs.CH1.DST_WRAP_SIZE = 0xFFFF;      // Put to maximum - don't want
                                           // destination wrap.
  DmaRegs.CH1.SRC_WRAP_SIZE = 0xFFFF;      // Put to maximum - don't want
                                           // source wrap.
  DmaRegs.CH1.MODE.bit.CHINTE = 1;         // Enable channel interrupt
  DmaRegs.CH1.MODE.bit.CHINTMODE = 1;      // Interrupt at end of transfer
  DmaRegs.CH1.MODE.bit.PERINTE = 1;        // Enable peripheral interrupt event
  DmaRegs.CH1.MODE.bit.PERINTSEL = 1;      // Peripheral interrupt select =
                                           // McBSP MXSYNCA
  DmaClaSrcSelRegs.DMACHSRCSEL1.bit.CH1 = DMA_MXEVTA;
  DmaRegs.CH1.CONTROL.bit.PERINTCLR = 1;   // Clear any spurious interrupt flags

  //
  // Channel 2, McBSPA Receive
  //
  DmaRegs.CH2.MODE.bit.CHINTE = 0;
  DmaRegs.CH2.BURST_SIZE.all = 0;        // 1 word/burst
  DmaRegs.CH2.SRC_BURST_STEP = 0;        // no effect when using 1 word/burst
  DmaRegs.CH2.DST_BURST_STEP = 0;        // no effect when using 1 word/burst
  DmaRegs.CH2.TRANSFER_SIZE = ChunkSizeI -1;       // Interrupt every 3167 bursts/transfer
  DmaRegs.CH2.SRC_TRANSFER_STEP = 0;     // Don't move source address
  DmaRegs.CH2.DST_TRANSFER_STEP = 1;     // Move to next word in buffer after
                                         // each word in a burst
  DmaRegs.CH2.SRC_ADDR_SHADOW = (Uint32) &McbspaRegs.DRR1.all; // Start address
                                                               // = McBSPA DRR
  //
  // Not needed unless using wrap function
  //
  DmaRegs.CH2.SRC_BEG_ADDR_SHADOW = (Uint32) &McbspaRegs.DRR1.all;
  DmaRegs.CH2.DST_ADDR_SHADOW = (Uint32) &ChunkDMA[0];      // Start address =
                                                         // Receive buffer
                                                         // (for McBSP-A)
  DmaRegs.CH2.DST_BEG_ADDR_SHADOW = (Uint32) &ChunkDMA[0];  // Not needed unless
                                                         // using wrap function
  DmaRegs.CH2.CONTROL.bit.PERINTCLR = 1; // Clear peripheral interrupt event
                                         // flag.
  DmaRegs.CH2.CONTROL.bit.ERRCLR = 1;    // Clear sync error flag
  DmaRegs.CH2.DST_WRAP_SIZE = 0xFFFF;    // Put to maximum - don't want
                                         // destination wrap.
  DmaRegs.CH2.SRC_WRAP_SIZE = 0xFFFF;    // Put to maximum - don't want
                                         // source wrap.
  DmaRegs.CH2.MODE.bit.CHINTE = 1;       // Enable channel interrupt
  DmaRegs.CH2.MODE.bit.CHINTMODE = 1;    // Interrupt at end of transfer
  DmaRegs.CH2.MODE.bit.PERINTE = 1;      // Enable peripheral interrupt event
  DmaRegs.CH2.MODE.bit.PERINTSEL = 2;    // Peripheral interrupt select =
                                         // McBSP MRSYNCA
  DmaClaSrcSelRegs.DMACHSRCSEL1.bit.CH2 = DMA_MREVTA;
  DmaRegs.CH2.CONTROL.bit.PERINTCLR = 1; // Clear any spurious interrupt flags
  EDIS;
}

