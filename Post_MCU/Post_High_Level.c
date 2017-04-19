
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
#include "Post_MCBSP_SPI.h"





#pragma DATA_SECTION(ImageP1, "ramgs0") //Allocate RAM space for storage of all image data, each section contains X 16 bit integers
#pragma DATA_SECTION(ImageP2, "ramgs1")
#pragma DATA_SECTION(ImageP3, "ramgs2")
#pragma DATA_SECTION(ImageP4, "ramgs3")
#pragma DATA_SECTION(ImageP5, "ramgs4")
#pragma DATA_SECTION(ImageP6, "ramgs5")
#pragma DATA_SECTION(ImageP7, "ramgs6")
#pragma DATA_SECTION(ImageP8, "ramgs7")
#pragma DATA_SECTION(ImageP9, "ramgs8")
#pragma DATA_SECTION(ImageP10, "ramgs9")
#pragma DATA_SECTION(ImageP11, "ramgs10")
#pragma DATA_SECTION(ImageP12, "ramgs11")
#pragma DATA_SECTION(ImageP13, "ramgs12")
#pragma DATA_SECTION(ImageP14, "ramgs13")
#pragma DATA_SECTION(ImageP15, "ramgs14")

Uint16 ImageP1[ImagePSize];               // Each array stores a portion of of our total image data in 16 bit integers
Uint16 ImageP2[ImagePSize];
Uint16 ImageP3[ImagePSize];
Uint16 ImageP4[ImagePSize];
Uint16 ImageP5[ImagePSize];
Uint16 ImageP6[ImagePSize];
Uint16 ImageP7[ImagePSize];
Uint16 ImageP8[ImagePSize];
Uint16 ImageP9[ImagePSize];
Uint16 ImageP10[ImagePSize];
Uint16 ImageP11[ImagePSize];
Uint16 ImageP12[ImagePSize];
Uint16 ImageP13[ImagePSize];
Uint16 ImageP14[ImagePSize];
Uint16 ImageP15[ImagePSize];

extern Uint16 intCounter;              // keeps track of the number of 16-bit words that have been transferred from base to post
extern Uint16 globalBrt;                       // Global brightness value received from Base, user programmed
extern Uint16 Refresh;                  //Refresh rate value sent from Base used to calculate timer interrupt value
Uint16 stframe = 0x0000;  		// 32 0 bits signal start of each LED frame, each stframe is 16 bits, must send 2
Uint16 startBlue =0;        	//First 16 bits of an LED frame
Uint16 greenRed;            	//Second 16 bits of an LED frame
char RefreshPacket[792];
char L1L2[264];
char L3L4[264];
char L5L6[264];

Uint16 imageIndex=0;


int rgbDim = 3;		//hardcode? for now.
int stripsPerPort = 2;
int numPorts = 3;
int numPosts = 3;
int LEDupdates = 216;
int  height = 44*2;
int pixelStart =0;


Uint16 updateLength = 44*2*3*3;  //total number of bytes sent out per update

int u = 0;


void StoreImageData(Uint16 rdata)
{
	int index;

	if(intCounter <ImagePSize)
	{
		index=intCounter;
	ImageP1[index]=rdata;
	}

	if(intCounter <2*ImagePSize)
	{
		index=intCounter-ImagePSize;
	ImageP2[index]=rdata;
	}

	if(intCounter <3*ImagePSize)
	{
		index=intCounter-2*ImagePSize;
	ImageP3[index]=rdata;
	}
}

void Runtime()
{


	pixelStart = 0b11100000 | globalBrt;   //define start byte for each LED frame

	DINT;

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




	}

}


void formatLEDChunk ()
{
	if(imageIndex)

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
	getLEDChunk(startL1L2, endL1L2,startL3L4, endL3L4, startL5L6, endL5L6, ImageP1);

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


