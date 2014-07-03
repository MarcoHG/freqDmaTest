/*
 * freqDma.h
 *
 *  Created on: Jul 2, 2014
 *      Author: Marco.HenryGin
 */

#ifndef FREQDMA_H_
#define FREQDMA_H_

/*************************************************************************
  *   $INCLUDES
*************************************************************************/
#include "PE_types.h"

/*************************************************************************
  *   $DEFINES
*************************************************************************/

typedef struct
{
	union 
	{
		unsigned short int w[2];
		unsigned int l;
	} utCalcPeriod;

	unsigned short int uNbrEdges;			/* 	Number of Pulse Edges detected in the last overflow isr	*/
	unsigned char fNewFreq;
	
} stDmaFrequency;


/*************************************************************************
  *   $GLOBAL PROTOTYPES
*************************************************************************/

void freqDmaRun(void);		// The application
void timerCaptureIsr(void );


/*************************************************************************
  *   $GLOBAL VARIABLES
*************************************************************************/
extern unsigned int uCapture;			// Test 
extern unsigned int mSSamplePre, timerTicks;
extern unsigned short aDmaCaptureTbl[];
extern LDD_TDeviceData *TimerPtr;
extern uint16 tPeriodIsr;
extern  stDmaFrequency tFreqInput;	

/*************************************************************************
  *   $INLINE FUNCTIONS 
  *************************************************************************/


#endif /* FREQDMA_H_ */
