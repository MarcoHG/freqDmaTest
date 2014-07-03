/*
 * FreqDma.c
 *
 *  Created on: Jul 2, 2014
 *      Author: Marco.HenryGin
 */
//==============================================================================
//  INCLUDES
//==============================================================================
// Typical PEX include
#include "Cpu.h"
#include "Events.h"
#include "CsIO1.h"
#include "freqDma.h"
#include "IO1.h"
#include "TCAP.h"
#include "TP3.h"
// #include "TINT1.h"
#include "DMA0.h"
/* Including shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include <PE_Types.h>
// Libraries
#include <stdio.h>
#include <string.h>
//==============================================================================
//  LOCAL DEFINES
//==============================================================================
#define NBR_DMA_CAPTURE_SAMPLES 100	/* 100 */
//	DMA samples are move to a circular array with indexes 0,1,.. NBR_DMA_CAPTURE_SAMPLES-1
#define DMA_PREV_POS_INDEX(INDX,PPOS) ((int16)INDX-PPOS >= 0 ?  INDX - PPOS : NBR_DMA_CAPTURE_SAMPLES - (PPOS - INDX)  )
#define NO_FREQ_DETECT_OVF_COUNTER_PRE	80 /* Time = 43 * 65536/1,125,000  ~ 2.5 Sec */
#define CAPTURE_CLOCK_HZ 1125000	// 1121549   /* Ideally should be 1.125MHz */
//==============================================================================
//  LOCAL PROTOTYPES.
//==============================================================================


//==============================================================================
//  GLOBAL DATA
//==============================================================================
unsigned int uCapture=0;
LDD_TDeviceData *TP3_ptr = NULL, *TP4_ptr=NULL;
LDD_TDeviceData *TimerPtr = NULL;
unsigned int timerTicks, ulPeriod;
unsigned short uNbrRestarts, uNbrOvfNoPulsePoll, aDmaCaptureTbl[NBR_DMA_CAPTURE_SAMPLES];
uint16 tPeriodIsr;
uint16 uMswDelta, uMswIndexSampled;
bool bDisplay = FALSE;
stDmaFrequency tFreqInput;
//==============================================================================
//  LOCAL DATA
//==============================================================================
uint32 uMeasPeriod;

bool fRefreshDisplay;

const LDD_DMA_TTransferDescriptor DMA_TRANSFER_DESC_NULL = {
  /* UserDataPtr = */                  NULL,
  /* SourceAddress = */                (LDD_DMA_TAddress)0,
  /* SourceAddressOffset = */          (LDD_DMA_TAddressOffset)0,
  /* SourceTransferSize = */           (LDD_DMA_TTransferSize)0,
  /* SourceModuloSize = */             (LDD_DMA_TModuloSize)0,
  /* DestinationAddress = */           (LDD_DMA_TAddress)0,
  /* DestinationAddressOffset = */     (LDD_DMA_TAddressOffset)0,
  /* DestinationTransferSize = */      0,
  /* DestinationModuloSize = */        (LDD_DMA_TModuloSize)0,
  /* TransferMode = */                 LDD_DMA_SINGLE_TRANSFER,
  /* ByteCount = */                    (LDD_DMA_TByteCount)0,
  /* OuterLoopCount = */               (LDD_DMA_TOuterLoopCount)0,
  /* InnerLoopChannelLink = */         FALSE,
  /* InnerLoopLinkedChannel = */       0,
  /* OuterLoopChannelLink = */         FALSE,
  /* OuterLoopLinkedChannel = */       0,
  /* AfterRequestComplete = */         (LDD_DMA_TAfterRequest)LDD_DMA_NO_ACTION,
  /* AddressOffset = */                (LDD_DMA_TAddressOffset)0,
  /* AfterTransferComplete = */        (LDD_DMA_TAfterTransfer)LDD_DMA_NO_ACTION,
  /* SourceAddressAdjustment = */      (LDD_DMA_TAddressOffset)0,
  /* DestinationAddressAdjustment = */ (LDD_DMA_TAddressOffset)0,
  /* ScatterGatherAddress = */         (LDD_DMA_TAddress)0,
  /* BandwidthControl = */             (LDD_DMA_TBandwidthControl)DMA_PDD_NO_STALL,
  /* ChannelAutoSelection = */         TRUE,
  /* ChannelNumber = */                (LDD_DMA_TChannelNumber)0,
  /* TriggerType = */                  LDD_DMA_SW_TRIGGER,
  /* TriggerSource = */                (LDD_DMA_TTriggerSource)0,
  /* PeriodicTrigger = */              FALSE,
  /* DisableAfterRequest = */          FALSE,
  /* Interrupts = */                   FALSE,
  /* OnComplete = */                   FALSE,
  /* OnHalfComplete = */               FALSE,
  /* OnError = */                      FALSE,
  /* OnCompleteEventPtr = */           NULL,
  /* OnErrorEventPtr = */              NULL,
  /* ChannelEnabled = */               FALSE 
};


//==============================================================================
// FUNCTIONS
//==============================================================================



void TransferComplete(LDD_TUserData *UserData)
{
  volatile bool *Completed = (volatile bool*)UserData;
  *Completed = TRUE;
  
  
}

void clearTable()
{
	memset(aDmaCaptureTbl, 0,sizeof(aDmaCaptureTbl));	// Empty data buffer
}

/*!
 * Called every timer capture overflow
 * 
 * The function counts the number of Overflows which indeed reflects the upper 16-bit of the capture timer.
 * 
 * The current DMA index is compared with the one in the previous ovf, any change represents the pulses that 
 * have been arrived since then.
 * 
 * 
 *  
 * the amount of carry overs elpased on the capture timer counter  
 */
void timerCaptureIsr(void)
{
	bool fOvfHalf = FALSE, bCatch = FALSE;
	uint16 				uCiter, dmaTblIndex;	// , lTable[NBR_DMA_CAPTURE_SAMPLES];
	static uint16 dmaTblIndexPrev, uNbrNoPulses =0;
								
	static uint16 uMswCapTmr=0;
	static uint32 uLastCapture;					// Last captured period
	static bool bNoPulseIsr= TRUE;			// Sync when Freq lost
	uint16 uNbrEdges;
	TP3_SetVal(TimerPtr); 
	// Simulate an error
	for(uCiter =0; uCiter<100; ++uCiter);	// Spend some time here 
	
	// Prepare next Interrupt - we avoid getting captures close to 0xFFFF-0x0000 when in the interrupt
	if(FTM0_CNT > 0x7FFF )
		FTM0_C0V = 0xFFFF;// 0xFFE0;									//	We just crossed 0x7F00
	else
	{
		fOvfHalf = TRUE;										// 	We just Crossed 0xFFE0
		FTM0_C0V = 0x7FFF;									//	Need to have DMA ready before OVrs
	}
	
	// 
	uCiter = DMA_TCD0_CITER_ELINKNO;			// Get the current DMA index, and use this value all over the routine
		
	// Translate DMA major loop iteration counter (CITR) to index in the destination table 
	dmaTblIndex = NBR_DMA_CAPTURE_SAMPLES - uCiter;	// Points to the position in the table that DMA is going to write in
		
	// Get the Number of Pulse Edges between this and last OverFlow
	uNbrEdges = dmaTblIndex >= dmaTblIndexPrev ?  dmaTblIndex - dmaTblIndexPrev : dmaTblIndex + (NBR_DMA_CAPTURE_SAMPLES - dmaTblIndexPrev);
	dmaTblIndexPrev = dmaTblIndex;	// for next detection
	
	// The uNbrEdges indicates the amount of captures since last time-isr
	if(uNbrEdges)
	{
		uint16 uTblIndex1, uTblIndex2, uTblIndexN;	
		uint32 uCapture, uCapture1, uPeriod;							// 	Last captured period
		uTblIndex1 = DMA_PREV_POS_INDEX(dmaTblIndex,1);		// 	Index of Ts-1 Last (and safe) capture position
		uTblIndex2 = DMA_PREV_POS_INDEX(dmaTblIndex,2);		//	Index of Ts-2 at DMA
		uTblIndexN = DMA_PREV_POS_INDEX(dmaTblIndex,uNbrEdges);		// Extend precision at High frequency
		//
		// Catch error - Debug when we have a capture just entering
		if(aDmaCaptureTbl[uTblIndex1] < 100)
			bCatch = TRUE;
				
		// Build the 32bit capture of the last pulse 
		uCapture = 		bCatch? uMswCapTmr + 1 : uMswCapTmr;
		uCapture <<=	16;
		uCapture += 	aDmaCaptureTbl[uTblIndex1];
		// uCapture = ((uint32)uMswCapTmr)<< 16 + (uint32)uCapLw;	// It is not working!!
		
			
		
		
		//	Build the 32bit capture of the previous to last pulse
		if(uNbrEdges == 1)// && !bNoPulseIsr)	//
			uCapture1 = uLastCapture;		// Use the last known value
		else
		{
			// uNbrEdges >= 2 
			// Calculate Period using the previous to last DMA sample instead
			uCapture1  = uMswCapTmr;
			uCapture1  <<= 16;
			uCapture1  += aDmaCaptureTbl[uTblIndex2];	// 	Capture at Ts-2
		}
		// Solve for a 22 bit counter ambiguity 
		if(uCapture >= uCapture1)
		{
			uPeriod = uCapture - uCapture1;
		}
		else
		{
			uPeriod = uCapture + (0x3FFFFF - uCapture1) +1;
		}
		
		// ========  Just a Debug artifact ======== 
		if(bDisplay || bCatch) // && iCiter == DMA_TCD0_CITER_ELINKNO)	// Don't display if a new capture
		{
			printf("\r\n%6u %d %u %u %u", uPeriod, uNbrEdges, uCapture, uCapture1, uLastCapture);
			if(uNbrEdges >= 2) 
				printf(" Ex-> [%u, %u]", aDmaCaptureTbl[uTblIndex1], aDmaCaptureTbl[uTblIndexN] );		// this will extend the counter
			if(bCatch)
				printf("= Capture inside=");
			bDisplay = FALSE;
		}
		
		uLastCapture = uCapture;
		
	}
	else
		bNoPulseIsr = TRUE;	// We have a Timed isr with no pulse in previous - resync the next isr for low frequency if needed
	
	// Epilog: keep the 6 bit msw 
	if(fOvfHalf)
		if(++uMswCapTmr > 0x3F)	// We just need 6 extra bits	
			uMswCapTmr =0;
	
	TP3_ClrVal(TimerPtr); 
	
}


void freqDmaRun(void)
{

	LDD_TDeviceData *DMAPtr = NULL;
	LDD_DMA_TTransferDescriptor TransferDesc;
	
	volatile bool Completed = FALSE;
	bool monitorEnable = FALSE;
	unsigned int uTimerCaptureClock = CAPTURE_CLOCK_HZ;
  
	// My local init
	clearTable();

	                         
		
	
	// Init the DMA		
	TransferDesc = DMA_TRANSFER_DESC_NULL;
  TP3_ptr = TP3_Init(NULL);
  TP4_ptr = TP4_Init(NULL);
  // TINT1Ptr = TINT1_Init(NULL);
  DMAPtr = DMA0_Init(NULL);
  /* Initialize transfer descriptor */
  	// Source
  TransferDesc.SourceAddress = (LDD_DMA_TAddress)&FTM0_C4V;											//	(FTM1_C0SC, FTM1_C0V)  FTM1_CH0 capture
  TransferDesc.SourceAddressOffset = (LDD_DMA_TAddressOffset)0;										//	Single source		
  TransferDesc.SourceTransferSize = (LDD_DMA_TTransferSize)DMA_PDD_16_BIT;				//	16-bit value
  TransferDesc.SourceModuloSize = (LDD_DMA_TModuloSize)0;													// 	non relevant (single source)
  	// Destination
  TransferDesc.DestinationAddress = (LDD_DMA_TAddress)aDmaCaptureTbl;								//	Move to Capture table	
  TransferDesc.DestinationAddressOffset = (LDD_DMA_TAddressOffset)sizeof(aDmaCaptureTbl[0]);	// Next write offset
  TransferDesc.DestinationTransferSize = (LDD_DMA_TTransferSize)DMA_PDD_16_BIT;		// moving 16-bit data
  TransferDesc.DestinationModuloSize = (LDD_DMA_TModuloSize)0;										// we will set manually a new address
    
    // We have Major loop = Move 4 bytes per interrupt and repeat 8 times 
  TransferDesc.TransferMode = LDD_DMA_NESTED_TRANSFERS;						
  TransferDesc.ByteCount = (LDD_DMA_TByteCount)2;	
  TransferDesc.OuterLoopCount = (LDD_DMA_TOuterLoopCount)NBR_DMA_CAPTURE_SAMPLES;
    
    //	DMA channel
  TransferDesc.ChannelAutoSelection = FALSE;// TRUE;								// Fixed to ch 0	
  TransferDesc.ChannelNumber = (LDD_DMA_TChannelNumber)0;		// Channel 0 in this example
    
    //	Trigger
  TransferDesc.TriggerType = LDD_DMA_HW_TRIGGER;						// Triggered by Peripheral request
  																										//	
  TransferDesc.TriggerSource = (LDD_DMA_TTriggerSource)24;	// DMA source Table 3-24 == FTM0_CH4 
  TransferDesc.Interrupts = TRUE;	// we are here									We are not going to interrupt on DMA transfer complete 
  TransferDesc.OnComplete = TRUE;										// Signal an DMA-done	
  TransferDesc.OnCompleteEventPtr = &TransferComplete;			// call this function
  TransferDesc.UserDataPtr = (LDD_TUserData*)&Completed;		// the UserDataPtr to pass data	
    
    // AFter transfer
  TransferDesc.AfterTransferComplete = LDD_DMA_ADDRESS_ADJUSTMENT;
  TransferDesc.DestinationAddressAdjustment = -(2*NBR_DMA_CAPTURE_SAMPLES);	
  
  /* Start DMA transfer */
  DMA0_AllocateChannel(DMAPtr, &TransferDesc);					// Still need to call this method even  we have a fixed channel
  DMA0_EnableChannel(DMAPtr, &TransferDesc);						// This moves the TransferDesc to the TCD 32 bytes
  // DMA0_StartChannelTransfer(DMAPtr, &TransferDesc);
    
  
      
  //============ Using PEX DMA, TCAP Components =========================
  // TCAP:TimerUnit_LDD  !!! DO not enable in init code and do not Autoninit in PEX properties !!  
  TimerPtr = TCAP_Init(NULL);		// 	Set the desired PEX Properties and get the pointer to static memory
  //	Individually disable Channels 
  FTM0_C0SC &= ~FTM_CnSC_CHIE_MASK;		// Disable CH0 interrupt and repeat for other channels
  FTM0_C4SC &= ~FTM_CnSC_CHIE_MASK;		// Disable CH1 interrupt
  FTM0_C0V = 0x7FFF;									//	Need to have DMA ready before OVrs
    
  // 	Enable the base Timer Module by selecting the clk source in CLKS[FTM1_SC]
  TCAP_Enable(TimerPtr);
  // DONE! to start capturing, enable the Channel capture IE in the App
  
  
  // remove Debug UART buffering (stream i/o)
  setvbuf(stdout, NULL, _IONBF, 0); // no buffering on stdout - for printf()
  setvbuf(stdin, NULL, _IONBF, 0); // no buffering on stdin - for getchar() etc
  
  printf("Capture FTM1_CH0 at PTA12 - press '1','t' to start, 'm' to monitor\n\r");
  printf("\n\r");
  unsigned int uLocalCapture=0;
  
  // Init Periodic Time Interrupt
  // TINT1_Enable(TINT1Ptr);	// Enable interrupts after we get the pointer to PTA17
  int i, ch =0;
  uCapture = uLocalCapture =0;
  float fFreq;
  unsigned int uCtr=0;
    	
  while(1)
  {
  	if(uLocalCapture != uCapture)
  	{
  		if(uCapture > 99)
  			uCapture =0;
  		uLocalCapture = uCapture; 
  		  		
  		// PTA17_NegVal(PTA17_ptr);
  		printf("(%2u,%u) ", uCapture, tPeriodIsr);	// Isr executed 
  	}
  	if( (ch = getchar()) != EOF )
  	switch(ch)
  	{
  	// Diable all interrupts
  	case '0':	
  		FTM0_C0SC &= 	~FTM_CnSC_CHIE_MASK;
  		FTM0_C4SC &=  ~(FTM_CnSC_CHIE_MASK | FTM_CnSC_DMA_MASK);		
  		break;
  	// Enable DMA 
  	case '1' :
  		FTM0_C4SC |= (FTM_CnSC_CHIE_MASK | FTM_CnSC_DMA_MASK);		// Enable TCAP_CH4 DMA request
  		break;
  		
  	// Enable Isr in Capture 
  	case '2' :
  		FTM0_C4SC |= FTM_CnSC_CHIE_MASK;													// Test TCAP_CH4 ISR is working
  		break;
  		
  	// Enable (OvfIsr)
  	case 't' : case 'T':
  	  FTM0_C0SC |= FTM_CnSC_CHIE_MASK;													// Test TCAP_CH0 (OVF) ISR is working		
  	  break;
  	
  	case '4':
 			monitorEnable = monitorEnable ? 0 :1;
 			break;
 			
 		case 'm':
 			//  			bDisplay = TRUE;
 			monitorEnable = monitorEnable? FALSE : TRUE;
 			break;
 		case '?': 
 			printf("\r\nMon=%d, Clk= %u, type '+'/'-' to inc/dec", monitorEnable, uTimerCaptureClock);
 			if(tFreqInput.uNbrEdges >1 )
 				fFreq = (float)tFreqInput.uNbrEdges * uTimerCaptureClock/tFreqInput.utCalcPeriod.l;	//(saves one multiplication)
 			else
 				fFreq = (float)uTimerCaptureClock/tFreqInput.utCalcPeriod.l;
 			//	Output the value
 			printf("%5u %f\r\n", ++uCtr, fFreq);
 			  		
 			break;
 			
 		case ' ':
 			bDisplay = TRUE;
 			break;
 		case '+':
 			++uTimerCaptureClock;
 			break;
 		case '-':
 			--uTimerCaptureClock;
 			break;
 			
 		case 'c': case 'C':
 			clearTable();
 			break;
 			
  	}
  	if(monitorEnable && tFreqInput.fNewFreq)
  	{
  		//TP3_SetVal(TimerPtr);
  		tFreqInput.fNewFreq = FALSE;
  		if(tFreqInput.uNbrEdges >1 )
  			fFreq = (float)tFreqInput.uNbrEdges * uTimerCaptureClock/tFreqInput.utCalcPeriod.l;	//(saves one multiplication)
  		else
  			fFreq = (float)uTimerCaptureClock/tFreqInput.utCalcPeriod.l;
  		//	Output the value
  		printf("%5u %f\r\n", ++uCtr, fFreq);
  		
  		//TP3_ClrVal(TimerPtr); 
  	}
  		
  }
  //  DMA_Deinit(DMAPtr);
  // TIMER_Deinit(TimerPtr);
	
}


