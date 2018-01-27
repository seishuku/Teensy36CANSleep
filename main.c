#include "MK66F18.h"
#include <stdlib.h>
#include "can.h"

// Useful 'bits'
#define BIT0             (0x00000001)
#define BIT1             (0x00000002)
#define BIT2             (0x00000004)
#define BIT3             (0x00000008)
#define BIT4             (0x00000010)
#define BIT5             (0x00000020)
#define BIT6             (0x00000040)
#define BIT7             (0x00000080)
#define BIT8             (0x00000100)
#define BIT9             (0x00000200)
#define BIT10            (0x00000400)
#define BIT11            (0x00000800)
#define BIT12            (0x00001000)
#define BIT13            (0x00002000)
#define BIT14            (0x00004000)
#define BIT15            (0x00008000)
#define BIT16			 (0x00010000)
#define BIT17			 (0x00020000)
#define BIT18			 (0x00040000)
#define BIT19			 (0x00080000)
#define BIT20			 (0x00100000)
#define BIT21			 (0x00200000)
#define BIT22			 (0x00400000)
#define BIT23			 (0x00800000)
#define BIT24			 (0x01000000)
#define BIT25			 (0x02000000)
#define BIT26			 (0x04000000)
#define BIT27			 (0x08000000)
#define BIT28			 (0x10000000)
#define BIT29			 (0x20000000)
#define BIT30			 (0x40000000)
#define BIT31			 (0x80000000)

// Timing variables
volatile uint32_t Tick=0;
uint32_t Time=0, Time2Sleep=0;

// CAN frame for transmit
CAN_Frame_t tx;

void SysTick_Handler(void)
{
	Tick++;
}

// Get random value from the hardware RNG
uint32_t random(void)
{
	while(((RNG->SR&RNG_SR_OREG_LVL_MASK)>>RNG_SR_OREG_LVL_SHIFT)==0);

	return RNG->OR;
}

void SetStopMode(void)
{
	// Enable the low-leakage wake-up interrupt
	NVIC_EnableIRQ(LLWU_IRQn);

	// Turn off LED for sleep
	GPIOC->PCOR|=BIT5;

	// Put SWCAN transceiver into "sleep" mode
	GPIOB->PCOR|=BIT16;
	GPIOB->PCOR|=BIT17;

	// Disable SysTick interrupt (if it isn't, the MCU will never sleep)
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;

	// Make sure LL wake-up flags are cleared
	LLWU->PF1=0xFF;
	LLWU->PF2=0xFF;
	LLWU->PF3=0xFF;
	LLWU->FILT1|=LLWU_FILT1_FILTF_MASK;
	LLWU->FILT2|=LLWU_FILT2_FILTF_MASK;

	// Set deep sleep flag
	SCB->SCR=SCB_SCR_SLEEPDEEP_Msk;

	// Set power management to LLS (low-leakage stop) mode
	SMC->PMCTRL=SMC_PMCTRL_STOPM(0x03);
	(void)(SMC->PMCTRL==0U); // Dummy read to insure it's set

	// Go to stop mode
	__WFI();
}


void LLWU_IRQHandler(void)
{
	// Clear wake up flags
	LLWU->PF1=0xFF;
	LLWU->PF2=0xFF;
	LLWU->PF3=0xFF;
	LLWU->FILT1|=LLWU_FILT1_FILTF_MASK;
	LLWU->FILT2|=LLWU_FILT2_FILTF_MASK;

	// Disable the wake-up IRQ (needed?)
	NVIC_DisableIRQ(LLWU_IRQn);

	// Switch SWCAN transceiver back to "normal" mode
	GPIOB->PSOR|=BIT16;
	GPIOB->PSOR|=BIT17;

	// Set the LED to show we're awake!
	GPIOC->PSOR|=BIT5;

	// Reset core and bus clocks
	Set_120MHz_Clock();

	// Re-enable SysTick interrupt
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;

	// And clear the deep sleep flags
	SCB->SCR&=~(SCB_SCR_SLEEPDEEP_Msk|SCB_SCR_SLEEPONEXIT_Msk);

	// Errata
    __DSB();
}

int main(void)
{
	// Configure LED output GPIO
	PORTC->PCR[5]=PORT_PCR_MUX(0x01);
	GPIOC->PDDR|=BIT5;
	GPIOC->PSOR|=BIT5;

	// Configure single wire (GMLAN) CAN transceiver mode pins
	// NXP NCV7356g and Melexis TH8056 are what I use.

	// Mode0	Mode1
	// L		L		= Sleep
	// H		L		= High speed mode
	// L		H		= High voltage wake up mode
	// H		H		= Normal mode
	
	// Mode 0
	PORTB->PCR[16]=PORT_PCR_MUX(1);
	GPIOB->PDDR|=BIT16;
	GPIOB->PSOR|=BIT16;

	// Mode 1
	PORTB->PCR[17]=PORT_PCR_MUX(1);
	GPIOB->PDDR|=BIT17;
	GPIOB->PSOR|=BIT17;

	// Initialize FlexCAN module
	CAN_Init();

	// Set up the hardware random number generator
	SIM->SCGC3|=SIM_SCGC3_RNGA_MASK;
	RNG->CR=0;
	RNG->CR=RNG_CR_GO(1);

	// Configure the low-leakage wake-up pin to PA13 (CAN RX), rising edge only
	LLWU->PE2|=LLWU_PE2_WUPE4(0x02);

	// Refresh timers to the current time
	Time=Time2Sleep=Tick;

	while(1)
	{
		// If CANbus has idled for 5 seconds, go to sleep
		if((Tick-Time2Sleep)>5000)
		{
			Time2Sleep=Tick;
			SetStopMode();
		}

		// Every ~10ms, send out a CAN message
		if((Tick-Time)>10)
		{
			Time=Tick;

			// Extendend ID, 0x123, just random garbage
			tx.MessageID=CAN_MESSAGE_ID_EXT|0x123;
			tx.Length=8;
			tx.FrameType=CAN_DATA_FRAME;
			tx.Data[0]=random()%0xFF;
			tx.Data[1]=random()%0xFF;
			tx.Data[2]=random()%0xFF;
			tx.Data[3]=random()%0xFF;
			tx.Data[4]=random()%0xFF;
			tx.Data[5]=random()%0xFF;
			tx.Data[6]=random()%0xFF;
			tx.Data[7]=random()%0xFF;
			CAN_SendFrame(tx);
		}
	}

	return 0;
}

void CAN0_ORed_Message_buffer_IRQHandler(void)
{
	uint32_t status=CAN0->IFLAG1; // Get interrupt status
	uint8_t i=0;
	CAN_Frame_t rx;

	// Check all message buffers
	for(i=0;i<16;i++)
	{
		// Skip over any that didn't trigger the IRQ
		if((status&(1UL<<i))==0)
			continue;

		// As long as CAN data in coming in, keep refreshing the sleep timer.
		// If CAN data stops, the timer elaspes, and the MCU goes into deep sleep.
		Time2Sleep=Tick;

		// Read in the CAN frame
		CAN_ReadFrame(&rx, i);

		// Clear IRQ flag
		CAN0->IFLAG1|=1<<i;
	}
}
