#include "MK66F18.h"
#include <string.h>
#include <stdlib.h>

#ifndef DISABLE_WDOG
#define DISABLE_WDOG 1
#endif

extern char __stack[];

void __thumb_startup(void);
extern int main(void);

__attribute__ ((weak, section(".after_vectors"))) void NMI_Handler(void) { while(1); }
__attribute__ ((weak, section(".after_vectors"))) void HardFault_Handler(void) { while(1); }
__attribute__ ((weak, section(".after_vectors"))) void MemManage_Handler(void) { while(1); }
__attribute__ ((weak, section(".after_vectors"))) void BusFault_Handler(void) { while(1); }
__attribute__ ((weak, section(".after_vectors"))) void UsageFault_Handler(void) { while(1); }
__attribute__ ((weak, section(".after_vectors"))) void SVC_Handler(void) { }
__attribute__ ((weak, section(".after_vectors"))) void DebugMon_Handler(void) { }
__attribute__ ((weak, section(".after_vectors"))) void PendSV_Handler(void) { }
__attribute__ ((weak, section(".after_vectors"))) void SysTick_Handler(void) { }
__attribute__ ((weak, section(".after_vectors"))) void IntDefaultHandler(void) { }
__attribute__ ((weak)) void DMA0_DMA16_IRQHandler(void) { }
__attribute__ ((weak)) void DMA1_DMA17_IRQHandler(void) { }
__attribute__ ((weak)) void DMA2_DMA18_IRQHandler(void) { }
__attribute__ ((weak)) void DMA3_DMA19_IRQHandler(void) { }
__attribute__ ((weak)) void DMA4_DMA20_IRQHandler(void) { }
__attribute__ ((weak)) void DMA5_DMA21_IRQHandler(void) { }
__attribute__ ((weak)) void DMA6_DMA22_IRQHandler(void) { }
__attribute__ ((weak)) void DMA7_DMA23_IRQHandler(void) { }
__attribute__ ((weak)) void DMA8_DMA24_IRQHandler(void) { }
__attribute__ ((weak)) void DMA9_DMA25_IRQHandler(void) { }
__attribute__ ((weak)) void DMA10_DMA26_IRQHandler(void) { }
__attribute__ ((weak)) void DMA11_DMA27_IRQHandler(void) { }
__attribute__ ((weak)) void DMA12_DMA28_IRQHandler(void) { }
__attribute__ ((weak)) void DMA13_DMA29_IRQHandler(void) { }
__attribute__ ((weak)) void DMA14_DMA30_IRQHandler(void) { }
__attribute__ ((weak)) void DMA15_DMA31_IRQHandler(void) { }
__attribute__ ((weak)) void DMA_Error_IRQHandler(void) { }
__attribute__ ((weak)) void MCM_IRQHandler(void) { }
__attribute__ ((weak)) void FTFE_IRQHandler(void) { }
__attribute__ ((weak)) void Read_Collision_IRQHandler(void) { }
__attribute__ ((weak)) void LVD_LVW_IRQHandler(void) { }
__attribute__ ((weak)) void LLWU_IRQHandler(void) { }
__attribute__ ((weak)) void WDOG_EWM_IRQHandler(void) { }
__attribute__ ((weak)) void RNG_IRQHandler(void) { }
__attribute__ ((weak)) void I2C0_IRQHandler(void) { }
__attribute__ ((weak)) void I2C1_IRQHandler(void) { }
__attribute__ ((weak)) void SPI0_IRQHandler(void) { }
__attribute__ ((weak)) void SPI1_IRQHandler(void) { }
__attribute__ ((weak)) void I2S0_Tx_IRQHandler(void) { }
__attribute__ ((weak)) void I2S0_Rx_IRQHandler(void) { }
__attribute__ ((weak)) void Reserved46_IRQHandler(void) { }
__attribute__ ((weak)) void UART0_RX_TX_IRQHandler(void) { }
__attribute__ ((weak)) void UART0_ERR_IRQHandler(void) { }
__attribute__ ((weak)) void UART1_RX_TX_IRQHandler(void) { }
__attribute__ ((weak)) void UART1_ERR_IRQHandler(void) { }
__attribute__ ((weak)) void UART2_RX_TX_IRQHandler(void) { }
__attribute__ ((weak)) void UART2_ERR_IRQHandler(void) { }
__attribute__ ((weak)) void UART3_RX_TX_IRQHandler(void) { }
__attribute__ ((weak)) void UART3_ERR_IRQHandler(void) { }
__attribute__ ((weak)) void ADC0_IRQHandler(void) { }
__attribute__ ((weak)) void CMP0_IRQHandler(void) { }
__attribute__ ((weak)) void CMP1_IRQHandler(void) { }
__attribute__ ((weak)) void FTM0_IRQHandler(void) { }
__attribute__ ((weak)) void FTM1_IRQHandler(void) { }
__attribute__ ((weak)) void FTM2_IRQHandler(void) { }
__attribute__ ((weak)) void CMT_IRQHandler(void) { }
__attribute__ ((weak)) void RTC_IRQHandler(void) { }
__attribute__ ((weak)) void RTC_Seconds_IRQHandler(void) { }
__attribute__ ((weak)) void PIT0_IRQHandler(void) { }
__attribute__ ((weak)) void PIT1_IRQHandler(void) { }
__attribute__ ((weak)) void PIT2_IRQHandler(void) { }
__attribute__ ((weak)) void PIT3_IRQHandler(void) { }
__attribute__ ((weak)) void PDB0_IRQHandler(void) { }
__attribute__ ((weak)) void USB0_IRQHandler(void) { }
__attribute__ ((weak)) void USBDCD_IRQHandler(void) { }
__attribute__ ((weak)) void Reserved71_IRQHandler(void) { }
__attribute__ ((weak)) void DAC0_IRQHandler(void) { }
__attribute__ ((weak)) void MCG_IRQHandler(void) { }
__attribute__ ((weak)) void LPTMR0_IRQHandler(void) { }
__attribute__ ((weak)) void PORTA_IRQHandler(void) { }
__attribute__ ((weak)) void PORTB_IRQHandler(void) { }
__attribute__ ((weak)) void PORTC_IRQHandler(void) { }
__attribute__ ((weak)) void PORTD_IRQHandler(void) { }
__attribute__ ((weak)) void PORTE_IRQHandler(void) { }
__attribute__ ((weak)) void SWI_IRQHandler(void) { }
__attribute__ ((weak)) void SPI2_IRQHandler(void) { }
__attribute__ ((weak)) void UART4_RX_TX_IRQHandler(void) { }
__attribute__ ((weak)) void UART4_ERR_IRQHandler(void) { }
__attribute__ ((weak)) void Reserved84_IRQHandler(void) { }
__attribute__ ((weak)) void Reserved85_IRQHandler(void) { }
__attribute__ ((weak)) void CMP2_IRQHandler(void) { }
__attribute__ ((weak)) void FTM3_IRQHandler(void) { }
__attribute__ ((weak)) void DAC1_IRQHandler(void) { }
__attribute__ ((weak)) void ADC1_IRQHandler(void) { }
__attribute__ ((weak)) void I2C2_IRQHandler(void) { }
__attribute__ ((weak)) void CAN0_ORed_Message_buffer_IRQHandler(void) { }
__attribute__ ((weak)) void CAN0_Bus_Off_IRQHandler(void) { }
__attribute__ ((weak)) void CAN0_Error_IRQHandler(void) { }
__attribute__ ((weak)) void CAN0_Tx_Warning_IRQHandler(void) { }
__attribute__ ((weak)) void CAN0_Rx_Warning_IRQHandler(void) { }
__attribute__ ((weak)) void CAN0_Wake_Up_IRQHandler(void) { }
__attribute__ ((weak)) void SDHC_IRQHandler(void) { }
__attribute__ ((weak)) void ENET_1588_Timer_IRQHandler(void) { }
__attribute__ ((weak)) void ENET_Transmit_IRQHandler(void) { }
__attribute__ ((weak)) void ENET_Receive_IRQHandler(void) { }
__attribute__ ((weak)) void ENET_Error_IRQHandler(void) { }
__attribute__ ((weak)) void LPUART0_IRQHandler(void) { }
__attribute__ ((weak)) void TSI0_IRQHandler(void) { }
__attribute__ ((weak)) void TPM1_IRQHandler(void) { }
__attribute__ ((weak)) void TPM2_IRQHandler(void) { }
__attribute__ ((weak)) void USBHSDCD_IRQHandler(void) { }
__attribute__ ((weak)) void I2C3_IRQHandler(void) { }
__attribute__ ((weak)) void CMP3_IRQHandler(void) { }
__attribute__ ((weak)) void USBHS_IRQHandler(void) { }
__attribute__ ((weak)) void CAN1_ORed_Message_buffer_IRQHandler(void) { }
__attribute__ ((weak)) void CAN1_Bus_Off_IRQHandler(void) { }
__attribute__ ((weak)) void CAN1_Error_IRQHandler(void) { }
__attribute__ ((weak)) void CAN1_Tx_Warning_IRQHandler(void) { }
__attribute__ ((weak)) void CAN1_Rx_Warning_IRQHandler(void) { }
__attribute__ ((weak)) void CAN1_Wake_Up_IRQHandler(void) { }

// Interrupt vector table
__attribute__ ((used, section(".vectortable"))) void (* const __vect_table[])(void)=
{
	(void *)&__stack,						// The initial stack pointer
	__thumb_startup,						// The reset handler
	NMI_Handler,							// The NMI handler
	HardFault_Handler,						// The hard fault handler
	MemManage_Handler,						// The MPU fault handler
	BusFault_Handler,						// The bus fault handler
	UsageFault_Handler,						// The usage fault handler
	0,										// Reserved
	0,										// Reserved
	0,										// Reserved
	0,										// Reserved
	SVC_Handler,							// SVCall handler
	DebugMon_Handler,						// Debug monitor handler
	0,										// Reserved
	PendSV_Handler,							// The PendSV handler
	SysTick_Handler,						// The SysTick handler
	DMA0_DMA16_IRQHandler,					// 16 : DMA Channel 0, 16 Transfer Complete
	DMA1_DMA17_IRQHandler,					// 17 : DMA Channel 1, 17 Transfer Complete
	DMA2_DMA18_IRQHandler,					// 18 : DMA Channel 2, 18 Transfer Complete
	DMA3_DMA19_IRQHandler,					// 19 : DMA Channel 3, 19 Transfer Complete
	DMA4_DMA20_IRQHandler,					// 20 : DMA Channel 4, 20 Transfer Complete
	DMA5_DMA21_IRQHandler,					// 21 : DMA Channel 5, 21 Transfer Complete
	DMA6_DMA22_IRQHandler,					// 22 : DMA Channel 6, 22 Transfer Complete
	DMA7_DMA23_IRQHandler,					// 23 : DMA Channel 7, 23 Transfer Complete
	DMA8_DMA24_IRQHandler,					// 24 : DMA Channel 8, 24 Transfer Complete
	DMA9_DMA25_IRQHandler,					// 25 : DMA Channel 9, 25 Transfer Complete
	DMA10_DMA26_IRQHandler,					// 26 : DMA Channel 10, 26 Transfer Complete
	DMA11_DMA27_IRQHandler,					// 27 : DMA Channel 11, 27 Transfer Complete
	DMA12_DMA28_IRQHandler,					// 28 : DMA Channel 12, 28 Transfer Complete
	DMA13_DMA29_IRQHandler,					// 29 : DMA Channel 13, 29 Transfer Complete
	DMA14_DMA30_IRQHandler,					// 30 : DMA Channel 14, 30 Transfer Complete
	DMA15_DMA31_IRQHandler,					// 31 : DMA Channel 15, 31 Transfer Complete
	DMA_Error_IRQHandler,					// 32 : DMA Error Interrupt
	MCM_IRQHandler,							// 33 : Normal Interrupt
	FTFE_IRQHandler,						// 34 : FTFE Command complete interrupt
	Read_Collision_IRQHandler,				// 35 : Read Collision Interrupt
	LVD_LVW_IRQHandler,						// 36 : Low Voltage Detect, Low Voltage Warning
	LLWU_IRQHandler,						// 37 : Low Leakage Wakeup Unit
	WDOG_EWM_IRQHandler,					// 38 : WDOG Interrupt
	RNG_IRQHandler,							// 39 : RNG Interrupt
	I2C0_IRQHandler,						// 40 : I2C0 interrupt
	I2C1_IRQHandler,						// 41 : I2C1 interrupt
	SPI0_IRQHandler,						// 42 : SPI0 Interrupt
	SPI1_IRQHandler,						// 43 : SPI1 Interrupt
	I2S0_Tx_IRQHandler,						// 44 : I2S0 transmit interrupt
	I2S0_Rx_IRQHandler,						// 45 : I2S0 receive interrupt
	Reserved46_IRQHandler,					// 46 : Reserved interrupt
	UART0_RX_TX_IRQHandler,					// 47 : UART0 Receive/Transmit interrupt
	UART0_ERR_IRQHandler,					// 48 : UART0 Error interrupt
	UART1_RX_TX_IRQHandler,					// 49 : UART1 Receive/Transmit interrupt
	UART1_ERR_IRQHandler,					// 50 : UART1 Error interrupt
	UART2_RX_TX_IRQHandler,					// 51 : UART2 Receive/Transmit interrupt
	UART2_ERR_IRQHandler,					// 52 : UART2 Error interrupt
	UART3_RX_TX_IRQHandler,					// 53 : UART3 Receive/Transmit interrupt
	UART3_ERR_IRQHandler,					// 54 : UART3 Error interrupt
	ADC0_IRQHandler,						// 55 : ADC0 interrupt
	CMP0_IRQHandler,						// 56 : CMP0 interrupt
	CMP1_IRQHandler,						// 57 : CMP1 interrupt
	FTM0_IRQHandler,						// 58 : FTM0 fault, overflow and channels interrupt
	FTM1_IRQHandler,						// 59 : FTM1 fault, overflow and channels interrupt
	FTM2_IRQHandler,						// 60 : FTM2 fault, overflow and channels interrupt
	CMT_IRQHandler,							// 61 : CMT interrupt
	RTC_IRQHandler,							// 62 : RTC interrupt
	RTC_Seconds_IRQHandler,					// 63 : RTC seconds interrupt
	PIT0_IRQHandler,						// 64 : PIT timer channel 0 interrupt
	PIT1_IRQHandler,						// 65 : PIT timer channel 1 interrupt
	PIT2_IRQHandler,						// 66 : PIT timer channel 2 interrupt
	PIT3_IRQHandler,						// 67 : PIT timer channel 3 interrupt
	PDB0_IRQHandler,						// 68 : PDB0 Interrupt
	USB0_IRQHandler,						// 69 : USB0 interrupt
	USBDCD_IRQHandler,						// 70 : USBDCD Interrupt
	Reserved71_IRQHandler,					// 71 : Reserved interrupt
	DAC0_IRQHandler,						// 72 : DAC0 interrupt
	MCG_IRQHandler,							// 73 : MCG Interrupt
	LPTMR0_IRQHandler,						// 74 : LPTimer interrupt
	PORTA_IRQHandler,						// 75 : Port A interrupt
	PORTB_IRQHandler,						// 76 : Port B interrupt
	PORTC_IRQHandler,						// 77 : Port C interrupt
	PORTD_IRQHandler,						// 78 : Port D interrupt
	PORTE_IRQHandler,						// 79 : Port E interrupt
	SWI_IRQHandler,							// 80 : Software interrupt
	SPI2_IRQHandler,						// 81 : SPI2 Interrupt
	UART4_RX_TX_IRQHandler,					// 82 : UART4 Receive/Transmit interrupt
	UART4_ERR_IRQHandler,					// 83 : UART4 Error interrupt
	Reserved84_IRQHandler,					// 84 : Reserved interrupt
	Reserved85_IRQHandler,					// 85 : Reserved interrupt
	CMP2_IRQHandler,						// 86 : CMP2 interrupt
	FTM3_IRQHandler,						// 87 : FTM3 fault, overflow and channels interrupt
	DAC1_IRQHandler,						// 88 : DAC1 interrupt
	ADC1_IRQHandler,						// 89 : ADC1 interrupt
	I2C2_IRQHandler,						// 90 : I2C2 interrupt
	CAN0_ORed_Message_buffer_IRQHandler,	// 91 : CAN0 OR'd message buffers interrupt
	CAN0_Bus_Off_IRQHandler,				// 92 : CAN0 bus off interrupt
	CAN0_Error_IRQHandler,					// 93 : CAN0 error interrupt
	CAN0_Tx_Warning_IRQHandler,				// 94 : CAN0 Tx warning interrupt
	CAN0_Rx_Warning_IRQHandler,				// 95 : CAN0 Rx warning interrupt
	CAN0_Wake_Up_IRQHandler,				// 96 : CAN0 wake up interrupt
	SDHC_IRQHandler,						// 97 : SDHC interrupt
	ENET_1588_Timer_IRQHandler,				// 98 : Ethernet MAC IEEE 1588 Timer Interrupt
	ENET_Transmit_IRQHandler,				// 99 : Ethernet MAC Transmit Interrupt
	ENET_Receive_IRQHandler,				// 100: Ethernet MAC Receive Interrupt
	ENET_Error_IRQHandler,					// 101: Ethernet MAC Error and miscelaneous Interrupt
	LPUART0_IRQHandler,						// 102: LPUART0 status/error interrupt
	TSI0_IRQHandler,						// 103: TSI0 interrupt
	TPM1_IRQHandler,						// 104: TPM1 fault, overflow and channels interrupt
	TPM2_IRQHandler,						// 105: TPM2 fault, overflow and channels interrupt
	USBHSDCD_IRQHandler,					// 106: USBHSDCD, USBHS Phy Interrupt
	I2C3_IRQHandler,						// 107: I2C3 interrupt
	CMP3_IRQHandler,						// 108: CMP3 interrupt
	USBHS_IRQHandler,						// 109: USB high speed OTG interrupt
	CAN1_ORed_Message_buffer_IRQHandler,	// 110: CAN1 OR'd message buffers interrupt
	CAN1_Bus_Off_IRQHandler,				// 111: CAN1 bus off interrupt
	CAN1_Error_IRQHandler,					// 112: CAN1 error interrupt
	CAN1_Tx_Warning_IRQHandler,				// 113: CAN1 Tx warning interrupt
	CAN1_Rx_Warning_IRQHandler,				// 114: CAN1 Rx warning interrupt
	CAN1_Wake_Up_IRQHandler,				// 115: CAN1 wake up interrupt
};

// Flash configuration field
__attribute__ ((section (".FlashConfig"))) const uint8_t _cfm[0x10]={ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// format of the ROM table info entry
typedef struct RomInfo
{
	unsigned long Source;
	unsigned long Target;
	unsigned long Size;
} RomInfo;

// linker defined symbol
extern RomInfo __S_romp[] __attribute__((weak));

// Current value of the FAULTMASK register and lock
volatile uint8_t SR_reg;
volatile uint8_t SR_lock=0x00;

// Routine to copy a single section from ROM to RAM
void __copy_rom_section(unsigned long dst, unsigned long src, unsigned long size)
{
	const int size_int=sizeof(int);
	const int mask_int=sizeof(int)-1;
	const int size_short=sizeof(short);
	const int mask_short=sizeof(short)-1;
	const int size_char=sizeof(char);
	unsigned long len=size;

	if(dst==src||size==0)
		return;

	while(len>0)
	{
		if(!(src&mask_int)&&!(dst&mask_int)&&len>=size_int)
		{
			*((int *)dst)=*((int*)src);
			dst+=size_int;
			src+=size_int;
			len-=size_int;
		}
		else if(!(src&mask_short)&&!(dst&mask_short)&&len>=size_short)
		{
			*((short *)dst)=*((short*)src);
			dst+=size_short;
			src+=size_short;
			len-=size_short;
		}
		else
		{
			*((char *)dst)=*((char *)src);
			dst+=size_char;
			src+=size_char;
			len-=size_char;
		}
	}
}

// Routine that copies all sections the user marked as ROM into their target RAM addresses.
// __S_romp is defined in the linker command file, it is a table of RomInfo structures.
// The final entry in the table has all-zero fields.
void __copy_rom_sections_to_ram(void)
{
	int index;

	if(__S_romp==0)
		return;

	// Go through the entire table, copying sections from ROM to RAM.
	for(index=0;__S_romp[index].Source!=0||__S_romp[index].Target!=0||__S_romp[index].Size!=0;++index)
		__copy_rom_section(__S_romp[index].Target, __S_romp[index].Source, __S_romp[index].Size);
}

void __zero_fill_bss(void)
{
	extern char __START_BSS[];
	extern char __END_BSS[];
	unsigned long len=__END_BSS-__START_BSS;
	unsigned long dst=(unsigned long)__START_BSS;
	const int size_int=sizeof(int);
	const int mask_int=sizeof(int)-1;
	const int size_short=sizeof(short);
	const int mask_short=sizeof(short)-1;
	const int size_char=sizeof(char);

	if(len==0)
		return;

	while(len>0)
	{
		if(!(dst&mask_int)&&len>=size_int)
		{
			*((int *)dst)=0;
			dst+=size_int;
			len-=size_int;
		}
		else if(!(dst&mask_short)&&len>=size_short)
		{
			*((short *)dst)=0;
			dst+=size_short;
			len-=size_short;
		}
		else
		{
			*((char *)dst)=0;
			dst+=size_char;
			len-=size_char;
		}
	}
}

uint32_t SystemCoreClock=0;

void Set_120MHz_Clock(void)
{
	volatile uint32_t i;

    /* Set the system clock dividers in SIM to safe value. */
	SIM->CLKDIV1=0x02260000U;

    /* Configure RTC clock including enabling RTC oscillator. */
	/* RTC clock gate enable */
	SIM->SCGC6|=SIM_SCGC6_RTC_MASK;

	if((RTC->CR&RTC_CR_OSCE_MASK)==0u)
	{
		/* Only if the Rtc oscillator is not already enabled */
		/* Set the specified capacitor configuration for the RTC oscillator */
		RTC->CR=(RTC->CR&~(RTC_CR_SC2P_MASK|RTC_CR_SC4P_MASK|RTC_CR_SC8P_MASK|RTC_CR_SC16P_MASK))|0x0U;

		/* Enable the RTC 32KHz oscillator */
		RTC->CR|=RTC_CR_OSCE_MASK;
	}

	/* Output to other peripherals */
	RTC->CR&=~RTC_CR_CLKO_MASK;

	/* Set RTC_TSR if there is fault value in RTC */
	if(RTC->SR&RTC_SR_TIF_MASK)
		RTC->TSR=RTC->TSR;

	/* RTC clock gate disable */
	SIM->SCGC6&=~SIM_SCGC6_RTC_MASK;

	/* Set Cap load to 0pF */
	OSC->CR=(OSC->CR&~(OSC_CR_SC2P_MASK|OSC_CR_SC4P_MASK|OSC_CR_SC8P_MASK|OSC_CR_SC16P_MASK))|0x0U;

	/* OSCERCLK enable, not in stop mode */
	OSC->CR=(OSC->CR&~(OSC_CR_ERCLKEN_MASK|OSC_CR_EREFSTEN_MASK))|OSC_CR_ERCLKEN_MASK;

	/* Set divider */
    OSC->DIV=OSC_DIV_ERPS(0);

	MCG->C2=(MCG->C2&~(MCG_C2_EREFS0_MASK|MCG_C2_HGO0_MASK|MCG_C2_RANGE0_MASK))|MCG_C2_RANGE(0x2U)|MCG_C2_EREFS_MASK;

	/* Wait for stable. */
	while(!(MCG->S&MCG_S_OSCINIT0_MASK));

    /* Configure the Internal Reference clock (MCGIRCLK). */
	MCG->C2=((MCG->C2&~MCG_C2_IRCS_MASK)|(MCG_C2_IRCS(0x0U)));

	while(((MCG->S&MCG_S_IRCST_MASK)>>MCG_S_IRCST_SHIFT)!=0x0U);

	/* Update FCRDIV. */
	MCG->SC=(MCG->SC&~(MCG_SC_FCRDIV_MASK|MCG_SC_ATMF_MASK|MCG_SC_LOCS0_MASK))|MCG_SC_FCRDIV(0x1U);

	/* Set internal reference clock selection. */
	MCG->C2=(MCG->C2&~MCG_C2_IRCS_MASK)|(MCG_C2_IRCS(0x0U));
	MCG->C1=(MCG->C1&~(MCG_C1_IRCLKEN_MASK|MCG_C1_IREFSTEN_MASK))|MCG_C1_IRCLKEN_MASK;

	/* If MCGIRCLK is used, need to wait for MCG_S_IRCST. */
	while(((MCG->S&MCG_S_IRCST_MASK)>>MCG_S_IRCST_SHIFT)!=0x0U);

    /* Set USB slow clock. */
    SIM->SOPT2=((SIM->SOPT2&~SIM_SOPT2_USBSLSRC_MASK)|SIM_SOPT2_USBSLSRC(0));

    /* Configure USBPHY PLL. */
	SIM->SCGC3|=SIM_SCGC3_USBHSPHY_MASK;
	SIM->SOPT2|=SIM_SOPT2_USBREGEN_MASK;

	// wait 50ms
	i=500000U;
	while(i--);

	USBPHY->TRIM_OVERRIDE_EN=0x01U;							/* Override the trim. */
	USBPHY->CTRL&=~USBPHY_CTRL_SFTRST_MASK;					/* release PHY from reset */
	USBPHY->PLL_SIC|=USBPHY_PLL_SIC_PLL_POWER_MASK;			/* power up PLL */
	USBPHY->PLL_SIC=(USBPHY->PLL_SIC&~USBPHY_PLL_SIC_PLL_DIV_SEL_MASK)|USBPHY_PLL_SIC_PLL_DIV_SEL(1U);
	USBPHY->PLL_SIC&=~USBPHY_PLL_SIC_PLL_BYPASS_MASK;		/* Clear bypass bit */
	USBPHY->CTRL&=~USBPHY_CTRL_CLKGATE_MASK;				/* Clear to 0U to run clocks */

	/* Wait for lock. */
	while(!(USBPHY->PLL_SIC&USBPHY_PLL_SIC_PLL_LOCK_MASK));

    /* Configure FLL external reference divider (FRDIV). */
    MCG->C1=((MCG->C1&~MCG_C1_FRDIV_MASK)|MCG_C1_FRDIV(0x0U));

    /* Set MCG to PEE mode. */

	/* Select external clock reference */
	MCG->C7=(MCG->C7&~MCG_C7_OSCSEL_MASK)|MCG_C7_OSCSEL(0x0U);

	/* Disable lowpower. */
	MCG->C2&=~MCG_C2_LP_MASK;

	/* Change to use external clock first. */
	MCG->C1=((MCG->C1&~(MCG_C1_CLKS_MASK|MCG_C1_IREFS_MASK))|MCG_C1_CLKS(0x2U));

	/* Wait for CLKST clock status bits to show clock source is ext ref clk */
	while((MCG->S&(MCG_S_IREFST_MASK|MCG_S_CLKST_MASK))!=(MCG_S_IREFST(0x0U)|MCG_S_CLKST(0x2U)));

	/* Disable PLL first, then configure PLL. */
	MCG->C6&=~MCG_C6_PLLS_MASK;
	while(MCG->S&MCG_S_PLLST_MASK);

	/* Configure the PLL. */
	MCG->C5=MCG_C5_PRDIV0(0x1U); /* Disable the PLL first. */

	MCG->C6=(MCG->C6&~MCG_C6_VDIV0_MASK)|MCG_C6_VDIV0(0xEU);

	/* Set enable mode, not in stop mode. */
	MCG->C5|=MCG_C5_PLLCLKEN0_MASK;

	/* Wait for PLL lock. */
	while(!(MCG->S&MCG_S_LOCK0_MASK));

	/* Change to PLL mode. */
	MCG->C6|=MCG_C6_PLLS_MASK;
	MCG->C11=((MCG->C11&~MCG_C11_PLLCS_MASK))|MCG_C11_PLLCS(0x0U);
	while(((MCG->S2&MCG_S2_PLLCST_MASK)>>MCG_S2_PLLCST_SHIFT)!=0x0U);

	/* Wait for PLL mode changed. */
	while(!(MCG->S&MCG_S_PLLST_MASK));

    /* Change to use PLL output clock. */
    MCG->C1=(MCG->C1&~MCG_C1_CLKS_MASK)|MCG_C1_CLKS(0x0U);

	while(((MCG->S&MCG_S_CLKST_MASK)>>MCG_S_CLKST_SHIFT)!=0x3U);

    /* Set the clock configuration in SIM module. */
	/* SIM_CLKDIV1 - OUTDIV1: /1, OUTDIV2: /2, OUTDIV3: /2, OUTDIV4: /5 */
    SIM->CLKDIV1=SIM_CLKDIV1_OUTDIV1(1)|SIM_CLKDIV1_OUTDIV2(2)|SIM_CLKDIV1_OUTDIV3(2)|SIM_CLKDIV1_OUTDIV4(5);

	/* SOPT2 select MCGFLLCLK */
    SIM->SOPT2=((SIM->SOPT2&~SIM_SOPT2_PLLFLLSEL_MASK)|SIM_SOPT2_PLLFLLSEL(0));
    SIM->CLKDIV3=SIM_CLKDIV3_PLLFLLDIV(0)|SIM_CLKDIV3_PLLFLLFRAC(0);

	/* OSC32KSEL select: OSC32KCLK clock */
	SIM->SOPT1=((SIM->SOPT1&~SIM_SOPT1_OSC32KSEL_MASK)|SIM_SOPT1_OSC32KSEL(0));

	SystemCoreClock=120000000U;
}

void Set_4MHz_Low_Power_Clock(void)
{
	volatile uint32_t i=1500U;

    /* Set the system clock dividers in SIM to safe value. */
	SIM->CLKDIV1=0x02260000U;

    /* Configure RTC clock including enabling RTC oscillator. */
	/* RTC clock gate enable */
	SIM->SCGC6|=SIM_SCGC6_RTC_MASK;

	if((RTC->CR&RTC_CR_OSCE_MASK)==0u)
	{
		/* Only if the Rtc oscillator is not already enabled */
		/* Set the specified capacitor configuration for the RTC oscillator */
		RTC->CR=(RTC->CR&~(RTC_CR_SC2P_MASK|RTC_CR_SC4P_MASK|RTC_CR_SC8P_MASK|RTC_CR_SC16P_MASK))|0x0U;

		/* Enable the RTC 32KHz oscillator */
		RTC->CR|=RTC_CR_OSCE_MASK;
	}

	/* Output to other peripherals */
	RTC->CR&=~RTC_CR_CLKO_MASK;

	/* Set RTC_TSR if there is fault value in RTC */
	if(RTC->SR&RTC_SR_TIF_MASK)
		RTC->TSR=RTC->TSR;

	/* RTC clock gate disable */
	SIM->SCGC6&=~SIM_SCGC6_RTC_MASK;

	/* Set external clock reference */
	OSC->CR=(OSC->CR&~(OSC_CR_SC2P_MASK|OSC_CR_SC4P_MASK|OSC_CR_SC8P_MASK|OSC_CR_SC16P_MASK))|0x0U;
	OSC->CR=(OSC->CR&~(OSC_CR_ERCLKEN_MASK|OSC_CR_EREFSTEN_MASK))|0x0U;
	OSC->DIV=OSC_DIV_ERPS(0x0U);

	MCG->C2=((MCG->C2&~(MCG_C2_EREFS0_MASK|MCG_C2_HGO0_MASK|MCG_C2_RANGE0_MASK))|MCG_C2_RANGE(0x2U)|MCG_C2_EREFS_MASK);

	while(!(MCG->S&MCG_S_OSCINIT0_MASK));

	/* Configure FLL external reference divider (FRDIV). */
	MCG->C1=((MCG->C1&~MCG_C1_FRDIV_MASK)|MCG_C1_FRDIV(0x0U));

	/* Set MCG to BLPE mode. */
    MCG->C7=(MCG->C7&~MCG_C7_OSCSEL_MASK)|MCG_C7_OSCSEL(0x0U);

	while(i--);

	/* Set to FBE mode. */
	MCG->C1=((MCG->C1&~(MCG_C1_CLKS_MASK|MCG_C1_IREFS_MASK))|(MCG_C1_CLKS(0x2U)|MCG_C1_IREFS(0x0U)));

	/* If use external crystal as clock source, wait for it stable. */
	while(!(MCG->S&MCG_S_OSCINIT0_MASK));

    /* Wait for MCG_S[CLKST] and MCG_S[IREFST]. */
	while((MCG->S&(MCG_S_IREFST_MASK|MCG_S_CLKST_MASK))!=(MCG_S_IREFST(0x0U)|MCG_S_CLKST(0x2U)));

	/* In FBE now, start to enter BLPE. */
	MCG->C2|=MCG_C2_LP_MASK;

	/* Set the clock configuration in SIM module. */
	SIM->CLKDIV1=SIM_CLKDIV1_OUTDIV1(4)|SIM_CLKDIV1_OUTDIV1(4)|SIM_CLKDIV1_OUTDIV1(4)|SIM_CLKDIV1_OUTDIV1(16);
	SIM->SOPT2=((SIM->SOPT2&~SIM_SOPT2_PLLFLLSEL_MASK)|SIM_SOPT2_PLLFLLSEL(0x0U));
	SIM->CLKDIV3=SIM_CLKDIV3_PLLFLLDIV(0x0U)|SIM_CLKDIV3_PLLFLLFRAC(0x0U);
	SIM->SOPT1=((SIM->SOPT1&~SIM_SOPT1_OSC32KSEL_MASK)|SIM_SOPT1_OSC32KSEL(0x0U));

	/* configure VLPR mode */
	SMC->PMCTRL=(SMC->PMCTRL&~SMC_PMCTRL_RUNM_MASK)|(0x2U<<SMC_PMCTRL_RUNM_SHIFT);

	/* wait for mode to enter */
	while(SMC->PMSTAT!=(0x01U<<2U));

	/* Set SystemCoreClock variable. */
	SystemCoreClock=4000000U;
}

void Set_500KHz_Low_Power_Clock(void)
{
	/* Set the system clock dividers in SIM to safe value. */
	SIM->CLKDIV1=0x02260000U;

    /* Configure RTC clock including enabling RTC oscillator. */
	/* RTC clock gate enable */
	SIM->SCGC6|=SIM_SCGC6_RTC_MASK;

	if((RTC->CR&RTC_CR_OSCE_MASK)==0u)
	{
		/* Only if the Rtc oscillator is not already enabled */
		/* Set the specified capacitor configuration for the RTC oscillator */
		RTC->CR=(RTC->CR&~(RTC_CR_SC2P_MASK|RTC_CR_SC4P_MASK|RTC_CR_SC8P_MASK|RTC_CR_SC16P_MASK))|0x0U;

		/* Enable the RTC 32KHz oscillator */
		RTC->CR|=RTC_CR_OSCE_MASK;
	}

	/* Output to other peripherals */
	RTC->CR&=~RTC_CR_CLKO_MASK;

	/* Set RTC_TSR if there is fault value in RTC */
	if(RTC->SR&RTC_SR_TIF_MASK)
		RTC->TSR=RTC->TSR;

	/* RTC clock gate disable */
	SIM->SCGC6&=~SIM_SCGC6_RTC_MASK;

	/* Set MCG to BLPI mode. */

	MCG->C2=(MCG->C2&~MCG_C2_IRCS_MASK)|(MCG_C2_IRCS(0x1U));
	MCG->C1=(MCG->C1&~(MCG_C1_IRCLKEN_MASK|MCG_C1_IREFSTEN_MASK))|(uint8_t)(MCG_C1_IRCLKEN_MASK|MCG_C1_IREFSTEN_MASK);

	while(((MCG->S&MCG_S_IRCST_MASK)>>MCG_S_IRCST_SHIFT)!=0x1U);

	/* If reset mode is not BLPI, first enter FBI mode. */
	MCG->C1=(MCG->C1&~MCG_C1_CLKS_MASK)|MCG_C1_CLKS(0x1U);

	while(((MCG->S&MCG_S_CLKST_MASK)>>MCG_S_CLKST_SHIFT)!=0x1U);

	/* Enter BLPI mode. */
	MCG->C2|=MCG_C2_LP_MASK;

	/* Set the clock configuration in SIM module. */
	SIM->CLKDIV1=SIM_CLKDIV1_OUTDIV1(8)|SIM_CLKDIV1_OUTDIV1(8)|SIM_CLKDIV1_OUTDIV1(8)|SIM_CLKDIV1_OUTDIV1(8);
	SIM->SOPT2=((SIM->SOPT2&~SIM_SOPT2_PLLFLLSEL_MASK)|SIM_SOPT2_PLLFLLSEL(0x0U));
	SIM->CLKDIV3=SIM_CLKDIV3_PLLFLLDIV(0x0U)|SIM_CLKDIV3_PLLFLLFRAC(0x0U);
	SIM->SOPT1=((SIM->SOPT1&~SIM_SOPT1_OSC32KSEL_MASK)|SIM_SOPT1_OSC32KSEL(0x0U));

	/* configure VLPR mode */
	SMC->PMCTRL=(SMC->PMCTRL&~SMC_PMCTRL_RUNM_MASK)|(0x2U<<SMC_PMCTRL_RUNM_SHIFT);

	/* wait for mode to enter */
	while(SMC->PMSTAT!=(0x01U<<2U));

	SystemCoreClock=500000U;
}

void __init_hardware(void)
{
	// Set the interrupt vector table position
	SCB->VTOR=(uint32_t)(&__vect_table);

#if (DISABLE_WDOG)
	// Watchdog disable
	// WDOG->UNLOCK: WDOGUNLOCK=0xC520
	WDOG->UNLOCK = WDOG_UNLOCK_WDOGUNLOCK(0xC520); /* Key 1 */
	/* WDOG->UNLOCK: WDOGUNLOCK=0xD928 */
	WDOG->UNLOCK = WDOG_UNLOCK_WDOGUNLOCK(0xD928); /* Key 2 */
	/* WDOG->STCTRLH: ?=0,DISTESTWDOG=0,BYTESEL=0,TESTSEL=0,TESTWDOG=0,?=0,?=1,WAITEN=1,STOPEN=1,DBGEN=0,ALLOWUPDATE=1,WINEN=0,IRQRSTEN=0,CLKSRC=1,WDOGEN=0 */
	WDOG->STCTRLH=WDOG_STCTRLH_BYTESEL(0x00)|WDOG_STCTRLH_WAITEN_MASK|WDOG_STCTRLH_STOPEN_MASK|WDOG_STCTRLH_ALLOWUPDATE_MASK|WDOG_STCTRLH_CLKSRC_MASK|0x0100U;
#endif

#if ((__FPU_PRESENT == 1) && (__FPU_USED == 1))
	// set CP10, CP11 Full Access
	SCB->CPACR|=((3UL<<10*2)|(3UL<<11*2));
#endif

	if(PMC->REGSC&PMC_REGSC_ACKISO_MASK)
		PMC->REGSC|=PMC_REGSC_ACKISO_MASK;

	SMC->PMPROT=SMC_PMPROT_AVLLS_MASK|SMC_PMPROT_ALLS_MASK|SMC_PMPROT_AVLP_MASK|SMC_PMPROT_AHSRUN_MASK;

	Set_120MHz_Clock();
}

void Cpu_SetBASEPRI(uint32_t Level)
{
	__asm__ volatile ("msr basepri, %[input]"::[input] "r" (Level):);
}

void __low_level_init(void)
{
	SIM->SCGC5|=SIM_SCGC5_PORTD_MASK|SIM_SCGC5_PORTC_MASK|SIM_SCGC5_PORTB_MASK|SIM_SCGC5_PORTA_MASK;

	// Set up SysTick
	SysTick->LOAD=(120000000/1000U)-1;
	SysTick->CTRL=SysTick_CTRL_CLKSOURCE_Msk|SysTick_CTRL_TICKINT_Msk|SysTick_CTRL_ENABLE_Msk;

	// Enable interrupts of the given priority level
	Cpu_SetBASEPRI(0);

	__asm volatile ("cpsie i");
}

void __thumb_startup(void)
{
	// setup hardware
	__init_hardware();

	// zero-fill the .bss section
	__zero_fill_bss();

	// SUPPORT_ROM_TO_RAM
	__copy_rom_sections_to_ram();

	__low_level_init();

	main();

	// should never get here
	while(1);
}
