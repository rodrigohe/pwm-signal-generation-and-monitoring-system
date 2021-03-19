//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//
// ----------------------------------------------------------------------------
//
// School: University of Victoria, Canada.
// Course: ECE 355 "Microprocessor-Based Systems".
// This is template code for Part 2 of Introductory Lab.
//
// Rodrigo He
//
// ----------------------------------------------------------------------------
//
// See "system/include/cmsis/stm32f0xx.h" for register/bit definitions.
// See "system/src/cmsis/vectors_stm32f0xx.c" for handler declarations.
//
// ----------------------------------------------------------------------------

#include <stdio.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"

// ----------------------------------------------------------------------------
//
// STM32F0 empty sample (trace via $(trace)).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the $(trace) output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

/*****************************************************************************/
/******************************* D E F I N E S *******************************/
/*****************************************************************************/

/* Clock prescaler for TIM2 timer: no prescaling */
#define myTIM2_PRESCALER ((uint16_t)0x0000)
/* Clock prescaler for TIM3 timer: no prescaling */
#define myTIM3_PRESCALER ((uint16_t)0x0000)
/* Maximum possible setting for overflow */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)

/*****************************************************************************/
/******************* F U N C T I O N S    P R O T O T Y P E ******************/
/*****************************************************************************/

void myGPIOA_Init(void);
void myGPIOB_Init(void);
void myGPIOC_Init(void);
void myTIM2_Init(void);
void myTIM3_Init(void);
void myEXTI_Init(void);
void myADC_Init(void);
void myDAC_Init(void);
void myLCD_Init(void);
void setLCD_DDRAMhome(void);
void setLCD_DisplayLine(uint32_t);
void sendLCD_data(uint32_t);
void displayValues(uint32_t, uint32_t);

/*****************************************************************************/
/********************* G L O B A L    V A R I A B L E S **********************/
/*****************************************************************************/

unsigned char timerTriggered = 0;       /* 1 = triggered, 0 = not triggered */
unsigned char currentDisplayLine = 1;	/* 1 = first line, 2 = second line */

float resistance = 0;
float frequency = 0;
float period = 0;

/*****************************************************************************/
/************************ I M P L E M E N T A T I O N ************************/
/*****************************************************************************/

int main(int argc, char* argv[])
{
	trace_printf("This is the Final Lab Project...\n");
	trace_printf("System clock: %u Hz\n", SystemCoreClock);

	myGPIOA_Init();		/* Initialize I/O port PA */
	myGPIOB_Init();		/* Initialize I/O port PB */
	myGPIOC_Init();		/* Initialize I/O port PC */
	myTIM2_Init();		/* Initialize timer TIM2 */
	myTIM3_Init();		/* Initialize timer TIM3 */
	myEXTI_Init();		/* Initialize EXTI */
    myADC_Init();       /* Initialize ADC */
    myDAC_Init();       /* Initialize DAC */
	myLCD_Init();		/* Initialize LCD */

    trace_printf("*** Starting ADC Sampling... ***\n");

	while(1)
    {
		/* Set ADC1 control register to start conversion*/
		ADC1->CR |= ADC_CR_ADSTART;

		/* Wait for "end of conversion" flag */
		while((ADC1->ISR & ADC_ISR_EOC) == 0){
			//trace_printf("*** main() - Waiting For ADC Sample... ***\n");
		}

        /* Retrieve value from ADC data register */
		uint32_t ADCval = ADC1->DR;

        /* Send ADC value to DAC data register */
		DAC->DHR12R1 = ADCval;

        /* Calculate Resistance using ADC val;
         * resistance = (ADCval * <POT Max Resistance>) / <ADC Resolution> */
		resistance = (float)(ADCval)*5000/0x0FFF;

		//trace_printf("\n*** Frequency : %f Hz***\n", frequency);
        //trace_printf("*** Resistance: %f Oh***\n", resistance);
	}

	return 0;
}


void myGPIOA_Init()
{
	/* PA1 - Input (555 Timer)
	 * PA4 - Output **Analog** (DAC)
	 */

	/* Enable clock for GPIOA peripheral */
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    /* Configure PA1 as input */
	/* Ensure no pull-up/pull-down for PA1 */
	GPIOA->MODER &= ~(GPIO_MODER_MODER1);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1);

	/* Configure PA4 as output */
	/* Ensure no pull-up/pull-down for PC4 */
	GPIOA->MODER |= GPIO_MODER_MODER4_0;
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4);
}

void myGPIOB_Init()
{
	/*  PB4 - Output (ENB ; LDC Handshaking: "Enable")
	 *  PB5 - Output (RS  ; 0 = Command, 1 = Data)
	 *  PB6 - Output (R/W ; 0 = Write,   1 = Read)
	 *  PB7 - Input  (DONE; LDC Handshaking: "Done")
	 *  PB8 - Output (D0  ; Data Signal)
	 *  PB9 - Output (D1  ; Data Signal)
	 *   :
	 * PB15 - Output (D7  ; Data Signal)
	 */

	/* Enable clock for GPIOB peripheral */
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	/* Configure PB7 as input */
	/* Ensure no pull-up/pull-down for PB7 */
	GPIOB->MODER &= ~(GPIO_MODER_MODER7);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR7);

	/* Configure PB4-6, PB8-15 as output */
	/* Ensure no pull-up/pull-down for PB4-6, PB8-15 */
	GPIOB->MODER |= GPIO_MODER_MODER4_0;
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR4);

	GPIOB->MODER |= GPIO_MODER_MODER5_0;
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR5);

	GPIOB->MODER |= GPIO_MODER_MODER6_0;
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR6);

	GPIOB->MODER |= GPIO_MODER_MODER8_0;
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR8);

	GPIOB->MODER |= GPIO_MODER_MODER9_0;
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR9);

	GPIOB->MODER |= GPIO_MODER_MODER10_0;
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR10);

	GPIOB->MODER |= GPIO_MODER_MODER11_0;
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR11);

	GPIOB->MODER |= GPIO_MODER_MODER12_0;
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR12);

	GPIOB->MODER |= GPIO_MODER_MODER13_0;
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR13);

	GPIOB->MODER |= GPIO_MODER_MODER14_0;
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR14);

	GPIOB->MODER |= GPIO_MODER_MODER15_0;
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR15);
}

void myGPIOC_Init()
{
	/* PC1 - Input  **Analog** (ADC) */

	/* Enable clock for GPIOC peripheral */
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

	/* Configure PC1 as input */
	/* Ensure no pull-up/pull-down for PC1 */
	GPIOC->MODER &= ~(GPIO_MODER_MODER1);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR1);

}

void myTIM2_Init()
{
	/* Enable clock for TIM2 peripheral */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	/* Configure TIM2: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
    TIM2->CR1 = 0x008C;

	/* Set clock prescaler value */
	TIM2->PSC = myTIM2_PRESCALER;

	/* Set auto-reloaded delay */
	TIM2->ARR = myTIM2_PERIOD;

	/* Update timer registers */
    TIM2->EGR = 0x0001;

	/* Assign TIM2 interrupt priority = 0 in NVIC */
    NVIC_SetPriority(TIM2_IRQn, 0);

	/* Enable TIM2 interrupts in NVIC */
    NVIC_EnableIRQ(TIM2_IRQn);

	/* Enable update interrupt generation */
    TIM2->DIER |= TIM_DIER_UIE;
}

void myTIM3_Init()
{
	/* Enable clock for TIM3 peripheral */
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	/* Configure TIM3: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
    TIM3->CR1 = 0x008C;

	/* Set clock prescaler value */
    TIM3->PSC = myTIM3_PRESCALER;

	/* Set auto-reloaded delay: 1/24sec (24 FPS) at 48MHz */
    TIM3->ARR = (uint32_t)(SystemCoreClock/24);

	/* Update timer registers */
    TIM3->EGR = 0x0001;

	/* Assign TIM3 interrupt priority = 0 in NVIC */
    NVIC_SetPriority(TIM3_IRQn, 1);

	/* Enable TIM3 interrupts in NVIC */
    NVIC_EnableIRQ(TIM3_IRQn);

	/* Enable update interrupt generation */
    TIM3->DIER |= TIM_DIER_UIE;

    /* Start timer (TIM3->CR1) */
    TIM3->CR1 |= TIM_CR1_CEN;
}

void myEXTI_Init()
{
	/* Map EXTI1 line to PA1 */
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PA;

	/* EXTI1 line interrupts: set rising-edge trigger */
    EXTI->RTSR |= EXTI_RTSR_TR1;

	/* Unmask interrupts from EXTI1 line */
    EXTI->IMR |= EXTI_IMR_MR1;

	/* Assign EXTI1 interrupt priority = 0 in NVIC */
    NVIC_SetPriority(EXTI0_1_IRQn, 0);

	/* Enable EXTI1 interrupts in NVIC */
    NVIC_EnableIRQ(EXTI0_1_IRQn);
}


void myADC_Init()
{
    /* Enable clock for ADC1 peripheral */
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

	/* Configure ADC1 configuration register 1:
	 *										bits	config
	 * 		data resolution to 12-bits;		[4:3]	RES = 00
	 * 		right data alignment;			[5]		ALIGN = 0
	 * 		overrun management mode; 		[12]	OVRMOD = 1
	 * 		continuous conversation mode;	[13]	CONT = 1
	 */
    ADC1->CFGR1 |= (ADC_CFGR1_CONT | ADC_CFGR1_OVRMOD);

	/* Configure ADC1 channel selection register to channel 11 */
    ADC1->CHSELR |= ADC_CHSELR_CHSEL11;

	/* Configure ADC1 sampling time selection to 239.5 clock cycles*/
    ADC1->SMPR |= ADC_SMPR_SMP;

	/* Configure ADC1 enable control */
    ADC1->CR |= ADC_CR_ADEN;

    /* Configure ADC1 interrupt and status register*/
	/* Wait until ISR is ready */
    while((ADC1->ISR & ADC_ISR_ADRDY) == 0){
		// trace_printf("*** myADC_Init() - Waiting for ADC Init... ***\n");
	}
	// trace_printf("*** ADC Init Complete... ***\n");
}

void myDAC_Init()
{
    /* Enable clock for DAC peripheral */
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;

    /* Configure DAC data register, 12-bit right-aligned*/
    DAC->DHR12R1 |= DAC_DHR12R1_DACC1DHR;

    /* Configure DAC control register:
	 *										bits	config
	 * 		channel 1 enabled				[0]		EN1 = 1
	 * 		channel 1 tri-state buffer dis.	[1]		BOFF1 = 0
	 * 		channel 1 trigger disabled		[2]		TEN1 = 0
	 */
    DAC->CR |= DAC_CR_EN1;
}


void myLCD_Init(){
	/* Setting LCD Initialization Command Values
	 *      Following Interface Slides, pg. 5
	 *
	 * set8BitInterface : DL = 1, N = 1, F = 0
	 * 					= 0b 00 0011 1000 = 0x38 = 0x3800
	 *
	 * setDisplayOn 	: D = 1, C = 0, B = 0
	 * 					= 0b 00 0000 1100 = 0x0C = 0x0C00
	 *
	 * setEntryModeInc	: I/D = 1, S = 0
	 * 					= 0b 00 0000 0110 = 0x06 = 0x0600
	 *
	 * setDisplayClear 	: -
	 * 					= 0b 00 0000 0001 = 0x01 = 0x0100
	 *
	 * 		Note: Values are shifted left by 8 to match correct data pins
	 */

	uint32_t set8BitInterface = 0x3800;
	uint32_t setDisplayOn = 0x0C00;
	uint32_t setEntryModeInc = 0x0600;
	uint32_t setDisplayClear = 0x0100;

    /* Put all initialization commands into an array */
	uint32_t LCDcmdArray[4] = {set8BitInterface, setDisplayOn,
								setEntryModeInc, setDisplayClear};

	/* Loop through array and send commands to LCD*/
	int i;
	for(i = 0; i < 4; i ++){
		sendLCD_data(LCDcmdArray[i]);
	}
}


void sendLCD_data(uint32_t data){
	/* Sending LCD ASCII Codes
	 *      Following Interface Slides, pg. 9-10
	 *
	 * 1. Send to data to Port B
	 * 2. Assert Handshake; set PB[4] = 1
	 * 3. Wait till "Done"; wait for PB[7] = 1
	 * 4. Deassert Handshake; set PB[4] = 0;
	 * 5. Wait till "Done"; wait for PB[7] = 0
	 */

    /* Send data code to Port B */
	GPIOB->ODR = data;

	/* Handshaking: assert "Enable"; PB[4] = 1*/
	GPIOB->ODR |= GPIO_ODR_4;

	/* Wait Handshaking: "Done" to be asserted; PB[7] = 1*/
	while((GPIOB->IDR & GPIO_IDR_7) == 0){
		//trace_printf("*** sendLCD_data() - 1st while ***\n");
	}

	/* Handshaking: DEassert "Enable"; PB[4] = 0 */
	GPIOB->ODR &= ~(GPIO_ODR_4);

	/* Wait Handshaking: "Done" to be deasserted; PB[7] = 0 */
	while((GPIOB->IDR & GPIO_IDR_7) != 0){
		//trace_printf("*** sendLCD_data() - 2nd while ***\n");
	}
}

void setLCD_DDRAMhome(){
	/* "Sets DDRAM address 0 in address counter. Also returns display from
	 * being shifted to original position. DDRAM contents remain unchanged."
     *
     *      More info see Page 24 at
     *      https://www.ece.uvic.ca/~ece355/lab/supplement/HD44780.pdf
     *
	 * returnHome   = 0b 00 0000 001x , where x can be anything.
     *              = 0x02 = 0x0200 (Shift left by 8 to match correct PBx)
	 */
	sendLCD_data(0x0200);
	currentDisplayLine = 1;
}

void setLCD_DisplayLine(uint32_t line){

    /* Initialize DDRAM address */
	uint32_t DDRAMaddress;

    /* Check that currentDisplayLine does not match given line number,
     * then set DDRAM address to match correct line
     */

	if(line != currentDisplayLine){
		if(line == 1){
			DDRAMaddress = 0x8000;
		}
		if(line == 2){
			DDRAMaddress = 0xC000;
		}
		sendLCD_data(DDRAMaddress);
		currentDisplayLine = line;
	}
}

void displayValues(uint32_t value, uint32_t line){
    /*	To extract a digit from a given value we use the following:
	 *		digit = (<Value>/<digitPositionInValue>) % 10
	 *  e.g. Given the value 1234
     *			thousandPlace = (1234/1000) % 10 = 1
     * 			onePlace = (1234/1) % 10 = 4
     */

	/* Note: Must add 48 to final val in order to covert val to ASCII */
	uint32_t thousand = ((value/1000) % 10) + 48;
	uint32_t hundred = ((value/100) % 10) + 48;
	uint32_t tenth = ((value/10) % 10) + 48;
	uint32_t one = ((value/1) % 10) + 48;

    /* Initialize alpha characters to be displayed */
	uint32_t firstLetter; 	/* 'F' or 'R' */
	uint32_t secondLetter;	/* 'H' or 'O' */
	uint32_t thirdLetter;	/* 'z' or 'h' */
	uint32_t colon = 0x3A;

    /* Set alpha characters wrt line number */
	if(line == 1){
		firstLetter = 0x46;		// 'F'
		secondLetter = 0x48; 	// 'H'
		thirdLetter = 0x7A;		// 'z'
	}

	if(line == 2){
		firstLetter = 0x52;		// 'R'
		secondLetter = 0x4F; 	// 'O'
		thirdLetter = 0x68;		// 'h'
	}

    /* Check if DDRAM address matches correct display line */
	if(currentDisplayLine != 1 && line == 1){
		setLCD_DisplayLine(1);
	}
	if(currentDisplayLine != 2 && line == 2){
		setLCD_DisplayLine(2);
	}

    /* Put all characters into an array */
	uint32_t valArray[8] = {firstLetter, colon,
							thousand, hundred, tenth, one,
							secondLetter, thirdLetter};

    /* Loop through valArray:
     * 		Shift value left by 8 to match PB[15:8]
     * 		Bit mask data s.t. RS is set to receive data, RS = 1; PB[5]
     * 		Send data to LCD
     */
	int i;
	for(i = 0; i < 8; i++){
		valArray[i] <<= 8;
		valArray[i] |= 0x20;
		sendLCD_data(valArray[i]);
	}
}

/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void TIM2_IRQHandler()
{
	/* Check if update interrupt flag is indeed set */
	if ((TIM2->SR & TIM_SR_UIF) != 0)
	{
		trace_printf("\n*** Overflow! ***\n");

		/* Clear update interrupt flag */
        TIM2->SR &= ~(TIM_SR_UIF);

		/* Restart stopped timer */
        TIM2->CR1 |= TIM_CR1_CEN;
	}
}

void TIM3_IRQHandler()
{
	/* Check if update interrupt flag is indeed set */
	if ((TIM3->SR & TIM_SR_UIF) != 0)
	{
        /* Display frequency and resistance */
		displayValues(frequency, 1);
		displayValues(resistance, 2);

        /* Set DDRAM address to 0, keep data unchanged*/
		setLCD_DDRAMhome();

		/* Clear update interrupt flag */
        TIM3->SR &= ~(TIM_SR_UIF);

		/* Restart stopped timer */
        TIM3->CR1 |= TIM_CR1_CEN;
	}
}

/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void EXTI0_1_IRQHandler()
{
 	float clockCycles = 0;

 	/* Check if EXTI1 interrupt pending flag is indeed set */
 	if ((EXTI->PR & EXTI_PR_PR1) != 0)
    {
        if(timerTriggered == 0)	/* If timerTriggered is 0*/
        {
            /* Set timerTriggered to 1 */
            timerTriggered = 1;

            /* Clear count register (TIM2->CNT) */
            TIM2->CNT = 0x0;

            /* Start timer (TIM2->CR1) */
            TIM2->CR1 |= TIM_CR1_CEN;
        }
        else    /* If timerTriggered is 1*/
        {
            /* Stop timer (TIM2->CR1) */
            TIM2->CR1 &= ~(TIM_CR1_CEN);

            /* Read out count register (TIM2->CNT) */
            clockCycles = TIM2->CNT;

            /* Calculate signal period and frequency */
            frequency = ((float)SystemCoreClock) / clockCycles;
            period = 1 / frequency;

            /* Print calculated values to the console */
            //trace_printf("*** Period = %f sec ***\n", period);
            //trace_printf("*** Frequency = %f Hz ***\n", frequency);

            /* Set timerTriggered flag to 0 */
            timerTriggered = 0;
        }

        /* Clear EXTI1 interrupt pending flag (EXTI->PR) */
        EXTI->PR |= EXTI_PR_PR1;
   }
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------