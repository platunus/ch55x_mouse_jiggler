#include <stdint.h>

#include <ch554.h>
#include <debug.h>

#include "ch55x_mouse.h"

#define JIGGLER_INTERVAL 50000		// ms
#define JIGGLER_MOVING_DISTANCE 2	// pixels
#define LED_ON_PERIOD 100			// ms

#define TIMER0_INTERVAL 1000	// us -> 1ms interval

// Indicator LED: P1.4
#define LED_PIN_PORT_MOD 	P1_MOD_OC
#define LED_PIN_NO       	4
#define LED 				T2_

volatile __idata uint32_t mills;
volatile __idata int8_t deltaX = JIGGLER_MOVING_DISTANCE;

void Timer0Init()
{
	T2MOD = (T2MOD | bTMR_CLK) & ~bT0_CLK;				// Fsys/12 = 2MHz
	TMOD = TMOD | bT0_M0;								// Mode1: 16bit timer
	PT0 = 0;											// Low priorty 
	ET0 = 1;											// Interrupt enable
	TH0 = (0 - FREQ_SYS / 12 / 1000000 * TIMER0_INTERVAL ) >> 8; 
	TL0 = (0 - FREQ_SYS / 12 / 1000000 * TIMER0_INTERVAL ) & 0xff; 
	TR0 = 1;											// Timer0 start

	EA = 1;
}

void Timer0InterruptHandler(void) __interrupt (INT_NO_TMR0) 
{
	TH0 = (0 - FREQ_SYS / 12 / 1000000 * TIMER0_INTERVAL ) >> 8; 
	TL0 = (0 - FREQ_SYS / 12 / 1000000 * TIMER0_INTERVAL ) & 0xff; 
	mills++;
}

void LEDInit()
{
	LED_PIN_PORT_MOD = LED_PIN_PORT_MOD & ~(1 << LED_PIN_NO);		// push-pull
	LED = 0;
}

main()
{
	CfgFsys();

	mDelaymS(5);

	ch55x_mouse_init();
	Timer0Init();

	LEDInit();


	mills = 0;
	while(1)
	{
		if ( mills > JIGGLER_INTERVAL )
		{
			ch55x_mouse_move(deltaX, 0);
			LED = 1;
			mills = 0;
			deltaX = -deltaX;
		}
		else if ( mills > LED_ON_PERIOD )
		{
			LED = 0;
		}
	}
}
