/*-------------------------------- FILE INFO -----------------------------------
* Filename        : tc_at32uc3l0256.c
* Author          : Ryuichi Lin
* Revision        : 1.0
* Updated         : 2024-03-31
* Purpose         : hardware specific layer
*
* This file is the header file for hardware specific TC code.
*
* Step 3 for hardware abstraction.
*
* (if applicable)
* Target Hardware    : AT32UC3L0256
* IDE                : Atmel Studio 7.4.2542
* SDK                : ASF 3.52.0
*-----------------------------------------------------------------------------*/ 

/*----------------------------------------------------------------------------*/
/*                               Include Files                                */
/*----------------------------------------------------------------------------*/
#include <asf.h>
#include <stdint.h>
#include "HAL/HAL_contracts/tc_contract.h"
#include "HAL/at32uc3l0256/tc_at32uc3l0256.h"
#include "HAL/HAL_contracts/clock_contract.h"       /* needed for PBA freq */
#include "HAL/at32uc3l0256/clock_at32uc3l0256.h"    /* needed for PBA freq */

/*----------------------------------------------------------------------------*/
/*                                Definitions                                 */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                               Debug Switches                               */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                              Global Variables                              */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                    Private (Static) Function Prototype                     */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                         Interrupt Service Routines                         */
/*----------------------------------------------------------------------------*/
/**
* ISR for timer counter interrupt
*
* \param    None
* \retval   None
*/
__attribute__((__interrupt__))
static void tc_irq(void)
{
    /* Clear the interrupt flag. This is a side effect of reading the TC SR */
    tc_read_sr(MM_TIMER_COUNTER_BASE_ADDRESS, MM_TIMER_COUNTER_CHANNEL);
    
    g_mm_TimerCounterCount++;
}

/*----------------------------------------------------------------------------*/
/*                       Public (Exportable) Functions                        */
/*----------------------------------------------------------------------------*/
/**
* Initialize TC interface for AT32UC3L0256 MCU.
*
* \retval TC_SUCCESS Success
* \retval TC_ERROR Failure: Failed to initialize timer counter
*/
tc_status_t at32uc3l0256_InitTc(void)
{
    tc_status_t tcStatus = TC_ERROR;
    
    /* Options for waveform generation */
	static const tc_waveform_opt_t waveform_opt = {
		.channel  = MM_TIMER_COUNTER_CHANNEL,           /* Channel selection */
		.bswtrg   = TC_EVT_EFFECT_NOOP,                 /* Software trigger effect on TIOB */
		.beevt    = TC_EVT_EFFECT_NOOP,                 /* External event effect on TIOB */
		.bcpc     = TC_EVT_EFFECT_NOOP,                 /* RC compare effect on TIOB */
		.bcpb     = TC_EVT_EFFECT_NOOP,                 /* RB compare effect on TIOB */
		.aswtrg   = TC_EVT_EFFECT_NOOP,                 /* Software trigger effect on TIOA */
		.aeevt    = TC_EVT_EFFECT_NOOP,                 /* External event effect on TIOA */
		.acpc     = TC_EVT_EFFECT_NOOP,                 /* RC compare effect on TIOA */
		.acpa     = TC_EVT_EFFECT_NOOP,                 /* RA compare effect on TIOA (none, set and clear) */
		.wavsel   = TC_WAVEFORM_SEL_UP_MODE_RC_TRIGGER, /* Up mode w/ auto trigger(reset) on RC compare */
		.enetrg   = false,                              /* External event trigger enable */
		.eevt     = 0,                                  /* External event selection */
		.eevtedg  = TC_SEL_NO_EDGE,                     /* External event edge selection */
		.cpcdis   = false,                              /* Counter disable when RC compare */
		.cpcstop  = false,                              /* Counter clock stopped with RC compare */
		.burst    = false,                              /* Burst signal selection */
		.clki     = false,                              /* Clock inversion */ 
		.tcclks   = TC_CLOCK_SOURCE_TC3                 /* Internal source clock 3, connected to fPBA / 8 */
	};

	/* Options for enabling TC interrupts */
	static const tc_interrupt_t tc_interrupt = {
		.etrgs = 0,
		.ldrbs = 0,
		.ldras = 0,
		.cpcs  = 1, /* Enable interrupt on RC compare alone */
		.cpbs  = 0,
		.cpas  = 0,
		.lovrs = 0,
		.covfs = 0
	};
	
	/* Initialize the timer/counter */
	if (tc_init_waveform(MM_TIMER_COUNTER_BASE_ADDRESS, &waveform_opt)
        == TC_INVALID_ARGUMENT)
        tcStatus = TC_ERROR;
    else
    {
        /* Set the compare triggers */
        /* We configure it to count every 1 milliseconds */
        /* We want: (1 / (fPBA / 8)) * RC = 1 ms; RC = (fPBA / 8) / 1000 */
        /* to get an interrupt every 10 ms */
        if (tc_write_rc(
            MM_TIMER_COUNTER_BASE_ADDRESS, 
            MM_TIMER_COUNTER_CHANNEL, 
            MM_PBA_CLK_FREQ_HZ / 8 / 1000 * 4) == TC_INVALID_ARGUMENT)
            tcStatus = TC_ERROR;
        else
        {
             /* configure the timer interrupt */
             if (tc_configure_interrupts(
                 MM_TIMER_COUNTER_BASE_ADDRESS, 
                 MM_TIMER_COUNTER_CHANNEL, 
                 &tc_interrupt) == TC_INVALID_ARGUMENT)
                 tcStatus = TC_ERROR;
             else
             {
                 /* Start the timer/counter */
                 if (tc_start(
                     MM_TIMER_COUNTER_BASE_ADDRESS, 
                     MM_TIMER_COUNTER_CHANNEL) == TC_INVALID_ARGUMENT)
                     tcStatus = TC_ERROR;
                 else
                     tcStatus = TC_SUCCESS;
             }
        }
    }
    
    /* Enable the clock to timer counter module */
    sysclk_enable_peripheral_clock(MM_TIMER_COUNTER_BASE_ADDRESS);
    
    /* Register the RTC interrupt handler to the interrupt controller */
    INTC_register_interrupt(&tc_irq, MM_TIMER_COUNTER_IRQ, MM_TIMER_COUNTER_IRQ_PRIORITY);
    
    /* return status */
    return tcStatus;
}

/**
* Access function for timer counter count variable for AT32UC3L0256 MCU.
*
* \param[out] p_timerCounterCount Counter variable for timer counter
* \retval TC_SUCCESS Success
* \retval TC_ERROR Failure: Failed to read timer counter count variable
*/
tc_status_t at32uc3l0256_GetTimerCounterCount(uint32_t* p_timerCounterCount)
{
    tc_status_t tcStatus = TC_ERROR;
    
    *p_timerCounterCount = g_mm_TimerCounterCount;
    
    tcStatus = TC_SUCCESS;
    
    /* return status */
    return tcStatus;
}

/*----------------------------------------------------------------------------*/
/*                           Local Shared Functions                           */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                         Private (Static) Functions                         */
/*----------------------------------------------------------------------------*/
/* None */
