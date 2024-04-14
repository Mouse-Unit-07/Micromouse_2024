/*-------------------------------- FILE INFO -----------------------------------
* Filename        : tc_at32uc3l0256.h
* Author          : Ryuichi Lin
* Revision        : 1.0
* Updated         : 2024-03-31
* Purpose         : hardware specific layer
*
* This file is the header file for hardware specific TC code.
*
* Step 3 for hardware abstraction.
*
* TODO:
*     - Break down TC init function (decouple from micromouse application)
*     - Add functions to modify TC parameters during runtime
*
*     Contract and config file simplified by tailoring hardware specific code
*     to micromouse application. HAL not scalable to broader MCU use.
*-----------------------------------------------------------------------------*/

#ifndef TC_AT32UC3L0256_H_
#define TC_AT32UC3L0256_H_

/*----------------------------------------------------------------------------*/
/*                                Definitions                                 */
/*----------------------------------------------------------------------------*/
#define MM_TIMER_COUNTER_BASE_ADDRESS    (&AVR32_TC1)
#define MM_TIMER_COUNTER_CHANNEL         (0)
#define MM_TIMER_COUNTER_IRQ             (AVR32_TC1_IRQ0)
#define MM_TIMER_COUNTER_IRQ_PRIORITY    (AVR32_INTC_INT0)

/* clock settings */
#define MM_PBA_CLK_FREQ_HZ               MM_PBA_FREQ_HZ

/*----------------------------------------------------------------------------*/
/*                              Global Variables                              */
/*----------------------------------------------------------------------------*/
/* timer counter count variable */
volatile uint32_t g_mm_TimerCounterCount;

/*----------------------------------------------------------------------------*/
/*                       Public (Exportable) Functions                        */
/*----------------------------------------------------------------------------*/
tc_status_t at32uc3l0256_InitTc(void);
tc_status_t at32uc3l0256_GetTimerCounterCount(uint32_t* p_timerCounterCount);

#endif /* TC_AT32UC3L0256_H_ */