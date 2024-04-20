/*-------------------------------- FILE INFO -----------------------------------
* Filename        : clock_at32uc3l0256.h
* Author          : Team Kirbo
* Revision        : 1.0
* Updated         : 2024-03-31
* Purpose         : hardware specific layer
*
* This file is the header file for hardware specific clock code.
*
* Step 3 for hardware abstraction.
*
* TODO: 
*     - Break down clock init function (decouple from micromouse application)
*     - Add functions to change clock parameters during runtime
* 
*     Contract and config file simplified by tailoring hardware specific code
*     to micromouse application. HAL not scalable to broader MCU use.
*-----------------------------------------------------------------------------*/

#ifndef CLOCK_AT32UC3L0256_H_
#define CLOCK_AT32UC3L0256_H_

/*----------------------------------------------------------------------------*/
/*                                Definitions                                 */
/*----------------------------------------------------------------------------*/
/* *** DFLL only mode *** */
/* target DFLL base frequency */
#define MM_DFLL_ONLY_CLK_FREQ_HZ         (130000000u)
/* F_CPU = (DFLL base) / 2^2 = 35MHz */
#define MM_DFLL_ONLY_FCPU_PRESCALER      (2u)
/* PBA = (DFLL base) / 2^1 = 70MHz */
#define MM_DFLL_ONLY_PBA_PRESCALER       (1u)
/* PBB = (DFLL base) / 2^1 = 70MHz */
#define MM_DFLL_ONLY_PBB_PRESCALER       (1u)

/* FCPU frequency */
#define MM_PBA_FREQ_HZ    (MM_DFLL_ONLY_CLK_FREQ_HZ >> \
                              MM_DFLL_ONLY_PBA_PRESCALER)

/* *** DFLL w/ 32kHz external oscillator reference mode *** */
/* 32kHz external oscillator frequency */
#define MM_DFLL_32KHZ_OSC32_FREQ_HZ       (32768u)
/* F_CPU = (DFLL base) / 2^0 = 50MHz */
#define MM_DFLL_32KHZ_FCPU_PRESCALER      (2u)
/* PBA = (DFLL base) / 2^0 = 50MHz */
#define MM_DFLL_32KHZ_PBA_PRESCALER       (0u)
/* PBB = (DFLL base) / 2^0 = 50MHz */
#define MM_DFLL_32KHZ_PBB_PRESCALER       (0u)

/*----------------------------------------------------------------------------*/
/*                              Global Variables                              */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                       Public (Exportable) Functions                        */
/*----------------------------------------------------------------------------*/
clock_status_t at32uc3l0256_InitDfll(void);
clock_status_t at32uc3l0256_InitDfllExternalOscillator(void);
clock_status_t at32uc3l0256_DelayMs(const uint32_t delayTime);
clock_status_t at32uc3l0256_DelayUs(const uint32_t delayTime);

#endif /* CLOCK_AT32UC3L0256_H_ */