/*-------------------------------- FILE INFO -----------------------------------
* Filename        : clock_at32uc3l0256.c
* Author          : Ryuichi Lin
* Revision        : 1.0
* Updated         : 2024-03-31
* Purpose         : hardware specific layer
*
* This file is the header file for hardware specific clock code.
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
#include "HAL/HAL_contracts/clock_contract.h"
#include "HAL/at32uc3l0256/clock_at32uc3l0256.h"

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
/*                       Public (Exportable) Functions                        */
/*----------------------------------------------------------------------------*/
/**
* Enable DFLL as main clock source for AT32UC3L0256 MCU.
*
* \retval CLOCK_SUCCESS Success
* \retval CLOCK_ERROR Failure: Failed to initialize clock source
*/
clock_status_t at32uc3l0256_InitDfll(void)
{
    clock_status_t clockStatus = CLOCK_ERROR;
    struct dfll_config dcfg; /* DFLL configuration struct */
    
    dfll_config_init_open_loop_mode(&dcfg);
    dfll_config_tune_for_target_hz(&dcfg, MM_DFLL_ONLY_CLK_FREQ_HZ);
    dfll_enable_open_loop(&dcfg, 0);
    
    /* change CPU and peripheral clock frequencies */
    sysclk_set_prescalers(
        MM_DFLL_ONLY_FCPU_PRESCALER,
        MM_DFLL_ONLY_PBA_PRESCALER,
        MM_DFLL_ONLY_PBB_PRESCALER
    );
    
    sysclk_set_source(SYSCLK_SRC_DFLL);
    osc_disable(OSC_ID_RC120M);
    
    clockStatus = CLOCK_SUCCESS;
    
    /* return status */
    return clockStatus;
}

/**
* Enable DFLL w/ external oscillator as main clock source for AT32UC3L0256 MCU.
*
* \retval CLOCK_SUCCESS Success
* \retval CLOCK_ERROR Failure: Failed to initialize clock source
*/
clock_status_t at32uc3l0256_InitDfllExternalOscillator(void)
{
    clock_status_t clockStatus = CLOCK_ERROR;
    struct dfll_config dcfg; /* DFLL configuration struct */
    
    osc_enable(OSC_ID_OSC32);
    dfll_config_init_closed_loop_mode(
        &dcfg,
        GENCLK_SRC_OSC32K,
        1,
        50000000 / MM_DFLL_32KHZ_OSC32_FREQ_HZ
    );
    osc_wait_ready(OSC_ID_OSC32);
    dfll_enable_closed_loop(&dcfg, 0);
    
    /* change CPU and peripheral clock frequencies */
    sysclk_set_prescalers(
        MM_DFLL_32KHZ_FCPU_PRESCALER,
        MM_DFLL_32KHZ_PBA_PRESCALER,
        MM_DFLL_32KHZ_PBB_PRESCALER
    );
    
    dfll_wait_for_fine_lock(0);
    sysclk_set_source(SYSCLK_SRC_DFLL);
    //osc_disable(OSC_ID_RC120M); //might not be needed?
    
    clockStatus = CLOCK_SUCCESS;
        
    /* return status */
    return clockStatus;
}

/**
* Delay in milliseconds for AT32UC3L0256 MCU.
*
* \param[in] delayTime Time to delay in milliseconds
* \retval CLOCK_SUCCESS Success
* \retval CLOCK_ERROR Failure: Failed to initialize clock source
*/
clock_status_t at32uc3l0256_DelayMs(const uint32_t delayTime)
{
    clock_status_t clockStatus = CLOCK_ERROR;
    
    delay_ms(delayTime);
    
    clockStatus = CLOCK_SUCCESS;
    
    /* return status */
    return clockStatus;
}

/**
* Delay in microseconds for AT32UC3L0256 MCU.
*
* \param[in] delayTime Time to delay in microseconds
* \retval CLOCK_SUCCESS Success
* \retval CLOCK_ERROR Failure: Failed to initialize clock source
*/
clock_status_t at32uc3l0256_DelayUs(const uint32_t delayTime)
{
    clock_status_t clockStatus = CLOCK_ERROR;
    
    delay_us(delayTime);
    
    clockStatus = CLOCK_SUCCESS;
    
    /* return status */
    return clockStatus;
}

/*----------------------------------------------------------------------------*/
/*                           Local Shared Functions                           */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                         Private (Static) Functions                         */
/*----------------------------------------------------------------------------*/
/* None */