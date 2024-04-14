/*-------------------------------- FILE INFO -----------------------------------
* Filename        : timer_mhi.c
* Author          : Ryuichi Lin
* Revision        : 1.0
* Updated         : 2024-04-07
* Purpose         : mouse hardware interface layer
*
* This is the source file for the mouse timer interface.
*
* The mouse hardware interface uses the HAL to define functions needed to
* interface w/ all mouse hardware.
*-----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*                               Include Files                                */
/*----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include "leds_mhi.h"
#include "HAL/HAL_contracts/tc_contract.h"
#include "HAL/HAL_configs/tc_config.h"
#include "timer_mhi.h"

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
* Initialize timer counter for micromouse.
*
* \param  None
* \retval None
*/
void mhi_InitTimerCounter(void)
{
    tc_handler_t *tcInterface = NULL;
    config_GetTcHandler(&tcInterface);
    
    if (tcInterface->tc_Init() == TC_ERROR)
    {
        mhi_IndicateError(MHI_LEDS_TC_ERROR);
    }
}

/**
* Get timer counter count for micromouse.
*
* \param  None
* \retval None
*/
uint32_t mhi_GetTimerCount(void)
{
    uint32_t timerCount = 0u;
    tc_handler_t *tcInterface = NULL;
    config_GetTcHandler(&tcInterface);
    
    if (tcInterface->tc_GetTimerCounterCount(&timerCount) == TC_ERROR)
    {
        mhi_IndicateError(MHI_LEDS_TC_ERROR);
    }
    
    return timerCount;
}

/**
* Timer count to milliseconds for micromouse.
*
* \param  None
* \retval None
*/
uint32_t mhi_TimerCountToMs(uint32_t count)
{
    /* varies w/ timer counter configuration */
    return count*17;
}

/*----------------------------------------------------------------------------*/
/*                           Local Shared Functions                           */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                         Private (Static) Functions                         */
/*----------------------------------------------------------------------------*/
/* None */
