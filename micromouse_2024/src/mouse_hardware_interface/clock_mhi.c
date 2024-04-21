/*-------------------------------- FILE INFO -----------------------------------
* Filename        : clock_mhi.c
* Author          : Team Kirbo
* Revision        : 1.0
* Updated         : 2024-04-07
* Purpose         : mouse hardware interface layer
*
* This is the source file for the mouse clock interface.
*
* The mouse hardware interface uses the HAL to define functions needed to
* interface w/ all mouse hardware.
*
* TODO:
*     - Check and handle status return values of clock HAL functions
*-----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*                               Include Files                                */
/*----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include "HAL/HAL_contracts/clock_contract.h"
#include "HAL/HAL_configs/clock_config.h"
#include "clock_mhi.h"

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
* Initialize clock sources for micromouse.
*
* \param  None
* \retval None
*/
void mhi_InitClock(void)
{
    clock_handler_t *clockInterface = NULL;
    config_GetClockHandler(&clockInterface);
    
    //clockInterface->clock_InitDfllExternalOscillator();
    clockInterface->clock_InitDfll();
}

/**
* Delay in milliseconds for AT32UC3L0256 MCU.
*
* \param[in] delayTime Time to delay in microseconds
* \retval None
*/
void mhi_DelayMs(const uint32_t delayTime)
{
    clock_handler_t *clockInterface = NULL;
    config_GetClockHandler(&clockInterface);
    
    clockInterface->clock_DelayMs(delayTime);
}

/**
* Delay in microseconds for AT32UC3L0256 MCU.
*
* \param[in] delayTime Time to delay in microseconds
* \retval None
*/
void mhi_DelayUs(const uint32_t delayTime)
{
    clock_handler_t *clockInterface = NULL;
    config_GetClockHandler(&clockInterface);
    
    clockInterface->clock_DelayUs(delayTime);
}

/*----------------------------------------------------------------------------*/
/*                           Local Shared Functions                           */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                         Private (Static) Functions                         */
/*----------------------------------------------------------------------------*/
/* None */
