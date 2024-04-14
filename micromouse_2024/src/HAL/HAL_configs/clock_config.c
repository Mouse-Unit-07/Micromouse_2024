/*-------------------------------- FILE INFO -----------------------------------
* Filename        : clock_config.c
* Author          : Ryuichi Lin
* Revision        : 1.0
* Updated         : 2024-03-31
* Purpose         : HAL config layer
*
* This file is the C source file for the clock config file.
*
* HAL config files define handlers for high level code to access hardware
* specific code abstracted out w/ HAL contracts. Bridges contracts and
* hardware specific code by creating an instance of a contract w/ members
* filled w/ hardware specific code that adheres to the contract.
*
* Step 2 for hardware abstraction.
*-----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*                               Include Files                                */
/*----------------------------------------------------------------------------*/
#include <stdint.h>
#include "HAL/HAL_contracts/clock_contract.h"
#include "HAL/HAL_configs/clock_config.h"
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
/* static instance of clock interface */
static clock_handler_t clockInterfaceHandler =
{
    .clock_InitDfll = at32uc3l0256_InitDfll,
    .clock_InitDfllExternalOscillator = 
        at32uc3l0256_InitDfllExternalOscillator,
    .clock_DelayMs = at32uc3l0256_DelayMs,
    .clock_DelayUs = at32uc3l0256_DelayUs,
};

/*----------------------------------------------------------------------------*/
/*                    Private (Static) Function Prototype                     */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                       Public (Exportable) Functions                        */
/*----------------------------------------------------------------------------*/
/**
* Returns static instance of clock interface handler defined in this file.
*
* \param[out] p_clockHandler Handler to link to this file's handler instance.
* \retval None
*/
void config_GetClockHandler(clock_handler_t** p_clockHandler)
{
    *p_clockHandler = &clockInterfaceHandler;
}

/*----------------------------------------------------------------------------*/
/*                           Local Shared Functions                           */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                         Private (Static) Functions                         */
/*----------------------------------------------------------------------------*/
/* None */
