/*-------------------------------- FILE INFO -----------------------------------
* Filename        : tc_config.c
* Author          : Team Kirbo
* Revision        : 1.0
* Updated         : 2024-03-31
* Purpose         : HAL config layer
*
* This file is the C source file for the TC config file.
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
#include "HAL/HAL_contracts/tc_contract.h"
#include "HAL/HAL_configs/tc_config.h"
#include "HAL/at32uc3l0256/tc_at32uc3l0256.h"

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
/* static instance of ADC interface */
static tc_handler_t tcInterfaceHandler =
{
    .tc_Init = at32uc3l0256_InitTc,
    .tc_GetTimerCounterCount = at32uc3l0256_GetTimerCounterCount,
};

/*----------------------------------------------------------------------------*/
/*                    Private (Static) Function Prototype                     */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                       Public (Exportable) Functions                        */
/*----------------------------------------------------------------------------*/
/**
* Returns static instance of TC interface handler defined in this file.
*
* \param[out] p_adcHandler Handler to link to this file's handler instance.
* \retval None
*/
void config_GetTcHandler(tc_handler_t** p_tcHandler)
{
    *p_tcHandler = &tcInterfaceHandler;
}

/*----------------------------------------------------------------------------*/
/*                           Local Shared Functions                           */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                         Private (Static) Functions                         */
/*----------------------------------------------------------------------------*/
/* None */