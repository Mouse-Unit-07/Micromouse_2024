/*-------------------------------- FILE INFO -----------------------------------
* Filename        : eic_config.c
* Author          : Team Kirbo
* Revision        : 1.0
* Updated         : 2024-03-31
* Purpose         : HAL config layer
*
* This file is the C source file for the external interrupt controller
* (eic) config file.
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
#include "HAL/HAL_contracts/eic_contract.h"
#include "HAL/HAL_configs/eic_config.h"
#include "HAL/at32uc3l0256/eic_at32uc3l0256.h"

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
/* static instance of EIC interface */
static eic_handler_t eicInterfaceHandler =
{
    .eic_InitEncoderInterrupts = at32uc3l0256_InitEncoderInterrupts,
    .eic_InitConfigPinInterrupt = at32uc3l0256_InitConfigPinInterrupt,
    .eic_GetEncoder1EdgeCount = at32uc3l0256_GetEncoder1EdgeCount,
    .eic_GetEncoder2EdgeCount = at32uc3l0256_GetEncoder2EdgeCount,
    .eic_GetConfigPinEdgeCount = at32uc3l0256_GetConfigPinEdgeCount,
    .eic_ClearEncoder1EdgeCount = at32uc3l0256_ClearEncoder1EdgeCount,
    .eic_ClearEncoder2EdgeCount = at32uc3l0256_ClearEncoder2EdgeCount, 
    .eic_ClearConfigPinEdgeCount = at32uc3l0256_ClearConfigPinEdgeCount,
};

/*----------------------------------------------------------------------------*/
/*                    Private (Static) Function Prototype                     */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                       Public (Exportable) Functions                        */
/*----------------------------------------------------------------------------*/
/**
* Returns static instance of EIC interface handler defined in this file.
*
* \param[out] p_eicHandler Handler to link to this file's handler instance.
* \retval None
*/
void config_GetEicHandler(eic_handler_t** p_eicHandler)
{
    *p_eicHandler = &eicInterfaceHandler;
}

/*----------------------------------------------------------------------------*/
/*                           Local Shared Functions                           */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                         Private (Static) Functions                         */
/*----------------------------------------------------------------------------*/
/* None */