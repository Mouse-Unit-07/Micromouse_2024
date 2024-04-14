/*-------------------------------- FILE INFO -----------------------------------
* Filename        : iic_config.c
* Author          : Ryuichi Lin
* Revision        : 1.0
* Updated         : 2024-03-31
* Purpose         : HAL config layer
*
* This file is the C source file for the "internal interrupt controller" (IIC)
* config file.
*
* IIC refers to the software interrupt controller as opposed to EIC.
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
#include "HAL/HAL_contracts/iic_contract.h"
#include "HAL/HAL_configs/iic_config.h"
#include "HAL/at32uc3l0256/iic_at32uc3l0256.h"

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
/* static instance of IIC interface */
static iic_handler_t iicInterfaceHandler =
{
    .iic_Init = at32uc3l0256_InitIic,
    .iic_EnableGlobalInterrupts = at32uc3l0256_EnableGlobalInterrupts,
    .iic_DisableGlobalInterrupts = at32uc3l0256_DisableGlobalInterrupts,
};

/*----------------------------------------------------------------------------*/
/*                    Private (Static) Function Prototype                     */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                       Public (Exportable) Functions                        */
/*----------------------------------------------------------------------------*/
/**
* Returns static instance of IIC interface handler defined in this file.
*
* \param[out] p_adcHandler Handler to link to this file's handler instance.
* \retval None
*/
void config_GetIicHandler(iic_handler_t** p_iicHandler)
{
    *p_iicHandler = &iicInterfaceHandler;
}

/*----------------------------------------------------------------------------*/
/*                           Local Shared Functions                           */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                         Private (Static) Functions                         */
/*----------------------------------------------------------------------------*/
/* None */
