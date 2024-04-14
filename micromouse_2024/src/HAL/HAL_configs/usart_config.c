/*-------------------------------- FILE INFO -----------------------------------
* Filename        : usart_config.c
* Author          : Ryuichi Lin
* Revision        : 1.0
* Updated         : 2024-03-31
* Purpose         : HAL config layer
*
* This file is the C source file for the USART config file.
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
#include "HAL/HAL_contracts/usart_contract.h"
#include "HAL/HAL_configs/usart_config.h"
#include "HAL/at32uc3l0256/usart_at32uc3l0256.h"

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
/* static instance of USART interface */
static usart_handler_t usartInterfaceHandler =
{
    .usart_Init = at32uc3l0256_InitUsart,
    .usart_PrintString = at32uc3l0256_PrintString,
    .usart_PrintInt = at32uc3l0256_PrintInt,
};    

/*----------------------------------------------------------------------------*/
/*                    Private (Static) Function Prototype                     */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                       Public (Exportable) Functions                        */
/*----------------------------------------------------------------------------*/
/**
* Returns static instance of USART interface handler defined in this file.
*
* \param[out] p_usartHandler Handler to link to this file's handler instance.
* \retval None
*/
void config_GetUsartHandler(usart_handler_t** p_usartHandler)
{
    *p_usartHandler = &usartInterfaceHandler;
}

/*----------------------------------------------------------------------------*/
/*                           Local Shared Functions                           */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                         Private (Static) Functions                         */
/*----------------------------------------------------------------------------*/
/* None */
