/*-------------------------------- FILE INFO -----------------------------------
* Filename        : io_config.c
* Author          : Ryuichi Lin
* Revision        : 1.0
* Updated         : 2024-03-31
* Purpose         : HAL config layer
*
* This file is the C source file for the IO config file.
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
#include "HAL/HAL_contracts/io_contract.h"
#include "HAL/HAL_configs/io_config.h"
#include "HAL/at32uc3l0256/io_at32uc3l0256.h"

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
/* static instance of IO interface */
static io_handler_t ioInterfaceHandler = 
{
    .io_Init = at32uc3l0256_InitIO,
    .io_ReadInput = at32uc3l0256_ReadPin,
    .io_WriteOutput = at32uc3l0256_WritePin, 
    .io_ToggleOutput = at32uc3l0256_TogglePin,
    .io_WritePullup = at32uc3l0256_WritePullUp,
};

/*----------------------------------------------------------------------------*/
/*                    Private (Static) Function Prototype                     */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                       Public (Exportable) Functions                        */
/*----------------------------------------------------------------------------*/
/**
* Returns static instance of IO interface handler defined in this file.
*
* \param[out] p_ioHandler Handler to link to this file's handler instance.
*/
void config_GetIOHandler(io_handler_t** p_ioHandler)
{
    *p_ioHandler = &ioInterfaceHandler;
}

/*----------------------------------------------------------------------------*/
/*                           Local Shared Functions                           */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                         Private (Static) Functions                         */
/*----------------------------------------------------------------------------*/
/* None */
