/*-------------------------------- FILE INFO -----------------------------------
* Filename        : adc_config.c
* Author          : Team Kirbo
* Revision        : 1.0
* Updated         : 2024-03-31
* Purpose         : HAL config layer
*
* This file is the C source file for the ADC config file.
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
#include "HAL/HAL_contracts/adc_contract.h"
#include "HAL/HAL_configs/adc_config.h"
#include "HAL/at32uc3l0256/adc_at32uc3l0256.h"

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
static adc_handler_t adcInterfaceHandler =
{
    .adc_Init = at32uc3l0256_InitAdc,
    .adc_EnableChannel = at32uc3l0256_EnableAdcChannel,
    .adc_DisableChannel = at32uc3l0256_DisableAdcChannel,
    .adc_ReadValue = at32uc3l0256_ReadValue,
};

/*----------------------------------------------------------------------------*/
/*                    Private (Static) Function Prototype                     */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                       Public (Exportable) Functions                        */
/*----------------------------------------------------------------------------*/
/**
* Returns static instance of ADC interface handler defined in this file.
*
* \param[out] p_adcHandler Handler to link to this file's handler instance.
* \retval None
*/
void config_GetAdcHandler(adc_handler_t** p_adcHandler)
{
    *p_adcHandler = &adcInterfaceHandler;
}

/*----------------------------------------------------------------------------*/
/*                           Local Shared Functions                           */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                         Private (Static) Functions                         */
/*----------------------------------------------------------------------------*/
/* None */
