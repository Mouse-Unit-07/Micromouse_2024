/*-------------------------------- FILE INFO -----------------------------------
* Filename        : power_mhi.c
* Author          : Team Kirbo
* Revision        : 1.0
* Updated         : 2024-04-07
* Purpose         : mouse hardware interface layer
*
* This is the source file for the mouse power interface.
*
* The mouse hardware interface uses the HAL to define functions needed to
* interface w/ all mouse hardware.
*
* TODO:
*     - Add contracts and handlers for this MHI layer as well
*
* (if applicable)
* Target Hardware    : AT32UC3L0256
* IDE                : Atmel Studio 7.4.2542
* SDK                : ASF 3.52.0
*-----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*                               Include Files                                */
/*----------------------------------------------------------------------------*/
#include <asf.h> //embodiment of pain- defeats abstraction effort
#include <stdint.h>
#include <stdio.h>
#include "HAL/HAL_contracts/io_contract.h"
#include "HAL/HAL_configs/io_config.h"
#include "leds_mhi.h"
#include "power_mhi.h"

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
static mhi_power_state_t regulatorState = MHI_REGULATORS_OFF;

/*----------------------------------------------------------------------------*/
/*                    Private (Static) Function Prototype                     */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                       Public (Exportable) Functions                        */
/*----------------------------------------------------------------------------*/
/**
* Initializes power pins- enable pin deasserted at start.
*
* \param  None
* \retval None
*/
void mhi_InitPower(void)
{
    io_handler_t *ioInterface = NULL;
    config_GetIOHandler(&ioInterface);
    
    regulatorState = MHI_REGULATORS_OFF;
    
    ioInterface->io_Init(0u, MHI_REGULATOR_ENABLE_PIN, IO_PIN_LOW,
        IO_OUTPUT, IO_PULLUP_DISABLE);
    ioInterface->io_Init(0u, MHI_LOW_BATTERY_PIN, IO_PIN_LOW,
        IO_INPUT, IO_PULLUP_DISABLE);
}

/**
* Check low battery pin and assert error if low battery
*
* \param  None
* \retval None
*/
void mhi_CheckLowBattery(void)
{
    io_pin_state_t pinState = IO_PIN_LOW;
    io_handler_t *ioInterface = NULL;
    config_GetIOHandler(&ioInterface);
    
    if (regulatorState == MHI_REGULATORS_ON) 
    {
        ioInterface->io_ReadInput(0u, MHI_LOW_BATTERY_PIN, &pinState);
        if (pinState == IO_PIN_LOW)
        {
            mhi_IndicateError(MHI_LEDS_LOW_BATTERY_ERROR);
        }
    }
}

/**
* Assert enable pin for regulators
*
* \param  None
* \retval None
*/
void mhi_EnableRegulators(void)
{
    io_handler_t *ioInterface = NULL;
    config_GetIOHandler(&ioInterface);
    
    regulatorState = MHI_REGULATORS_ON;
    
    ioInterface->io_WriteOutput(0u, MHI_REGULATOR_ENABLE_PIN, IO_PIN_HIGH);
}

/**
* De-assert enable pin for regulators
*
* \param  None
* \retval None
*/
void mhi_DisableRegulators(void)
{
    io_handler_t *ioInterface = NULL;
    config_GetIOHandler(&ioInterface);
    
    regulatorState = MHI_REGULATORS_OFF;
    
    ioInterface->io_WriteOutput(0u, MHI_REGULATOR_ENABLE_PIN, IO_PIN_LOW);
}

/*----------------------------------------------------------------------------*/
/*                           Local Shared Functions                           */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                         Private (Static) Functions                         */
/*----------------------------------------------------------------------------*/
/* None */
