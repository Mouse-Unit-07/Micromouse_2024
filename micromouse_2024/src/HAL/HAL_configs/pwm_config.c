/*-------------------------------- FILE INFO -----------------------------------
* Filename        : pwm_config.c
* Author          : Ryuichi Lin
* Revision        : 1.0
* Updated         : 2024-03-31
* Purpose         : HAL config layer
*
* This file is the C source file for the PWM config file.
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
#include "HAL/HAL_contracts/pwm_contract.h"
#include "HAL/HAL_configs/pwm_config.h"
#include "HAL/at32uc3l0256/pwm_at32uc3l0256.h"

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
/* static instance of PWM interface */
static pwm_handler_t pwmInterfaceHandler =
{
    .pwm_Init = at32uc3l0256_InitPwm,
    .pwm_SetWheelMotor1DutyCycle = at32uc3l0256_SetWheelMotor1DutyCycle,
    .pwm_SetWheelMotor2DutyCycle = at32uc3l0256_SetWheelMotor2DutyCycle,
    .pwm_SetVacuumMotorDutyCycle = at32uc3l0256_SetVacuumMotorDutyCycle,
};

/*----------------------------------------------------------------------------*/
/*                    Private (Static) Function Prototype                     */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                       Public (Exportable) Functions                        */
/*----------------------------------------------------------------------------*/
/**
* Returns static instance of PWM interface handler defined in this file.
*
* \param[out] p_pwmHandler Handler to link to this file's handler instance.
* \retval None
*/
void config_GetPwmHandler(pwm_handler_t** p_pwmHandler)
{
    *p_pwmHandler = &pwmInterfaceHandler;
}

/*----------------------------------------------------------------------------*/
/*                           Local Shared Functions                           */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                         Private (Static) Functions                         */
/*----------------------------------------------------------------------------*/
/* None */
