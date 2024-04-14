/*-------------------------------- FILE INFO -----------------------------------
* Filename        : motors_mhi.c
* Author          : Ryuichi Lin
* Revision        : 1.0
* Updated         : 2024-04-07
* Purpose         : mouse hardware interface layer
*
* This is the source file for the mouse motors interface.
* This includes brushed DC vacuum and wheel motor control interfaces.
*
* The mouse hardware interface uses the HAL to define functions needed to
* interface w/ all mouse hardware.
*
* TODO:
*     - Add contracts and handlers for this MHI layer as well
*     - Check and handle status return values of IO HAL functions
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
#include "HAL/HAL_contracts/pwm_contract.h"
#include "HAL/HAL_configs/pwm_config.h"
#include "motors_mhi.h"

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
static mhi_wheel_motor_direction_t wheelMotor1Direction 
    = MHI_WHEEL_MOTOR_DIRECTION_STOP;

static mhi_wheel_motor_direction_t wheelMotor2Direction
    = MHI_WHEEL_MOTOR_DIRECTION_STOP;

static mhi_vacuum_motor_state_t vacuumMotorState
    = MHI_VACUUM_MOTOR_STATE_STOP;
/*----------------------------------------------------------------------------*/
/*                    Private (Static) Function Prototype                     */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                       Public (Exportable) Functions                        */
/*----------------------------------------------------------------------------*/
/**
* Initialize PWM interface for micromouse.
*
* \param  None
* \retval None
*/
void mhi_InitPwm(void)
{
    pwm_handler_t *pwmInterface = NULL;
    config_GetPwmHandler(&pwmInterface);
    
    if (pwmInterface->pwm_Init() != PWM_SUCCESS)
        mhi_IndicateError(MHI_LEDS_PWM_ERROR);
}

/**
* Initialize wheel motor control pins for micromouse.
*
* \param  None
* \retval None
*/
void mhi_InitWheelMotors(void)
{
    io_handler_t *ioInterface = NULL;
    config_GetIOHandler(&ioInterface);
    
    ioInterface->io_Init(0u, MHI_WHEEL_MOTOR_STANDBY_PIN, 
        IO_PIN_LOW, IO_OUTPUT, IO_PULLUP_DISABLE);
        
    ioInterface->io_Init(0u, MHI_WHEEL_MOTOR_CLD_PIN,
        IO_PIN_LOW, IO_INPUT, IO_PULLUP_DISABLE);
    
    ioInterface->io_Init(0u, MHI_WHEEL_MOTOR_1_IN1_PIN,
        IO_PIN_LOW, IO_OUTPUT, IO_PULLUP_DISABLE);
    
    ioInterface->io_Init(0u, MHI_WHEEL_MOTOR_1_IN2_PIN,
        IO_PIN_LOW, IO_OUTPUT, IO_PULLUP_DISABLE);
    
    ioInterface->io_Init(0u, MHI_WHEEL_MOTOR_2_IN1_PIN,
        IO_PIN_LOW, IO_OUTPUT, IO_PULLUP_DISABLE);
    
    ioInterface->io_Init(0u, MHI_WHEEL_MOTOR_2_IN2_PIN,
        IO_PIN_LOW, IO_OUTPUT, IO_PULLUP_DISABLE);
}

/**
* Initialize vacuum motor control pins for micromouse.
*
* \param  None
* \retval None
*/
void mhi_InitVacuumMotor(void)
{
    io_handler_t *ioInterface = NULL;
    config_GetIOHandler(&ioInterface);
    
    ioInterface->io_Init(0u, MHI_VACUUM_MOTOR_IN1_PIN,
        IO_PIN_LOW, IO_OUTPUT, IO_PULLUP_DISABLE);
}

/**
* Start micromouse wheel motor 1 in forward direction.
*
* \param  None
* \retval None
*/
void mhi_StartWheelMotor1Forward(void)
{
    io_handler_t *ioInterface = NULL;
    config_GetIOHandler(&ioInterface);
    
    wheelMotor1Direction = MHI_WHEEL_MOTOR_DIRECTION_FORWARD;
    ioInterface->io_WriteOutput(0u, MHI_WHEEL_MOTOR_STANDBY_PIN, IO_PIN_HIGH);
    ioInterface->io_WriteOutput(0u, MHI_WHEEL_MOTOR_1_IN1_PIN, IO_PIN_HIGH);
    ioInterface->io_WriteOutput(0u, MHI_WHEEL_MOTOR_1_IN2_PIN, IO_PIN_LOW);
}

/**
* Start micromouse wheel motor 1 in backward direction.
*
* \param  None
* \retval None
*/
void mhi_StartWheelMotor1Backward(void)
{
    io_handler_t *ioInterface = NULL;
    config_GetIOHandler(&ioInterface);
    
    wheelMotor1Direction = MHI_WHEEL_MOTOR_DIRECTION_BACKWARD;
    ioInterface->io_WriteOutput(0u, MHI_WHEEL_MOTOR_STANDBY_PIN, IO_PIN_HIGH);
    ioInterface->io_WriteOutput(0u, MHI_WHEEL_MOTOR_1_IN1_PIN, IO_PIN_LOW);
    ioInterface->io_WriteOutput(0u, MHI_WHEEL_MOTOR_1_IN2_PIN, IO_PIN_HIGH);
}

/**
* Set micromouse wheel motor 1 duty cycle.
*
* \param[in] speed Duty cycle to set wheel motor 1 to (0~255)
* \retval None
*/
void mhi_SetWheelMotor1Speed(uint16_t speed)
{
    pwm_handler_t *pwmInterface = NULL;
    config_GetPwmHandler(&pwmInterface);
    
    if (pwmInterface->pwm_SetWheelMotor1DutyCycle(speed) != PWM_SUCCESS)
        mhi_IndicateError(MHI_LEDS_PWM_ERROR);
}

/**
* Stop micromouse wheel motor 1.
*
* \param  None
* \retval None
*/
void mhi_StopWheelMotor1(void)
{
    io_handler_t *ioInterface = NULL;
    config_GetIOHandler(&ioInterface);
    
    mhi_SetWheelMotor1Speed(0u);
    wheelMotor1Direction = MHI_WHEEL_MOTOR_DIRECTION_STOP;
    ioInterface->io_WriteOutput(0u, MHI_WHEEL_MOTOR_STANDBY_PIN, IO_PIN_HIGH);
    ioInterface->io_WriteOutput(0u, MHI_WHEEL_MOTOR_1_IN1_PIN, IO_PIN_LOW);
    ioInterface->io_WriteOutput(0u, MHI_WHEEL_MOTOR_1_IN2_PIN, IO_PIN_LOW);
}

/**
* Access function for wheel motor 1 direction
*
* \param  None
* \retval wheelMotor1Direction Direction variable for wheel motor 1
*/
mhi_wheel_motor_direction_t mhi_GetWheelMotor1Direction(void)
{
    return wheelMotor1Direction;
}

/**
* Start micromouse wheel motor 2 in forward direction.
*
* \param  None
* \retval None
*/
void mhi_StartWheelMotor2Forward(void)
{
    io_handler_t *ioInterface = NULL;
    config_GetIOHandler(&ioInterface);
    
    wheelMotor2Direction = MHI_WHEEL_MOTOR_DIRECTION_FORWARD;
    ioInterface->io_WriteOutput(0u, MHI_WHEEL_MOTOR_STANDBY_PIN, IO_PIN_HIGH);
    ioInterface->io_WriteOutput(0u, MHI_WHEEL_MOTOR_2_IN1_PIN, IO_PIN_LOW);
    ioInterface->io_WriteOutput(0u, MHI_WHEEL_MOTOR_2_IN2_PIN, IO_PIN_HIGH);
}

/**
* Start micromouse wheel motor 2 in backward direction.
*
* \param  None
* \retval None
*/
void mhi_StartWheelMotor2Backward(void)
{
    io_handler_t *ioInterface = NULL;
    config_GetIOHandler(&ioInterface);
    
    wheelMotor2Direction = MHI_WHEEL_MOTOR_DIRECTION_BACKWARD;
    ioInterface->io_WriteOutput(0u, MHI_WHEEL_MOTOR_STANDBY_PIN, IO_PIN_HIGH);
    ioInterface->io_WriteOutput(0u, MHI_WHEEL_MOTOR_2_IN1_PIN, IO_PIN_HIGH);
    ioInterface->io_WriteOutput(0u, MHI_WHEEL_MOTOR_2_IN2_PIN, IO_PIN_LOW);
}

/**
* Set micromouse wheel motor 2 duty cycle.
*
* \param[in] speed Duty cycle to set vacuum motor to (0~255)
* \retval None
*/
void mhi_SetWheelMotor2Speed(uint16_t speed)
{
    pwm_handler_t *pwmInterface = NULL;
    config_GetPwmHandler(&pwmInterface);
    
    if (pwmInterface->pwm_SetWheelMotor2DutyCycle(speed) != PWM_SUCCESS)
        mhi_IndicateError(MHI_LEDS_PWM_ERROR);
}

/**
* Stop micromouse wheel motor 2.
*
* \param  None
* \retval None
*/
void mhi_StopWheelMotor2(void)
{
    io_handler_t *ioInterface = NULL;
    config_GetIOHandler(&ioInterface);
    
    mhi_SetWheelMotor2Speed(0u);
    wheelMotor2Direction = MHI_WHEEL_MOTOR_DIRECTION_STOP;
    ioInterface->io_WriteOutput(0u, MHI_WHEEL_MOTOR_STANDBY_PIN, IO_PIN_HIGH);
    ioInterface->io_WriteOutput(0u, MHI_WHEEL_MOTOR_2_IN1_PIN, IO_PIN_LOW);
    ioInterface->io_WriteOutput(0u, MHI_WHEEL_MOTOR_2_IN2_PIN, IO_PIN_LOW);
}

/**
* Access function for wheel motor 2 direction
*
* \param  None
* \retval wheelMotor1Direction Direction variable for wheel motor 2
*/
mhi_wheel_motor_direction_t mhi_GetWheelMotor2Direction(void)
{
    return wheelMotor2Direction;
}

/**
* Start micromouse vacuum motor w/ given duty cycle.
*
* \param[in] speed Duty cycle to set wheel motor 1 to (0~255)
* \retval None
*/
void mhi_StartVacuumMotor(uint16_t speed)
{
    io_handler_t *ioInterface = NULL;
    pwm_handler_t *pwmInterface = NULL;
    config_GetIOHandler(&ioInterface);
    config_GetPwmHandler(&pwmInterface);
    
    vacuumMotorState = MHI_VACUUM_MOTOR_STATE_MOVING;
    ioInterface->io_WriteOutput(0u, MHI_VACUUM_MOTOR_IN1_PIN, IO_PIN_HIGH);
    
    if (pwmInterface->pwm_SetVacuumMotorDutyCycle(speed) != PWM_SUCCESS)
        mhi_IndicateError(MHI_LEDS_PWM_ERROR);
}

/**
* Stop micromouse vacuum motor.
*
* \param  None
* \retval None
*/
void mhi_StopVacuumMotor(void)
{
    io_handler_t *ioInterface = NULL;
    pwm_handler_t *pwmInterface = NULL;
    config_GetIOHandler(&ioInterface);
    config_GetPwmHandler(&pwmInterface);
    
    vacuumMotorState = MHI_VACUUM_MOTOR_STATE_STOP;
    ioInterface->io_WriteOutput(0u, MHI_VACUUM_MOTOR_IN1_PIN, IO_PIN_LOW);
    
    if (pwmInterface->pwm_SetVacuumMotorDutyCycle(0u) != PWM_SUCCESS)
        mhi_IndicateError(MHI_LEDS_PWM_ERROR);
}

/**
* Access function for vacuum motor state
*
* \param  None
* \retval vacuumMotorState State variable for vacuum motor
*/
mhi_vacuum_motor_state_t mhi_GetVacuumMotorState(void)
{
    return vacuumMotorState;
}

/*----------------------------------------------------------------------------*/
/*                           Local Shared Functions                           */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                         Private (Static) Functions                         */
/*----------------------------------------------------------------------------*/
/* None */