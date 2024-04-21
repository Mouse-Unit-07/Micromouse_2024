/*-------------------------------- FILE INFO -----------------------------------
* Filename        : pwm_contract.h
* Author          : Team Kirbo
* Revision        : 1.0
* Updated         : 2024-03-31
* Purpose         : HAL contract layer
*
* This file contains the contract to abstract a PWM interface.
*
* Contracts define features of interfaces so high level code can use abstract
* handlers instead of hardware specific code. HAL config files use these
* contracts to link to hardware specific code.
*
* Step 1 for hardware abstraction.
*-----------------------------------------------------------------------------*/

#ifndef PWM_CONTRACT_H_
#define PWM_CONTRACT_H_

/*----------------------------------------------------------------------------*/
/*                                Definitions                                 */
/*----------------------------------------------------------------------------*/
/* PWM status enumeration */
typedef enum
{
    PWM_SUCCESS = 0u,
    PWM_ERROR,
    PWM_WRITE_ERROR,
} pwm_status_t;

/* PWM interface contract- used to create handlers */
typedef struct
{
    pwm_status_t (*pwm_Init)(void);
    pwm_status_t (*pwm_SetWheelMotor1DutyCycle)(uint16_t dutyCycle);
    pwm_status_t (*pwm_SetWheelMotor2DutyCycle)(uint16_t dutyCycle);
    pwm_status_t (*pwm_SetVacuumMotorDutyCycle)(uint16_t dutyCycle);
} pwm_handler_t;

/*----------------------------------------------------------------------------*/
/*                              Global Variables                              */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                       Public (Exportable) Functions                        */
/*----------------------------------------------------------------------------*/
/* None */

#endif /* PWM_CONTRACT_H_ */