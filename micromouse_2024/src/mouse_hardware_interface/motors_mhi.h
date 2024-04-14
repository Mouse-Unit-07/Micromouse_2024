/*-------------------------------- FILE INFO -----------------------------------
* Filename        : motors_mhi.h
* Author          : Ryuichi Lin
* Revision        : 1.0
* Updated         : 2024-04-07
* Purpose         : mouse hardware interface layer
*
* This is the header file for the mouse motors interface.
* This includes brushed DC vacuum and wheel motor control interfaces.
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

#ifndef MOTORS_MHI_H_
#define MOTORS_MHI_H_

/*----------------------------------------------------------------------------*/
/*                                Definitions                                 */
/*----------------------------------------------------------------------------*/
/* wheel motor driver pins */
#define MHI_WHEEL_MOTOR_STANDBY_PIN    AVR32_PIN_PB04
#define MHI_WHEEL_MOTOR_CLD_PIN        AVR32_PIN_PB10
#define MHI_WHEEL_MOTOR_1_IN1_PIN      AVR32_PIN_PB01
#define MHI_WHEEL_MOTOR_1_IN2_PIN      AVR32_PIN_PA07
#define MHI_WHEEL_MOTOR_2_IN1_PIN      AVR32_PIN_PA21
#define MHI_WHEEL_MOTOR_2_IN2_PIN      AVR32_PIN_PA20

/* vacuum motor driver pins */
#define MHI_VACUUM_MOTOR_IN1_PIN       AVR32_PIN_PA14

/* wheel motor direction */
typedef enum
{
    MHI_WHEEL_MOTOR_DIRECTION_STOP = 0u,
    MHI_WHEEL_MOTOR_DIRECTION_FORWARD,
    MHI_WHEEL_MOTOR_DIRECTION_BACKWARD,
} mhi_wheel_motor_direction_t;

/* vacuum motor state */
typedef enum 
{
    MHI_VACUUM_MOTOR_STATE_STOP = 0u,
    MHI_VACUUM_MOTOR_STATE_MOVING
} mhi_vacuum_motor_state_t;

/*----------------------------------------------------------------------------*/
/*                              Global Variables                              */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                       Public (Exportable) Functions                        */
/*----------------------------------------------------------------------------*/
void mhi_InitPwm(void);
void mhi_InitWheelMotors(void);
void mhi_InitVacuumMotor(void);

void mhi_StartWheelMotor1Forward(void);
void mhi_StartWheelMotor1Backward(void);
void mhi_SetWheelMotor1Speed(uint16_t speed);
void mhi_StopWheelMotor1(void);
mhi_wheel_motor_direction_t mhi_GetWheelMotor1Direction(void);

void mhi_StartWheelMotor2Forward(void);
void mhi_StartWheelMotor2Backward(void);
void mhi_SetWheelMotor2Speed(uint16_t speed);
void mhi_StopWheelMotor2(void);
mhi_wheel_motor_direction_t mhi_GetWheelMotor2Direction(void);

void mhi_StartVacuumMotor(uint16_t speed);
void mhi_StopVacuumMotor(void);
mhi_vacuum_motor_state_t mhi_GetVacuumMotorState(void);

#endif /* MOTORS_MHI_H_ */