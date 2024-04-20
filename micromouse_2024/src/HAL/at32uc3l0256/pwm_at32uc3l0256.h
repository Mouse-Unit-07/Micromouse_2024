/*-------------------------------- FILE INFO -----------------------------------
* Filename        : pwm_at32uc3l0256.h
* Author          : Team Kirbo
* Revision        : 1.0
* Updated         : 2024-03-31
* Purpose         : hardware specific layer
*
* This file is the header file for hardware specific PWM code.
*
* Step 3 for hardware abstraction.
*
* TODO:
*     - Break down PWM init functions (decouple from micromouse application)
*     - Add functions to modify PWM parameters during runtime
*
*     Contract and config file simplified by tailoring hardware specific code
*     to micromouse application. HAL not scalable to broader MCU use.
*-----------------------------------------------------------------------------*/

#ifndef PWM_AT32UC3L0256_H_
#define PWM_AT32UC3L0256_H_

/*----------------------------------------------------------------------------*/
/*                                Definitions                                 */
/*----------------------------------------------------------------------------*/
/* *** generic PWM configuration *** */
/* PWM output freq limited such that: */
/* MM_PWMA_GCLK_FREQUENCY/OUTPUT_FREQUENCY < 255 */
/* to minimize PWM frequency, 48MHz/190kHz = 252 */
/* for some reason we can't change GCLK, but we can change OUTPUT_FREQUENCY */
#define MM_PWMA_GCLK_ID                          AVR32_SCIF_GCLK_PWMA
#define MM_PWMA_GCLK_FREQUENCY                   (48000000u)
#define MM_PWMA_GCLK_SOURCE                      AVR32_SCIF_GC_USES_CLK_PBA
#define MM_PWMA_OUTPUT_FREQUENCY                 (190000u) //(2000000)
#define MM_PWMA_SPREAD                           (0u)
#define MM_PWMA_INTERRUPT_PRIORITY               0       /* no parenthesis */
#define MM_PWMA_TOP                              (0xFFu) /* 0~255 */

/* wheel motor 1 PWM pin configuration */
#define MM_PWMA_WHEEL_MOTOR_1_DUTY_CYCLE_INDEX    (0u)
#define MM_PWMA_WHEEL_MOTOR_1_CHANNEL_ID          (28u)
#define MM_PWMA_WHEEL_MOTOR_1_PIN                 AVR32_PWMA_28_PIN
#define MM_PWMA_WHEEL_MOTOR_1_PIN_FUNCTION        AVR32_PWMA_28_FUNCTION

/* wheel motor 2 PWM pin configuration */
#define MM_PWMA_WHEEL_MOTOR_2_DUTY_CYCLE_INDEX    (1u)
#define MM_PWMA_WHEEL_MOTOR_2_CHANNEL_ID          (13u)
#define MM_PWMA_WHEEL_MOTOR_2_PIN                 AVR32_PWMA_13_PIN
#define MM_PWMA_WHEEL_MOTOR_2_PIN_FUNCTION        AVR32_PWMA_13_FUNCTION

/* vacuum PWM pin configuration */
#define MM_PWMA_VACUUM_MOTOR_DUTY_CYCLE_INDEX     (2u)
#define MM_PWMA_VACUUM_MOTOR_CHANNEL_ID           (31u)
#define MM_PWMA_VACUUM_MOTOR_PIN                  AVR32_PWMA_31_PIN
#define MM_PWMA_VACUUM_MOTOR_PIN_FUNCTION         AVR32_PWMA_31_FUNCTION

/* clock settings */
#define MM_PBA_CLK_FREQ_HZ                        MM_PBA_FREQ_HZ

/*----------------------------------------------------------------------------*/
/*                              Global Variables                              */
/*----------------------------------------------------------------------------*/
// /* None */
// /* PWM configuration struct */
// static volatile avr32_pwma_t *pwma = &AVR32_PWMA;
// 
// /* initialize duty cycle to 0% at start for PWM off */
// static uint16_t duty_cycle[] = {0u, 0u, 0u};

/*----------------------------------------------------------------------------*/
/*                       Public (Exportable) Functions                        */
/*----------------------------------------------------------------------------*/
pwm_status_t at32uc3l0256_InitPwm(void);
pwm_status_t at32uc3l0256_SetWheelMotor1DutyCycle(uint16_t dutyCycle);
pwm_status_t at32uc3l0256_SetWheelMotor2DutyCycle(uint16_t dutyCycle);
pwm_status_t at32uc3l0256_SetVacuumMotorDutyCycle(uint16_t dutyCycle);

#endif /* PWM_AT32UC3L0256_H_ */