/*-------------------------------- FILE INFO -----------------------------------
* Filename        : pwm_at32uc3l0256.c
* Author          : Ryuichi Lin
* Revision        : 1.0
* Updated         : 2024-03-31
* Purpose         : hardware specific layer
*
* This file is the header file for hardware specific PWM code.
*
* Step 3 for hardware abstraction.
*
* (if applicable)
* Target Hardware    : AT32UC3L0256
* IDE                : Atmel Studio 7.4.2542
* SDK                : ASF 3.52.0
*-----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*                               Include Files                                */
/*----------------------------------------------------------------------------*/
#include <asf.h>
#include <stdint.h>
#include "HAL/HAL_contracts/pwm_contract.h"
#include "HAL/at32uc3l0256/pwm_at32uc3l0256.h"
#include "HAL/HAL_contracts/clock_contract.h"       /* needed for PBA freq */
#include "HAL/at32uc3l0256/clock_at32uc3l0256.h"    /* needed for PBA freq */

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
/* PWM configuration struct */
static volatile avr32_pwma_t *pwma = &AVR32_PWMA;

/* initialize duty cycle to 0% at start for PWM off */
static uint16_t duty_cycle[] = {0u, 0u, 0u};

/*----------------------------------------------------------------------------*/
/*                    Private (Static) Function Prototype                     */
/*----------------------------------------------------------------------------*/
static void mm_EnablePwmInterrupts(void);

/*----------------------------------------------------------------------------*/
/*                         Interrupt Service Routines                         */
/*----------------------------------------------------------------------------*/
/* ISR for PWM */
ISR(tofl_irq, AVR32_PWMA_IRQ_GROUP, MM_PWMA_INTERRUPT_PRIORITY)
{
    pwma->scr=AVR32_PWMA_SCR_TOFL_MASK;
}

/*----------------------------------------------------------------------------*/
/*                       Public (Exportable) Functions                        */
/*----------------------------------------------------------------------------*/
/**
* Initialize PWMA interface for motor control.
* Configure PWM frequency, top value, and PWM pins.
*
* \param    None
* \retval   None
*/
pwm_status_t at32uc3l0256_InitPwm(void)
{
    pwm_status_t pwmStatus = PWM_ERROR;
    
    uint32_t div;
    bool config_status = FAIL;
    bool set_value_status = FAIL;
    
    /* initialize PWM pins */
    static const gpio_map_t PWMA_GPIO_MAP = {
        {MM_PWMA_WHEEL_MOTOR_1_PIN, MM_PWMA_WHEEL_MOTOR_1_PIN_FUNCTION},
        {MM_PWMA_WHEEL_MOTOR_2_PIN, MM_PWMA_WHEEL_MOTOR_2_PIN_FUNCTION},
        {MM_PWMA_VACUUM_MOTOR_PIN, MM_PWMA_VACUUM_MOTOR_PIN_FUNCTION},
    };
    gpio_enable_module(PWMA_GPIO_MAP,
    sizeof(PWMA_GPIO_MAP) / sizeof(PWMA_GPIO_MAP[0]));
    
    /* initialize clock source for PWM */
    div = div_ceil((sysclk_get_pba_hz()), MM_PWMA_GCLK_FREQUENCY);
    genclk_enable_config(MM_PWMA_GCLK_ID, MM_PWMA_GCLK_SOURCE, div);
    
    /* configure PWM interface- frequencies and spread */
    config_status = pwma_config_enable(pwma, MM_PWMA_OUTPUT_FREQUENCY,
        MM_PWMA_GCLK_FREQUENCY, MM_PWMA_SPREAD);
    if (config_status == FAIL)
        pwmStatus = PWM_ERROR;
    else 
    {
        /* load initial PWM duty cycles */
        set_value_status = pwma_set_multiple_values(
            pwma,
            ((MM_PWMA_WHEEL_MOTOR_1_CHANNEL_ID << 0) |
                (MM_PWMA_WHEEL_MOTOR_2_CHANNEL_ID << 8) |
                (MM_PWMA_VACUUM_MOTOR_CHANNEL_ID << 16)),
            (uint16_t*)&duty_cycle);
        if (set_value_status == FAIL)
            pwmStatus = PWM_ERROR;
        else
        {
            /* set TOP value to change set speed param range to 0~255 */
            config_status = pwma_write_top_value(pwma, MM_PWMA_TOP);
            if (config_status == FAIL)
                pwmStatus = PWM_ERROR;
            else
            {
                /* PWM interrupt init moved to MAIN- "tofl_irq" scope issue */
                mm_EnablePwmInterrupts();
                
                pwmStatus = PWM_SUCCESS;
            }
        }
    }
    
    /* return status */
    return pwmStatus;
}

pwm_status_t at32uc3l0256_SetWheelMotor1DutyCycle(uint16_t dutyCycle)
{
    pwm_status_t pwmStatus = PWM_ERROR;
    
    bool set_value_status = FAIL;
    
    /* check for valid duty cycle */
    if (dutyCycle > MM_PWMA_TOP)
        pwmStatus = PWM_ERROR;
    else
    {
        /* change PWM duty cycle w/ param */
        duty_cycle[MM_PWMA_WHEEL_MOTOR_1_DUTY_CYCLE_INDEX] = dutyCycle;
//         set_value_status = pwma_set_channels_value(
//             pwma,
//             MM_PWMA_WHEEL_MOTOR_1_CHANNEL_ID,
//             duty_cycle[MM_PWMA_WHEEL_MOTOR_1_DUTY_CYCLE_INDEX]
//         );
        
        set_value_status = pwma_set_multiple_values(
            pwma,
            ((MM_PWMA_WHEEL_MOTOR_1_CHANNEL_ID << 0) | 
             (MM_PWMA_WHEEL_MOTOR_2_CHANNEL_ID << 8) | 
             (MM_PWMA_VACUUM_MOTOR_CHANNEL_ID << 16)),
            (uint16_t*)&duty_cycle);

        if (set_value_status == FAIL)
            pwmStatus = PWM_ERROR;
        else
            pwmStatus = PWM_SUCCESS;
    }
    
    /* return status */
    return pwmStatus;
}

pwm_status_t at32uc3l0256_SetWheelMotor2DutyCycle(uint16_t dutyCycle)
{
    pwm_status_t pwmStatus = PWM_ERROR;
    
    bool set_value_status = FAIL;
    
    /* check for valid duty cycle */
    if (dutyCycle > MM_PWMA_TOP)
        pwmStatus = PWM_ERROR;
    else
    {
        /* change PWM duty cycle w/ param */
        duty_cycle[MM_PWMA_WHEEL_MOTOR_2_DUTY_CYCLE_INDEX] = dutyCycle;
//         set_value_status = pwma_set_channels_value(
//             pwma,
//             MM_PWMA_WHEEL_MOTOR_2_CHANNEL_ID,
//             duty_cycle[MM_PWMA_WHEEL_MOTOR_2_DUTY_CYCLE_INDEX]
//         );
        set_value_status = pwma_set_multiple_values(
            pwma,
            ((MM_PWMA_WHEEL_MOTOR_1_CHANNEL_ID << 0) |
             (MM_PWMA_WHEEL_MOTOR_2_CHANNEL_ID << 8) |
             (MM_PWMA_VACUUM_MOTOR_CHANNEL_ID << 16)),
            (uint16_t*)&duty_cycle);

        if (set_value_status == FAIL)
            pwmStatus = PWM_ERROR;
        else
            pwmStatus = PWM_SUCCESS;
    }
    
    /* return status */
    return pwmStatus;
}

pwm_status_t at32uc3l0256_SetVacuumMotorDutyCycle(uint16_t dutyCycle)
{
    pwm_status_t pwmStatus = PWM_ERROR;
    
    bool set_value_status = FAIL;
    
    /* check for valid duty cycle */
    if (dutyCycle > MM_PWMA_TOP)
        pwmStatus = PWM_ERROR;
    else
    {
        /* change PWM duty cycle w/ param */
        duty_cycle[MM_PWMA_VACUUM_MOTOR_DUTY_CYCLE_INDEX] = dutyCycle;
//         set_value_status = pwma_set_channels_value(
//             pwma,
//             MM_PWMA_VACUUM_MOTOR_CHANNEL_ID,
//             duty_cycle[MM_PWMA_VACUUM_MOTOR_DUTY_CYCLE_INDEX]
//         );
        set_value_status = pwma_set_multiple_values(
            pwma,
            ((MM_PWMA_WHEEL_MOTOR_1_CHANNEL_ID << 0) |
             (MM_PWMA_WHEEL_MOTOR_2_CHANNEL_ID << 8) |
             (MM_PWMA_VACUUM_MOTOR_CHANNEL_ID << 16)),
            (uint16_t*)&duty_cycle);

        if (set_value_status == FAIL)
            pwmStatus = PWM_ERROR;
        else
            pwmStatus = PWM_SUCCESS;
    }
    
    /* return status */
    return pwmStatus;
}

/*----------------------------------------------------------------------------*/
/*                           Local Shared Functions                           */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                         Private (Static) Functions                         */
/*----------------------------------------------------------------------------*/
/**
* Enables interrupts for PWM.
* Enables interrupts for PWM- has to be in MAIN to be in scope of "tofl_irq"
*
* \param    None
* \retval   None
*/
static void mm_EnablePwmInterrupts(void)
{
    //cpu_irq_disable();
     
    //irq_initialize_vectors();
    irq_register_handler(&tofl_irq, AVR32_PWMA_IRQ, MM_PWMA_INTERRUPT_PRIORITY);
    pwma->ier = AVR32_PWMA_IER_TOFL_MASK;
     
    //cpu_irq_enable();
}
