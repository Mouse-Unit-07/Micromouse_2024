/*-------------------------------- FILE INFO -----------------------------------
* Filename        : eic_at32uc3l0256.c
* Author          : Ryuichi Lin
* Revision        : 1.0
* Updated         : 2024-03-31
* Purpose         : hardware specific layer
*
* This file is the header file for hardware specific external interrupt
* controller (eic) code.
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
#include "HAL/HAL_contracts/eic_contract.h"
#include "HAL/at32uc3l0256/eic_at32uc3l0256.h"

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
/* None */

/*----------------------------------------------------------------------------*/
/*                    Private (Static) Function Prototype                     */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                         Interrupt Service Routines                         */
/*----------------------------------------------------------------------------*/
/* ISR for motor 1 encoder channel A rising edge  */
__attribute__((__interrupt__))
static void eic_int_handler1(void)
{
    Disable_global_interrupt();
    
    eic_clear_interrupt_line(&AVR32_EIC, MM_EIC_M1_LINE);
    
    if (gpio_get_pin_value(MM_M1_ENC_B_PIN) > 0)
    {
        g_mm_Motor1EncoderEdgeCount--; /* backwards */
    }
    else
    {
        g_mm_Motor1EncoderEdgeCount++; /* forwards */
    }
    
    Enable_global_interrupt();
}

/* ISR for motor 2 encoder channel A rising edge */
__attribute__((__interrupt__))
static void eic_int_handler2(void)
{
    eic_clear_interrupt_line(&AVR32_EIC, MM_EIC_M2_LINE);
    
    if (gpio_get_pin_value(MM_M2_ENC_B_PIN) > 0)
    {
        g_mm_Motor2EncoderEdgeCount++; /* forwards */
    }
    else
    {
        g_mm_Motor2EncoderEdgeCount--; /* backwards */
    }
}

/* ISR for software configuration pin rising edge */
__attribute__((__interrupt__))
static void eic_int_handler3(void)
{
    eic_clear_interrupt_line(&AVR32_EIC, MM_EIC_CONFIG_LINE);
    
    /* delay to debounce make time */
    delay_ms(MM_CONFIG_PIN_BOUNCE_TIME_MS);
    
    /* wait to read HIGH on the pin for switch release */
    while (gpio_get_pin_value(MM_CONFIG_PIN) != MM_CONFIG_PIN_DEFAULT_VALUE)
    {
        asm("NOP");
    }
    
    /* delay to debounce break time */
    delay_ms(MM_CONFIG_PIN_BOUNCE_TIME_MS);
    
    /* increment edge count */
    g_mm_ConfigSwitchEdgeCount++;
}

/*----------------------------------------------------------------------------*/
/*                       Public (Exportable) Functions                        */
/*----------------------------------------------------------------------------*/
/**
* Enable interrupts for micromouse wheel motor encoder channel A pins for 
* AT32UC3L0256 MCU.
*
* \retval EIC_SUCCESS Success
* \retval EIC_ERROR Failure: Failed to initialize interrupt
*/
eic_status_t at32uc3l0256_InitEncoderInterrupts(void)
{
    eic_status_t eicStatus = EIC_ERROR;
    
    /* configure motor encoder channel B pins for reading with pull-ups*/
    gpio_configure_pin(MM_M1_ENC_B_PIN, GPIO_DIR_INPUT | GPIO_PULL_UP);
    gpio_configure_pin(MM_M2_ENC_B_PIN, GPIO_DIR_INPUT | GPIO_PULL_UP);
    
    eic_options_t eic_options[MM_EIC_ENCODER_LINES_COUNT];
    
    /* motor 1 */
    eic_options[0].eic_mode  = MM_EIC_M1_MODE;
    eic_options[0].eic_edge  = MM_EIC_M1_EDGE;
    eic_options[0].eic_async = MM_EIC_M1_SYNC;
    eic_options[0].eic_line  = MM_EIC_M1_LINE;
    
    /* motor 2 */
    eic_options[1].eic_mode  = MM_EIC_M2_MODE;
    eic_options[1].eic_edge  = MM_EIC_M2_EDGE;
    eic_options[1].eic_async = MM_EIC_M2_SYNC;
    eic_options[1].eic_line  = MM_EIC_M2_LINE;
    
    static const gpio_map_t EIC_GPIO_MAP =
    {
        {MM_EIC_M1_PIN_LINE, MM_EIC_M1_FUNCTION_LINE},
        {MM_EIC_M2_PIN_LINE, MM_EIC_M2_FUNCTION_LINE}
    };
    gpio_enable_module(EIC_GPIO_MAP, 
        sizeof(EIC_GPIO_MAP) / sizeof(EIC_GPIO_MAP[0]) );
    
    //Disable_global_interrupt(); /* removed- moved to INTC interface */
    /* Initialize interrupt vectors. */
    //INTC_init_interrupts();    /* removed- moved to INTC interface */
    
    /* Register the EIC interrupt handlers to the interrupt controller. */
    INTC_register_interrupt(&eic_int_handler1,
        MM_EIC_M1_IRQ_LINE, MM_INTC_M1_LEVEL);
    INTC_register_interrupt(&eic_int_handler2,
        MM_EIC_M2_IRQ_LINE, MM_INTC_M2_LEVEL);
    
    /* Initialize the EIC controller with the options */
    eic_init(&AVR32_EIC, eic_options, MM_EIC_ENCODER_LINES_COUNT);
    
    eic_enable_lines(&AVR32_EIC,
        (1 << eic_options[1].eic_line) | (1 << eic_options[0].eic_line));
    eic_enable_interrupt_lines(&AVR32_EIC,
        (1 << eic_options[1].eic_line) | (1 << eic_options[0].eic_line));
    
    /* Enable all interrupts. */
    //Enable_global_interrupt();    /* removed- moved to INTC interface */
    
    gpio_enable_pin_pull_up(MM_EIC_M1_PIN_LINE);
    gpio_enable_pin_pull_up(MM_EIC_M2_PIN_LINE);
    
    eicStatus = EIC_SUCCESS;
    
    /* return status */
    return eicStatus;
}

/**
* Enable interrupt for micromouse software configuration pin for AT32UC3L0256
* MCU.
*
* \retval EIC_SUCCESS Success
* \retval EIC_ERROR Failure: Failed to initialize interrupt
*/
eic_status_t at32uc3l0256_InitConfigPinInterrupt(void)
{
     eic_status_t eicStatus = EIC_ERROR;
     
     /* enable pull-up for config pushbutton */
     //gpio_configure_pin(MM_CONFIG_PIN, GPIO_DIR_INPUT | GPIO_PULL_UP);
     
     eic_options_t eic_options[1]; /* configuring a single pin */
     eic_options[0].eic_mode  = MM_EIC_CONFIG_MODE;
     eic_options[0].eic_edge  = MM_EIC_CONFIG_EDGE;
     eic_options[0].eic_async = MM_EIC_CONFIG_SYNC;
     eic_options[0].eic_line  = MM_EIC_CONFIG_LINE;
     
     static const gpio_map_t EIC_GPIO_MAP =
     {
         {MM_EIC_CONFIG_PIN_LINE, MM_EIC_COFIG_FUNCTION_LINE}
     };
     gpio_enable_module(EIC_GPIO_MAP, 
         sizeof(EIC_GPIO_MAP) / sizeof(EIC_GPIO_MAP[0]));
     
     //Disable_global_interrupt(); /* removed- moved to INTC interface */
     /* Initialize interrupt vectors. */
     //INTC_init_interrupts();    /* removed- moved to INTC interface */
     
     /* Register the EIC interrupt handlers to the interrupt controller. */
     INTC_register_interrupt(&eic_int_handler3,
         MM_EIC_CONFIG_IRQ_LINE, MM_INTC_CONFIG_LEVEL);
     
     /* Initialize the EIC controller with the options */
     eic_init(&AVR32_EIC, eic_options, 1);
     
     eic_enable_lines(&AVR32_EIC, (1 << eic_options[0].eic_line));
     eic_enable_interrupt_lines(&AVR32_EIC, (1 << eic_options[0].eic_line));
     
     /* Enable all interrupts. */
     //Enable_global_interrupt();    /* removed- moved to INTC interface */
     
     gpio_enable_pin_pull_up(MM_EIC_CONFIG_PIN_LINE);
     
     eicStatus = EIC_SUCCESS;
     
     /* return status */
     return eicStatus;
}

/**
* Access function for encoder 1 edge count for AT32UC3L0256 MCU.
*
* \param[out] p_edgeCount Edge counter variable value for encoder 1
* \retval EIC_SUCCESS Success
* \retval EIC_ERROR Failure: Failed to read edge count
*/
eic_status_t at32uc3l0256_GetEncoder1EdgeCount(uint32_t *p_edgeCount)
{
    eic_status_t eicStatus = EIC_ERROR;
    
    *p_edgeCount = g_mm_Motor1EncoderEdgeCount;
    
    eicStatus = EIC_SUCCESS;
    
    return eicStatus;
}

/**
* Access function for encoder 2 edge count for AT32UC3L0256 MCU.
*
* \param[out] p_edgeCount Edge counter variable value for encoder 2
* \retval EIC_SUCCESS Success
* \retval EIC_ERROR Failure: Failed to read edge count
*/
eic_status_t at32uc3l0256_GetEncoder2EdgeCount(uint32_t *p_edgeCount)
{
    eic_status_t eicStatus = EIC_ERROR;
    
    *p_edgeCount = g_mm_Motor2EncoderEdgeCount;
    
    eicStatus = EIC_SUCCESS;
    
    return eicStatus;
}

/**
* Access function for config pin edge count for AT32UC3L0256 MCU.
*
* \param[out] p_edgeCount Edge counter variable value for config pin
* \retval EIC_SUCCESS Success
* \retval EIC_ERROR Failure: Failed to read edge count
*/
eic_status_t at32uc3l0256_GetConfigPinEdgeCount(uint32_t *p_edgeCount)
{
    eic_status_t eicStatus = EIC_ERROR;
    
    *p_edgeCount = g_mm_ConfigSwitchEdgeCount;
    
    eicStatus = EIC_SUCCESS;
    
    return eicStatus;
}

/**
* Clear encoder 1 pin edge count for AT32UC3L0256 MCU.
*
* \retval EIC_SUCCESS Success
* \retval EIC_ERROR Failure: Failed to read edge count
*/
eic_status_t at32uc3l0256_ClearEncoder1EdgeCount(void)
{
    g_mm_Motor1EncoderEdgeCount = 0u;
    
    return EIC_SUCCESS;
}

/**
* Clear encoder 2 pin edge count for AT32UC3L0256 MCU.
*
* \retval EIC_SUCCESS Success
* \retval EIC_ERROR Failure: Failed to read edge count
*/
eic_status_t at32uc3l0256_ClearEncoder2EdgeCount(void)
{
    g_mm_Motor2EncoderEdgeCount = 0u;
    
    return EIC_SUCCESS;
}

/**
* Clear config pin edge count for AT32UC3L0256 MCU.
*
* \retval EIC_SUCCESS Success
* \retval EIC_ERROR Failure: Failed to read edge count
*/
eic_status_t at32uc3l0256_ClearConfigPinEdgeCount(void)
{
    g_mm_ConfigSwitchEdgeCount = 0u;
    
    return EIC_SUCCESS;
}
