/*-------------------------------- FILE INFO -----------------------------------
* Filename        : leds_mhi.c
* Author          : Team Kirbo
* Revision        : 1.0
* Updated         : 2024-04-07
* Purpose         : mouse hardware interface layer
*
* This is the source file for the mouse LED interface.
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
#include "HAL/HAL_contracts/clock_contract.h"
#include "HAL/HAL_configs/clock_config.h"
#include "clock_mhi.h"

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
/*                       Public (Exportable) Functions                        */
/*----------------------------------------------------------------------------*/
/**
* Initialize all micromouse LED pins.
*
* \param  None
* \retval None
*/
void mhi_InitLeds(void)
{
    io_handler_t *ioInterface = NULL;
    config_GetIOHandler(&ioInterface);
    
    ioInterface->io_Init(0u, MHI_D1_LED_PIN, IO_PIN_LOW, 
        IO_OUTPUT, IO_PULLUP_DISABLE);
        
    ioInterface->io_Init(0u, MHI_D2_LED_PIN, IO_PIN_LOW,
        IO_OUTPUT, IO_PULLUP_DISABLE);
    
    ioInterface->io_Init(0u, MHI_D3_LED_PIN, IO_PIN_LOW,
        IO_OUTPUT, IO_PULLUP_DISABLE);
    
    ioInterface->io_Init(0u, MHI_ERR_LED_PIN, IO_PIN_LOW,
        IO_OUTPUT, IO_PULLUP_DISABLE);
}

/**
* Turns on D1 LED.
*
* \param  None
* \retval None
*/
void mhi_SetD1Led(void)
{
    io_handler_t *ioInterface = NULL;
    config_GetIOHandler(&ioInterface);
    
    ioInterface->io_WriteOutput(0u, MHI_D1_LED_PIN, IO_PIN_HIGH);
}

/**
* Turns on D2 LED.
*
* \param  None
* \retval None
*/
void mhi_SetD2Led(void)
{
    io_handler_t *ioInterface = NULL;
    config_GetIOHandler(&ioInterface);
    
    ioInterface->io_WriteOutput(0u, MHI_D2_LED_PIN, IO_PIN_HIGH);
}

/**
* Turns on D3 LED.
*
* \param  None
* \retval None
*/
void mhi_SetD3Led(void)
{
    io_handler_t *ioInterface = NULL;
    config_GetIOHandler(&ioInterface);
    
    ioInterface->io_WriteOutput(0u, MHI_D3_LED_PIN, IO_PIN_HIGH);
}

/**
* Turns on ERR LED.
*
* \param  None
* \retval None
*/
void mhi_SetErrLed(void)
{
    io_handler_t *ioInterface = NULL;
    config_GetIOHandler(&ioInterface);
    
    ioInterface->io_WriteOutput(0u, MHI_ERR_LED_PIN, IO_PIN_HIGH);
}

/**
* Turns off D1 LED.
*
* \param  None
* \retval None
*/
void mhi_ClearD1Led(void)
{
    io_handler_t *ioInterface = NULL;
    config_GetIOHandler(&ioInterface);
    
    ioInterface->io_WriteOutput(0u, MHI_D1_LED_PIN, IO_PIN_LOW);
}

/**
* Turns off D2 LED.
*
* \param  None
* \retval None
*/
void mhi_ClearD2Led(void)
{
    io_handler_t *ioInterface = NULL;
    config_GetIOHandler(&ioInterface);
    
    ioInterface->io_WriteOutput(0u, MHI_D2_LED_PIN, IO_PIN_LOW);
}

/**
* Turns off D3 LED.
*
* \param  None
* \retval None
*/
void mhi_ClearD3Led(void)
{
    io_handler_t *ioInterface = NULL;
    config_GetIOHandler(&ioInterface);
    
    ioInterface->io_WriteOutput(0u, MHI_D3_LED_PIN, IO_PIN_LOW);
}

/**
* Turns off ERR LED.
*
* \param  None
* \retval None
*/
void mhi_ClearErrLed(void)
{
    io_handler_t *ioInterface = NULL;
    config_GetIOHandler(&ioInterface);
    
    ioInterface->io_WriteOutput(0u, MHI_ERR_LED_PIN, IO_PIN_LOW);
}

/**
* Toggles D1 LED.
*
* \param  None
* \retval None
*/
void mhi_ToggleD1Led(void)
{
    io_handler_t *ioInterface = NULL;
    config_GetIOHandler(&ioInterface);
    
    ioInterface->io_ToggleOutput(0u, MHI_D1_LED_PIN);
}

/**
* Toggles D2 LED.
*
* \param  None
* \retval None
*/
void mhi_ToggleD2Led(void)
{
    io_handler_t *ioInterface = NULL;
    config_GetIOHandler(&ioInterface);
    
    ioInterface->io_ToggleOutput(0u, MHI_D2_LED_PIN);
}

/**
* Toggles D3 LED.
*
* \param  None
* \retval None
*/
void mhi_ToggleD3Led(void)
{
    io_handler_t *ioInterface = NULL;
    config_GetIOHandler(&ioInterface);
    
    ioInterface->io_ToggleOutput(0u, MHI_D3_LED_PIN);
}

/**
* Toggles ERR LED.
*
* \param  None
* \retval None
*/
void mhi_ToggleErrLed(void)
{
    io_handler_t *ioInterface = NULL;
    config_GetIOHandler(&ioInterface);
    
    ioInterface->io_ToggleOutput(0u, MHI_ERR_LED_PIN);
}

/**
* Indicates error by turning on ERR LED and toggling other LEDs.
*
* \param[in] 
* \retval None
*/
void mhi_IndicateError(const mhi_error_type_t error)
{
    clock_handler_t *clockInterface = NULL;
    config_GetClockHandler(&clockInterface);
    
    mhi_SetErrLed();
    mhi_ClearD1Led();
    mhi_ClearD2Led();
    mhi_ClearD3Led();
    
    if (error == MHI_LEDS_GENERIC_ERROR)
    {
        while(1)
        {
            mhi_SetD1Led();
            clockInterface->clock_DelayMs(500);
            mhi_ClearD1Led();
            clockInterface->clock_DelayMs(500);
        }
    }
    else if (error == MHI_LEDS_TC_ERROR)
    {
        while(1)
        {
            mhi_SetD2Led();
            clockInterface->clock_DelayMs(500);
            mhi_ClearD2Led();
            clockInterface->clock_DelayMs(500);
        }
    }
    else if (error == MHI_LEDS_USART_ERROR)
    {
        while(1)
        {
            mhi_SetD1Led();
            mhi_SetD2Led();
            clockInterface->clock_DelayMs(500);
            mhi_ClearD1Led();
            mhi_ClearD2Led();
            clockInterface->clock_DelayMs(500);
        }
    }
    else if (error == MHI_LEDS_LOW_BATTERY_ERROR)
    {
        while(1)
        {
            mhi_SetD3Led();
            clockInterface->clock_DelayMs(500);
            mhi_ClearD3Led();
            clockInterface->clock_DelayMs(500);
        }
    }
    else if (error == MHI_LEDS_IR_SENSOR_ERROR)
    {
        while(1)
        {
            mhi_SetD3Led();
            mhi_SetD1Led();
            clockInterface->clock_DelayMs(500);
            mhi_ClearD3Led();
            mhi_ClearD1Led();
            clockInterface->clock_DelayMs(500);
        }
    }
}

/*----------------------------------------------------------------------------*/
/*                           Local Shared Functions                           */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                         Private (Static) Functions                         */
/*----------------------------------------------------------------------------*/
/* None */
