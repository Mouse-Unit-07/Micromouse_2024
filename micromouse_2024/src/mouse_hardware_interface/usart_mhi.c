/*-------------------------------- FILE INFO -----------------------------------
* Filename        : timer_mhi.c
* Author          : Team Kirbo
* Revision        : 1.0
* Updated         : 2024-04-07
* Purpose         : mouse hardware interface layer
*
* This is the source file for the mouse timer interface.
*
* The mouse hardware interface uses the HAL to define functions needed to
* interface w/ all mouse hardware.
*-----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*                               Include Files                                */
/*----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include "leds_mhi.h"
#include "HAL/HAL_contracts/usart_contract.h"
#include "HAL/HAL_configs/usart_config.h"
#include "usart_mhi.h"

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
* Initialize USART for micromouse.
*
* \param  None
* \retval None
*/
void mhi_InitUsart(void)
{
    usart_handler_t *usartInterface = NULL;
    config_GetUsartHandler(&usartInterface);
    
    if (usartInterface->usart_Init() != USART_SERIAL_SUCCESS)
    {
        mhi_IndicateError(MHI_LEDS_USART_ERROR);
    }
}

/**
* Print string over USART for micromouse.
*
* \param  None
* \retval None
*/
void mhi_PrintString(const char *p_userString)
{
    usart_handler_t *usartInterface = NULL;
    config_GetUsartHandler(&usartInterface);
    
    if (usartInterface->usart_PrintString(p_userString) != USART_SERIAL_SUCCESS)
    {
        mhi_IndicateError(MHI_LEDS_USART_ERROR);
    }
}

/**
* Print integer over USART for micromouse.
*
* \param  None
* \retval None
*/
void mhi_PrintInt(unsigned long userInt)
{
    usart_handler_t *usartInterface = NULL;
    config_GetUsartHandler(&usartInterface);
    
    if (usartInterface->usart_PrintInt(userInt) != USART_SERIAL_SUCCESS)
    {
        mhi_IndicateError(MHI_LEDS_USART_ERROR);
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