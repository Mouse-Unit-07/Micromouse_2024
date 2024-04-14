/*-------------------------------- FILE INFO -----------------------------------
* Filename        : usart_at32uc3l0256.c
* Author          : Ryuichi Lin
* Revision        : 1.0
* Updated         : 2024-03-31
* Purpose         : hardware specific layer
*
* This file is the header file for hardware specific USART code.
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
#include "HAL/HAL_contracts/usart_contract.h"
#include "HAL/at32uc3l0256/usart_at32uc3l0256.h"

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
* Initialize USART interface for for AT32UC3L0256 MCU.
*
* \retval USART_SUCCESS Success
* \retval USART_ERROR Failure: Failed to initialize USART
*/
usart_status_t at32uc3l0256_InitUsart(void)
{
    usart_status_t usartStatus = USART_SERIAL_ERROR;
    
    static const gpio_map_t USART_GPIO_MAP =
    {
        {MM_USART_RX_PIN, MM_USART_RX_FUNCTION},
        {MM_USART_TX_PIN, MM_USART_TX_FUNCTION}
    };
    
    /* USART options */
    static const usart_options_t USART_OPTIONS =
    {
        .baudrate     = MM_USART_BAUDRTE,
        .charlength   = MM_USART_CHAR_LENGTH,
        .paritytype   = USART_NO_PARITY,
        .stopbits     = USART_1_STOPBIT,
        .channelmode  = USART_NORMAL_CHMODE
    };
    
    /* Assign GPIO to USART */
    gpio_enable_module(USART_GPIO_MAP,
        sizeof(USART_GPIO_MAP) / sizeof(USART_GPIO_MAP[0]));
    
    /* Initialize USART in RS232 mode */
    if (usart_init_rs232(MM_USART_USART_ADDRESS, 
        &USART_OPTIONS, MM_PBA_CLK_FREQ_HZ) != USART_SUCCESS)
        usartStatus = USART_SERIAL_ERROR;
    else
        usartStatus = USART_SERIAL_SUCCESS;
        
    /* return status */
    return usartStatus;
}


/**
* Print string using serial USART.
*
* \param[in] p_userString String to print
* \retval USART_SUCCESS Success
* \retval USART_ERROR Failure: Failed to print using USART
*/
usart_status_t at32uc3l0256_PrintString(const char *p_userString)
{
    usart_status_t usartStatus = USART_SERIAL_ERROR;
    
    /* Transmit string with USART */
    usart_write_line(MM_USART_USART_ADDRESS, p_userString);
    
    usartStatus =  USART_SERIAL_SUCCESS;
    
    /* return status */
    return usartStatus;
}

/**
* Print integer using serial USART.
*
* \param[in] userInt Integer to print
* \retval USART_SUCCESS Success
* \retval USART_ERROR Failure: Failed to print using USART
*/
usart_status_t at32uc3l0256_PrintInt(unsigned long userInt)
{
    usart_status_t usartStatus = USART_SERIAL_ERROR;
    
    /* uint32_t max has 10 digits, so 11 digits including termination char */
    char tmp[11];
    int i = sizeof(tmp) - 1;

    /* Convert the given number to an ASCII decimal representation */
    tmp[i] = '\0';
    do
    {
        tmp[--i] = '0' + userInt % 10;
        userInt /= 10;
    } while (userInt);

    // Transmit the resulting string with the given USART.
    usart_write_line(MM_USART_USART_ADDRESS, tmp + i);
    
    usartStatus = USART_SERIAL_SUCCESS;
    
    /* return status */
    return usartStatus;
}

/*----------------------------------------------------------------------------*/
/*                           Local Shared Functions                           */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                         Private (Static) Functions                         */
/*----------------------------------------------------------------------------*/
/* None */