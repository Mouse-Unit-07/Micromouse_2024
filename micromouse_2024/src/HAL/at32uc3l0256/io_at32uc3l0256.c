/*-------------------------------- FILE INFO -----------------------------------
* Filename        : io_at32uc3l0256.c
* Author          : Team Kirbo
* Revision        : 1.0
* Updated         : 2024-03-31
* Purpose         : hardware specific layer
*
* This file is the header file for hardware specific IO code.
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
#include "HAL/HAL_contracts/io_contract.h"
#include "HAL/at32uc3l0256/io_at32uc3l0256.h"

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
* Initialize an IO pin for AT32UC3L0256 MCU.
* 
* \param[in] channelAddress Channel of IO pin (not used)
* \param[in] pinAddress Address of IO pin
* \param[in] pinOutput Output value to initialize pin to
* \param[in] pinDirection Direction of IO pin
* \param[in] pinPullUp Pull-up enable/disable
* \retval IO_SUCCESS Success
* \retval IO_ERROR Failure: Failed to initialize IO pin
*/
io_status_t at32uc3l0256_InitIO (
    const uint32_t channelAddress, const uint32_t pinAddress,
    const io_pin_state_t pinOutput, const io_direction_t pinDirection,
    const io_pullup_state_t pinPullUp)
{
    //TODO: check for invalid pin address on entry and update status
    io_status_t ioStatus = IO_ERROR;
    uint32_t ioOutput = GPIO_INIT_LOW;
    uint32_t ioDirection = GPIO_DIR_INPUT;
    uint32_t ioPullUp = 0u;
    
    /* check output */
    ioOutput = ( (pinOutput == IO_PIN_HIGH) ? GPIO_INIT_HIGH : GPIO_INIT_LOW );
    
    /* check direction */
    ioDirection = 
        ( (pinDirection == IO_OUTPUT) ?  GPIO_DIR_OUTPUT : GPIO_DIR_INPUT );
    
    /* check pull-up */
    ioPullUp = ( (pinPullUp == IO_PULLUP_ENABLE) ? GPIO_PULL_UP : 0u );
        
    /* configure pin */
    if (pinDirection == IO_OUTPUT)
    {
        gpio_configure_pin(pinAddress, ioDirection | ioOutput);
        ioStatus = IO_SUCCESS;
    }        
    else
    {
        gpio_configure_pin(pinAddress, ioDirection | pinPullUp);
        ioStatus = IO_SUCCESS;
    }    
        
    /* return status */
    return ioStatus;
}

/**
* Read an IO pin for AT32UC3L0256 MCU.
*
* \param[in] channelAddress Channel of IO pin (not used)
* \param[in] pinAddress Address of IO pin
* \param[out] p_readState Value of input pin
* \retval IO_SUCCESS Success
* \retval IO_ERROR Failure: Failed to read IO pin
*/
io_status_t at32uc3l0256_ReadPin (
    const uint32_t channelAddress, const uint32_t pinAddress,
    io_pin_state_t* p_readState)
{
    //TODO: check for invalid pin address on entry and update status
    io_status_t ioStatus = IO_ERROR;
    
    /* read pin */
    *p_readState = 
        ( (gpio_get_pin_value(pinAddress) == 0u) ? IO_PIN_LOW : IO_PIN_HIGH);
    ioStatus = IO_SUCCESS;
        
    /* return status */
    return ioStatus;
}

/**
* Write to an IO pin for AT32UC3L0256 MCU.
*
* \param[in] channelAddress Channel of IO pin (not used)
* \param[in] pinAddress Address of IO pin
* \param[in] writeState Value to write
* \retval IO_SUCCESS Success
* \retval IO_ERROR Failure: Failed to write to IO pin
*/
io_status_t at32uc3l0256_WritePin (
    const uint32_t channelAddress, const uint32_t pinAddress,
    const io_pin_state_t writeState)
{
    //TODO: check for invalid pin address on entry and update status
    io_status_t ioStatus = IO_ERROR;
    
    /* write to pin */
    if (writeState == IO_PIN_LOW)
    {
        gpio_clr_gpio_pin(pinAddress);
        ioStatus = IO_SUCCESS;
    }   
    else
    {
        gpio_set_gpio_pin(pinAddress);
        ioStatus = IO_SUCCESS;
    }
    
    /* return status */
    return ioStatus;
}

/**
* Toggle an IO pin for AT32UC3L0256 MCU.
*
* \param[in] channelAddress Channel of IO pin (not used)
* \param[in] pinAddress Address of IO pin
* \retval IO_SUCCESS Success
* \retval IO_ERROR Failure: Failed to write to IO pin
*/
io_status_t at32uc3l0256_TogglePin (
    const uint32_t channelAddress, const uint32_t pinAddress)
{
    //TODO: check for invalid pin address on entry and update status
    io_status_t ioStatus = IO_ERROR;
    
    gpio_tgl_gpio_pin(pinAddress);
    ioStatus = IO_SUCCESS;
    
    /* return status */
    return ioStatus;
}

/**
* Enable/disable pull-up on a pin for AT32UC3L0256 MCU.
*
* \param[in] channelAddress Channel of IO pin (not used)
* \param[in] pinAddress Address of IO pin
* \retval IO_SUCCESS Success
* \retval IO_ERROR Failure: Failed to write to IO pin
*/
io_status_t at32uc3l0256_WritePullUp (
    const uint32_t channelAddress, const uint32_t pinAddress,
    const io_pullup_state_t writeState)
{
    //TODO: check for invalid pin address on entry and update status
    io_status_t ioStatus = IO_ERROR;
    
    if (writeState == IO_PULLUP_ENABLE)
    {
        gpio_enable_pin_pull_up(pinAddress);
        ioStatus = IO_SUCCESS;
    }
    else 
    {
        gpio_disable_pin_pull_up(pinAddress);
        ioStatus = IO_SUCCESS;
    }
    
    /* return status */
    return ioStatus;
}

/*----------------------------------------------------------------------------*/
/*                           Local Shared Functions                           */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                         Private (Static) Functions                         */
/*----------------------------------------------------------------------------*/
/* None */

