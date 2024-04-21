/*-------------------------------- FILE INFO -----------------------------------
* Filename        : io_contract.h
* Author          : Team Kirbo
* Revision        : 1.0
* Updated         : 2024-03-31
* Purpose         : HAL contract layer
*
* This file contains the contract to abstract an IO interface.
* 
* Contracts define features of interfaces so high level code can use abstract
* handlers instead of hardware specific code. HAL config files use these
* contracts to link to hardware specific code. 
* 
* Step 1 for hardware abstraction.
*-----------------------------------------------------------------------------*/

#ifndef IO_CONTRACT_H_
#define IO_CONTRACT_H_

/*----------------------------------------------------------------------------*/
/*                                Definitions                                 */
/*----------------------------------------------------------------------------*/
/* pin digital state enumeration */
typedef enum
{
    IO_PIN_LOW = 0u,
    IO_PIN_HIGH
} io_pin_state_t;

/* pin direction enumeration */
typedef enum
{
    IO_INPUT = 0u,
    IO_OUTPUT
} io_direction_t;

/* pin pull up state enumeration */
typedef enum
{
    IO_PULLUP_DISABLE = 0u,
    IO_PULLUP_ENABLE
} io_pullup_state_t;

/* IO status enumeration */
typedef enum
{
    IO_SUCCESS = 0u,
    IO_ERROR,
    IO_WRITE_ERROR,
    IO_READ_ERROR
} io_status_t;

/* IO interface contract- used to create handlers */
typedef struct  
{
    io_status_t (*io_Init)(
        const uint32_t channelAddress, const uint32_t pinAddress, 
        const io_pin_state_t pinOutput, const io_direction_t pinDirection, 
        const io_pullup_state_t pinPullUp);
    
    io_status_t (*io_ReadInput)(
        const uint32_t channelAddress, const uint32_t pinAddress, 
        io_pin_state_t* p_readState);
    
    io_status_t (*io_WriteOutput)(
        const uint32_t channelAddress, const uint32_t pinAddress, 
        const io_pin_state_t writeState);
    
    io_status_t (*io_ToggleOutput)(
        const uint32_t channelAddress, const uint32_t pinAddress);
    
    io_status_t (*io_WritePullup)(
        const uint32_t channelAddress, const uint32_t pinAddress, 
        const io_pullup_state_t writeState);
} io_handler_t;

/*----------------------------------------------------------------------------*/
/*                              Global Variables                              */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                       Public (Exportable) Functions                        */
/*----------------------------------------------------------------------------*/
/* None */

#endif /* IO_CONTRACT_H_ */
