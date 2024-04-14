/*-------------------------------- FILE INFO -----------------------------------
* Filename        : io_at32uc3l0256.h
* Author          : Ryuichi Lin
* Revision        : 1.0
* Updated         : 2024-03-31
* Purpose         : hardware specific layer
*
* This file is the header file for hardware specific IO code.
* 
* Step 3 for hardware abstraction.
*-----------------------------------------------------------------------------*/

#ifndef IO_AT32UC3L0256_H_
#define IO_AT32UC3L0256_H_

/*----------------------------------------------------------------------------*/
/*                                Definitions                                 */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                              Global Variables                              */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                       Public (Exportable) Functions                        */
/*----------------------------------------------------------------------------*/
io_status_t at32uc3l0256_InitIO (
    const uint32_t channelAddress, const uint32_t pinAddress,
    const io_pin_state_t pinOutput, const io_direction_t pinDirection,
    const io_pullup_state_t pinPullUp);
    
io_status_t at32uc3l0256_ReadPin (
    const uint32_t channelAddress, const uint32_t pinAddress,
    io_pin_state_t* p_readState);

io_status_t at32uc3l0256_WritePin (
    const uint32_t channelAddress, const uint32_t pinAddress,
    const io_pin_state_t writeState);

io_status_t at32uc3l0256_TogglePin (
    const uint32_t channelAddress, const uint32_t pinAddress);

io_status_t at32uc3l0256_WritePullUp (
    const uint32_t channelAddress, const uint32_t pinAddress,
    const io_pullup_state_t writeState);

#endif /* IO_AT32UC3L0256_H_ */