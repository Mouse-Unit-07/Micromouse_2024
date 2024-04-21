/*-------------------------------- FILE INFO -----------------------------------
* Filename        : clock_contract.h
* Author          : Team Kirbo
* Revision        : 1.0
* Updated         : 2024-03-31
* Purpose         : HAL contract layer
*
* This file contains the contract to abstract a clock interface.
*
* Contracts define features of interfaces so high level code can use abstract
* handlers instead of hardware specific code. HAL config files use these
* contracts to link to hardware specific code.
*
* Step 1 for hardware abstraction.
*-----------------------------------------------------------------------------*/

#ifndef CLOCK_CONTRACT_H_
#define CLOCK_CONTRACT_H_

/*----------------------------------------------------------------------------*/
/*                                Definitions                                 */
/*----------------------------------------------------------------------------*/
/* clock status enumeration */
typedef enum
{
    CLOCK_SUCCESS = 0u,
    CLOCK_ERROR,
} clock_status_t;

/* clock interface contract- used to create handlers */
typedef struct
{
    clock_status_t (*clock_InitDfll)(void);
    clock_status_t (*clock_InitDfllExternalOscillator)(void);
    clock_status_t (*clock_DelayMs)(const uint32_t delayTime);
    clock_status_t (*clock_DelayUs)(const uint32_t delayTime);
} clock_handler_t;

/*----------------------------------------------------------------------------*/
/*                              Global Variables                              */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                       Public (Exportable) Functions                        */
/*----------------------------------------------------------------------------*/
/* None */

#endif /* CLOCK_CONTRACT_H_ */