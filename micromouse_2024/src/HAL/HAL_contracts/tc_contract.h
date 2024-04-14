/*-------------------------------- FILE INFO -----------------------------------
* Filename        : tc_contract.h
* Author          : Ryuichi Lin
* Revision        : 1.0
* Updated         : 2024-03-31
* Purpose         : HAL contract layer
*
* This file contains the contract to abstract an TC interface.
*
* Contracts define features of interfaces so high level code can use abstract
* handlers instead of hardware specific code. HAL config files use these
* contracts to link to hardware specific code.
*
* Step 1 for hardware abstraction.
*-----------------------------------------------------------------------------*/

#ifndef TC_CONTRACT_H_
#define TC_CONTRACT_H_

/*----------------------------------------------------------------------------*/
/*                                Definitions                                 */
/*----------------------------------------------------------------------------*/
/* TC status enumeration */
typedef enum
{
    TC_SUCCESS = 0u,
    TC_ERROR
} tc_status_t;

/* TC interface contract- used to create handlers */
typedef struct
{
    tc_status_t (*tc_Init)(void);
    tc_status_t (*tc_GetTimerCounterCount)(uint32_t* p_timerCounterCount);
} tc_handler_t;

/*----------------------------------------------------------------------------*/
/*                              Global Variables                              */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                       Public (Exportable) Functions                        */
/*----------------------------------------------------------------------------*/
/* None */

#endif /* TC_CONTRACT_H_ */