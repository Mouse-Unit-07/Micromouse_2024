/*-------------------------------- FILE INFO -----------------------------------
* Filename        : usart_contract.h
* Author          : Ryuichi Lin
* Revision        : 1.0
* Updated         : 2024-03-31
* Purpose         : HAL contract layer
*
* This file contains the contract to abstract an USART interface.
*
* Contracts define features of interfaces so high level code can use abstract
* handlers instead of hardware specific code. HAL config files use these
* contracts to link to hardware specific code.
*
* Step 1 for hardware abstraction.
*-----------------------------------------------------------------------------*/

#ifndef USART_CONTRACT_H_
#define USART_CONTRACT_H_

/*----------------------------------------------------------------------------*/
/*                                Definitions                                 */
/*----------------------------------------------------------------------------*/
/* USART status enumeration */
typedef enum
{
    USART_SERIAL_SUCCESS = 0u,
    USART_SERIAL_ERROR
} usart_status_t;

/* USART interface contract- used to create handlers */
typedef struct
{
    usart_status_t (*usart_Init)(void);
    usart_status_t (*usart_PrintString)(const char *p_userString);
    usart_status_t (*usart_PrintInt)(unsigned long userInt);
} usart_handler_t;

/*----------------------------------------------------------------------------*/
/*                              Global Variables                              */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                       Public (Exportable) Functions                        */
/*----------------------------------------------------------------------------*/
/* None */

#endif /* USART_CONTRACT_H_ */