/*-------------------------------- FILE INFO -----------------------------------
* Filename        : eic_contract.h
* Author          : Team Kirbo
* Revision        : 1.0
* Updated         : 2024-03-31
* Purpose         : HAL contract layer
*
* This file contains the contract to abstract an external interrupt controller 
* (eic) interface.
*
* Contracts define features of interfaces so high level code can use abstract
* handlers instead of hardware specific code. HAL config files use these
* contracts to link to hardware specific code.
*
* Step 1 for hardware abstraction.
*-----------------------------------------------------------------------------*/

#ifndef EIC_CONTRACT_H_
#define EIC_CONTRACT_H_

/*----------------------------------------------------------------------------*/
/*                                Definitions                                 */
/*----------------------------------------------------------------------------*/
/* EIC status enumeration */
typedef enum
{
    EIC_SUCCESS = 0u,
    EIC_ERROR,
} eic_status_t;

/* EIC interface contract- used to create handlers */
typedef struct
{
    eic_status_t (*eic_InitEncoderInterrupts)(void); 
    eic_status_t (*eic_InitConfigPinInterrupt)(void);
    eic_status_t (*eic_GetEncoder1EdgeCount)(uint32_t *p_edgeCount);
    eic_status_t (*eic_GetEncoder2EdgeCount)(uint32_t *p_edgeCount);
    eic_status_t (*eic_GetConfigPinEdgeCount)(uint32_t *p_edgeCount);
    eic_status_t (*eic_ClearEncoder1EdgeCount)(void);
    eic_status_t (*eic_ClearEncoder2EdgeCount)(void);
    eic_status_t (*eic_ClearConfigPinEdgeCount)(void);
} eic_handler_t;

/*----------------------------------------------------------------------------*/
/*                              Global Variables                              */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                       Public (Exportable) Functions                        */
/*----------------------------------------------------------------------------*/
/* None */

#endif /* EIC_CONTRACT_H_ */