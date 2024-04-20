/*-------------------------------- FILE INFO -----------------------------------
* Filename        : iic_contract.h
* Author          : Team Kirbo
* Revision        : 1.0
* Updated         : 2024-03-31
* Purpose         : HAL contract layer
*
* This file contains the contract to abstract the "internal interrupt
* controller" (IIC) interface. 
* 
* IIC refers to the software interrupt controller as opposed to EIC.
*
* Contracts define features of interfaces so high level code can use abstract
* handlers instead of hardware specific code. HAL config files use these
* contracts to link to hardware specific code.
*
* Step 1 for hardware abstraction.
*-----------------------------------------------------------------------------*/

#ifndef IIC_CONTRACT_H_
#define IIC_CONTRACT_H_

/*----------------------------------------------------------------------------*/
/*                                Definitions                                 */
/*----------------------------------------------------------------------------*/
/* IIC status enumeration */
typedef enum
{
    IIC_SUCCESS = 0u,
    IIC_ERROR,
} iic_status_t;

/* IIC interface contract- used to create handlers */
typedef struct
{
    iic_status_t (*iic_Init)(void);
    iic_status_t (*iic_EnableGlobalInterrupts)(void);
    iic_status_t (*iic_DisableGlobalInterrupts)(void);
} iic_handler_t;

/*----------------------------------------------------------------------------*/
/*                              Global Variables                              */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                       Public (Exportable) Functions                        */
/*----------------------------------------------------------------------------*/
/* None */

#endif /* IIC_CONTRACT_H_ */