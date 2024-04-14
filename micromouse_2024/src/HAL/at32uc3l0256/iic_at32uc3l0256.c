/*-------------------------------- FILE INFO -----------------------------------
* Filename        : iic_at32uc3l0256.c
* Author          : Ryuichi Lin
* Revision        : 1.0
* Updated         : 2024-03-31
* Purpose         : hardware specific layer
*
* This file is the header file for hardware specific "internal interrupt
* controller" (IIC) code.
*
* IIC refers to the software interrupt controller as opposed to EIC.
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
#include "HAL/HAL_contracts/iic_contract.h"
#include "HAL/at32uc3l0256/iic_at32uc3l0256.h"

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
* Initialize IIC interface for AT32UC3L0256 MCU.
*
* \retval IIC_SUCCESS Success
* \retval IIC_ERROR Failure: Failed to initialize internal interrupt controller
*/
iic_status_t at32uc3l0256_InitIic(void)
{ 
    iic_status_t iicStatus = IIC_ERROR;
    
    INTC_init_interrupts();
    
    iicStatus = IIC_SUCCESS;
    
    /* return status */
    return iicStatus;
}

/**
* Enable global interrupts for AT32UC3L0256 MCU.
*
* \retval IIC_SUCCESS Success
* \retval IIC_ERROR Failure: Failed to enable interrupts
*/
iic_status_t at32uc3l0256_EnableGlobalInterrupts(void)
{
    iic_status_t iicStatus = IIC_ERROR;
    
    cpu_irq_enable();
    
    iicStatus = IIC_SUCCESS;
     
    /* return status */
    return iicStatus;
}

/**
* Disable global interrupts for AT32UC3L0256 MCU.
*
* \retval IIC_SUCCESS Success
* \retval IIC_ERROR Failure: Failed to disable interrupts
*/
iic_status_t at32uc3l0256_DisableGlobalInterrupts(void)
{
    iic_status_t iicStatus = IIC_ERROR;
    
    cpu_irq_disable();
    
    iicStatus = IIC_SUCCESS;
    
    /* return status */
    return iicStatus;
}

/*----------------------------------------------------------------------------*/
/*                           Local Shared Functions                           */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                         Private (Static) Functions                         */
/*----------------------------------------------------------------------------*/
/* None */