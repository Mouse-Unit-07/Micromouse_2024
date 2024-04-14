/*-------------------------------- FILE INFO -----------------------------------
* Filename        : iic_at32uc3l0256.h
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
*-----------------------------------------------------------------------------*/

#ifndef IIC_AT32UC3L0256_H_
#define IIC_AT32UC3L0256_H_

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
iic_status_t at32uc3l0256_InitIic(void);
iic_status_t at32uc3l0256_EnableGlobalInterrupts(void);
iic_status_t at32uc3l0256_DisableGlobalInterrupts(void);

#endif /* IIC_AT32UC3L0256_H_ */