/*-------------------------------- FILE INFO -----------------------------------
* Filename        : interrupts_mhi.h
* Author          : Team Kirbo
* Revision        : 1.0
* Updated         : 2024-04-07
* Purpose         : mouse hardware interface layer
*
* This is the header file for the mouse interrupt interface.
* This interface includes just the interrupt related functions that don't
* fit into the other interfaces.
*
* The mouse hardware interface uses the HAL to define functions needed to
* interface w/ all mouse hardware.
*-----------------------------------------------------------------------------*/

#ifndef INTERRUPTS_MHI_H_
#define INTERRUPTS_MHI_H_

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
void mhi_InitInterrupts(void);
void mhi_EnableGlobalInterrupts(void);
void mhi_DisableGlobalInterrupts(void);
uint32_t mhi_GetConfigPinEdgeCount(void);
uint32_t mhi_GetEncoder1EdgeCount(void);
uint32_t mhi_GetEncoder2EdgeCount(void);
void mhi_ClearConfigPinEdgeCount(void);
void mhi_ClearEncoder1EdgeCount(void);
void mhi_ClearEncoder2EdgeCount(void);

#endif /* INTERRUPTS_MHI_H_ */