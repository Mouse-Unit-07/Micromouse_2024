/*-------------------------------- FILE INFO -----------------------------------
* Filename        : usart_mhi.h
* Author          : Ryuichi Lin
* Revision        : 1.0
* Updated         : 2024-04-07
* Purpose         : mouse hardware interface layer
*
* This is the header file for the mouse USART interface.
*
* The mouse hardware interface uses the HAL to define functions needed to
* interface w/ all mouse hardware.
*-----------------------------------------------------------------------------*/ 

#ifndef USART_MHI_H_
#define USART_MHI_H_

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
void mhi_InitUsart(void);
void mhi_PrintString(const char *p_userString);
void mhi_PrintInt(unsigned long userInt);

#endif /* USART_MHI_H_ */