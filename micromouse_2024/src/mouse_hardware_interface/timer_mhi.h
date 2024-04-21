/*-------------------------------- FILE INFO -----------------------------------
* Filename        : timer_mhi.h
* Author          : Team Kirbo
* Revision        : 1.0
* Updated         : 2024-04-07
* Purpose         : mouse hardware interface layer
*
* This is the header file for the mouse timer interface.
*
* The mouse hardware interface uses the HAL to define functions needed to
* interface w/ all mouse hardware.
*-----------------------------------------------------------------------------*/ 

#ifndef TIMER_MHI_H_
#define TIMER_MHI_H_

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
void mhi_InitTimerCounter(void);                /* init timer counter */
uint32_t mhi_GetTimerCount(void);               /* get timer counter count */
uint32_t mhi_TimerCountToMs(uint32_t count);    /* translate count to ms */

#endif /* TIMER_MHI_H_ */