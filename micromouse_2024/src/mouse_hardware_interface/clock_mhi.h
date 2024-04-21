/*-------------------------------- FILE INFO -----------------------------------
* Filename        : clock_mhi.h
* Author          : Team Kirbo
* Revision        : 1.0
* Updated         : 2024-04-07
* Purpose         : mouse hardware interface layer
*
* This is the header file for the mouse clock interface.
*
* The mouse hardware interface uses the HAL to define functions needed to
* interface w/ all mouse hardware.
*-----------------------------------------------------------------------------*/

#ifndef CLOCK_MHI_H_
#define CLOCK_MHI_H_

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
void mhi_InitClock(void);                      /* init clock sources */
void mhi_DelayMs(const uint32_t delayTime);    /* delay ms */
void mhi_DelayUs(const uint32_t delayTime);    /* delay us */

#endif /* CLOCK_MHI_H_ */