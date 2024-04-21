/*-------------------------------- FILE INFO -----------------------------------
* Filename        : time_mci.h
* Author          : Team Kirbo
* Revision        : 1.0
* Updated         : 2024-04-13
* Purpose         : mouse control interface layer
*
* This is the header file for mouse time under the mouse control
* interface.
*
* The mouse control interface uses the mouse hardware interface to define high
* level micromouse functionality.
*-----------------------------------------------------------------------------*/

#ifndef TIME_MCI_H_
#define TIME_MCI_H_

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
void mci_StartTimer(void);
void mci_ResetTimer(void);
uint32_t mci_GetTimeMs(void);
void mci_DelayMs(const uint32_t delayTime);
void mci_DelayUs(const uint32_t delayTime);

#endif /* TIME_MCI_H_ */