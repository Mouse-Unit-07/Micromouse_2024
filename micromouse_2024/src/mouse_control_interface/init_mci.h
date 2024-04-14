/*-------------------------------- FILE INFO -----------------------------------
* Filename        : init_mci.h
* Author          : Ryuichi Lin
* Revision        : 1.0
* Updated         : 2024-04-13
* Purpose         : mouse control interface layer
*
* This is the header file for mouse initialization under the mouse control 
* interface.
* 
* The mouse control interface uses the mouse hardware interface to define high
* level micromouse functionality.
*-----------------------------------------------------------------------------*/

#ifndef INIT_MCI_H_
#define INIT_MCI_H_

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
void mci_InitializeMouse(void);

#endif /* INIT_MCI_H_ */