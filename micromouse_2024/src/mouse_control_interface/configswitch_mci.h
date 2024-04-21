/*-------------------------------- FILE INFO -----------------------------------
* Filename        : configswitch_mci.h
* Author          : Team Kirbo
* Revision        : 1.0
* Updated         : 2024-04-20
* Purpose         : mouse control interface layer
*
* This is the header file for mouse configuration switch under the mouse 
* control interface.
*
* The mouse control interface uses the mouse hardware interface to define high
* level micromouse functionality.
*-----------------------------------------------------------------------------*/

#ifndef CONFIGSWITCH_MCI_H_
#define CONFIGSWITCH_MCI_H_

/*----------------------------------------------------------------------------*/
/*                                Definitions                                 */
/*----------------------------------------------------------------------------*/
typedef enum 
{
    MCI_BUTTON_NOT_PRESSED = 0u,
    MCI_BUTTON_PRESSED
} mci_button_pressed_t;

/*----------------------------------------------------------------------------*/
/*                              Global Variables                              */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                       Public (Exportable) Functions                        */
/*----------------------------------------------------------------------------*/
mci_button_pressed_t mci_CheckConfigButtonPressed(void);

#endif /* CONFIGSWITCH_MCI_H_ */