/*-------------------------------- FILE INFO -----------------------------------
* Filename        : configswitch_mci.c
* Author          : Team Kirbo
* Revision        : 1.0
* Updated         : 2024-04-13
* Purpose         : mouses control interface layer
*
* This is the header file for mouse configuration switch under the mouse 
* control interface.
*
* The mouse control interface uses the mouse hardware interface to define high
* level micromouse functionality.
*-----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*                               Include Files                                */
/*----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "micromouse_dimensions.h"
#include "mouse_hardware_interface/leds_mhi.h"
#include "mouse_hardware_interface/interrupts_mhi.h"
#include "mouse_control_interface/configswitch_mci.h"

/*----------------------------------------------------------------------------*/
/*                                Definitions                                 */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                               Debug Switches                               */
/*----------------------------------------------------------------------------*/
/* 1 = Enable Debug Trace Output */
#define DEBUG_MCI_CONFIG_SWITCH_ENABLE    (1)

/*----------------------------------------------------------------------------*/
/*                              Global Variables                              */
/*----------------------------------------------------------------------------*/
static uint32_t oldEdgeCount = 0u;
static uint32_t newEdgeCount = 0u;

/*----------------------------------------------------------------------------*/
/*                    Private (Static) Function Prototype                     */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                       Public (Exportable) Functions                        */
/*----------------------------------------------------------------------------*/
/**
* Check whether the config button has been pressed
*
* \param None
* \retval None
*/
mci_button_pressed_t mci_CheckConfigButtonPressed(void)
{
    mci_button_pressed_t buttonPressed = MCI_BUTTON_NOT_PRESSED;
    
    newEdgeCount = mhi_GetConfigPinEdgeCount();
    if (newEdgeCount != oldEdgeCount)
    {
        buttonPressed = MCI_BUTTON_PRESSED;
        oldEdgeCount = newEdgeCount;
    }
    else
    {
        buttonPressed = MCI_BUTTON_NOT_PRESSED;
    }
    
    return buttonPressed;
}

/*----------------------------------------------------------------------------*/
/*                           Local Shared Functions                           */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                         Private (Static) Functions                         */
/*----------------------------------------------------------------------------*/
/* None */
