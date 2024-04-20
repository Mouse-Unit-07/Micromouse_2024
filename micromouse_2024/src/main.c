/*-------------------------------- FILE INFO -----------------------------------
* Filename        : main.c
* Author          : Team Kirbo
* Revision        : 1.0
* Updated         : 2024-04-07
* Purpose         : main layer
*
* This is the highest firmware layer containing the main source code to run 
* the micromouse.
*-----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*                               Include Files                                */
/*----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include "mouse_hardware_interface/leds_mhi.h"
#include "mouse_hardware_interface/clock_mhi.h"
#include "mouse_hardware_interface/timer_mhi.h"
#include "mouse_hardware_interface/interrupts_mhi.h"
#include "mouse_hardware_interface/usart_mhi.h"
#include "mouse_hardware_interface/power_mhi.h"
#include "mouse_hardware_interface/irsensors_mhi.h"
#include "mouse_hardware_interface/motors_mhi.h"

#include "mouse_control_interface/init_mci.h"
#include "mouse_control_interface/walldetection_mci.h"
#include "mouse_control_interface/movement_mci.h"

/*----------------------------------------------------------------------------*/
/*                                Definitions                                 */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                               Debug Switches                               */
/*----------------------------------------------------------------------------*/
#define DEBUG_MAIN_ENABLE    (0)    /* 1 = Enable Debug Trace Output */

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
/* None */

/*----------------------------------------------------------------------------*/
/*                           Local Shared Functions                           */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                         Private (Static) Functions                         */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*                                    Main                                    */
/*----------------------------------------------------------------------------*/
int main (void)
{   
    /* initialize mouse */
    mci_InitializeMouse();
    
    uint32_t oldEdgeCount = mhi_GetConfigPinEdgeCount();
    uint32_t newEdgeCount = mhi_GetConfigPinEdgeCount();
    mhi_DelayMs(3000);
    
    /* infinite while loop */
    while(1)
    {
        mhi_CheckLowBattery();
        
        /* right wall follower */
        mci_MoveForward1MazeSquarePid();
        mhi_DelayMs(80);
        
        if (mci_CheckLeftWall() == MCI_WALL_NOT_FOUND)
        {
            mci_TurnLeft90Degrees();
            mhi_DelayMs(100);
        }
        while (mci_CheckFrontWall() != MCI_WALL_NOT_FOUND)
        {
            mci_TurnRight90Degrees();
            mhi_DelayMs(100);
        }
    }
}

