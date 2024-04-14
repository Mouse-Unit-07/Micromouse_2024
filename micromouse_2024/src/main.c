/*-------------------------------- FILE INFO -----------------------------------
* Filename        : main.c
* Author          : Ryuichi Lin
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
    
//     int32_t time1 = 0u;
//     int32_t time2 = 0u;
    
    uint32_t oldEdgeCount = mhi_GetConfigPinEdgeCount();
    uint32_t newEdgeCount = mhi_GetConfigPinEdgeCount();
    mhi_DelayMs(5000);
    
    /* infinite while loop */
	
	
    while(1)
    {
        mhi_CheckLowBattery();
		
        mci_MoveForward1MazeSquarePid();
		mhi_DelayMs(1000);
//         time1 = mhi_GetTimerCount();
//         while ((time2 - time1) < 60)
//         {
//             time2 = mhi_GetTimerCount();
//         }
//         mhi_PrintString("toggled LEDs at count: ");
//         mhi_PrintInt((unsigned long)(time2-time1));
//         mhi_PrintString("\r\n");
        
        //mci_PrintWallSensorReadings();
        //mci_PrintWallPresence();
        
//         mci_MoveForward1Revolution();
// 		mhi_DelayMs(1000);//
        //mhi_DelayMs(3000);
        
//         newEdgeCount = mhi_GetConfigPinEdgeCount();
//         if (newEdgeCount != oldEdgeCount)
//         {
//             mhi_ToggleD1Led();
//             oldEdgeCount = newEdgeCount;
//         }
    }
}

