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
#include "mouse_hardware_interface/power_mhi.h"

#include "mouse_control_interface/init_mci.h"
#include "mouse_control_interface/walldetection_mci.h"
#include "mouse_control_interface/movement_mci.h"
#include "mouse_control_interface/configswitch_mci.h"
#include "mouse_control_interface/time_mci.h"
#include "mouse_hardware_interface/clock_mhi.h"
#include "algo/algo.h"
#include "algo/wallfollower_algo.h"

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
    
    while(1)
    {
	    //mhi_CheckLowBattery();
	    
	    algoIterate();
	    if(mci_CheckFrontWall()){
		    mci_AdjustToFrontWall();
	    }
	    
    }
}

