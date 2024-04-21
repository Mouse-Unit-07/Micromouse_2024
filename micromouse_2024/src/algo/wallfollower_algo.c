/*-------------------------------- FILE INFO -----------------------------------
* Filename        : wallfollower_algo.c
* Author          : Team Kirbo
* Revision        : 1.0
* Updated         : 2024-04-13
* Purpose         : mouses control interface layer
*
* This is the source file for mouse wall follower algorithm.
*
* The algo layer defines all mouse maze solving algorithms as the second most
* highest layer to be called directly in main.
*-----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*                               Include Files                                */
/*----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include "micromouse_dimensions.h"
#include "mouse_hardware_interface/leds_mhi.h"
#include "mouse_hardware_interface/clock_mhi.h"
#include "mouse_control_interface/walldetection_mci.h"
#include "mouse_control_interface/movement_mci.h"
#include "algo/wallfollower_algo.h"

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
/* None */

/*----------------------------------------------------------------------------*/
/*                    Private (Static) Function Prototype                     */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                       Public (Exportable) Functions                        */
/*----------------------------------------------------------------------------*/
/**
* Left wall following algorithm
*
* \param None
* \retval None
*/
void algo_LeftWallFollow()
{
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

/**
* Right wall following algorithm
*
* \param None
* \retval None
*/
void algo_RightWallFollow()
{
    mci_MoveForward1MazeSquarePid();
    mhi_DelayMs(80);
    
    if (mci_CheckRightWall() == MCI_WALL_NOT_FOUND)
    {
        mci_TurnRight90Degrees();
        mhi_DelayMs(100);
    }
    while (mci_CheckFrontWall() != MCI_WALL_NOT_FOUND)
    {
        mci_TurnLeft90Degrees();
        mhi_DelayMs(100);
    }
}

/*----------------------------------------------------------------------------*/
/*                           Local Shared Functions                           */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                         Private (Static) Functions                         */
/*----------------------------------------------------------------------------*/
/* None */