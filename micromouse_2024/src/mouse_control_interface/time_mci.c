/*-------------------------------- FILE INFO -----------------------------------
* Filename        : time_mci.c
* Author          : Team Kirbo
* Revision        : 1.0
* Updated         : 2024-04-13
* Purpose         : mouses control interface layer
*
* This is the source file for mouse time under the mouse control
* interface.
*
* The mouse control interface uses the mouse hardware interface to define high
* level micromouse functionality.
*-----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*                               Include Files                                */
/*----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "mouse_hardware_interface/leds_mhi.h"
#include "mouse_hardware_interface/timer_mhi.h"
#include "mouse_hardware_interface/clock_mhi.h"
#include "mouse_control_interface/time_mci.h"

/*----------------------------------------------------------------------------*/
/*                                Definitions                                 */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                               Debug Switches                               */
/*----------------------------------------------------------------------------*/
/* 1 = Enable Debug Trace Output */
#define DEBUG_MCI_TIME_ENABLE    (1)

/*----------------------------------------------------------------------------*/
/*                              Global Variables                              */
/*----------------------------------------------------------------------------*/
int32_t startTime = 0;

/*----------------------------------------------------------------------------*/
/*                    Private (Static) Function Prototype                     */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                       Public (Exportable) Functions                        */
/*----------------------------------------------------------------------------*/
/**
* Start the timer and set start time variable
*
* \param None
* \retval None
*/
void mci_StartTimer(void)
{
    startTime = mhi_GetTimerCount();
}

/**
* Reset start time variable to current time*
* \param None
* \retval None
*/
void mci_ResetTimer(void)
{
    startTime = mhi_GetTimerCount();
}

/**
* Get time elapsed from start time
*
* \param None
* \retval None
*/
uint32_t mci_GetTimeMs(void)
{
    int32_t currentTime = mhi_GetTimerCount();
    uint32_t timeElapsed = (uint32_t)(currentTime - startTime);
    timeElapsed *= 8;
    
    return timeElapsed;
}

/**
* Wrapper for delay ms
*
* \param None
* \retval None
*/
void mci_DelayMs(const uint32_t delayTime)
{
    mhi_DelayMs(delayTime);
}

/**
* Wrapper for delay us
*
* \param None
* \retval None
*/
void mci_DelayUs(const uint32_t delayTime)
{
    mhi_DelayUs(delayTime);
}

/*----------------------------------------------------------------------------*/
/*                           Local Shared Functions                           */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                         Private (Static) Functions                         */
/*----------------------------------------------------------------------------*/
/* None */