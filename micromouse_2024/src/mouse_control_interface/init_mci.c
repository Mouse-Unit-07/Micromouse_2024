/*-------------------------------- FILE INFO -----------------------------------
* Filename        : init_mci.c
* Author          : Team Kirbo
* Revision        : 1.0
* Updated         : 2024-04-13
* Purpose         : mouses control interface layer
*
* This is the source file for mouse initialization under the mouse control
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
#include "mouse_hardware_interface/leds_mhi.h"
#include "mouse_hardware_interface/clock_mhi.h"
#include "mouse_hardware_interface/timer_mhi.h"
#include "mouse_hardware_interface/interrupts_mhi.h"
#include "mouse_hardware_interface/usart_mhi.h"
#include "mouse_hardware_interface/power_mhi.h"
#include "mouse_hardware_interface/irsensors_mhi.h"
#include "mouse_hardware_interface/motors_mhi.h"
#include "mouse_control_interface/init_mci.h"

/*----------------------------------------------------------------------------*/
/*                                Definitions                                 */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                               Debug Switches                               */
/*----------------------------------------------------------------------------*/
#define DEBUG_MCI_INIT_ENABLE    (1)    /* 1 = Enable Debug Trace Output */

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
* Mouse startup initialization function
*
* \param None
* \retval None
*/
void mci_InitializeMouse(void)
{
    /* disable global interrupts on entry */
    mhi_DisableGlobalInterrupts();
    
    /* initialize interrupts first */
    mhi_InitInterrupts();
    
    /* initialize all hardware interfaces */
    mhi_InitLeds();
    mhi_InitClock();
    mhi_InitTimerCounter();
    mhi_InitUsart();
    mhi_InitPower();
    mhi_InitAdc();
    mhi_InitPwm();
    mhi_InitWheelMotors();
    mhi_InitVacuumMotor();
    
    /* enable global interrupts */
    mhi_EnableGlobalInterrupts();
    
    /* enable power to mouse */
    mhi_EnableRegulators();
    
#if defined(DEBUG_MCI_INIT_ENABLE) && (DEBUG_MCI_INIT_ENABLE == 1)
    mhi_PrintString("Micromouse initialized\r\n");
#endif /* DEBUG_MCI_INIT_ENABLE */
}

/*----------------------------------------------------------------------------*/
/*                           Local Shared Functions                           */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                         Private (Static) Functions                         */
/*----------------------------------------------------------------------------*/
/* None */