/*-------------------------------- FILE INFO -----------------------------------
* Filename        : walldetection_mci.c
* Author          : Ryuichi Lin
* Revision        : 1.0
* Updated         : 2024-04-13
* Purpose         : mouses control interface layer
*
* This is the header file for mouse wall detection under the mouse control
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
#include <math.h>
#include "micromouse_dimensions.h"
#include "mouse_hardware_interface/leds_mhi.h"
#include "mouse_hardware_interface/usart_mhi.h"
#include "mouse_hardware_interface/irsensors_mhi.h"
#include "mouse_control_interface/walldetection_mci.h"

/*----------------------------------------------------------------------------*/
/*                                Definitions                                 */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                               Debug Switches                               */
/*----------------------------------------------------------------------------*/
/* 1 = Enable Debug Trace Output */
#define DEBUG_MCI_WALL_DETECTION_ENABLE    (1)

/*----------------------------------------------------------------------------*/
/*                              Global Variables                              */
/*----------------------------------------------------------------------------*/
static mci_wall_presence_t frontWallPresence = MCI_WALL_NOT_FOUND;
static mci_wall_presence_t leftWallPresence = MCI_WALL_NOT_FOUND;
static mci_wall_presence_t rightWallPresence = MCI_WALL_NOT_FOUND;

static mci_wall_check_availability_t frontWallCheckAvailable 
    = MCI_WALL_CHECK_NOT_AVAILABLE;
static mci_wall_check_availability_t leftWallCheckAvailable 
    = MCI_WALL_CHECK_NOT_AVAILABLE;
static mci_wall_check_availability_t rightWallCheckAvailable 
    = MCI_WALL_CHECK_NOT_AVAILABLE;

/*----------------------------------------------------------------------------*/
/*                    Private (Static) Function Prototype                     */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                       Public (Exportable) Functions                        */
/*----------------------------------------------------------------------------*/
/**
* Print all mouse wall detection sensor readings
*
* Made for debug purposes
* 
* \param None
* \retval None
*/
void mci_PrintWallSensorReadings(void)
{
    uint32_t reading1 = 0u;
    uint32_t reading2 = 0u;
    uint32_t reading3 = 0u;
    uint32_t reading4 = 0u;
    
    reading1 = mhi_ReadIr1();
    reading2 = mhi_ReadIr2();
    reading3 = mhi_ReadIr3();
    reading4 = mhi_ReadIr4();
    
    mhi_PrintInt(reading1);
    mhi_PrintString(" ");
    
    mhi_PrintInt(reading2);
    mhi_PrintString(" ");
    
    mhi_PrintInt(reading3);
    mhi_PrintString(" ");
    
    mhi_PrintInt(reading4);
    mhi_PrintString("\r\n");
}

/**
* Print all mouse wall presence values
* 
* Made for debug purposes
*
* \param None
* \retval None
*/
void mci_PrintWallPresence(void)
{
    mci_SetFrontWallCheckAvailable();
    mci_SetLeftWallCheckAvailable();
    mci_SetRightWallCheckAvailable();
    mci_UpdateFrontWallPresence();
    mci_UpdateLeftWallPresence();
    mci_UpdateRightWallPresence();
    
    /* check front */
    if (mci_CheckFrontWall() == MCI_WALL_FOUND)
    {
        mhi_SetD1Led();
        mhi_PrintString("FRONT | ");
    }
    else
    {
        mhi_ClearD1Led();
        mhi_PrintString("NO FRONT | ");
    }
    
    /* check left */
    if (mci_CheckLeftWall() == MCI_WALL_FOUND)
    {
        mhi_SetD2Led();
        mhi_PrintString("LEFT | ");
    }
    else
    {
        mhi_ClearD2Led();
        mhi_PrintString("NO LEFT | ");
    }
    
    /* check right */
    if (mci_CheckRightWall() == MCI_WALL_FOUND)
    {
        mhi_SetD3Led();
        mhi_PrintString("RIGHT | ");
    }
    else
    {
        mhi_ClearD3Led();
        mhi_PrintString("NO RIGHT | ");
    }
    
    mhi_PrintString("\r\n");
    
    mhi_PrintInt((uint32_t)MCI_FRONT_SENSOR_READING_THRESHOLD_MM);
    mhi_PrintString(" ");
    mhi_PrintInt((uint32_t)MCI_LEFT_SENSOR_READING_THRESHOLD_MM);
    mhi_PrintString(" ");
    mhi_PrintInt((uint32_t)MCI_RIGHT_SENSOR_READING_THRESHOLD_MM);
    mhi_PrintString("\r\n");
    
    mhi_PrintInt((uint32_t)MCI_FRONT_SENSOR_READING_THRESHOLD_RAW);
    mhi_PrintString(" ");
    mhi_PrintInt((uint32_t)MCI_LEFT_SENSOR_READING_THRESHOLD_RAW);
    mhi_PrintString(" ");
    mhi_PrintInt((uint32_t)MCI_RIGHT_SENSOR_READING_THRESHOLD_RAW);
    mhi_PrintString("\r\n");
}

/**
* Check for front wall presence
*
* \param None
* \retval MCI_CANNOT_READ_WALL Front wall variable is outdated
* \retval MCI_WALL_NOT_FOUND There is no front wall
* \retval MCI_WALL_FOUND There is a front wall
*/
mci_wall_presence_t mci_CheckFrontWall(void)
{
    if (frontWallCheckAvailable == MCI_WALL_CHECK_AVAILABLE)
        return frontWallPresence;
    else
        return MCI_CANNOT_READ_WALL;
}

/**
* Check for left wall presence
*
* \param None
* \retval MCI_CANNOT_READ_WALL Left wall variable is outdated
* \retval MCI_WALL_NOT_FOUND There is no left wall
* \retval MCI_WALL_FOUND There is a left wall
*/
mci_wall_presence_t mci_CheckLeftWall(void)
{
    if (leftWallCheckAvailable == MCI_WALL_CHECK_AVAILABLE)
        return leftWallPresence;
    else
        return MCI_CANNOT_READ_WALL;
}

/**
* Check for right wall presence
*
* \param None
* \retval MCI_CANNOT_READ_WALL Right wall variable is outdated
* \retval MCI_WALL_NOT_FOUND There is no right wall
* \retval MCI_WALL_FOUND There is a right wall
*/
mci_wall_presence_t mci_CheckRightWall(void)
{
    if (rightWallCheckAvailable == MCI_WALL_CHECK_AVAILABLE)
        return rightWallPresence;
    else
        return MCI_CANNOT_READ_WALL;
}

/**
* Prevent checks for whether front wall is present
*
* \param None
* \retval None
*/
void mci_SetFrontWallCheckUnavailable(void)
{
    frontWallCheckAvailable = MCI_WALL_CHECK_NOT_AVAILABLE;
}

/**
* Prevent checks for whether left wall is present
*
* \param None
* \retval None
*/
void mci_SetLeftWallCheckUnavailable(void)
{
    leftWallCheckAvailable = MCI_WALL_CHECK_NOT_AVAILABLE;
}

/**
* Prevent checks for whether right wall is present
*
* \param None
* \retval None
*/
void mci_SetRightWallCheckUnavailable(void)
{
    rightWallCheckAvailable = MCI_WALL_CHECK_NOT_AVAILABLE;
}

/**
* Allow checks for whether front wall is present
*
* \param None
* \retval None
*/
void mci_SetFrontWallCheckAvailable(void)
{
    frontWallCheckAvailable = MCI_WALL_CHECK_AVAILABLE;
}

/**
* Allow checks for whether left wall is present
*
* \param None
* \retval None
*/
void mci_SetLeftWallCheckAvailable(void)
{
    leftWallCheckAvailable = MCI_WALL_CHECK_AVAILABLE;
}

/**
* Allow checks for whether right wall is present
*
* \param None
* \retval None
*/
void mci_SetRightWallCheckAvailable(void)
{
    rightWallCheckAvailable = MCI_WALL_CHECK_AVAILABLE;
}

/**
* Update front wall presence variable
*
* \param None
* \retval None
*/
void mci_UpdateFrontWallPresence(void)
{
    uint32_t reading1 = 0u;
    uint32_t reading2 = 0u;
    
    /* read front IR sensors */
    reading1 = mhi_ReadIr1();
    reading2 = mhi_ReadIr4();
        
    /* update front wall presence variable */
    if ((reading1 >= MCI_FRONT_SENSOR_READING_THRESHOLD_RAW) && 
         (reading2 >= MCI_FRONT_SENSOR_READING_THRESHOLD_RAW))
    {
        frontWallPresence = MCI_WALL_FOUND;
    }
    else 
    {
        frontWallPresence = MCI_WALL_NOT_FOUND;
    }
}

/**
* Update left wall presence variable
*
* \param None
* \retval None
*/
void mci_UpdateLeftWallPresence(void)
{
    uint32_t reading = 0u;
    
    /* read left IR sensor */
    reading = mhi_ReadIr2();
    
    /* update front wall presence variable */
    if (reading >= MCI_LEFT_SENSOR_READING_THRESHOLD_RAW)        
    {
        leftWallPresence = MCI_WALL_FOUND;
    }
    else
    {
        leftWallPresence = MCI_WALL_NOT_FOUND;
    }
}

/**
* Update right wall presence variable
*
* \param None
* \retval None
*/
void mci_UpdateRightWallPresence(void)
{
    uint32_t reading = 0u;
    
    /* read left IR sensor */
    reading = mhi_ReadIr3();
    
    /* update front wall presence variable */
    if (reading >= MCI_RIGHT_SENSOR_READING_THRESHOLD_RAW)
    {
        rightWallPresence = MCI_WALL_FOUND;
    }
    else
    {
        rightWallPresence = MCI_WALL_NOT_FOUND;
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
