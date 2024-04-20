/*-------------------------------- FILE INFO -----------------------------------
* Filename        : movement_mci.c
* Author          : Ryuichi Lin
* Revision        : 1.0
* Updated         : 2024-04-13
* Purpose         : mouses control interface layer
*
* This is the header file for mouse movement under the mouse control
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
#include "micromouse_dimensions.h"
#include "shared_functions/constrain_sf.h"
#include "mouse_hardware_interface/leds_mhi.h"
#include "mouse_hardware_interface/usart_mhi.h"
#include "mouse_hardware_interface/irsensors_mhi.h"
#include "mouse_control_interface/walldetection_mci.h"
#include "mouse_hardware_interface/motors_mhi.h"
#include "mouse_hardware_interface/interrupts_mhi.h"
#include "mouse_control_interface/movement_mci.h"
#include "mouse_control_interface/walldetection_mci.h"

/*----------------------------------------------------------------------------*/
/*                                Definitions                                 */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                               Debug Switches                               */
/*----------------------------------------------------------------------------*/
/* 1 = Enable Debug Trace Output */
#define DEBUG_MCI_MOVEMENT_ENABLE    (1)

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
* Move mouse 1 wheel revolution
*
* Made for debugging purposes
*
* \param None
* \retval None
*/
void mci_MoveForward1Revolution(void)
{
    uint32_t rightDone = 0u;
    uint32_t leftDone = 0u;
    
    /* reset encoder counts */
    mhi_ClearEncoder1EdgeCount();
    mhi_ClearEncoder2EdgeCount();
    
    /* set speed and start in forward direction */
    mhi_SetWheelMotor1Speed(MCI_MINIMUM_SPEED);
    mhi_StartWheelMotor1Forward();
    mhi_SetWheelMotor2Speed(MCI_MINIMUM_SPEED);
    mhi_StartWheelMotor2Forward();
    
    /* keep moving motors until encoder edge counts are reached */
    while((!rightDone) || (!leftDone))
    {
        /* right motor */
        if ((!rightDone) && (mhi_GetEncoder1EdgeCount() >= MCI_WHEEL_MOTOR_EDGES_PER_REVOLUTION))
        {
            mhi_StopWheelMotor1();
            rightDone = 1u;
            mhi_PrintString("stopped motor 1: ");
            mhi_PrintInt(mhi_GetEncoder1EdgeCount());
            mhi_PrintString("\r\n");
        }
        else if (mhi_GetEncoder1EdgeCount() >= (MCI_WHEEL_MOTOR_EDGES_PER_REVOLUTION / 3))
        {
            mhi_SetWheelMotor1Speed(MCI_FORWARD_FAST_SPEED);
        }
        else if (mhi_GetEncoder1EdgeCount() >= ((MCI_WHEEL_MOTOR_EDGES_PER_REVOLUTION / 3) * 2))
        {
            mhi_SetWheelMotor1Speed(MCI_MINIMUM_SPEED);
        }
        
        /* left motor */
        if ((!leftDone) && (mhi_GetEncoder2EdgeCount() >= MCI_WHEEL_MOTOR_EDGES_PER_REVOLUTION))
        {
            mhi_StopWheelMotor2();
            leftDone = 1u;
            mhi_PrintString("stopped motor 2: ");
            mhi_PrintInt(mhi_GetEncoder2EdgeCount());
            mhi_PrintString("\r\n");
        }
        else if (mhi_GetEncoder2EdgeCount() >= (MCI_WHEEL_MOTOR_EDGES_PER_REVOLUTION / 3))
        {
            mhi_SetWheelMotor2Speed(MCI_FORWARD_FAST_SPEED);
        }
        else if (mhi_GetEncoder2EdgeCount() >= ((MCI_WHEEL_MOTOR_EDGES_PER_REVOLUTION / 3) * 2))
        {
            mhi_SetWheelMotor2Speed(MCI_MINIMUM_SPEED);
        }
    }
    
    mhi_PrintString("final: ");
    mhi_PrintInt(mhi_GetEncoder1EdgeCount());
    mhi_PrintString(" ");
    mhi_PrintInt(mhi_GetEncoder2EdgeCount());
    mhi_PrintString("\r\n");
    
    /* clear encoder edge counts and set motor speeds to 0 */
    mhi_ClearEncoder1EdgeCount();
    mhi_ClearEncoder2EdgeCount();
    mhi_SetWheelMotor1Speed(0);
    mhi_SetWheelMotor2Speed(0);
}


/**
* Move mouse 1 maze square forward using PID
*
* \param None
* \retval None
*/
void mci_MoveForward1MazeSquarePid(void)
{
    /* initialize encoder PID constants (integral term not needed) */
    float kp = 2;      /* proportional term */
    float kd = 0.2;    /* derivative term */
    
    /* initialize sensor PD constants */
    float kpSensor = 0.1;
    float kdSensor = 0.01;
    
    /* initialize left sensor PD constants */
    float kpSensorL = 0.5; // was 5
    float kdSensorL = 5;
    
    /* initialize right sensor PD constants */
    float kpSensorR = 0.5; // was 5
    float kdSensorR = 5;
    
    /* other encoder PID variables */
    int32_t initialPosition = mhi_GetEncoder1EdgeCount() + mhi_GetEncoder2EdgeCount();
    int32_t targetPosition = initialPosition + (MCI_WHEEL_MOTOR_EDGES_PER_MAZE_SQUARE * 2);
    int32_t targetAngle = 0;
    int32_t prevError = 0;
    float error = 0;
    float dError = 0;
    
    /* other sensor PD variables */
    int32_t output = 0;
    int32_t outputSensors = 0;
    int32_t prevErrorSensors = 0;
    int32_t errorSensors = 0;
    float dErrorSensors = 0;
    
    /* other left sensor PD variables */
    float errorSensorLeft = 0;
    
    /* other right sensor PD variables */
    float errorSensorRight = 0;
    
    /* new motor speeds provided all PDs */
    int32_t newLeftSpeed = 0;
    int32_t newRightSpeed = 0;
    
    /* IR sensor variables */
    uint32_t ir2Reading = 0u;
    uint32_t ir3Reading = 0u;
    
    /* local wall presence variables */
    mci_wall_presence_t leftWall = MCI_CANNOT_READ_WALL;
    mci_wall_presence_t rightWall = MCI_CANNOT_READ_WALL;
    
    /* configure both motors to move forward at base speed */
    mhi_SetWheelMotor1Speed(MCI_FORWARD_FAST_SPEED);
    mhi_SetWheelMotor2Speed(MCI_FORWARD_FAST_SPEED);
    mhi_StartWheelMotor1Forward();
    mhi_StartWheelMotor2Forward();
    
    /* allow wall updates on start */
    mci_SetLeftWallUpdateAvailable();
    mci_SetRightWallUpdateAvailable();
    
    /* set up target and initial position */
    mhi_ClearEncoder1EdgeCount();
    mhi_ClearEncoder2EdgeCount();
    initialPosition = mhi_GetEncoder1EdgeCount() + mhi_GetEncoder2EdgeCount();
    targetPosition = initialPosition + (MCI_WHEEL_MOTOR_EDGES_PER_MAZE_SQUARE * 2);
    
    
    /* main control loop */
    while ((mhi_GetEncoder1EdgeCount() + mhi_GetEncoder2EdgeCount()) < targetPosition)
    {
        /* stop if there's a wall in front */
        if (((mhi_ReadIr1() + mhi_ReadIr4()) / 2) >= MCI_FRONT_WALL_TOO_CLOSE_THRESHOLD_RAW_30MM_HARD_CODED)
        {
            break;
        }
        
        /* prevent wall updates if moved more than 30% */
        if ( (mhi_GetEncoder1EdgeCount() + mhi_GetEncoder2EdgeCount()) > ((uint32_t)(targetPosition / 2)) )
        {
            mci_SetLeftWallUpdateUnavailable();
            mci_SetRightWallUpdateUnavailable();
        }
        
        /* try to update global wall presences- no need to save return value */
        mci_CheckLeftWall();
        mci_CheckRightWall();
        
        /* check for walls for PID move forward */
        leftWall = mci_CheckLeftWallMoveForwardPid();
        rightWall = mci_CheckRightWallMoveForwardPid();
        
        //output error sensors
        if ((leftWall == MCI_WALL_FOUND) && (rightWall == MCI_WALL_FOUND))
        {   
            mhi_ClearD2Led();
            mhi_ClearD3Led();
            mhi_SetD1Led();
            
            /* read left wall sensor */
            ir2Reading = mhi_ReadIr2();
            
            /* read right wall sensor */
            ir3Reading = mhi_ReadIr3();
            
            // Error with the sensors
            errorSensorLeft =  MCI_LEFT_SENSOR_READING_THRESHOLD_RAW - ir2Reading;
            errorSensorRight = MCI_RIGHT_SENSOR_READING_THRESHOLD_RAW - ir3Reading;
            errorSensors =  errorSensorRight - errorSensorLeft;
            dErrorSensors = errorSensors - prevErrorSensors;
            
            // Current way of trying to add a pid controller to the mouse using the sensors going to aggregate with the other output
            outputSensors = (kpSensor * errorSensors) + (kdSensor * dErrorSensors);
            
            prevErrorSensors = errorSensors;
        }
        
        else if ((leftWall == MCI_WALL_FOUND) && (rightWall != MCI_WALL_FOUND)) 
        {   
            mhi_ClearD1Led();
            mhi_ClearD3Led();
            mhi_SetD2Led();
            
            //Read left one
            ir2Reading = mhi_ReadIr2();
             
            //error with just left sensor
            errorSensorLeft = MCI_LEFT_SENSOR_READING_THRESHOLD_RAW - ir2Reading;
            //errorSensors = -errorSensorLeft * 2;
            errorSensors = -errorSensorLeft;
            dErrorSensors = errorSensors - prevErrorSensors;
            outputSensors = (kpSensorL * errorSensors) + (kdSensorL * dErrorSensors);
            // 			 if(targetAngle - (g_sbu_mm_Motor1EncoderEdgeCount - g_sbu_mm_Motor2EncoderEdgeCount) < -45){
            // 				outputSensors*=-1;
            // 			 }
            prevErrorSensors = errorSensors;
        }
         
        else if ((rightWall == MCI_WALL_FOUND) && (leftWall != MCI_WALL_FOUND))
        {
            mhi_ClearD1Led();
            mhi_ClearD2Led();
            mhi_SetD3Led();
            
            //Read Right one
            ir3Reading = mhi_ReadIr3();
             
            errorSensorRight = MCI_RIGHT_SENSOR_READING_THRESHOLD_RAW - ir3Reading;
            //errorSensors = errorSensorRight * 2;
            errorSensors = errorSensorRight;
            dErrorSensors = errorSensors - prevErrorSensors;
            outputSensors = (kpSensorR * errorSensors) + (kdSensorR * dErrorSensors);
            // 			  if(targetAngle - (g_sbu_mm_Motor1EncoderEdgeCount - g_sbu_mm_Motor2EncoderEdgeCount) < 45){
            // 				  outputSensors*=-1;
            // 			  }
            prevErrorSensors = errorSensors;
        }
        
        else 
        {
            outputSensors = 0;
            prevErrorSensors = 0;
        }
        
        /* calculate error (for proportional term) */
        error = targetAngle - ((int32_t)mhi_GetEncoder1EdgeCount() - (int32_t)mhi_GetEncoder2EdgeCount());
        
        /* calculate difference in error (for the derivative term) */
		error += outputSensors;
        dError = error - prevError;
        prevError = error;
        
        /* calculate PID output */
        output = (kp * error) + (kd * dError);
        
        /* calculate new speeds */
        newLeftSpeed = sf_constrain(MCI_MINIMUM_SPEED + output, 255, -255);
        newRightSpeed = sf_constrain(MCI_MINIMUM_SPEED - output, 255, -255);
        
        
        /* set new motor speeds */
        if (newLeftSpeed < 0)
        {
            mhi_SetWheelMotor1Speed((uint16_t)abs(newLeftSpeed));
            mhi_StartWheelMotor1Backward();
        }
        else
        {
            mhi_SetWheelMotor1Speed((uint16_t)newLeftSpeed);
            mhi_StartWheelMotor1Forward();
        }
        
        if (newRightSpeed < 0)
        {
            mhi_SetWheelMotor2Speed((uint16_t)abs(newRightSpeed));
            mhi_StartWheelMotor2Backward();
        }
        else
        {
            mhi_SetWheelMotor2Speed((uint16_t)newRightSpeed);
            mhi_StartWheelMotor2Forward();
        }
    }
    
    /* clear encoder edge counts and set motor speeds to 0 */
    mhi_StopWheelMotor1();
    mhi_StopWheelMotor2();
    mhi_ClearEncoder1EdgeCount();
    mhi_ClearEncoder2EdgeCount();    
}

/**
* Rotate mouse 90 degrees right
*
* \param None
* \retval None
*/
void mci_TurnRight90Degrees(void)
{
    uint32_t rightDone = 0u;
    uint32_t leftDone = 0u;
    
    /* reset encoder counts */
    mhi_ClearEncoder1EdgeCount();
    mhi_ClearEncoder2EdgeCount();
    
    /* set motor directions and speeds */
    mhi_SetWheelMotor1Speed(MCI_TURN_SPEED);
    mhi_SetWheelMotor2Speed(MCI_TURN_SPEED);
    mhi_StartWheelMotor1Forward();
    mhi_StartWheelMotor2Backward();
    
    /* keep moving motors until encoder edge counts are reached */
    while ((!rightDone) || (!leftDone))
    {      
        if ((!rightDone) && (mhi_GetEncoder1EdgeCount() == 
            MCI_WHEEL_MOTOR_EDGES_PER_90_DEGREE_TURN_RIGHT))
        {
            mhi_StopWheelMotor1();
            rightDone = 1u;
        }
        if ((!leftDone) && (mhi_GetEncoder2EdgeCount() == 
            (-MCI_WHEEL_MOTOR_EDGES_PER_90_DEGREE_TURN_RIGHT)))
        {
            mhi_StopWheelMotor2();
            leftDone = 1u;
        }
    }
    
    /* update wall presences */
    mci_UpdateWallPresenceRightTurn();
    
    
    /* clear encoder edge counts and set motor speeds to 0 */
    mhi_StopWheelMotor1();
    mhi_StopWheelMotor2();
    mhi_ClearEncoder1EdgeCount();
    mhi_ClearEncoder2EdgeCount();
}

/**
* Rotate mouse 90 degrees left
*
* \param None
* \retval None
*/
void mci_TurnLeft90Degrees(void)
{
    uint32_t rightDone = 0u;
    uint32_t leftDone = 0u;
    
    /* reset encoder counts */
    mhi_ClearEncoder1EdgeCount();
    mhi_ClearEncoder2EdgeCount();
    
    /* set motor directions and speeds */
    mhi_SetWheelMotor1Speed(MCI_TURN_SPEED);
    mhi_SetWheelMotor2Speed(MCI_TURN_SPEED);
    mhi_StartWheelMotor1Backward();
    mhi_StartWheelMotor2Forward();
    
    /* keep moving motors until encoder edge counts are reached */
    while ((!rightDone) || (!leftDone))
    {    
        if ((!rightDone) && (mhi_GetEncoder1EdgeCount() == (-MCI_WHEEL_MOTOR_EDGES_PER_90_DEGREE_TURN_LEFT)))
        {
            mhi_StopWheelMotor1();
            rightDone = 1u;
        }
        if ((!leftDone) && (mhi_GetEncoder2EdgeCount() == MCI_WHEEL_MOTOR_EDGES_PER_90_DEGREE_TURN_LEFT))
        {
            mhi_StopWheelMotor2();
            leftDone = 1u;
        }
    }
    
    /* update wall presences */
    mci_UpdateWallPresenceLeftTurn();
    
    /* clear encoder edge counts and set motor speeds to 0 */
    mhi_StopWheelMotor1();
    mhi_StopWheelMotor2();
    mhi_ClearEncoder1EdgeCount();
    mhi_ClearEncoder2EdgeCount();
}

/**
* Adjust mouse to front wall
*
* \param None
* \retval None
*/
void mci_AdjustToFrontWall(void)
{
    uint32_t ir1Reading = 0u;
    uint32_t ir4Reading = 0u;
    
    /* read both sensors */
    ir1Reading = mhi_ReadIr1();
    ir4Reading = mhi_ReadIr4();
    
    while (abs(ir1Reading - ir4Reading) > 2)
    {
        if (ir1Reading > ir4Reading)
        {
            mhi_StartWheelMotor1Backward();
            mhi_StartWheelMotor2Forward();
            mhi_SetWheelMotor1Speed(100);
            mhi_SetWheelMotor2Speed(100);
            
            while (ir1Reading >= ir4Reading)
            {
                /* read both sensors */
                ir1Reading = mhi_ReadIr1();
                ir4Reading = mhi_ReadIr4();
            }
        }
        else if (ir1Reading < ir4Reading)
        {
            mhi_StartWheelMotor1Forward();
            mhi_StartWheelMotor2Backward();
            mhi_SetWheelMotor1Speed(100);
            mhi_SetWheelMotor2Speed(100);
            
            while (ir1Reading <= ir4Reading)
            {
                /* read both sensors */
                ir1Reading = mhi_ReadIr1();
                ir4Reading = mhi_ReadIr4();
            }
        }
    }
    
    /* clear encoder edge counts and set motor speeds to 0 */
    mhi_ClearEncoder1EdgeCount();
    mhi_ClearEncoder2EdgeCount();
    mhi_SetWheelMotor1Speed(0);
    mhi_SetWheelMotor2Speed(0);
}

/**
* Check for walls on left without an update flag to move forward w/ PID
*
* \param None
* \retval MCI_WALL_NOT_FOUND
* \retval MCI_WALL_FOUND
*/
mci_wall_presence_t mci_CheckLeftWallMoveForwardPid(void)
{
    uint32_t reading = 0u;
    mci_wall_presence_t presence = MCI_WALL_NOT_FOUND;
    
    /* read left IR sensor */
    reading = mhi_ReadIr2();
    
    /* update front wall presence variable if available */
    if (reading >= MCI_LEFT_SENSOR_READING_THRESHOLD_RAW)
    {
        presence = MCI_WALL_FOUND;
    }
    else
    {
        presence = MCI_WALL_NOT_FOUND;
    }
    
    return presence;
}

/**
* Check for walls on right without an update flag to move forward w/ PID
*
* \param None
* \retval MCI_WALL_NOT_FOUND
* \retval MCI_WALL_FOUND
*/
mci_wall_presence_t mci_CheckRightWallMoveForwardPid(void)
{
    uint32_t reading = 0u;
    mci_wall_presence_t presence = MCI_WALL_NOT_FOUND;
    
    /* read left IR sensor */
    reading = mhi_ReadIr3();
    
    /* update front wall presence variable if available */
    if (reading >= MCI_RIGHT_SENSOR_READING_THRESHOLD_RAW)
    {
        presence = MCI_WALL_FOUND;
    }
    else
    {
        presence = MCI_WALL_NOT_FOUND;
    }
    
    return presence;
}

/*----------------------------------------------------------------------------*/
/*                           Local Shared Functions                           */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                         Private (Static) Functions                         */
/*----------------------------------------------------------------------------*/
/* None */
