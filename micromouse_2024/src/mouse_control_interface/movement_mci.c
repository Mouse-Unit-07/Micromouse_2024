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
    mhi_SetWheelMotor1Speed(MCI_FORWARD_FAST_SPEED);
    mhi_SetWheelMotor2Speed(MCI_FORWARD_FAST_SPEED);
    mhi_StartWheelMotor1Forward();
    mhi_StartWheelMotor2Forward();
    
    /* keep moving motors until encoder edge counts are reached */
    while((!rightDone) || (!leftDone))
    {        
        if ((!rightDone) && (mhi_GetEncoder1EdgeCount() == MCI_WHEEL_MOTOR_EDGES_PER_REVOLUTION))
        {
            mhi_StopWheelMotor1();
            rightDone = 1u;
        }
        else if (mhi_GetEncoder1EdgeCount() >= (MCI_WHEEL_MOTOR_EDGES_PER_REVOLUTION / 3))
        {
            mhi_SetWheelMotor1Speed(MCI_FORWARD_FAST_SPEED);
        }
        else if (mhi_GetEncoder1EdgeCount() >= ((MCI_WHEEL_MOTOR_EDGES_PER_REVOLUTION / 3) * 2))
        {
            mhi_SetWheelMotor1Speed(MCI_FORWARD_SLOW_SPEED);
        }
        
        if ((!leftDone) && (mhi_GetEncoder2EdgeCount() == MCI_WHEEL_MOTOR_EDGES_PER_REVOLUTION))
        {
            mhi_StopWheelMotor2();
            leftDone = 1u;
        }
        else if (mhi_GetEncoder2EdgeCount() >= (MCI_WHEEL_MOTOR_EDGES_PER_REVOLUTION / 3))
        {
            mhi_SetWheelMotor2Speed(MCI_FORWARD_FAST_SPEED);
        }
        else if (mhi_GetEncoder2EdgeCount() >= ((MCI_WHEEL_MOTOR_EDGES_PER_REVOLUTION / 3) * 2))
        {
            mhi_SetWheelMotor2Speed(MCI_FORWARD_SLOW_SPEED);
        }
    }
    
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
    float kd = 0.1;    /* derivative term */
    
    /* initialize sensor PD constants */
    float kpSensor = 0.2;
    float kdSensor = 0.1;
    
    /* initialize left sensor PD constants */
    float kpSensorL = 0.2; // was 5
    float kdSensorL = 0;
    
    /* initialize right sensor PD constants */
    float kpSensorR = 0.2; // was 5
    float kdSensorR = 0;
    
    /* other encoder PID variables */
    int32_t initialPosition = mhi_GetEncoder1EdgeCount() + mhi_GetEncoder2EdgeCount();
    int32_t targetPosition = initialPosition + (int32_t)((MCI_WHEEL_MOTOR_EDGES_PER_MAZE_SQUARE * 2));
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
    
    /* configure both motors to move forward at base speed */
    mhi_SetWheelMotor1Speed(MCI_FORWARD_FAST_SPEED);
    mhi_SetWheelMotor2Speed(MCI_FORWARD_FAST_SPEED);
    mhi_StartWheelMotor1Forward();
    mhi_StartWheelMotor2Forward();
    
    /* main control loop */
	
    while((mhi_GetEncoder1EdgeCount() + mhi_GetEncoder2EdgeCount()) < targetPosition)
    {
        /* stop if there's a wall in front */
        // 		if(sbu_mm_CheckForFrontWall() == 1)
        // 		{
        // 			break;
        // 		}
//         uint32_t leftWall = sbu_mm_CheckForLeftWall();
//         uint32_t rightWall = sbu_mm_CheckForRightWall();
        
        //output error sensors
//         if ((leftWall == 1U) && (rightWall == 1U))
//         {mhi_DelayMs(1000);
            /* read left wall sensor */
            ir2Reading = mhi_ReadIr2();
            
            /* read right wall sensor */
            ir3Reading = mhi_ReadIr3();
            
            // Error with the sensors
            errorSensorLeft =  MCI_LEFT_SENSOR_READING_THRESHOLD_RAW - ir2Reading;
//             mhi_PrintInt(abs(errorSensorLeft));
//             mhi_PrintString(" ");
            errorSensorRight = MCI_RIGHT_SENSOR_READING_THRESHOLD_RAW - ir3Reading;
//             mhi_PrintInt(abs(errorSensorRight));
//             mhi_PrintString("\r\n");
            errorSensors =  errorSensorRight - errorSensorLeft;
            dErrorSensors = errorSensors - prevErrorSensors;
            
            // Current way of trying to add a pid controller to the mouse using the sensors going to aggregate with the other output
            outputSensors = (kpSensor * errorSensors) + (kdSensor * dErrorSensors);
//             mhi_PrintString("s: ");
//             if(outputSensors < 0)
//                 mhi_PrintString("-");
//             mhi_PrintInt((uint32_t)abs(outputSensors));
//             mhi_PrintString("\r\n");
            
            prevErrorSensors = errorSensors;
        //}
        
//         else if((leftWall == 1U) && (rightWall == 0U)) {
//             //Read left one
//             //sbu_mm_SetLed1();
//             sbu_mm_EnableIR2();
//             ir2Reading = sbu_mm_ReadADC();
//             sbu_mm_DisableIR2();
//             
//             //error with just left sensor
//             errorSensorLeft = g_sbu_mm_Ir2WallDetectedThreshold - ir2Reading;
//             //errorSensors = -errorSensorLeft * 2;
//             errorSensors = -errorSensorLeft;
//             dErrorSensors = errorSensors - prevErrorSensors;
//             outputSensors = (kpSensorL * errorSensors) + (kdSensorL * dErrorSensors);
//             // 			 if(targetAngle - (g_sbu_mm_Motor1EncoderEdgeCount - g_sbu_mm_Motor2EncoderEdgeCount) < -45){
//             // 				outputSensors*=-1;
//             // 			 }
             prevErrorSensors = errorSensors;
//             // sbu_mm_ClrLed1();
//         }
//         
//         else if((rightWall == 1U) && (leftWall == 0U)){
//             //sbu_mm_SetLed2();
//             //Read Right one
//             sbu_mm_EnableIR3();
//             ir3Reading = sbu_mm_ReadADC();
//             sbu_mm_DisableIR3();
//             
//             errorSensorRight = g_sbu_mm_Ir3WallDetectedThreshold - ir3Reading;
//             //errorSensors = errorSensorRight * 2;
//             errorSensors = errorSensorRight;
//             dErrorSensors = errorSensors - prevErrorSensors;
//             outputSensors = (kpSensorR * errorSensors) + (kdSensorR * dErrorSensors);
//             // 			  if(targetAngle - (g_sbu_mm_Motor1EncoderEdgeCount - g_sbu_mm_Motor2EncoderEdgeCount) < 45){
//             // 				  outputSensors*=-1;
//             // 			  }
//             prevErrorSensors = errorSensors;
//             //sbu_mm_ClrLed2();
//         }
        
//         else {
//             //sbu_mm_SetLed3();
//             outputSensors = 0;
//             prevErrorSensors = 0;
//         }
        
        /* calculate error (for proportional term) */
		mhi_PrintInt(mhi_GetEncoder1EdgeCount());
		mhi_PrintString(" ");
		mhi_PrintInt(mhi_GetEncoder2EdgeCount());
		mhi_PrintString("\r\n");
        error = targetAngle - ((int32_t)mhi_GetEncoder1EdgeCount() - (int32_t)mhi_GetEncoder2EdgeCount());
//         mhi_PrintString("e: ");
//         if(error < 0)
//             mhi_PrintString("-");
//         mhi_PrintInt((uint32_t)abs(error));
//         mhi_PrintString("\r\n");
       // error += outputSensors;
//         mhi_PrintString("e2: ");
//         if(error < 0)
//             mhi_PrintString("-");
//         mhi_PrintInt((uint32_t)abs(error));
//         mhi_PrintString("\r\n");
        
        /* calculate difference in error (for the derivative term) */
		error += outputSensors;
        dError = error - prevError;
        prevError = error;
        
        /* calculate PID output */
        output = (kp * error) + (kd * dError);
         mhi_PrintString("o: ");
         if(output < 0)
            mhi_PrintString("-");
         mhi_PrintInt((uint32_t)abs(output));
         mhi_PrintString(" = ");
         if(kp < 0)
             mhi_PrintString("-");
         mhi_PrintInt((uint32_t)kp);
         mhi_PrintString(" * ");
         if(error < 0)
             mhi_PrintString("-");
         mhi_PrintInt((uint32_t)abs(error));
         mhi_PrintString("\r\n");
        
        /* calculate new speeds */
        newLeftSpeed = sf_constrain(MCI_FORWARD_SLOW_SPEED + output, 255, -255);
        newRightSpeed = sf_constrain(MCI_FORWARD_SLOW_SPEED - output, 255, -255);
         mhi_PrintString("l: ");
         if(newLeftSpeed < 0)
             mhi_PrintString("-");
         mhi_PrintInt((uint32_t)abs(newLeftSpeed));
         mhi_PrintString("\r\n");
         mhi_PrintString("r: ");
         if(newRightSpeed < 0)
             mhi_PrintString("-");
         mhi_PrintInt((uint32_t)abs(newRightSpeed));
         mhi_PrintString("\r\n");
         mhi_PrintString("\r\n");
        
        
        /* set new motor speeds */
        if(newLeftSpeed < 0)
        {
            mhi_SetWheelMotor1Speed((uint16_t)abs(newLeftSpeed));
            mhi_StartWheelMotor1Backward();
        }
        else
        {
            mhi_SetWheelMotor1Speed((uint16_t)newLeftSpeed);
            mhi_StartWheelMotor1Forward();
        }
        
        if(newRightSpeed < 0)
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
    mhi_ClearEncoder1EdgeCount();
    mhi_ClearEncoder2EdgeCount();
    mhi_SetWheelMotor1Speed(0);
    mhi_SetWheelMotor2Speed(0);
}

/**
* Rotate mouse 90 degrees right
*
* \param None
* \retval None
*/
void mci_TurnRight90Degrees(void)
{
    
}

/**
* Rotate mouse 90 degrees left
*
* \param None
* \retval None
*/
void mci_TurnLeft90Degrees(void)
{
    
}

/*----------------------------------------------------------------------------*/
/*                           Local Shared Functions                           */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                         Private (Static) Functions                         */
/*----------------------------------------------------------------------------*/
/* None */
