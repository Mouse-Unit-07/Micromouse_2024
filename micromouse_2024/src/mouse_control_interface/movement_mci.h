/*-------------------------------- FILE INFO -----------------------------------
* Filename        : movement_mci.h
* Author          : Ryuichi Lin
* Revision        : 1.0
* Updated         : 2024-04-13
* Purpose         : mouse control interface layer
*
* This is the header file for mouse movement under the mouse control
* interface.
*
* The mouse control interface uses the mouse hardware interface to define high
* level micromouse functionality.
*-----------------------------------------------------------------------------*/

#ifndef MOVEMENT_MCI_H_
#define MOVEMENT_MCI_H_

/*----------------------------------------------------------------------------*/
/*                                Definitions                                 */
/*----------------------------------------------------------------------------*/
#define MCI_TURN_SPEED            (100)
#define MCI_FORWARD_FAST_SPEED    (200)
#define MCI_FORWARD_SLOW_SPEED    (110)

/* ServoCity's N20 4900RPM Gear Motor: 60.8077 countable events per rev */
/* and 3D printed gear ratio of 44:13. (44/13)*60.8077 = 205.81 */
/* 205.81/4 = 51.45 rising edges per revolution */
/* New value found experimentally:  29 - a lot of slip when moving at 200 */
#define MCI_WHEEL_MOTOR_EDGES_PER_REVOLUTION     (29)

/* maze square = wall + pillar; 165mm+12mm = 177mm */
/* (edges/rev)/(wheel circumference) = edges/mm; (edges/mm)*(maze square) = */
/* edges/maze square; (52/103mm)*(177mm) = 89 edges/maze square */
/* Using new edges/rev: (29/103mm)*(177mm) = 50 edges/maze square */
#define MCI_WHEEL_MOTOR_EDGES_PER_MAZE_SQUARE    (50)

/* c = pi*d; pi*90mm = 282.7mm */
/* (edges/rev)/(wheel circumference) = edges/mm; (edges/mm)*(c/4) = */
/* edges/90deg turn; (52/103mm)*(282.7mm/4) = 36 edges/maze square */
#define MCI_WHEEL_MOTOR_EDGES_PER_90_DEGREE_TURN    (36)
/*----------------------------------------------------------------------------*/
/*                              Global Variables                              */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                       Public (Exportable) Functions                        */
/*----------------------------------------------------------------------------*/
void mci_MoveForward1Revolution(void);
void mci_MoveForward1MazeSquarePid(void);
void mci_TurnRight90Degrees(void);
void mci_TurnLeft90Degrees(void);

#endif /* MOVEMENT_MCI_H_ */