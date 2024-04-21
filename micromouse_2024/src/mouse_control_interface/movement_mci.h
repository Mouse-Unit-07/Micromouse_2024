/*-------------------------------- FILE INFO -----------------------------------
* Filename        : movement_mci.h
* Author          : Team Kirbo
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
#define MCI_TURN_SPEED            (140)
#define MCI_FORWARD_FAST_SPEED    (140)
/* minimum speed required for mouse to move */
#define MCI_MINIMUM_SPEED         (140)

/* ServoCity's N20 4900RPM Gear Motor: 60.8077 countable events per rev */
/* and 3D printed gear ratio of 44:13. (44/13)*60.8077 = 205.81 */
/* 205.81/4 = 51.45 rising edges per revolution */
/*
* New value found experimentally:  28 - a lot of slip when moving at 140 
*/
#define MCI_WHEEL_MOTOR_EDGES_PER_REVOLUTION        (28)

/* maze square = wall + pillar; 165mm+12mm = 177mm (actual maze dimensions) */
/* test maze square = 180mm+12mm = 192mm */
/* (edges/rev)/(wheel circumference) = edges/mm */
/* (edges/mm)*(maze square in mm) = edges/maze square */
/* (52/103mm)*(177mm) = 89 edges/maze square */
/* 
* Using  found edges/rev & real maze: (28/103mm)*(177mm) = 48 edges/square
* Using found edges/rev & test maze: (28/103mm)*(192mm) = 52 edges/square 
* Value found experimentally: 64
*/
#define MCI_WHEEL_MOTOR_EDGES_PER_MAZE_SQUARE       (75)

/* c = pi*d; pi*100mm = 295.3mm */
/* (edges/rev)/(wheel circumference) = edges/mm; (edges/mm)*(c/4) = */
/* edges/90deg turn; (52/103mm)*(295.3mm/4) = 37 edges/maze square */
/*
* Using found edges/rev: (28/103mm)*(295.3mm/4) = 20 edges/turn
* Value found experimentally for right turn: 32
* Value found experimentally for left turn: 32
*/
#define MCI_WHEEL_MOTOR_EDGES_PER_90_DEGREE_TURN_RIGHT    (32)
#define MCI_WHEEL_MOTOR_EDGES_PER_90_DEGREE_TURN_LEFT     (32)

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
void mci_AdjustToFrontWall(void);

mci_wall_presence_t mci_CheckLeftWallMoveForwardPid(void);
mci_wall_presence_t mci_CheckRightWallMoveForwardPid(void);

#endif /* MOVEMENT_MCI_H_ */