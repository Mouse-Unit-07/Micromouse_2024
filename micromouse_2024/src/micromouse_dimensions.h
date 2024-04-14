/*-------------------------------- FILE INFO -----------------------------------
* Filename        : micromouse_dimensions.h
* Author          : Ryuichi Lin
* Revision        : 1.0
* Updated         : 2024-04-13
* Purpose         : micromouse maze and bot dimensions
*
* This file contains dimensions of the micromouse maze and mouse used by the 
* micromouse control interface layer
*-----------------------------------------------------------------------------*/

#ifndef MICROMOUSE_DIMENSIONS_H_
#define MICROMOUSE_DIMENSIONS_H_

/*----------------------------------------------------------------------------*/
/*                                Definitions                                 */
/*----------------------------------------------------------------------------*/
/* CHANGE ACCORDING TO MAZE: */
/* length of maze walls from side to side in millimeters */
#define MCI_MAZE_WALL_LENGTH_MM                 (165)
/* width of maze pillar from side to side in millimeters */
#define MCI_MAZE_PILLAR_WIDTH_MM                (12)
/* IR sensor reading tolerances */
#define MCI_FRONT_SENSOR_READING_TOLERANCE      (10)
#define MCI_LEFT_SENSOR_READING_TOLERANCE       (10)
#define MCI_RIGHT_SENSOR_READING_TOLERANCE      (10)

/* DO NOT CHANGE BELOW:*/
/* mouse size from top of mouse to bottom of mouse */
#define MCI_MOUSE_LENGTH_MM                     (93)
/* mouse side to side size from left to right */
#define MCI_MOUSE_WIDTH_MM                      (61)
/* distance from top of mouse to tip of front sensors */
#define MCI_MOUSE_FRONT_SENSOR_OFFSET_MM        (19)
/* distance from side of mouse to tip of diagonal sensors */
#define MCI_MOUSE_DIAGONAL_SENSOR_OFFSET_MM     (18)
/* mouse wheel circumference in millimeters */
#define MCI_MOUSE_WHEEL_CIRCUMFERENCE_MM        (103)
/* mouse wheel to wheel distance in millimeters */
#define MCI_MOUSE_WHEEL_TO_WHEEL_DISTANCE_MM    (90)

/*----------------------------------------------------------------------------*/
/*                              Global Variables                              */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                       Public (Exportable) Functions                        */
/*----------------------------------------------------------------------------*/
/* None */

#endif /* MICROMOUSE_DIMENSIONS_H_ */