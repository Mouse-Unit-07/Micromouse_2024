/*-------------------------------- FILE INFO -----------------------------------
* Filename        : walldetection_mci.h
* Author          : Ryuichi Lin
* Revision        : 1.0
* Updated         : 2024-04-13
* Purpose         : mouse control interface layer
*
* This is the header file for mouse wall detection under the mouse control
* interface.
*
* The mouse control interface uses the mouse hardware interface to define high
* level micromouse functionality.
*-----------------------------------------------------------------------------*/ 

#ifndef WALLDETECTION_MCI_H_
#define WALLDETECTION_MCI_H_

/*----------------------------------------------------------------------------*/
/*                                Definitions                                 */
/*----------------------------------------------------------------------------*/
typedef enum
{
    MCI_WALL_NOT_FOUND = 0u,
    MCI_WALL_FOUND,
    MCI_CANNOT_READ_WALL
} mci_wall_presence_t;

typedef enum
{
    MCI_WALL_CHECK_NOT_AVAILABLE = 0u,
    MCI_WALL_CHECK_AVAILABLE
} mci_wall_check_availability_t;

#define MCI_COS45DEG    (0.707f)    /* cosine of 45 degrees to reduce math */

/* front sensor reading threshold in millimeters */
#define MCI_FRONT_SENSOR_READING_THRESHOLD_MM \
    (((MCI_MAZE_WALL_LENGTH_MM - MCI_MOUSE_LENGTH_MM) + \
    MCI_MOUSE_FRONT_SENSOR_OFFSET_MM) - MCI_FRONT_SENSOR_READING_TOLERANCE)
/* left sensor reading threshold in millimeters */
#define MCI_LEFT_SENSOR_READING_THRESHOLD_MM \
    ((((MCI_MAZE_WALL_LENGTH_MM - MCI_MOUSE_WIDTH_MM) + \
    MCI_MOUSE_DIAGONAL_SENSOR_OFFSET_MM) / MCI_COS45DEG) - \
    MCI_LEFT_SENSOR_READING_TOLERANCE)
/* right sensor reading threshold in millimeters */
#define MCI_RIGHT_SENSOR_READING_THRESHOLD_MM \
    ((((MCI_MAZE_WALL_LENGTH_MM - MCI_MOUSE_WIDTH_MM) + \
    MCI_MOUSE_DIAGONAL_SENSOR_OFFSET_MM) / MCI_COS45DEG) - \
    MCI_RIGHT_SENSOR_READING_TOLERANCE)

/* sensor threshold mm to raw converter, values found experimentally */
#define MCI_READING_MM_TO_RAW(x) (896 * pow(0.98, x))

/* sensor threshold hard coded raw values- prepared since math is extremely slow */
/* values found by running mci_PrintWallPresence() w/ thresholds printed after */
/* changing wall length MCI_MAZE_WALL_LENGTH_MM */
#define MCI_FRONT_SENSOR_READING_THRESHOLD_RAW_165MM_WALLS_HARD_CODED    (174)
#define MCI_LEFT_SENSOR_READING_THRESHOLD_RAW_165MM_WALLS_HARD_CODED     (33)
#define MCI_RIGHT_SENSOR_READING_THRESHOLD_RAW_165MM_WALLS_HARD_CODED    (33)

#define MCI_FRONT_SENSOR_READING_THRESHOLD_RAW_180MM_WALLS_HARD_CODED    (128)
#define MCI_LEFT_SENSOR_READING_THRESHOLD_RAW_180MM_WALLS_HARD_CODED     (21)
#define MCI_RIGHT_SENSOR_READING_THRESHOLD_RAW_180MM_WALLS_HARD_CODED    (21)

/* front sensor threshold raw value */
#define MCI_FRONT_SENSOR_READING_THRESHOLD_RAW \
    MCI_FRONT_SENSOR_READING_THRESHOLD_RAW_165MM_WALLS_HARD_CODED
    //MCI_READING_MM_TO_RAW(MCI_FRONT_SENSOR_READING_THRESHOLD_MM)

/* left sensor threshold raw value */
#define MCI_LEFT_SENSOR_READING_THRESHOLD_RAW \
    MCI_LEFT_SENSOR_READING_THRESHOLD_RAW_165MM_WALLS_HARD_CODED
    //MCI_READING_MM_TO_RAW(MCI_LEFT_SENSOR_READING_THRESHOLD_MM)

/* right sensor threshold raw value */
#define MCI_RIGHT_SENSOR_READING_THRESHOLD_RAW \
    MCI_RIGHT_SENSOR_READING_THRESHOLD_RAW_165MM_WALLS_HARD_CODED
    //MCI_READING_MM_TO_RAW(MCI_RIGHT_SENSOR_READING_THRESHOLD_MM)

/*----------------------------------------------------------------------------*/
/*                              Global Variables                              */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                       Public (Exportable) Functions                        */
/*----------------------------------------------------------------------------*/
void mci_PrintWallSensorReadings(void);
void mci_PrintWallPresence(void);

mci_wall_presence_t mci_CheckFrontWall(void);
mci_wall_presence_t mci_CheckLeftWall(void);
mci_wall_presence_t mci_CheckRightWall(void);

void mci_SetFrontWallCheckUnavailable(void);
void mci_SetLeftWallCheckUnavailable(void);
void mci_SetRightWallCheckUnavailable(void);
void mci_SetFrontWallCheckAvailable(void);
void mci_SetLeftWallCheckAvailable(void);
void mci_SetRightWallCheckAvailable(void);

void mci_UpdateFrontWallPresence(void);
void mci_UpdateLeftWallPresence(void);
void mci_UpdateRightWallPresence(void);

#endif /* WALLDETECTION_MCI_H_ */
