/*-------------------------------- FILE INFO -----------------------------------
* Filename        : constrain_sf.c
* Author          : Ryuichi Lin
* Revision        : 1.0
* Updated         : 2024-04-13
* Purpose         : shared functions layer
*
* This is the source file for the constrain function.
*
* Shared functions are used by the mouse control interface for generic
* functionality.
*-----------------------------------------------------------------------------*/ 

/*----------------------------------------------------------------------------*/
/*                               Include Files                                */
/*----------------------------------------------------------------------------*/
#include <stdint.h>
#include "shared_functions/constrain_sf.h"

/*----------------------------------------------------------------------------*/
/*                                Definitions                                 */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                               Debug Switches                               */
/*----------------------------------------------------------------------------*/
/* None */

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
* Constrain an integer to be between a min and max value
*
* \param[in] userValue Integer to constrain
* \param[in] max Maximum value that the provided integer can be
* \param[in] min Minimum value that the provided integer can be
* \retval constrained integer
*/
int32_t sf_constrain(int32_t userValue, int32_t max, int32_t min)
{
    if (userValue < min)
        return min;
    else if (userValue > max)
        return max;
    else
        return userValue;
}

/**
* Check whether a float is effectively zero
*
* \param[in] userValue Float to check below epsilon
* \retval zero if less than epsilon, userValue otherwise
*/
float sf_CheckFloatZero(float userValue)
{
    float epsilon = 0.001f;
    
    if (userValue < epsilon)
        return 0.0f;
    else
        return userValue;
}

/*----------------------------------------------------------------------------*/
/*                           Local Shared Functions                           */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                         Private (Static) Functions                         */
/*----------------------------------------------------------------------------*/
