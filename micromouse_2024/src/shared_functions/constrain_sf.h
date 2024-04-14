/*-------------------------------- FILE INFO -----------------------------------
* Filename        : constrain_sf.h
* Author          : Ryuichi Lin
* Revision        : 1.0
* Updated         : 2024-04-13
* Purpose         : shared functions layer
*
* This is the header file for the constrain function.
*
* Shared functions are used by the mouse control interface for generic
* functionality.
*-----------------------------------------------------------------------------*/

#ifndef CONSTRAIN_SF_H_
#define CONSTRAIN_SF_H_

/*----------------------------------------------------------------------------*/
/*                                Definitions                                 */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                              Global Variables                              */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                       Public (Exportable) Functions                        */
/*----------------------------------------------------------------------------*/
int32_t sf_constrain(int32_t userValue, int32_t max, int32_t min);
float sf_CheckFloatZero(float userValue);

#endif /* CONSTRAIN_SF_H_ */