/*-------------------------------- FILE INFO -----------------------------------
* Filename        : wallfollower_algo.h
* Author          : Team Kirbo
* Revision        : 1.0
* Updated         : 2024-04-20
* Purpose         : algo layer
*
* This is the header file for mouse wall follower algorithm.
*
* The algo layer defines all mouse maze solving algorithms as the second most
* highest layer to be called directly in main.
*-----------------------------------------------------------------------------*/

#ifndef WALLFOLLOWER_ALGO_H_
#define WALLFOLLOWER_ALGO_H_

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
void algo_LeftWallFollow();
void algo_RightWallFollow();

#endif /* WALLFOLLOWER_ALGO_H_ */