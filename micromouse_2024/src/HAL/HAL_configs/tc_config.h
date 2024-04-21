/*-------------------------------- FILE INFO -----------------------------------
* Filename        : tc_config.h
* Author          : Team Kirbo
* Revision        : 1.0
* Updated         : 2024-03-31
* Purpose         : HAL config layer
*
* This file is the header file for the TC config file.
*
* HAL config files define handlers for high level code to access hardware
* specific code abstracted out w/ HAL contracts. Bridges contracts and
* hardware specific code by creating an instance of a contract w/ members
* filled w/ hardware specific code that adheres to the contract.
*
* Step 2 for hardware abstraction.
*-----------------------------------------------------------------------------*/

#ifndef TC_CONFIG_H_
#define TC_CONFIG_H_

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
void config_GetTcHandler(tc_handler_t** p_tcHandler);

#endif /* TC_CONFIG_H_ */