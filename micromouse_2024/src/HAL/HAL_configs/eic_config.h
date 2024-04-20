/*-------------------------------- FILE INFO -----------------------------------
* Filename        : eic_config.h
* Author          : Team Kirbo
* Revision        : 1.0
* Updated         : 2024-03-31
* Purpose         : HAL config layer
*
* This file is the header file for the external interrupt controller (eic) 
* config file.
*
* HAL config files define handlers for high level code to access hardware
* specific code abstracted out w/ HAL contracts. Bridges contracts and
* hardware specific code by creating an instance of a contract w/ members
* filled w/ hardware specific code that adheres to the contract.
*
* Step 2 for hardware abstraction.
*-----------------------------------------------------------------------------*/

#ifndef EIC_CONFIG_H_
#define EIC_CONFIG_H_

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
void config_GetEicHandler(eic_handler_t** p_eicHandler);

#endif /* EIC_CONFIG_H_ */