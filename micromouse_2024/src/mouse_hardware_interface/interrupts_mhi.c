/*-------------------------------- FILE INFO -----------------------------------
* Filename        : interrupts_mhi.c
* Author          : Ryuichi Lin
* Revision        : 1.0
* Updated         : 2024-04-07
* Purpose         : mouse hardware interface layer
*
* This is the source file for the mouse interrupt interface.
* This interface includes just the interrupt related functions that don't
* fit into the other interfaces.
*
* The mouse hardware interface uses the HAL to define functions needed to
* interface w/ all mouse hardware.
*
* TODO:
*     - Check and handle status return values of IIC HAL functions
*-----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*                               Include Files                                */
/*----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include "HAL/HAL_contracts/iic_contract.h"
#include "HAL/HAL_configs/iic_config.h"
#include "HAL/HAL_contracts/eic_contract.h"
#include "HAL/HAL_configs/eic_config.h"
#include "interrupts_mhi.h"

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
* Initialize interrupts for micromouse.
*
* \param  None
* \retval None
*/
void mhi_InitInterrupts(void)
{
    iic_handler_t *iicInterface = NULL;
    eic_handler_t *eicInterface = NULL;
    config_GetIicHandler(&iicInterface);
    config_GetEicHandler(&eicInterface);
    
    iicInterface->iic_Init();
    eicInterface->eic_InitEncoderInterrupts();
    eicInterface->eic_InitConfigPinInterrupt();
}

/**
* Enable global interrupts for micromouse.
*
* \param  None
* \retval None
*/
void mhi_EnableGlobalInterrupts(void)
{
    iic_handler_t *iicInterface = NULL;
    config_GetIicHandler(&iicInterface);
    
    iicInterface->iic_EnableGlobalInterrupts();
}

/**
* Disable global interrupts for micromouse.
*
* \param  None
* \retval None
*/
void mhi_DisableGlobalInterrupts(void)
{
    iic_handler_t *iicInterface = NULL;
    config_GetIicHandler(&iicInterface);
    
    iicInterface->iic_DisableGlobalInterrupts();
}

/**
* Access function for config pin edge counts.
*
* \param  None
* \retval None
*/
uint32_t mhi_GetConfigPinEdgeCount(void)
{
    uint32_t edgeCount = 0u;
    eic_handler_t *eicInterface = NULL;
    config_GetEicHandler(&eicInterface);
    
    eicInterface->eic_GetConfigPinEdgeCount(&edgeCount);
    
    return edgeCount;
}

/**
* Access function for encoder 1 edge counts.
*
* \param  None
* \retval None
*/
uint32_t mhi_GetEncoder1EdgeCount(void)
{
    uint32_t edgeCount = 0u;
    eic_handler_t *eicInterface = NULL;
    config_GetEicHandler(&eicInterface);
    
    eicInterface->eic_GetEncoder1EdgeCount(&edgeCount);
    
    return edgeCount;
}

/**
* Access function for encoder 2 edge counts.
*
* \param  None
* \retval None
*/
uint32_t mhi_GetEncoder2EdgeCount(void)
{
    uint32_t edgeCount = 0u;
    eic_handler_t *eicInterface = NULL;
    config_GetEicHandler(&eicInterface);
    
    eicInterface->eic_GetEncoder2EdgeCount(&edgeCount);
    
    return edgeCount;
}

/**
* Clear config pin edge count.
*
* \param  None
* \retval None
*/
void mhi_ClearConfigPinEdgeCount(void)
{
    eic_handler_t *eicInterface = NULL;
    config_GetEicHandler(&eicInterface);
    
    eicInterface->eic_ClearConfigPinEdgeCount();
}

/**
* Clear encoder 1 edge count.
*
* \param  None
* \retval None
*/
void mhi_ClearEncoder1EdgeCount(void)
{
    eic_handler_t *eicInterface = NULL;
    config_GetEicHandler(&eicInterface);
    
    eicInterface->eic_ClearEncoder1EdgeCount();
}

/**
* Clear encoder 2 edge count.
*
* \param  None
* \retval None
*/
void mhi_ClearEncoder2EdgeCount(void)
{
    eic_handler_t *eicInterface = NULL;
    config_GetEicHandler(&eicInterface);
    
    eicInterface->eic_ClearEncoder2EdgeCount();
}

/*----------------------------------------------------------------------------*/
/*                           Local Shared Functions                           */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                         Private (Static) Functions                         */
/*----------------------------------------------------------------------------*/
/* None */