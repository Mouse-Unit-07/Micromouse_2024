/*-------------------------------- FILE INFO -----------------------------------
* Filename        : irsensors_mhi.c
* Author          : Ryuichi Lin
* Revision        : 1.0
* Updated         : 2024-04-07
* Purpose         : mouse hardware interface layer
*
* This is the source file for the mouse IR sensors interface.
*
* The mouse hardware interface uses the HAL to define functions needed to
* interface w/ all mouse hardware.
*
* TODO:
*     - Add contracts and handlers for this MHI layer as well
*
* (if applicable)
* Target Hardware    : AT32UC3L0256
* IDE                : Atmel Studio 7.4.2542
* SDK                : ASF 3.52.0
*-----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*                               Include Files                                */
/*----------------------------------------------------------------------------*/
#include <asf.h> //embodiment of pain- defeats abstraction effort
#include <stdint.h>
#include <stdio.h>
#include "HAL/HAL_contracts/io_contract.h"
#include "HAL/HAL_configs/io_config.h"
#include "leds_mhi.h"
#include "HAL/HAL_contracts/adc_contract.h"
#include "HAL/HAL_configs/adc_config.h"
#include "irsensors_mhi.h"

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
* Disable all IR sensor receiver ADC channels for micromouse.
*
* \param  None
* \retval None
*/
void mhi_DisableIrSensors(void)
{
    adc_handler_t *adcInterface = NULL;
    config_GetAdcHandler(&adcInterface);
    
    if (adcInterface->adc_DisableChannel(MHI_ADC_CHANNEL_MASK_IR1)
        != ADC_SUCCESS)
        mhi_IndicateError(MHI_LEDS_IR_SENSOR_ERROR);
    if (adcInterface->adc_DisableChannel(MHI_ADC_CHANNEL_MASK_IR2)
        != ADC_SUCCESS)
        mhi_IndicateError(MHI_LEDS_IR_SENSOR_ERROR);
    if (adcInterface->adc_DisableChannel(MHI_ADC_CHANNEL_MASK_IR3)
        != ADC_SUCCESS)
        mhi_IndicateError(MHI_LEDS_IR_SENSOR_ERROR);
    if (adcInterface->adc_DisableChannel(MHI_ADC_CHANNEL_MASK_IR4)
        != ADC_SUCCESS)
        mhi_IndicateError(MHI_LEDS_IR_SENSOR_ERROR);
}

/**
* Initialize ADC interface for micromouse.
*
* \param  None
* \retval None
*/
void mhi_InitAdc(void)
{
    adc_handler_t *adcInterface = NULL;
    config_GetAdcHandler(&adcInterface);
    
    if (adcInterface->adc_Init() != ADC_SUCCESS)
        mhi_IndicateError(MHI_LEDS_IR_SENSOR_ERROR);
    
    mhi_DisableIrSensors();
}

/**
* Initialize IR sensor receiver pins for micromouse.
*
* \param  None
* \retval sensorReading ADC sensor reading from IR receiver 1
*/
uint32_t mhi_ReadIr1(void)
{
    uint32_t sensorReading = 0u;
    adc_handler_t *adcInterface = NULL;
    config_GetAdcHandler(&adcInterface);
    
    if (adcInterface->adc_EnableChannel(MHI_ADC_CHANNEL_MASK_IR1)
        != ADC_SUCCESS)
        mhi_IndicateError(MHI_LEDS_IR_SENSOR_ERROR);
    if (adcInterface->adc_ReadValue(0u, &sensorReading) 
        != ADC_SUCCESS)
        mhi_IndicateError(MHI_LEDS_IR_SENSOR_ERROR);
    if (adcInterface->adc_DisableChannel(MHI_ADC_CHANNEL_MASK_IR1)
        != ADC_SUCCESS)
        mhi_IndicateError(MHI_LEDS_IR_SENSOR_ERROR);
        
    return sensorReading;
}

/**
* Initialize IR sensor receiver pins for micromouse.
*
* \param  None
* \retval sensorReading ADC sensor reading from IR receiver 2
*/
uint32_t mhi_ReadIr2(void)
{
    uint32_t sensorReading = 0u;
    adc_handler_t *adcInterface = NULL;
    config_GetAdcHandler(&adcInterface);
    
    if (adcInterface->adc_EnableChannel(MHI_ADC_CHANNEL_MASK_IR2)
        != ADC_SUCCESS)
        mhi_IndicateError(MHI_LEDS_IR_SENSOR_ERROR);
    if (adcInterface->adc_ReadValue(0u, &sensorReading)
        != ADC_SUCCESS)
        mhi_IndicateError(MHI_LEDS_IR_SENSOR_ERROR);
    if (adcInterface->adc_DisableChannel(MHI_ADC_CHANNEL_MASK_IR2)
        != ADC_SUCCESS)
        mhi_IndicateError(MHI_LEDS_IR_SENSOR_ERROR);
        
    return sensorReading;
}

/**
* Initialize IR sensor receiver pins for micromouse.
*
* \param  None
* \retval sensorReading ADC sensor reading from IR receiver 3
*/
uint32_t mhi_ReadIr3(void)
{
    uint32_t sensorReading = 0u;
    adc_handler_t *adcInterface = NULL;
    config_GetAdcHandler(&adcInterface);
    
    if (adcInterface->adc_EnableChannel(MHI_ADC_CHANNEL_MASK_IR3)
        != ADC_SUCCESS)
        mhi_IndicateError(MHI_LEDS_IR_SENSOR_ERROR);
    if (adcInterface->adc_ReadValue(0u, &sensorReading)
        != ADC_SUCCESS)
        mhi_IndicateError(MHI_LEDS_IR_SENSOR_ERROR);
    if (adcInterface->adc_DisableChannel(MHI_ADC_CHANNEL_MASK_IR3)
        != ADC_SUCCESS)
        mhi_IndicateError(MHI_LEDS_IR_SENSOR_ERROR);
        
    return sensorReading;
}

/**
* Initialize IR sensor receiver pins for micromouse.
*
* \param  None
* \retval sensorReading ADC sensor reading from IR receiver 4
*/
uint32_t mhi_ReadIr4(void)
{
    uint32_t sensorReading = 0u;
    adc_handler_t *adcInterface = NULL;
    config_GetAdcHandler(&adcInterface);
    
    if (adcInterface->adc_EnableChannel(MHI_ADC_CHANNEL_MASK_IR4)
        != ADC_SUCCESS)
        mhi_IndicateError(MHI_LEDS_IR_SENSOR_ERROR);
    if (adcInterface->adc_ReadValue(0u, &sensorReading)
        != ADC_SUCCESS)
        mhi_IndicateError(MHI_LEDS_IR_SENSOR_ERROR);
    if (adcInterface->adc_DisableChannel(MHI_ADC_CHANNEL_MASK_IR4)
        != ADC_SUCCESS)
        mhi_IndicateError(MHI_LEDS_IR_SENSOR_ERROR);
        
    return sensorReading;
}

/*----------------------------------------------------------------------------*/
/*                           Local Shared Functions                           */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                         Private (Static) Functions                         */
/*----------------------------------------------------------------------------*/
/* None */
