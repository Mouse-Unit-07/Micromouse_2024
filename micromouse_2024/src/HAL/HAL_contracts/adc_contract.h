/*-------------------------------- FILE INFO -----------------------------------
* Filename        : adc_contract.h
* Author          : Team Kirbo
* Revision        : 1.0
* Updated         : 2024-03-31
* Purpose         : HAL contract layer
*
* This file contains the contract to abstract an ADC interface.
*
* Contracts define features of interfaces so high level code can use abstract
* handlers instead of hardware specific code. HAL config files use these
* contracts to link to hardware specific code.
*
* Step 1 for hardware abstraction.
*-----------------------------------------------------------------------------*/

#ifndef ADC_CONTRACT_H_
#define ADC_CONTRACT_H_

/*----------------------------------------------------------------------------*/
/*                                Definitions                                 */
/*----------------------------------------------------------------------------*/
/* ADC status enumeration */
typedef enum
{
    ADC_SUCCESS = 0u,
    ADC_ERROR,
    ADC_READ_ERROR
} adc_status_t;

/* ADC interface contract- used to create handlers */
typedef struct
{
    adc_status_t (*adc_Init)(void);
    adc_status_t (*adc_EnableChannel)(const uint32_t channelAddress);
    adc_status_t (*adc_DisableChannel)(const uint32_t channelAddress);
    adc_status_t (*adc_ReadValue)(
        const uint32_t channelAddress, uint32_t* p_readValue);
} adc_handler_t;

/*----------------------------------------------------------------------------*/
/*                              Global Variables                              */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                       Public (Exportable) Functions                        */
/*----------------------------------------------------------------------------*/
/* None */

#endif /* ADC_CONTRACT_H_ */