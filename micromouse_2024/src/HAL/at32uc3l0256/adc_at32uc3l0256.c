/*-------------------------------- FILE INFO -----------------------------------
* Filename        : adc_at32uc3l0256.c
* Author          : Ryuichi Lin
* Revision        : 1.0
* Updated         : 2024-03-31
* Purpose         : hardware specific layer
*
* This file is the header file for hardware specific ADC code.
*
* Step 3 for hardware abstraction.
*
* (if applicable)
* Target Hardware    : AT32UC3L0256
* IDE                : Atmel Studio 7.4.2542
* SDK                : ASF 3.52.0
*-----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*                               Include Files                                */
/*----------------------------------------------------------------------------*/
#include <asf.h>
#include <stdint.h>
#include "HAL/HAL_contracts/adc_contract.h"
#include "HAL/at32uc3l0256/adc_at32uc3l0256.h"

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
* Initialize ADC interface for AT32UC3L0256 MCU.
*
* \retval ADC_SUCCESS Success
* \retval ADC_ERROR Failure: Failed to initialize ADC
*/
adc_status_t at32uc3l0256_InitAdc(void)
{
    adc_status_t adcStatus = ADC_SUCCESS;
    
    const gpio_map_t ADCIFB_GPIO_MAP = MM_ADC_INIT_MAP;
    
    adcifb_opt_t adcifb_opt = {
		.resolution = AVR32_ADCIFB_ACR_RES_10BIT, /* 10 or 12 bit ADC */
		.shtim  = 15, /* Channels Sample & Hold Time in [0,15] */
		.ratio_clkadcifb_clkadc =
				(sysclk_get_pba_hz() / MM_ADC_CLK_FREQ_HZ),
        /* Startup time in [0,127]; Tstartup = startup * 8 * Tclk_adc */
        /* (assuming Tstartup ~ 15us max) */
		.startup = 3,
		.sleep_mode_enable = false /* ADCIFB Sleep Mode disabled */
	};
	
	sysclk_init();
	
	gpio_enable_module(ADCIFB_GPIO_MAP,
	    sizeof(ADCIFB_GPIO_MAP) / sizeof(ADCIFB_GPIO_MAP[0]));
	
	if (adcifb_configure(&AVR32_ADCIFB, &adcifb_opt) != PASS) {
		/* Config error. */
        adcStatus = ADC_ERROR;
	}
	
	if (adcifb_configure_trigger(&AVR32_ADCIFB, AVR32_ADCIFB_TRGMOD_NT, 0)
	    != PASS) {
		/* Config error. */
        adcStatus = ADC_ERROR;
	}
	
	delay_ms(MM_ADC_INIT_DELAY_MS);
    
    /* return status */
    return adcStatus;
}

/**
* Enable ADC channel for AT32UC3L0256 MCU.
*
* \param[in] channelAddress Channel to enable ADC
* \retval ADC_SUCCESS Success
* \retval ADC_ERROR Failure: Failed to enable ADC
*/
adc_status_t at32uc3l0256_EnableAdcChannel(const uint32_t channelAddress)
{
    adc_status_t adcStatus = ADC_ERROR;
    uint32_t adcWatchdog = 0u;
    
    adcifb_channels_enable(&AVR32_ADCIFB, channelAddress);
    while ( (adcifb_is_ready(&AVR32_ADCIFB) != true) && 
        (adcWatchdog < MM_ADC_WATCHDOG_MAX) ) {
        adcWatchdog++; //asm("NOP");
    }
    
    if (adcWatchdog < MM_ADC_WATCHDOG_MAX)
    {
        adcWatchdog = 0u;
        adcifb_start_conversion_sequence(&AVR32_ADCIFB);
        while ( (adcifb_is_drdy(&AVR32_ADCIFB) != true) &&
        (adcWatchdog < MM_ADC_WATCHDOG_MAX) ) {
            adcWatchdog++; //asm("NOP");
        }    
    }
    
    delay_ms(MM_ENABLE_DELAY_MS);
    
    if (adcWatchdog >= MM_ADC_WATCHDOG_MAX)
        adcStatus = ADC_ERROR;
    else
        adcStatus = ADC_SUCCESS;
        
    /* return status */
    return adcStatus;
}

/**
* Disable ADC channel for AT32UC3L0256 MCU.
*
* \param[in] channelAddress Channel to disable ADC
* \retval ADC_SUCCESS Success
* \retval ADC_ERROR Failure: Failed to disable ADC
*/
adc_status_t at32uc3l0256_DisableAdcChannel(const uint32_t channelAddress)
{
    adc_status_t adcStatus = ADC_ERROR;
    
    adcifb_channels_disable(&AVR32_ADCIFB, channelAddress);
    delay_ms(MM_DISABLE_DELAY_MS);
    adcStatus = ADC_SUCCESS;
    
    /* return status */
    return adcStatus;
}

/**
* Read ADC channel for AT32UC3L0256 MCU.
*
* \param[in] channelAddress Channel to read ADC (not used)
* \param[out] p_readValue ADC output value
* \retval ADC_SUCCESS Success
* \retval ADC_ERROR Failure: Failed to disable ADC
*/
adc_status_t at32uc3l0256_ReadValue(
    const uint32_t channelAddress, uint32_t* p_readValue)
{
    adc_status_t adcStatus = ADC_ERROR;
    
    *p_readValue = (uint32_t)(adcifb_get_last_data(&AVR32_ADCIFB) & 0xFFF);
    adcStatus = ADC_SUCCESS;
    
    /* return status */
    return adcStatus;
}

/*----------------------------------------------------------------------------*/
/*                           Local Shared Functions                           */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                         Private (Static) Functions                         */
/*----------------------------------------------------------------------------*/
/* None */
