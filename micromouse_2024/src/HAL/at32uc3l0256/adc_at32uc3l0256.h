/*-------------------------------- FILE INFO -----------------------------------
* Filename        : adc_at32uc3l0256.h
* Author          : Ryuichi Lin
* Revision        : 1.0
* Updated         : 2024-03-31
* Purpose         : hardware specific layer
*
* This file is the header file for hardware specific ADC code.
*
* Step 3 for hardware abstraction.
*
* TODO: 
*     - Break down ADC init function (decouple from micromouse application)
*     - Add functions to modify ADC parameters during runtime
*
*     Contract and config file simplified by tailoring hardware specific code
*     to micromouse application. HAL not scalable to broader MCU use.
*-----------------------------------------------------------------------------*/

#ifndef ADC_AT32UC3L0256_H_
#define ADC_AT32UC3L0256_H_

/*----------------------------------------------------------------------------*/
/*                                Definitions                                 */
/*----------------------------------------------------------------------------*/
/* IR1 - IR4 ADC pins */
#define MM_ADC_1             AVR32_ADCIFB_AD_1_PIN
#define MM_ADC_2             AVR32_ADCIFB_AD_2_PIN
#define MM_ADC_3             AVR32_ADCIFB_AD_5_PIN
#define MM_ADC_4             AVR32_ADCIFB_AD_4_PIN

/* IR1 - IR4 ADC pin functions */
#define MM_ADC_1_FUNCTION    AVR32_ADCIFB_AD_1_FUNCTION
#define MM_ADC_2_FUNCTION    AVR32_ADCIFB_AD_2_FUNCTION
#define MM_ADC_3_FUNCTION    AVR32_ADCIFB_AD_5_FUNCTION
#define MM_ADC_4_FUNCTION    AVR32_ADCIFB_AD_4_FUNCTION

/* ADC init GPIO map */
#define MM_ADC_INIT_MAP { \
    {MM_ADC_1, MM_ADC_1_FUNCTION}, \
    {MM_ADC_2, MM_ADC_2_FUNCTION}, \
    {MM_ADC_3, MM_ADC_3_FUNCTION}, \
    {MM_ADC_4, MM_ADC_4_FUNCTION}, \
}

#define MM_ADC_CLK_FREQ_HZ      1500000        /* ADC clock frequency */
#define MM_ADC_INIT_DELAY_MS    (30u)          /* ADC init delay in ms */
#define MM_ENABLE_DELAY_MS      (2u)           /* ADC enable delay in ms */
#define MM_DISABLE_DELAY_MS     (0u)           /* ADC disable delay in ms */
#define MM_ADC_WATCHDOG_MAX     (50000000u)    /* ADC watchdog counter max */

/*----------------------------------------------------------------------------*/
/*                              Global Variables                              */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                       Public (Exportable) Functions                        */
/*----------------------------------------------------------------------------*/
adc_status_t at32uc3l0256_InitAdc(void);
adc_status_t at32uc3l0256_EnableAdcChannel(const uint32_t channelAddress);
adc_status_t at32uc3l0256_DisableAdcChannel(const uint32_t channelAddress);
adc_status_t at32uc3l0256_ReadValue(
    const uint32_t channelAddress, uint32_t* p_readValue);

#endif /* ADC_AT32UC3L0256_H_ */