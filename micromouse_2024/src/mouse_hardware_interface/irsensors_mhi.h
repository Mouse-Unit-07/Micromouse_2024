/*-------------------------------- FILE INFO -----------------------------------
* Filename        : irsensors_mhi.h
* Author          : Ryuichi Lin
* Revision        : 1.0
* Updated         : 2024-04-07
* Purpose         : mouse hardware interface layer
*
* This is the header file for the mouse IR sensors interface.
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

#ifndef IRSENSORS_MHI_H_
#define IRSENSORS_MHI_H_

/*----------------------------------------------------------------------------*/
/*                                Definitions                                 */
/*----------------------------------------------------------------------------*/
#define MHI_IR1_PIN     AVR32_PIN_PA15    /* IR receiver 1 pin */
#define MHI_IR2_PIN     AVR32_PIN_PA16    /* IR receiver 2 pin */
#define MHI_IR3_PIN     AVR32_PIN_PA19    /* IR receiver 3 pin */
#define MHI_IR4_PIN     AVR32_PIN_PA18    /* IR receiver 4 pin */

#define MHI_ADC_CHANNEL_IR1    (1u)    /* ADC channel number for IR 1 */
#define MHI_ADC_CHANNEL_IR2    (2u)    /* ADC channel number for IR 2 */
#define MHI_ADC_CHANNEL_IR3    (5u)    /* ADC channel number for IR 3 */
#define MHI_ADC_CHANNEL_IR4    (4u)    /* ADC channel number for IR 4 */

#define MHI_ADC_CHANNEL_MASK_IR1 (1 << MHI_ADC_CHANNEL_IR1) /* IR1 ADC mask */
#define MHI_ADC_CHANNEL_MASK_IR2 (1 << MHI_ADC_CHANNEL_IR2) /* IR1 ADC mask */
#define MHI_ADC_CHANNEL_MASK_IR3 (1 << MHI_ADC_CHANNEL_IR3) /* IR1 ADC mask */
#define MHI_ADC_CHANNEL_MASK_IR4 (1 << MHI_ADC_CHANNEL_IR4) /* IR1 ADC mask */
/*----------------------------------------------------------------------------*/
/*                              Global Variables                              */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                       Public (Exportable) Functions                        */
/*----------------------------------------------------------------------------*/
void mhi_DisableIrSensors(void);    /* disable all IR sensors */
void mhi_InitAdc(void);             /* initialize ADC interface */
uint32_t mhi_ReadIr1(void);         /* initialize LEDs */
uint32_t mhi_ReadIr2(void);         /* initialize LEDs */
uint32_t mhi_ReadIr3(void);         /* initialize LEDs */
uint32_t mhi_ReadIr4(void);         /* initialize LEDs */

#endif /* IRSENSORS_MHI_H_ */