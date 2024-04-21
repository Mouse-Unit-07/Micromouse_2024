/*-------------------------------- FILE INFO -----------------------------------
* Filename        : leds_mhi.h
* Author          : Team Kirbo
* Revision        : 1.0
* Updated         : 2024-04-07
* Purpose         : mouse hardware interface layer
*
* This is the header file for the mouse LED interface.
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

#ifndef LEDS_MHI_H_
#define LEDS_MHI_H_

/*----------------------------------------------------------------------------*/
/*                                Definitions                                 */
/*----------------------------------------------------------------------------*/
#define MHI_D1_LED_PIN     AVR32_PIN_PA22    /* LED D1 pin */
#define MHI_D2_LED_PIN     AVR32_PIN_PB03    /* LED D2 pin */
#define MHI_D3_LED_PIN     AVR32_PIN_PB02    /* LED D3 pin */
#define MHI_ERR_LED_PIN    AVR32_PIN_PB00    /* LED ERR pin */

typedef enum
{
    MHI_LEDS_GENERIC_ERROR = 0u,
    MHI_LEDS_TC_ERROR,
    MHI_LEDS_USART_ERROR,
    MHI_LEDS_LOW_BATTERY_ERROR,
    MHI_LEDS_IR_SENSOR_ERROR,
    MHI_LEDS_PWM_ERROR
} mhi_error_type_t;

/*----------------------------------------------------------------------------*/
/*                              Global Variables                              */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                       Public (Exportable) Functions                        */
/*----------------------------------------------------------------------------*/
void mhi_InitLeds(void);        /* initialize LEDs */

void mhi_SetD1Led(void);        /* set D1 LED */
void mhi_SetD2Led(void);        /* set D2 LED */
void mhi_SetD3Led(void);        /* set D3 LED */
void mhi_SetErrLed(void);       /* set ERR LED */

void mhi_ClearD1Led(void);      /* clear D1 LED */
void mhi_ClearD2Led(void);      /* clear D1 LED */
void mhi_ClearD3Led(void);      /* clear D1 LED */
void mhi_ClearErrLed(void);     /* clear D1 LED */

void mhi_ToggleD1Led(void);     /* toggle D1 LED */
void mhi_ToggleD2Led(void);     /* toggle D1 LED */
void mhi_ToggleD3Led(void);     /* toggle D1 LED */
void mhi_ToggleErrLed(void);    /* toggle D1 LED */

void mhi_IndicateError(const mhi_error_type_t error);   /* error indication */

#endif /* LEDS_MHI_H_ */