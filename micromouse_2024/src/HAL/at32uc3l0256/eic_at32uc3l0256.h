/*-------------------------------- FILE INFO -----------------------------------
* Filename        : eic_at32uc3l0256.h
* Author          : Team Kirbo
* Revision        : 1.0
* Updated         : 2024-03-31
* Purpose         : hardware specific layer
*
* This file is the header file for hardware specific external interrupt
* controller (eic) code.
*
* Step 3 for hardware abstraction.
*
* TODO: 
*     - Break down EIC init function (decouple from micromouse application)
*     - Change ISR to call a function defined in config file
*     - Add functions to change interrupt parameters during runtime
*     
*     Contract and config file simplified by tailoring hardware specific code
*     to micromouse application. HAL not scalable to broader MCU use.
*-----------------------------------------------------------------------------*/

#ifndef EIC_AT32UC3L0256_H_
#define EIC_AT32UC3L0256_H_

/*----------------------------------------------------------------------------*/
/*                                Definitions                                 */
/*----------------------------------------------------------------------------*/
/* motor encoder channel A pins */
#define MM_M1_ENC_A_PIN					   AVR32_PIN_PB07
#define MM_M2_ENC_A_PIN					   AVR32_PIN_PB09

/* motor encoder channel B pins */
#define MM_M1_ENC_B_PIN                    AVR32_PIN_PA04
#define MM_M2_ENC_B_PIN                    AVR32_PIN_PA11

/* interrupt for motor 1 encoder channel A */
#define MM_EIC_M1_PIN_LINE                 AVR32_EIC_EXTINT_1_1_PIN
#define MM_EIC_M1_FUNCTION_LINE            AVR32_EIC_EXTINT_1_1_FUNCTION
#define MM_EIC_M1_LINE                     AVR32_EIC_INT1
#define MM_EIC_M1_IRQ_LINE                 AVR32_EIC_IRQ_1
#define MM_INTC_M1_LEVEL                   AVR32_INTC_INT3
#define MM_EIC_M1_MODE                     EIC_MODE_EDGE_TRIGGERED
#define MM_EIC_M1_EDGE                     AVR32_EIC_RISING_EDGE
#define MM_EIC_M1_SYNC                     EIC_SYNCH_MODE

/* interrupt for motor 2 encoder channel A */
#define MM_EIC_M2_PIN_LINE                 AVR32_EIC_EXTINT_3_1_PIN
#define MM_EIC_M2_FUNCTION_LINE            AVR32_EIC_EXTINT_3_1_FUNCTION
#define MM_EIC_M2_LINE                     AVR32_EIC_INT3
#define MM_EIC_M2_IRQ_LINE                 AVR32_EIC_IRQ_3
#define MM_INTC_M2_LEVEL                   AVR32_INTC_INT3
#define MM_EIC_M2_MODE                     EIC_MODE_EDGE_TRIGGERED
#define MM_EIC_M2_EDGE                     AVR32_EIC_RISING_EDGE
#define MM_EIC_M2_SYNC                     EIC_SYNCH_MODE

/* number of encoder interrupts to configure */
#define MM_EIC_ENCODER_LINES_COUNT         (2u)

/* config pushbutton pin */
#define MM_CONFIG_PIN                      AVR32_PIN_PA06

/* interrupt for config pushbutton pin */
#define MM_EIC_CONFIG_PIN_LINE             6//AVR32_EIC_EXTINT_2_0_PIN
#define MM_EIC_COFIG_FUNCTION_LINE         5//AVR32_EIC_EXTINT_2_0_FUNCTION
#define MM_EIC_CONFIG_LINE                 AVR32_EIC_INT2
#define MM_EIC_CONFIG_IRQ_LINE             AVR32_EIC_IRQ_2
#define MM_INTC_CONFIG_LEVEL               AVR32_INTC_INT3
#define MM_EIC_CONFIG_MODE                 EIC_MODE_EDGE_TRIGGERED
#define MM_EIC_CONFIG_EDGE                 AVR32_EIC_FALLING_EDGE
#define MM_EIC_CONFIG_SYNC                 EIC_SYNCH_MODE

/* bounce time for config switch */
#define MM_CONFIG_PIN_BOUNCE_TIME_MS       (10u)

/* config switch idle pin value */
#define MM_CONFIG_PIN_DEFAULT_VALUE        (1u)

/*----------------------------------------------------------------------------*/
/*                              Global Variables                              */
/*----------------------------------------------------------------------------*/
/* encoder rising edge count */
volatile int32_t g_mm_Motor1EncoderEdgeCount;
volatile int32_t g_mm_Motor2EncoderEdgeCount;

/* software configuration rising edge count */
volatile uint32_t g_mm_ConfigSwitchEdgeCount;

/*----------------------------------------------------------------------------*/
/*                       Public (Exportable) Functions                        */
/*----------------------------------------------------------------------------*/
eic_status_t at32uc3l0256_InitEncoderInterrupts(void);
eic_status_t at32uc3l0256_InitConfigPinInterrupt(void);
eic_status_t at32uc3l0256_GetEncoder1EdgeCount(uint32_t *p_edgeCount);
eic_status_t at32uc3l0256_GetEncoder2EdgeCount(uint32_t *p_edgeCount);
eic_status_t at32uc3l0256_GetConfigPinEdgeCount(uint32_t *p_edgeCount);
eic_status_t at32uc3l0256_ClearEncoder1EdgeCount(void);
eic_status_t at32uc3l0256_ClearEncoder2EdgeCount(void);
eic_status_t at32uc3l0256_ClearConfigPinEdgeCount(void);

#endif /* EIC_AT32UC3L0256_H_ */