/*-------------------------------- FILE INFO -----------------------------------
* Filename        : usart_at32uc3l0256.h
* Author          : Ryuichi Lin
* Revision        : 1.0
* Updated         : 2024-03-31
* Purpose         : hardware specific layer
*
* This file is the header file for hardware specific USART code.
*
* Step 3 for hardware abstraction.
*
* TODO:
*     - Add parameters to USART init function (avoid using macros)
*     - Add functions to modify USART parameters during runtime
*
*     Contract and config file simplified by tailoring hardware specific code
*     to micromouse application. HAL not scalable to broader MCU use.
*-----------------------------------------------------------------------------*/

#ifndef USART_AT32UC3L0256_H_
#define USART_AT32UC3L0256_H_

/*----------------------------------------------------------------------------*/
/*                                Definitions                                 */
/*----------------------------------------------------------------------------*/
#define MM_USART_USART_ADDRESS    (&AVR32_USART1)
#define MM_USART_RX_PIN           (AVR32_USART1_RXD_0_0_PIN)
#define MM_USART_TX_PIN           (AVR32_USART1_TXD_0_0_PIN)
#define MM_USART_RX_FUNCTION      (AVR32_USART1_RXD_0_0_FUNCTION)
#define MM_USART_TX_FUNCTION      (AVR32_USART1_TXD_0_0_FUNCTION)
#define MM_USART_BAUDRTE          (115200 + 10000)
#define MM_USART_CHAR_LENGTH      (8)

/* clock settings */
#define MM_PBA_CLK_FREQ_HZ         (35000000u)

/*----------------------------------------------------------------------------*/
/*                              Global Variables                              */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                       Public (Exportable) Functions                        */
/*----------------------------------------------------------------------------*/
usart_status_t at32uc3l0256_InitUsart(void);
usart_status_t at32uc3l0256_PrintString(const char *p_userString);
usart_status_t at32uc3l0256_PrintInt(unsigned long userInt);

#endif /* USART_AT32UC3L0256_H_ */