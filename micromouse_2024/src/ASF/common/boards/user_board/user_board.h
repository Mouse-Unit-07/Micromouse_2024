/**
 * \file
 *
 * \brief User board definition template
 *
 */

 /* This file is intended to contain definitions and configuration details for
 * features and devices that are available on the board, e.g., frequency and
 * startup time for an external crystal, external memory devices, LED and USART
 * pins.
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#ifndef USER_BOARD_H
#define USER_BOARD_H

#include <conf_board.h>

// External oscillator settings.
// Uncomment and set correct values if external oscillator is used.

// External oscillator frequency
//#define BOARD_XOSC_HZ          8000000

// External oscillator type.
//!< External clock signal
//#define BOARD_XOSC_TYPE        XOSC_TYPE_EXTERNAL
//!< 32.768 kHz resonator on TOSC
//#define BOARD_XOSC_TYPE        XOSC_TYPE_32KHZ
//!< 0.4 to 16 MHz resonator on XTALS
//#define BOARD_XOSC_TYPE        XOSC_TYPE_XTAL

// External oscillator startup time
//#define BOARD_XOSC_STARTUP_US  500000

#define FOSC32                  (32768)
#define OSC32_STARTUP           (AVR32_SCIF_OSCCTRL32_STARTUP_131072_RCOSC)

#define BOARD_OSC32_IS_XTAL     true
#define BOARD_OSC32_HZ          FOSC32
#define BOARD_OSC32_STARTUP_US  (920000)
#define BOARD_OSC32_PINSEL      (0)


#endif // USER_BOARD_H
