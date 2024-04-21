/*-------------------------------- FILE INFO -----------------------------------
* Filename        : power_mhi.h
* Author          : Team Kirbo
* Revision        : 1.0
* Updated         : 2024-04-07
* Purpose         : mouse hardware interface layer
*
* This is the header file for the mouse power interface.
*
* The mouse hardware interface uses the HAL to define functions needed to
* interface w/ all mouse hardware.
*-----------------------------------------------------------------------------*/

#ifndef POWER_MHI_H_
#define POWER_MHI_H_

/*----------------------------------------------------------------------------*/
/*                                Definitions                                 */
/*----------------------------------------------------------------------------*/
#define MHI_LOW_BATTERY_PIN         AVR32_PIN_PB06
#define MHI_REGULATOR_ENABLE_PIN    AVR32_PIN_PB11

typedef enum 
{
    MHI_REGULATORS_OFF = 0u,
    MHI_REGULATORS_ON
} mhi_power_state_t;

/*----------------------------------------------------------------------------*/
/*                              Global Variables                              */
/*----------------------------------------------------------------------------*/
/* None */

/*----------------------------------------------------------------------------*/
/*                       Public (Exportable) Functions                        */
/*----------------------------------------------------------------------------*/
void mhi_InitPower(void);
void mhi_CheckLowBattery(void);
void mhi_EnableRegulators(void);
void mhi_DisableRegulators(void);

#endif /* POWER_MHI_H_ */