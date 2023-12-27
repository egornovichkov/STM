/*
 * BMS_Power_Control.h
 *
 *  Created on: Jul 10, 2023
 *      Author: TDM
 */

#ifndef INC_BMS_POWER_CONTROL_H_
#define INC_BMS_POWER_CONTROL_H_

void TURN_OFF_ALL_MOSFETS(void);
void TURN_ON_ALL_MOSFETS(void);
void TURN_ON_CHARGE_MOSFET(void);
void TURN_ON_DISCHARGE_MOSFET(void);
void TURN_OFF_DISCHARGE_MOSFET(void);
void TURN_OFF_CHARGE_MOSFET(void);
void TURN_ON_PRE_CHARGE(void);
void TURN_OFF_PRE_CHARGE(void);

void LED_ALARM_ON(void);
void LED_ALARM_OFF(void);
void LED_OK_ON(void);
void LED_OK_OFF(void);
void LED_DISCHARGE_ON(void);
void LED_DISCHARGE_OFF(void);
void LED_CHARGE_ON(void);
void LED_CHARGE_OFF(void);


#endif /* INC_BMS_POWER_CONTROL_H_ */
