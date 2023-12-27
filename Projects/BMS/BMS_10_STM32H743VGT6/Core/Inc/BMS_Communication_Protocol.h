/*
 * BMS_Communication_Protocol.h
 *
 *  Created on: 18 апр. 2023 г.
 *      Author: TDM
 */

#ifndef INC_BMS_COMMUNICATION_PROTOCOL_H_
#define INC_BMS_COMMUNICATION_PROTOCOL_H_

#include "stdint.h"

#define LOW_BYTE(x) (x & 0xFF)
#define HIGH_BYTE(x) ((x >> 8) & 0xFF)

#define VOLTAGE_DIVIDER 10
#define INFO_FRAME_LENGTH_REMOTE_MEAS 118
#define INFO_FRAME_LENGTH_REMOTE_ADJUST 80
#define INFO_FRAME_LENGTH_REMOTE_SIGNALING 74
#define INFO_FRAME_LENGTH_REMOTE_HISTORY 130

uint16_t Count_Lchksum (uint16_t info_length);
uint16_t Count_Chksum(uint8_t *buffer_ptr, uint16_t info_length);
uint16_t Hex_to_ASCII(uint8_t Hex_Input);
uint8_t ASCII_to_Hex (uint8_t ASCII_Input);

//int32_t ParametricArray[70];

enum ParametricArrayEnum{
	The_pack_location,
	Cell_high_voltage_alarm_parameter,
	Cell_low_voltage_alarm_parameter,
	Cell_high_temperature_alarm_parameter,
	Cell_low_temperature_alarm_parameter,
	Charge_over_current_alarm_parameter,
	Battery_high_voltage_alarm_parameter,
	Battery_low_voltage_alarm_parameter,
	Self_definition_parameter_number,
	Discharge_over_current_alarm_parameter,
	Cell_over_voltage_protection_parameter,
	Cell_over_voltage_recover_parameter,
	Cell_under_voltage_recover_parameter,
	Cell_under_voltage_protection_parameter,
	Battery_over_voltage_protection_parameter,
	Battery_over_voltage_recover_parameter,
	Battery_under_voltage_recover_parameter,
	Battery_under_voltage_protection_parameter,
	Charge_over_temperature_protection_parameter,
	Charge_over_temperature_recover_parameter,
	Charge_under_temperature_recover_parameter,
	Charge_under_temperature_protection_parameter,
	Discharge_over_temperature_protection_parameter,
	Discharge_over_temperature_recover_parameter,
	Discharge_under_temperature_recover_parameter,
	Discharge_under_temperature_protection_parameter,
	High_environment_temperature_alarm_parameter,
	Low_environment_temperature_alarm_parameter,
	Over_environment_temperature_protection_parameter,
	Over_environment_temperature_recover_parameter,
	Under_environment_temperature_protection_parameter,
	Under_environment_temperature_recover_parameter,
	Cell_heating_temperature_open_parameter,
	Cell_heating_temperature_stop_parameter,
	High_power_temperature_alarm_parameter,
	Low_power_temperature_alarm_parameter,
	Over_power_temperature_protection_parameter,
	Over_power_temperature_recover_parameter,
	Under_power_temperature_recover_parameter,
	Under_power_temperature_protection_parameter,
	Charge_over_current_protection_parameter,
	Charge_over_current_time_delay_parameter,
	Discharge_over_current_protection_parameter,
	Discharge_over_current_time_delay_parameter,
	Secondary_over_current_protection_parameter,
	Secondary_over_current_time_delay_parameter,
	Output_shortcut_protection_parameter,
	Output_shortcut_time_delay_parameter,
	Over_current_recover_time_delay_parameter,
	Over_current_lock_times_parameter,
	Battery_rated_capacity_parameter,
	Cell_number_serial_battery_parameter,
	Charge_current_limit_set_parameter,
	Equalization_high_temperature_prohibit_parameter,
	Equalization_low_temperature_prohibit_parameter,
	Static_equilibrium_time_parameter,
	Equalization_open_voltage_parameter,
	Equalization_open_voltage_difference_parameter,
	Equalization_stop_voltage_difference_parameter,
	Cell_failure_voltage_difference_parameter,
	Cell_failure_voltage_recover_parameter,
	Self_definition_switch_number_parameter,
	Voltage_function_switch_parameter,
	Temperature_function_switch_parameter,
	Current_function_switch_parameter,
	Capacity_and_other_function_switch,
	Equalization_function_switch_parameter,
	Indicator_function_switch_parameter,
	BMS_name
};


void RS485_Transmit_Remote_Measuring(void);
void RS485_Transmit_Remote_Signaling(void);
void RS485_Transmit_Remote_Adjusting(void);
void RS485_Remote_Set_Command(void);

void RS485_RX_FLAG_STATE(void);
uint8_t Pack_Number_Jamper_Read(void);

void RS485_Transmit_Remote_History(uint32_t Page_Num);

void Alarm_State_Update(void);

#endif /* INC_BMS_COMMUNICATION_PROTOCOL_H_ */
