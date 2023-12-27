/*
 * BMS_Communication_Protocol.c
 *
 *  Created on: 18 апр. 2023 г.
 *      Author: TDM
 */

#include "main.h"
#include "BMS_Communication_Protocol.h"
#include "BMS_Power_Control.h"
#include "stdlib.h"
#include "stdint.h"
#include "LTC6813_BMS.h"
#include "LTC6813.h"
#include "INA229.h"
#include "stdio.h"
#include "w25qxx.h"
#include "Event_Log.h"

#define START_INFO_BYTE 0x7E
#define STOP_INFO_BYTE 0x0D

extern const uint16_t M;
extern const uint16_t N;

extern uint8_t r_buf[];
extern int32_t Para_Array[];
extern int16_t Current_mA;
extern UART_HandleTypeDef huart2;
extern uint8_t RS485_RX_DATA[];
extern uint8_t RS485_RX_DATA_TEMP;
extern uint8_t RS485_RX_START_FRAME_FLAG;
extern uint8_t RS485_RX_STOP_FRAME_FLAG;
extern uint8_t RS485_RX_FRAME_DONE_FLAG;
extern uint8_t RS485_RX_COUNTER;

extern uint8_t Alarm_Flag;

uint8_t RS485_ASCII_DATA[INFO_FRAME_LENGTH_REMOTE_MEAS + 18];
uint8_t RS485_HEX_DATA[255];
uint16_t Hex_to_ASCII_buffer;
uint32_t RS485_Vpack;
uint16_t RS485_Cell_Voltage;
int16_t RS485_Current_mA;

uint16_t Page_Address;
uint16_t RS485_temperature;

extern uint8_t Pack_Location;
extern int16_t Alarm_Max_Temper_Cell;
extern int16_t Alarm_Min_Temper_Cell;
extern uint16_t Alarm_Charge_Over_Current;
extern uint16_t Alarm_Discharge_Over_Current;
extern uint16_t Alarm_Cell_Over_Voltage;
extern uint16_t Alarm_Cell_Under_Voltage;
extern uint16_t Alarm_Battery_Over_Voltage;
extern uint16_t Alarm_Battery_Under_Voltage;

extern uint8_t Cell_Voltage_Alarm_State[];
extern uint8_t Temperature_Alarm_State[];
extern uint8_t Current_Alarm_State ;
extern uint8_t Battery_Voltage_Alarm_State;
extern uint8_t System_State_Code;
extern uint8_t Voltage_Event_Code;
extern uint8_t Temperature_Event_Code;
extern uint8_t Current_Event_Code;

extern uint16_t Protection_Cell_Over_Voltage;
extern uint16_t Protection_Recover_Cell_Over_Voltage;
extern uint16_t Protection_Recover_Cell_Under_Voltage;
extern uint16_t Protection_Cell_Under_Voltage;
extern uint16_t Protection_Battery_Over_Voltage;
extern uint16_t Protection_Recover_Battery_Over_Voltage ;
extern uint16_t Protection_Recover_Battery_Under_Voltage;
extern uint16_t Protection_Battery_Under_Voltage;
extern uint16_t Protection_Charge_Over_Current;
extern uint16_t Protection_Discharge_Over_Current;

extern int16_t Equalization_State;
extern uint16_t Total_Battery_Capacity_mAh;
extern uint16_t Battery_Residual_Capacity_mAh;
extern uint16_t Battery_Cycles;
extern int16_t Current_mA;
extern uint16_t Vpack_Voltage_mV;
extern int16_t Environment_temperature;
extern uint16_t Cell_Voltage_Array[18];

void RS485_RX_FRAME(uint8_t *RS485_ASCII_DATA, uint8_t LENGTH) {
	HAL_GPIO_WritePin(CAN_PWR_EN_GPIO_Port, CAN_PWR_EN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(USART2_DE_GPIO_Port, USART2_DE_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit(&huart2, (uint8_t*)RS485_ASCII_DATA, LENGTH, 1000);
	HAL_GPIO_WritePin(USART2_DE_GPIO_Port, USART2_DE_Pin, GPIO_PIN_RESET);
}

void RS485_RX_FLAG_STATE(void) {
	if (RS485_RX_FRAME_DONE_FLAG == 0) {
		if (RS485_RX_DATA_TEMP == START_INFO_BYTE) {
			RS485_RX_START_FRAME_FLAG = 1;
			RS485_RX_STOP_FRAME_FLAG = 0;
		} else if (RS485_RX_DATA_TEMP == STOP_INFO_BYTE) {
			RS485_RX_STOP_FRAME_FLAG = 1;
			RS485_RX_START_FRAME_FLAG = 0;
		}

		if (RS485_RX_START_FRAME_FLAG == 1) {
			RS485_RX_DATA[RS485_RX_COUNTER++] = RS485_RX_DATA_TEMP;
		} else if (RS485_RX_STOP_FRAME_FLAG == 1) {
			RS485_RX_DATA[RS485_RX_COUNTER] = RS485_RX_DATA_TEMP;
			RS485_RX_START_FRAME_FLAG = 0;
			RS485_RX_STOP_FRAME_FLAG = 0;
			RS485_RX_FRAME_DONE_FLAG = 1;
		}

		if (RS485_RX_COUNTER >= RS485_RX_MAX_FRAME_LENGTH) {
			RS485_RX_COUNTER = 0;
			RS485_RX_START_FRAME_FLAG = 0;
			RS485_RX_STOP_FRAME_FLAG = 0;
		}
	}
}


uint16_t Count_Lchksum (uint16_t info_length) {
	uint16_t Lchsksum_temp;
	Lchsksum_temp = (info_length & 0x000F) + ((info_length>>4) & 0x000F) + ((info_length>>8) & 0x000F);
	Lchsksum_temp %= 16;
	Lchsksum_temp = ~Lchsksum_temp + 1;
	return Lchsksum_temp & 0x000F;
}

uint16_t Count_Chksum(uint8_t *buffer_ptr, uint16_t info_length){
	uint32_t chksum_temp;
	uint16_t chksum_result;
	chksum_temp = 0;

	for (uint16_t chksum_cnt = 1; chksum_cnt <= info_length + 12; chksum_cnt++){
		chksum_temp += buffer_ptr[chksum_cnt];
	}
	chksum_temp = ~chksum_temp + 1;

	chksum_result = chksum_temp & 0x0000FFFF;
	return chksum_result;
}

uint16_t Hex_to_ASCII (uint8_t Hex_Input){
	uint8_t MSB_buffer = 0;
	uint8_t LSB_buffer = 0;
	uint16_t ASCII_out = 0;

	MSB_buffer = Hex_Input >> 4;
	if (MSB_buffer < 10){
		MSB_buffer = MSB_buffer + 0x30;
	}
	else {
		MSB_buffer = MSB_buffer + 0x37;
	}

	LSB_buffer = Hex_Input & 0x0F;
	if (LSB_buffer < 10){
		LSB_buffer = LSB_buffer + 0x30;
	}
	else {
		LSB_buffer = LSB_buffer + 0x37;
	}

	ASCII_out = MSB_buffer<<8 | LSB_buffer;
	return ASCII_out;
}

uint8_t ASCII_to_Hex (uint8_t ASCII_Input){
	uint8_t HEX_out = 0;

	if (ASCII_Input >= 0x30 && ASCII_Input < 0x40){
		HEX_out = ASCII_Input - 0x30;
	}
	else if (ASCII_Input >= 0x41 && ASCII_Input < 0x47) {
		HEX_out = ASCII_Input - 0x37;
	}
	else {
		HEX_out = 0;
	}
	return HEX_out;
}


void RS485_Transmit_Remote_Measuring(void){

	  uint16_t N = 0;
	  uint16_t M = 18;

	  RS485_HEX_DATA[N++] = 0x26; // VER
	  RS485_HEX_DATA[N++] = 0x00; // ADR
	  RS485_HEX_DATA[N++] = 0x46; // CID1
	  RS485_HEX_DATA[N++] = 0x00; // CID2

	  uint16_t temp_Lchksum;
	  temp_Lchksum = Count_Lchksum(INFO_FRAME_LENGTH_REMOTE_MEAS);
	  RS485_HEX_DATA[N++] = (temp_Lchksum << 4) + (INFO_FRAME_LENGTH_REMOTE_MEAS >> 8); // LENGTH
	  RS485_HEX_DATA[N++] = INFO_FRAME_LENGTH_REMOTE_MEAS; // LENGTH

	  RS485_HEX_DATA[N++] = 0x00; 		   // DATA_FLAG
	  RS485_HEX_DATA[N++] = Pack_Location; // The pack location

	  RS485_HEX_DATA[N++] = M; // M (the number of the single battery)

	  // Cell Voltage
	  for (int i = 0; i < M; i++){
		  RS485_HEX_DATA[N++] = HIGH_BYTE(LTC6813_Read_Cell(i)/VOLTAGE_DIVIDER);
		  RS485_HEX_DATA[N++] = LOW_BYTE(LTC6813_Read_Cell(i)/VOLTAGE_DIVIDER);
	  }

	  RS485_HEX_DATA[N++] = 0x04; // количесво измерений температуры ячеек

	  RS485_temperature = LTC6813_Temp_Read(8) + 2731;
	  RS485_HEX_DATA[N++] = HIGH_BYTE(RS485_temperature);
	  RS485_HEX_DATA[N++] = LOW_BYTE(RS485_temperature); // температура ячейки 1

	  RS485_temperature = LTC6813_Temp_Read(9) + 2731;
	  RS485_HEX_DATA[N++] = HIGH_BYTE(RS485_temperature);
	  RS485_HEX_DATA[N++] = LOW_BYTE(RS485_temperature); // температура ячейки 2

	  RS485_HEX_DATA[N++] = 0x00;
	  RS485_HEX_DATA[N++] = 0x19; // температура окружающей среды

	  RS485_HEX_DATA[N++] = 0x00;
	  RS485_HEX_DATA[N++] = 0x19; // температура силовой части

//	  RS485_Current_mA = -INA229_Read_VSHUNT()/RSHUNT/100;
	  RS485_HEX_DATA[N++] = HIGH_BYTE(Current_mA/10); // Ток батареи
	  RS485_HEX_DATA[N++] = LOW_BYTE(Current_mA/10);

	  	RS485_Vpack = 0;
		for (int i = 0; i < 18; i++){
			RS485_Vpack += LTC6813_Read_Cell(i);
		}

		RS485_HEX_DATA[N++] = HIGH_BYTE(RS485_Vpack/100); 				// общее напряжение ячеек
		RS485_HEX_DATA[N++] = LOW_BYTE(RS485_Vpack/100);
		RS485_HEX_DATA[N++] = HIGH_BYTE(Battery_Residual_Capacity_mAh/10);	// Остаточная емкость батареи
		RS485_HEX_DATA[N++] = LOW_BYTE(Battery_Residual_Capacity_mAh/10);
		RS485_HEX_DATA[N++] = 0x00; 									// Self-define the number of remote measuring data
		RS485_HEX_DATA[N++] = HIGH_BYTE(Total_Battery_Capacity_mAh/10); 	// Total capacity
		RS485_HEX_DATA[N++] = LOW_BYTE(Total_Battery_Capacity_mAh/10);
		RS485_HEX_DATA[N++] = HIGH_BYTE(Battery_Cycles);			// Number of Battery cycles
		RS485_HEX_DATA[N++] = LOW_BYTE(Battery_Cycles);

		RS485_HEX_DATA[N++] = 0x00;	// CHKSUM
		RS485_HEX_DATA[N] = 0x00;	// CHKSUM

		N = 0;
		RS485_ASCII_DATA[N++] = 0x7E; // SOI
		for (int M = 0; M < INFO_FRAME_LENGTH_REMOTE_MEAS/2 + 8; M++){
			Hex_to_ASCII_buffer = Hex_to_ASCII(RS485_HEX_DATA[M]);
			RS485_ASCII_DATA[N++] = Hex_to_ASCII_buffer >> 8;
			RS485_ASCII_DATA[N++] = Hex_to_ASCII_buffer;
		}
		RS485_ASCII_DATA[N++] = 0x0D;	// EOI

		uint16_t Temp_Chksum = 0;
		Temp_Chksum = Count_Chksum((uint8_t*)RS485_ASCII_DATA, INFO_FRAME_LENGTH_REMOTE_MEAS);

		Hex_to_ASCII_buffer = Hex_to_ASCII((uint8_t)(Temp_Chksum>>8));
		RS485_ASCII_DATA[INFO_FRAME_LENGTH_REMOTE_MEAS + 13] = Hex_to_ASCII_buffer >> 8;
		RS485_ASCII_DATA[INFO_FRAME_LENGTH_REMOTE_MEAS + 14] = Hex_to_ASCII_buffer;
		Hex_to_ASCII_buffer = Hex_to_ASCII((uint8_t)(Temp_Chksum));
		RS485_ASCII_DATA[INFO_FRAME_LENGTH_REMOTE_MEAS + 15] = Hex_to_ASCII_buffer >> 8;
		RS485_ASCII_DATA[INFO_FRAME_LENGTH_REMOTE_MEAS + 16] = Hex_to_ASCII_buffer;

		  HAL_GPIO_WritePin(CAN_PWR_EN_GPIO_Port, CAN_PWR_EN_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(USART2_DE_GPIO_Port, USART2_DE_Pin, GPIO_PIN_SET);
		  HAL_UART_Transmit(&huart2, (uint8_t*)RS485_ASCII_DATA, INFO_FRAME_LENGTH_REMOTE_MEAS + 18, 1000);
		  HAL_GPIO_WritePin(USART2_DE_GPIO_Port, USART2_DE_Pin, GPIO_PIN_RESET);
}

void RS485_Transmit_Remote_Signaling(void){
	uint16_t Num_Bit = 0;

	RS485_HEX_DATA[Num_Bit++] = 0x26; // VER
	RS485_HEX_DATA[Num_Bit++] = 0x00; // ADR
	RS485_HEX_DATA[Num_Bit++] = 0x46; // CID1
	RS485_HEX_DATA[Num_Bit++] = 0x00; // CID2

	uint16_t temp_Lchksum;
	temp_Lchksum = Count_Lchksum(INFO_FRAME_LENGTH_REMOTE_SIGNALING);
	RS485_HEX_DATA[Num_Bit++] = (temp_Lchksum << 4) + (INFO_FRAME_LENGTH_REMOTE_SIGNALING >> 8); 	// LENGTH
	RS485_HEX_DATA[Num_Bit++] = INFO_FRAME_LENGTH_REMOTE_SIGNALING; 								// LENGTH

	RS485_HEX_DATA[Num_Bit++] = 0x00; 		   	// DATA_FLAG
	RS485_HEX_DATA[Num_Bit++] = Pack_Location; 	// The pack location

	RS485_HEX_DATA[Num_Bit++] = M; // M (the number of the single battery)

	// Cell Voltage Alarm
	for (int i = 0; i < M; i++){
		RS485_HEX_DATA[Num_Bit++] = Cell_Voltage_Alarm_State[i];
	}

	RS485_HEX_DATA[Num_Bit++] = N; // количество измерений температуры
	for (int i = 0; i < N; i++){
		RS485_HEX_DATA[Num_Bit++] = Temperature_Alarm_State[i];
	}

	RS485_HEX_DATA[Num_Bit++] = Current_Alarm_State;
	RS485_HEX_DATA[Num_Bit++] = Battery_Voltage_Alarm_State;
	RS485_HEX_DATA[Num_Bit++] = 0x07; // the number of self-define alarm

	RS485_HEX_DATA[Num_Bit++] = 0x00;
	RS485_HEX_DATA[Num_Bit++] = 0x00;
	RS485_HEX_DATA[Num_Bit++] = 0x00;
	RS485_HEX_DATA[Num_Bit++] = 0x00;
	RS485_HEX_DATA[Num_Bit++] = 0x00;
	RS485_HEX_DATA[Num_Bit++] = 0x00;
	RS485_HEX_DATA[Num_Bit++] = 0x00;
	  if (Current_mA < 0){
		  RS485_HEX_DATA[Num_Bit++] = 0x01;
	  }
	  else if (Current_mA > 0){
		  RS485_HEX_DATA[Num_Bit++] = 0x02;
	  }
	RS485_HEX_DATA[Num_Bit++] = 0x00;	// CHKSUM
	RS485_HEX_DATA[Num_Bit] = 0x00;	// CHKSUM

	Num_Bit = 0;
	RS485_ASCII_DATA[Num_Bit++] = 0x7E; // SOI
	for (int M = 0; M < INFO_FRAME_LENGTH_REMOTE_SIGNALING / 2 + 8; M++) {
		Hex_to_ASCII_buffer = Hex_to_ASCII(RS485_HEX_DATA[M]);
		RS485_ASCII_DATA[Num_Bit++] = Hex_to_ASCII_buffer >> 8;
		RS485_ASCII_DATA[Num_Bit++] = Hex_to_ASCII_buffer;
	}
	RS485_ASCII_DATA[Num_Bit++] = 0x0D;	// EOI

	uint16_t Temp_Chksum = 0;
	Temp_Chksum = Count_Chksum((uint8_t*) RS485_ASCII_DATA, INFO_FRAME_LENGTH_REMOTE_SIGNALING);

	Hex_to_ASCII_buffer = Hex_to_ASCII((uint8_t) (Temp_Chksum >> 8));
	RS485_ASCII_DATA[INFO_FRAME_LENGTH_REMOTE_SIGNALING + 13] = Hex_to_ASCII_buffer >> 8;
	RS485_ASCII_DATA[INFO_FRAME_LENGTH_REMOTE_SIGNALING + 14] = Hex_to_ASCII_buffer;
	Hex_to_ASCII_buffer = Hex_to_ASCII((uint8_t) (Temp_Chksum));
	RS485_ASCII_DATA[INFO_FRAME_LENGTH_REMOTE_SIGNALING + 15] = Hex_to_ASCII_buffer >> 8;
	RS485_ASCII_DATA[INFO_FRAME_LENGTH_REMOTE_SIGNALING + 16] = Hex_to_ASCII_buffer;

	HAL_GPIO_WritePin(CAN_PWR_EN_GPIO_Port, CAN_PWR_EN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(USART2_DE_GPIO_Port, USART2_DE_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit(&huart2, (uint8_t*) RS485_ASCII_DATA, INFO_FRAME_LENGTH_REMOTE_SIGNALING + 18, 1000);
	HAL_GPIO_WritePin(USART2_DE_GPIO_Port, USART2_DE_Pin, GPIO_PIN_RESET);
}

void RS485_Transmit_Remote_Adjusting(void){
	uint16_t N = 0;
	uint16_t P_num = 0;
	uint16_t temp_Lchksum;

	RS485_HEX_DATA[N++] = 0x26; // VER
	RS485_HEX_DATA[N++] = 0x00; // ADR
	RS485_HEX_DATA[N++] = 0x46; // CID1
	RS485_HEX_DATA[N++] = 0x00; // CID2

	temp_Lchksum = Count_Lchksum(INFO_FRAME_LENGTH_REMOTE_ADJUST);
	RS485_HEX_DATA[N++] = (temp_Lchksum << 4) + (INFO_FRAME_LENGTH_REMOTE_ADJUST >> 8); // LENGTH
	RS485_HEX_DATA[N++] = INFO_FRAME_LENGTH_REMOTE_ADJUST; 			// LENGTH


	RS485_HEX_DATA[N++] = (uint8_t)Para_Array[P_num++]; 				// The pack location

	RS485_HEX_DATA[N++] = HIGH_BYTE((uint16_t)Para_Array[P_num]);		// Cell high voltage alarm parameter
	RS485_HEX_DATA[N++] = LOW_BYTE((uint16_t)Para_Array[P_num++]);
	RS485_HEX_DATA[N++] = HIGH_BYTE((uint16_t)Para_Array[P_num]);		// Cell low voltage alarm parameter
	RS485_HEX_DATA[N++] = LOW_BYTE((uint16_t)Para_Array[P_num++]);
	RS485_HEX_DATA[N++] = HIGH_BYTE((uint16_t)Para_Array[P_num]);		// Cell high temperature alarm parameter
	RS485_HEX_DATA[N++] = LOW_BYTE((uint16_t)Para_Array[P_num++]);
	RS485_HEX_DATA[N++] = HIGH_BYTE((uint16_t)Para_Array[P_num]);		// Cell low temperature alarm parameter
	RS485_HEX_DATA[N++] = LOW_BYTE((uint16_t)Para_Array[P_num++]);
	RS485_HEX_DATA[N++] = HIGH_BYTE((uint16_t)Para_Array[P_num]);		// Charge over current alarm parameter
	RS485_HEX_DATA[N++] = LOW_BYTE((uint16_t)Para_Array[P_num++]);
	RS485_HEX_DATA[N++] = HIGH_BYTE((uint16_t)Para_Array[P_num]);		// Battery high voltage alarm parameter
	RS485_HEX_DATA[N++] = LOW_BYTE((uint16_t)Para_Array[P_num++]);
	RS485_HEX_DATA[N++] = HIGH_BYTE((uint16_t)Para_Array[P_num]);		// Battery low voltage alarm parameter
	RS485_HEX_DATA[N++] = LOW_BYTE((uint16_t)Para_Array[P_num++]);

// *******************************************************************************************************

	RS485_HEX_DATA[N++] = 12;											// Self-definition parameter number
	P_num++;
	RS485_HEX_DATA[N++] = HIGH_BYTE((uint16_t)Para_Array[P_num]);		// Discharge over current alarm parameter
	RS485_HEX_DATA[N++] = LOW_BYTE((uint16_t)Para_Array[P_num++]);
	RS485_HEX_DATA[N++] = HIGH_BYTE((uint16_t)Para_Array[P_num]);		// Cell over voltage protection parameter
	RS485_HEX_DATA[N++] = LOW_BYTE((uint16_t)Para_Array[P_num++]);
	RS485_HEX_DATA[N++] = HIGH_BYTE((uint16_t)Para_Array[P_num]);		// Cell over voltage recover parameter
	RS485_HEX_DATA[N++] = LOW_BYTE((uint16_t)Para_Array[P_num++]);
	RS485_HEX_DATA[N++] = HIGH_BYTE((uint16_t)Para_Array[P_num]);		// Cell under voltage recover parameter
	RS485_HEX_DATA[N++] = LOW_BYTE((uint16_t)Para_Array[P_num++]);
	RS485_HEX_DATA[N++] = HIGH_BYTE((uint16_t)Para_Array[P_num]);		// Cell under voltage protection parameter
	RS485_HEX_DATA[N++] = LOW_BYTE((uint16_t)Para_Array[P_num++]);
	RS485_HEX_DATA[N++] = HIGH_BYTE((uint16_t)Para_Array[P_num]); 		// Battery over voltage protection parameter
	RS485_HEX_DATA[N++] = LOW_BYTE((uint16_t)Para_Array[P_num++]);
	RS485_HEX_DATA[N++] = HIGH_BYTE((uint16_t)Para_Array[P_num]);		// Battery over voltage recover parameter
	RS485_HEX_DATA[N++] = LOW_BYTE((uint16_t)Para_Array[P_num++]);
	RS485_HEX_DATA[N++] = HIGH_BYTE((uint16_t)Para_Array[P_num]);		// Battery under voltage recover parameter
	RS485_HEX_DATA[N++] = LOW_BYTE((uint16_t)Para_Array[P_num++]);
	RS485_HEX_DATA[N++] = HIGH_BYTE((uint16_t)Para_Array[P_num]);		// Battery under voltage protection parameter
	RS485_HEX_DATA[N++] = LOW_BYTE((uint16_t)Para_Array[P_num++]);
	RS485_HEX_DATA[N++] = HIGH_BYTE((uint16_t)Para_Array[Charge_over_current_protection_parameter]);			// Charge over current protection parameter
	RS485_HEX_DATA[N++] = LOW_BYTE((uint16_t)Para_Array[Charge_over_current_protection_parameter]);
	RS485_HEX_DATA[N++] = HIGH_BYTE((uint16_t)Para_Array[Discharge_over_current_protection_parameter]);  		// Discharge over current protection parameter
	RS485_HEX_DATA[N++] = LOW_BYTE((uint16_t)Para_Array[Discharge_over_current_protection_parameter]);
	RS485_HEX_DATA[N++] = HIGH_BYTE((uint16_t)Para_Array[Discharge_over_current_time_delay_parameter]);  		// Discharge over current time delay parameter
	RS485_HEX_DATA[N++] = LOW_BYTE((uint16_t)Para_Array[Discharge_over_current_time_delay_parameter]);
			 // Charge over temperature protection parameter
			 // Charge over temperature recover parameter
			 // Charge under temperature recover parameter
			 // Charge under temperature protection parameter
			 // Discharge over temperature protection parameter
			 // Discharge over temperature recover parameter //
			 // Discharge under temperature recover parameter
			 // Discharge under temperature protection parameter
			 // High environment temperature alarm parameter
			 // Low environment temperature alarm parameter
			 // Over environment temperature protection parameter
			 // Over environment temperature recover parameter
			 // Under environment temperature protection parameter
			 // Under environment temperature recover parameter
			 // Cell heating temperature open parameter
			 // Cell heating temperature stop parameter
			 // High power temperature alarm parameter
			 // Low power temperature alarm parameter
			 // Over power temperature protection parameter
			 // Over power temperature recover parameter
			 // Under power temperature recover parameter
			 // Under power temperature protection parameter
			 // Charge over current protection parameter
			 // Charge over current time delay parameter
			 // Discharge over current protection parameter
			 // Discharge over current time delay parameter
			 // Secondary over current protection parameter
			 // Secondary over current time delay parameter
			 // Output shortcut protection parameter
			 // Output shortcut time delay parameter
			 // Over current recover time delay parameter
			 // Over current lock times parameter
			 // Battery rated capacity parameter
			 // Cell number serial battery parameter
			 // Charge current limit set parameter
			 // Equalization high temperature prohibit parameter
			 // Equalization low temperature prohibit parameter
			 // Static equilibrium time parameter
			 // Equalization open voltage parameter
			 // Equalization open voltage difference parameter
			 // Equalization stop voltage difference parameter
			 // Cell failure voltage difference parameter
			 // Cell failure voltage recover parameter
			 // Self-definition switch number parameter
			 // Voltage function switch parameter //
			 // Temperature function switch parameter
			 // Current function switch parameter
			 // Capacity and other function switch
			 // Equalization function switch parameter
			 // Indicator function switch parameter
			 // BMS name

	RS485_HEX_DATA[N++] = 0x00;	// CHKSUM
	RS485_HEX_DATA[N] = 0x00;	// CHKSUM

	N = 0;
	RS485_ASCII_DATA[N++] = 0x7E; // SOI
	for (int M = 0; M < INFO_FRAME_LENGTH_REMOTE_ADJUST / 2 + 8; M++) {
		Hex_to_ASCII_buffer = Hex_to_ASCII(RS485_HEX_DATA[M]);
		RS485_ASCII_DATA[N++] = Hex_to_ASCII_buffer >> 8;
		RS485_ASCII_DATA[N++] = Hex_to_ASCII_buffer;
	}
	RS485_ASCII_DATA[N++] = 0x0D; // EOI

	uint16_t Temp_Chksum = 0;
	Temp_Chksum = Count_Chksum((uint8_t*) RS485_ASCII_DATA, INFO_FRAME_LENGTH_REMOTE_ADJUST);

	Hex_to_ASCII_buffer = Hex_to_ASCII((uint8_t) (Temp_Chksum >> 8));
	RS485_ASCII_DATA[INFO_FRAME_LENGTH_REMOTE_ADJUST + 13] = Hex_to_ASCII_buffer >> 8;
	RS485_ASCII_DATA[INFO_FRAME_LENGTH_REMOTE_ADJUST + 14] = Hex_to_ASCII_buffer;
	Hex_to_ASCII_buffer = Hex_to_ASCII((uint8_t)(Temp_Chksum));
	RS485_ASCII_DATA[INFO_FRAME_LENGTH_REMOTE_ADJUST + 15] = Hex_to_ASCII_buffer >> 8;
	RS485_ASCII_DATA[INFO_FRAME_LENGTH_REMOTE_ADJUST + 16] = Hex_to_ASCII_buffer;

	HAL_GPIO_WritePin(CAN_PWR_EN_GPIO_Port, CAN_PWR_EN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(USART2_DE_GPIO_Port, USART2_DE_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit(&huart2, (uint8_t*) RS485_ASCII_DATA, INFO_FRAME_LENGTH_REMOTE_ADJUST + 18, 1000);
	HAL_GPIO_WritePin(USART2_DE_GPIO_Port, USART2_DE_Pin, GPIO_PIN_RESET);
}

void RS485_Remote_Set_Command(void){

// Response message with the remote set command
	uint8_t RS485_REMOTE_SET_RESPONSE_OK_ASCII[] = {
			0x7E, 0x32, 0x36, 0x30, 0x30, 0x34, 0x36, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x46, 0x44, 0x41, 0x45, 0x0D};
	uint8_t RS485_REMOTE_SET_RESPONSE_CMD_ERROR_ASCII[] = {
			0x7E, 0x32, 0x36, 0x30, 0x30, 0x34, 0x36, 0x30, 0x34, 0x30, 0x30, 0x30, 0x30, 0x46, 0x44, 0x41, 0x41, 0x0D};

	uint8_t Remote_Command = (ASCII_to_Hex(RS485_RX_DATA[COMMAND_TYPE_ADDRES])<<4) + ASCII_to_Hex(RS485_RX_DATA[COMMAND_TYPE_ADDRES+1]);
	uint16_t Remote_Data = (ASCII_to_Hex(RS485_RX_DATA[COMMAND_DATA_ADDRES])<<12) + (ASCII_to_Hex(RS485_RX_DATA[COMMAND_DATA_ADDRES+1])<<8)
			+ (ASCII_to_Hex(RS485_RX_DATA[COMMAND_DATA_ADDRES+2])<<4) + (ASCII_to_Hex(RS485_RX_DATA[COMMAND_DATA_ADDRES+3]));

	switch (Remote_Command) {
// The pack location
	case 1:
		Para_Array[The_pack_location] = Remote_Data;
		RS485_RX_FRAME(RS485_REMOTE_SET_RESPONSE_OK_ASCII, sizeof(RS485_REMOTE_SET_RESPONSE_OK_ASCII));
		break;
// Cell high voltage alarm parameter
	case 2:
		Para_Array[Cell_high_voltage_alarm_parameter] = Remote_Data;
		RS485_RX_FRAME(RS485_REMOTE_SET_RESPONSE_OK_ASCII, sizeof(RS485_REMOTE_SET_RESPONSE_OK_ASCII));
		break;
// Cell low voltage alarm parameter
	case 3:
		Para_Array[Cell_low_voltage_alarm_parameter] = Remote_Data;
		RS485_RX_FRAME(RS485_REMOTE_SET_RESPONSE_OK_ASCII, sizeof(RS485_REMOTE_SET_RESPONSE_OK_ASCII));
		break;
// Cell high temperature alarm parameter
	case 4:
		Para_Array[Cell_high_temperature_alarm_parameter] = Remote_Data;
		RS485_RX_FRAME(RS485_REMOTE_SET_RESPONSE_OK_ASCII, sizeof(RS485_REMOTE_SET_RESPONSE_OK_ASCII));
		break;
// Cell Low temperature alarm parameter
	case 5:
		Para_Array[Cell_low_temperature_alarm_parameter] = Remote_Data;
		RS485_RX_FRAME(RS485_REMOTE_SET_RESPONSE_OK_ASCII, sizeof(RS485_REMOTE_SET_RESPONSE_OK_ASCII));
		break;
// Charge over current alarm parameter
	case 6:
		Para_Array[Charge_over_current_alarm_parameter] = Remote_Data;
		RS485_RX_FRAME(RS485_REMOTE_SET_RESPONSE_OK_ASCII, sizeof(RS485_REMOTE_SET_RESPONSE_OK_ASCII));
		break;
// Battery high voltage alarm parameter
	case 7:
		Para_Array[Battery_high_voltage_alarm_parameter] = Remote_Data;
		RS485_RX_FRAME(RS485_REMOTE_SET_RESPONSE_OK_ASCII, sizeof(RS485_REMOTE_SET_RESPONSE_OK_ASCII));
		break;
// Battery low voltage alarm parameter
	case 8:
		Para_Array[Battery_low_voltage_alarm_parameter] = Remote_Data;
		RS485_RX_FRAME(RS485_REMOTE_SET_RESPONSE_OK_ASCII, sizeof(RS485_REMOTE_SET_RESPONSE_OK_ASCII));
		break;
// Discharge over current alarm parameter
	case 10:
		Para_Array[Discharge_over_current_alarm_parameter] = Remote_Data;
		RS485_RX_FRAME(RS485_REMOTE_SET_RESPONSE_OK_ASCII, sizeof(RS485_REMOTE_SET_RESPONSE_OK_ASCII));
		break;
// Cell over voltage protection parameter
	case 11:
		Para_Array[Cell_over_voltage_protection_parameter] = Remote_Data;
		RS485_RX_FRAME(RS485_REMOTE_SET_RESPONSE_OK_ASCII, sizeof(RS485_REMOTE_SET_RESPONSE_OK_ASCII));
		break;
// Cell over voltage recover parameter
	case 12:
		Para_Array[Cell_over_voltage_recover_parameter] = Remote_Data;
		RS485_RX_FRAME(RS485_REMOTE_SET_RESPONSE_OK_ASCII, sizeof(RS485_REMOTE_SET_RESPONSE_OK_ASCII));
		break;
// Cell under voltage recover parameter
	case 13:
		Para_Array[Cell_under_voltage_recover_parameter] = Remote_Data;
		RS485_RX_FRAME(RS485_REMOTE_SET_RESPONSE_OK_ASCII, sizeof(RS485_REMOTE_SET_RESPONSE_OK_ASCII));
		break;
// Cell under voltage protection parameter
	case 14:
		Para_Array[Cell_under_voltage_protection_parameter] = Remote_Data;
		RS485_RX_FRAME(RS485_REMOTE_SET_RESPONSE_OK_ASCII, sizeof(RS485_REMOTE_SET_RESPONSE_OK_ASCII));
		break;
// Battery over voltage protection parameter
	case 15:
		Para_Array[Battery_over_voltage_protection_parameter] = Remote_Data;
		RS485_RX_FRAME(RS485_REMOTE_SET_RESPONSE_OK_ASCII, sizeof(RS485_REMOTE_SET_RESPONSE_OK_ASCII));
		break;
// Battery over voltage recover parameter
	case 16:
		Para_Array[Battery_over_voltage_recover_parameter] = Remote_Data;
		RS485_RX_FRAME(RS485_REMOTE_SET_RESPONSE_OK_ASCII, sizeof(RS485_REMOTE_SET_RESPONSE_OK_ASCII));
		break;
// Battery under voltage recover parameter
	case 17:
		Para_Array[Battery_under_voltage_recover_parameter] = Remote_Data;
		RS485_RX_FRAME(RS485_REMOTE_SET_RESPONSE_OK_ASCII, sizeof(RS485_REMOTE_SET_RESPONSE_OK_ASCII));
		break;
// Battery under voltage protection parameter
	case 18:
		Para_Array[Battery_under_voltage_protection_parameter] = Remote_Data;
		RS485_RX_FRAME(RS485_REMOTE_SET_RESPONSE_OK_ASCII, sizeof(RS485_REMOTE_SET_RESPONSE_OK_ASCII));
		break;
// Charge over current protection parameter
	case 41:
		Para_Array[Charge_over_current_protection_parameter] = Remote_Data;
		RS485_RX_FRAME(RS485_REMOTE_SET_RESPONSE_OK_ASCII, sizeof(RS485_REMOTE_SET_RESPONSE_OK_ASCII));
		break;
// Discharge over current protection parameter
	case 43:
		Para_Array[Discharge_over_current_protection_parameter] = Remote_Data;
		RS485_RX_FRAME(RS485_REMOTE_SET_RESPONSE_OK_ASCII, sizeof(RS485_REMOTE_SET_RESPONSE_OK_ASCII));
		break;
// Discharge over current time delay parameter
	case 44:
		Para_Array[Discharge_over_current_time_delay_parameter] = Remote_Data;
		RS485_RX_FRAME(RS485_REMOTE_SET_RESPONSE_OK_ASCII, sizeof(RS485_REMOTE_SET_RESPONSE_OK_ASCII));
		break;
// Equalization state bit 0-18
	case 67:
//		Equalization_State = Remote_Data;
//		LTC6813_Equalization(Equalization_State);
//		RS485_RX_FRAME(RS485_REMOTE_SET_RESPONSE_OK_ASCII, sizeof(RS485_REMOTE_SET_RESPONSE_OK_ASCII));
		break;

	default:
		RS485_RX_FRAME(RS485_REMOTE_SET_RESPONSE_CMD_ERROR_ASCII, sizeof(RS485_REMOTE_SET_RESPONSE_CMD_ERROR_ASCII));
		break;
	}
}

uint8_t Pack_Number_Jamper_Read(void){
	uint8_t Pack_Number = 0;
	  if (HAL_GPIO_ReadPin(LED_GREEN_OK_GPIO_Port, LED_GREEN_OK_Pin) == GPIO_PIN_RESET ) {
		  Pack_Number += 1;
	  }
	  if (HAL_GPIO_ReadPin(LED_YELLOW_DISCHARGE_GPIO_Port, LED_YELLOW_DISCHARGE_Pin) == GPIO_PIN_RESET ) {
		  Pack_Number += 2;
	  }
	  if (HAL_GPIO_ReadPin(LED_YELLOW_CHARGE_GPIO_Port, LED_YELLOW_CHARGE_Pin) == GPIO_PIN_RESET ) {
		  Pack_Number += 4;
	  }
	  if (HAL_GPIO_ReadPin(LED_RED_ALARM_GPIO_Port, LED_RED_ALARM_Pin) == GPIO_PIN_RESET ) {
		  Pack_Number += 8;
	  }
	  return Pack_Number;
}

void RS485_Transmit_Remote_History(uint32_t Page_Num){
	  uint16_t N = 0;
//	  uint16_t M = 18;
	  Read_History_from_Flash(Page_Num);

	  RS485_HEX_DATA[N++] = 0x26; // VER
	  RS485_HEX_DATA[N++] = 0x00; // ADR
	  RS485_HEX_DATA[N++] = 0x46; // CID1
	  RS485_HEX_DATA[N++] = 0x00; // CID2

	  uint16_t temp_Lchksum;
	  temp_Lchksum = Count_Lchksum(INFO_FRAME_LENGTH_REMOTE_HISTORY);
	  RS485_HEX_DATA[N++] = (temp_Lchksum << 4) + (INFO_FRAME_LENGTH_REMOTE_HISTORY >> 8); // LENGTH
	  RS485_HEX_DATA[N++] = INFO_FRAME_LENGTH_REMOTE_HISTORY;

	  for (int i = 0; i < INFO_FRAME_LENGTH_REMOTE_HISTORY/2; i++){
		  RS485_HEX_DATA[N++] = r_buf[i];
	  }

	  RS485_HEX_DATA[N++] = 0x00;	// CHKSUM
	  RS485_HEX_DATA[N] = 0x00;		// CHKSUM

	  N = 0;
	  RS485_ASCII_DATA[N++] = 0x7E; // SOI
	  for (int M = 0; M < INFO_FRAME_LENGTH_REMOTE_HISTORY/2 + 8; M++){
			Hex_to_ASCII_buffer = Hex_to_ASCII(RS485_HEX_DATA[M]);
			RS485_ASCII_DATA[N++] = Hex_to_ASCII_buffer >> 8;
			RS485_ASCII_DATA[N++] = Hex_to_ASCII_buffer;
	  }
	  RS485_ASCII_DATA[N++] = 0x0D;	// EOI

	  uint16_t Temp_Chksum = 0;
	  Temp_Chksum = Count_Chksum((uint8_t*)RS485_ASCII_DATA, INFO_FRAME_LENGTH_REMOTE_HISTORY);

	  Hex_to_ASCII_buffer = Hex_to_ASCII((uint8_t)(Temp_Chksum>>8));
	  RS485_ASCII_DATA[INFO_FRAME_LENGTH_REMOTE_HISTORY + 13] = Hex_to_ASCII_buffer >> 8;
	  RS485_ASCII_DATA[INFO_FRAME_LENGTH_REMOTE_HISTORY + 14] = Hex_to_ASCII_buffer;
	  Hex_to_ASCII_buffer = Hex_to_ASCII((uint8_t)(Temp_Chksum));
	  RS485_ASCII_DATA[INFO_FRAME_LENGTH_REMOTE_HISTORY + 15] = Hex_to_ASCII_buffer >> 8;
	  RS485_ASCII_DATA[INFO_FRAME_LENGTH_REMOTE_HISTORY + 16] = Hex_to_ASCII_buffer;

	  HAL_GPIO_WritePin(CAN_PWR_EN_GPIO_Port, CAN_PWR_EN_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(USART2_DE_GPIO_Port, USART2_DE_Pin, GPIO_PIN_SET);
	  HAL_UART_Transmit(&huart2, (uint8_t*)RS485_ASCII_DATA, INFO_FRAME_LENGTH_REMOTE_HISTORY + 18, 1000);
	  HAL_GPIO_WritePin(USART2_DE_GPIO_Port, USART2_DE_Pin, GPIO_PIN_RESET);
}


void Alarm_State_Update(void){

	  for (int i = 0; i < 16; i++){
		  if(LTC6813_Read_Cell(i)/VOLTAGE_DIVIDER > Para_Array[Cell_over_voltage_protection_parameter]){
			  Alarm_Flag = Alarm_Flag | 0x10;
			  Cell_Voltage_Alarm_State[i] = 0x01;
		  }
		  else if (LTC6813_Read_Cell(i)/VOLTAGE_DIVIDER > Para_Array[Cell_high_voltage_alarm_parameter]){
			  Cell_Voltage_Alarm_State[i] = 0x01;
		  }
		  else {
			  Cell_Voltage_Alarm_State[i] = Cell_Voltage_Alarm_State[i] & 0xFE;
		  }
		  if (LTC6813_Read_Cell(i)/VOLTAGE_DIVIDER <= Para_Array[Cell_over_voltage_recover_parameter]){
			  Alarm_Flag = Alarm_Flag & 0xEF;
		  }

		  if(LTC6813_Read_Cell(i)/VOLTAGE_DIVIDER < Para_Array[Cell_under_voltage_protection_parameter]){
			  Alarm_Flag = Alarm_Flag | 0x20;
			  Cell_Voltage_Alarm_State[i] = 0x02;
		  }
		  else if(LTC6813_Read_Cell(i)/VOLTAGE_DIVIDER < Para_Array[Cell_low_voltage_alarm_parameter]){
			  Cell_Voltage_Alarm_State[i] = 0x02;
		  }
		  else {
			  Cell_Voltage_Alarm_State[i] = Cell_Voltage_Alarm_State[i] & 0xFD;
		  }
		  if (LTC6813_Read_Cell(i)/VOLTAGE_DIVIDER >= Para_Array[Cell_under_voltage_recover_parameter]){
			  Alarm_Flag = Alarm_Flag & 0xDF;
		  }
	  }

	  for (int i = 0; i < 3; i++){
		  if (LTC6813_Temp_Read(i)/10 + 273 > Para_Array[Cell_high_temperature_alarm_parameter]){
			  Temperature_Alarm_State[i] = 0x01;
		  }
		  else if(LTC6813_Temp_Read(i)/10 + 273 < Para_Array[Cell_low_temperature_alarm_parameter]){
			  Temperature_Alarm_State[i] = 0x02;
		  }
		  else {
			  Temperature_Alarm_State[i] = 0x00;
		  }
	  }

	  	RS485_Vpack = 0;
		for (int i = 0; i < 18; i++){
			RS485_Vpack += LTC6813_Read_Cell(i)/10;
		}

	  if (RS485_Vpack > Para_Array[Battery_high_voltage_alarm_parameter]){
		  Battery_Voltage_Alarm_State = 0x01;
	  }
	  else if(RS485_Vpack < Para_Array[Battery_low_voltage_alarm_parameter]){
		  Battery_Voltage_Alarm_State = 0x02;
	  }
	  else {
		  Battery_Voltage_Alarm_State = 0x00;
	  }

	  if (Current_mA > Para_Array[Charge_over_current_alarm_parameter] * 1000){
		  Current_Alarm_State = 0x01;
	  }
	  else if(Current_mA < -Para_Array[Discharge_over_current_alarm_parameter] * 1000){
		  Current_Alarm_State = 0x02;
	  }
	  else {
		  Current_Alarm_State = 0x00;
	  }
}

















