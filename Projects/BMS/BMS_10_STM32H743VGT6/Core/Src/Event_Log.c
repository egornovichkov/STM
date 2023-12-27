/*
 * Event_Log.c
 *
 *  Created on: Jul 10, 2023
 *      Author: TDM
 */

#include "main.h"
#include "Event_Log.h"
#include "w25qxx.h"
#include "LTC6813_BMS.h"
#include "LTC6813.h"
#include "INA229.h"

extern const uint16_t M;
extern const uint16_t N;

extern uint16_t History_Data_Num;

extern RTC_HandleTypeDef hrtc;
extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef DateToUpdate;
extern char trans_str[6];
extern uint8_t w_buf[];
extern uint8_t r_buf[];

extern uint16_t Battery_Residual_Capacity_mAh;
extern uint16_t Battery_Cycles;
extern int16_t Current_mA;
extern uint16_t Vpack_Voltage_mV;
extern int16_t Environment_temperature;
extern uint16_t Cell_Voltage_Array[18];

extern uint8_t Pack_Location;
extern uint8_t System_State_Code;
extern uint8_t Voltage_Event_Code;
extern uint8_t Temperature_Event_Code;
extern uint8_t Current_Event_Code;

uint8_t Command_type;

void Set_Clock_and_Date(void){

}

void Read_History_from_Flash(uint16_t Page_Address){
	W25qxx_ReadPage(r_buf, (uint32_t)Page_Address, 0, 64);
}

void Read_Measured_Param(void){
	for (int i = 0; i < M; i++){
		Cell_Voltage_Array[i] = LTC6813_Read_Cell(i);
		Vpack_Voltage_mV += LTC6813_Read_Cell(i);
	}
	Current_mA = INA229_Read_VSHUNT()/RSHUNT/100;
}

void Read_Clock_and_Date(void){
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN); // RTC_FORMAT_BIN , RTC_FORMAT_BCD
	HAL_RTC_GetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN);
}

 void Alarm_Event_Log(void){
	 Save_History_to_Flash(History_Data_Num++);
 }

 void Clear_Flash_Block(uint32_t nBlock){
	 W25qxx_EraseBlock(nBlock);
 }

 void Save_History_to_Flash(uint16_t Page_Address){

 	Read_Clock_and_Date();
 	Read_Measured_Param();
 //	snprintf(trans_str, 6, "%d%d%d%d%d", DateToUpdate.Month, DateToUpdate.Date, sTime.Hours, sTime.Minutes, sTime.Seconds);

 	uint8_t N = 0;
 	uint16_t year = DateToUpdate.Year + 2000;

 	w_buf[N++] = Pack_Location;
 	w_buf[N++] = Command_type;
 	w_buf[N++] = HIGH_BYTE(year);
 	w_buf[N++] = LOW_BYTE(year);
 	w_buf[N++] = DateToUpdate.Month;
 	w_buf[N++] = DateToUpdate.Date;
 	w_buf[N++] = sTime.Hours;
 	w_buf[N++] = sTime.Minutes;
 	w_buf[N++] = sTime.Seconds;
 	w_buf[N++] = System_State_Code;
 	w_buf[N++] = 3; 				// Alarm bytes number (A)
 	w_buf[N++] = Voltage_Event_Code;
 	w_buf[N++] = Temperature_Event_Code;
 	w_buf[N++] = Current_Event_Code;
 	w_buf[N++] = 25; // No. of remote measuring data(3+N+M)

 	w_buf[N++] = HIGH_BYTE(Current_mA);
 	w_buf[N++] = LOW_BYTE(Current_mA);

 	w_buf[N++] = HIGH_BYTE(Vpack_Voltage_mV);
 	w_buf[N++] = LOW_BYTE(Vpack_Voltage_mV);

 	w_buf[N++] = HIGH_BYTE(Battery_Residual_Capacity_mAh); // Остаточная емкость батареи
 	w_buf[N++] = LOW_BYTE(Battery_Residual_Capacity_mAh);

 	w_buf[N++] = 0x00;
 	w_buf[N++] = 0x19; // температура ячейки 1

 	w_buf[N++] = 0x00;
 	w_buf[N++] = 0x19; // температура ячейки 2

 	w_buf[N++] = HIGH_BYTE(Environment_temperature);
 	w_buf[N++] = LOW_BYTE(Environment_temperature); // температура окружающей среды

 	w_buf[N++] = 0x00;
 	w_buf[N++] = 0x19; // температура силовой части

 	for (int i = 0; i < M; i++){
 		w_buf[N++] = HIGH_BYTE(Cell_Voltage_Array[i]);
 		w_buf[N++] = LOW_BYTE(Cell_Voltage_Array[i]);
 	}

 	W25qxx_WritePage(w_buf, (uint32_t)Page_Address, 0, 64);
 }
























