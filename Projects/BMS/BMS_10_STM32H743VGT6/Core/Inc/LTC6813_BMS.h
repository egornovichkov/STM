/*
 * LTC6813_BMS.h
 *
 *  Created on: May 19, 2023
 *      Author: TDM
 */
#include "stdint.h"
#include "LTC6813.h"

#ifndef INC_LTC6813_BMS_H_
#define INC_LTC6813_BMS_H_

#define LOW_BYTE(x) (x & 0xFF)
#define HIGH_BYTE(x) ((x >> 8) & 0xFF)

#define TOTAL_IC_1 1
#define RSHUNT 5 // R1 in 0,1 mOhm

void LTC6813_Init(void);
int16_t LTC6813_Temp_Read(uint8_t Channel);
uint16_t LTC6813_Read_Cell(uint8_t Cell);
void LTC6813_Equalization(int16_t dccBits);
int16_t LTC6813_Chk_Start_Error(void);
void LTC6813_Chk_Cell_Error(void);

#endif /* INC_LTC6813_BMS_H_ */
