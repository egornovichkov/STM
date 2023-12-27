/*
 * Event_Log.h
 *
 *  Created on: Jul 10, 2023
 *      Author: TDM
 */

#ifndef INC_EVENT_LOG_H_
#define INC_EVENT_LOG_H_

#include "main.h"

void Alarm_Event_Log(void);
void Read_Clock_and_Date(void);
void Read_Measured_Param(void);
void Save_History_to_Flash(uint16_t Page_Address);
void Save_History_to_Flash(uint16_t Page_Address);
void Read_History_from_Flash(uint16_t Page_Address);
void Clear_Flash_Block(uint32_t nBlock);

#endif /* INC_EVENT_LOG_H_ */
