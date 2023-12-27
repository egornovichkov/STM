/*
 * INA229.h
 *
 *      Author: TDM
 */

#ifndef INC_INA229_H_
#define INC_INA229_H_

#include "stdint.h"
#include "main.h"

#define LOW_BYTE(x) (x & 0xFF)
#define HIGH_BYTE(x) ((x >> 8) & 0xFF)

//*****************************************
#define INA_SPI_PORT hspi2
#define INA_SPI_CS_Pin SPI2_NSS_INA_Pin
#define INA_SPI_CS_Port SPI2_NSS_INA_GPIO_Port
#define SHUNT_RES 500 // Shunt resistance in uOhm

//#define	ADCRANGE 0x00;	// 0h = ±163.84 mV
#define	ADCRANGE 0x10;	// 1h = ± 40.96 mV
/* The user can set the MODE bits for continuous or triggered mode on bus voltage, shunt voltage or temperature measurement.*/
#define MODE 0x0F;
#define	VBUSCT 0x07; 	// Sets the conversion time of the bus voltage measurement: 0h=50 µs - 7h=4120 µs
#define VSHCT 0x07; 	// Sets the conversion time of the shunt voltage measurement: 0h=50 µs - 7h=4120 µs
#define VTCT 0x07;		// Sets the conversion time of the temperature measurement: 0h=50 µs - 7h=4120 µs
#define AVG 0x04;		// Selects ADC sample averaging count: 0h = 1 - 7h = 1024
//*****************************************

extern SPI_HandleTypeDef INA_SPI_PORT;

/* INA229 Registers Address */
#define ADR_CONFIG 0x00 		// Configuration (16bit)
#define ADR_ADC_CONFIG 0x01 	// ADC Configuration (16bit)
#define ADR_SHUNT_CAL 0x02 		// Shunt Calibration (16bit)
#define ADR_SHUNT_TEMPCO 0x03 	// Shunt Temperature Coefficient (16bit)
#define ADR_VSHUNT 0x04			// Shunt Voltage Measurement (24bit)
#define ADR_VBUS 0x05			// Bus Voltage Measurement (24bit)
#define ADR_DIETEMP 0x06		// Temperature Measurement (16bit)
#define ADR_CURRENT 0x07		// Current Result (24bit)
#define ADR_POWER 0x08			// Power Result (24bit)
#define ADR_ENERGY 0x09			// Energy Result (40bit)
#define ADR_CHARGE 0x0A			// Charge Result (40bit)
#define ADR_DIAG_ALRT 0x0B		// Diagnostic Flags and Alert (16bit)
#define ADR_SOVL 0x0C			// Shunt Overvoltage Threshold (16bit)
#define ADR_SUVL 0x0D			// Shunt Undervoltage Threshold (16bit)
#define ADR_BOVL 0x0E			// Bus Overvoltage Threshold (16bit)
#define ADR_BUVL 0x0F			// Bus Undervoltage Threshold (16bit)
#define ADR_TEMP_LIMIT 0x10		// Temperature Over-Limit Threshold (16bit)
#define ADR_PWR_LIMIT 0x11 		// Power Over-Limit Threshold (16bit)
#define ADR_MANUFACTURER_ID 0x3E// Manufacturer ID (16bit)
#define ADR_DEVICE_ID 0x3F		// Device ID (16bit)

void INA229_Init(void);
uint32_t INA229_Read_VBUS(void);
int32_t INA229_Read_VSHUNT(void);
int32_t INA229_Read_CURRENT(void);
int32_t INA229_Read_POWER(void);
int32_t INA229_Read_ENERGY(void);
int32_t INA229_Read_CHARGE(void);
uint16_t INA229_Read_DIETEMP(void);
uint16_t INA229_Read_ALERT(void);
void INA229_Write_SHUNT_CAL(uint16_t SHUNT_CAL);
void INA229_Set_Current_Charge_ALRM(uint16_t Current_Set);
void INA229_Set_Current_Discharge_ALRM(uint16_t Current_Set);

#endif /* INC_INA229_H_ */
