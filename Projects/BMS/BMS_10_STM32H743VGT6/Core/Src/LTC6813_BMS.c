/*
 * LTC6813_BMS.c
 *
 *  Created on: May 19, 2023
 *      Author: TDM
 */
#include "LTC6813_BMS.h"
#include "stdint.h"
#include "main.h"
#include "INA229.h"
#include "stdlib.h"

extern SPI_HandleTypeDef hspi2;
extern int32_t Para_Array[];

const uint8_t TOTAL_IC = 1;			//!<number of ICs in the daisy chain
cell_asic bms_ic[TOTAL_IC_1]; 		//!< Global Battery Variable
uint8_t current_ic = 0;
int8_t ERR;


/* R1 resistance */
#define NTC_UP_R 10000.0f
/* constants of Steinhart-Hart equation */
#define A 0.001031f
#define B 0.0002521f
#define C 0.0000000018f

int32_t Vshunt_Voltage;
int16_t Temperature_Gradus;
int32_t Temperature_Code;

float Ntc_Tmp = 0;
float Ntc_R;

/*************************************************************************
 ADC Command Configurations. See LTC681x.h for options
**************************************************************************/
const uint8_t ADC_OPT = ADC_OPT_DISABLED; //!< ADC Mode option bit
const uint8_t ADC_CONVERSION_MODE = MD_7KHZ_3KHZ; //!< ADC Mode
const uint8_t ADC_DCP = DCP_DISABLED; //!< Discharge Permitted
const uint8_t CELL_CH_TO_CONVERT = CELL_CH_ALL; //!< Channel Selection for ADC conversion
uint8_t AUX_CH_TO_CONVERT = AUX_CH_ALL; //!< Channel Selection for ADC conversion
const uint8_t STAT_CH_TO_CONVERT = STAT_CH_ALL; //!< Channel Selection for ADC conversion
uint8_t NO_OF_REG = REG_ALL; //!< Register Selection

//Under Voltage and Over Voltage Thresholds
const uint16_t OV_THRESHOLD = 36000; //!< Over voltage threshold ADC Code. LSB = 0.0001 ---(3.6V)
const uint16_t UV_THRESHOLD = 25000; //!< Under voltage threshold ADC Code. LSB = 0.0001 ---(2.5V)

/*************************************************************************
 Set LTC681x configuration register. Refer to the data sheet
**************************************************************************/
bool REFON = true; //!< Reference Powered Up Bit
bool ADCOPT = false; //!< ADC Mode option bit
bool gpioBits_a[5] = {false,false,false,false,false}; //!< GPIO Pin Control // Gpio 1,2,3,4,5
bool gpioBits_b[4] = {false,false,false,false}; //!< GPIO Pin Control // Gpio 6,7,8,9
uint16_t UV=UV_THRESHOLD; //!< Under voltage Comparison Voltage
uint16_t OV=OV_THRESHOLD; //!< Over voltage Comparison Voltage
bool dccBits_a[12] = {true,true,true,true,true,true,true,true,true,true,true,true}; //!< Discharge cell switch //Dcc 1,2,3,4,5,6,7,8,9,10,11,12
bool dccBits_b[7]= {true,true,true,true,false,false,false}; //!< Discharge cell switch //Dcc 0,13,14,15
bool dctoBits[4] = {true,false,true,false}; //!< Discharge time value //Dcto 0,1,2,3  // Programed for 4 min
/*Ensure that Dcto bits are set according to the required discharge time. Refer to the data sheet */
bool FDRF = false; //!< Force Digital Redundancy Failure Bit
bool DTMEN = true; //!< Enable Discharge Timer Monitor
bool psBits[2]= {false,false}; //!< Digital Redundancy Path Selection//ps-0,1
bool CellMeasurement_flag = true;
bool ExpentedStatusCHARGE = true;
bool ExpentedStatusDISCHARGE = true;
bool BMS_OFF_flag = false; //!< Force Digital Redundancy Failure Bit

void LTC6813_Init(void){
/* Init LTC6813 Configuration Register */
  LTC6813_init_cfg(TOTAL_IC, bms_ic);
  LTC6813_init_cfgb(TOTAL_IC,bms_ic);
  LTC6813_set_cfgr(current_ic,bms_ic,REFON,ADCOPT,gpioBits_a,dccBits_a, dctoBits, UV, OV);
  LTC6813_set_cfgrb(current_ic,bms_ic,FDRF,DTMEN,psBits,gpioBits_b,dccBits_b);
  LTC6813_reset_crc_count(TOTAL_IC,bms_ic);
  LTC6813_init_reg_limits(TOTAL_IC,bms_ic);

/* Write LTC6813 Configuration Register */
  wakeup_sleep(TOTAL_IC);
  LTC6813_wrcfg(TOTAL_IC,bms_ic);
  LTC6813_wrcfgb(TOTAL_IC,bms_ic);

/* Read LTC6813 Configuration Register */
  wakeup_idle(TOTAL_IC);
  ERR = LTC6813_rdcfg(TOTAL_IC,bms_ic);
  ERR = LTC6813_rdcfgb(TOTAL_IC,bms_ic);
}

uint16_t LTC6813_Read_Cell(uint8_t Cell){
	uint16_t Cell_Volt;

	wakeup_sleep(TOTAL_IC);
	LTC6813_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT); // Start Cell ADC Measurement
	LTC6813_pollAdc();
	wakeup_idle(TOTAL_IC);
	ERR = LTC6813_rdcv(REG_ALL,TOTAL_IC,bms_ic); 	// Set to read back all cell voltage registers

	Cell_Volt = bms_ic[current_ic].cells.c_codes[Cell];
	return Cell_Volt;
}

int16_t LTC6813_Temp_Read(uint8_t Channel){
	int16_t Temperature_Celsius;

	wakeup_sleep(TOTAL_IC);
	LTC6813_adax(ADC_CONVERSION_MODE, AUX_CH_TO_CONVERT);
	LTC6813_pollAdc();
//	HAL_Delay(10);
	ERR = LTC6813_rdaux(NO_OF_REG, TOTAL_IC, bms_ic); // Set to read back all aux registers
	if (ERR == 0){
		Temperature_Code = bms_ic[current_ic].aux.a_codes[Channel];
	}
	else {
		return 333;
	}
	Ntc_R = ((NTC_UP_R)/((32767.0/Temperature_Code) - 1));
	// temp
	float Ntc_Ln = log(Ntc_R);
	// calc. temperature
	Ntc_Tmp = (1.0/(A + B*Ntc_Ln + C*Ntc_Ln*Ntc_Ln*Ntc_Ln)) - 273.15;
	// nullify
	Temperature_Celsius = (int)(Ntc_Tmp*10);
	return Temperature_Celsius;
}

void LTC6813_Equalization(int16_t dccBits){

//dccBits_a[12] = {false,false,false,false,false,false,false,false,false,false,false,false}; //!< Discharge cell switch //Dcc 1,2,3,4,5,6,7,8,9,10,11,12
//dccBits_b[7] = {false,false,false,false,false,false,false}; 								 //!< Discharge cell switch //Dcc 0,13,14,15

dccBits_a[0] = dccBits>>1 & 0x0001;
dccBits_a[1] = dccBits>>2 & 0x0001;
dccBits_a[2] = dccBits>>3 & 0x0001;
dccBits_a[3] = dccBits>>4 & 0x0001;
dccBits_a[4] = dccBits>>5 & 0x0001;
dccBits_a[5] = dccBits>>6 & 0x0001;
dccBits_a[6] = dccBits>>7 & 0x0001;
dccBits_a[7] = dccBits>>8 & 0x0001;
dccBits_a[8] = dccBits>>9 & 0x0001;
dccBits_a[9] = dccBits>>10 & 0x0001;
dccBits_a[10] = dccBits>>11 & 0x0001;
dccBits_a[11] = dccBits>>12 & 0x0001;

dccBits_b[0] = dccBits>>0 & 0x0001;
dccBits_b[1] = dccBits>>13 & 0x0001;
dccBits_b[2] = dccBits>>14 & 0x0001;
dccBits_b[3] = dccBits>>15 & 0x0001;

/* Init LTC6813 Configuration Register */
  LTC6813_init_cfg(TOTAL_IC, bms_ic);
  LTC6813_init_cfgb(TOTAL_IC,bms_ic);
  LTC6813_set_cfgr(current_ic,bms_ic,REFON,ADCOPT,gpioBits_a,dccBits_a, dctoBits, UV, OV);
  LTC6813_set_cfgrb(current_ic,bms_ic,FDRF,DTMEN,psBits,gpioBits_b,dccBits_b);
  LTC6813_reset_crc_count(TOTAL_IC,bms_ic);

/* Write LTC6813 Configuration Register */
  wakeup_sleep(TOTAL_IC);
  LTC6813_wrcfg(TOTAL_IC,bms_ic);
  LTC6813_wrcfgb(TOTAL_IC,bms_ic);
}

int16_t LTC6813_Chk_Start_Error(void){
	#define MIN_CURRENT 10
	int16_t Current_mA;
	int16_t Error_Flag = 0;

	HAL_Delay(10);
	wakeup_sleep(TOTAL_IC);
	LTC6813_adcv(ADC_CONVERSION_MODE,ADC_DCP,CELL_CH_TO_CONVERT); 	// Start LTC ADC conversion
	HAL_Delay(10);
	wakeup_sleep(TOTAL_IC);
	LTC6813_rdcv(NO_OF_REG,TOTAL_IC,bms_ic); 				// Set to read back all cell voltage registers
	LTC6813_rdstat(REG_ALL, TOTAL_IC, bms_ic);				// Set to read back all stat registers

	Current_mA = -INA229_Read_VSHUNT()/RSHUNT/100*75/100;

	if(abs(Current_mA) >= MIN_CURRENT) {
	   Error_Flag = Error_Flag | 0x01;	// Флаг ошибки FETs damage
	}
	for (int i = 0; i < 16 ; i++){
	   if (LTC6813_Read_Cell(i)/10 >= Para_Array[1]){
		   Error_Flag = Error_Flag | 0x02; 	// Флаг ошибки Cells voltage
	   }
	}
	for (int i = 0; i < 16 ; i++){
	   if (LTC6813_Read_Cell(i)/10 <= Para_Array[2]){
		   Error_Flag = Error_Flag | 0x04;	// Флаг ошибки Cells voltage
	   }
	}
	return Error_Flag;
}
/*
void LTC6813_Chk_Cell_Error(void){
	LTC6813_Read_Cell(0);
	for (int i = 0; i < 16; i++){
	  if (bms_ic[current_ic].cells.c_codes[i]/10 > Para_Array[Cell_over_voltage_protection_parameter]){
		  TURN_OFF_CHARGE_MOSFET();
		  LED_ALARM_ON();
	  }
	  else if(bms_ic[current_ic].cells.c_codes[i]/10 < Para_Array[Cell_under_voltage_protection_parameter]){
		  TURN_OFF_DISCHARGE_MOSFET();
		  LED_ALARM_ON();
	  }
	  else if (bms_ic[current_ic].cells.c_codes[i]/10 <= Para_Array[Cell_over_voltage_recover_parameter]) {
		  TURN_ON_CHARGE_MOSFET();
	  }
	  else if (bms_ic[current_ic].cells.c_codes[i]/10 >= Para_Array[Cell_under_voltage_recover_parameter]) {
		  TURN_ON_DISCHARGE_MOSFET();
	  }
	}
}
*/
