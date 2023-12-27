#include "LTC681x.h"


/*  Reads and parses the LTC6813 cell voltage registers */
uint8_t LTC6813_rdcv(uint8_t reg, // Controls which cell voltage register is read back.
                     uint8_t total_ic, // The number of ICs in the system
                     cell_asic *ic // Array of the parsed cell codes
                    )
{
	int8_t pec_error = 0;
	pec_error = LTC681x_rdcv(reg,total_ic,ic);
	return(pec_error);
}


uint16_t LTC6813_Read_Cell(uint8_t Cell){
	uint16_t Cell_Volt;

	LTC6813_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT); // Start Cell ADC Measurement
	LTC6813_pollAdc();
	ERR = LTC6813_rdcv(REG_ALL,TOTAL_IC,bms_ic); 	// Set to read back all cell voltage registers

	Cell_Volt = bms_ic[current_ic].cells.c_codes[Cell];
	return Cell_Volt;
}
