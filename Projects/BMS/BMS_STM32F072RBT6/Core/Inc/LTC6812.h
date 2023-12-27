#ifndef LTC6182_H
#define LTC6182_H

/*  Reads and parses the LTC6813 cell voltage registers */
uint8_t LTC6813_rdcv(uint8_t reg, // Controls which cell voltage register is read back.
                     uint8_t total_ic, // The number of ICs in the system
                     cell_asic *ic // Array of the parsed cell codes
                    );

uint16_t LTC6813_Read_Cell(uint8_t Cell);

#endif
