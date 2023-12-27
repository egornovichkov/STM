/**
 *
 *
 */

#include "STM32_EEPROM_SPI.h"
#include "BMS_Communication_Protocol.h"
#include "main.h"

SPI_HandleTypeDef * EEPROM_SPI;
uint8_t EEPROM_StatusByte;
uint8_t RxBuffer[EEPROM_BUFFER_SIZE] = {0x00};
extern int32_t EEPROM_Read_Array[];


/**
 * @brief Init EEPROM SPI
 *
 * @param hspi Pointer to SPI struct handler
 */
void EEPROM_SPI_INIT(SPI_HandleTypeDef * hspi) {
	HAL_GPIO_WritePin(HOLD_EEPROM_GPIO_Port, HOLD_EEPROM_Pin, GPIO_PIN_SET);
    EEPROM_SPI = hspi;
}

/**
  * @brief  Writes more than one byte to the EEPROM with a single WRITE cycle
  *         (Page WRITE sequence).
  *
  * @note   The number of byte can't exceed the EEPROM page size.
  * @param  pBuffer: pointer to the buffer  containing the data to be written
  *         to the EEPROM.
  * @param  WriteAddr: EEPROM's internal address to write to.
  * @param  NumByteToWrite: number of bytes to write to the EEPROM, must be equal
  *         or less than "EEPROM_PAGESIZE" value.
  * @retval EepromOperations value: EEPROM_STATUS_COMPLETE or EEPROM_STATUS_ERROR
  */
EepromOperations EEPROM_SPI_WritePage(uint8_t* pBuffer, uint16_t WriteAddr, uint16_t NumByteToWrite) {
    while (EEPROM_SPI->State != HAL_SPI_STATE_READY) {
//        HAL_Delay(1);
    	HAL_Delay(1);
    }

    HAL_StatusTypeDef spiTransmitStatus;

    sEE_WriteEnable();

    /*
        We gonna send commands in one packet of 3 bytes
     */
    uint8_t header[3];

    header[0] = EEPROM_WRITE;   // Send "Write to Memory" instruction
    header[1] = WriteAddr >> 8; // Send 16-bit address
    header[2] = WriteAddr;

    // Select the EEPROM: Chip Select low
    EEPROM_CS_LOW();

    EEPROM_SPI_SendInstruction((uint8_t*)header, 3);

    // Make 5 attemtps to write the data
    for (uint8_t i = 0; i < 5; i++) {
        spiTransmitStatus = HAL_SPI_Transmit(EEPROM_SPI, pBuffer, NumByteToWrite, 100);

        if (spiTransmitStatus == HAL_BUSY) {
            HAL_Delay(5);
        } else {
            break;
        }
    }

    // Deselect the EEPROM: Chip Select high
    EEPROM_CS_HIGH();

    // Wait the end of EEPROM writing
    EEPROM_SPI_WaitStandbyState();
    HAL_Delay(1);
    // Disable the write access to the EEPROM
    sEE_WriteDisable();

    if (spiTransmitStatus == HAL_ERROR) {
        return EEPROM_STATUS_ERROR;
    } else {
        return EEPROM_STATUS_COMPLETE;
    }
}

void EEPROM_SPI_Write_int32_t(uint16_t WriteAddr, int32_t value) {
	uint8_t Buf[4] = {0,};

	Buf[0] = (value & 0xFF);
	Buf[1] = ((value >> 8) & 0xFF);
	Buf[2] = ((value >> 16) & 0xFF);
	Buf[3] = ((value >> 24) & 0xFF);

	EEPROM_SPI_WriteBuffer(Buf, WriteAddr, 4);
}

void EEPROM_SPI_WriteBuffer_int32_t(int32_t* pBuffer, uint16_t WriteAddr, uint16_t NumToWrite){
	for (int i = 0; i < NumToWrite; i++){
		EEPROM_SPI_Write_int32_t(WriteAddr + i*4, pBuffer[i]);
	}
}

int32_t EEPROM_SPI_Read_int32_t(uint16_t ReadAddr) {
	uint8_t Buf[4] = {0,};

	EEPROM_SPI_ReadBuffer(Buf, ReadAddr, 4);

	int32_t four = Buf[0];
	int32_t three = Buf[1];
	int32_t two = Buf[2];
	int32_t one = Buf[3];

    //Return the recomposed long by using bitshift.
    return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}

void EEPROM_SPI_ReadBuffer_int32_t(uint16_t ReadAddr, uint16_t NumToRead){
	for (int i = 0; i < NumToRead; i++){
		EEPROM_Read_Array[i] = EEPROM_SPI_Read_int32_t(ReadAddr + i*4);
	}
}

/**
  * @brief  Writes block of data to the EEPROM. In this function, the number of
  *         WRITE cycles are reduced, using Page WRITE sequence.
  *
  * @param  pBuffer: pointer to the buffer  containing the data to be written
  *         to the EEPROM.
  * @param  WriteAddr: EEPROM's internal address to write to.
  * @param  NumByteToWrite: number of bytes to write to the EEPROM.
  * @retval EepromOperations value: EEPROM_STATUS_COMPLETE or EEPROM_STATUS_ERROR
  */
EepromOperations EEPROM_SPI_WriteBuffer(uint8_t* pBuffer, uint16_t WriteAddr, uint16_t NumByteToWrite) {
    uint16_t NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;
    uint16_t sEE_DataNum = 0;

    EepromOperations pageWriteStatus = EEPROM_STATUS_PENDING;

    Addr = WriteAddr % EEPROM_PAGESIZE;
    count = EEPROM_PAGESIZE - Addr;
    NumOfPage =  NumByteToWrite / EEPROM_PAGESIZE;
    NumOfSingle = NumByteToWrite % EEPROM_PAGESIZE;

    if (Addr == 0) { /* WriteAddr is EEPROM_PAGESIZE aligned  */
        if (NumOfPage == 0) { /* NumByteToWrite < EEPROM_PAGESIZE */
            sEE_DataNum = NumByteToWrite;
            pageWriteStatus = EEPROM_SPI_WritePage(pBuffer, WriteAddr, sEE_DataNum);

            if (pageWriteStatus != EEPROM_STATUS_COMPLETE) {
                return pageWriteStatus;
            }

        } else { /* NumByteToWrite > EEPROM_PAGESIZE */
            while (NumOfPage--) {
                sEE_DataNum = EEPROM_PAGESIZE;
                pageWriteStatus = EEPROM_SPI_WritePage(pBuffer, WriteAddr, sEE_DataNum);

                if (pageWriteStatus != EEPROM_STATUS_COMPLETE) {
                    return pageWriteStatus;
                }

                WriteAddr +=  EEPROM_PAGESIZE;
                pBuffer += EEPROM_PAGESIZE;
            }

            sEE_DataNum = NumOfSingle;
            pageWriteStatus = EEPROM_SPI_WritePage(pBuffer, WriteAddr, sEE_DataNum);

            if (pageWriteStatus != EEPROM_STATUS_COMPLETE) {
                return pageWriteStatus;
            }
        }
    } else { /* WriteAddr is not EEPROM_PAGESIZE aligned  */
        if (NumOfPage == 0) { /* NumByteToWrite < EEPROM_PAGESIZE */
            if (NumOfSingle > count) { /* (NumByteToWrite + WriteAddr) > EEPROM_PAGESIZE */
                temp = NumOfSingle - count;
                sEE_DataNum = count;
                pageWriteStatus = EEPROM_SPI_WritePage(pBuffer, WriteAddr, sEE_DataNum);

                if (pageWriteStatus != EEPROM_STATUS_COMPLETE) {
                    return pageWriteStatus;
                }

                WriteAddr +=  count;
                pBuffer += count;

                sEE_DataNum = temp;
                pageWriteStatus = EEPROM_SPI_WritePage(pBuffer, WriteAddr, sEE_DataNum);
            } else {
                sEE_DataNum = NumByteToWrite;
                pageWriteStatus = EEPROM_SPI_WritePage(pBuffer, WriteAddr, sEE_DataNum);
            }

            if (pageWriteStatus != EEPROM_STATUS_COMPLETE) {
                return pageWriteStatus;
            }
        } else { /* NumByteToWrite > EEPROM_PAGESIZE */
            NumByteToWrite -= count;
            NumOfPage =  NumByteToWrite / EEPROM_PAGESIZE;
            NumOfSingle = NumByteToWrite % EEPROM_PAGESIZE;

            sEE_DataNum = count;

            pageWriteStatus = EEPROM_SPI_WritePage(pBuffer, WriteAddr, sEE_DataNum);

            if (pageWriteStatus != EEPROM_STATUS_COMPLETE) {
                return pageWriteStatus;
            }

            WriteAddr +=  count;
            pBuffer += count;

            while (NumOfPage--) {
                sEE_DataNum = EEPROM_PAGESIZE;

                pageWriteStatus = EEPROM_SPI_WritePage(pBuffer, WriteAddr, sEE_DataNum);

                if (pageWriteStatus != EEPROM_STATUS_COMPLETE) {
                    return pageWriteStatus;
                }

                WriteAddr +=  EEPROM_PAGESIZE;
                pBuffer += EEPROM_PAGESIZE;
            }

            if (NumOfSingle != 0) {
                sEE_DataNum = NumOfSingle;

                pageWriteStatus = EEPROM_SPI_WritePage(pBuffer, WriteAddr, sEE_DataNum);

                if (pageWriteStatus != EEPROM_STATUS_COMPLETE) {
                    return pageWriteStatus;
                }
            }
        }
    }

    return EEPROM_STATUS_COMPLETE;
}

/**
  * @brief  Reads a block of data from the EEPROM.
  *
  * @param  pBuffer: pointer to the buffer that receives the data read from the EEPROM.
  * @param  ReadAddr: EEPROM's internal address to read from.
  * @param  NumByteToRead: number of bytes to read from the EEPROM.
  * @retval None
  */
EepromOperations EEPROM_SPI_ReadBuffer(uint8_t* pBuffer, uint16_t ReadAddr, uint16_t NumByteToRead) {
    while (EEPROM_SPI->State != HAL_SPI_STATE_READY) {
        HAL_Delay(1);
    }

    /*
        We gonna send all commands in one packet of 3 bytes
     */

    uint8_t header[3];

    header[0] = EEPROM_READ;    // Send "Read from Memory" instruction
    header[1] = ReadAddr >> 8;  // Send 16-bit address
    header[2] = ReadAddr;

    // Select the EEPROM: Chip Select low
    EEPROM_CS_LOW();

    /* Send WriteAddr address byte to read from */
    EEPROM_SPI_SendInstruction(header, 3);

    while (HAL_SPI_Receive(EEPROM_SPI, (uint8_t*)pBuffer, NumByteToRead, 200) == HAL_BUSY) {
        HAL_Delay(1);
    };

    // Deselect the EEPROM: Chip Select high
    EEPROM_CS_HIGH();

    return EEPROM_STATUS_COMPLETE;
}

/**
  * @brief  Sends a byte through the SPI interface and return the byte received
  *         from the SPI bus.
  *
  * @param  byte: byte to send.
  * @retval The value of the received byte.
  */
uint8_t EEPROM_SendByte(uint8_t byte) {
    uint8_t answerByte;

    /* Loop while DR register in not empty */
    while (EEPROM_SPI->State == HAL_SPI_STATE_RESET) {
        HAL_Delay(1);
    }

    /* Send byte through the SPI peripheral */
    if (HAL_SPI_Transmit(EEPROM_SPI, &byte, 1, 200) != HAL_OK) {
        Error_Handler();
    }

    /* Wait to receive a byte */
    while (EEPROM_SPI->State == HAL_SPI_STATE_RESET) {
        HAL_Delay(1);
    }

    /* Return the byte read from the SPI bus */
    if (HAL_SPI_Receive(EEPROM_SPI, &answerByte, 1, 200) != HAL_OK) {
        Error_Handler();
    }

    return (uint8_t)answerByte;
}
/**
  * @brief  Enables the write access to the EEPROM.
  *
  * @param  None
  * @retval None
  */
void sEE_WriteEnable(void) {
    // Select the EEPROM: Chip Select low
    EEPROM_CS_LOW();

    uint8_t command[1] = { EEPROM_WREN };
    /* Send "Write Enable" instruction */
    EEPROM_SPI_SendInstruction((uint8_t*)command, 1);

    // Deselect the EEPROM: Chip Select high
    EEPROM_CS_HIGH();
}

/**
  * @brief  Disables the write access to the EEPROM.
  *
  * @param  None
  * @retval None
  */
void sEE_WriteDisable(void) {
    // Select the EEPROM: Chip Select low
    EEPROM_CS_LOW();

    uint8_t command[1] = { EEPROM_WRDI };

    /* Send "Write Disable" instruction */
    EEPROM_SPI_SendInstruction((uint8_t*)command, 1);

    // Deselect the EEPROM: Chip Select high
    EEPROM_CS_HIGH();
}

/**
  * @brief  Write new value in EEPROM Status Register.
  *
  * @param  regval : new value of register
  * @retval None
  */
void sEE_WriteStatusRegister(uint8_t regval) {
    uint8_t command[2];

    command[0] = EEPROM_WRSR;
    command[1] = regval;

    // Enable the write access to the EEPROM
    sEE_WriteEnable();

    // Select the EEPROM: Chip Select low
    EEPROM_CS_LOW();

    // Send "Write Status Register" instruction
    // and Regval in one packet
    EEPROM_SPI_SendInstruction((uint8_t*)command, 2);

    // Deselect the EEPROM: Chip Select high
    EEPROM_CS_HIGH();

    sEE_WriteDisable();
}


/**
  * @brief  Polls the status of the Write In Progress (WIP) flag in the EEPROM's
  *         status register and loop until write operation has completed.
  *
  * @param  None
  * @retval None
  */
uint8_t EEPROM_SPI_WaitStandbyState(void) {
    uint8_t sEEstatus[1] = { 0x00 };
    uint8_t command[1] = { EEPROM_RDSR };

    // Select the EEPROM: Chip Select low
    EEPROM_CS_LOW();

    // Send "Read Status Register" instruction
    EEPROM_SPI_SendInstruction((uint8_t*)command, 1);

    // Loop as long as the memory is busy with a write cycle
    do {

        while (HAL_SPI_Receive(EEPROM_SPI, (uint8_t*)sEEstatus, 1, 200) == HAL_BUSY) {
            HAL_Delay(1);
        };

        HAL_Delay(1);

    } while ((sEEstatus[0] & EEPROM_WIP_FLAG) == SET); // Write in progress

    // Deselect the EEPROM: Chip Select high
    EEPROM_CS_HIGH();

    return 0;
}

/**
 * @brief Low level function to send header data to EEPROM
 *
 * @param instruction array of bytes to send
 * @param size        data size in bytes
 */
void EEPROM_SPI_SendInstruction(uint8_t *instruction, uint8_t size) {
    while (EEPROM_SPI->State == HAL_SPI_STATE_RESET) {
        HAL_Delay(1);
    }

    if (HAL_SPI_Transmit(EEPROM_SPI, (uint8_t*)instruction, (uint16_t)size, 200) != HAL_OK) {
        Error_Handler();
    }
}

void EEPROM_LOAD_DEFAULT(void) {
	int32_t EEPROM_TxBuffer[69] = {0,};

	EEPROM_TxBuffer[The_pack_location] = 1;
	EEPROM_TxBuffer[Cell_high_voltage_alarm_parameter] = 3600;
	EEPROM_TxBuffer[Cell_low_voltage_alarm_parameter] = 3000;
	EEPROM_TxBuffer[Cell_high_temperature_alarm_parameter] = 273 + 60;
	EEPROM_TxBuffer[Cell_low_temperature_alarm_parameter] = 273 - 20;
	EEPROM_TxBuffer[Charge_over_current_alarm_parameter] = 1;
	EEPROM_TxBuffer[Battery_high_voltage_alarm_parameter] = 46000;
	EEPROM_TxBuffer[Battery_low_voltage_alarm_parameter] = 36000;
	EEPROM_TxBuffer[Self_definition_parameter_number] = 11;
	EEPROM_TxBuffer[Discharge_over_current_alarm_parameter] = 1;
	EEPROM_TxBuffer[Cell_over_voltage_protection_parameter] = 3800;
	EEPROM_TxBuffer[Cell_over_voltage_recover_parameter] = 3600;
	EEPROM_TxBuffer[Cell_under_voltage_recover_parameter] = 3000;
	EEPROM_TxBuffer[Cell_under_voltage_protection_parameter] = 2800;
	EEPROM_TxBuffer[Battery_over_voltage_protection_parameter] = 4700;
	EEPROM_TxBuffer[Battery_over_voltage_recover_parameter] = 4500;
	EEPROM_TxBuffer[Battery_under_voltage_recover_parameter] = 37000;
	EEPROM_TxBuffer[Battery_under_voltage_protection_parameter] = 35000;
	EEPROM_TxBuffer[Charge_over_temperature_protection_parameter] = 273 + 50;
	EEPROM_TxBuffer[Charge_over_temperature_recover_parameter] = 273 + 40;
	EEPROM_TxBuffer[Charge_under_temperature_recover_parameter] = 0;
	EEPROM_TxBuffer[Charge_under_temperature_protection_parameter] = -20;
	EEPROM_TxBuffer[Discharge_over_temperature_protection_parameter] = 50;
	EEPROM_TxBuffer[Discharge_over_temperature_recover_parameter] = 45;
	EEPROM_TxBuffer[Discharge_under_temperature_recover_parameter] = 0;
	EEPROM_TxBuffer[Discharge_under_temperature_protection_parameter] = -20;
	EEPROM_TxBuffer[High_environment_temperature_alarm_parameter] = 50;
	EEPROM_TxBuffer[Low_environment_temperature_alarm_parameter] = 0;
	EEPROM_TxBuffer[Over_environment_temperature_protection_parameter] = 60;
	EEPROM_TxBuffer[Over_environment_temperature_recover_parameter] = 50;
	EEPROM_TxBuffer[Under_environment_temperature_protection_parameter] = -20;
	EEPROM_TxBuffer[Under_environment_temperature_recover_parameter] = 0;
	EEPROM_TxBuffer[Cell_heating_temperature_open_parameter] = 0;
	EEPROM_TxBuffer[Cell_heating_temperature_stop_parameter] = 25;
	EEPROM_TxBuffer[High_power_temperature_alarm_parameter] = 50;
	EEPROM_TxBuffer[Low_power_temperature_alarm_parameter] = 0;
	EEPROM_TxBuffer[Over_power_temperature_protection_parameter] = 50;
	EEPROM_TxBuffer[Over_power_temperature_recover_parameter] = 40;
	EEPROM_TxBuffer[Under_power_temperature_recover_parameter] = 0;
	EEPROM_TxBuffer[Under_power_temperature_protection_parameter] = -20;
	EEPROM_TxBuffer[Charge_over_current_protection_parameter] = 2;
	EEPROM_TxBuffer[Charge_over_current_time_delay_parameter] = 0;
	EEPROM_TxBuffer[Discharge_over_current_protection_parameter] = 2;
	EEPROM_TxBuffer[Discharge_over_current_time_delay_parameter] = 1000;
	EEPROM_TxBuffer[Secondary_over_current_protection_parameter] = 0;
	EEPROM_TxBuffer[Secondary_over_current_time_delay_parameter] = 0;
	EEPROM_TxBuffer[Output_shortcut_protection_parameter] = 0;
	EEPROM_TxBuffer[Output_shortcut_time_delay_parameter] = 0;
	EEPROM_TxBuffer[Over_current_recover_time_delay_parameter] = 5;
	EEPROM_TxBuffer[Over_current_lock_times_parameter] = 0;
	EEPROM_TxBuffer[Battery_rated_capacity_parameter] = 800;
	EEPROM_TxBuffer[Cell_number_serial_battery_parameter] = 16;
	EEPROM_TxBuffer[Charge_current_limit_set_parameter] = 0;
	EEPROM_TxBuffer[Equalization_high_temperature_prohibit_parameter] = 0;
	EEPROM_TxBuffer[Equalization_low_temperature_prohibit_parameter] = 0;
	EEPROM_TxBuffer[Static_equilibrium_time_parameter] = 0;
	EEPROM_TxBuffer[Equalization_open_voltage_parameter] = 0;
	EEPROM_TxBuffer[Equalization_open_voltage_difference_parameter] = 0;
	EEPROM_TxBuffer[Equalization_stop_voltage_difference_parameter] = 0;
	EEPROM_TxBuffer[Cell_failure_voltage_difference_parameter] = 0;
	EEPROM_TxBuffer[Cell_failure_voltage_recover_parameter] = 0;
	EEPROM_TxBuffer[Self_definition_switch_number_parameter] = 0;
	EEPROM_TxBuffer[Voltage_function_switch_parameter] = 0;
	EEPROM_TxBuffer[Temperature_function_switch_parameter] = 0;
	EEPROM_TxBuffer[Current_function_switch_parameter] = 0;
	EEPROM_TxBuffer[Capacity_and_other_function_switch] = 0;
	EEPROM_TxBuffer[Equalization_function_switch_parameter] = 0;
	EEPROM_TxBuffer[Indicator_function_switch_parameter] = 0;
	EEPROM_TxBuffer[BMS_name] = 1;

	EEPROM_SPI_WriteBuffer_int32_t(EEPROM_TxBuffer, (uint16_t)0x00, (uint16_t)69);
}






















