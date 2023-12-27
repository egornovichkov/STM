/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdint.h"
#include "stdlib.h"
#include "stm32_tm1637.h"
#include "LTC6813.h"
#include "LTC6813_BMS.h"
#include "INA229.h"
#include "BMS_Communication_Protocol.h"
#include "w25qxx.h"
#include "STM32_EEPROM_SPI.h"
#include "Event_Log.h"
#include "BMS_Power_Control.h"
#include "stdio.h"
#include "string.h"

//#include "LTC6813_BMS.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi4;
SPI_HandleTypeDef hspi6;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

const uint16_t M = 18;
const uint16_t N = 4;

RTC_TimeTypeDef sTime = {0};
RTC_DateTypeDef DateToUpdate = {0};
char trans_str[5] = {0,};

uint8_t w_buf[64] = {0,};
uint8_t r_buf[64] = {0,};

uint16_t History_Data_Num = 0;

uint8_t Cell_Voltage_Alarm_State[18] = {0,};
uint8_t Temperature_Alarm_State[4] = {0,};
uint8_t Current_Alarm_State = 0;
uint8_t Battery_Voltage_Alarm_State = 0;
uint8_t System_State_Code = 0;
uint8_t Voltage_Event_Code;
uint8_t Temperature_Event_Code;
uint8_t Current_Event_Code;

uint8_t Pack_Location;

int16_t Alarm_Max_Temper_Cell;
int16_t Alarm_Min_Temper_Cell;
uint16_t Alarm_Charge_Over_Current;
uint16_t Alarm_Discharge_Over_Current;
uint16_t Alarm_Cell_Over_Voltage;
uint16_t Alarm_Cell_Under_Voltage;
uint16_t Alarm_Battery_Over_Voltage;
uint16_t Alarm_Battery_Under_Voltage;

uint16_t Protection_Cell_Over_Voltage;
uint16_t Protection_Recover_Cell_Over_Voltage;
uint16_t Protection_Recover_Cell_Under_Voltage;
uint16_t Protection_Cell_Under_Voltage;
uint16_t Protection_Battery_Over_Voltage;
uint16_t Protection_Recover_Battery_Over_Voltage;
uint16_t Protection_Recover_Battery_Under_Voltage;
uint16_t Protection_Battery_Under_Voltage;
uint16_t Protection_Charge_Over_Current;
uint16_t Protection_Discharge_Over_Current;

int16_t Equalization_State = 0;
uint16_t Total_Battery_Capacity_mAh = BATTERY_CAPACITY_DEFAULT;
uint32_t Battery_Residual_Capacity_mAh;
uint64_t Battery_Capacity_Accum;
uint16_t Battery_Cycles = 1;

uint16_t Cell_Voltage_mV;
uint16_t Vpack_Voltage_mV;
int16_t Current_mA;
int16_t Current_Average;
int16_t Environment_temperature;
uint16_t Cell_Voltage_Array[18] = {0,};

uint8_t USART_TX_SEND = 0;
uint8_t RS485_RX_DATA[RS485_RX_MAX_FRAME_LENGTH] = {0,};
uint8_t RS485_RX_DATA_TEMP;
uint8_t RS485_RX_START_FRAME_FLAG = 0;
uint8_t RS485_RX_STOP_FRAME_FLAG = 0;
uint8_t RS485_RX_FRAME_DONE_FLAG = 0;
uint8_t RS485_RX_COUNTER = 0;

uint8_t Alarm_Flag = 0;
uint8_t screen_num = 0;

char PackNum_Char_1;
char PackNum_Char_2;

uint8_t SOC_percent;

uint8_t Temp_Ch;

uint16_t EEPROM_Write_timer;

#define CURRENT_COEF 75/100
#define CURRENT_COEF_INV 100/75

#define STARTING_CURRENT 10

#define ParaArraySize 69
int32_t EEPROM_Read_Array[ParaArraySize] = {0,};
int32_t Para_Array[ParaArraySize] = {0,};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI6_Init(void);
static void MX_TIM6_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){ // INA229 ALARM
   if(GPIO_Pin == GPIO_PIN_10){
		if (INA229_Read_ALERT() & 0x20){
			HAL_GPIO_WritePin(TURN_OFF_CHARGE_GPIO_Port, TURN_OFF_CHARGE_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(TURN_ON_CHARGE_GPIO_Port, TURN_ON_CHARGE_Pin, GPIO_PIN_RESET);
			Alarm_Flag = 3;
		}
		else if (INA229_Read_ALERT() & 0x40){
			HAL_GPIO_WritePin(TURN_OFF_DISCHARGE_GPIO_Port, TURN_OFF_DISCHARGE_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(TURN_ON_DISCHARGE_GPIO_Port, TURN_ON_DISCHARGE_Pin, GPIO_PIN_RESET);
			Alarm_Flag = 5;
		}
   }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == &huart2){
		RS485_RX_FLAG_STATE();
		HAL_UART_Receive_IT(&huart2, &RS485_RX_DATA_TEMP, 1);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim->Instance == TIM6) {

//		Current_mA = INA229_Read_VSHUNT()/RSHUNT/25;
//		Battery_Capacity_Accum += Current_mA*2^32/(1000*60*60);
		Battery_Capacity_Accum += Current_mA*4294967296/3600000;
		Battery_Residual_Capacity_mAh = Battery_Capacity_Accum >> 32;

		if (Battery_Residual_Capacity_mAh > Total_Battery_Capacity_mAh){
			Battery_Residual_Capacity_mAh = Total_Battery_Capacity_mAh;
		}
		else if (Battery_Residual_Capacity_mAh < 0){
			Battery_Residual_Capacity_mAh = 0;
		}
	}
}

// Фильтр скользящего среднего
int16_t Moving_Average_Simple_Current(int16_t Zk) {
    #define windowLengthCurrent 32
    static int16_t delayLine[windowLengthCurrent];
    static uint16_t pointer;
    static int32_t average_Out;
  if (++pointer >= windowLengthCurrent) pointer = 0;
  average_Out -= delayLine[pointer];
  average_Out += Zk;
  delayLine[pointer] = Zk;
  return (average_Out / windowLengthCurrent);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_SPI6_Init();
  MX_TIM6_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_SPI4_Init();
  MX_USART1_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_PWR_EnableBkUpAccess();

  TURN_OFF_ALL_MOSFETS();

  HAL_GPIO_WritePin(CAN_PWR_EN_GPIO_Port, CAN_PWR_EN_Pin, GPIO_PIN_SET);
  Pack_Location = Pack_Number_Jamper_Read();
  PackNum_Char_1 = Hex_to_ASCII(Pack_Location);
  PackNum_Char_2 = Hex_to_ASCII(Pack_Location)>>8;

  EEPROM_SPI_INIT(&hspi4);
  W25qxx_Init();
  Clear_Flash_Block(0);

  EEPROM_LOAD_DEFAULT();

  EEPROM_SPI_ReadBuffer_int32_t(0x00, ParaArraySize);
	for (int i = 0; i < ParaArraySize; i++){
		Para_Array[i] = EEPROM_Read_Array[i];
	}

  Battery_Capacity_Accum = Para_Array[Battery_rated_capacity_parameter] * 42949672960;

  INA229_Init();
  INA229_Set_Current_Charge_ALRM(STARTING_CURRENT * 1000 / 25 * 10 * CURRENT_COEF_INV);
  INA229_Set_Current_Discharge_ALRM(STARTING_CURRENT * (-1000) / 25 * 10 * CURRENT_COEF_INV);

  LTC6813_Init();

  tm1637Init();
  tm1637WriteSnake(100);
  tm1637WriteSnake(100);

  uint16_t Err_Chk = LTC6813_Chk_Start_Error();

  while (Err_Chk != 0){
	  LED_ALARM_ON();
	  if (Err_Chk == 0x01){
		  tm1637WriteSymbol('E', 3);
		  tm1637WriteSymbol('-', 2);
		  tm1637WriteSymbol('0', 1);
		  tm1637WriteSymbol('1', 0);
	  }
	  else if (Err_Chk == 0x02){
		  tm1637WriteSymbol('E', 3);
		  tm1637WriteSymbol('-', 2);
		  tm1637WriteSymbol('0', 1);
		  tm1637WriteSymbol('2', 0);
	  }
	  else if (Err_Chk == 0x04){
		  tm1637WriteSymbol('E', 3);
		  tm1637WriteSymbol('-', 2);
		  tm1637WriteSymbol('0', 1);
		  tm1637WriteSymbol('4', 0);
	  }
	  Err_Chk = LTC6813_Chk_Start_Error();
  }

  TURN_ON_PRE_CHARGE();
  HAL_Delay(Para_Array[Discharge_over_current_time_delay_parameter]);
  TURN_ON_ALL_MOSFETS();
  TURN_OFF_PRE_CHARGE();

//  Read_History_from_Flash(History_Data_Num - 1);

  INA229_Set_Current_Charge_ALRM(Para_Array[Charge_over_current_protection_parameter] * 1000 / 25 * 10 * CURRENT_COEF_INV);
  INA229_Set_Current_Discharge_ALRM(-Para_Array[Discharge_over_current_protection_parameter] * 1000 / 25 * 10 * CURRENT_COEF_INV);

  HAL_TIM_Base_Start_IT(&htim6);
  HAL_UART_Receive_IT(&huart2, &RS485_RX_DATA_TEMP, 1);

  LED_OK_ON();

// HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0xBEBE); // 32 битные регистры BKUP памяти
// HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Alarm_State_Update();
//	  LTC6813_Chk_Cell_Error();

	  Current_mA = -INA229_Read_VSHUNT()/RSHUNT/100*CURRENT_COEF;
	  Current_Average = Moving_Average_Simple_Current(Current_mA);
	  SOC_percent = Battery_Residual_Capacity_mAh * 100 / Total_Battery_Capacity_mAh;

	  Cell_Voltage_mV = LTC6813_Read_Cell(0)/10;
//	  tm1637DisplayDecimalPnt(Cell_Voltage_mV, 4);
	  tm1637DisplayDecimalPnt(SOC_percent, 0);

	  if (Current_mA < 0){
		  LED_DISCHARGE_ON();
		  LED_CHARGE_OFF();
	  }
	  else if (Current_mA > 0){
		  LED_CHARGE_ON();
		  LED_DISCHARGE_OFF();
	  }

	  if (Alarm_Flag != 0) {
		  LED_ALARM_ON();
		  LED_OK_OFF();
		  if (Alarm_Flag & 0x02) {
			  Alarm_Event_Log();
			  HAL_Delay(2000);
			  Alarm_Event_Log();
			  Alarm_Flag = 0;
			  TURN_ON_CHARGE_MOSFET();
		  }
		  if (Alarm_Flag & 0x04) {
			  Alarm_Event_Log();
			  HAL_Delay(2000);
			  Alarm_Event_Log();
			  Alarm_Flag = 0;
			  TURN_ON_DISCHARGE_MOSFET();
		  }
		  if (Alarm_Flag & 0x10) {
			  TURN_OFF_CHARGE_MOSFET();
			  LED_CHARGE_OFF();
		  }
		  if (Alarm_Flag & 0x20) {
			  TURN_OFF_DISCHARGE_MOSFET();
			  LED_DISCHARGE_OFF();
		  }
	  }
	  else {
		  LED_ALARM_OFF();
		  LED_OK_ON();
		  TURN_ON_CHARGE_MOSFET();
		  TURN_ON_DISCHARGE_MOSFET();
	  }

	  if(RS485_RX_FRAME_DONE_FLAG == 1) {
		if (RS485_RX_DATA[13] == PackNum_Char_2 && RS485_RX_DATA[14] == PackNum_Char_1){
			if (RS485_RX_DATA[7] == '4' && RS485_RX_DATA[8] == '2'){
				RS485_Transmit_Remote_Measuring();
			}
			else if (RS485_RX_DATA[7] == '4' && RS485_RX_DATA[8] == '4'){
				RS485_Transmit_Remote_Signaling();
			}
			else if (RS485_RX_DATA[7] == '4' && RS485_RX_DATA[8] == '7'){
				RS485_Transmit_Remote_Adjusting();
			}
			else if (RS485_RX_DATA[7] == '4' && RS485_RX_DATA[8] == '9'){
				RS485_Remote_Set_Command();
			}
			else if (RS485_RX_DATA[7] == '4' && RS485_RX_DATA[8] == 'B'){
				RS485_Transmit_Remote_History(History_Data_Num - 1);
			}
		}
		RS485_RX_COUNTER = 0;
		RS485_RX_FRAME_DONE_FLAG = 0;
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_Delay(100);

	  if (EEPROM_Write_timer < 50){
		  EEPROM_Write_timer++;
	  }
	  else {
		  EEPROM_Write_timer = 0;
		  int32_t EEPROM_Temp[1];
		  EEPROM_Temp[0] = Battery_Capacity_Accum/42949672960;
		  EEPROM_SPI_Write_int32_t(50*4, EEPROM_Temp[0]);
	  }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 15;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI6|RCC_PERIPHCLK_SPI4
                              |RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2;
  PeriphClkInitStruct.PLL2.PLL2M = 1;
  PeriphClkInitStruct.PLL2.PLL2N = 9;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 3;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 3072;
  PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_PLL2;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_PLL2;
  PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_PLL2;
  PeriphClkInitStruct.Spi6ClockSelection = RCC_SPI6CLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM6_DAC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x12;
  sTime.Minutes = 0x12;
  sTime.Seconds = 0x12;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_SUNDAY;
  sDate.Month = RTC_MONTH_JULY;
  sDate.Date = 0x9;
  sDate.Year = 0x23;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable Calibrartion
  */
  if (HAL_RTCEx_SetCalibrationOutPut(&hrtc, RTC_CALIBOUTPUT_1HZ) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief SPI6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI6_Init(void)
{

  /* USER CODE BEGIN SPI6_Init 0 */

  /* USER CODE END SPI6_Init 0 */

  /* USER CODE BEGIN SPI6_Init 1 */

  /* USER CODE END SPI6_Init 1 */
  /* SPI6 parameter configuration*/
  hspi6.Instance = SPI6;
  hspi6.Init.Mode = SPI_MODE_MASTER;
  hspi6.Init.Direction = SPI_DIRECTION_2LINES;
  hspi6.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi6.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi6.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi6.Init.NSS = SPI_NSS_SOFT;
  hspi6.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi6.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi6.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi6.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi6.Init.CRCPolynomial = 0x0;
  hspi6.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi6.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi6.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi6.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi6.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi6.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi6.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi6.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi6.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
  hspi6.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI6_Init 2 */

  /* USER CODE END SPI6_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 60000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_TXINVERT_INIT|UART_ADVFEATURE_RXINVERT_INIT
                              |UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart2.AdvancedInit.TxPinLevelInvert = UART_ADVFEATURE_TXINV_ENABLE;
  huart2.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
  huart2.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  huart2.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, TURN_ON_CHARGE_Pin|TURN_ON_DISCHARGE_Pin|EN_RELAY_Pin|TURN_OFF_CHARGE_Pin
                          |SPI4_NSS_EEPROM_Pin|WRPT_EEPROM_Pin|HOLD_EEPROM_Pin|TURN_OFF_DISCHARGE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI6_NSS_LTC_GPIO_Port, SPI6_NSS_LTC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI4_NSS_FLASH_Pin|LED_GREEN_BUZEER_Pin|LED_CHARGE_NC2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, SPI2_NSS_INA_Pin|TURN_ON_PRECHARGE_Pin|LED_ALARM_NC3_Pin|LED_DISCHARGE_NC1_Pin
                          |USART2_DE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAN_PWR_EN_GPIO_Port, CAN_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : TURN_ON_CHARGE_Pin TURN_ON_DISCHARGE_Pin EN_RELAY_Pin TURN_OFF_CHARGE_Pin
                           SPI4_NSS_EEPROM_Pin WRPT_EEPROM_Pin HOLD_EEPROM_Pin TURN_OFF_DISCHARGE_Pin */
  GPIO_InitStruct.Pin = TURN_ON_CHARGE_Pin|TURN_ON_DISCHARGE_Pin|EN_RELAY_Pin|TURN_OFF_CHARGE_Pin
                          |SPI4_NSS_EEPROM_Pin|WRPT_EEPROM_Pin|HOLD_EEPROM_Pin|TURN_OFF_DISCHARGE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI6_NSS_LTC_Pin */
  GPIO_InitStruct.Pin = SPI6_NSS_LTC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI6_NSS_LTC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FNC_BTN_Pin LED_GREEN_OK_Pin */
  GPIO_InitStruct.Pin = FNC_BTN_Pin|LED_GREEN_OK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_YELLOW_DISCHARGE_Pin LED_YELLOW_CHARGE_Pin */
  GPIO_InitStruct.Pin = LED_YELLOW_DISCHARGE_Pin|LED_YELLOW_CHARGE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI4_NSS_FLASH_Pin LED_GREEN_BUZEER_Pin LED_CHARGE_NC2_Pin */
  GPIO_InitStruct.Pin = SPI4_NSS_FLASH_Pin|LED_GREEN_BUZEER_Pin|LED_CHARGE_NC2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_RED_ALARM_Pin */
  GPIO_InitStruct.Pin = LED_RED_ALARM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LED_RED_ALARM_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI2_NSS_INA_Pin TURN_ON_PRECHARGE_Pin LED_ALARM_NC3_Pin LED_DISCHARGE_NC1_Pin
                           USART2_DE_Pin */
  GPIO_InitStruct.Pin = SPI2_NSS_INA_Pin|TURN_ON_PRECHARGE_Pin|LED_ALARM_NC3_Pin|LED_DISCHARGE_NC1_Pin
                          |USART2_DE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : ALARM_INA_Pin */
  GPIO_InitStruct.Pin = ALARM_INA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ALARM_INA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CAN_PWR_EN_Pin */
  GPIO_InitStruct.Pin = CAN_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAN_PWR_EN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
