#pragma once

// Configuration.

#define CLK_PORT GPIOB
#define DIO_PORT GPIOB
#define CLK_PIN GPIO_PIN_8
#define DIO_PIN GPIO_PIN_9
#define CLK_PORT_CLK_ENABLE __HAL_RCC_GPIOB_CLK_ENABLE
#define DIO_PORT_CLK_ENABLE __HAL_RCC_GPIOB_CLK_ENABLE

void tm1637Init(void);
void tm1637DisplayDecimalZero(int v, int displaySeparator);
void tm1637SetBrightness(char brightness);
void tm1637DisplayDecimalPnt(int v, int point);
void tm1637WriteSymbol(unsigned char Symbol, uint8_t pos);
void tm1637WriteSnake(uint32_t SnakeDelay);
void tm1637ClearScreen(void);
void tm1637DisplayDecimal(int v);
void tm1637DisplayDecimalZeroPnt(int v, int point);
