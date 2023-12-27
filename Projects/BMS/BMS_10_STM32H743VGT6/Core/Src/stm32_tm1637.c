
#include "stm32h7xx_hal.h"
#include "stm32_tm1637.h"

void _tm1637Start(void);
void _tm1637Stop(void);
void _tm1637ReadResult(void);
void _tm1637WriteByte(unsigned char b);
void _tm1637DelayUsec(unsigned int i);
void _tm1637ClkHigh(void);
void _tm1637ClkLow(void);
void _tm1637DioHigh(void);
void _tm1637DioLow(void);

const char segmentMap[] = {
    0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, // 0-7
    0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71, // 8-9, A-F
    0x00, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40  // _|¯|
	, 0x80 // -
};


void tm1637Init(void)
{
    CLK_PORT_CLK_ENABLE();
    DIO_PORT_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};

    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Pin = CLK_PIN;
    HAL_GPIO_Init(CLK_PORT, &g);

//    g.Pull = GPIO_PULLUP;
//    g.Mode = GPIO_MODE_OUTPUT_OD; // OD = open drain
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Pin = DIO_PIN;
    HAL_GPIO_Init(DIO_PORT, &g);

    tm1637SetBrightness(8);
}

void tm1637DisplayDecimalZero(int v, int displaySeparator)
{
    unsigned char digitArr[4];
    for (int i = 0; i < 4; ++i) {
        digitArr[i] = segmentMap[v % 10];
        if (i == 2 && displaySeparator) {
            digitArr[i] |= 1 << 7;
        }
        v /= 10;
    }

    _tm1637Start();
    _tm1637WriteByte(0x40);
    _tm1637ReadResult();
    _tm1637Stop();

    _tm1637Start();
    _tm1637WriteByte(0xC0);
    _tm1637ReadResult();

    for (int i = 0; i < 4; ++i) {
        _tm1637WriteByte(digitArr[3 - i]);
        _tm1637ReadResult();
    }

    _tm1637Stop();
}

void tm1637DisplayDecimalZeroPnt(int v, int point)
{
    unsigned char digitArr[4];

    digitArr[1]=0;
    digitArr[2]=0;
    digitArr[3]=0;
    digitArr[4]=0;

    for (int i = 0; i < 4; ++i) {
        digitArr[i] = segmentMap[v % 10];
//        if (i == 2 && displaySeparator) {
//            digitArr[i] |= 1 << 7;
//        }
        v /= 10;
    }
    switch(point){
       case 1:
        	digitArr[0] |= 0x80;
        break;
        case 2:
        	digitArr[1] |= 0x80;
        break;
        case 3:
        	digitArr[2] |= 0x80;
        break;
        case 4:
        	digitArr[3] |= 0x80;
        break;
    }

    _tm1637Start();
    _tm1637WriteByte(0x40);
    _tm1637ReadResult();
    _tm1637Stop();

    _tm1637Start();
    _tm1637WriteByte(0xC0);
    _tm1637ReadResult();

    for (int i = 0; i < 4; ++i) {
        _tm1637WriteByte(digitArr[3 - i]);
        _tm1637ReadResult();
    }
    _tm1637Stop();
}


void tm1637DisplayDecimalPnt(int v, int point)
{
    unsigned char digitArr[4];

    digitArr[1]=0;
    digitArr[2]=0;
    digitArr[3]=0;
    digitArr[4]=0;

if (v <= 9) {
    for (int i = 0; i < 1; ++i) {
        digitArr[i] = segmentMap[v % 10];
        v /= 10;
    }
}
else if (v <= 99) {
    for (int i = 0; i < 2; ++i) {
        digitArr[i] = segmentMap[v % 10];
        v /= 10;
    }
}
else if (v <= 999) {
    for (int i = 0; i < 3; ++i) {
        digitArr[i] = segmentMap[v % 10];
        v /= 10;
    }
}
else {
    for (int i = 0; i < 4; ++i) {
        digitArr[i] = segmentMap[v % 10];
        v /= 10;
    }
}
    switch(point)
    {
       case 1:
        	digitArr[0] |= 0x80;
        break;
        case 2:
        	digitArr[1] |= 0x80;
        break;
        case 3:
        	digitArr[2] |= 0x80;
        break;
        case 4:
        	digitArr[3] |= 0x80;
        break;
    }

    _tm1637Start();
    _tm1637WriteByte(0x40);
    _tm1637ReadResult();
    _tm1637Stop();

    _tm1637Start();
    _tm1637WriteByte(0xC0);
    _tm1637ReadResult();

    for (int i = 0; i < 4; ++i) {
        _tm1637WriteByte(digitArr[3 - i]);
        _tm1637ReadResult();
    }

    _tm1637Stop();
}

// Valid brightness values: 0 - 8.
// 0 = display off.
void tm1637SetBrightness(char brightness)
{
    // Brightness command:
    // 1000 0XXX = display off
    // 1000 1BBB = display on, brightness 0-7
    // X = don't care
    // B = brightness
    _tm1637Start();
    _tm1637WriteByte(0x87 + brightness);
    _tm1637ReadResult();
    _tm1637Stop();
}

void _tm1637Start(void)
{
    _tm1637ClkHigh();
    _tm1637DioHigh();
    _tm1637DelayUsec(2);
    _tm1637DioLow();
}

void _tm1637Stop(void)
{
    _tm1637ClkLow();
    _tm1637DelayUsec(2);
    _tm1637DioLow();
    _tm1637DelayUsec(2);
    _tm1637ClkHigh();
    _tm1637DelayUsec(2);
    _tm1637DioHigh();
}

void _tm1637ReadResult(void)
{
    _tm1637ClkLow();
    _tm1637DelayUsec(5);
    // while (dio); // We're cheating here and not actually reading back the response.
    _tm1637ClkHigh();
    _tm1637DelayUsec(2);
    _tm1637ClkLow();
}

void _tm1637WriteByte(unsigned char b)
{
    for (int i = 0; i < 8; ++i) {
        _tm1637ClkLow();
        if (b & 0x01) {
            _tm1637DioHigh();
        }
        else {
            _tm1637DioLow();
        }
        _tm1637DelayUsec(3);
        b >>= 1;
        _tm1637ClkHigh();
        _tm1637DelayUsec(3);
    }
}

void _tm1637DelayUsec(unsigned int i)
{
    for (; i>0; i--) {
        for (int j = 0; j < 10; ++j) {
            __asm__ __volatile__("nop\n\t":::"memory");
        }
    }
}

void _tm1637ClkHigh(void)
{
    HAL_GPIO_WritePin(CLK_PORT, CLK_PIN, GPIO_PIN_SET);
}

void _tm1637ClkLow(void)
{
    HAL_GPIO_WritePin(CLK_PORT, CLK_PIN, GPIO_PIN_RESET);
}

void _tm1637DioHigh(void)
{
    HAL_GPIO_WritePin(DIO_PORT, DIO_PIN, GPIO_PIN_SET);
}

void _tm1637DioLow(void)
{
    HAL_GPIO_WritePin(DIO_PORT, DIO_PIN, GPIO_PIN_RESET);
}

void tm1637WriteSymbol(unsigned char Symbol, uint8_t pos) {

    static unsigned char digitArr[4];
    if (Symbol == 'A')
    digitArr[pos] = segmentMap[10];
    else if (Symbol == 'B')
    digitArr[pos] = segmentMap[11];
    else if (Symbol == 'C')
    digitArr[pos] = segmentMap[12];
    else if (Symbol == 'D')
    digitArr[pos] = segmentMap[13];
    else if (Symbol == 'E')
    digitArr[pos] = segmentMap[14];
    else if (Symbol == 'F')
    digitArr[pos] = segmentMap[15];
    else if (Symbol == ' ')
    digitArr[pos] = segmentMap[16];
    else if (Symbol == '0')
    digitArr[pos] = segmentMap[0];
    else if (Symbol == '1')
    digitArr[pos] = segmentMap[1];
    else if (Symbol == '2')
    digitArr[pos] = segmentMap[2];
    else if (Symbol == '3')
    digitArr[pos] = segmentMap[3];
    else if (Symbol == '4')
    digitArr[pos] = segmentMap[4];
    else if (Symbol == '5')
    digitArr[pos] = segmentMap[5];
    else if (Symbol == '6')
    digitArr[pos] = segmentMap[6];
    else if (Symbol == '7')
    digitArr[pos] = segmentMap[7];
    else if (Symbol == '8')
    digitArr[pos] = segmentMap[8];
    else if (Symbol == '9')
    digitArr[pos] = segmentMap[9];
    else if (Symbol == '-')
    digitArr[pos] = segmentMap[23];

    _tm1637Start();
    _tm1637WriteByte(0x40);
    _tm1637ReadResult();
    _tm1637Stop();

    _tm1637Start();
    _tm1637WriteByte(0xC0);
    _tm1637ReadResult();

    for (int i = 0; i < 4; ++i) {
        _tm1637WriteByte(digitArr[3 - i]);
        _tm1637ReadResult();
    }
   _tm1637Stop();
}

void tm1637ClearScreen(void) {
	tm1637WriteSymbol(' ', 0);
	tm1637WriteSymbol(' ', 1);
	tm1637WriteSymbol(' ', 2);
	tm1637WriteSymbol(' ', 3);
}

void tm1637WriteDigitArr(unsigned char digitArr[4]) {
_tm1637Start();
_tm1637WriteByte(0x40);
_tm1637ReadResult();
_tm1637Stop();

_tm1637Start();
_tm1637WriteByte(0xC0);
_tm1637ReadResult();

for (int i = 0; i < 4; ++i) {
    _tm1637WriteByte(digitArr[3 - i]);
    _tm1637ReadResult();
	}
	_tm1637Stop();
}

void  tm1637WriteSnake(uint32_t SnakeDelay) {
	unsigned char digitArr[4];

	digitArr[0] = segmentMap[16];
	digitArr[1] = segmentMap[16];
	digitArr[2] = segmentMap[16];
	digitArr[3] = segmentMap[16];
	tm1637WriteDigitArr(digitArr);

	digitArr[0] = segmentMap[17];
	tm1637WriteDigitArr(digitArr);
	HAL_Delay(SnakeDelay);

	digitArr[0] = segmentMap[18];
	tm1637WriteDigitArr(digitArr);
	HAL_Delay(SnakeDelay);

	digitArr[0] = segmentMap[19];
	tm1637WriteDigitArr(digitArr);
	HAL_Delay(SnakeDelay);

	digitArr[0] = segmentMap[20];
	tm1637WriteDigitArr(digitArr);
	HAL_Delay(SnakeDelay);

	digitArr[0] = segmentMap[16];
	digitArr[1] = segmentMap[20];
	tm1637WriteDigitArr(digitArr);
	HAL_Delay(SnakeDelay);

	digitArr[1] = segmentMap[16];
	digitArr[2] = segmentMap[20];
	tm1637WriteDigitArr(digitArr);
	HAL_Delay(SnakeDelay);

	digitArr[2] = segmentMap[16];
	digitArr[3] = segmentMap[20];
	tm1637WriteDigitArr(digitArr);
	HAL_Delay(SnakeDelay);

	digitArr[3] = segmentMap[21];
	tm1637WriteDigitArr(digitArr);
	HAL_Delay(SnakeDelay);

	digitArr[3] = segmentMap[22];
	tm1637WriteDigitArr(digitArr);
	HAL_Delay(SnakeDelay);

	digitArr[3] = segmentMap[17];
	tm1637WriteDigitArr(digitArr);
	HAL_Delay(SnakeDelay);

	digitArr[3] = segmentMap[16];
	digitArr[2] = segmentMap[17];
	tm1637WriteDigitArr(digitArr);
	HAL_Delay(SnakeDelay);

	digitArr[2] = segmentMap[16];
	digitArr[1] = segmentMap[17];
	tm1637WriteDigitArr(digitArr);
	HAL_Delay(SnakeDelay);
}

void tm1637DisplayDecimal(int v)
{
    unsigned char digitArr[4];
    digitArr[1]=0;
    digitArr[2]=0;
    digitArr[3]=0;
    digitArr[4]=0;

if (v <= 9) {
    for (int i = 0; i < 1; ++i) {
        digitArr[i] = segmentMap[v % 10];
        if (i == 2) {
            digitArr[i] |= 1 << 7;
        }
        v /= 10;
    }
}
else if (v <= 99) {
    for (int i = 0; i < 2; ++i) {
        digitArr[i] = segmentMap[v % 10];
        if (i == 2) {
            digitArr[i] |= 1 << 7;
        }
        v /= 10;
    }
}
else if (v <= 999) {
    for (int i = 0; i < 3; ++i) {
        digitArr[i] = segmentMap[v % 10];
        if (i == 2) {
            digitArr[i] |= 1 << 7;
        }
        v /= 10;
    }
}
else {
    for (int i = 0; i < 4; ++i) {
        digitArr[i] = segmentMap[v % 10];
        if (i == 2) {
            digitArr[i] |= 1 << 7;
        }
        v /= 10;
    }
}
    _tm1637Start();
    _tm1637WriteByte(0x40);
    _tm1637ReadResult();
    _tm1637Stop();

    _tm1637Start();
    _tm1637WriteByte(0xC0);
    _tm1637ReadResult();

    for (int i = 0; i < 4; ++i) {
        _tm1637WriteByte(digitArr[3 - i]);
        _tm1637ReadResult();
    }

    _tm1637Stop();
}




