#include "OLED.h"
#include "OLED_Font.h"

/* --- ?? I2C ?????? --- */

static void OLED_I2C_Start(void)
{
    OLED_SDA_Set();
    OLED_SCL_Set();
    OLED_SDA_Clr();
    OLED_SCL_Clr();
}

static void OLED_I2C_Stop(void)
{
    OLED_SDA_Clr();
    OLED_SCL_Set();
    OLED_SDA_Set();
}

static void OLED_I2C_WriteByte(uint8_t Byte)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        if (Byte & (0x80 >> i))
        {
            OLED_SDA_Set();
        }
        else
        {
            OLED_SDA_Clr();
        }
        OLED_SCL_Set();
        OLED_SCL_Clr();
    }
    OLED_SCL_Set(); // ????????
    OLED_SCL_Clr();
}

/* --- OLED ??/???? --- */

static void OLED_WriteCommand(uint8_t Command)
{
    OLED_I2C_Start();
    OLED_I2C_WriteByte(0x78); // OLED I2C ????
    OLED_I2C_WriteByte(0x00); // ???????
    OLED_I2C_WriteByte(Command);
    OLED_I2C_Stop();
}

static void OLED_WriteData(uint8_t Data)
{
    OLED_I2C_Start();
    OLED_I2C_WriteByte(0x78);
    OLED_I2C_WriteByte(0x40); // ???????
    OLED_I2C_WriteByte(Data);
    OLED_I2C_Stop();
}

/* --- ?????? --- */
static void OLED_SetCursor(uint8_t Y, uint8_t X)
{
    OLED_WriteCommand(0xB0 | Y);                 // ?????(0~7)
    OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4)); // ???????
    OLED_WriteCommand(0x00 | (X & 0x0F));        // ???????
}

/* --- ?? --- */
void OLED_Clear(void)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        OLED_SetCursor(i, 0);
        for (uint8_t j = 0; j < 128; j++)
        {
            OLED_WriteData(0x00);
        }
    }
}

/* --- ?????? --- */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{
    uint8_t c = Char - ' '; // ?????????????
    OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8);
    for (uint8_t i = 0; i < 8; i++)
    {
        OLED_WriteData(OLED_F8x16[c][i]);
    }
    OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8);
    for (uint8_t i = 0; i < 8; i++)
    {
        OLED_WriteData(OLED_F8x16[c][i + 8]);
    }
}

/* --- ????? --- */
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
    for (uint8_t i = 0; String[i] != '\0'; i++)
    {
        OLED_ShowChar(Line, Column + i, String[i]);
    }
}

/* --- ?????? --- */
static uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--) Result *= X;
    return Result;
}

/* --- ??????? --- */
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    for (uint8_t i = 0; i < Length; i++)
    {
        OLED_ShowChar(Line, Column + i, Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
    }
}

/* --- ??????? --- */
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
    uint32_t Number1;
    if (Number >= 0)
    {
        OLED_ShowChar(Line, Column, '+');
        Number1 = Number;
    }
    else
    {
        OLED_ShowChar(Line, Column, '-');
        Number1 = -Number;
    }
    for (uint8_t i = 0; i < Length; i++)
    {
        OLED_ShowChar(Line, Column + i + 1, Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0');
    }
}

/* --- OLED ????? --- */
void OLED_Init(void)
{
    HAL_Delay(100); // ??????????
    
    OLED_WriteCommand(0xAE); // ????
    OLED_WriteCommand(0xD5); // ??????
    OLED_WriteCommand(0x80);
    OLED_WriteCommand(0xA8); // ?????
    OLED_WriteCommand(0x3F);
    OLED_WriteCommand(0xD3); // ??????
    OLED_WriteCommand(0x00);
    OLED_WriteCommand(0x40); // ?????
    OLED_WriteCommand(0x8D); // ?????
    OLED_WriteCommand(0x14);
    OLED_WriteCommand(0x20); // ??????
    OLED_WriteCommand(0x02);
    OLED_WriteCommand(0xA1); // ????? (????)
    OLED_WriteCommand(0xC8); // COM???? (????)
    OLED_WriteCommand(0xDA); // COM????
    OLED_WriteCommand(0x12);
    OLED_WriteCommand(0x81); // ?????
    OLED_WriteCommand(0xCF);
    OLED_WriteCommand(0xD9); // ?????
    OLED_WriteCommand(0xF1);
    OLED_WriteCommand(0xDB); // VCOMH ????
    OLED_WriteCommand(0x30);
    OLED_WriteCommand(0xA4); // ????/??
    OLED_WriteCommand(0xA6); // ????
    OLED_WriteCommand(0xAF); // ????

    OLED_Clear();            // ??
}