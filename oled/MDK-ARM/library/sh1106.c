#include "sh1106.h"

// B? d?m
static uint8_t SH1106_Buffer[SH1106_WIDTH * (SH1106_HEIGHT / 8)];
static uint16_t CurrentX = 0;
static uint16_t CurrentY = 0;

// ----- Giao ti?p I2C -----
extern I2C_HandleTypeDef hi2c1;
#define SH1106_I2C_ADDR (0x3C << 1)

// G?i l?nh
static void SH1106_WriteCommand(uint8_t cmd) {
    uint8_t buf[2] = {0x00, cmd};
    HAL_I2C_Master_Transmit(&hi2c1, SH1106_I2C_ADDR, buf, 2, 100);
}

// G?i d? li?u
static void SH1106_WriteData(uint8_t *data, uint16_t size) {
    uint8_t buffer[size + 1];
    buffer[0] = 0x40;
    memcpy(&buffer[1], data, size);
    HAL_I2C_Master_Transmit(&hi2c1, SH1106_I2C_ADDR, buffer, size + 1, 100);
}

// ----- Các hàm chính -----
void SH1106_Fill(uint8_t color) {
    memset(SH1106_Buffer, (color ? 0xFF : 0x00), sizeof(SH1106_Buffer));
}

void SH1106_Clear(void) {
    SH1106_Fill(0);
    SH1106_UpdateScreen();
}

void SH1106_Init(void) {
    HAL_Delay(100);

    SH1106_WriteCommand(0xAE);
    SH1106_WriteCommand(0xD5);
    SH1106_WriteCommand(0x80);
    SH1106_WriteCommand(0xA8);
    SH1106_WriteCommand(0x3F);
    SH1106_WriteCommand(0xD3);
    SH1106_WriteCommand(0x00);
    SH1106_WriteCommand(0x40);
    SH1106_WriteCommand(0xAD);
    SH1106_WriteCommand(0x8B);
    SH1106_WriteCommand(0xA1);
    SH1106_WriteCommand(0xC8);
    SH1106_WriteCommand(0xDA);
    SH1106_WriteCommand(0x12);
    SH1106_WriteCommand(0x81);
    SH1106_WriteCommand(0xCF);
    SH1106_WriteCommand(0xD9);
    SH1106_WriteCommand(0xF1);
    SH1106_WriteCommand(0xDB);
    SH1106_WriteCommand(0x40);
    SH1106_WriteCommand(0xA4);
    SH1106_WriteCommand(0xA6);
    SH1106_WriteCommand(0xAF);

    SH1106_Clear();
}

void SH1106_UpdateScreen(void) {
    for (uint8_t page = 0; page < (SH1106_HEIGHT / 8); page++) {
        SH1106_WriteCommand(0xB0 + page);
        SH1106_WriteCommand(0x02);
        SH1106_WriteCommand(0x10);
        SH1106_WriteData(&SH1106_Buffer[page * SH1106_WIDTH], SH1106_WIDTH);
    }
}

void SH1106_DrawPixel(uint8_t x, uint8_t y, uint8_t color) {
    if (x >= SH1106_WIDTH || y >= SH1106_HEIGHT) return;
    x += 2; // Offset cho SH1106
    if (x >= SH1106_WIDTH) return;

    uint16_t index = x + (y / 8) * SH1106_WIDTH;
    if (color)
        SH1106_Buffer[index] |= (1 << (y % 8));
    else
        SH1106_Buffer[index] &= ~(1 << (y % 8));
}

void SH1106_GotoXY(uint8_t x, uint8_t y) {
    CurrentX = x;
    CurrentY = y;
}

void SH1106_Putc(char ch, const FontDef_t *Font, uint8_t color) {
    if ((ch < 32) || (ch > 126)) return;
    for (uint8_t i = 0; i < Font->FontHeight; i++) {
        uint16_t line = Font->data[(ch - 32) * Font->FontHeight + i];
        for (uint8_t j = 0; j < Font->FontWidth; j++) {
            if (line & (1 << (15 - j))) {
                SH1106_DrawPixel(CurrentX + j, CurrentY + i, color);
            } else {
                SH1106_DrawPixel(CurrentX + j, CurrentY + i, !color);
            }
        }
    }
    CurrentX += Font->FontWidth;
}
void SH1106_DrawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color)
{
    for (uint8_t x = x1; x <= x2; x++) {
        SH1106_DrawPixel(x, y1, color);
        SH1106_DrawPixel(x, y2, color);
    }
    for (uint8_t y = y1; y <= y2; y++) {
        SH1106_DrawPixel(x1, y, color);
        SH1106_DrawPixel(x2, y, color);
    }
}


void SH1106_Puts(const char *str, const FontDef_t *Font, uint8_t color) {
    while (*str) {
        SH1106_Putc(*str++, Font, color);
    }
}
