#ifndef __SH1106_H__
#define __SH1106_H__

#include "stm32f1xx_hal.h"
#include "fonts.h"
#include <string.h>

#define SH1106_WIDTH 128
#define SH1106_HEIGHT 64

void SH1106_Init(void);
void SH1106_UpdateScreen(void);
void SH1106_Clear(void);
void SH1106_Fill(uint8_t color);
void SH1106_DrawPixel(uint8_t x, uint8_t y, uint8_t color);
void SH1106_GotoXY(uint8_t x, uint8_t y);
void SH1106_Putc(char ch, const FontDef_t *Font, uint8_t color);
void SH1106_Puts(const char *str, const FontDef_t *Font, uint8_t color);

#endif
