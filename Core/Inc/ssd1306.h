#ifndef SSD1306_H
#define SSD1306_H

#include "main.h"
#include <stdint.h>

#define SSD1306_WIDTH  128
#define SSD1306_HEIGHT 64

void SSD1306_Init(void);
void SSD1306_UpdateScreen(void);
void SSD1306_Clear(void);

void SSD1306_DrawPixel(uint8_t x, uint8_t y, uint8_t color);

void SSD1306_SetCursor(uint8_t x, uint8_t y);
void SSD1306_WriteChar(char ch);
void SSD1306_WriteString(const char* str);

#endif
