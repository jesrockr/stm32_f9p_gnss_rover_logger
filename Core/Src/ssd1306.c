#include "ssd1306.h"
#include "i2c.h"
#include "font6x8.h"
#include <string.h>

#define SSD1306_ADDR (0x3C << 1)

static uint8_t buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

static uint8_t cursor_x = 0;
static uint8_t cursor_y = 0;

/* ================= LOW LEVEL ================= */

static void write_cmd(uint8_t cmd)
{
    uint8_t data[2] = {0x00, cmd};
    HAL_I2C_Master_Transmit(&hi2c1, SSD1306_ADDR, data, 2, 50);
}

static void write_data(uint8_t* data, size_t size)
{
    uint8_t tmp[129];
    tmp[0] = 0x40;

    if (size > 128) size = 128;

    memcpy(&tmp[1], data, size);
    HAL_I2C_Master_Transmit(&hi2c1, SSD1306_ADDR, tmp, size + 1, 100);
}

/* ================= INIT ================= */

void SSD1306_Init(void)
{
    HAL_Delay(100);

    write_cmd(0xAE);
    write_cmd(0x20); write_cmd(0x00);
    write_cmd(0xB0);
    write_cmd(0xC8);
    write_cmd(0x00);
    write_cmd(0x10);
    write_cmd(0x40);
    write_cmd(0x81); write_cmd(0x7F);
    write_cmd(0xA1);
    write_cmd(0xA6);
    write_cmd(0xA8); write_cmd(0x3F);
    write_cmd(0xD3); write_cmd(0x00);
    write_cmd(0xD5); write_cmd(0x80);
    write_cmd(0xD9); write_cmd(0xF1);
    write_cmd(0xDA); write_cmd(0x12);
    write_cmd(0xDB); write_cmd(0x40);
    write_cmd(0x8D); write_cmd(0x14);
    write_cmd(0xAF);

    SSD1306_Clear();
}

/* ================= FRAMEBUFFER ================= */

void SSD1306_Clear(void)
{
    memset(buffer, 0x00, sizeof(buffer));
    cursor_x = 0;
    cursor_y = 0;
}

void SSD1306_UpdateScreen(void)
{
    for (uint8_t page = 0; page < 8; page++)
    {
        write_cmd(0xB0 + page);
        write_cmd(0x00);
        write_cmd(0x10);

        write_data(&buffer[128 * page], 128);
    }
}

/* ================= PIXEL ================= */

void SSD1306_DrawPixel(uint8_t x, uint8_t y, uint8_t color)
{
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) return;

    if (color)
        buffer[x + (y / 8) * 128] |= (1 << (y % 8));
    else
        buffer[x + (y / 8) * 128] &= ~(1 << (y % 8));
}

/* ================= TEXT ================= */

void SSD1306_SetCursor(uint8_t x, uint8_t y)
{
    cursor_x = x;
    cursor_y = y;
}

void SSD1306_WriteChar(char ch)
{
    if (ch < 32 || ch > 127) return;
    if (cursor_x > (SSD1306_WIDTH - 6) || cursor_y > 7) return;

    for (uint8_t i = 0; i < 6; i++)
    {
        buffer[cursor_x + (cursor_y * SSD1306_WIDTH) + i] =
            Font6x8[ch - 32][i];
    }

    cursor_x += 6;
}

void SSD1306_WriteString(const char* str)
{
    while (*str)
    {
        SSD1306_WriteChar(*str++);
    }
}
