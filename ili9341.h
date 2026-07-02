#ifndef INC_ILI9341_H_
#define INC_ILI9341_H_

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ILI9341 portrait resolution */
#define ILI9341_WIDTH   240U
#define ILI9341_HEIGHT  320U

/* RGB565 colors */
#define ILI9341_BLACK       0x0000U
#define ILI9341_NAVY        0x000FU
#define ILI9341_DARKGREEN   0x03E0U
#define ILI9341_DARKCYAN    0x03EFU
#define ILI9341_MAROON      0x7800U
#define ILI9341_PURPLE      0x780FU
#define ILI9341_OLIVE       0x7BE0U
#define ILI9341_LIGHTGREY   0xC618U
#define ILI9341_DARKGREY    0x7BEFU
#define ILI9341_BLUE        0x001FU
#define ILI9341_GREEN       0x07E0U
#define ILI9341_CYAN        0x07FFU
#define ILI9341_RED         0xF800U
#define ILI9341_MAGENTA     0xF81FU
#define ILI9341_YELLOW      0xFFE0U
#define ILI9341_WHITE       0xFFFFU
#define ILI9341_ORANGE      0xFD20U
#define ILI9341_GREENYELLOW 0xAFE5U
#define ILI9341_PINK        0xFC18U

/*
 * Header files contain declarations only.
 * Function bodies belong in ili9341.c.
 */
void ILI9341_Init(void);
void ILI9341_FillScreen(uint16_t color);
void ILI9341_TestPattern(void);

void ILI9341_FillRect(
    uint16_t x,
    uint16_t y,
    uint16_t width,
    uint16_t height,
    uint16_t color
);

void ILI9341_DrawPixel(
    uint16_t x,
    uint16_t y,
    uint16_t color
);

#ifdef __cplusplus
}
#endif

#endif /* INC_ILI9341_H_ */
