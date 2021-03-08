
#ifndef _LCD_PCD8544_
#define _LCD_PCD8544_

#include <stdbool.h>

#ifdef __cplusplus

#include "stm32h7xx_hal.h"

// Fonts
#include "ascii.h"
#include "ascii_3.h"
#include "ascii_4.h"

extern "C"
{
#endif

#define LCD_COMMAND 0
#define LCD_DATA 1

#define LCD_SETYADDR 0x40
#define LCD_SETXADDR 0x80
#define LCD_DISPLAY_BLANK 0x08
#define LCD_DISPLAY_NORMAL 0x0C
#define LCD_DISPLAY_ALL_ON 0x09
#define LCD_DISPLAY_INVERTED 0x0D

#define LCD_WIDTH 84
#define LCD_HEIGHT 48
#define LCD_SIZE LCD_WIDTH *LCD_HEIGHT / 8

	typedef struct
	{
		uint8_t buffer[LCD_SIZE];
		bool inverttext;
	} LCD_att;

	typedef struct
	{
		GPIO_TypeDef *CEPORT;
		uint16_t CEPIN;

		GPIO_TypeDef *DCPORT;
		uint16_t DCPIN;

		GPIO_TypeDef *BLPORT;
		uint16_t BLPIN;

		SPI_HandleTypeDef *SPICH;
	} LCD_GPIO;

#ifdef __cplusplus
	class LCD
	{
	public:
		LCD(LCD_GPIO *hgpio);
		void print(char *str, uint8_t x, uint8_t y);
		void clrScr();
		void goXY(uint8_t x, uint8_t y);
		void drawBitmap(int x, int y, uint8_t *bitmap, int sx, int sy);
		void putChar(char c);
		void invert(bool mode);
		void invertText(bool mode);
		void drawHLine(int x, int y, int l);
		void drawVLine(int x, int y, int l);
		void drawLine(int x1, int y1, int x2, int y2);
		void drawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);
		void refreshScr();
		void setFont(const uint8_t *f);

	private:
		LCD_GPIO *gpio;
		LCD_att display;
		void init();
		void send(uint8_t val);
		void write(uint8_t data, uint8_t mode);
		void refreshArea(uint8_t xmin, uint8_t ymin, uint8_t xmax, uint8_t ymax);
		void setPixel(uint8_t x, uint8_t y, bool pixel);
		int abs(int x);
	};
}

#endif

#endif
