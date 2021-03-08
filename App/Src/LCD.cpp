#include "lcd.h"
#include "stm32h7xx.h"

LCD::LCD(LCD_GPIO *hgpio)
{
    if (hgpio == 0x00)
    {
        // throw std::runtime_exeption("pointer is null");
    }
    gpio = hgpio;
    HAL_GPIO_WritePin(gpio->BLPORT, gpio->BLPIN, GPIO_PIN_SET);
    init();
}

void LCD::print(char *str, uint8_t x, uint8_t y)
{
    goXY(x, y);
    while (*str)
    {
        putChar(*str++);
    }
};
void LCD::clrScr()
{
    for (int i = 0; i < 504; i++)
    {
        write(0x00, LCD_DATA);
        display.buffer[i] = 0;
    }
}
void LCD::goXY(uint8_t x, uint8_t y)
{
    write(0x80 | x, LCD_COMMAND); //Column.
    write(0x40 | y, LCD_COMMAND); //Row.
};
void LCD::drawBitmap(int x, int y, uint8_t *bitmap, int sx, int sy)
{
    // int bit;
    // int data;

    // for (int cy = 0; cy < sy; cy++)
    // {
    //     bit = cy % 8;
    //     for (int cx = 0; cx < sx; cx++)
    //     {
    //         data = bitmapbyte(cx + ((cy / 8) * sx));
    //         if ((data & (1 << bit)) > 0)
    //             setPixel(x + cx, y + cy, true);
    //         else
    //             setPixel(x + cx, y + cy, false);
    //     }
    // }

    // refreshScr();
}


void LCD::putChar(char c)
{
    for (int i = 0; i < 6; i++)
    {
        if (display.inverttext != true)
            write(ASCII[c - 0x20][i], LCD_DATA);
        else
            write(~(ASCII[c - 0x20][i]), LCD_DATA);
    }
}
void LCD::invert(bool mode)
{
    if (mode == true)
    {
        write(LCD_DISPLAY_INVERTED, LCD_COMMAND);
    }
    else
    {
        write(LCD_DISPLAY_NORMAL, LCD_COMMAND);
    }
}
void LCD::invertText(bool mode)
{
    display.inverttext = mode;
}

void LCD::init()
{
    write(0x21, LCD_COMMAND);               //LCD extended commands.
    write(0xBE, LCD_COMMAND);               //set LCD Vop(Contrast).
    write(0x04, LCD_COMMAND);               //set temp coefficent.
    write(0x14, LCD_COMMAND);               //LCD bias mode 1:40.
    write(0x20, LCD_COMMAND);               //LCD basic commands.
    write(LCD_DISPLAY_NORMAL, LCD_COMMAND); //LCD normal.
    clrScr();
    display.inverttext = false;
}

void LCD::setFont(const uint8_t *f)
{
}

void LCD::send(uint8_t val)
{
    HAL_StatusTypeDef status = HAL_SPI_Transmit(gpio->SPICH, &val, 0x01, 1000);

    if(status != HAL_OK) {
        return;
    } else {
    	return;
    }
};
void LCD::write(uint8_t data, uint8_t mode)
{
    if (mode == LCD_COMMAND)
    {
        HAL_GPIO_WritePin(gpio->DCPORT, gpio->DCPIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(gpio->CEPORT, gpio->CEPIN, GPIO_PIN_RESET);
        send(data);
        HAL_GPIO_WritePin(gpio->CEPORT, gpio->CEPIN, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(gpio->DCPORT, gpio->DCPIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(gpio->CEPORT, gpio->CEPIN, GPIO_PIN_RESET);
        send(data);
        HAL_GPIO_WritePin(gpio->CEPORT, gpio->CEPIN, GPIO_PIN_SET);
    }
}
void LCD::refreshScr()
{
    goXY(LCD_SETXADDR, LCD_SETYADDR);
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < LCD_WIDTH; j++)
        {
            write(display.buffer[(i * LCD_WIDTH) + j], LCD_DATA);
        }
    }
};
void LCD::refreshArea(uint8_t xmin, uint8_t ymin, uint8_t xmax, uint8_t ymax)
{
    for (int i = 0; i < 6; i++)
    {
        if (i * 8 > ymax)
        {
            break;
        }
        //LCD_goXY(xmin, i);
        write(LCD_SETYADDR | i, LCD_COMMAND);
        write(LCD_SETXADDR | xmin, LCD_COMMAND);
        for (int j = xmin; j <= xmax; j++)
        {
            write(display.buffer[(i * LCD_WIDTH) + j], LCD_DATA);
        }
    }
};
void LCD::setPixel(uint8_t x, uint8_t y, bool pixel)
{
    if (x >= LCD_WIDTH)
        x = LCD_WIDTH - 1;
    if (y >= LCD_HEIGHT)
        y = LCD_HEIGHT - 1;

    if (pixel != false)
    {
        display.buffer[x + (y / 8) * LCD_WIDTH] |= 1 << (y % 8);
    }
    else
    {
        display.buffer[x + (y / 8) * LCD_WIDTH] &= ~(1 << (y % 8));
    }
};

void LCD::drawHLine(int x, int y, int l)
{
    int by, bi;

    if ((x >= 0) && (x < LCD_WIDTH) && (y >= 0) && (y < LCD_HEIGHT))
    {
        for (int cx = 0; cx < l; cx++)
        {
            by = ((y / 8) * 84) + x;
            bi = y % 8;
            display.buffer[by + cx] |= (1 << bi);
        }
    }
};
void LCD::drawVLine(int x, int y, int l)
{

    if ((x >= 0) && (x < 84) && (y >= 0) && (y < 48))
    {
        for (int cy = 0; cy <= l; cy++)
        {
            setPixel(x, y + cy, true);
        }
    }
};
/*
 * @brief abs function used in LCD_drawLine
 * @param x: any integer
 * @return absolute value of x
 */
int LCD::abs(int x)
{
    if (x < 0)
    {
        return x * -1;
    }
    return x;
}
void LCD::drawLine(int x1, int y1, int x2, int y2)
{
    int tmp;
    double delta, tx, ty;

    if (((x2 - x1) < 0))
    {
        tmp = x1;
        x1 = x2;
        x2 = tmp;
        tmp = y1;
        y1 = y2;
        y2 = tmp;
    }
    if (((y2 - y1) < 0))
    {
        tmp = x1;
        x1 = x2;
        x2 = tmp;
        tmp = y1;
        y1 = y2;
        y2 = tmp;
    }

    if (y1 == y2)
    {
        if (x1 > x2)
        {
            tmp = x1;
            x1 = x2;
            x2 = tmp;
        }
        drawHLine(x1, y1, x2 - x1);
    }
    else if (x1 == x2)
    {
        if (y1 > y2)
        {
            tmp = y1;
            y1 = y2;
            y2 = tmp;
        }
        drawHLine(x1, y1, y2 - y1);
    }
    else if (abs(x2 - x1) > abs(y2 - y1))
    {
        delta = ((double)(y2 - y1) / (double)(x2 - x1));
        ty = (double)y1;
        if (x1 > x2)
        {
            for (int i = x1; i >= x2; i--)
            {
                setPixel(i, (int)(ty + 0.5), true);
                ty = ty - delta;
            }
        }
        else
        {
            for (int i = x1; i <= x2; i++)
            {
                setPixel(i, (int)(ty + 0.5), true);
                ty = ty + delta;
            }
        }
    }
    else
    {
        delta = ((float)(x2 - x1) / (float)(y2 - y1));
        tx = (float)(x1);
        if (y1 > y2)
        {
            for (int i = y2 + 1; i > y1; i--)
            {
                setPixel((int)(tx + 0.5), i, true);
                tx = tx + delta;
            }
        }
        else
        {
            for (int i = y1; i < y2 + 1; i++)
            {
                setPixel((int)(tx + 0.5), i, true);
                tx = tx + delta;
            }
        }
    }
};
void LCD::drawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2)
{
    drawLine(x1, y1, x2, y1);
    drawLine(x1, y1, x1, y2);
    drawLine(x2, y1, x2, y2);
    drawLine(x1, y2, x2, y2);
};
