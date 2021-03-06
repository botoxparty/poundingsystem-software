#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32h7xx_hal.h"
#include "lcd.h"
#include "audiocodec.h"

#include "menu.h"
// #include "PS_application.h"

#include "synth.h"

    typedef struct
    {
        I2C_HandleTypeDef *i2c;
        I2S_HandleTypeDef *i2s;
        SPI_HandleTypeDef *spi;
//        UART_HandleTypeDef *uart;
    } PS_Communication_Interface;

    typedef struct
    {
        TIM_HandleTypeDef *encoder1;
        TIM_HandleTypeDef *encoder2;
        TIM_HandleTypeDef *encoder3;
        TIM_HandleTypeDef *encoder4;
    } PS_Encoder_Timers;

    int maincpp(PS_Communication_Interface *comms, PS_Encoder_Timers *enc, LCD_GPIO *lcdGpio);
    void EXTI15_10_IRQHandler(void);

#ifdef __cplusplus
}
#endif
