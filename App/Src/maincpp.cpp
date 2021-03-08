#include "maincpp.h"
// class class_that_needs_i2c{
//     //ctor
//     class_that_needs_i2c(i2c_enc& i2c){
//         i2c.DR
//     }
// };

uint16_t audiobuff[BUFF_LEN];
Synth *synth = Synth::getInstance();
MenuSystem *menu;
AudioCodec *audio;
bool sw1aon;
bool sw1bon;
bool sw1dir;
uint16_t sw1counter = 0;
extern TIM_HandleTypeDef htim1;

int maincpp(PS_Communication_Interface *comms, PS_Encoder_Timers *enc, LCD_GPIO *lcdGpio)
{

  LCD lcd(lcdGpio);
  MenuSystem m(&lcd);
  menu = &m;
  menu->MainMenu();

  AudioCodec a(comms->i2c, comms->i2s);
  audio = &a;



  // make use of i2c struct
  while (1)
  {
    // PS_Application();
  }
  return 0;
}

/**
  * @brief  Tx Transfer completed callbacks.
  * @param  hi2s: I2S handle
  * @retval None
  */
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  if (hi2s->Instance == SPI1)
  {
    /* Call the user function which will manage directly transfer complete */
    synth->make_sound((uint16_t *)(audiobuff + BUFF_LEN_DIV2), BUFF_LEN_DIV4);
  }
}

/**
  * @brief  Tx Half Transfer completed callbacks.
  * @param  hi2s: I2S handle
  * @retval None
  */
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
  if (hi2s->Instance == SPI1)
  {
    /* Manage the remaining file size and new address offset: This function should
       be coded by user (its prototype is already declared in stm32f4_discovery_audio.h) */
    synth->make_sound((uint16_t *)audiobuff, BUFF_LEN_DIV4);
  }
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
  if (__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_13))
  { // SW1_A
    if (sw1bon)
    {
      sw1dir = true;
      sw1bon = false;
    }
    else
    {
      sw1aon = true;
    }
  }
  if (__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_14))
  { // SW1_B
    if (sw1aon)
    {
      sw1dir = false;
      sw1aon = false;
    }
    else
    {
      sw1bon = true;
    }
    menu->TriggerEncoder(sw1dir);
  }
  if (__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_15))
  {
    menu->TriggerPushEncoder();
  }
  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */
  menu->EncoderCallback(&htim1);
  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}
