
#ifndef _PS_MENU_
#define _PS_MENU_

#include "lcd.h"
#include "synth.h"

#ifdef __cplusplus

enum PSMenus
{
    MAIN_MENU,
    MIXER_MENU,
    SOUND_MENU,
    DELAY_MENU,
    OSC_MENU
};

extern Synth *synth;

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

class MenuSystem
{
public:
    MenuSystem(LCD *hlcd);

    void SetMenu(PSMenus menu);
    void TriggerScrollEncoder(TIM_HandleTypeDef* timer);
    void TriggerPushEncoder();
    void EncoderCallback(TIM_HandleTypeDef* timer);

    void MenuSelect();
    void RefreshMenu();
    void MenuFooter();
    void UpdateValue(TIM_HandleTypeDef* timer);

    void MainMenu();
    void MixerMenu();
    void SoundMenu();
    void DelayMenu();
    void OscMenu();

private:
    LCD *lcd;
};
#endif
#endif
