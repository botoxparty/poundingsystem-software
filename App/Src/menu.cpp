#include "main.h"
#include "lcd.h"
#include "menu.h"
#include "audiocodec.h"
#include "soundgenerator.h"
#include<stdio.h>

/**
 *
 * Menus:
 * 0 main menu
 * 1 mixer menu
 * 2 sound menu
 * 3 delay menu
 * 4 osc menu
 *
 */

PSMenus currentMenu = MAIN_MENU;
int menuCount = 4;
int activeItem = 0;
int activeItemCount = 3;
bool scrollMode = false;
bool looping = true;

extern uint8_t WM8978_SPK_Volume;
extern AudioCodec *audio;

char *intToChar(int i)
{
    int length = (log10(i) + 1);

    char chararray[length];
    sprintf(chararray, "%d", i);
    chararray[length] = '\0';

    return chararray;
}

MenuSystem::MenuSystem(LCD *hlcd)
{
    lcd = hlcd;
}

/**
  * @brief Load the current state onto the display
  */
void MenuSystem::RefreshMenu(void)
{
    lcd->clrScr();
    switch (currentMenu)
    {
    case MAIN_MENU:
        MainMenu();
        break;
    case MIXER_MENU:
        MixerMenu();
        break;
    case SOUND_MENU:
        SoundMenu();
        break;
    case DELAY_MENU:
        DelayMenu();
        break;
    case OSC_MENU:
        OscMenu();
        break;
    default:
        break;
    }

    if (scrollMode)
    {
        lcd->invertText(true);
        lcd->print("<            >", 0, 5);
        lcd->invertText(false);
    }
}

/**
  * @brief Load the current state onto the display
  */
void MenuSystem::SetMenu(PSMenus menu)
{
    currentMenu = menu;
    RefreshMenu();
}

void MenuSystem::MenuSelect(void)
{
    if (scrollMode)
    {
        scrollMode = false;
        RefreshMenu();
        return;
    }
    switch (currentMenu)
    {
    case MIXER_MENU:
        activeItem = 1;
        SetMenu(MAIN_MENU);
        return;
        break;
    case MAIN_MENU:
        if (activeItem == 2)
        {
            activeItem = synth->sound;
            SetMenu(SOUND_MENU);
            return;
        }
        break;
    case SOUND_MENU:
        synth->sound = Sounds(activeItem);
        activeItem = 2;
        SetMenu(MAIN_MENU);
        break;
    default:
        break;
    }
    switch (activeItem)
    {
    case 0:
        // set to scroll thru pages
        scrollMode = true;
        RefreshMenu();
        break;
    case 1:
        // set previous menu to current menu
        // show mixer menu
        SetMenu(MIXER_MENU);
        break;
    default:
        // switch (currentMenu)
        // {
        // case MAIN_MENU:
        //     break;
        // case MIXER_MENU:
        //     break;
        // default:
        //     break;
        // }
        break;
    }
}

/**
  * @brief Go to next/prev menu page.
  */
void MenuSystem::TriggerEncoder(bool direction)
{
    if (scrollMode)
    {
        int newmenu = (int)currentMenu;

        if (direction)
        {
            newmenu--;
            if (newmenu > 4)
            {
                newmenu = 4;
            }
        }
        else
        {
            newmenu++;
            if (newmenu > 4)
            {
                newmenu = 0;
            }
        }

        SetMenu((PSMenus)newmenu);
        return;
    }

    if (direction)
    {
        activeItem++;
        if (activeItem > activeItemCount - 1)
        {
            activeItem = looping ? 0 : activeItemCount - 1;
        }
    }
    else
    {
        activeItem--;
        if (activeItem < 0)
        {
            activeItem = looping ? activeItemCount - 1 : 0;
        }
    }
    RefreshMenu();
}

void MenuSystem::TriggerPushEncoder()
{
    MenuSelect();
}

/**
  * @brief Mixer
  */
void MenuSystem::EncoderCallback(TIM_HandleTypeDef *timer)
{
    timer->Instance->CNT;
    switch (currentMenu)
    {
    case MAIN_MENU:

        break;
    }
}

/**
  * @brief Main Menu
  */
void MenuSystem::MainMenu()
{
    currentMenu = MAIN_MENU;
    activeItemCount = 3;
    switch (activeItem)
    {
    case 0:
        lcd->print(SoundGenerator::soundNames[synth->sound], 0, 0);
        lcd->print("Freq       440", 0, 1);
        lcd->print("Mod        100", 0, 2);
        lcd->print("Rate      1200", 0, 3);
        lcd->print("- - - -       ", 0, 4);
        lcd->print("         MIXER", 0, 5);
        lcd->invertText(true);
        lcd->print("<  >", 0, 5);
        lcd->invertText(false);
        break;
    case 1:
        lcd->print(SoundGenerator::soundNames[synth->sound], 0, 0);
        lcd->print("Freq       440", 0, 1);
        lcd->print("Mod        100", 0, 2);
        lcd->print("Rate      1200", 0, 3);
        lcd->print("- - - -       ", 0, 4);
        lcd->print("<  >", 0, 5);
        lcd->invertText(true);
        lcd->print("MIXER", 54, 5);
        lcd->invertText(false);
        break;
    case 2:
        lcd->invertText(true);
        lcd->print(SoundGenerator::soundNames[synth->sound], 0, 0);
        lcd->invertText(false);
        lcd->print("Freq       440", 0, 1);
        lcd->print("Mod        100", 0, 2);
        lcd->print("Rate      1200", 0, 3);
        lcd->print("- - - -       ", 0, 4);
        lcd->print("<  >     MIXER", 0, 5);
        break;
    default:
        break;
    }
}

/**
  * @brief Mixer
  */
void MenuSystem::MixerMenu()
{
    currentMenu = MIXER_MENU;

    lcd->drawHLine(0, 9, 83);
    lcd->refreshScr();

    lcd->invertText(true);
    lcd->print("             X", 0, 0);
    lcd->invertText(false);
    lcd->print("MIXER        ", 0, 0);
    lcd->print("Headphones   ", 0, 2);
    lcd->print(intToChar(audio->HP_Volume), 60, 1);
    lcd->print("LineOut      ", 0, 3);
    lcd->print(intToChar(audio->SPK_Volume), 60, 2);
    lcd->print("LineIn      --", 0, 4);
    lcd->print("Mic         --", 0, 5);
}

/**
  * @brief Sound select Menu
  */
void MenuSystem::SoundMenu()
{
    currentMenu = SOUND_MENU;
    looping = false;

    activeItemCount = SoundGenerator::soundscount;

    lcd->drawHLine(0, 8, 83);
    lcd->refreshScr();
    lcd->print("SOUND        X", 0, 0);

    for (int i = activeItem < 5 ? 0 : activeItem - 4, j = 1; j < 6; i++, j++)
    {
        if (i == activeItem)
        {
            lcd->invertText(true);
        }
        lcd->print(SoundGenerator::soundNames[i], 0, j);
        if (i == activeItem)
        {
            lcd->invertText(false);
        }
    }
}

/**
  * @brief Delay Menu
  */
void MenuSystem::DelayMenu()
{
    lcd->drawHLine(0, 8, 83);
    lcd->refreshScr();
    lcd->print("DELAY         ", 0, 0);
    //  lcd->goXY(0,20);
    lcd->print("Speed       90", 0, 1);
    lcd->print("Feedback   100", 0, 2);
    lcd->print("Filter      --", 0, 3);
    lcd->print("            --", 0, 4);
    lcd->print("<  >     MIXER", 0, 5);
}

/**
  * @brief Osc Menu
  */
void MenuSystem::OscMenu()
{
    currentMenu = OSC_MENU;

    lcd->drawHLine(0, 9, 83);
    lcd->refreshScr();

    lcd->invertText(true);
    lcd->print("             X", 0, 0);
    lcd->invertText(false);
    lcd->print("OSC1         ", 0, 0);
    lcd->print("Freq         ", 0, 2);
    lcd->print(intToChar(audio->HP_Volume), 60, 1);
    lcd->print("LineOut      ", 0, 3);
    lcd->print(intToChar(audio->SPK_Volume), 60, 2);
    lcd->print("LineIn      --", 0, 4);
    lcd->print("Mic         --", 0, 5);
}

/**
  * @brief Footer
  */
void MenuSystem::MenuFooter()
{
    switch (activeItem)
    {
    case 0:
        lcd->print("         MIXER", 0, 5);
        lcd->invertText(true);
        lcd->print("<  >", 0, 5);
        lcd->invertText(false);
        break;
    case 1:
        lcd->print("<  >", 0, 5);
        lcd->invertText(true);
        lcd->print("MIXER", 54, 5);
        lcd->invertText(false);
        break;
    default:
        lcd->print("<  >     MIXER", 0, 5);
        break;
    }
}
