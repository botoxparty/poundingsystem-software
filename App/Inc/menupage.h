
#ifndef _PS_MENU_PAGE_
#define _PS_MENU_PAGE_

#include "lcd.h"
#ifdef __cplusplus
class MenuPage
{
public:
    int activeItem;
    int activeItemCount;
    MenuPage(LCD *hlcd);
    void Previous();
    void Next();
    void MixerMenu();
    void select();
    LCD *lcd;
};
#endif
#endif