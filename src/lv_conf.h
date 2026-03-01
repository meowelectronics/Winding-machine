/**
 * lv_conf.h for LVGL 9.2 + ESP32-2432S028R (CYD)
 *
 * Place in: Documents/Arduino/libraries/lv_conf.h
 * (one level UP from the lvgl folder, not inside it)
 *
 * Directory structure should look like:
 *   libraries/
 *     lvgl/
 *     TFT_eSPI/
 *     XPT2046_Touchscreen/
 *     lv_conf.h       <-- this file goes here
 */

#if 1  /* Set this to 1 to enable content — DO NOT change to 0 */

#ifndef LV_CONF_H
#define LV_CONF_H

#include <stdint.h>

/*====================
   COLOR SETTINGS
 *====================*/
#define LV_COLOR_DEPTH 16          /* ILI9341 is 16-bit RGB565 */
#define LV_COLOR_16_SWAP 0

/*====================
   MEMORY SETTINGS
 *====================*/
#define LV_MEM_CUSTOM 0
#define LV_MEM_SIZE (48 * 1024U)   /* 48 KB internal heap for LVGL */
#define LV_MEM_ADR 0

/*====================
   HAL SETTINGS
 *====================*/
#define LV_TICK_CUSTOM 1
#define LV_TICK_CUSTOM_INCLUDE "Arduino.h"
#define LV_TICK_CUSTOM_SYS_TIME_EXPR (millis())

/*====================
   DRAWING
 *====================*/
#define LV_DRAW_BUF_ALIGN 4
#define LV_USE_DRAW_SW 1

/*====================
   FEATURES
 *====================*/
#define LV_USE_PERF_MONITOR 0
#define LV_USE_MEM_MONITOR  0
#define LV_USE_LOG          0

/*====================
   WIDGETS
 *====================*/
#define LV_USE_ARC        0
#define LV_USE_BAR        1    /* used for progress bar */
#define LV_USE_BTN        1    /* used extensively */
#define LV_USE_BTNMATRIX  0
#define LV_USE_CANVAS     0
#define LV_USE_CHECKBOX   0
#define LV_USE_DROPDOWN   1    /* used for mode selector */
#define LV_USE_IMG        0
#define LV_USE_LABEL      1    /* used extensively */
#define LV_USE_LINE       1
#define LV_USE_ROLLER     0
#define LV_USE_SLIDER     0
#define LV_USE_SWITCH     0
#define LV_USE_TEXTAREA   0
#define LV_USE_TABLE      0
#define LV_USE_SPAN       0
#define LV_USE_SCALE      0
/*====================
   EXTRA WIDGETS
 *====================*/
#define LV_USE_ANIMIMG    0
#define LV_USE_CALENDAR   0
#define LV_USE_CHART      0
#define LV_USE_COLORWHEEL 0
#define LV_USE_IMGBTN     0
#define LV_USE_KEYBOARD   0
#define LV_USE_LED        0
#define LV_USE_LIST       0
#define LV_USE_MENU       0
#define LV_USE_METER      0
#define LV_USE_MSGBOX     1    /* used for confirm dialogs */
#define LV_USE_SPINBOX    0
#define LV_USE_SPINNER    0    /* used during calculation */
#define LV_USE_TABVIEW    0
#define LV_USE_TILEVIEW   0
#define LV_USE_WIN        0

/*====================
   FONTS
 *====================*/
#define LV_FONT_MONTSERRAT_10 0
#define LV_FONT_MONTSERRAT_12 1
#define LV_FONT_MONTSERRAT_14 1   /* param labels */
#define LV_FONT_MONTSERRAT_16 1   /* param values */
#define LV_FONT_MONTSERRAT_18 0
#define LV_FONT_MONTSERRAT_20 1   /* section headers */
#define LV_FONT_MONTSERRAT_22 0
#define LV_FONT_MONTSERRAT_24 1   /* large values, result display */
#define LV_FONT_MONTSERRAT_28 0
#define LV_FONT_MONTSERRAT_32 0
#define LV_FONT_MONTSERRAT_36 0
#define LV_FONT_MONTSERRAT_48 0

#define LV_FONT_DEFAULT &lv_font_montserrat_14

#define LV_USE_FONT_PLACEHOLDER 1

/*====================
   THEME
 *====================*/
#define LV_USE_THEME_DEFAULT 1
#define LV_THEME_DEFAULT_DARK 1       /* dark theme looks great on TFT */
#define LV_THEME_DEFAULT_GROW 1
#define LV_THEME_DEFAULT_TRANSITION_TIME 80

#define LV_USE_THEME_SIMPLE  0
#define LV_USE_THEME_MONO    0

/*====================
   LAYOUT
 *====================*/
#define LV_USE_FLEX  1
#define LV_USE_GRID  0

/*====================
   OTHER
 *====================*/
#define LV_USE_FS_STDIO 0
#define LV_USE_PNG      0
#define LV_USE_BMP      0
#define LV_USE_TJPGD    0
#define LV_USE_GIF      0

#define LV_USE_SNAPSHOT  0
#define LV_USE_MONKEY    0
#define LV_USE_GRIDNAV   0
#define LV_USE_FRAGMENT  0
#define LV_USE_IMGFONT   0
#define LV_USE_OBSERVER  0
#define LV_USE_IME_PINYIN 0

#endif /* LV_CONF_H */
#endif /* End of "if 1" */