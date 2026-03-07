/* =============================================================================
 * lv_conf.h - Minimal LVGL configuration for TeslaBMS M5Dial
 * All active options are set via build_flags in platformio.ini.
 * This file satisfies the LVGL requirement for lv_conf.h to exist.
 * ============================================================================= */
#if 0  /* Enable content when LVGL is included */

#ifndef LV_CONF_H
#define LV_CONF_H

#include <stdint.h>

/*====================
   COLOR SETTINGS
 *====================*/
#define LV_COLOR_DEPTH     16
#define LV_COLOR_16_SWAP   0

/*====================
   MEMORY SETTINGS
 *====================*/
#define LV_MEM_CUSTOM      0
#define LV_MEM_SIZE        (64 * 1024U)    /* 64KB - LVGL internal heap */

/*====================
   HAL SETTINGS
 *====================*/
#define LV_TICK_CUSTOM     0
#define LV_DPI_DEF         130

/*====================
   FONT USAGE
 *====================*/
#define LV_FONT_MONTSERRAT_12   1
#define LV_FONT_MONTSERRAT_16   1
#define LV_FONT_MONTSERRAT_20   1
#define LV_FONT_MONTSERRAT_28   1
#define LV_FONT_DEFAULT         &lv_font_montserrat_16

/*====================
   WIDGET ENABLE
 *====================*/
#define LV_USE_ARC   1
#define LV_USE_BAR   1
#define LV_USE_LABEL 1
#define LV_USE_BTN   1
#define LV_USE_OBJ   1

/*====================
   THEME
 *====================*/
#define LV_USE_THEME_DEFAULT 1
#define LV_THEME_DEFAULT_DARK 1

#endif /* LV_CONF_H */
#endif /* End of enable */
