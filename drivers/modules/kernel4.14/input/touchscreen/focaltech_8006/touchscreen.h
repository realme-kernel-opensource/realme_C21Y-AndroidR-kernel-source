/*
 *
 * touchscreen driver
 *
 * Copyright (C) 2009 ww6, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef TOUCHSCREEN_HEADER
#define TOUCHSCREEN_HEADER

#define    LCD_DISPLAY_HEIGHT 1600
#define    LCD_DISPLAY_WIDTH  720

#if defined(LCD_DISPLAY_HEIGHT)&&defined(LCD_DISPLAY_WIDTH)
#define CTP_MAX_HEIGHT   LCD_DISPLAY_HEIGHT
#define CTP_MAX_WIDTH    LCD_DISPLAY_WIDTH
#elif defined(CONFIG_LCD_WVGA)
#define CTP_MAX_HEIGHT   800
#define CTP_MAX_WIDTH    480
#elif defined(CONFIG_LCD_QHD)
#define CTP_MAX_HEIGHT   960
#define CTP_MAX_WIDTH    540
#elif defined(CONFIG_LCD_720P)
#define CTP_MAX_HEIGHT   1280
#define CTP_MAX_WIDTH    720
#else
#define CTP_MAX_HEIGHT   854
#define CTP_MAX_WIDTH    480
#endif

#if defined(CONFIG_LCD_QHD)
#define CTP_BUTTON_KEY_Y    (1100)
#elif 1//defined(CONFIG_LCD_720P)
#define CTP_BUTTON_KEY_Y    (1350)
#else
#define CTP_BUTTON_KEY_Y    (900)
#endif

#define MENU_CTP_BUTTON_X   60
#define HOME_CTP_BUTTON_X   120
#define BACK_CTP_BUTTON_X   180
#define SEARCH_CTP_BUTTON_X 240

#define BUTTON_WIDTH 		10
#define BUTTON_HEIGHT 		10


#define CTP_PROXIMITY_DEVICE_NAME	"ctp_proximity" 


#define CTP_IOCTL_MAGIC			0X5D
#define CTP_IOCTL_PROX_ON		_IO(CTP_IOCTL_MAGIC, 7)
#define CTP_IOCTL_PROX_OFF		_IO(CTP_IOCTL_MAGIC, 8)

#define GPIO_TOUCH_RESET 145
#define GPIO_TOUCH_IRQ 144

#endif /*TOUCHSCREEN_HEADER*/


