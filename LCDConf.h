/*********************************************************************
*                SEGGER Microcontroller GmbH                         *
*        Solutions for real time microcontroller applications        *
**********************************************************************
*                                                                    *
*        (c) 1996 - 2018  SEGGER Microcontroller GmbH                *
*                                                                    *
*        Internet: www.segger.com    Support:  support@segger.com    *
*                                                                    *
**********************************************************************

** emWin V5.48 - Graphical user interface for embedded applications **
All  Intellectual Property rights  in the Software belongs to  SEGGER.
emWin is protected by  international copyright laws.  Knowledge of the
source code may not be used to write a similar product.  This file may
only be used in accordance with the following terms:

The software  has been licensed to  Cypress Semiconductor Corporation,
whose registered  office is situated  at 198 Champion Ct. San Jose, CA
95134 USA  solely for the  purposes of creating  libraries for Cypress
PSoC3 and  PSoC5 processor-based devices,  sublicensed and distributed
under  the  terms  and  conditions  of  the  Cypress  End User License
Agreement.
Full source code is available at: www.segger.com

We appreciate your understanding and fairness.
----------------------------------------------------------------------
Licensing information
Licensor:                 SEGGER Microcontroller Systems LLC
Licensed to:              Cypress Semiconductor Corp, 198 Champion Ct., San Jose, CA 95134, USA
Licensed SEGGER software: emWin
License number:           GUI-00319
License model:            Services and License Agreement, signed June 10th, 2009
Licensed platform:        Any Cypress platform (Initial targets are: PSoC3, PSoC5)
----------------------------------------------------------------------
Support and Update Agreement (SUA)
SUA period:               2009-06-12 - 2022-07-27
Contact to extend SUA:    sales@segger.com
----------------------------------------------------------------------
File        : LCDConf.h
Purpose     : Display driver configuration file
----------------------------------------------------------------------
*/

#include <cy_syslib.h>

//
// Physical display size
//
#define MTB_DISPLAY_SIZE_X       240
#define MTB_DISPLAY_SIZE_Y       320
#define MTB_DISPLAY_COLOR_BITS   16


#define	BLACK	0x0000
#define	BLUE	0x001F
#define	RED 	0xF800
#define	GREEN 	0x07E0
#define CYAN	0x07FF
#define MAGENTA 0xF81F
#define YELLOW	0xFFE0
#define WHITE	0xFFFF
#define BACKCOLOR 0xFFFF
#define	WORDCOLOR 0x0000

#define	GUI_Delay 	Cy_SysLib_Delay

void LCD_Initialization(void);

void  LCD_Demo1(void);

void LCD_Demo2(void);

void LCD_WindowSet(unsigned s_x, unsigned e_x, unsigned s_y, unsigned e_y);

void LCD_DrawBox(uint16 xStart,uint16 yStart,uint16 xlong,uint16 ylong);

void LCD_ConigPixel(uint16 xStart,uint16 xEnd,uint16 yStart,uint16 yEnd);

void LCD_ConfigColor(uint16 color);

void LCD_WritePixel(uint16 x,uint16 y,uint16 color);

void put_px(uint16 x,uint16 y,uint16 *color);

void LCD_Clear(uint16 Color);

void LCD_Border_Fill(void);

void LCD_Demo3(void);


#if defined(__cplusplus)
}
#endif


/*************************** End of file ****************************/
