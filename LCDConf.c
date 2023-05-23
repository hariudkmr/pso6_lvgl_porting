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
File        : LCDConf.c
Purpose     : Generic  configuration file for GUIDRV_FlexColor
---------------------------END-OF-HEADER------------------------------
*/

#include "LCDConf.h"
#include "mtb_st7789v.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/*********************************************************************
*
*       Layer configuration
*
**********************************************************************
*/
//
// Physical display size
//
#define XSIZE_PHYS MTB_DISPLAY_SIZE_X
#define YSIZE_PHYS MTB_DISPLAY_SIZE_Y

//
// Color conversion
//   The color conversion functions should be selected according to
//   the color mode of the target display. Details can be found in
//   the chapter "Colors" in the emWin user manual.
//
#define COLOR_CONVERSION GUICC_M565

//
// Display driver
//
#define DISPLAY_DRIVER GUIDRV_FLEXCOLOR

/*********************************************************************
*
*       Configuration checking
*
**********************************************************************
*/
#ifndef   VXSIZE_PHYS
    #define VXSIZE_PHYS XSIZE_PHYS
#endif
#ifndef   VYSIZE_PHYS
    #define VYSIZE_PHYS YSIZE_PHYS
#endif
#ifndef   XSIZE_PHYS
    #error Physical X size of display is not defined!
#endif
#ifndef   YSIZE_PHYS
    #error Physical Y size of display is not defined!
#endif
#ifndef   COLOR_CONVERSION
    #error Color conversion not defined!
#endif
#ifndef   DISPLAY_DRIVER
    #error No display driver defined!
#endif


/*********************************************************************
*
*       ST7789V Commands
*
* See the datasheet for additional details:
* https://www.newhavendisplay.com/appnotes/datasheets/LCDs/ST7789V.pdf
*
**********************************************************************
*/
#define ST7789V_CMD_NOP       (0x00) /* Empty command */
#define ST7789V_CMD_SWRESET   (0x01) /* Software Reset */
#define ST7789V_CMD_RDDID     (0x04) /* Read Display ID */
#define ST7789V_CMD_RDDST     (0x09) /* Read Display Status */
#define ST7789V_CMD_RDDPM     (0x0A) /* Read Display Power Mode */
#define ST7789V_CMD_RDDMADCTL (0x0B) /* Read Display MADCTL */
#define ST7789V_CMD_RDDCOLMOD (0x0C) /* Read Display Pixel Format */
#define ST7789V_CMD_RDDIM     (0x0D) /* Read Display Image Mode */
#define ST7789V_CMD_RDDSM     (0x0E) /* Read Display Signal Mode */
#define ST7789V_CMD_RDDSDR    (0x0F) /* Read Display Self-Diagnostic Result */
#define ST7789V_CMD_SLPIN     (0x10) /* Sleep in */
#define ST7789V_CMD_SLPOUT    (0x11) /* Sleep Out */
#define ST7789V_CMD_PTLON     (0x12) /* Partial Display Mode On */
#define ST7789V_CMD_NORON     (0x13) /* Normal Display Mode On */
#define ST7789V_CMD_INVOFF    (0x20) /* Display Inversion Off */
#define ST7789V_CMD_INVON     (0x21) /* Display Inversion On */
#define ST7789V_CMD_GAMSET    (0x26) /* Gamma Set */
#define ST7789V_CMD_DISPOFF   (0x28) /* Display Off */
#define ST7789V_CMD_DISPON    (0x29) /* Display On */
#define ST7789V_CMD_CASET     (0x2A) /* Column Address Set */
#define ST7789V_CMD_RASET     (0x2B) /* Row Address Set */
#define ST7789V_CMD_RAMWR     (0x2C) /* Memory Write */
#define ST7789V_CMD_RAMRD     (0x2E) /* Memory Read */
#define ST7789V_CMD_PTLAR     (0x30) /* Partial Area */
#define ST7789V_CMD_VSCRDEF   (0x33) /* Vertical Scrolling Definition */
#define ST7789V_CMD_TEOFF     (0x34) /* Tearing Effect Line OFF */
#define ST7789V_CMD_TEON      (0x35) /* Tearing Effect Line On */
#define ST7789V_CMD_MADCTL    (0x36) /* Memory Data Access Control */
#define ST7789V_CMD_VSCSAD    (0x37) /* Vertical Scroll Start Address of RAM */
#define ST7789V_CMD_IDMOFF    (0x38) /* Idle Mode Off */
#define ST7789V_CMD_IDMON     (0x39) /* Idle mode on */
#define ST7789V_CMD_COLMOD    (0x3A) /* Interface Pixel Format */
#define ST7789V_CMD_WRMEMC    (0x3C) /* Write Memory Continue */
#define ST7789V_CMD_RDMEMC    (0x3E) /* Read Memory Continue */
#define ST7789V_CMD_STE       (0x44) /* Set Tear Scanline */
#define ST7789V_CMD_GSCAN     (0x45) /* Get Scanline */
#define ST7789V_CMD_WRDISBV   (0x51) /* Write Display Brightness */
#define ST7789V_CMD_RDDISBV   (0x52) /* Read Display Brightness Value */
#define ST7789V_CMD_WRCTRLD   (0x53) /* Write CTRL Display */
#define ST7789V_CMD_RDCTRLD   (0x54) /* Read CTRL Value Display */
#define ST7789V_CMD_WRCACE    (0x55) /* Wr. Content Adaptive Brightness Control & Color Enhance */
#define ST7789V_CMD_RDCABC    (0x56) /* Read Content Adaptive Brightness Control */
#define ST7789V_CMD_WRCABCMB  (0x5E) /* Write CABC Minimum Brightness */
#define ST7789V_CMD_RDCABCMB  (0x5F) /* Read CABC Minimum Brightness */
#define ST7789V_CMD_RDABCSDR  (0x68) /* Read Automatic Brightness Control Self-Diagnostic Result */
#define ST7789V_CMD_RDID1     (0xDA) /* Read ID1 */
#define ST7789V_CMD_RDID2     (0xDB) /* Read ID2 */
#define ST7789V_CMD_RDID3     (0xDC) /* Read ID3 */
#define ST7789V_CMD_RAMCTRL   (0xB0) /* RAM Control */
#define ST7789V_CMD_RGBCTRL   (0xB1) /* RGB Interface Control */
#define ST7789V_CMD_PORCTRL   (0xB2) /* Porch Setting */
#define ST7789V_CMD_FRCTRL1   (0xB3) /* Frame Rate Control 1 (In partial mode/ idle colors) */
#define ST7789V_CMD_PARCTRL   (0xB5) /* Partial mode Control */
#define ST7789V_CMD_GCTRL     (0xB7) /* Gate Control */
#define ST7789V_CMD_GTADJ     (0xB8) /* Gate On Timing Adjustment */
#define ST7789V_CMD_DGMEN     (0xBA) /* Digital Gamma Enable */
#define ST7789V_CMD_VCOMS     (0xBB) /* VCOMS Setting */
#define ST7789V_CMD_LCMCTRL   (0xC0) /* LCM Control */
#define ST7789V_CMD_IDSET     (0xC1) /* ID Code Setting */
#define ST7789V_CMD_VDVVRHEN  (0xC2) /* VDV and VRH Command Enable */
#define ST7789V_CMD_VRHS      (0xC3) /* VRH Set */
#define ST7789V_CMD_VDVS      (0xC4) /* VDV Set */
#define ST7789V_CMD_VCMOFSET  (0xC5) /* VCOMS Offset Set */
#define ST7789V_CMD_FRCTRL2   (0xC6) /* Frame Rate Control in Normal Mode */
#define ST7789V_CMD_CABCCTRL  (0xC7) /* CABC Control */
#define ST7789V_CMD_REGSEL1   (0xC8) /* Register Value Selection 1 */
#define ST7789V_CMD_REGSEL2   (0xCA) /* Register Value Selection 2 */
#define ST7789V_CMD_PWMFRSEL  (0xCC) /* PWM Frequency Selection */
#define ST7789V_CMD_PWCTRL1   (0xD0) /* Power Control 1 */
#define ST7789V_CMD_VAPVANEN  (0xD2) /* Enable VAP/VAN signal output */
#define ST7789V_CMD_CMD2EN    (0xDF) /* Command 2 Enable */
#define ST7789V_CMD_PVGAMCTRL (0xE0) /* Positive Voltage Gamma Control */
#define ST7789V_CMD_NVGAMCTRL (0xE1) /* Negative Voltage Gamma Control */
#define ST7789V_CMD_DGMLUTR   (0xE2) /* Digital Gamma Look-up Table for Red */
#define ST7789V_CMD_DGMLUTB   (0xE3) /* Digital Gamma Look-up Table for Blue */
#define ST7789V_CMD_GATECTRL  (0xE4) /* Gate Control */
#define ST7789V_CMD_SPI2EN    (0xE7) /* SPI2 Enable */
#define ST7789V_CMD_PWCTRL2   (0xE8) /* Power Control 2 */
#define ST7789V_CMD_EQCTRL    (0xE9) /* Equalize time control */
#define ST7789V_CMD_PROMCTRL  (0xEC) /* Program Mode Control */
#define ST7789V_CMD_PROMEN    (0xFA) /* Program Mode Enable */
#define ST7789V_CMD_NVMSET    (0xFC) /* NVM Setting */
#define ST7789V_CMD_PROMACT   (0xFE) /* Program action */


/********************************************************************
*
*       CY8CKIT_028_TFT_InitController
*
* Purpose:
*   Initializes the display controller
*/
void LCD_Initialization(void)
{
    /* Reset the display controller */
    mtb_st7789v_write_reset_pin(0u);
    GUI_Delay(100);
    mtb_st7789v_write_reset_pin(1u);
    GUI_Delay(50);

    mtb_st7789v_write_command(ST7789V_CMD_DISPOFF);
    mtb_st7789v_write_command(ST7789V_CMD_SLPOUT);    /* Exit Sleep mode */

    GUI_Delay(100);

    mtb_st7789v_write_command(ST7789V_CMD_MADCTL);
    mtb_st7789v_write_data(0xA0);    /* MADCTL: memory data access control */

    mtb_st7789v_write_command(ST7789V_CMD_COLMOD);
    mtb_st7789v_write_data(0x65);    /* COLMOD: Interface Pixel format */

    mtb_st7789v_write_command(ST7789V_CMD_PORCTRL);
    mtb_st7789v_write_data(0x0C);
    mtb_st7789v_write_data(0x0C);
    mtb_st7789v_write_data(0x00);
    mtb_st7789v_write_data(0x33);
    mtb_st7789v_write_data(0x33);    /* PORCTRK: Porch setting */

    mtb_st7789v_write_command(ST7789V_CMD_GCTRL);
    mtb_st7789v_write_data(0x35);    /* GCTRL: Gate Control */

    mtb_st7789v_write_command(ST7789V_CMD_VCOMS);
    mtb_st7789v_write_data(0x2B);    /* VCOMS: VCOM setting */

    mtb_st7789v_write_command(ST7789V_CMD_LCMCTRL);
    mtb_st7789v_write_data(0x2C);    /* LCMCTRL: LCM Control */

    mtb_st7789v_write_command(ST7789V_CMD_VDVVRHEN);
    mtb_st7789v_write_data(0x01);
    mtb_st7789v_write_data(0xFF);    /* VDVVRHEN: VDV and VRH Command Enable */

    mtb_st7789v_write_command(ST7789V_CMD_VRHS);
    mtb_st7789v_write_data(0x11);    /* VRHS: VRH Set */

    mtb_st7789v_write_command(ST7789V_CMD_VDVS);
    mtb_st7789v_write_data(0x20);    /* VDVS: VDV Set */

    mtb_st7789v_write_command(ST7789V_CMD_FRCTRL2);
    mtb_st7789v_write_data(0x0F);    /* FRCTRL2: Frame Rate control in normal mode */

    mtb_st7789v_write_command(ST7789V_CMD_PWCTRL1);
    mtb_st7789v_write_data(0xA4);
    mtb_st7789v_write_data(0xA1);    /* PWCTRL1: Power Control 1 */

    mtb_st7789v_write_command(ST7789V_CMD_PVGAMCTRL);
    mtb_st7789v_write_data(0xD0);
    mtb_st7789v_write_data(0x00);
    mtb_st7789v_write_data(0x05);
    mtb_st7789v_write_data(0x0E);
    mtb_st7789v_write_data(0x15);
    mtb_st7789v_write_data(0x0D);
    mtb_st7789v_write_data(0x37);
    mtb_st7789v_write_data(0x43);
    mtb_st7789v_write_data(0x47);
    mtb_st7789v_write_data(0x09);
    mtb_st7789v_write_data(0x15);
    mtb_st7789v_write_data(0x12);
    mtb_st7789v_write_data(0x16);
    mtb_st7789v_write_data(0x19);    /* PVGAMCTRL: Positive Voltage Gamma control */

    mtb_st7789v_write_command(ST7789V_CMD_NVGAMCTRL);
    mtb_st7789v_write_data(0xD0);
    mtb_st7789v_write_data(0x00);
    mtb_st7789v_write_data(0x05);
    mtb_st7789v_write_data(0x0D);
    mtb_st7789v_write_data(0x0C);
    mtb_st7789v_write_data(0x06);
    mtb_st7789v_write_data(0x2D);
    mtb_st7789v_write_data(0x44);
    mtb_st7789v_write_data(0x40);
    mtb_st7789v_write_data(0x0E);
    mtb_st7789v_write_data(0x1C);
    mtb_st7789v_write_data(0x18);
    mtb_st7789v_write_data(0x16);
    mtb_st7789v_write_data(0x19);    /* NVGAMCTRL: Negative Voltage Gamma control */

    mtb_st7789v_write_command(ST7789V_CMD_RASET);
    mtb_st7789v_write_data(0x00);
    mtb_st7789v_write_data(0x00);
    mtb_st7789v_write_data(0x00);
    mtb_st7789v_write_data(0xEF);    /* Y address set */

    mtb_st7789v_write_command(ST7789V_CMD_CASET);
    mtb_st7789v_write_data(0x00);
    mtb_st7789v_write_data(0x00);
    mtb_st7789v_write_data(0x01);
    mtb_st7789v_write_data(0x3F);    /* X address set */

    GUI_Delay(10);

    mtb_st7789v_write_command(ST7789V_CMD_DISPON);
}


void LCD_Demo1()
{
  {
  unsigned int i;
  mtb_st7789v_write_command(0x2C);              //command to begin writing to frame memory
  for(i=0;i<38400;i++)         //fill screen with blue pixels
  {
            mtb_st7789v_write_data(0x00);
            mtb_st7789v_write_data(0x1F);
            mtb_st7789v_write_data(0x00);
            mtb_st7789v_write_data(0x1F);
  }
  for(i=0;i<38400;i++)         //fill screen with green pixels
  {
            mtb_st7789v_write_data(0x07);
            mtb_st7789v_write_data(0xE0);
            mtb_st7789v_write_data(0x07);
            mtb_st7789v_write_data(0xE0);
  }

  for(i=0;i<38400;i++)         //fill screen with red pixels
  {
            mtb_st7789v_write_data(0xF8);
            mtb_st7789v_write_data(0x00);
            mtb_st7789v_write_data(0xF8);
            mtb_st7789v_write_data(0x00);
  }
  Cy_SysLib_Delay(300);
}
}


void LCD_Demo2()
{
  {
  unsigned int i;
  mtb_st7789v_write_command(0x2C);              //command to begin writing to frame memory
  for(i=0;i<12800;i++)         //fill screen with blue pixels
  {
            mtb_st7789v_write_data(0x00); //R
            mtb_st7789v_write_data(0x1F); //R
            mtb_st7789v_write_data(0x00); //R
            mtb_st7789v_write_data(0x1F); //R

            mtb_st7789v_write_data(0xF8); //B
            mtb_st7789v_write_data(0x00); //B
            mtb_st7789v_write_data(0xF8); //B
            mtb_st7789v_write_data(0x00); //B

            mtb_st7789v_write_data(0x07); //G
            mtb_st7789v_write_data(0xE0); //G
            mtb_st7789v_write_data(0x07); //G
            mtb_st7789v_write_data(0xE0); //G

  }
 }
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Window Set Function
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void LCD_WindowSet(unsigned s_x, unsigned e_x, unsigned s_y, unsigned e_y)
{
  mtb_st7789v_write_command(0x2a);    //SET column address
  mtb_st7789v_write_reset_pin(1);
  mtb_st7789v_write_data((s_x)>>8);     //SET start column address
  mtb_st7789v_write_data(s_x);
  mtb_st7789v_write_data((e_x)>>8);     //SET end column address
  mtb_st7789v_write_data(e_x);

  mtb_st7789v_write_command(0x2b);    //SET page address
  mtb_st7789v_write_reset_pin(1);
  mtb_st7789v_write_data((s_y)>>8);     //SET start page address
  mtb_st7789v_write_data(s_y);
  mtb_st7789v_write_data((e_y)>>8);     //SET end page address
  mtb_st7789v_write_data(e_y);
}

void LCD_DrawBox(uint16 xStart,uint16 yStart,uint16 xlong,uint16 ylong)
{
	uint16 xEnd=0, yEnd=0;
	xEnd=xStart+xlong-1;
	yEnd=yStart+ylong-1;

	mtb_st7789v_write_command(0x2a);
	mtb_st7789v_write_data(xStart>>8);
	mtb_st7789v_write_data(xStart);
	mtb_st7789v_write_data(xEnd>>8);
	mtb_st7789v_write_data(xEnd);

	mtb_st7789v_write_command(0x2b);
	mtb_st7789v_write_data(yStart>>8);
	mtb_st7789v_write_data(yStart);
	mtb_st7789v_write_data(yEnd>>8);
	mtb_st7789v_write_data(yEnd);

	mtb_st7789v_write_command(0x2c);
}


void LCD_ConigPixel(uint16 xStart,uint16 xEnd,uint16 yStart,uint16 yEnd)
{
	mtb_st7789v_write_command(0x2a);
	mtb_st7789v_write_data(xStart>>8);
	mtb_st7789v_write_data(xStart);
	mtb_st7789v_write_data(xEnd>>8);
	mtb_st7789v_write_data(xEnd);

	mtb_st7789v_write_command(0x2b);
	mtb_st7789v_write_data(yStart>>8);
	mtb_st7789v_write_data(yStart);
	mtb_st7789v_write_data(yEnd>>8);
	mtb_st7789v_write_data(yEnd);

	mtb_st7789v_write_command(0x2c);
}

void LCD_ConfigColor(uint16 color)
{
	mtb_st7789v_write_data(color>>8 & 0xFF);
	mtb_st7789v_write_data(color & 0xFF);
	mtb_st7789v_write_data(color>>8 & 0xFF);
	mtb_st7789v_write_data(color & 0xFF);
}


void LCD_WritePixel(uint16 x,uint16 y,uint16 color)
{
	LCD_ConigPixel(x,x+1,y,y+1);
	LCD_ConfigColor(color);
}

void LCD_Clear(uint16 Color)
{
   uint32 i;
   LCD_DrawBox(0,0,320,240);
   for(i=0;i<78900;i++){
	   LCD_ConfigColor(Color);
  }
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Border and Fill Function
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void LCD_Border_Fill()
{
  unsigned int i,j;
  LCD_WindowSet(0,239,0,0);
  mtb_st7789v_write_command(0x2C);
  mtb_st7789v_write_reset_pin(1);
  for(i=0;i<240;i++)        //Bottom White Border
  {
    for (j=0;j<1;j++)
    {
    mtb_st7789v_write_data(0xFF);
    mtb_st7789v_write_data(0xFF);
    mtb_st7789v_write_data(0xFF);
    mtb_st7789v_write_data(0xFF);
    }
  }
  Cy_SysLib_Delay(100);
  LCD_WindowSet(0,0,0,319);
  mtb_st7789v_write_command(0x2C);
  mtb_st7789v_write_reset_pin(1);
  for(i=0;i<1;i++)        //Left White Border
  {
    for (j=0;j<320;j++)
    {
    mtb_st7789v_write_data(0xFF);
    mtb_st7789v_write_data(0xFF);
    mtb_st7789v_write_data(0xFF);
    mtb_st7789v_write_data(0xFF);
    }
  }
  Cy_SysLib_Delay(100);
  LCD_WindowSet(0,239,319,319);
  mtb_st7789v_write_command(0x2C);
  mtb_st7789v_write_reset_pin(1);
  for(i=0;i<240;i++)      //Top White Border
  {
    for (j=0;j<1;j++)
    {
    mtb_st7789v_write_data(0xFF);
    mtb_st7789v_write_data(0xFF);
    mtb_st7789v_write_data(0xFF);
    mtb_st7789v_write_data(0xFF);
    }
  }
  Cy_SysLib_Delay(100);
  LCD_WindowSet(239,239,0,319);
  mtb_st7789v_write_command(0x2C);
  mtb_st7789v_write_reset_pin(1);
  for(i=0;i<1;i++)        //Right White Border
  {
    for (j=0;j<240;j++)
    {
    mtb_st7789v_write_data(0xFF);
    mtb_st7789v_write_data(0xFF);
    mtb_st7789v_write_data(0xFF);
    mtb_st7789v_write_data(0xFF);
    }
  }
  Cy_SysLib_Delay(100);
  LCD_WindowSet(1,238,1,318);
  mtb_st7789v_write_command(0x2C);
  mtb_st7789v_write_reset_pin(1);
  for(i=0;i<238;i++)      //fill inside with Black Pixels
  {
    for (j=0;j<318;j++)
    {
      mtb_st7789v_write_data(0x00);
      mtb_st7789v_write_data(0x00);
      mtb_st7789v_write_data(0x00);
      mtb_st7789v_write_data(0x00);
    }
  }
  Cy_SysLib_Delay(250);
  LCD_WindowSet(0,239,0,319);
  mtb_st7789v_write_command(0x2C);
  mtb_st7789v_write_reset_pin(1);
  for(i=0;i<38400;i++)    //fill screen with White Pixels
  {
            mtb_st7789v_write_data(0xFF);
            mtb_st7789v_write_data(0xFF);
            mtb_st7789v_write_data(0xFF);
            mtb_st7789v_write_data(0xFF);
  }
}


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Fill Screen (All Black) Function
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void LCD_Demo3()
{
  unsigned int i;
  LCD_WindowSet(0,239,0,319);
  mtb_st7789v_write_command(0x2C);              //command to begin writing to frame memory
        for(i=0;i<38400;i++)   //fill screen with black pixels
  {
        	mtb_st7789v_write_data(0x00);
        	mtb_st7789v_write_data(0x00);
        	mtb_st7789v_write_data(0x00);
        	mtb_st7789v_write_data(0x00);
  }
}



#if defined(__cplusplus)
}
#endif




/*************************** End of file ****************************/
