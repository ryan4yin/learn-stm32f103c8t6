/*
 * Modify: Roberto Benjami
 * date: 2023.02
 *
 * - BSP_LCD_DrawCircle : delete the BSP_LCD_SetFont(&LCD_DEFAULT_FONT); (interesting bug)
 * - BSP_LCD_Init : DrawProp.pFont = &Font24 change to DrawProp.pFont = &LCD_DEFAULT_FONT
 * - FillTriangle -> BSP_LCD_FillTriangle, change to public, changed to fast algorythm
 * - Add : BSP_LCD_ReadID
 * - Add : BSP_LCD_ReadPixel
 * - Add : BSP_LCD_DrawRGB16Image
 * - Add : BSP_LCD_ReadRGB16Image
 * - Add : BSP_LCD_FillTriangle (faster algorithm)
 * - Modify : BSP_LCD_Init (default font from header file, default colors from header file, otptional clear from header file)
 * - Modify : ReadID return type: uint16_t to uint32_t
 * - Modify : DrawChar function character size limit removed, and smaller bitmap array is sufficient for operation
 * - Add : BSP_LCD_DisplayMultilayerChar function (mainly for drawing icons and buttons)
 * */

/**
  ******************************************************************************
  * @file    stm32_adafruit_lcd.c
  * @author  MCD Application Team
  * @brief   This file includes the driver for Liquid Crystal Display (LCD) module
  *          mounted on the Adafruit 1.8" TFT LCD shield (reference ID 802), 
  *          that is used with the STM32 Nucleo board through SPI interface.     
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* File Info : -----------------------------------------------------------------
                                   User NOTES
1. How To use this driver:
--------------------------
   - The LCD st7735 component driver MUST be included with this driver.  

2. Driver description:
---------------------
  + Initialization steps:
     o Initialize the LCD using the BSP_LCD_Init() function.
  
  + Display on LCD
     o Clear the whole LCD using the BSP_LCD_Clear() function or only one specified 
       string line using the BSP_LCD_ClearStringLine() function.
     o Display a character on the specified line and column using the BSP_LCD_DisplayChar()
       function or a complete string line using the BSP_LCD_DisplayStringAtLine() function.
     o Display a string line on the specified position (x,y in pixel) and align mode
       using the BSP_LCD_DisplayStringAtLine() function.          
     o Draw and fill a basic shapes (dot, line, rectangle, circle, ellipse, ..) 
       on LCD using a set of functions.    
 
------------------------------------------------------------------------------*/

/* Dependencies
- fonts.h
- font24.c
- font20.c
- font16.c
- font12.c
- font8.c"
EndDependencies */
    
/* Includes ------------------------------------------------------------------*/
#include "lcd.h"
#include "stm32_adafruit_lcd.h"
#include "Fonts/fonts.h"

/* @defgroup STM32_ADAFRUIT_LCD_Private_Defines */
#define POLY_X(Z)             ((int32_t)((Points + (Z))->X))
#define POLY_Y(Z)             ((int32_t)((Points + (Z))->Y))
#define NULL                  (void *)0

#define OFFSET_BITMAP         54

/* @defgroup STM32_ADAFRUIT_LCD_Private_Macros */
#define ABS(X) ((X) > 0 ? (X) : -(X))
#define SWAP16(a, b) {uint16_t t = a; a = b; b = t;}

/* @defgroup STM32_ADAFRUIT_LCD_Private_Variables */ 
LCD_DrawPropTypeDef DrawProp;

extern LCD_DrvTypeDef  *lcd_drv;

/* Max size of bitmap will based on a font24 (17x24) */
static uint8_t bitmap[MAX_HEIGHT_FONT * MAX_WIDTH_FONT * 2 + OFFSET_BITMAP] = {0};

/* @defgroup STM32_ADAFRUIT_LCD_Private_FunctionPrototypes */ 
static void DrawChar(uint16_t Xpos, uint16_t Ypos, const uint8_t *c);
static void SetDisplayWindow(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height);
  
/**
  * @brief  Initializes the LCD.
  * @param  None
  * @retval LCD state
  */
uint8_t BSP_LCD_Init(void)
{ 
  uint8_t ret = LCD_ERROR;

  /* LCD Init */   
  lcd_drv->Init();
  
  /* Default value for draw propriety */
  DrawProp.BackColor = LCD_DEFAULT_BACKCOLOR;
  DrawProp.TextColor = LCD_DEFAULT_TEXTCOLOR;
  DrawProp.pFont     = &LCD_DEFAULT_FONT;
  
  /* Clear the LCD screen */
  #if LCD_INIT_CLEAR == 1
  BSP_LCD_Clear(LCD_DEFAULT_BACKCOLOR);
  #endif
  
  ret = LCD_OK;
  
  return ret;
}

/**
  * @brief  Gets the LCD X size.
  * @param  None    
  * @retval Used LCD X size
  */
uint16_t BSP_LCD_GetXSize(void)
{
  return(lcd_drv->GetLcdPixelWidth());
}

/**
  * @brief  Gets the LCD Y size.
  * @param  None   
  * @retval Used LCD Y size
  */
uint16_t BSP_LCD_GetYSize(void)
{
  return(lcd_drv->GetLcdPixelHeight());
}

/**
  * @brief  Gets the LCD text color.
  * @param  None 
  * @retval Used text color.
  */
uint16_t BSP_LCD_GetTextColor(void)
{
  return DrawProp.TextColor;
}

/**
  * @brief  Gets the LCD background color.
  * @param  None
  * @retval Used background color
  */
uint16_t BSP_LCD_GetBackColor(void)
{
  return DrawProp.BackColor;
}

/**
  * @brief  Sets the LCD text color.
  * @param  Color: Text color code RGB(5-6-5)
  * @retval None
  */
void BSP_LCD_SetTextColor(uint16_t Color)
{
  DrawProp.TextColor = Color;
}

/**
  * @brief  Sets the LCD background color.
  * @param  Color: Background color code RGB(5-6-5)
  * @retval None
  */
void BSP_LCD_SetBackColor(uint16_t Color)
{
  DrawProp.BackColor = Color;
}

/**
  * @brief  Sets the LCD text font.
  * @param  fonts: Font to be used
  * @retval None
  */
void BSP_LCD_SetFont(sFONT *pFonts)
{
  DrawProp.pFont = pFonts;
}

/**
  * @brief  Gets the LCD text font.
  * @param  None
  * @retval Used font
  */
sFONT *BSP_LCD_GetFont(void)
{
  return DrawProp.pFont;
}

/**
  * @brief  Clears the hole LCD.
  * @param  Color: Color of the background
  * @retval None
  */
void BSP_LCD_Clear(uint16_t Color)
{
  lcd_drv->FillRect(0, 0, BSP_LCD_GetXSize(), BSP_LCD_GetYSize(), Color);
}

/**
  * @brief  Clears the selected line.
  * @param  Line: Line to be cleared
  *          This parameter can be one of the following values:
  *            @arg  0..9: if the Current fonts is Font16x24
  *            @arg  0..19: if the Current fonts is Font12x12 or Font8x12
  *            @arg  0..29: if the Current fonts is Font8x8
  * @retval None
  */
void BSP_LCD_ClearStringLine(uint16_t Line)
{ 
  uint32_t color_backup = DrawProp.TextColor; 
  DrawProp.TextColor = DrawProp.BackColor;;
    
  /* Draw a rectangle with background color */
  BSP_LCD_FillRect(0, (Line * DrawProp.pFont->Height), BSP_LCD_GetXSize(), DrawProp.pFont->Height);
  
  DrawProp.TextColor = color_backup;
  BSP_LCD_SetTextColor(DrawProp.TextColor);
}

/**
  * @brief  Displays one character.
  * @param  Xpos: Start column address
  * @param  Ypos: Line where to display the character shape.
  * @param  Ascii: Character ascii code
  *           This parameter must be a number between Min_Data = 0x20 and Max_Data = 0x7E 
  * @retval None
  */
void BSP_LCD_DisplayChar(uint16_t Xpos, uint16_t Ypos, uint8_t Ascii)
{
  DrawChar(Xpos, Ypos, &DrawProp.pFont->table[(Ascii-' ') *\
                        DrawProp.pFont->Height * ((DrawProp.pFont->Width + 7) / 8)]);
}

/**
  * @brief  Displays one multilayer character.
  * @param  Xpos: Start column address
  * @param  Ypos: Line where to display the character shape.
  * @param  Chars: Characters ascii code
  *           This parameter must be a number between Min_Data = 0x20 and Max_Data = 0x7E
  * @param  Colors: Character layer colors
  * @param  sf: Font table address
  * @retval None
  */
void BSP_LCD_DisplayMultilayerChar(uint16_t Xpos, uint16_t Ypos, uint8_t *Chars, uint16_t *Colors, sFONT *sf)
{
  uint32_t x, y, ay = 0;                /* actual coordinatas */
  uint8_t *pcs[MAX_CHAR_LAYER];         /* character set pointers for layers */
  uint8_t *pc;                          /* actual character set pointer */
  uint32_t li = 0;                      /* layer index */
  uint32_t csi = 0;                     /* character set byte array index */
  uint16_t *pb;                         /* bitmap pointer */
  uint32_t bmsy;                        /* bitmap buffer y size */
  uint8_t  cbm;                         /* character set bitmap mask */

  bmsy = (sizeof(bitmap) >> 1) / sf->Width;
  if(!bmsy)
    return;

  while(Chars[li] >= ' ' && li < 8)
  {
    pcs[li] = (uint8_t *)&sf->table[(Chars[li]-' ') * sf->Height * ((sf->Width + 7) / 8)];
    li++;
  }

  cbm = 0x80;
  pb = (uint16_t *)bitmap;

  for(y = 0; y < sf->Height; y++)
  {
    for(x = 0; x < sf->Width; x++)
    {
      if(!cbm)
      { /* charcter set byte step */
        cbm = 0x80;
        csi++;
      }

      *pb = DrawProp.BackColor;
      li = 0;
      while(Chars[li] >= ' ' && li < 8)
      {
        pc = pcs[li];
        if(pc[csi] & cbm)
          *pb = Colors[li];
        li++;
      }
      pb++;
      cbm >>= 1;
    }
    cbm = 0;
    ay++;
    if(ay >= bmsy - 1)
    {
      BSP_LCD_DrawRGB16Image(Xpos, Ypos + y + 1 - ay, sf->Width, ay, (uint16_t *)bitmap);
      ay = 0;
      pb = (uint16_t *)bitmap;
    }
  }
  if(ay)
  {
    BSP_LCD_DrawRGB16Image(Xpos, Ypos + y - ay, sf->Width, ay, (uint16_t *)bitmap);
  }

}

/**
  * @brief  Displays characters on the LCD.
  * @param  Xpos: X position (in pixel)
  * @param  Ypos: Y position (in pixel)   
  * @param  Text: Pointer to string to display on LCD
  * @param  Mode: Display mode
  *          This parameter can be one of the following values:
  *            @arg  CENTER_MODE
  *            @arg  RIGHT_MODE
  *            @arg  LEFT_MODE   
  * @retval None
  */
void BSP_LCD_DisplayStringAt(uint16_t Xpos, uint16_t Ypos, uint8_t *Text, Line_ModeTypdef Mode)
{
  uint16_t refcolumn = 1, i = 0;
  uint32_t size = 0, xsize = 0; 
  uint8_t  *ptr = Text;
  
  /* Get the text size */
  while (*ptr++) size ++ ;
  
  /* Characters number per line */
  xsize = (BSP_LCD_GetXSize()/DrawProp.pFont->Width);
  
  switch (Mode)
  {
  case CENTER_MODE:
    {
      refcolumn = Xpos + ((xsize - size)* DrawProp.pFont->Width) / 2;
      break;
    }
  case LEFT_MODE:
    {
      refcolumn = Xpos;
      break;
    }
  case RIGHT_MODE:
    {
      refcolumn =  - Xpos + ((xsize - size)*DrawProp.pFont->Width);
      break;
    }    
  default:
    {
      refcolumn = Xpos;
      break;
    }
  }
  
  /* Send the string character by character on lCD */
  while ((*Text != 0) & (((BSP_LCD_GetXSize() - (i*DrawProp.pFont->Width)) & 0xFFFF) >= DrawProp.pFont->Width))
  {
    /* Display one character on LCD */
    BSP_LCD_DisplayChar(refcolumn, Ypos, *Text);
    /* Decrement the column position by 16 */
    refcolumn += DrawProp.pFont->Width;
    /* Point on the next character */
    Text++;
    i++;
  }
}

/**
  * @brief  Displays a character on the LCD.
  * @param  Line: Line where to display the character shape
  *          This parameter can be one of the following values:
  *            @arg  0..19: if the Current fonts is Font8
  *            @arg  0..12: if the Current fonts is Font12
  *            @arg  0...9: if the Current fonts is Font16
  *            @arg  0...7: if the Current fonts is Font20
  *            @arg  0...5: if the Current fonts is Font24
  * @param  ptr: Pointer to string to display on LCD
  * @retval None
  */
void BSP_LCD_DisplayStringAtLine(uint16_t Line, uint8_t *ptr)
{
  BSP_LCD_DisplayStringAt(0, LINE(Line), ptr, LEFT_MODE);
}

/**
  * @brief  Draws a pixel on LCD.
  * @param  Xpos: X position 
  * @param  Ypos: Y position
  * @param  RGB_Code: Pixel color in RGB mode (5-6-5)  
  * @retval None
  */
void BSP_LCD_DrawPixel(uint16_t Xpos, uint16_t Ypos, uint16_t RGB_Code)
{
  lcd_drv->WritePixel(Xpos, Ypos, RGB_Code);
}
  
/**
  * @brief  Draws an horizontal line.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  Length: Line length
  * @retval None
  */
void BSP_LCD_DrawHLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length)
{
  lcd_drv->DrawHLine(DrawProp.TextColor, Xpos, Ypos, Length);
}

/**
  * @brief  Draws a vertical line.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  Length: Line length
  * @retval None
  */
void BSP_LCD_DrawVLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length)
{
  lcd_drv->DrawVLine(DrawProp.TextColor, Xpos, Ypos, Length);
}

/**
  * @brief  Draws an uni-line (between two points).
  * @param  x1: Point 1 X position
  * @param  y1: Point 1 Y position
  * @param  x2: Point 2 X position
  * @param  y2: Point 2 Y position
  * @retval None
  */
void BSP_LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
  int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0, 
  yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0, 
  curpixel = 0;
  
  deltax = ABS(x2 - x1);        /* The difference between the x's */
  deltay = ABS(y2 - y1);        /* The difference between the y's */
  x = x1;                       /* Start x off at the first pixel */
  y = y1;                       /* Start y off at the first pixel */
  
  if (x2 >= x1)                 /* The x-values are increasing */
  {
    xinc1 = 1;
    xinc2 = 1;
  }
  else                          /* The x-values are decreasing */
  {
    xinc1 = -1;
    xinc2 = -1;
  }
  
  if (y2 >= y1)                 /* The y-values are increasing */
  {
    yinc1 = 1;
    yinc2 = 1;
  }
  else                          /* The y-values are decreasing */
  {
    yinc1 = -1;
    yinc2 = -1;
  }
  
  if (deltax >= deltay)         /* There is at least one x-value for every y-value */
  {
    xinc1 = 0;                  /* Don't change the x when numerator >= denominator */
    yinc2 = 0;                  /* Don't change the y for every iteration */
    den = deltax;
    num = deltax / 2;
    numadd = deltay;
    numpixels = deltax;         /* There are more x-values than y-values */
  }
  else                          /* There is at least one y-value for every x-value */
  {
    xinc2 = 0;                  /* Don't change the x for every iteration */
    yinc1 = 0;                  /* Don't change the y when numerator >= denominator */
    den = deltay;
    num = deltay / 2;
    numadd = deltax;
    numpixels = deltay;         /* There are more y-values than x-values */
  }
  
  for (curpixel = 0; curpixel <= numpixels; curpixel++)
  {
    BSP_LCD_DrawPixel(x, y, DrawProp.TextColor);  /* Draw the current pixel */
    num += numadd;                            /* Increase the numerator by the top of the fraction */
    if (num >= den)                           /* Check if numerator >= denominator */
    {
      num -= den;                             /* Calculate the new numerator value */
      x += xinc1;                             /* Change the x as appropriate */
      y += yinc1;                             /* Change the y as appropriate */
    }
    x += xinc2;                               /* Change the x as appropriate */
    y += yinc2;                               /* Change the y as appropriate */
  }
}

/**
  * @brief  Draws a rectangle.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  Width: Rectangle width  
  * @param  Height: Rectangle height
  * @retval None
  */
void BSP_LCD_DrawRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
  /* Draw horizontal lines */
  BSP_LCD_DrawHLine(Xpos, Ypos, Width);
  BSP_LCD_DrawHLine(Xpos, (Ypos + Height - 1), Width);
  
  /* Draw vertical lines */
  BSP_LCD_DrawVLine(Xpos, Ypos, Height);
  BSP_LCD_DrawVLine((Xpos + Width - 1), Ypos, Height);
}
                            
/**
  * @brief  Draws a circle.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  Radius: Circle radius
  * @retval None
  */
void BSP_LCD_DrawCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius)
{
  int32_t  D;       /* Decision Variable */ 
  uint32_t  CurX;   /* Current X Value */
  uint32_t  CurY;   /* Current Y Value */ 
  
  D = 3 - (Radius << 1);
  CurX = 0;
  CurY = Radius;
  
  while (CurX <= CurY)
  {
    BSP_LCD_DrawPixel((Xpos + CurX), (Ypos - CurY), DrawProp.TextColor);
    BSP_LCD_DrawPixel((Xpos - CurX), (Ypos - CurY), DrawProp.TextColor);
    BSP_LCD_DrawPixel((Xpos + CurY), (Ypos - CurX), DrawProp.TextColor);
    BSP_LCD_DrawPixel((Xpos - CurY), (Ypos - CurX), DrawProp.TextColor);
    BSP_LCD_DrawPixel((Xpos + CurX), (Ypos + CurY), DrawProp.TextColor);
    BSP_LCD_DrawPixel((Xpos - CurX), (Ypos + CurY), DrawProp.TextColor);
    BSP_LCD_DrawPixel((Xpos + CurY), (Ypos + CurX), DrawProp.TextColor);
    BSP_LCD_DrawPixel((Xpos - CurY), (Ypos + CurX), DrawProp.TextColor);   

    if (D < 0)
    { 
      D += (CurX << 2) + 6;
    }
    else
    {
      D += ((CurX - CurY) << 2) + 10;
      CurY--;
    }
    CurX++;
  } 
}

/**
  * @brief  Draws an poly-line (between many points).
  * @param  Points: Pointer to the points array
  * @param  PointCount: Number of points
  * @retval None
  */
void BSP_LCD_DrawPolygon(pPoint Points, uint16_t PointCount)
{
  int16_t X = 0, Y = 0;

  if(PointCount < 2)
  {
    return;
  }

  BSP_LCD_DrawLine(Points->X, Points->Y, (Points+PointCount-1)->X, (Points+PointCount-1)->Y);
  
  while(--PointCount)
  {
    X = Points->X;
    Y = Points->Y;
    Points++;
    BSP_LCD_DrawLine(X, Y, Points->X, Points->Y);
  }
}

/**
  * @brief  Draws an ellipse on LCD.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  XRadius: Ellipse X radius
  * @param  YRadius: Ellipse Y radius
  * @retval None
  */
void BSP_LCD_DrawEllipse(uint16_t Xpos, uint16_t Ypos, uint16_t XRadius, uint16_t YRadius)
{
  int x = 0, y = -YRadius, err = 2-2*XRadius, e2;
  float K = 0, rad1 = 0, rad2 = 0;
  
  rad1 = XRadius;
  rad2 = YRadius;
  
  K = (float)(rad2/rad1);
  
  do {      
    BSP_LCD_DrawPixel((Xpos-(uint16_t)(x/K)), (Ypos+y), DrawProp.TextColor);
    BSP_LCD_DrawPixel((Xpos+(uint16_t)(x/K)), (Ypos+y), DrawProp.TextColor);
    BSP_LCD_DrawPixel((Xpos+(uint16_t)(x/K)), (Ypos-y), DrawProp.TextColor);
    BSP_LCD_DrawPixel((Xpos-(uint16_t)(x/K)), (Ypos-y), DrawProp.TextColor);      
    
    e2 = err;
    if (e2 <= x) {
      err += ++x*2+1;
      if (-y == x && e2 <= y) e2 = 0;
    }
    if (e2 > y) err += ++y*2+1;     
  }
  while (y <= 0);
}

/**
  * @brief  Draws a bitmap picture loaded in the STM32 MCU internal memory.
  * @param  Xpos: Bmp X position in the LCD
  * @param  Ypos: Bmp Y position in the LCD
  * @param  pBmp: Pointer to Bmp picture address
  * @retval None
  */
void BSP_LCD_DrawBitmap(uint16_t Xpos, uint16_t Ypos, uint8_t *pBmp)
{
  uint32_t height = 0;
  uint32_t width  = 0;
  
  /* Read bitmap width */
  width = pBmp[18] + (pBmp[19] << 8) + (pBmp[20] << 16)  + (pBmp[21] << 24);

  /* Read bitmap height */
  height = pBmp[22] + (pBmp[23] << 8) + (pBmp[24] << 16)  + (pBmp[25] << 24);
  
  SetDisplayWindow(Xpos, Ypos, width, height);
  
  lcd_drv->DrawBitmap(Xpos, Ypos, pBmp);
}

/**
  * @brief  Draws a full rectangle.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  Width: Rectangle width  
  * @param  Height: Rectangle height
  * @retval None
  */
void BSP_LCD_FillRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
  lcd_drv->FillRect(Xpos, Ypos, Width, Height, DrawProp.TextColor);
}

/**
  * @brief  Draws a full circle.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  Radius: Circle radius
  * @retval None
  */
void BSP_LCD_FillCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius)
{
  int32_t  D;        /* Decision Variable */ 
  uint32_t  CurX;    /* Current X Value */
  uint32_t  CurY;    /* Current Y Value */ 
  
  D = 3 - (Radius << 1);

  CurX = 0;
  CurY = Radius;
  
  BSP_LCD_SetTextColor(DrawProp.TextColor);

  while (CurX <= CurY)
  {
    if(CurY > 0) 
    {
      BSP_LCD_DrawHLine(Xpos - CurY, Ypos + CurX, 2*CurY);
      BSP_LCD_DrawHLine(Xpos - CurY, Ypos - CurX, 2*CurY);
    }

    if(CurX > 0) 
    {
      BSP_LCD_DrawHLine(Xpos - CurX, Ypos - CurY, 2*CurX);
      BSP_LCD_DrawHLine(Xpos - CurX, Ypos + CurY, 2*CurX);
    }
    if (D < 0)
    { 
      D += (CurX << 2) + 6;
    }
    else
    {
      D += ((CurX - CurY) << 2) + 10;
      CurY--;
    }
    CurX++;
  }

  BSP_LCD_DrawCircle(Xpos, Ypos, Radius);
}

/**
  * @brief  Draws a full poly-line (between many points).
  * @param  Points: Pointer to the points array
  * @param  PointCount: Number of points
  * @retval None
  */
void BSP_LCD_FillPolygon(pPoint Points, uint16_t PointCount)
{
  int16_t X = 0, Y = 0, X2 = 0, Y2 = 0, X_center = 0, Y_center = 0, X_first = 0, Y_first = 0, pixelX = 0, pixelY = 0, counter = 0;
  uint16_t  IMAGE_LEFT = 0, IMAGE_RIGHT = 0, IMAGE_TOP = 0, IMAGE_BOTTOM = 0;  
  
  IMAGE_LEFT = IMAGE_RIGHT = Points->X;
  IMAGE_TOP= IMAGE_BOTTOM = Points->Y;
  
  for(counter = 1; counter < PointCount; counter++)
  {
    pixelX = POLY_X(counter);
    if(pixelX < IMAGE_LEFT)
    {
      IMAGE_LEFT = pixelX;
    }
    if(pixelX > IMAGE_RIGHT)
    {
      IMAGE_RIGHT = pixelX;
    }
    
    pixelY = POLY_Y(counter);
    if(pixelY < IMAGE_TOP)
    {
      IMAGE_TOP = pixelY;
    }
    if(pixelY > IMAGE_BOTTOM)
    {
      IMAGE_BOTTOM = pixelY;
    }
  }  
  
  if(PointCount < 2)
  {
    return;
  }
  
  X_center = (IMAGE_LEFT + IMAGE_RIGHT)/2;
  Y_center = (IMAGE_BOTTOM + IMAGE_TOP)/2;
  
  X_first = Points->X;
  Y_first = Points->Y;
  
  while(--PointCount)
  {
    X = Points->X;
    Y = Points->Y;
    Points++;
    X2 = Points->X;
    Y2 = Points->Y;    
    
    BSP_LCD_FillTriangle(X, Y, X2, Y2, X_center, Y_center);
    BSP_LCD_FillTriangle(X, Y, X_center, Y_center, X2, Y2);
    BSP_LCD_FillTriangle(X_center, Y_center, X2, Y2, X, Y);
  }
  
  BSP_LCD_FillTriangle(X_first, Y_first, X2, Y2, X_center, Y_center);
  BSP_LCD_FillTriangle(X_first, Y_first, X_center, Y_center, X2, Y2);
  BSP_LCD_FillTriangle(X_center, Y_center, X2, Y2, X_first, Y_first);
}

/**
  * @brief  Draws a full ellipse.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  XRadius: Ellipse X radius
  * @param  YRadius: Ellipse Y radius  
  * @retval None
  */
void BSP_LCD_FillEllipse(uint16_t Xpos, uint16_t Ypos, uint16_t XRadius, uint16_t YRadius)
{
  int x = 0, y = -YRadius, err = 2-2*XRadius, e2;
  float K = 0, rad1 = 0, rad2 = 0;
  
  rad1 = XRadius;
  rad2 = YRadius;
  
  K = (float)(rad2/rad1);    
  
  do 
  { 
    BSP_LCD_DrawHLine((Xpos-(uint16_t)(x/K)), (Ypos+y), (2*(uint16_t)(x/K) + 1));
    BSP_LCD_DrawHLine((Xpos-(uint16_t)(x/K)), (Ypos-y), (2*(uint16_t)(x/K) + 1));
    
    e2 = err;
    if (e2 <= x) 
    {
      err += ++x*2+1;
      if (-y == x && e2 <= y) e2 = 0;
    }
    if (e2 > y) err += ++y*2+1;
  }
  while (y <= 0);
}

/**
  * @brief  Enables the display.
  * @param  None
  * @retval None
  */
void BSP_LCD_DisplayOn(void)
{
  lcd_drv->DisplayOn();
}

/**
  * @brief  Disables the display.
  * @param  None
  * @retval None
  */
void BSP_LCD_DisplayOff(void)
{
  lcd_drv->DisplayOff();
}

/*******************************************************************************
                            Static Functions
*******************************************************************************/

/**
  * @brief  Draws a character on LCD.
  * @param  Xpos: Line where to display the character shape
  * @param  Ypos: Start column address
  * @param  pChar: Pointer to the character data
  * @retval None
  */
static void DrawChar(uint16_t Xpos, uint16_t Ypos, const uint8_t *pChar)
{
  uint32_t x, y, ay = 0;
  uint16_t *pb;                         /* bitmap pointer */
  uint32_t bmsy;                        /* bitmap buffer y size */
  uint8_t  c;                           /* character set actual byte */
  uint8_t  cbm;                         /* character set bitmap mask */

  bmsy = (sizeof(bitmap) >> 1) / DrawProp.pFont->Width;
  if(!bmsy)
    return;

  cbm = 0x80;
  pb = (uint16_t *)bitmap;
  c = *pChar;

  for(y = 0; y < DrawProp.pFont->Height; y++)
  {
    for(x = 0; x < DrawProp.pFont->Width; x++)
    {
      if(!cbm)
      { /* byte step */
        cbm = 0x80;
        pChar++;
        c = *pChar;
      }

      if(c & cbm)
        *pb = DrawProp.TextColor;
      else
        *pb = DrawProp.BackColor;
      pb++;
      cbm >>= 1;
    }
    cbm = 0;
    ay++;
    if(ay >= bmsy - 1)
    {
      BSP_LCD_DrawRGB16Image(Xpos, Ypos + y + 1 - ay, DrawProp.pFont->Width, ay, (uint16_t *)bitmap);
      ay = 0;
      pb = (uint16_t *)bitmap;
    }
  }
  if(ay)
  {
    BSP_LCD_DrawRGB16Image(Xpos, Ypos + y - ay, DrawProp.pFont->Width, ay, (uint16_t *)bitmap);
  }
}

/**
  * @brief  Fills a triangle (between 3 points).
  * @param  Points: Pointer to the points array
  * @param  x1: Point 1 X position
  * @param  y1: Point 1 Y position
  * @param  x2: Point 2 X position
  * @param  y2: Point 2 Y position
  * @param  x3: Point 3 X position
  * @param  y3: Point 3 Y position
  * @retval None
  */
void BSP_LCD_FillTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3)
{
  int16_t a, b, y, last;

  // Sort coordinates by Y order (y3 >= y2 >= y1)
  if (y1 > y2)
  {
    SWAP16(y1, y2); SWAP16(x1, x2);
  }
  if (y2 > y3) {
    SWAP16(y3, y2); SWAP16(x3, x2);
  }
  if (y1 > y2) {
    SWAP16(y1, y2); SWAP16(x1, x2);
  }

  if(y1 == y3)
  { // Handle awkward all-on-same-line case as its own thing
    a = b = x1;
    if(x2 < a)      a = x2;
    else if(x2 > b) b = x2;
    if(x3 < a)      a = x3;
    else if(x3 > b) b = x3;
    BSP_LCD_DrawHLine(a, y1, b - a + 1);
    return;
  }

  int16_t
  dx12 = x2 - x1,
  dy12 = y2 - y1,
  dx13 = x3 - x1,
  dy13 = y3 - y1,
  dx23 = x3 - x2,
  dy23 = y3 - y2;
  int32_t
  sa   = 0,
  sb   = 0;

  // For upper part of triangle, find scanline crossings for segments
  // 1-2 and 1-3.  If y2=y3 (flat-bottomed triangle), the scanline y2
  // is included here (and second loop will be skipped, avoiding a /0
  // error there), otherwise scanline y2 is skipped here and handled
  // in the second loop...which also avoids a /0 error here if y1=y2
  // (flat-topped triangle).
  if(y2 == y3) last = y2;   // Include y2 scanline
  else         last = y2 - 1; // Skip it

  for(y = y1; y <= last; y++)
  {
    a   = x1 + sa / dy12;
    b   = x1 + sb / dy13;
    sa += dx12;
    sb += dx13;
    /* longhand:
    a = x1 + (x2 - x1) * (y - y1) / (y2 - y1);
    b = x1 + (x3 - x1) * (y - y2) / (y3 - y1);
    */
    if(a > b) SWAP16(a, b);
    BSP_LCD_DrawHLine(a, y, b - a + 1);
  }

  // For lower part of triangle, find scanline crossings for segments
  // 1-3 and 2-3.  This loop is skipped if y1=y2.
  sa = (int32_t)dx23 * (y - y2);
  sb = (int32_t)dx13 * (y - y1);
  for(; y <= y3; y++)
  {
    a   = x2 + sa / dy23;
    b   = x1 + sb / dy13;
    sa += dx23;
    sb += dx13;
    /* longhand:
    a = x2 + (x3 - x2) * (y - y2) / (y3 - y2);
    b = x1 + (x3 - x1) * (y - y1) / (y3 - y1);
    */
    if(a > b) SWAP16(a, b);
    BSP_LCD_DrawHLine(a, y, b - a + 1);
  }
}

/**
  * @brief  Sets display window.
  * @param  LayerIndex: layer index
  * @param  Xpos: LCD X position
  * @param  Ypos: LCD Y position
  * @param  Width: LCD window width
  * @param  Height: LCD window height  
  * @retval None
  */
static void SetDisplayWindow(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
  lcd_drv->SetDisplayWindow(Xpos, Ypos, Width, Height);
}

/**
  * @brief  Get display ID
  * @param  none
  * @retval ID number
  */
uint32_t BSP_LCD_ReadID(void)
{
  return lcd_drv->ReadID();
}

/**
  * @brief  Get pixel
  * @param  Xpos: LCD X position
  * @param  Ypos: LCD Y position
  * @retval RGB565 pixel color
  */
uint16_t BSP_LCD_ReadPixel(uint16_t Xpos, uint16_t Ypos)
{
  return lcd_drv->ReadPixel(Xpos, Ypos);
}

/**
  * @brief  Draw RGB565 image (draw direction: right then down)
  * @param  Xpos: LCD X position
  * @param  Ypos: LCD Y position
  * @param  Width: image width
  * @param  Height: image height
  * @param  *pData: image data pointer
  * @retval None
  */
void BSP_LCD_DrawRGB16Image(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint16_t *pData)
{
  lcd_drv->DrawRGBImage(Xpos, Ypos, Xsize, Ysize, pData);
}

/**
  * @brief  Read RGB565 image (draw direction: right then down)
  * @param  Xpos: LCD X position
  * @param  Ypos: LCD Y position
  * @param  Width: image width
  * @param  Height: image height
  * @param  *pData: image data pointer
  * @retval *pData
  */
void BSP_LCD_ReadRGB16Image(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint16_t *pData)
{
  lcd_drv->ReadRGBImage(Xpos, Ypos, Xsize, Ysize, pData);
}

/**
  * @brief  Set display scroll parameters
  * @param  Scroll    : Scroll size [pixel]
  * @param  TopFix    : Top fix size [pixel]
  * @param  BottonFix : Botton fix size [pixel]
  * @retval None
  */
void BSP_LCD_Scroll(int16_t Scroll, uint16_t TopFix, uint16_t BottonFix)
{
  lcd_drv->Scroll(Scroll, TopFix, BottonFix);
}

/**
  * @brief  User direct Lcd write and read
  * @param  Cmd       : Lcd command
  * @param  ptr       : data pointer
  * @param  Size      : data number
  * @retval None
  */
void BSP_LCD_DataWrite8(uint16_t Cmd, uint8_t *ptr, uint32_t Size)
{
  lcd_drv->UserCommand(Cmd, ptr, Size, 0);
}
void BSP_LCD_DataWrite16(uint16_t Cmd, uint16_t *ptr, uint32_t Size)
{
  lcd_drv->UserCommand(Cmd, (uint8_t *)ptr, Size, 1);
}
void BSP_LCD_DataRead8(uint16_t Cmd, uint8_t *ptr, uint32_t Size)
{
  lcd_drv->UserCommand(Cmd, ptr, Size, 2);
}
void BSP_LCD_DataRead16(uint16_t Cmd, uint16_t *ptr, uint32_t Size)
{
  lcd_drv->UserCommand(Cmd, (uint8_t *)ptr, Size, 2);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
