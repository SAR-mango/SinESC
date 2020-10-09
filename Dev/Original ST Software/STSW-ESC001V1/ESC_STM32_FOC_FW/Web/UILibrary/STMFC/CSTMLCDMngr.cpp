/**
  ******************************************************************************
  * @file    CSTMLCDMngr.cpp
  * @author  Systems Lab & Techincal Marketing (SDD Group)
  * @version V0.0.1
  * @date    04/26/2010
  * @brief   This file provides all the function to work with the LCD using the
  *          standard driver of STEval boards
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */ 

#include "CSTMLCDMngr.h"

static uint16_t m_Width = LCD_PIXEL_WIDTH;
//static uint16_t m_Height = LCD_PIXEL_HEIGHT;

void CSTMLCDMngr::Init()
{
  /* Initialize the LCD */
  LCD_HW_Init();
  
  LCD_Clear(White);

  LCD_SetBackColor(White);
  LCD_SetTextColor(Black);

}

void CSTMLCDMngr::Clear(uint16_t rgb)
{
  LCD_Clear(rgb);
}

void CSTMLCDMngr::DrawChar(uint16_t Xpos, uint16_t Ypos, char cChar)
{
  uint16_t refcolumn = Xpos;
  if (LCD_GetXAxesDirection() == LCD_X_AXES_INVERTED)
  {
    refcolumn = m_Width - 1 - Xpos;
  }
  
  if (Xpos + LCD_GetFont()->Width <= m_Width)
  {
    if (LCD_GetFont()->Width == 6 && LCD_GetFont()->Height == 8)
    {
      LCD_DrawCharHorizontal(Ypos, refcolumn, &LCD_GetFont()->table[(cChar - 32) * LCD_GetFont()->Width]);      
    }
    else
    {
      LCD_DisplayChar(Ypos, refcolumn, cChar);
    }
  }
}

void CSTMLCDMngr::DrawText(uint16_t Xpos, uint16_t Ypos, char *lpzText)
{
   
  /* Send the string character by character on lCD */
  while (*lpzText != 0) 
  {
    /* Display one character on LCD */
    DrawChar(Xpos, Ypos, *lpzText);
    /* Decrement the column position by 16 */
    Xpos += LCD_GetFont()->Width;
    /* Point on the next character */
    lpzText++;
  }
}
  
void CSTMLCDMngr::DrawLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length, uint8_t Direction, uint16_t rgb)
{
    uint16_t XposLocal = m_Width - 1 - Xpos;
    uint16_t oldColor, oldColor_temp;

    LCD_GetColors(&oldColor, &oldColor_temp);
    LCD_SetTextColor(rgb);
    if (Length > 0)
    {
      LCD_DrawLine(Ypos, XposLocal, Length - 1, Direction);
    }
    
    LCD_SetTextColor(oldColor);
}

void CSTMLCDMngr::DrawRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height, uint16_t rgb)
{
  uint16_t XposLocal = m_Width - 1 - Xpos;
  uint16_t oldColor, oldColor_temp;

  if (Width > 0 && Height > 0)
  {
    LCD_GetColors(&oldColor, &oldColor_temp);
    LCD_SetTextColor(rgb);
    
    LCD_DrawLine(Ypos, XposLocal, Width -1, LCD_DIR_HORIZONTAL);
    LCD_DrawLine((Ypos + Height - 1), XposLocal, Width -1, LCD_DIR_HORIZONTAL);
    
    LCD_DrawLine(Ypos, XposLocal, Height-1, LCD_DIR_VERTICAL);
    LCD_DrawLine(Ypos, (XposLocal - (Width - 1)), Height, LCD_DIR_VERTICAL);
      
    LCD_SetTextColor(oldColor);
  }
}

void CSTMLCDMngr::DrawFillRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height, uint16_t rgb)
{
  uint16_t XposLocal = m_Width - 1 - Xpos;
  uint16_t oldColor, oldColor_temp;
  
  if (Width > 0)
  {
    LCD_GetColors(&oldColor, &oldColor_temp);
    LCD_SetTextColor(rgb);  
    
    while(Height--)
    {
      LCD_DrawLine(Ypos++, XposLocal, Width, LCD_DIR_HORIZONTAL);
    }
    LCD_SetTextColor(oldColor);
  }
  
}

/*

1)#define _RGB16BIT565(r,g,b) ((b%32) + ((g%64) << 6) + ((r%32) << 11))  
2)#define _RGB16BIT565(r,g,b) ((b&0x1F) + ((g&0x3F) << 6) + ((r&0x1F) << 11))  

3)#define _RGB16BIT565(r,g,b) ((b>>3) | ((g>>2) << 5) | ((r>>3) << 11))  

void Bitmap_Convert(BITMAP_FILE_PTR bitmap, UCHAR * buffer16bit)
{
	int n_padding = 0;
	int image_size = (bitmap->bitmapinfoheader.biSizeImage)/3;
	
	UCHAR	blue	= NULL, 
			green	= NULL,
			red		= NULL;
	USHORT	color	= NULL;

	for(int index = 0; index<=image_size; index++)
	{
		n_padding = index * 3;

		blue	= (bitmap->buffer[n_padding]);
		green	= (bitmap->buffer[n_padding + 1]);
		red		= (bitmap->buffer[n_padding + 2]);

		//color += _RGB16BIT565(red, green, blue);

		color = _RGB16BIT565(red, green, blue);
	}

	buffer16bit = (UCHAR *)color;
}

*/

void CSTMLCDMngr::DrawBitmap16(uint16_t *pBufferImage, uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
  LCD_PlotBMP(pBufferImage, Xpos, Ypos, Width, Height);
}
