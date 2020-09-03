/**
  ******************************************************************************
  * @file    CSTMLCDMngr.h
  * @author  Systems Lab & Techincal Marketing (SDD Group)
  * @version V0.0.1
  * @date    04/26/2010
  * @brief   This file provides the header file for the module CSTMLCDMngr
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



#ifndef __CSTMLCDMNG_H_
#define __CSTMLCDMNG_H_

#if defined USE_STM32303C_EVAL
  #include "stm32f30x.h"
  #include "stm32303c_eval.h"
  #include "stm32303c_eval_lcd.h"
#else
  #include "stm32_eval.h"
#endif

class CSTMLCDMngr
{
//public:  
//  CSTMLCDMngr(){};
//  virtual ~CSTMLCDMngr(){};
  
private:
//  static uint16_t m_TextColor = Black;
//  static uint16_t m_BackColor = White;
  
public:
  static void Init();
  static void Clear(uint16_t rgb);
  static void DrawChar(uint16_t Xpos, uint16_t Ypos, char cChar);
  static void DrawText(uint16_t Xpos, uint16_t Ypos, char *lpzText);
  static void DrawLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length, uint8_t Direction, uint16_t rgb);
  static void DrawRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height, uint16_t rgb);
  static void DrawFillRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height, uint16_t rgb);
  static void DrawBitmap16(uint16_t *pBufferImage, uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height);  
  inline static void GetColors(__IO uint16_t *_TextColor, __IO uint16_t *_BackColor){LCD_GetColors(_TextColor,_BackColor); };
  inline static void SetColors(__IO uint16_t _TextColor, __IO uint16_t _BackColor){LCD_SetColors(_TextColor,_BackColor); };
  inline static void SetTextColor(__IO uint16_t Color){LCD_SetTextColor(Color);};
  inline static void SetBackColor(__IO uint16_t Color){LCD_SetBackColor(Color);};
  inline static void SetFont(sFONT *fonts){LCD_SetFont(fonts);};
  inline static sFONT * GetFont(){return LCD_GetFont();};

};


#endif //__CSTMLCDMNG_H_
