/**
  ******************************************************************************
  * @file    CSTMWinCtrls.cpp
  * @author  Systems Lab & Techincal Marketing (SDD Group)
  * @version V0.0.1
  * @date    04/26/2010
  * @brief   This file provides all the controls implemented for the STM 
  *          fondation class library
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



#include "CSTMWinCtrls.h"
#include "string.h"
#include "MC_type.h"


CSTMLabel::CSTMLabel(char *lpzText, CSTMWin *hParent, int16_t xPos, int16_t yPos) 
          :CSTMWinCtrl(hParent, xPos, yPos)
{
  SetText(lpzText);
}

CSTMLabel::CSTMLabel(char *lpzText) : CSTMWinCtrl(INVALID_HANDLE)
{
  SetText(lpzText);
}

void CSTMLabel::SetText(char *lpzText)
{
  m_lpzText = lpzText;
  //InvalidateArea();
  RefreshAsync();
}

void CSTMLabel::Refresh()
{
  if (m_lpzText)
  {
    SetSize(GetFont()->Width * strlen(m_lpzText), GetFont()->Height);
    CSTMLCDMngr::DrawText(GetTopLeft().X, GetTopLeft().Y, m_lpzText);
  }
  else
  {
    ClearArea();
  }
}

CSTMBitmap::CSTMBitmap(uint16_t *pBuffer, uint16_t Width, uint16_t Height, CSTMWin *hParent, int16_t xPos, int16_t yPos)
  :CSTMWinCtrl(hParent, xPos, yPos)
{
  SetSize(Width, Height);
  SetImageBuffer(pBuffer);
}

void CSTMBitmap::Refresh()
{
  if (m_pBitmap)
    CSTMLCDMngr::DrawBitmap16(m_pBitmap, GetTopLeft().X, GetTopLeft().Y, GetWidth(), GetHeight()); 
  else
    ClearArea();
}

CSTMText::CSTMText(char *lpzText, CSTMWin *hParent, int16_t xPos, int16_t yPos)  
          :CSTMEditBase(hParent, xPos, yPos)
{
  OnChangeText = NULL;
  SetText(lpzText);
}

CSTMText::CSTMText(char *lpzText) : CSTMEditBase(INVALID_HANDLE)
{
  OnChangeText = NULL;
  SetText(lpzText);
  
}

void CSTMText::SetText(char *lpzText)
{
  m_lpzText = lpzText;
  RefreshAsync();
  //SetSize((GetFont()->Width * strlen(m_lpzText)) + 5, GetFont()->Height + 4);
  //InvalidateArea();

  if (OnChangeText)
    OnChangeText(this, (uint32_t)lpzText);

}

void CSTMText::Refresh()
{
  if (m_lpzText)
    SetSize((GetFont()->Width * strlen(m_lpzText)) + 5, GetFont()->Height + 4);
  
  //Draw rect of TextArea
  CSTMLCDMngr::DrawRect(GetTopLeft().X, GetTopLeft().Y,  GetWidth(), GetHeight(), GetTextColor());
  CSTMLCDMngr::DrawLine(GetTopLeft().X + 2, GetTopLeft().Y+2,  GetHeight()-3,  LCD_DIR_VERTICAL, GetBackColor());
  if (IsFocused())
    CSTMLCDMngr::DrawRect(GetTopLeft().X + 1, GetTopLeft().Y + 1,  GetWidth() - 2, GetHeight() - 2, Blue); // Focus color is blue
  else
    CSTMLCDMngr::DrawRect(GetTopLeft().X + 1, GetTopLeft().Y + 1,  GetWidth() - 2, GetHeight() - 2, GetBackColor());
  
  //Draw of Text
  if (IsEdit() && IsFocused())
    CSTMLCDMngr::SetBackColor(Yellow);     
  CSTMLCDMngr::DrawText(GetTopLeft().X + 3, GetTopLeft().Y + 2, m_lpzText);
}

uint8_t CSTMText::KeyPress(InputKey key)
{
  uint8_t nRet = 0; //Not Consumed
  if (IsEdit())
  {
      nRet = 1; //Consumed    
  }
  return nRet;
}
uint8_t CSTMText::KeyPressed(InputKey key)
{
  uint8_t nRet = 0; //Not Consumed
  if (IsEdit())
  {
      nRet = 1; //Consumed    
  }
  return nRet;
}

uint8_t CSTMText::KeyReleased(InputKey key)
{
  uint8_t nRet = 0; //Not Consumed

   if (IsEdit())
   {
       nRet = 1;
   }

  
   if (key == KEY_JOYSTICK_SEL)
   {
     SetEdit(!GetEdit());
     InvalidateArea();
   }

  
  return nRet;
}

void CSTMCombo::SetText(char *lpzText)
{
  SetTextBase(lpzText);
//  SetSize((GetFont()->Width * (m_MaxItemsSize + 1)) + 2*5 , GetFont()->Height + 4);
//  InvalidateArea();
  RefreshAsync();
  
  if (OnChangeText)
    OnChangeText(this, (uint32_t)lpzText);

}

void CSTMCombo::SetItemsCount()
{
  m_ItemsCount = 0;
  if (m_lpzList)
  {
    for (; m_ItemsCount < 256 && m_lpzList[m_ItemsCount] ; m_ItemsCount++)
    {
      if (strlen(m_lpzList[m_ItemsCount])> m_MaxItemsSize)
        m_MaxItemsSize = strlen(m_lpzList[m_ItemsCount]);
    }
    
  }
}

uint8_t CSTMCombo::KeyPress(InputKey key)
{
  uint8_t nRet = 0; //Not Consumed
  if (IsEdit())
  {
      nRet = 1; //Consumed    
  }
  return nRet;
}
uint8_t CSTMCombo::KeyPressed(InputKey key)
{
  uint8_t nRet = 0; //Not Consumed
  if (IsEdit())
  {
      nRet = 1; //Consumed    
  }
  return nRet;
}

uint8_t CSTMCombo::KeyReleased(InputKey key)
{
  uint8_t nRet = 0; //Not Consumed
 
  if (IsEdit())
  {
    switch (key)
    {
    case KEY_JOYSTICK_UP:
      if (GetSelectedIndex() == 0)
      {
        SetSelectedIndex(m_ItemsCount-1); // Roll to the last element
      }
      else
      {
        SetSelectedIndex(GetSelectedIndex()-1);
      }
      break;
    case KEY_JOYSTICK_DOWN:
      if (GetSelectedIndex() == m_ItemsCount-1)
      {
        SetSelectedIndex(0); // Roll to the first element
      }
      else
      {
        SetSelectedIndex(GetSelectedIndex()+1);
      }
      break;
    }
    nRet = 1;
  }   

  if (key == KEY_JOYSTICK_SEL)
  {
     SetEdit(!GetEdit());
     InvalidateArea();
     nRet = 1;
  }
    
  return nRet;
}

void CSTMCombo::Refresh()
{
   SetSize((GetFont()->Width * (m_MaxItemsSize + 1)) + 2*5 , GetFont()->Height + 4);
   
  //Draw rect of TextArea
  CSTMLCDMngr::DrawRect(GetTopLeft().X, GetTopLeft().Y,  GetWidth(), GetHeight(), GetTextColor());
  CSTMLCDMngr::DrawLine(GetTopLeft().X + 2, GetTopLeft().Y + 2,  GetHeight() - 3,  LCD_DIR_VERTICAL, GetBackColor());
  if (IsFocused())
    CSTMLCDMngr::DrawRect(GetTopLeft().X + 1, GetTopLeft().Y + 1,  GetWidth() - 2, GetHeight() - 2, Blue); // Focus color is blue
  else
    CSTMLCDMngr::DrawRect(GetTopLeft().X + 1, GetTopLeft().Y + 1,  GetWidth() - 2, GetHeight() - 2, GetBackColor());
  
  //Draw of Text
  if (IsEdit() && IsFocused())
    CSTMLCDMngr::SetBackColor(Yellow);     
  CSTMLCDMngr::DrawText(GetTopLeft().X + 3, GetTopLeft().Y + 2, GetText());


  CSTMLCDMngr::DrawLine(GetTopLeft().X + GetWidth() - 5 - GetFont()->Width, GetTopLeft().Y + 2,  GetHeight() - 3,  LCD_DIR_VERTICAL, GetTextColor());
  CSTMLCDMngr::DrawLine(GetTopLeft().X + GetWidth() - 4 - GetFont()->Width, GetTopLeft().Y + 2,  GetHeight() - 3,  LCD_DIR_VERTICAL, GetTextColor());
  CSTMLCDMngr::DrawLine(GetTopLeft().X + GetWidth() - 3 , GetTopLeft().Y + 2,  GetHeight() - 3,  LCD_DIR_VERTICAL, GetTextColor());
    
  
  CSTMLCDMngr::SetColors(GetBackColor(), GetTextColor());
  CSTMLCDMngr::DrawText(GetTopLeft().X + GetWidth() - 3 - GetFont()->Width, GetTopLeft().Y + 2, "v");
}


void CSTMGroup::Refresh()
{
  CSTMLCDMngr::DrawRect(GetTopLeft().X, GetTopLeft().Y + GetFont()->Width/2,  GetWidth(), GetHeight() - GetFont()->Width/2, GetTextColor());
  CSTMLCDMngr::DrawText(GetTopLeft().X + 6, GetTopLeft().Y, GetText());
}

CSTMEditInt::CSTMEditInt()
      :CSTMEditBase()
{ 
  m_pEditInt = NULL; 
  m_lpzFormat = NULL; 
  m_Inc = 1; 
  m_Type = IntType_Unkwnon; 
  m_MinValue = 0; 
  m_MaxValue = U16_MAX;
  OnChangeValue = NULL;
}

CSTMEditInt::CSTMEditInt(uint32_t *pInt, char *lpzFormat, uint8_t size, CSTMWin *hParent, int16_t xPos, int16_t yPos)
      :CSTMEditBase(hParent, xPos, yPos)
{
  m_Type = IntType_UnSigned_32; 
  m_pEditInt = (uint32_t *)pInt;  
  m_lpzFormat = lpzFormat;  
  m_Inc = 1; 
  SetSize(GetFont()->Width * size + 5, GetFont()->Height + 4);
  m_MinValue = 0; 
  m_MaxValue = U16_MAX;
  OnChangeValue = NULL;
}
CSTMEditInt::CSTMEditInt(int32_t *pInt, char *lpzFormat, uint8_t size, CSTMWin *hParent, int16_t xPos, int16_t yPos)
      :CSTMEditBase(hParent, xPos, yPos)
{
  m_Type = IntType_Signed_32; 
  m_pEditInt = (uint32_t *)pInt;  
  m_lpzFormat = lpzFormat;  
  m_Inc = 1; 
  SetSize(GetFont()->Width * size + 5, GetFont()->Height + 4);
  m_MinValue = (uint16_t)S16_MIN; 
  m_MaxValue = (uint16_t)S16_MAX;
  OnChangeValue = NULL;  
}

CSTMEditInt::CSTMEditInt(uint16_t *pInt, char *lpzFormat, uint8_t size, CSTMWin *hParent, int16_t xPos, int16_t yPos)
      :CSTMEditBase(hParent, xPos, yPos)
{
  m_Type = IntType_UnSigned_16; 
  m_pEditInt = (uint32_t *)pInt;  
  m_lpzFormat = lpzFormat;  
  m_Inc = 1; 
  SetSize(GetFont()->Width * size + 5, GetFont()->Height + 4);
  m_MinValue = (uint32_t)0; 
  m_MaxValue = (uint32_t)U16_MAX;
  OnChangeValue = NULL;

}
CSTMEditInt::CSTMEditInt(int16_t *pInt, char *lpzFormat, uint8_t size, CSTMWin *hParent, int16_t xPos, int16_t yPos)
      :CSTMEditBase(hParent, xPos, yPos)
{
  m_Type = IntType_Signed_16; 
  m_pEditInt = (uint32_t *)pInt;  
  m_lpzFormat = lpzFormat;  
  m_Inc = 1; 
  SetSize(GetFont()->Width * size + 5, GetFont()->Height + 4);
  m_MinValue = (uint16_t)S16_MIN; 
  m_MaxValue = (uint16_t)S16_MAX;
  OnChangeValue = NULL;
}
CSTMEditInt::CSTMEditInt(uint8_t *pInt, char *lpzFormat, uint8_t size, CSTMWin *hParent, int16_t xPos, int16_t yPos)
      :CSTMEditBase(hParent, xPos, yPos)
{
  m_Type = IntType_UnSigned_8; 
  m_pEditInt = (uint32_t *)pInt;  
  m_lpzFormat = lpzFormat;  
  m_Inc = 1; 
  SetSize(GetFont()->Width * size + 5, GetFont()->Height + 4);
  m_MinValue = (uint32_t)0; 
  m_MaxValue = (uint32_t)U8_MAX;
  OnChangeValue = NULL;
}

CSTMEditInt::CSTMEditInt(int8_t *pInt, char *lpzFormat, uint8_t size, CSTMWin *hParent, int16_t xPos, int16_t yPos)
      :CSTMEditBase(hParent, xPos, yPos)
{
  m_Type = IntType_Signed_8; 
  m_pEditInt = (uint32_t *)pInt;  
  m_lpzFormat = lpzFormat;  
  m_Inc = 1; 
  SetSize(GetFont()->Width * size + 5, GetFont()->Height + 4);
  m_MinValue = (uint16_t)S8_MIN; 
  m_MaxValue = (uint16_t)S8_MAX;
  OnChangeValue = NULL;
}


void CSTMEditInt::Refresh()
{
  char lpzBuffer[32];
  
  memset(lpzBuffer, 0, 32);
  
  //Draw rect of TextArea
  CSTMLCDMngr::DrawRect(GetTopLeft().X, GetTopLeft().Y,  GetWidth(), GetHeight(), GetTextColor());
  CSTMLCDMngr::DrawLine(GetTopLeft().X + 2, GetTopLeft().Y+2,  GetHeight()-3,  LCD_DIR_VERTICAL, GetBackColor());
  if (IsFocused())
    CSTMLCDMngr::DrawRect(GetTopLeft().X + 1, GetTopLeft().Y + 1,  GetWidth() - 2, GetHeight() - 2, Blue); // Focus color is blue
  else
    CSTMLCDMngr::DrawRect(GetTopLeft().X + 1, GetTopLeft().Y + 1,  GetWidth() - 2, GetHeight() - 2, GetBackColor());
  
  //Draw of Text
  if (IsEdit() && IsFocused())
    CSTMLCDMngr::SetBackColor(Yellow);
  
  //Read only visualization
  if (!IsFocusable())
    CSTMLCDMngr::SetBackColor(Cyan);    
  
  if (GetIntAddress() != NULL)
  {
    FormatText(lpzBuffer);
    
    CSTMLCDMngr::DrawText(GetTopLeft().X + 3, GetTopLeft().Y + 2, lpzBuffer);
  }
}

void CSTMEditInt::FormatText(char *lpzTxt)
{
  const char *lpzFormat;
  
    if (m_lpzFormat)
      lpzFormat = m_lpzFormat;
    else
      lpzFormat = "%d";
    
    switch(m_Type)
    {
    case IntType_UnSigned_32:
      sprintf(lpzTxt, lpzFormat, GetIntValue());
      break;
    case IntType_Signed_32:
      sprintf(lpzTxt, lpzFormat, (int32_t)GetIntValue());
      break;
    case IntType_UnSigned_16:
      sprintf(lpzTxt, lpzFormat, (uint16_t)(GetIntValue()));
      break;
    case IntType_Signed_16:
      sprintf(lpzTxt, lpzFormat, (int16_t)((GetIntValue())));
      break;
    case IntType_UnSigned_8:  
      sprintf(lpzTxt, lpzFormat, (uint8_t)((GetIntValue())));
       break;
    case IntType_Signed_8:  
    default:
      sprintf(lpzTxt, lpzFormat, ((int8_t)((GetIntValue()))));
       break;
    }
}


uint8_t CSTMEditInt::KeyPress(InputKey key)
{
  uint8_t nRet = 0; //Not Consumed
  
  if (IsEdit())
  {
    if (GetIntAddress() != NULL)
    {
      switch (key)
      {
      case KEY_JOYSTICK_UP:
        m_Inc = 1;
        m_IncAcc = 0;
        StepItUp();
        InvalidateArea();
        break;
      case KEY_JOYSTICK_DOWN:
        m_Inc = 1;
        m_IncAcc = 0;
        StepItDown();
        InvalidateArea();
        break;
      }
    }
   nRet = 1; //Consumed    
  }
  return nRet;
}

uint8_t CSTMEditInt::KeyPressed(InputKey key)
{
  uint8_t nRet = 0; //Not Consumed

  if (IsEdit())
  {
    if (GetIntAddress() != NULL)
    {
      switch (key)
      {
      case KEY_JOYSTICK_UP:
        m_Inc+=m_IncAcc;
        if (m_IncAcc != 255)
        {
          m_IncAcc++;
        }
        StepItUp();
        InvalidateArea();
        break;
      case KEY_JOYSTICK_DOWN:
        m_Inc+=m_IncAcc;
        if (m_IncAcc != 255)
        {
          m_IncAcc++;
        }
        StepItDown();
        InvalidateArea();
        break;
      }
    }
     nRet = 1; //Consumed        
  }
  return nRet;
}

uint8_t CSTMEditInt::KeyReleased(InputKey key)
{
  uint8_t nRet = 0; //Not Consumed

  if (IsEdit())
  {
    nRet = 1;    
  }
  
  if (key == KEY_JOYSTICK_SEL)
  {
     SetEdit(!GetEdit());
     InvalidateArea();
  }

  return nRet;

}


void CSTMEditInt::SetIntValue(uint32_t Value)
{
    if (m_pEditInt)
    {
      if (Value >= m_MinValue && Value <=  m_MaxValue)
      {
        switch(m_Type)
        {
        case IntType_UnSigned_8:
          *(uint8_t*)m_pEditInt = (uint8_t)Value;
          break;
        default:
          *(uint8_t*)m_pEditInt = (uint8_t)Value;
          break;
        }
        
        if(OnChangeValue) 
          OnChangeValue(this, GetIntValue());
      }
    }
}
void CSTMEditInt::SetIntValue(int32_t Value)
{
  if (m_pEditInt)
  {
    if (Value >= (int16_t)m_MinValue && Value <=  (int16_t)m_MaxValue)
    {
      switch(m_Type)
      {
      case IntType_Signed_32:
        *(int32_t*)m_pEditInt = (int32_t)Value;
        break;
      case IntType_Signed_16:
        *(int16_t*)m_pEditInt = (int16_t)Value;
        break;
      case IntType_Signed_8: 
        *(int8_t*)m_pEditInt = (int8_t)Value;
         break;
      default:
        *(int8_t*)m_pEditInt = (int8_t)Value;
        break;
      }
      if(OnChangeValue) 
        OnChangeValue(this, GetIntValue());
    }
  }
  
}

uint32_t CSTMEditInt::GetIntValue()
{
  uint32_t val;
  switch(m_Type)
  {
  case IntType_UnSigned_32:
    val = (*(uint32_t *)m_pEditInt);
    break;
  case IntType_Signed_32:
    val = (*(int32_t *)m_pEditInt);
    break;
  case IntType_UnSigned_16:
    val = ((*(uint16_t *)m_pEditInt));
    break;
  case IntType_Signed_16:
    val = ((*(int16_t *)m_pEditInt));
    break;
  case IntType_UnSigned_8:
    val = ((*(uint8_t *)m_pEditInt));
    break;
  case IntType_Signed_8:  
  default:
    val = ((*(int8_t *)m_pEditInt));
     break;
  }
  
  return val;
}

void CSTMEditInt::StepItUp()
{
  int32_t val = GetIntValue() + m_Inc;
  
  if (val > (int16_t)m_MaxValue)
  {
    m_Inc >>= 2;
    m_IncAcc >>= 2;
  }
  
  switch(m_Type)
  {
  case IntType_UnSigned_32:
  case IntType_UnSigned_16:
  case IntType_UnSigned_8:
    SetIntValue(val);
    break;
  case IntType_Signed_32:
  case IntType_Signed_16:
  case IntType_Signed_8:  
  default:
    SetIntValue((int32_t)val);
     break;
  }
}
void CSTMEditInt::StepItDown()
{
  int32_t val = GetIntValue() - m_Inc;
  
  if (val < (int16_t)m_MinValue)
  {
    m_Inc >>= 2;
    m_IncAcc >>= 2;
  }
  
  switch(m_Type)
  {
  case IntType_UnSigned_32:
  case IntType_UnSigned_16:
  case IntType_UnSigned_8:
    SetIntValue(val);
    break;
  case IntType_Signed_32:
  case IntType_Signed_16:
  case IntType_Signed_8:  
  default:
    SetIntValue((int32_t)val);
     break;
  }

}

CSTMEditInt32::CSTMEditInt32(int32_t *pInt, char *lpzFormat, uint8_t size, CSTMWin *hParent, int16_t xPos, int16_t yPos)
      :CSTMEditInt(pInt, lpzFormat, size, hParent, xPos, yPos)
{
  m_MinValue32 = -99999; 
  m_MaxValue32 = 999999;
}

void CSTMEditInt32::SetIntValue(int32_t Value)
{
  if (GetIntAddress())
  {
    if (Value >= m_MinValue32 && Value <=  m_MaxValue32)
    {
      *(int32_t*)GetIntAddress() = Value;
      
      if(OnChangeValue) 
        OnChangeValue(this, GetIntValue());
    }
  }
}

void CSTMEditInt32::StepItUp()
{
  int32_t val = GetIntValue() + GetInc();
  if (val > m_MaxValue32)
  {
    SetInc(GetInc()>>2);
    SetIncAcc(GetIncAcc()>>2);
  }
  SetIntValue(val);
}
void CSTMEditInt32::StepItDown()
{
  int32_t val = GetIntValue() - GetInc();
  if (val < m_MinValue32)
  {
    SetInc(GetInc()>>2);
    SetIncAcc(GetIncAcc()>>2);
  }
  SetIntValue(val);
}

uint8_t CSTMEditInt32::KeyPress(InputKey key)
{
  uint8_t nRet = 0; //Not Consumed
  
  if (IsEdit())
  {
    if (GetIntAddress() != NULL)
    {
      switch (key)
      {
      case KEY_JOYSTICK_UP:
        SetInc(1);
        SetIncAcc(0);
        StepItUp();
        InvalidateArea();
        break;
      case KEY_JOYSTICK_DOWN:
        SetInc(1);
        SetIncAcc(0);
        StepItDown();
        InvalidateArea();
        break;
      }
    }
   nRet = 1; //Consumed    
  }
  return nRet;
}

uint8_t CSTMEditInt32::KeyPressed(InputKey key)
{
  uint8_t nRet = 0; //Not Consumed

  if (IsEdit())
  {
    if (GetIntAddress() != NULL)
    {
      switch (key)
      {
      case KEY_JOYSTICK_UP:
        SetInc(GetInc()+GetIncAcc());
        if (GetIncAcc() != 255)
        {
          SetIncAcc(GetIncAcc()+1);
        }
        StepItUp();
        InvalidateArea();
        break;
      case KEY_JOYSTICK_DOWN:
        SetInc(GetInc()+GetIncAcc());
        if (GetIncAcc() != 255)
        {
          SetIncAcc(GetIncAcc()+1);
        }
        StepItDown();
        InvalidateArea();
        break;
      }
    }
     nRet = 1; //Consumed        
  }
  return nRet;
}

CSTMButton::CSTMButton(char *lpzText, CSTMWin *hParent, int16_t xPos, int16_t yPos)  
          :CSTMWinCtrl(hParent, xPos, yPos)
{
  OnClick = NULL;
  m_bPressed = 0;
  SetFocusable(1);
  SetText(lpzText);
}

CSTMButton::CSTMButton(char *lpzText) : CSTMWinCtrl(INVALID_HANDLE)
{
  OnClick = NULL;
  m_bPressed = 0;
  SetFocusable(1);
  SetText(lpzText);
}

void CSTMButton::SetText(char *lpzText)
{
  SetTextBase(lpzText);
//  SetSize((GetFont()->Width * strlen(GetText())) + 7, GetFont()->Height + 6);
//  InvalidateArea();
  RefreshAsync();
}

void CSTMButton::Refresh()
{
  uint16_t textX, textY, textHeight, textWidth;

  SetSize((GetFont()->Width * strlen(GetText())) + 7, GetFont()->Height + 6);
  
  textX = GetTopLeft().X;
  textY = GetTopLeft().Y;
  textHeight = GetHeight() - 2;
  textWidth = GetWidth() - 2;

  CSTMLCDMngr::DrawRect(GetTopLeft().X, GetTopLeft().Y,  GetWidth(), GetHeight(), GetTextColor());
  
  if (IsPressed())
  {
    textX = GetTopLeft().X + 2;
    textY = GetTopLeft().Y + 2;
    
    CSTMLCDMngr::DrawLine(textX - 1, textY - 1, GetHeight() - 1, LCD_DIR_VERTICAL,   GetTextColor());
    CSTMLCDMngr::DrawLine(textX,     textY - 1, GetWidth() - 2,  LCD_DIR_HORIZONTAL, GetTextColor());
  }
  else
  {
    CSTMLCDMngr::DrawLine(textX + textWidth, textY + 1,           textHeight,     LCD_DIR_VERTICAL,   GetTextColor());
    CSTMLCDMngr::DrawLine(textX,             textY  + textHeight, textWidth + 2,  LCD_DIR_HORIZONTAL, GetTextColor());
  }
  

  //Draw rect of TextArea
  CSTMLCDMngr::DrawRect(textX, textY,  textWidth, textHeight, GetTextColor());
  CSTMLCDMngr::DrawLine(textX + 2, textY+2,  textHeight-3,  LCD_DIR_VERTICAL, GetBackColor());
  
  if (IsFocused())
    CSTMLCDMngr::DrawRect(textX + 1, textY + 1,  textWidth - 2, textHeight - 2, Blue); // Focus color is blue
  else
    CSTMLCDMngr::DrawRect(textX + 1, textY + 1,  textWidth - 2, textHeight - 2, GetBackColor());
  
  
  CSTMLCDMngr::DrawText(textX + 3, textY + 2, GetText());
}

uint8_t CSTMButton::KeyPress(InputKey key)
{
    uint8_t nRet = 0; //not Consumed
    if (key == KEY_JOYSTICK_SEL)
    {
      this->SetPressed(1);
      this->InvalidateArea();
      nRet = 1; //Consumed
    }
    return nRet;

}

uint8_t CSTMButton::KeyPressed(InputKey key)
{
  uint8_t nRet = 0; //not Consumed
  if (IsPressed())
      nRet = 1; //Consumed
  return nRet;    
}
uint8_t CSTMButton::KeyReleased(InputKey key)
{
  uint8_t nRet = 0; //not Consumed
  
  if (key == KEY_JOYSTICK_SEL)
  {
    this->SetPressed(0);
    this->InvalidateArea();
    nRet = 1; //Consumed
  }

  if (IsPressed())
    nRet = 1; //Consumed

  return nRet;   
}



CSTMCheckBox::CSTMCheckBox(char *lpzText, CSTMWin *hParent, int16_t xPos, int16_t yPos)  
          :CSTMWinCtrl(hParent, xPos, yPos)
{
  OnClick = NULL;
  m_bChecked = 0;
  SetFocusable(1);
  SetText(lpzText);
}

CSTMCheckBox::CSTMCheckBox(char *lpzText) : CSTMWinCtrl(INVALID_HANDLE)
{
  OnClick = NULL;
  m_bChecked = 0;
  SetFocusable(1);
  SetText(lpzText);
}

void CSTMCheckBox::SetText(char *lpzText)
{
  SetTextBase(lpzText);
//  SetSize((GetFont()->Width * (strlen(GetText())+1)) + 2*5-1, GetFont()->Height + 4);
//  InvalidateArea();
  RefreshAsync();
}

void CSTMCheckBox::Refresh()
{
  SetSize((GetFont()->Width * (strlen(GetText())+1)) + 2*5-1, GetFont()->Height + 4);
  //Draw rect of TextArea
  CSTMLCDMngr::DrawRect(GetTopLeft().X, GetTopLeft().Y,  GetWidth(), GetHeight(), GetTextColor());

  if (IsChecked())
    CSTMLCDMngr::DrawText(GetTopLeft().X + 4, GetTopLeft().Y + 1, "x");
  else
    CSTMLCDMngr::DrawText(GetTopLeft().X + 4, GetTopLeft().Y + 1, " ");


  CSTMLCDMngr::DrawLine(GetTopLeft().X + 3, GetTopLeft().Y + 2,  GetFont()->Height,  LCD_DIR_VERTICAL, GetBackColor());
  CSTMLCDMngr::DrawLine(GetTopLeft().X + 6 + GetFont()->Width, GetTopLeft().Y + 2,  GetFont()->Height+1,  LCD_DIR_VERTICAL, GetBackColor());
  CSTMLCDMngr::DrawLine(GetTopLeft().X + 5 + GetFont()->Width, GetTopLeft().Y + 2,  GetFont()->Height+1,  LCD_DIR_VERTICAL, GetBackColor());

  CSTMLCDMngr::DrawRect(GetTopLeft().X + 2, GetTopLeft().Y + 2,  GetFont()->Width + 3, GetHeight() - 4, GetTextColor());


  
//  CSTMLCDMngr::DrawLine(GetTopLeft().X + 2, GetTopLeft().Y + 2,  GetHeight() - 3,  LCD_DIR_VERTICAL, GetBackColor());

  if (IsFocused())
    CSTMLCDMngr::DrawRect(GetTopLeft().X + 1, GetTopLeft().Y + 1,  GetWidth() - 2, GetHeight() - 2, Blue); // Focus color is blue
  else
    CSTMLCDMngr::DrawRect(GetTopLeft().X + 1, GetTopLeft().Y + 1,  GetWidth() - 2, GetHeight() - 2, GetBackColor());
  

  //Draw of Text
  CSTMLCDMngr::DrawText(GetTopLeft().X + 7 + GetFont()->Width, GetTopLeft().Y + 2, GetText());
  
  

}

uint8_t CSTMCheckBox::KeyPress(InputKey key)
{
    uint8_t nRet = 0; //not Consumed
//    if (key == KEY_JOYSTICK_SEL)
//    {
//      this->SetPressed(1);
//      this->InvalidateArea();
//      nRet = 1; //Consumed
//    }
    return nRet;

}

uint8_t CSTMCheckBox::KeyPressed(InputKey key)
{
  uint8_t nRet = 0; //not Consumed
//  if (IsPressed())
//      nRet = 1; //Consumed
  return nRet;    
}

uint8_t CSTMCheckBox::KeyReleased(InputKey key)
{
  uint8_t nRet = 0; //not Consumed
  
  if (key == KEY_JOYSTICK_SEL)
  {
    SetChecked(!IsChecked());
    InvalidateArea();
    nRet = 1; //Consumed
  }

//  if (IsPressed())
//    nRet = 1; //Consumed

  return nRet;   
}

