/**
  ******************************************************************************
  * @file    CSTMWin.h
  * @author  Systems Lab & Techincal Marketing (SDD Group)
  * @version V0.0.1
  * @date    04/26/2010
  * @brief   This file provides the header for the STMFC base class
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



#ifndef __CSTMWIN_H_
#define __CSTMWIN_H_

#include "CSTMLCDMngr.h"
#include "CSTMInput.h"


typedef Point *PPoint;

#ifndef NULL
#define NULL 0
#endif 

#define HANDLE_WIN         CSTMWin *
#define INVALID_HANDLE     NULL


//#define  EVNT_CLICK(x,y) ((x).OnClick = (y))

typedef enum CSTMControl_Type_e
{
  CONTROL_TYPE_WIN      = 0,
  CONTROL_TYPE_LABEL    = 1,
  CONTROL_TYPE_TEXT     = 2,
  CONTROL_TYPE_BUTTON   = 3,
  CONTROL_TYPE_EDIT_INT = 4,
  CONTROL_TYPE_COMBO    = 5,
  CONTROL_TYPE_CHECKBOX = 6,
  CONTROL_TYPE_BITMAP   = 7,
  CONTROL_TYPE_GROUP    = 8
} CSTMControl_Type;

class CRect
{
public:  
  CRect();
  CRect(Point oTopLeft, Point oBottomRigth);
  CRect(Point oTopLeft, uint16_t nWidth, uint16_t nHeight);
  CRect(int16_t X, int16_t Y, uint16_t nWidth, uint16_t nHeight);

  virtual ~CRect(){}; 

public:  
  Point TopLeft;
  Point BottomRigth;

  inline uint16_t Width() {return BottomRigth.X - TopLeft.X;};
  inline uint16_t Height() {return BottomRigth.Y - TopLeft.Y;};
};

class CSTMWin
{
public:  
  CSTMWin();
  CSTMWin(CSTMWin *hParent);
  CSTMWin(CSTMWin *hParent, CRect AreaRect);
  CSTMWin(CSTMWin *hParent, CRect AreaRect, uint16_t TextColor,  uint16_t BackColor);
  CSTMWin(CSTMWin *hParent, CRect AreaRect, uint16_t TextColor,  uint16_t BackColor, sFONT *pFont);
  virtual ~CSTMWin(){};

private:  
  CRect m_WinArea;
  uint8_t m_Enabled;
  uint8_t m_Visible;
  CSTMWin *m_hParent;
  uint16_t m_BackColor;
  uint16_t m_TextColor;
  sFONT *m_pFont;
  uint8_t m_NeedRefresh;

public:
  inline HANDLE_WIN GetHandle(){return (HANDLE_WIN)this;};
  inline HANDLE_WIN GetParent(){return (HANDLE_WIN)m_hParent;};
  inline void SetParent(HANDLE_WIN hParent){m_hParent=hParent;};
  inline void SetEnable(uint8_t bEnabled){m_Enabled = bEnabled; /*InvalidateArea();*/};  
  inline void SetVisible(uint8_t bVisible){m_Visible = bVisible; /*InvalidateArea();*/};
  inline uint8_t IsEnabled(){return m_Enabled;};  
  inline uint8_t IsVisible(){return (m_hParent == INVALID_HANDLE ? m_Visible : ((CSTMWin *)m_hParent)->IsVisible() && m_Visible );};
  void MoveToPosition(int16_t x, int16_t y);
  void SetSize(uint16_t nWidth, uint16_t nHeight);
  inline sFONT *GetFont(){return m_pFont;};
  inline uint16_t GetBackColor(){return m_BackColor;};
  inline uint16_t GetTextColor(){return m_TextColor;};
  inline void SetFont(sFONT *pFont){m_pFont = pFont;};
  inline void SetBackColor(uint16_t BackColor){m_BackColor = BackColor;};
  inline void SetTextColor(uint16_t TextColor){m_TextColor = TextColor;};
  inline void ClearArea(){CSTMLCDMngr::DrawFillRect(GetTopLeft().X, GetTopLeft().Y , 
                                           GetWidth(), GetHeight(), GetBackColor());};
  
  virtual void Refresh()=0; 
  
  inline virtual CSTMControl_Type GetType(){return CONTROL_TYPE_WIN;};
  Point GetTopLeft();
  inline uint16_t GetWidth(){return m_WinArea.Width();};
  inline uint16_t GetHeight(){return m_WinArea.Height();};  
  inline Point GetTopLeftRelative(){return m_WinArea.TopLeft;};

  void InvalidateArea();
  inline void RefreshAsync(){m_NeedRefresh = 1; /*if (m_hParent) m_hParent->RefreshAsync();*/};
  inline uint8_t IsInvalid(){return m_NeedRefresh;};
  
  
  virtual uint8_t KeyPress(InputKey key){return 0;};
  virtual uint8_t KeyPressed(InputKey key){return 0;};
  virtual uint8_t KeyReleased(InputKey key){return 0;};

};

#endif 
