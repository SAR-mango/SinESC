/**
  ******************************************************************************
  * @file    CSTMWin.cpp
  * @author  Systems Lab & Techincal Marketing (SDD Group)
  * @version V0.0.1
  * @date    04/26/2010
  * @brief   This file provides all the base classes of the STM fondation class
  *          library
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


#include "CSTMWin.h"
#include "string.h"

CRect::CRect()
{
  TopLeft.X = 0;
  TopLeft.Y = 0;
  BottomRigth.X = 0;
  BottomRigth.Y = 0;
}

CRect::CRect(Point oTopLeft, Point oBottomRigth)
{
  TopLeft.X = oTopLeft.X;
  TopLeft.Y = oTopLeft.Y;
  BottomRigth.X = oBottomRigth.X;
  BottomRigth.Y = oBottomRigth.Y;
}

CRect::CRect(Point oTopLeft, uint16_t nWidth, uint16_t nHeight)
{
    TopLeft.X = oTopLeft.X;
    TopLeft.Y = oTopLeft.Y;
    BottomRigth.X = oTopLeft.X + nWidth;;
    BottomRigth.Y = oTopLeft.Y + nHeight;
}
CRect::CRect(int16_t X, int16_t Y, uint16_t nWidth, uint16_t nHeight)
{
    TopLeft.X = X;
    TopLeft.Y = Y;
    BottomRigth.X = X + nWidth;;
    BottomRigth.Y = Y + nHeight;
}

CSTMWin::CSTMWin()
{
   m_WinArea.TopLeft.X = 0;
   m_WinArea.TopLeft.Y = 0;
   m_WinArea.BottomRigth.X = 0;
   m_WinArea.BottomRigth.Y = 0;
   m_Visible = 1;
   m_Enabled = 1;
   m_NeedRefresh = 0;
   m_hParent = INVALID_HANDLE;
   CSTMLCDMngr::GetColors(&m_TextColor, &m_BackColor); 
   this->SetFont(CSTMLCDMngr::GetFont());
}

CSTMWin::CSTMWin(CSTMWin *hParent)
{
   m_WinArea.TopLeft.X = 0;
   m_WinArea.TopLeft.Y = 0;
   m_WinArea.BottomRigth.X = 0;
   m_WinArea.BottomRigth.Y = 0;
   m_Visible = 1;
   m_Enabled = 1;
   m_NeedRefresh = 0;
   m_hParent = hParent;
   CSTMLCDMngr::GetColors(&m_TextColor, &m_BackColor); 
   this->SetFont(CSTMLCDMngr::GetFont());

   //if parent defined set the same for the child
   if (m_hParent)
   {
     this->SetTextColor(m_hParent->GetTextColor());
     this->SetBackColor(m_hParent->GetBackColor());     
     this->SetFont(m_hParent->GetFont());
//     this->SetSize(m_hParent->GetWidth(), m_hParent->GetHeight());
   }
}

CSTMWin::CSTMWin(CSTMWin *hParent, CRect RectArea) 
{
   m_WinArea.TopLeft.X = RectArea.TopLeft.X;
   m_WinArea.TopLeft.Y = RectArea.TopLeft.Y;
   m_WinArea.BottomRigth.X = RectArea.BottomRigth.X;
   m_WinArea.BottomRigth.Y = RectArea.BottomRigth.Y;
   m_Visible = 1;
   m_Enabled = 1;
   m_NeedRefresh = 0;
   m_hParent = hParent;
   CSTMLCDMngr::GetColors(&m_TextColor, &m_BackColor); 
   SetFont(CSTMLCDMngr::GetFont());

   //if parent defined set the same for the child
   if (m_hParent)
   {
     SetTextColor(m_hParent->GetTextColor());
     SetBackColor(m_hParent->GetBackColor());     
     SetFont(m_hParent->GetFont());
   }
   
}

CSTMWin::CSTMWin(CSTMWin *hParent, CRect RectArea, uint16_t TextColor,  uint16_t BackColor)
{
   m_WinArea.TopLeft.X = RectArea.TopLeft.X;
   m_WinArea.TopLeft.Y = RectArea.TopLeft.Y;
   m_WinArea.BottomRigth.X = RectArea.BottomRigth.X;
   m_WinArea.BottomRigth.Y = RectArea.BottomRigth.Y;
   m_Visible = 1;
   m_Enabled = 1;
   m_NeedRefresh = 0;
   m_hParent = hParent;
   SetTextColor(TextColor);
   SetBackColor(BackColor);     
   SetFont(CSTMLCDMngr::GetFont());

   //if parent defined set the same for the child
   if (m_hParent)
   {
     SetFont(m_hParent->GetFont());
   }
}

CSTMWin::CSTMWin(CSTMWin *hParent, CRect RectArea, uint16_t TextColor,  uint16_t BackColor, sFONT *pFont)
{
   m_WinArea.TopLeft.X = RectArea.TopLeft.X;
   m_WinArea.TopLeft.Y = RectArea.TopLeft.Y;
   m_WinArea.BottomRigth.X = RectArea.BottomRigth.X;
   m_WinArea.BottomRigth.Y = RectArea.BottomRigth.Y;
   m_Visible = 1;
   m_Enabled = 1;
   m_hParent = hParent;
   m_NeedRefresh = 0;
   
   SetTextColor(TextColor);
   SetBackColor(BackColor);     
   SetFont(CSTMLCDMngr::GetFont());

   if (pFont)
     SetFont(pFont);
}


void CSTMWin::MoveToPosition(int16_t X, int16_t Y)
{
   Point Delta = {0,0};
  
   Delta.X = m_WinArea.TopLeft.X - X;
   Delta.Y = m_WinArea.TopLeft.Y - Y;
   m_WinArea.TopLeft.X = X;
   m_WinArea.TopLeft.Y = Y;
   m_WinArea.BottomRigth.X -= Delta.X;
   m_WinArea.BottomRigth.Y -= Delta.Y;
}

void CSTMWin::SetSize(uint16_t nWidth, uint16_t nHeight)
{
   m_WinArea.BottomRigth.X = m_WinArea.TopLeft.X + nWidth;
   m_WinArea.BottomRigth.Y = m_WinArea.TopLeft.Y + nHeight;
}

Point CSTMWin::GetTopLeft()
{
  Point pointRet = {0,0};
  
  if (GetParent() != INVALID_HANDLE)
  {
      pointRet.X = GetParent()->GetTopLeft().X + m_WinArea.TopLeft.X;
      pointRet.Y = GetParent()->GetTopLeft().Y + m_WinArea.TopLeft.Y;       
  }
  else
  {
    pointRet.X = m_WinArea.TopLeft.X;
    pointRet.Y = m_WinArea.TopLeft.Y;
  }
  
  return pointRet;
}

void CSTMWin::InvalidateArea()
{
  m_NeedRefresh = 0;

  if (IsVisible())
  {
    if (m_pFont)
      CSTMLCDMngr::SetFont(m_pFont);
    
    CSTMLCDMngr::SetColors(m_TextColor, m_BackColor);
  
    Refresh();
  }
  else
  {
    ClearArea();
  }
}


