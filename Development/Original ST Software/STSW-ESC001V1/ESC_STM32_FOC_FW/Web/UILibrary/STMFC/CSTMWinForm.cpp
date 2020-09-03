/**
  ******************************************************************************
  * @file    CSTMWinForm.cpp
  * @author  Systems Lab & Techincal Marketing (SDD Group)
  * @version V0.0.1
  * @date    04/26/2010
  * @brief   This file provides all the forms class of the STM fondation class
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


#include "CSTMWinForm.h"
#include "string.h"

CSTMWinForm::CSTMWinForm(CSTMWin *hParent): CSTMWin(hParent)
{
    if (hParent != INVALID_HANDLE)
    {
      this->SetSize(hParent->GetWidth(), hParent->GetHeight());
    }
    m_Visualizable = 1;
}


CSTMWinMainForm::CSTMWinMainForm(char *lpzText, CRect* prAreaRect, uint16_t nTextColor,  uint16_t nBackColor, sFONT *pFont)
                :CSTMWinForm(INVALID_HANDLE, *prAreaRect, nTextColor, nBackColor, pFont)
{
  //init the LCD driver
  CSTMLCDMngr::Init(); 

  m_CountPage = 0;
  m_ActivePage = 0;
  m_TitleBar = CSTMWinForm(this, CRect(0, 0, prAreaRect->Width(), Font16x24.Height + 2), White, Blue, &Font16x24);
  m_TitleBar.InvalidateArea();
  m_StatusBar = CSTMWinForm(this, CRect(0, prAreaRect->Height()-(pFont->Height + 2), prAreaRect->Width(), pFont->Height + 2), nTextColor, nBackColor, pFont);
  m_StatusBar.InvalidateArea();
  m_WorkArea = CSTMWinForm(this, CRect(0, m_TitleBar.GetHeight(), prAreaRect->Width(), prAreaRect->Height() - m_StatusBar.GetHeight() - m_TitleBar.GetHeight()), Black, White, pFont);
  m_WorkArea.InvalidateArea();

  m_TitleBarCaption  = CSTMLabel(lpzText, &m_TitleBar, 1, 1);
  m_StatusBarLabel = CSTMLabel(" ", &m_StatusBar, 1, 0);
  
  memset(m_PageList, 0, sizeof(CSTMWinPage *) * MAX_NUMBER_OF_PAGES);
  
  m_Input = CSTMInput(this);
  
  OnChangePage = NULL;
}

void CSTMWinMainForm::AddPage(CSTMWin *newPage)
{
  if (newPage)
  {
    if (m_CountPage < MAX_NUMBER_OF_PAGES)
    {
      m_PageList[m_CountPage++] = (CSTMWinPage *)newPage;
    }
  }
}

void CSTMWinMainForm::SetNextActivePage()
{
  if (m_CountPage > 0)
  {
    uint32_t LocalNewCurrentPage = m_ActivePage;
    do
    {
      LocalNewCurrentPage++;
      if (LocalNewCurrentPage >= m_CountPage )
        LocalNewCurrentPage = 0;

    }while (LocalNewCurrentPage != m_ActivePage && !m_PageList[LocalNewCurrentPage]->IsVisualizable()) ;
    
    SetActivePage(LocalNewCurrentPage);
  }
}

void CSTMWinMainForm::SetActivePage(uint16_t page)
{
  if (page < m_CountPage)
  {
    ((CSTMWinForm *)GetActivePage())->SetVisible(0);
    m_ActivePage = page;
    ((CSTMWinForm *)GetActivePage())->SetVisible(1);
    GetActivePage()->RefreshAsync();
    
    if(OnChangePage) 
      OnChangePage(this, page);
    
  }
}

void CSTMWinMainForm::SetPreviousActivePage()
{
  if (m_CountPage > 0)
  {
    uint32_t LocalNewCurrentPage = m_ActivePage;
    do
    {
      if (LocalNewCurrentPage > 0)
        LocalNewCurrentPage--;
      else
        LocalNewCurrentPage = m_CountPage - 1;
    }while (LocalNewCurrentPage != m_ActivePage && !m_PageList[LocalNewCurrentPage]->IsVisualizable()) ;
    
    SetActivePage(LocalNewCurrentPage);
  }
}

uint8_t CSTMWinMainForm::KeyPress(InputKey key)
{
  return this->GetActivePage()->KeyPress(key);
}
  
uint8_t CSTMWinMainForm::KeyPressed(InputKey key)
{
  return this->GetActivePage()->KeyPressed(key);
}

uint8_t CSTMWinMainForm::KeyReleased(InputKey key)
{
  uint8_t nRet = this->GetActivePage()->KeyReleased(key);
  if (!nRet)
  {
  switch(key)
   {
   case KEY_JOYSTICK_LEFT:
     this->SetPreviousActivePage();
//     this->GetActivePage()->InvalidateArea();
     nRet = 1;
     break;
   case KEY_JOYSTICK_RIGHT:
     this->SetNextActivePage();
     nRet = 1;
     break;
   }
  }
  return nRet;
}

void CSTMWinMainForm::SetCaption(char *lpzText)
{
  //((CSTMLabel *)GetTitleBar())->ClearArea(); 
  ((CSTMLabel *)GetTitleBar())->SetText(lpzText);
  
}

void CSTMWinMainForm::SetStatusLabel(char *lpzText)
{
  //((CSTMLabel *)GetStatusBar())->ClearArea(); 
  ((CSTMLabel *)GetStatusBar())->SetText(lpzText);
}

void CSTMWinMainForm::ValidateAreaAsync()
{
  if (GetTitleBar()->IsInvalid()) 
  {
     GetTitleBar()->ClearArea();
     GetTitleBar()->InvalidateArea();
  }
  if (GetStatusBar()->IsInvalid()) 
  {
    GetStatusBar()->ClearArea();
    GetStatusBar()->InvalidateArea();
  }

  if (IsInvalid()) 
  {
    InvalidateArea();
  }
  
  if (m_CountPage > m_ActivePage && GetActivePage() != INVALID_HANDLE) 
    GetActivePage()->ValidateAreaAsync();
}

void CSTMWinPage::ValidateAreaAsync()
{
  if (IsInvalid()) 
  {
    InvalidateArea();
  }
  if (m_NumberOfControl > 0)
  {
    for (int i = 0; i < m_NumberOfControl; i++)
    {
      if (m_ListControls[i]->IsInvalid())
        m_ListControls[i]->InvalidateArea();
    }
  }
}

void CSTMWinPage::Refresh()
{
  CSTMWinForm::Refresh();
  
  if (m_NumberOfControl > 0)
  {
    for (int i = 0; i < m_NumberOfControl; i++)
    {
      m_ListControls[i]->InvalidateArea();
    }
  }
}

void CSTMWinPage::AddControl(CSTMWinCtrl *pControl)
{
  if (pControl)
  {
    if (m_NumberOfControl < MAX_NUMEBER_OF_COMPONENT )
    {
      m_ListControls[m_NumberOfControl++] = pControl;
    }
  }
}

void CSTMWinPage::RemoveAllControls()
{
  uint8_t i;
  for (i = 0; i < m_NumberOfControl; i++)
  {
    delete m_ListControls[i];
  }
  m_CurrentControl = 0; 
  m_NumberOfControl = 0;
};

void CSTMWinPage::SetActiveNextControl()
{
  if (m_NumberOfControl > 0)
  {
    uint32_t LocalNewCurrentControl = m_CurrentControl;
    do
    {
      LocalNewCurrentControl++;
      if (LocalNewCurrentControl >= m_NumberOfControl)
        LocalNewCurrentControl = 0;
        
    }while (LocalNewCurrentControl != m_CurrentControl && !m_ListControls[LocalNewCurrentControl]->IsFocusable());
    
    SetActiveControl(LocalNewCurrentControl);
  }
}

void CSTMWinPage::SetActivePreviousControl()
{
  if (m_NumberOfControl > 0)
  {
    uint32_t LocalNewCurrentControl = m_CurrentControl;
    do
    {
      if (LocalNewCurrentControl == 0)
        LocalNewCurrentControl = m_NumberOfControl;
      LocalNewCurrentControl--;
      
    }while (LocalNewCurrentControl != m_CurrentControl && !m_ListControls[LocalNewCurrentControl]->IsFocusable());
    
    SetActiveControl(LocalNewCurrentControl);
  }
}

CSTMWinCtrl *CSTMWinPage::GetActiveControl()
{
  if (m_NumberOfControl > m_CurrentControl)
    return m_ListControls[m_CurrentControl];
  else
    return INVALID_HANDLE;
}
    
void CSTMWinPage::SetActiveControl(uint16_t toActivate)
{
  if ( toActivate < m_NumberOfControl)
  {
    GetActiveControl()->LeaveComponent();
    GetActiveControl()->InvalidateArea();
    m_CurrentControl = toActivate;
    GetActiveControl()->EnterComponent();
    GetActiveControl()->InvalidateArea();
    
  }
  
}

void CSTMWinPage::RefreshAllCtrlAsync()
{ 
  if (m_NumberOfControl > 0)
  {
    for (int i = 0; i < m_NumberOfControl; i++)
    {
      m_ListControls[i]->RefreshAsync();
    }
  }

}

uint8_t CSTMWinPage::KeyReleased(InputKey key)
{
  
  uint8_t nRet = GetActiveControl()->KeyReleased(key);  
  return nRet;
}
  
uint8_t CSTMWinPage::KeyPressed(InputKey key)
{
  uint8_t nRet = GetActiveControl()->KeyPressed(key);
  
  if(!nRet)
  {
    switch(key)
   {
   case KEY_JOYSTICK_UP:
     this->SetActivePreviousControl();
     nRet = 1;
     break;
   case KEY_JOYSTICK_DOWN:
     this->SetActiveNextControl();
     nRet = 1;
     break;
   }
  }
  return nRet;  
}

uint8_t CSTMWinPage::KeyPress(InputKey key)
{
  uint8_t nRet = GetActiveControl()->KeyPress(key);
  
  if(!nRet)
  {
    switch(key)
    {
     case KEY_JOYSTICK_UP:
       this->SetActivePreviousControl();
       nRet = 1;
       break;
     case KEY_JOYSTICK_DOWN:
       this->SetActiveNextControl();
       nRet = 1;
       break;
    }
  }
  return nRet;  
}
