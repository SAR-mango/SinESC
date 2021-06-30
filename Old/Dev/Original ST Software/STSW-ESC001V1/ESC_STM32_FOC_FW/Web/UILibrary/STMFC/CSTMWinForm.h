/**
  ******************************************************************************
  * @file    CSTMWin.cpp
  * @author  Systems Lab & Techincal Marketing (SDD Group)
  * @version V0.0.1
  * @date    04/26/2010
  * @brief   This file provides all froms class of the STM fondation class
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


#ifndef __CSTMWINFORM_H_
#define __CSTMWINFORM_H_

#include "CSTMWin.h"
#include "CSTMInput.h"
#include "CSTMWinCtrls.h"

#include "Parameters conversion.h"
#if defined(PFC_ENABLED)
#define MAX_NUMEBER_OF_COMPONENT 37
#else
#define MAX_NUMEBER_OF_COMPONENT 32
#endif
#define MAX_NUMBER_OF_PAGES 8

class CSTMWinForm : public CSTMWin
{
public:
  CSTMWinForm():CSTMWin(){m_Visualizable = 1;};
  CSTMWinForm(CSTMWin *hParent);
  CSTMWinForm(CSTMWin *hParent, CRect AreaRect, uint16_t TextColor,  uint16_t BackColor, sFONT *pFont):
    CSTMWin(hParent, AreaRect, TextColor, BackColor, pFont){m_Visualizable = 1;};

  virtual ~CSTMWinForm(){};
  inline virtual void Refresh(){ClearArea();};
  void SetVisualizable(uint8_t bVisualizable){m_Visualizable = bVisualizable; if(!bVisualizable)CSTMWin::SetVisible(0);};
  uint8_t IsVisualizable(){return m_Visualizable;};
  inline void SetVisible(uint8_t bVisible){CSTMWin::SetVisible(0); if (bVisible && IsVisualizable())CSTMWin::SetVisible(bVisible);};

  
private:
  uint8_t m_Visualizable;
  
};

class CSTMWinPage : public CSTMWinForm
{
public:
  CSTMWinPage():CSTMWinForm(){m_CurrentControl = 0; m_NumberOfControl = 0; this->SetVisible(0);};
  CSTMWinPage(CSTMWin *hParent):CSTMWinForm(hParent){m_CurrentControl = 0; m_NumberOfControl = 0; this->SetVisible(0);};
  CSTMWinPage(CSTMWin *hParent, CRect AreaRect, uint16_t TextColor,  uint16_t BackColor, sFONT *pFont):
    CSTMWinForm(hParent, AreaRect, TextColor, BackColor, pFont){m_CurrentControl = 0; m_NumberOfControl = 0; this->SetVisible(0);};

  virtual ~CSTMWinPage(){};
  virtual void Refresh();
  
private:
  CSTMWinCtrl *m_ListControls[MAX_NUMEBER_OF_COMPONENT];
  uint16_t m_CurrentControl;
  uint16_t m_NumberOfControl;

public:
  void AddControl(CSTMWinCtrl *pControl);
  void SetActiveNextControl();
  void SetActivePreviousControl();
  CSTMWinCtrl *GetActiveControl();
  void SetActiveControl(uint16_t toActivate);
  void RemoveAllControls();

  void ValidateAreaAsync();
  void RefreshAllCtrlAsync();
  
  virtual uint8_t KeyPress(InputKey key);
  virtual uint8_t KeyPressed(InputKey key);
  virtual uint8_t KeyReleased(InputKey key);
  
};

class CSTMWinMainForm : public CSTMWinForm
{
public:
//  CSTMWinMainForm();
  CSTMWinMainForm(char *lpzCaption, CRect* AreaRect, uint16_t TextColor,  uint16_t BackColor, sFONT *pFont);
  virtual ~CSTMWinMainForm(){};

public:  
  CSTMWinForm m_WorkArea; 

private:  
  CSTMWinForm m_TitleBar;
  CSTMWinForm m_StatusBar;
  CSTMLabel m_TitleBarCaption;
  CSTMLabel m_StatusBarLabel;

  CSTMWinPage * m_PageList[MAX_NUMBER_OF_PAGES];
  uint16_t m_ActivePage;
  uint16_t m_CountPage;
  CSTMInput m_Input;

public:

  CSTMWin * GetTitleBar(){return &m_TitleBarCaption;};
  CSTMWin * GetWorkArea(){return &m_WorkArea;};
  CSTMWin * GetStatusBar(){return &m_StatusBarLabel;};
  
  void SetCaption(char *lpzText);
  void SetStatusLabel(char *lpzText);
  void AddPage(CSTMWin *newPage); 
  inline uint16_t GetNumberPages(){return m_CountPage;}
  inline uint16_t GetActivePageNumber(){return m_ActivePage;}
  void SetActivePage(uint16_t page);
  inline void MainWinUpdate(){ValidateAreaAsync(); m_Input.ReadKeys();};
  
  virtual uint8_t KeyPress(InputKey key);
  virtual uint8_t KeyPressed(InputKey key);
  virtual uint8_t KeyReleased(InputKey key);

  virtual void Refresh(){/* if (m_CountPage > m_ActivePage && GetActivePage() != INVALID_HANDLE) GetActivePage()->InvalidateArea();*/};
  
  PFN_ON_EVENT OnChangePage;
  
  CSTMWinPage * GetActivePage(){return m_PageList[m_ActivePage];};
  
private:  
  void SetNextActivePage();
  void SetPreviousActivePage();
  
  void ValidateAreaAsync();
  
  
};

#endif 
