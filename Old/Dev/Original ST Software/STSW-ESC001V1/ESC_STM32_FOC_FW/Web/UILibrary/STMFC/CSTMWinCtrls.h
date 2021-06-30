/**
  ******************************************************************************
  * @file    CSTMWinCtrls.h
  * @author  Systems Lab & Techincal Marketing (SDD Group)
  * @version V0.0.1
  * @date    04/26/2010
  * @brief   This file provides the haeder for the controls available in the STM
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


#ifndef __CSTMWINCTRLS_H_
#define __CSTMWINCTRLS_H_

#include "CSTMWin.h"

#define  EVNT_CTRL_ON_CLICK(x,y) ((x).OnClick = (y))
#define  EVNT_EDIT_INT_ON_CHANGEVALUE(x,y) ((x).OnChangeValue = (y))
#define  EVNT_EDIT_ON_CHANGETEXT(x,y) ((x).OnChangeText = (y))
#define  EVNT_EDIT_ON_CHANGEPAGE(x,y) ((x).OnChangePage = (y))

typedef void (*PFN_ON_EVENT)(HANDLE_WIN hWin, uint32_t wParam);

class CSTMWinCtrl : public CSTMWin
{
public:  
  CSTMWinCtrl():CSTMWin(){m_bCanFocus = 0; m_bFocus = 0; };
  CSTMWinCtrl(CSTMWin *hParent) :CSTMWin(){m_bCanFocus = 0; m_bFocus = 0;};
  CSTMWinCtrl(CSTMWin *hParent, int16_t xPos, int16_t yPos) : CSTMWin(hParent, CRect(xPos, yPos,0,0)){m_bCanFocus = 0; m_bFocus = 0;};
  virtual ~CSTMWinCtrl(){};

private:  
  uint8_t m_bFocus;
  uint8_t m_bCanFocus;
  
public:  
  inline void SetFocus(uint8_t bFocus){ m_bFocus = 0; if (m_bCanFocus) m_bFocus = bFocus;};
  inline void LeaveComponent() {m_bFocus = 0;};
  inline void EnterComponent() {SetFocus(1);};
  inline uint8_t IsFocused(){return m_bFocus;};
  inline void SetFocusable(uint8_t bCanFocus) {m_bCanFocus = bCanFocus; if(!bCanFocus) m_bFocus = 0;};
  inline uint8_t IsFocusable(){return m_bCanFocus;};
};

class CSTMEditBase : public CSTMWinCtrl
{
public :
  CSTMEditBase():CSTMWinCtrl(){m_bEdit = 0;  SetFocusable(1);};
  CSTMEditBase(CSTMWin *hParent){m_bEdit = 0;  SetFocusable(1);}
  CSTMEditBase(CSTMWin *hParent, int16_t xPos, int16_t yPos):CSTMWinCtrl(hParent, xPos, yPos){m_bEdit = 0;  SetFocusable(1);};
  virtual ~CSTMEditBase(){};

private:
  uint8_t m_bEdit;

public:  
  inline void SetEdit(uint8_t bEdit){m_bEdit = bEdit;};
  inline uint8_t GetEdit() {return m_bEdit;};
  inline uint8_t IsEdit() {return m_bEdit && IsFocused();};
  
};

class CSTMText : public CSTMEditBase
{
public :
  CSTMText():CSTMEditBase(){m_lpzText = NULL; };
  CSTMText(char *lpzText );
  CSTMText(char *lpzText, CSTMWin *phParent, int16_t xPos, int16_t yPos);
  virtual ~CSTMText(){};
  
  PFN_ON_EVENT OnChangeText;
    
private:
  char *m_lpzText;
  
public: 
  inline virtual CSTMControl_Type GetType(){return CONTROL_TYPE_TEXT;};
  virtual void SetText(char *lpzText);
  inline void SetTextBase(char *lpzText){m_lpzText = lpzText;};
  inline char *GetText(){return m_lpzText;}

  virtual void Refresh();
  
  virtual uint8_t KeyPress(InputKey key);
  virtual uint8_t KeyPressed(InputKey key);
  virtual uint8_t KeyReleased(InputKey key);
};

class CSTMCombo : public CSTMText
{
public:  
  CSTMCombo():CSTMText(){m_lpzList = NULL; m_SelectedIndex = 0; m_ItemsCount = 0; m_MaxItemsSize = 0;};
  CSTMCombo(const char * const lpzList[], CSTMWin *hParent, int16_t xPos, int16_t yPos):
    CSTMText(NULL, hParent, xPos, yPos){m_lpzList = (const char**)lpzList; m_MaxItemsSize = 0; SetItemsCount(); SetSelectedIndex(0);};
  virtual ~CSTMCombo(){};

private:    
  const char **m_lpzList;
  uint16_t m_SelectedIndex;
  uint16_t m_ItemsCount;
  uint16_t m_MaxItemsSize;
  
  void SetItemsCount();
  virtual void SetText(char *lpzText);
  
public:
  inline virtual CSTMControl_Type GetType(){return CONTROL_TYPE_COMBO;};
  inline void SetSelectedIndex(uint16_t idx){if (idx < m_ItemsCount && m_ItemsCount){ m_SelectedIndex = idx; SetText((char *)m_lpzList[m_SelectedIndex]);}};
  inline uint16_t GetSelectedIndex(){return m_SelectedIndex;};
  inline char *GetSelectedText(){return CSTMText::GetText();};
  inline uint16_t GetItemsCount(){return m_ItemsCount;}

  virtual void Refresh();  

  virtual uint8_t KeyPress(InputKey key);
  virtual uint8_t KeyPressed(InputKey key);
  virtual uint8_t KeyReleased(InputKey key);
   
};


class CSTMButton : public CSTMWinCtrl
{
public :
  CSTMButton():CSTMWinCtrl(){m_lpzText = NULL; m_bPressed = 0; OnClick = NULL;};
  CSTMButton(char *lpzText );
  CSTMButton(char *lpzText, CSTMWin *phParent, int16_t xPos, int16_t yPos);
  virtual ~CSTMButton(){};

  PFN_ON_EVENT OnClick;
  
private:
  char *m_lpzText;
  uint8_t m_bPressed;
  

public: 
  inline virtual CSTMControl_Type GetType(){return CONTROL_TYPE_BUTTON;};
  virtual void SetText(char *lpzText);
  inline void SetTextBase(char *lpzText){m_lpzText = lpzText;};
  inline char *GetText(){return m_lpzText;}
  inline void SetPressed(uint8_t bPress){if (OnClick && m_bPressed && bPress == 0 && IsFocused())OnClick(this, 0); m_bPressed = bPress; };
  inline uint8_t IsPressed() {return m_bPressed && IsFocused();};
  virtual void Refresh();

  virtual uint8_t KeyPress(InputKey key);
  virtual uint8_t KeyPressed(InputKey key);
  virtual uint8_t KeyReleased(InputKey key);
  
};

class CSTMCheckBox : public CSTMWinCtrl
{
public :
  CSTMCheckBox():CSTMWinCtrl(){m_lpzText = NULL; m_bChecked = 0; OnClick = NULL;};
  CSTMCheckBox(char *lpzText );
  CSTMCheckBox(char *lpzText, CSTMWin *phParent, int16_t xPos, int16_t yPos);
  virtual ~CSTMCheckBox(){};

  PFN_ON_EVENT OnClick;
  
private:
  char *m_lpzText;
  uint8_t m_bChecked;

public: 
  inline virtual CSTMControl_Type GetType(){return CONTROL_TYPE_CHECKBOX;};
  virtual void SetText(char *lpzText);
  inline void SetTextBase(char *lpzText){m_lpzText = lpzText;};
  inline char *GetText(){return m_lpzText;}
  inline void SetChecked(uint8_t bCheck){if (OnClick && IsFocused())OnClick(this, bCheck); m_bChecked = bCheck; };
  inline uint8_t IsChecked() {return m_bChecked; };
  virtual void Refresh();

  virtual uint8_t KeyPress(InputKey key);
  virtual uint8_t KeyPressed(InputKey key);
  virtual uint8_t KeyReleased(InputKey key);
  
};


class CSTMLabel : public CSTMWinCtrl
{
public:
  CSTMLabel():CSTMWinCtrl(){m_lpzText = NULL;};
  CSTMLabel(char *lpzText);
  CSTMLabel(char *lpzText, CSTMWin *phParent, int16_t xPos, int16_t yPos);
  virtual ~CSTMLabel(){};
  
private:
  char *m_lpzText;

public:  
  inline virtual CSTMControl_Type GetType(){return CONTROL_TYPE_LABEL;};
  void SetText(char *lpzText);
  char *GetText(){return m_lpzText;};
  virtual void Refresh();
};

class CSTMGroup : public CSTMLabel
{
public:  
  CSTMGroup():CSTMLabel(){};
  CSTMGroup(char *lpzName, int16_t Width, int16_t Height, CSTMWin *hParent, int16_t xPos, int16_t yPos):
    CSTMLabel(lpzName, hParent, xPos, yPos){SetSize(Width, Height);};
  virtual ~CSTMGroup(){};

  inline virtual CSTMControl_Type GetType(){return CONTROL_TYPE_GROUP;};

  virtual void Refresh();  
   
};

class CSTMBitmap : public CSTMWinCtrl
{
public:
  CSTMBitmap():CSTMWinCtrl(){m_pBitmap = NULL;};
  CSTMBitmap(uint16_t *pBuffer, uint16_t Width, uint16_t Height, CSTMWin *phParent, int16_t xPos, int16_t yPos);
  virtual ~CSTMBitmap(){};
  
private:
  uint16_t *m_pBitmap;

public:  
  inline virtual CSTMControl_Type GetType(){return CONTROL_TYPE_BITMAP;};
  void SetImageBuffer(uint16_t *pBuffer){m_pBitmap = pBuffer; if (m_pBitmap)RefreshAsync();};
  uint16_t *GetImageBuffer(){return m_pBitmap;};
  virtual void Refresh();
};

typedef enum STMIntType_e
{
  IntType_Unkwnon = 0,
  IntType_UnSigned_32,
  IntType_Signed_32,
  IntType_UnSigned_16,
  IntType_Signed_16,
  IntType_UnSigned_8,
  IntType_Signed_8
}STMIntType;

class CSTMEditInt : public CSTMEditBase
{
public :
  CSTMEditInt();
  CSTMEditInt(uint32_t *pInt, char *lpzFormat, uint8_t size, CSTMWin *hParent, int16_t xPos, int16_t yPos);
  CSTMEditInt(int32_t *pInt, char *lpzFormat, uint8_t size, CSTMWin *hParent, int16_t xPos, int16_t yPos);
  CSTMEditInt(uint16_t *pInt, char *lpzFormat, uint8_t size, CSTMWin *hParent, int16_t xPos, int16_t yPos);
  CSTMEditInt(int16_t *pInt, char *lpzFormat, uint8_t size, CSTMWin *hParent, int16_t xPos, int16_t yPos);
  CSTMEditInt(uint8_t *pInt, char *lpzFormat, uint8_t size, CSTMWin *hParent, int16_t xPos, int16_t yPos);
  CSTMEditInt(int8_t *pInt, char *lpzFormat, uint8_t size, CSTMWin *hParent, int16_t xPos, int16_t yPos);

    virtual ~CSTMEditInt(){};
  
  PFN_ON_EVENT OnChangeValue;
  
private:
  uint32_t *m_pEditInt;
  char *m_lpzFormat;
  int16_t m_Inc;
  uint8_t m_IncAcc;
  STMIntType m_Type;
  uint16_t m_MinValue;
  uint16_t m_MaxValue;
  

public: 
  inline virtual CSTMControl_Type GetType(){return CONTROL_TYPE_EDIT_INT;};
  inline void SetIntAddress(uint32_t *pValue){m_Type = IntType_UnSigned_32; m_pEditInt = (uint32_t *)pValue;};
  inline void SetIntAddress(int32_t *pValue){m_Type = IntType_Signed_32;m_pEditInt = (uint32_t *)pValue;};
  inline void SetIntAddress(uint16_t *pValue){m_Type = IntType_UnSigned_16;m_pEditInt = (uint32_t *)pValue;};
  inline void SetIntAddress(int16_t *pValue){m_Type = IntType_Signed_16;m_pEditInt = (uint32_t *)pValue;};
  inline void SetIntAddress(uint8_t *pValue){m_Type = IntType_UnSigned_8;m_pEditInt = (uint32_t *)pValue;};
  inline void SetIntAddress(int8_t *pValue){m_Type = IntType_Signed_8; m_pEditInt = (uint32_t *)pValue;};
  inline uint32_t *GetIntAddress(){return m_pEditInt;}
  inline STMIntType GetIntType(){return m_Type;};
  inline int16_t GetInc(){return m_Inc;};
  inline void SetInc(int16_t Inc){m_Inc = Inc;};
  inline uint8_t GetIncAcc(){return m_IncAcc;};
  inline void SetIncAcc(uint8_t IncAcc){m_IncAcc = IncAcc;};

  inline void SetBounds(uint32_t min, uint32_t max){m_MinValue = min; m_MaxValue = max;};
  inline void SetBounds(int32_t min, int32_t max){m_MinValue = (uint32_t)min; m_MaxValue = (uint32_t)max;};
  void SetIntValue(uint32_t Value);
  void SetIntValue(int32_t Value);
  uint32_t GetIntValue();
  void StepItUp();
  void StepItDown();
  
  virtual void Refresh();
  
  virtual uint8_t KeyPress(InputKey key);
  virtual uint8_t KeyPressed(InputKey key);
  virtual uint8_t KeyReleased(InputKey key);

private:
  void FormatText(char *lpzTxt);
  
};

class CSTMEditInt32 : public CSTMEditInt
{
public :
  CSTMEditInt32();
  CSTMEditInt32(int32_t *pInt, char *lpzFormat, uint8_t size, CSTMWin *hParent, int16_t xPos, int16_t yPos);
  virtual ~CSTMEditInt32(){};
  
private:
  int32_t m_MinValue32;
  int32_t m_MaxValue32;
  
public: 
  inline void SetBounds(int32_t min, int32_t max){m_MinValue32 = min; m_MaxValue32 = max;};
  void SetIntValue(int32_t Value);
  void StepItUp();
  void StepItDown();
  
  virtual uint8_t KeyPress(InputKey key);
  virtual uint8_t KeyPressed(InputKey key);
};

class CSTMIntText : public CSTMEditInt
{
public:
  CSTMIntText():CSTMEditInt(){m_IntValue = 0;};
  CSTMIntText(char *lpzFormat, uint8_t size, CSTMWin *hParent, int16_t xPos, int16_t yPos):
    CSTMEditInt(&m_IntValue, lpzFormat, size, hParent, xPos, yPos){m_IntValue = 0;}  
    virtual ~CSTMIntText(){};
    
private:
  int16_t m_IntValue;

public:
  void SetIntValue(int32_t Value){CSTMEditInt::SetIntValue(Value);};
  int32_t GetIntValue(){return CSTMEditInt::GetIntValue();};

};



#endif 
