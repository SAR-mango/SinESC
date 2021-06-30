/**
  ******************************************************************************
  * @file    CSTMInput.h
  * @author  Systems Lab & Techincal Marketing (SDD Group)
  * @version V0.0.1
  * @date    04/26/2010
  * @brief   This file is the header for the relative CSTMInput module 
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


#ifndef __CSTMINPUT_H_
#define __CSTMINPUT_H_

#include <stdio.h>

class CSTMWinMainForm;

typedef enum InputKey_e
{
  KEY_NONE           = 0,
  KEY_JOYSTICK_UP    = 1,
  KEY_JOYSTICK_DOWN  = 2,
  KEY_JOYSTICK_LEFT  = 3,
  KEY_JOYSTICK_RIGHT = 4,
  KEY_JOYSTICK_SEL   = 5,
  KEY_USER_BUTTON    = 6
}InputKey;

typedef void (*PFN_ON_KEYS_EVENT)(InputKey key);


class CSTMInput
{
public :  
  CSTMInput(){m_pMainForm = NULL;};
  CSTMInput(CSTMWinMainForm *hMainForm){m_pMainForm = hMainForm;  CSTMInput::Init();};
  virtual  ~CSTMInput(){};
  
  static void Init();
  void ReadKeys();
  
private :
  CSTMWinMainForm *m_pMainForm; 
  void ReadKey(InputKey inKey);
  
};


#endif //__CSTMINPUT_H_
