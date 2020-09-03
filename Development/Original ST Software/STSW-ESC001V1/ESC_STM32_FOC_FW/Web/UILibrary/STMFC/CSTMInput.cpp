/**
  ******************************************************************************
  * @file    CSTMInput.cpp
  * @author  Systems Lab & Techincal Marketing (SDD Group)
  * @version V0.0.1
  * @date    04/26/2010
  * @brief   This file provides all the function to interact with Joystick in 
  *           polling.
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

#define EVAL_USE_IOE (defined(USE_STM3210C_EVAL) || defined(USE_STM322xG_EVAL) || defined(USE_STM324xG_EVAL) || defined(USE_STM32446E_EVAL))

#include "CSTMWinForm.h"
#include "CSTMInput.h"


void CSTMInput::Init()
{
  STM_EVAL_JOYInit();
}

#define INPUT_MAX_VALUE_DELAY 5
#define NO_BUTTON_PRESSED 255

void CSTMInput::ReadKey(InputKey inKey)
{
  static uint8_t delay = INPUT_MAX_VALUE_DELAY;
  
#if EVAL_USE_IOE
  uint8_t Button = JOY_SEL;
#else
  Button_TypeDef Button = BUTTON_SEL;
#endif
  
  static uint8_t bKeyPress = NO_BUTTON_PRESSED;

  switch (inKey)
  {
    case KEY_JOYSTICK_UP:
      
#if EVAL_USE_IOE
      Button = JOY_UP;
#else
      Button = BUTTON_UP;      
#endif
      
      break;
    case KEY_JOYSTICK_DOWN:
      
#if EVAL_USE_IOE
      Button = JOY_DOWN;
#else
      Button = BUTTON_DOWN;      
#endif
      
      break;
    case KEY_JOYSTICK_LEFT:
      
#if EVAL_USE_IOE
      Button = JOY_LEFT;
#else
      Button = BUTTON_LEFT;      
#endif
      
      break;
    case KEY_JOYSTICK_RIGHT:
            
#if EVAL_USE_IOE
      Button = JOY_RIGHT;
#else
      Button = BUTTON_RIGHT;      
#endif
      
      break;
    case KEY_JOYSTICK_SEL:
            
#if EVAL_USE_IOE
      Button = JOY_SEL;
#else
      Button = BUTTON_SEL;      
#endif
      break;
   }

#if EVAL_USE_IOE
  JOYState_TypeDef joystate = IOE_JoyStickGetState();
  if (joystate == Button)
#else
  if (STM_EVAL_PBGetState(Button)== JOYSTIK_ACTIVE)    
#endif
  {
    if (bKeyPress == Button)
    {
      if (delay > 0)
      {
        delay--;
      }
      else      
      {
        if (m_pMainForm)
          m_pMainForm->KeyPressed(inKey);
      }
    }
    if (bKeyPress == NO_BUTTON_PRESSED)
    {
      bKeyPress = Button;
      if (m_pMainForm)
        m_pMainForm->KeyPress(inKey);
    }
  }

#if EVAL_USE_IOE
  if (joystate == JOY_NONE)
#else
  if (STM_EVAL_PBGetState(Button)== JOYSTIK_INACTIVE)
#endif
  {
    if (bKeyPress == Button)
    {
      bKeyPress = NO_BUTTON_PRESSED;
      delay = INPUT_MAX_VALUE_DELAY;
      if (m_pMainForm)
        m_pMainForm->KeyReleased(inKey);
    }
  }
}

void CSTMInput::ReadKeys()
{
  ReadKey(KEY_JOYSTICK_SEL);
  ReadKey(KEY_JOYSTICK_UP);
  ReadKey(KEY_JOYSTICK_DOWN);
  ReadKey(KEY_JOYSTICK_LEFT);
  ReadKey(KEY_JOYSTICK_RIGHT);
}
