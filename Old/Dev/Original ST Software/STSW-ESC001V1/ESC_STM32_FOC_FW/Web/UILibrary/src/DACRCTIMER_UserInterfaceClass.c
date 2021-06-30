/**
  ******************************************************************************
  * @file    DACRCTIMER_UserInterfaceClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private impelementation of DACRCTIMER class      
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "UserInterfaceClass.h"
#include "UserInterfacePrivate.h"
#include "DACRCTIMER_UserInterfaceClass.h"
#include "DACRCTIMER_UserInterfacePrivate.h"

#ifdef MC_CLASS_DYNAMIC
	#include "stdlib.h" /* Used for dynamic allocation */
#else
	#include "MC_type.h"
	#define MAX_DACT_UI_NUM 1u

	_DCDACT_UI_t DACT_UIpool[MAX_DACT_UI_NUM];
	unsigned char DACT_UI_Allocated = 0u;
#endif

static void DACT_Init_(CUI this);
static void DACT_Exec(CUI this);
static void DACT_SetChannelConfig(CUI this, DAC_Channel_t bChannel, 
                              MC_Protocol_REG_t bVariable);
static MC_Protocol_REG_t DACT_GetChannelConfig(CUI this, DAC_Channel_t bChannel);
static void DACT_SetUserChannelValue(CUI this, uint8_t bUserChNumber, 
                              int16_t hValue);
static int16_t DACT_GetUserChannelValue(CUI this, uint8_t bUserChNumber);

/**
  * @brief  Creates an object of the class DACRCTIMER
  * @param  pUserInterfaceParams pointer to an UserInterface parameters structure
  * @param  pDACRCTIMERParams pointer to an DACRCTIMER parameters structure
  * @retval CDACT_UI new instance of DACRCTIMER object
  */
CDACT_UI DACT_NewObject(pUserInterfaceParams_t pUserInterfaceParams, pDACRCTIMERParams_t pDACRCTIMERParams)
{
	_CUI _oUserInterface;
	_DCDACT_UI _oDACRCTIMER;

	_oUserInterface = (_CUI)UI_NewObject(pUserInterfaceParams);

	#ifdef MC_CLASS_DYNAMIC
		_oDACRCTIMER = (_DCDACT_UI)calloc(1u,sizeof(_DCDACT_UI_t));
	#else
		if (DACT_UI_Allocated  < MAX_DACT_UI_NUM)
		{
			_oDACRCTIMER = &DACT_UIpool[DACT_UI_Allocated++];
		}
		else
		{
			_oDACRCTIMER = MC_NULL;
		}
	#endif
  
	_oDACRCTIMER->pDParams_str = pDACRCTIMERParams;
	_oUserInterface->DerivedClass = (void*)_oDACRCTIMER;
  
	_oUserInterface->Methods_str.pUI_DACInit = &DACT_Init_;
  _oUserInterface->Methods_str.pUI_DACExec = &DACT_Exec;
  _oUserInterface->Methods_str.pUI_DACSetChannelConfig = &DACT_SetChannelConfig;
  _oUserInterface->Methods_str.pUI_DACGetChannelConfig = &DACT_GetChannelConfig;
  _oUserInterface->Methods_str.pUI_DACSetUserChannelValue = 
                                                      &DACT_SetUserChannelValue;
  _oUserInterface->Methods_str.pUI_DACGetUserChannelValue = 
                                                      &DACT_GetUserChannelValue;
  
	return ((CDACT_UI)_oUserInterface);
}

/** @addtogroup STM32F10x_PMSM_UI_Library
  * @{
  */
  
/** @addtogroup DAC_RC_TIMER_UserInterface
  * @{
  */

/** @defgroup DACRCTIMER_class_private_methods DACRCTIMER class private methods
* @{
*/

/**
  * @brief  Hardware and software initialization of the DAC object. This is the
  *         implementation of the virtual function.
  * @param  this related object of class UI. It must be a DACx_UI object casted
  *         to CUI otherwise the DACInit method will have no effect.
  * @retval none.
  */
static void DACT_Init_(CUI this)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  GPIO_InitTypeDef GPIO_InitStructure; 
  TIM_OCInitTypeDef TIM_OCInitStructure;
  
  /* Enable GPIOB */
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE);
  
  GPIO_StructInit(&GPIO_InitStructure);
  
  /* Configure PB.00 as alternate function output */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /* Enable TIM3 clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  
  TIM_DeInit(TIM3);
  
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  
  TIM_OCStructInit(&TIM_OCInitStructure);
  
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 0x800;          
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0;       
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;    
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  
  /* Output Compare PWM Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;                
  TIM_OCInitStructure.TIM_Pulse = 0x400; //Dummy value;    
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  
  /* Output Compare PWM Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;                   
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0x400; //Dummy value;    
  TIM_OC4Init(TIM3, &TIM_OCInitStructure);
  
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable); 
  
  /* Enable TIM3 counter */
  TIM_Cmd(TIM3, ENABLE);
}

/**
  * @brief  This method is used to update the DAC outputs. The selected 
  *         variables will be provided in the related output channels. This is 
  *         the implementation of the virtual function.
  * @param  this related object of class UI. It must be a DACx_UI object casted
  *         to CUI otherwise the DACInit method will have no effect.
  * @retval none.
  */
static void DACT_Exec(CUI this)
{
  MC_Protocol_REG_t bCh1_var,bCh2_var;
  bCh1_var = ((_DCDACT_UI)(((_CUI)this)->DerivedClass))->
    DVars_str.bChannel_variable[DAC_CH0];
  bCh2_var = ((_DCDACT_UI)(((_CUI)this)->DerivedClass))->
    DVars_str.bChannel_variable[DAC_CH1];
  
  TIM_SetCompare3(TIM3, ((u16)((s16)(((int16_t)UI_GetReg(this,bCh1_var)+32768)/32))));
  TIM_SetCompare4(TIM3, ((u16)((s16)(((int16_t)UI_GetReg(this,bCh2_var)+32768)/32))));
}

/**
  * @brief  This method is used to set up the DAC outputs. The selected
  *         variables will be provided in the related output channels after next
  *         DACExec. This is the implementation of the virtual function.
  * @param  this related object of class UI. It must be a DACx_UI object casted
  *         to CUI otherwise the DACInit method will have no effect.
  * @param  bChannel the DAC channel to be programmed. It must be one of the 
  *         exported channels Ex. DAC_CH0.
  * @param  bVariable the variables to be provided in out through the selected
  *         channel. It must be one of the exported UI register Ex. 
  *         MC_PROTOCOL_REG_I_A.
  * @retval none.
  */
static void DACT_SetChannelConfig(CUI this, DAC_Channel_t bChannel, 
                              MC_Protocol_REG_t bVariable)
{
  ((_DCDACT_UI)(((_CUI)this)->DerivedClass))->
    DVars_str.bChannel_variable[bChannel] = bVariable;
}

/**
  * @brief  This method is used to get the current DAC channel selected output.
  * @param  this related object of class UI. It must be a DACx_UI object casted
  *         to CUI otherwise the method will have no effect.
  * @param  bChannel the inspected DAC channel. It must be one of the 
  *         exported channels (Ex. DAC_CH0).
  * @retval MC_Protocol_REG_t The variables provided in out through the inspected
  *         channel. It will be one of the exported UI register (Ex. 
  *         MC_PROTOCOL_REG_I_A).
  */
static MC_Protocol_REG_t DACT_GetChannelConfig(CUI this, DAC_Channel_t bChannel)
{
  return ((_DCDACT_UI)(((_CUI)this)->DerivedClass))->
    DVars_str.bChannel_variable[bChannel];
}

/**
  * @brief  This method is used to set the value of the "User DAC channel".
  *         This is the implementation of the virtual function.
  * @param  this related object of class UI. It must be a DACx_UI object casted
  *         to CUI otherwise the DACInit method will have no effect.
  * @param  bUserChNumber the "User DAC channel" to be programmed.
  * @param  hValue the value to be put in output.
  * @retval none.
  */
void DACT_SetUserChannelValue(CUI this, uint8_t bUserChNumber, 
                              int16_t hValue)
{
  ((_DCDACT_UI)(((_CUI)this)->DerivedClass))->
    DVars_str.hUserValue[bUserChNumber] = hValue;
}

/**
  * @brief  This method is used to get the value of the "User DAC channel".
  *         This is the implementation of the virtual function.
  * @param  this related object of class UI. It must be a DACx_UI object casted
  *         to CUI otherwise the DACInit method will have no effect.
  * @param  bUserChNumber the "User DAC channel" to be programmed.
  * @retval none.
  */
int16_t DACT_GetUserChannelValue(CUI this, uint8_t bUserChNumber)
{
  return ((_DCDACT_UI)(((_CUI)this)->DerivedClass))->
    DVars_str.hUserValue[bUserChNumber];
}
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
