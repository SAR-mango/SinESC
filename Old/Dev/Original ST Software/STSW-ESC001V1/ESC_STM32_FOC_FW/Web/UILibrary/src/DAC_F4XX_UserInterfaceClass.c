/**
  ******************************************************************************
  * @file    DAC_F4XX_UserInterfaceClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private impelementation of DAC class for F4XX      
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
#include "DAC_UserInterfaceClass.h"
#include "DAC_UserInterfacePrivate.h"

#ifdef MC_CLASS_DYNAMIC
	#include "stdlib.h" /* Used for dynamic allocation */
#else
	#include "MC_type.h"
	#define MAX_DAC_UI_NUM 1u

	_DCDAC_UI_t DAC_UIpool[MAX_DAC_UI_NUM];
	unsigned char DAC_UI_Allocated = 0u;
#endif
  
#define DACOFF 32768

static void DAC_Init_(CUI this);
static void DAC_Exec(CUI this);
static void DAC_SetChannelConfig(CUI this, DAC_Channel_t bChannel, 
                              MC_Protocol_REG_t bVariable);
static MC_Protocol_REG_t DAC_GetChannelConfig(CUI this, DAC_Channel_t bChannel);
static void DAC_SetUserChannelValue(CUI this, uint8_t bUserChNumber, 
                              int16_t hValue);
static int16_t DAC_GetUserChannelValue(CUI this, uint8_t bUserChNumber);

/**
  * @brief  Creates an object of the class DAC
  * @param  pUserInterfaceParams pointer to an UserInterface parameters structure
  * @param  pDACParams pointer to an DAC parameters structure
  * @retval CDAC_UI new instance of DAC object
  */
CDAC_UI DAC_NewObject(pUserInterfaceParams_t pUserInterfaceParams, pDACParams_t pDACParams)
{
	_CUI _oUserInterface;
	_DCDAC_UI _oDAC;

	_oUserInterface = (_CUI)UI_NewObject(pUserInterfaceParams);

	#ifdef MC_CLASS_DYNAMIC
		_oDAC = (_DCDAC_UI)calloc(1u,sizeof(_DCDAC_UI_t));
	#else
		if (DAC_UI_Allocated  < MAX_DAC_UI_NUM)
		{
			_oDAC = &DAC_UIpool[DAC_UI_Allocated++];
		}
		else
		{
			_oDAC = MC_NULL;
		}
	#endif
  
	_oDAC->pDParams_str = pDACParams;
	_oUserInterface->DerivedClass = (void*)_oDAC;
  
	_oUserInterface->Methods_str.pUI_DACInit = &DAC_Init_;
  _oUserInterface->Methods_str.pUI_DACExec = &DAC_Exec;
  _oUserInterface->Methods_str.pUI_DACSetChannelConfig = &DAC_SetChannelConfig;
  _oUserInterface->Methods_str.pUI_DACGetChannelConfig = &DAC_GetChannelConfig;
  _oUserInterface->Methods_str.pUI_DACSetUserChannelValue = 
                                                      &DAC_SetUserChannelValue;
  _oUserInterface->Methods_str.pUI_DACGetUserChannelValue = 
                                                      &DAC_GetUserChannelValue;

	return ((CDAC_UI)_oUserInterface);
}

/** @addtogroup STM32F10x_PMSM_UI_Library
  * @{
  */
  
/** @addtogroup DAC_UserInterface
  * @{
  */

/** @defgroup DAC_class_private_methods DAC class private methods
* @{
*/

/**
  * @brief  Hardware and software initialization of the DAC object. This is the
  *         implementation of the virtual function.
  * @param  this related object of class UI. It must be a DACx_UI object casted
  *         to CUI otherwise the DACInit method will have no effect.
  * @retval none.
  */
static void DAC_Init_(CUI this)
{  
  GPIO_InitTypeDef GPIO_InitStructure; 
  DAC_InitTypeDef DAC_InitStructure;
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
  
  /* DAC Configuration */
  DAC_DeInit();
  DAC_InitStructure.DAC_Trigger = DAC_Trigger_Software;
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;
  DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude=DAC_TriangleAmplitude_1;
  DAC_Init(DAC_Channel_1, &DAC_InitStructure);
  DAC_Init(DAC_Channel_2, &DAC_InitStructure);
  /* Enable DAC Channel1 */
  DAC_Cmd(DAC_Channel_1, ENABLE);
  /* Enable DAC Channel2 */
  DAC_Cmd(DAC_Channel_2, ENABLE);
  
  /* Configure DAC Output Pin */
  GPIO_StructInit(&GPIO_InitStructure);
  /* GPIOA Configuration: PA4, PA5 Channel Output */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**
  * @brief  This method is used to update the DAC outputs. The selected 
  *         variables will be provided in the related output channels. This is 
  *         the implementation of the virtual function.
  * @param  this related object of class UI. It must be a DACx_UI object casted
  *         to CUI otherwise the DACInit method will have no effect.
  * @retval none.
  */
static void DAC_Exec(CUI this)
{
	MC_Protocol_REG_t bCh1_var,bCh2_var;
  bCh1_var = ((_DCDAC_UI)(((_CUI)this)->DerivedClass))->
    DVars_str.bChannel_variable[DAC_CH0];
  bCh2_var = ((_DCDAC_UI)(((_CUI)this)->DerivedClass))->
    DVars_str.bChannel_variable[DAC_CH1];
  
  DAC_SetDualChannelData(DAC_Align_12b_L,
                  DACOFF + ((int16_t)UI_GetReg(this,bCh2_var)),
                  DACOFF + ((int16_t)UI_GetReg(this,bCh1_var)));
  
  DAC_DualSoftwareTriggerCmd(ENABLE);
  
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
static void DAC_SetChannelConfig(CUI this, DAC_Channel_t bChannel, 
                              MC_Protocol_REG_t bVariable)
{
  ((_DCDAC_UI)(((_CUI)this)->DerivedClass))->
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
static MC_Protocol_REG_t DAC_GetChannelConfig(CUI this, DAC_Channel_t bChannel)
{
  return ((_DCDAC_UI)(((_CUI)this)->DerivedClass))->
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
void DAC_SetUserChannelValue(CUI this, uint8_t bUserChNumber, 
                              int16_t hValue)
{
  ((_DCDAC_UI)(((_CUI)this)->DerivedClass))->
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
int16_t DAC_GetUserChannelValue(CUI this, uint8_t bUserChNumber)
{
  return ((_DCDAC_UI)(((_CUI)this)->DerivedClass))->
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
