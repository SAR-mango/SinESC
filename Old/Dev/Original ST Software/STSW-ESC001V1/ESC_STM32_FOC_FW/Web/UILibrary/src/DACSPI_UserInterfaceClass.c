/**
  ******************************************************************************
  * @file    DACSPI_UserInterfaceClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private impelementation of DACSPI class      
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
#include "DACSPI_UserInterfaceClass.h"
#include "DACSPI_UserInterfacePrivate.h"

#ifdef MC_CLASS_DYNAMIC
	#include "stdlib.h" /* Used for dynamic allocation */
#else
	#include "MC_type.h"
	#define MAX_DACS_UI_NUM 1u

	_DCDACS_UI_t DACS_UIpool[MAX_DACS_UI_NUM];
	unsigned char DACS_UI_Allocated = 0u;
#endif

static void DACS_Init_(CUI this);
static void DACS_Exec(CUI this);
static void DACS_SetChannelConfig(CUI this, DAC_Channel_t bChannel, 
                              MC_Protocol_REG_t bVariable);
static MC_Protocol_REG_t DACS_GetChannelConfig(CUI this, DAC_Channel_t bChannel);
static void DACS_SetUserChannelValue(CUI this, uint8_t bUserChNumber, 
                              int16_t hValue);
static int16_t DACS_GetUserChannelValue(CUI this, uint8_t bUserChNumber);

/**
  * @brief  Creates an object of the class DACSPI
  * @param  pUserInterfaceParams pointer to an UserInterface parameters structure
  * @param  pDACSPIParams pointer to an DACSPI parameters structure
  * @retval CDACS_UI new instance of DACSPI object
  */
CDACS_UI DACS_NewObject(pUserInterfaceParams_t pUserInterfaceParams, pDACSPIParams_t pDACSPIParams)
{
	_CUI _oUserInterface;
	_DCDACS_UI _oDACSPI;

	_oUserInterface = (_CUI)UI_NewObject(pUserInterfaceParams);

	#ifdef MC_CLASS_DYNAMIC
		_oDACSPI = (_DCDACS_UI)calloc(1u,sizeof(_DCDACS_UI_t));
	#else
		if (DACS_UI_Allocated  < MAX_DACS_UI_NUM)
		{
			_oDACSPI = &DACS_UIpool[DACS_UI_Allocated++];
		}
		else
		{
			_oDACSPI = MC_NULL;
		}
	#endif
  
	_oDACSPI->pDParams_str = pDACSPIParams;
	_oUserInterface->DerivedClass = (void*)_oDACSPI;
  
	_oUserInterface->Methods_str.pUI_DACInit = &DACS_Init_;
  _oUserInterface->Methods_str.pUI_DACExec = &DACS_Exec;
  _oUserInterface->Methods_str.pUI_DACSetChannelConfig = &DACS_SetChannelConfig;
  _oUserInterface->Methods_str.pUI_DACGetChannelConfig = &DACS_GetChannelConfig;
  _oUserInterface->Methods_str.pUI_DACSetUserChannelValue = 
                                                      &DACS_SetUserChannelValue;
  _oUserInterface->Methods_str.pUI_DACGetUserChannelValue = 
                                                      &DACS_GetUserChannelValue;
  
	return ((CDACS_UI)_oUserInterface);
}

/** @addtogroup STM32F10x_PMSM_UI_Library
  * @{
  */
  
/** @addtogroup DAC_SPI_UserInterface
  * @{
  */

/** @defgroup DACSPI_class_private_methods DACSPI class private methods
* @{
*/

/**
  * @brief  Hardware and software initialization of the DAC object. This is the
  *         implementation of the virtual function.
  * @param  this related object of class UI. It must be a DACx_UI object casted
  *         to CUI otherwise the DACInit method will have no effect.
  * @retval none.
  */
static void DACS_Init_(CUI this)
{  
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef   SPI_InitStructure;
  
  /* Configure SPI1 pins: SCK, MISO and MOSI ---------------------------------*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* SPI1 Config -------------------------------------------------------------*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
  SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &SPI_InitStructure);

  /* Enable SPI1 */
  SPI_Cmd(SPI1, ENABLE);
}

/**
  * @brief  This method is used to update the DAC outputs. The selected 
  *         variables will be provided in the related output channels. This is 
  *         the implementation of the virtual function.
  * @param  this related object of class UI. It must be a DACx_UI object casted
  *         to CUI otherwise the DACInit method will have no effect.
  * @retval none.
  */
static void DACS_Exec(CUI this)
{
  int16_t hData;
	MC_Protocol_REG_t bCh1_var,bCh2_var;
  bCh1_var = ((_DCDACS_UI)(((_CUI)this)->DerivedClass))->
    DVars_str.bChannel_variable[DAC_CH0];
  bCh2_var = ((_DCDACS_UI)(((_CUI)this)->DerivedClass))->
    DVars_str.bChannel_variable[DAC_CH1];
  
  hData = (int16_t)UI_GetReg(this,bCh1_var);
  
  /* Wait for SPI1 Tx buffer empty */ 
  while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)==RESET);	
  SPI_I2S_SendData(SPI1, (uint8_t)(hData>>8));
  /* Wait for SPI1 Tx buffer empty */ 
  while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)==RESET);	  
  SPI_I2S_SendData(SPI1, (uint8_t)(hData));
  
  hData = (int16_t)UI_GetReg(this,bCh2_var);
  /* Wait for SPI1 Tx buffer empty */ 
  while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)==RESET);	
  SPI_I2S_SendData(SPI1, (uint8_t)(hData>>8));
  /* Wait for SPI1 Tx buffer empty */ 
  while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)==RESET);	
  SPI_I2S_SendData(SPI1, (uint8_t)(hData));
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
static void DACS_SetChannelConfig(CUI this, DAC_Channel_t bChannel, 
                              MC_Protocol_REG_t bVariable)
{
  ((_DCDACS_UI)(((_CUI)this)->DerivedClass))->
    DVars_str.bChannel_variable[bChannel] = bVariable;
}

static MC_Protocol_REG_t DACS_GetChannelConfig(CUI this, DAC_Channel_t bChannel)
{
  return ((_DCDACS_UI)(((_CUI)this)->DerivedClass))->
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
void DACS_SetUserChannelValue(CUI this, uint8_t bUserChNumber, 
                              int16_t hValue)
{
  ((_DCDACS_UI)(((_CUI)this)->DerivedClass))->
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
int16_t DACS_GetUserChannelValue(CUI this, uint8_t bUserChNumber)
{
  return ((_DCDACS_UI)(((_CUI)this)->DerivedClass))->
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
