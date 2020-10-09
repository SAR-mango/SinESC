/**
  ******************************************************************************
  * @file    UserInterfacePrivate.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private definition of UserInterface class      
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USERINTERFACEPRIVATE_H
#define __USERINTERFACEPRIVATE_H

/** @addtogroup STM32_PMSM_UI_Library
  * @{
  */

/** @addtogroup UserInterface
  * @{
  */

/** @defgroup UserInterface_class_private_types UserInterface class private types
* @{
*/

/** 
  * @brief  UserInterface class members definition
  */
typedef struct
{
  uint8_t bDriveNum;      /*!< Total number of MC objects.*/
  CMCI* pMCI;             /*!< Pointer of MC interface list.*/
  CMCT* pMCT;             /*!< Pointer of MC tuning list.*/
  uint32_t* pUICfg;       /*!< Pointer of UI configuration list.*/
  uint8_t bSelectedDrive; /*!< Current selected MC object in the list.*/
} Vars_t,*pVars_t;

/** 
  * @brief  Redefinition of parameter structure
  */
typedef UserInterfaceParams_t Params_t, *pParams_t;

/**
  * @brief Virtual methods container
  */
typedef struct
{
  void* (*pIRQ_Handler)(void *this, unsigned char flag, unsigned short rx_data);
  
  /* DAC related functions */
  void (*pUI_DACInit)(CUI this);
  void (*pUI_DACExec)(CUI this);
  void (*pUI_DACSetChannelConfig)(CUI this, DAC_Channel_t bChannel, 
                                  MC_Protocol_REG_t bVariable);
  MC_Protocol_REG_t (*pUI_DACGetChannelConfig)(CUI this, DAC_Channel_t bChannel);
  void (*pUI_DACSetUserChannelValue)(CUI this, uint8_t bUserChNumber, 
                                     int16_t hValue);
  int16_t (*pUI_DACGetUserChannelValue)(CUI this, uint8_t bUserChNumber);
  /* LCD related functions */
  void (*pUI_LCDInit)(CUI this, CUI oDAC, const char* s_fwVer);
  void (*pUI_LCDExec)(CUI this);
  void (*pUI_LCDUpdateAll)(CUI this);
  void (*pUI_LCDUpdateMeasured)(CUI this);
} Methods_t,*pMethods_t;

/** 
  * @brief  Private UserInterface class definition 
  */
typedef struct
{
  Methods_t Methods_str;	/*!< Virtual methods container */
  Vars_t Vars_str; 		/*!< Class members container */
  pParams_t pParams_str;	/*!< Class parameters container */
  void *DerivedClass;		/*!< Pointer to derived class */
} _CUI_t, *_CUI;
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__USERINTERFACEPRIVATE_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
