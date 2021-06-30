/**
  ******************************************************************************
  * @file    LCDVintage_UserInterfacePrivate.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private definition of LCDVintage class      
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
#ifndef __LCDVINTAGE_USERINTERFACEPRIVATE_H
#define __LCDVINTAGE_USERINTERFACEPRIVATE_H

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup UserInterface_LCDVintage
  * @{
  */

/** @defgroup LCDVintage_private_types LCDVintage private types
* @{
*/

/** 
  * @brief  LCDVintage class members definition 
  */
typedef struct
{
  CUI oDAC;                      /*!< Related DAC */
  uint8_t bDAC_CH0_Sel;          /*!< Selected DAC CH0 element */
  MC_Protocol_REG_t bDAC_CH0_ID; /*!< Selected DAC CH0 MC Protocol code */
  uint8_t bDAC_CH1_Sel;          /*!< Selected DAC CH1 element */
  MC_Protocol_REG_t bDAC_CH1_ID; /*!< Selected DAC CH1 MC Protocol code */
  uint8_t bDAC_Size;             /*!< Number of valid DAC variables */
  Curr_Components Iqdref;        /*!< Iqdref setled by LCD user */
} DVars_t,*pDVars_t;

/** 
  * @brief  Redefinition of parameter structure
  */
typedef LCDVintageParams_t DParams_t, *pDParams_t; 

/** 
  * @brief Private LCDVintage class definition 
  */
typedef struct
{
	DVars_t DVars_str;			/*!< Derived class members container */
	pDParams_t pDParams_str;	/*!< Derived class parameters container */
} _DCLCDV_UI_t, *_DCLCDV_UI;
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__LCDVINTAGE_USERINTERFACEPRIVATE_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
