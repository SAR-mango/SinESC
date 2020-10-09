/**
  ******************************************************************************
  * @file    DACRCTIMER_UserInterfaceClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of DACRCTIMER class      
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
#ifndef __DACRCTIMER_USERINTERFACECLASS_H
#define __DACRCTIMER_USERINTERFACECLASS_H

/** @addtogroup STM32_PMSM_UI_Library
  * @{
  */
  
/** @addtogroup DAC_RC_TIMER_UserInterface
  * @{
  */

/** @defgroup DACRCTIMER_class_exported_types DACRCTIMER class exported types
* @{
*/

/** 
  * @brief  Public DACRCTIMER class definition
  */
typedef struct CDACT_UI_t *CDACT_UI;

/** 
  * @brief  DACRCTIMER class parameters definition
  */
typedef const void DACRCTIMERParams_t, *pDACRCTIMERParams_t;
/**
  * @}
  */

/** @defgroup DACRCTIMER_class_exported_methods DACRCTIMER class exported methods
  * @{
  */
  
/**
  * @brief  Creates an object of the class DACRCTIMER
  * @param  pUserInterfaceParams pointer to an UserInterface parameters structure
  * @param  pDACRCTIMERParams pointer to an DACRCTIMER parameters structure
  * @retval CDACT_UI new instance of DACRCTIMER object
  */
CDACT_UI DACT_NewObject(pUserInterfaceParams_t pUserInterfaceParams, pDACRCTIMERParams_t pDACRCTIMERParams);

/**
  * @brief  Example of public method of the class DACRCTIMER
  * @param  this related object of class CDACT_UI
  * @retval none
  */
void DACT_Func(CDACT_UI this);
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__DACRCTIMER_USERINTERFACECLASS_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
