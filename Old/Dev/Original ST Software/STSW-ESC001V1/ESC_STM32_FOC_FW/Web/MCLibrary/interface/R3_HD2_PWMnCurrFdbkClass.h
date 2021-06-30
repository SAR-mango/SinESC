/**
  ******************************************************************************
  * @file    R3_HD2_PWMnCurrFdbkClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of R3_HD2 class      
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
#ifndef __R3_HD2_PWMNCURRFDBKCLASS_H
#define __R3_HD2_PWMNCURRFDBKCLASS_H

#include "R3_DD_PWMnCurrFdbkClass.h"

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup PWMnCurrFdbk_R3_HD2
  * @{
  */

/** @defgroup R3_HD2_class_exported_types R3_HD2 class exported types
* @{
*/

/** 
  * @brief  Public R3_HD2 class definition
  */
typedef struct CR3HD2_PWMC_t *CR3HD2_PWMC;

/**
  * @}
  */

/** @defgroup R3_HD2_class_exported_methods R3_HD2 class exported 
  *           methods
  * @{
  */
  
/**
  * @brief  Creates an object of the class R3_HD2
  * @param  pPWMnCurrFdbkParams pointer to an PWMnCurrFdbk parameters structure
  * @param  pR3_DDParams pointer to an R3_DD parameters structure
  * @retval CR3HD2_PWMC new instance of R3_HD2 object
  */
CR3HD2_PWMC R3HD2_NewObject(pPWMnCurrFdbkParams_t pPWMnCurrFdbkParams, 
                                              pR3_DDParams_t pR3_DDParams);

/**
  * @brief  It perform the start of all the timers required by the control. 
            It utilizes TIM2 as temporary timer to achieve synchronization between 
            PWM signals.
            When this function is called, TIM1 and/or TIM8 must be in frozen state
            with CNT, ARR, REP RATE and trigger correctly set (these setting are 
            usually performed in the Init method accordingly with the configuration)
  * @param  none
  * @retval none
  */
void R3HD2_StartTimers(void);

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__R3_HD2_PWMNCURRFDBKCLASS_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
