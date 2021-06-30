/**
  ******************************************************************************
  * @file    MCLibraryISRPriorityConf.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains the priority configuration for each interrupt 
  *			 service routine used by the MC library      
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
#ifndef __MCLIBRARYISRPRIORITYCONF_H
#define __MCLIBRARYISRPRIORITYCONF_H

/* Includes ------------------------------------------------------------------*/

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */
  
/** @defgroup MCLibraryISRPriorityConf_definitions MCLibraryISRPriorityConf definitions
* @{
*/

/** @brief Current sensor TIMx update ISR priority */
#define TIMx_UP_PRE_EMPTION_PRIORITY 0u
/** @brief Current sensor TIMx update ISR sub-priority */
#define TIMx_UP_SUB_PRIORITY 0u

/** @brief Current sensor DMAx TC ISR priority */ 
#define DMAx_TC_PRE_EMPTION_PRIORITY 0u
/** @brief Current sensor DMAx TC ISR sub-priority */
#define DMAx_TC_SUB_PRIORITY 0u

#if !defined(STM32F0XX)
/** @brief Current sensor ADC1_2 ISR priority */
#define ADC_PRE_EMPTION_PRIORITY 2u
/** @brief Current sensor ADC1_2 ISR sub-priority */
#define ADC_SUB_PRIORITY 0u

/** @brief Speed feedback sensor TIM ISR priority */
#define TIMx_PRE_EMPTION_PRIORITY 3u
/** @brief Speed feedback sensor TIM ISR sub-priority */
#define TIMx_SUB_PRIORITY 0u

/** @brief Serial communication USART ISR priority */
#define USART_PRE_EMPTION_PRIORITY 3u
/** @brief Serial communication USART ISR sub-priority */
#define USART_SUB_PRIORITY 1u

/** @brief Systick ISR priority */
#define SYSTICK_PRE_EMPTION_PRIORITY 4u
/** @brief Systick ISR sub-priority */
#define SYSTICK_SUB_PRIORITY 0u

/** @brief Current sensor TIMx BRK ISR priority */
#define TIMx_BRK_PRE_EMPTION_PRIORITY 4u
/** @brief Current sensor TIMx BRK ISR sub-priority */
#define TIMx_BRK_SUB_PRIORITY 1u

/** @brief PendSV ISR priority */
#define PENDSV_PRE_EMPTION_PRIORITY 5u
/** @brief PendSV ISR sub-priority */
#define PENDSV_SUB_PRIORITY 0u

/* Do not modify (NVIC_PriorityGroup_3 is assumed to be set) */
#define SYSTICK_PRIORITY (((SYSTICK_PRE_EMPTION_PRIORITY & 0x07) << 1) | (SYSTICK_SUB_PRIORITY & 0x01))
#define PENDSV_PRIORITY  (((PENDSV_PRE_EMPTION_PRIORITY & 0x07)  << 1) | (PENDSV_SUB_PRIORITY & 0x01))
#endif

#if defined(STM32F0XX)
/* Current sensor ADC1_2 ISR priority */
#define ADC_PRE_EMPTION_PRIORITY 1u
#define ADC_SUB_PRIORITY 0u

/* Speed feedback sensor TIM ISR priority */
#define TIMx_PRE_EMPTION_PRIORITY 2u
#define TIMx_SUB_PRIORITY 0u

/* Serial communication USART ISR priority */
#define USART_PRE_EMPTION_PRIORITY 3u
#define USART_SUB_PRIORITY 1u

/* Systick ISR priority */
#define SYSTICK_PRE_EMPTION_PRIORITY 2u
#define SYSTICK_SUB_PRIORITY 0u

/* PendSV ISR priority */
#define PENDSV_PRE_EMPTION_PRIORITY 3u
#define PENDSV_SUB_PRIORITY 0u

/* Do not modify (NVIC_PriorityGroup_3 is assumed to be set) */
#define SYSTICK_PRIORITY (SYSTICK_PRE_EMPTION_PRIORITY & 0x03)
#define PENDSV_PRIORITY  (PENDSV_PRE_EMPTION_PRIORITY  & 0x03)
#endif

/**
  * @}
  */

/**
  * @}
  */

#endif /* __MCLIBRARYISRPRIORITYCONF_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
