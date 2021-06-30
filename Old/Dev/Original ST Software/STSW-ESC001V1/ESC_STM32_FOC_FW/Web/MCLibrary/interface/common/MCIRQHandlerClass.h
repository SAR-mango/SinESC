/**
  ******************************************************************************
  * @file    MCIRQHandlerClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of FOCDrive class      
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
#ifndef __MC_IRQHANDLERCLASS_H
#define __MC_IRQHANDLERCLASS_H

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup MCIRQ_Handler
  * @{
  */

/** @defgroup MC_IRQHandler_exported_definitions MCIRQ_Handler module exported definitions
  * @{
  */

/** 
  * @brief  MC IRQ interrupts addresses inside MC vector table 
  */
#define MC_IRQ_PWMNCURRFDBK_1   0u  /*!< Reserved for PWMnCurrFdbk first instance.*/
#define MC_IRQ_PWMNCURRFDBK_2   1u  /*!< Reserved for PWMnCurrFdbk second instance.*/
#define MC_IRQ_SPEEDNPOSFDBK_1  2u  /*!< Reserved for SpeednPosFdbk first instance.*/
#define MC_IRQ_SPEEDNPOSFDBK_2  3u  /*!< Reserved for SpeednPosFdbk second instance.*/

/**
* @}
*/

/** @defgroup MC_IRQHandler_exported_functions MCIRQ Handler module exported functions
  * @{
  */

/**
  * @brief Start the execution of the MC_IRQ at bIRQAddr position inside MC 
  *        vector table
  * @param bIRQAddr MC IRQ position inside MC vector table
  * @param flag parameter passed to the MC interrupt, for instance to identify 
  *             the event request the MC ISR execution
  * @retval pvoid pointer to Drive object related to the interrupt
  */
void* Exec_IRQ_Handler(unsigned char bIRQAddr, unsigned char flag);

/**
* @}
*/

/**
* @}
*/

/**
* @}
*/
#endif /* __MC_IRQHANDLERCLASS_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
