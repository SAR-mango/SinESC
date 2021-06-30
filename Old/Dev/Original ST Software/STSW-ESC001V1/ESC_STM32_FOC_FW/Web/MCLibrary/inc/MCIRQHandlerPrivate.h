/**
  ******************************************************************************
  * @file    MCIRQHandlerPrivate.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private definition of FOCDrive class      
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
#ifndef __MC_IRQHANDLERPRIVATE_H
#define __MC_IRQHANDLERPRIVATE_H

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup MCIRQ_Handler
  * @{
  */

/** @defgroup MC_IRQHandler_private_types MCIRQ_Handler module private types
  * @{
  */

/** 
  * @brief  MCIRQ private module definition 
  */
typedef struct
{
  void* (*pIRQ_Handler)(void *this, unsigned char flag);
}_CMCIRQ_t,*_CMCIRQ;

/**
  * @}
  */
  
/** @defgroup MCIRQ_Handler_private_methods MCIRQ_Handler module private functions
  * @{
  */

/**
  * @brief  Insert the object oIRQ into the MC vector table
  * @param  oIRQ MCIRQ object to be added to the MC vector list
  * @param  bIRQAddr MC vector table position 
  * @retval None
  */
void Set_IRQ_Handler(unsigned char bIRQAddr,_CMCIRQ oIRQ);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* __MC_IRQHANDLERPRIVATE_H */


/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
