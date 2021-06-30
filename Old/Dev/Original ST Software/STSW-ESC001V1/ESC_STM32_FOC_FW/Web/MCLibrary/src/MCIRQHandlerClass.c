/**
  ******************************************************************************
  * @file    MCIRQClass.c.c
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

/* Includes ------------------------------------------------------------------*/
#include "MCIRQHandlerClass.h"
#include "MCIRQHandlerPrivate.h"

/* Definition of MC IRQ Table */
#define MAX_MC_IRQ_NUM 4
_CMCIRQ MC_IRQTable[MAX_MC_IRQ_NUM];

/**
  * @brief  Insert the object oIRQ into the MC vector table
  * @param  oIRQ MCIRQ object to be added to the MC vector list
  * @param  bIRQAddr MC vector table position 
  * @retval None
  */
void Set_IRQ_Handler(unsigned char bIRQAddr, _CMCIRQ oIRQ)
{
  MC_IRQTable[bIRQAddr] = oIRQ;
}

/**
  * @brief Start the execution of the MC_IRQ at bIRQAddr position inside MC 
  *        vector table
  * @param bIRQAddr MC IRQ position inside MC vector table
  * @param flag parameter passed to the MC interrupt, for instance to identify 
  *             the event request the MC ISR execution
  * @retval pvoid pointer to Drive object related to the interrupt
  */
void* Exec_IRQ_Handler(unsigned char bIRQAddr, unsigned char flag)
{  
  return(MC_IRQTable[bIRQAddr]->pIRQ_Handler((void*)(MC_IRQTable)[bIRQAddr],flag));
}
/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/

