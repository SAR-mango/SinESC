/**
  ******************************************************************************
  * @file    MCTasks.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file implementes tasks definition
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
#ifndef __MCTASKS_H
#define __MCTASKS_H

/* Includes ------------------------------------------------------------------*/
#include "MCTuningClass.h"
#include "MCInterfaceClass.h"

/** @addtogroup STM32_PMSM_MC_Application
  * @{
  */

/** @addtogroup MCTasks
  * @{
  */

/** @defgroup MCTask_exported_types MCTasks exported types
  * @{
  */

/**
  * @}
  */
  
/** @defgroup MCboot_exported_methods MCTasks exported methods
  * @{
  */

/**
  * @brief  It initializes the whole MC core according to user defined 
  *         parameters.
  * @param  oMCIList pointer to the vector of MCInterface objects that will be
  *         created and initialized. The vector must have length equal to the 
  *         number of motor drives.
  * @param  oMCTList pointer to the vector of MCTuning objects that will be
  *         created and initialized. The vector must have length equal to the 
  *         number of motor drives.
  * @retval None
  */
void MCboot(CMCI oMCIList[],CMCT oMCTList[]);

/**
  * @brief  It executes MC tasks: safety task and medium frequency for all
  *         drive instances. It have to be clocked with Systick frequnecy.
  * @param  None
  * @retval None
  */
void MC_Scheduler(void);

/**
  * @brief  It executes safety checks (e.g. bus voltage and temperature) for all
  *         drive instances. Faults flags are also here updated     
  * @param  None
  * @retval None
  */
void TSK_SafetyTask(void);

/**
  * @brief  Accordingly with the present state(s) of the state machine(s), it 
  *         executes those motor control duties requiring a high frequency rate
  *         and a precise timing (e.g. FOC current control loop)
  * @param  None
  * @retval uint8_t It return the motor instance number of last executed FOC.
  */
uint8_t TSK_HighFrequencyTask(void);

/**
  * @brief  This function is called by TIMx_UP_IRQHandler in case of dual MC and
  *         it allows to reserve half PWM period in advance the FOC execution on
  *         ADC ISR
  * @param  oDrive pointer to a CFOC object
  * @retval None
  */
void TSK_DualDriveFIFOUpdate(void *oDrive);

/**
  * @brief  It is executed when a general hardware failure has been detected by
  *         the microcontroller and is used to put the system in safety 
  *         condition.
  * @param  None
  * @retval None
  */
void TSK_HardwareFaultTask(void);

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /* __MCTASKS_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
