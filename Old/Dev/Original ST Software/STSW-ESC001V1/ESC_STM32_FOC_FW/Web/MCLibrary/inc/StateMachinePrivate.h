/**
  ******************************************************************************
  * @file    StateMachinePrivate.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private definition of StateMachine class      
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
#ifndef __STATEMACHINEPRIVATE_H
#define __STATEMACHINEPRIVATE_H

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */

/** @addtogroup StateMachine
  * @{
  */

/** @defgroup StateMachine_class_private_types StateMachine class private types
* @{
*/

/** 
  * @brief  StateMachine class members definition
  */
typedef struct
{
    State_t   bState;          /*!< Variable containing state machine current
                                    state */
    FaultCondition_t  hFaultNow;       /*!< Bit fields variable containing faults 
                                    currently present */
    FaultCondition_t  hFaultOccurred;  /*!< Bit fields variable containing faults 
                                    historically occurred since the state 
                                    machine has been moved to FAULT_NOW state */
} Vars_t,*pVars_t;

/** 
  * @brief  Private StateMachine class definition 
  */
typedef struct
{
	Vars_t Vars_str; 		/*!< Class members container */
}_CSTM_t, *_CSTM;
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__STATEMACHINEPRIVATE_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
