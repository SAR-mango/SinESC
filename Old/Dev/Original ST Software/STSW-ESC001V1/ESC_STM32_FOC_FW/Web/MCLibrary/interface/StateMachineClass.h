/**
  ******************************************************************************
  * @file    StateMachineClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of StateMachine class      
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
#ifndef __STATEMACHINECLASS_H
#define __STATEMACHINECLASS_H

/* Includes ------------------------------------------------------------------*/
#include "MC_type.h"

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */

/** @addtogroup StateMachine
  * @{
  */

/** @defgroup StateMachine_class_exported_constants StateMachine class exported constants
  * @{
  */


/**
* @}
*/

/** @defgroup StateMachine_class_exported_types StateMachine class exported types
* @{
*/


/** 
  * @brief  Public StateMachine class definition 
  */
typedef struct CSTM_t *CSTM;
 
/**
* @}
*/

/** @defgroup StateMachine_class_exported_methods StateMachine class exported methods
  * @{
  */

/**
  * @brief  Creates an object of the class StateMachine.
  * @param pStateMachineParams pointer to a StateMachine parameters structure.
  * @retval CSTM new instance of StateMachine object.
  */
CSTM STM_NewObject(void);

/**
  * @brief  Initializes all the object variables, usually it has to be called 
  *         once right after object creation.
  * @param this related object of class CSTM.
  * @retval none.
  */
void STM_Init(CSTM this);

/**
  * @brief It submits the request for moving the state machine into the state 
  *        specified by bState (FAULT_NOW and FAUL_OVER are not handled by this 
  *        method). Accordingly with the current state, the command is really
  *        executed (state machine set to bState) or discarded (no 
  *        state changes) 
  * @param this related object of class CSTM.
  * @param bState New requested state
  * @retval bool It returns TRUE if the state has been really set equal to 
  *              bState, FALSE if the request has been discarded
  */
bool STM_NextState(CSTM this, State_t bState);

/**
  * @brief It clocks both HW and SW faults processing and update the state 
  *        machine accordingly with hSetErrors, hResetErrors and present state. 
  *        Refer to State_t description for more information about fault states.
  * @param this object of class CSTM
  * @param hSetErrors Bit field reporting faults currently present
  * @param hResetErrors Bit field reporting faults to be cleared
  * @retval State_t New state machine state after fault processing
  */
State_t STM_FaultProcessing(CSTM this, uint16_t hSetErrors, uint16_t 
                                                                  hResetErrors);

/**
  * @brief  Returns the current state machine state
  * @param  this object of class CSTM
  * @retval State_t Current state machine state
  */
State_t STM_GetState(CSTM this);

/**
  * @brief It reports to the state machine that the fault state has been 
  *        acknowledged by the user. If the state machine is in FAULT_OVER state
  *        then it is moved into STOP_IDLE and the bit field variable containing 
  *        information about the faults historically occured is cleared. 
  *        The method call is discarded if the state machine is not in FAULT_OVER
  * @param this object of class CSTM
  * @retval bool TRUE if the state machine has been moved to IDLE, FALSE if the 
  *        method call had no effects 
  */
bool STM_FaultAcknowledged(CSTM this);

/**
  * @brief It returns two 16 bit fields containing information about both faults
  *        currently present and faults historically occurred since the state 
  *        machine has been moved into state
  * \n\link Fault_generation_error_codes Returned error codes are listed here \endlink
  * @param this object of class CSTM.
  * @retval uint32_t  Two 16 bit fields: in the most significant half are stored
  *         the information about currently present faults. In the least 
  *         significant half are stored the information about the faults 
  *         historically occurred since the state machine has been moved into 
  *         FAULT_NOW state
  * \n\link Fault_generation_error_codes Returned error codes are listed here \endlink
  */
uint32_t STM_GetFaultState(CSTM this);

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /* __STATEMACHINECLASS_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
