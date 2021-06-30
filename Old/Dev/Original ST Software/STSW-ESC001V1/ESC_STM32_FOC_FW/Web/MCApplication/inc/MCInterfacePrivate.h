/**
  ******************************************************************************
  * @file    MCInterfacePrivate.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private definition of MCInterface class      
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
#ifndef __MCINTERFACEPRIVATE_H
#define __MCINTERFACEPRIVATE_H

/** @addtogroup STM32F10x_PMSM_MC_Interface
  * @{
  */

/** @addtogroup MCInterface
  * @{
  */

/** @defgroup MCInterface_class_private_types MCInterface class private types
* @{
*/

/** 
  * @brief  MCInterface class members definition
  */
typedef struct
{
  CSTM oSTM; /*!< State machine object used by MCI.*/
  CSTC oSTC; /*!< Speed and torque controller object used by MCI.*/
  pFOCVars_t pFOCVars;    /*!< Pointer to FOC vars used by MCI.*/
  
  UserCommands_t lastCommand; /*!< Last command coming from the user.*/
  int16_t hFinalSpeed;        /*!< Final speed of last ExecSpeedRamp command.*/
  int16_t hFinalTorque;       /*!< Final torque of last ExecTorqueRamp
                                   command.*/
  Curr_Components Iqdref;     /*!< Current component of last 
                                   SetCurrentReferences command.*/
  uint16_t hDurationms;       /*!< Duration in ms of last ExecSpeedRamp or
                                   ExecTorqueRamp command.*/
                                     
  CommandState_t CommandState; /*!< The status of the buffered command.*/
  STC_Modality_t LastModalitySetByUser; /*!< The last STC_Modality_t set by the 
                                             user. */ 
} Vars_t,*pVars_t;

/** 
  * @brief  Redefinition of parameter structure
  */
typedef MCInterfaceParams_t Params_t, *pParams_t;

/** 
  * @brief  Private MCInterface class definition 
  */
typedef struct
{
	Vars_t Vars_str; 		/*!< Class members container */
	pParams_t pParams_str;	/*!< Class parameters container */
} _CMCI_t, *_CMCI;
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__MCINTERFACEPRIVATE_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
