/**
  ******************************************************************************
  * @file    MCTuningClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private interface of MCTuning class       
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
#ifndef __MCTUNINGCLASSPRIVATE_H
#define __MCTUNINGCLASSPRIVATE_H

/* Includes ------------------------------------------------------------------*/
#include "MC_type.h"

/** @addtogroup STM32F10x_PMSM_MC_Interface
  * @{
  */

/** @addtogroup MCTuning
  * @{
  */
  
/** @defgroup MCTuning_class_exported_types MCTuning class exported types
* @{
*/

/** 
  * @brief  MC tuning internal objects initialization structure type;
  */
typedef struct
{
  CPI   oPIDSpeed;
  CPI   oPIDIq;
  CPI   oPIDId;
  CPI   oPIDFluxWeakening;
  CPWMC oPWMnCurrFdbk;
  CRUC  oRevupCtrl;
  CSPD  oSpeedSensorMain;
  CSPD  oSpeedSensorAux;
  CSPD  oSpeedSensorVirtual;
  CSTC  oSpeednTorqueCtrl;
  CSTM  oStateMachine;
  CTSNS oTemperatureSensor;
  CVBS  oBusVoltageSensor;
  CDOUT oBrakeDigitalOutput;
  CDOUT oNTCRelay;
  CMPM  oMPM;
  CFW   oFW;
  CFF   oFF;
  CHFI_FP oHFI;
  CSCC  oSCC;
  COTT  oOTT;
} MCTuningInitStruct_t;
  
/**
* @}
*/

/** @defgroup MCTuning_class_private_methods MCTuning class private methods
  * @{
  */

/**
  * @brief  Creates an object of the class MCTuning
  * @param  pMCTuningParams pointer to an MCTuning parameters structure
  * @retval CMCT new instance of MCTuning object
  */
CMCT MCT_NewObject(pMCTuningParams_t pMCTuningParams);

/**
  * @brief  Example of public method of the class MCTuning
  * @param  this related object of class CMCT
  * @retval none
  */
void MCT_Init(CMCT this, MCTuningInitStruct_t MCTuningInitStruct);

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /* __MCTUNINGCLASSPRIVATE_H */





/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
