/**
  ******************************************************************************
  * @file    PID_PIRegulatorPrivate.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of Derived class      
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
#ifndef __PID_PIREGULATORPRIVATE_H
#define __PID_PIREGULATORPRIVATE_H


/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup PI_regulator_PID
  * @{
  */

/** @defgroup PID_private_types PID private types
* @{
*/

/** 
  * @brief  PID class members definition 
  */
typedef struct
{
  int32_t wPrevProcessVarError;
  int16_t hKdGain;
}DVars_t,*pDVars_t;

/** 
  * @brief  Redefinition of parameter structure
  */

typedef PIDParams_t DParams_t, *pDParams_t; 
/** 
  * @brief Private PID class definition 
  */
typedef struct
{
  DVars_t DVars_str;
  pDParams_t pDParams_str;  
}_DCPID_t, *_DCPID;

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */
#endif /*__PID_PIREGULATORPRIVATE_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
