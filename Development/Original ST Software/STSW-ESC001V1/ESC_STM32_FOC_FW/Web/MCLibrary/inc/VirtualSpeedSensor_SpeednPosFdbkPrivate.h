/**
  ******************************************************************************
  * @file    VirtualSpeedSensor_SpeednPosFdbkPrivate.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private definition of VirtualSpeedSensor class      
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
#ifndef __VIRTUALSPEEDSENSOR_SPEEDNPOSFDBKPRIVATE_H
#define __VIRTUALSPEEDSENSOR_SPEEDNPOSFDBKPRIVATE_H

#ifdef STM32F0XX
#include "FastDivClass.h"
#endif

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup SpeednPosFdbk_VirtualSpeedSensor
  * @{
  */

/** @defgroup VirtualSpeedSensor_private_types VirtualSpeedSensor private types
* @{
*/

/** 
  * @brief  VirtualSpeedSensor class members definition 
  */
typedef struct
{
  int32_t wElAccDppP32;   /*!< Delta electrical speed expressed in dpp per speed 
                               sampling period to be appied each time is called 
                               SPD_CalcAvrgMecSpeed01Hz multiplied by scaling
                               factor of 65536.*/
  int32_t wElSpeedDpp32;  /*!< Electrical speed expressed in dpp multiplied by
                               scaling factor 65536.*/
  uint16_t hRemainingStep;/*!< Number of steps remaining to reach the final
                               speed.*/
  int16_t hFinalMecSpeed01Hz;/*!< Backup of hFinalMecSpeed01Hz to be applied in 
                               the last step.*/
  bool bTransitionStarted;    /*!< Retaining information about Transition status.*/
  bool bTransitionEnded;      /*!< Retaining information about ransition status.*/
  int16_t hTransitionRemainingSteps;  /*!< Number of steps remaining to end
                               transition from CVSS_SPD to other CSPD*/
  int16_t hElAngleAccu;        /*!< Electrical angle accumulator*/
  bool bTransitionLocked;      /*!< Transition acceleration started*/
  bool bCopyObserver;          /*!< Command to set VSPD output same as state observer*/

#ifdef STM32F0XX
  /* (Fast division optimization for cortex-M0 micros)*/  
  CFD fd;                       /*!< Fast division obj.*/
#endif
} DVars_t,*pDVars_t;

/** 
  * @brief  Redefinition of parameter structure
  */
typedef VirtualSpeedSensorParams_t DParams_t, *pDParams_t; 

/** 
  * @brief Private VirtualSpeedSensor class definition 
  */
typedef struct
{
	DVars_t DVars_str;			/*!< Derived class members container */
	pDParams_t pDParams_str;	/*!< Derived class parameters container */
}_DCVSS_SPD_t, *_DCVSS_SPD;
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__VIRTUALSPEEDSENSOR_SPEEDNPOSFDBKPRIVATE_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
