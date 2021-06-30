/**
  ******************************************************************************
  * @file    OpenLoopClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of OpenLoop class      
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
#ifndef __OPENLOOPCLASS_H
#define __OPENLOOPCLASS_H

/* Includes ------------------------------------------------------------------*/
#include "MC_type.h"
#include "SpeednPosFdbkClass.h"

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */

/** @addtogroup OpenLoop
  * @{
  */
  
/** @defgroup OpenLoop_class_exported_types OpenLoop class exported types
* @{
*/

/** 
  * @brief  Public OpenLoop class definition 
  */
typedef struct COL_t *COL;

/** 
  * @brief  OpenLoop class parameters definition  
  */
typedef const struct
{
  int16_t hDefaultVoltage; /*! Default Open loop voltage. */
  bool VFMode;             /*!< Default mode. TRUE to enable V/F mode otherwise
                                FALSE */
  int16_t hVFOffset;       /*! Offset of V/F curve expressed in s16 Voltage 
                               applied when frequency is zero. */
  int16_t hVFSlope;        /*! Slope of V/F curve expressed in s16 Voltage for 
                               each 0.1Hz of mecchanical frequency increment. */
} OpenLoopParams_t, *pOpenLoopParams_t;
  
/**
* @}
*/

/** @defgroup OpenLoop_class_exported_methods OpenLoop class exported methods
  * @{
  */

/**
  * @brief  Creates an object of the class OpenLoop.
  * @param  pOpenLoopParams pointer to an OpenLoop parameters structure.
  * @retval COL new instance of OpenLoop object.
  */
COL OL_NewObject(pOpenLoopParams_t pOpenLoopParams);

/**
  * @brief  Initializes all the object variables, usually it has to be called 
  *         once right after object creation.
  * @param  this related object of class COL.
  * @param  oVSS Related VSS object used.
  * @retval none
  */
void OL_Init(COL this, CSPD oVSS);

/**
  * @brief  It return the open loop Vqd.
  * @param  this related object of class COL.
  * @retval Volt_Components Vqd conditioned values.
  */
Volt_Components OL_VqdConditioning(COL this);

/**
  * @brief  It allows changing applied open loop phase voltage.
  * @param  this related object of class COL
  * @param  hNewVoltage New voltage value to be applied by the open loop.
  * @retval None
  */
void OL_UpdateVoltage(COL this, int16_t hNewVoltage);

/**
  * @brief  It have to be called regularly to update the applied voltage in case
  *         V/F is enabled. If V/F is disabled nothing is done.
  * @param  this related object of class COL
  * @retval None
  */
void OL_Calc(COL this);

/**
  * @brief  It is used to enable or disable the V/F mode.
  * @param  this related object of class COL
  * @param  VFEnabling TRUE to enable V/F mode, FALSE to disable. 
  * @retval None
  */
void OL_VF(COL this, bool VFEnabling);

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /* __OPENLOOPCLASS_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
