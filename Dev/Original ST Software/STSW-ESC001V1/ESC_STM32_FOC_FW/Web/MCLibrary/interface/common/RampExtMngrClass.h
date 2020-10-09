/**
  ******************************************************************************
  * @file    RampExtMngrClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of RampExtMngr class      
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
#ifndef __RAMPEXTMNGRCLASS_H
#define __RAMPEXTMNGRCLASS_H

/* Includes ------------------------------------------------------------------*/
#include "MC_type.h"

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */

/** @addtogroup RampExtMngr
  * @{
  */
  
/** @defgroup RampExtMngr_class_exported_types RampExtMngr class exported types
* @{
*/

/** 
  * @brief  Public RampExtMngr class definition 
  */
typedef struct CREMNG_t *CREMNG;

/** 
  * @brief  RampExtMngr class parameters definition  
  */
typedef const struct
{
  uint16_t hFrequencyHz; /*!< Execution frequency expressed in Hz */
}RampExtMngrParams_t, *pRampExtMngrParams_t;
  
/**
* @}
*/

/** @defgroup RampExtMngr_class_exported_methods RampExtMngr class exported methods
  * @{
  */

/**
  * @brief  Creates an object of the class RampExtMngr
  * @param  pRampExtMngrParams pointer to an RampExtMngr parameters structure
  * @retval CREMNG new instance of RampExtMngr object
  */
CREMNG REMNG_NewObject(pRampExtMngrParams_t pRampExtMngrParams);

/**
  * @brief  It reset the state variable to zero.
  * @param  this related object of class CREMNG
  * @retval none.
  */
void REMNG_Init(CREMNG this);

/**
  * @brief  Exec the ramp calculations and returns the current value of the 
            state variable. 
            It must be called at fixed interval defined in the hExecFreq.
  * @param  this related object of class CREMNG
  * @retval int32_t value of the state variable
  */
int32_t REMNG_Calc(CREMNG this);

/**
  * @brief  Setup the ramp to be executed
  * @param  this related object of class CREMNG
  * @param  hTargetFinal (signed 32bit) final value of state variable at the end
  *         of the ramp.
  * @param  hDurationms (unsigned 32bit) the duration of the ramp expressed in 
  *         milliseconds. It is possible to set 0 to perform an instantaneous 
  *         change in the value.
  * @retval bool It returns TRUE is command is valid, FALSE otherwise
  */
bool REMNG_ExecRamp(CREMNG this, int32_t wTargetFinal, uint32_t wDurationms);

/**
  * @brief  Returns the current value of the state variable.
  * @param  this related object of class CREMNG
  * @retval int32_t value of the state variable
  */
int32_t REMNG_GetValue(CREMNG this);

/**
  * @brief  Check if the settled ramp has been completed.
  * @param  this related object of class CREMNG.
  * @retval bool It returns TRUE if the ramp is completed, FALSE otherwise.
  */
bool REMNG_RampCompleted(CREMNG this);

/**
  * @brief  Stop the execution of the ramp keeping the last reached value.
  * @param  this related object of class CREMNG.
  * @retval none
  */
void REMNG_StopRamp(CREMNG this);

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /* __RAMPEXTMNGRCLASS_H */
/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
