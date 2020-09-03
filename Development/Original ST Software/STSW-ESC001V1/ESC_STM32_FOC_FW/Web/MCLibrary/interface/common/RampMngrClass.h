/**
  ******************************************************************************
  * @file    RampMngrClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of RampMngr class      
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
#ifndef __RAMPMNGRCLASS_H
#define __RAMPMNGRCLASS_H

/* Includes ------------------------------------------------------------------*/
#include "MC_type.h"

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */

/** @addtogroup RampMngr
  * @{
  */
  
/** @defgroup RampMngr_class_exported_types RampMngr class exported types
* @{
*/

/** 
  * @brief  Public RampMngr class definition 
  */
typedef struct CRMNG_t *CRMNG;

/** 
  * @brief  RampMngr class parameters definition  
  */
typedef const struct
{
  uint16_t hFrequencyHz; /*!< Execution frequency expressed in Hz */
}RampMngrParams_t, *pRampMngrParams_t;
  
/**
* @}
*/

/** @defgroup RampMngr_class_exported_methods RampMngr class exported methods
  * @{
  */

/**
  * @brief  Creates an object of the class RampMngr
  * @param  pRampMngrParams pointer to an RampMngr parameters structure
  * @retval CRMNG new instance of RampMngr object
  */
CRMNG RMNG_NewObject(pRampMngrParams_t pRampMngrParams);

/**
  * @brief  It reset the state variable to zero.
  * @param  this related object of class CRMNG
  * @retval none.
  */
void RMNG_Init(CRMNG this);

/**
  * @brief  Exec the ramp calculations and returns the current value of the 
            state variable. 
            It must be called at fixed interval defined in the hExecFreq.
  * @param  this related object of class CRMNG
  * @retval int16_t value of the state variable
  */
int16_t RMNG_Calc(CRMNG this);

/**
  * @brief  Setup the ramp to be executed
  * @param  this related object of class CRMNG
  * @param  hTargetFinal final value of state variable at the end of the ramp.
  * @param  hDurationms the duration of the ramp expressed in milliseconds. It
  *         is possible to set 0 to perform an instantaneous change in the value.
  * @retval none
  */
void RMNG_ExecRamp(CRMNG this, int16_t hTargetFinal, uint16_t hDurationms);

/**
  * @brief  Returns the current value of the state variable.
  * @param  this related object of class CRMNG
  * @retval int16_t value of the state variable
  */
int16_t RMNG_GetValue(CRMNG this);

/**
  * @brief  Check if the settled ramp has been completed.
  * @param  this related object of class CRMNG.
  * @retval bool It returns TRUE if the ramp is completed, FALSE otherwise.
  */
bool RMNG_RampCompleted(CRMNG this);

/**
  * @brief  Stop the execution of the ramp keeping the last reached value.
  * @param  this related object of class CRMNG.
  * @retval none
  */
void RMNG_StopRamp(CRMNG this);

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /* __RAMPMNGRCLASS_H */
/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
