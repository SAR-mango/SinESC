/**
  ******************************************************************************
  * @file    InrushCurrentLimiterClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of InrushCurrentLimiter class      
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
#ifndef __INRUSHCURRENTLIMITERCLASS_H
#define __INRUSHCURRENTLIMITERCLASS_H

/* Includes ------------------------------------------------------------------*/
#include "MC_type.h"
#include "BusVoltageSensorClass.h"
#include "DigitalOutputClass.h"

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */

/** @addtogroup InrushCurrentLimiter
  * @{
  */
  
/** @defgroup InrushCurrentLimiter_class_exported_types InrushCurrentLimiter class exported types
* @{
*/

/** 
  * @brief  ICLState_t type definition, it's returned by ICL_GetState method 
  *         of ICL class to specify the actual ICL state
  */
typedef enum
{
 ICL_IDLE, ICL_ACTIVATION, ICL_ACTIVE, ICL_DEACTIVATION, ICL_INACTIVE
} ICLState_t;

/** 
  * @brief  Public InrushCurrentLimiter class definition 
  */
typedef struct CICL_t *CICL;

/** 
  * @brief  InrushCurrentLimiter class parameters definition  
  */
typedef const struct
{
  uint16_t hICLFrequencyHz; /*!< Frequency expressed in Hz at which the user 
                                 clocks the ICL calling ICL_Exec method */
  uint16_t hDurationms;     /*!< Duration of inrush limiter activation or
                                 deactivation expressed in milliseconds.*/
}InrushCurrentLimiterParams_t, *pInrushCurrentLimiterParams_t;
  
/**
* @}
*/

/** @defgroup InrushCurrentLimiter_class_exported_methods InrushCurrentLimiter class exported methods
  * @{
  */

/**
  * @brief  Creates an object of the class InrushCurrentLimiter.
  * @param  pInrushCurrentLimiterParams pointer to an InrushCurrentLimiter parameters structure.
  * @retval CICL new instance of InrushCurrentLimiter object.
  */
CICL ICL_NewObject(pInrushCurrentLimiterParams_t pInrushCurrentLimiterParams);

/**
  * @brief  Initializes all the object variables, usually it has to be called 
  *         once right after object creation. It is also used to assign the 
  *         bus voltage sensor and digital output to be used by the ICL.
  * @param  this related object of class CICL.
  * @param  oVBS the bus voltage sensor used by the ICL.
  * @param  oDOUT the digital output used by the ICL.
  * @retval none.
  */
void ICL_Init(CICL this, CVBS oVBS, CDOUT oDOUT);

/**
  * @brief  It clocks the inrush current limiter and must be called with a 
  *         frequency equal to the one settled in the parameters
  *         hEACFrequencyHz. 
  * @param  this related object of class CICL.
  * @retval ICLState_t returns the ICL state see 
  * \link InrushCurrentLimiter_class_exported_types ICLState_t\endlink.
  */
ICLState_t ICL_Exec(CICL this);

/**
  * @brief It returns the state of the ICL. See 
  * \link InrushCurrentLimiter_class_exported_types ICLState_t\endlink.
  * @param this related object of class CICL.
  * @retval ICLState_t returns the ICL state see 
  * \link InrushCurrentLimiter_class_exported_types ICLState_t\endlink.
  */
ICLState_t ICL_GetState(CICL this);

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /* __INRUSHCURRENTLIMITERCLASS_H */
/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
