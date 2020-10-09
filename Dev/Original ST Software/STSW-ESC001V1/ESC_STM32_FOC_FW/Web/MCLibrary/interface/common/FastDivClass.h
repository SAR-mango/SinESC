/**
  ******************************************************************************
  * @file    FastDivClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of FastDiv class
  *          Fast division optimization for cortex-M0 micros
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
#ifndef __FASTDIVCLASS_H
#define __FASTDIVCLASS_H

/* Includes ------------------------------------------------------------------*/
#include "MC_type.h"

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */

/** @addtogroup FastDiv
  * @{
  */
  
/** @defgroup FastDiv_class_exported_types FastDiv class exported types
* @{
*/

/** 
  * @brief  Public FastDiv class definition 
  */
typedef struct CFD_t *CFD;
  
/**
* @}
*/

/** @defgroup FastDiv_class_exported_methods FastDiv class exported methods
  * @{
  */

/**
  * @brief  Creates an object of the class FastDiv
  * @param  pFastDivParams pointer to an FastDiv parameters structure
  * @retval CFD new instance of FastDiv object
  */
CFD FD_NewObject(void);

/**
  * @brief  Software initialization of FD object
  * @param  this related object of class CFD
  * @retval none
  */
void FD_Init(CFD this);

/**
  * @brief  Execute the fast division. Note: The first execution of new divider
  *         will take more time and need to be done in low frequency low 
  *         priority task. Further execution will be fast if same divider is 
  *         used. See MAX_FDIV definition for the progammed buffer lenght of 
  *         dividers.
  * @param  this related object of class CFD
  * @param  n Numerator
  * @param  d Denominator
  * @retval Integer division between n and d
  */
int32_t FD_FastDiv(CFD this, int32_t n, int32_t d);

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /* __FASTDIVCLASS_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
