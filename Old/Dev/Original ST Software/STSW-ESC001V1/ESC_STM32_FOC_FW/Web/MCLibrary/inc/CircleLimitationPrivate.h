/**
  ******************************************************************************
  * @file    CircleLimitationPrivate.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private definition of CircleLimitation class      
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
#ifndef __CIRCLELIMITATIONPRIVATE_H
#define __CIRCLELIMITATIONPRIVATE_H

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */

/** @addtogroup CircleLimitation
  * @{
  */

/** @defgroup CircleLimitation_class_private_types CircleLimitation class private types
* @{
*/

/** 
  * @brief  Redefinition of parameter structure
  */
typedef CircleLimitationParams_t Params_t, *pParams_t;

/** 
  * @brief  Private CircleLimitation class definition 
  */
typedef struct
{
  pParams_t pParams_str;	/*!< Class parameters container */
} _CCLM_t, *_CCLM;
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__CIRCLELIMITATIONPRIVATE_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
