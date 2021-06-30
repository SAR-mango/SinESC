/**
  ******************************************************************************
  * @file    OpenLoopPrivate.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private definition of OpenLoop class      
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
#ifndef __OPENLOOPPRIVATE_H
#define __OPENLOOPPRIVATE_H

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */

/** @addtogroup OpenLoop
  * @{
  */

/** @defgroup OpenLoop_class_private_types OpenLoop class private types
* @{
*/

/** 
  * @brief  OpenLoop class members definition
  */
typedef struct
{
  int16_t hVoltage; /*!< Open loop voltage to be applied */
  CSPD oVSS;        /*!< Related VSS object used */
  bool VFMode;      /*!< TRUE to enable V/F mode otherwise FALSE */
} Vars_t,*pVars_t;

/** 
  * @brief  Redefinition of parameter structure
  */
typedef OpenLoopParams_t Params_t, *pParams_t;

/** 
  * @brief  Private OpenLoop class definition 
  */
typedef struct
{
	Vars_t Vars_str; 		/*!< Class members container */
	pParams_t pParams_str;	/*!< Class parameters container */
} _COL_t, *_COL;
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__OPENLOOPPRIVATE_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
