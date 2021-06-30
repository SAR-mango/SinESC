/**
  ******************************************************************************
  * @file    InrushCurrentLimiterPrivate.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private definition of InrushCurrentLimiter class      
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
#ifndef __INRUSHCURRENTLIMITERPRIVATE_H
#define __INRUSHCURRENTLIMITERPRIVATE_H

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */

/** @addtogroup InrushCurrentLimiter
  * @{
  */

/** @defgroup InrushCurrentLimiter_class_private_types InrushCurrentLimiter class private types
* @{
*/

/** 
  * @brief  InrushCurrentLimiter class members definition
  */
typedef struct
{
	CVBS oVBS;                /*!< CVBS object used for the automatic ICL 
                                 activation/deactivation.*/
  CDOUT oDOUT;              /*!< DOUT object used for activate/deactivate the ICL.*/
  ICLState_t ICLState;      /*!< State of ICL. See \link InrushCurrentLimiter_class_exported_types ICLState_t\endlink*/
  uint16_t hRemainingTicks; /*!< Number of clock events remaining to complete 
                                 the ICL activation/deactivation.*/
  uint16_t hTotalTicks;     /*!< Total number of clock events to complete 
                                 the ICL activation/deactivation.*/
}Vars_t,*pVars_t;

/** 
  * @brief  Redefinition of parameter structure
  */
typedef InrushCurrentLimiterParams_t Params_t, *pParams_t;

/** 
  * @brief  Private InrushCurrentLimiter class definition 
  */
typedef struct
{
	Vars_t Vars_str; 		/*!< Class members container */
	pParams_t pParams_str;	/*!< Class parameters container */
}_CICL_t, *_CICL;
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__INRUSHCURRENTLIMITERPRIVATE_H*/
/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
