/**
  ******************************************************************************
  * @file    MTPACtrlClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of MTPACtrl class      
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
#ifndef __MTPACTRLCLASS_H
#define __MTPACTRLCLASS_H

/* Includes ------------------------------------------------------------------*/
#include "MC_type.h"

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */

/** @addtogroup MTPACtrl
  * @{
  */
  
/** @defgroup MTPACtrl_class_exported_types MTPACtrl class exported types
* @{
*/

/** 
  * @brief  Public MTPACtrl class definition 
  */
typedef struct CMTPA_t *CMTPA;

/** 
  * @brief  MTPACtrl class parameters definition  
  */
typedef const struct
{
  int16_t  hSegDiv;               /*!< Segments divisor */
  int32_t  wAngCoeff[8];          /*!< Angular coefficients table */
  int32_t  wOffset[8];            /*!< Offsets table */
} MTPACtrlParams_t, *pMTPACtrlParams_t;
  
/**
* @}
*/

/** @defgroup MTPACtrl_class_exported_methods MTPACtrl class exported methods
  * @{
  */

/**
  * @brief  Creates an object of the class MTPACtrl.
  * @param  pMTPACtrlParams pointer to an MTPACtrl parameters structure.
  * @retval CMTPA new instance of MTPACtrl object.
  */
CMTPA MTPA_NewObject(pMTPACtrlParams_t pMTPACtrlParams);

/**
  * @brief  It compute new Iqdref starting from the input Iqdref.
  *         Iqref is keep unchanged and  is used to compute Idref according the
  *         MTPA algorithm. 
  * @param  this related object of class CMTPA
  * @param  Iqdref The starting current components that have to be 
  *         manipulated by the flux weakening algorithm.
  * @retval none
  */
Curr_Components MTPA_CalcCurrRef(CMTPA this, Curr_Components Iqdref);

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /* __MTPACTRLCLASS_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
