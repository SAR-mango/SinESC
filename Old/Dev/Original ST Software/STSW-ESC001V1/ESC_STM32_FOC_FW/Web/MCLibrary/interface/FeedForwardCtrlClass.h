/**
  ******************************************************************************
  * @file    FeedForwardCtrlClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of FeedForwardCtrl class      
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
#ifndef __FEEDFORWARDCTRLCLASS_H
#define __FEEDFORWARDCTRLCLASS_H

/* Includes ------------------------------------------------------------------*/
#include "MC_type.h"
#include "BusVoltageSensorClass.h"
#include "PIRegulatorClass.h"
#include "SpeednPosFdbkClass.h"


/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */

/** @addtogroup FeedForwardCtrl
  * @{
  */
  
/** @defgroup FeedForwardCtrl_class_exported_types FeedForwardCtrl class exported types
* @{
*/

/** 
  * @brief FeedForwardCtrl class init structure type definition
  */
typedef struct
{
  CVBS oVBS;
  CPI  oPI_q;
  CPI  oPI_d;
} FFInit_t, *pFFInit_t;

/** 
  * @brief  Public FeedForwardCtrl class definition 
  */
typedef struct CFF_t *CFF;

/** 
  * @brief  FeedForwardCtrl class parameters definition  
  */
typedef const struct
{
  uint16_t hVqdLowPassFilterBW;   /*!< Use this parameter to configure the Vqd 
                                      first order software filter bandwidth. 
                                      hVqdLowPassFilterBW = FOC_CurrController 
                                      call rate [Hz]/ FilterBandwidth[Hz] in 
                                      case FULL_MISRA_COMPLIANCY is defined. 
                                      On the contrary, if FULL_MISRA_COMPLIANCY 
                                      is not defined, hVqdLowPassFilterBW is 
                                      equal to log with base two of previous 
                                      definition */ 
  int32_t  wDefConstant_1D;       /*!< Feed forward default constant for d axes */
  int32_t  wDefConstant_1Q;       /*!< Feed forward default constant for q axes */
  int32_t  wDefConstant_2;        /*!< Default constant value used by 
                                       Feed-Forward algorithm*/
  uint16_t hVqdLowPassFilterBWLOG;/*!< hVqdLowPassFilterBW expressed as power of 2.
                                       E.g. if gain divisor is 512 the value 
                                       must be 9 because 2^9 = 512 */
} FeedForwardCtrlParams_t, *pFeedForwardCtrlParams_t;
  
/**
* @}
*/

/** @defgroup FeedForwardCtrl_class_exported_methods FeedForwardCtrl class exported methods
  * @{
  */

/**
  * @brief  Creates an object of the class FeedForwardCtrl.
  * @param  pFeedForwardCtrlParams pointer to an FeedForwardCtrl parameters structure.
  * @retval CFF new instance of FeedForwardCtrl object.
  */
CFF FF_NewObject(pFeedForwardCtrlParams_t pFeedForwardCtrlParams);

/**
  * @brief  Initializes all the object variables, usually it has to be called 
  *         once right after object creation.
  * @param  this related object of class CFF.
  * @param  pFFInitStr Feed forward init strutcture.
  * @retval none
  */
void FF_Init(CFF this, pFFInit_t pFFInitStruct);

/**
  * @brief  It should be called before each motor restart and clears the Flux 
  *         weakening internal variables.
  * @param  this related object of class CFF.
  * @retval none
  */
void FF_Clear(CFF this);

/**
  * @brief  It implements feed-forward controller by computing new Vqdff value. 
  *         This will be then summed up to PI output in IMFF_VqdConditioning 
  *         method.
  * @param  this related object of class CFF.
  * @param  oSPD related SPD object.
  * @param  Iqdref Idq reference componets used to calcupate the feed forward
  *         action.
  * @retval none
  */
void FF_VqdffComputation(CFF this, CSPD oSPD, Curr_Components Iqdref);

/**
  * @brief  It return the Vqd componets fed in input plus the feed forward 
  *         action and store the last Vqd values in the internal variable.
  * @param  this related object of class CFF.
  * @param  Vqd Initial value of Vqd to be manipulated by FF object.
  * @retval none
  */
Volt_Components FF_VqdConditioning(CFF this, Volt_Components Vqd);

/**
  * @brief  It low-pass filters the Vqd voltage coming from the speed PI. Filter 
  *         bandwidth depends on hVqdLowPassFilterBW parameter.
  * @param  this related object of class CFF.
  * @retval none
  */
void FF_DataProcess(CFF this);

/**
  * @brief  Use this method to initialize FF vars in START_TO_RUN state.
  * @param  this related object of class CFF.
  * @retval none
  */
void FF_InitFOCAdditionalMethods(CFF this);

/**
  * @brief  Use this method to set new values for the constants utilized by 
  *         feed-forward algorithm.
  * @param  this related object of class CFF.
  * @param  sNewConstants The FF_TuningStruct_t containing constants utilized by 
  *         feed-forward algorithm.
  * @retval none
  */
void FF_SetFFConstants(CFF this, FF_TuningStruct_t sNewConstants);

/**
  * @brief  Use this method to get present values for the constants utilized by 
  *         feed-forward algorithm.
  * @param  this related object of class CFF.
  * @retval FF_TuningStruct_t Values of the constants utilized by 
  *         feed-forward algorithm.
  */
FF_TuningStruct_t FF_GetFFConstants(CFF this);

/**
  * @brief  Use this method to get present values for the Vqd feed-forward 
  *         components.
  * @param  this related object of class CFF.
  * @retval Volt_Components Vqd feed-forward components.
  */
Volt_Components FF_GetVqdff(CFF this);

/**
  * @brief  Use this method to get values of the averaged output of qd axes 
  *         currents PI regulators.
  * @param  this related object of class CFF.
  * @retval Volt_Components Averaged output of qd axes currents PI regulators.
  */
Volt_Components FF_GetVqdAvPIout(CFF this);

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /* __FEEDFORWARDCTRLCLASS_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
