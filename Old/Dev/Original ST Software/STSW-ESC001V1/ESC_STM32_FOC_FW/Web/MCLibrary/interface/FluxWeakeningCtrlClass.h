/**
  ******************************************************************************
  * @file    FluxWeakeningCtrlClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of FluxWeakeningCtrl class      
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
#ifndef __FLUXWEAKENINGCTRLCLASS_H
#define __FLUXWEAKENINGCTRLCLASS_H

/* Includes ------------------------------------------------------------------*/
#include "MC_type.h"
#include "PIRegulatorClass.h"

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */

/** @addtogroup FluxWeakeningCtrl
  * @{
  */
  
/** @defgroup FluxWeakeningCtrl_class_exported_types FluxWeakeningCtrl class exported types
* @{
*/

/** 
  * @brief FluxWeakeningCtrl class init structure type definition
  */
typedef struct
{
  CPI oFluxWeakeningPI;
  CPI oSpeedPI;  
} FWInit_t, *pFWInit_t;

/** 
  * @brief  Public FluxWeakeningCtrl class definition 
  */
typedef struct CFW_t *CFW;

/** 
  * @brief  FluxWeakeningCtrl class parameters definition  
  */
typedef const struct
{
  uint16_t hMaxModule; /*!< Circle limitation maximum allowed module */
  
  uint16_t hDefaultFW_V_Ref;      /*!< Default flux weakening voltage reference,
                                       tenth of percentage points*/
  int16_t  hDemagCurrent;         /*!< Demagnetization current in s16A: 
                                    Current(Amp) = [Current(s16A) * Vdd micro]/ 
                                    [65536 * Rshunt * Aop] */
  int32_t  wNominalSqCurr;        /*!< Squared motor nominal current in (s16A)^2 
                                       where: 
                                     Current(Amp) = [Current(s16A) * Vdd micro]/ 
                                     [65536 * Rshunt * Aop] */
  uint16_t hVqdLowPassFilterBW;   /*!< Use this parameter to configure the Vqd 
                                      first order software filter bandwidth. 
                                      hVqdLowPassFilterBW = FOC_CurrController 
                                      call rate [Hz]/ FilterBandwidth[Hz] in 
                                      case FULL_MISRA_COMPLIANCY is defined. 
                                      On the contrary, if FULL_MISRA_COMPLIANCY 
                                      is not defined, hVqdLowPassFilterBW is 
                                      equal to log with base two of previous 
                                      definition */  
  uint16_t hVqdLowPassFilterBWLOG;/*!< hVqdLowPassFilterBW expressed as power of 2.
                                       E.g. if gain divisor is 512 the value 
                                       must be 9 because 2^9 = 512 */
} FluxWeakeningCtrlParams_t, *pFluxWeakeningCtrlParams_t;
  
/**
* @}
*/

/** @defgroup FluxWeakeningCtrl_class_exported_methods FluxWeakeningCtrl class exported methods
  * @{
  */

/**
  * @brief  Creates an object of the class FeedForwardCtrl.
  * @param  pFeedForwardCtrlParams pointer to an FeedForwardCtrl parameters structure.
  * @retval CFF new instance of FeedForwardCtrl object.
  */
CFW FW_NewObject(pFluxWeakeningCtrlParams_t pFluxWeakeningCtrlParams);

/**
  * @brief  Initializes all the object variables, usually it has to be called 
  *         once right after object creation.
  * @param  this related object of class CFW.
  * @param  pFWInitStr Flux weakening init strutcture.
  * @retval none.
  */
void FW_Init(CFW this, pFWInit_t pFWInitStr);

/**
  * @brief  It should be called before each motor restart and clears the Flux 
  *         weakening internal variables with the exception of the target 
  *         voltage (hFW_V_Ref).
  * @param  this related object of class CFW.
  * @retval none
  */
void FW_Clear(CFW this);

/**
  * @brief  It computes Iqdref according the flux weakening algorithm.  Inputs
  *         are the starting Iqref components.
  *         As soon as the speed increases beyond the nominal one, fluxweakening 
  *         algorithm take place and handles Idref value. Finally, accordingly
  *         with new Idref, a new Iqref saturation value is also computed and
  *         put into speed PI.
  * @param  this related object of class CFW
  * @param  Iqdref The starting current components that have to be 
  *         manipulated by the flux weakening algorithm.
  * @retval Curr_Components Computed Iqdref.
  */
Curr_Components FW_CalcCurrRef(CFW this, Curr_Components Iqdref);

/**
  * @brief  It low-pass filters both the Vqd voltage components. Filter 
  *         bandwidth depends on hVqdLowPassFilterBW parameter 
  * @param  this related object of class CFW.
  * @param  Vqd Voltage componets to be averaged.
  * @retval none
  */
void FW_DataProcess(CFW this, Volt_Components Vqd);

/**
  * @brief  Use this method to set a new value for the voltage reference used by 
  *         flux weakening algorithm.
  * @param  this related object of class CFW.
  * @param  uint16_t New target voltage value, expressend in tenth of percentage
  *         points of available voltage.
  * @retval none
  */
void FW_SetVref(CFW this, uint16_t hNewVref);

/**
  * @brief  It returns the present value of target voltage used by flux 
  *         weakening algorihtm.
  * @param  this related object of class CFW.
  * @retval int16_t Present target voltage value expressed in tenth of 
  *         percentage points of available voltage.
  */
uint16_t FW_GetVref(CFW this);

/**
  * @brief  It returns the present value of voltage actually used by flux 
  *         weakening algorihtm.
  * @param  this related object of class CFW.
  * @retval int16_t Present averaged phase stator voltage value, expressed 
  *         in s16V (0-to-peak), where 
  *         PhaseVoltage(V) = [PhaseVoltage(s16A) * Vbus(V)] /[sqrt(3) *32767].
  */
int16_t FW_GetAvVAmplitude(CFW this);

/**
  * @brief  It returns the measure of present voltage actually used by flux 
  *         weakening algorihtm as percentage of available voltage.
  * @param  this related object of class CFW.
  * @retval uint16_t Present averaged phase stator voltage value, expressed in 
  *         tenth of percentage points of available voltage.
  */
uint16_t FW_GetAvVPercentage(CFW this);

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /* __FLUXWEAKENINGCTRLCLASS_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
