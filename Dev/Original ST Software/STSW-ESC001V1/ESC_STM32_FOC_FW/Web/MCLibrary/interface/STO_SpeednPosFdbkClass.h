/**
  ******************************************************************************
  * @file    STO_SpeednPosFdbkClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of STO class      
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
#ifndef __STO_SPEEDNPOSFDBKCLASS_H
#define __STO_SPEEDNPOSFDBKCLASS_H
#include "PIRegulatorClass.h"
#include "SpeednPosFdbkClass.h"
/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup SpeednPosFdbk_STO
  * @{
  */

/** @defgroup STO_class_exported_types STO class exported types
* @{
*/
/** 
  * @brief STO_SPD derived class input structure type definition for SPD_CalcAngle
  */
typedef struct
{ 
  Volt_Components  Valfa_beta;
  Curr_Components  Ialfa_beta;
  uint16_t         Vbus;
} Observer_Inputs_t;
/** 
  * @brief  Public STO class definition
  */
typedef struct CSTO_SPD_t *CSTO_SPD;

/** 
  * @brief  STO class parameters definition
  */
typedef const struct
{
   int16_t  hC1;                      /*!< State observer constant C1, it can 
                                           be computed as F1 * Rs(ohm)/(Ls[H] * 
                                           State observer execution rate [Hz])*/
   int16_t  hC2;                     /*!<  Variable containing state observer 
                                           constant C2, it can be computed as 
                                           F1 * K1/ State observer execution 
                                           rate [Hz] being K1 one of the two 
                                           observer gains   */   
   int16_t  hC3;                      /*!< State observer constant C3, it can 
                                           be computed as F1 * Max application 
                                           speed [rpm] * Motor B-emf constant 
                                           [Vllrms/krpm] * sqrt(2)/ (Ls [H] *
                                           max measurable current (A) * State 
                                           observer execution rate [Hz])*/ 
   int16_t  hC4;                      /*!< State Observer constant C4, it can 
                                           be computed as K2 * max measurable 
                                           current (A) / (Max application speed 
                                           [rpm] * Motor B-emf constant 
                                           [Vllrms/krpm] * sqrt(2) * F2 * State 
                                           observer execution rate [Hz]) being 
                                           K2 one of the two observer gains  */
   int16_t  hC5;                      /*!< State Observer constant C5, it can 
                                           be computed as F1 * max measurable 
                                           voltage / (2 * Ls [Hz] * max 
                                           measurable current * State observer 
                                           execution rate [Hz]) */
   int16_t  hF1;                      /*!< State observer scaling factor F1 */
   int16_t  hF2;                      /*!< State observer scaling factor F2 */
   PIParams_t PIParams;               /*!< It contains the parameters of the PI
                                           object necessary for PLL 
                                           implementation */
   uint8_t bSpeedBufferSize01Hz;       /*!< Depth of FIFO used to average 
                                           estimated speed exported by 
                                           SPD_GetAvrgMecSpeed01Hz. It 
                                           must be an integer number between 1 
                                           and 64 */  
   uint8_t bSpeedBufferSizedpp;       /*!< Depth of FIFO used for both averaging 
                                           estimated speed exported by 
                                           SPD_GetElSpeedDpp and state 
                                           observer equations. It must be an 
                                           integer number between 1 and
                                           bSpeedBufferSize01Hz */
   uint16_t hVariancePercentage;        /*!< Parameter expressing the maximum 
                                           allowed variance of speed estimation 
                                           */ 
   uint8_t bSpeedValidationBand_H;   /*!< It expresses how much estimated speed
                                           can exceed forced stator electrical 
                                           frequency during start-up without 
                                           being considered wrong. The 
                                           measurement unit is 1/16 of forced 
                                           speed */
   uint8_t bSpeedValidationBand_L;   /*!< It expresses how much estimated speed
                                           can be below forced stator electrical 
                                           frequency during start-up without 
                                           being considered wrong. The 
                                           measurement unit is 1/16 of forced 
                                           speed */
   uint16_t hMinStartUpValidSpeed;     /*!< Absolute vaule of minimum mechanical
                                            speed (expressed in 01Hz) required 
                                            to validate the start-up */
   uint8_t bStartUpConsistThreshold;   /*!< Number of consecutive tests on speed 
                                           consistency to be passed before 
                                           validating the start-up */
   uint8_t bReliability_hysteresys;    /*!< Number of reliability failed 
                                           consecutive tests before a speed 
                                           check fault is returned to base class 
                                           */
   uint8_t bBemfConsistencyCheck;      /*!< Degree of consistency of the observed
                                           back-emfs, it must be an integer
                                           number ranging from 1 (very high 
                                           consistency) down to 64 (very poor
                                           consistency) */
   uint8_t bBemfConsistencyGain;       /*!< Gain to be applied when checking
                                           back-emfs consistency; default value
                                           is 64 (neutral), max value 105
                                           (x1.64 amplification), min value 1
                                           (/64 attenuation) */
   uint16_t hMaxAppPositiveMecSpeed01Hz; /*!< Maximum positive value
                                             of rotor speed. It's expressed in 
                                             tenth of mechanical Hertz. It can be
                                             x1.1 greater than max application
                                             speed*/
   uint16_t hF1LOG;                    /*!< F1 gain divisor expressed as power of 2.
                                            E.g. if gain divisor is 512 the value 
                                            must be 9 because 2^9 = 512 */
   uint16_t hF2LOG;                    /*!< F2 gain divisor expressed as power of 2.
                                            E.g. if gain divisor is 512 the value 
                                            must be 9 because 2^9 = 512 */
   uint16_t bSpeedBufferSizedppLOG;    /*!< bSpeedBufferSizedpp expressed as power of 2.
                                            E.g. if gain divisor is 512 the value 
                                            must be 9 because 2^9 = 512 */
} STOParams_t, *pSTOParams_t;


typedef struct
{
   int16_t  hC1;                      /*!< State observer constant C1, it can 
                                           be computed as F1 * Rs(ohm)/(Ls[H] * 
                                           State observer execution rate [Hz])*/
   int16_t  hC2;                     /*!<  Variable containing state observer 
                                           constant C2, it can be computed as 
                                           F1 * K1/ State observer execution 
                                           rate [Hz] being K1 one of the two 
                                           observer gains   */   
   int16_t  hC3;                      /*!< State observer constant C3, it can 
                                           be computed as F1 * Max application 
                                           speed [rpm] * Motor B-emf constant 
                                           [Vllrms/krpm] * sqrt(2)/ (Ls [H] *
                                           max measurable current (A) * State 
                                           observer execution rate [Hz])*/ 
   int16_t  hC4;                      /*!< State Observer constant C4, it can 
                                           be computed as K2 * max measurable 
                                           current (A) / (Max application speed 
                                           [rpm] * Motor B-emf constant 
                                           [Vllrms/krpm] * sqrt(2) * F2 * State 
                                           observer execution rate [Hz]) being 
                                           K2 one of the two observer gains  */
   int16_t  hC5;                      /*!< State Observer constant C5, it can 
                                           be computed as F1 * max measurable 
                                           voltage / (2 * Ls [Hz] * max 
                                           measurable current * State observer 
                                           execution rate [Hz]) */
   int16_t  hF1;                      /*!< State observer scaling factor F1 */
   int16_t  hF2;                      /*!< State observer scaling factor F2 */
   uint16_t hF1LOG;                   /*!< F1 gain divisor expressed as power of 2.
                                            E.g. if gain divisor is 512 the value 
                                            must be 9 because 2^9 = 512 */
   uint16_t hF2LOG;                   /*!< F2 gain divisor expressed as power of 2.
                                            E.g. if gain divisor is 512 the value 
                                            must be 9 because 2^9 = 512 */
} STOInitVars_t, *pSTOInitVars_t;

/**
  * @}
  */

/** @defgroup STO_class_exported_methods STO class exported methods
  * @{
  */
  
/**
  * @brief  Creates an object of the class STO
  * @param  pSpeednPosFdbkParams pointer to an SpeednPosFdbk parameters structure
  * @param  pSTOParams pointer to an STO parameters structure
  * @retval CSTO_SPD new instance of STO object
  */
CSTO_SPD STO_NewObject(pSpeednPosFdbkParams_t pSpeednPosFdbkParams, pSTOParams_t
                                                                    pSTOParams);

/**
  * @brief  It internally performes a set of checks necessary to state whether
  *         the state observer algorithm converged. To be periodically called 
  *         during motor open-loop ramp-up (e.g. at the same frequency of 
  *         SPD_CalcElAngle), it returns TRUE if the estimated angle and speed 
  *         can be considered reliable, FALSE otherwise
  * @param  this related object of class CSTO_SPD
  * @param  hForcedMecSpeed01Hz Mechanical speed in 0.1Hz unit as forced by VSS
  * @retval bool sensor reliability state
  */
bool STO_IsObserverConverged(CSTO_SPD this, int16_t hForcedMecSpeed01Hz);

/**
  * @brief  It exports estimated Bemf alpha-beta in Volt_Components format
  * @param  this related object of class CSTO_SPD
  * @retval Volt_Components Bemf alpha-beta 
  */
Volt_Components STO_GetEstimatedBemf(CSTO_SPD this);

/**
  * @brief  It exports the stator current alpha-beta as estimated by state 
  *         observer
  * @param  this related object of class CSTO_SPD
  * @retval Curr_Components State observer estimated stator current Ialpha-beta 
  */
Curr_Components STO_GetEstimatedCurrent(CSTO_SPD this);

/**
  * @brief  It exports current observer gains through parameters hC2 and hC4
  * @param  this related object of class CSTO_SPD
  * @param  pC2 pointer to int16_t used to return parameters hC2
  * @param  pC4 pointer to int16_t used to return parameters hC4
  * @retval none 
  */
void STO_GetObserverGains(CSTO_SPD this, int16_t *pC2, int16_t *pC4);

/**
  * @brief  It allows setting new values for observer gains
  * @param  this related object of class CSTO_SPD
  * @param  hC1 new value for observer gain hC1
  * @param  hC2 new value for observer gain hC2
  * @retval none 
  */
void STO_SetObserverGains(CSTO_SPD this, int16_t hC1, int16_t hC2);

/**
  * @brief  It exports current PLL gains through parameters pPgain and pIgain
  * @param  this related object of class CSTO_SPD
  * @param  pPgain pointer to int16_t used to return PLL proportional gain
  * @param  pIgain pointer to int16_t used to return PLL integral gain
  * @retval none 
  */
void STO_GetPLLGains(CSTO_SPD this, int16_t *pPgain, int16_t *pIgain);

/**
  * @brief  It allows setting new values for PLL gains
  * @param  this related object of class CSTO_SPD
  * @param  hPgain new value for PLL proportional gain 
  * @param  hIgain new value for PLL integral gain 
  * @retval none 
  */
void STO_SetPLLGains(CSTO_SPD this, int16_t hPgain, int16_t hIgain);

/**
  * @brief  It resets integral term of PLL
  * @param  this related object of class CSTO_SPD 
  * @retval none 
  */
void STO_ResetPLL(CSTO_SPD this);

/**
  * @brief  It sends locking info for PLL
  * @param  this related object of class CSTO_SPD 
  * @retval none 
  */
void STO_SetPLL(CSTO_SPD this, int16_t hElSpeedDpp, int16_t hElAngle);

/**
  * @brief  This method must be called - at least - with the same periodicity
  *         on which speed control is executed. It computes and update the 
  *         estimated average electrical speed - expressed in dpp - used for 
  *         instance in observer equations. 
  *         Average is computed considering a FIFO depth equal to 
  *         bSpeedBufferSizedpp. 
  * @param  this related object of class CSPD
  * @retval none
  */
void STO_CalcAvrgElSpeedDpp(CSTO_SPD this);

/**
  * @brief  It exports estimated Bemf squared level
  * @param  this related object of class CSTO_SPD
  * @retval int32_t 
  */
int32_t STO_GetEstimatedBemfLevel(CSTO_SPD this);

/**
  * @brief  It exports observed Bemf squared level
  * @param  this related object of class CSTO_SPD
  * @retval int32_t 
  */
int32_t STO_GetObservedBemfLevel(CSTO_SPD this);

/**
  * @brief  It enables/disables the bemf consistency check
  * @param  this related object of class CSTO_SPD
  * @param  bSel boolean; TRUE enables check; FALSE disables check
  * @retval int32_t 
  */
void STO_BemfConsistencyCheckSwitch(CSTO_SPD this, bool bSel);

/**
  * @brief  It returns the result of the Bemf consistency check
  * @param  this related object of class CSTO_SPD
  * @retval bool Bemf consistency state
  */
bool STO_IsBemfConsistent(CSTO_SPD this);

/**
  * @brief  It re-initializes the state observer object with new constants
  * @param  this related object of class CSPD
  * @retval none
  */
void STO_ReInit(CSPD this, pSTOInitVars_t pSTOInitVars);

/**
  * @brief  It returns the result of the last variance check
  * @param  this related object of class CSTO_SPD
  * @retval bool Variance state
  */
bool STO_IsVarianceTight(CSTO_SPD this);

/**
  * @brief  It forces the state-observer to declare convergencyu
  * @param  this related object of class CSTO_SPD
  */
void STO_ForceConvergency1(CSTO_SPD this);

/**
  * @brief  It forces the state-observer to declare convergencyu
  * @param  this related object of class CSTO_SPD
  */
void STO_ForceConvergency2(CSTO_SPD this);

/**
  * @brief  Set the Absolute vaule of minimum mechanical speed (expressed in 
  *         01Hz) required to validate the start-up.
  * @param  this related object of class CSTO_SPD
  * @param  
  */
void STO_SetMinStartUpValidSpeed01HZ(CSTO_SPD this, uint16_t hMinStartUpValidSpeed);

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__STO_SPEEDNPOSFDBKCLASS_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
