/**
  ******************************************************************************
  * @file    STO_CORDIC_SpeednPosFdbkClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of STO_CORDIC class      
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
#ifndef __STO_CORDIC_SPEEDNPOSFDBKCLASS_H
#define __STO_CORDIC_SPEEDNPOSFDBKCLASS_H

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup SpeednPosFdbk_STO_CORDIC
  * @{
  */

/** @defgroup STO_CORDIC_class_exported_types STO_CORDIC class exported types
* @{
*/

/** 
  * @brief  Public STO_CORDIC class definition
  */
typedef struct CSTO_CR_SPD_t *CSTO_CR_SPD;

/** 
  * @brief  STO_CORDIC class parameters definition
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
   uint16_t hMinStartUpValidSpeed;     /*!< Minimum mechanical speed (expressed in 
                                          01Hz required to validate the start-up
                                          */
   uint8_t bStartUpConsistThreshold;   /*!< Number of consecutive tests on speed 
                                           consistency to be passed before 
                                           validating the start-up */
   uint8_t bReliability_hysteresys;    /*!< Number of reliability failed 
                                           consecutive tests before a speed 
                                           check fault is returned to base class 
                                           */
   int16_t hMaxInstantElAcceleration;   /*!< maximum instantaneous electrical
                                         acceleration (dpp per control period) */
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
}STO_CORDICParams_t, *pSTO_CORDICParams_t;

/** 
  * @brief STO_CR_SPD derived class input structure type definition for SPD_CalcAngle
  */
typedef struct
{ 
  Volt_Components  Valfa_beta;
  Curr_Components  Ialfa_beta;
  uint16_t         Vbus;
} STO_CR_Observer_Inputs_t;
/**
  * @}
  */

/** @defgroup STO_CORDIC_class_exported_methods STO_CORDIC class exported methods
  * @{
  */
  
/**
  * @brief  Creates an object of the class STO_CORDIC
  * @param  pSpeednPosFdbkParams pointer to an SpeednPosFdbk parameters structure
  * @param  pSTO_CORDICParams pointer to an STO_CORDIC parameters structure
  * @retval CSTO_CR_SPD new instance of STO_CORDIC object
  */
CSTO_CR_SPD STO_CR_NewObject(pSpeednPosFdbkParams_t pSpeednPosFdbkParams, pSTO_CORDICParams_t pSTO_CORDICParams);

/**
  * @brief  It internally performes a set of checks necessary to state whether
  *         the state observer algorithm converged. To be periodically called 
  *         during motor open-loop ramp-up (e.g. at the same frequency of 
  *         SPD_CalcElAngle), it returns TRUE if the estimated angle and speed 
  *         can be considered reliable, FALSE otherwise
  * @param  this related object of class CSTO_CR_SPD
  * @param  hForcedMecSpeed01Hz Mechanical speed in 0.1Hz unit as forced by VSS
  * @retval bool sensor reliability state
  */
bool STO_CR_IsObserverConverged(CSTO_CR_SPD this, int16_t hForcedMecSpeed01Hz);

/**
  * @brief  It exports estimated Bemf alpha-beta in Volt_Components format
  * @param  this related object of class CSTO_CR_SPD
  * @retval Volt_Components Bemf alpha-beta 
  */
Volt_Components STO_CR_GetEstimatedBemf(CSTO_CR_SPD this);

/**
  * @brief  It exports the stator current alpha-beta as estimated by state 
  *         observer
  * @param  this related object of class CSTO_CR_SPD
  * @retval Curr_Components State observer estimated stator current Ialpha-beta 
  */
Curr_Components STO_CR_GetEstimatedCurrent(CSTO_CR_SPD this);

/**
  * @brief  It exports current observer gains through parameters hC2 and hC4
  * @param  this related object of class CSTO_CR_SPD
  * @param  pC2 pointer to int16_t used to return parameters hC2
  * @param  pC4 pointer to int16_t used to return parameters hC4
  * @retval none 
  */
void STO_CR_GetObserverGains(CSTO_CR_SPD this, int16_t *pC2, int16_t *pC4);

/**
  * @brief  It allows setting new values for observer gains
  * @param  this related object of class CSTO_CR_SPD
  * @param  hC1 new value for observer gain hC1
  * @param  hC2 new value for observer gain hC2
  * @retval none 
  */
void STO_CR_SetObserverGains(CSTO_CR_SPD this, int16_t hC1, int16_t hC2);

/**
  * @brief  This method must be called - at least - with the same periodicity
  *         on which speed control is executed. It computes and update the 
  *         estimated average electrical speed - expressed in dpp - used for 
  *         instance in observer equations. 
  *         Average is computed considering a FIFO depth equal to 
  *         bSpeedBufferSizedpp. 
  * @param  this related object of class CSTO_CR_SPD
  * @retval none
  */
void STO_CR_CalcAvrgElSpeedDpp(CSTO_CR_SPD this);

/**
  * @brief  It exports estimated Bemf squared level
  * @param  this related object of class CSTO_CR_SPD
  * @retval int32_t 
  */
int32_t STO_CR_GetEstimatedBemfLevel(CSTO_CR_SPD this);

/**
  * @brief  It exports observed Bemf squared level
  * @param  this related object of class CSTO_CR_SPD
  * @retval int32_t 
  */
int32_t STO_CR_GetObservedBemfLevel(CSTO_CR_SPD this);

/**
  * @brief  It enables/disables the bemf consistency check
  * @param  this related object of class CSTO_CR_SPD
  * @param  bSel boolean; TRUE enables check; FALSE disables check
  */
void STO_CR_BemfConsistencyCheckSwitch(CSTO_CR_SPD this, bool bSel);

/**
  * @brief  It returns the result of the Bemf consistency check
  * @param  this related object of class CSTO_CR_SPD
  * @retval bool Bemf consistency state
  */
bool STO_CR_IsBemfConsistent(CSTO_CR_SPD this);

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__STO_CORDIC_SPEEDNPOSFDBKCLASS_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
