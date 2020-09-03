/**
  ******************************************************************************
  * @file    R3_LM1_PWMnCurrFdbkPrivate.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private definition of R3_LM1 class      
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
#ifndef __R3_LM1_PWMNCURRFDBKPRIVATE_H
#define __R3_LM1_PWMNCURRFDBKPRIVATE_H

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup PWMnCurrFdbk_R3_LM1
  * @{
  */

/** @defgroup R3_LM1_private_types R3_LM1 private types
* @{
*/

/** 
  * @brief  R3_LM1 class members definition 
  */
typedef struct
{
  uint16_t hPhaseAOffset;   /*!< Offset of Phase A current sensing network  */
  uint16_t hPhaseBOffset;   /*!< Offset of Phase B current sensing network  */
  uint16_t hPhaseCOffset;   /*!< Offset of Phase C current sensing network  */
  uint16_t Half_PWMPeriod;  /*!< Half PWM Period in timer clock counts */
  uint16_t hRegConv;        /*!< Variable used to store regular conversions 
                                 result*/
}DVars_t,*pDVars_t;

/** 
  * @brief  Redefinition of parameter structure
  */
typedef R3_LM1Params_t DParams_t, *pDParams_t; 

/** 
  * @brief Private R3_LM1 class definition 
  */
typedef struct
{
	DVars_t DVars_str;			/*!< Derived class members container */
	pDParams_t pDParams_str;	/*!< Derived class parameters container */
}_DCR3LM1_PWMC_t, *_DCR3LM1_PWMC;
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__R3_LM1_PWMNCURRFDBKPRIVATE_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
