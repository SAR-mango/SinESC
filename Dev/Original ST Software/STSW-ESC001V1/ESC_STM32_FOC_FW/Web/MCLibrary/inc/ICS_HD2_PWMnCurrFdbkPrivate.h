/**
  ******************************************************************************
  * @file    ICS_HD2_PWMnCurrFdbkPrivate.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private definition of ICS_HD2 class      
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
#ifndef __ICS_HD2_PWMNCURRFDBKPRIVATE_H
#define __ICS_HD2_PWMNCURRFDBKPRIVATE_H

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup PWMnCurrFdbk_ICS_HD2
  * @{
  */

/** @defgroup ICS_HD2_Class_Private_Defines ICS_HD2 private defines
* @{
*/
/** 
* @brief  Flags definition
*/
#define SOFOC 0x0008u /*!< This flag is reset at the beginning of FOC
                           and it is set in the TIM UP IRQ. If at the end of
                           FOC this flag is set, it means that FOC rate is too 
                           high and thus an error is generated */
/**
* @}
*/
/** @defgroup ICS_HD2_private_types ICS_HD2 private types
* @{
*/

/** 
  * @brief  ICS_HD2 class members definition 
  */
typedef struct
{
  uint32_t wPhaseAOffset;   /*!< Offset of Phase A current sensing network  */
  uint32_t wPhaseBOffset;   /*!< Offset of Phase B current sensing network  */
  uint16_t Half_PWMPeriod;     /* Half PWM Period in timer clock counts */  
  uint16_t hRegConv;          /*!< Variable used to store regular conversions 
                              result*/  
  volatile uint16_t hFlags;   /* Variable containing private flags */
  volatile uint8_t  bIndex;
  void *    pDrive;     /* Pointer to drive object related to PWMnCurr object. 
                          It is returned by the MC TIMx update IRQ handler */
}DVars_t,*pDVars_t;

/** 
  * @brief  Redefinition of parameter structure
  */
typedef ICS_DDParams_t DParams_t, *pDParams_t; 

/** 
  * @brief Private ICS_HD2 class definition 
  */
typedef struct
{
   DVars_t DVars_str;		/*!< Derived class members container */
   pDParams_t pDParams_str;	/*!< Derived class parameters container */      
}_DCIHD2_PWMC_t, *_DCIHD2_PWMC;
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__ICS_HD2_PWMNCURRFDBKPRIVATE_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
