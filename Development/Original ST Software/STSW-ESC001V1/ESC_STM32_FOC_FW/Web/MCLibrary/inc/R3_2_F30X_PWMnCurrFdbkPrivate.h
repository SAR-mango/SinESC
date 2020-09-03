/**
  ******************************************************************************
  * @file    R3_2_F30X_PWMnCurrFdbkPrivate.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private definition of current sensor class to be
  *          instantiated when the three shunts current sensing topology is 
  *          used.
  *          It is specifically designed for STM32F30x microcontrollers and
  *          implements the simultaneous dual sampling method using separate
  *          resource for each motor drives (OPAMPs and ADCs).      
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
#ifndef __R3_2_F30X_PWMNCURRFDBKPRIVATE_H
#define __R3_2_F30X_PWMNCURRFDBKPRIVATE_H

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup PWMnCurrFdbk_R3_F30X
  * @{
  */

/** @defgroup R3_2_F30X_private_types R3_2_F30X private types
* @{
*/

/** 
  * @brief  R3_2_F30X class members definition 
  */
typedef struct
{
  uint32_t wPhaseAOffset;   /*!< Offset of Phase A current sensing network  */
  uint32_t wPhaseBOffset;   /*!< Offset of Phase B current sensing network  */
  uint32_t wPhaseCOffset;   /*!< Offset of Phase C current sensing network  */
  uint32_t wOAMP1CR;        /*!< OAMP1 control register to select channel current 
                                 sampling */
  uint32_t wOAMP2CR;        /*!< OAMP2 control register to select channel current 
                                 sampling */
  uint16_t Half_PWMPeriod;  /*!< Half PWM Period in timer clock counts */  
  uint16_t hRegConv;        /*!< Variable used to store regular conversions 
                                 result*/
  volatile uint8_t bSoFOC;  /*!< This flag is reset at the beginning of FOC
                                 and it is set in the TIM UP IRQ. If at the end of
                                 FOC this flag is set, it means that FOC rate is too 
                                 high and thus an error is generated */
  uint32_t wADC1_JSQR;      /*!< Stores the value for JSQR register to select
                                 first acquisiton.*/
  uint32_t wADC2_JSQR;      /*!< Stores the value for JSQR register to select
                                 second acquisiton.*/
  uint32_t wADC_JSQR_phA;   /*!< Stores the value for JSQR register to select
                                 phase A motor current.*/
  uint32_t wADC_JSQR_phB;   /*!< Stores the value for JSQR register to select
                                 phase B motor current.*/
  uint32_t wADC_JSQR_phC;   /*!< Stores the value for JSQR register to select
                                 phase C motor current.*/
  volatile uint8_t  bIndex;
  uint16_t ADC_ExternalTriggerInjected;
                            /*!< Trigger selection for ADC peripheral.*/
  bool OverCurrentFlag;     /*!< This flag is set when an overcurrent occurs.*/
  bool OverVoltageFlag;     /*!< This flag is set when an overvoltage occurs.*/
  bool BrakeActionLock;     /*!< This flag is set to avoid that brake action is 
                                 interrupted.*/
}DVars_t,*pDVars_t;

/** 
  * @brief  Redefinition of parameter structure
  */
typedef R3_2_F30XParams_t DParams_t, *pDParams_t; 

/** 
  * @brief Private R3_2_F30X class definition 
  */
typedef struct
{
   DVars_t DVars_str;		/*!< Derived class members container */
   pDParams_t pDParams_str;	/*!< Derived class parameters container */      
}_DCR3_2_F30X_PWMC_t, *_DCR3_2_F30X_PWMC;
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__R3_2_F30X_PWMNCURRFDBKPRIVATE_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
