/**
  ******************************************************************************
  * @file    R3_F4XX_PWMnCurrFdbkPrivate.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private definition of R3_F4XX class      
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
#ifndef __R3_F4XX_PWMNCURRFDBKPRIVATE_H
#define __R3_F4XX_PWMNCURRFDBKPRIVATE_H

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup PWMnCurrFdbk_R3_F4XX
  * @{
  */

/** @defgroup R3_F4XX_private_types R3_F4XX private types
* @{
*/

/** 
  * @brief  R3_F4XX class members definition 
  */
typedef struct
{
  uint32_t wPhaseAOffset;   /*!< Offset of Phase A current sensing network  */
  uint32_t wPhaseBOffset;   /*!< Offset of Phase B current sensing network  */
  uint32_t wPhaseCOffset;   /*!< Offset of Phase C current sensing network  */
  uint32_t wADC1Channel;    /*!< ADC1 programmed channel for motor current 
                                 sampling */
  uint32_t wADC2Channel;    /*!< ADC2 programmed channel for motor current 
                                 sampling */
  uint16_t Half_PWMPeriod;  /* Half PWM Period in timer clock counts */  
  volatile uint8_t bSoFOC;  /*!< This flag is reset at the beginning of FOC
                                 and it is set in the TIM UP IRQ. If at the end of
                                 FOC this flag is set, it means that FOC rate is too 
                                 high and thus an error is generated */
  volatile uint8_t  bIndex;
  uint32_t wADCTriggerSet;  /*!< Store the value for ADC CR2 to proper configure
                                 current sampling during the context switching*/
  uint32_t wADCTriggerUnSet;/*!< Store the value for ADC CR2 to disable the 
                                 current sampling during the context switching*/
  uint32_t wTIMxCH4_BB_Addr;/*!< Store the bit-banding address to activate/deactivate 
                                 TIMx CH4 channel */
  void *   pDrive;          /*!< Pointer to drive object related to PWMnCurr object. 
                                 It is returned by the MC TIMx update IRQ handler */
}DVars_t,*pDVars_t;

/** 
  * @brief  Redefinition of parameter structure
  */
typedef R3_DDParams_t DParams_t, *pDParams_t; 

/** 
  * @brief Private R3_F4XX class definition 
  */
typedef struct
{
   DVars_t DVars_str;		/*!< Derived class members container */
   pDParams_t pDParams_str;	/*!< Derived class parameters container */      
}_DCR3F4XX_PWMC_t, *_DCR3F4XX_PWMC;
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__R3_F4XX_PWMNCURRFDBKPRIVATE_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
