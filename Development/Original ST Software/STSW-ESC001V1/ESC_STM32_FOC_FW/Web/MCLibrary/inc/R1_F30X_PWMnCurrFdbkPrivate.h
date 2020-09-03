/**
  ******************************************************************************
  * @file    R1_F30X_PWMnCurrFdbkPrivate.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private definition of R1_F30X class      
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
#ifndef __R1_F30X_PWMNCURRFDBKPRIVATE_H
#define __R1_F30X_PWMNCURRFDBKPRIVATE_H

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup PWMnCurrFdbk_R1_F30X
  * @{
  */

/** @defgroup R1_F30X_Class_Private_Defines R1_F30X private defines
* @{
*/
/** 
* @brief  Flags definition
*/
#define STBD3 0x0002u /*!< Flag to indicate which phase has been distorted 
                           in boudary 3 zone (A or B)*/
#define DSTEN 0x0004u /*!< Flag to indicate if the distortion must be performed 
                           or not (in case of charge of bootstrap capacitor phase
                           is not required)*/

/**
* @}
*/

/** @defgroup R1_F30X_private_types R1_F30X private types
* @{
*/

/** 
  * @brief  R1_F30X class members definition 
  */
typedef struct
{
  uint16_t hDmaBuff[2];       /*!< Buffer used for PWM distortion points*/
  uint16_t hCntSmp1;          /*!< First sampling point express in timer counts*/
  uint16_t hCntSmp2;          /*!< Second sampling point express in timer counts*/
  uint8_t sampCur1;           /*!< Current sampled in the first sampling point*/
  uint8_t sampCur2;           /*!< Current sampled in the second sampling point*/
  int16_t hCurrAOld;          /*!< Previous measured value of phase A current*/
  int16_t hCurrBOld;          /*!< Previous measured value of phase B current*/
  uint8_t bInverted_pwm_new;  /*!< This value indicates the type of the current 
                                   PWM period (Regular, Distort PHA, PHB or PHC)*/
  ADC_TypeDef* ADCx;          /*!< ADC Pperipheral used for phase current sampling */
  TIM_TypeDef* TIMx;        /*!< Timer used for single shunt */
  uint16_t hFlags;            /*!< Flags 
                                   STBD3: Flag to indicate which phase has been distorted 
                                          in boudary 3 zone (A or B)
                                   DSTEN: Flag to indicate if the distortion must be 
                                          performed or not (charge of bootstrap 
                                          capacitor phase) */
  uint16_t hRegConv;          /*!< Temporary variables used to store regular conversions*/
  uint32_t wPhaseOffset;      /*!< Offset of Phase current sensing network  */
  volatile uint8_t  bIndex;   /*!< Number of conversions performed during the 
                                   calibration phase*/
  uint16_t Half_PWMPeriod;  /* Half PWM Period in timer clock counts */
  uint32_t wADC_JSQR;   /*!< Stores the value for JSQR register to select
                                 phase A motor current.*/
  uint32_t wPreloadDisableActing; /*!< Preload disable to be applied.*/
  uint32_t wPreloadDisableCC1; /*!< CCMR1 that disables the preload register
                                     of the channel to be distorted.*/
  uint32_t wPreloadDisableCC2; /*!< CCMR1 that disables the preload register
                                     of the channel to be distorted.*/  
  uint32_t wPreloadDisableCC3; /*!< CCMR2 that disables the preload register
                                     of the channel to be distorted.*/  
  DMA_Channel_TypeDef* PreloadDMAy_Chx;  /*!< DMA resource used for disabling the preload register*/
  DMA_Channel_TypeDef* DistortionDMAy_Chx; /*!< DMA resource used for doing the distortion*/
  bool OverCurrentFlag;     /*!< This flag is set when an overcurrent occurs.*/
  bool OverVoltageFlag;     /*!< This flag is set when an overvoltage occurs.*/
  bool BrakeActionLock;     /*!< This flag is set to avoid that brake action is interrupted.*/  
}DVars_t,*pDVars_t;

/** 
  * @brief  Redefinition of parameter structure
  */
typedef R1_F30XParams_t DParams_t, *pDParams_t; 

/** 
  * @brief Private R1_F30X class definition 
  */
typedef struct
{
	DVars_t DVars_str;			/*!< Derived class members container */
	pDParams_t pDParams_str;	/*!< Derived class parameters container */
}_DCR1F30X_PWMC_t, *_DCR1F30X_PWMC;
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__R1_F30X_PWMNCURRFDBKPRIVATE_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
