/**
  ******************************************************************************
  * @file    R1_F2XX_PWMnCurrFdbkPrivate.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private definition of R1_F2XX class      
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
#ifndef __R1_F2XX_PWMNCURRFDBKPRIVATE_H
#define __R1_F2XX_PWMNCURRFDBKPRIVATE_H

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup PWMnCurrFdbk_R1_F2XX
  * @{
  */

/** @defgroup R1_F2XX_Class_Private_Defines R1_F2XX private defines
* @{
*/
/** 
* @brief  Flags definition
*/
#define EOFOC 0x0001u /*!< Flag to indicate end of FOC duty available */
#define STBD3 0x0002u /*!< Flag to indicate which phase has been distorted 
                           in boudary 3 zone (A or B)*/
#define DSTEN 0x0004u /*!< Flag to indicate if the distortion must be performed 
                           or not (in case of charge of bootstrap capacitor phase
                           is not required)*/
#define SOFOC 0x0008u /*!< This flag will be reset to zero at the begin of FOC
                           and will be set in the UP IRQ. If at the end of
                           FOC it is set the software error must be generated*/

/**
* @}
*/

/** @defgroup R1_F2XX_private_types R1_F2XX private types
* @{
*/

/** 
  * @brief  R1_F2XX class members definition 
  */
typedef struct
{
  uint16_t hDmaBuff[2];       /*!< Buffer used for PWM distortion points*/
  uint32_t hCCDmaBuffCh4[4];  /*!< Buffer used for dual ADC sampling points*/
  uint16_t hCntSmp1;          /*!< First sampling point express in timer counts*/
  uint16_t hCntSmp2;          /*!< Second sampling point express in timer counts*/
  uint8_t sampCur1;           /*!< Current sampled in the first sampling point*/
  uint8_t sampCur2;           /*!< Current sampled in the second sampling point*/
  int16_t hCurrAOld;          /*!< Previous measured value of phase A current*/
  int16_t hCurrBOld;          /*!< Previous measured value of phase B current*/
  int16_t hCurrCOld;          /*!< Previous measured value of phase C current*/
  uint8_t bInverted_pwm;      /*!< This value indicates the type of the previous 
                                   PWM period (Regular, Distort PHA, PHB or PHC)*/
  uint8_t bInverted_pwm_new;  /*!< This value indicates the type of the current 
                                   PWM period (Regular, Distort PHA, PHB or PHC)*/
  uint16_t hPreloadCCMR2Set;  /*!< Preload value for TIMx->CCMR2 register used to
                                    set the mode of TIMx CH4*/
  ADC_TypeDef* ADCx;          /*!< ADC Pperipheral used for phase current sampling */
  TIM_TypeDef* TIMx_2;        /*!< Auxiliary timer used for single shunt */
  uint32_t hADCTrigger;       /*!< ADC trigger selection */
  uint8_t bDMATot;            /*!< Value to indicate the total number of expected 
                                   DMA TC events*/
  uint8_t bDMACur;            /*!< Current number of DMA TC events occurred */
  uint16_t hFlags;            /*!< Flags 
                                   EOFOC: Flag to indicate end of FOC duty available
                                   STBD3: Flag to indicate which phase has been distorted 
                                          in boudary 3 zone (A or B)
                                   DSTEN: Flag to indicate if the distortion must be 
                                          performed or not (charge of bootstrap 
                                          capacitor phase)
                                   SOFOC: This flag will be reset to zero at the begin of FOC
                                          and will be set in the UP IRQ. If at the end of
                                          FOC it is set the software error must be generated*/
  void* pDrive;               /*!< Pointer to drive object related to PWMnCurr object. 
                                   It is set in the init function and returned by the MC TIMx update 
                                   IRQ handler */
  uint32_t wPhaseOffset;      /*!< Offset of Phase current sensing network  */
  volatile uint8_t  bIndex;   /*!< Number of conversions performed during the 
                                   calibration phase*/
  __IO uint32_t* pTIMx_2_CCR; /*!< Pointer to the used CCR of the auxiliary timer */
}DVars_t,*pDVars_t;

/** 
  * @brief  Redefinition of parameter structure
  */
typedef R1_DDParams_t DParams_t, *pDParams_t; 

/** 
  * @brief Private R1_F2XX class definition 
  */
typedef struct
{
	DVars_t DVars_str;			/*!< Derived class members container */
	pDParams_t pDParams_str;	/*!< Derived class parameters container */
}_DCR1F2XX_PWMC_t, *_DCR1F2XX_PWMC;
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__R1_F2XX_PWMNCURRFDBKPRIVATE_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
