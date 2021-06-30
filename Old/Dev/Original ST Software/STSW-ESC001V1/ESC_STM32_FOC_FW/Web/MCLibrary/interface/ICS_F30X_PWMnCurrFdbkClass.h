/**
  ******************************************************************************
  * @file    ICS_F30X_PWMnCurrFdbkClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of ICS_F30X class      
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
#ifndef __ICS_F30X_PWMNCURRFDBKCLASS_H
#define __ICS_F30X_PWMNCURRFDBKCLASS_H

#include "ICS_DD_PWMnCurrFdbkClass.h"

#define NONE		((uint8_t)(0x00))
#define EXT_MODE 	((uint8_t)(0x01))
#define INT_MODE 	((uint8_t)(0x02))

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup PWMnCurrFdbk_ICS_F30X
  * @{
  */

/** @defgroup ICS_F30X_class_exported_types ICS_F30X class exported types
* @{
*/

/** 
  * @brief  Public ICS_F30X class definition
  */
typedef struct CIF30X_PWMC_t *CIF30X_PWMC;

/** 
  * @brief  ICS_F30X class parameters definition
  */
typedef const struct
{
  uint32_t wADC_Clock_Divider;       /*!< APB2 clock prescaling factor for 
                                          ADC peripheral. It must be RCC_PCLK2_DivX
                                          x = 2, 4, 6, 8 */
  uint32_t wAHBPeriph;              /*!< AHB periph used. It must be
                                       RCC_AHBPeriph_ADC12 or
                                       RCC_AHBPeriph_ADC34. */
  uint8_t bTim_Clock_Divider;        /*!< APB2 clock prescaling factor for 
                                          TIM peripheral. It must be equal to 1, 
                                          2 or 4*/
/* Dual MC parameters --------------------------------------------------------*/ 
  uint8_t bInstanceNbr;              /*!< Instance number with reference to PWMC  
                                          base class. It is necessary to properly
                                          synchronize TIM8 with TIM1 at 
                                          peripheral initializations */ 
  uint16_t Tw;                       /*!< It is used for switching the context 
                                           in dual MC. It contains biggest delay
                                           (expressed in counter ticks) between 
                                           the counter crest and ADC latest 
                                           trigger  */
  uint8_t  bFreqRatio;               /*!< It is used in case of dual MC to 
                                           synchronize TIM1 and TIM8. It has 
                                           effect only on the second instanced 
                                           object and must be equal to the 
                                           ratio between the two PWM frequencies
                                           (higher/lower). Supported values are 
                                           1, 2 or 3 */  
  uint8_t  bIsHigherFreqTim;         /*!< When bFreqRatio is greather than 1
                                          this param is used to indicate if this 
                                          instance is the one with the highest 
                                          frequency. Allowed value are: HIGHER_FREQ
                                          or LOWER_FREQ */
  uint8_t  IRQnb;                    /*!< MC IRQ number used as TIMx Update 
                                           event */
/* Current reading A/D Conversions initialization -----------------------------*/ 
  uint8_t bIaChannel;                  /*!< ADC channel used for conversion of 
                                           current Ia. It must be equal to  
                                           ADC_Channel_x x= 0, ..., 15*/
  GPIO_TypeDef* hIaPort;                /*!< GPIO port used by bIaChannel. It must 
                                           be equal to GPIOx x= A, B, ...*/
  uint16_t hIaPin;                      /*!< GPIO pin used by bIaChannel. It must be 
                                          equal to GPIO_Pin_x x= 0, 1, ...*/ 
  uint8_t b_IaSamplingTime;            /*!< Sampling time used to convert bIaChannel. 
                                          It must be equal to ADC_SampleTime_xCycles5 
                                          x= 1, 7, ...*/ 
  uint8_t bIbChannel;                  /*!< ADC channel used for conversion of 
                                           current Ib. It must be equal to 
                                           ADC_Channel_x x= 0, ..., 15*/  
  GPIO_TypeDef* hIbPort;                /*!< GPIO port used by bIbChannel. It must 
                                           be equal to GPIOx x= A, B, ...*/
  uint16_t hIbPin;                      /*!< GPIO pin used by bIbChannel. It must be 
                                          equal to GPIO_Pin_x x= 0, 1, ...*/
  uint8_t b_IbSamplingTime;               /*!< Sampling time used to convert bIbChannel. 
                                          It must be equal to ADC_SampleTime_xCycles5 
                                          x= 1, 7, ...*/ 
  
/* PWM generation parameters --------------------------------------------------*/   
  uint16_t hDeadTime;                  /*!< Dead time in number of TIM clock
                                            cycles. If CHxN are enabled, it must
                                            contain the dead time to be generated
                                            by the microcontroller, otherwise it 
                                            expresses the maximum dead time 
                                            generated by driving network */
  uint8_t  bRepetitionCounter;         /*!< It expresses the number of PWM 
                                           periods to be elapsed before compare 
                                           registers are updated again. In 
                                           particular: 
                                           RepetitionCounter= (2* #PWM periods)-1
                                         */   
  TIM_TypeDef*  TIMx;                   /*!< It contains the pointer to the timer 
                                           used for PWM generation. It must 
                                           equal to TIM1 if bInstanceNbr is 
                                           equal to 1, to TIM8 otherwise */
/* PWM Driving signals initialization ----------------------------------------*/  

  uint32_t wTIM1Remapping;            /*!< Used only for TIM1, it 
                                           remaps its outputs. It must equal to 
                                           GPIO_PartialRemap_TIM1 or 
                                           GPIO_FullRemap_TIM1 or
                                           GPIO_NoRemap_TIM1 */
  uint16_t hCh1Polarity;              /*!< Channel 1 (high side) output polarity, 
                                           it must be TIM_OCPolarity_High or 
                                           TIM_OCPolarity_Low */
  GPIO_TypeDef* hCh1Port;               /*!< Channel 1 (high side) GPIO output 
                                         port (if used, after re-mapping). 
                                         It must be GPIOx x= A, B, ...*/
  uint16_t hCh1Pin;                    /*!< Channel 1 (high side) GPIO output pin 
                                          (if used, after re-mapping). It must be 
                                          GPIO_Pin_x x= 0, 1, ...*/  
  uint16_t hCh1IdleState;              /*!< Channel 1 (high side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCIdleState_Set or
                                            TIM_OCIdleState_Reset*/  
  
  
  uint16_t hCh2Polarity;               /*!< Channel 2 (high side) output polarity, 
                                           it must be TIM_OCPolarity_High or 
                                           TIM_OCPolarity_Low */ 
  GPIO_TypeDef* hCh2Port;               /*!< Channel 2 (high side) GPIO output 
                                           port (if used, after re-mapping). 
                                           It must be GPIOx x= A, B, ...*/
  uint16_t hCh2Pin;                    /*!< Channel 2 (high side) GPIO output pin 
                                          (if used, after re-mapping). It must be 
                                          GPIO_Pin_x x= 0, 1, ...*/
  uint16_t hCh2IdleState;              /*!< Channel 2 (high side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCIdleState_Set or
                                            TIM_OCIdleState_Reset*/    
 
  uint16_t hCh3Polarity;       /*!< Channel 3 (high side) output polarity, 
                                           it must be TIM_OCPolarity_High or 
                                           TIM_OCPolarity_Low */  
  GPIO_TypeDef* hCh3Port;               /*!< Channel 3 (high side) GPIO output 
                                           port (if used, after re-mapping). 
                                           It must be GPIOx x= A, B, ...*/
  uint16_t hCh3Pin;                    /*!< Channel 3 (high side) GPIO output pin 
                                          (if used, after re-mapping). It must be 
                                          GPIO_Pin_x x= 0, 1, ...*/  
  uint16_t hCh3IdleState;              /*!< Channel 3 (high side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCIdleState_Set or
                                            TIM_OCIdleState_Reset*/  
  
  LowSideOutputsFunction_t LowSideOutputs; /*!< Low side or enabling signals 
                                                generation method are defined 
                                                here.*/ 
  
  uint16_t hCh1NPolarity;               /*!< Channel 1N (low side) output polarity, 
                                           it must be TIM_OCNPolarity_High or 
                                           TIM_OCNPolarity_Low */
  GPIO_TypeDef* hCh1NPort;               /*!< Channel 1N (low side) GPIO output 
                                           port (if used, after re-mapping). 
                                           It must be GPIOx x= A, B, ...*/
  uint16_t hCh1NPin;                    /*!< Channel 1N (low side) GPIO output pin 
                                          (if used, after re-mapping). It must be 
                                          GPIO_Pin_x x= 0, 1, ...*/ 
  uint16_t hCh1NIdleState;              /*!< Channel 1N (low side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCNIdleState_Set or
                                            TIM_OCNIdleState_Reset*/ 

  
  uint16_t hCh2NPolarity;                 /*!< Channel 2N (low side) output polarity, 
                                           it must be TIM_OCNPolarity_High or 
                                           TIM_OCNPolarity_Low */
  GPIO_TypeDef* hCh2NPort;               /*!< Channel 2N (low side) GPIO output 
                                           port (if used, after re-mapping). 
                                           It must be GPIOx x= A, B, ...*/
  uint16_t hCh2NPin;                     /*!< Channel 2N (low side) GPIO output pin 
                                          (if used, after re-mapping). It must be 
                                          GPIO_Pin_x x= 0, 1, ...*/ 
  uint16_t hCh2NIdleState;              /*!< Channel 2N (low side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCNIdleState_Set or
                                            TIM_OCNIdleState_Reset*/ 
  
  uint16_t hCh3NPolarity;          /*!< Channel 3N (low side) output polarity, 
                                           it must be TIM_OCNPolarity_High or 
                                           TIM_OCNPolarity_Low */
  GPIO_TypeDef* hCh3NPort;               /*!< Channel 3N (low side)  GPIO output 
                                           port (if used, after re-mapping). 
                                           It must be GPIOx x= A, B, ...*/
  uint16_t hCh3NPin;                    /*!< Channel 3N (low side)  GPIO output pin 
                                          (if used, after re-mapping). It must be 
                                          GPIO_Pin_x x= 0, 1, ...*/   
  uint16_t hCh3NIdleState;              /*!< Channel 3N (low side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCNIdleState_Set or
                                            TIM_OCNIdleState_Reset*/ 
/* Emergency input signal initialization -------------------------------------*/  
  uint8_t bBKINMode;                    /*!< It defines the modality of emergency
                                             input. It must be any of the
                                             the following:
                                             NONE - feature disabled.
                                             INT_MODE - Internal comparator used
                                             as source of emergency event.
                                             EXT_MODE - External comparator used
                                             as source of emergency event.*/  
  uint16_t hBKINPolarity;               /*!< Emergency Stop (BKIN) input polarity, 
                                           it must be TIM_BreakPolarity_Low or 
                                           TIM_BreakPolarity_High */
  GPIO_TypeDef* hBKINPort;               /*!< Emergency Stop (BKIN) GPIO input 
                                           port (if used, after re-mapping). 
                                           It must be GPIOx x= A, B, ...*/
  uint16_t hBKINPin;                    /*!< Emergency Stop (BKIN) GPIO input pin 
                                          (if used, after re-mapping). It must be 
                                          GPIO_Pin_x x= 0, 1, ...*/
/* Alternate functions definition --------------------------------------------*/
  uint8_t bCh1AF;                       /*!< Channel 1 (high side) alternate 
                                           functions setting. It must be one
                                           of the GPIO_AF_x (x=0,1, ...) 
                                           according to the defined GPIO port 
                                           and pin.*/
  uint8_t bCh2AF;                       /*!< Channel 2 (high side) alternate 
                                           functions setting. It must be one
                                           of the GPIO_AF_x (x=0,1, ...) 
                                           according to the defined GPIO port 
                                           and pin.*/
  uint8_t bCh3AF;                       /*!< Channel 3 (high side) alternate 
                                           functions setting. It must be one
                                           of the GPIO_AF_x (x=0,1, ...) 
                                           according to the defined GPIO port 
                                           and pin.*/
  uint8_t bCh1NAF;                       /*!< Channel 1 (low side) alternate 
                                           functions setting. It must be one
                                           of the GPIO_AF_x (x=0,1, ...) 
                                           according to the defined GPIO port 
                                           and pin.*/
  uint8_t bCh2NAF;                       /*!< Channel 2 (low side) alternate 
                                           functions setting. It must be one
                                           of the GPIO_AF_x (x=0,1, ...) 
                                           according to the defined GPIO port 
                                           and pin.*/
  uint8_t bCh3NAF;                       /*!< Channel 3 (low side) alternate 
                                           functions setting. It must be one
                                           of the GPIO_AF_x (x=0,1, ...) 
                                           according to the defined GPIO port 
                                           and pin.*/
  uint8_t bBKINAF;                       /*!< Emergency Stop (BKIN) alternate 
                                           functions setting. It must be one
                                           of the GPIO_AF_x (x=0,1, ...) 
                                           according to the defined GPIO port 
                                           and pin.*/
  uint8_t bBKIN2AF;                      /*!< Emergency Stop (BKIN2) alternate 
                                           functions setting. It must be one
                                           of the GPIO_AF_x (x=0,1, ...) 
                                           according to the defined GPIO port 
                                           and pin.*/
/* Emergency input (BKIN2) signal initialization -----------------------------*/  
  uint8_t bBKIN2Mode;                 /*!< It defines the modality of emergency
                                           input 2. It must be any of the
                                           the following:
                                           NONE - feature disabled.
                                           INT_MODE - Internal comparator used
                                           as source of emergency event.
                                           EXT_MODE - External comparator used
                                           as source of emergency event.*/
  uint32_t hBKIN2Polarity;            /*!< Emergency Stop (BKIN2) input polarity, 
                                           it must be TIM_Break2Polarity_Low or 
                                           TIM_Break2Polarity_High */
  GPIO_TypeDef* hBKIN2Port;           /*!< Emergency Stop (BKIN2) GPIO input 
                                           port (if used, after re-mapping). 
                                           It must be GPIOx x= A, B, ...*/
  uint16_t hBKIN2Pin;                 /*!< Emergency Stop (BKIN2) GPIO input pin 
                                           (if used, after re-mapping). It must be 
                                           GPIO_Pin_x x= 0, 1, ...*/
/* Filtering settings of the emergency inputs --------------------------------*/
  uint8_t bBKINFilter;                  /*!< Emergency Stop (BKIN) digital 
                                             filter. See TIMx_BDTR (BKF bits) 
                                             definition in the reference 
                                             manual.*/
  uint8_t bBKIN2Filter;                 /*!< Emergency Stop (BKIN2) digital 
                                             filter. See TIMx_BDTR (BKF bits) 
                                             definition in the reference 
                                             manual.*/
} ICS_F30XParams_t, *pICS_F30XParams_t;

/**
  * @}
  */

/** @defgroup ICS_F30X_class_exported_methods ICS_F30X class exported 
  *           methods
  * @{
  */
  
/**
  * @brief  Creates an object of the class ICS_F30X
  * @param  pPWMnCurrFdbkParams pointer to an PWMnCurrFdbk parameters structure
  * @param  pICS_DDParams pointer to an ICS_DD parameters structure
  * @retval CIF30X_PWMC new instance of ICS_F30X object
  */
CIF30X_PWMC IF3XX_NewObject(pPWMnCurrFdbkParams_t pPWMnCurrFdbkParams, 
                                              pICS_F30XParams_t pICS_DDParams);

/**
  * @brief  It perform the start of all the timers required by the control. 
            It utilizes TIM2 as temporary timer to achieve synchronization between 
            PWM signals.
            When this function is called, TIM1 and/or TIM8 must be in frozen state
            with CNT, ARR, REP RATE and trigger correctly set (these setting are 
            usually performed in the Init method accordingly with the configuration)
  * @param  none
  * @retval none
  */
void IF3XX_StartTimers(void);

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__ICS_F30X_PWMNCURRFDBKCLASS_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
