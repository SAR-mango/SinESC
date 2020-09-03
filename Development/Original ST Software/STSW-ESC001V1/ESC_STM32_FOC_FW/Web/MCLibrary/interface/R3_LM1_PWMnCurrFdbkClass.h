/**
  ******************************************************************************
  * @file    R3_LM1_PWMnCurrFdbkClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of R3_LM1 class      
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
#ifndef __R3_LM1_PWMNCURRFDBKCLASS_H
#define __R3_LM1_PWMNCURRFDBKCLASS_H


#define GPIO_NoRemap_TIM1 ((uint32_t)(0))

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup PWMnCurrFdbk_R3_LM1
  * @{
  */

/** @defgroup R3_LM1_class_exported_types R3_LM1_class exported types
* @{
*/

/** 
  * @brief  Public R3_LM1 class definition
  */
typedef struct CR3LM1_PWMC_t *CR3LM1_PWMC;

/** 
  * @brief  R3_LM1 class parameters definition
  */
typedef const struct
{
  uint32_t wADC_Clock_Divider;          /*!<  APB2 clock prescaling factor for 
                                          ADC peripheral. It must be RCC_PCLK2_DivX
                                          x = 2, 4, 6, 8 */    
  uint8_t bTim_Clock_Divider;           /*!< APB2 clock prescaling factor for 
                                          TIM peripheral. It must be equal to 1, 
                                          2 or 4*/
/* Current reading A/D Conversions initialization -----------------------------*/ 
  uint8_t bIaChannel;                  /*!< ADC channel used for conversion of 
                                           current Ia. It must be equal to  
                                           ADC_Channel_x x= 0, ..., 15*/
  GPIO_TypeDef* hIaPort;                /*!< GPIO port used by hIaChannel. It must 
                                           be equal to GPIOx x= A, B, ...*/
  uint16_t hIaPin;                      /*!< GPIO pin used by hIaChannel. It must be 
                                          equal to GPIO_Pin_x x= 0, 1, ...*/ 
  uint8_t b_IaSamplingTime;            /*!< Sampling time used to convert hIaChannel. 
                                          It must be equal to ADC_SampleTime_xCycles5 
                                          x= 1, 7, ...*/ 
  uint8_t bIbChannel;                  /*!< ADC channel used for conversion of 
                                           current Ib. It must be equal to 
                                           ADC_Channel_x x= 0, ..., 15*/  
  GPIO_TypeDef* hIbPort;                /*!< GPIO port used by hIbChannel. It must 
                                           be equal to GPIOx x= A, B, ...*/
  uint16_t hIbPin;                      /*!< GPIO pin used by hIaChannel. It must be 
                                          equal to GPIO_Pin_x x= 0, 1, ...*/
  uint8_t b_IbSamplingTime;               /*!< Sampling time used to convert hIbChannel. 
                                          It must be equal to ADC_SampleTime_xCycles5 
                                          x= 1, 7, ...*/
  uint8_t bIcChannel;                  /*!< ADC channel used for conversion of 
                                           current Ia. It must be equal to  
                                           ADC_Channel_x x= 0, ..., 15*/
  GPIO_TypeDef* hIcPort;                /*!< GPIO port used by hIaChannel. It must 
                                           be equal to GPIOx x= A, B, ...*/
  uint16_t hIcPin;                      /*!< GPIO pin used by hIaChannel. It must be 
                                          equal to GPIO_Pin_x x= 0, 1, ...*/ 
  uint8_t b_IcSamplingTime;            /*!< Sampling time used to convert hIaChannel. 
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
                                            RepetitionCounter= (2* #PWM periods)-1*/
  uint16_t hTafter;                    /*!< It is the sum of dead time plus max 
                                            value between rise time and noise time
                                            express in number of TIM clocks.*/
  uint16_t hTbefore;                   /*!< It is the sampling time express in
                                            number of TIM clocks.*/ 
  TIM_TypeDef*  TIMx;                  /*!< It contains the pointer to the timer 
                                           used for PWM generation. */
/* PWM Driving signals initialization ----------------------------------------*/  

  uint32_t wTIM1Remapping;            /*!< Used only for instance #1, it 
                                           remaps TIM1 outputs. It must equal to 
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
/* PWM Driving signals initialization ----------------------------------------*/  
  FunctionalState EmergencyStop;        /*!< It enable/disable the management of 
                                            an emergency input instantaneously 
                                            stopping PWM generation. It must be 
                                            either equal to ENABLE or DISABLE */  
  uint16_t hBKINPolarity;               /*!< Emergency Stop (BKIN) input polarity, 
                                           it must be TIM_BreakPolarity_Low or 
                                           TIM_BreakPolarity_High */
  GPIO_TypeDef* hBKINPort;               /*!< Emergency Stop (BKIN) GPIO input 
                                           port (if used, after re-mapping). 
                                           It must be GPIOx x= A, B, ...*/
  uint16_t hBKINPin;                    /*!< Emergency Stop (BKIN) GPIO input pin 
                                          (if used, after re-mapping). It must be 
                                          GPIO_Pin_x x= 0, 1, ...*/     
}R3_LM1Params_t, *pR3_LM1Params_t;
/**
  * @}
  */

/** @defgroup R3_LM1_class_exported_methods R3_LM1 class exported 
  *           methods
  * @{
  */
  
/**
  * @brief  Creates an object of the class ICS_LM_1M
  * @param  pPWMnCurrFdbkParams pointer to an PWMnCurrFdbk parameters structure
  * @param  pR3_LM1Params pointer to an R3_LM1 parameters structure
  * @retval CR3LM1_PWMC new instance of R3_LM1 object
  */
CR3LM1_PWMC R3LM1_NewObject(pPWMnCurrFdbkParams_t pPWMnCurrFdbkParams, 
                                  pR3_LM1Params_t pR3_LM1Params);
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__R3_LM1_PWMNCURRFDBKCLASS_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
