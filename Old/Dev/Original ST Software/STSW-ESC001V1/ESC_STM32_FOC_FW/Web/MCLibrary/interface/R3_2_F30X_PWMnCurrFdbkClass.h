/**
  ******************************************************************************
  * @file    R3_2_F30X_PWMnCurrFdbkClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of current sensor class to be
  *          instantiated when the three shunts current sensing topology is 
  *          used.
  *          It is specifically designed for STM32F30x microcontrollers and
  *          implements the simultaneous dual sampling method using shared
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
#ifndef __R3_2_F30X_PWMNCURRFDBKCLASS_H
#define __R3_2_F30X_PWMNCURRFDBKCLASS_H

#define GPIO_NoRemap_TIM1 ((uint32_t)(0))
#define SHIFTED_TIMs      ((uint8_t) 1)
#define NO_SHIFTED_TIMs   ((uint8_t) 0)
#define HIGHER_FREQ        ((uint8_t) 1)
#define LOWER_FREQ         ((uint8_t) 2)

#define NONE		((uint8_t)(0x00))
#define EXT_MODE 	((uint8_t)(0x01))
#define INT_MODE 	((uint8_t)(0x02))

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup PWMnCurrFdbk_R3_2_F30X
  * @{
  */

/** @defgroup R3_2_F30X_class_exported_types R3_2_F30X class exported types
* @{
*/

/** 
  * @brief  Public R3_2_F30X class definition
  */
typedef struct CR3_2_F30X_PWMC_t *CR3_2_F30X_PWMC;

/** 
  * @brief  R3_2_F30X class parameters definition
  */
typedef const struct
{
/* First OPAMP settings ------------------------------------------------------*/
  uint8_t bOPAMP_InvertingInput_MODE; /*!< First OPAMP inverting input mode.
                                           It must be either equal to EXT_MODE
                                           or INT_MODE.*/
  uint32_t wOPAMP_InvertingInput;     /*!< First OPAMP inverting input pin.
                                           It must be one of the following:
                                           OPAMP1_InvertingInput_PC5 or
                                           OPAMP1_InvertingInput_PA3 if the 
                                           bOPAMP_InvertingInput_MODE is 
                                           EXT_MODE or
                                           OPAMP1_InvertingInput_PGA or
                                           OPAMP1_InvertingInput_FOLLOWER if the
                                           bOPAMP_InvertingInput_MODE is 
                                           INT_MODE.*/
  GPIO_TypeDef* hOPAMP_InvertingInput_GPIO_PORT; 
                                      /*!< First OPAMP inverting input GPIO port
                                           as defined in wOPAMP_InvertingInput.
                                           It must be GPIOx x= A, B, ... if 
                                           bOPAMP_InvertingInput_MODE is 
                                           EXT_MODE, otherwise can be dummy.*/
  uint16_t hOPAMP_InvertingInput_GPIO_PIN;
                                      /*!< First OPAMP inverting input GPIO pin
                                           as defined in wOPAMP_InvertingInput.
                                           It must be GPIO_Pin_x x= 0, 1, ... if 
                                           bOPAMP_InvertingInput_MODE is 
                                           EXT_MODE, otherwise can be dummy.*/
  uint32_t wOPAMP_NonInvertingInput_PHA; 
                                      /*!< First OPAMP non inverting input first
                                           selection.
                                           It must be one of the following:
                                           OPAMP1_NonInvertingInput_PA7,
                                           OPAMP1_NonInvertingInput_PA5,
                                           OPAMP1_NonInvertingInput_PA3,
                                           OPAMP1_NonInvertingInput_PA1.
					   Note: It must be the same PIN of 
                                           wOPAMP2_NonInvertingInput_PHA.*/
  GPIO_TypeDef* hOPAMP_NonInvertingInput_PHA_GPIO_PORT; 
                                      /*!< First OPAMP non inverting input GPIO 
                                           port as defined in 
                                           wOPAMP_NonInvertingInput_PHA. 
                                           It must be GPIOx x= A, B, ...*/
  uint16_t hOPAMP_NonInvertingInput_PHA_GPIO_PIN;
                                      /*!< First OPAMP non inverting input GPIO
                                           pin as defined in 
                                           wOPAMP_NonInvertingInput_PHA. 
                                           It must be GPIO_Pin_x x= 0, 1, ...*/
  uint32_t wOPAMP_NonInvertingInput_PHB; 
                                      /*!< First OPAMP non inverting input 2nd
                                           selection.
                                           It must be one of the following:
                                           OPAMP1_NonInvertingInput_PA7,
                                           OPAMP1_NonInvertingInput_PA5,
                                           OPAMP1_NonInvertingInput_PA3,
                                           OPAMP1_NonInvertingInput_PA1.*/
  GPIO_TypeDef* hOPAMP_NonInvertingInput_PHB_GPIO_PORT; 
                                      /*!< First OPAMP non inverting input GPIO 
                                           port as defined in 
                                           wOPAMP_NonInvertingInput_PHB. 
                                           It must be GPIOx x= A, B, ...*/
  uint16_t hOPAMP_NonInvertingInput_PHB_GPIO_PIN;
                                      /*!< First OPAMP non inverting input GPIO
                                           pin as defined in 
                                           wOPAMP_NonInvertingInput_PHB. 
                                           It must be GPIO_Pin_x x= 0, 1, ...*/
/* Second OPAMP settings -----------------------------------------------------*/
  uint8_t bOPAMP2_InvertingInput_MODE;/*!< Second OPAMP inverting input mode.
                                           It must be either equal to EXT_MODE
                                           or INT_MODE.*/
  uint32_t wOPAMP2_InvertingInput;    /*!< Second OPAMP inverting input pin.
                                           It must be one of the following:
                                           OPAMP3_InvertingInput_PB2 or
                                           OPAMP3_InvertingInput_PB10 if the 
                                           bOPAMP_InvertingInput_MODE is
                                           EXT_MODE or
                                           OPAMP3_InvertingInput_PGA or
                                           OPAMP3_InvertingInput_FOLLOWER if the
                                           bOPAMP_InvertingInput_MODE is 
                                           INT_MODE.*/
  GPIO_TypeDef* hOPAMP2_InvertingInput_GPIO_PORT; 
                                      /*!< 2nd OPAMP inverting input GPIO port
                                           as defined in wOPAMP2_InvertingInput.
                                           It must be GPIOx x= A, B, ...if 
                                           bOPAMP2_InvertingInput_MODE is 
                                           EXT_MODE, otherwise can be dummy.*/
  uint16_t hOPAMP2_InvertingInput_GPIO_PIN;
                                      /*!< 2nd OPAMP inverting input GPIO pin as 
                                           defined in wOPAMP2_InvertingInput.
                                           It must be GPIO_Pin_x x= 0, 1, ...if 
                                           bOPAMP2_InvertingInput_MODE is 
                                           EXT_MODE, otherwise can be dummy.*/
  uint32_t wOPAMP2_NonInvertingInput_PHA; /*!< 2nd OPAMP non inverting input 
                                           first selection.
                                           It must be one of the following:
                                           OPAMP3_NonInvertingInput_PB0,
                                           OPAMP3_NonInvertingInput_PB13,
                                           OPAMP3_NonInvertingInput_PA1,
                                           OPAMP3_NonInvertingInput_PA5.
                                           Note: It must be the same PIN of 
                                           wOPAMP_NonInvertingInput_PHA.*/
  GPIO_TypeDef* hOPAMP2_NonInvertingInput_PHA_GPIO_PORT; 
                                      /*!< Second OPAMP non inverting input GPIO
                                           port as defined in 
                                           wOPAMP2_NonInvertingInput_PHA. 
                                           It must be GPIOx x= A, B, ...*/
  uint16_t hOPAMP2_NonInvertingInput_PHA_GPIO_PIN;
                                      /*!< Second OPAMP non inverting input GPIO
                                           pin as defined in 
                                           wOPAMP2_NonInvertingInput_PHA. 
                                           It must be GPIO_Pin_x x= 0, 1, ...*/
  uint32_t wOPAMP2_NonInvertingInput_PHC; 
                                      /*!< Second OPAMP non inverting input 2nd
                                           selection.
					   It must be one of the following:
                                           OPAMP3_NonInvertingInput_PB0,
                                           OPAMP3_NonInvertingInput_PB13,
                                           OPAMP3_NonInvertingInput_PA1,
                                           OPAMP3_NonInvertingInput_PA5.*/
  GPIO_TypeDef* hOPAMP2_NonInvertingInput_PHC_GPIO_PORT; 
                                      /*!< Second OPAMP non inverting input GPIO
                                           port as defined in 
                                           wOPAMP2_NonInvertingInput_PHC. 
                                           It must be GPIOx x= A, B, ...*/
  uint16_t hOPAMP2_NonInvertingInput_PHC_GPIO_PIN;
                                      /*!< Second OPAMP non inverting input GPIO
                                           pin as defined in 
                                           wOPAMP2_NonInvertingInput_PHC. 
                                           It must be GPIO_Pin_x x= 0, 1, ...*/
/* Common settings -----------------------------------------------------------*/
  uint32_t wOPAMP_PGAGain;            /*!< It defines the OPAMP PGA gains.
	                                       It must be one of the following:
                                           OPAMP_OPAMP_PGAGain_2,
					   OPAMP_OPAMP_PGAGain_4,
					   OPAMP_OPAMP_PGAGain_8,
					   OPAMP_OPAMP_PGAGain_16.
					   This value is taken in account
					   just if wOPAMPx_InvertingInput is
        				   equal to OPAMPx_InvertingInput_PGA*/
  uint32_t OPAMP_PGAConnect;          /*!< It defines the OPAMP connection
					   with an external filter when PGA
					   is enabled.
					   It must be one of the following:
					   OPAMP_PGAConnect_No,
					   OPAMP_PGAConnect_IO1,
					   OPAMP_PGAConnect_IO2.
					   See reference manual RM0316.
					   This value is taken in account
					   just if wOPAMPx_InvertingInput is
					   equal to OPAMPx_InvertingInput_PGA*/
} R3_2_F30XOPAMPParams_t, *pR3_2_F30XOPAMPParams_t;

typedef const struct
{
  uint32_t wADC_Clock_Divider;    /*!< Select the clock of the ADC. The clock is
                                       common for both master and slave ADCs.
                                       It must be one of the following:
                                       ADC_Clock_SynClkModeDiv1,
                                       ADC_Clock_SynClkModeDiv2,
                                       ADC_Clock_SynClkModeDiv4.*/
  uint32_t wAHBPeriph;            /*!< AHB periph used. It must be
                                       RCC_AHBPeriph_ADC12 or
                                       RCC_AHBPeriph_ADC34 or
                                       RCC_AHBPeriph_ADC12 | RCC_AHBPeriph_ADC34
                                       . */
  uint8_t bTim_Clock_Divider;     /*!< APB2 clock prescaling factor for 
                                       TIM peripheral. It must be equal to 1, 
                                       2 or 4*/    
/* Dual MC parameters --------------------------------------------------------*/ 
  uint8_t bInstanceNbr;           /*!< Instance number with reference to PWMC  
                                       base class. It is necessary to properly
                                       synchronize TIM8 with TIM1 at peripheral
                                       initializations */ 
  uint16_t Tw;                    /*!< It is used for switching the context 
                                       in dual MC. It contains biggest delay
                                       (expressed in counter ticks) between 
                                       the counter crest and ADC latest trigger 
                                       */
                                           
  uint8_t  bFreqRatio;             /*!< It is used in case of dual MC to 
                                        synchronize TIM1 and TIM8. It has 
                                        effect only on the second instanced 
                                        object and must be equal to the 
                                        ratio between the two PWM frequencies
                                        (higher/lower). Supported values are 
                                        1, 2 or 3 */  
  uint8_t  bIsHigherFreqTim;       /*!< When bFreqRatio is greather than 1
                                        this param is used to indicate if this 
                                        instance is the one with the highest 
                                        frequency. Allowed value are: HIGHER_FREQ
                                        or LOWER_FREQ */
  uint8_t  IRQnb;                  /*!< MC IRQ number used as TIMx Update 
                                        event */
/* Current reading A/D Conversions initialization -----------------------------*/ 
  ADC_TypeDef* ADCx_1;             /*!< First ADC peripheral to be used.*/
  ADC_TypeDef* ADCx_2;             /*!< Second ADC peripheral to be used.*/
  uint8_t bIaChannel;              /*!< ADC channel used for conversion of 
                                        current Ia. It must be equal to  
                                        ADC_Channel_x x= 0, ..., 15*/
  GPIO_TypeDef* hIaPort;           /*!< GPIO port used by bIaChannel. It must 
                                        be equal to GPIOx x= A, B, ...*/
  uint16_t hIaPin;                 /*!< GPIO pin used by bIaChannel. It must be 
                                        equal to GPIO_Pin_x x= 0, 1, ...*/ 
  uint8_t b_IaSamplingTime;        /*!< Sampling time used to convert bIaChannel. 
                                        It must be equal to ADC_SampleTime_xCycles5 
                                        x= 1, 7, ...*/ 
  uint8_t bIbChannel;              /*!< ADC channel used for conversion of 
                                        current Ib. It must be equal to 
                                        ADC_Channel_x x= 0, ..., 15*/  
  GPIO_TypeDef* hIbPort;           /*!< GPIO port used by bIbChannel. It must 
                                        be equal to GPIOx x= A, B, ...*/
  uint16_t hIbPin;                 /*!< GPIO pin used by bIbChannel. It must be 
                                        equal to GPIO_Pin_x x= 0, 1, ...*/
  uint8_t b_IbSamplingTime;        /*!< Sampling time used to convert bIbChannel. 
                                        It must be equal to ADC_SampleTime_xCycles5 
                                        x= 1, 7, ...*/
  uint8_t bIcChannel;              /*!< ADC channel used for conversion of 
                                        current Ia. It must be equal to  
                                        ADC_Channel_x x= 0, ..., 15*/
  GPIO_TypeDef* hIcPort;           /*!< GPIO port used by hIaChannel. It must 
                                        be equal to GPIOx x= A, B, ...*/
  uint16_t hIcPin;                 /*!< GPIO pin used by hIaChannel. It must be 
                                        equal to GPIO_Pin_x x= 0, 1, ...*/ 
  uint8_t b_IcSamplingTime;        /*!< Sampling time used to convert hIaChannel. 
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
  TIM_TypeDef*  TIMx;                   /*!< It contains the pointer to the timer 
                                            used for PWM generation. It must 
                                            equal to TIM1 if bInstanceNbr is 
                                            equal to 1, to TIM8 otherwise */
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
/* Emergency input signal initialization -------------------------------------*/  
  uint8_t bBKINMode;                    /*!< It defines the modality of emergency
                                             input. It must be any of the
                                             the following:
                                             NONE - feature disabled.
                                             INT_MODE - Internal comparator used
                                             as source of emergency event.
                                             EXT_MODE - External comparator used
                                             as source of emergency event.*/
  uint32_t hBKINPolarity;               /*!< Emergency Stop (BKIN) input polarity, 
                                             it must be TIM_Break1Polarity_Low or 
                                             TIM_Break1Polarity_High */
  GPIO_TypeDef* hBKINPort;              /*!< Emergency Stop (BKIN) GPIO input 
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
/* Internal OPAMP common settings --------------------------------------------*/
  pR3_2_F30XOPAMPParams_t pOPAMPParams;  /*!< Pointer to the OPAMP params struct.
                                           It must be MC_NULL if internal
					   OPAMP are not used.*/
/* Internal COMP settings ----------------------------------------------------*/
  pF30XCOMPParams_t pOCPA_COMPParams;
                                      /*!< Pointer to the COMP params struct 
                                           used for overcurrent protection of
                                           phase A.
                                           It must be MC_NULL if internal COMP
					   are not used for OCP protection.*/
  pF30XCOMPParams_t pOCPB_COMPParams;
                                      /*!< Pointer to the COMP params struct 
                                           used for overcurrent protection of
                                           phase B.
                                           It must be MC_NULL if internal COMP
					   are not used for OCP protection.*/
  pF30XCOMPParams_t pOCPC_COMPParams;
                                      /*!< Pointer to the COMP params struct 
                                           used for overcurrent protection of
                                           phase C.
                                           It must be MC_NULL if internal COMP
					   are not used for OCP protection.*/
  pF30XCOMPParams_t pOVP_COMPParams;
                                      /*!< Pointer to the COMP params struct 
                                           used for overvoltage protection.
                                           It must be MC_NULL if internal COMP
					   are not used for OVP protection.*/
/* DAC settings --------------------------------------------------------------*/
  uint16_t hDAC_OCP_Threshold;        /*!< Value of analog reference expressed 
                                           as 16bit unsigned integer.
                                           Ex. 0 = 0V 65536 = VDD_DAC.*/
  uint16_t hDAC_OVP_Threshold;        /*!< Value of analog reference expressed 
                                           as 16bit unsigned integer.
                                           Ex. 0 = 0V 65536 = VDD_DAC.*/
/* Regular conversion --------------------------------------------------------*/
  ADC_TypeDef * regconvADCx;          /*!< ADC peripheral used for regular 
                                           conversion.*/
} R3_2_F30XParams_t, *pR3_2_F30XParams_t;

/**
  * @}
  */

/** @defgroup R3_2_F30X_class_exported_methods R3_2_F30X class exported 
  *           methods
  * @{
  */
  
/**
  * @brief  Creates an object of the class R3_2_F30X
  * @param  pPWMnCurrFdbkParams pointer to an PWMnCurrFdbk parameters structure
  * @param  pR3_F30XParams pointer to an pR3_2_F30XParams_t parameters structure
  * @retval CR3_2_F30X_PWMC new instance of R3_2_F30X object
  */
CR3_2_F30X_PWMC R3_2_F3XX_NewObject(pPWMnCurrFdbkParams_t pPWMnCurrFdbkParams, 
                                              pR3_2_F30XParams_t pR3_2_F30XParams);

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
void R3_2_F3XX_StartTimers(void);

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__R3_2_F30X_PWMNCURRFDBKCLASS_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
