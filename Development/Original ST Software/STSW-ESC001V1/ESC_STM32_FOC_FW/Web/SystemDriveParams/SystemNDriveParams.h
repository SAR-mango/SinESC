/**
  ******************************************************************************
  * @file    SystemNDriveParams.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains System & Drive constant initialization
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
#ifndef __SYSTEMNDRIVE_PARAMS_H
#define __SYSTEMNDRIVE_PARAMS_H

#include "Parameters conversion.h"

#ifdef SINGLEDRIVE
  #define MAX_TWAIT 0                 /* Dummy value for single drive */
  #define FREQ_RATIO 1                /* Dummy value for single drive */
  #define FREQ_RELATION HIGHEST_FREQ  /* Dummy value for single drive */
#endif

#ifdef DUALDRIVE

#include "Parameters conversion motor 2.h"

/**
  * @brief  PI / PID Speed loop parameters Motor 2
  */
PIParams_t PISpeedParamsM2 =
{
  .hDefKpGain          = (int16_t)PID_SPEED_KP_DEFAULT2,   /*!< Default Kp gain, used to initialize Kp gain
                                                                software variable*/
  .hDefKiGain          = (int16_t)PID_SPEED_KI_DEFAULT2,   /*!< Default Ki gain, used to initialize Ki gain
                                                                software variable*/
  .hKpDivisor          = (uint16_t)SP_KPDIV2,              /*!< Kp gain divisor, used in conjuction with
                                                                Kp gain allows obtaining fractional values.
                                                                If FULL_MISRA_C_COMPLIANCY is not defined
                                                                the divisor is implemented through
                                                                algebrical right shifts to speed up PI
                                                                execution. Only in this case this parameter
                                                                specifies the number of right shifts to be
                                                                executed */
  .hKiDivisor          = (uint16_t)SP_KIDIV2,              /*!< Ki gain divisor, used in conjuction with
                                                                Ki gain allows obtaining fractional values.
                                                                If FULL_MISRA_C_COMPLIANCY is not defined
                                                                the divisor is implemented through
                                                                algebrical right shifts to speed up PI
                                                                execution. Only in this case this parameter
                                                                specifies the number of right shifts to be
                                                                executed */
  .wDefMaxIntegralTerm = (int32_t)IQMAX2 * (int32_t)SP_KIDIV2,/*!< Upper limit used to saturate the integral
                                                                   term given by Ki / Ki divisor * integral of
                                                                   process variable error */
  .wDefMinIntegralTerm = -(int32_t)IQMAX2 * (int32_t)SP_KIDIV2,/*!< Lower limit used to saturate the integral
                                                                    term given by Ki / Ki divisor * integral of
                                                                    process variable error */
  .hDefMaxOutput       = (int16_t)IQMAX2,                 /*!< Upper limit used to saturate the PI output
                                                           */
  .hDefMinOutput       = -(int16_t)IQMAX2,                /*!< Lower limit used to saturate the PI output
                                                           */
  .hKpDivisorPOW2      = (uint16_t)SP_KPDIV_LOG2,         /*!< Kp gain divisor expressed as power of 2.
                                                               E.g. if gain divisor is 512 the value
                                                               must be 9 because 2^9 = 512 */
  .hKiDivisorPOW2      =( uint16_t)SP_KIDIV_LOG2         /*!< Ki gain divisor expressed as power of 2.
                                                              E.g. if gain divisor is 512 the value
                                                              must be 9 because 2^9 = 512 */
};

/**
  * @brief  PI / PID Iq loop parameter Motor 2
  */
PIParams_t PIIqParamsM2 =
{
  .hDefKpGain          = (int16_t)PID_TORQUE_KP_DEFAULT2, /*!< Default Kp gain, used to initialize Kp gain
                                                               software variable*/
  .hDefKiGain          = (int16_t)PID_TORQUE_KI_DEFAULT2, /*!< Default Ki gain, used to initialize Ki gain
                                                               software variable*/
  .hKpDivisor          = (uint16_t)TF_KPDIV2,               /*!< Kp gain divisor, used in conjuction with
                                                                 Kp gain allows obtaining fractional values.
                                                                 If FULL_MISRA_C_COMPLIANCY is not defined
                                                                 the divisor is implemented through
                                                                 algebrical right shifts to speed up PI
                                                                 execution. Only in this case this parameter
                                                                 specifies the number of right shifts to be
                                                                 executed */
  .hKiDivisor          = (uint16_t)TF_KIDIV2,               /*!< Ki gain divisor, used in conjuction with
                                                                 Ki gain allows obtaining fractional values.
                                                                 If FULL_MISRA_C_COMPLIANCY is not defined
                                                                 the divisor is implemented through
                                                                 algebrical right shifts to speed up PI
                                                                 execution. Only in this case this parameter
                                                                 specifies the number of right shifts to be
                                                                 executed */
  .wDefMaxIntegralTerm = (int32_t)S16_MAX * TF_KIDIV2,     /*!< Upper limit used to saturate the integral
                                                                term given by Ki / Ki divisor * integral of
                                                                process variable error */
  .wDefMinIntegralTerm = (int32_t)S16_MIN * TF_KIDIV2,             /*!< Lower limit used to saturate the integral
                                                                        term given by Ki / Ki divisor * integral of
                                                                        process variable error */
  .hDefMaxOutput       = S16_MAX,                        /*!< Upper limit used to saturate the PI output
                                                         */
  .hDefMinOutput       = -S16_MAX,                       /*!< Lower limit used to saturate the PI output
                                                         */
  .hKpDivisorPOW2      = (uint16_t)TF_KPDIV_LOG2,         /*!< Kp gain divisor expressed as power of 2.
                                                               E.g. if gain divisor is 512 the value
                                                               must be 9 because 2^9 = 512 */
  .hKiDivisorPOW2      = (uint16_t)TF_KIDIV_LOG2          /*!< Ki gain divisor expressed as power of 2.
                                                               E.g. if gain divisor is 512 the value
                                                               must be 9 because 2^9 = 512 */
};

/**
  * @brief  PI / PID Id loop parameter Motor 2
  */
PIParams_t PIIdParamsM2 =
{
  .hDefKpGain          = (int16_t)PID_FLUX_KP_DEFAULT2,    /*!< Default Kp gain, used to initialize Kp gain
                                                                software variable*/
  .hDefKiGain          = (int16_t)PID_FLUX_KI_DEFAULT2,    /*!< Default Ki gain, used to initialize Ki gain
                                                                software variable*/
  .hKpDivisor          = (uint16_t)TF_KPDIV2,              /*!< Kp gain divisor, used in conjuction with
                                                                Kp gain allows obtaining fractional values.
                                                                If FULL_MISRA_C_COMPLIANCY is not defined
                                                                the divisor is implemented through
                                                                algebrical right shifts to speed up PI
                                                                execution. Only in this case this parameter
                                                                specifies the number of right shifts to be
                                                                executed */
  .hKiDivisor          = (uint16_t)TF_KIDIV2,              /*!< Ki gain divisor, used in conjuction with
                                                                Ki gain allows obtaining fractional values.
                                                                If FULL_MISRA_C_COMPLIANCY is not defined
                                                                the divisor is implemented through
                                                                algebrical right shifts to speed up PI
                                                                execution. Only in this case this parameter
                                                                specifies the number of right shifts to be
                                                                executed */
  .wDefMaxIntegralTerm = (int32_t)S16_MAX * TF_KIDIV2,             /*!< Upper limit used to saturate the integral
                                                                        term given by Ki / Ki divisor * integral of
                                                                        process variable error */
  .wDefMinIntegralTerm = (int32_t)S16_MIN * TF_KIDIV2,             /*!< Lower limit used to saturate the integral
                                                                        term given by Ki / Ki divisor * integral of
                                                                        process variable error */
  .hDefMaxOutput       = S16_MAX,                       /*!< Upper limit used to saturate the PI output
                                                         */
  .hDefMinOutput       = -S16_MAX,                      /*!< Lower limit used to saturate the PI output
                                                         */
  .hKpDivisorPOW2      = (uint16_t)TF_KPDIV_LOG2,       /*!< Kp gain divisor expressed as power of 2.
                                                             E.g. if gain divisor is 512 the value
                                                             must be 9 because 2^9 = 512 */
  .hKiDivisorPOW2      = (uint16_t)TF_KIDIV_LOG2        /*!< Ki gain divisor expressed as power of 2.
                                                             E.g. if gain divisor is 512 the value
                                                             must be 9 because 2^9 = 512 */
};

/**
  * @brief  PI Flux Weakening control parameters Motor 2
  */
PIParams_t PIFluxWeakeningParamsM2 =
{
  .hDefKpGain          = (int16_t)FW_KP_GAIN2,             /*!< Default Kp gain, used to initialize Kp gain
                                                                software variable*/
  .hDefKiGain          = (int16_t)FW_KI_GAIN2,             /*!< Default Ki gain, used to initialize Ki gain
                                                                software variable*/
  .hKpDivisor          = (uint16_t)FW_KPDIV2,              /*!< Kp gain divisor, used in conjuction with
                                                                Kp gain allows obtaining fractional values.
                                                                If FULL_MISRA_C_COMPLIANCY is not defined
                                                                the divisor is implemented through
                                                                algebrical right shifts to speed up PI
                                                                execution. Only in this case this parameter
                                                                specifies the number of right shifts to be
                                                                executed */
  .hKiDivisor          = (uint16_t)FW_KIDIV2,              /*!< Ki gain divisor, used in conjuction with
                                                                Ki gain allows obtaining fractional values.
                                                                If FULL_MISRA_C_COMPLIANCY is not defined
                                                                the divisor is implemented through
                                                                algebrical right shifts to speed up PI
                                                                execution. Only in this case this parameter
                                                                specifies the number of right shifts to be
                                                                executed */
  .wDefMaxIntegralTerm = 0,                              /*!< Upper limit used to saturate the integral
                                                              term given by Ki / Ki divisor * integral of
                                                              process variable error */
  .wDefMinIntegralTerm = (int32_t)(-NOMINAL_CURRENT2) * (int32_t)FW_KIDIV2, /*!< Lower limit used to saturate the integral
                                                                                 term given by Ki / Ki divisor * integral of
                                                                                 process variable error */
  .hDefMaxOutput       = 0,                            /*!< Upper limit used to saturate the PI output
                                                        */
  .hDefMinOutput       = -S16_MAX,                     /*!< Lower limit used to saturate the PI output
                                                        */
  .hKpDivisorPOW2      = (uint16_t)FW_KPDIV_LOG2,       /*!< Kp gain divisor expressed as power of 2.
                                                             E.g. if gain divisor is 512 the value
                                                             must be 9 because 2^9 = 512 */
  .hKiDivisorPOW2      =(uint16_t)FW_KIDIV_LOG2        /*!< Ki gain divisor expressed as power of 2.
                                                            E.g. if gain divisor is 512 the value
                                                            must be 9 because 2^9 = 512 */
};

/**
  * @brief  SpeednTorque Controller parameters Motor 2
  */
SpeednTorqCtrlParams_t SpeednTorqCtrlParamsM2 =
{
  .hSTCFrequencyHz             = MEDIUM_FREQUENCY_TASK_RATE2,              /*!< Frequency on which the user updates
                                                                                the torque reference calling
                                                                                STC_CalcTorqueReference method
                                                                                expressed in Hz */
  .hMaxAppPositiveMecSpeed01Hz = (uint16_t)(MAX_APPLICATION_SPEED2/6),      /*!< Application maximum positive value
                                                                                 of rotor speed. It's expressed in
                                                                                 tenth of mechanical Hertz.*/
  .hMinAppPositiveMecSpeed01Hz = (uint16_t)(MIN_APPLICATION_SPEED2/6),      /*!< Application minimum positive value
                                                                                 of rotor speed. It's expressed in
                                                                                 tenth of mechanical Hertz.*/
  .hMaxAppNegativeMecSpeed01Hz = (int16_t)(-MIN_APPLICATION_SPEED2/6),      /*!< Application maximum negative value
                                                                                 of rotor speed. It's expressed in
                                                                                 tenth of mechanical Hertz.*/
  .hMinAppNegativeMecSpeed01Hz = (int16_t)(-MAX_APPLICATION_SPEED2/6),      /*!< Application minimum negative value
                                                                                 of rotor speed. It's expressed in
                                                                                 tenth of mechanical Hertz.*/
  .hMaxPositiveTorque          = (int16_t)NOMINAL_CURRENT2,                 /*!< Maximum positive value of motor
                                                                                 torque. This value represents
                                                                                 actually the maximum Iq current
                                                                                 expressed in digit.*/
  .hMinNegativeTorque          = -(int16_t)NOMINAL_CURRENT2,                /*!< Maximum negative value of motor
                                                                                 torque. This value represents
                                                                                 actually the maximum Iq current
                                                                                 expressed in digit.*/
  .bModeDefault                = DEFAULT_CONTROL_MODE2,                     /*!< Default STC modality.*/
  .hMecSpeedRef01HzDefault     = (int16_t)(DEFAULT_TARGET_SPEED_RPM2/6),    /*!< Default mechanical rotor speed
                                                                                 reference expressed in tenths of
                                                                                 HZ.*/
  .hTorqueRefDefault           = (int16_t)DEFAULT_TORQUE_COMPONENT2,        /*!< Default motor torque reference.
                                                                                 This value represents actually the
                                                                                 Iq current reference expressed in
                                                                                 digit.*/
  .hIdrefDefault               = (int16_t)DEFAULT_FLUX_COMPONENT2,          /*!< Default motor torque reference.
                                                                                 This value represents actually the
                                                                                 Iq current reference expressed in
                                                                                 digit.*/
};

/**
  * @brief  Current sensor parameters Motor 2 - base class
  */
PWMnCurrFdbkParams_t PWMnCurrFdbkParamsM2 =
{
  .hPWMperiod          = PWM_PERIOD_CYCLES2,              /*!< It contains the PWM period expressed
                                                               in timer clock cycles unit:
                                                               hPWMPeriod = Timer Fclk / Fpwm    */
  .hOffCalibrWaitTicks = OFFCALIBRWAITTICKS2,             /*!< Wait time duration before current reading
                                                               calibration expressed in number of calls
                                                               of PWMC_CurrentReadingCalibr with action
                                                               CRC_EXEC */
  .hDTCompCnt          = DTCOMPCNT2,                      /*!< It contains half of Dead time expressed
                                                               in timer clock cycles unit:
                                                               hDTCompCnt = (DT(s) * Timer Fclk)/2 */
  .Ton                 = TON2,                            /*!< Reserved */
  .Toff                = TOFF2                            /*!< Reserved */
};

/**
  * @brief  Current sensor parameters Dual Drive Motor 2 - ICS
  */
ICS_DDParams_t ICS_DDParamsM2 = {
  .wADC_Clock_Divider = ADC_CLOCK_DIVIDER2,                /*!< APB2 clock prescaling factor for
                                                                ADC peripheral. It must be RCC_PCLK2_DivX
                                                                x = 2, 4, 6, 8 */
  .bTim_Clock_Divider = TIM_CLOCK_DIVIDER2,                /* System clock divider for Advanded Timer */
/* Dual MC parameters --------------------------------------------------------*/
  .bInstanceNbr     = 2,                        /*!< Instance number with reference to PWMC
                                                     base class. It is necessary to properly
                                                     synchronize TIM8 with TIM1 at
                                                     peripheral initializations */
  .Tw               = MAX_TWAIT2,               /*!< It is used for switching the context
                                                     in dual MC. It contains biggest delay
                                                     (expressed in counter ticks) between
                                                     the counter crest and ADC latest
                                                     trigger  */
  .bFreqRatio       = FREQ_RATIO,               /*!< It is used in case of dual MC to
                                                     synchronize TIM1 and TIM8. It has
                                                     effect only on the second instanced
                                                     object and must be equal to the
                                                     ratio between the two PWM frequencies
                                                     (higher/lower). Supported values are
                                                     1, 2 or 3 */
  .bIsHigherFreqTim = FREQ_RELATION2,           /*!< When bFreqRatio is greather than 1
                                                     this param is used to indicate if this
                                                     instance is the one with the highest
                                                     frequency. Allowed value are: HIGHER_FREQ
                                                     or LOWER_FREQ */
  .IRQnb            = MC_IRQ_PWMNCURRFDBK_2,    /*!< MC IRQ number used as TIMx Update
                                                     event */
/* Current reading A/D Conversions initialization -----------------------------*/
  .bIaChannel       = PHASE_U_CURR_CHANNEL2,            /*!< ADC channel used for conversion of
                                                             current Ia. It must be equal to
                                                             ADC_Channel_x x= 0, ..., 15*/
  .hIaPort          = PHASE_U_GPIO_PORT2,               /*!< GPIO port used by hIaChannel. It must
                                                             be equal to GPIOx x= A, B, ...*/
  .hIaPin           = PHASE_U_GPIO_PIN2,                /*!< GPIO pin used by hIaChannel. It must be
                                                             equal to GPIO_Pin_x x= 0, 1, ...*/
  .b_IaSamplingTime = SAMPLING_TIME_SEL2,               /*!< Sampling time used to convert hIaChannel.
                                                             It must be equal to ADC_SampleTime_xCycles5
                                                             x= 1, 7, ...*/
  .bIbChannel       = PHASE_V_CURR_CHANNEL2,            /*!< ADC channel used for conversion of
                                                             current Ib. It must be equal to
                                                             ADC_Channel_x x= 0, ..., 15*/
  .hIbPort          = PHASE_V_GPIO_PORT2,               /*!< GPIO port used by hIbChannel. It must
                                                             be equal to GPIOx x= A, B, ...*/
  .hIbPin           = PHASE_V_GPIO_PIN2,                /*!< GPIO pin used by hIaChannel. It must be
                                                             equal to GPIO_Pin_x x= 0, 1, ...*/
  .b_IbSamplingTime = SAMPLING_TIME_SEL2,               /*!< Sampling time used to convert hIbChannel.
                                                             It must be equal to ADC_SampleTime_xCycles5
                                                             x= 1, 7, ...*/
/* PWM generation parameters --------------------------------------------------*/
  .hDeadTime          = DEAD_TIME_COUNTS2,              /*!< Dead time in number of TIM clock
                                                             cycles. If CHxN are enabled, it must
                                                             contain the dead time to be generated
                                                             by the microcontroller, otherwise it
                                                             expresses the maximum dead time
                                                             generated by driving network */
  .bRepetitionCounter = REP_COUNTER2,                   /*!< It expresses the number of PWM
                                                             periods to be elapsed before compare
                                                             registers are updated again. In
                                                             particular:
                                                             RepetitionCounter= (2* #PWM periods)-1*/
  .TIMx               = PWM_TIMER_SELECTION2,           /*!< It contains the pointer to the timer
                                                             used for PWM generation. */
/* PWM Driving signals initialization ----------------------------------------*/

  .wTIM1Remapping    = PWM_TIMER_REMAPPING2,    /*!< Used only for instance #1, it
                                                     remaps TIM1 outputs. It must equal to
                                                     GPIO_PartialRemap_TIM1 or
                                                     GPIO_FullRemap_TIM1 or
                                                     GPIO_NoRemap_TIM1 */
  .hCh1Polarity      = PHASE_UH_POLARITY2,      /*!< Channel 1 (high side) output polarity,
                                                     it must be TIM_OCPolarity_High or
                                                     TIM_OCPolarity_Low */
  .hCh1Port          = PHASE_UH_GPIO_PORT2,     /*!< Channel 1 (high side) GPIO output
                                                     port (if used, after re-mapping).
                                                     It must be GPIOx x= A, B, ...*/
  .hCh1Pin           = PHASE_UH_GPIO_PIN2,      /*!< Channel 1 (high side) GPIO output pin
                                                     (if used, after re-mapping). It must be
                                                     GPIO_Pin_x x= 0, 1, ...*/
  .hCh1IdleState     = IDLE_UH_POLARITY2,       /*!< Channel 1 (high side) state (low or high)
                                                     when TIM peripheral is in Idle state.
                                                     It must be TIM_OCIdleState_Set or
                                                     TIM_OCIdleState_Reset*/

  .hCh2Polarity      = PHASE_VH_POLARITY2,      /*!< Channel 2 (high side) output polarity,
                                                     it must be TIM_OCPolarity_High or
                                                     TIM_OCPolarity_Low */
  .hCh2Port          = PHASE_VH_GPIO_PORT2,     /*!< Channel 2 (high side) GPIO output
                                                     port (if used, after re-mapping).
                                                     It must be GPIOx x= A, B, ...*/
  .hCh2Pin           = PHASE_VH_GPIO_PIN2,      /*!< Channel 2 (high side) GPIO output pin
                                                     (if used, after re-mapping). It must be
                                                     GPIO_Pin_x x= 0, 1, ...*/
  .hCh2IdleState     = IDLE_VH_POLARITY2,       /*!< Channel 2 (high side) state (low or high)
                                                     when TIM peripheral is in Idle state.
                                                     It must be TIM_OCIdleState_Set or
                                                     TIM_OCIdleState_Reset*/

  .hCh3Polarity      = PHASE_WH_POLARITY2,      /*!< Channel 3 (high side) output polarity,
                                                     it must be TIM_OCPolarity_High or
                                                     TIM_OCPolarity_Low */
  .hCh3Port          = PHASE_WH_GPIO_PORT2,     /*!< Channel 3 (high side) GPIO output
                                                     port (if used, after re-mapping).
                                                     It must be GPIOx x= A, B, ...*/
  .hCh3Pin           = PHASE_WH_GPIO_PIN2,      /*!< Channel 3 (high side) GPIO output pin
                                                     (if used, after re-mapping). It must be
                                                     GPIO_Pin_x x= 0, 1, ...*/
  .hCh3IdleState     = IDLE_WH_POLARITY2,       /*!< Channel 3 (high side) state (low or high)
                                                     when TIM peripheral is in Idle state.
                                                     It must be TIM_OCIdleState_Set or
                                                     TIM_OCIdleState_Reset*/

  .LowSideOutputs    = (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING2,  /*!< Low side or enabling signals
                                                                                   generation method are defined
                                                                                   here.*/

  .hCh1NPolarity     = PHASE_UL_POLARITY2,      /*!< Channel 1N (low side) output polarity,
                                                     it must be TIM_OCNPolarity_High or
                                                     TIM_OCNPolarity_Low */
  .hCh1NPort         = PHASE_UL_GPIO_PORT2,     /*!< Channel 1N (low side) GPIO output
                                                     port (if used, after re-mapping).
                                                     It must be GPIOx x= A, B, ...*/
  .hCh1NPin          = PHASE_UL_GPIO_PIN2,      /*!< Channel 1N (low side) GPIO output pin
                                                     (if used, after re-mapping). It must be
                                                     GPIO_Pin_x x= 0, 1, ...*/
  .hCh1NIdleState    = IDLE_UL_POLARITY2,       /*!< Channel 1N (low side) state (low or high)
                                                     when TIM peripheral is in Idle state.
                                                     It must be TIM_OCNIdleState_Set or
                                                     TIM_OCNIdleState_Reset*/


  .hCh2NPolarity     = PHASE_VL_POLARITY2,      /*!< Channel 2N (low side) output polarity,
                                                     it must be TIM_OCNPolarity_High or
                                                     TIM_OCNPolarity_Low */
  .hCh2NPort         = PHASE_VL_GPIO_PORT2,     /*!< Channel 2N (low side) GPIO output
                                                     port (if used, after re-mapping).
                                                     It must be GPIOx x= A, B, ...*/
  .hCh2NPin          = PHASE_VL_GPIO_PIN2,      /*!< Channel 2N (low side) GPIO output pin
                                                     (if used, after re-mapping). It must be
                                                     GPIO_Pin_x x= 0, 1, ...*/
  .hCh2NIdleState    = IDLE_VL_POLARITY2,       /*!< Channel 2N (low side) state (low or high)
                                                     when TIM peripheral is in Idle state.
                                                     It must be TIM_OCNIdleState_Set or
                                                     TIM_OCNIdleState_Reset*/

  .hCh3NPolarity     = PHASE_WL_POLARITY2,      /*!< Channel 3N (low side) output polarity,
                                                     it must be TIM_OCNPolarity_High or
                                                     TIM_OCNPolarity_Low */
  .hCh3NPort         = PHASE_WL_GPIO_PORT2,     /*!< Channel 3N (low side)  GPIO output
                                                     port (if used, after re-mapping).
                                                     It must be GPIOx x= A, B, ...*/
  .hCh3NPin          = PHASE_WL_GPIO_PIN2,      /*!< Channel 3N (low side)  GPIO output pin
                                                     (if used, after re-mapping). It must be
                                                     GPIO_Pin_x x= 0, 1, ...*/
  .hCh3NIdleState    = IDLE_WL_POLARITY2,       /*!< Channel 3N (low side) state (low or high)
                                                     when TIM peripheral is in Idle state.
                                                     It must be TIM_OCNIdleState_Set or
                                                     TIM_OCNIdleState_Reset*/
/* Emergency signal initialization ----------------------------------------*/
  .EmergencyStop =   (FunctionalState)SW_OV_CURRENT_PROT_ENABLING2, /*!< It enable/disable the management of
                                                                         an emergency input instantaneously
                                                                         stopping PWM generation. It must be
                                                                         either equal to ENABLE or DISABLE */
  .hBKINPolarity =   OVERCURR_FEEDBACK_POLARITY2,                   /*!< Emergency Stop (BKIN) input polarity,
                                                                         it must be TIM_BreakPolarity_Low or
                                                                         TIM_BreakPolarity_High */
  .hBKINPort     =   EMERGENCY_STOP_GPIO_PORT2,                     /*!< Emergency Stop (BKIN) GPIO input
                                                                         port (if used, after re-mapping).
                                                                         It must be GPIOx x= A, B, ...*/
  .hBKINPin      =   EMERGENCY_STOP_GPIO_PIN2                       /*!< Emergency Stop (BKIN) GPIO input pin
                                                                         (if used, after re-mapping). It must be
                                                                         GPIO_Pin_x x= 0, 1, ...*/
};

/**
  * @brief  Current sensor parameters Motor 2 - three shunt
  */
R3_DDParams_t R3_DDParamsM2 =
{
  .wADC_Clock_Divider = ADC_CLOCK_DIVIDER2,    /*!< APB2 clock prescaling factor for
                                                    ADC peripheral. It must be RCC_PCLK2_DivX
                                                    x = 2, 4, 6, 8 */
  .bTim_Clock_Divider = TIM_CLOCK_DIVIDER2,   /* System clock divider for Advanded Timer */
/* Dual MC parameters --------------------------------------------------------*/
  .bInstanceNbr     = 2,                             /*!< Instance number. Necessary to
                                                          properly synchronize TIM8 with
                                                          TIM1 at peripheral initializations
                                                           */
  .Tw               = MAX_TWAIT2,                     /*!< It is used for switching the context
                                                           in dual MC. It contains biggest delay
                                                           (expressed in counter ticks) between
                                                           the counter crest and ADC latest
                                                           trigger  */
  .bFreqRatio       = FREQ_RATIO,                     /*!< It is used in case of dual MC to
                                                           synchronize TIM1 and TIM8. It has
                                                           effect only on the second instanced
                                                           object and must be equal to the
                                                           ratio between the two PWM frequencies
                                                           (higher/lower). Supported values are
                                                           1, 2 or 3 */

  .bIsHigherFreqTim = FREQ_RELATION2,                 /*!< When bFreqRatio is greather than 1
                                                           this param is used to indicate if this
                                                           instance is the one with the highest
                                                           frequency. Allowed value are: HIGHER_FREQ
                                                           or LOWER_FREQ */
  .IRQnb            = MC_IRQ_PWMNCURRFDBK_2,          /*!< MC IRQ number used as TIMx Update
                                                           event */
/* Current reading A/D Conversions initialization -----------------------------*/
  .bIaChannel       = PHASE_U_CURR_CHANNEL2,          /*!< ADC channel used for conversion of
                                                           current Ia. It must be equal to
                                                           ADC_Channel_x x= 0, ..., 15*/
  .hIaPort          = PHASE_U_GPIO_PORT2,             /*!< GPIO port used by hIaChannel. It must
                                                           be equal to GPIOx x= A, B, ...*/
  .hIaPin           = PHASE_U_GPIO_PIN2,              /*!< GPIO pin used by hIaChannel. It must be
                                                           equal to GPIO_Pin_x x= 0, 1, ...*/
  .b_IaSamplingTime = SAMPLING_TIME_SEL2,             /*!< Sampling time used to convert hIaChannel.
                                                           It must be equal to ADC_SampleTime_xCycles5
                                                           x= 1, 7, ...*/
  .bIbChannel       = PHASE_V_CURR_CHANNEL2,          /*!< ADC channel used for conversion of
                                                           current Ib. It must be equal to
                                                           ADC_Channel_x x= 0, ..., 15*/
  .hIbPort          = PHASE_V_GPIO_PORT2,             /*!< GPIO port used by hIbChannel. It must
                                                           be equal to GPIOx x= A, B, ...*/
  .hIbPin           = PHASE_V_GPIO_PIN2,              /*!< GPIO pin used by hIaChannel. It must be
                                                           equal to GPIO_Pin_x x= 0, 1, ...*/
  .b_IbSamplingTime = SAMPLING_TIME_SEL2,             /*!< Sampling time used to convert hIbChannel.
                                                           It must be equal to ADC_SampleTime_xCycles5
                                                           x= 1, 7, ...*/
  .bIcChannel       = PHASE_W_CURR_CHANNEL2,          /*!< ADC channel used for conversion of
                                                           current Ia. It must be equal to
                                                           ADC_Channel_x x= 0, ..., 15*/
  .hIcPort          = PHASE_W_GPIO_PORT2,             /*!< GPIO port used by hIaChannel. It must
                                                           be equal to GPIOx x= A, B, ...*/
  .hIcPin           = PHASE_W_GPIO_PIN2,              /*!< GPIO pin used by hIaChannel. It must be
                    =                                      equal to GPIO_Pin_x x= 0, 1, ...*/
  .b_IcSamplingTime = SAMPLING_TIME_SEL2,             /*!< Sampling time used to convert hIaChannel.
                                                           It must be equal to ADC_SampleTime_xCycles5
                                                           x= 1, 7, ...*/

/* PWM generation parameters --------------------------------------------------*/
  .hDeadTime          = DEAD_TIME_COUNTS2,                    /*!< Dead time in number of TIM clock
                                                                   cycles. If CHxN are enabled, it must
                                                                   contain the dead time to be generated
                                                                   by the microcontroller, otherwise it
                                                                   expresses the maximum dead time
                                                                   generated by driving network */
  .bRepetitionCounter = REP_COUNTER2,                         /*!< It expresses the number of PWM
                                                                   periods to be elapsed before compare
                                                                   registers are updated again. In
                                                                   particular:
                                                                   RepetitionCounter= (2* #PWM periods)-1*/
  .hTafter            = TW_AFTER2,                            /*!< It is the sum of dead time plus max
                                                                   value between rise time and noise time
                                                                   express in number of TIM clocks.*/
  .hTbefore           = TW_BEFORE2,                           /*!< It is the sampling time express in
                                                                   number of TIM clocks.*/
  .TIMx               = PWM_TIMER_SELECTION2,                 /*!< It contains the pointer to the timer
                                                                   used for PWM generation. */
/* PWM Driving signals initialization ----------------------------------------*/

  .wTIM1Remapping = PWM_TIMER_REMAPPING2,          /*!< Used only for instance #1, it
                                                        remaps TIM1 outputs. It must equal to
                                                        GPIO_PartialRemap_TIM1 or
                                                        GPIO_FullRemap_TIM1 or
                                                        GPIO_NoRemap_TIM1 */
  .hCh1Polarity   = PHASE_UH_POLARITY2,            /*!< Channel 1 (high side) output polarity,
                                                        it must be TIM_OCPolarity_High or
                                                        TIM_OCPolarity_Low */
  .hCh1Port       = PHASE_UH_GPIO_PORT2,           /*!< Channel 1 (high side) GPIO output
                                                        port (if used, after re-mapping).
                                                        It must be GPIOx x= A, B, ...*/
  .hCh1Pin        = PHASE_UH_GPIO_PIN2,            /*!< Channel 1 (high side) GPIO output pin
                                                        (if used, after re-mapping). It must be
                                                        GPIO_Pin_x x= 0, 1, ...*/
  .hCh1IdleState  = IDLE_UH_POLARITY2,             /*!< Channel 1 (high side) state (low or high)
                                                        when TIM peripheral is in Idle state.
                                                        It must be TIM_OCIdleState_Set or
                                                        TIM_OCIdleState_Reset*/


  .hCh2Polarity   = PHASE_VH_POLARITY2,            /*!< Channel 2 (high side) output polarity,
                                                        it must be TIM_OCPolarity_High or
                                                        TIM_OCPolarity_Low */
  .hCh2Port       = PHASE_VH_GPIO_PORT2,           /*!< Channel 2 (high side) GPIO output
                                                        port (if used, after re-mapping).
                                                        It must be GPIOx x= A, B, ...*/
  .hCh2Pin        = PHASE_VH_GPIO_PIN2,            /*!< Channel 2 (high side) GPIO output pin
                                                        (if used, after re-mapping). It must be
                                                        GPIO_Pin_x x= 0, 1, ...*/
  .hCh2IdleState  = IDLE_VH_POLARITY2,             /*!< Channel 2 (high side) state (low or high)
                                                        when TIM peripheral is in Idle state.
                                                        It must be TIM_OCIdleState_Set or
                                                        TIM_OCIdleState_Reset*/

  .hCh3Polarity   = PHASE_WH_POLARITY2,            /*!< Channel 3 (high side) output polarity,
                                                        it must be TIM_OCPolarity_High or
                                                        TIM_OCPolarity_Low */
  .hCh3Port       = PHASE_WH_GPIO_PORT2,           /*!< Channel 3 (high side) GPIO output
                                                        port (if used, after re-mapping).
                                                        It must be GPIOx x= A, B, ...*/
  .hCh3Pin        = PHASE_WH_GPIO_PIN2,            /*!< Channel 3 (high side) GPIO output pin
                                                        (if used, after re-mapping). It must be
                                                        GPIO_Pin_x x= 0, 1, ...*/
  .hCh3IdleState  = IDLE_WH_POLARITY2,             /*!< Channel 3 (high side) state (low or high)
                                                        when TIM peripheral is in Idle state.
                                                        It must be TIM_OCIdleState_Set or
                                                        TIM_OCIdleState_Reset*/

  .LowSideOutputs = (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING2,  /*!< Low side or enabling signals
                                                                                generation method are defined
                                                                                here.*/

  .hCh1NPolarity  = PHASE_UL_POLARITY2,            /*!< Channel 1N (low side) output polarity,
                                                        it must be TIM_OCNPolarity_High or
                                                        TIM_OCNPolarity_Low */
  .hCh1NPort      = PHASE_UL_GPIO_PORT2,           /*!< Channel 1N (low side) GPIO output
                                                        port (if used, after re-mapping).
                                                        It must be GPIOx x= A, B, ...*/
  .hCh1NPin       = PHASE_UL_GPIO_PIN2,            /*!< Channel 1N (low side) GPIO output pin
                                                        (if used, after re-mapping). It must be
                                                        GPIO_Pin_x x= 0, 1, ...*/
  .hCh1NIdleState = IDLE_UL_POLARITY2,             /*!< Channel 1N (low side) state (low or high)
                                                        when TIM peripheral is in Idle state.
                                                        It must be TIM_OCNIdleState_Set or
                                                        TIM_OCNIdleState_Reset*/


  .hCh2NPolarity  = PHASE_VL_POLARITY2,            /*!< Channel 2N (low side) output polarity,
                                                        it must be TIM_OCNPolarity_High or
                                                        TIM_OCNPolarity_Low */
  .hCh2NPort      = PHASE_VL_GPIO_PORT2,           /*!< Channel 2N (low side) GPIO output
                                                        port (if used, after re-mapping).
                                                        It must be GPIOx x= A, B, ...*/
  .hCh2NPin       = PHASE_VL_GPIO_PIN2,            /*!< Channel 2N (low side) GPIO output pin
                                                        (if used, after re-mapping). It must be
                                                        GPIO_Pin_x x= 0, 1, ...*/
  .hCh2NIdleState = IDLE_VL_POLARITY2,             /*!< Channel 2N (low side) state (low or high)
                                                        when TIM peripheral is in Idle state.
                                                        It must be TIM_OCNIdleState_Set or
                                                        TIM_OCNIdleState_Reset*/

  .hCh3NPolarity  = PHASE_WL_POLARITY2,            /*!< Channel 3N (low side) output polarity,
                                                        it must be TIM_OCNPolarity_High or
                                                        TIM_OCNPolarity_Low */
  .hCh3NPort      = PHASE_WL_GPIO_PORT2,           /*!< Channel 3N (low side)  GPIO output
                                                        port (if used, after re-mapping).
                                                        It must be GPIOx x= A, B, ...*/
  .hCh3NPin       = PHASE_WL_GPIO_PIN2,            /*!< Channel 3N (low side)  GPIO output pin
                                                        (if used, after re-mapping). It must be
                                                        GPIO_Pin_x x= 0, 1, ...*/
  .hCh3NIdleState = IDLE_WL_POLARITY2,             /*!< Channel 3N (low side) state (low or high)
                                                        when TIM peripheral is in Idle state.
                                                        It must be TIM_OCNIdleState_Set or
                                                        TIM_OCNIdleState_Reset*/
/* PWM Driving signals initialization ----------------------------------------*/
  .EmergencyStop = (FunctionalState)SW_OV_CURRENT_PROT_ENABLING2, /*!< It enable/disable the management of
                                                                       an emergency input instantaneously
                                                                       stopping PWM generation. It must be
                                                                       either equal to ENABLE or DISABLE */
  .hBKINPolarity = OVERCURR_FEEDBACK_POLARITY2,     /*!< Emergency Stop (BKIN) input polarity,
                                                         it must be TIM_BreakPolarity_Low or
                                                         TIM_BreakPolarity_High */
  .hBKINPort     = EMERGENCY_STOP_GPIO_PORT2,       /*!< Emergency Stop (BKIN) GPIO input
                                                         port (if used, after re-mapping).
                                                         It must be GPIOx x= A, B, ...*/
  .hBKINPin      = EMERGENCY_STOP_GPIO_PIN2,        /*!< Emergency Stop (BKIN) GPIO input pin
                                                         (if used, after re-mapping). It must be
                                                         GPIO_Pin_x x= 0, 1, ...*/
};

/**
  * @brief  Current sensor parameters Dual Drive Motor 2 - one shunt
  */
R1_DDParams_t R1_DDParamsM2 =
{
  .wADC_Clock_Divider = ADC_CLOCK_DIVIDER2,          /*!<  APB2 clock prescaling factor for
                                                           ADC peripheral. It must be RCC_PCLK2_DivX
                                                           x = 2, 4, 6, 8. Only the first instance
                                                           parameter is used to set the prescaling
                                                           factor*/
  .bTim_Clock_Divider = TIM_CLOCK_DIVIDER2,          /* System clock divider for Advanded Timer */
  .IRQnb              = MC_IRQ_PWMNCURRFDBK_2,       /*!< MC IRQ number used for UPDATE event */

/* Instance number -----------------------------------------------------------*/
  .bInstanceNbr        = 2,                          /*!< Instance number. Necessary to
                                                           properly synchronize PWM Timers and
                                                           start peripheral clocks*/
  .IsFirstR1DDInstance = M2FIRSTR1,                  /*!< Specifies whether this object is the first
                                                          R1HD2 instance or not.*/
  .bFreqRatio          = FREQ_RATIO,                 /*!< It is used in case of dual MC to
                                                          synchronize TIM1 and TIM8. It has
                                                          effect only on the second instanced
                                                          object and must be equal to the
                                                          ratio between the two PWM frequencies
                                                          (higher/lower). Supported values are
                                                          1, 2 or 3 */
  .bIsHigherFreqTim    = FREQ_RELATION2,             /*!< When bFreqRatio is greather than 1
                                                          this param is used to indicate if this
                                                          instance is the one with the highest
                                                          frequency. Allowed value are: HIGHER_FREQ
                                                          or LOWER_FREQ */
  .TIMx                = PWM_TIMER_SELECTION2,       /*!< Timer used for PWM generation. It should be
                                                          TIM1 or TIM8*/

/* Current reading A/D Conversions initialization -----------------------------*/
  .hIChannel       = PHASE_CURRENTS_CHANNEL2,        /*!< ADC channel used for conversion of
                                                          current. It must be equal to
                                                          ADC_Channel_x x= 0, ..., 15*/
  .hIPort          = PHASE_CURRENTS_GPIO_PORT2,       /*!< GPIO port used by hIChannel. It must
                                                          be equal to GPIOx x= A, B, ...*/
  .hIPin           = PHASE_CURRENTS_GPIO_PIN2,                  /*!< GPIO pin used by hIChannel. It must be
                                                          equal to GPIO_Pin_x x= 0, 1, ...*/
  .b_ISamplingTime = SAMPLING_TIME_SEL2,             /*!< Sampling time used to convert hIChannel.
                                                          It must be equal to ADC_SampleTime_xCycles5
                                                          x= 1, 7, ...*/

/* PWM generation parameters --------------------------------------------------*/
  .hDeadTime          = DEAD_TIME_COUNTS2,           /*!< Dead time in number of TIM clock
                                                          cycles. If CHxN are enabled, it must
                                                          contain the dead time to be generated
                                                          by the microcontroller, otherwise it
                                                          expresses the maximum dead time
                                                          generated by driving network */
  .bRepetitionCounter = REP_COUNTER2,                /*!< It expresses the number of PWM
                                                          periods to be elapsed before compare
                                                          registers are updated again. In
                                                          particular:
                                                          RepetitionCounter= (2* #PWM periods)-1*/
  .hTafter            = TAFTER2,     /*!< It must be equal to ((u16)(((DEADTIME_NS+((u16)(TRISE_NS)))*24ul)/1000ul))*/

  .hTbefore           = TBEFORE2,    /*!< It must be equal to (((u16)(((((u16)(SAMPLING_TIME_NS)))*24ul)/1000ul))+1)*/

  .hTMin              = TMIN2,       /*!< It must be equal to (((u16)(((DEADTIME_NS+((u16)(TRISE_NS))+((u16)(SAMPLING_TIME_NS)))*24uL)/1000ul))+1)*/


  .hHTMin             = HTMIN2,      /*!< It must be equal to (u16)(TMIN >> 1)*/
  .hTSample           = TSAMPLE2,    /*!< It must be equal to (u16)(((u16)(SAMPLING_TIME_NS) * 24uL)/1000uL)*/

  .hMaxTrTs           = MAX_TRTS2,   /*!< It must be equal to 2*(u16)((((u16)(TRISE_NS)) * 24uL)/1000uL) if TRISE_NS > SAMPLING_TIME_NS
                                          to 2*hTSample otherwise */

/* PWM Driving signals initialization ----------------------------------------*/

  .wTIM1Remapping = PWM_TIMER_REMAPPING2,              /*!< Used only for instance #1, it
                                                            remaps TIM1 outputs. It must equal to
                                                            GPIO_PartialRemap_TIM1 or
                                                            GPIO_FullRemap_TIM1 or
                                                            GPIO_NoRemap_TIM1 */
  .hCh1Polarity   = PHASE_UH_POLARITY2,                /*!< Channel 1 (high side) output polarity,
                                                            it must be TIM_OCPolarity_High or
                                                            TIM_OCPolarity_Low */
  .hCh1Port       = PHASE_UH_GPIO_PORT2,               /*!< Channel 1 (high side) GPIO output
                                                            port (if used, after re-mapping).
                                                            It must be GPIOx x= A, B, ...*/
  .hCh1Pin        = PHASE_UH_GPIO_PIN2,                /*!< Channel 1 (high side) GPIO output pin
                                                            (if used, after re-mapping). It must be
                                                            GPIO_Pin_x x= 0, 1, ...*/
  .hCh1IdleState  = IDLE_UH_POLARITY2,                 /*!< Channel 1 (high side) state (low or high)
                                                            when TIM peripheral is in Idle state.
                                                            It must be TIM_OCIdleState_Set or
                                                            TIM_OCIdleState_Reset*/

  .hCh2Polarity   = PHASE_VH_POLARITY2,                /*!< Channel 2 (high side) output polarity,
                                                            it must be TIM_OCPolarity_High or
                                                            TIM_OCPolarity_Low */
  .hCh2Port       = PHASE_VH_GPIO_PORT2,               /*!< Channel 2 (high side) GPIO output
                                                            port (if used, after re-mapping).
                                                            It must be GPIOx x= A, B, ...*/
  .hCh2Pin        = PHASE_VH_GPIO_PIN2,                /*!< Channel 2 (high side) GPIO output pin
                                                            (if used, after re-mapping). It must be
                                                            GPIO_Pin_x x= 0, 1, ...*/
  .hCh2IdleState  = IDLE_VH_POLARITY2,                 /*!< Channel 2 (high side) state (low or high)
                                                            when TIM peripheral is in Idle state.
                                                            It must be TIM_OCIdleState_Set or
                                                            TIM_OCIdleState_Reset*/

  .hCh3Polarity   = PHASE_WH_POLARITY2,                /*!< Channel 3 (high side) output polarity,
                                                            it must be TIM_OCPolarity_High or
                                                            TIM_OCPolarity_Low */
  .hCh3Port       = PHASE_WH_GPIO_PORT2,               /*!< Channel 3 (high side) GPIO output
                                                            port (if used, after re-mapping).
                                                            It must be GPIOx x= A, B, ...*/
  .hCh3Pin        = PHASE_WH_GPIO_PIN2,                /*!< Channel 3 (high side) GPIO output pin
                                                           (if used, after re-mapping). It must be
                                                            GPIO_Pin_x x= 0, 1, ...*/
  .hCh3IdleState  = IDLE_WH_POLARITY2,                 /*!< Channel 3 (high side) state (low or high)
                                                            when TIM peripheral is in Idle state.
                                                            It must be TIM_OCIdleState_Set or
                                                            TIM_OCIdleState_Reset*/

  .LowSideOutputs = (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING2,  /*!< Low side or enabling signals
                                                                                generation method are defined
                                                                                here.*/

  .hCh1NPolarity  = PHASE_UL_POLARITY2,                /*!< Channel 1N (low side) output polarity,
                                                            it must be TIM_OCNPolarity_High or
                                                            TIM_OCNPolarity_Low */
  .hCh1NPort      = PHASE_UL_GPIO_PORT2,               /*!< Channel 1N (low side) GPIO output
                                                            port (if used, after re-mapping).
                                                            It must be GPIOx x= A, B, ...*/
  .hCh1NPin       = PHASE_UL_GPIO_PIN2,                /*!< Channel 1N (low side) GPIO output pin
                                                           (if used, after re-mapping). It must be
                                                            GPIO_Pin_x x= 0, 1, ...*/
  .hCh1NIdleState = IDLE_UL_POLARITY2,                 /*!< Channel 1N (low side) state (low or high)
                                                            when TIM peripheral is in Idle state.
                                                            It must be TIM_OCNIdleState_Set or
                                                            TIM_OCNIdleState_Reset*/


  .hCh2NPolarity  = PHASE_VL_POLARITY2,                /*!< Channel 2N (low side) output polarity,
                                                            it must be TIM_OCNPolarity_High or
                                                            TIM_OCNPolarity_Low */
  .hCh2NPort      = PHASE_VL_GPIO_PORT2,               /*!< Channel 2N (low side) GPIO output
                                                            port (if used, after re-mapping).
                                                            It must be GPIOx x= A, B, ...*/
  .hCh2NPin       = PHASE_VL_GPIO_PIN2,                /*!< Channel 2N (low side) GPIO output pin
                                                           (if used, after re-mapping). It must be
                                                            GPIO_Pin_x x= 0, 1, ...*/
  .hCh2NIdleState = IDLE_VL_POLARITY2,                 /*!< Channel 2N (low side) state (low or high)
                                                            when TIM peripheral is in Idle state.
                                                            It must be TIM_OCNIdleState_Set or
                                                            TIM_OCNIdleState_Reset*/

  .hCh3NPolarity  = PHASE_WL_POLARITY2,                /*!< Channel 3N (low side) output polarity,
                                                            it must be TIM_OCNPolarity_High or
                                                            TIM_OCNPolarity_Low */
  .hCh3NPort      = PHASE_WL_GPIO_PORT2,               /*!< Channel 3N (low side)  GPIO output
                                                            port (if used, after re-mapping).
                                                            It must be GPIOx x= A, B, ...*/
  .hCh3NPin       = PHASE_WL_GPIO_PIN2,                /*!< Channel 3N (low side)  GPIO output pin
                                                            (if used, after re-mapping). It must be
                                                            GPIO_Pin_x x= 0, 1, ...*/
  .hCh3NIdleState = IDLE_WL_POLARITY2,                 /*!< Channel 3N (low side) state (low or high)
                                                            when TIM peripheral is in Idle state.
                                                            It must be TIM_OCNIdleState_Set or
                                                            TIM_OCNIdleState_Reset*/
/* Emergengy signal initialization ----------------------------------------*/
  .EmergencyStop = (FunctionalState)SW_OV_CURRENT_PROT_ENABLING2, /*!< It enable/disable the management of
                                                                       an emergency input instantaneously
                                                                       stopping PWM generation. It must be
                                                                       either equal to ENABLE or DISABLE */
  .hBKINPolarity = OVERCURR_FEEDBACK_POLARITY2,                   /*!< Emergency Stop (BKIN) input polarity,
                                                                       it must be TIM_BreakPolarity_Low or
                                                                       TIM_BreakPolarity_High */
  .hBKINPort     = EMERGENCY_STOP_GPIO_PORT2,                     /*!< Emergency Stop (BKIN) GPIO input
                                                                       port (if used, after re-mapping).
                                                                       It must be GPIOx x= A, B, ...*/
  .hBKINPin      = EMERGENCY_STOP_GPIO_PIN2                       /*!< Emergency Stop (BKIN) GPIO input pin
                                                                       (if used, after re-mapping). It must be
                                                                       GPIO_Pin_x x= 0, 1, ...*/
};

/**
  * @brief  SpeedNPosition sensor parameters Motor 2 - Base Class
  */
SpeednPosFdbkParams_t SpeednPosFdbkParamsM2 =
{
  .bElToMecRatio             = POLE_PAIR_NUM2,                            /*!< Coefficient used to transform electrical to
                                                                              mechanical quantities and viceversa. It usually
                                                                              coincides with motor pole pairs number*/
  .hMaxReliableMecSpeed01Hz  = (uint16_t)(1.15*MAX_APPLICATION_SPEED2/6), /*< Maximum value of measured speed that is
                                                                              considered to be valid. It's expressed
                                                                              in tenth of mechanical Hertz.*/
  .hMinReliableMecSpeed01Hz  = (uint16_t)(MIN_APPLICATION_SPEED2/6),      /*< Minimum value of measured speed that is
                                                                              considered to be valid. It's expressed
                                                                              in tenth of mechanical Hertz.*/
  .bMaximumSpeedErrorsNumber = MEAS_ERRORS_BEFORE_FAULTS2,                /*< Maximum value of not valid measurements
                                                                              before an error is reported.*/
  .hMaxReliableMecAccel01HzP = 65535,                                     /*< Maximum value of measured acceleration
                                                                              that is considered to be valid. It's
                                                                              expressed in 01HzP (tenth of Hertz per
                                                                              speed calculation period)*/
  .hMeasurementFrequency     = TF_REGULATION_RATE2,                       /*< Frequency on which the user will request
                                                                              a measurement of the rotor electrical angle.
                                                                              It's also used to convert measured speed from
                                                                              tenth of Hz to dpp and viceversa.*/
};

/**
  * @brief  SpeedNPosition sensor parameters Motor 2 - State Observer + PLL
  */
STOParams_t STOParamsM2 =
{
  .hC1  = C12,    /*!< State observer constant C1, it can
                       be computed as F1 * Rs(ohm)/(Ls[H] *
                       State observer execution rate [Hz])*/
  .hC2  = C22,    /*!<  Variable containing state observer
                        constant C2, it can be computed as
                        F1 * K1/ State observer execution
                        rate [Hz] being K1 one of the two
                        observer gains   */
  .hC3  = C32,    /*!< State observer constant C3, it can
                       be computed as F1 * Max application
                       speed [rpm] * Motor B-emf constant
                       [Vllrms/krpm] * sqrt(2)/ (Ls [H] *
                       max measurable current (A) * State
                       observer execution rate [Hz])*/
  .hC4  = C42,    /*!< State Observer constant C4, it can
                       be computed as K2 * max measurable
                       current (A) / (Max application speed
                       [rpm] * Motor B-emf constant
                       [Vllrms/krpm] * sqrt(2) * F2 * State
                       observer execution rate [Hz]) being
                       K2 one of the two observer gains  */
  .hC5  = C52,    /*!< State Observer constant C5, it can
                       be computed as F1 * max measurable
                       voltage / (2 * Ls [Hz] * max
                       measurable current * State observer
                       execution rate [Hz]) */
  .hF1  = F12,    /*!< State observer scaling factor F1 */
  .hF2  = F22,    /*!< State observer scaling factor F2 */
   {
     PLL_KP_GAIN2,               /*!< Default Kp gain, used to initialize Kp gain
                                     software variable*/
     PLL_KI_GAIN2,               /*!< Default Ki gain, used to initialize Ki gain
                                     software variable*/
     PLL_KPDIV2,                 /*!< Kp gain divisor, used in conjuction with
                                     Kp gain allows obtaining fractional values.
                                     If FULL_MISRA_C_COMPLIANCY is not defined
                                     the divisor is implemented through
                                     algebrical right shifts to speed up PI
                                     execution. Only in this case this parameter
                                     specifies the number of right shifts to be
                                     executed */
     PLL_KIDIV2,                /*!< Ki gain divisor, used in conjuction with
                                     Ki gain allows obtaining fractional values.
                                     If FULL_MISRA_C_COMPLIANCY is not defined
                                     the divisor is implemented through
                                     algebrical right shifts to speed up PI
                                     execution. Only in this case this parameter
                                     specifies the number of right shifts to be
                                     executed */
     S32_MAX,                   /*!< Upper limit used to saturate the integral
                                     term given by Ki / Ki divisor * integral of
                                     process variable error */
     -S32_MAX,                  /*!< Lower limit used to saturate the integral
                                     term given by Ki / Ki divisor * integral of
                                     process variable error */
     S16_MAX,                   /*!< Upper limit used to saturate the PI output
                                     */
     -S16_MAX,                  /*!< Lower limit used to saturate the PI output
                                     */
     PLL_KPDIV_LOG2,             /*!< Kp gain divisor expressed as power of 2.
                                     E.g. if gain divisor is 512 the value
                                     must be 9 because 2^9 = 512 */
     PLL_KIDIV_LOG2              /*!< Ki gain divisor expressed as power of 2.
                                     E.g. if gain divisor is 512 the value
                                     must be 9 because 2^9 = 512 */
   },                                 /*!< It contains the parameters of the PI
                                           object necessary for PLL
                                           implementation */
  .bSpeedBufferSize01Hz        = STO_FIFO_DEPTH_01HZ2,              /*!< Depth of FIFO used to average
                                                                         estimated speed exported by
                                                                         SPD_GetAvrgMecSpeed01Hz. It
                                                                         must be an integer number between 1
                                                                         and 64 */
  .bSpeedBufferSizedpp         = STO_FIFO_DEPTH_DPP2,               /*!< Depth of FIFO used for both averaging
                                                                         estimated speed exported by
                                                                         SPD_GetElSpeedDpp and state
                                                                         observer equations. It must be an
                                                                         integer number between 1 and
                                                                         bSpeedBufferSize01Hz */
  .hVariancePercentage         = PERCENTAGE_FACTOR2,                /*!< Parameter expressing the maximum
                                                                         allowed variance of speed estimation
                                                                         */
  .bSpeedValidationBand_H      = SPEED_BAND_UPPER_LIMIT2,           /*!< It expresses how much estimated speed
                                                                         can exceed forced stator electrical
                                                                         frequency during start-up without
                                                                         being considered wrong. The
                                                                         measurement unit is 1/16 of forced
                                                                         speed */
  .bSpeedValidationBand_L      = SPEED_BAND_LOWER_LIMIT2,           /*!< It expresses how much estimated speed
                                                                         can be below forced stator electrical
                                                                         frequency during start-up without
                                                                         being considered wrong. The
                                                                         measurement unit is 1/16 of forced
                                                                         speed */
  .hMinStartUpValidSpeed       = OBS_MINIMUM_SPEED2,                /*!< Minimum mechanical speed (expressed in
                                                                         01Hz required to validate the start-up
                                                                    */
  .bStartUpConsistThreshold    = NB_CONSECUTIVE_TESTS2,             /*!< Number of consecutive tests on speed */


  .bReliability_hysteresys     = OBS_MEAS_ERRORS_BEFORE_FAULTS2,    /*!< Number of reliability failed
                                                                                    consecutive tests before a speed
                                                                                    check fault is returned to base class
                                                                                    */
  .bBemfConsistencyCheck       = BEMF_CONSISTENCY_TOL2,             /*!< Degree of consistency of the observed
                                                                         back-emfs, it must be an integer
                                                                         number ranging from 1 (very high
                                                                         consistency) down to 64 (very poor
                                                                         consistency) */
  .bBemfConsistencyGain        = BEMF_CONSISTENCY_GAIN2,            /*!< Gain to be applied when checking
                                                                         back-emfs consistency; default value
                                                                         is 64 (neutral), max value 105
                                                                         (x1,64 amplification), min value 1
                                                                         (/64 attenuation) */
  .hMaxAppPositiveMecSpeed01Hz = (uint16_t)(MAX_APPLICATION_SPEED2*1.15/6.0),  /*!< Maximum positive value
                                                                                    of rotor speed. It's expressed in
                                                                                    tenth of mechanical Hertz. It can be
                                                                                    x1.1 greater than max application*/

  .hF1LOG                      = F1_LOG2,                           /*!< F1 gain divisor expressed as power of 2.
                                                                         E.g. if gain divisor is 512 the value
                                                                         must be 9 because 2^9 = 512 */
  .hF2LOG                      = F2_LOG2,                           /*!< F2 gain divisor expressed as power of 2.
                                                                         E.g. if gain divisor is 512 the value
                                                                         must be 9 because 2^9 = 512 */
  .bSpeedBufferSizedppLOG      = STO_FIFO_DEPTH_DPP_LOG2            /*!< bSpeedBufferSizedpp expressed as power of 2.
                                                                         E.g. if gain divisor is 512 the value
                                                                         must be 9 because 2^9 = 512 */
};

/**
  * @brief  SpeedNPosition sensor parameters Motor 2 - State Observer + CORDIC
  */
STO_CORDICParams_t STO_CORDICParamsM2 =
{
  .hC1                         = CORD_C12,                         /*!< State observer constant C1, it can
                                                                        be computed as F1 * Rs(ohm)/(Ls[H] *
                                                                        State observer execution rate [Hz])*/
  .hC2                         = CORD_C22,                         /*!<  Variable containing state observer
                                                                         constant C2, it can be computed as
                                                                         F1 * K1/ State observer execution
                                                                         rate [Hz] being K1 one of the two
                                                                         observer gains   */
  .hC3                         = CORD_C32,                         /*!< State observer constant C3, it can
                                                                        be computed as F1 * Max application
                                                                        speed [rpm] * Motor B-emf constant
                                                                        [Vllrms/krpm] * sqrt(2)/ (Ls [H] *
                                                                        max measurable current (A) * State
                                                                        observer execution rate [Hz])*/
  .hC4                         = CORD_C42,                         /*!< State Observer constant C4, it can
                                                                        be computed as K2 * max measurable
                                                                        current (A) / (Max application speed
                                                                        [rpm] * Motor B-emf constant
                                                                        [Vllrms/krpm] * sqrt(2) * F2 * State
                                                                        observer execution rate [Hz]) being
                                                                        K2 one of the two observer gains  */
  .hC5                         = CORD_C52,                         /*!< State Observer constant C5, it can
                                                                        be computed as F1 * max measurable
                                                                        voltage / (2 * Ls [Hz] * max
                                                                        measurable current * State observer
                                                                        execution rate [Hz]) */
  .hF1                         = CORD_F12,                         /*!< State observer scaling factor F1 */
  .hF2                         = CORD_F22,                         /*!< State observer scaling factor F2 */
  .bSpeedBufferSize01Hz        = CORD_FIFO_DEPTH_01HZ2,            /*!< Depth of FIFO used to average
                                                                        estimated speed exported by
                                                                        SPD_GetAvrgMecSpeed01Hz. It
                                                                        must be an integer number between 1
                                                                        and 64 */
  .bSpeedBufferSizedpp         = CORD_FIFO_DEPTH_DPP2,             /*!< Depth of FIFO used for both averaging
                                                                        estimated speed exported by
                                                                        SPD_GetElSpeedDpp and state
                                                                        observer equations. It must be an
                                                                        integer number between 1 and
                                                                        bSpeedBufferSize01Hz */
  .hVariancePercentage         = CORD_PERCENTAGE_FACTOR2,          /*!< Parameter expressing the maximum
                                                                        allowed variance of speed estimation
                                                                        */
  .bSpeedValidationBand_H      = SPEED_BAND_UPPER_LIMIT2,          /*!< It expresses how much estimated speed
                                                                        can exceed forced stator electrical
                                                                        frequency during start-up without
                                                                        being considered wrong. The
                                                                        measurement unit is 1/16 of forced
                                                                        speed */
  .bSpeedValidationBand_L      = SPEED_BAND_LOWER_LIMIT2,          /*!< It expresses how much estimated speed
                                                                        can be below forced stator electrical
                                                                        frequency during start-up without
                                                                        being considered wrong. The
                                                                        measurement unit is 1/16 of forced
                                                                        speed */
  .hMinStartUpValidSpeed       = OBS_MINIMUM_SPEED2,               /*!< Minimum mechanical speed (expressed in
                                                                        01Hz required to validate the start-up
                                                                        */
  .bStartUpConsistThreshold    = NB_CONSECUTIVE_TESTS2,            /*!< Number of consecutive tests on speed */


  .bReliability_hysteresys     = CORD_MEAS_ERRORS_BEFORE_FAULTS2,            /*!< Number of reliability failed
                                                                                  consecutive tests before a speed
                                                                                  check fault is returned to base class
                                                                                  */
  .hMaxInstantElAcceleration   = CORD_MAX_ACCEL_DPPP2,             /*!< maximum instantaneous electrical
                                                                        acceleration (dpp per control period) */
  .bBemfConsistencyCheck       = CORD_BEMF_CONSISTENCY_TOL2,       /*!< Degree of consistency of the observed
                                                                        back-emfs, it must be an integer
                                                                        number ranging from 1 (very high
                                                                        consistency) down to 64 (very poor
                                                                        consistency) */
  .bBemfConsistencyGain        = CORD_BEMF_CONSISTENCY_GAIN2,      /*!< Gain to be applied when checking
                                                                        back-emfs consistency; default value
                                                                        is 64 (neutral), max value 105
                                                                        (x1,64 amplification), min value 1
                                                                        (/64 attenuation) */
  .hMaxAppPositiveMecSpeed01Hz = (uint16_t)(MAX_APPLICATION_SPEED2*1.15/6.0),  /*!< Maximum positive value
                                                                                    of rotor speed. It's expressed in
                                                                                    tenth of mechanical Hertz. It can be
                                                                                    x1.1 greater than max application*/

  .hF1LOG                      = F1_LOG2,                          /*!< F1 gain divisor expressed as power of 2.
                                                                        E.g. if gain divisor is 512 the value
                                                                        must be 9 because 2^9 = 512 */
  .hF2LOG                      = F2_LOG2,                          /*!< F2 gain divisor expressed as power of 2.
                                                                        E.g. if gain divisor is 512 the value
                                                                        must be 9 because 2^9 = 512 */
  .bSpeedBufferSizedppLOG      = CORD_FIFO_DEPTH_DPP_LOG2          /*!< bSpeedBufferSizedpp expressed as power of 2.
                                                                        E.g. if gain divisor is 512 the value
                                                                        must be 9 because 2^9 = 512 */
};

/**
  * @brief  SpeedNPosition sensor parameters Motor 2 - HFI
  */
HiFreqInj_FPUParams_t HiFreqInjParamsM2 =
{
  .bSpeedBufferSize01Hz = HFI_SPD_BUFFER_DEPTH_01HZ2,
  .hSwapMecSpeed01Hz    = RPM2MEC01HZ(HFI_MINIMUM_SPEED_RPM2),
  .hSTOHFI01Hz          = RPM2MEC01HZ(STO_HFI_RPM_TH2),
  .hHFISTO01Hz          = RPM2MEC01HZ(HFI_STO_RPM_TH2),
  .hHFIRestart01Hz      = RPM2MEC01HZ(HFI_RESTART_RPM_TH2)
};

/**
  * @brief  SpeedNPosition sensor parameters Motor 2 - Virtual Sensor
  */
VirtualSpeedSensorParams_t VirtualSpeedSensorParamsM2 =
{
  .hSpeedSamplingFreqHz = MEDIUM_FREQUENCY_TASK_RATE2,  /*!< Frequency (Hz) at which motor speed is to
                                                             be computed. It must be equal to the frequency
                                                             at which function SPD_CalcAvrgMecSpeed01Hz
                                                             is called.*/
  .hTransitionSteps     = (int16_t)(TF_REGULATION_RATE2*TRANSITION_DURATION2/1000.0),
                            /*< Number of steps to perform
                                the transition phase from CVSS_SPD to other CSPD;
                                if the Transition PHase should last TPH milliseconds,
                                and the FOC Execution Frequency is set to FEF kHz,
                                then hTransitionSteps = TPH * FEF*/
};

/**
  * @brief  RevUp controller parameters Motor 2
  */
RUCPhasesParams_t Phase5_M2 =
{
  .hDurationms        = (uint16_t)PHASE5_DURATION2,          /*!< Duration of the rev up phase expressed in
                                                                  milliseconds.*/
  .hFinalMecSpeed01Hz = (int16_t)(PHASE5_FINAL_SPEED_RPM2/6),/*!< Mechanical speed expressed in 0.1Hz assumed
                                                                  by VSS at the end of the rev up phase.*/
  .hFinalTorque       = (int16_t)PHASE5_FINAL_CURRENT2,      /*!< Motor torque reference imposed by STC at the
                                                                  end of rev up phase. This value represents
                                                                  actually the Iq current expressed in digit.*/
  .pNext              =	MC_NULL
};
RUCPhasesParams_t Phase4_M2 =
{
  .hDurationms        = (uint16_t)PHASE4_DURATION2,          /*!< Duration of the rev up phase expressed in
                                                                  milliseconds.*/
  .hFinalMecSpeed01Hz = (int16_t)(PHASE4_FINAL_SPEED_RPM2/6),/*!< Mechanical speed expressed in 0.1Hz assumed
                                                                  by VSS at the end of the rev up phase.*/
  .hFinalTorque       = (int16_t)PHASE4_FINAL_CURRENT2,      /*!< Motor torque reference imposed by STC at the
                                                                  end of rev up phase. This value represents
                                                                  actually the Iq current expressed in digit.*/
  .pNext              = (void*)&Phase5_M2
};

RUCPhasesParams_t Phase3_M2 =
{
  .hDurationms        = (uint16_t)PHASE3_DURATION2,          /*!< Duration of the rev up phase expressed in
                                                                  milliseconds.*/
  .hFinalMecSpeed01Hz = (int16_t)(PHASE3_FINAL_SPEED_RPM2/6),/*!< Mechanical speed expressed in 0.1Hz assumed
                                                                  by VSS at the end of the rev up phase.*/
  .hFinalTorque       = (int16_t)PHASE3_FINAL_CURRENT2,      /*!< Motor torque reference imposed by STC at the
                                                                  end of rev up phase. This value represents
                                                                  actually the Iq current expressed in digit.*/
  .pNext              = (void*)&Phase4_M2
};

RUCPhasesParams_t Phase2_M2 =
{
  .hDurationms        = (uint16_t)PHASE2_DURATION2,          /*!< Duration of the rev up phase expressed in
                                                                  milliseconds.*/
  .hFinalMecSpeed01Hz = (int16_t)(PHASE2_FINAL_SPEED_RPM2/6),/*!< Mechanical speed expressed in 0.1Hz assumed
                                                                  by VSS at the end of the rev up phase.*/
  .hFinalTorque       = (int16_t)PHASE2_FINAL_CURRENT2,      /*!< Motor torque reference imposed by STC at the
                                                                  end of rev up phase. This value represents
                                                                  actually the Iq current expressed in digit.*/
  .pNext              = (void*)&Phase3_M2
};

RUCPhasesParams_t Phase1_M2 =
{
  .hDurationms        = (uint16_t)PHASE1_DURATION2,          /*!< Duration of the rev up phase expressed in
                                                                  milliseconds.*/
  .hFinalMecSpeed01Hz = (int16_t)(PHASE1_FINAL_SPEED_RPM2/6),/*!< Mechanical speed expressed in 0.1Hz assumed
                                                                  by VSS at the end of the rev up phase.*/
  .hFinalTorque       = (int16_t)PHASE1_FINAL_CURRENT2,      /*!< Motor torque reference imposed by STC at the
                                                                  end of rev up phase. This value represents
                                                                  actually the Iq current expressed in digit.*/
  .pNext              = (void*)&Phase2_M2
};

RevupCtrlParams_t RevupCtrlParamsM2 =
{
  .hRUCFrequencyHz         = MEDIUM_FREQUENCY_TASK_RATE2,    /*!< Frequency expressed in Hz at which the user
                                                                  clocks the RUC calling RUC_Exec method */
  .hStartingMecAngle       = (int16_t)((int32_t)(STARTING_ANGLE_DEG2)* 65536/360),    /*!< Starting angle of programmed rev up.*/
  .pPhaseParam             = &Phase1_M2,                     /*!< Pointer of the first element of
                                                                  phase params that constitute the programmed
                                                                  rev up. Each element is of type
                                                                  RUCPhasesParams_t.*/
  .bFirstAccelerationStage = FIRST_SLESS_ALGO_PHASE2,        /*< Indicate here in which of the rev-up phases
                                                                 the final acceleration is started. The
                                                                 sensor-less algorithm is re-initialized
                                                                 (cleared) when this phase begins.
                                                                 NOTE: The rev up phases counting starts from 0.
                                                                 First phase should be indicated with zero */
  .hMinStartUpValidSpeed   = OBS_MINIMUM_SPEED2,             /*!< Minimum mechanical speed (expressed in
                                                                  01Hz required to validate the start-up */
  .hMinStartUpFlySpeed     = (int16_t)((OBS_MINIMUM_SPEED2)/2),    /*!< Minimum mechanical speed (expressed in
                                                                        01Hz required to validate the on the fly*/
  .bOTFvalidation          = NB_CONSECUTIVE_TESTS2,          /*!< Minimum number of consecutive times the observer
                                                                  is reliable to validate OTF startup*/
  .hFinalRevUpCurrent      = (int16_t)PHASE5_FINAL_CURRENT2, /*!< Final revup current */
  .hOTFSection1Duration    = (uint16_t) 500,                 /*!< On-the-fly section1 duration, millisecond */
  .OTFStartupEnabled       = OTF_STARTUP_EN2                 /*!< ENABLE for OTF strartup otherwise FALSE.*/
};

/**
  * @brief  SpeedNPosition sensor parameters Motor 2 - Encoder
  */
ENCODERParams_t ENCParamsM2 =
{
  /* SW Settings */
  .IRQnb                  = MC_IRQ_SPEEDNPOSFDBK_2,         /*!< MC IRQ number used for TIMx capture update event.*/

  .hPulseNumber           = ENCODER_PPR2*4,                 /*!< Number of pulses per revolution, provided by each
                                                               of the two encoder signals, multiplied by 4 */
  .RevertSignal           = (FunctionalState)ENC_INVERT_SPEED2,
                                                            /*!< To be enabled if measured speed is opposite
                                                                 to real one (ENABLE/DISABLE)*/
  .hSpeedSamplingFreq01Hz = 10*MEDIUM_FREQUENCY_TASK_RATE2, /*!< Frequency (01Hz) at which motor speed is to be
                                                                 computed. It must be equal to the frequency
                                                                 at which function SPD_CalcAvrgMecSpeed01Hz
                                                                 is called.*/

  .bSpeedBufferSize       = ENC_AVERAGING_FIFO_DEPTH2,      /*!< Size of the buffer used to calculate the average
                                                                 speed. It must be <= 16.*/

  .hInpCaptFilter         = ENC_IC_FILTER2,                 /*!< Time filter applied to validate ENCODER sensor
                                                                 capture. This value defines the frequency
                                                                 used to sample ENCODER sensors input and the
                                                                 length of the digital filter applied.
                                                                 The digital filter is made of an event
                                                                 counter in which N events are needed to
                                                                 validate a transition on the input.
                                                                 0000: No filter, sampling is done at fCK_INT.
                                                                 0001: fSAMPLING=fCK_INT , N=2.
                                                                 0010: fSAMPLING=fCK_INT , N=4.
                                                                 0011: fSAMPLING=fCK_INT , N=8.
                                                                 0100: fSAMPLING=fCK_INT/2, N=6.
                                                                 0101: fSAMPLING=fCK_INT/2, N=8.
                                                                 0110: fSAMPLING=fCK_INT/4, N=6.
                                                                 0111: fSAMPLING=fCK_INT/4, N=8.
                                                                 1000: fSAMPLING=fCK_INT/8, N=6.
                                                                 1001: fSAMPLING=fCK_INT/8, N=8.
                                                                 1010: fSAMPLING=fCK_INT/16, N=5.
                                                                 1011: fSAMPLING=fCK_INT/16, N=6.
                                                                 1100: fSAMPLING=fCK_INT/16, N=8.
                                                                 1101: fSAMPLING=fCK_INT/32, N=5.
                                                                 1110: fSAMPLING=fCK_INT/32, N=6.
                                                                 1111: fSAMPLING=fCK_INT/32, N=8 */
  /* HW Settings */
  .TIMx                = ENC_TIMER2,                        /*!< Timer used for ENCODER sensor management.*/

  .RCC_APB1Periph_TIMx = ENC_RCC_PERIPHERAL2,               /*!< RCC Clock related to timer TIMx.
                                                                 (Ex. RCC_APB1Periph_TIM2).*/

  .TIMx_IRQChannel     = ENC_IRQ_CHANNEL2,                  /*!< IRQ Channel related to TIMx.
                                                                 (Ex. TIM2_IRQChannel).*/

  .wTIMxRemapping      = ENC_TIMER_REMAPPING2,              /*!< It remaps TIMx outputs. It must equal to
                                                                 GPIO_PartialRemap_TIMx, GPIO_FullRemap_TIMx or
                                                                 GPIO_NoRemap_TIMx according the HW availability*/

  .hAPort              = ENC_A_GPIO_PORT2,                  /*!< ENCODER sensor A channel GPIO input port (if used,
                                                                 after re-mapping). It must be GPIOx x= A, B, ...*/

  .hAPin               = ENC_A_GPIO_PIN2,                   /*!< ENCODER sensor A channel GPIO input pin (if used,
                                                                 after re-mapping). It must be GPIO_Pin_x x=0,1,.*/

  .hBPort              = ENC_B_GPIO_PORT2,                  /*!< ENCODER sensor B channel GPIO input port (if used,
                                                                 after re-mapping). It must be GPIOx x= A, B, ...*/

  .hBPin               = ENC_B_GPIO_PIN2,                   /*!< ENCODER sensor B channel GPIO input pin (if used,
                                                                 after re-mapping). It must be GPIO_Pin_x x= 0,1,.*/

};

/**
  * @brief  Encoder Alignment Controller parameters Motor 2
  */
EncAlignCtrlParams_t EncAlignCtrlParamsM2 =
{
  .hEACFrequencyHz = MEDIUM_FREQUENCY_TASK_RATE2, /*!< Frequency
                                                       expressed in Hz at which the user
                                                       clocks the EAC calling EAC_Exec method */
  .hFinalTorque    = FINAL_I_ALIGNMENT2,          /*!< Motor torque reference imposed by STC at the
                                                       end of programmed alignment. This value
                                                       represents actually the Iq current expressed in
                                                       digit.*/
  .hElAngle        = ALIGNMENT_ANGLE_S162,        /*!< Mechanical angle of programmed alignment
                                                       expressed in s16degrees.*/
  .hDurationms     = T_ALIGNMENT2,                /*!< Duration of the programmed alignment expressed
                                                       in milliseconds.*/
  .bElToMecRatio   = POLE_PAIR_NUM2,              /*!< Coefficient used to transform electrical to
                                                       mechanical quantities and viceversa. It usually
                                                       coincides with motor pole pairs number*/
};

/**
  * @brief  SpeedNPosition sensor parameters Motor 2 - HALL
  */
HALLParams_t HALLParamsM2 =
{
  /* SW Settings */
  .IRQnb                = MC_IRQ_SPEEDNPOSFDBK_2,      /*!< MC IRQ number used for TIMx capture update event.*/

  .bSensorPlacement     = HALL_SENSORS_PLACEMENT2,     /*!< Define here the mechanical position of the sensors
                                                            with reference to an electrical cycle.
                                                            Allowed values are: DEGREES_120 or DEGREES_60.*/

  .hPhaseShift          = (int16_t)(HALL_PHASE_SHIFT2 * 65536/360),
                                                       /*!< Define here in s16degree the electrical phase shift
                                                            between the low to high transition of signal H1 and
                                                            the maximum of the Bemf induced on phase A.*/

  .hSpeedSamplingFreqHz = MEDIUM_FREQUENCY_TASK_RATE2, /*!< Frequency (Hz) at which motor speed is to
                                                            be computed. It must be equal to the frequency
                                                            at which function SPD_CalcAvrgMecSpeed01Hz
                                                            is called.*/

  .bSpeedBufferSize     = HALL_AVERAGING_FIFO_DEPTH2,  /*!< Size of the buffer used to calculate the average
                                                            speed. It must be less than 18.*/

  .hInpCaptFilter       = HALL_IC_FILTER2,             /*!< Time filter applied to validate HALL sensor capture.
                                                              This value defines the frequency used to sample
                                                              HALL sensors input and the length of the digital
                                                              filter applied. The digital filter is made of an
                                                              event counter in which N events are needed to
                                                              validate a transition on the input.
                                                                 0000: No filter, sampling is done at fCK_INT.
                                                                 0001: fSAMPLING=fCK_INT , N=2.
                                                                 0010: fSAMPLING=fCK_INT , N=4.
                                                                 0011: fSAMPLING=fCK_INT , N=8.
                                                                 0100: fSAMPLING=fCK_INT/2, N=6.
                                                                 0101: fSAMPLING=fCK_INT/2, N=8.
                                                                 0110: fSAMPLING=fCK_INT/4, N=6.
                                                                 0111: fSAMPLING=fCK_INT/4, N=8.
                                                                 1000: fSAMPLING=fCK_INT/8, N=6.
                                                                 1001: fSAMPLING=fCK_INT/8, N=8.
                                                                 1010: fSAMPLING=fCK_INT/16, N=5.
                                                                 1011: fSAMPLING=fCK_INT/16, N=6.
                                                                 1100: fSAMPLING=fCK_INT/16, N=8.
                                                                 1101: fSAMPLING=fCK_INT/32, N=5.
                                                                 1110: fSAMPLING=fCK_INT/32, N=6.
                                                                 1111: fSAMPLING=fCK_INT/32, N=8 */

  /* HW Settings */
  .wTIMClockFreq       = HALL_TIM_CLK2,                /*!< Timer clock frequency express in Hz.*/

  .TIMx                = HALL_TIMER2,                  /*!< Timer used for HALL sensor management.*/

  .RCC_APB1Periph_TIMx = HALL_RCC_PERIPHERAL2,
                                                       /*!< RCC Clock related to timer TIMx.
                                                            (Ex. RCC_APB1Periph_TIM2).*/

  .TIMx_IRQChannel     = HALL_IRQ_CHANNEL2,            /*!< IRQ Channle related to TIMx.
                                                            (Ex. TIM2_IRQChannel).*/


  .wTIMxRemapping      = HALL_TIMER_REMAPPING2,
                                                       /*!< It remaps TIMx outputs. It must equal to
                                                            GPIO_PartialRemap_TIMx, GPIO_FullRemap_TIMx or
                                                            GPIO_NoRemap_TIMx according the HW availability.*/

  .hH1Port             = H1_GPIO_PORT2,
                                                       /*!< HALL sensor H1 channel GPIO input port (if used,
                                                            after re-mapping). It must be GPIOx x= A, B, ...*/

  .hH1Pin              = H1_GPIO_PIN2,                 /*!< HALL sensor H1 channel GPIO output pin (if used,
                                                          after re-mapping). It must be GPIO_Pin_x x= 0, 1,
                                                        */

  .hH2Port             = H2_GPIO_PORT2,
                                                       /*!< HALL sensor H2 channel GPIO input port (if used,
                                                            after re-mapping). It must be GPIOx x= A, B, ...*/

  .hH2Pin              = H2_GPIO_PIN2,                 /*!< HALL sensor H2 channel GPIO output pin (if used,
                                                            after re-mapping). It must be GPIO_Pin_x x= 0, 1,
                                                        */

  .hH3Port             = H3_GPIO_PORT2,
                                                       /*!< HALL sensor H3 channel GPIO input port (if used,
                                                            after re-mapping). It must be GPIOx x= A, B, ...*/

  .hH3Pin              = H3_GPIO_PIN2                  /*!< HALL sensor H3 channel GPIO output pin (if used,
                                                            after re-mapping). It must be GPIO_Pin_x x= 0, 1,
                                                        */
};

/**
  * @brief  Real Bus Voltage sensor parameters Motor 2 - Base class
  */
BusVoltageSensorParams_t RealBusVoltageSensorParamsM2 =
{
  .bSensorType       = REAL_SENSOR,                          /*!< It contains the information about the type
                                                                  of instanced bus voltage sensor object.
                                                                  It can be equal to REAL_SENSOR or
                                                                  VIRTUAL_SENSOR */
  .hConversionFactor = (uint16_t)(MCU_SUPPLY_VOLTAGE/BUS_ADC_CONV_RATIO2),  /*!< It is used to convert bus voltage from
                                                                  u16Volts into real Volts (V).
                                                                  1 u16Volt = 65536/hConversionFactor Volts
                                                                  For real sensors hConversionFactor it's
                                                                  equal to the product between the expected MCU
                                                                  voltage and the partioning network
                                                                  attenuation. For virtual sensors it is
                                                                  assumed to be equal to 500 */
};

/**
  * @brief  Virtual Bus Voltage sensor parameters Motor 2 - Base class
  */
BusVoltageSensorParams_t VirtualBusVoltageSensorParamsM2 =
{
  .bSensorType       = VIRTUAL_SENSOR,                   /*!< It contains the information about the type
                                                              of instanced bus voltage sensor object.
                                                              It can be equal to REAL_SENSOR or
                                                              VIRTUAL_SENSOR */
  .hConversionFactor = 500                               /*!< It is used to convert bus voltage from
                                                              u16Volts into real Volts (V).
                                                              1 u16Volt = 65536/hConversionFactor Volts
                                                              For real sensors hConversionFactor it's
                                                              equal to the product between the expected MCU
                                                              voltage and the partioning network
                                                              attenuation. For virtual sensors it is
                                                              assumed to be equal to 500 */
};

/**
  * @brief  Bus Voltage sensor parameters Motor 2 - Rdivider
  */
RdividerParams_t RdividerParamsM2 =
{
  .bVbusADChannel         = VBUS_CHANNEL2,                    /*!< ADC channel used for conversion of
                                                                   bus voltage. It must be equal to
                                                                   ADC_Channel_x x= 0, ..., 15*/
  .hVbusPort              = VBUS_GPIO_PORT2,                  /*!< GPIO port used by bVbusADChannel.
                                                                   It must be equal to GPIOx x= A, B, ...*/
  .hVbusPin               = VBUS_GPIO_PIN2,                   /*!< GPIO pin used by bVbusChannel. It must
                                                                   be equal to GPIO_Pin_x x= 0, 1, ...*/
  .bVbusSamplingTime      = VBUS_SAMPLING_TIME2,              /*!< Sampling time used for bVbusChannel AD
                                                                   conversion. It must be equal to
                                                                   ADC_SampleTime_xCycles5 x= 1, 7, ...*/
  .hLowPassFilterBW       = VBUS_SW_FILTER_BW_FACTOR2,        /*!< Use this number to configure the Vbus
                                                                   first order software filter bandwidth.
                                                                   hLowPassFilterBW = VBS_CalcBusReading
                                                                   call rate [Hz]/ FilterBandwidth[Hz] */
  .hOverVoltageThreshold  = OVERVOLTAGE_THRESHOLD_d2,         /*!< It represents the over voltage protection
                                                                   intervention threshold. To be expressed
                                                                   in digit through formula:
                                                                   hOverVoltageThreshold (digit) =
                                                                   Over Voltage Threshold (V) * 65536
                                                                                    / hConversionFactor */
  .hUnderVoltageThreshold = UNDERVOLTAGE_THRESHOLD_d2,        /*!< It represents the under voltage protection
                                                                   intervention threshold. To be expressed
                                                                   in digit through formula:
                                                                   hUnderVoltageThreshold (digit) =
                                                                   Under Voltage Threshold (V) * 65536
                                                                   / hConversionFactor */
};

/**
  * @brief  Virtual Bus Voltage sensor parameters Motor 2 - VirtualParams
  */
VirtualParams_t VirtualBusParamsM2 =
{
  .hExpectedVbus_d = 1+(NOMINAL_BUS_VOLTAGE_V2*65536)/500 /*!< Expected Vbus voltage expressed in
                                                               digit*/
};

/**
  * @brief  Real temperature sensor parameters Motor 2 - Base class
  */
TempSensorParams_t RealTempSensorParamsM2=
{
  .bSensorType = REAL_SENSOR,    /*!< It contains the information about the type
                                      of instanced temperature sensor object.
                                      It can be equal to REAL_SENSOR or
                                      VIRTUAL_SENSOR */
};

/**
  * @brief  Virtual temperature sensor parameters Motor 2 - Base class
  */
TempSensorParams_t VirtualTempSensorParamsM2=
{
  .bSensorType = VIRTUAL_SENSOR,    /*!< It contains the information about the type
                                         of instanced temperature sensor object.
                                         It can be equal to REAL_SENSOR or
                                         VIRTUAL_SENSOR */
};

/**
  * @brief  Temperature sensor parameters Motor 2 - NTC
  */
NTCParams_t NTCParamsM2 =
{
  .bTsensADChannel         = TEMP_FDBK_CHANNEL2,                       /*!< ADC channel used for conversion of
                                                                            bus voltage. It must be equal to
                                                                            ADC_Channel_x x= 0, ..., 15*/
  .hTsensPort              = TEMP_FDBK_GPIO_PORT2,                     /*!< GPIO port used by bVbusADChannel.
                                                                            It must be equal to GPIOx x= A, B, ...*/
  .hTsensPin               = TEMP_FDBK_GPIO_PIN2,                      /*!< GPIO pin used by bVbusChannel. It must
                                                                            be equal to GPIO_Pin_x x= 0, 1, ...*/
  .bTsensSamplingTime      = TEMP_SAMPLING_TIME2,                      /*!< Sampling time used for bVbusChannel AD
                                                                            conversion. It must be equal to
                                                                            ADC_SampleTime_xCycles5 x= 1, 7, ...*/
  .hLowPassFilterBW        = TEMP_SW_FILTER_BW_FACTOR2,                /*!< Use this number to configure the
                                                                            temperature first order software filter
                                                                            bandwidth
                                                                            hLowPassFilterBW = TSNS_CalcBusReading
                                                                            call rate [Hz]/ FilterBandwidth[Hz] */
  .hOverTempThreshold      = (uint16_t)(OV_TEMPERATURE_THRESHOLD_d2),  /*!< It represents the over voltage protection
                                                                            intervention threshold. To be expressed
                                                                            in u16Celsius through formula:
                                                                            hOverTempThreshold [u16Celsius] =
                                                                            (V0[V]+dV/dT[V/°C]*(OverTempThreshold[°C]
                                                                             - T0[°C]))* 65536 / MCU supply voltage */
  .hOverTempDeactThreshold = (uint16_t)(OV_TEMPERATURE_THRESHOLD_d2 - OV_TEMPERATURE_HYSTERESIS_d2),
                                                                       /*!< It represents the temperature
                                                                          expressed in u16Celsius below which an
                                                                          active over temperature fault is cleared
                                                                          hOverTempDeactThreshold[u16Celsius]=
                                                                          (V0[V]+dV/dT[V/°C]*(OverTempDeactThresh[°C]
                                                                           - T0[°C]))* 65536 / MCU supply voltage*/
  .hSensitivity            = (uint16_t)(MCU_SUPPLY_VOLTAGE/dV_dT2),    /*!< It is equal to MCU supply voltage [V] /
                                                                            dV/dT [V/°C] and represents the NTC
                                                                            sensitivity */
  .wV0                     = (uint16_t)(V0_V2 *65536/ MCU_SUPPLY_VOLTAGE), /*!< It is equal to V0*65536/MCU supply
                                                                                voltage where V0 is used to convert the
                                                                                temperature into Volts
                                                                                V[V]=V0+dV/dT[V/°C]*(T-T0)[°C]      */
  .hT0                     = T0_C2,                                     /*!< It is equal to T0 [°C], used to convert
                                                                             the temperature into Volts
                                                                             V[V]=V0+dV/dT[V/°C]*(T-T0)[°C]*/
};
/**
  * @brief  Temperature sensor parameters Motor 2 - NTC
  */
VirtualTParams_t VirtualTParamsM2=
{
  .hExpectedTemp_d = 555,  /*!< Value returned by base class method TSNS_GetAvTemp_d */
  .hExpectedTemp_C = VIRTUAL_HEAT_SINK_TEMPERATURE_VALUE   /*!< Expected temperature in degrees
                                                                (value returned by base class
                                                                method TSNS_GetAvTemp_C) */
};

/**
  * @brief  CircleLimitation class parameters Motor 2 - Base Class
  */
CircleLimitationParams_t CircleLimitationParamsM2 =
{
  .hMaxModule          = MAX_MODULE2,     /*!< Circle limitation maximum allowed
                                               module */
  .hCircle_limit_table = MMITABLE2,       /*!< Circle limitation table */
  .bStart_index        = START_INDEX2,    /*!< Circle limitation table indexing
                                               start */
};

/**
  * @brief  FluxWeakeningCtrlClass class parameters Motor 2
  */
FluxWeakeningCtrlParams_t FWParamsM2 =
{
  .hMaxModule             = MAX_MODULE2,                  /*!< Circle limitation maximum allowed module */
  .hDefaultFW_V_Ref       = (int16_t)FW_VOLTAGE_REF2,     /*!< Default flux weakening voltage reference,
                                                               tenth of percentage points*/
  .hDemagCurrent          = ID_DEMAG2,                    /*!< Demagnetization current in s16A:
                                                               Current(Amp) = [Current(s16A) * Vdd micro]/
                                                               [65536 * Rshunt * Aop] */
  .wNominalSqCurr         = ((int32_t)NOMINAL_CURRENT2*(int32_t)NOMINAL_CURRENT2),
                                                         /*!< Squared motor nominal current in (s16A)^2
                                                              where:
                                                              Current(Amp) = [Current(s16A) * Vdd micro]/
                                                              [65536 * Rshunt * Aop] */
  .hVqdLowPassFilterBW    = VQD_SW_FILTER_BW_FACTOR2,     /*!< Use this parameter to configure the Vqd
                                                               first order software filter bandwidth.
                                                               hVqdLowPassFilterBW = FOC_CurrController
                                                               call rate [Hz]/ FilterBandwidth[Hz] in
                                                               case FULL_MISRA_COMPLIANCY is defined.
                                                               On the contrary, if FULL_MISRA_COMPLIANCY
                                                               is not defined, hVqdLowPassFilterBW is
                                                               equal to log with base two of previous
                                                               definition */
  .hVqdLowPassFilterBWLOG = VQD_SW_FILTER_BW_FACTOR_LOG2  /*!< hVqdLowPassFilterBW expressed as power of 2.
                                                               E.g. if gain divisor is 512 the value
                                                               must be 9 because 2^9 = 512 */
};

/**
  * @brief  FeedForwardCtrlClass class parameters Motor 2
  */
FeedForwardCtrlParams_t FFParamsM2 =
{
  .hVqdLowPassFilterBW    = VQD_SW_FILTER_BW_FACTOR2,       /*!< Use this parameter to configure the Vqd
                                                                 first order software filter bandwidth.
                                                                 hVqdLowPassFilterBW = FOC_CurrController
                                                                 call rate [Hz]/ FilterBandwidth[Hz] in
                                                                 case FULL_MISRA_COMPLIANCY is defined.
                                                                 On the contrary, if FULL_MISRA_COMPLIANCY
                                                                 is not defined, hVqdLowPassFilterBW is
                                                                 equal to log with base two of previous
                                                                 definition */
  .wDefConstant_1D        = (int32_t)CONSTANT1_D2,          /*!< Feed forward constant for d axes */
  .wDefConstant_1Q        = (int32_t)CONSTANT1_Q2,          /*!< Feed forward constant for q axes */
  .wDefConstant_2         = (int32_t)CONSTANT2_QD2,         /*!< Constant used by Feed-Forward algorithm*/
  .hVqdLowPassFilterBWLOG = VQD_SW_FILTER_BW_FACTOR_LOG2    /*!< hVqdLowPassFilterBW expressed as power of 2.
                                                                 E.g. if gain divisor is 512 the value
                                                                 must be 9 because 2^9 = 512 */
};

/**
  * @brief  MTPACtrlClass class parameters Motor 2
  */
MTPACtrlParams_t MTPAParamsM2 =
{
  .hSegDiv   = (int16_t)SEGDIV2,        /*!< Segment divisor */
  .wAngCoeff = ANGC2,                   /*!< Angular coefficients table */
  .wOffset   = OFST2,                   /*!< Offsets table */
};

/**
  * @brief  OpenLoop class parameters Motor 2
  */
OpenLoopParams_t OpenLoop_ParamsM2 =
{
  .hDefaultVoltage  = OPEN_LOOP_VOLTAGE_d2, /*!< Default Open loop voltage. */
  .VFMode           = OPEN_LOOP_VF2,        /*!< Default mode. TRUE to enable V/F mode otherwise
                                                 FALSE */
  .hVFOffset        = OPEN_LOOP_OFF2,       /*!< Offset of V/F curve expressed in s16 Voltage
                                                 applied when frequency is zero. */
  .hVFSlope         = OPEN_LOOP_K2          /*!< Slope of V/F curve expressed in s16 Voltage for
                                                 each 0.1Hz of mecchanical frequency increment. */
};

/**
  * @brief  IMHiFreqInj_FPU class parameters Motor 2 - Derived Class - HFI
  */
HiFreqInj_FPU_CtrlParams_t HiFreqInj_FPU_ParamsM2 =
{
  {HFI_NOTCH_0_COEFF2,HFI_NOTCH_1_COEFF2,HFI_NOTCH_2_COEFF2,
   HFI_NOTCH_3_COEFF2,HFI_NOTCH_4_COEFF2},
  {HFI_LP_0_COEFF2,HFI_LP_1_COEFF2,HFI_LP_2_COEFF2,
   HFI_LP_3_COEFF2,HFI_LP_4_COEFF2},
  {HFI_HP_0_COEFF2,HFI_HP_1_COEFF2,HFI_HP_2_COEFF2,
   HFI_HP_3_COEFF2,HFI_HP_4_COEFF2},
  {HFI_DC_0_COEFF2,HFI_DC_1_COEFF2,HFI_DC_2_COEFF2,
   HFI_DC_3_COEFF2,HFI_DC_4_COEFF2},
  .hHiFrFreq      =HFI_FREQUENCY2,         /*!< High frequency signal, Hertz */
  .hHiFrAmplVolts =HFI_AMPLITUDE2,         /*!< High frequency signal amplitude, volts */
  .hPWMFr         =TF_REGULATION_RATE2,    /*!< PWM frequency, Hertz*/
  .hIdhPhase      =HFI_IDH_DELAY2,         /*!< Idh delay*/
  {
    (int16_t)HFI_PID_KP_DEFAULT2,
                          /*!< Default Kp gain, used to initialize Kp gain
                               software variable*/
    (int16_t)HFI_PID_KI_DEFAULT2,
                          /*!< Default Ki gain, used to initialize Ki gain
                               software variable*/
    (uint16_t)HFI_PID_KPDIV2,
                          /*!< Kp gain divisor, used in conjuction with
                               Kp gain allows obtaining fractional values.
                               If FULL_MISRA_C_COMPLIANCY is not defined
                               the divisor is implemented through
                               algebrical right shifts to speed up PI
                               execution. Only in this case this parameter
                               specifies the number of right shifts to be
                               executed */
    (uint16_t)HFI_PID_KIDIV2,
                          /*!< Ki gain divisor, used in conjuction with
                               Ki gain allows obtaining fractional values.
                               If FULL_MISRA_C_COMPLIANCY is not defined
                               the divisor is implemented through
                               algebrical right shifts to speed up PI
                               execution. Only in this case this parameter
                               specifies the number of right shifts to be
                               executed */
    S32_MAX,              /*!< Upper limit used to saturate the integral
                               term given by Ki / Ki divisor * integral of
                               process variable error */
    -S32_MAX,             /*!< Lower limit used to saturate the integral
                               term given by Ki / Ki divisor * integral of
                               process variable error */
    S16_MAX,              /*!< Upper limit used to saturate the PI output */
    -S16_MAX,             /*!< Lower limit used to saturate the PI output */
    (uint16_t)HFI_PID_KPDIV_LOG2,
                          /*!< Kp gain divisor expressed as power of 2.
                               E.g. if gain divisor is 512 the value
                               must be 9 because 2^9 = 512 */
    (uint16_t)HFI_PID_KIDIV_LOG2
                          /*!< Ki gain divisor expressed as power of 2.
                               E.g. if gain divisor is 512 the value
                               must be 9 because 2^9 = 512 */
  },                      /*!< It contains the parameters of the PI
                                  object necessary for track
                                  implementation */

  /* init scan parameters */
  .fDefPLL_KP                  = HFI_PLL_KP_DEFAULT2,     /*!< KP for init scan PLL */
  .fDefPLL_KI                  = HFI_PLL_KI_DEFAULT2,     /*!< KI for init scan PLL */
  .hLockFreq                   = HFI_LOCKFREQ2,           /*!< rotation freq of the initial scan, expressed in dpp */
  .hScanRotationsNo            = HFI_SCANROTATIONSNO2,    /*!< Number of initial rotor scans */
  .hHiFrAmplScanVolts          = HFI_HIFRAMPLSCAN2,       /*!< High frequency signal amplification during scan, volts */
  .hWaitBeforeNS               = HFI_WAITBEFORESN2,       /*!< Wait time before NS detection, expressed in number of half HFI period*/
  .hWaitAfterNS                = HFI_WAITAFTERNS2,        /*!< Wait time after NS detection, expressed in number of half HFI period */
  .hNSMaxDetPoints             = HFI_NSMAXDETPOINTS2,     /*!< NS det points, < 31*/
  .hNSDetPointsSkip            = HFI_NSDETPOINTSSKIP2,    /*!< NS det points skipped*/
  .hDefMinSaturationDifference = HFI_NS_MIN_SAT_DIFF2,    /*!< Minimum saturation difference to validate NS detection.*/
  .debugmode                   = HFI_DEBUG_MODE2,         /*!< HFI in debug mode just angle detection */
  .RevertDirection             = HFI_REVERT_DIRECTION2,   /*!< TRUE reverts the detected direction. */
  .hWaitTrack                  = HFI_WAITTRACK2,          /*!< Reserved */
  .hWaitSynch                  = HFI_WAITSYNCH2,          /*!< Reserved */
  .hStepAngle                  = HFI_STEPANGLE2,          /*!< Reserved */
  .hMaxangleDiff               = HFI_MAXANGLEDIFF2,       /*!< Reserved */
  .fRestartTime                = HFI_RESTARTTIMESEC2      /*!< Reserved */
};

DigitalOutputParams_t R_BrakeParamsM2 =
{
  .hDOutputPort      = R_BRAKE_GPIO_PORT2,             /*!< GPIO output port. It must be equal
                                                            to GPIOx x= A, B, ...*/
  .hDOutputPin       = R_BRAKE_GPIO_PIN2,              /*!< GPIO output pin. It must be equal to
                                                            GPIO_Pin_x x= 0, 1, ...*/
  .bDOutputPolarity  = DISSIPATIVE_BRAKE_POLARITY2     /*!< GPIO output polarity. It must be equal
                                                            to DOutputActiveHigh or DOutputActiveLow */
};

DigitalOutputParams_t DOUT_OCPDisablingParamsM2 =
{
  .hDOutputPort      = OV_CURR_BYPASS_GPIO_PORT2,           /*!< GPIO output port. It must be equal
                                                                 to GPIOx x= A, B, ...*/
  .hDOutputPin       = OV_CURR_BYPASS_GPIO_PIN2,            /*!< GPIO output pin. It must be equal to
                                                                 GPIO_Pin_x x= 0, 1, ...*/
  .bDOutputPolarity  = OVERCURR_PROTECTION_HW_DISABLING2    /*!< GPIO output polarity. It must be equal
                                                                 to DOutputActiveHigh or DOutputActiveLow */
};

PQDParams_t PQD_MotorPowerMeasurementParamsM2 =
{
  .wConvFact = PQD_CONVERSION_FACTOR2/*!< It is the conversion factor used to convert the
                                          variables expressed in digit into variables expressed
                                          in physical measurement unit. It is used to convert
                                          the power in watts. It must be equal to
                                          (1000 * 3 * Vdd)/(sqrt(3) * Rshunt * Aop) */
};

DigitalOutputParams_t ICLDOUTParamsM2 =
{
  .hDOutputPort      = INRUSH_CURRLIMIT_GPIO_PORT2,   /*!< GPIO output port. It must be equal
                                                           to GPIOx x= A, B, ...*/
  .hDOutputPin       = INRUSH_CURRLIMIT_GPIO_PIN2,    /*!< GPIO output pin. It must be equal to
                                                           GPIO_Pin_x x= 0, 1, ...*/
  .bDOutputPolarity  = INRUSH_CURR_LIMITER_POLARITY2  /*!< GPIO output polarity. It must be equal
                                                           to DOutputActiveHigh or DOutputActiveLow */
};

InrushCurrentLimiterParams_t InrushCurrentLimiterParamsM2 =
{
  .hICLFrequencyHz = SPEED_LOOP_FREQUENCY_HZ,             /*!< Frequency expressed in Hz at which the user
                                                               clocks the ICL calling ICL_Exec method */
  .hDurationms     = INRUSH_CURRLIMIT_CHANGE_AFTER_MS2,   /*!< Duration of inrush limiter activation or
                                                               deactivation expressed in milliseconds.*/
};

RampExtMngrParams_t rampExtMngrHFParamsM2 =
{
  .hFrequencyHz = TF_REGULATION_RATE2 /*!< Execution frequency expressed in Hz */
};

#if defined(STM32F30X) /* To be fixed adding dummy defines to avoid conditional compilation (if required) */

/**
  * @brief  Internal OPAMP parameters Motor 2 - three shunt - F30x - Independent Resources
  */
R3_4_F30XOPAMPParams_t R3_4_F30XOPAMPParamsM2 =
{
/* Internal OPAMP1 settings --------------------------------------------------*/
  .wOPAMP_Selection                       = OPAMP1_SELECTION2,                  /*!< First OPAMP selection. It must be
                                                                                     either equal to
                                                                                     OPAMP_Selection_OPAMP1 or
                                                                                     OPAMP_Selection_OPAMP3.*/
  .bOPAMP_InvertingInput_MODE             = OPAMP1_INVERTINGINPUT_MODE2,        /*!< First OPAMP inverting input mode.
                                                                                     It must be either equal to EXT_MODE
                                                                                     or INT_MODE.*/
  .wOPAMP_InvertingInput                  = OPAMP1_INVERTINGINPUT2,             /*!< First OPAMP inverting input pin.

                                                                                     If wOPAMP_Selection is
                                                                                     OPAMP_Selection_OPAMP1 it
                                                                                     must be one of the following:
                                                                                     OPAMP1_InvertingInput_PC5 or
                                                                                     OPAMP1_InvertingInput_PA3 if the
                                                                                     bOPAMP_InvertingInput_MODE is
                                                                                     EXT_MODE or
                                                                                     OPAMP1_InvertingInput_PGA or
                                                                                     OPAMP1_InvertingInput_FOLLOWER if the
                                                                                     bOPAMP_InvertingInput_MODE is
                                                                                     INT_MODE.

                                                                                     If wOPAMP_Selection is
                                                                                     OPAMP_Selection_OPAMP3 it
                                                                                     must be one of the following:
                                                                                     OPAMP3_InvertingInput_PB10 or
                                                                                     OPAMP3_InvertingInput_PB2 if the
                                                                                     bOPAMP_InvertingInput_MODE is
                                                                                     EXT_MODE or
                                                                                     OPAMP3_InvertingInput_PGA or
                                                                                     OPAMP3_InvertingInput_FOLLOWER if the
                                                                                     bOPAMP_InvertingInput_MODE is
                                                                                     INT_MODE.*/
  .hOPAMP_InvertingInput_GPIO_PORT        = OPAMP1_INVERTINGINPUT_GPIO_PORT2,   /*!< First OPAMP inverting input GPIO port
                                                                                     as defined in wOPAMP_InvertingInput.
                                                                                     It must be GPIOx x= A, B, ... if
                                                                                     bOPAMP_InvertingInput_MODE is
                                                                                     EXT_MODE, otherwise can be dummy.*/
  .hOPAMP_InvertingInput_GPIO_PIN         = OPAMP1_INVERTINGINPUT_GPIO_PIN2,    /*!< First OPAMP inverting input GPIO pin
                                                                                     as defined in wOPAMP_InvertingInput.
                                                                                     It must be GPIO_Pin_x x= 0, 1, ... if
                                                                                     bOPAMP_InvertingInput_MODE is
                                                                                     EXT_MODE, otherwise can be dummy.*/
  .wOPAMP_NonInvertingInput_PHA           = OPAMP1_NONINVERTINGINPUT_PHA2,      /*!< First OPAMP non inverting input first
                                                                                     selection.

                                                                                     If wOPAMP_Selection is
                                                                                     OPAMP_Selection_OPAMP1 it
                                                                                     must be one of the following:
                                                                                     OPAMP1_NonInvertingInput_PA7,
                                                                                     OPAMP1_NonInvertingInput_PA5,
                                                                                     OPAMP1_NonInvertingInput_PA3,
                                                                                     OPAMP1_NonInvertingInput_PA1.

                                                                                     If wOPAMP_Selection is
                                                                                     OPAMP_Selection_OPAMP3 it
                                                                                     must be one of the following:
                                                                                     OPAMP3_NonInvertingInput_PB13,
                                                                                     OPAMP3_NonInvertingInput_PA5,
                                                                                     OPAMP3_NonInvertingInput_PA1,
                                                                                     OPAMP3_NonInvertingInput_PB0.*/
  .hOPAMP_NonInvertingInput_PHA_GPIO_PORT = OPAMP1_NONINVERTINGINPUT_PHA_GPIO_PORT2,
                                                                                /*!< First OPAMP non inverting input GPIO
                                                                                     port as defined in
                                                                                     wOPAMP_NonInvertingInput_PHA.
                                                                                     It must be GPIOx x= A, B, ...*/
  .hOPAMP_NonInvertingInput_PHA_GPIO_PIN  = OPAMP1_NONINVERTINGINPUT_PHA_GPIO_PIN2,
                                                                                /*!< First OPAMP non inverting input GPIO
                                                                                     pin as defined in
                                                                                     wOPAMP_NonInvertingInput_PHA.
                                                                                     It must be GPIO_Pin_x x= 0, 1, ...*/
  .wOPAMP_NonInvertingInput_PHB           = OPAMP1_NONINVERTINGINPUT_PHB2,      /*!< First OPAMP non inverting input 2nd
                                                                                     selection.

                                                                                     If wOPAMP_Selection is
                                                                                     OPAMP_Selection_OPAMP1 it
                                                                                     must be one of the following:
                                                                                     OPAMP1_NonInvertingInput_PA7,
                                                                                     OPAMP1_NonInvertingInput_PA5,
                                                                                     OPAMP1_NonInvertingInput_PA3,
                                                                                     OPAMP1_NonInvertingInput_PA1.

                                                                                     If wOPAMP_Selection is
                                                                                     OPAMP_Selection_OPAMP3 it
                                                                                     must be one of the following:
                                                                                     OPAMP3_NonInvertingInput_PB13,
                                                                                     OPAMP3_NonInvertingInput_PA5,
                                                                                     OPAMP3_NonInvertingInput_PA1,
                                                                                     OPAMP3_NonInvertingInput_PB0.

                                                                                     Note: It must be the same PIN of
                                                                                     wOPAMP2_NonInvertingInput_PHB.*/
  .hOPAMP_NonInvertingInput_PHB_GPIO_PORT = OPAMP1_NONINVERTINGINPUT_PHB_GPIO_PORT2,
                                                                                /*!< First OPAMP non inverting input GPIO
                                                                                     port as defined in
                                                                                     wOPAMP_NonInvertingInput_PHB.
                                                                                     It must be GPIOx x= A, B, ...*/
  .hOPAMP_NonInvertingInput_PHB_GPIO_PIN  = OPAMP1_NONINVERTINGINPUT_PHB_GPIO_PIN2,
                                                                                /*!< First OPAMP non inverting input GPIO
                                                                                     pin as defined in
                                                                                     wOPAMP_NonInvertingInput_PHB.
                                                                                     It must be GPIO_Pin_x x= 0, 1, ...*/
  .hOPAMP_Output_GPIO_PORT                = OPAMP1_OUT_GPIO_PORT2,
                                                                                /*!< First OPAMP output GPIO port.
                                                                                     It must be GPIOx x= A, B, ...
                                                                                     Note: Output pin is PA2 for OPAMP1,
                                                                                     PB1 for OPAMP3.*/
  .hOPAMP_Output_GPIO_PIN                 = OPAMP1_OUT_GPIO_PIN2,
                                                                                /*!< First OPAMP output GPIO pin.
                                                                                     It must be GPIO_Pin_x x= 0, 1, ...
                                                                                     Note: Output pin is PA2 for OPAMP1,
                                                                                     PB1 for OPAMP3.*/
/* Internal OPAMP2 settings --------------------------------------------------*/
  .wOPAMP2_Selection                       = OPAMP2_SELECTION2,                  /*!< Second OPAMP selection. It must be
                                                                                      either equal to
                                                                                      OPAMP_Selection_OPAMP2 or
                                                                                      OPAMP_Selection_OPAMP4.*/
  .bOPAMP2_InvertingInput_MODE             = OPAMP2_INVERTINGINPUT_MODE2,        /*!< Second OPAMP inverting input mode.
                                                                                      It must be either equal to EXT_MODE
                                                                                      or INT_MODE.*/
  .wOPAMP2_InvertingInput                  = OPAMP2_INVERTINGINPUT2,             /*!< Second OPAMP inverting input pin.

                                                                                      If wOPAMP2_Selection is
                                                                                      OPAMP_Selection_OPAMP2 it
                                                                                      must be one of the following:
                                                                                      OPAMP2_InvertingInput_PC5 or
                                                                                      OPAMP2_InvertingInput_PA5 if the
                                                                                      bOPAMP_InvertingInput_MODE is
                                                                                      EXT_MODE or
                                                                                      OPAMP2_InvertingInput_PGA or
                                                                                      OPAMP2_InvertingInput_FOLLOWER if the
                                                                                      bOPAMP_InvertingInput_MODE is
                                                                                      INT_MODE.

                                                                                      If wOPAMP2_Selection is
                                                                                      OPAMP_Selection_OPAMP4 it
                                                                                      must be one of the following:
                                                                                      OPAMP4_InvertingInput_PB10 or
                                                                                      OPAMP4_InvertingInput_PD8 if the
                                                                                      bOPAMP2_InvertingInput_MODE is
                                                                                      EXT_MODE or
                                                                                      OPAMP4_InvertingInput_PGA or
                                                                                      OPAMP4_InvertingInput_FOLLOWER if the
                                                                                      InvertingInput_MODE is INT_MODE
                                                                                      bOPAMP2_InvertingInput_MODE is
                                                                                      INT_MODE.*/
  .hOPAMP2_InvertingInput_GPIO_PORT        = OPAMP2_INVERTINGINPUT_GPIO_PORT2,   /*!< 2nd OPAMP inverting input GPIO port
                                                                                      as defined in wOPAMP2_InvertingInput.
                                                                                      It must be GPIOx x= A, B, ...if
                                                                                      bOPAMP2_InvertingInput_MODE is
                                                                                      EXT_MODE, otherwise can be dummy.*/
  .hOPAMP2_InvertingInput_GPIO_PIN         = OPAMP2_INVERTINGINPUT_GPIO_PIN2,    /*!< 2nd OPAMP inverting input GPIO pin as
                                                                                      defined in wOPAMP2_InvertingInput.
                                                                                      It must be GPIO_Pin_x x= 0, 1, ...if
                                                                                      bOPAMP2_InvertingInput_MODE is
                                                                                      EXT_MODE, otherwise can be dummy.*/
  .wOPAMP2_NonInvertingInput_PHB           = OPAMP2_NONINVERTINGINPUT_PHB2,      /*!< 2nd OPAMP non inverting input
                                                                                      first selection.

                                                                                      If wOPAMP2_Selection is
                                                                                      OPAMP_Selection_OPAMP2 it
                                                                                      must be one of the following:
                                                                                      OPAMP2_NonInvertingInput_PD14,
                                                                                      OPAMP2_NonInvertingInput_PB14,
                                                                                      OPAMP2_NonInvertingInput_PB0,
                                                                                      OPAMP2_NonInvertingInput_PA7.

                                                                                      If wOPAMP2_Selection is
                                                                                      OPAMP_Selection_OPAMP4 it
                                                                                      must be one of the following:
                                                                                      OPAMP4_NonInvertingInput_PD11,
                                                                                      OPAMP4_NonInvertingInput_PB11,
                                                                                      OPAMP4_NonInvertingInput_PA4,
                                                                                      OPAMP4_NonInvertingInput_PB13.

                                                                                      Note: It must be the same PIN of
                                                                                      wOPAMP_NonInvertingInput_PHB.*/
  .hOPAMP2_NonInvertingInput_PHB_GPIO_PORT = OPAMP2_NONINVERTINGINPUT_PHB_GPIO_PORT2,
                                                                                 /*!< Second OPAMP non inverting input GPIO
                                                                                      port as defined in
                                                                                      wOPAMP2_NonInvertingInput_PHB.
                                                                                      It must be GPIOx x= A, B, ...*/
  .hOPAMP2_NonInvertingInput_PHB_GPIO_PIN  = OPAMP2_NONINVERTINGINPUT_PHB_GPIO_PIN2,
                                                                                 /*!< Second OPAMP non inverting input GPIO
                                                                                      pin as defined in
                                                                                      wOPAMP2_NonInvertingInput_PHB.
                                                                                      It must be GPIO_Pin_x x= 0, 1, ...*/
  .wOPAMP2_NonInvertingInput_PHC           = OPAMP2_NONINVERTINGINPUT_PHC2,      /*!< Second OPAMP non inverting input 2nd
                                                                                      selection.

                                                                                      If wOPAMP2_Selection is
                                                                                      OPAMP_Selection_OPAMP2 it
                                                                                      must be one of the following:
                                                                                      OPAMP2_NonInvertingInput_PD14,
                                                                                      OPAMP2_NonInvertingInput_PB14,
                                                                                      OPAMP2_NonInvertingInput_PB0,
                                                                                      OPAMP2_NonInvertingInput_PA7.

                                                                                       If wOPAMP2_Selection is
                                                                                      OPAMP_Selection_OPAMP4 it
                                                                                      must be one of the following:
                                                                                      OPAMP4_NonInvertingInput_PD11,
                                                                                      OPAMP4_NonInvertingInput_PB11,
                                                                                      OPAMP4_NonInvertingInput_PA4,
                                                                                      OPAMP4_NonInvertingInput_PB13.*/
  .hOPAMP2_NonInvertingInput_PHC_GPIO_PORT = OPAMP2_NONINVERTINGINPUT_PHC_GPIO_PORT2,
                                                                                 /*!< Second OPAMP non inverting input GPIO
                                                                                      port as defined in
                                                                                      wOPAMP2_NonInvertingInput_PHC.
                                                                                      It must be GPIOx x= A, B, ...*/
  .hOPAMP2_NonInvertingInput_PHC_GPIO_PIN  = OPAMP2_NONINVERTINGINPUT_PHC_GPIO_PIN2,
                                                                                 /*!< Second OPAMP non inverting input GPIO
                                                                                      pin as defined in
                                                                                      wOPAMP2_NonInvertingInput_PHC.
                                                                                      It must be GPIO_Pin_x x= 0, 1, ...*/
  .hOPAMP2_Output_GPIO_PORT                = OPAMP2_OUT_GPIO_PORT2,
                                                                                 /*!< Second OPAMP output GPIO port.
                                                                                      It must be GPIOx x= A, B, ...
                                                                                      Note: Output pin is PA6 for OPAMP2,
                                                                                      PB12 for OPAMP4.*/
  .hOPAMP2_Output_GPIO_PIN                 = OPAMP2_OUT_GPIO_PIN2,
                                                                                 /*!< Second OPAMP output GPIO pin.
                                                                                      It must be GPIO_Pin_x x= 0, 1, ...
                                                                                      Note: Output pin is PA6 for OPAMP2,
                                                                                      PB12 for OPAMP4.*/
/* Common settings -----------------------------------------------------------*/
  .wOPAMP_PGAGain   = OPAMP_PGAGAIN2,                     /*!< It defines the OPAMP PGA gains.
                                                               It must be one of the following:
                                                               OPAMP_OPAMP_PGAGain_2,
                                                               OPAMP_OPAMP_PGAGain_4,
                                                               OPAMP_OPAMP_PGAGain_8,
                                                               OPAMP_OPAMP_PGAGain_16.
                                                               This value is taken in account
                                                               just if wOPAMPx_InvertingInput is
                                                               equal to OPAMP2_InvertingInput_PGA*/
  .OPAMP_PGAConnect = OPAMP_PGACONNECT2,                  /*!< It defines the OPAMP connection
                                                               with an external filter when PGA
                                                               is enabled.
                                                               It must be one of the following:
                                                               OPAMP_PGAConnect_No,
                                                               OPAMP_PGAConnect_IO1,
                                                               OPAMP_PGAConnect_IO2.
                                                               See reference manual RM0316.
                                                               This value is taken in account
                                                               just if wOPAMPx_InvertingInput is
                                                               equal to OPAMP2_InvertingInput_PGA*/
};

/**
  * @brief  Internal OPAMP parameters Motor 2 - three shunt - F30x - Shared Resources
  */
R3_2_F30XOPAMPParams_t R3_2_F30XOPAMPParamsM2 =
{
/* Internal OPAMP1 settings --------------------------------------------------*/
  .bOPAMP_InvertingInput_MODE             = OPAMP1_INVERTINGINPUT_MODE2,        /*!< First OPAMP inverting input mode.
                                                                                     It must be either equal to EXT_MODE
                                                                                     or INT_MODE.*/
  .wOPAMP_InvertingInput                  = OPAMP1_INVERTINGINPUT2,             /*!< First OPAMP inverting input pin.
                                                                                     It must be one of the following:
                                                                                     OPAMP1_InvertingInput_PC5 or
                                                                                     OPAMP1_InvertingInput_PA3 if the
                                                                                     bOPAMP_InvertingInput_MODE is
                                                                                     EXT_MODE or
                                                                                     OPAMP1_InvertingInput_PGA or
                                                                                     OPAMP1_InvertingInput_FOLLOWER if the
                                                                                     bOPAMP_InvertingInput_MODE is
                                                                                     INT_MODE.*/
  .hOPAMP_InvertingInput_GPIO_PORT        = OPAMP1_INVERTINGINPUT_GPIO_PORT2,   /*!< First OPAMP inverting input GPIO port
                                                                                     as defined in wOPAMP_InvertingInput.
                                                                                     It must be GPIOx x= A, B, ... if
                                                                                     bOPAMP_InvertingInput_MODE is
                                                                                     EXT_MODE, otherwise can be dummy.*/
  .hOPAMP_InvertingInput_GPIO_PIN         = OPAMP1_INVERTINGINPUT_GPIO_PIN2,    /*!< First OPAMP inverting input GPIO pin
                                                                                     as defined in wOPAMP_InvertingInput.
                                                                                     It must be GPIO_Pin_x x= 0, 1, ... if
                                                                                     bOPAMP_InvertingInput_MODE is
                                                                                     EXT_MODE, otherwise can be dummy.*/
  .wOPAMP_NonInvertingInput_PHA           = OPAMP1_NONINVERTINGINPUT_PHA2,      /*!< First OPAMP non inverting input first
                                                                                     selection.
                                                                                     It must be one of the following:
                                                                                     OPAMP1_NonInvertingInput_PA7,
                                                                                     OPAMP1_NonInvertingInput_PA5,
                                                                                     OPAMP1_NonInvertingInput_PA3,
                                                                                     OPAMP1_NonInvertingInput_PA1.

                                                                                     Note: It must be the same PIN of
                                                                                     wOPAMP2_NonInvertingInput_PHA.*/
  .hOPAMP_NonInvertingInput_PHA_GPIO_PORT = OPAMP1_NONINVERTINGINPUT_PHA_GPIO_PORT2,
                                                                                /*!< First OPAMP non inverting input GPIO
                                                                                     port as defined in
                                                                                     wOPAMP_NonInvertingInput_PHA.
                                                                                     It must be GPIOx x= A, B, ...*/
  .hOPAMP_NonInvertingInput_PHA_GPIO_PIN  = OPAMP1_NONINVERTINGINPUT_PHA_GPIO_PIN2,
                                                                                /*!< First OPAMP non inverting input GPIO
                                                                                     pin as defined in
                                                                                     wOPAMP_NonInvertingInput_PHA.
                                                                                     It must be GPIO_Pin_x x= 0, 1, ...*/
  .wOPAMP_NonInvertingInput_PHB           = OPAMP1_NONINVERTINGINPUT_PHB2,      /*!< First OPAMP non inverting input 2nd
                                                                                     selection.
                                                                                     It must be one of the following:
                                                                                     OPAMP1_NonInvertingInput_PA7,
                                                                                     OPAMP1_NonInvertingInput_PA5,
                                                                                     OPAMP1_NonInvertingInput_PA3,
                                                                                     OPAMP1_NonInvertingInput_PA1.*/
  .hOPAMP_NonInvertingInput_PHB_GPIO_PORT = OPAMP1_NONINVERTINGINPUT_PHB_GPIO_PORT2,
                                                                                /*!< First OPAMP non inverting input GPIO
                                                                                     port as defined in
                                                                                     wOPAMP_NonInvertingInput_PHB.
                                                                                     It must be GPIOx x= A, B, ...*/
  .hOPAMP_NonInvertingInput_PHB_GPIO_PIN  = OPAMP1_NONINVERTINGINPUT_PHB_GPIO_PIN2,
                                                                                /*!< First OPAMP non inverting input GPIO
                                                                                     pin as defined in
                                                                                     wOPAMP_NonInvertingInput_PHB.
                                                                                     It must be GPIO_Pin_x x= 0, 1, ...*/
/* Internal OPAMP2 settings --------------------------------------------------*/
  .bOPAMP2_InvertingInput_MODE             = OPAMP2_INVERTINGINPUT_MODE2,        /*!< Second OPAMP inverting input mode.
                                                                                      It must be either equal to EXT_MODE
                                                                                      or INT_MODE.*/
  .wOPAMP2_InvertingInput                  = OPAMP2_INVERTINGINPUT2,             /*!< Second OPAMP inverting input pin.
                                                                                      It must be one of the following:
                                                                                      OPAMP3_InvertingInput_PB2 or
                                                                                      OPAMP3_InvertingInput_PB10 if the
                                                                                      bOPAMP_InvertingInput_MODE is
                                                                                      EXT_MODE or
                                                                                      OPAMP3_InvertingInput_PGA or
                                                                                      OPAMP3_InvertingInput_FOLLOWER if the
                                                                                      bOPAMP_InvertingInput_MODE is
                                                                                      INT_MODE.*/
  .hOPAMP2_InvertingInput_GPIO_PORT        = OPAMP2_INVERTINGINPUT_GPIO_PORT2,   /*!< 2nd OPAMP inverting input GPIO port
                                                                                      as defined in wOPAMP2_InvertingInput.
                                                                                      It must be GPIOx x= A, B, ...if
                                                                                      bOPAMP2_InvertingInput_MODE is
                                                                                      EXT_MODE, otherwise can be dummy.*/
  .hOPAMP2_InvertingInput_GPIO_PIN         = OPAMP2_INVERTINGINPUT_GPIO_PIN2,    /*!< 2nd OPAMP inverting input GPIO pin as
                                                                                      defined in wOPAMP2_InvertingInput.
                                                                                      It must be GPIO_Pin_x x= 0, 1, ...if
                                                                                      bOPAMP2_InvertingInput_MODE is
                                                                                      EXT_MODE, otherwise can be dummy.*/
  .wOPAMP2_NonInvertingInput_PHA           = OPAMP2_NONINVERTINGINPUT_PHA2,      /*!< 2nd OPAMP non inverting input
                                                                                      first selection.
                                                                                      It must be one of the following:
                                                                                      OPAMP3_NonInvertingInput_PB0,
                                                                                      OPAMP3_NonInvertingInput_PB13,
                                                                                      OPAMP3_NonInvertingInput_PA1,
                                                                                      OPAMP3_NonInvertingInput_PA5.
                                                                                      Note: It must be the same PIN of
                                                                                      wOPAMP_NonInvertingInput_PHA.*/
  .hOPAMP2_NonInvertingInput_PHA_GPIO_PORT = OPAMP2_NONINVERTINGINPUT_PHB_GPIO_PORT2,
                                                                                 /*!< Second OPAMP non inverting input GPIO
                                                                                      port as defined in
                                                                                      wOPAMP2_NonInvertingInput_PHB.
                                                                                      It must be GPIOx x= A, B, ...*/
  .hOPAMP2_NonInvertingInput_PHA_GPIO_PIN  = OPAMP2_NONINVERTINGINPUT_PHB_GPIO_PIN2,
                                                                                 /*!< Second OPAMP non inverting input GPIO
                                                                                      pin as defined in
                                                                                      wOPAMP2_NonInvertingInput_PHB.
                                                                                      It must be GPIO_Pin_x x= 0, 1, ...*/
  .wOPAMP2_NonInvertingInput_PHC           = OPAMP2_NONINVERTINGINPUT_PHC2,      /*!< Second OPAMP non inverting input 2nd
                                                                                      selection.
                                                                                      It must be one of the following:
                                                                                      OPAMP3_NonInvertingInput_PB0,
                                                                                      OPAMP3_NonInvertingInput_PB13,
                                                                                      OPAMP3_NonInvertingInput_PA1,
                                                                                      OPAMP3_NonInvertingInput_PA5.*/
  .hOPAMP2_NonInvertingInput_PHC_GPIO_PORT = OPAMP2_NONINVERTINGINPUT_PHC_GPIO_PORT2,
                                                                                 /*!< Second OPAMP non inverting input GPIO
                                                                                      port as defined in
                                                                                      wOPAMP2_NonInvertingInput_PHC.
                                                                                      It must be GPIOx x= A, B, ...*/
  .hOPAMP2_NonInvertingInput_PHC_GPIO_PIN  = OPAMP2_NONINVERTINGINPUT_PHC_GPIO_PIN2,
                                                                                 /*!< Second OPAMP non inverting input GPIO
                                                                                      pin as defined in
                                                                                      wOPAMP2_NonInvertingInput_PHC.
                                                                                      It must be GPIO_Pin_x x= 0, 1, ...*/
/* Common settings -----------------------------------------------------------*/
  .wOPAMP_PGAGain  = OPAMP_PGAGAIN2,                     /*!< It defines the OPAMP PGA gains.
                                                              It must be one of the following:
                                                              OPAMP_OPAMP_PGAGain_2,
                                                              OPAMP_OPAMP_PGAGain_4,
                                                              OPAMP_OPAMP_PGAGain_8,
                                                              OPAMP_OPAMP_PGAGain_16.
                                                              This value is taken in account
                                                              just if wOPAMPx_InvertingInput is
                                                              equal to OPAMP2_InvertingInput_PGA*/
  .OPAMP_PGAConnect = OPAMP_PGACONNECT2                   /*!< It defines the OPAMP connection
                                                               with an external filter when PGA
                                                               is enabled.
                                                               It must be one of the following:
                                                               OPAMP_PGAConnect_No,
                                                               OPAMP_PGAConnect_IO1,
                                                               OPAMP_PGAConnect_IO2.
                                                               See reference manual RM0316.
                                                               This value is taken in account
                                                               just if wOPAMPx_InvertingInput is
                                                               equal to OPAMP2_InvertingInput_PGA*/
};

/**
  * @brief  Internal COMP parameters Motor 2 - three shunt - F30x - OCPA
  */
F30XCOMPParams_t R3_F30XOCPACOMPParamsM2 =
{
/* Phase A over current protections ------------------------------------------*/
  .wSelection                   = OCPA_SELECTION2,
                                                              /*!< Internal comparator used for phase A over
                                                                   current protection. It must be
                                                                   COMP_Selection_COMPx x = 1,2,3,4,5,6,7.*/
  .bInvertingInput_MODE         = OCPA_INVERTINGINPUT_MODE2,
                                                              /*!< COMPx inverting input mode. It must be either
                                                                   equal to EXT_MODE or INT_MODE. */
  .wInvertingInput              = OCPA_INVERTINGINPUT2,
                                                              /*!< COMPx inverting input pin. It must be one of
                                                                   the following:
                                                                   COMP1_InvertingInput_PA0,
                                                                   COMP2_InvertingInput_PA2,
                                                                   COMP3_InvertingInput_PD15,
                                                                   COMP3_InvertingInput_PB12,
                                                                   COMP4_InvertingInput_PE8,
                                                                   COMP4_InvertingInput_PB2,
                                                                   COMP5_InvertingInput_PD13,
                                                                   COMP5_InvertingInput_PB10,
                                                                   COMP6_InvertingInput_PD10,
                                                                   COMP6_InvertingInput_PB15 if
                                                                   bInvertingInput_MODE is EXT_MODE or
                                                                   COMPX_InvertingInput_DAC1,
                                                                   COMPX_InvertingInput_DAC2,
                                                                   COMPX_InvertingInput_VREF,
                                                                   COMPX_InvertingInput_VREF_1_4,
                                                                   COMPX_InvertingInput_VREF_1_2,
                                                                   COMPX_InvertingInput_VREF_3_4 if
                                                                   bInvertingInput_MODE is INT_MODE.
                                                                   If bInvertingInput_MODE is EXT_MODE, the
                                                                   only available options are related to the
                                                                   selected COMP in wSelection.*/
  .hInvertingInput_GPIO_PORT    = OCPA_INVERTINGINPUT_GPIO_PORT2,
                                                              /*!< COMPx inverting input GPIO port as defined in
                                                                   wInvertingInput (Just if
                                                                   bInvertingInput_MODE is EXT_MODE).
                                                                   It must be GPIOx x= A, B, ...*/
  .hInvertingInput_GPIO_PIN     = OCPA_INVERTINGINPUT_GPIO_PIN2,
                                                              /*!< COMPx inverting input GPIO pin as defined in
                                                                   wInvertingInput (Just if
                                                                   bInvertingInput_MODE is EXT_MODE).
                                                                   It must be GPIO_Pin_x x= 0, 1, ...*/
  .wNonInvertingInput           = OCPA_NONINVERTINGINPUT2,
                                                              /*!< COMPx non inverting input. It must be one of
                                                                   the following:
                                                                   COMP1_NonInvertingInput_PA1,
                                                                   COMP2_NonInvertingInput_PA3,
                                                                   COMP2_NonInvertingInput_PA7,
                                                                   COMP3_NonInvertingInput_PB14,
                                                                   COMP3_NonInvertingInput_PD14,
                                                                   COMP4_NonInvertingInput_PB0,
                                                                   COMP4_NonInvertingInput_PE7,
                                                                   COMP5_NonInvertingInput_PB13,
                                                                   COMP5_NonInvertingInput_PD12,
                                                                   COMP6_NonInvertingInput_PB11,
                                                                   COMP6_NonInvertingInput_PD11,
                                                                   COMP7_NonInvertingInput_PC1,
                                                                   COMP7_NonInvertingInput_PA0.
                                                                   The only available options are related to the
                                                                   selected COMP in wSelection.*/
  .hNonInvertingInput_GPIO_PORT = OCPA_NONINVERTINGINPUT_GPIO_PORT2,
                                                              /*!< COMPx non inverting input GPIO port as
                                                                   defined in wNonInvertingInput.
                                                                   It must be GPIOx x= A, B, ...*/
  .hNonInvertingInput_GPIO_PIN  = OCPA_NONINVERTINGINPUT_GPIO_PIN2,
                                                              /*!< COMPx non inverting input GPIO pin as defined
                                                                   in wNonInvertingInput.
                                                                   It must be GPIO_Pin_x x= 0, 1, ...*/
  .bOutput_MODE                 = OCPA_OUTPUT_MODE2,
                                                              /*!< COMPx output. It must be either
                                                                   equal to EXT_MODE or INT_MODE. */
  .wOutput                      = OCPA_OUTPUT2,                /*!< COMPx output selection. It must be one of
                                                                   the following:
                                                                   COMP_Output_TIM1BKIN,
                                                                   COMP_Output_TIM1BKIN2,
                                                                   COMP_Output_TIM8BKIN,
                                                                   COMP_Output_TIM8BKIN2,
                                                                   COMP_Output_TIM1BKIN2_TIM8BKIN2.*/
  .hOutput_GPIO_PORT            = OCPA_OUTPUT_GPIO_PORT2,
                                                              /*!< COMPx output GPIO port.
                                                                   It must be GPIOx x= A, B, ...*/
  .hOutput_GPIO_PIN             = OCPA_OUTPUT_GPIO_PIN2,
                                                              /*!< COMPx output GPIO pin as defined.
                                                                   It must be GPIO_Pin_x x= 0, 1, ...*/
  .bOutput_GPIO_AF              = OCPA_OUTPUT_GPIO_AF2,       /*!< COMPx output alternate functions setting.
                                                                   It must be one of the GPIO_AF_x (x=0,1, ...)
                                                                   according to the defined GPIO port and pin.*/
  .wOutputPol                   = OCPA_OUTPUTPOL2,
                                                              /*!< COMPx output polarity. It must be one of
                                                                   the following:
                                                                   COMP_OutputPol_NonInverted,
                                                                   COMP_OutputPol_Inverted.*/
  .wMode                        = OCP_FILTER2                  /*!< COMPx mode it is used to define both
                                                                    the speed (analog filter) and
                                                                    the power consumption of the internal
                                                                    comparator. It must be one of the
                                                                    following:
                                                                    COMP_Mode_HighSpeed,
                                                                    COMP_Mode_MediumSpeed,
                                                                    COMP_Mode_LowPower,
                                                                    COMP_Mode_UltraLowPower.
                                                                    More speed means less filter and more
                                                                    consumption.*/
};

/**
  * @brief  Internal COMP parameters Motor 2 - three shunt - F30x - OCPB
  */
F30XCOMPParams_t R3_F30XOCPBCOMPParamsM2 =
{
/* Phase B over current protections ------------------------------------------*/
 .wSelection                   = OCPB_SELECTION2,
                                                              /*!< Internal comparator used for phase B over
                                                                  current protection. It must be
                                                                  COMP_Selection_COMPx x = 1,2,3,4,5,6,7.*/
 .bInvertingInput_MODE         = OCPB_INVERTINGINPUT_MODE2,
                                                             /*!< COMPx inverting input mode. It must be either
                                                                  equal to EXT_MODE or INT_MODE. */
 .wInvertingInput              = OCPB_INVERTINGINPUT2,
                                                             /*!< COMPx inverting input pin. It must be one of
                                                                  the following:
                                                                  COMP1_InvertingInput_PA0,
                                                                  COMP2_InvertingInput_PA2,
                                                                  COMP3_InvertingInput_PD15,
                                                                  COMP3_InvertingInput_PB12,
                                                                  COMP4_InvertingInput_PE8,
                                                                  COMP4_InvertingInput_PB2,
                                                                  COMP5_InvertingInput_PD13,
                                                                  COMP5_InvertingInput_PB10,
                                                                  COMP6_InvertingInput_PD10,
                                                                  COMP6_InvertingInput_PB15 if
                                                                  bInvertingInput_MODE is EXT_MODE or
                                                                  COMPX_InvertingInput_DAC1,
                                                                  COMPX_InvertingInput_DAC2,
                                                                  COMPX_InvertingInput_VREF,
                                                                  COMPX_InvertingInput_VREF_1_4,
                                                                  COMPX_InvertingInput_VREF_1_2,
                                                                  COMPX_InvertingInput_VREF_3_4 if
                                                                  bInvertingInput_MODE is INT_MODE.
                                                                  If bInvertingInput_MODE is EXT_MODE, the
                                                                  only available options are related to the
                                                                  selected COMP in wSelection.*/
 .hInvertingInput_GPIO_PORT    = OCPB_INVERTINGINPUT_GPIO_PORT2,
                                                             /*!< COMPx inverting input GPIO port as defined in
                                                                  wInvertingInput (Just if
                                                                  bInvertingInput_MODE is EXT_MODE).
                                                                  It must be GPIOx x= A, B, ...*/
 .hInvertingInput_GPIO_PIN     = OCPB_INVERTINGINPUT_GPIO_PIN2,
                                                             /*!< COMPx inverting input GPIO pin as defined in
                                                                  wInvertingInput (Just if
                                                                  bInvertingInput_MODE is EXT_MODE).
                                                                  It must be GPIO_Pin_x x= 0, 1, ...*/
 .wNonInvertingInput           = OCPB_NONINVERTINGINPUT2,
                                                             /*!< COMPx non inverting input. It must be one of
                                                                  the following:
                                                                  COMP1_NonInvertingInput_PA1,
                                                                  COMP2_NonInvertingInput_PA3,
                                                                  COMP2_NonInvertingInput_PA7,
                                                                  COMP3_NonInvertingInput_PB14,
                                                                  COMP3_NonInvertingInput_PD14,
                                                                  COMP4_NonInvertingInput_PB0,
                                                                  COMP4_NonInvertingInput_PE7,
                                                                  COMP5_NonInvertingInput_PB13,
                                                                  COMP5_NonInvertingInput_PD12,
                                                                  COMP6_NonInvertingInput_PB11,
                                                                  COMP6_NonInvertingInput_PD11,
                                                                  COMP7_NonInvertingInput_PC1,
                                                                  COMP7_NonInvertingInput_PA0.
                                                                  The only available options are related to the
                                                                  selected COMP in wSelection.*/
 .hNonInvertingInput_GPIO_PORT = OCPB_NONINVERTINGINPUT_GPIO_PORT2,
                                                             /*!< COMPx non inverting input GPIO port as
                                                                  defined in wNonInvertingInput.
                                                                  It must be GPIOx x= A, B, ...*/
 .hNonInvertingInput_GPIO_PIN  = OCPB_NONINVERTINGINPUT_GPIO_PIN2,
                                                             /*!< COMPx non inverting input GPIO pin as defined
                                                                  in wNonInvertingInput.
                                                                  It must be GPIO_Pin_x x= 0, 1, ...*/
 .bOutput_MODE                 = OCPB_OUTPUT_MODE2,
                                                             /*!< COMPx output. It must be either
                                                                  equal to EXT_MODE or INT_MODE. */
 .wOutput                      = OCPB_OUTPUT2,                /*!< COMPx output selection. It must be one of
                                                                  the following:
                                                                  COMP_Output_TIM1BKIN,
                                                                  COMP_Output_TIM1BKIN2,
                                                                  COMP_Output_TIM8BKIN,
                                                                  COMP_Output_TIM8BKIN2,
                                                                  COMP_Output_TIM1BKIN2_TIM8BKIN2.*/
 .hOutput_GPIO_PORT            = OCPB_OUTPUT_GPIO_PORT2,
                                                             /*!< COMPx output GPIO port.
                                                                  It must be GPIOx x= A, B, ...*/
 .hOutput_GPIO_PIN             = OCPB_OUTPUT_GPIO_PIN2,
                                                             /*!< COMPx output GPIO pin as defined.
                                                                  It must be GPIO_Pin_x x= 0, 1, ...*/
 .bOutput_GPIO_AF              = OCPB_OUTPUT_GPIO_AF2,       /*!< COMPx output alternate functions setting.
                                                                  It must be one of the GPIO_AF_x (x=0,1, ...)
                                                                  according to the defined GPIO port and pin.*/
 .wOutputPol                   = OCPB_OUTPUTPOL2,
                                                             /*!< COMPx output polarity. It must be one of
                                                                  the following:
                                                                  COMP_OutputPol_NonInverted,
                                                                  COMP_OutputPol_Inverted.*/
 .wMode                        = OCP_FILTER2                  /*!< COMPx mode it is used to define both
                                                                  the speed (analog filter) and
                                                                  the power consumption of the internal
                                                                  comparator. It must be one of the
                                                                  following:
                                                                  COMP_Mode_HighSpeed,
                                                                  COMP_Mode_MediumSpeed,
                                                                  COMP_Mode_LowPower,
                                                                  COMP_Mode_UltraLowPower.
                                                                  More speed means less filter and more
                                                                  consumption.*/
};

/**
  * @brief  Internal COMP parameters Motor 2 - three shunt - F30x - OCPC
  */
F30XCOMPParams_t R3_F30XOCPCCOMPParamsM2 =
{
  /* Phase C over current protections ------------------------------------------*/
 .wSelection                   = OCPC_SELECTION2,
                                                              /*!< Internal comparator used for phase C over
                                                                  current protection. It must be
                                                                  COMP_Selection_COMPx x = 1,2,3,4,5,6,7.*/
 .bInvertingInput_MODE         = OCPC_INVERTINGINPUT_MODE2,
                                                             /*!< COMPx inverting input mode. It must be either
                                                                  equal to EXT_MODE or INT_MODE. */
 .wInvertingInput              = OCPC_INVERTINGINPUT2,
                                                             /*!< COMPx inverting input pin. It must be one of
                                                                  the following:
                                                                  COMP1_InvertingInput_PA0,
                                                                  COMP2_InvertingInput_PA2,
                                                                  COMP3_InvertingInput_PD15,
                                                                  COMP3_InvertingInput_PB12,
                                                                  COMP4_InvertingInput_PE8,
                                                                  COMP4_InvertingInput_PB2,
                                                                  COMP5_InvertingInput_PD13,
                                                                  COMP5_InvertingInput_PB10,
                                                                  COMP6_InvertingInput_PD10,
                                                                  COMP6_InvertingInput_PB15 if
                                                                  bInvertingInput_MODE is EXT_MODE or
                                                                  COMPX_InvertingInput_DAC1,
                                                                  COMPX_InvertingInput_DAC2,
                                                                  COMPX_InvertingInput_VREF,
                                                                  COMPX_InvertingInput_VREF_1_4,
                                                                  COMPX_InvertingInput_VREF_1_2,
                                                                  COMPX_InvertingInput_VREF_3_4 if
                                                                  bInvertingInput_MODE is INT_MODE.
                                                                  If bInvertingInput_MODE is EXT_MODE, the
                                                                  only available options are related to the
                                                                  selected COMP in wSelection.*/
 .hInvertingInput_GPIO_PORT    = OCPC_INVERTINGINPUT_GPIO_PORT2,
                                                             /*!< COMPx inverting input GPIO port as defined in
                                                                  wInvertingInput (Just if
                                                                  bInvertingInput_MODE is EXT_MODE).
                                                                  It must be GPIOx x= A, B, ...*/
 .hInvertingInput_GPIO_PIN     = OCPC_INVERTINGINPUT_GPIO_PIN2,
                                                             /*!< COMPx inverting input GPIO pin as defined in
                                                                  wInvertingInput (Just if
                                                                  bInvertingInput_MODE is EXT_MODE).
                                                                  It must be GPIO_Pin_x x= 0, 1, ...*/
 .wNonInvertingInput           = OCPC_NONINVERTINGINPUT2,
                                                             /*!< COMPx non inverting input. It must be one of
                                                                  the following:
                                                                  COMP1_NonInvertingInput_PA1,
                                                                  COMP2_NonInvertingInput_PA3,
                                                                  COMP2_NonInvertingInput_PA7,
                                                                  COMP3_NonInvertingInput_PB14,
                                                                  COMP3_NonInvertingInput_PD14,
                                                                  COMP4_NonInvertingInput_PB0,
                                                                  COMP4_NonInvertingInput_PE7,
                                                                  COMP5_NonInvertingInput_PB13,
                                                                  COMP5_NonInvertingInput_PD12,
                                                                  COMP6_NonInvertingInput_PB11,
                                                                  COMP6_NonInvertingInput_PD11,
                                                                  COMP7_NonInvertingInput_PC1,
                                                                  COMP7_NonInvertingInput_PA0.
                                                                  The only available options are related to the
                                                                  selected COMP in wSelection.*/
 .hNonInvertingInput_GPIO_PORT = OCPC_NONINVERTINGINPUT_GPIO_PORT2,
                                                             /*!< COMPx non inverting input GPIO port as
                                                                  defined in wNonInvertingInput.
                                                                  It must be GPIOx x= A, B, ...*/
 .hNonInvertingInput_GPIO_PIN  = OCPC_NONINVERTINGINPUT_GPIO_PIN2,
                                                             /*!< COMPx non inverting input GPIO pin as defined
                                                                  in wNonInvertingInput.
                                                                  It must be GPIO_Pin_x x= 0, 1, ...*/
 .bOutput_MODE                 = OCPC_OUTPUT_MODE2,
                                                             /*!< COMPx output. It must be either
                                                                  equal to EXT_MODE or INT_MODE. */
 .wOutput                      = OCPC_OUTPUT2,                 /*!< COMPx output selection. It must be one of
                                                                  the following:
                                                                  COMP_Output_TIM1BKIN,
                                                                  COMP_Output_TIM1BKIN2,
                                                                  COMP_Output_TIM8BKIN,
                                                                  COMP_Output_TIM8BKIN2,
                                                                  COMP_Output_TIM1BKIN2_TIM8BKIN2.*/
 .hOutput_GPIO_PORT            = OCPC_OUTPUT_GPIO_PORT2,
                                                             /*!< COMPx output GPIO port.
                                                                  It must be GPIOx x= A, B, ...*/
 .hOutput_GPIO_PIN             = OCPC_OUTPUT_GPIO_PIN2,
                                                             /*!< COMPx output GPIO pin as defined.
                                                                  It must be GPIO_Pin_x x= 0, 1, ...*/
 .bOutput_GPIO_AF              = OCPC_OUTPUT_GPIO_AF2,       /*!< COMPx output alternate functions setting.
                                                                  It must be one of the GPIO_AF_x (x=0,1, ...)
                                                                  according to the defined GPIO port and pin.*/
 .wOutputPol                   = OCPC_OUTPUTPOL2,
                                                             /*!< COMPx output polarity. It must be one of
                                                                  the following:
                                                                  COMP_OutputPol_NonInverted,
                                                                  COMP_OutputPol_Inverted.*/
 .wMode                        = OCP_FILTER2                  /*!< COMPx mode it is used to define both
                                                                  the speed (analog filter) and
                                                                  the power consumption of the internal
                                                                  comparator. It must be one of the
                                                                  following:
                                                                  COMP_Mode_HighSpeed,
                                                                  COMP_Mode_MediumSpeed,
                                                                  COMP_Mode_LowPower,
                                                                  COMP_Mode_UltraLowPower.
                                                                  More speed means less filter and more
                                                                  consumption.*/
};

/**
  * @brief  Internal COMP parameters Motor 2 - three shunt - F30x - OVP
  */
F30XCOMPParams_t F30XOVPCOMPParamsM2 =
{
  /* Over voltage protections ------------------------------------------*/
 .wSelection                   = OVP_SELECTION2,
                                                              /*!< Internal comparator used for over
                                                                  voltage protection. It must be
                                                                  COMP_Selection_COMPx x = 1,2,3,4,5,6,7.*/
 .bInvertingInput_MODE         = OVP_INVERTINGINPUT_MODE2,
                                                             /*!< COMPx inverting input mode. It must be either
                                                                  equal to EXT_MODE or INT_MODE. */
 .wInvertingInput              = OVP_INVERTINGINPUT2,
                                                             /*!< COMPx inverting input pin. It must be one of
                                                                  the following:
                                                                  COMP1_InvertingInput_PA0,
                                                                  COMP2_InvertingInput_PA2,
                                                                  COMP3_InvertingInput_PD15,
                                                                  COMP3_InvertingInput_PB12,
                                                                  COMP4_InvertingInput_PE8,
                                                                  COMP4_InvertingInput_PB2,
                                                                  COMP5_InvertingInput_PD13,
                                                                  COMP5_InvertingInput_PB10,
                                                                  COMP6_InvertingInput_PD10,
                                                                  COMP6_InvertingInput_PB15 if
                                                                  bInvertingInput_MODE is EXT_MODE or
                                                                  COMPX_InvertingInput_DAC1,
                                                                  COMPX_InvertingInput_DAC2,
                                                                  COMPX_InvertingInput_VREF,
                                                                  COMPX_InvertingInput_VREF_1_4,
                                                                  COMPX_InvertingInput_VREF_1_2,
                                                                  COMPX_InvertingInput_VREF_3_4 if
                                                                  bInvertingInput_MODE is INT_MODE.
                                                                  If bInvertingInput_MODE is EXT_MODE, the
                                                                  only available options are related to the
                                                                  selected COMP in wSelection.*/
 .hInvertingInput_GPIO_PORT    = OVP_INVERTINGINPUT_GPIO_PORT2,
                                                             /*!< COMPx inverting input GPIO port as defined in
                                                                  wInvertingInput (Just if
                                                                  bInvertingInput_MODE is EXT_MODE).
                                                                  It must be GPIOx x= A, B, ...*/
 .hInvertingInput_GPIO_PIN     = OVP_INVERTINGINPUT_GPIO_PIN2,
                                                             /*!< COMPx inverting input GPIO pin as defined in
                                                                  wInvertingInput (Just if
                                                                  bInvertingInput_MODE is EXT_MODE).
                                                                  It must be GPIO_Pin_x x= 0, 1, ...*/
 .wNonInvertingInput           = OVP_NONINVERTINGINPUT2,
                                                             /*!< COMPx non inverting input. It must be one of
                                                                  the following:
                                                                  COMP1_NonInvertingInput_PA1,
                                                                  COMP2_NonInvertingInput_PA3,
                                                                  COMP2_NonInvertingInput_PA7,
                                                                  COMP3_NonInvertingInput_PB14,
                                                                  COMP3_NonInvertingInput_PD14,
                                                                  COMP4_NonInvertingInput_PB0,
                                                                  COMP4_NonInvertingInput_PE7,
                                                                  COMP5_NonInvertingInput_PB13,
                                                                  COMP5_NonInvertingInput_PD12,
                                                                  COMP6_NonInvertingInput_PB11,
                                                                  COMP6_NonInvertingInput_PD11,
                                                                  COMP7_NonInvertingInput_PC1,
                                                                  COMP7_NonInvertingInput_PA0.
                                                                  The only available options are related to the
                                                                  selected COMP in wSelection.*/
 .hNonInvertingInput_GPIO_PORT = OVP_NONINVERTINGINPUT_GPIO_PORT2,
                                                             /*!< COMPx non inverting input GPIO port as
                                                                  defined in wNonInvertingInput.
                                                                  It must be GPIOx x= A, B, ...*/
 .hNonInvertingInput_GPIO_PIN  = OVP_NONINVERTINGINPUT_GPIO_PIN2,
                                                             /*!< COMPx non inverting input GPIO pin as defined
                                                                  in wNonInvertingInput.
                                                                  It must be GPIO_Pin_x x= 0, 1, ...*/
 .bOutput_MODE                 = OVP_OUTPUT_MODE2,
                                                             /*!< COMPx output. It must be either
                                                                  equal to EXT_MODE or INT_MODE. */
 .wOutput                      = OVP_OUTPUT2,                 /*!< COMPx output selection. It must be one of
                                                                  the following:
                                                                  COMP_Output_TIM1BKIN,
                                                                  COMP_Output_TIM1BKIN2,
                                                                  COMP_Output_TIM8BKIN,
                                                                  COMP_Output_TIM8BKIN2,
                                                                  COMP_Output_TIM1BKIN2_TIM8BKIN2.*/
 .hOutput_GPIO_PORT            = OVP_OUTPUT_GPIO_PORT2,
                                                             /*!< COMPx output GPIO port.
                                                                  It must be GPIOx x= A, B, ...*/
 .hOutput_GPIO_PIN             = OVP_OUTPUT_GPIO_PIN2,
                                                             /*!< COMPx output GPIO pin as defined.
                                                                  It must be GPIO_Pin_x x= 0, 1, ...*/
 .bOutput_GPIO_AF              = OVP_OUTPUT_GPIO_AF2,       /*!< COMPx output alternate functions setting.
                                                                  It must be one of the GPIO_AF_x (x=0,1, ...)
                                                                  according to the defined GPIO port and pin.*/
 .wOutputPol                   = OVP_OUTPUTPOL2,
                                                             /*!< COMPx output polarity. It must be one of
                                                                  the following:
                                                                  COMP_OutputPol_NonInverted,
                                                                  COMP_OutputPol_Inverted.*/
 .wMode                        = OVP_FILTER2                  /*!< COMPx mode it is used to define both
                                                                  the speed (analog filter) and
                                                                  the power consumption of the internal
                                                                  comparator. It must be one of the
                                                                  following:
                                                                  COMP_Mode_HighSpeed,
                                                                  COMP_Mode_MediumSpeed,
                                                                  COMP_Mode_LowPower,
                                                                  COMP_Mode_UltraLowPower.
                                                                  More speed means less filter and more
                                                                  consumption.*/
};

/**
  * @brief  Current sensor parameters Motor 2 - three shunt - F30x - - Independent Resources
  */
R3_4_F30XParams_t R3_4_F30XParamsM2 =
{
  .wADC_Clock_Divider = ADC_CLOCK_DIVIDER2,                /*!<  AHB clock prescaling factor for
                                                                 ADC peripheral. It must be
                                                                 RCC_ADC12PLLCLK_Divx or
                                                                 RCC_ADC34PLLCLK_Divx
                                                                 x = 1, 2, 4, 6, 8, ... */
  .wAHBPeriph         = ADC_AHBPERIPH2,                     /*!< AHB periph used. It must be
                                                                 RCC_AHBPeriph_ADC12 or
                                                                 RCC_AHBPeriph_ADC34. */
  .bTim_Clock_Divider = TIM_CLOCK_DIVIDER2,                  /* System clock divider for Advanded Timer */
/* Dual MC parameters --------------------------------------------------------*/
  .bInstanceNbr     = 2,                                    /*!< Instance number. Necessary to
                                                                 properly synchronize TIM8 with
                                                                 TIM1 at peripheral initializations
                                                                  */
  .Tw               = MAX_TWAIT2,                            /*!< It is used for switching the context
                                                                  in dual MC. It contains biggest delay
                                                                  (expressed in counter ticks) between
                                                                  the counter crest and ADC latest
                                                                  trigger  */
  .bFreqRatio       = FREQ_RATIO,                         /*!< It is used in case of dual MC to
                                                               synchronize TIM1 and TIM8. It has
                                                               effect only on the second instanced
                                                               object and must be equal to the
                                                               ratio between the two PWM frequencies
                                                               (higher/lower). Supported values are
                                                               1, 2 or 3 */
  .bIsHigherFreqTim = FREQ_RELATION2,                           /*!< When bFreqRatio is greather than 1
                                                                     this param is used to indicate if this
                                                                     instance is the one with the highest
                                                                     frequency. Allowed value are: HIGHER_FREQ
                                                                     or LOWER_FREQ */
  .IRQnb            = MC_IRQ_PWMNCURRFDBK_2,                     /*!< MC IRQ number used as TIMx Update
                                                                      event */
/* Current reading A/D Conversions initialization -----------------------------*/
  .bIaChannel       = PHASE_U_CURR_CHANNEL2,                  /*!< ADC channel used for conversion of
                                                                   current Ia. It must be equal to
                                                                   ADC_Channel_x x= 0, ..., 15*/
  .hIaPort          = PHASE_U_GPIO_PORT2,                  /*!< GPIO port used by hIaChannel. It must
                                                                be equal to GPIOx x= A, B, ...*/
  .hIaPin           = PHASE_U_GPIO_PIN2,                      /*!< GPIO pin used by hIaChannel. It must be
                                                                   equal to GPIO_Pin_x x= 0, 1, ...*/
  .b_IaSamplingTime = SAMPLING_TIME_SEL2,               /*!< Sampling time used to convert hIaChannel.
                                                             It must be equal to ADC_SampleTime_xCycles5
                                                             x= 1, 7, ...*/
  .bIbChannel       = PHASE_V_CURR_CHANNEL2,                  /*!< ADC channel used for conversion of
                                                                   current Ib. It must be equal to
                                                                   ADC_Channel_x x= 0, ..., 15*/
  .hIbPort          = PHASE_V_GPIO_PORT2,                /*!< GPIO port used by hIbChannel. It must
                                                              be equal to GPIOx x= A, B, ...*/
  .hIbPin           = PHASE_V_GPIO_PIN2,                      /*!< GPIO pin used by hIaChannel. It must be
                                                                   equal to GPIO_Pin_x x= 0, 1, ...*/
  .b_IbSamplingTime = SAMPLING_TIME_SEL2,                /*!< Sampling time used to convert hIbChannel.
                                                              It must be equal to ADC_SampleTime_xCycles5
                                                              x= 1, 7, ...*/
  .bIcChannel       = PHASE_W_CURR_CHANNEL2,                  /*!< ADC channel used for conversion of
                                                                   current Ia. It must be equal to
                                                                   ADC_Channel_x x= 0, ..., 15*/
  .hIcPort          = PHASE_W_GPIO_PORT2,                /*!< GPIO port used by hIaChannel. It must
                                                              be equal to GPIOx x= A, B, ...*/
  .hIcPin           = PHASE_W_GPIO_PIN2,                      /*!< GPIO pin used by hIaChannel. It must be
                                                                   equal to GPIO_Pin_x x= 0, 1, ...*/
  .b_IcSamplingTime = SAMPLING_TIME_SEL2,            /*!< Sampling time used to convert hIaChannel.
                                                          It must be equal to ADC_SampleTime_xCycles5
                                                          x= 1, 7, ...*/

/* PWM generation parameters --------------------------------------------------*/
  .hDeadTime          = DEAD_TIME_COUNTS2,                    /*!< Dead time in number of TIM clock
                                                                   cycles. If CHxN are enabled, it must
                                                                   contain the dead time to be generated
                                                                   by the microcontroller, otherwise it
                                                                   expresses the maximum dead time
                                                                   generated by driving network */
  .bRepetitionCounter = REP_COUNTER2,                         /*!< It expresses the number of PWM
                                                                   periods to be elapsed before compare
                                                                   registers are updated again. In
                                                                   particular:
                                                                   RepetitionCounter= (2* #PWM periods)-1*/
  .hTafter            = TW_AFTER2,                             /*!< It is the sum of dead time plus max
                                                                    value between rise time and noise time
                                                                    express in number of TIM clocks.*/
  .hTbefore           = TW_BEFORE2,                           /*!< It is the sampling time express in
                                                                   number of TIM clocks.*/
  .TIMx               = PWM_TIMER_SELECTION2,                               /*!< It contains the pointer to the timer
                                                                                 used for PWM generation. */
/* PWM Driving signals initialization ----------------------------------------*/

  .wTIM1Remapping = PWM_TIMER_REMAPPING2,                        /*!< Used only for instance #1, it
                                                                      remaps TIM1 outputs. It must equal to
                                                                      GPIO_PartialRemap_TIM1 or
                                                                      GPIO_FullRemap_TIM1 or
                                                                      GPIO_NoRemap_TIM1 */
  .hCh1Polarity   = PHASE_UH_POLARITY2,   	              /*!< Channel 1 (high side) output polarity,
                                                               it must be TIM_OCPolarity_High or
                                                               TIM_OCPolarity_Low */
  .hCh1Port       = PHASE_UH_GPIO_PORT2,                 /*!< Channel 1 (high side) GPIO output
                                                              port (if used, after re-mapping).
                                                              It must be GPIOx x= A, B, ...*/
  .hCh1Pin        = PHASE_UH_GPIO_PIN2,                  /*!< Channel 1 (high side) GPIO output pin
                                                              (if used, after re-mapping). It must be
                                                               GPIO_Pin_x x= 0, 1, ...*/
  .hCh1IdleState  = BRAKE_UH_POLARITY2,                   /*!< Channel 1 (high side) state (low or high)
                                                               when TIM peripheral is in Idle state.
                                                               It must be TIM_OCIdleState_Set or
                                                               TIM_OCIdleState_Reset*/

  . hCh2Polarity  = PHASE_VH_POLARITY2,                   /*!< Channel 2 (high side) output polarity,
                                                               it must be TIM_OCPolarity_High or
                                                               TIM_OCPolarity_Low */
  .hCh2Port       = PHASE_VH_GPIO_PORT2,                               /*!< Channel 2 (high side) GPIO output
                                                                            port (if used, after re-mapping).
                                                                            It must be GPIOx x= A, B, ...*/
  .hCh2Pin        = PHASE_VH_GPIO_PIN2,                       /*!< Channel 2 (high side) GPIO output pin
                                                                   (if used, after re-mapping). It must be
                                                                   GPIO_Pin_x x= 0, 1, ...*/
  .hCh2IdleState  = BRAKE_VH_POLARITY2,              /*!< Channel 2 (high side) state (low or high)
                                                          when TIM peripheral is in Idle state.
                                                          It must be TIM_OCIdleState_Set or
                                                          TIM_OCIdleState_Reset*/

  .hCh3Polarity   = PHASE_WH_POLARITY2,            	 /*!< Channel 3 (high side) output polarity,
                                                          it must be TIM_OCPolarity_High or
                                                          TIM_OCPolarity_Low */
  .hCh3Port       = PHASE_WH_GPIO_PORT2,                               /*!< Channel 3 (high side) GPIO output
                                                                            port (if used, after re-mapping).
                                                                            It must be GPIOx x= A, B, ...*/
  .hCh3Pin        = PHASE_WH_GPIO_PIN2,                        /*!< Channel 3 (high side) GPIO output pin
                                                                    (if used, after re-mapping). It must be
                                                                    GPIO_Pin_x x= 0, 1, ...*/
  .hCh3IdleState  = BRAKE_WH_POLARITY2,              /*!< Channel 3 (high side) state (low or high)
                                                          when TIM peripheral is in Idle state.
                                                          It must be TIM_OCIdleState_Set or
                                                          TIM_OCIdleState_Reset*/

  .LowSideOutputs = (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING2,  /*!< Low side or enabling signals
                                                                                generation method are defined
                                                                                here.*/

  .hCh1NPolarity  = PHASE_UL_POLARITY2,               /*!< Channel 1N (low side) output polarity,
                                                           it must be TIM_OCNPolarity_High or
                                                           TIM_OCNPolarity_Low */
  .hCh1NPort      = PHASE_UL_GPIO_PORT2,                                 /*!< Channel 1N (low side) GPIO output
                                                                              port (if used, after re-mapping).
                                                                              It must be GPIOx x= A, B, ...*/
  .hCh1NPin       = PHASE_UL_GPIO_PIN2,                       /*!< Channel 1N (low side) GPIO output pin
                                                                   (if used, after re-mapping). It must be
                                                                   GPIO_Pin_x x= 0, 1, ...*/
  .hCh1NIdleState = BRAKE_UL_POLARITY2,              /*!< Channel 1N (low side) state (low or high)
                                                          when TIM peripheral is in Idle state.
                                                          It must be TIM_OCNIdleState_Set or
                                                          TIM_OCNIdleState_Reset*/


  .hCh2NPolarity  = PHASE_VL_POLARITY2,                 /*!< Channel 2N (low side) output polarity,
                                                             it must be TIM_OCNPolarity_High or
                                                             TIM_OCNPolarity_Low */
  .hCh2NPort      = PHASE_VL_GPIO_PORT2,                                 /*!< Channel 2N (low side) GPIO output
                                                                              port (if used, after re-mapping).
                                                                              It must be GPIOx x= A, B, ...*/
  .hCh2NPin       = PHASE_VL_GPIO_PIN2,                     /*!< Channel 2N (low side) GPIO output pin
                                                                 (if used, after re-mapping). It must be
                                                                 GPIO_Pin_x x= 0, 1, ...*/
  .hCh2NIdleState = BRAKE_VL_POLARITY2,              /*!< Channel 2N (low side) state (low or high)
                                                          when TIM peripheral is in Idle state.
                                                          It must be TIM_OCNIdleState_Set or
                                                          TIM_OCNIdleState_Reset*/

  .hCh3NPolarity  = PHASE_WL_POLARITY2,             /*!< Channel 3N (low side) output polarity,
                                                         it must be TIM_OCNPolarity_High or
                                                         TIM_OCNPolarity_Low */
  .hCh3NPort      = PHASE_WL_GPIO_PORT2,                               /*!< Channel 3N (low side)  GPIO output
                                                                            port (if used, after re-mapping).
                                                                            It must be GPIOx x= A, B, ...*/
  .hCh3NPin       = PHASE_WL_GPIO_PIN2,                    /*!< Channel 3N (low side)  GPIO output pin
                                                               (if used, after re-mapping). It must be
                                                               GPIO_Pin_x x= 0, 1, ...*/
  .hCh3NIdleState = BRAKE_WL_POLARITY2,              /*!< Channel 3N (low side) state (low or high)
                                                          when TIM peripheral is in Idle state.
                                                          It must be TIM_OCNIdleState_Set or
                                                          TIM_OCNIdleState_Reset*/
/* PWM Driving signals initialization ----------------------------------------*/
  .bBKINMode     = BKIN_MODE2,                         /*!< It defines the modality of emergency
                                                            input. It must be any of the
                                                            the following:
                                                            NONE - feature disabled.
                                                            INT_MODE - Internal comparator used
                                                            as source of emergency event.
                                                            EXT_MODE - External comparator used
                                                            as source of emergency event.
                                                            EXTINT_MODE - The or-ed signal
                                                            between internal and external
                                                            comparator is used as source event.*/
  .hBKINPolarity = EMSTOP_ACTIVE_HIGH,                 /*!< Emergency Stop (BKIN) input polarity,
                                                            it must be TIM_BreakPolarity_Low or
                                                            TIM_BreakPolarity_High */
  .hBKINPort     = EMERGENCY_STOP_GPIO_PORT2,             /*!< Emergency Stop (BKIN) GPIO input
                                                            port (if used, after re-mapping).
                                                            It must be GPIOx x= A, B, ...*/
  .hBKINPin      = EMERGENCY_STOP_GPIO_PIN2,              /*!< Emergency Stop (BKIN) GPIO input pin
                                                               (if used, after re-mapping). It must be
                                                                GPIO_Pin_x x= 0, 1, ...*/
/* Alternate functions definition --------------------------------------------*/
  .bCh1AF   = PHASE_UH_GPIO_AF2,                   /*!< Channel 1 (high side) alternate
                                                        functions setting. It must be one
                                                        of the GPIO_AF_x (x=0,1, ...)
                                                        according to the defined GPIO port
                                                        and pin.*/
  .bCh2AF   = PHASE_VH_GPIO_AF2,                   /*!< Channel 2 (high side) alternate
                                                        functions setting. It must be one
                                                        of the GPIO_AF_x (x=0,1, ...)
                                                        according to the defined GPIO port
                                                        and pin.*/
  .bCh3AF   = PHASE_WH_GPIO_AF2,                   /*!< Channel 3 (high side) alternate
                                                        functions setting. It must be one
                                                        of the GPIO_AF_x (x=0,1, ...)
                                                        according to the defined GPIO port
                                                        and pin.*/
  .bCh1NAF  = PHASE_UL_GPIO_AF2,                   /*!< Channel 1 (low side) alternate
                                                        functions setting. It must be one
                                                        of the GPIO_AF_x (x=0,1, ...)
                                                        according to the defined GPIO port
                                                        and pin.*/
  .bCh2NAF  = PHASE_VL_GPIO_AF2,                   /*!< Channel 2 (low side) alternate
                                                        functions setting. It must be one
                                                        of the GPIO_AF_x (x=0,1, ...)
                                                        according to the defined GPIO port
                                                        and pin.*/
  .bCh3NAF  = PHASE_WL_GPIO_AF2,                   /*!< Channel 3 (low side) alternate
                                                        functions setting. It must be one
                                                        of the GPIO_AF_x (x=0,1, ...)
                                                        according to the defined GPIO port
                                                        and pin.*/
  .bBKINAF  = BRKIN_GPIO_AF2,                      /*!< Emergency Stop (BKIN) alternate
                                                        functions setting. It must be one
                                                        of the GPIO_AF_x (x=0,1, ...)
                                                        according to the defined GPIO port
                                                        and pin.*/
  .bBKIN2AF = BRKIN2_GPIO_AF2,                     /*!< Emergency Stop (BKIN2) alternate
                                                        functions setting. It must be one
                                                        of the GPIO_AF_x (x=0,1, ...)
                                                        according to the defined GPIO port
                                                        and pin.*/
/* Emergency input (BKIN2) signal initialization -----------------------------*/
  .bBKIN2Mode     = BKIN2_MODE2,                        /*!< It defines the modality of emergency
                                                             input 2. It must be any of the
                                                             the following:
                                                             NONE - feature disabled.
                                                             INT_MODE - Internal comparator used
                                                             as source of emergency event.
                                                             EXT_MODE - External comparator used
                                                             as source of emergency event.
                                                             EXTINT_MODE - The or-ed signal
                                                             between internal and external
                                                             comparator is used as source event.*/
  .hBKIN2Polarity = OVERCURR_FEEDBACK_POLARITY2,         /*!< Emergency Stop (BKIN2) input polarity,
                                                              it must be TIM_Break2Polarity_Low or
                                                              TIM_Break2Polarity_High */
  .hBKIN2Port     = EMERGENCY2_STOP_GPIO_PORT2,          /*!< Emergency Stop (BKIN) GPIO input
                                                              port (if used, after re-mapping).
                                                              It must be GPIOx x= A, B, ...*/
  .hBKIN2Pin      = EMERGENCY2_STOP_GPIO_PIN2,           /*!< Emergency Stop (BKIN) GPIO input pin
                                                              (if used, after re-mapping). It must be
                                                              GPIO_Pin_x x= 0, 1, ...*/
/* Filtering settings of the emergency inputs --------------------------------*/
  .bBKINFilter  = BKIN1_FILTER2,                         /*!< Emergency Stop (BKIN) digital
                                                              filter. See TIMx_BDTR (BKF bits)
                                                              definition in the reference
                                                              manual.*/
  .bBKIN2Filter = BKIN2_FILTER2,                         /*!< Emergency Stop (BKIN2) digital
                                                              filter. See TIMx_BDTR (BKF bits)
                                                              definition in the reference
                                                              manual.*/
/* Internal OPAMP common settings --------------------------------------------*/
  .pOPAMPParams = R3_4_OPAMPPARAMSM2,                 /*!< Pointer to the OPAMP params struct.
                                                           It must be MC_NULL if internal
                                                           OPAMP are not used.*/
/* Internal COMP settings ----------------------------------------------------*/
  .pOCPA_COMPParams = OCPA_COMPPARAMSM2,
                                                          /*!< Pointer to the COMP params struct
                                                               used for overcurrent protection of
                                                               phase A.
                                                               It must be MC_NULL if internal COMP
                                                               are not used for OCP protection.*/
  .pOCPB_COMPParams = OCPB_COMPPARAMSM2,
                                                          /*!< Pointer to the COMP params struct
                                                               used for overcurrent protection of
                                                               phase B.
                                                               It must be MC_NULL if internal COMP
                                                               are not used for OCP protection.*/
  .pOCPC_COMPParams = OCPC_COMPPARAMSM2,
                                                          /*!< Pointer to the COMP params struct
                                                               used for overcurrent protection of
                                                               phase C.
                                                               It must be MC_NULL if internal COMP
                                                               are not used for OCP protection.*/
  .pOVP_COMPParams  = OVP_COMPPARAMSM2,
                                                       /*!< Pointer to the COMP params struct
                                                            used for overvoltage protection.
                                                            It must be MC_NULL if internal COMP
                                                            are not used for OVP protection.*/
/* DAC settings --------------------------------------------------------------*/
  .hDAC_OCP_Threshold = OCPREF,                             /*!< Value of analog reference expressed
                                                                 as 16bit unsigned integer.
                                                                 Ex. 0 = 0V 65536 = VDD_DAC.*/
  .hDAC_OVP_Threshold = OVPREF,                             /*!< Value of analog reference expressed
                                                                 as 16bit unsigned integer.
                                                                 Ex. 0 = 0V 65536 = VDD_DAC.*/
/* Regular conversion --------------------------------------------------------*/
  .regconvADCx = REGCONVADC2                         /*!< ADC peripheral used for regular
                                                          conversion.*/
};

/**
  * @brief  Current sensor parameters Motor 2 - three shunt - F30x - - Shared Resources
  */
R3_2_F30XParams_t R3_2_F30XParamsM2 =
{
  .wADC_Clock_Divider = ADC_CLOCK_DIVIDER2,                /*!<  AHB clock prescaling factor for
                                                                 ADC peripheral. It must be
                                                                 RCC_ADC12PLLCLK_Divx or
                                                                 RCC_ADC34PLLCLK_Divx
                                                                 x = 1, 2, 4, 6, 8, ... */
  .wAHBPeriph         = ADC_AHBPERIPH2,                    /*!< AHB periph used. It must be
                                                                RCC_AHBPeriph_ADC12 or
                                                                RCC_AHBPeriph_ADC34 or
                                                                RCC_AHBPeriph_ADC12 | RCC_AHBPeriph_ADC34.*/
  .bTim_Clock_Divider = TIM_CLOCK_DIVIDER2,                  /* System clock divider for Advanded Timer */
/* Dual MC parameters --------------------------------------------------------*/
  .bInstanceNbr     = 2,                                    /*!< Instance number. Necessary to
                                                                 properly synchronize TIM8 with
                                                                 TIM1 at peripheral initializations
                                                                  */
  .Tw               = MAX_TWAIT2,                            /*!< It is used for switching the context
                                                                  in dual MC. It contains biggest delay
                                                                  (expressed in counter ticks) between
                                                                  the counter crest and ADC latest
                                                                  trigger  */
  .bFreqRatio       = FREQ_RATIO,                                    /*!< It is used in case of dual MC to
                                                                          synchronize TIM1 and TIM8. It has
                                                                          effect only on the second instanced
                                                                          object and must be equal to the
                                                                          ratio between the two PWM frequencies
                                                                          (higher/lower). Supported values are
                                                                          1, 2 or 3 */
  .bIsHigherFreqTim = FREQ_RELATION2,                           /*!< When bFreqRatio is greather than 1
                                                                     this param is used to indicate if this
                                                                     instance is the one with the highest
                                                                     frequency. Allowed value are: HIGHER_FREQ
                                                                     or LOWER_FREQ */
  .IRQnb            = MC_IRQ_PWMNCURRFDBK_2,                     /*!< MC IRQ number used as TIMx Update
                                                                      event */
/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx_1           = ADC_1_PERIPH2,                        /*!< First ADC peripheral to be used.*/
  .ADCx_2           = ADC_2_PERIPH2,                        /*!< Second ADC peripheral to be used.*/
  .bIaChannel       = PHASE_U_CURR_CHANNEL2,                  /*!< ADC channel used for conversion of
                                                                   current Ia. It must be equal to
                                                                   ADC_Channel_x x= 0, ..., 15*/
  .hIaPort          = PHASE_U_GPIO_PORT2,                  /*!< GPIO port used by hIaChannel. It must
                                                                be equal to GPIOx x= A, B, ...*/
  .hIaPin           = PHASE_U_GPIO_PIN2,                      /*!< GPIO pin used by hIaChannel. It must be
                    =                                         equal to GPIO_Pin_x x= 0, 1, ...*/
  .b_IaSamplingTime = SAMPLING_TIME_SEL2,               /*!< Sampling time used to convert hIaChannel.
                                                             It must be equal to ADC_SampleTime_xCycles5
                                                             x= 1, 7, ...*/
  .bIbChannel       = PHASE_V_CURR_CHANNEL2,                  /*!< ADC channel used for conversion of
                                                                   current Ib. It must be equal to
                                                                   ADC_Channel_x x= 0, ..., 15*/
  .hIbPort          = PHASE_V_GPIO_PORT2,                /*!< GPIO port used by hIbChannel. It must
                                                              be equal to GPIOx x= A, B, ...*/
  .hIbPin           = PHASE_V_GPIO_PIN2,                      /*!< GPIO pin used by hIaChannel. It must be
                                                                   equal to GPIO_Pin_x x= 0, 1, ...*/
  .b_IbSamplingTime = SAMPLING_TIME_SEL2,                /*!< Sampling time used to convert hIbChannel.
                                                              It must be equal to ADC_SampleTime_xCycles5
                                                              x= 1, 7, ...*/
  .bIcChannel       = PHASE_W_CURR_CHANNEL2,                  /*!< ADC channel used for conversion of
                                                                   current Ia. It must be equal to
                                                                   ADC_Channel_x x= 0, ..., 15*/
  .hIcPort          = PHASE_W_GPIO_PORT2,                /*!< GPIO port used by hIaChannel. It must
                                                              be equal to GPIOx x= A, B, ...*/
  .hIcPin           = PHASE_W_GPIO_PIN2,                      /*!< GPIO pin used by hIaChannel. It must be
                                                                   equal to GPIO_Pin_x x= 0, 1, ...*/
  .b_IcSamplingTime = SAMPLING_TIME_SEL2,            /*!< Sampling time used to convert hIaChannel.
                                                          It must be equal to ADC_SampleTime_xCycles5
                                                          x= 1, 7, ...*/

/* PWM generation parameters --------------------------------------------------*/
  .hDeadTime          = DEAD_TIME_COUNTS2,                    /*!< Dead time in number of TIM clock
                                                                   cycles. If CHxN are enabled, it must
                                                                   contain the dead time to be generated
                                                                   by the microcontroller, otherwise it
                                                                   expresses the maximum dead time
                                                                   generated by driving network */
  .bRepetitionCounter = REP_COUNTER2,                         /*!< It expresses the number of PWM
                                                                   periods to be elapsed before compare
                                                                   registers are updated again. In
                                                                   particular:
                                                                   RepetitionCounter= (2* #PWM periods)-1*/
  .hTafter            = TW_AFTER2,                             /*!< It is the sum of dead time plus max
                                                                    value between rise time and noise time
                                                                    express in number of TIM clocks.*/
  .hTbefore           = TW_BEFORE2,                           /*!< It is the sampling time express in
                                                                   number of TIM clocks.*/
  .TIMx               = PWM_TIMER_SELECTION2,                 /*!< It contains the pointer to the timer
                                                                   used for PWM generation. */
/* PWM Driving signals initialization ----------------------------------------*/

  .wTIM1Remapping = PWM_TIMER_REMAPPING2,                        /*!< Used only for instance #1, it
                                                                      remaps TIM1 outputs. It must equal to
                                                                      GPIO_PartialRemap_TIM1 or
                                                                      GPIO_FullRemap_TIM1 or
                                                                      GPIO_NoRemap_TIM1 */
  .hCh1Polarity   = PHASE_UH_POLARITY2,   	              /*!< Channel 1 (high side) output polarity,
                                                              it must be TIM_OCPolarity_High or
                                                              TIM_OCPolarity_Low */
  .hCh1Port       = PHASE_UH_GPIO_PORT2,                 /*!< Channel 1 (high side) GPIO output
                                                              port (if used, after re-mapping).
                                                              It must be GPIOx x= A, B, ...*/
  .hCh1Pin        = PHASE_UH_GPIO_PIN2,                  /*!< Channel 1 (high side) GPIO output pin
                                                              (if used, after re-mapping). It must be
                                                              GPIO_Pin_x x= 0, 1, ...*/
  .hCh1IdleState  = BRAKE_UH_POLARITY2,                   /*!< Channel 1 (high side) state (low or high)
                                                               when TIM peripheral is in Idle state.
                                                               It must be TIM_OCIdleState_Set or
                                                               TIM_OCIdleState_Reset*/

  .hCh2Polarity   = PHASE_VH_POLARITY2,                   /*!< Channel 2 (high side) output polarity,
                                                               it must be TIM_OCPolarity_High or
                                                               TIM_OCPolarity_Low */
  .hCh2Port       = PHASE_VH_GPIO_PORT2,                               /*!< Channel 2 (high side) GPIO output
                                                                            port (if used, after re-mapping).
                                                                            It must be GPIOx x= A, B, ...*/
  .hCh2Pin        = PHASE_VH_GPIO_PIN2,                       /*!< Channel 2 (high side) GPIO output pin
                                                                  (if used, after re-mapping). It must be
                                                                  GPIO_Pin_x x= 0, 1, ...*/
  .hCh2IdleState  = BRAKE_VH_POLARITY2,              /*!< Channel 2 (high side) state (low or high)
                                                          when TIM peripheral is in Idle state.
                                                          It must be TIM_OCIdleState_Set or
                                                          TIM_OCIdleState_Reset*/

  .hCh3Polarity   = PHASE_WH_POLARITY2,            	 /*!< Channel 3 (high side) output polarity,
                                                             it must be TIM_OCPolarity_High or
                                                             TIM_OCPolarity_Low */
  .hCh3Port       = PHASE_WH_GPIO_PORT2,                               /*!< Channel 3 (high side) GPIO output
                                                                            port (if used, after re-mapping).
                                                                            It must be GPIOx x= A, B, ...*/
  .hCh3Pin        = PHASE_WH_GPIO_PIN2,                        /*!< Channel 3 (high side) GPIO output pin
                                                                    (if used, after re-mapping). It must be
                                                                    GPIO_Pin_x x= 0, 1, ...*/
  .hCh3IdleState  = BRAKE_WH_POLARITY2,              /*!< Channel 3 (high side) state (low or high)
                                                          when TIM peripheral is in Idle state.
                                                          It must be TIM_OCIdleState_Set or
                                                          TIM_OCIdleState_Reset*/

  .LowSideOutputs = (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING2,  /*!< Low side or enabling signals
                                                                                generation method are defined
                                                                                here.*/

  .hCh1NPolarity  = PHASE_UL_POLARITY2,               /*!< Channel 1N (low side) output polarity,
                                                           it must be TIM_OCNPolarity_High or
                                                           TIM_OCNPolarity_Low */
  .hCh1NPort      = PHASE_UL_GPIO_PORT2,                                 /*!< Channel 1N (low side) GPIO output
                                                                              port (if used, after re-mapping).
                                                                              It must be GPIOx x= A, B, ...*/
  .hCh1NPin       = PHASE_UL_GPIO_PIN2,                       /*!< Channel 1N (low side) GPIO output pin
                                                                   (if used, after re-mapping). It must be
                                                                   GPIO_Pin_x x= 0, 1, ...*/
  .hCh1NIdleState = BRAKE_UL_POLARITY2,              /*!< Channel 1N (low side) state (low or high)
                                                          when TIM peripheral is in Idle state.
                                                          It must be TIM_OCNIdleState_Set or
                                                          TIM_OCNIdleState_Reset*/


  .hCh2NPolarity  = PHASE_VL_POLARITY2,                 /*!< Channel 2N (low side) output polarity,
                                                             it must be TIM_OCNPolarity_High or
                                                             TIM_OCNPolarity_Low */
  .hCh2NPort      = PHASE_VL_GPIO_PORT2,                                 /*!< Channel 2N (low side) GPIO output
                                                                              port (if used, after re-mapping).
                                                                              It must be GPIOx x= A, B, ...*/
  .hCh2NPin       = PHASE_VL_GPIO_PIN2,                     /*!< Channel 2N (low side) GPIO output pin
                                                                 (if used, after re-mapping). It must be
                                                                 GPIO_Pin_x x= 0, 1, ...*/
  .hCh2NIdleState = BRAKE_VL_POLARITY2,              /*!< Channel 2N (low side) state (low or high)
                                                          when TIM peripheral is in Idle state.
                                                          It must be TIM_OCNIdleState_Set or
                                                          TIM_OCNIdleState_Reset*/

  .hCh3NPolarity  = PHASE_WL_POLARITY2,             /*!< Channel 3N (low side) output polarity,
                                                         it must be TIM_OCNPolarity_High or
                                                         TIM_OCNPolarity_Low */
  .hCh3NPort      = PHASE_WL_GPIO_PORT2,                               /*!< Channel 3N (low side)  GPIO output
                                                                            port (if used, after re-mapping).
                                                                            It must be GPIOx x= A, B, ...*/
  .hCh3NPin       = PHASE_WL_GPIO_PIN2,                    /*!< Channel 3N (low side)  GPIO output pin
                                                                (if used, after re-mapping). It must be
                                                                GPIO_Pin_x x= 0, 1, ...*/
  .hCh3NIdleState = BRAKE_WL_POLARITY2,              /*!< Channel 3N (low side) state (low or high)
                                                          when TIM peripheral is in Idle state.
                                                          It must be TIM_OCNIdleState_Set or
                                                          TIM_OCNIdleState_Reset*/
/* PWM Driving signals initialization ----------------------------------------*/
  .bBKINMode     = BKIN_MODE2,                         /*!< It defines the modality of emergency
                                                            input. It must be any of the
                                                            the following:
                                                            NONE - feature disabled.
                                                            INT_MODE - Internal comparator used
                                                            as source of emergency event.
                                                            EXT_MODE - External comparator used
                                                            as source of emergency event.
                                                            EXTINT_MODE - The or-ed signal
                                                            between internal and external
                                                            comparator is used as source event.*/
  .hBKINPolarity = EMSTOP_ACTIVE_HIGH,               /*!< Emergency Stop (BKIN) input polarity,
                                                          it must be TIM_BreakPolarity_Low or
                                                          TIM_BreakPolarity_High */
  .hBKINPort     = EMERGENCY_STOP_GPIO_PORT2,             /*!< Emergency Stop (BKIN) GPIO input
                                                               port (if used, after re-mapping).
                                                               It must be GPIOx x= A, B, ...*/
  .hBKINPin      = EMERGENCY_STOP_GPIO_PIN2,              /*!< Emergency Stop (BKIN) GPIO input pin
                                                            (if used, after re-mapping). It must be
                                                            GPIO_Pin_x x= 0, 1, ...*/
/* Alternate functions definition --------------------------------------------*/
  .bCh1AF   = PHASE_UH_GPIO_AF2,                   /*!< Channel 1 (high side) alternate
                                                        functions setting. It must be one
                                                        of the GPIO_AF_x (x=0,1, ...)
                                                        according to the defined GPIO port
                                                        and pin.*/
  .bCh2AF   = PHASE_VH_GPIO_AF2,                   /*!< Channel 2 (high side) alternate
                                                        functions setting. It must be one
                                                        of the GPIO_AF_x (x=0,1, ...)
                                                        according to the defined GPIO port
                                                        and pin.*/
  .bCh3AF   = PHASE_WH_GPIO_AF2,                   /*!< Channel 3 (high side) alternate
                                                        functions setting. It must be one
                                                        of the GPIO_AF_x (x=0,1, ...)
                                                        according to the defined GPIO port
                                                        and pin.*/
  .bCh1NAF  = PHASE_UL_GPIO_AF2,                   /*!< Channel 1 (low side) alternate
                                                        functions setting. It must be one
                                                        of the GPIO_AF_x (x=0,1, ...)
                                                        according to the defined GPIO port
                                                        and pin.*/
  .bCh2NAF  = PHASE_VL_GPIO_AF2,                   /*!< Channel 2 (low side) alternate
                                                        functions setting. It must be one
                                                        of the GPIO_AF_x (x=0,1, ...)
                                                        according to the defined GPIO port
                                                        and pin.*/
  .bCh3NAF  = PHASE_WL_GPIO_AF2,                   /*!< Channel 3 (low side) alternate
                                                        functions setting. It must be one
                                                        of the GPIO_AF_x (x=0,1, ...)
                                                        according to the defined GPIO port
                                                        and pin.*/
  .bBKINAF  = BRKIN_GPIO_AF2,                      /*!< Emergency Stop (BKIN) alternate
                                                        functions setting. It must be one
                                                        of the GPIO_AF_x (x=0,1, ...)
                                                        according to the defined GPIO port
                                                        and pin.*/
  .bBKIN2AF = BRKIN2_GPIO_AF2,                     /*!< Emergency Stop (BKIN2) alternate
                                                        functions setting. It must be one
                                                        of the GPIO_AF_x (x=0,1, ...)
                                                        according to the defined GPIO port
                                                        and pin.*/
/* Emergency input (BKIN2) signal initialization -----------------------------*/
  .bBKIN2Mode     = BKIN2_MODE2,                        /*!< It defines the modality of emergency
                                                             input 2. It must be any of the
                                                             the following:
                                                             NONE - feature disabled.
                                                             INT_MODE - Internal comparator used
                                                             as source of emergency event.
                                                             EXT_MODE - External comparator used
                                                             as source of emergency event.
                                                             EXTINT_MODE - The or-ed signal
                                                             between internal and external
                                                             comparator is used as source event.*/
  .hBKIN2Polarity = OVERCURR_FEEDBACK_POLARITY2,        /*!< Emergency Stop (BKIN2) input polarity,
                                                             it must be TIM_Break2Polarity_Low or
                                                             TIM_Break2Polarity_High */
  .hBKIN2Port     = EMERGENCY2_STOP_GPIO_PORT2,          /*!< Emergency Stop (BKIN) GPIO input
                                                              port (if used, after re-mapping).
                                                              It must be GPIOx x= A, B, ...*/
  .hBKIN2Pin      = EMERGENCY2_STOP_GPIO_PIN2,           /*!< Emergency Stop (BKIN) GPIO input pin
                                                              (if used, after re-mapping). It must be
                                                              GPIO_Pin_x x= 0, 1, ...*/
/* Filtering settings of the emergency inputs --------------------------------*/
  .bBKINFilter  = BKIN1_FILTER2,                         /*!< Emergency Stop (BKIN) digital
                                                              filter. See TIMx_BDTR (BKF bits)
                                                              definition in the reference
                                                              manual.*/
  .bBKIN2Filter = BKIN2_FILTER2,                         /*!< Emergency Stop (BKIN2) digital
                                                              filter. See TIMx_BDTR (BKF bits)
                                                              definition in the reference
                                                              manual.*/
/* Internal OPAMP common settings --------------------------------------------*/
  .pOPAMPParams     = R3_2_OPAMPPARAMSM2,                /*!< Pointer to the OPAMP params struct.
                                                              It must be MC_NULL if internal
                                                              OPAMP are not used.*/
/* Internal COMP settings ----------------------------------------------------*/
  .pOCPA_COMPParams = OCPA_COMPPARAMSM2,
                                                          /*!< Pointer to the COMP params struct
                                                               used for overcurrent protection of
                                                               phase A.
                                                               It must be MC_NULL if internal COMP
                                                               are not used for OCP protection.*/
  .pOCPB_COMPParams = OCPB_COMPPARAMSM2,
                                                          /*!< Pointer to the COMP params struct
                                                               used for overcurrent protection of
                                                               phase B.
                                                               It must be MC_NULL if internal COMP
                                                               are not used for OCP protection.*/
  .pOCPC_COMPParams = OCPC_COMPPARAMSM2,
                                                          /*!< Pointer to the COMP params struct
                                                               used for overcurrent protection of
                                                               phase C.
                                                               It must be MC_NULL if internal COMP
                                                               are not used for OCP protection.*/
  .pOVP_COMPParams  = OVP_COMPPARAMSM2,
                                                          /*!< Pointer to the COMP params struct
                                                               used for overvoltage protection.
                                                               It must be MC_NULL if internal COMP
                                                               are not used for OVP protection.*/
/* DAC settings --------------------------------------------------------------*/
  .hDAC_OCP_Threshold = OCPREF,                             /*!< Value of analog reference expressed
                                                                 as 16bit unsigned integer.
                                                                 Ex. 0 = 0V 65536 = VDD_DAC.*/
  .hDAC_OVP_Threshold = OVPREF,                             /*!< Value of analog reference expressed
                                                                 as 16bit unsigned integer.
                                                                 Ex. 0 = 0V 65536 = VDD_DAC.*/
/* Regular conversion --------------------------------------------------------*/
  .regconvADCx = REGCONVADC2                                /*!< ADC peripheral used for regular
                                                                 conversion.*/
};

/**
  * @brief  Internal OPAMP parameters Motor 2 - single shunt - F30x
  */
R1_F30XOPAMPParams_t R1_F30XOPAMPParamsM2 =
{
/* OPAMP settings ------------------------------------------------------*/
  .wOPAMP_Selection                   = OPAMP_SELECTION2,                        /*!< First OPAMP selection. It must be
                                                                                      either equal to
                                                                                      OPAMP_Selection_OPAMP1 or
                                                                                      OPAMP_Selection_OPAMP3.*/
  .bOPAMP_InvertingInput_MODE         = OPAMP_INVERTINGINPUT_MODE2,            /*!< First OPAMP inverting input mode.
                                                                                    It must be either equal to EXT_MODE
                                                                                    or INT_MODE.*/
  .wOPAMP_InvertingInput              = OPAMP_INVERTINGINPUT2,              /*!< First OPAMP inverting input pin.
                                                                                 If wOPAMP_Selection is
                                                                                 OPAMP_Selection_OPAMP1 it
                                                                                 must be one of the following:
                                                                                 OPAMP1_InvertingInput_PC5 or
                                                                                 OPAMP1_InvertingInput_PA3 if the
                                                                                 bOPAMP_InvertingInput_MODE is
                                                                                 EXT_MODE or
                                                                                 OPAMP1_InvertingInput_PGA or
                                                                                 OPAMP1_InvertingInput_FOLLOWER if the
                                                                                 bOPAMP_InvertingInput_MODE is
                                                                                 INT_MODE.

                                                                                 If wOPAMP_Selection is
                                                                                 OPAMP_Selection_OPAMP3 it
                                                                                 must be one of the following:
                                                                                 OPAMP3_InvertingInput_PB10 or
                                                                                 OPAMP3_InvertingInput_PB2 if the
                                                                                 bOPAMP_InvertingInput_MODE is
                                                                                 EXT_MODE or
                                                                                 OPAMP3_InvertingInput_PGA or
                                                                                 OPAMP3_InvertingInput_FOLLOWER if the
                                                                                 bOPAMP_InvertingInput_MODE is
                                                                                 INT_MODE.*/
  .hOPAMP_InvertingInput_GPIO_PORT    = OPAMP_INVERTINGINPUT_GPIO_PORT2,          /*!< First OPAMP inverting input GPIO port
                                                                                       as defined in wOPAMP_InvertingInput.
                                                                                       It must be GPIOx x= A, B, ... if
                                                                                       bOPAMP_InvertingInput_MODE is
                                                                                       EXT_MODE, otherwise can be dummy.*/
  .hOPAMP_InvertingInput_GPIO_PIN     = OPAMP_INVERTINGINPUT_GPIO_PIN2,           /*!< First OPAMP inverting input GPIO pin
                                                                                       as defined in wOPAMP_InvertingInput.
                                                                                       It must be GPIO_Pin_x x= 0, 1, ... if
                                                                                       bOPAMP_InvertingInput_MODE is
                                                                                       EXT_MODE, otherwise can be dummy.*/
  .wOPAMP_NonInvertingInput           = OPAMP_NONINVERTINGINPUT2,                 /*!< OPAMP non inverting input first
                                                                                       selection.
                                                                                       If wOPAMP_Selection is
                                                                                       OPAMP_Selection_OPAMP1 it
                                                                                       must be one of the following:
                                                                                       OPAMP1_NonInvertingInput_PA7,
                                                                                       OPAMP1_NonInvertingInput_PA5,
                                                                                       OPAMP1_NonInvertingInput_PA3,
                                                                                       OPAMP1_NonInvertingInput_PA1.

                                                                                       If wOPAMP_Selection is
                                                                                       OPAMP_Selection_OPAMP3 it
                                                                                       must be one of the following:
                                                                                       OPAMP3_NonInvertingInput_PB13,
                                                                                       OPAMP3_NonInvertingInput_PA5,
                                                                                       OPAMP3_NonInvertingInput_PA1,
                                                                                       OPAMP3_NonInvertingInput_PB0.*/
  .hOPAMP_NonInvertingInput_GPIO_PORT = OPAMP_NONINVERTINGINPUT_GPIO_PORT2,        /*!< OPAMP non inverting input GPIO
                                                                                        port as defined in
                                                                                        wOPAMP_NonInvertingInput_PHA.
                                                                                        It must be GPIOx x= A, B, ...*/
  .hOPAMP_NonInvertingInput_GPIO_PIN  = OPAMP_NONINVERTINGINPUT_GPIO_PIN2,        /*!< OPAMP non inverting input GPIO
                                                                                       pin as defined in
                                                                                       wOPAMP_NonInvertingInput_PHA.
                                                                                       It must be GPIO_Pin_x x= 0, 1, ...*/
  .hOPAMP_Output_GPIO_PORT            = OPAMP_OUT_GPIO_PORT2,                     /*!< OPAMP output GPIO port.
                                                                                       It must be GPIOx x= A, B, ...
                                                                                       Note: Output pin is PA2 for OPAMP1,
                                                                                       PB1 for OPAMP3.*/
  .hOPAMP_Output_GPIO_PIN             = OPAMP_OUT_GPIO_PIN2,                      /*!< OPAMP output GPIO pin.
                                                                                       It must be GPIO_Pin_x x= 0, 1, ...
                                                                                       Note: Output pin is PA2 for OPAMP1,
                                                                                       PB1 for OPAMP3.*/
/* Common settings -----------------------------------------------------------*/
  .wOPAMP_PGAGain   = OPAMP_PGAGAIN2,                            /*!< It defines the OPAMP PGA gains.
                                                                    It must be one of the following:
                                                                    OPAMP_OPAMP_PGAGain_2,
                                                                    OPAMP_OPAMP_PGAGain_4,
                                                                    OPAMP_OPAMP_PGAGain_8,
                                                                    OPAMP_OPAMP_PGAGain_16.
                                                                    This value is taken in account
                                                                    just if wOPAMPx_InvertingInput is
                                                                    equal to OPAMP2_InvertingInput_PGA*/
  .OPAMP_PGAConnect = OPAMP_PGACONNECT2,                       /*!< It defines the OPAMP connection
                                                                    with an external filter when PGA																					 is enabled.
                                                                    It must be one of the following:
                                                                    OPAMP_PGAConnect_No,
                                                                    OPAMP_PGAConnect_IO1,
                                                                    OPAMP_PGAConnect_IO2.
                                                                    See reference manual RM0316.
                                                                    This value is taken in account
                                                                    just if wOPAMPx_InvertingInput is
                                                                    equal to OPAMP2_InvertingInput_PGA*/
};

/**
  * @brief  Internal COMP parameters Motor 2 - single shunt - F30x - OCP
  */
F30XCOMPParams_t R1_F30XOCPCOMPParamsM2 =
{
 .wSelection                   = OCP_SELECTION2,                   /*!< Internal comparator used for protection.
                                                                  It must be
                                                                  COMP_Selection_COMPx x = 1,2,3,4,5,6,7.*/

 .bInvertingInput_MODE         = OCP_INVERTINGINPUT_MODE2,         /*!< COMPx inverting input mode. It must be either
                                                                  equal to EXT_MODE or INT_MODE. */

 .wInvertingInput              = OCP_INVERTINGINPUT2,              /*!< COMPx inverting input pin. It must be one of
                                                                  the following:
                                                                  COMP1_InvertingInput_PA0,
                                                                  COMP2_InvertingInput_PA2,
                                                                  COMP3_InvertingInput_PD15,
                                                                  COMP3_InvertingInput_PB12,
                                                                  COMP4_InvertingInput_PE8,
                                                                  COMP4_InvertingInput_PB2,
                                                                  COMP5_InvertingInput_PD13,
                                                                  COMP5_InvertingInput_PB10,
                                                                  COMP6_InvertingInput_PD10,
                                                                  COMP6_InvertingInput_PB15 if
                                                                  bInvertingInput_MODE is EXT_MODE or
                                                                  COMPX_InvertingInput_DAC1,
                                                                  COMPX_InvertingInput_DAC2,
                                                                  COMPX_InvertingInput_VREF,
                                                                  COMPX_InvertingInput_VREF_1_4,
                                                                  COMPX_InvertingInput_VREF_1_2,
                                                                  COMPX_InvertingInput_VREF_3_4 if
                                                                  bInvertingInput_MODE is INT_MODE.
                                                                  If bInvertingInput_MODE is EXT_MODE, the
                                                                  only available options are related to the
                                                                  selected COMP in wSelection.*/

 .hInvertingInput_GPIO_PORT    = OCP_INVERTINGINPUT_GPIO_PORT2,    /*!< COMPx inverting input GPIO port as defined in
                                                                  wInvertingInput (Just if
                                                                  bInvertingInput_MODE is EXT_MODE).
                                                                  It must be GPIOx x= A, B, ...*/

 .hInvertingInput_GPIO_PIN     = OCP_INVERTINGINPUT_GPIO_PIN2,     /*!< COMPx inverting input GPIO pin as defined in
                                                                  wInvertingInput (Just if
                                                                  bInvertingInput_MODE is EXT_MODE).
                                                                  It must be GPIO_Pin_x x= 0, 1, ...*/

 .wNonInvertingInput           = OCP_NONINVERTINGINPUT2,           /*!< COMPx non inverting input. It must be one of
                                                                  the following:
                                                                  COMP1_NonInvertingInput_PA1,
                                                                  COMP2_NonInvertingInput_PA3,
                                                                  COMP2_NonInvertingInput_PA7,
                                                                  COMP3_NonInvertingInput_PB14,
                                                                  COMP3_NonInvertingInput_PD14,
                                                                  COMP4_NonInvertingInput_PB0,
                                                                  COMP4_NonInvertingInput_PE7,
                                                                  COMP5_NonInvertingInput_PB13,
                                                                  COMP5_NonInvertingInput_PD12,
                                                                  COMP6_NonInvertingInput_PB11,
                                                                  COMP6_NonInvertingInput_PD11,
                                                                  COMP7_NonInvertingInput_PC1,
                                                                  COMP7_NonInvertingInput_PA0.
                                                                  The only available options are related to the
                                                                  selected COMP in wSelection.*/

 .hNonInvertingInput_GPIO_PORT = OCP_NONINVERTINGINPUT_GPIO_PORT2, /*!< COMPx non inverting input GPIO port as
                                                                  defined in wNonInvertingInput.
                                                                  It must be GPIOx x= A, B, ...*/

 .hNonInvertingInput_GPIO_PIN  = OCP_NONINVERTINGINPUT_GPIO_PIN2,  /*!< COMPx non inverting input GPIO pin as defined
                                                                  in wNonInvertingInput.
                                                                  It must be GPIO_Pin_x x= 0, 1, ...*/

 .bOutput_MODE                 = OCP_OUTPUT_MODE2,                 /*!< COMPx output. It must be either
                                                                  equal to EXT_MODE or INT_MODE. */

 .wOutput                      = OCP_OUTPUT2,                      /*!< COMPx output selection. It must be one of
                                                                  the following:
                                                                  COMP_Output_TIM1BKIN,
                                                                  COMP_Output_TIM1BKIN2,
                                                                  COMP_Output_TIM8BKIN,
                                                                  COMP_Output_TIM8BKIN2,
                                                                  COMP_Output_TIM1BKIN2_TIM8BKIN2.*/
 .hOutput_GPIO_PORT            = OCP_OUTPUT_GPIO_PORT2,            /*!< COMPx output GPIO port.
                                                                  It must be GPIOx x= A, B, ...*/

 .hOutput_GPIO_PIN             = OCP_OUTPUT_GPIO_PIN2,             /*!< COMPx output GPIO pin as defined.
                                                                  It must be GPIO_Pin_x x= 0, 1, ...*/

 .bOutput_GPIO_AF              = OCP_OUTPUT_GPIO_AF2,       /*!< COMPx output alternate functions setting.
                                                                  It must be one of the GPIO_AF_x (x=0,1, ...)
                                                                  according to the defined GPIO port and pin.*/
 .wOutputPol                   = OCP_OUTPUTPOL2,                   /*!< COMPx output polarity. It must be one of
                                                                  the following:
                                                                  COMP_OutputPol_NonInverted,
                                                                  COMP_OutputPol_Inverted.*/

 .wMode                        = OCP_FILTER2,                      /*!< COMPx mode it is used to define both
                                                                  the speed (analog filter) and
                                                                  the power consumption of the internal
                                                                  comparator. It must be one of the
                                                                  following:
                                                                  COMP_Mode_HighSpeed,
                                                                  COMP_Mode_MediumSpeed,
                                                                  COMP_Mode_LowPower,
                                                                  COMP_Mode_UltraLowPower.
                                                                  More speed means less filter and more
                                                                  consumption.*/
};

/**
  * @brief  Current sensor parameters Motor 1 - single shunt - F30x
  */
R1_F30XParams_t R1_F30XParamsM2 =
{
  .wADC_Clock_Divider = ADC_CLOCK_DIVIDER2,                /*!<  AHB clock prescaling factor for
                                                                 ADC peripheral. It must be
                                                                 RCC_ADC12PLLCLK_Divx or
                                                                 RCC_ADC34PLLCLK_Divx
                                                                 x = 1, 2, 4, 6, 8, ... */
  .wAHBPeriph         = ADC_AHBPERIPH2,                     /*!< AHB periph used. It must be
                                                                 RCC_AHBPeriph_ADC12 or
                                                                 RCC_AHBPeriph_ADC34. */
  .bTim_Clock_Divider = TIM_CLOCK_DIVIDER2,                 /*!< APB2 clock prescaling factor for
                                                                 ADC peripheral. It must be equal to 1,
                                                                 2 or 4*/
/* Dual MC parameters --------------------------------------------------------*/
  .bInstanceNbr     = 2,                              /*!< Instance number with reference to PWMC
                                                           base class. It is necessary to properly
                                                           synchronize TIM8 with TIM1 at peripheral
                                                           initializations */
  .bFreqRatio       = FREQ_RATIO,                     /*!< It is used in case of dual MC to
                                                           synchronize TIM1 and TIM8. It has
                                                           effect only on the second instanced
                                                           object and must be equal to the
                                                           ratio between the two PWM frequencies
                                                           (higher/lower). Supported values are
                                                           1, 2 or 3 */
  .bIsHigherFreqTim = FREQ_RELATION2,                   /*!< When bFreqRatio is greather than 1
                                                             this param is used to indicate if this
                                                             instance is the one with the highest
                                                             frequency. Allowed value are: HIGHER_FREQ
                                                             or LOWER_FREQ */
  .IRQnb            = MC_IRQ_PWMNCURRFDBK_2,            /*!< MC IRQ number used */
/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx            = ADC_PERIPH2,                          /*!< ADC peripheral to be used*/
  .bIChannel       = PHASE_CURRENTS_CHANNEL2,               /*!< ADC channel used for conversion of
                                                                 current. It must be equal to
                                                                 ADC_Channel_x x= 0, ..., 15*/
  .hIPort          = PHASE_CURRENTS_GPIO_PORT2,             /*!< GPIO port used by bIChannel. It must
                                                                 be equal to GPIOx x= A, B, ...*/
  .hIPin           = PHASE_CURRENTS_GPIO_PIN2,              /*!< GPIO pin used by bIChannel. It must be
                                                                 equal to GPIO_Pin_x x= 0, 1, ...*/
  .b_ISamplingTime = SAMPLING_TIME_SEL2,                /*!< Sampling time used to convert bIChannel.
                                                             It must be equal to ADC_SampleTime_xCycles5
                                                             x= 1, 7, ...*/
/* PWM generation parameters --------------------------------------------------*/
  .hDeadTime          = DEAD_TIME_COUNTS2,                  /*!< Dead time in number of TIM clock
                                                                 cycles. If CHxN are enabled, it must
                                                                 contain the dead time to be generated
                                                                 by the microcontroller, otherwise it
                                                                 expresses the maximum dead time
                                                                 generated by driving network*/
  .bRepetitionCounter = REP_COUNTER2,                      /*!< It expresses the number of PWM
                                                                periods to be elapsed before compare
                                                                registers are updated again. In
                                                                particular:
                                                                RepetitionCounter= (2* PWM periods) -1*/
  .hTafter            = TAFTER2,                           /*!< It is the sum of dead time plus rise time
                                                                express in number of TIM clocks.*/
  .hTbefore           = TBEFORE2,                          /*!< It is the value of sampling time
                                                                expressed in numbers of TIM clocks.*/
  .hTMin              = TMIN2,                             /*!< It is the sum of dead time plus rise time
                                                                plus sampling time express in numbers of
                                                                TIM clocks.*/
  .hHTMin             = HTMIN2,                            /*!< It is the half of hTMin value*/
  .hCHTMin            = CHTMIN2,                           /*!< It is the half of hTMin value, considering FOC rate*/
  .hTSample           = TSAMPLE2,                          /*!< It is the sampling time express in
                                                                numbers of TIM clocks.*/
  .hMaxTrTs           = MAX_TRTS2,                         /*!< It is the maximum between twice of rise
                                                                time express in number of TIM clocks and
                                                                twice of sampling time express in numbers
                                                                of TIM clocks.*/
  .TIMx               = PWM_TIMER_SELECTION2,              /*!< It contains the pointer to the timer
                                                                used for PWM generation. It must
                                                                equal to TIM1 if bInstanceNbr is
                                                                equal to 1, to TIM8 otherwise */
/* PWM Driving signals initialization ----------------------------------------*/

  .wTIM1Remapping = PWM_TIMER_REMAPPING2,                        /*!< Used only for instance #1, it
                                                                      remaps TIM1 outputs. It must equal to
                                                                      GPIO_PartialRemap_TIM1 or
                                                                      GPIO_FullRemap_TIM1 or
                                                                      GPIO_NoRemap_TIM1 */
  .hCh1Polarity   = PHASE_UH_POLARITY2,   	              /*!< Channel 1 (high side) output polarity,
                                                              it must be TIM_OCPolarity_High or
                                                              TIM_OCPolarity_Low */
  .hCh1Port       = PHASE_UH_GPIO_PORT2,                 /*!< Channel 1 (high side) GPIO output
                                                              port (if used, after re-mapping).
                                                              It must be GPIOx x= A, B, ...*/
  .hCh1Pin        = PHASE_UH_GPIO_PIN2,                  /*!< Channel 1 (high side) GPIO output pin
                                                             (if used, after re-mapping). It must be
                                                              GPIO_Pin_x x= 0, 1, ...*/
  .hCh1IdleState  = BRAKE_UH_POLARITY2,                   /*!< Channel 1 (high side) state (low or high)
                                                               when TIM peripheral is in Idle state.
                                                               It must be TIM_OCIdleState_Set or
                                                               TIM_OCIdleState_Reset*/

  .hCh2Polarity   = PHASE_VH_POLARITY2,                   /*!< Channel 2 (high side) output polarity,
                                                               it must be TIM_OCPolarity_High or
                                                               TIM_OCPolarity_Low */
  .hCh2Port       = PHASE_VH_GPIO_PORT2,                  /*!< Channel 2 (high side) GPIO output
                                                               port (if used, after re-mapping).
                                                               It must be GPIOx x= A, B, ...*/
  .hCh2Pin        = PHASE_VH_GPIO_PIN2,                   /*!< Channel 2 (high side) GPIO output pin
                                                               (if used, after re-mapping). It must be
                                                               GPIO_Pin_x x= 0, 1, ...*/
  .hCh2IdleState  = BRAKE_VH_POLARITY2,                   /*!< Channel 2 (high side) state (low or high)
                                                                when TIM peripheral is in Idle state.
                                                                It must be TIM_OCIdleState_Set or
                                                                TIM_OCIdleState_Reset*/

  .hCh3Polarity   = PHASE_WH_POLARITY2,            	 /*!< Channel 3 (high side) output polarity,
                                                             it must be TIM_OCPolarity_High or
                                                             TIM_OCPolarity_Low */
  .hCh3Port       = PHASE_WH_GPIO_PORT2,                /*!< Channel 3 (high side) GPIO output
                                                             port (if used, after re-mapping).
                                                             It must be GPIOx x= A, B, ...*/
  .hCh3Pin        = PHASE_WH_GPIO_PIN2,                 /*!< Channel 3 (high side) GPIO output pin
                                                             (if used, after re-mapping). It must be
                                                             GPIO_Pin_x x= 0, 1, ...*/
  .hCh3IdleState  = BRAKE_WH_POLARITY2,                 /*!< Channel 3 (high side) state (low or high)
                                                             when TIM peripheral is in Idle state.
                                                             It must be TIM_OCIdleState_Set or
                                                             TIM_OCIdleState_Reset*/

  .LowSideOutputs = (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING2,  /*!< Low side or enabling signals
                                                                                generation method are defined
                                                                                here.*/

  .hCh1NPolarity  = PHASE_UL_POLARITY2,               /*!< Channel 1N (low side) output polarity,
                                                           it must be TIM_OCNPolarity_High or
                                                           TIM_OCNPolarity_Low */
  .hCh1NPort      = PHASE_UL_GPIO_PORT2,              /*!< Channel 1N (low side) GPIO output
                                                           port (if used, after re-mapping).
                                                           It must be GPIOx x= A, B, ...*/
  .hCh1NPin       = PHASE_UL_GPIO_PIN2,                       /*!< Channel 1N (low side) GPIO output pin
                                                                   (if used, after re-mapping). It must be
                                                                   GPIO_Pin_x x= 0, 1, ...*/
  .hCh1NIdleState = BRAKE_UL_POLARITY2,              /*!< Channel 1N (low side) state (low or high)
                                                          when TIM peripheral is in Idle state.
                                                          It must be TIM_OCNIdleState_Set or
                                                          TIM_OCNIdleState_Reset*/


  .hCh2NPolarity  = PHASE_VL_POLARITY2,                 /*!< Channel 2N (low side) output polarity,
                                                             it must be TIM_OCNPolarity_High or
                                                             TIM_OCNPolarity_Low */
  .hCh2NPort      = PHASE_VL_GPIO_PORT2,                /*!< Channel 2N (low side) GPIO output
                                                             port (if used, after re-mapping).
                                                             It must be GPIOx x= A, B, ...*/
  .hCh2NPin       = PHASE_VL_GPIO_PIN2,                     /*!< Channel 2N (low side) GPIO output pin
                                                                 (if used, after re-mapping). It must be
                                                                 GPIO_Pin_x x= 0, 1, ...*/
  .hCh2NIdleState = BRAKE_VL_POLARITY2,              /*!< Channel 2N (low side) state (low or high)
                                                          when TIM peripheral is in Idle state.
                                                          It must be TIM_OCNIdleState_Set or
                                                          TIM_OCNIdleState_Reset*/

  .hCh3NPolarity  = PHASE_WL_POLARITY2,             /*!< Channel 3N (low side) output polarity,
                                                         it must be TIM_OCNPolarity_High or
                                                         TIM_OCNPolarity_Low */
  .hCh3NPort      = PHASE_WL_GPIO_PORT2,             /*!< Channel 3N (low side)  GPIO output
                                                          port (if used, after re-mapping).
                                                          It must be GPIOx x= A, B, ...*/
  .hCh3NPin       = PHASE_WL_GPIO_PIN2,                    /*!< Channel 3N (low side)  GPIO output pin
                                                                (if used, after re-mapping). It must be
                                                                GPIO_Pin_x x= 0, 1, ...*/
  .hCh3NIdleState = BRAKE_WL_POLARITY2,              /*!< Channel 3N (low side) state (low or high)
                                                          when TIM peripheral is in Idle state.
                                                          It must be TIM_OCNIdleState_Set or
                                                          TIM_OCNIdleState_Reset*/
/* Emergency input signal initialization -------------------------------------*/
  .bBKINMode     = BKIN_MODE2,                           /*!< It defines the modality of emergency
                                                              input. It must be any of the
                                                              the following:
                                                              NONE - feature disabled.
                                                              INT_MODE - Internal comparator used
                                                              as source of emergency event.
                                                              EXT_MODE - External comparator used
                                                              as source of emergency event.
                                                              EXTINT_MODE - The or-ed signal
                                                              between internal and external
                                                              comparator is used as source event.*/
  .hBKINPolarity = OVERCURR_FEEDBACK_POLARITY2,               /*!< Emergency Stop (BKIN) input polarity,
                                                                   it must be TIM_BreakPolarity_Low or
                                                                   TIM_BreakPolarity_High */
  .hBKINPort     = EMERGENCY_STOP_GPIO_PORT2,             /*!< Emergency Stop (BKIN) GPIO input
                                                               port (if used, after re-mapping).
                                                               It must be GPIOx x= A, B, ...*/
  .hBKINPin      = EMERGENCY_STOP_GPIO_PIN2,              /*!< Emergency Stop (BKIN) GPIO input pin
                                                               (if used, after re-mapping). It must be
                                                                GPIO_Pin_x x= 0, 1, ...*/
/* Alternate functions definition --------------------------------------------*/
  .bCh1AF   = PHASE_UH_GPIO_AF2,                   /*!< Channel 1 (high side) alternate
                                                        functions setting. It must be one
                                                        of the GPIO_AF_x (x=0,1, ...)
                                                        according to the defined GPIO port
                                                        and pin.*/
  .bCh2AF   = PHASE_VH_GPIO_AF2,                   /*!< Channel 2 (high side) alternate
                                                        functions setting. It must be one
                                                        of the GPIO_AF_x (x=0,1, ...)
                                                        according to the defined GPIO port
                                                        and pin.*/
  .bCh3AF   = PHASE_WH_GPIO_AF2,                   /*!< Channel 3 (high side) alternate
                                                       functions setting. It must be one
                                                       of the GPIO_AF_x (x=0,1, ...)
                                                       according to the defined GPIO port
                                                       and pin.*/
  .bCh1NAF  = PHASE_UL_GPIO_AF2,                   /*!< Channel 1 (low side) alternate
                                                        functions setting. It must be one
                                                        of the GPIO_AF_x (x=0,1, ...)
                                                        according to the defined GPIO port
                                                        and pin.*/
  .bCh2NAF  = PHASE_VL_GPIO_AF2,                   /*!< Channel 2 (low side) alternate
                                                        functions setting. It must be one
                                                        of the GPIO_AF_x (x=0,1, ...)
                                                        according to the defined GPIO port
                                                        and pin.*/
  .bCh3NAF  = PHASE_WL_GPIO_AF2,                   /*!< Channel 3 (low side) alternate
                                                        functions setting. It must be one
                                                        of the GPIO_AF_x (x=0,1, ...)
                                                        according to the defined GPIO port
                                                        and pin.*/
  .bBKINAF  = BRKIN_GPIO_AF2,                      /*!< Emergency Stop (BKIN) alternate
                                                        functions setting. It must be one
                                                        of the GPIO_AF_x (x=0,1, ...)
                                                        according to the defined GPIO port
                                                        and pin.*/
  .bBKIN2AF = BRKIN2_GPIO_AF2,                     /*!< Emergency Stop (BKIN2) alternate
                                                        functions setting. It must be one
                                                        of the GPIO_AF_x (x=0,1, ...)
                                                        according to the defined GPIO port
                                                        and pin.*/
/* Emergency input (BKIN2) signal initialization -----------------------------*/
  .bBKIN2Mode      = BKIN2_MODE2,                         /*!< It defines the modality of emergency
                                                               input 2. It must be any of the
                                                               the following:
                                                               NONE - feature disabled.
                                                               INT_MODE - Internal comparator used
                                                               as source of emergency event.
                                                               EXT_MODE - External comparator used
                                                               as source of emergency event.
                                                               EXTINT_MODE - The or-ed signal
                                                               between internal and external
                                                               comparator is used as source event.*/
  .hBKIN2Polarity  = OVERCURR_FEEDBACK_POLARITY2,         /*!< Emergency Stop (BKIN2) input polarity,
                                                               it must be TIM_Break2Polarity_Low or
                                                               TIM_Break2Polarity_High */
  .hBKIN2Port      = EMERGENCY2_STOP_GPIO_PORT2,          /*!< Emergency Stop (BKIN) GPIO input
                                                               port (if used, after re-mapping).
                                                               It must be GPIOx x= A, B, ...*/
  .hBKIN2Pin       = EMERGENCY2_STOP_GPIO_PIN2,           /*!< Emergency Stop (BKIN) GPIO input pin
                                                               (if used, after re-mapping). It must be
                                                               GPIO_Pin_x x= 0, 1, ...*/
/* Filtering settings of the emergency inputs --------------------------------*/
  .bBKINFilter  = BKIN1_FILTER2,                         /*!< Emergency Stop (BKIN) digital
                                                              filter. See TIMx_BDTR (BKF bits)
                                                              definition in the reference
                                                              manual.*/
  .bBKIN2Filter = BKIN2_FILTER2,                         /*!< Emergency Stop (BKIN2) digital
                                                              filter. See TIMx_BDTR (BKF bits)
                                                              definition in the reference
                                                              manual.*/
/* Internal OPAMP common settings --------------------------------------------*/
  .pOPAMPParams = R1_OPAMPPARAMSM2,                   /*!< Pointer to the OPAMP params struct.
                                                           It must be MC_NULL if internal
                                                           OPAMP are not used.*/
/* Internal COMP settings ----------------------------------------------------*/
  .pOCP_COMPParams = OCP_COMPPARAMSM2,                   /*!< Pointer to the COMP params struct
                                                              used for overcurrent protection of
                                                              phase A.
                                                              It must be MC_NULL if internal COMP
                                                              are not used for OCP protection.*/
  .pOVP_COMPParams = OVP_COMPPARAMSM2,                   /*!< Pointer to the COMP params struct
                                                              used for overvoltage protection.
                                                              It must be MC_NULL if internal COMP
                                                              are not used for OVP protection.*/
/* DAC settings --------------------------------------------------------------*/
  .hDAC_OCP_Threshold = OCPREF,                             /*!< Value of analog reference expressed
                                                                 as 16bit unsigned integer.
                                                                 Ex. 0 = 0V 65536 = VDD_DAC.*/
  .hDAC_OVP_Threshold = OVPREF,                             /*!< Value of analog reference expressed
                                                                 as 16bit unsigned integer.
                                                                 Ex. 0 = 0V 65536 = VDD_DAC.*/
/* Regular conversion --------------------------------------------------------*/
  .regconvADCx = REGCONVADC2,                        /*!< ADC peripheral used for regular
                                           conversion.*/
};

/**
  * @brief  SpeedNPosition sensor parameters Motor 2 - HALL
  */
HALL_F30XParams_t HALL_F30XParamsM2 =
{
  /* SW Settings */
  MC_IRQ_SPEEDNPOSFDBK_2,   /*!< MC IRQ number used for TIMx capture update event.*/

  HALL_SENSORS_PLACEMENT2,   /*!< Define here the mechanical position of the sensors
                             with reference to an electrical cycle.
                             Allowed values are: DEGREES_120 or DEGREES_60.*/

  (int16_t)(HALL_PHASE_SHIFT2 * 65536/360),
                            /*!< Define here in s16degree the electrical phase shift
                             between the low to high transition of signal H1 and
                             the maximum of the Bemf induced on phase A.*/

  MEDIUM_FREQUENCY_TASK_RATE2, /*!< Frequency (Hz) at which motor speed is to
                             be computed. It must be equal to the frequency
                             at which function SPD_CalcAvrgMecSpeed01Hz
                             is called.*/

  HALL_AVERAGING_FIFO_DEPTH2, /*!< Size of the buffer used to calculate the average
                             speed. It must be less than 18.*/

  HALL_IC_FILTER2,         /*!< Time filter applied to validate HALL sensor capture.
                             This value defines the frequency used to sample
                             HALL sensors input and the length of the digital
                             filter applied. The digital filter is made of an
                             event counter in which N events are needed to
                             validate a transition on the input.
                                0000: No filter, sampling is done at fCK_INT.
                                0001: fSAMPLING=fCK_INT , N=2.
                                0010: fSAMPLING=fCK_INT , N=4.
                                0011: fSAMPLING=fCK_INT , N=8.
                                0100: fSAMPLING=fCK_INT/2, N=6.
                                0101: fSAMPLING=fCK_INT/2, N=8.
                                0110: fSAMPLING=fCK_INT/4, N=6.
                                0111: fSAMPLING=fCK_INT/4, N=8.
                                1000: fSAMPLING=fCK_INT/8, N=6.
                                1001: fSAMPLING=fCK_INT/8, N=8.
                                1010: fSAMPLING=fCK_INT/16, N=5.
                                1011: fSAMPLING=fCK_INT/16, N=6.
                                1100: fSAMPLING=fCK_INT/16, N=8.
                                1101: fSAMPLING=fCK_INT/32, N=5.
                                1110: fSAMPLING=fCK_INT/32, N=6.
                                1111: fSAMPLING=fCK_INT/32, N=8 */

  /* HW Settings */
  HALL_TIM_CLK2,          /*!< Timer clock frequency express in Hz.*/

  HALL_TIMER2,        /*!< Timer used for HALL sensor management.*/

  HALL_RCC_PERIPHERAL2,
                        /*!< RCC Clock related to timer TIMx.
                             (Ex. RCC_APB1Periph_TIM2).*/

  HALL_IRQ_CHANNEL2,            /*!< IRQ Channle related to TIMx.
                             (Ex. TIM2_IRQChannel).*/

  HALL_TIMER_REMAPPING2,
                        /*!< It remaps TIMx outputs. It must equal to
                             GPIO_PartialRemap_TIMx, GPIO_FullRemap_TIMx or
                             GPIO_NoRemap_TIMx according the HW availability.*/

  H1_GPIO_PORT2,
                        /*!< HALL sensor H1 channel GPIO input port (if used,
                             after re-mapping). It must be GPIOx x= A, B, ...*/

  H1_GPIO_PIN2,           /*!< HALL sensor H1 channel GPIO output pin (if used,
                             after re-mapping). It must be GPIO_Pin_x x= 0, 1,
                             ...*/

  H2_GPIO_PORT2,
                        /*!< HALL sensor H2 channel GPIO input port (if used,
                             after re-mapping). It must be GPIOx x= A, B, ...*/

  H2_GPIO_PIN2,           /*!< HALL sensor H2 channel GPIO output pin (if used,
                             after re-mapping). It must be GPIO_Pin_x x= 0, 1,
                             ...*/

  H3_GPIO_PORT2,
                        /*!< HALL sensor H3 channel GPIO input port (if used,
                             after re-mapping). It must be GPIOx x= A, B, ...*/

  H3_GPIO_PIN2,         /*!< HALL sensor H3 channel GPIO output pin (if used,
                             after re-mapping). It must be GPIO_Pin_x x= 0, 1,
                             ...*/
/* Alternate functions definition --------------------------------------------*/
  H1_GPIO_AF2,          /*!< HALL sensor H1 alternate
                             functions setting. It must be one
                             of the GPIO_AF_x (x=0,1, ...)
                             according to the defined GPIO port
                             and pin or MC_NULL if not AF setting
                             is not supported by selected microcontroller.*/
  H2_GPIO_AF2,          /*!< HALL sensor H2 alternate
                             functions setting. It must be one
                             of the GPIO_AF_x (x=0,1, ...)
                             according to the defined GPIO port
                             and pin or MC_NULL if not AF setting
                             is not supported by selected microcontroller.*/
  H3_GPIO_AF2,          /*!< HALL sensor H3 alternate
                             functions setting. It must be one
                             of the GPIO_AF_x (x=0,1, ...)
                             according to the defined GPIO port
                             and pin or MC_NULL if not AF setting
                             is not supported by selected microcontroller.*/
};

#endif // STM32F30X

#endif

/****************************************************************************/
/*    MOTOR1  */

#ifdef SINGLEDRIVE
#define NBR_OF_MOTORS 1
#else
#define NBR_OF_MOTORS 2
#endif

/**
  * @brief  PI / PID Speed loop parameters Motor 1
  */
PIParams_t PISpeedParamsM1 =
{
  .hDefKpGain          = (int16_t)PID_SPEED_KP_DEFAULT,       /*!< Default Kp gain, used to initialize Kp gain
                                                                   software variable*/
  .hDefKiGain          = (int16_t)PID_SPEED_KI_DEFAULT,       /*!< Default Ki gain, used to initialize Ki gain
                                                                   software variable*/
  .hKpDivisor          = (uint16_t)SP_KPDIV,                  /*!< Kp gain divisor, used in conjuction with
                                                                   Kp gain allows obtaining fractional values.
                                                                   If FULL_MISRA_C_COMPLIANCY is not defined
                                                                   the divisor is implemented through
                                                                   algebrical right shifts to speed up PI
                                                                   execution. Only in this case this parameter
                                                                   specifies the number of right shifts to be
                                                                   executed */
  .hKiDivisor          = (uint16_t)SP_KIDIV,                  /*!< Ki gain divisor, used in conjuction with
                                                                   Ki gain allows obtaining fractional values.
                                                                   If FULL_MISRA_C_COMPLIANCY is not defined
                                                                   the divisor is implemented through
                                                                   algebrical right shifts to speed up PI
                                                                   execution. Only in this case this parameter
                                                                   specifies the number of right shifts to be
                                                                   executed */
  .wDefMaxIntegralTerm = (int32_t)IQMAX * (int32_t)SP_KIDIV,  /*!< Upper limit used to saturate the integral
                                                                   term given by Ki / Ki divisor * integral of
                                                                   process variable error */
  .wDefMinIntegralTerm = -(int32_t)IQMAX * (int32_t)SP_KIDIV, /*!< Lower limit used to saturate the integral
                                                                   term given by Ki / Ki divisor * integral of
                                                                   process variable error */
  .hDefMaxOutput       = (int16_t)IQMAX,                      /*!< Upper limit used to saturate the PI output
                                                               */
  .hDefMinOutput       = -(int16_t)IQMAX,                     /*!< Lower limit used to saturate the PI output
                                                               */
  .hKpDivisorPOW2      = (uint16_t)SP_KPDIV_LOG,              /*!< Kp gain divisor expressed as power of 2.
                                                                   E.g. if gain divisor is 512 the value
                                                                   must be 9 because 2^9 = 512 */
  .hKiDivisorPOW2      = (uint16_t)SP_KIDIV_LOG               /*!< Ki gain divisor expressed as power of 2.
                                                                   E.g. if gain divisor is 512 the value
                                                                   must be 9 because 2^9 = 512 */
};


/**
  * @brief  PI / PID Iq loop parameters Motor 1
  */
PIParams_t PIIqParamsM1 =
{
  .hDefKpGain          = (int16_t)PID_TORQUE_KP_DEFAULT,   /*!< Default Kp gain, used to initialize Kp gain
                                                                  software variable*/
  .hDefKiGain          = (int16_t)PID_TORQUE_KI_DEFAULT,   /*!< Default Ki gain, used to initialize Ki gain
                                                                software variable*/
  .hKpDivisor          = (uint16_t)TF_KPDIV,               /*!< Kp gain divisor, used in conjuction with
                                                                Kp gain allows obtaining fractional values.
                                                                If FULL_MISRA_C_COMPLIANCY is not defined
                                                                the divisor is implemented through
                                                                algebrical right shifts to speed up PI
                                                                execution. Only in this case this parameter
                                                                specifies the number of right shifts to be
                                                                executed */
  .hKiDivisor          = (uint16_t)TF_KIDIV,               /*!< Ki gain divisor, used in conjuction with
                                                                Ki gain allows obtaining fractional values.
                                                                If FULL_MISRA_C_COMPLIANCY is not defined
                                                                the divisor is implemented through
                                                                algebrical right shifts to speed up PI
                                                                execution. Only in this case this parameter
                                                                specifies the number of right shifts to be
                                                                executed */
  .wDefMaxIntegralTerm = (int32_t)S16_MAX * TF_KIDIV,      /*!< Upper limit used to saturate the integral
                                                                term given by Ki / Ki divisor * integral of
                                                                process variable error */
  .wDefMinIntegralTerm = (int32_t)S16_MIN * TF_KIDIV,      /*!< Lower limit used to saturate the integral
                                                                term given by Ki / Ki divisor * integral of
                                                                process variable error */
  .hDefMaxOutput       = S16_MAX,                          /*!< Upper limit used to saturate the PI output
                                                           */
  .hDefMinOutput       = -S16_MAX,                         /*!< Lower limit used to saturate the PI output
                                                           */
  .hKpDivisorPOW2      = (uint16_t)TF_KPDIV_LOG,           /*!< Kp gain divisor expressed as power of 2.
                                                                E.g. if gain divisor is 512 the value
                                                                must be 9 because 2^9 = 512 */
  .hKiDivisorPOW2      = (uint16_t)TF_KIDIV_LOG            /*!< Ki gain divisor expressed as power of 2.
                                                                E.g. if gain divisor is 512 the value
                                                                must be 9 because 2^9 = 512 */
};


/**
  * @brief  PI / PID Id loop parameters Motor 1
  */
PIParams_t PIIdParamsM1 =
{
  .hDefKpGain          = (int16_t)PID_FLUX_KP_DEFAULT,    /*!< Default Kp gain, used to initialize Kp gain
                                                               software variable*/
  .hDefKiGain          = (int16_t)PID_FLUX_KI_DEFAULT,    /*!< Default Ki gain, used to initialize Ki gain
                                                               software variable*/
  .hKpDivisor          = (uint16_t)TF_KPDIV,              /*!< Kp gain divisor, used in conjuction with
                                                               Kp gain allows obtaining fractional values.
                                                               If FULL_MISRA_C_COMPLIANCY is not defined
                                                               the divisor is implemented through
                                                               algebrical right shifts to speed up PI
                                                               execution. Only in this case this parameter
                                                               specifies the number of right shifts to be
                                                               executed */
  .hKiDivisor          = (uint16_t)TF_KIDIV,              /*!< Ki gain divisor, used in conjuction with
                                                               Ki gain allows obtaining fractional values.
                                                               If FULL_MISRA_C_COMPLIANCY is not defined
                                                               the divisor is implemented through
                                                               algebrical right shifts to speed up PI
                                                               execution. Only in this case this parameter
                                                               specifies the number of right shifts to be
                                                               executed */
  .wDefMaxIntegralTerm = (int32_t)S16_MAX * TF_KIDIV,     /*!< Upper limit used to saturate the integral
                                                               term given by Ki / Ki divisor * integral of
                                                               process variable error */
  .wDefMinIntegralTerm = (int32_t)S16_MIN * TF_KIDIV,     /*!< Lower limit used to saturate the integral
                                                               term given by Ki / Ki divisor * integral of
                                                               process variable error */
  .hDefMaxOutput       = S16_MAX,                         /*!< Upper limit used to saturate the PI output
                                                           */
  .hDefMinOutput       = -S16_MAX,                        /*!< Lower limit used to saturate the PI output
                                                           */
  .hKpDivisorPOW2      = (uint16_t)TF_KPDIV_LOG,          /*!< Kp gain divisor expressed as power of 2.
                                                               E.g. if gain divisor is 512 the value
                                                               must be 9 because 2^9 = 512 */
  .hKiDivisorPOW2      = (uint16_t)TF_KIDIV_LOG           /*!< Ki gain divisor expressed as power of 2.
                                                               E.g. if gain divisor is 512 the value
                                                               must be 9 because 2^9 = 512 */
};

/**
  * @brief  PI Flux Weakening control parameters Motor 1
  */
PIParams_t PIFluxWeakeningParamsM1 =
{
  .hDefKpGain          = (int16_t)FW_KP_GAIN,             /*!< Default Kp gain, used to initialize Kp gain
                                                               software variable*/
  .hDefKiGain          = (int16_t)FW_KI_GAIN,             /*!< Default Ki gain, used to initialize Ki gain
                                                               software variable*/
  .hKpDivisor          = (uint16_t)FW_KPDIV_LOG,          /*!< Kp gain divisor, used in conjuction with
                                                               Kp gain allows obtaining fractional values.
                                                               If FULL_MISRA_C_COMPLIANCY is not defined
                                                               the divisor is implemented through
                                                               algebrical right shifts to speed up PI
                                                               execution. Only in this case this parameter
                                                               specifies the number of right shifts to be
                                                               executed */
  .hKiDivisor          = (uint16_t)FW_KIDIV_LOG,          /*!< Ki gain divisor, used in conjuction with
                                                               Ki gain allows obtaining fractional values.
                                                               If FULL_MISRA_C_COMPLIANCY is not defined
                                                               the divisor is implemented through
                                                               algebrical right shifts to speed up PI
                                                               execution. Only in this case this parameter
                                                               specifies the number of right shifts to be
                                                               executed */
  .wDefMaxIntegralTerm = 0,                               /*!< Upper limit used to saturate the integral
                                                               term given by Ki / Ki divisor * integral of
                                                               process variable error */
  .wDefMinIntegralTerm = (int32_t)(-NOMINAL_CURRENT) * (int32_t)FW_KIDIV, /*!< Lower limit used to saturate the integral
                                                                               term given by Ki / Ki divisor * integral of
                                                                               process variable error */
  .hDefMaxOutput       = 0,                               /*!< Upper limit used to saturate the PI output
                                                           */
  .hDefMinOutput       = -S16_MAX,                        /*!< Lower limit used to saturate the PI output
                                                           */
  .hKpDivisorPOW2      = (uint16_t)FW_KPDIV_LOG,          /*!< Kp gain divisor expressed as power of 2.
                                                               E.g. if gain divisor is 512 the value
                                                               must be 9 because 2^9 = 512 */
  .hKiDivisorPOW2      = (uint16_t)FW_KIDIV_LOG           /*!< Ki gain divisor expressed as power of 2.
                                                               E.g. if gain divisor is 512 the value
                                                               must be 9 because 2^9 = 512 */
};



/**
  * @brief  SpeednTorque Controller parameters Motor 1
  */
SpeednTorqCtrlParams_t SpeednTorqCtrlParamsM1 =
{
  .hSTCFrequencyHz =           		MEDIUM_FREQUENCY_TASK_RATE, 	         /*!< Frequency on which the user updates
                                                                               the torque reference calling
                                                                               STC_CalcTorqueReference method
                                                                               expressed in Hz */
  .hMaxAppPositiveMecSpeed01Hz =	(uint16_t)(MAX_APPLICATION_SPEED/6),  /*!< Application maximum positive value
                                                                               of rotor speed. It's expressed in
                                                                               tenth of mechanical Hertz.*/
  .hMinAppPositiveMecSpeed01Hz =	(uint16_t)(MIN_APPLICATION_SPEED/6),      /*!< Application minimum positive value
                                                                               of rotor speed. It's expressed in
                                                                               tenth of mechanical Hertz.*/
  .hMaxAppNegativeMecSpeed01Hz =	(int16_t)(-MIN_APPLICATION_SPEED/6),    /*!< Application maximum negative value
                                                                               of rotor speed. It's expressed in
                                                                               tenth of mechanical Hertz.*/
  .hMinAppNegativeMecSpeed01Hz =	(int16_t)(-MAX_APPLICATION_SPEED/6),/*!< Application minimum negative value
                                                                               of rotor speed. It's expressed in
                                                                               tenth of mechanical Hertz.*/
  .hMaxPositiveTorque =				(int16_t)NOMINAL_CURRENT,		     /*!< Maximum positive value of motor
                                                                               torque. This value represents
                                                                               actually the maximum Iq current
                                                                               expressed in digit.*/
  .hMinNegativeTorque =				-(int16_t)NOMINAL_CURRENT,                 /*!< Maximum negative value of motor
                                                                               torque. This value represents
                                                                               actually the maximum Iq current
                                                                               expressed in digit.*/
  .bModeDefault =					DEFAULT_CONTROL_MODE,                     /*!< Default STC modality.*/
  .hMecSpeedRef01HzDefault =		(int16_t)(DEFAULT_TARGET_SPEED_RPM/6),    /*!< Default mechanical rotor speed
                                                                               reference expressed in tenths of
                                                                               HZ.*/
  .hTorqueRefDefault =				(int16_t)DEFAULT_TORQUE_COMPONENT,        /*!< Default motor torque reference.
                                                                               This value represents actually the
                                                                               Iq current reference expressed in
                                                                               digit.*/
  .hIdrefDefault =					(int16_t)DEFAULT_FLUX_COMPONENT,      /*!< Default motor torque reference.
                                                                               This value represents actually the
                                                                               Iq current reference expressed in
                                                                               digit.*/
};


/**
  * @brief  Current sensor parameters Motor 1 - base class
  */
PWMnCurrFdbkParams_t PWMnCurrFdbkParamsM1 =
{
  .hPWMperiod          = PWM_PERIOD_CYCLES,                 /*!< It contains the PWM period expressed
                                                                     in timer clock cycles unit:
                                                                     hPWMPeriod = Timer Fclk / Fpwm    */
  .hOffCalibrWaitTicks = OFFCALIBRWAITTICKS,              /*!< Wait time duration before current reading
                                                                calibration expressed in number of calls
                                                                of PWMC_CurrentReadingCalibr with action
                                                                CRC_EXEC */
  .hDTCompCnt          = DTCOMPCNT,                        /*!< It contains half of Dead time expressed
                                                                in timer clock cycles unit:
                                                                hDTCompCnt = (DT(s) * Timer Fclk)/2 */
  .Ton                 = TON,                              /*!< Reserved */
  .Toff                = TOFF                              /*!< Reserved */
};

/**
  * @brief  Current sensor parameters Single Drive - three shunt, STM32F103 LOW/MEDIUM DENSITY
  */
R3_LM1Params_t R3_LM1ParamsSD =
{
  ADC_CLOCK_DIVIDER,                         /*!<  APB2 clock prescaling factor for
                                          ADC peripheral. It must be RCC_PCLK2_DivX
                                          x = 2, 4, 6, 8 */
  TIM_CLOCK_DIVIDER,                 /* System clock divider for Advanded Timer */
/* Current reading A/D Conversions initialization -----------------------------*/
  PHASE_U_CURR_CHANNEL,                  /*!< ADC channel used for conversion of
                                           current Ia. It must be equal to
                                           ADC_Channel_x x= 0, ..., 15*/
  PHASE_U_GPIO_PORT,                  /*!< GPIO port used by hIaChannel. It must
                                           be equal to GPIOx x= A, B, ...*/
  PHASE_U_GPIO_PIN,                      /*!< GPIO pin used by hIaChannel. It must be
                                          equal to GPIO_Pin_x x= 0, 1, ...*/
  SAMPLING_TIME_SEL,               /*!< Sampling time used to convert hIaChannel.
                                          It must be equal to ADC_SampleTime_xCycles5
                                          x= 1, 7, ...*/
  PHASE_V_CURR_CHANNEL,                  /*!< ADC channel used for conversion of
                                           current Ib. It must be equal to
                                           ADC_Channel_x x= 0, ..., 15*/
  PHASE_V_GPIO_PORT,                /*!< GPIO port used by hIbChannel. It must
                                           be equal to GPIOx x= A, B, ...*/
  PHASE_V_GPIO_PIN,                      /*!< GPIO pin used by hIaChannel. It must be
                                          equal to GPIO_Pin_x x= 0, 1, ...*/
  SAMPLING_TIME_SEL,                /*!< Sampling time used to convert hIbChannel.
                                          It must be equal to ADC_SampleTime_xCycles5
                                          x= 1, 7, ...*/
  PHASE_W_CURR_CHANNEL,                  /*!< ADC channel used for conversion of
                                           current Ia. It must be equal to
                                           ADC_Channel_x x= 0, ..., 15*/
  PHASE_W_GPIO_PORT,                /*!< GPIO port used by hIaChannel. It must
                                           be equal to GPIOx x= A, B, ...*/
  PHASE_W_GPIO_PIN,                      /*!< GPIO pin used by hIaChannel. It must be
                                          equal to GPIO_Pin_x x= 0, 1, ...*/
  SAMPLING_TIME_SEL,            /*!< Sampling time used to convert hIaChannel.
                                          It must be equal to ADC_SampleTime_xCycles5
                                          x= 1, 7, ...*/

/* PWM generation parameters --------------------------------------------------*/
  DEAD_TIME_COUNTS,
                                       /*!< Dead time in number of TIM clock
                                         cycles. If CHxN are enabled, it must
                                         contain the dead time to be generated
                                         by the microcontroller, otherwise it
                                         expresses the maximum dead time
                                         generated by driving network */
  REP_COUNTER,                           /*!< It expresses the number of PWM
                                         periods to be elapsed before compare
                                         registers are updated again. In
                                         particular:
                                         RepetitionCounter= (2* #PWM periods)-1*/
  TW_AFTER,                            /*!< It is the sum of dead time plus max
                                         value between rise time and noise time
                                         express in number of TIM clocks.*/
  TW_BEFORE,                           /*!< It is the sampling time express in
                                          number of TIM clocks.*/
  PWM_TIMER_SELECTION,                 /*!< It contains the pointer to the timer
                                          used for PWM generation. */
/* PWM Driving signals initialization ----------------------------------------*/

  PWM_TIMER_REMAPPING,                        /*!< Used only for instance #1, it
                                           remaps TIM1 outputs. It must equal to
                                           GPIO_PartialRemap_TIM1 or
                                           GPIO_FullRemap_TIM1 or
                                           GPIO_NoRemap_TIM1 */
  PHASE_UH_POLARITY,   	           /*!< Channel 1 (high side) output polarity,
                                           it must be TIM_OCPolarity_High or
                                           TIM_OCPolarity_Low */
  PHASE_UH_GPIO_PORT,                                 /*!< Channel 1 (high side) GPIO output
                                         port (if used, after re-mapping).
                                         It must be GPIOx x= A, B, ...*/
  PHASE_UH_GPIO_PIN,                       /*!< Channel 1 (high side) GPIO output pin
                                          (if used, after re-mapping). It must be
                                          GPIO_Pin_x x= 0, 1, ...*/
  IDLE_UH_POLARITY,              /*!< Channel 1 (high side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCIdleState_Set or
                                            TIM_OCIdleState_Reset*/

  PHASE_VH_POLARITY,               /*!< Channel 2 (high side) output polarity,
                                           it must be TIM_OCPolarity_High or
                                           TIM_OCPolarity_Low */
  PHASE_VH_GPIO_PORT,                               /*!< Channel 2 (high side) GPIO output
                                           port (if used, after re-mapping).
                                           It must be GPIOx x= A, B, ...*/
  PHASE_VH_GPIO_PIN,                       /*!< Channel 2 (high side) GPIO output pin
                                          (if used, after re-mapping). It must be
                                          GPIO_Pin_x x= 0, 1, ...*/
  IDLE_VH_POLARITY,              /*!< Channel 2 (high side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCIdleState_Set or
                                            TIM_OCIdleState_Reset*/

  PHASE_WH_POLARITY,            	 /*!< Channel 3 (high side) output polarity,
                                           it must be TIM_OCPolarity_High or
                                           TIM_OCPolarity_Low */
  PHASE_WH_GPIO_PORT,                               /*!< Channel 3 (high side) GPIO output
                                           port (if used, after re-mapping).
                                           It must be GPIOx x= A, B, ...*/
  PHASE_WH_GPIO_PIN,                        /*!< Channel 3 (high side) GPIO output pin
                                          (if used, after re-mapping). It must be
                                          GPIO_Pin_x x= 0, 1, ...*/
  IDLE_WH_POLARITY,              /*!< Channel 3 (high side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCIdleState_Set or
                                            TIM_OCIdleState_Reset*/

  (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING,  /*!< Low side or enabling signals
                                                generation method are defined
                                                here.*/

  PHASE_UL_POLARITY,               /*!< Channel 1N (low side) output polarity,
                                           it must be TIM_OCNPolarity_High or
                                           TIM_OCNPolarity_Low */
  PHASE_UL_GPIO_PORT,                                 /*!< Channel 1N (low side) GPIO output
                                           port (if used, after re-mapping).
                                           It must be GPIOx x= A, B, ...*/
  PHASE_UL_GPIO_PIN,                       /*!< Channel 1N (low side) GPIO output pin
                                          (if used, after re-mapping). It must be
                                          GPIO_Pin_x x= 0, 1, ...*/
  IDLE_UL_POLARITY,              /*!< Channel 1N (low side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCNIdleState_Set or
                                            TIM_OCNIdleState_Reset*/


  PHASE_VL_POLARITY,                 /*!< Channel 2N (low side) output polarity,
                                           it must be TIM_OCNPolarity_High or
                                           TIM_OCNPolarity_Low */
  PHASE_VL_GPIO_PORT,                                 /*!< Channel 2N (low side) GPIO output
                                           port (if used, after re-mapping).
                                           It must be GPIOx x= A, B, ...*/
  PHASE_VL_GPIO_PIN,                     /*!< Channel 2N (low side) GPIO output pin
                                          (if used, after re-mapping). It must be
                                          GPIO_Pin_x x= 0, 1, ...*/
  IDLE_VL_POLARITY,              /*!< Channel 2N (low side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCNIdleState_Set or
                                            TIM_OCNIdleState_Reset*/

  PHASE_WL_POLARITY,             /*!< Channel 3N (low side) output polarity,
                                           it must be TIM_OCNPolarity_High or
                                           TIM_OCNPolarity_Low */
  PHASE_WL_GPIO_PORT,                               /*!< Channel 3N (low side)  GPIO output
                                           port (if used, after re-mapping).
                                           It must be GPIOx x= A, B, ...*/
  PHASE_WL_GPIO_PIN,                    /*!< Channel 3N (low side)  GPIO output pin
                                          (if used, after re-mapping). It must be
                                          GPIO_Pin_x x= 0, 1, ...*/
  IDLE_WL_POLARITY,              /*!< Channel 3N (low side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCNIdleState_Set or
                                            TIM_OCNIdleState_Reset*/
/* PWM Driving signals initialization ----------------------------------------*/
(FunctionalState)SW_OV_CURRENT_PROT_ENABLING, /*!< It enable/disable the management of
                                            an emergency input instantaneously
                                            stopping PWM generation. It must be
                                            either equal to ENABLE or DISABLE */
  (uint16_t)(OVERCURR_FEEDBACK_POLARITY),/*!< Emergency Stop (BKIN) input polarity,
                                           it must be TIM_BreakPolarity_Low or
                                           TIM_BreakPolarity_High */
  EMERGENCY_STOP_GPIO_PORT,             /*!< Emergency Stop (BKIN) GPIO input
                                           port (if used, after re-mapping).
                                           It must be GPIOx x= A, B, ...*/
  EMERGENCY_STOP_GPIO_PIN,              /*!< Emergency Stop (BKIN) GPIO input pin
                                         (if used, after re-mapping). It must be
                                          GPIO_Pin_x x= 0, 1, ...*/
};

/**
  * @brief  Current sensor parameters Single Drive - three shunt, STM32F0X
  */
R3_F0XXParams_t R3_F0XXParamsSD =
{
  ADC_CLOCK_DIVIDER,                         /*!<  APB2 clock prescaling factor for
                                          ADC peripheral. It must be RCC_PCLK2_DivX
                                          x = 2, 4, 6, 8 */
  TIM_CLOCK_DIVIDER,                 /* System clock divider for Advanded Timer */
  MC_IRQ_PWMNCURRFDBK_1,             /*!< MC IRQ number used for TIM1 Update event
                                       and for DMA TC event.*/
/* Current reading A/D Conversions initialization -----------------------------*/
  PHASE_U_CURR_CHANNEL,                  /*!< ADC channel used for conversion of
                                           current Ia. It must be equal to
                                           ADC_Channel_x x= 0, ..., 15*/
  PHASE_U_GPIO_PORT,                  /*!< GPIO port used by hIaChannel. It must
                                           be equal to GPIOx x= A, B, ...*/
  PHASE_U_GPIO_PIN,                      /*!< GPIO pin used by hIaChannel. It must be
                                          equal to GPIO_Pin_x x= 0, 1, ...*/
  SAMPLING_TIME_SEL,               /*!< Sampling time used to convert hIaChannel.
                                          It must be equal to ADC_SampleTime_xCycles5
                                          x= 1, 7, ...*/
  PHASE_V_CURR_CHANNEL,                  /*!< ADC channel used for conversion of
                                           current Ib. It must be equal to
                                           ADC_Channel_x x= 0, ..., 15*/
  PHASE_V_GPIO_PORT,                /*!< GPIO port used by hIbChannel. It must
                                           be equal to GPIOx x= A, B, ...*/
  PHASE_V_GPIO_PIN,                      /*!< GPIO pin used by hIaChannel. It must be
                                          equal to GPIO_Pin_x x= 0, 1, ...*/
  SAMPLING_TIME_SEL,                /*!< Sampling time used to convert hIbChannel.
                                          It must be equal to ADC_SampleTime_xCycles5
                                          x= 1, 7, ...*/
  PHASE_W_CURR_CHANNEL,                  /*!< ADC channel used for conversion of
                                           current Ia. It must be equal to
                                           ADC_Channel_x x= 0, ..., 15*/
  PHASE_W_GPIO_PORT,                /*!< GPIO port used by hIaChannel. It must
                                           be equal to GPIOx x= A, B, ...*/
  PHASE_W_GPIO_PIN,                      /*!< GPIO pin used by hIaChannel. It must be
                                          equal to GPIO_Pin_x x= 0, 1, ...*/
  SAMPLING_TIME_SEL,            /*!< Sampling time used to convert hIaChannel.
                                          It must be equal to ADC_SampleTime_xCycles5
                                          x= 1, 7, ...*/

/* PWM generation parameters --------------------------------------------------*/
  DEAD_TIME_COUNTS,
                                       /*!< Dead time in number of TIM clock
                                         cycles. If CHxN are enabled, it must
                                         contain the dead time to be generated
                                         by the microcontroller, otherwise it
                                         expresses the maximum dead time
                                         generated by driving network */
  REP_COUNTER,                           /*!< It expresses the number of PWM
                                         periods to be elapsed before compare
                                         registers are updated again. In
                                         particular:
                                         RepetitionCounter= (2* #PWM periods)-1*/
  TW_AFTER,                            /*!< It is the sum of dead time plus max
                                         value between rise time and noise time
                                         express in number of TIM clocks.*/
  TW_BEFORE,                           /*!< It is the sampling time express in
                                          number of TIM clocks.*/
  PWM_TIMER_SELECTION,                 /*!< It contains the pointer to the timer
                                          used for PWM generation. */
/* PWM Driving signals initialization ----------------------------------------*/

  PWM_TIMER_REMAPPING,                        /*!< Used only for instance #1, it
                                           remaps TIM1 outputs. It must equal to
                                           GPIO_PartialRemap_TIM1 or
                                           GPIO_FullRemap_TIM1 or
                                           GPIO_NoRemap_TIM1 */
  PHASE_UH_POLARITY,   	           /*!< Channel 1 (high side) output polarity,
                                           it must be TIM_OCPolarity_High or
                                           TIM_OCPolarity_Low */
  PHASE_UH_GPIO_PORT,                                 /*!< Channel 1 (high side) GPIO output
                                         port (if used, after re-mapping).
                                         It must be GPIOx x= A, B, ...*/
  PHASE_UH_GPIO_PIN,                       /*!< Channel 1 (high side) GPIO output pin
                                          (if used, after re-mapping). It must be
                                          GPIO_Pin_x x= 0, 1, ...*/
  IDLE_UH_POLARITY,              /*!< Channel 1 (high side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCIdleState_Set or
                                            TIM_OCIdleState_Reset*/

  PHASE_VH_POLARITY,               /*!< Channel 2 (high side) output polarity,
                                           it must be TIM_OCPolarity_High or
                                           TIM_OCPolarity_Low */
  PHASE_VH_GPIO_PORT,                               /*!< Channel 2 (high side) GPIO output
                                           port (if used, after re-mapping).
                                           It must be GPIOx x= A, B, ...*/
  PHASE_VH_GPIO_PIN,                       /*!< Channel 2 (high side) GPIO output pin
                                          (if used, after re-mapping). It must be
                                          GPIO_Pin_x x= 0, 1, ...*/
  IDLE_VH_POLARITY,              /*!< Channel 2 (high side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCIdleState_Set or
                                            TIM_OCIdleState_Reset*/

  PHASE_WH_POLARITY,            	 /*!< Channel 3 (high side) output polarity,
                                           it must be TIM_OCPolarity_High or
                                           TIM_OCPolarity_Low */
  PHASE_WH_GPIO_PORT,                               /*!< Channel 3 (high side) GPIO output
                                           port (if used, after re-mapping).
                                           It must be GPIOx x= A, B, ...*/
  PHASE_WH_GPIO_PIN,                        /*!< Channel 3 (high side) GPIO output pin
                                          (if used, after re-mapping). It must be
                                          GPIO_Pin_x x= 0, 1, ...*/
  IDLE_WH_POLARITY,              /*!< Channel 3 (high side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCIdleState_Set or
                                            TIM_OCIdleState_Reset*/

  (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING,  /*!< Low side or enabling signals
                                                generation method are defined
                                                here.*/

  PHASE_UL_POLARITY,               /*!< Channel 1N (low side) output polarity,
                                           it must be TIM_OCNPolarity_High or
                                           TIM_OCNPolarity_Low */
  PHASE_UL_GPIO_PORT,                                 /*!< Channel 1N (low side) GPIO output
                                           port (if used, after re-mapping).
                                           It must be GPIOx x= A, B, ...*/
  PHASE_UL_GPIO_PIN,                       /*!< Channel 1N (low side) GPIO output pin
                                          (if used, after re-mapping). It must be
                                          GPIO_Pin_x x= 0, 1, ...*/
  IDLE_UL_POLARITY,              /*!< Channel 1N (low side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCNIdleState_Set or
                                            TIM_OCNIdleState_Reset*/


  PHASE_VL_POLARITY,                 /*!< Channel 2N (low side) output polarity,
                                           it must be TIM_OCNPolarity_High or
                                           TIM_OCNPolarity_Low */
  PHASE_VL_GPIO_PORT,                                 /*!< Channel 2N (low side) GPIO output
                                           port (if used, after re-mapping).
                                           It must be GPIOx x= A, B, ...*/
  PHASE_VL_GPIO_PIN,                     /*!< Channel 2N (low side) GPIO output pin
                                          (if used, after re-mapping). It must be
                                          GPIO_Pin_x x= 0, 1, ...*/
  IDLE_VL_POLARITY,              /*!< Channel 2N (low side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCNIdleState_Set or
                                            TIM_OCNIdleState_Reset*/

  PHASE_WL_POLARITY,             /*!< Channel 3N (low side) output polarity,
                                           it must be TIM_OCNPolarity_High or
                                           TIM_OCNPolarity_Low */
  PHASE_WL_GPIO_PORT,                               /*!< Channel 3N (low side)  GPIO output
                                           port (if used, after re-mapping).
                                           It must be GPIOx x= A, B, ...*/
  PHASE_WL_GPIO_PIN,                    /*!< Channel 3N (low side)  GPIO output pin
                                          (if used, after re-mapping). It must be
                                          GPIO_Pin_x x= 0, 1, ...*/
  IDLE_WL_POLARITY,              /*!< Channel 3N (low side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCNIdleState_Set or
                                            TIM_OCNIdleState_Reset*/
/* PWM Driving signals initialization ----------------------------------------*/
(FunctionalState)SW_OV_CURRENT_PROT_ENABLING, /*!< It enable/disable the management of
                                            an emergency input instantaneously
                                            stopping PWM generation. It must be
                                            either equal to ENABLE or DISABLE */
  (uint16_t)(OVERCURR_FEEDBACK_POLARITY),/*!< Emergency Stop (BKIN) input polarity,
                                           it must be TIM_BreakPolarity_Low or
                                           TIM_BreakPolarity_High */
  EMERGENCY_STOP_GPIO_PORT,             /*!< Emergency Stop (BKIN) GPIO input
                                           port (if used, after re-mapping).
                                           It must be GPIOx x= A, B, ...*/
  EMERGENCY_STOP_GPIO_PIN,              /*!< Emergency Stop (BKIN) GPIO input pin
                                         (if used, after re-mapping). It must be
                                          GPIO_Pin_x x= 0, 1, ...*/
/* Alternate functions definition --------------------------------------------*/
  .bCh1AF   = PHASE_UH_GPIO_AF,                   /*!< Channel 1 (high side) alternate
                                                        functions setting. It must be one
                                                        of the GPIO_AF_x (x=0,1, ...)
                                                        according to the defined GPIO port
                                                        and pin.*/
  .bCh2AF   = PHASE_VH_GPIO_AF,                   /*!< Channel 2 (high side) alternate
                                                        functions setting. It must be one
                                                        of the GPIO_AF_x (x=0,1, ...)
                                                        according to the defined GPIO port
                                                        and pin.*/
  .bCh3AF   = PHASE_WH_GPIO_AF,                   /*!< Channel 3 (high side) alternate
                                                        functions setting. It must be one
                                                        of the GPIO_AF_x (x=0,1, ...)
                                                        according to the defined GPIO port
                                                        and pin.*/
  .bCh1NAF  = PHASE_UL_GPIO_AF,                   /*!< Channel 1 (low side) alternate
                                                        functions setting. It must be one
                                                        of the GPIO_AF_x (x=0,1, ...)
                                                        according to the defined GPIO port
                                                        and pin.*/
  .bCh2NAF  = PHASE_VL_GPIO_AF,                   /*!< Channel 2 (low side) alternate
                                                        functions setting. It must be one
                                                        of the GPIO_AF_x (x=0,1, ...)
                                                        according to the defined GPIO port
                                                        and pin.*/
  .bCh3NAF  = PHASE_WL_GPIO_AF,                   /*!< Channel 3 (low side) alternate
                                                        functions setting. It must be one
                                                        of the GPIO_AF_x (x=0,1, ...)
                                                        according to the defined GPIO port
                                                        and pin.*/
  .bBKINAF  = BRKIN_GPIO_AF,                      /*!< Emergency Stop (BKIN) alternate
                                                        functions setting. It must be one
                                                        of the GPIO_AF_x (x=0,1, ...)
                                                        according to the defined GPIO port
                                                        and pin.*/
};

/**
  * @brief  Current sensor parameters Single Drive - one shunt, STM32F103 LOW/MEDIUM DENSITY
  */
R1_LM1Params_t R1_LM1ParamsSD =
{

  ADC_CLOCK_DIVIDER,                 /*!<  APB2 clock prescaling factor for
                                          ADC peripheral. It must be RCC_PCLK2_DivX
                                          x = 2, 4, 6, 8 */
  TIM_CLOCK_DIVIDER,                 /* System clock divider for Advanded Timer */
  MC_IRQ_PWMNCURRFDBK_1,             /*!< MC IRQ number used for TIM1 Update event
                                       and for DMA TC event.*/
/* Current reading A/D Conversions initialization -----------------------------*/

  PHASE_CURRENTS_CHANNEL,                  /*!< ADC channel used for conversion of
                                           current Ib. It must be equal to
                                           ADC_Channel_x x= 0, ..., 15*/
  PHASE_CURRENTS_GPIO_PORT,                /*!< GPIO port used by hIbChannel. It must
                                           be equal to GPIOx x= A, B, ...*/
  PHASE_CURRENTS_GPIO_PIN,                      /*!< GPIO pin used by hIaChannel. It must be
                                          equal to GPIO_Pin_x x= 0, 1, ...*/
  SAMPLING_TIME_SEL,                /*!< Sampling time used to convert hIbChannel.
                                          It must be equal to ADC_SampleTime_xCycles5
                                          x= 1, 7, ...*/
/* PWM generation parameters --------------------------------------------------*/
  DEAD_TIME_COUNTS,                 /*!< Dead time in number of TIM clock
                                         cycles. If CHxN are enabled, it must
                                         contain the dead time to be generated
                                         by the microcontroller, otherwise it
                                         expresses the maximum dead time
                                         generated by driving network */
  REP_COUNTER,                      /*!< It expresses the number of PWM
                                       periods to be elapsed before compare
                                       registers are updated again. In
                                       particular:
                                       RepetitionCounter= (2* PWM periods) -1*/
  TAFTER,                         /*!< It is the sum of dead time plus rise time
                                       express in number of TIM clocks.*/
  TBEFORE,                            /*!< */
  TMIN,                            /*!< It is the sum of dead time plus rise time
                                       plus sampling time express in numbers of
                                       TIM clocks.*/
  HTMIN,                          /*!< It is the half of hTMin value.*/
  TSAMPLE,                        /*!< It is the sampling time express in
                                       numbers of TIM clocks.*/
  MAX_TRTS,                       /*!< It is the maximum between twice of rise
                                       time express in number of TIM clocks and
                                       twice of sampling time express in numbers
                                       of TIM clocks.*/

/* PWM Driving signals initialization ----------------------------------------*/

  R1_PWM_AUX_TIM,                 /*!< Auxiliary timer used for single shunt ADC
                                       triggering. It should be TIM3 or TIM4.*/
  PWM_TIMER_REMAPPING,            /*!< Used only for instance #1, it
                                       remaps TIM1 outputs. It must equal to
                                       GPIO_PartialRemap_TIM1 or
                                       GPIO_FullRemap_TIM1 or
                                       GPIO_NoRemap_TIM1 */
  PHASE_UH_POLARITY,   	           /*!< Channel 1 (high side) output polarity,
                                           it must be TIM_OCPolarity_High or
                                           TIM_OCPolarity_Low */
  PHASE_UH_GPIO_PORT,                                 /*!< Channel 1 (high side) GPIO output
                                         port (if used, after re-mapping).
                                         It must be GPIOx x= A, B, ...*/
  PHASE_UH_GPIO_PIN,                       /*!< Channel 1 (high side) GPIO output pin
                                          (if used, after re-mapping). It must be
                                          GPIO_Pin_x x= 0, 1, ...*/
  IDLE_UH_POLARITY,              /*!< Channel 1 (high side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCIdleState_Set or
                                            TIM_OCIdleState_Reset*/

  PHASE_VH_POLARITY,               /*!< Channel 2 (high side) output polarity,
                                           it must be TIM_OCPolarity_High or
                                           TIM_OCPolarity_Low */
  PHASE_VH_GPIO_PORT,                               /*!< Channel 2 (high side) GPIO output
                                           port (if used, after re-mapping).
                                           It must be GPIOx x= A, B, ...*/
  PHASE_VH_GPIO_PIN,                       /*!< Channel 2 (high side) GPIO output pin
                                          (if used, after re-mapping). It must be
                                          GPIO_Pin_x x= 0, 1, ...*/
  IDLE_VH_POLARITY,              /*!< Channel 2 (high side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCIdleState_Set or
                                            TIM_OCIdleState_Reset*/

  PHASE_WH_POLARITY,            	 /*!< Channel 3 (high side) output polarity,
                                           it must be TIM_OCPolarity_High or
                                           TIM_OCPolarity_Low */
  PHASE_WH_GPIO_PORT,                               /*!< Channel 3 (high side) GPIO output
                                           port (if used, after re-mapping).
                                           It must be GPIOx x= A, B, ...*/
  PHASE_WH_GPIO_PIN,                        /*!< Channel 3 (high side) GPIO output pin
                                          (if used, after re-mapping). It must be
                                          GPIO_Pin_x x= 0, 1, ...*/
  IDLE_WH_POLARITY,              /*!< Channel 3 (high side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCIdleState_Set or
                                            TIM_OCIdleState_Reset*/

  (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING,  /*!< Low side or enabling signals
                                                generation method are defined
                                                here.*/

  PHASE_UL_POLARITY,               /*!< Channel 1N (low side) output polarity,
                                           it must be TIM_OCNPolarity_High or
                                           TIM_OCNPolarity_Low */
  PHASE_UL_GPIO_PORT,                                 /*!< Channel 1N (low side) GPIO output
                                           port (if used, after re-mapping).
                                           It must be GPIOx x= A, B, ...*/
  PHASE_UL_GPIO_PIN,                       /*!< Channel 1N (low side) GPIO output pin
                                          (if used, after re-mapping). It must be
                                          GPIO_Pin_x x= 0, 1, ...*/
  IDLE_UL_POLARITY,              /*!< Channel 1N (low side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCNIdleState_Set or
                                            TIM_OCNIdleState_Reset*/


  PHASE_VL_POLARITY,                 /*!< Channel 2N (low side) output polarity,
                                           it must be TIM_OCNPolarity_High or
                                           TIM_OCNPolarity_Low */
  PHASE_VL_GPIO_PORT,                                 /*!< Channel 2N (low side) GPIO output
                                           port (if used, after re-mapping).
                                           It must be GPIOx x= A, B, ...*/
  PHASE_VL_GPIO_PIN,                     /*!< Channel 2N (low side) GPIO output pin
                                          (if used, after re-mapping). It must be
                                          GPIO_Pin_x x= 0, 1, ...*/
  IDLE_VL_POLARITY,              /*!< Channel 2N (low side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCNIdleState_Set or
                                            TIM_OCNIdleState_Reset*/

  PHASE_WL_POLARITY,             /*!< Channel 3N (low side) output polarity,
                                           it must be TIM_OCNPolarity_High or
                                           TIM_OCNPolarity_Low */
  PHASE_WL_GPIO_PORT,                               /*!< Channel 3N (low side)  GPIO output
                                           port (if used, after re-mapping).
                                           It must be GPIOx x= A, B, ...*/
  PHASE_WL_GPIO_PIN,                    /*!< Channel 3N (low side)  GPIO output pin
                                          (if used, after re-mapping). It must be
                                          GPIO_Pin_x x= 0, 1, ...*/
  IDLE_WL_POLARITY,              /*!< Channel 3N (low side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCNIdleState_Set or
                                            TIM_OCNIdleState_Reset*/
/* PWM Driving signals initialization ----------------------------------------*/
(FunctionalState)SW_OV_CURRENT_PROT_ENABLING, /*!< It enable/disable the management of
                                            an emergency input instantaneously
                                            stopping PWM generation. It must be
                                            either equal to ENABLE or DISABLE */
  (uint16_t)(OVERCURR_FEEDBACK_POLARITY),/*!< Emergency Stop (BKIN) input polarity,
                                           it must be TIM_BreakPolarity_Low or
                                           TIM_BreakPolarity_High */
  EMERGENCY_STOP_GPIO_PORT,             /*!< Emergency Stop (BKIN) GPIO input
                                           port (if used, after re-mapping).
                                           It must be GPIOx x= A, B, ...*/
  EMERGENCY_STOP_GPIO_PIN,              /*!< Emergency Stop (BKIN) GPIO input pin
                                         (if used, after re-mapping). It must be
                                          GPIO_Pin_x x= 0, 1, ...*/

};

/**
  * @brief  Current sensor parameters Single Drive - one shunt, STM32F100
  */
R1_VL1Params_t R1_VL1ParamsSD =
{
  ADC_CLOCK_DIVIDER,                 /*!<  APB2 clock prescaling factor for
                                          ADC peripheral. It must be RCC_PCLK2_DivX
                                          x = 2, 4, 6, 8 */
  TIM_CLOCK_DIVIDER,                 /* System clock divider for Advanded Timer */
  MC_IRQ_PWMNCURRFDBK_1,             /*!< MC IRQ number used for TIM1 Update event
                                       and for DMA TC event.*/
/* Current reading A/D Conversions initialization -----------------------------*/

  PHASE_CURRENTS_CHANNEL,                  /*!< ADC channel used for conversion of
                                           current Ib. It must be equal to
                                           ADC_Channel_x x= 0, ..., 15*/
  PHASE_CURRENTS_GPIO_PORT,                /*!< GPIO port used by hIbChannel. It must
                                           be equal to GPIOx x= A, B, ...*/
  PHASE_CURRENTS_GPIO_PIN,                      /*!< GPIO pin used by hIaChannel. It must be
                                          equal to GPIO_Pin_x x= 0, 1, ...*/
  SAMPLING_TIME_SEL,                /*!< Sampling time used to convert hIbChannel.
                                          It must be equal to ADC_SampleTime_xCycles5
                                          x= 1, 7, ...*/
/* PWM generation parameters --------------------------------------------------*/
  DEAD_TIME_COUNTS,                  /*!< Dead time in number of TIM clock
                                            cycles. If CHxN are enabled, it must
                                            contain the dead time to be generated
                                            by the microcontroller, otherwise it
                                            expresses the maximum dead time
                                            generated by driving network
                                            It must be equal to (u16)((DEADTIME_NS * 24uL)/1000uL)*/
  REP_COUNTER,              /*!< It expresses the number of PWM
                                           periods to be elapsed before compare
                                           registers are updated again. In
                                           particular:
                                           RepetitionCounter= (2* #PWM periods) -1
                                         */
  TAFTER,     /*!< It must be equal to ((u16)(((DEADTIME_NS+((u16)(TRISE_NS)))*24ul)/1000ul))*/
  TBEFORE,    /*!< It must be equal to (((u16)(((((u16)(SAMPLING_TIME_NS)))*24ul)/1000ul))+1)*/
  TMIN,       /*!< It must be equal to (((u16)(((DEADTIME_NS+((u16)(TRISE_NS))+((u16)(SAMPLING_TIME_NS)))*24uL)/1000ul))+1)*/
  HTMIN,      /*!< It must be equal to (u16)(TMIN >> 1)*/
  TSAMPLE,    /*!< It must be equal to (u16)(((u16)(SAMPLING_TIME_NS) * 24uL)/1000uL)*/
  MAX_TRTS,    /*!< It must be equal to 2*(u16)((((u16)(TRISE_NS)) * 24uL)/1000uL) if TRISE_NS > SAMPLING_TIME_NS
                             to 2*hTSample otherwise */

/* PWM Driving signals initialization ----------------------------------------*/

  R1_PWM_AUX_TIM,                 /*!< Auxiliary timer used for single shunt ADC
                                       triggering. It should be TIM3 or TIM4.*/
  PWM_TIMER_REMAPPING,            /*!< Used only for instance #1, it
                                       remaps TIM1 outputs. It must equal to
                                       GPIO_PartialRemap_TIM1 or
                                       GPIO_FullRemap_TIM1 or
                                       GPIO_NoRemap_TIM1 */
  PHASE_UH_POLARITY,   	           /*!< Channel 1 (high side) output polarity,
                                        it must be TIM_OCPolarity_High or
                                        TIM_OCPolarity_Low */
  PHASE_UH_GPIO_PORT,              /*!< Channel 1 (high side) GPIO output
                                        port (if used, after re-mapping).
                                        It must be GPIOx x= A, B, ...*/
  PHASE_UH_GPIO_PIN,               /*!< Channel 1 (high side) GPIO output pin
                                        (if used, after re-mapping). It must be
                                        GPIO_Pin_x x= 0, 1, ...*/
  IDLE_UH_POLARITY,                /*!< Channel 1 (high side) state (low or high)
                                        when TIM peripheral is in Idle state.
                                        It must be TIM_OCIdleState_Set or
                                        TIM_OCIdleState_Reset*/

  PHASE_VH_POLARITY,               /*!< Channel 2 (high side) output polarity,
                                        it must be TIM_OCPolarity_High or
                                        TIM_OCPolarity_Low */
  PHASE_VH_GPIO_PORT,              /*!< Channel 2 (high side) GPIO output
                                        port (if used, after re-mapping).
                                        It must be GPIOx x= A, B, ...*/
  PHASE_VH_GPIO_PIN,               /*!< Channel 2 (high side) GPIO output pin
                                        (if used, after re-mapping). It must be
                                        GPIO_Pin_x x= 0, 1, ...*/
  IDLE_VH_POLARITY,                /*!< Channel 2 (high side) state (low or high)
                                        when TIM peripheral is in Idle state.
                                        It must be TIM_OCIdleState_Set or
                                        TIM_OCIdleState_Reset*/
  PHASE_WH_POLARITY,               /*!< Channel 3 (high side) output polarity,
                                         it must be TIM_OCPolarity_High or
                                         TIM_OCPolarity_Low */
  PHASE_WH_GPIO_PORT,              /*!< Channel 3 (high side) GPIO output
                                        port (if used, after re-mapping).
                                        It must be GPIOx x= A, B, ...*/
  PHASE_WH_GPIO_PIN,               /*!< Channel 3 (high side) GPIO output pin
                                        (if used, after re-mapping). It must be
                                        GPIO_Pin_x x= 0, 1, ...*/
  IDLE_WH_POLARITY,                /*!< Channel 3 (high side) state (low or high)
                                        when TIM peripheral is in Idle state.
                                        It must be TIM_OCIdleState_Set or
                                        TIM_OCIdleState_Reset*/
  (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING,  /*!< Low side or enabling signals
                                                generation method are defined
                                                here.*/
  PHASE_UL_POLARITY,               /*!< Channel 1N (low side) output polarity,
                                         it must be TIM_OCNPolarity_High or
                                         TIM_OCNPolarity_Low */
  PHASE_UL_GPIO_PORT,              /*!< Channel 1N (low side) GPIO output
                                        port (if used, after re-mapping).
                                        It must be GPIOx x= A, B, ...*/
  PHASE_UL_GPIO_PIN,               /*!< Channel 1N (low side) GPIO output pin
                                        (if used, after re-mapping). It must be
                                        GPIO_Pin_x x= 0, 1, ...*/
  IDLE_UL_POLARITY,                 /*!< Channel 1N (low side) state (low or high)
                                         when TIM peripheral is in Idle state.
                                         It must be TIM_OCNIdleState_Set or
                                         TIM_OCNIdleState_Reset*/


  PHASE_VL_POLARITY,                /*!< Channel 2N (low side) output polarity,
                                         it must be TIM_OCNPolarity_High or
                                         TIM_OCNPolarity_Low */
  PHASE_VL_GPIO_PORT,               /*!< Channel 2N (low side) GPIO output
                                         port (if used, after re-mapping).
                                         It must be GPIOx x= A, B, ...*/
  PHASE_VL_GPIO_PIN,                /*!< Channel 2N (low side) GPIO output pin
                                         (if used, after re-mapping). It must be
                                          GPIO_Pin_x x= 0, 1, ...*/
  IDLE_VL_POLARITY,                 /*!< Channel 2N (low side) state (low or high)
                                          when TIM peripheral is in Idle state.
                                          It must be TIM_OCNIdleState_Set or
                                          TIM_OCNIdleState_Reset*/

  PHASE_WL_POLARITY,                /*!< Channel 3N (low side) output polarity,
                                         it must be TIM_OCNPolarity_High or
                                         TIM_OCNPolarity_Low */
  PHASE_WL_GPIO_PORT,               /*!< Channel 3N (low side)  GPIO output
                                         port (if used, after re-mapping).
                                         It must be GPIOx x= A, B, ...*/
  PHASE_WL_GPIO_PIN,                /*!< Channel 3N (low side)  GPIO output pin
                                          (if used, after re-mapping). It must be
                                          GPIO_Pin_x x= 0, 1, ...*/
  IDLE_WL_POLARITY,                 /*!< Channel 3N (low side) state (low or high)
                                         when TIM peripheral is in Idle state.
                                         It must be TIM_OCNIdleState_Set or
                                         TIM_OCNIdleState_Reset*/
/* PWM Driving signals initialization ----------------------------------------*/
(FunctionalState)SW_OV_CURRENT_PROT_ENABLING, /*!< It enable/disable the management of
                                         an emergency input instantaneously
                                         stopping PWM generation. It must be
                                         either equal to ENABLE or DISABLE */
  (uint16_t)(OVERCURR_FEEDBACK_POLARITY),/*!< Emergency Stop (BKIN) input polarity,
                                           it must be TIM_BreakPolarity_Low or
                                           TIM_BreakPolarity_High */
  EMERGENCY_STOP_GPIO_PORT,           /*!< Emergency Stop (BKIN) GPIO input
                                           port (if used, after re-mapping).
                                           It must be GPIOx x= A, B, ...*/
  EMERGENCY_STOP_GPIO_PIN,            /*!< Emergency Stop (BKIN) GPIO input pin
                                           (if used, after re-mapping). It must be
                                           GPIO_Pin_x x= 0, 1, ...*/
};

/**
  * @brief  Current sensor parameters Single Drive - ICS, STM32F103 LOW/MEDIUM DENSITY
  */
ICS_LM1Params_t ICS_LM1ParamsSD =
{
  ADC_CLOCK_DIVIDER,                         /*!<  APB2 clock prescaling factor for
                                          ADC peripheral. It must be RCC_PCLK2_DivX
                                          x = 2, 4, 6, 8 */
  TIM_CLOCK_DIVIDER,                 /* System clock divider for Advanded Timer */
/* Current reading A/D Conversions initialization -----------------------------*/
  PHASE_U_CURR_CHANNEL,                  /*!< ADC channel used for conversion of
                                           current Ia. It must be equal to
                                           ADC_Channel_x x= 0, ..., 15*/
  PHASE_U_GPIO_PORT,                  /*!< GPIO port used by hIaChannel. It must
                                           be equal to GPIOx x= A, B, ...*/
  PHASE_U_GPIO_PIN,                      /*!< GPIO pin used by hIaChannel. It must be
                                          equal to GPIO_Pin_x x= 0, 1, ...*/
  SAMPLING_TIME_SEL,               /*!< Sampling time used to convert hIaChannel.
                                          It must be equal to ADC_SampleTime_xCycles5
                                          x= 1, 7, ...*/
  PHASE_V_CURR_CHANNEL,                  /*!< ADC channel used for conversion of
                                           current Ib. It must be equal to
                                           ADC_Channel_x x= 0, ..., 15*/
  PHASE_V_GPIO_PORT,                /*!< GPIO port used by hIbChannel. It must
                                           be equal to GPIOx x= A, B, ...*/
  PHASE_V_GPIO_PIN,                      /*!< GPIO pin used by hIaChannel. It must be
                                          equal to GPIO_Pin_x x= 0, 1, ...*/
  SAMPLING_TIME_SEL,                /*!< Sampling time used to convert hIbChannel.
                                          It must be equal to ADC_SampleTime_xCycles5
                                          x= 1, 7, ...*/

/* PWM generation parameters --------------------------------------------------*/
  DEAD_TIME_COUNTS,
                                       /*!< Dead time in number of TIM clock
                                         cycles. If CHxN are enabled, it must
                                         contain the dead time to be generated
                                         by the microcontroller, otherwise it
                                         expresses the maximum dead time
                                         generated by driving network */
  REP_COUNTER,                           /*!< It expresses the number of PWM
                                         periods to be elapsed before compare
                                         registers are updated again. In
                                         particular:
                                         RepetitionCounter= (2* #PWM periods)-1 */
  PWM_TIMER_SELECTION,                 /*!< It contains the pointer to the timer
                                          used for PWM generation. */
/* PWM Driving signals initialization ----------------------------------------*/

  PWM_TIMER_REMAPPING,                /*!< Used only for instance #1, it
                                           remaps TIM1 outputs. It must equal to
                                           GPIO_PartialRemap_TIM1 or
                                           GPIO_FullRemap_TIM1 or
                                           GPIO_NoRemap_TIM1 */
  PHASE_UH_POLARITY,   	              /*!< Channel 1 (high side) output polarity,
                                           it must be TIM_OCPolarity_High or
                                           TIM_OCPolarity_Low */
  PHASE_UH_GPIO_PORT,                 /*!< Channel 1 (high side) GPIO output
                                          port (if used, after re-mapping).
                                          It must be GPIOx x= A, B, ...*/
  PHASE_UH_GPIO_PIN,                 /*!< Channel 1 (high side) GPIO output pin
                                          (if used, after re-mapping). It must be
                                           GPIO_Pin_x x= 0, 1, ...*/
  IDLE_UH_POLARITY,                  /*!< Channel 1 (high side) state (low or high)
                                          when TIM peripheral is in Idle state.
                                          It must be TIM_OCIdleState_Set or
                                          TIM_OCIdleState_Reset*/

  PHASE_VH_POLARITY,                /*!< Channel 2 (high side) output polarity,
                                         it must be TIM_OCPolarity_High or
                                         TIM_OCPolarity_Low */
  PHASE_VH_GPIO_PORT,                /*!< Channel 2 (high side) GPIO output
                                          port (if used, after re-mapping).
                                          It must be GPIOx x= A, B, ...*/
  PHASE_VH_GPIO_PIN,                 /*!< Channel 2 (high side) GPIO output pin
                                          (if used, after re-mapping). It must be
                                          GPIO_Pin_x x= 0, 1, ...*/
  IDLE_VH_POLARITY,                  /*!< Channel 2 (high side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCIdleState_Set or
                                            TIM_OCIdleState_Reset*/

  PHASE_WH_POLARITY,            	   /*!< Channel 3 (high side) output polarity,
                                           it must be TIM_OCPolarity_High or
                                           TIM_OCPolarity_Low */
  PHASE_WH_GPIO_PORT,                  /*!< Channel 3 (high side) GPIO output
                                           port (if used, after re-mapping).
                                           It must be GPIOx x= A, B, ...*/
  PHASE_WH_GPIO_PIN,                   /*!< Channel 3 (high side) GPIO output pin
                                          (if used, after re-mapping). It must be
                                          GPIO_Pin_x x= 0, 1, ...*/
  IDLE_WH_POLARITY,                    /*!< Channel 3 (high side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCIdleState_Set or
                                            TIM_OCIdleState_Reset*/

  (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING,  /*!< Low side or enabling signals
                                                generation method are defined
                                                here.*/

  PHASE_UL_POLARITY,               /*!< Channel 1N (low side) output polarity,
                                           it must be TIM_OCNPolarity_High or
                                           TIM_OCNPolarity_Low */
  PHASE_UL_GPIO_PORT,                                 /*!< Channel 1N (low side) GPIO output
                                           port (if used, after re-mapping).
                                           It must be GPIOx x= A, B, ...*/
  PHASE_UL_GPIO_PIN,                       /*!< Channel 1N (low side) GPIO output pin
                                          (if used, after re-mapping). It must be
                                          GPIO_Pin_x x= 0, 1, ...*/
  IDLE_UL_POLARITY,              /*!< Channel 1N (low side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCNIdleState_Set or
                                            TIM_OCNIdleState_Reset*/


  PHASE_VL_POLARITY,                  /*!< Channel 2N (low side) output polarity,
                                           it must be TIM_OCNPolarity_High or
                                           TIM_OCNPolarity_Low */
  PHASE_VL_GPIO_PORT,                 /*!< Channel 2N (low side) GPIO output
                                           port (if used, after re-mapping).
                                           It must be GPIOx x= A, B, ...*/
  PHASE_VL_GPIO_PIN,                  /*!< Channel 2N (low side) GPIO output pin
                                           (if used, after re-mapping). It must be
                                           GPIO_Pin_x x= 0, 1, ...*/
  IDLE_VL_POLARITY,                   /*!< Channel 2N (low side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCNIdleState_Set or
                                            TIM_OCNIdleState_Reset*/

  PHASE_WL_POLARITY,                  /*!< Channel 3N (low side) output polarity,
                                           it must be TIM_OCNPolarity_High or
                                           TIM_OCNPolarity_Low */
  PHASE_WL_GPIO_PORT,                 /*!< Channel 3N (low side)  GPIO output
                                           port (if used, after re-mapping).
                                           It must be GPIOx x= A, B, ...*/
  PHASE_WL_GPIO_PIN,                  /*!< Channel 3N (low side)  GPIO output pin
                                          (if used, after re-mapping). It must be
                                          GPIO_Pin_x x= 0, 1, ...*/
  IDLE_WL_POLARITY,                   /*!< Channel 3N (low side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCNIdleState_Set or
                                            TIM_OCNIdleState_Reset*/
/* PWM Driving signals initialization ----------------------------------------*/
(FunctionalState)SW_OV_CURRENT_PROT_ENABLING, /*!< It enable/disable the management of
                                            an emergency input instantaneously
                                            stopping PWM generation. It must be
                                            either equal to ENABLE or DISABLE */
  (uint16_t)(OVERCURR_FEEDBACK_POLARITY),/*!< Emergency Stop (BKIN) input polarity,
                                           it must be TIM_BreakPolarity_Low or
                                           TIM_BreakPolarity_High */
  EMERGENCY_STOP_GPIO_PORT,             /*!< Emergency Stop (BKIN) GPIO input
                                           port (if used, after re-mapping).
                                           It must be GPIOx x= A, B, ...*/
  EMERGENCY_STOP_GPIO_PIN,              /*!< Emergency Stop (BKIN) GPIO input pin
                                         (if used, after re-mapping). It must be
                                          GPIO_Pin_x x= 0, 1, ...*/
};

/**
  * @brief  Current sensor parameters Motor 1 - three shunt
  */
R3_DDParams_t R3_DDParamsM1 =
{
  .wADC_Clock_Divider         =	ADC_CLOCK_DIVIDER,                       /*!<  APB2 clock prescaling factor for
                                                                        ADC peripheral. It must be RCC_PCLK2_DivX
                                                                        x = 2, 4, 6, 8 */
  .bTim_Clock_Divider         =	TIM_CLOCK_DIVIDER,                  /* System clock divider for Advanded Timer */
/* Dual MC parameters --------------------------------------------------------*/
  .bInstanceNbr             =	1,                                    /*!< Instance number. Necessary to
                                                                           properly synchronize TIM8 with
                                                                           TIM1 at peripheral initializations
                                                                            */
  .Tw                       =	MAX_TWAIT,                            /*!< It is used for switching the context
                                                                         in dual MC. It contains biggest delay
                                                                         (expressed in counter ticks) between
                                                                         the counter crest and ADC latest
                                                                         trigger  */
  .bFreqRatio               =	FREQ_RATIO,                                    /*!< It is used in case of dual MC to
                                                                         synchronize TIM1 and TIM8. It has
                                                                         effect only on the second instanced
                                                                         object and must be equal to the
                                                                         ratio between the two PWM frequencies
                                                                         (higher/lower). Supported values are
                                                                         1, 2 or 3 */

  .bIsHigherFreqTim         =	FREQ_RELATION,                           /*!< When bFreqRatio is greather than 1
                                                                        this param is used to indicate if this
                                                                        instance is the one with the highest
                                                                        frequency. Allowed value are: HIGHER_FREQ
                                                                        or LOWER_FREQ */
  .IRQnb                    =	MC_IRQ_PWMNCURRFDBK_1,                     /*!< MC IRQ number used as TIMx Update
                                           event */
/* Current reading A/D Conversions initialization -----------------------------*/
  .bIaChannel              =	PHASE_U_CURR_CHANNEL,                  /*!< ADC channel used for conversion of
                                                                         current Ia. It must be equal to
                                                                         ADC_Channel_x x= 0, ..., 15*/
  .hIaPort                 =	PHASE_U_GPIO_PORT,                  /*!< GPIO port used by hIaChannel. It must
                                                                         be equal to GPIOx x= A, B, ...*/
  .hIaPin                  =	PHASE_U_GPIO_PIN,                      /*!< GPIO pin used by hIaChannel. It must be
                                                                        equal to GPIO_Pin_x x= 0, 1, ...*/
  .b_IaSamplingTime        =	SAMPLING_TIME_SEL,               /*!< Sampling time used to convert hIaChannel.
                                                                        It must be equal to ADC_SampleTime_xCycles5
                                                                        x= 1, 7, ...*/
  .bIbChannel              =	PHASE_V_CURR_CHANNEL,                  /*!< ADC channel used for conversion of
                                                                         current Ib. It must be equal to
                                                                         ADC_Channel_x x= 0, ..., 15*/
  .hIbPort                 =	PHASE_V_GPIO_PORT,                /*!< GPIO port used by hIbChannel. It must
                                                                         be equal to GPIOx x= A, B, ...*/
  .hIbPin                  =	PHASE_V_GPIO_PIN,                      /*!< GPIO pin used by hIaChannel. It must be
                                                                        equal to GPIO_Pin_x x= 0, 1, ...*/
  .b_IbSamplingTime        =	SAMPLING_TIME_SEL,                /*!< Sampling time used to convert hIbChannel.
                                                                        It must be equal to ADC_SampleTime_xCycles5
                                                                        x= 1, 7, ...*/
  .bIcChannel              =	PHASE_W_CURR_CHANNEL,                  /*!< ADC channel used for conversion of
                                                                         current Ia. It must be equal to
                                                                         ADC_Channel_x x= 0, ..., 15*/
  .hIcPort                 =	PHASE_W_GPIO_PORT,                /*!< GPIO port used by hIaChannel. It must
                                                                         be equal to GPIOx x= A, B, ...*/
  .hIcPin                  =	PHASE_W_GPIO_PIN,                      /*!< GPIO pin used by hIaChannel. It must be
                           =                                            equal to GPIO_Pin_x x= 0, 1, ...*/
  .b_IcSamplingTime        =	SAMPLING_TIME_SEL,            /*!< Sampling time used to convert hIaChannel.
                                                                   It must be equal to ADC_SampleTime_xCycles5
                                                                   x= 1, 7, ...*/

/* PWM generation parameters --------------------------------------------------*/
  .hDeadTime                  =	DEAD_TIME_COUNTS,                    /*!< Dead time in number of TIM clock
                                                                          cycles. If CHxN are enabled, it must
                                                                          contain the dead time to be generated
                                                                          by the microcontroller, otherwise it
                                                                          expresses the maximum dead time
                                                                          generated by driving network */
  .bRepetitionCounter         =	REP_COUNTER,                         /*!< It expresses the number of PWM
                                                                          periods to be elapsed before compare
                                                                          registers are updated again. In
                                                                          particular:
                                                                          RepetitionCounter= (2* #PWM periods)-1*/
  .hTafter                    =	TW_AFTER,                             /*!< It is the sum of dead time plus max
                                                                          value between rise time and noise time
                                                                          express in number of TIM clocks.*/
  .hTbefore                   =	TW_BEFORE,                           /*!< It is the sampling time express in
                                                                          number of TIM clocks.*/
  .TIMx                       =	PWM_TIMER_SELECTION,                               /*!< It contains the pointer to the timer
                                           used for PWM generation. */
/* PWM Driving signals initialization ----------------------------------------*/

  .wTIM1Remapping                       =	PWM_TIMER_REMAPPING,                        /*!< Used only for instance #1, it
                                                                                     remaps TIM1 outputs. It must equal to
                                                                                     GPIO_PartialRemap_TIM1 or
                                                                                     GPIO_FullRemap_TIM1 or
                                                                                     GPIO_NoRemap_TIM1 */
  .hCh1Polarity                         =	PHASE_UH_POLARITY,   	              /*!< Channel 1 (high side) output polarity,
                                                                                     it must be TIM_OCPolarity_High or
                                                                                     TIM_OCPolarity_Low */
  .hCh1Port                             =	PHASE_UH_GPIO_PORT,                 /*!< Channel 1 (high side) GPIO output
                                                                                     port (if used, after re-mapping).
                                                                                     It must be GPIOx x= A, B, ...*/
  .hCh1Pin                              =	PHASE_UH_GPIO_PIN,                  /*!< Channel 1 (high side) GPIO output pin
                                                                                    (if used, after re-mapping). It must be
                                                                                     GPIO_Pin_x x= 0, 1, ...*/
  .hCh1IdleState                        =	IDLE_UH_POLARITY,                   /*!< Channel 1 (high side) state (low or high)
                                                                                      when TIM peripheral is in Idle state.
                                                                                      It must be TIM_OCIdleState_Set or
                                                                                      TIM_OCIdleState_Reset*/

  .hCh2Polarity                         =   PHASE_VH_POLARITY,                   /*!< Channel 2 (high side) output polarity,
                                                                                    it must be TIM_OCPolarity_High or
                                                                                     TIM_OCPolarity_Low */
  .hCh2Port                             =   PHASE_VH_GPIO_PORT,                  /*!< Channel 2 (high side) GPIO output
                                                                                       port (if used, after re-mapping).
                                                                                     It must be GPIOx x= A, B, ...*/
  .hCh2Pin                              =   PHASE_VH_GPIO_PIN,                       /*!< Channel 2 (high side) GPIO output pin
                                                                                   (if used, after re-mapping). It must be
                                                                                    GPIO_Pin_x x= 0, 1, ...*/
  .hCh2IdleState                        =   IDLE_VH_POLARITY,              /*!< Channel 2 (high side) state (low or high)
                                                                                      when TIM peripheral is in Idle state.
                                                                                      It must be TIM_OCIdleState_Set or
                                                                                      TIM_OCIdleState_Reset*/


  .hCh3Polarity                         =	PHASE_WH_POLARITY,            	 /*!< Channel 3 (high side) output polarity,
                                                                                     it must be TIM_OCPolarity_High or
                                                                                     TIM_OCPolarity_Low */
  .hCh3Port                             =	PHASE_WH_GPIO_PORT,                               /*!< Channel 3 (high side) GPIO output
                                                                                     port (if used, after re-mapping).
                                                                                     It must be GPIOx x= A, B, ...*/
  .hCh3Pin                              =	PHASE_WH_GPIO_PIN,                        /*!< Channel 3 (high side) GPIO output pin
                                                                                    (if used, after re-mapping). It must be
                                                                                    GPIO_Pin_x x= 0, 1, ...*/
  .hCh3IdleState                        =	IDLE_WH_POLARITY,              /*!< Channel 3 (high side) state (low or high)
                                                                                      when TIM peripheral is in Idle state.
                                                                                      It must be TIM_OCIdleState_Set or
                                                                                      TIM_OCIdleState_Reset*/

  .LowSideOutputs                       =	(LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING,  /*!< Low side or enabling signals
                                                                                          generation method are defined
                                                                                          here.*/

  .hCh1NPolarity                        =	PHASE_UL_POLARITY,               /*!< Channel 1N (low side) output polarity,
                                                                                     it must be TIM_OCNPolarity_High or
                                                                                     TIM_OCNPolarity_Low */
  .hCh1NPort                            =	PHASE_UL_GPIO_PORT,                                 /*!< Channel 1N (low side) GPIO output
                                                                                     port (if used, after re-mapping).
                                                                                     It must be GPIOx x= A, B, ...*/
  .hCh1NPin                             =	PHASE_UL_GPIO_PIN,                       /*!< Channel 1N (low side) GPIO output pin
                                                                                    (if used, after re-mapping). It must be
                                                                                    GPIO_Pin_x x= 0, 1, ...*/
  .hCh1NIdleState                       =	IDLE_UL_POLARITY,              /*!< Channel 1N (low side) state (low or high)
                                                                                      when TIM peripheral is in Idle state.
                                                                                      It must be TIM_OCNIdleState_Set or
                                                                                      TIM_OCNIdleState_Reset*/


  .hCh2NPolarity                        =	PHASE_VL_POLARITY,                 /*!< Channel 2N (low side) output polarity,
                                                                                     it must be TIM_OCNPolarity_High or
                                                                                     TIM_OCNPolarity_Low */
  .hCh2NPort                            =	PHASE_VL_GPIO_PORT,                                 /*!< Channel 2N (low side) GPIO output
                                                                                     port (if used, after re-mapping).
                                                                                     It must be GPIOx x= A, B, ...*/
  .hCh2NPin                             =	PHASE_VL_GPIO_PIN,                     /*!< Channel 2N (low side) GPIO output pin
                                                                                    (if used, after re-mapping). It must be
                                                                                    GPIO_Pin_x x= 0, 1, ...*/
  .hCh2NIdleState                       =	IDLE_VL_POLARITY,              /*!< Channel 2N (low side) state (low or high)
                                                                                      when TIM peripheral is in Idle state.
                                                                                      It must be TIM_OCNIdleState_Set or
                                                                                      TIM_OCNIdleState_Reset*/

  .hCh3NPolarity                        =	PHASE_WL_POLARITY,             /*!< Channel 3N (low side) output polarity,
                                                                                     it must be TIM_OCNPolarity_High or
                                                                                     TIM_OCNPolarity_Low */
  .hCh3NPort                            =	PHASE_WL_GPIO_PORT,                               /*!< Channel 3N (low side)  GPIO output
                                                                                     port (if used, after re-mapping).
                                                                                     It must be GPIOx x= A, B, ...*/
  .hCh3NPin                             =	PHASE_WL_GPIO_PIN,                    /*!< Channel 3N (low side)  GPIO output pin
                                                                                    (if used, after re-mapping). It must be
                                                                                    GPIO_Pin_x x= 0, 1, ...*/
  .hCh3NIdleState                       =	IDLE_WL_POLARITY,              /*!< Channel 3N (low side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCNIdleState_Set or
                                            TIM_OCNIdleState_Reset*/
/* PWM Driving signals initialization ----------------------------------------*/
  .EmergencyStop                =	(FunctionalState)SW_OV_CURRENT_PROT_ENABLING, /*!< It enable/disable the management of
                                                                              an emergency input instantaneously
                                                                              stopping PWM generation. It must be
                                                                              either equal to ENABLE or DISABLE */
  .hBKINPolarity                =	(uint16_t)(OVERCURR_FEEDBACK_POLARITY),/*!< Emergency Stop (BKIN) input polarity,
                                                                             it must be TIM_BreakPolarity_Low or
                                                                             TIM_BreakPolarity_High */
  .hBKINPort                    =	EMERGENCY_STOP_GPIO_PORT,             /*!< Emergency Stop (BKIN) GPIO input
                                                                             port (if used, after re-mapping).
                                                                             It must be GPIOx x= A, B, ...*/
  .hBKINPin                     =	EMERGENCY_STOP_GPIO_PIN,              /*!< Emergency Stop (BKIN) GPIO input pin
                                                                               (if used, after re-mapping). It must be
                                                                                GPIO_Pin_x x= 0, 1, ...*/
};

#if defined(STM32F30X) /* To be fixed adding dummy defines to avoid conditional compilation (if required) */

/**
  * @brief  Internal OPAMP parameters Motor 1 - three shunt - F30x - Independent Resources
  */
R3_4_F30XOPAMPParams_t R3_4_F30XOPAMPParamsM1 =
{
/* Internal OPAMP1 settings --------------------------------------------------*/
  .wOPAMP_Selection                       = OPAMP1_SELECTION,		      /*!< First OPAMP selection. It must be
                                                                                     either equal to
                                                                                     OPAMP_Selection_OPAMP1 or
                                                                                     OPAMP_Selection_OPAMP3.*/
  .bOPAMP_InvertingInput_MODE             = OPAMP1_INVERTINGINPUT_MODE,         /*!< First OPAMP inverting input mode.
                                                                                     It must be either equal to EXT_MODE
                                                                                     or INT_MODE.*/
  .wOPAMP_InvertingInput                  = OPAMP1_INVERTINGINPUT,              /*!< First OPAMP inverting input pin.

                                                                                     If wOPAMP_Selection is
                                                                                     OPAMP_Selection_OPAMP1 it
                                                                                     must be one of the following:
                                                                                     OPAMP1_InvertingInput_PC5 or
                                                                                     OPAMP1_InvertingInput_PA3 if the
                                                                                     bOPAMP_InvertingInput_MODE is
                                                                                     EXT_MODE or
                                                                                     OPAMP1_InvertingInput_PGA or
                                                                                     OPAMP1_InvertingInput_FOLLOWER if the
                                                                                     bOPAMP_InvertingInput_MODE is
                                                                                     INT_MODE.

                                                                                     If wOPAMP_Selection is
                                                                                     OPAMP_Selection_OPAMP3 it
                                                                                     must be one of the following:
                                                                                     OPAMP3_InvertingInput_PB10 or
                                                                                     OPAMP3_InvertingInput_PB2 if the
                                                                                     bOPAMP_InvertingInput_MODE is
                                                                                     EXT_MODE or
                                                                                     OPAMP3_InvertingInput_PGA or
                                                                                     OPAMP3_InvertingInput_FOLLOWER if the
                                                                                     bOPAMP_InvertingInput_MODE is
                                                                                     INT_MODE.*/
  .hOPAMP_InvertingInput_GPIO_PORT        = OPAMP1_INVERTINGINPUT_GPIO_PORT,    /*!< First OPAMP inverting input GPIO port
                                                                                     as defined in wOPAMP_InvertingInput.
                                                                                     It must be GPIOx x= A, B, ... if
                                                                                     bOPAMP_InvertingInput_MODE is
                                                                                     EXT_MODE, otherwise can be dummy.*/
  .hOPAMP_InvertingInput_GPIO_PIN         = OPAMP1_INVERTINGINPUT_GPIO_PIN,     /*!< First OPAMP inverting input GPIO pin
                                                                                     as defined in wOPAMP_InvertingInput.
                                                                                     It must be GPIO_Pin_x x= 0, 1, ... if
                                                                                     bOPAMP_InvertingInput_MODE is
                                                                                     EXT_MODE, otherwise can be dummy.*/
  .wOPAMP_NonInvertingInput_PHA           = OPAMP1_NONINVERTINGINPUT_PHA,       /*!< First OPAMP non inverting input first
                                                                                     selection.

                                                                                     If wOPAMP_Selection is
                                                                                     OPAMP_Selection_OPAMP1 it
                                                                                     must be one of the following:
                                                                                     OPAMP1_NonInvertingInput_PA7,
                                                                                     OPAMP1_NonInvertingInput_PA5,
                                                                                     OPAMP1_NonInvertingInput_PA3,
                                                                                     OPAMP1_NonInvertingInput_PA1.

                                                                                     If wOPAMP_Selection is
                                                                                     OPAMP_Selection_OPAMP3 it
                                                                                     must be one of the following:
                                                                                     OPAMP3_NonInvertingInput_PB13,
                                                                                     OPAMP3_NonInvertingInput_PA5,
                                                                                     OPAMP3_NonInvertingInput_PA1,
                                                                                     OPAMP3_NonInvertingInput_PB0.*/
  .hOPAMP_NonInvertingInput_PHA_GPIO_PORT = OPAMP1_NONINVERTINGINPUT_PHA_GPIO_PORT,
                                                                                /*!< First OPAMP non inverting input GPIO
                                                                                     port as defined in
                                                                                     wOPAMP_NonInvertingInput_PHA.
                                                                                     It must be GPIOx x= A, B, ...*/
  .hOPAMP_NonInvertingInput_PHA_GPIO_PIN  = OPAMP1_NONINVERTINGINPUT_PHA_GPIO_PIN,
                                                                                /*!< First OPAMP non inverting input GPIO
                                                                                     pin as defined in
                                                                                     wOPAMP_NonInvertingInput_PHA.
                                                                                     It must be GPIO_Pin_x x= 0, 1, ...*/
  .wOPAMP_NonInvertingInput_PHB           = OPAMP1_NONINVERTINGINPUT_PHB,       /*!< First OPAMP non inverting input 2nd
                                                                                     selection.

                                                                                     If wOPAMP_Selection is
                                                                                     OPAMP_Selection_OPAMP1 it
                                                                                     must be one of the following:
                                                                                     OPAMP1_NonInvertingInput_PA7,
                                                                                     OPAMP1_NonInvertingInput_PA5,
                                                                                     OPAMP1_NonInvertingInput_PA3,
                                                                                     OPAMP1_NonInvertingInput_PA1.

                                                                                     If wOPAMP_Selection is
                                                                                     OPAMP_Selection_OPAMP3 it
                                                                                     must be one of the following:
                                                                                     OPAMP3_NonInvertingInput_PB13,
                                                                                     OPAMP3_NonInvertingInput_PA5,
                                                                                     OPAMP3_NonInvertingInput_PA1,
                                                                                     OPAMP3_NonInvertingInput_PB0.

                                                                                     Note: It must be the same PIN of
                                                                                     wOPAMP2_NonInvertingInput_PHB.*/
  .hOPAMP_NonInvertingInput_PHB_GPIO_PORT = OPAMP1_NONINVERTINGINPUT_PHB_GPIO_PORT,
                                                                                /*!< First OPAMP non inverting input GPIO
                                                                                     port as defined in
                                                                                     wOPAMP_NonInvertingInput_PHB.
                                                                                     It must be GPIOx x= A, B, ...*/
  .hOPAMP_NonInvertingInput_PHB_GPIO_PIN  = OPAMP1_NONINVERTINGINPUT_PHB_GPIO_PIN,
                                                                                /*!< First OPAMP non inverting input GPIO
                                                                                     pin as defined in
                                                                                     wOPAMP_NonInvertingInput_PHB.
                                                                                     It must be GPIO_Pin_x x= 0, 1, ...*/
  .hOPAMP_Output_GPIO_PORT                = OPAMP1_OUT_GPIO_PORT,
                                                                                /*!< First OPAMP output GPIO port.
                                                                                     It must be GPIOx x= A, B, ...
                                                                                     Note: Output pin is PA2 for OPAMP1,
                                                                                     PB1 for OPAMP3.*/
  .hOPAMP_Output_GPIO_PIN                 = OPAMP1_OUT_GPIO_PIN,
                                                                                /*!< First OPAMP output GPIO pin.
                                                                                     It must be GPIO_Pin_x x= 0, 1, ...
                                                                                     Note: Output pin is PA2 for OPAMP1,
                                                                                     PB1 for OPAMP3.*/
/* Internal OPAMP2 settings --------------------------------------------------*/
  .wOPAMP2_Selection                       = OPAMP2_SELECTION,                   /*!< Second OPAMP selection. It must be
                                                                                      either equal to
                                                                                      OPAMP_Selection_OPAMP2 or
                                                                                      OPAMP_Selection_OPAMP4.*/
  .bOPAMP2_InvertingInput_MODE             = OPAMP2_INVERTINGINPUT_MODE,         /*!< Second OPAMP inverting input mode.
                                                                                      It must be either equal to EXT_MODE
                                                                                      or INT_MODE.*/
  .wOPAMP2_InvertingInput                  = OPAMP2_INVERTINGINPUT,              /*!< Second OPAMP inverting input pin.

                                                                                      If wOPAMP2_Selection is
                                                                                      OPAMP_Selection_OPAMP2 it
                                                                                      must be one of the following:
                                                                                      OPAMP2_InvertingInput_PC5 or
                                                                                      OPAMP2_InvertingInput_PA5 if the
                                                                                      bOPAMP_InvertingInput_MODE is
                                                                                      EXT_MODE or
                                                                                      OPAMP2_InvertingInput_PGA or
                                                                                      OPAMP2_InvertingInput_FOLLOWER if the
                                                                                      bOPAMP_InvertingInput_MODE is
                                                                                      INT_MODE.

                                                                                      If wOPAMP2_Selection is
                                                                                      OPAMP_Selection_OPAMP4 it
                                                                                      must be one of the following:
                                                                                      OPAMP4_InvertingInput_PB10 or
                                                                                      OPAMP4_InvertingInput_PD8 if the
                                                                                      bOPAMP2_InvertingInput_MODE is
                                                                                      EXT_MODE or
                                                                                      OPAMP4_InvertingInput_PGA or
                                                                                      OPAMP4_InvertingInput_FOLLOWER if the
                                                                                      InvertingInput_MODE is INT_MODE
                                                                                      bOPAMP2_InvertingInput_MODE is
                                                                                      INT_MODE.*/
  .hOPAMP2_InvertingInput_GPIO_PORT        = OPAMP2_INVERTINGINPUT_GPIO_PORT,    /*!< 2nd OPAMP inverting input GPIO port
                                                                                      as defined in wOPAMP2_InvertingInput.
                                                                                      It must be GPIOx x= A, B, ...if
                                                                                      bOPAMP2_InvertingInput_MODE is
                                                                                      EXT_MODE, otherwise can be dummy.*/
  .hOPAMP2_InvertingInput_GPIO_PIN         = OPAMP2_INVERTINGINPUT_GPIO_PIN,     /*!< 2nd OPAMP inverting input GPIO pin as
                                                                                      defined in wOPAMP2_InvertingInput.
                                                                                      It must be GPIO_Pin_x x= 0, 1, ...if
                                                                                      bOPAMP2_InvertingInput_MODE is
                                                                                      EXT_MODE, otherwise can be dummy.*/
  .wOPAMP2_NonInvertingInput_PHB           = OPAMP2_NONINVERTINGINPUT_PHB,       /*!< 2nd OPAMP non inverting input
                                                                                      first selection.

                                                                                      If wOPAMP2_Selection is
                                                                                      OPAMP_Selection_OPAMP2 it
                                                                                      must be one of the following:
                                                                                      OPAMP2_NonInvertingInput_PD14,
                                                                                      OPAMP2_NonInvertingInput_PB14,
                                                                                      OPAMP2_NonInvertingInput_PB0,
                                                                                      OPAMP2_NonInvertingInput_PA7.

                                                                                      If wOPAMP2_Selection is
                                                                                      OPAMP_Selection_OPAMP4 it
                                                                                      must be one of the following:
                                                                                      OPAMP4_NonInvertingInput_PD11,
                                                                                      OPAMP4_NonInvertingInput_PB11,
                                                                                      OPAMP4_NonInvertingInput_PA4,
                                                                                      OPAMP4_NonInvertingInput_PB13.

                                                                                      Note: It must be the same PIN of
                                                                                      wOPAMP_NonInvertingInput_PHB.*/
  .hOPAMP2_NonInvertingInput_PHB_GPIO_PORT = OPAMP2_NONINVERTINGINPUT_PHB_GPIO_PORT,
                                                                                 /*!< Second OPAMP non inverting input GPIO
                                                                                      port as defined in
                                                                                      wOPAMP2_NonInvertingInput_PHB.
                                                                                      It must be GPIOx x= A, B, ...*/
  .hOPAMP2_NonInvertingInput_PHB_GPIO_PIN  = OPAMP2_NONINVERTINGINPUT_PHB_GPIO_PIN,
                                                                                 /*!< Second OPAMP non inverting input GPIO
                                                                                      pin as defined in
                                                                                      wOPAMP2_NonInvertingInput_PHB.
                                                                                      It must be GPIO_Pin_x x= 0, 1, ...*/
  .wOPAMP2_NonInvertingInput_PHC           = OPAMP2_NONINVERTINGINPUT_PHC,       /*!< Second OPAMP non inverting input 2nd
                                                                                      selection.

                                                                                      If wOPAMP2_Selection is
                                                                                      OPAMP_Selection_OPAMP2 it
                                                                                      must be one of the following:
                                                                                      OPAMP2_NonInvertingInput_PD14,
                                                                                      OPAMP2_NonInvertingInput_PB14,
                                                                                      OPAMP2_NonInvertingInput_PB0,
                                                                                      OPAMP2_NonInvertingInput_PA7.

                                                                                      If wOPAMP2_Selection is
                                                                                      OPAMP_Selection_OPAMP4 it
                                                                                      must be one of the following:
                                                                                      OPAMP4_NonInvertingInput_PD11,
                                                                                      OPAMP4_NonInvertingInput_PB11,
                                                                                      OPAMP4_NonInvertingInput_PA4,
                                                                                      OPAMP4_NonInvertingInput_PB13.*/
  .hOPAMP2_NonInvertingInput_PHC_GPIO_PORT = OPAMP2_NONINVERTINGINPUT_PHC_GPIO_PORT,
                                                                                 /*!< Second OPAMP non inverting input GPIO
                                                                                      port as defined in
                                                                                      wOPAMP2_NonInvertingInput_PHC.
                                                                                      It must be GPIOx x= A, B, ...*/
  .hOPAMP2_NonInvertingInput_PHC_GPIO_PIN  = OPAMP2_NONINVERTINGINPUT_PHC_GPIO_PIN,
                                                                                 /*!< Second OPAMP non inverting input GPIO
                                                                                      pin as defined in
                                                                                      wOPAMP2_NonInvertingInput_PHC.
                                                                                      It must be GPIO_Pin_x x= 0, 1, ...*/
  .hOPAMP2_Output_GPIO_PORT                = OPAMP2_OUT_GPIO_PORT,
                                                                                 /*!< Second OPAMP output GPIO port.
                                                                                      It must be GPIOx x= A, B, ...
                                                                                      Note: Output pin is PA6 for OPAMP2,
                                                                                      PB12 for OPAMP4.*/
  .hOPAMP2_Output_GPIO_PIN                 = OPAMP2_OUT_GPIO_PIN,
                                                                                 /*!< Second OPAMP output GPIO pin.
                                                                                      It must be GPIO_Pin_x x= 0, 1, ...
                                                                                      Note: Output pin is PA6 for OPAMP2,
                                                                                      PB12 for OPAMP4.*/
/* Common settings -----------------------------------------------------------*/
  .wOPAMP_PGAGain   = OPAMP_PGAGAIN,                      /*!< It defines the OPAMP PGA gains.
                                                               It must be one of the following:
                                                               OPAMP_OPAMP_PGAGain_2,
                                                               OPAMP_OPAMP_PGAGain_4,
                                                               OPAMP_OPAMP_PGAGain_8,
                                                               OPAMP_OPAMP_PGAGain_16.
                                                               This value is taken in account
                                                               just if wOPAMPx_InvertingInput is
                                                               equal to OPAMP2_InvertingInput_PGA*/
  .OPAMP_PGAConnect = OPAMP_PGACONNECT                    /*!< It defines the OPAMP connection
                                                               with an external filter when PGA
                                                               is enabled.
                                                               It must be one of the following:
                                                               OPAMP_PGAConnect_No,
                                                               OPAMP_PGAConnect_IO1,
                                                               OPAMP_PGAConnect_IO2.
                                                               See reference manual RM0316.
                                                               This value is taken in account
                                                               just if wOPAMPx_InvertingInput is
                                                               equal to OPAMP2_InvertingInput_PGA*/
};

/**
  * @brief  Internal OPAMP parameters Motor 1 - three shunt - F30x - Shared Resources
  */
R3_2_F30XOPAMPParams_t R3_2_F30XOPAMPParamsM1 =
{
/* Internal OPAMP1 settings --------------------------------------------------*/
  .bOPAMP_InvertingInput_MODE             = OPAMP1_INVERTINGINPUT_MODE,         /*!< First OPAMP inverting input mode.
                                                                                     It must be either equal to EXT_MODE
                                                                                     or INT_MODE.*/
  .wOPAMP_InvertingInput                  = OPAMP1_INVERTINGINPUT,              /*!< First OPAMP inverting input pin.
                                                                                     It must be one of the following:
                                                                                     OPAMP1_InvertingInput_PC5 or
                                                                                     OPAMP1_InvertingInput_PA3 if the
                                                                                     bOPAMP_InvertingInput_MODE is
                                                                                     EXT_MODE or
                                                                                     OPAMP1_InvertingInput_PGA or
                                                                                     OPAMP1_InvertingInput_FOLLOWER if the
                                                                                     bOPAMP_InvertingInput_MODE is
                                                                                     INT_MODE.*/
  .hOPAMP_InvertingInput_GPIO_PORT        = OPAMP1_INVERTINGINPUT_GPIO_PORT,    /*!< First OPAMP inverting input GPIO port
                                                                                     as defined in wOPAMP_InvertingInput.
                                                                                     It must be GPIOx x= A, B, ... if
                                                                                     bOPAMP_InvertingInput_MODE is
                                                                                     EXT_MODE, otherwise can be dummy.*/
  .hOPAMP_InvertingInput_GPIO_PIN         = OPAMP1_INVERTINGINPUT_GPIO_PIN,     /*!< First OPAMP inverting input GPIO pin
                                                                                     as defined in wOPAMP_InvertingInput.
                                                                                     It must be GPIO_Pin_x x= 0, 1, ... if
                                                                                     bOPAMP_InvertingInput_MODE is
                                                                                     EXT_MODE, otherwise can be dummy.*/
  .wOPAMP_NonInvertingInput_PHA           = OPAMP1_NONINVERTINGINPUT_PHA,       /*!< First OPAMP non inverting input first
                                                                                     selection.
                                                                                     It must be one of the following:
                                                                                     OPAMP1_NonInvertingInput_PA7,
                                                                                     OPAMP1_NonInvertingInput_PA5,
                                                                                     OPAMP1_NonInvertingInput_PA3,
                                                                                     OPAMP1_NonInvertingInput_PA1.

                                                                                     Note: It must be the same PIN of
                                                                                     wOPAMP2_NonInvertingInput_PHA.*/
  .hOPAMP_NonInvertingInput_PHA_GPIO_PORT = OPAMP1_NONINVERTINGINPUT_PHA_GPIO_PORT,
                                                                                /*!< First OPAMP non inverting input GPIO
                                                                                     port as defined in
                                                                                     wOPAMP_NonInvertingInput_PHA.
                                                                                     It must be GPIOx x= A, B, ...*/
  .hOPAMP_NonInvertingInput_PHA_GPIO_PIN  = OPAMP1_NONINVERTINGINPUT_PHA_GPIO_PIN,
                                                                                /*!< First OPAMP non inverting input GPIO
                                                                                     pin as defined in
                                                                                     wOPAMP_NonInvertingInput_PHA.
                                                                                     It must be GPIO_Pin_x x= 0, 1, ...*/
  .wOPAMP_NonInvertingInput_PHB           = OPAMP1_NONINVERTINGINPUT_PHB,       /*!< First OPAMP non inverting input 2nd
                                                                                     selection.
                                                                                     It must be one of the following:
                                                                                     OPAMP1_NonInvertingInput_PA7,
                                                                                     OPAMP1_NonInvertingInput_PA5,
                                                                                     OPAMP1_NonInvertingInput_PA3,
                                                                                     OPAMP1_NonInvertingInput_PA1.*/
  .hOPAMP_NonInvertingInput_PHB_GPIO_PORT = OPAMP1_NONINVERTINGINPUT_PHB_GPIO_PORT,
                                                                                /*!< First OPAMP non inverting input GPIO
                                                                                     port as defined in
                                                                                     wOPAMP_NonInvertingInput_PHB.
                                                                                     It must be GPIOx x= A, B, ...*/
  .hOPAMP_NonInvertingInput_PHB_GPIO_PIN  = OPAMP1_NONINVERTINGINPUT_PHB_GPIO_PIN,
                                                                                /*!< First OPAMP non inverting input GPIO
                                                                                     pin as defined in
                                                                                     wOPAMP_NonInvertingInput_PHB.
                                                                                     It must be GPIO_Pin_x x= 0, 1, ...*/
/* Internal OPAMP2 settings --------------------------------------------------*/
  .bOPAMP2_InvertingInput_MODE             = OPAMP2_INVERTINGINPUT_MODE,         /*!< Second OPAMP inverting input mode.
                                                                                      It must be either equal to EXT_MODE
                                                                                      or INT_MODE.*/
  .wOPAMP2_InvertingInput                  = OPAMP2_INVERTINGINPUT,              /*!< Second OPAMP inverting input pin.
                                                                                      It must be one of the following:
                                                                                      OPAMP3_InvertingInput_PB2 or
                                                                                      OPAMP3_InvertingInput_PB10 if the
                                                                                      bOPAMP_InvertingInput_MODE is
                                                                                      EXT_MODE or
                                                                                      OPAMP3_InvertingInput_PGA or
                                                                                      OPAMP3_InvertingInput_FOLLOWER if the
                                                                                      bOPAMP_InvertingInput_MODE is
                                                                                      INT_MODE.*/
  .hOPAMP2_InvertingInput_GPIO_PORT        = OPAMP2_INVERTINGINPUT_GPIO_PORT,    /*!< 2nd OPAMP inverting input GPIO port
                                                                                      as defined in wOPAMP2_InvertingInput.
                                                                                      It must be GPIOx x= A, B, ...if
                                                                                      bOPAMP2_InvertingInput_MODE is
                                                                                      EXT_MODE, otherwise can be dummy.*/
  .hOPAMP2_InvertingInput_GPIO_PIN         = OPAMP2_INVERTINGINPUT_GPIO_PIN,     /*!< 2nd OPAMP inverting input GPIO pin as
                                                                                      defined in wOPAMP2_InvertingInput.
                                                                                      It must be GPIO_Pin_x x= 0, 1, ...if
                                                                                      bOPAMP2_InvertingInput_MODE is
                                                                                      EXT_MODE, otherwise can be dummy.*/
  .wOPAMP2_NonInvertingInput_PHA           = OPAMP2_NONINVERTINGINPUT_PHA,       /*!< 2nd OPAMP non inverting input
                                                                                      first selection.
                                                                                      It must be one of the following:
                                                                                      OPAMP3_NonInvertingInput_PB0,
                                                                                      OPAMP3_NonInvertingInput_PB13,
                                                                                      OPAMP3_NonInvertingInput_PA1,
                                                                                      OPAMP3_NonInvertingInput_PA5.
                                                                                      Note: It must be the same PIN of
                                                                                      wOPAMP_NonInvertingInput_PHA.*/
  .hOPAMP2_NonInvertingInput_PHA_GPIO_PORT = OPAMP2_NONINVERTINGINPUT_PHB_GPIO_PORT,
                                                                                 /*!< Second OPAMP non inverting input GPIO
                                                                                      port as defined in
                                                                                      wOPAMP2_NonInvertingInput_PHB.
                                                                                      It must be GPIOx x= A, B, ...*/
  .hOPAMP2_NonInvertingInput_PHA_GPIO_PIN  = OPAMP2_NONINVERTINGINPUT_PHB_GPIO_PIN,
                                                                                 /*!< Second OPAMP non inverting input GPIO
                                                                                      pin as defined in
                                                                                      wOPAMP2_NonInvertingInput_PHB.
                                                                                      It must be GPIO_Pin_x x= 0, 1, ...*/
  .wOPAMP2_NonInvertingInput_PHC           = OPAMP2_NONINVERTINGINPUT_PHC,       /*!< Second OPAMP non inverting input 2nd
                                                                                      selection.
                                                                  It must be one of the following:
                                                                                      OPAMP3_NonInvertingInput_PB0,
                                                                                      OPAMP3_NonInvertingInput_PB13,
                                                                                      OPAMP3_NonInvertingInput_PA1,
                                                                                      OPAMP3_NonInvertingInput_PA5.*/
  .hOPAMP2_NonInvertingInput_PHC_GPIO_PORT = OPAMP2_NONINVERTINGINPUT_PHC_GPIO_PORT,
                                                                                 /*!< Second OPAMP non inverting input GPIO
                                                                                      port as defined in
                                                                                      wOPAMP2_NonInvertingInput_PHC.
                                                                                      It must be GPIOx x= A, B, ...*/
  .hOPAMP2_NonInvertingInput_PHC_GPIO_PIN  = OPAMP2_NONINVERTINGINPUT_PHC_GPIO_PIN,
                                                                                 /*!< Second OPAMP non inverting input GPIO
                                                                                      pin as defined in
                                                                                      wOPAMP2_NonInvertingInput_PHC.
                                                                                      It must be GPIO_Pin_x x= 0, 1, ...*/
/* Common settings -----------------------------------------------------------*/
  .wOPAMP_PGAGain  = OPAMP_PGAGAIN,                      /*!< It defines the OPAMP PGA gains.
                                                              It must be one of the following:
                                                              OPAMP_OPAMP_PGAGain_2,
                                                              OPAMP_OPAMP_PGAGain_4,
                                                              OPAMP_OPAMP_PGAGain_8,
                                                              OPAMP_OPAMP_PGAGain_16.
                                                              This value is taken in account
                                                              just if wOPAMPx_InvertingInput is
                                                              equal to OPAMP2_InvertingInput_PGA*/
  .OPAMP_PGAConnect = OPAMP_PGACONNECT                    /*!< It defines the OPAMP connection
                                                               with an external filter when PGA
                                                               is enabled.
                                                               It must be one of the following:
                                                               OPAMP_PGAConnect_No,
                                                               OPAMP_PGAConnect_IO1,
                                                               OPAMP_PGAConnect_IO2.
                                                               See reference manual RM0316.
                                                               This value is taken in account
                                                               just if wOPAMPx_InvertingInput is
                                                               equal to OPAMP2_InvertingInput_PGA*/
};

/**
  * @brief  Internal COMP parameters Motor 1 - three shunt - F30x - OCPA
  */
F30XCOMPParams_t R3_F30XOCPACOMPParamsM1 =
{
/* Phase A over current protections ------------------------------------------*/
 .wSelection                   = OCPA_SELECTION,
                                                             /*!< Internal comparator used for phase A over
                                                                  current protection. It must be
                                                                  COMP_Selection_COMPx x = 1,2,3,4,5,6,7.*/
 .bInvertingInput_MODE         = OCPA_INVERTINGINPUT_MODE,
                                                             /*!< COMPx inverting input mode. It must be either
                                                                  equal to EXT_MODE or INT_MODE. */
 .wInvertingInput              = OCPA_INVERTINGINPUT,
                                                             /*!< COMPx inverting input pin. It must be one of
                                                                  the following:
                                                                  COMP1_InvertingInput_PA0,
                                                                  COMP2_InvertingInput_PA2,
                                                                  COMP3_InvertingInput_PD15,
                                                                  COMP3_InvertingInput_PB12,
                                                                  COMP4_InvertingInput_PE8,
                                                                  COMP4_InvertingInput_PB2,
                                                                  COMP5_InvertingInput_PD13,
                                                                  COMP5_InvertingInput_PB10,
                                                                  COMP6_InvertingInput_PD10,
                                                                  COMP6_InvertingInput_PB15 if
                                                                  bInvertingInput_MODE is EXT_MODE or
                                                                  COMPX_InvertingInput_DAC1,
                                                                  COMPX_InvertingInput_DAC2,
                                                                  COMPX_InvertingInput_VREF,
                                                                  COMPX_InvertingInput_VREF_1_4,
                                                                  COMPX_InvertingInput_VREF_1_2,
                                                                  COMPX_InvertingInput_VREF_3_4 if
                                                                  bInvertingInput_MODE is INT_MODE.
                                                                  If bInvertingInput_MODE is EXT_MODE, the
                                                                  only available options are related to the
                                                                  selected COMP in wSelection.*/
 .hInvertingInput_GPIO_PORT    = OCPA_INVERTINGINPUT_GPIO_PORT,
                                                             /*!< COMPx inverting input GPIO port as defined in
                                                                  wInvertingInput (Just if
                                                                  bInvertingInput_MODE is EXT_MODE).
                                                                  It must be GPIOx x= A, B, ...*/
 .hInvertingInput_GPIO_PIN     = OCPA_INVERTINGINPUT_GPIO_PIN,
                                                             /*!< COMPx inverting input GPIO pin as defined in
                                                                  wInvertingInput (Just if
                                                                  bInvertingInput_MODE is EXT_MODE).
                                                                  It must be GPIO_Pin_x x= 0, 1, ...*/
 .wNonInvertingInput           = OCPA_NONINVERTINGINPUT,
                                                             /*!< COMPx non inverting input. It must be one of
                                                                  the following:
                                                                  COMP1_NonInvertingInput_PA1,
                                                                  COMP2_NonInvertingInput_PA3,
                                                                  COMP2_NonInvertingInput_PA7,
                                                                  COMP3_NonInvertingInput_PB14,
                                                                  COMP3_NonInvertingInput_PD14,
                                                                  COMP4_NonInvertingInput_PB0,
                                                                  COMP4_NonInvertingInput_PE7,
                                                                  COMP5_NonInvertingInput_PB13,
                                                                  COMP5_NonInvertingInput_PD12,
                                                                  COMP6_NonInvertingInput_PB11,
                                                                  COMP6_NonInvertingInput_PD11,
                                                                  COMP7_NonInvertingInput_PC1,
                                                                  COMP7_NonInvertingInput_PA0.
                                                                  The only available options are related to the
                                                                  selected COMP in wSelection.*/
 .hNonInvertingInput_GPIO_PORT = OCPA_NONINVERTINGINPUT_GPIO_PORT,
                                                             /*!< COMPx non inverting input GPIO port as
                                                                  defined in wNonInvertingInput.
                                                                  It must be GPIOx x= A, B, ...*/
 .hNonInvertingInput_GPIO_PIN  = OCPA_NONINVERTINGINPUT_GPIO_PIN,
                                                             /*!< COMPx non inverting input GPIO pin as defined
                                                                  in wNonInvertingInput.
                                                                  It must be GPIO_Pin_x x= 0, 1, ...*/
 .bOutput_MODE                 = OCPA_OUTPUT_MODE,
                                                             /*!< COMPx output. It must be either
                                                                  equal to EXT_MODE or INT_MODE. */
 .wOutput                      = OCPA_OUTPUT,                /*!< COMPx output selection. It must be one of
                                                                  the following:
                                                                  COMP_Output_TIM1BKIN,
                                                                  COMP_Output_TIM1BKIN2,
                                                                  COMP_Output_TIM8BKIN,
                                                                  COMP_Output_TIM8BKIN2,
                                                                  COMP_Output_TIM1BKIN2_TIM8BKIN2.*/
 .hOutput_GPIO_PORT            = OCPA_OUTPUT_GPIO_PORT,
                                                             /*!< COMPx output GPIO port.
                                                                  It must be GPIOx x= A, B, ...*/
 .hOutput_GPIO_PIN             = OCPA_OUTPUT_GPIO_PIN,
                                                             /*!< COMPx output GPIO pin as defined.
                                                                  It must be GPIO_Pin_x x= 0, 1, ...*/
 .bOutput_GPIO_AF              = OCPA_OUTPUT_GPIO_AF,        /*!< COMPx output alternate functions setting.
                                                                  It must be one of the GPIO_AF_x (x=0,1, ...)
                                                                  according to the defined GPIO port and pin.*/
 .wOutputPol                   = OCPA_OUTPUTPOL,
                                                             /*!< COMPx output polarity. It must be one of
                                                                  the following:
                                                                  COMP_OutputPol_NonInverted,
                                                                  COMP_OutputPol_Inverted.*/
 .wMode                        = OCP_FILTER                  /*!< COMPx mode it is used to define both
                                                                  the speed (analog filter) and
                                                                  the power consumption of the internal
                                                                  comparator. It must be one of the
                                                                  following:
                                                                  COMP_Mode_HighSpeed,
                                                                  COMP_Mode_MediumSpeed,
                                                                  COMP_Mode_LowPower,
                                                                  COMP_Mode_UltraLowPower.
                                                                  More speed means less filter and more
                                                                  consumption.*/
};

/**
  * @brief  Internal COMP parameters Motor 1 - three shunt - F30x - OCPB
  */
F30XCOMPParams_t R3_F30XOCPBCOMPParamsM1 =
{
/* Phase B over current protections ------------------------------------------*/
 .wSelection                   = OCPB_SELECTION,
                                                              /*!< Internal comparator used for phase B over
                                                                  current protection. It must be
                                                                  COMP_Selection_COMPx x = 1,2,3,4,5,6,7.*/
 .bInvertingInput_MODE         = OCPB_INVERTINGINPUT_MODE,
                                                             /*!< COMPx inverting input mode. It must be either
                                                                  equal to EXT_MODE or INT_MODE. */
 .wInvertingInput              = OCPB_INVERTINGINPUT,
                                                             /*!< COMPx inverting input pin. It must be one of
                                                                  the following:
                                                                  COMP1_InvertingInput_PA0,
                                                                  COMP2_InvertingInput_PA2,
                                                                  COMP3_InvertingInput_PD15,
                                                                  COMP3_InvertingInput_PB12,
                                                                  COMP4_InvertingInput_PE8,
                                                                  COMP4_InvertingInput_PB2,
                                                                  COMP5_InvertingInput_PD13,
                                                                  COMP5_InvertingInput_PB10,
                                                                  COMP6_InvertingInput_PD10,
                                                                  COMP6_InvertingInput_PB15 if
                                                                  bInvertingInput_MODE is EXT_MODE or
                                                                  COMPX_InvertingInput_DAC1,
                                                                  COMPX_InvertingInput_DAC2,
                                                                  COMPX_InvertingInput_VREF,
                                                                  COMPX_InvertingInput_VREF_1_4,
                                                                  COMPX_InvertingInput_VREF_1_2,
                                                                  COMPX_InvertingInput_VREF_3_4 if
                                                                  bInvertingInput_MODE is INT_MODE.
                                                                  If bInvertingInput_MODE is EXT_MODE, the
                                                                  only available options are related to the
                                                                  selected COMP in wSelection.*/
 .hInvertingInput_GPIO_PORT    = OCPB_INVERTINGINPUT_GPIO_PORT,
                                                             /*!< COMPx inverting input GPIO port as defined in
                                                                  wInvertingInput (Just if
                                                                  bInvertingInput_MODE is EXT_MODE).
                                                                  It must be GPIOx x= A, B, ...*/
 .hInvertingInput_GPIO_PIN     = OCPB_INVERTINGINPUT_GPIO_PIN,
                                                             /*!< COMPx inverting input GPIO pin as defined in
                                                                  wInvertingInput (Just if
                                                                  bInvertingInput_MODE is EXT_MODE).
                                                                  It must be GPIO_Pin_x x= 0, 1, ...*/
 .wNonInvertingInput           = OCPB_NONINVERTINGINPUT,
                                                             /*!< COMPx non inverting input. It must be one of
                                                                  the following:
                                                                  COMP1_NonInvertingInput_PA1,
                                                                  COMP2_NonInvertingInput_PA3,
                                                                  COMP2_NonInvertingInput_PA7,
                                                                  COMP3_NonInvertingInput_PB14,
                                                                  COMP3_NonInvertingInput_PD14,
                                                                  COMP4_NonInvertingInput_PB0,
                                                                  COMP4_NonInvertingInput_PE7,
                                                                  COMP5_NonInvertingInput_PB13,
                                                                  COMP5_NonInvertingInput_PD12,
                                                                  COMP6_NonInvertingInput_PB11,
                                                                  COMP6_NonInvertingInput_PD11,
                                                                  COMP7_NonInvertingInput_PC1,
                                                                  COMP7_NonInvertingInput_PA0.
                                                                  The only available options are related to the
                                                                  selected COMP in wSelection.*/
 .hNonInvertingInput_GPIO_PORT = OCPB_NONINVERTINGINPUT_GPIO_PORT,
                                                             /*!< COMPx non inverting input GPIO port as
                                                                  defined in wNonInvertingInput.
                                                                  It must be GPIOx x= A, B, ...*/
 .hNonInvertingInput_GPIO_PIN  = OCPB_NONINVERTINGINPUT_GPIO_PIN,
                                                             /*!< COMPx non inverting input GPIO pin as defined
                                                                  in wNonInvertingInput.
                                                                  It must be GPIO_Pin_x x= 0, 1, ...*/
 .bOutput_MODE                 = OCPB_OUTPUT_MODE,
                                                             /*!< COMPx output. It must be either
                                                                  equal to EXT_MODE or INT_MODE. */
 .wOutput                      = OCPB_OUTPUT,                /*!< COMPx output selection. It must be one of
                                                                  the following:
                                                                  COMP_Output_TIM1BKIN,
                                                                  COMP_Output_TIM1BKIN2,
                                                                  COMP_Output_TIM8BKIN,
                                                                  COMP_Output_TIM8BKIN2,
                                                                  COMP_Output_TIM1BKIN2_TIM8BKIN2.*/
 .hOutput_GPIO_PORT            = OCPB_OUTPUT_GPIO_PORT,
                                                             /*!< COMPx output GPIO port.
                                                                  It must be GPIOx x= A, B, ...*/
 .hOutput_GPIO_PIN             = OCPB_OUTPUT_GPIO_PIN,
                                                             /*!< COMPx output GPIO pin as defined.
                                                                  It must be GPIO_Pin_x x= 0, 1, ...*/
 .bOutput_GPIO_AF              = OCPB_OUTPUT_GPIO_AF,        /*!< COMPx output alternate functions setting.
                                                                  It must be one of the GPIO_AF_x (x=0,1, ...)
                                                                  according to the defined GPIO port and pin.*/
 .wOutputPol                   = OCPB_OUTPUTPOL,
                                                             /*!< COMPx output polarity. It must be one of
                                                                  the following:
                                                                  COMP_OutputPol_NonInverted,
                                                                  COMP_OutputPol_Inverted.*/
 .wMode                        = OCP_FILTER                  /*!< COMPx mode it is used to define both
                                                                  the speed (analog filter) and
                                                                  the power consumption of the internal
                                                                  comparator. It must be one of the
                                                                  following:
                                                                  COMP_Mode_HighSpeed,
                                                                  COMP_Mode_MediumSpeed,
                                                                  COMP_Mode_LowPower,
                                                                  COMP_Mode_UltraLowPower.
                                                                  More speed means less filter and more
                                                                  consumption.*/
};

/**
  * @brief  Internal COMP parameters Motor 1 - three shunt - F30x - OCPC
  */
F30XCOMPParams_t R3_F30XOCPCCOMPParamsM1 =
{
  /* Phase C over current protections ------------------------------------------*/
 .wSelection                   = OCPC_SELECTION,
                                                              /*!< Internal comparator used for phase C over
                                                                  current protection. It must be
                                                                  COMP_Selection_COMPx x = 1,2,3,4,5,6,7.*/
 .bInvertingInput_MODE         = OCPC_INVERTINGINPUT_MODE,
                                                             /*!< COMPx inverting input mode. It must be either
                                                                  equal to EXT_MODE or INT_MODE. */
 .wInvertingInput              = OCPC_INVERTINGINPUT,
                                                             /*!< COMPx inverting input pin. It must be one of
                                                                  the following:
                                                                  COMP1_InvertingInput_PA0,
                                                                  COMP2_InvertingInput_PA2,
                                                                  COMP3_InvertingInput_PD15,
                                                                  COMP3_InvertingInput_PB12,
                                                                  COMP4_InvertingInput_PE8,
                                                                  COMP4_InvertingInput_PB2,
                                                                  COMP5_InvertingInput_PD13,
                                                                  COMP5_InvertingInput_PB10,
                                                                  COMP6_InvertingInput_PD10,
                                                                  COMP6_InvertingInput_PB15 if
                                                                  bInvertingInput_MODE is EXT_MODE or
                                                                  COMPX_InvertingInput_DAC1,
                                                                  COMPX_InvertingInput_DAC2,
                                                                  COMPX_InvertingInput_VREF,
                                                                  COMPX_InvertingInput_VREF_1_4,
                                                                  COMPX_InvertingInput_VREF_1_2,
                                                                  COMPX_InvertingInput_VREF_3_4 if
                                                                  bInvertingInput_MODE is INT_MODE.
                                                                  If bInvertingInput_MODE is EXT_MODE, the
                                                                  only available options are related to the
                                                                  selected COMP in wSelection.*/
 .hInvertingInput_GPIO_PORT    = OCPC_INVERTINGINPUT_GPIO_PORT,
                                                             /*!< COMPx inverting input GPIO port as defined in
                                                                  wInvertingInput (Just if
                                                                  bInvertingInput_MODE is EXT_MODE).
                                                                  It must be GPIOx x= A, B, ...*/
 .hInvertingInput_GPIO_PIN     = OCPC_INVERTINGINPUT_GPIO_PIN,
                                                             /*!< COMPx inverting input GPIO pin as defined in
                                                                  wInvertingInput (Just if
                                                                  bInvertingInput_MODE is EXT_MODE).
                                                                  It must be GPIO_Pin_x x= 0, 1, ...*/
 .wNonInvertingInput           = OCPC_NONINVERTINGINPUT,
                                                             /*!< COMPx non inverting input. It must be one of
                                                                  the following:
                                                                  COMP1_NonInvertingInput_PA1,
                                                                  COMP2_NonInvertingInput_PA3,
                                                                  COMP2_NonInvertingInput_PA7,
                                                                  COMP3_NonInvertingInput_PB14,
                                                                  COMP3_NonInvertingInput_PD14,
                                                                  COMP4_NonInvertingInput_PB0,
                                                                  COMP4_NonInvertingInput_PE7,
                                                                  COMP5_NonInvertingInput_PB13,
                                                                  COMP5_NonInvertingInput_PD12,
                                                                  COMP6_NonInvertingInput_PB11,
                                                                  COMP6_NonInvertingInput_PD11,
                                                                  COMP7_NonInvertingInput_PC1,
                                                                  COMP7_NonInvertingInput_PA0.
                                                                  The only available options are related to the
                                                                  selected COMP in wSelection.*/
 .hNonInvertingInput_GPIO_PORT = OCPC_NONINVERTINGINPUT_GPIO_PORT,
                                                             /*!< COMPx non inverting input GPIO port as
                                                                  defined in wNonInvertingInput.
                                                                  It must be GPIOx x= A, B, ...*/
 .hNonInvertingInput_GPIO_PIN  = OCPC_NONINVERTINGINPUT_GPIO_PIN,
                                                             /*!< COMPx non inverting input GPIO pin as defined
                                                                  in wNonInvertingInput.
                                                                  It must be GPIO_Pin_x x= 0, 1, ...*/
 .bOutput_MODE                 = OCPC_OUTPUT_MODE,
                                                             /*!< COMPx output. It must be either
                                                                  equal to EXT_MODE or INT_MODE. */
 .wOutput                      = OCPC_OUTPUT,                 /*!< COMPx output selection. It must be one of
                                                                  the following:
                                                                  COMP_Output_TIM1BKIN,
                                                                  COMP_Output_TIM1BKIN2,
                                                                  COMP_Output_TIM8BKIN,
                                                                  COMP_Output_TIM8BKIN2,
                                                                  COMP_Output_TIM1BKIN2_TIM8BKIN2.*/
 .hOutput_GPIO_PORT            = OCPC_OUTPUT_GPIO_PORT,
                                                             /*!< COMPx output GPIO port.
                                                                  It must be GPIOx x= A, B, ...*/
 .hOutput_GPIO_PIN             = OCPC_OUTPUT_GPIO_PIN,
                                                             /*!< COMPx output GPIO pin as defined.
                                                                  It must be GPIO_Pin_x x= 0, 1, ...*/
 .bOutput_GPIO_AF              = OCPC_OUTPUT_GPIO_AF,        /*!< COMPx output alternate functions setting.
                                                                  It must be one of the GPIO_AF_x (x=0,1, ...)
                                                                  according to the defined GPIO port and pin.*/
 .wOutputPol                   = OCPC_OUTPUTPOL,
                                                             /*!< COMPx output polarity. It must be one of
                                                                  the following:
                                                                  COMP_OutputPol_NonInverted,
                                                                  COMP_OutputPol_Inverted.*/
 .wMode                        = OCP_FILTER                  /*!< COMPx mode it is used to define both
                                                                  the speed (analog filter) and
                                                                  the power consumption of the internal
                                                                  comparator. It must be one of the
                                                                  following:
                                                                  COMP_Mode_HighSpeed,
                                                                  COMP_Mode_MediumSpeed,
                                                                  COMP_Mode_LowPower,
                                                                  COMP_Mode_UltraLowPower.
                                                                  More speed means less filter and more
                                                                  consumption.*/
};

/**
  * @brief  Internal COMP parameters Motor 1 - F30x - OVP
  */
F30XCOMPParams_t F30XOVPCOMPParamsM1 =
{
  /* Over voltage protections ------------------------------------------*/
 .wSelection                   = OVP_SELECTION,
                                                              /*!< Internal comparator used for over
                                                                  voltage protection. It must be
                                                                  COMP_Selection_COMPx x = 1,2,3,4,5,6,7.*/
 .bInvertingInput_MODE         = OVP_INVERTINGINPUT_MODE,
                                                             /*!< COMPx inverting input mode. It must be either
                                                                  equal to EXT_MODE or INT_MODE. */
 .wInvertingInput              = OVP_INVERTINGINPUT,
                                                             /*!< COMPx inverting input pin. It must be one of
                                                                  the following:
                                                                  COMP1_InvertingInput_PA0,
                                                                  COMP2_InvertingInput_PA2,
                                                                  COMP3_InvertingInput_PD15,
                                                                  COMP3_InvertingInput_PB12,
                                                                  COMP4_InvertingInput_PE8,
                                                                  COMP4_InvertingInput_PB2,
                                                                  COMP5_InvertingInput_PD13,
                                                                  COMP5_InvertingInput_PB10,
                                                                  COMP6_InvertingInput_PD10,
                                                                  COMP6_InvertingInput_PB15 if
                                                                  bInvertingInput_MODE is EXT_MODE or
                                                                  COMPX_InvertingInput_DAC1,
                                                                  COMPX_InvertingInput_DAC2,
                                                                  COMPX_InvertingInput_VREF,
                                                                  COMPX_InvertingInput_VREF_1_4,
                                                                  COMPX_InvertingInput_VREF_1_2,
                                                                  COMPX_InvertingInput_VREF_3_4 if
                                                                  bInvertingInput_MODE is INT_MODE.
                                                                  If bInvertingInput_MODE is EXT_MODE, the
                                                                  only available options are related to the
                                                                  selected COMP in wSelection.*/
 .hInvertingInput_GPIO_PORT    = OVP_INVERTINGINPUT_GPIO_PORT,
                                                             /*!< COMPx inverting input GPIO port as defined in
                                                                  wInvertingInput (Just if
                                                                  bInvertingInput_MODE is EXT_MODE).
                                                                  It must be GPIOx x= A, B, ...*/
 .hInvertingInput_GPIO_PIN     = OVP_INVERTINGINPUT_GPIO_PIN,
                                                             /*!< COMPx inverting input GPIO pin as defined in
                                                                  wInvertingInput (Just if
                                                                  bInvertingInput_MODE is EXT_MODE).
                                                                  It must be GPIO_Pin_x x= 0, 1, ...*/
 .wNonInvertingInput           = OVP_NONINVERTINGINPUT,
                                                             /*!< COMPx non inverting input. It must be one of
                                                                  the following:
                                                                  COMP1_NonInvertingInput_PA1,
                                                                  COMP2_NonInvertingInput_PA3,
                                                                  COMP2_NonInvertingInput_PA7,
                                                                  COMP3_NonInvertingInput_PB14,
                                                                  COMP3_NonInvertingInput_PD14,
                                                                  COMP4_NonInvertingInput_PB0,
                                                                  COMP4_NonInvertingInput_PE7,
                                                                  COMP5_NonInvertingInput_PB13,
                                                                  COMP5_NonInvertingInput_PD12,
                                                                  COMP6_NonInvertingInput_PB11,
                                                                  COMP6_NonInvertingInput_PD11,
                                                                  COMP7_NonInvertingInput_PC1,
                                                                  COMP7_NonInvertingInput_PA0.
                                                                  The only available options are related to the
                                                                  selected COMP in wSelection.*/
 .hNonInvertingInput_GPIO_PORT = OVP_NONINVERTINGINPUT_GPIO_PORT,
                                                             /*!< COMPx non inverting input GPIO port as
                                                                  defined in wNonInvertingInput.
                                                                  It must be GPIOx x= A, B, ...*/
 .hNonInvertingInput_GPIO_PIN  = OVP_NONINVERTINGINPUT_GPIO_PIN,
                                                             /*!< COMPx non inverting input GPIO pin as defined
                                                                  in wNonInvertingInput.
                                                                  It must be GPIO_Pin_x x= 0, 1, ...*/
 .bOutput_MODE                 = OVP_OUTPUT_MODE,
                                                             /*!< COMPx output. It must be either
                                                                  equal to EXT_MODE or INT_MODE. */
 .wOutput                      = OVP_OUTPUT,                 /*!< COMPx output selection. It must be one of
                                                                  the following:
                                                                  COMP_Output_TIM1BKIN,
                                                                  COMP_Output_TIM1BKIN2,
                                                                  COMP_Output_TIM8BKIN,
                                                                  COMP_Output_TIM8BKIN2,
                                                                  COMP_Output_TIM1BKIN2_TIM8BKIN2.*/
 .hOutput_GPIO_PORT            = OVP_OUTPUT_GPIO_PORT,
                                                             /*!< COMPx output GPIO port.
                                                                  It must be GPIOx x= A, B, ...*/
 .hOutput_GPIO_PIN             = OVP_OUTPUT_GPIO_PIN,
                                                             /*!< COMPx output GPIO pin as defined.
                                                                  It must be GPIO_Pin_x x= 0, 1, ...*/
 .bOutput_GPIO_AF              = OVP_OUTPUT_GPIO_AF,        /*!< COMPx output alternate functions setting.
                                                                  It must be one of the GPIO_AF_x (x=0,1, ...)
                                                                  according to the defined GPIO port and pin.*/
 .wOutputPol                   = OVP_OUTPUTPOL,
                                                             /*!< COMPx output polarity. It must be one of
                                                                  the following:
                                                                  COMP_OutputPol_NonInverted,
                                                                  COMP_OutputPol_Inverted.*/
 .wMode                        = OVP_FILTER                  /*!< COMPx mode it is used to define both
                                                                  the speed (analog filter) and
                                                                  the power consumption of the internal
                                                                  comparator. It must be one of the
                                                                  following:
                                                                  COMP_Mode_HighSpeed,
                                                                  COMP_Mode_MediumSpeed,
                                                                  COMP_Mode_LowPower,
                                                                  COMP_Mode_UltraLowPower.
                                                                  More speed means less filter and more
                                                                  consumption.*/
};

/**
  * @brief  Current sensor parameters Motor 1 - three shunt - STM32F302x8
  */
R3_1_F30XParams_t R3_1_F30XParamsM1 =
{
  ADC_CLOCK_DIVIDER,                /*!<  AHB clock prescaling factor for
                                          ADC peripheral. It must be
                                          RCC_ADC12PLLCLK_Divx or
                                          RCC_ADC34PLLCLK_Divx
                                          x = 1, 2, 4, 6, 8, ... */
  ADC_AHBPERIPH,                     /*!< AHB periph used. It must be
                                          RCC_AHBPeriph_ADC12 or
                                          RCC_AHBPeriph_ADC34. */
  TIM_CLOCK_DIVIDER,                  /* System clock divider for Advanded Timer */
/* Dual MC parameters --------------------------------------------------------*/
  1,                                    /*!< Instance number. Necessary to
                                             properly synchronize TIM8 with
                                             TIM1 at peripheral initializations
                                              */
  MAX_TWAIT,                            /*!< It is used for switching the context
                                           in dual MC. It contains biggest delay
                                           (expressed in counter ticks) between
                                           the counter crest and ADC latest
                                           trigger  */
FREQ_RATIO,                                    /*!< It is used in case of dual MC to
                                           synchronize TIM1 and TIM8. It has
                                           effect only on the second instanced
                                           object and must be equal to the
                                           ratio between the two PWM frequencies
                                           (higher/lower). Supported values are
                                           1, 2 or 3 */
  FREQ_RELATION,                           /*!< When bFreqRatio is greather than 1
                                          this param is used to indicate if this
                                          instance is the one with the highest
                                          frequency. Allowed value are: HIGHER_FREQ
                                          or LOWER_FREQ */
  MC_IRQ_PWMNCURRFDBK_1,                     /*!< MC IRQ number used as TIMx Update
                                           event */
/* Current reading A/D Conversions initialization -----------------------------*/
  PHASE_U_CURR_CHANNEL,                  /*!< ADC channel used for conversion of
                                           current Ia. It must be equal to
                                           ADC_Channel_x x= 0, ..., 15*/
  PHASE_U_GPIO_PORT,                  /*!< GPIO port used by hIaChannel. It must
                                           be equal to GPIOx x= A, B, ...*/
  PHASE_U_GPIO_PIN,                      /*!< GPIO pin used by hIaChannel. It must be
                                          equal to GPIO_Pin_x x= 0, 1, ...*/
  SAMPLING_TIME_SEL,               /*!< Sampling time used to convert hIaChannel.
                                          It must be equal to ADC_SampleTime_xCycles5
                                          x= 1, 7, ...*/
  PHASE_V_CURR_CHANNEL,                  /*!< ADC channel used for conversion of
                                           current Ib. It must be equal to
                                           ADC_Channel_x x= 0, ..., 15*/
  PHASE_V_GPIO_PORT,                /*!< GPIO port used by hIbChannel. It must
                                           be equal to GPIOx x= A, B, ...*/
  PHASE_V_GPIO_PIN,                      /*!< GPIO pin used by hIaChannel. It must be
                                          equal to GPIO_Pin_x x= 0, 1, ...*/
  SAMPLING_TIME_SEL,                /*!< Sampling time used to convert hIbChannel.
                                          It must be equal to ADC_SampleTime_xCycles5
                                          x= 1, 7, ...*/
  PHASE_W_CURR_CHANNEL,                  /*!< ADC channel used for conversion of
                                           current Ia. It must be equal to
                                           ADC_Channel_x x= 0, ..., 15*/
  PHASE_W_GPIO_PORT,                /*!< GPIO port used by hIaChannel. It must
                                           be equal to GPIOx x= A, B, ...*/
  PHASE_W_GPIO_PIN,                      /*!< GPIO pin used by hIaChannel. It must be
                                          equal to GPIO_Pin_x x= 0, 1, ...*/
  SAMPLING_TIME_SEL,            /*!< Sampling time used to convert hIaChannel.
                                          It must be equal to ADC_SampleTime_xCycles5
                                          x= 1, 7, ...*/

/* PWM generation parameters --------------------------------------------------*/
  DEAD_TIME_COUNTS,                    /*!< Dead time in number of TIM clock
                                            cycles. If CHxN are enabled, it must
                                            contain the dead time to be generated
                                            by the microcontroller, otherwise it
                                            expresses the maximum dead time
                                            generated by driving network */
  REP_COUNTER,                         /*!< It expresses the number of PWM
                                            periods to be elapsed before compare
                                            registers are updated again. In
                                            particular:
                                            RepetitionCounter= (2* #PWM periods)-1*/
  TW_AFTER,                             /*!< It is the sum of dead time plus max
                                            value between rise time and noise time
                                            express in number of TIM clocks.*/
  TW_BEFORE_R3_1,                      /*!< It is the sampling time express in
                                            number of TIM clocks.*/
  PWM_TIMER_SELECTION,                               /*!< It contains the pointer to the timer
                                           used for PWM generation. */
/* PWM Driving signals initialization ----------------------------------------*/

  PWM_TIMER_REMAPPING,                        /*!< Used only for instance #1, it
                                           remaps TIM1 outputs. It must equal to
                                           GPIO_PartialRemap_TIM1 or
                                           GPIO_FullRemap_TIM1 or
                                           GPIO_NoRemap_TIM1 */
  PHASE_UH_POLARITY,   	              /*!< Channel 1 (high side) output polarity,
                                           it must be TIM_OCPolarity_High or
                                           TIM_OCPolarity_Low */
  PHASE_UH_GPIO_PORT,                 /*!< Channel 1 (high side) GPIO output
                                           port (if used, after re-mapping).
                                           It must be GPIOx x= A, B, ...*/
  PHASE_UH_GPIO_PIN,                  /*!< Channel 1 (high side) GPIO output pin
                                          (if used, after re-mapping). It must be
                                           GPIO_Pin_x x= 0, 1, ...*/
  BRAKE_UH_POLARITY,                   /*!< Channel 1 (high side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCIdleState_Set or
                                            TIM_OCIdleState_Reset*/

  PHASE_VH_POLARITY,                   /*!< Channel 2 (high side) output polarity,
                                           it must be TIM_OCPolarity_High or
                                           TIM_OCPolarity_Low */
  PHASE_VH_GPIO_PORT,                               /*!< Channel 2 (high side) GPIO output
                                           port (if used, after re-mapping).
                                           It must be GPIOx x= A, B, ...*/
  PHASE_VH_GPIO_PIN,                       /*!< Channel 2 (high side) GPIO output pin
                                          (if used, after re-mapping). It must be
                                          GPIO_Pin_x x= 0, 1, ...*/
  BRAKE_VH_POLARITY,              /*!< Channel 2 (high side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCIdleState_Set or
                                            TIM_OCIdleState_Reset*/

  PHASE_WH_POLARITY,            	 /*!< Channel 3 (high side) output polarity,
                                           it must be TIM_OCPolarity_High or
                                           TIM_OCPolarity_Low */
  PHASE_WH_GPIO_PORT,                               /*!< Channel 3 (high side) GPIO output
                                           port (if used, after re-mapping).
                                           It must be GPIOx x= A, B, ...*/
  PHASE_WH_GPIO_PIN,                        /*!< Channel 3 (high side) GPIO output pin
                                          (if used, after re-mapping). It must be
                                          GPIO_Pin_x x= 0, 1, ...*/
  BRAKE_WH_POLARITY,              /*!< Channel 3 (high side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCIdleState_Set or
                                            TIM_OCIdleState_Reset*/

  (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING,  /*!< Low side or enabling signals
                                                generation method are defined
                                                here.*/

  PHASE_UL_POLARITY,               /*!< Channel 1N (low side) output polarity,
                                           it must be TIM_OCNPolarity_High or
                                           TIM_OCNPolarity_Low */
  PHASE_UL_GPIO_PORT,                                 /*!< Channel 1N (low side) GPIO output
                                           port (if used, after re-mapping).
                                           It must be GPIOx x= A, B, ...*/
  PHASE_UL_GPIO_PIN,                       /*!< Channel 1N (low side) GPIO output pin
                                          (if used, after re-mapping). It must be
                                          GPIO_Pin_x x= 0, 1, ...*/
  BRAKE_UL_POLARITY,              /*!< Channel 1N (low side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCNIdleState_Set or
                                            TIM_OCNIdleState_Reset*/


  PHASE_VL_POLARITY,                 /*!< Channel 2N (low side) output polarity,
                                           it must be TIM_OCNPolarity_High or
                                           TIM_OCNPolarity_Low */
  PHASE_VL_GPIO_PORT,                                 /*!< Channel 2N (low side) GPIO output
                                           port (if used, after re-mapping).
                                           It must be GPIOx x= A, B, ...*/
  PHASE_VL_GPIO_PIN,                     /*!< Channel 2N (low side) GPIO output pin
                                          (if used, after re-mapping). It must be
                                          GPIO_Pin_x x= 0, 1, ...*/
  BRAKE_VL_POLARITY,              /*!< Channel 2N (low side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCNIdleState_Set or
                                            TIM_OCNIdleState_Reset*/

  PHASE_WL_POLARITY,             /*!< Channel 3N (low side) output polarity,
                                           it must be TIM_OCNPolarity_High or
                                           TIM_OCNPolarity_Low */
  PHASE_WL_GPIO_PORT,                               /*!< Channel 3N (low side)  GPIO output
                                           port (if used, after re-mapping).
                                           It must be GPIOx x= A, B, ...*/
  PHASE_WL_GPIO_PIN,                    /*!< Channel 3N (low side)  GPIO output pin
                                          (if used, after re-mapping). It must be
                                          GPIO_Pin_x x= 0, 1, ...*/
  BRAKE_WL_POLARITY,              /*!< Channel 3N (low side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCNIdleState_Set or
                                            TIM_OCNIdleState_Reset*/
/* PWM Driving signals initialization ----------------------------------------*/
  BKIN_MODE,                          /*!< It defines the modality of emergency
                                           input. It must be any of the
                                           the following:
                                           NONE - feature disabled.
                                           INT_MODE - Internal comparator used
                                           as source of emergency event.
                                           EXT_MODE - External comparator used
                                           as source of emergency event.
                                           EXTINT_MODE - The or-ed signal
                                           between internal and external
                                           comparator is used as source event.*/
  EMSTOP_ACTIVE_HIGH,                   /*!< Emergency Stop (BKIN) input polarity,
                                           it must be TIM_BreakPolarity_Low or
                                           TIM_BreakPolarity_High */
  EMERGENCY_STOP_GPIO_PORT,             /*!< Emergency Stop (BKIN) GPIO input
                                           port (if used, after re-mapping).
                                           It must be GPIOx x= A, B, ...*/
  EMERGENCY_STOP_GPIO_PIN,              /*!< Emergency Stop (BKIN) GPIO input pin
                                         (if used, after re-mapping). It must be
                                          GPIO_Pin_x x= 0, 1, ...*/
/* Alternate functions definition --------------------------------------------*/
  PHASE_UH_GPIO_AF,                   /*!< Channel 1 (high side) alternate
                                           functions setting. It must be one
                                           of the GPIO_AF_x (x=0,1, ...)
                                           according to the defined GPIO port
                                           and pin.*/
  PHASE_VH_GPIO_AF,                   /*!< Channel 2 (high side) alternate
                                           functions setting. It must be one
                                           of the GPIO_AF_x (x=0,1, ...)
                                           according to the defined GPIO port
                                           and pin.*/
  PHASE_WH_GPIO_AF,                   /*!< Channel 3 (high side) alternate
                                           functions setting. It must be one
                                           of the GPIO_AF_x (x=0,1, ...)
                                           according to the defined GPIO port
                                           and pin.*/
  PHASE_UL_GPIO_AF,                   /*!< Channel 1 (low side) alternate
                                           functions setting. It must be one
                                           of the GPIO_AF_x (x=0,1, ...)
                                           according to the defined GPIO port
                                           and pin.*/
  PHASE_VL_GPIO_AF,                   /*!< Channel 2 (low side) alternate
                                           functions setting. It must be one
                                           of the GPIO_AF_x (x=0,1, ...)
                                           according to the defined GPIO port
                                           and pin.*/
  PHASE_WL_GPIO_AF,                   /*!< Channel 3 (low side) alternate
                                           functions setting. It must be one
                                           of the GPIO_AF_x (x=0,1, ...)
                                           according to the defined GPIO port
                                           and pin.*/
  BRKIN_GPIO_AF,                      /*!< Emergency Stop (BKIN) alternate
                                           functions setting. It must be one
                                           of the GPIO_AF_x (x=0,1, ...)
                                           according to the defined GPIO port
                                           and pin.*/
  BRKIN2_GPIO_AF,                     /*!< Emergency Stop (BKIN2) alternate
                                           functions setting. It must be one
                                           of the GPIO_AF_x (x=0,1, ...)
                                           according to the defined GPIO port
                                           and pin.*/
/* Emergency input (BKIN2) signal initialization -----------------------------*/
  BKIN2_MODE,                         /*!< It defines the modality of emergency
                                           input 2. It must be any of the
                                           the following:
                                           NONE - feature disabled.
                                           INT_MODE - Internal comparator used
                                           as source of emergency event.
                                           EXT_MODE - External comparator used
                                           as source of emergency event.
                                           EXTINT_MODE - The or-ed signal
                                           between internal and external
                                           comparator is used as source event.*/
  OVERCURR_FEEDBACK_POLARITY,         /*!< Emergency Stop (BKIN2) input polarity,
                                           it must be TIM_Break2Polarity_Low or
                                           TIM_Break2Polarity_High */
  EMERGENCY2_STOP_GPIO_PORT,          /*!< Emergency Stop (BKIN) GPIO input
                                           port (if used, after re-mapping).
                                           It must be GPIOx x= A, B, ...*/
  EMERGENCY2_STOP_GPIO_PIN,           /*!< Emergency Stop (BKIN) GPIO input pin
                                           (if used, after re-mapping). It must be
                                           GPIO_Pin_x x= 0, 1, ...*/
/* Filtering settings of the emergency inputs --------------------------------*/
  BKIN1_FILTER,                         /*!< Emergency Stop (BKIN) digital
                                             filter. See TIMx_BDTR (BKF bits)
                                             definition in the reference
                                             manual.*/
  BKIN2_FILTER,                         /*!< Emergency Stop (BKIN2) digital
                                             filter. See TIMx_BDTR (BKF bits)
                                             definition in the reference
                                             manual.*/
/* Internal COMP settings ----------------------------------------------------*/
  OCPA_COMPPARAMSM1,
                                      /*!< Pointer to the COMP params struct
                                           used for overcurrent protection of
                                           phase A.
                                           It must be MC_NULL if internal COMP
                       are not used for OCP protection.*/
  OCPB_COMPPARAMSM1,
                                      /*!< Pointer to the COMP params struct
                                           used for overcurrent protection of
                                           phase B.
                                           It must be MC_NULL if internal COMP
                       are not used for OCP protection.*/
  OCPC_COMPPARAMSM1,
                                      /*!< Pointer to the COMP params struct
                                           used for overcurrent protection of
                                           phase C.
                                           It must be MC_NULL if internal COMP
                       are not used for OCP protection.*/
  OVP_COMPPARAMSM1,
                                      /*!< Pointer to the COMP params struct
                                           used for overvoltage protection.
                                           It must be MC_NULL if internal COMP
                       are not used for OVP protection.*/
/* DAC settings --------------------------------------------------------------*/
  OCPREF,                             /*!< Value of analog reference expressed
                                           as 16bit unsigned integer.
                                           Ex. 0 = 0V 65536 = VDD_DAC.*/
  OVPREF,                             /*!< Value of analog reference expressed
                                           as 16bit unsigned integer.
                                           Ex. 0 = 0V 65536 = VDD_DAC.*/
/* Regular conversion --------------------------------------------------------*/
  REGCONVADC                          /*!< ADC peripheral used for regular
                                           conversion.*/
};

/**
  * @brief  Current sensor parameters Motor 1 - three shunt - F30x - Independent Resources
  */
R3_4_F30XParams_t R3_4_F30XParamsM1 =
{
  .wADC_Clock_Divider = ADC_CLOCK_DIVIDER,                /*!<  AHB clock prescaling factor for
                                                                ADC peripheral. It must be
                                                                RCC_ADC12PLLCLK_Divx or
                                                                RCC_ADC34PLLCLK_Divx
                                                                x = 1, 2, 4, 6, 8, ... */
  .wAHBPeriph         = ADC_AHBPERIPH,                     /*!< AHB periph used. It must be
                                                                RCC_AHBPeriph_ADC12 or
                                                                RCC_AHBPeriph_ADC34. */
  .bTim_Clock_Divider = TIM_CLOCK_DIVIDER,                  /* System clock divider for Advanded Timer */
/* Dual MC parameters --------------------------------------------------------*/
  .bInstanceNbr     = 1,                                    /*!< Instance number. Necessary to
                                                                 properly synchronize TIM8 with
                                                                 TIM1 at peripheral initializations
                                                                  */
  .Tw               = MAX_TWAIT,                            /*!< It is used for switching the context
                                                                 in dual MC. It contains biggest delay
                                                                 (expressed in counter ticks) between
                                                                 the counter crest and ADC latest
                                                                 trigger  */
  .bFreqRatio       = FREQ_RATIO,                           /*!< It is used in case of dual MC to
                                                                 synchronize TIM1 and TIM8. It has
                                                                 effect only on the second instanced
                                                                 object and must be equal to the
                                                                 ratio between the two PWM frequencies
                                                                 (higher/lower). Supported values are
                                                                 1, 2 or 3 */
  .bIsHigherFreqTim = FREQ_RELATION,                           /*!< When bFreqRatio is greather than 1
                                                                    this param is used to indicate if this
                                                                    instance is the one with the highest
                                                                    frequency. Allowed value are: HIGHER_FREQ
                                                                    or LOWER_FREQ */
  .IRQnb            = MC_IRQ_PWMNCURRFDBK_1,                     /*!< MC IRQ number used as TIMx Update
                                                                      event */
/* Current reading A/D Conversions initialization -----------------------------*/
 .bIaChannel       = PHASE_U_CURR_CHANNEL,                  /*!< ADC channel used for conversion of
                                                                 current Ia. It must be equal to
                                                                 ADC_Channel_x x= 0, ..., 15*/
 .hIaPort          = PHASE_U_GPIO_PORT,                  /*!< GPIO port used by hIaChannel. It must
                                                              be equal to GPIOx x= A, B, ...*/
 .hIaPin           = PHASE_U_GPIO_PIN,                      /*!< GPIO pin used by hIaChannel. It must be
                                                                 equal to GPIO_Pin_x x= 0, 1, ...*/
 .b_IaSamplingTime = SAMPLING_TIME_SEL,               /*!< Sampling time used to convert hIaChannel.
                                                           It must be equal to ADC_SampleTime_xCycles5
                                                           x= 1, 7, ...*/
 .bIbChannel       = PHASE_V_CURR_CHANNEL,                  /*!< ADC channel used for conversion of
                                                                 current Ib. It must be equal to
                                                                 ADC_Channel_x x= 0, ..., 15*/
 .hIbPort          = PHASE_V_GPIO_PORT,                /*!< GPIO port used by hIbChannel. It must
                                                            be equal to GPIOx x= A, B, ...*/
 .hIbPin           = PHASE_V_GPIO_PIN,                      /*!< GPIO pin used by hIaChannel. It must be
                                                                 equal to GPIO_Pin_x x= 0, 1, ...*/
 .b_IbSamplingTime = SAMPLING_TIME_SEL,                /*!< Sampling time used to convert hIbChannel.
                                                            It must be equal to ADC_SampleTime_xCycles5
                                                            x= 1, 7, ...*/
 .bIcChannel       = PHASE_W_CURR_CHANNEL,                  /*!< ADC channel used for conversion of
                                                                 current Ia. It must be equal to
                                                                 ADC_Channel_x x= 0, ..., 15*/
 .hIcPort          = PHASE_W_GPIO_PORT,                /*!< GPIO port used by hIaChannel. It must
                                                            be equal to GPIOx x= A, B, ...*/
 .hIcPin           = PHASE_W_GPIO_PIN,                      /*!< GPIO pin used by hIaChannel. It must be
                                                                 equal to GPIO_Pin_x x= 0, 1, ...*/
 .b_IcSamplingTime = SAMPLING_TIME_SEL,            /*!< Sampling time used to convert hIaChannel.
                                                        It must be equal to ADC_SampleTime_xCycles5
                                                        x= 1, 7, ...*/

/* PWM generation parameters --------------------------------------------------*/
  .hDeadTime          = DEAD_TIME_COUNTS,                    /*!< Dead time in number of TIM clock
                                                                  cycles. If CHxN are enabled, it must
                                                                  contain the dead time to be generated
                                                                  by the microcontroller, otherwise it
                                                                  expresses the maximum dead time
                                                                  generated by driving network */
  .bRepetitionCounter = REP_COUNTER,                         /*!< It expresses the number of PWM
                                                                  periods to be elapsed before compare
                                                                  registers are updated again. In
                                                                  particular:
                                                                  RepetitionCounter= (2* #PWM periods)-1*/
  .hTafter            = TW_AFTER,                             /*!< It is the sum of dead time plus max
                                                                  value between rise time and noise time
                                                                  express in number of TIM clocks.*/
  .hTbefore           = TW_BEFORE,                            /*!< It is the sampling time express in
                                                                   number of TIM clocks.*/
  .TIMx               = PWM_TIMER_SELECTION,                  /*!< It contains the pointer to the timer
                                                                   used for PWM generation. */
/* PWM Driving signals initialization ----------------------------------------*/

 .wTIM1Remapping = PWM_TIMER_REMAPPING,                        /*!< Used only for instance #1, it
                                                            remaps TIM1 outputs. It must equal to
                                                            GPIO_PartialRemap_TIM1 or
                                                            GPIO_FullRemap_TIM1 or
                                                            GPIO_NoRemap_TIM1 */
 .hCh1Polarity   = PHASE_UH_POLARITY,   	              /*!< Channel 1 (high side) output polarity,
                                                            it must be TIM_OCPolarity_High or
                                                            TIM_OCPolarity_Low */
 .hCh1Port       = PHASE_UH_GPIO_PORT,                 /*!< Channel 1 (high side) GPIO output
                                                            port (if used, after re-mapping).
                                                            It must be GPIOx x= A, B, ...*/
 .hCh1Pin        = PHASE_UH_GPIO_PIN,                  /*!< Channel 1 (high side) GPIO output pin
                                                           (if used, after re-mapping). It must be
                                                            GPIO_Pin_x x= 0, 1, ...*/
 .hCh1IdleState  = BRAKE_UH_POLARITY,                   /*!< Channel 1 (high side) state (low or high)
                                                             when TIM peripheral is in Idle state.
                                                             It must be TIM_OCIdleState_Set or
                                                             TIM_OCIdleState_Reset*/

 . hCh2Polarity  = PHASE_VH_POLARITY,                   /*!< Channel 2 (high side) output polarity,
                                                            it must be TIM_OCPolarity_High or
                                                            TIM_OCPolarity_Low */
 .hCh2Port       = PHASE_VH_GPIO_PORT,                               /*!< Channel 2 (high side) GPIO output
                                                            port (if used, after re-mapping).
                                                            It must be GPIOx x= A, B, ...*/
 .hCh2Pin        = PHASE_VH_GPIO_PIN,                       /*!< Channel 2 (high side) GPIO output pin
                                                           (if used, after re-mapping). It must be
                                                           GPIO_Pin_x x= 0, 1, ...*/
 .hCh2IdleState  = BRAKE_VH_POLARITY,              /*!< Channel 2 (high side) state (low or high)
                                                             when TIM peripheral is in Idle state.
                                                             It must be TIM_OCIdleState_Set or
                                                             TIM_OCIdleState_Reset*/

 .hCh3Polarity   = PHASE_WH_POLARITY,            	 /*!< Channel 3 (high side) output polarity,
                                                            it must be TIM_OCPolarity_High or
                                                            TIM_OCPolarity_Low */
 .hCh3Port       = PHASE_WH_GPIO_PORT,                               /*!< Channel 3 (high side) GPIO output
                                                            port (if used, after re-mapping).
                                                            It must be GPIOx x= A, B, ...*/
 .hCh3Pin        = PHASE_WH_GPIO_PIN,                        /*!< Channel 3 (high side) GPIO output pin
                                                           (if used, after re-mapping). It must be
                                                           GPIO_Pin_x x= 0, 1, ...*/
 .hCh3IdleState  = BRAKE_WH_POLARITY,              /*!< Channel 3 (high side) state (low or high)
                                                             when TIM peripheral is in Idle state.
                                                             It must be TIM_OCIdleState_Set or
                                                             TIM_OCIdleState_Reset*/

 .LowSideOutputs = (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING,  /*!< Low side or enabling signals
                                                                 generation method are defined
                                                                 here.*/

 .hCh1NPolarity  = PHASE_UL_POLARITY,               /*!< Channel 1N (low side) output polarity,
                                                            it must be TIM_OCNPolarity_High or
                                                            TIM_OCNPolarity_Low */
 .hCh1NPort      = PHASE_UL_GPIO_PORT,                                 /*!< Channel 1N (low side) GPIO output
                                                            port (if used, after re-mapping).
                                                            It must be GPIOx x= A, B, ...*/
 .hCh1NPin       = PHASE_UL_GPIO_PIN,                       /*!< Channel 1N (low side) GPIO output pin
                                                           (if used, after re-mapping). It must be
                                                           GPIO_Pin_x x= 0, 1, ...*/
 .hCh1NIdleState = BRAKE_UL_POLARITY,              /*!< Channel 1N (low side) state (low or high)
                                                             when TIM peripheral is in Idle state.
                                                             It must be TIM_OCNIdleState_Set or
                                                             TIM_OCNIdleState_Reset*/


 .hCh2NPolarity  = PHASE_VL_POLARITY,                 /*!< Channel 2N (low side) output polarity,
                                                            it must be TIM_OCNPolarity_High or
                                                            TIM_OCNPolarity_Low */
 .hCh2NPort      = PHASE_VL_GPIO_PORT,                                 /*!< Channel 2N (low side) GPIO output
                                                            port (if used, after re-mapping).
                                                            It must be GPIOx x= A, B, ...*/
 .hCh2NPin       = PHASE_VL_GPIO_PIN,                     /*!< Channel 2N (low side) GPIO output pin
                                                           (if used, after re-mapping). It must be
                                                           GPIO_Pin_x x= 0, 1, ...*/
 .hCh2NIdleState = BRAKE_VL_POLARITY,              /*!< Channel 2N (low side) state (low or high)
                                                             when TIM peripheral is in Idle state.
                                                             It must be TIM_OCNIdleState_Set or
                                                             TIM_OCNIdleState_Reset*/

 .hCh3NPolarity  = PHASE_WL_POLARITY,             /*!< Channel 3N (low side) output polarity,
                                                            it must be TIM_OCNPolarity_High or
                                                            TIM_OCNPolarity_Low */
 .hCh3NPort      = PHASE_WL_GPIO_PORT,                               /*!< Channel 3N (low side)  GPIO output
                                                            port (if used, after re-mapping).
                                                            It must be GPIOx x= A, B, ...*/
 .hCh3NPin       = PHASE_WL_GPIO_PIN,                    /*!< Channel 3N (low side)  GPIO output pin
                                                           (if used, after re-mapping). It must be
                                                           GPIO_Pin_x x= 0, 1, ...*/
 .hCh3NIdleState = BRAKE_WL_POLARITY,              /*!< Channel 3N (low side) state (low or high)
                                                             when TIM peripheral is in Idle state.
                                                             It must be TIM_OCNIdleState_Set or
                                                             TIM_OCNIdleState_Reset*/
/* PWM Driving signals initialization ----------------------------------------*/
  .bBKINMode     = BKIN_MODE,                          /*!< It defines the modality of emergency
                                                            input. It must be any of the
                                                            the following:
                                                            NONE - feature disabled.
                                                            INT_MODE - Internal comparator used
                                                            as source of emergency event.
                                                            EXT_MODE - External comparator used
                                                            as source of emergency event.
                                                            EXTINT_MODE - The or-ed signal
                                                            between internal and external
                                                            comparator is used as source event.*/
  .hBKINPolarity = EMSTOP_ACTIVE_HIGH,                   /*!< Emergency Stop (BKIN) input polarity,
                                                            it must be TIM_BreakPolarity_Low or
                                                            TIM_BreakPolarity_High */
  .hBKINPort     = EMERGENCY_STOP_GPIO_PORT,             /*!< Emergency Stop (BKIN) GPIO input
                                                            port (if used, after re-mapping).
                                                            It must be GPIOx x= A, B, ...*/
  .hBKINPin      = EMERGENCY_STOP_GPIO_PIN,              /*!< Emergency Stop (BKIN) GPIO input pin
                                                          (if used, after re-mapping). It must be
                                                           GPIO_Pin_x x= 0, 1, ...*/
/* Alternate functions definition --------------------------------------------*/
  .bCh1AF   = PHASE_UH_GPIO_AF,                   /*!< Channel 1 (high side) alternate
                                                       functions setting. It must be one
                                                       of the GPIO_AF_x (x=0,1, ...)
                                                       according to the defined GPIO port
                                                       and pin.*/
  .bCh2AF   = PHASE_VH_GPIO_AF,                   /*!< Channel 2 (high side) alternate
                                                       functions setting. It must be one
                                                       of the GPIO_AF_x (x=0,1, ...)
                                                       according to the defined GPIO port
                                                       and pin.*/
  .bCh3AF   = PHASE_WH_GPIO_AF,                   /*!< Channel 3 (high side) alternate
                                                       functions setting. It must be one
                                                       of the GPIO_AF_x (x=0,1, ...)
                                                       according to the defined GPIO port
                                                       and pin.*/
  .bCh1NAF  = PHASE_UL_GPIO_AF,                   /*!< Channel 1 (low side) alternate
                                                       functions setting. It must be one
                                                       of the GPIO_AF_x (x=0,1, ...)
                                                       according to the defined GPIO port
                                                       and pin.*/
  .bCh2NAF  = PHASE_VL_GPIO_AF,                   /*!< Channel 2 (low side) alternate
                                                       functions setting. It must be one
                                                       of the GPIO_AF_x (x=0,1, ...)
                                                       according to the defined GPIO port
                                                       and pin.*/
  .bCh3NAF  = PHASE_WL_GPIO_AF,                   /*!< Channel 3 (low side) alternate
                                                       functions setting. It must be one
                                                       of the GPIO_AF_x (x=0,1, ...)
                                                       according to the defined GPIO port
                                                       and pin.*/
  .bBKINAF  = BRKIN_GPIO_AF,                      /*!< Emergency Stop (BKIN) alternate
                                                       functions setting. It must be one
                                                       of the GPIO_AF_x (x=0,1, ...)
                                                       according to the defined GPIO port
                                                       and pin.*/
  .bBKIN2AF = BRKIN2_GPIO_AF,                     /*!< Emergency Stop (BKIN2) alternate
                                                       functions setting. It must be one
                                                       of the GPIO_AF_x (x=0,1, ...)
                                                       according to the defined GPIO port
                                                       and pin.*/
/* Emergency input (BKIN2) signal initialization -----------------------------*/
  .bBKIN2Mode     = BKIN2_MODE,                         /*!< It defines the modality of emergency
                                                             input 2. It must be any of the
                                                             the following:
                                                             NONE - feature disabled.
                                                             INT_MODE - Internal comparator used
                                                             as source of emergency event.
                                                             EXT_MODE - External comparator used
                                                             as source of emergency event.
                                                             EXTINT_MODE - The or-ed signal
                                                             between internal and external
                                                             comparator is used as source event.*/
  .hBKIN2Polarity = OVERCURR_FEEDBACK_POLARITY,         /*!< Emergency Stop (BKIN2) input polarity,
                                                             it must be TIM_Break2Polarity_Low or
                                                             TIM_Break2Polarity_High */
  .hBKIN2Port     = EMERGENCY2_STOP_GPIO_PORT,          /*!< Emergency Stop (BKIN) GPIO input
                                                             port (if used, after re-mapping).
                                                             It must be GPIOx x= A, B, ...*/
  .hBKIN2Pin      = EMERGENCY2_STOP_GPIO_PIN,           /*!< Emergency Stop (BKIN) GPIO input pin
                                                             (if used, after re-mapping). It must be
                                                             GPIO_Pin_x x= 0, 1, ...*/
/* Filtering settings of the emergency inputs --------------------------------*/
  .bBKINFilter  = BKIN1_FILTER,                         /*!< Emergency Stop (BKIN) digital
                                                             filter. See TIMx_BDTR (BKF bits)
                                                             definition in the reference
                                                             manual.*/
  .bBKIN2Filter = BKIN2_FILTER,                         /*!< Emergency Stop (BKIN2) digital
                                                             filter. See TIMx_BDTR (BKF bits)
                                                             definition in the reference
                                                             manual.*/
/* Internal OPAMP common settings --------------------------------------------*/
 .pOPAMPParams = R3_4_OPAMPPARAMSM1,                 /*!< Pointer to the OPAMP params struct.
                                                          It must be MC_NULL if internal
                                                          OPAMP are not used.*/
/* Internal COMP settings ----------------------------------------------------*/
 .pOCPA_COMPParams = OCPA_COMPPARAMSM1,
                                                         /*!< Pointer to the COMP params struct
                                                              used for overcurrent protection of
                                                              phase A.
                                                              It must be MC_NULL if internal COMP
                                                              are not used for OCP protection.*/
 .pOCPB_COMPParams = OCPB_COMPPARAMSM1,
                                                         /*!< Pointer to the COMP params struct
                                                              used for overcurrent protection of
                                                              phase B.
                                                              It must be MC_NULL if internal COMP
                                                               are not used for OCP protection.*/
 .pOCPC_COMPParams = OCPC_COMPPARAMSM1,
                                                         /*!< Pointer to the COMP params struct
                                                              used for overcurrent protection of
                                                              phase C.
                                                              It must be MC_NULL if internal COMP
                                                              are not used for OCP protection.*/
 .pOVP_COMPParams  = OVP_COMPPARAMSM1,
                                                         /*!< Pointer to the COMP params struct
                                                              used for overvoltage protection.
                                                              It must be MC_NULL if internal COMP
                                                              are not used for OVP protection.*/
/* DAC settings --------------------------------------------------------------*/
  .hDAC_OCP_Threshold = OCPREF,                             /*!< Value of analog reference expressed
                                                                 as 16bit unsigned integer.
                                                                 Ex. 0 = 0V 65536 = VDD_DAC.*/
  .hDAC_OVP_Threshold = OVPREF,                             /*!< Value of analog reference expressed
                                                                 as 16bit unsigned integer.
                                                                 Ex. 0 = 0V 65536 = VDD_DAC.*/
/* Regular conversion --------------------------------------------------------*/
  .regconvADCx = REGCONVADC                          /*!< ADC peripheral used for regular
                                                          conversion.*/
};

/**
  * @brief  Current sensor parameters Motor 1 - three shunt - F30x - Shared Resources
  */
R3_2_F30XParams_t R3_2_F30XParamsM1 =
{
  .wADC_Clock_Divider = ADC_CLOCK_DIVIDER,                /*!<  AHB clock prescaling factor for
                                                                ADC peripheral. It must be
                                                                RCC_ADC12PLLCLK_Divx or
                                                                RCC_ADC34PLLCLK_Divx
                                                                x = 1, 2, 4, 6, 8, ... */
  .wAHBPeriph         = ADC_AHBPERIPH,                     /*!< AHB periph used. It must be
                                                                RCC_AHBPeriph_ADC12 or
                                                                RCC_AHBPeriph_ADC34 or
                                                                RCC_AHBPeriph_ADC12 | RCC_AHBPeriph_ADC34.*/
  .bTim_Clock_Divider = TIM_CLOCK_DIVIDER,                  /* System clock divider for Advanded Timer */
/* Dual MC parameters --------------------------------------------------------*/
  .bInstanceNbr     = 1,                                    /*!< Instance number. Necessary to
                                                                 properly synchronize TIM8 with
                                                                 TIM1 at peripheral initializations
                                                                  */
  .Tw               = MAX_TWAIT,                            /*!< It is used for switching the context
                                                               in dual MC. It contains biggest delay
                                                               (expressed in counter ticks) between
                                                               the counter crest and ADC latest
                                                               trigger  */
  .bFreqRatio       = FREQ_RATIO,                           /*!< It is used in case of dual MC to
                                                               synchronize TIM1 and TIM8. It has
                                                               effect only on the second instanced
                                                               object and must be equal to the
                                                               ratio between the two PWM frequencies
                                                               (higher/lower). Supported values are
                                                               1, 2 or 3 */
  .bIsHigherFreqTim = FREQ_RELATION,                           /*!< When bFreqRatio is greather than 1
                                                              this param is used to indicate if this
                                                              instance is the one with the highest
                                                              frequency. Allowed value are: HIGHER_FREQ
                                                              or LOWER_FREQ */
  .IRQnb            = MC_IRQ_PWMNCURRFDBK_1,                     /*!< MC IRQ number used as TIMx Update
                                                               event */
/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx_1           = ADC_1_PERIPH,                         /*!< First ADC peripheral to be used.*/
  .ADCx_2           = ADC_2_PERIPH,                         /*!< Second ADC peripheral to be used.*/
  .bIaChannel       = PHASE_U_CURR_CHANNEL,                  /*!< ADC channel used for conversion of
                                                               current Ia. It must be equal to
                                                               ADC_Channel_x x= 0, ..., 15*/
  .hIaPort          = PHASE_U_GPIO_PORT,                  /*!< GPIO port used by hIaChannel. It must
                                                               be equal to GPIOx x= A, B, ...*/
  .hIaPin           = PHASE_U_GPIO_PIN,                      /*!< GPIO pin used by hIaChannel. It must be
                    =                                         equal to GPIO_Pin_x x= 0, 1, ...*/
  .b_IaSamplingTime = SAMPLING_TIME_SEL,               /*!< Sampling time used to convert hIaChannel.
                                                              It must be equal to ADC_SampleTime_xCycles5
                                                              x= 1, 7, ...*/
  .bIbChannel       = PHASE_V_CURR_CHANNEL,                  /*!< ADC channel used for conversion of
                                                               current Ib. It must be equal to
                                                               ADC_Channel_x x= 0, ..., 15*/
  .hIbPort          = PHASE_V_GPIO_PORT,                /*!< GPIO port used by hIbChannel. It must
                                                               be equal to GPIOx x= A, B, ...*/
  .hIbPin           = PHASE_V_GPIO_PIN,                      /*!< GPIO pin used by hIaChannel. It must be
                                                              equal to GPIO_Pin_x x= 0, 1, ...*/
  .b_IbSamplingTime = SAMPLING_TIME_SEL,                /*!< Sampling time used to convert hIbChannel.
                                                              It must be equal to ADC_SampleTime_xCycles5
                                                              x= 1, 7, ...*/
  .bIcChannel       = PHASE_W_CURR_CHANNEL,                  /*!< ADC channel used for conversion of
                                                               current Ia. It must be equal to
                                                               ADC_Channel_x x= 0, ..., 15*/
  .hIcPort          = PHASE_W_GPIO_PORT,                /*!< GPIO port used by hIaChannel. It must
                                                               be equal to GPIOx x= A, B, ...*/
  .hIcPin           = PHASE_W_GPIO_PIN,                      /*!< GPIO pin used by hIaChannel. It must be
                                                              equal to GPIO_Pin_x x= 0, 1, ...*/
  .b_IcSamplingTime = SAMPLING_TIME_SEL,            /*!< Sampling time used to convert hIaChannel.
                                                              It must be equal to ADC_SampleTime_xCycles5
                                                              x= 1, 7, ...*/

/* PWM generation parameters --------------------------------------------------*/
  .hDeadTime          = DEAD_TIME_COUNTS,                    /*!< Dead time in number of TIM clock
                                                                  cycles. If CHxN are enabled, it must
                                                                  contain the dead time to be generated
                                                                  by the microcontroller, otherwise it
                                                                  expresses the maximum dead time
                                                                  generated by driving network */
  .bRepetitionCounter = REP_COUNTER,                         /*!< It expresses the number of PWM
                                                                  periods to be elapsed before compare
                                                                  registers are updated again. In
                                                                  particular:
                                                                  RepetitionCounter= (2* #PWM periods)-1*/
  .hTafter            = TW_AFTER,                             /*!< It is the sum of dead time plus max
                                                                  value between rise time and noise time
                                                                  express in number of TIM clocks.*/
  .hTbefore           = TW_BEFORE,                           /*!< It is the sampling time express in
                                                                  number of TIM clocks.*/
  .TIMx               = PWM_TIMER_SELECTION,                               /*!< It contains the pointer to the timer
                                                                 used for PWM generation. */
/* PWM Driving signals initialization ----------------------------------------*/

  .wTIM1Remapping = PWM_TIMER_REMAPPING,                        /*!< Used only for instance #1, it
                                                             remaps TIM1 outputs. It must equal to
                                                             GPIO_PartialRemap_TIM1 or
                                                             GPIO_FullRemap_TIM1 or
                                                             GPIO_NoRemap_TIM1 */
  .hCh1Polarity   = PHASE_UH_POLARITY,   	              /*!< Channel 1 (high side) output polarity,
                                                             it must be TIM_OCPolarity_High or
                                                             TIM_OCPolarity_Low */
  .hCh1Port       = PHASE_UH_GPIO_PORT,                 /*!< Channel 1 (high side) GPIO output
                                                             port (if used, after re-mapping).
                                                             It must be GPIOx x= A, B, ...*/
  .hCh1Pin        = PHASE_UH_GPIO_PIN,                  /*!< Channel 1 (high side) GPIO output pin
                                                            (if used, after re-mapping). It must be
                                                             GPIO_Pin_x x= 0, 1, ...*/
  .hCh1IdleState  = BRAKE_UH_POLARITY,                   /*!< Channel 1 (high side) state (low or high)
                                                              when TIM peripheral is in Idle state.
                                                              It must be TIM_OCIdleState_Set or
                                                              TIM_OCIdleState_Reset*/

  .hCh2Polarity   = PHASE_VH_POLARITY,                   /*!< Channel 2 (high side) output polarity,
                                                             it must be TIM_OCPolarity_High or
                                                             TIM_OCPolarity_Low */
  .hCh2Port       = PHASE_VH_GPIO_PORT,                               /*!< Channel 2 (high side) GPIO output
                                                             port (if used, after re-mapping).
                                                             It must be GPIOx x= A, B, ...*/
  .hCh2Pin        = PHASE_VH_GPIO_PIN,                       /*!< Channel 2 (high side) GPIO output pin
                                                            (if used, after re-mapping). It must be
                                                            GPIO_Pin_x x= 0, 1, ...*/
  .hCh2IdleState  = BRAKE_VH_POLARITY,              /*!< Channel 2 (high side) state (low or high)
                                                              when TIM peripheral is in Idle state.
                                                              It must be TIM_OCIdleState_Set or
                                                              TIM_OCIdleState_Reset*/

  .hCh3Polarity   = PHASE_WH_POLARITY,            	 /*!< Channel 3 (high side) output polarity,
                                                             it must be TIM_OCPolarity_High or
                                                             TIM_OCPolarity_Low */
  .hCh3Port       = PHASE_WH_GPIO_PORT,                               /*!< Channel 3 (high side) GPIO output
                                                             port (if used, after re-mapping).
                                                             It must be GPIOx x= A, B, ...*/
  .hCh3Pin        = PHASE_WH_GPIO_PIN,                        /*!< Channel 3 (high side) GPIO output pin
                                                            (if used, after re-mapping). It must be
                                                            GPIO_Pin_x x= 0, 1, ...*/
  .hCh3IdleState  = BRAKE_WH_POLARITY,              /*!< Channel 3 (high side) state (low or high)
                                                              when TIM peripheral is in Idle state.
                                                              It must be TIM_OCIdleState_Set or
                                                              TIM_OCIdleState_Reset*/

  .LowSideOutputs = (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING,  /*!< Low side or enabling signals
                                                                  generation method are defined
                                                                  here.*/

  .hCh1NPolarity  = PHASE_UL_POLARITY,               /*!< Channel 1N (low side) output polarity,
                                                             it must be TIM_OCNPolarity_High or
                                                             TIM_OCNPolarity_Low */
  .hCh1NPort      = PHASE_UL_GPIO_PORT,                                 /*!< Channel 1N (low side) GPIO output
                                                             port (if used, after re-mapping).
                                                             It must be GPIOx x= A, B, ...*/
  .hCh1NPin       = PHASE_UL_GPIO_PIN,                       /*!< Channel 1N (low side) GPIO output pin
                                                            (if used, after re-mapping). It must be
                                                            GPIO_Pin_x x= 0, 1, ...*/
  .hCh1NIdleState = BRAKE_UL_POLARITY,              /*!< Channel 1N (low side) state (low or high)
                                                              when TIM peripheral is in Idle state.
                                                              It must be TIM_OCNIdleState_Set or
                                                              TIM_OCNIdleState_Reset*/


  .hCh2NPolarity  = PHASE_VL_POLARITY,                 /*!< Channel 2N (low side) output polarity,
                                                             it must be TIM_OCNPolarity_High or
                                                             TIM_OCNPolarity_Low */
  .hCh2NPort      = PHASE_VL_GPIO_PORT,                                 /*!< Channel 2N (low side) GPIO output
                                                             port (if used, after re-mapping).
                                                             It must be GPIOx x= A, B, ...*/
  .hCh2NPin       = PHASE_VL_GPIO_PIN,                     /*!< Channel 2N (low side) GPIO output pin
                                                            (if used, after re-mapping). It must be
                                                            GPIO_Pin_x x= 0, 1, ...*/
  .hCh2NIdleState = BRAKE_VL_POLARITY,              /*!< Channel 2N (low side) state (low or high)
                                                              when TIM peripheral is in Idle state.
                                                              It must be TIM_OCNIdleState_Set or
                                                              TIM_OCNIdleState_Reset*/

  .hCh3NPolarity  = PHASE_WL_POLARITY,             /*!< Channel 3N (low side) output polarity,
                                                             it must be TIM_OCNPolarity_High or
                                                             TIM_OCNPolarity_Low */
  .hCh3NPort      = PHASE_WL_GPIO_PORT,                               /*!< Channel 3N (low side)  GPIO output
                                                             port (if used, after re-mapping).
                                                             It must be GPIOx x= A, B, ...*/
  .hCh3NPin       = PHASE_WL_GPIO_PIN,                    /*!< Channel 3N (low side)  GPIO output pin
                                                            (if used, after re-mapping). It must be
                                                            GPIO_Pin_x x= 0, 1, ...*/
  .hCh3NIdleState = BRAKE_WL_POLARITY,              /*!< Channel 3N (low side) state (low or high)
                                                              when TIM peripheral is in Idle state.
                                                              It must be TIM_OCNIdleState_Set or
                                                              TIM_OCNIdleState_Reset*/
/* PWM Driving signals initialization ----------------------------------------*/
  .bBKINMode     = BKIN_MODE,                          /*!< It defines the modality of emergency
                                                            input. It must be any of the
                                                            the following:
                                                            NONE - feature disabled.
                                                            INT_MODE - Internal comparator used
                                                            as source of emergency event.
                                                            EXT_MODE - External comparator used
                                                            as source of emergency event.
                                                            EXTINT_MODE - The or-ed signal
                                                            between internal and external
                                                            comparator is used as source event.*/
  .hBKINPolarity = EMSTOP_ACTIVE_HIGH,                   /*!< Emergency Stop (BKIN) input polarity,
                                                            it must be TIM_BreakPolarity_Low or
                                                            TIM_BreakPolarity_High */
  .hBKINPort     = EMERGENCY_STOP_GPIO_PORT,             /*!< Emergency Stop (BKIN) GPIO input
                                                            port (if used, after re-mapping).
                                                            It must be GPIOx x= A, B, ...*/
  .hBKINPin      = EMERGENCY_STOP_GPIO_PIN,              /*!< Emergency Stop (BKIN) GPIO input pin
                                                          (if used, after re-mapping). It must be
                                                           GPIO_Pin_x x= 0, 1, ...*/
/* Alternate functions definition --------------------------------------------*/
  .bCh1AF   = PHASE_UH_GPIO_AF,                   /*!< Channel 1 (high side) alternate
                                                       functions setting. It must be one
                                                       of the GPIO_AF_x (x=0,1, ...)
                                                       according to the defined GPIO port
                                                       and pin.*/
  .bCh2AF   = PHASE_VH_GPIO_AF,                   /*!< Channel 2 (high side) alternate
                                                       functions setting. It must be one
                                                       of the GPIO_AF_x (x=0,1, ...)
                                                       according to the defined GPIO port
                                                       and pin.*/
  .bCh3AF   = PHASE_WH_GPIO_AF,                   /*!< Channel 3 (high side) alternate
                                                       functions setting. It must be one
                                                       of the GPIO_AF_x (x=0,1, ...)
                                                       according to the defined GPIO port
                                                       and pin.*/
  .bCh1NAF  = PHASE_UL_GPIO_AF,                   /*!< Channel 1 (low side) alternate
                                                       functions setting. It must be one
                                                       of the GPIO_AF_x (x=0,1, ...)
                                                       according to the defined GPIO port
                                                       and pin.*/
  .bCh2NAF  = PHASE_VL_GPIO_AF,                   /*!< Channel 2 (low side) alternate
                                                       functions setting. It must be one
                                                       of the GPIO_AF_x (x=0,1, ...)
                                                       according to the defined GPIO port
                                                       and pin.*/
  .bCh3NAF  = PHASE_WL_GPIO_AF,                   /*!< Channel 3 (low side) alternate
                                                       functions setting. It must be one
                                                       of the GPIO_AF_x (x=0,1, ...)
                                                       according to the defined GPIO port
                                                       and pin.*/
  .bBKINAF  = BRKIN_GPIO_AF,                      /*!< Emergency Stop (BKIN) alternate
                                                       functions setting. It must be one
                                                       of the GPIO_AF_x (x=0,1, ...)
                                                       according to the defined GPIO port
                                                       and pin.*/
  .bBKIN2AF = BRKIN2_GPIO_AF,                     /*!< Emergency Stop (BKIN2) alternate
                                                       functions setting. It must be one
                                                       of the GPIO_AF_x (x=0,1, ...)
                                                       according to the defined GPIO port
                                                       and pin.*/
/* Emergency input (BKIN2) signal initialization -----------------------------*/
  .bBKIN2Mode     = BKIN2_MODE,                         /*!< It defines the modality of emergency
                                                             input 2. It must be any of the
                                                             the following:
                                                             NONE - feature disabled.
                                                             INT_MODE - Internal comparator used
                                                             as source of emergency event.
                                                             EXT_MODE - External comparator used
                                                             as source of emergency event.
                                                             EXTINT_MODE - The or-ed signal
                                                             between internal and external
                                                             comparator is used as source event.*/
  .hBKIN2Polarity = OVERCURR_FEEDBACK_POLARITY,         /*!< Emergency Stop (BKIN2) input polarity,
                                                             it must be TIM_Break2Polarity_Low or
                                                             TIM_Break2Polarity_High */
  .hBKIN2Port     = EMERGENCY2_STOP_GPIO_PORT,          /*!< Emergency Stop (BKIN) GPIO input
                                                             port (if used, after re-mapping).
                                                             It must be GPIOx x= A, B, ...*/
  .hBKIN2Pin      = EMERGENCY2_STOP_GPIO_PIN,           /*!< Emergency Stop (BKIN) GPIO input pin
                                                             (if used, after re-mapping). It must be
                                                             GPIO_Pin_x x= 0, 1, ...*/
/* Filtering settings of the emergency inputs --------------------------------*/
  .bBKINFilter  = BKIN1_FILTER,                         /*!< Emergency Stop (BKIN) digital
                                                             filter. See TIMx_BDTR (BKF bits)
                                                             definition in the reference
                                                             manual.*/
  .bBKIN2Filter = BKIN2_FILTER,                         /*!< Emergency Stop (BKIN2) digital
                                                             filter. See TIMx_BDTR (BKF bits)
                                                             definition in the reference
                                                             manual.*/
/* Internal OPAMP common settings --------------------------------------------*/
  .pOPAMPParams     = R3_2_OPAMPPARAMSM1,               /*!< Pointer to the OPAMP params struct.
                                                             It must be MC_NULL if internal
                                                             OPAMP are not used.*/
/* Internal COMP settings ----------------------------------------------------*/
  .pOCPA_COMPParams = OCPA_COMPPARAMSM1,
                                                          /*!< Pointer to the COMP params struct
                                                               used for overcurrent protection of
                                                               phase A.
                                                               It must be MC_NULL if internal COMP
                                                               are not used for OCP protection.*/
  .pOCPB_COMPParams = OCPB_COMPPARAMSM1,
                                                          /*!< Pointer to the COMP params struct
                                                               used for overcurrent protection of
                                                               phase B.
                                                               It must be MC_NULL if internal COMP
                                                               are not used for OCP protection.*/
  .pOCPC_COMPParams = OCPC_COMPPARAMSM1,
                                                          /*!< Pointer to the COMP params struct
                                                               used for overcurrent protection of
                                                               phase C.
                                                               It must be MC_NULL if internal COMP
                                                               are not used for OCP protection.*/
  .pOVP_COMPParams  = OVP_COMPPARAMSM1,
                                                          /*!< Pointer to the COMP params struct
                                                               used for overvoltage protection.
                                                               It must be MC_NULL if internal COMP
                                                               are not used for OVP protection.*/
/* DAC settings --------------------------------------------------------------*/
  .hDAC_OCP_Threshold = OCPREF,                             /*!< Value of analog reference expressed
                                                                 as 16bit unsigned integer.
                                                                 Ex. 0 = 0V 65536 = VDD_DAC.*/
  .hDAC_OVP_Threshold = OVPREF,                             /*!< Value of analog reference expressed
                                                                 as 16bit unsigned integer.
                                                                 Ex. 0 = 0V 65536 = VDD_DAC.*/
/* Regular conversion --------------------------------------------------------*/
  .regconvADCx = REGCONVADC                          /*!< ADC peripheral used for regular
                                                          conversion.*/
};

/**
  * @brief  Internal OPAMP parameters Motor 1 - single shunt - F30x
  */
R1_F30XOPAMPParams_t R1_F30XOPAMPParamsM1 =
{
/* OPAMP settings ------------------------------------------------------*/
  .wOPAMP_Selection                   = OPAMP_SELECTION,                        /*!< First OPAMP selection. It must be
                                                                                 either equal to
                                                                                 OPAMP_Selection_OPAMP1 or
                                                                                 OPAMP_Selection_OPAMP3.*/
  .bOPAMP_InvertingInput_MODE         = OPAMP_INVERTINGINPUT_MODE,            /*!< First OPAMP inverting input mode.
                                                                                 It must be either equal to EXT_MODE
                                                                                 or INT_MODE.*/
  .wOPAMP_InvertingInput              = OPAMP_INVERTINGINPUT,              /*!< First OPAMP inverting input pin.
                                                                                 If wOPAMP_Selection is
                                                                                 OPAMP_Selection_OPAMP1 it
                                                                                 must be one of the following:
                                                                                 OPAMP1_InvertingInput_PC5 or
                                                                                 OPAMP1_InvertingInput_PA3 if the
                                                                                 bOPAMP_InvertingInput_MODE is
                                                                                 EXT_MODE or
                                                                                 OPAMP1_InvertingInput_PGA or
                                                                                 OPAMP1_InvertingInput_FOLLOWER if the
                                                                                 bOPAMP_InvertingInput_MODE is
                                                                                 INT_MODE.

                                                                                 If wOPAMP_Selection is
                                                                                 OPAMP_Selection_OPAMP3 it
                                                                                 must be one of the following:
                                                                                 OPAMP3_InvertingInput_PB10 or
                                                                                 OPAMP3_InvertingInput_PB2 if the
                                                                                 bOPAMP_InvertingInput_MODE is
                                                                                 EXT_MODE or
                                                                                 OPAMP3_InvertingInput_PGA or
                                                                                 OPAMP3_InvertingInput_FOLLOWER if the
                                                                                 bOPAMP_InvertingInput_MODE is
                                                                                 INT_MODE.*/
  .hOPAMP_InvertingInput_GPIO_PORT    = OPAMP_INVERTINGINPUT_GPIO_PORT,          /*!< First OPAMP inverting input GPIO port
                                                                                 as defined in wOPAMP_InvertingInput.
                                                                                 It must be GPIOx x= A, B, ... if
                                                                                 bOPAMP_InvertingInput_MODE is
                                                                                 EXT_MODE, otherwise can be dummy.*/
  .hOPAMP_InvertingInput_GPIO_PIN     = OPAMP_INVERTINGINPUT_GPIO_PIN,           /*!< First OPAMP inverting input GPIO pin
                                                                                 as defined in wOPAMP_InvertingInput.
                                                                                 It must be GPIO_Pin_x x= 0, 1, ... if
                                                                                 bOPAMP_InvertingInput_MODE is
                                                                                 EXT_MODE, otherwise can be dummy.*/
  .wOPAMP_NonInvertingInput           = OPAMP_NONINVERTINGINPUT,                 /*!< OPAMP non inverting input first
                                                                                 selection.
                                                                                 If wOPAMP_Selection is
                                                                                 OPAMP_Selection_OPAMP1 it
                                                                                 must be one of the following:
                                                                                 OPAMP1_NonInvertingInput_PA7,
                                                                                 OPAMP1_NonInvertingInput_PA5,
                                                                                 OPAMP1_NonInvertingInput_PA3,
                                                                                 OPAMP1_NonInvertingInput_PA1.

                                                                                 If wOPAMP_Selection is
                                                                                 OPAMP_Selection_OPAMP3 it
                                                                                 must be one of the following:
                                                                                 OPAMP3_NonInvertingInput_PB13,
                                                                                 OPAMP3_NonInvertingInput_PA5,
                                                                                 OPAMP3_NonInvertingInput_PA1,
                                                                                 OPAMP3_NonInvertingInput_PB0.*/
  .hOPAMP_NonInvertingInput_GPIO_PORT = OPAMP_NONINVERTINGINPUT_GPIO_PORT,        /*!< OPAMP non inverting input GPIO
                                                                                 port as defined in
                                                                                 wOPAMP_NonInvertingInput_PHA.
                                                                                 It must be GPIOx x= A, B, ...*/
  .hOPAMP_NonInvertingInput_GPIO_PIN  = OPAMP_NONINVERTINGINPUT_GPIO_PIN,        /*!< OPAMP non inverting input GPIO
                                                                                 pin as defined in
                                                                                 wOPAMP_NonInvertingInput_PHA.
                                                                                 It must be GPIO_Pin_x x= 0, 1, ...*/
  .hOPAMP_Output_GPIO_PORT            = OPAMP_OUT_GPIO_PORT,                     /*!< OPAMP output GPIO port.
                                                                                 It must be GPIOx x= A, B, ...
                                                                                 Note: Output pin is PA2 for OPAMP1,
                                                                                 PB1 for OPAMP3.*/
  .hOPAMP_Output_GPIO_PIN             = OPAMP_OUT_GPIO_PIN,                      /*!< OPAMP output GPIO pin.
                                                                                 It must be GPIO_Pin_x x= 0, 1, ...
                                                                                 Note: Output pin is PA2 for OPAMP1,
                                                                                 PB1 for OPAMP3.*/
/* Common settings -----------------------------------------------------------*/
  .wOPAMP_PGAGain   = OPAMP_PGAGAIN,                            /*!< It defines the OPAMP PGA gains.
                                                                     It must be one of the following:
                                                                     OPAMP_OPAMP_PGAGain_2,
                                                                     OPAMP_OPAMP_PGAGain_4,
                                                                     OPAMP_OPAMP_PGAGain_8,
                                                                     OPAMP_OPAMP_PGAGain_16.
                                                                     This value is taken in account
                                                                     just if wOPAMPx_InvertingInput is
                                                                     equal to OPAMP2_InvertingInput_PGA*/
  .OPAMP_PGAConnect = OPAMP_PGACONNECT,                       /*!< It defines the OPAMP connection
                                                                   with an external filter when PGA is enabled.
                                                                   It must be one of the following:
                                                                   OPAMP_PGAConnect_No,
                                                                   OPAMP_PGAConnect_IO1,
                                                                   OPAMP_PGAConnect_IO2.
                                                                   See reference manual RM0316.
                                                                   This value is taken in account
                                                                   just if wOPAMPx_InvertingInput is
                                                                   equal to OPAMP2_InvertingInput_PGA*/
};

/**
  * @brief  Internal COMP parameters Motor 1 - single shunt - F30x - OCP
  */
F30XCOMPParams_t R1_F30XOCPCOMPParamsM1 =
{
 .wSelection                   = OCP_SELECTION,                   /*!< Internal comparator used for protection.
                                                                  It must be
                                                                  COMP_Selection_COMPx x = 1,2,3,4,5,6,7.*/

 .bInvertingInput_MODE         = OCP_INVERTINGINPUT_MODE,         /*!< COMPx inverting input mode. It must be either
                                                                  equal to EXT_MODE or INT_MODE. */

 .wInvertingInput              = OCP_INVERTINGINPUT,              /*!< COMPx inverting input pin. It must be one of
                                                                  the following:
                                                                  COMP1_InvertingInput_PA0,
                                                                  COMP2_InvertingInput_PA2,
                                                                  COMP3_InvertingInput_PD15,
                                                                  COMP3_InvertingInput_PB12,
                                                                  COMP4_InvertingInput_PE8,
                                                                  COMP4_InvertingInput_PB2,
                                                                  COMP5_InvertingInput_PD13,
                                                                  COMP5_InvertingInput_PB10,
                                                                  COMP6_InvertingInput_PD10,
                                                                  COMP6_InvertingInput_PB15 if
                                                                  bInvertingInput_MODE is EXT_MODE or
                                                                  COMPX_InvertingInput_DAC1,
                                                                  COMPX_InvertingInput_DAC2,
                                                                  COMPX_InvertingInput_VREF,
                                                                  COMPX_InvertingInput_VREF_1_4,
                                                                  COMPX_InvertingInput_VREF_1_2,
                                                                  COMPX_InvertingInput_VREF_3_4 if
                                                                  bInvertingInput_MODE is INT_MODE.
                                                                  If bInvertingInput_MODE is EXT_MODE, the
                                                                  only available options are related to the
                                                                  selected COMP in wSelection.*/

 .hInvertingInput_GPIO_PORT    = OCP_INVERTINGINPUT_GPIO_PORT,    /*!< COMPx inverting input GPIO port as defined in
                                                                  wInvertingInput (Just if
                                                                  bInvertingInput_MODE is EXT_MODE).
                                                                  It must be GPIOx x= A, B, ...*/

 .hInvertingInput_GPIO_PIN     = OCP_INVERTINGINPUT_GPIO_PIN,     /*!< COMPx inverting input GPIO pin as defined in
                                                                  wInvertingInput (Just if
                                                                  bInvertingInput_MODE is EXT_MODE).
                                                                  It must be GPIO_Pin_x x= 0, 1, ...*/

 .wNonInvertingInput           = OCP_NONINVERTINGINPUT,           /*!< COMPx non inverting input. It must be one of
                                                                  the following:
                                                                  COMP1_NonInvertingInput_PA1,
                                                                  COMP2_NonInvertingInput_PA3,
                                                                  COMP2_NonInvertingInput_PA7,
                                                                  COMP3_NonInvertingInput_PB14,
                                                                  COMP3_NonInvertingInput_PD14,
                                                                  COMP4_NonInvertingInput_PB0,
                                                                  COMP4_NonInvertingInput_PE7,
                                                                  COMP5_NonInvertingInput_PB13,
                                                                  COMP5_NonInvertingInput_PD12,
                                                                  COMP6_NonInvertingInput_PB11,
                                                                  COMP6_NonInvertingInput_PD11,
                                                                  COMP7_NonInvertingInput_PC1,
                                                                  COMP7_NonInvertingInput_PA0.
                                                                  The only available options are related to the
                                                                  selected COMP in wSelection.*/

 .hNonInvertingInput_GPIO_PORT = OCP_NONINVERTINGINPUT_GPIO_PORT, /*!< COMPx non inverting input GPIO port as
                                                                  defined in wNonInvertingInput.
                                                                  It must be GPIOx x= A, B, ...*/

 .hNonInvertingInput_GPIO_PIN  = OCP_NONINVERTINGINPUT_GPIO_PIN,  /*!< COMPx non inverting input GPIO pin as defined
                                                                  in wNonInvertingInput.
                                                                  It must be GPIO_Pin_x x= 0, 1, ...*/

 .bOutput_MODE                 = OCP_OUTPUT_MODE,                 /*!< COMPx output. It must be either
                                                                  equal to EXT_MODE or INT_MODE. */

 .wOutput                      = OCP_OUTPUT,                      /*!< COMPx output selection. It must be one of
                                                                  the following:
                                                                  COMP_Output_TIM1BKIN,
                                                                  COMP_Output_TIM1BKIN2,
                                                                  COMP_Output_TIM8BKIN,
                                                                  COMP_Output_TIM8BKIN2,
                                                                  COMP_Output_TIM1BKIN2_TIM8BKIN2.*/
 .hOutput_GPIO_PORT            = OCP_OUTPUT_GPIO_PORT,            /*!< COMPx output GPIO port.
                                                                  It must be GPIOx x= A, B, ...*/

 .hOutput_GPIO_PIN             = OCP_OUTPUT_GPIO_PIN,             /*!< COMPx output GPIO pin as defined.
                                                                  It must be GPIO_Pin_x x= 0, 1, ...*/

 .bOutput_GPIO_AF              = OCP_OUTPUT_GPIO_AF,         /*!< COMPx output alternate functions setting.
                                                                  It must be one of the GPIO_AF_x (x=0,1, ...)
                                                                  according to the defined GPIO port and pin.*/
 .wOutputPol                   = OCP_OUTPUTPOL,                   /*!< COMPx output polarity. It must be one of
                                                                  the following:
                                                                  COMP_OutputPol_NonInverted,
                                                                  COMP_OutputPol_Inverted.*/

 .wMode                        = OCP_FILTER,                      /*!< COMPx mode it is used to define both
                                                                  the speed (analog filter) and
                                                                  the power consumption of the internal
                                                                  comparator. It must be one of the
                                                                  following:
                                                                  COMP_Mode_HighSpeed,
                                                                  COMP_Mode_MediumSpeed,
                                                                  COMP_Mode_LowPower,
                                                                  COMP_Mode_UltraLowPower.
                                                                  More speed means less filter and more
                                                                  consumption.*/
};

/**
  * @brief  Current sensor parameters Motor 1 - single shunt - F30x
  */
R1_F30XParams_t R1_F30XParamsM1 =
{
  .wADC_Clock_Divider = ADC_CLOCK_DIVIDER,                /*!<  AHB clock prescaling factor for
                                                                ADC peripheral. It must be
                                                                RCC_ADC12PLLCLK_Divx or
                                                                RCC_ADC34PLLCLK_Divx
                                                                x = 1, 2, 4, 6, 8, ... */
  .wAHBPeriph         = ADC_AHBPERIPH,                     /*!< AHB periph used. It must be
                                                                RCC_AHBPeriph_ADC12 or
                                                                RCC_AHBPeriph_ADC34. */
  .bTim_Clock_Divider = TIM_CLOCK_DIVIDER,                 /*!< APB2 clock prescaling factor for
                                                                ADC peripheral. It must be equal to 1,
                                                                2 or 4*/
/* Dual MC parameters --------------------------------------------------------*/
  .bInstanceNbr     = 1,                              /*!< Instance number with reference to PWMC
                                                           base class. It is necessary to properly
                                                           synchronize TIM8 with TIM1 at peripheral
                                                           initializations */
  .bFreqRatio       = FREQ_RATIO,                     /*!< It is used in case of dual MC to
                                                            synchronize TIM1 and TIM8. It has
                                                            effect only on the second instanced
                                                            object and must be equal to the
                                                            ratio between the two PWM frequencies
                                                            (higher/lower). Supported values are
                                                            1, 2 or 3 */
  .bIsHigherFreqTim = FREQ_RELATION,                   /*!< When bFreqRatio is greather than 1
                                                            this param is used to indicate if this
                                                            instance is the one with the highest
                                                            frequency. Allowed value are: HIGHER_FREQ
                                                            or LOWER_FREQ */
  .IRQnb            = MC_IRQ_PWMNCURRFDBK_1,             /*!< MC IRQ number used */
/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx            = ADC_PERIPH,                          /*!< ADC peripheral to be used*/
  .bIChannel       = PHASE_CURRENTS_CHANNEL,               /*!< ADC channel used for conversion of
                                                              current. It must be equal to
                                                              ADC_Channel_x x= 0, ..., 15*/
  .hIPort          = PHASE_CURRENTS_GPIO_PORT,             /*!< GPIO port used by bIChannel. It must
                                                              be equal to GPIOx x= A, B, ...*/
  .hIPin           = PHASE_CURRENTS_GPIO_PIN,              /*!< GPIO pin used by bIChannel. It must be
                                                             equal to GPIO_Pin_x x= 0, 1, ...*/
  .b_ISamplingTime = SAMPLING_TIME_SEL,                /*!< Sampling time used to convert bIChannel.
                                                             It must be equal to ADC_SampleTime_xCycles5
                                                             x= 1, 7, ...*/
/* PWM generation parameters --------------------------------------------------*/
  .hDeadTime          = DEAD_TIME_COUNTS,                  /*!< Dead time in number of TIM clock
                                                             cycles. If CHxN are enabled, it must
                                                             contain the dead time to be generated
                                                             by the microcontroller, otherwise it
                                                             expresses the maximum dead time
                                                             generated by driving network*/
  .bRepetitionCounter = REP_COUNTER,                      /*!< It expresses the number of PWM
                                                             periods to be elapsed before compare
                                                             registers are updated again. In
                                                             particular:
                                                             RepetitionCounter= (2* PWM periods) -1*/
  .hTafter            = TAFTER,                           /*!< It is the sum of dead time plus rise time
                                                             express in number of TIM clocks.*/
  .hTbefore           = TBEFORE,                          /*!< It is the value of sampling time
                                                             expressed in numbers of TIM clocks.*/
  .hTMin              = TMIN,                             /*!< It is the sum of dead time plus rise time
                                                             plus sampling time express in numbers of
                                                             TIM clocks.*/
  .hHTMin             = HTMIN,                            /*!< It is the half of hTMin value*/
  .hCHTMin            = CHTMIN,                           /*!< It is the half of hTMin value, considering FOC rate*/
  .hTSample           = TSAMPLE,                          /*!< It is the sampling time express in
                                                             numbers of TIM clocks.*/
  .hMaxTrTs           = MAX_TRTS,                         /*!< It is the maximum between twice of rise
                                                             time express in number of TIM clocks and
                                                             twice of sampling time express in numbers
                                                             of TIM clocks.*/
  .TIMx               = PWM_TIMER_SELECTION,              /*!< It contains the pointer to the timer
                                                                  used for PWM generation. It must
                                                                  equal to TIM1 if bInstanceNbr is
                                                                  equal to 1, to TIM8 otherwise */
/* PWM Driving signals initialization ----------------------------------------*/

  .wTIM1Remapping = PWM_TIMER_REMAPPING,                        /*!< Used only for instance #1, it
                                                             remaps TIM1 outputs. It must equal to
                                                             GPIO_PartialRemap_TIM1 or
                                                             GPIO_FullRemap_TIM1 or
                                                             GPIO_NoRemap_TIM1 */
  .hCh1Polarity   = PHASE_UH_POLARITY,   	              /*!< Channel 1 (high side) output polarity,
                                                             it must be TIM_OCPolarity_High or
                                                             TIM_OCPolarity_Low */
  .hCh1Port       = PHASE_UH_GPIO_PORT,                 /*!< Channel 1 (high side) GPIO output
                                                             port (if used, after re-mapping).
                                                             It must be GPIOx x= A, B, ...*/
  .hCh1Pin        = PHASE_UH_GPIO_PIN,                  /*!< Channel 1 (high side) GPIO output pin
                                                            (if used, after re-mapping). It must be
                                                             GPIO_Pin_x x= 0, 1, ...*/
  .hCh1IdleState  = BRAKE_UH_POLARITY,                   /*!< Channel 1 (high side) state (low or high)
                                                              when TIM peripheral is in Idle state.
                                                              It must be TIM_OCIdleState_Set or
                                                              TIM_OCIdleState_Reset*/

  .hCh2Polarity   = PHASE_VH_POLARITY,                   /*!< Channel 2 (high side) output polarity,
                                                             it must be TIM_OCPolarity_High or
                                                             TIM_OCPolarity_Low */
  .hCh2Port       = PHASE_VH_GPIO_PORT,                               /*!< Channel 2 (high side) GPIO output
                                                             port (if used, after re-mapping).
                                                             It must be GPIOx x= A, B, ...*/
  .hCh2Pin        = PHASE_VH_GPIO_PIN,                       /*!< Channel 2 (high side) GPIO output pin
                                                            (if used, after re-mapping). It must be
                                                            GPIO_Pin_x x= 0, 1, ...*/
  .hCh2IdleState  = BRAKE_VH_POLARITY,              /*!< Channel 2 (high side) state (low or high)
                                                              when TIM peripheral is in Idle state.
                                                              It must be TIM_OCIdleState_Set or
                                                              TIM_OCIdleState_Reset*/

  .hCh3Polarity   = PHASE_WH_POLARITY,            	 /*!< Channel 3 (high side) output polarity,
                                                             it must be TIM_OCPolarity_High or
                                                             TIM_OCPolarity_Low */
  .hCh3Port       = PHASE_WH_GPIO_PORT,                               /*!< Channel 3 (high side) GPIO output
                                                             port (if used, after re-mapping).
                                                             It must be GPIOx x= A, B, ...*/
  .hCh3Pin        = PHASE_WH_GPIO_PIN,                        /*!< Channel 3 (high side) GPIO output pin
                                                            (if used, after re-mapping). It must be
                                                            GPIO_Pin_x x= 0, 1, ...*/
  .hCh3IdleState  = BRAKE_WH_POLARITY,              /*!< Channel 3 (high side) state (low or high)
                                                              when TIM peripheral is in Idle state.
                                                              It must be TIM_OCIdleState_Set or
                                                              TIM_OCIdleState_Reset*/

  .LowSideOutputs = (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING,  /*!< Low side or enabling signals
                                                                  generation method are defined
                                                                  here.*/

  .hCh1NPolarity  = PHASE_UL_POLARITY,               /*!< Channel 1N (low side) output polarity,
                                                             it must be TIM_OCNPolarity_High or
                                                             TIM_OCNPolarity_Low */
  .hCh1NPort      = PHASE_UL_GPIO_PORT,                                 /*!< Channel 1N (low side) GPIO output
                                                             port (if used, after re-mapping).
                                                             It must be GPIOx x= A, B, ...*/
  .hCh1NPin       = PHASE_UL_GPIO_PIN,                       /*!< Channel 1N (low side) GPIO output pin
                                                            (if used, after re-mapping). It must be
                                                            GPIO_Pin_x x= 0, 1, ...*/
  .hCh1NIdleState = BRAKE_UL_POLARITY,              /*!< Channel 1N (low side) state (low or high)
                                                              when TIM peripheral is in Idle state.
                                                              It must be TIM_OCNIdleState_Set or
                                                              TIM_OCNIdleState_Reset*/


  .hCh2NPolarity  = PHASE_VL_POLARITY,                 /*!< Channel 2N (low side) output polarity,
                                                             it must be TIM_OCNPolarity_High or
                                                             TIM_OCNPolarity_Low */
  .hCh2NPort      = PHASE_VL_GPIO_PORT,                                 /*!< Channel 2N (low side) GPIO output
                                                             port (if used, after re-mapping).
                                                             It must be GPIOx x= A, B, ...*/
  .hCh2NPin       = PHASE_VL_GPIO_PIN,                     /*!< Channel 2N (low side) GPIO output pin
                                                            (if used, after re-mapping). It must be
                                                            GPIO_Pin_x x= 0, 1, ...*/
  .hCh2NIdleState = BRAKE_VL_POLARITY,              /*!< Channel 2N (low side) state (low or high)
                                                              when TIM peripheral is in Idle state.
                                                              It must be TIM_OCNIdleState_Set or
                                                              TIM_OCNIdleState_Reset*/

  .hCh3NPolarity  = PHASE_WL_POLARITY,             /*!< Channel 3N (low side) output polarity,
                                                             it must be TIM_OCNPolarity_High or
                                                             TIM_OCNPolarity_Low */
  .hCh3NPort      = PHASE_WL_GPIO_PORT,                               /*!< Channel 3N (low side)  GPIO output
                                                             port (if used, after re-mapping).
                                                             It must be GPIOx x= A, B, ...*/
  .hCh3NPin       = PHASE_WL_GPIO_PIN,                    /*!< Channel 3N (low side)  GPIO output pin
                                                            (if used, after re-mapping). It must be
                                                            GPIO_Pin_x x= 0, 1, ...*/
  .hCh3NIdleState = BRAKE_WL_POLARITY,              /*!< Channel 3N (low side) state (low or high)
                                                              when TIM peripheral is in Idle state.
                                                              It must be TIM_OCNIdleState_Set or
                                                              TIM_OCNIdleState_Reset*/
/* Emergency input signal initialization -------------------------------------*/
  .bBKINMode     = BKIN_MODE,                           /*!< It defines the modality of emergency
                                                            input. It must be any of the
                                                            the following:
                                                            NONE - feature disabled.
                                                            INT_MODE - Internal comparator used
                                                            as source of emergency event.
                                                            EXT_MODE - External comparator used
                                                            as source of emergency event.
                                                            EXTINT_MODE - The or-ed signal
                                                            between internal and external
                                                            comparator is used as source event.*/
  .hBKINPolarity = EMSTOP_ACTIVE_HIGH,                 /*!< Emergency Stop (BKIN) input polarity,
                                                            it must be TIM_BreakPolarity_Low or
                                                            TIM_BreakPolarity_High */
  .hBKINPort     = EMERGENCY_STOP_GPIO_PORT,             /*!< Emergency Stop (BKIN) GPIO input
                                                            port (if used, after re-mapping).
                                                            It must be GPIOx x= A, B, ...*/
  .hBKINPin      = EMERGENCY_STOP_GPIO_PIN,              /*!< Emergency Stop (BKIN) GPIO input pin
                                                              (if used, after re-mapping). It must be
                                                              GPIO_Pin_x x= 0, 1, ...*/
/* Alternate functions definition --------------------------------------------*/
 .bCh1AF   = PHASE_UH_GPIO_AF,                   /*!< Channel 1 (high side) alternate
                                                      functions setting. It must be one
                                                      of the GPIO_AF_x (x=0,1, ...)
                                                      according to the defined GPIO port
                                                      and pin.*/
 .bCh2AF   = PHASE_VH_GPIO_AF,                   /*!< Channel 2 (high side) alternate
                                                      functions setting. It must be one
                                                      of the GPIO_AF_x (x=0,1, ...)
                                                      according to the defined GPIO port
                                                      and pin.*/
 .bCh3AF   = PHASE_WH_GPIO_AF,                   /*!< Channel 3 (high side) alternate
                                                      functions setting. It must be one
                                                      of the GPIO_AF_x (x=0,1, ...)
                                                      according to the defined GPIO port
                                                      and pin.*/
 .bCh1NAF  = PHASE_UL_GPIO_AF,                   /*!< Channel 1 (low side) alternate
                                                      functions setting. It must be one
                                                      of the GPIO_AF_x (x=0,1, ...)
                                                      according to the defined GPIO port
                                                      and pin.*/
 .bCh2NAF  = PHASE_VL_GPIO_AF,                   /*!< Channel 2 (low side) alternate
                                                      functions setting. It must be one
                                                      of the GPIO_AF_x (x=0,1, ...)
                                                      according to the defined GPIO port
                                                      and pin.*/
 .bCh3NAF  = PHASE_WL_GPIO_AF,                   /*!< Channel 3 (low side) alternate
                                                      functions setting. It must be one
                                                      of the GPIO_AF_x (x=0,1, ...)
                                                      according to the defined GPIO port
                                                      and pin.*/
 .bBKINAF  = BRKIN_GPIO_AF,                      /*!< Emergency Stop (BKIN) alternate
                                                      functions setting. It must be one
                                                      of the GPIO_AF_x (x=0,1, ...)
                                                      according to the defined GPIO port
                                                      and pin.*/
 .bBKIN2AF = BRKIN2_GPIO_AF,                     /*!< Emergency Stop (BKIN2) alternate
                                                      functions setting. It must be one
                                                      of the GPIO_AF_x (x=0,1, ...)
                                                      according to the defined GPIO port
                                                      and pin.*/
/* Emergency input (BKIN2) signal initialization -----------------------------*/
  .bBKIN2Mode      = BKIN2_MODE,                         /*!< It defines the modality of emergency
                                                              input 2. It must be any of the
                                                              the following:
                                                              NONE - feature disabled.
                                                              INT_MODE - Internal comparator used
                                                              as source of emergency event.
                                                              EXT_MODE - External comparator used
                                                              as source of emergency event.
                                                              EXTINT_MODE - The or-ed signal
                                                              between internal and external
                                                              comparator is used as source event.*/
  .hBKIN2Polarity  = OVERCURR_FEEDBACK_POLARITY,         /*!< Emergency Stop (BKIN2) input polarity,
                                                              it must be TIM_Break2Polarity_Low or
                                                              TIM_Break2Polarity_High */
  .hBKIN2Port      = EMERGENCY2_STOP_GPIO_PORT,          /*!< Emergency Stop (BKIN) GPIO input
                                                              port (if used, after re-mapping).
                                                              It must be GPIOx x= A, B, ...*/
  .hBKIN2Pin       = EMERGENCY2_STOP_GPIO_PIN,           /*!< Emergency Stop (BKIN) GPIO input pin
                                                              (if used, after re-mapping). It must be
                                                              GPIO_Pin_x x= 0, 1, ...*/
/* Filtering settings of the emergency inputs --------------------------------*/
  .bBKINFilter  = BKIN1_FILTER,                         /*!< Emergency Stop (BKIN) digital
                                                             filter. See TIMx_BDTR (BKF bits)
                                                             definition in the reference
                                                             manual.*/
  .bBKIN2Filter = BKIN2_FILTER,                         /*!< Emergency Stop (BKIN2) digital
                                                             filter. See TIMx_BDTR (BKF bits)
                                                             definition in the reference
                                                             manual.*/
/* Internal OPAMP common settings --------------------------------------------*/
  .pOPAMPParams = R1_OPAMPPARAMSM1,                   /*!< Pointer to the OPAMP params struct.
                                                           It must be MC_NULL if internal
                                                           OPAMP are not used.*/
/* Internal COMP settings ----------------------------------------------------*/
  .pOCP_COMPParams = OCP_COMPPARAMSM1,                     /*!< Pointer to the COMP params struct
                                                              used for overcurrent protection of
                                                              phase A.
                                                              It must be MC_NULL if internal COMP
                                                              are not used for OCP protection.*/
  .pOVP_COMPParams = OVP_COMPPARAMSM1,                     /*!< Pointer to the COMP params struct
                                                              used for overvoltage protection.
                                                              It must be MC_NULL if internal COMP
                                                              are not used for OVP protection.*/
/* DAC settings --------------------------------------------------------------*/
  .hDAC_OCP_Threshold = OCPREF,                            /*!< Value of analog reference expressed
                                                                as 16bit unsigned integer.
                                                                Ex. 0 = 0V 65536 = VDD_DAC.*/
  .hDAC_OVP_Threshold = OVPREF,                            /*!< Value of analog reference expressed
                                                                as 16bit unsigned integer.
                                                                Ex. 0 = 0V 65536 = VDD_DAC.*/
/* Regular conversion --------------------------------------------------------*/
  .regconvADCx = REGCONVADC,                            /*!< ADC peripheral used for regular
                                                             conversion.*/
};

/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - HALL
  */
HALL_F30XParams_t HALL_F30XParamsM1 =
{
  /* SW Settings */
  MC_IRQ_SPEEDNPOSFDBK_1,   /*!< MC IRQ number used for TIMx capture update event.*/

  HALL_SENSORS_PLACEMENT,
                        /*!< Define here the mechanical position of the sensors
                             with reference to an electrical cycle.
                             Allowed values are: DEGREES_120 or DEGREES_60.*/

  (int16_t)(HALL_PHASE_SHIFT * 65536/360),
                        /*!< Define here in s16degree the electrical phase shift
                             between the low to high transition of signal H1 and
                             the maximum of the Bemf induced on phase A.*/

  MEDIUM_FREQUENCY_TASK_RATE, /*!< Frequency (Hz) at which motor speed is to
                             be computed. It must be equal to the frequency
                             at which function SPD_CalcAvrgMecSpeed01Hz
                             is called.*/

  HALL_AVERAGING_FIFO_DEPTH,  /*!< Size of the buffer used to calculate the average
                             speed. It must be less than 18.*/

  HALL_IC_FILTER,         /*!< Time filter applied to validate HALL sensor capture.
                             This value defines the frequency used to sample
                             HALL sensors input and the length of the digital
                             filter applied. The digital filter is made of an
                             event counter in which N events are needed to
                             validate a transition on the input.
                                0000: No filter, sampling is done at fCK_INT.
                                0001: fSAMPLING=fCK_INT , N=2.
                                0010: fSAMPLING=fCK_INT , N=4.
                                0011: fSAMPLING=fCK_INT , N=8.
                                0100: fSAMPLING=fCK_INT/2, N=6.
                                0101: fSAMPLING=fCK_INT/2, N=8.
                                0110: fSAMPLING=fCK_INT/4, N=6.
                                0111: fSAMPLING=fCK_INT/4, N=8.
                                1000: fSAMPLING=fCK_INT/8, N=6.
                                1001: fSAMPLING=fCK_INT/8, N=8.
                                1010: fSAMPLING=fCK_INT/16, N=5.
                                1011: fSAMPLING=fCK_INT/16, N=6.
                                1100: fSAMPLING=fCK_INT/16, N=8.
                                1101: fSAMPLING=fCK_INT/32, N=5.
                                1110: fSAMPLING=fCK_INT/32, N=6.
                                1111: fSAMPLING=fCK_INT/32, N=8 */

  /* HW Settings */
  HALL_TIM_CLK,          /*!< Timer clock frequency express in Hz.*/

  HALL_TIMER,        /*!< Timer used for HALL sensor management.*/

  HALL_RCC_PERIPHERAL,
                        /*!< RCC Clock related to timer TIMx.
                             (Ex. RCC_APB1Periph_TIM2).*/

  HALL_IRQ_CHANNEL,            /*!< IRQ Channle related to TIMx.
                             (Ex. TIM2_IRQChannel).*/

  HALL_TIMER_REMAPPING,
                        /*!< It remaps TIMx outputs. It must equal to
                             GPIO_PartialRemap_TIMx, GPIO_FullRemap_TIMx or
                             GPIO_NoRemap_TIMx according the HW availability.*/

  H1_GPIO_PORT,
                        /*!< HALL sensor H1 channel GPIO input port (if used,
                             after re-mapping). It must be GPIOx x= A, B, ...*/

  H1_GPIO_PIN,           /*!< HALL sensor H1 channel GPIO output pin (if used,
                             after re-mapping). It must be GPIO_Pin_x x= 0, 1,
                             ...*/

  H2_GPIO_PORT,
                        /*!< HALL sensor H2 channel GPIO input port (if used,
                             after re-mapping). It must be GPIOx x= A, B, ...*/

  H2_GPIO_PIN,           /*!< HALL sensor H2 channel GPIO output pin (if used,
                             after re-mapping). It must be GPIO_Pin_x x= 0, 1,
                             ...*/

  H3_GPIO_PORT,
                        /*!< HALL sensor H3 channel GPIO input port (if used,
                             after re-mapping). It must be GPIOx x= A, B, ...*/

  H3_GPIO_PIN,            /*!< HALL sensor H3 channel GPIO output pin (if used,
                             after re-mapping). It must be GPIO_Pin_x x= 0, 1,
                             ...*/
/* Alternate functions definition --------------------------------------------*/
  H1_GPIO_AF,           /*!< HALL sensor H1 alternate
                             functions setting. It must be one
                             of the GPIO_AF_x (x=0,1, ...)
                             according to the defined GPIO port
                             and pin or MC_NULL if not AF setting
                             is not supported by selected microcontroller.*/
  H2_GPIO_AF,           /*!< HALL sensor H2 alternate
                             functions setting. It must be one
                             of the GPIO_AF_x (x=0,1, ...)
                             according to the defined GPIO port
                             and pin or MC_NULL if not AF setting
                             is not supported by selected microcontroller.*/
  H3_GPIO_AF,           /*!< HALL sensor H3 alternate
                             functions setting. It must be one
                             of the GPIO_AF_x (x=0,1, ...)
                             according to the defined GPIO port
                             and pin or MC_NULL if not AF setting
                             is not supported by selected microcontroller.*/
};

#endif // STM32F30X

/**
  * @brief  Current sensor parameters Single Drive - one shunt
  */
R1_SDParams_t R1_SDParams =
{

  ADC_CLOCK_DIVIDER,                 /*!<  APB2 clock prescaling factor for
                                          ADC peripheral. It must be RCC_PCLK2_DivX
                                          x = 2, 4, 6, 8 */
  TIM_CLOCK_DIVIDER,                 /* System clock divider for Advanded Timer */
  MC_IRQ_PWMNCURRFDBK_1,             /*!< MC IRQ number used for TIM1 Update event
                                       and for DMA TC event.*/
/* Current reading A/D Conversions initialization -----------------------------*/

  PHASE_CURRENTS_CHANNEL,                  /*!< ADC channel used for conversion of
                                           current Ib. It must be equal to
                                           ADC_Channel_x x= 0, ..., 15*/
  PHASE_CURRENTS_GPIO_PORT,                /*!< GPIO port used by hIbChannel. It must
                                           be equal to GPIOx x= A, B, ...*/
  PHASE_CURRENTS_GPIO_PIN,                      /*!< GPIO pin used by hIaChannel. It must be
                                          equal to GPIO_Pin_x x= 0, 1, ...*/
  SAMPLING_TIME_SEL,                /*!< Sampling time used to convert hIbChannel.
                                          It must be equal to ADC_SampleTime_xCycles5
                                          x= 1, 7, ...*/
/* PWM generation parameters --------------------------------------------------*/
  DEAD_TIME_COUNTS,                 /*!< Dead time in number of TIM clock
                                         cycles. If CHxN are enabled, it must
                                         contain the dead time to be generated
                                         by the microcontroller, otherwise it
                                         expresses the maximum dead time
                                         generated by driving network */
  REP_COUNTER,                      /*!< It expresses the number of PWM
                                       periods to be elapsed before compare
                                       registers are updated again. In
                                       particular:
                                       RepetitionCounter= (2* PWM periods) -1*/
  TAFTER,                         /*!< It is the sum of dead time plus rise time
                                       express in number of TIM clocks.*/
  TBEFORE,                            /*!< */
  TMIN,                            /*!< It is the sum of dead time plus rise time
                                       plus sampling time express in numbers of
                                       TIM clocks.*/
  HTMIN,                          /*!< It is the half of hTMin value.*/
  TSAMPLE,                        /*!< It is the sampling time express in
                                       numbers of TIM clocks.*/
  MAX_TRTS,                       /*!< It is the maximum between twice of rise
                                       time express in number of TIM clocks and
                                       twice of sampling time express in numbers
                                       of TIM clocks.*/

/* PWM Driving signals initialization ----------------------------------------*/

  R1_PWM_AUX_TIM,                 /*!< Auxiliary timer used for single shunt ADC
                                       triggering. It should be TIM3 or TIM4.*/
  PWM_TIMER_REMAPPING,            /*!< Used only for instance #1, it
                                       remaps TIM1 outputs. It must equal to
                                       GPIO_PartialRemap_TIM1 or
                                       GPIO_FullRemap_TIM1 or
                                       GPIO_NoRemap_TIM1 */
  PHASE_UH_POLARITY,   	           /*!< Channel 1 (high side) output polarity,
                                           it must be TIM_OCPolarity_High or
                                           TIM_OCPolarity_Low */
  PHASE_UH_GPIO_PORT,                                 /*!< Channel 1 (high side) GPIO output
                                         port (if used, after re-mapping).
                                         It must be GPIOx x= A, B, ...*/
  PHASE_UH_GPIO_PIN,                       /*!< Channel 1 (high side) GPIO output pin
                                          (if used, after re-mapping). It must be
                                          GPIO_Pin_x x= 0, 1, ...*/
  IDLE_UH_POLARITY,              /*!< Channel 1 (high side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCIdleState_Set or
                                            TIM_OCIdleState_Reset*/

  PHASE_VH_POLARITY,               /*!< Channel 2 (high side) output polarity,
                                           it must be TIM_OCPolarity_High or
                                           TIM_OCPolarity_Low */
  PHASE_VH_GPIO_PORT,                               /*!< Channel 2 (high side) GPIO output
                                           port (if used, after re-mapping).
                                           It must be GPIOx x= A, B, ...*/
  PHASE_VH_GPIO_PIN,                       /*!< Channel 2 (high side) GPIO output pin
                                          (if used, after re-mapping). It must be
                                          GPIO_Pin_x x= 0, 1, ...*/
  IDLE_VH_POLARITY,              /*!< Channel 2 (high side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCIdleState_Set or
                                            TIM_OCIdleState_Reset*/

  PHASE_WH_POLARITY,            	 /*!< Channel 3 (high side) output polarity,
                                           it must be TIM_OCPolarity_High or
                                           TIM_OCPolarity_Low */
  PHASE_WH_GPIO_PORT,                               /*!< Channel 3 (high side) GPIO output
                                           port (if used, after re-mapping).
                                           It must be GPIOx x= A, B, ...*/
  PHASE_WH_GPIO_PIN,                        /*!< Channel 3 (high side) GPIO output pin
                                          (if used, after re-mapping). It must be
                                          GPIO_Pin_x x= 0, 1, ...*/
  IDLE_WH_POLARITY,              /*!< Channel 3 (high side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCIdleState_Set or
                                            TIM_OCIdleState_Reset*/

  (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING,  /*!< Low side or enabling signals
                                                generation method are defined
                                                here.*/

  PHASE_UL_POLARITY,               /*!< Channel 1N (low side) output polarity,
                                           it must be TIM_OCNPolarity_High or
                                           TIM_OCNPolarity_Low */
  PHASE_UL_GPIO_PORT,                                 /*!< Channel 1N (low side) GPIO output
                                           port (if used, after re-mapping).
                                           It must be GPIOx x= A, B, ...*/
  PHASE_UL_GPIO_PIN,                       /*!< Channel 1N (low side) GPIO output pin
                                          (if used, after re-mapping). It must be
                                          GPIO_Pin_x x= 0, 1, ...*/
  IDLE_UL_POLARITY,              /*!< Channel 1N (low side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCNIdleState_Set or
                                            TIM_OCNIdleState_Reset*/


  PHASE_VL_POLARITY,                 /*!< Channel 2N (low side) output polarity,
                                           it must be TIM_OCNPolarity_High or
                                           TIM_OCNPolarity_Low */
  PHASE_VL_GPIO_PORT,                                 /*!< Channel 2N (low side) GPIO output
                                           port (if used, after re-mapping).
                                           It must be GPIOx x= A, B, ...*/
  PHASE_VL_GPIO_PIN,                     /*!< Channel 2N (low side) GPIO output pin
                                          (if used, after re-mapping). It must be
                                          GPIO_Pin_x x= 0, 1, ...*/
  IDLE_VL_POLARITY,              /*!< Channel 2N (low side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCNIdleState_Set or
                                            TIM_OCNIdleState_Reset*/

  PHASE_WL_POLARITY,             /*!< Channel 3N (low side) output polarity,
                                           it must be TIM_OCNPolarity_High or
                                           TIM_OCNPolarity_Low */
  PHASE_WL_GPIO_PORT,                               /*!< Channel 3N (low side)  GPIO output
                                           port (if used, after re-mapping).
                                           It must be GPIOx x= A, B, ...*/
  PHASE_WL_GPIO_PIN,                    /*!< Channel 3N (low side)  GPIO output pin
                                          (if used, after re-mapping). It must be
                                          GPIO_Pin_x x= 0, 1, ...*/
  IDLE_WL_POLARITY,              /*!< Channel 3N (low side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCNIdleState_Set or
                                            TIM_OCNIdleState_Reset*/
/* PWM Driving signals initialization ----------------------------------------*/
(FunctionalState)SW_OV_CURRENT_PROT_ENABLING, /*!< It enable/disable the management of
                                            an emergency input instantaneously
                                            stopping PWM generation. It must be
                                            either equal to ENABLE or DISABLE */
  (uint16_t)(OVERCURR_FEEDBACK_POLARITY),/*!< Emergency Stop (BKIN) input polarity,
                                           it must be TIM_BreakPolarity_Low or
                                           TIM_BreakPolarity_High */
  EMERGENCY_STOP_GPIO_PORT,             /*!< Emergency Stop (BKIN) GPIO input
                                           port (if used, after re-mapping).
                                           It must be GPIOx x= A, B, ...*/
  EMERGENCY_STOP_GPIO_PIN,              /*!< Emergency Stop (BKIN) GPIO input pin
                                         (if used, after re-mapping). It must be
                                          GPIO_Pin_x x= 0, 1, ...*/
/* Alternate functions definition --------------------------------------------*/
  .bCh1AF   = PHASE_UH_GPIO_AF,                   /*!< Channel 1 (high side) alternate
                                                        functions setting. It must be one
                                                        of the GPIO_AF_x (x=0,1, ...)
                                                        according to the defined GPIO port
                                                        and pin.*/
  .bCh2AF   = PHASE_VH_GPIO_AF,                   /*!< Channel 2 (high side) alternate
                                                        functions setting. It must be one
                                                        of the GPIO_AF_x (x=0,1, ...)
                                                        according to the defined GPIO port
                                                        and pin.*/
  .bCh3AF   = PHASE_WH_GPIO_AF,                   /*!< Channel 3 (high side) alternate
                                                        functions setting. It must be one
                                                        of the GPIO_AF_x (x=0,1, ...)
                                                        according to the defined GPIO port
                                                        and pin.*/
  .bCh1NAF  = PHASE_UL_GPIO_AF,                   /*!< Channel 1 (low side) alternate
                                                        functions setting. It must be one
                                                        of the GPIO_AF_x (x=0,1, ...)
                                                        according to the defined GPIO port
                                                        and pin.*/
  .bCh2NAF  = PHASE_VL_GPIO_AF,                   /*!< Channel 2 (low side) alternate
                                                        functions setting. It must be one
                                                        of the GPIO_AF_x (x=0,1, ...)
                                                        according to the defined GPIO port
                                                        and pin.*/
  .bCh3NAF  = PHASE_WL_GPIO_AF,                   /*!< Channel 3 (low side) alternate
                                                        functions setting. It must be one
                                                        of the GPIO_AF_x (x=0,1, ...)
                                                        according to the defined GPIO port
                                                        and pin.*/
  .bBKINAF  = BRKIN_GPIO_AF,                      /*!< Emergency Stop (BKIN) alternate
                                                        functions setting. It must be one
                                                        of the GPIO_AF_x (x=0,1, ...)
                                                        according to the defined GPIO port
                                                        and pin.*/
};

/**
  * @brief  Current sensor parameters Dual Drive Motor 1 - one shunt
  */
R1_DDParams_t R1_DDParamsM1 =
{
  .wADC_Clock_Divider         =	ADC_CLOCK_DIVIDER,                /*!<  APB2 clock prescaling factor for
                                                                        ADC peripheral. It must be RCC_PCLK2_DivX
                                                                        x = 2, 4, 6, 8. Only the first instance
                                                                        parameter is used to set the prescaling
                                                                        factor*/
  .bTim_Clock_Divider         =	TIM_CLOCK_DIVIDER,                  /* System clock divider for Advanded Timer */
  .IRQnb                      =	MC_IRQ_PWMNCURRFDBK_1,              /*!< MC IRQ number used for UPDATE event */

/* Instance number -----------------------------------------------------------*/
  .bInstanceNbr             =	1,                                    /*!< Instance number. Necessary to
                                             properly synchronize PWM Timers and
                                             start peripheral clocks*/
  .IsFirstR1DDInstance      =	TRUE,                             /*!< Specifies whether this object is the first
                                                                           R1HD2 instance or not.*/
  .bFreqRatio               =	FREQ_RATIO,                            /*!< It is used in case of dual MC to
                                                                           synchronize TIM1 and TIM8. It has
                                                                           effect only on the second instanced
                                                                           object and must be equal to the
                                                                           ratio between the two PWM frequencies
                                                                           (higher/lower). Supported values are
                                                                           1, 2 or 3 */
  .bIsHigherFreqTim         =	FREQ_RELATION,        			    /*!< When bFreqRatio is greather than 1
                                                                           this param is used to indicate if this
                                                                           instance is the one with the highest
                                                                           frequency. Allowed value are: HIGHER_FREQ
                                                                           or LOWER_FREQ */
  .TIMx                     =	PWM_TIMER_SELECTION,                  /*!< Timer used for PWM generation. It should be
                                              TIM1 or TIM8*/


  /* Current reading A/D Conversions initialization --------------------------*/
  .hIChannel              =	PHASE_CURRENTS_CHANNEL,                /*!< ADC channel used for conversion of
                                                                 current. It must be equal to
                                                                 ADC_Channel_x x= 0, ..., 15*/
  .hIPort                 =	PHASE_CURRENTS_GPIO_PORT,                  /*!< GPIO port used by hIChannel. It must
                                                                 be equal to GPIOx x= A, B, ...*/
  .hIPin                  =	PHASE_CURRENTS_GPIO_PIN,                   /*!< GPIO pin used by hIChannel. It must be
                                                                 equal to GPIO_Pin_x x= 0, 1, ...*/
  .b_ISamplingTime        =	SAMPLING_TIME_SEL,        /*!< Sampling time used to convert hIChannel.
                                       It must be equal to ADC_SampleTime_xCycles5
                                       x= 1, 7, ...*/

/* PWM generation parameters --------------------------------------------------*/
  .hDeadTime                  =	DEAD_TIME_COUNTS,                    /*!< Dead time in number of TIM clock
                                                                          cycles. If CHxN are enabled, it must
                                                                          contain the dead time to be generated
                                                                          by the microcontroller, otherwise it
                                                                          expresses the maximum dead time
                                                                          generated by driving network */
  .bRepetitionCounter         =	REP_COUNTER,                         /*!< It expresses the number of PWM
                                                                          periods to be elapsed before compare
                                                                          registers are updated again. In
                                                                          particular:
                                                                          RepetitionCounter= (2* #PWM periods)-1*/
  .hTafter                    =	TAFTER,                         /*!< It is the sum of dead time plus rise time
                                                                     express in number of TIM clocks.*/
  .hTbefore                   =	TBEFORE,                            /*!< */
  .hTMin                      =	 TMIN,                            /*!< It is the sum of dead time plus rise time
                                                                     plus sampling time express in numbers of
                                                                     TIM clocks.*/

  .hHTMin                     =	HTMIN,                          /*!< It is the half of hTMin value.*/
  .hTSample                   =	TSAMPLE,                        /*!< It is the sampling time express in
                                                                     numbers of TIM clocks.*/
  .hMaxTrTs                   =	MAX_TRTS,                       /*!< It is the maximum between twice of rise
                                       time express in number of TIM clocks and
                                       twice of sampling time express in numbers
                                       of TIM clocks.*/

/* PWM Driving signals initialization ----------------------------------------*/

  .wTIM1Remapping         =	PWM_TIMER_REMAPPING,                        /*!< Used only for instance #1, it
                                                                     remaps TIM1 outputs. It must equal to
                                                                     GPIO_PartialRemap_TIM1 or
                                                                     GPIO_FullRemap_TIM1 or
                                                                     GPIO_NoRemap_TIM1 */
  .hCh1Polarity           =	PHASE_UH_POLARITY,   	              /*!< Channel 1 (high side) output polarity,
                                                                     it must be TIM_OCPolarity_High or
                                                                     TIM_OCPolarity_Low */
  .hCh1Port               =	PHASE_UH_GPIO_PORT,                 /*!< Channel 1 (high side) GPIO output
                                                                    port (if used, after re-mapping).
                                                                    It must be GPIOx x= A, B, ...*/
  .hCh1Pin                =	PHASE_UH_GPIO_PIN,                 /*!< Channel 1 (high side) GPIO output pin
                                                                    (if used, after re-mapping). It must be
                                                                    GPIO_Pin_x x= 0, 1, ...*/
  .hCh1IdleState          =	IDLE_UH_POLARITY,                  /*!< Channel 1 (high side) state (low or high)
                                                                      when TIM peripheral is in Idle state.
                                                                      It must be TIM_OCIdleState_Set or
                                                                      TIM_OCIdleState_Reset*/

  .hCh2Polarity           =	PHASE_VH_POLARITY,                 /*!< Channel 2 (high side) output polarity,
                                                                     it must be TIM_OCPolarity_High or
                                                                     TIM_OCPolarity_Low */
  .hCh2Port               =	PHASE_VH_GPIO_PORT,                  /*!< Channel 2 (high side) GPIO output
                                                                     port (if used, after re-mapping).
                                                                     It must be GPIOx x= A, B, ...*/
  .hCh2Pin                =	PHASE_VH_GPIO_PIN,                   /*!< Channel 2 (high side) GPIO output pin
                                                                    (if used, after re-mapping). It must be
                                                                    GPIO_Pin_x x= 0, 1, ...*/
  .hCh2IdleState          =	IDLE_VH_POLARITY,                    /*!< Channel 2 (high side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCIdleState_Set or
                                            TIM_OCIdleState_Reset*/

  .hCh3Polarity           =	PHASE_WH_POLARITY,            	 /*!< Channel 3 (high side) output polarity,
                                                                     it must be TIM_OCPolarity_High or
                                                                     TIM_OCPolarity_Low */
  .hCh3Port               =	PHASE_WH_GPIO_PORT,                               /*!< Channel 3 (high side) GPIO output
                                                                     port (if used, after re-mapping).
                                                                     It must be GPIOx x= A, B, ...*/
  .hCh3Pin                =	PHASE_WH_GPIO_PIN,                        /*!< Channel 3 (high side) GPIO output pin
                                                                    (if used, after re-mapping). It must be
                                                                    GPIO_Pin_x x= 0, 1, ...*/
  .hCh3IdleState          =	IDLE_WH_POLARITY,              /*!< Channel 3 (high side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCIdleState_Set or
                                            TIM_OCIdleState_Reset*/

  .LowSideOutputs         =	(LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING,  /*!< Low side or enabling signals
                                                generation method are defined
                                                here.*/

  .hCh1NPolarity          =	PHASE_UL_POLARITY,               /*!< Channel 1N (low side) output polarity,
                                                                     it must be TIM_OCNPolarity_High or
                                                                     TIM_OCNPolarity_Low */
  .hCh1NPort              =	PHASE_UL_GPIO_PORT,                                 /*!< Channel 1N (low side) GPIO output
                                                                     port (if used, after re-mapping).
                                                                     It must be GPIOx x= A, B, ...*/
  .hCh1NPin               =	PHASE_UL_GPIO_PIN,                       /*!< Channel 1N (low side) GPIO output pin
                                                                    (if used, after re-mapping). It must be
                                                                    GPIO_Pin_x x= 0, 1, ...*/
  .hCh1NIdleState         =	IDLE_UL_POLARITY,              /*!< Channel 1N (low side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCNIdleState_Set or
                                            TIM_OCNIdleState_Reset*/


  .hCh2NPolarity          =	PHASE_VL_POLARITY,                 /*!< Channel 2N (low side) output polarity,
                                                                     it must be TIM_OCNPolarity_High or
                                                                     TIM_OCNPolarity_Low */
  .hCh2NPort              =	PHASE_VL_GPIO_PORT,                                 /*!< Channel 2N (low side) GPIO output
                                                                     port (if used, after re-mapping).
                                                                     It must be GPIOx x= A, B, ...*/
  .hCh2NPin               =	PHASE_VL_GPIO_PIN,                     /*!< Channel 2N (low side) GPIO output pin
                                                                    (if used, after re-mapping). It must be
                                                                    GPIO_Pin_x x= 0, 1, ...*/
  .hCh2NIdleState         =	IDLE_VL_POLARITY,              /*!< Channel 2N (low side) state (low or high)
                                                                      when TIM peripheral is in Idle state.
                                                                      It must be TIM_OCNIdleState_Set or
                                                                      TIM_OCNIdleState_Reset*/

  .hCh3NPolarity          =	PHASE_WL_POLARITY,             /*!< Channel 3N (low side) output polarity,
                                                                     it must be TIM_OCNPolarity_High or
                                                                     TIM_OCNPolarity_Low */
  .hCh3NPort              =	PHASE_WL_GPIO_PORT,                               /*!< Channel 3N (low side)  GPIO output
                                                                     port (if used, after re-mapping).
                                                                     It must be GPIOx x= A, B, ...*/
  .hCh3NPin               =	PHASE_WL_GPIO_PIN,                    /*!< Channel 3N (low side)  GPIO output pin
                                                                    (if used, after re-mapping). It must be
                                                                    GPIO_Pin_x x= 0, 1, ...*/
  .hCh3NIdleState         =	IDLE_WL_POLARITY,              /*!< Channel 3N (low side) state (low or high)
                                            when TIM peripheral is in Idle state.
                                            It must be TIM_OCNIdleState_Set or
                                            TIM_OCNIdleState_Reset*/
/* PWM Driving signals initialization ----------------------------------------*/
  .EmergencyStop                =	(FunctionalState)SW_OV_CURRENT_PROT_ENABLING, /*!< It enable/disable the management of
                                                                              an emergency input instantaneously
                                                                              stopping PWM generation. It must be
                                                                              either equal to ENABLE or DISABLE */
  .hBKINPolarity                =	(uint16_t)(OVERCURR_FEEDBACK_POLARITY),/*!< Emergency Stop (BKIN) input polarity,
                                                                             it must be TIM_BreakPolarity_Low or
                                                                             TIM_BreakPolarity_High */
  .hBKINPort                    =	EMERGENCY_STOP_GPIO_PORT,             /*!< Emergency Stop (BKIN) GPIO input
                                                                             port (if used, after re-mapping).
                                                                             It must be GPIOx x= A, B, ...*/
  .hBKINPin                     =	EMERGENCY_STOP_GPIO_PIN,              /*!< Emergency Stop (BKIN) GPIO input pin
                                                                               (if used, after re-mapping). It must be
                                                                                GPIO_Pin_x x= 0, 1, ...*/
};

/**
  * @brief  Current sensor parameters Dual Drive Motor 1 - ICS
  */
ICS_DDParams_t ICS_DDParamsM1 = {
  .wADC_Clock_Divider =		ADC_CLOCK_DIVIDER,                /*!< APB2 clock prescaling factor for
                                                                    ADC peripheral. It must be RCC_PCLK2_DivX
                                                                    x = 2, 4, 6, 8 */
  .bTim_Clock_Divider =		TIM_CLOCK_DIVIDER,                  /* System clock divider for Advanded Timer */
/* Dual MC parameters --------------------------------------------------------*/
  .bInstanceNbr =			1,                                 /*!< Instance number with reference to PWMC
                                                                    base class. It is necessary to properly
                                                                    synchronize TIM8 with TIM1 at
                                                                    peripheral initializations */
  .Tw =						MAX_TWAIT,                               /*!< It is used for switching the context
                                                                     in dual MC. It contains biggest delay
                                                                     (expressed in counter ticks) between
                                                                     the counter crest and ADC latest
                                                                     trigger  */
  .bFreqRatio =				FREQ_RATIO,                 /*!< It is used in case of dual MC to
                                                                     synchronize TIM1 and TIM8. It has
                                                                     effect only on the second instanced
                                                                     object and must be equal to the
                                                                     ratio between the two PWM frequencies
                                                                     (higher/lower). Supported values are
                                                                     1, 2 or 3 */
  .bIsHigherFreqTim =		FREQ_RELATION,                           /*!< When bFreqRatio is greather than 1
                                                                    this param is used to indicate if this
                                                                    instance is the one with the highest
                                                                    frequency. Allowed value are: HIGHER_FREQ
                                                                    or LOWER_FREQ */
  .IRQnb = 					MC_IRQ_PWMNCURRFDBK_1,                     /*!< MC IRQ number used as TIMx Update
                                           event */
/* Current reading A/D Conversions initialization -----------------------------*/
  .bIaChannel       =	PHASE_U_CURR_CHANNEL,                  /*!< ADC channel used for conversion of
                                                                 current Ia. It must be equal to
                                                                 ADC_Channel_x x= 0, ..., 15*/
  .hIaPort          =	PHASE_U_GPIO_PORT,                  /*!< GPIO port used by hIaChannel. It must
                                                                 be equal to GPIOx x= A, B, ...*/
  .hIaPin           =	PHASE_U_GPIO_PIN,                      /*!< GPIO pin used by hIaChannel. It must be
                                                                equal to GPIO_Pin_x x= 0, 1, ...*/
  .b_IaSamplingTime =	SAMPLING_TIME_SEL,               /*!< Sampling time used to convert hIaChannel.
                                                                It must be equal to ADC_SampleTime_xCycles5
                                                                x= 1, 7, ...*/
  .bIbChannel       =	PHASE_V_CURR_CHANNEL,                  /*!< ADC channel used for conversion of
                                                                 current Ib. It must be equal to
                                                                 ADC_Channel_x x= 0, ..., 15*/
  .hIbPort          =	PHASE_V_GPIO_PORT,                /*!< GPIO port used by hIbChannel. It must
                                                                 be equal to GPIOx x= A, B, ...*/
  .hIbPin           =	PHASE_V_GPIO_PIN,                      /*!< GPIO pin used by hIaChannel. It must be
                                                                equal to GPIO_Pin_x x= 0, 1, ...*/
  .b_IbSamplingTime =	SAMPLING_TIME_SEL,                /*!< Sampling time used to convert hIbChannel.
                                          It must be equal to ADC_SampleTime_xCycles5
                                          x= 1, 7, ...*/
/* PWM generation parameters --------------------------------------------------*/
  .hDeadTime          =	DEAD_TIME_COUNTS,
                                                                /*!< Dead time in number of TIM clock
                                                                     cycles. If CHxN are enabled, it must
                                                                     contain the dead time to be generated
                                                                     by the microcontroller, otherwise it
                                                                     expresses the maximum dead time
                                                                     generated by driving network */
  .bRepetitionCounter =	REP_COUNTER,                            /*!< It expresses the number of PWM
                                                                     periods to be elapsed before compare
                                                                     registers are updated again. In
                                                                     particular:
                                                                     RepetitionCounter= (2* #PWM periods)-1*/
  .TIMx               =	PWM_TIMER_SELECTION,                    /*!< It contains the pointer to the timer
                                                                     used for PWM generation. */
/* PWM Driving signals initialization ----------------------------------------*/

  .wTIM1Remapping		                  =	PWM_TIMER_REMAPPING,            /*!< Used only for instance #1, it
                                                                                 remaps TIM1 outputs. It must equal to
                                                                                 GPIO_PartialRemap_TIM1 or
                                                                                 GPIO_FullRemap_TIM1 or
                                                                                 GPIO_NoRemap_TIM1 */
  .hCh1Polarity                           =	PHASE_UH_POLARITY,   	           /*!< Channel 1 (high side) output polarity,
                                                                                     it must be TIM_OCPolarity_High or
                                                                                     TIM_OCPolarity_Low */
  .hCh1Port                               =	PHASE_UH_GPIO_PORT,                                 /*!< Channel 1 (high side) GPIO output
                                                                                   port (if used, after re-mapping).
                                                                                   It must be GPIOx x= A, B, ...*/
  .hCh1Pin                                =	PHASE_UH_GPIO_PIN,                       /*!< Channel 1 (high side) GPIO output pin
                                                                                    (if used, after re-mapping). It must be
                                                                                    GPIO_Pin_x x= 0, 1, ...*/
  .hCh1IdleState                          =	IDLE_UH_POLARITY,              /*!< Channel 1 (high side) state (low or high)
                                                                                      when TIM peripheral is in Idle state.
                                                                                      It must be TIM_OCIdleState_Set or
                                                                                      TIM_OCIdleState_Reset*/

  .hCh2Polarity                           =	PHASE_VH_POLARITY,               /*!< Channel 2 (high side) output polarity,
                                                                                     it must be TIM_OCPolarity_High or
                                                                                     TIM_OCPolarity_Low */
  .hCh2Port                               =	PHASE_VH_GPIO_PORT,                               /*!< Channel 2 (high side) GPIO output
                                                                                     port (if used, after re-mapping).
                                                                                     It must be GPIOx x= A, B, ...*/
  .hCh2Pin                                =	PHASE_VH_GPIO_PIN,                       /*!< Channel 2 (high side) GPIO output pin
                                                                                    (if used, after re-mapping). It must be
                                                                                    GPIO_Pin_x x= 0, 1, ...*/
  .hCh2IdleState                          =	IDLE_VH_POLARITY,              /*!< Channel 2 (high side) state (low or high)
                                                                                      when TIM peripheral is in Idle state.
                                                                                      It must be TIM_OCIdleState_Set or
                                                                                      TIM_OCIdleState_Reset*/

  .hCh3Polarity                           =	PHASE_WH_POLARITY,            	 /*!< Channel 3 (high side) output polarity,
                                                                                     it must be TIM_OCPolarity_High or
                                                                                     TIM_OCPolarity_Low */
  .hCh3Port                               =	PHASE_WH_GPIO_PORT,                               /*!< Channel 3 (high side) GPIO output
                                                                                     port (if used, after re-mapping).
                                                                                     It must be GPIOx x= A, B, ...*/
  .hCh3Pin                                =	PHASE_WH_GPIO_PIN,                        /*!< Channel 3 (high side) GPIO output pin
                                                                                    (if used, after re-mapping). It must be
                                                                                    GPIO_Pin_x x= 0, 1, ...*/
  .hCh3IdleState                          =	IDLE_WH_POLARITY,              /*!< Channel 3 (high side) state (low or high)
                                                                                      when TIM peripheral is in Idle state.
                                                                                      It must be TIM_OCIdleState_Set or
                                                                                      TIM_OCIdleState_Reset*/

  .LowSideOutputs 						  =	(LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING,  /*!< Low side or enabling signals
                                                                                          generation method are defined
                                                                                          here.*/

  .hCh1NPolarity                          =	PHASE_UL_POLARITY,               /*!< Channel 1N (low side) output polarity,
                                                                                     it must be TIM_OCNPolarity_High or
                                                                                     TIM_OCNPolarity_Low */
  .hCh1NPort                              =	PHASE_UL_GPIO_PORT,                                 /*!< Channel 1N (low side) GPIO output
                                                                                     port (if used, after re-mapping).
                                                                                     It must be GPIOx x= A, B, ...*/
  .hCh1NPin                               =	PHASE_UL_GPIO_PIN,                       /*!< Channel 1N (low side) GPIO output pin
                                                                                    (if used, after re-mapping). It must be
                                                                                    GPIO_Pin_x x= 0, 1, ...*/
  .hCh1NIdleState                         =	IDLE_UL_POLARITY,              /*!< Channel 1N (low side) state (low or high)
                                                                                      when TIM peripheral is in Idle state.
                                                                                      It must be TIM_OCNIdleState_Set or
                                                                                      TIM_OCNIdleState_Reset*/


  .hCh2NPolarity                          =	PHASE_VL_POLARITY,                 /*!< Channel 2N (low side) output polarity,
                                                                                     it must be TIM_OCNPolarity_High or
                                                                                     TIM_OCNPolarity_Low */
  .hCh2NPort                              =	PHASE_VL_GPIO_PORT,                                 /*!< Channel 2N (low side) GPIO output
                                                                                     port (if used, after re-mapping).
                                                                                     It must be GPIOx x= A, B, ...*/
  .hCh2NPin                               =	PHASE_VL_GPIO_PIN,                     /*!< Channel 2N (low side) GPIO output pin
                                                                                    (if used, after re-mapping). It must be
                                                                                    GPIO_Pin_x x= 0, 1, ...*/
  .hCh2NIdleState                         =	IDLE_VL_POLARITY,              /*!< Channel 2N (low side) state (low or high)
                                                                                      when TIM peripheral is in Idle state.
                                                                                      It must be TIM_OCNIdleState_Set or
                                                                                      TIM_OCNIdleState_Reset*/

  .hCh3NPolarity                          =	PHASE_WL_POLARITY,             /*!< Channel 3N (low side) output polarity,
                                                                                     it must be TIM_OCNPolarity_High or
                                                                                     TIM_OCNPolarity_Low */
  .hCh3NPort                              =	PHASE_WL_GPIO_PORT,                               /*!< Channel 3N (low side)  GPIO output
                                                                                     port (if used, after re-mapping).
                                                                                     It must be GPIOx x= A, B, ...*/
  .hCh3NPin                               =	PHASE_WL_GPIO_PIN,                    /*!< Channel 3N (low side)  GPIO output pin
                                                                                      (if used, after re-mapping). It must be
                                                                                      GPIO_Pin_x x= 0, 1, ...*/
  .hCh3NIdleState                         =	IDLE_WL_POLARITY,              /*!< Channel 3N (low side) state (low or high)
                                                                                when TIM peripheral is in Idle state.
                                                                                It must be TIM_OCNIdleState_Set or
                                                                                TIM_OCNIdleState_Reset*/
/* Emergengy signal initialization ----------------------------------------*/
  .EmergencyStop				=	(FunctionalState)SW_OV_CURRENT_PROT_ENABLING, /*!< It enable/disable the management of
                                                                              an emergency input instantaneously
                                                                              stopping PWM generation. It must be
                                                                              either equal to ENABLE or DISABLE */
  .hBKINPolarity                =	(uint16_t)(OVERCURR_FEEDBACK_POLARITY),/*!< Emergency Stop (BKIN) input polarity,
                                                                             it must be TIM_BreakPolarity_Low or
                                                                             TIM_BreakPolarity_High */
  .hBKINPort                    =	EMERGENCY_STOP_GPIO_PORT,             /*!< Emergency Stop (BKIN) GPIO input
                                                                             port (if used, after re-mapping).
                                                                             It must be GPIOx x= A, B, ...*/
  .hBKINPin                     =	EMERGENCY_STOP_GPIO_PIN              /*!< Emergency Stop (BKIN) GPIO input pin
                                                                              (if used, after re-mapping). It must be
                                                                              GPIO_Pin_x x= 0, 1, ...*/
};

/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - Base Class
  */
SpeednPosFdbkParams_t SpeednPosFdbkParamsM1 =
{
  .bElToMecRatio                     =	POLE_PAIR_NUM,                /*!< Coefficient used to transform electrical to
                                                                           mechanical quantities and viceversa. It usually
                                                                           coincides with motor pole pairs number*/
  .hMaxReliableMecSpeed01Hz          =	(uint16_t)(1.15*MAX_APPLICATION_SPEED/6), /*< Maximum value of measured speed that is
                                                                                      considered to be valid. It's expressed
                                                                                      in tenth of mechanical Hertz.*/
  .hMinReliableMecSpeed01Hz          =	(uint16_t)(MIN_APPLICATION_SPEED/6),/*< Minimum value of measured speed that is
                                                                                considered to be valid. It's expressed
                                                                                in tenth of mechanical Hertz.*/
  .bMaximumSpeedErrorsNumber         =	MEAS_ERRORS_BEFORE_FAULTS,            /*< Maximum value of not valid measurements
                                                                                  before an error is reported.*/
  .hMaxReliableMecAccel01HzP         =	65535,                                 /*< Maximum value of measured acceleration
                                                                                   that is considered to be valid. It's
                                                                                   expressed in 01HzP (tenth of Hertz per
                                                                                   speed calculation period)*/
  .hMeasurementFrequency             =	TF_REGULATION_RATE,                    /*< Frequency on which the user will request
                                                                                   a measurement of the rotor electrical angle.
                                                                                   It's also used to convert measured speed from
                                                                                   tenth of Hz to dpp and viceversa.*/
};

/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - State Observer + PLL
  */
STOParams_t STOParamsM1 =
{
 .hC1                         =	C1,                                /*!< State observer constant C1, it can
                                                                           be computed as F1 * Rs(ohm)/(Ls[H] *
                                                                           State observer execution rate [Hz])*/
 .hC2                         =	C2,                               /*!<  Variable containing state observer
                                                                           constant C2, it can be computed as
                                                                           F1 * K1/ State observer execution
                                                                           rate [Hz] being K1 one of the two
                                                                           observer gains   */
 .hC3                         =	C3,                                /*!< State observer constant C3, it can
                                                                           be computed as F1 * Max application
                                                                           speed [rpm] * Motor B-emf constant
                                                                           [Vllrms/krpm] * sqrt(2)/ (Ls [H] *
                                                                           max measurable current (A) * State
                                                                           observer execution rate [Hz])*/
 .hC4                         =	C4,                                /*!< State Observer constant C4, it can
                                                                           be computed as K2 * max measurable
                                                                           current (A) / (Max application speed
                                                                           [rpm] * Motor B-emf constant
                                                                           [Vllrms/krpm] * sqrt(2) * F2 * State
                                                                           observer execution rate [Hz]) being
                                                                           K2 one of the two observer gains  */
 .hC5                         =	C5,                                /*!< State Observer constant C5, it can
                                                                           be computed as F1 * max measurable
                                                                           voltage / (2 * Ls [Hz] * max
                                                                           measurable current * State observer
                                                                           execution rate [Hz]) */
 .hF1                         =	F1,                                /*!< State observer scaling factor F1 */
 .hF2                         =	F2,                                /*!< State observer scaling factor F2 */
    {
     PLL_KP_GAIN,               /*!< Default Kp gain, used to initialize Kp gain
                                     software variable*/
     PLL_KI_GAIN,               /*!< Default Ki gain, used to initialize Ki gain
                                     software variable*/
     PLL_KPDIV,                       /*!< Kp gain divisor, used in conjuction with
                                     Kp gain allows obtaining fractional values.
                                     If FULL_MISRA_C_COMPLIANCY is not defined
                                     the divisor is implemented through
                                     algebrical right shifts to speed up PI
                                     execution. Only in this case this parameter
                                     specifies the number of right shifts to be
                                     executed */
     PLL_KIDIV,                     /*!< Ki gain divisor, used in conjuction with
                                     Ki gain allows obtaining fractional values.
                                     If FULL_MISRA_C_COMPLIANCY is not defined
                                     the divisor is implemented through
                                     algebrical right shifts to speed up PI
                                     execution. Only in this case this parameter
                                     specifies the number of right shifts to be
                                     executed */
     S32_MAX,                   /*!< Upper limit used to saturate the integral
                                     term given by Ki / Ki divisor * integral of
                                     process variable error */
     -S32_MAX,                  /*!< Lower limit used to saturate the integral
                                     term given by Ki / Ki divisor * integral of
                                     process variable error */
     S16_MAX,                   /*!< Upper limit used to saturate the PI output
                                     */
     -S16_MAX,                  /*!< Lower limit used to saturate the PI output
                                     */
     PLL_KPDIV_LOG,             /*!< Kp gain divisor expressed as power of 2.
                                     E.g. if gain divisor is 512 the value
                                     must be 9 because 2^9 = 512 */
     PLL_KIDIV_LOG              /*!< Ki gain divisor expressed as power of 2.
                                     E.g. if gain divisor is 512 the value
                                     must be 9 because 2^9 = 512 */
   },      			/*!< It contains the parameters of the PI
                                     object necessary for PLL
                                     implementation */

 .bSpeedBufferSize01Hz                =	STO_FIFO_DEPTH_01HZ,              /*!< Depth of FIFO used to average
                                                                                    estimated speed exported by
                                                                                    SPD_GetAvrgMecSpeed01Hz. It
                                                                                    must be an integer number between 1
                                                                                    and 64 */
 .bSpeedBufferSizedpp                 =	STO_FIFO_DEPTH_DPP,               /*!< Depth of FIFO used for both averaging
                                                                                    estimated speed exported by
                                                                                    SPD_GetElSpeedDpp and state
                                                                                    observer equations. It must be an
                                                                                    integer number between 1 and
                                                                                    bSpeedBufferSize01Hz */
 .hVariancePercentage                 =	PERCENTAGE_FACTOR,                /*!< Parameter expressing the maximum
                                                                                    allowed variance of speed estimation
                                                                                    */
 .bSpeedValidationBand_H              =	SPEED_BAND_UPPER_LIMIT,           /*!< It expresses how much estimated speed
                                                                                    can exceed forced stator electrical
                                                                                    frequency during start-up without
                                                                                    being considered wrong. The
                                                                                    measurement unit is 1/16 of forced
                                                                                    speed */
 .bSpeedValidationBand_L              =	SPEED_BAND_LOWER_LIMIT,             /*!< It expresses how much estimated speed
                                                                                    can be below forced stator electrical
                                                                                    frequency during start-up without
                                                                                    being considered wrong. The
                                                                                    measurement unit is 1/16 of forced
                                                                                    speed */
 .hMinStartUpValidSpeed               =	OBS_MINIMUM_SPEED,                  /*!< Minimum mechanical speed (expressed in
                                                                                   01Hz required to validate the start-up
                                                                                   */
 .bStartUpConsistThreshold            =	NB_CONSECUTIVE_TESTS,  	       /*!< Number of consecutive tests on speed
                                                                                    consistency to be passed before
                                                                                    validating the start-up */
 .bReliability_hysteresys             =	OBS_MEAS_ERRORS_BEFORE_FAULTS,        /*!< Number of reliability failed
                                                                                    consecutive tests before a speed
                                                                                    check fault is returned to base class
                                                                                    */
 .bBemfConsistencyCheck               =	BEMF_CONSISTENCY_TOL,                    /*!< Degree of consistency of the observed
                                                                                    back-emfs, it must be an integer
                                                                                    number ranging from 1 (very high
                                                                                    consistency) down to 64 (very poor
                                                                                    consistency) */
 .bBemfConsistencyGain                =	BEMF_CONSISTENCY_GAIN,               /*!< Gain to be applied when checking
                                                                                    back-emfs consistency; default value
                                                                                    is 64 (neutral), max value 105
                                                                                    (x1,64 amplification), min value 1
                                                                                    (/64 attenuation) */
 .hMaxAppPositiveMecSpeed01Hz         =	(uint16_t)(MAX_APPLICATION_SPEED*1.15/6.0),  /*!< Maximum positive value
                                                                                      of rotor speed. It's expressed in
                                                                                      tenth of mechanical Hertz. It can be
                                                                                      x1.1 greater than max application*/

 .hF1LOG                              =	F1_LOG,                             /*!< F1 gain divisor expressed as power of 2.
                                                                                     E.g. if gain divisor is 512 the value
                                                                                     must be 9 because 2^9 = 512 */
 .hF2LOG                              =	F2_LOG,                             /*!< F2 gain divisor expressed as power of 2.
                                                                                     E.g. if gain divisor is 512 the value
                                                                                     must be 9 because 2^9 = 512 */
 .bSpeedBufferSizedppLOG              =	STO_FIFO_DEPTH_DPP_LOG              /*!< bSpeedBufferSizedpp expressed as power of 2.
                                                                                 E.g. if gain divisor is 512 the value
                                                                                 must be 9 because 2^9 = 512 */
};

/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - State Observer + CORDIC
  */
STO_CORDICParams_t STO_CORDICParamsM1 =
{
  .hC1                                 =	CORD_C1,                                /*!< State observer constant C1, it can
                                                                                    be computed as F1 * Rs(ohm)/(Ls[H] *
                                                                                    State observer execution rate [Hz])*/
  .hC2                                 =	CORD_C2,                          /*!<  Variable containing state observer
                                                                                    constant C2, it can be computed as
                                                                                    F1 * K1/ State observer execution
                                                                                    rate [Hz] being K1 one of the two
                                                                                    observer gains   */
  .hC3                                 =	CORD_C3,                                /*!< State observer constant C3, it can
                                                                                    be computed as F1 * Max application
                                                                                    speed [rpm] * Motor B-emf constant
                                                                                    [Vllrms/krpm] * sqrt(2)/ (Ls [H] *
                                                                                    max measurable current (A) * State
                                                                                    observer execution rate [Hz])*/
  .hC4                                 =	CORD_C4,                           /*!< State Observer constant C4, it can
                                                                                    be computed as K2 * max measurable
                                                                                    current (A) / (Max application speed
                                                                                    [rpm] * Motor B-emf constant
                                                                                    [Vllrms/krpm] * sqrt(2) * F2 * State
                                                                                    observer execution rate [Hz]) being
                                                                                    K2 one of the two observer gains  */
  .hC5                                 =	CORD_C5,                                /*!< State Observer constant C5, it can
                                                                                    be computed as F1 * max measurable
                                                                                    voltage / (2 * Ls [Hz] * max
                                                                                    measurable current * State observer
                                                                                    execution rate [Hz]) */
  .hF1                                 =	CORD_F1,                                /*!< State observer scaling factor F1 */
  .hF2                                 =	CORD_F2,                                /*!< State observer scaling factor F2 */
  .bSpeedBufferSize01Hz                =	CORD_FIFO_DEPTH_01HZ,              /*!< Depth of FIFO used to average
                                                                                    estimated speed exported by
                                                                                    SPD_GetAvrgMecSpeed01Hz. It
                                                                                    must be an integer number between 1
                                                                                    and 64 */
  .bSpeedBufferSizedpp                 =	CORD_FIFO_DEPTH_DPP,               /*!< Depth of FIFO used for both averaging
                                                                                    estimated speed exported by
                                                                                    SPD_GetElSpeedDpp and state
                                                                                    observer equations. It must be an
                                                                                    integer number between 1 and
                                                                                    bSpeedBufferSize01Hz */
  .hVariancePercentage                 =	CORD_PERCENTAGE_FACTOR,                /*!< Parameter expressing the maximum
                                                                                    allowed variance of speed estimation
                                                                                    */
  .bSpeedValidationBand_H              =	SPEED_BAND_UPPER_LIMIT,           /*!< It expresses how much estimated speed
                                                                                    can exceed forced stator electrical
                                                                                    frequency during start-up without
                                                                                    being considered wrong. The
                                                                                    measurement unit is 1/16 of forced
                                                                                    speed */
  .bSpeedValidationBand_L              =	SPEED_BAND_LOWER_LIMIT,            /*!< It expresses how much estimated speed
                                                                                    can be below forced stator electrical
                                                                                    frequency during start-up without
                                                                                    being considered wrong. The
                                                                                    measurement unit is 1/16 of forced
                                                                                    speed */
  .hMinStartUpValidSpeed               =	OBS_MINIMUM_SPEED,                  /*!< Minimum mechanical speed (expressed in
                                                                                    01Hz required to validate the start-up
                                                                                    */
  .bStartUpConsistThreshold            =	NB_CONSECUTIVE_TESTS,  		      /*!< Number of consecutive tests on speed
                                                                                    consistency to be passed before
                                                                                    validating the start-up */
  .bReliability_hysteresys             =	CORD_MEAS_ERRORS_BEFORE_FAULTS,      /*!< Number of reliability failed
                                                                                    consecutive tests before a speed
                                                                                    check fault is returned to base class
                                                                                    */
  .hMaxInstantElAcceleration           =	CORD_MAX_ACCEL_DPPP,                 /*!< maximum instantaneous electrical
                                                                                  acceleration (dpp per control period) */
  .bBemfConsistencyCheck               =	CORD_BEMF_CONSISTENCY_TOL,                    /*!< Degree of consistency of the observed
                                                                                    back-emfs, it must be an integer
                                                                                    number ranging from 1 (very high
                                                                                    consistency) down to 64 (very poor
                                                                                    consistency) */
  .bBemfConsistencyGain                =	CORD_BEMF_CONSISTENCY_GAIN,               /*!< Gain to be applied when checking
                                                                                    back-emfs consistency; default value
                                                                                    is 64 (neutral), max value 105
                                                                                    (x1,64 amplification), min value 1
                                                                                    (/64 attenuation) */
  .hMaxAppPositiveMecSpeed01Hz         =	(uint16_t)(MAX_APPLICATION_SPEED*1.15/6.0),  /*!< Maximum positive value
                                                                                      of rotor speed. It's expressed in
                                                                                      tenth of mechanical Hertz. It can be
                                                                                      x1.1 greater than max application*/

  .hF1LOG                              =	F1_LOG,                             /*!< F1 gain divisor expressed as power of 2.
                                                                                     E.g. if gain divisor is 512 the value
                                                                                     must be 9 because 2^9 = 512 */
  .hF2LOG                              =	F2_LOG,                             /*!< F2 gain divisor expressed as power of 2.
                                                                                     E.g. if gain divisor is 512 the value
                                                                                     must be 9 because 2^9 = 512 */
  .bSpeedBufferSizedppLOG              =	CORD_FIFO_DEPTH_DPP_LOG             /*!< bSpeedBufferSizedpp expressed as power of 2.
                                                                                     E.g. if gain divisor is 512 the value
                                                                                     must be 9 because 2^9 = 512 */
};

/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - HFI
  */
HiFreqInj_FPUParams_t HiFreqInjParamsM1 =
{
  .bSpeedBufferSize01Hz =	HFI_SPD_BUFFER_DEPTH_01HZ,
  .hSwapMecSpeed01Hz    =	RPM2MEC01HZ(HFI_MINIMUM_SPEED_RPM),
  .hSTOHFI01Hz          =	RPM2MEC01HZ(STO_HFI_RPM_TH),
  .hHFISTO01Hz          =	RPM2MEC01HZ(HFI_STO_RPM_TH),
  .hHFIRestart01Hz      =	RPM2MEC01HZ(HFI_RESTART_RPM_TH)
};

/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - Virtual Sensor
  */
VirtualSpeedSensorParams_t VirtualSpeedSensorParamsM1 =
{
  .hSpeedSamplingFreqHz =	MEDIUM_FREQUENCY_TASK_RATE,      /*!< Frequency (Hz) at which motor speed is to
                                                                  be computed. It must be equal to the frequency
                                                                  at which function SPD_CalcAvrgMecSpeed01Hz
                                                                  is called.*/
  .hTransitionSteps     =	(int16_t)(TF_REGULATION_RATE * TRANSITION_DURATION/ 1000.0),
                             /*!< Number of steps to perform
                                  the transition phase from CVSS_SPD to other CSPD;
                                  if the Transition PHase should last TPH milliseconds,
                                  and the FOC Execution Frequency is set to FEF kHz,
                                  then hTransitionSteps = TPH * FEF. If set to zero,
                                  it disables soft transition. It's recommended to
                                  set TPH as multiple of medium frequency task period
                              */
};

/**
  * @brief  RevUp controller parameters Motor 1
  */
RUCPhasesParams_t Phase5_M1 =
{
  .hDurationms        =	(uint16_t)PHASE5_DURATION,        /*!< Duration of the rev up phase expressed in
                                                               milliseconds.*/
  .hFinalMecSpeed01Hz =	(int16_t)(PHASE5_FINAL_SPEED_RPM/6),/*!< Mechanical speed expressed in 0.1Hz assumed
                                                                 by VSS at the end of the rev up phase.*/
  .hFinalTorque       =	(int16_t)PHASE5_FINAL_CURRENT,     /*!< Motor torque reference imposed by STC at the
                                                                end of rev up phase. This value represents
                                                                actually the Iq current expressed in digit.*/
  .pNext              =	MC_NULL
};


RUCPhasesParams_t Phase4_M1 =
{
  .hDurationms        =	(uint16_t)PHASE4_DURATION,       /*!< Duration of the rev up phase expressed in
                                                              milliseconds.*/
  .hFinalMecSpeed01Hz =	(int16_t)(PHASE4_FINAL_SPEED_RPM/6),/*!< Mechanical speed expressed in 0.1Hz assumed
                                                                 by VSS at the end of the rev up phase.*/
  .hFinalTorque       =	(int16_t)PHASE4_FINAL_CURRENT,   /*!< Motor torque reference imposed by STC at the
                                                              end of rev up phase. This value represents
                                                              actually the Iq current expressed in digit.*/
  .pNext              =	(void*)&Phase5_M1
};

RUCPhasesParams_t Phase3_M1 =
{
  .hDurationms        =	(uint16_t)PHASE3_DURATION,       /*!< Duration of the rev up phase expressed in
                                                              milliseconds.*/
  .hFinalMecSpeed01Hz =	(int16_t)(PHASE3_FINAL_SPEED_RPM/6), /*!< Mechanical speed expressed in 0.1Hz assumed
                                                                  by VSS at the end of the rev up phase.*/
  .hFinalTorque       =	(int16_t)PHASE3_FINAL_CURRENT, /*!< Motor torque reference imposed by STC at the
                                                            end of rev up phase. This value represents
                                                            actually the Iq current expressed in digit.*/
  .pNext              =	(void*)&Phase4_M1
};

RUCPhasesParams_t Phase2_M1 =
{
  .hDurationms        =	(uint16_t)PHASE2_DURATION,      /*!< Duration of the rev up phase expressed in
                                                             milliseconds.*/
  .hFinalMecSpeed01Hz =	(int16_t)(PHASE2_FINAL_SPEED_RPM/6),/*!< Mechanical speed expressed in 0.1Hz assumed
                                                                 by VSS at the end of the rev up phase.*/
  .hFinalTorque       =	(int16_t)PHASE2_FINAL_CURRENT,    /*!< Motor torque reference imposed by STC at the
                                                               end of rev up phase. This value represents
                                                               actually the Iq current expressed in digit.*/
  .pNext              =	(void*)&Phase3_M1
};

RUCPhasesParams_t Phase1_M1 =
{
  .hDurationms        =	(uint16_t)PHASE1_DURATION,     /*!< Duration of the rev up phase expressed in
                                                            milliseconds.*/
  .hFinalMecSpeed01Hz =	(int16_t)(PHASE1_FINAL_SPEED_RPM/6),/*!< Mechanical speed expressed in 0.1Hz assumed
                                                                 by VSS at the end of the rev up phase.*/
  .hFinalTorque       =	(int16_t)PHASE1_FINAL_CURRENT,  /*!< Motor torque reference imposed by STC at the
                                                             end of rev up phase. This value represents
                                                             actually the Iq current expressed in digit.*/
  .pNext              =	(void*)&Phase2_M1
};

RevupCtrlParams_t RevupCtrlParamsM1 =
{
  .hRUCFrequencyHz         =	MEDIUM_FREQUENCY_TASK_RATE,         /*!< Frequency expressed in Hz at which the user
                                                                         clocks the RUC calling RUC_Exec method */
  .hStartingMecAngle       =	(int16_t)((int32_t)(STARTING_ANGLE_DEG)* 65536/360), /*!< Starting angle of programmed rev up.*/
  .pPhaseParam             =	&Phase1_M1,                /*!< Pointer of the first element of
                                                                phase params that constitute the programmed
                                                                rev up. Each element is of type
                                                                RUCPhasesParams_t.*/
  .bFirstAccelerationStage =	FIRST_SLESS_ALGO_PHASE,    /*< Indicate here in which of the rev-up phases
                                                               the final acceleration is started. The
                                                               sensor-less algorithm is re-initialized
                                                               (cleared) when this phase begins.
                                                                NOTE: The rev up phases counting starts from 0.
                                                                      First phase should be indicated with zero */
  .hMinStartUpValidSpeed   =	OBS_MINIMUM_SPEED,                  /*!< Minimum mechanical speed (expressed in
                                                                         01Hz required to validate the start-up */
  .hMinStartUpFlySpeed     =	(int16_t)(OBS_MINIMUM_SPEED/2),     /*!< Minimum mechanical speed (expressed in
                                                                         01Hz required to validate the on the fly*/
  .bOTFvalidation          =	NB_CONSECUTIVE_TESTS,          /*!< Minimum number of consecutive times the observer
                                                                    is reliable to validate OTF startup*/
  .hFinalRevUpCurrent      =	(int16_t)PHASE5_FINAL_CURRENT, /*!<  final revup current */
  .hOTFSection1Duration    =	(uint16_t) 500,                /*!<  on-the-fly section1 duration, millisecond */
  .OTFStartupEnabled       =	OTF_STARTUP_EN                 /*!< ENABLE for OTF strartup otherwise FALSE.*/
};

/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - Encoder
  */
ENCODERParams_t ENCParamsM1 =
{
  /* SW Settings */
  .IRQnb                  =	MC_IRQ_SPEEDNPOSFDBK_1, /*!< MC IRQ number used for TIMx capture update event.*/

  .hPulseNumber           =	ENCODER_PPR*4,		  /*!< Number of pulses per revolution, provided by each
                                                       of the two encoder signals, multiplied by 4 */
  .RevertSignal           =	(FunctionalState)ENC_INVERT_SPEED, /*!< To be enabled if measured speed is opposite
                                                                    to real one (ENABLE/DISABLE)*/

  .hSpeedSamplingFreq01Hz =	10*MEDIUM_FREQUENCY_TASK_RATE, /*!< Frequency (01Hz) at which motor speed is to be
                                                                computed. It must be equal to the frequency
                                                                at which function SPD_CalcAvrgMecSpeed01Hz
                                                                is called.*/

  .bSpeedBufferSize       =	ENC_AVERAGING_FIFO_DEPTH,  /*!< Size of the buffer used to calculate the average
                                                            speed. It must be <= 16.*/

  .hInpCaptFilter         =	ENC_IC_FILTER,	   /*!< Time filter applied to validate ENCODER sensor
                                                    capture. This value defines the frequency
                                                    used to sample ENCODER sensors input and the
                                                    length of the digital filter applied.
                                                    The digital filter is made of an event
                                                    counter in which N events are needed to
                                                    validate a transition on the input.
                                                    0000: No filter, sampling is done at fCK_INT.
                                                    0001: fSAMPLING=fCK_INT , N=2.
                                                    0010: fSAMPLING=fCK_INT , N=4.
                                                    0011: fSAMPLING=fCK_INT , N=8.
                                                    0100: fSAMPLING=fCK_INT/2, N=6.
                                                    0101: fSAMPLING=fCK_INT/2, N=8.
                                                    0110: fSAMPLING=fCK_INT/4, N=6.
                                                    0111: fSAMPLING=fCK_INT/4, N=8.
                                                    1000: fSAMPLING=fCK_INT/8, N=6.
                                                    1001: fSAMPLING=fCK_INT/8, N=8.
                                                    1010: fSAMPLING=fCK_INT/16, N=5.
                                                    1011: fSAMPLING=fCK_INT/16, N=6.
                                                    1100: fSAMPLING=fCK_INT/16, N=8.
                                                    1101: fSAMPLING=fCK_INT/32, N=5.
                                                    1110: fSAMPLING=fCK_INT/32, N=6.
                                                    1111: fSAMPLING=fCK_INT/32, N=8 */
  /* HW Settings */
  .TIMx                =	ENC_TIMER,	    /*!< Timer used for ENCODER sensor management.*/

  .RCC_APB1Periph_TIMx =	ENC_RCC_PERIPHERAL,        /*!< RCC Clock related to timer TIMx.
                                                           (Ex. RCC_APB1Periph_TIM2).*/

  .TIMx_IRQChannel     =	ENC_IRQ_CHANNEL,        /*!< IRQ Channel related to TIMx.
                                                         (Ex. TIM2_IRQChannel).*/

  .wTIMxRemapping      =	ENC_TIMER_REMAPPING,		 /*!< It remaps TIMx outputs. It must equal to
                                                              GPIO_PartialRemap_TIMx, GPIO_FullRemap_TIMx or
                                                              GPIO_NoRemap_TIMx according the HW availability*/

  .hAPort              =	ENC_A_GPIO_PORT,        /*!< ENCODER sensor A channel GPIO input port (if used,
                                                         after re-mapping). It must be GPIOx x= A, B, ...*/

  .hAPin               =	ENC_A_GPIO_PIN,       /*!< ENCODER sensor A channel GPIO input pin (if used,
                                                       after re-mapping). It must be GPIO_Pin_x x=0,1,.*/

  .hBPort              =	ENC_B_GPIO_PORT,	 /*!< ENCODER sensor B channel GPIO input port (if used,
                                                      after re-mapping). It must be GPIOx x= A, B, ...*/

  .hBPin               =	ENC_B_GPIO_PIN,       /*!< ENCODER sensor B channel GPIO input pin (if used,
                                                       after re-mapping). It must be GPIO_Pin_x x= 0,1,.*/

};

/**
  * @brief  Encoder Alignment Controller parameters Motor 1
  */
EncAlignCtrlParams_t EncAlignCtrlParamsM1 =
{
  .hEACFrequencyHz =	MEDIUM_FREQUENCY_TASK_RATE, /*!< Frequency
                                                       expressed in Hz at which the user
                                                       clocks the EAC calling EAC_Exec method */
  .hFinalTorque    =	FINAL_I_ALIGNMENT,              /*!< Motor torque reference imposed by STC at the
                                                       end of programmed alignment. This value
                                                       represents actually the Iq current expressed in
                                                       digit.*/
  .hElAngle        =	ALIGNMENT_ANGLE_S16,      /*!< Mechanical angle of programmed alignment
                                                       expressed in s16degrees.*/
  .hDurationms     =	T_ALIGNMENT,              /*!< Duration of the programmed alignment expressed
                                                       in milliseconds.*/
  .bElToMecRatio   =	POLE_PAIR_NUM,            /*!< Coefficient used to transform electrical to
                                 mechanical quantities and viceversa. It usually
                                 coincides with motor pole pairs number*/
};

/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - HALL
  */
HALLParams_t HALLParamsM1 =
{
  /* SW Settings */
  .IRQnb                = MC_IRQ_SPEEDNPOSFDBK_1,   /*!< MC IRQ number used for TIMx capture update event.*/

  .bSensorPlacement     = HALL_SENSORS_PLACEMENT,
                                                /*!< Define here the mechanical position of the sensors
                                                     with reference to an electrical cycle.
                                                     Allowed values are: DEGREES_120 or DEGREES_60.*/
  .hPhaseShift          = (int16_t)(HALL_PHASE_SHIFT * 65536/360),
                                                /*!< Define here in s16degree the electrical phase shift
                                                     between the low to high transition of signal H1 and
                                                     the maximum of the Bemf induced on phase A.*/

  .hSpeedSamplingFreqHz = MEDIUM_FREQUENCY_TASK_RATE, /*!< Frequency (Hz) at which motor speed is to
                                                           be computed. It must be equal to the frequency
                                                           at which function SPD_CalcAvrgMecSpeed01Hz
                                                           is called.*/

  .bSpeedBufferSize     = HALL_AVERAGING_FIFO_DEPTH,  /*!< Size of the buffer used to calculate the average
                                                           speed. It must be less than 18.*/

  .hInpCaptFilter       = HALL_IC_FILTER,         /*!< Time filter applied to validate HALL sensor capture.
                                                       This value defines the frequency used to sample
                                                       HALL sensors input and the length of the digital
                                                       filter applied. The digital filter is made of an
                                                       event counter in which N events are needed to
                                                       validate a transition on the input.
                                                          0000: No filter, sampling is done at fCK_INT.
                                                          0001: fSAMPLING=fCK_INT , N=2.
                                                          0010: fSAMPLING=fCK_INT , N=4.
                                                          0011: fSAMPLING=fCK_INT , N=8.
                                                          0100: fSAMPLING=fCK_INT/2, N=6.
                                                          0101: fSAMPLING=fCK_INT/2, N=8.
                                                          0110: fSAMPLING=fCK_INT/4, N=6.
                                                          0111: fSAMPLING=fCK_INT/4, N=8.
                                                          1000: fSAMPLING=fCK_INT/8, N=6.
                                                          1001: fSAMPLING=fCK_INT/8, N=8.
                                                          1010: fSAMPLING=fCK_INT/16, N=5.
                                                          1011: fSAMPLING=fCK_INT/16, N=6.
                                                          1100: fSAMPLING=fCK_INT/16, N=8.
                                                          1101: fSAMPLING=fCK_INT/32, N=5.
                                                          1110: fSAMPLING=fCK_INT/32, N=6.
                                                          1111: fSAMPLING=fCK_INT/32, N=8 */

  /* HW Settings */
 .wTIMClockFreq       = HALL_TIM_CLK,          /*!< Timer clock frequency express in Hz.*/

 .TIMx                = HALL_TIMER,        /*!< Timer used for HALL sensor management.*/

 .RCC_APB1Periph_TIMx = HALL_RCC_PERIPHERAL,
                                              /*!< RCC Clock related to timer TIMx.
                                                   (Ex. RCC_APB1Periph_TIM2).*/

 .TIMx_IRQChannel     = HALL_IRQ_CHANNEL,            /*!< IRQ Channle related to TIMx.
                                                          (Ex. TIM2_IRQChannel).*/


 .wTIMxRemapping      = HALL_TIMER_REMAPPING,
                                              /*!< It remaps TIMx outputs. It must equal to
                                                   GPIO_PartialRemap_TIMx, GPIO_FullRemap_TIMx or
                                                   GPIO_NoRemap_TIMx according the HW availability.*/

 .hH1Port             = H1_GPIO_PORT,
                                              /*!< HALL sensor H1 channel GPIO input port (if used,
                                                   after re-mapping). It must be GPIOx x= A, B, ...*/

 .hH1Pin              = H1_GPIO_PIN,           /*!< HALL sensor H1 channel GPIO output pin (if used,
                                                    after re-mapping). It must be GPIO_Pin_x x= 0, 1,
                                                    ...*/

 .hH2Port             = H2_GPIO_PORT,
                                              /*!< HALL sensor H2 channel GPIO input port (if used,
                                                   after re-mapping). It must be GPIOx x= A, B, ...*/

 .hH2Pin              = H2_GPIO_PIN,           /*!< HALL sensor H2 channel GPIO output pin (if used,
                                                    after re-mapping). It must be GPIO_Pin_x x= 0, 1,
                                                    ...*/

 .hH3Port             = H3_GPIO_PORT,
                                              /*!< HALL sensor H3 channel GPIO input port (if used,
                                                   after re-mapping). It must be GPIOx x= A, B, ...*/

 .hH3Pin              = H3_GPIO_PIN,            /*!< HALL sensor H3 channel GPIO output pin (if used,
                                                     after re-mapping). It must be GPIO_Pin_x x= 0, 1,
                                                     ...*/
};
/**
  * @brief  Real Bus Voltage sensor parameters Motor 1 - Base class
  */
BusVoltageSensorParams_t RealBusVoltageSensorParamsM1 =
{
  .bSensorType       = REAL_SENSOR,                   /*!< It contains the information about the type
                                                           of instanced bus voltage sensor object.
                                                           It can be equal to REAL_SENSOR or
                                                           VIRTUAL_SENSOR */
  .hConversionFactor = (uint16_t)(MCU_SUPPLY_VOLTAGE/BUS_ADC_CONV_RATIO),  /*!< It is used to convert bus voltage from
                                                                 u16Volts into real Volts (V).
                                                                 1 u16Volt = 65536/hConversionFactor Volts
                                                                 For real sensors hConversionFactor it's
                                                                 equal to the product between the expected MCU
                                                                 voltage and the partioning network
                                                                 attenuation. For virtual sensors it is
                                                                 assumed to be equal to 500 */
};

/**
  * @brief  Virtual Bus Voltage sensor parameters Motor 1 - Base class
  */
BusVoltageSensorParams_t VirtualBusVoltageSensorParamsM1 =
{
  .bSensorType       = VIRTUAL_SENSOR,                   /*!< It contains the information about the type
                                                              of instanced bus voltage sensor object.
                                                              It can be equal to REAL_SENSOR or
                                                              VIRTUAL_SENSOR */
  .hConversionFactor = 500                              /*!< It is used to convert bus voltage from
                                                             u16Volts into real Volts (V).
                                                             1 u16Volt = 65536/hConversionFactor Volts
                                                             For real sensors hConversionFactor it's
                                                             equal to the product between the expected MCU
                                                             voltage and the partioning network
                                                             attenuation. For virtual sensors it is
                                                             assumed to be equal to 500 */
};

/**
  * @brief  Bus Voltage sensor parameters Motor 1 - Rdivider
  */
RdividerParams_t RdividerParamsM1 =
{
  .bVbusADChannel         = VBUS_CHANNEL,                    /*!< ADC channel used for conversion of
                                                                  bus voltage. It must be equal to
                                                                  ADC_Channel_x x= 0, ..., 15*/
  .hVbusPort              = VBUS_GPIO_PORT,                  /*!< GPIO port used by bVbusADChannel.
                                                                  It must be equal to GPIOx x= A, B, ...*/
  .hVbusPin               = VBUS_GPIO_PIN,                   /*!< GPIO pin used by bVbusChannel. It must
                                                                  be equal to GPIO_Pin_x x= 0, 1, ...*/
  .bVbusSamplingTime      = VBUS_SAMPLING_TIME,         /*!< Sampling time used for bVbusChannel AD
                                                             conversion. It must be equal to
                                                             ADC_SampleTime_xCycles5 x= 1, 7, ...*/
  .hLowPassFilterBW       = VBUS_SW_FILTER_BW_FACTOR,         /*!< Use this number to configure the Vbus
                                                                   first order software filter bandwidth.
                                                                   hLowPassFilterBW = VBS_CalcBusReading
                                                                   call rate [Hz]/ FilterBandwidth[Hz] */
  .hOverVoltageThreshold  = OVERVOLTAGE_THRESHOLD_d,          /*!< It represents the over voltage protection
                                                                   intervention threshold. To be expressed
                                                                   in digit through formula:
                                                                   hOverVoltageThreshold (digit) =
                                                                   Over Voltage Threshold (V) * 65536
                                                                                    / hConversionFactor */
  .hUnderVoltageThreshold = UNDERVOLTAGE_THRESHOLD_d,        /*!< It represents the under voltage protection
                                                                  intervention threshold. To be expressed
                                                                  in digit through formula:
                                                                  hUnderVoltageThreshold (digit) =
                                                                  Under Voltage Threshold (V) * 65536
                                                                  / hConversionFactor */
};

/**
  * @brief  Virtual Bus Voltage sensor parameters Motor 1 - VirtualParams
  */
VirtualParams_t VirtualBusParamsM1 =
{
 .hExpectedVbus_d = 1+(NOMINAL_BUS_VOLTAGE_V*65536)/500 /*!< Expected Vbus voltage expressed in
                                                             digit*/
};




/**
  * @brief  Real temperature sensor parameters Motor 1 - Base class
  */
TempSensorParams_t RealTempSensorParamsM1 =
{
  .bSensorType = REAL_SENSOR,                   /*!< It contains the information about the type
                                                     of instanced temperature sensor object.
                                                     It can be equal to REAL_SENSOR or
                                                     VIRTUAL_SENSOR */
};

/**
  * @brief  Virtual temperature sensor parameters Motor 1 - Base class
  */
TempSensorParams_t VirtualTempSensorParamsM1 =
{
  .bSensorType = VIRTUAL_SENSOR,                   /*!< It contains the information about the type
                                                        of instanced temperature sensor object.
                                                        It can be equal to REAL_SENSOR or
                                                        VIRTUAL_SENSOR */
};

/**
  * @brief  Temperature sensor parameters Motor 1 - NTC
  */
NTCParams_t NTCParamsM1 =
{
  .bTsensADChannel         = TEMP_FDBK_CHANNEL,                       /*!< ADC channel used for conversion of
                                                                           bus voltage. It must be equal to
                                                                           ADC_Channel_x x= 0, ..., 15*/
  .hTsensPort              = TEMP_FDBK_GPIO_PORT,                                 /*!< GPIO port used by bVbusADChannel.
                                                                                       It must be equal to GPIOx x= A, B, ...*/
  .hTsensPin               = TEMP_FDBK_GPIO_PIN,                       /*!< GPIO pin used by bVbusChannel. It must
                                                                            be equal to GPIO_Pin_x x= 0, 1, ...*/
  .bTsensSamplingTime      = TEMP_SAMPLING_TIME,         /*!< Sampling time used for bVbusChannel AD
                                                              conversion. It must be equal to
                                                              ADC_SampleTime_xCycles5 x= 1, 7, ...*/
  .hLowPassFilterBW        = TEMP_SW_FILTER_BW_FACTOR,       /*!< Use this number to configure the
                                                                  temperature first order software filter
                                                                  bandwidth
                                                                  hLowPassFilterBW = TSNS_CalcBusReading
                                                                  call rate [Hz]/ FilterBandwidth[Hz] */
  .hOverTempThreshold      = (uint16_t)(OV_TEMPERATURE_THRESHOLD_d),/*!< It represents the over voltage protection
                                                                         intervention threshold. To be expressed
                                                                         in u16Celsius through formula:
                                                                         hOverTempThreshold [u16Celsius] =
                                                                         (V0[V]+dV/dT[V/°C]*(OverTempThreshold[°C]
                                                                          - T0[°C]))* 65536 / MCU supply voltage */
  .hOverTempDeactThreshold = (uint16_t)(OV_TEMPERATURE_THRESHOLD_d - OV_TEMPERATURE_HYSTERESIS_d),
                                                /*!< It represents the temperature
                                                     expressed in u16Celsius below which an
                                                     active over temperature fault is cleared
                                                     hOverTempDeactThreshold[u16Celsius]=
                                                     (V0[V]+dV/dT[V/°C]*(OverTempDeactThresh[°C]
                                                      - T0[°C]))* 65536 / MCU supply voltage*/
  .hSensitivity            = (uint16_t)(MCU_SUPPLY_VOLTAGE/dV_dT),         /*!< It is equal to MCU supply voltage [V] /
                                                                                dV/dT [V/°C] and represents the NTC
                                                                                sensitivity */
  .wV0                     = (uint16_t)(V0_V *65536/ MCU_SUPPLY_VOLTAGE), /*!< It is equal to V0*65536/MCU supply
                                                                               voltage where V0 is used to convert the
                                                                               temperature into Volts
                                                                               V[V]=V0+dV/dT[V/°C]*(T-T0)[°C]      */
  .hT0                     = T0_C,                                /*!< It is equal to T0 [°C], used to convert
                                                                       the temperature into Volts
                                                                       V[V]=V0+dV/dT[V/°C]*(T-T0)[°C]*/
};

/**
  * @brief  Temperature sensor parameters Motor 1 - NTC
  */
VirtualTParams_t VirtualTParamsM1=
{
  .hExpectedTemp_d = 555,  /*!< Value returned by base class method TSNS_GetAvTemp_d */
  .hExpectedTemp_C = VIRTUAL_HEAT_SINK_TEMPERATURE_VALUE   /*!< Expected temperature in degrees
                                        (value returned by base class
                                        method TSNS_GetAvTemp_C) */
};

/**
  * @brief  CircleLimitation class parameters Motor 1 - Base Class
  */
CircleLimitationParams_t CircleLimitationParamsM1 =
{
  .hMaxModule          = MAX_MODULE,                           /*!< Circle limitation maximum allowed
                                                                 module */
  .hCircle_limit_table = MMITABLE,                             /*!< Circle limitation table */
  .bStart_index        = START_INDEX,                          /*!< Circle limitation table indexing
                                           start */
};

/**
  * @brief  FluxWeakeningCtrlClass class parameters Motor 1
  */
FluxWeakeningCtrlParams_t FWParamsM1 =
{
  .hMaxModule             = MAX_MODULE,                 /*!< Circle limitation maximum allowed module */
  .hDefaultFW_V_Ref       = (int16_t)FW_VOLTAGE_REF,     /*!< Default flux weakening voltage reference,
                                                              tenth of percentage points*/
  .hDemagCurrent          = ID_DEMAG,                    /*!< Demagnetization current in s16A:
                                                              Current(Amp) = [Current(s16A) * Vdd micro]/
                                                              [65536 * Rshunt * Aop] */
  .wNominalSqCurr         = ((int32_t)NOMINAL_CURRENT*(int32_t)NOMINAL_CURRENT),
                                                         /*!< Squared motor nominal current in (s16A)^2
                                                              where:
                                                              Current(Amp) = [Current(s16A) * Vdd micro]/
                                                              [65536 * Rshunt * Aop] */
  .hVqdLowPassFilterBW    = VQD_SW_FILTER_BW_FACTOR,     /*!< Use this parameter to configure the Vqd
                                                              first order software filter bandwidth.
                                                              hVqdLowPassFilterBW = FOC_CurrController
                                                              call rate [Hz]/ FilterBandwidth[Hz] in
                                                              case FULL_MISRA_COMPLIANCY is defined.
                                                              On the contrary, if FULL_MISRA_COMPLIANCY
                                                              is not defined, hVqdLowPassFilterBW is
                                                              equal to log with base two of previous
                                                              definition */
  .hVqdLowPassFilterBWLOG = VQD_SW_FILTER_BW_FACTOR_LOG  /*!< hVqdLowPassFilterBW expressed as power of 2.
                                                              E.g. if gain divisor is 512 the value
                                                              must be 9 because 2^9 = 512 */
};

/**
  * @brief  FeedForwardCtrlClass class parameters Motor 1
  */
FeedForwardCtrlParams_t FFParamsM1 =
{
  .hVqdLowPassFilterBW    = VQD_SW_FILTER_BW_FACTOR,       /*!< Use this parameter to configure the Vqd
                                                                first order software filter bandwidth.
                                                                hVqdLowPassFilterBW = FOC_CurrController
                                                                call rate [Hz]/ FilterBandwidth[Hz] in
                                                                case FULL_MISRA_COMPLIANCY is defined.
                                                                On the contrary, if FULL_MISRA_COMPLIANCY
                                                                is not defined, hVqdLowPassFilterBW is
                                                                equal to log with base two of previous
                                                                definition */
  .wDefConstant_1D        = (int32_t)CONSTANT1_D,          /*!< Feed forward constant for d axes */
  .wDefConstant_1Q        = (int32_t)CONSTANT1_Q,          /*!< Feed forward constant for q axes */
  .wDefConstant_2         = (int32_t)CONSTANT2_QD,         /*!< Constant used by Feed-Forward algorithm*/
  .hVqdLowPassFilterBWLOG = VQD_SW_FILTER_BW_FACTOR_LOG    /*!< hVqdLowPassFilterBW expressed as power of 2.
                                                                E.g. if gain divisor is 512 the value
                                                                must be 9 because 2^9 = 512 */
};

/**
  * @brief  MTPACtrlClass class parameters Motor 1
  */
MTPACtrlParams_t MTPAParamsM1 =
{
  .hSegDiv   = (int16_t)SEGDIV,        /*!< Segment divisor */
  .wAngCoeff = ANGC,                   /*!< Angular coefficients table */
  .wOffset   = OFST,                   /*!< Offsets table */
};

/**
  * @brief  OpenLoop class parameters Motor 1
  */
OpenLoopParams_t OpenLoop_ParamsM1 =
{
  .hDefaultVoltage  = OPEN_LOOP_VOLTAGE_d, /*!< Default Open loop voltage. */
  .VFMode           = OPEN_LOOP_VF,        /*!< Default mode. TRUE to enable V/F mode otherwise
                                                FALSE */
  .hVFOffset        = OPEN_LOOP_OFF,       /*!< Offset of V/F curve expressed in s16 Voltage
                                                applied when frequency is zero. */
  .hVFSlope         = OPEN_LOOP_K          /*!< Slope of V/F curve expressed in s16 Voltage for
                                                each 0.1Hz of mecchanical frequency increment. */
};

/**
  * @brief  IMHiFreqInj_FPU class parameters Motor 1 - Derived Class - HFI
  */
HiFreqInj_FPU_CtrlParams_t HiFreqInj_FPU_ParamsM1 =
{
  {HFI_NOTCH_0_COEFF,HFI_NOTCH_1_COEFF,HFI_NOTCH_2_COEFF,
   HFI_NOTCH_3_COEFF,HFI_NOTCH_4_COEFF},
  {HFI_LP_0_COEFF,HFI_LP_1_COEFF,HFI_LP_2_COEFF,
   HFI_LP_3_COEFF,HFI_LP_4_COEFF},
  {HFI_HP_0_COEFF,HFI_HP_1_COEFF,HFI_HP_2_COEFF,
   HFI_HP_3_COEFF,HFI_HP_4_COEFF},
  {HFI_DC_0_COEFF,HFI_DC_1_COEFF,HFI_DC_2_COEFF,
   HFI_DC_3_COEFF,HFI_DC_4_COEFF},
  HFI_FREQUENCY,          /*!< High frequency signal, Hertz */
  HFI_AMPLITUDE,          /*!< High frequency signal amplitude, volts */
  TF_REGULATION_RATE,     /*!< PWM frequency, Hertz*/
  HFI_IDH_DELAY,          /*!< Idh delay*/
  {
    (int16_t)HFI_PID_KP_DEFAULT,
                          /*!< Default Kp gain, used to initialize Kp gain
                               software variable*/
    (int16_t)HFI_PID_KI_DEFAULT,
                          /*!< Default Ki gain, used to initialize Ki gain
                               software variable*/
    (uint16_t)HFI_PID_KPDIV,
                          /*!< Kp gain divisor, used in conjuction with
                               Kp gain allows obtaining fractional values.
                               If FULL_MISRA_C_COMPLIANCY is not defined
                               the divisor is implemented through
                               algebrical right shifts to speed up PI
                               execution. Only in this case this parameter
                               specifies the number of right shifts to be
                               executed */
    (uint16_t)HFI_PID_KIDIV,
                          /*!< Ki gain divisor, used in conjuction with
                               Ki gain allows obtaining fractional values.
                               If FULL_MISRA_C_COMPLIANCY is not defined
                               the divisor is implemented through
                               algebrical right shifts to speed up PI
                               execution. Only in this case this parameter
                               specifies the number of right shifts to be
                               executed */
    S32_MAX,              /*!< Upper limit used to saturate the integral
                               term given by Ki / Ki divisor * integral of
                               process variable error */
    -S32_MAX,             /*!< Lower limit used to saturate the integral
                               term given by Ki / Ki divisor * integral of
                               process variable error */
    S16_MAX,              /*!< Upper limit used to saturate the PI output */
    -S16_MAX,             /*!< Lower limit used to saturate the PI output */
    (uint16_t)HFI_PID_KPDIV_LOG,
                          /*!< Kp gain divisor expressed as power of 2.
                               E.g. if gain divisor is 512 the value
                               must be 9 because 2^9 = 512 */
    (uint16_t)HFI_PID_KIDIV_LOG
                          /*!< Ki gain divisor expressed as power of 2.
                               E.g. if gain divisor is 512 the value
                               must be 9 because 2^9 = 512 */
  },      	          /*!< It contains the parameters of the PI
                               object necessary for track
                               implementation */

  /* init scan parameters */
  HFI_PLL_KP_DEFAULT,     /*!< KP for init scan PLL */
  HFI_PLL_KI_DEFAULT,     /*!< KI for init scan PLL */
  HFI_LOCKFREQ,           /*!< rotation freq of the initial scan, expressed in dpp */
  HFI_SCANROTATIONSNO,    /*!< Number of initial rotor scans */
  HFI_HIFRAMPLSCAN,       /*!< High frequency signal amplification during scan, volts */
  HFI_WAITBEFORESN,       /*!< Wait time before NS detection, expressed in number of half HFI period*/
  HFI_WAITAFTERNS,        /*!< Wait time after NS detection, expressed in number of half HFI period */
  HFI_NSMAXDETPOINTS,     /*!< NS det points, < 31*/
  HFI_NSDETPOINTSSKIP,    /*!< NS det points skipped*/
  HFI_NS_MIN_SAT_DIFF,    /*!< Minimum saturation difference to validate NS detection.*/
  HFI_DEBUG_MODE,         /*!< HFI in debug mode just angle detection */
  HFI_REVERT_DIRECTION,   /*!< TRUE reverts the detected direction. */
  HFI_WAITTRACK,          /*!< Reserved */
  HFI_WAITSYNCH,          /*!< Reserved */
  HFI_STEPANGLE,          /*!< Reserved */
  HFI_MAXANGLEDIFF,       /*!< Reserved */
  HFI_RESTARTTIMESEC      /*!< Reserved */
};

DigitalOutputParams_t R_BrakeParamsM1 =
{
  .hDOutputPort      = R_BRAKE_GPIO_PORT,       /*!< GPIO output port. It must be equal
                                                     to GPIOx x= A, B, ...*/
  .hDOutputPin       = R_BRAKE_GPIO_PIN,        /*!< GPIO output pin. It must be equal to
                                                     GPIO_Pin_x x= 0, 1, ...*/
  .bDOutputPolarity  = DISSIPATIVE_BRAKE_POLARITY /*!< GPIO output polarity. It must be equal
                                                       to DOutputActiveHigh or DOutputActiveLow */ };

DigitalOutputParams_t DOUT_OCPDisablingParamsM1 =
{
  .hDOutputPort      = OV_CURR_BYPASS_GPIO_PORT,       /*!< GPIO output port. It must be equal
                                                            to GPIOx x= A, B, ...*/
  .hDOutputPin       = OV_CURR_BYPASS_GPIO_PIN,        /*!< GPIO output pin. It must be equal to
                                                            GPIO_Pin_x x= 0, 1, ...*/
  .bDOutputPolarity  = OVERCURR_PROTECTION_HW_DISABLING      /*!< GPIO output polarity. It must be equal
                                                                  to DOutputActiveHigh or DOutputActiveLow */
};

PQDParams_t PQD_MotorPowerMeasurementParamsM1 =
{
  .wConvFact = PQD_CONVERSION_FACTOR/*!< It is the conversion factor used to convert the
                                         variables expressed in digit into variables expressed
                                         in physical measurement unit. It is used to convert
                                         the power in watts. It must be equal to
                                         (1000 * 3 * Vdd)/(sqrt(3) * CurrentAmpGain) */
};

DigitalOutputParams_t ICLDOUTParamsM1 =
{
  .hDOutputPort      = INRUSH_CURRLIMIT_GPIO_PORT, /*!< GPIO output port. It must be equal
                                                        to GPIOx x= A, B, ...*/
  .hDOutputPin       = INRUSH_CURRLIMIT_GPIO_PIN,  /*!< GPIO output pin. It must be equal to
                                                        GPIO_Pin_x x= 0, 1, ...*/
  .bDOutputPolarity  = INRUSH_CURR_LIMITER_POLARITY  /*!< GPIO output polarity. It must be equal
                                                          to DOutputActiveHigh or DOutputActiveLow */
};

InrushCurrentLimiterParams_t InrushCurrentLimiterParamsM1 =
{
  .hICLFrequencyHz = SPEED_LOOP_FREQUENCY_HZ,         /*!< Frequency expressed in Hz at which the user
                                                           clocks the ICL calling ICL_Exec method */
  .hDurationms     = INRUSH_CURRLIMIT_CHANGE_AFTER_MS, /*!< Duration of inrush limiter activation or
                                                            deactivation expressed in milliseconds.*/
};

SelfComCtrlParams_t SelfComCtrlParamsM1 =
{
  {
    TF_REGULATION_RATE /*!< Execution frequency expressed in Hz */
  },
  (float)(RSHUNT),   /*!< Value of shunt resistor.*/
  (float)(AMPLIFICATION_GAIN), /*!< Current sensing amplification gain.*/
  (float)(BUS_VOLTAGE_CONVERSION_FACTOR),/*!< Bus voltage conversion factor.*/
  (float)(VBUS_PARTITIONING_FACTOR), /*!< Bus voltage partitioning factor.
                          (Vmcu / Bus voltage conversion factor).*/
  (float)(CALIBRATION_FACTOR),/*!< Power stage calibration factor.
                         (Measured experimentally).*/
  (float)(DC_CURRENT_RS_MEAS),/*!< Maxium level of DC current used
                          for RS measurement.*/
  (uint16_t)(8000),  /*!< Duration of voltage ramp executed
                          for the duty cycle determination stage.*/
  (uint16_t)(1000),  /*!< Duration of the alignment stage.*/
  (uint16_t)(500),   /*!< Duration of R detection stage.*/
  (float)(LDLQ_RATIO),/*!< Ld vs Lq ratio.*/
  (float)(CURRENT_REGULATOR_BANDWIDTH),/*!< Bandwidth of current regulator.*/
  FALSE,             /*!< Set to TRUE for characterization
                           of power board, otherwise FALSE.*/
  MOTOR_MAX_SPEED_RPM, /*!< Nominal speed set by the user expressed in RPM.*/
  (uint16_t)(PWM_FREQUENCY), /*!< PWM frequency used for the test.*/
  (uint8_t)(REGULATION_EXECUTION_RATE), /*!< FOC repetition rate used for the test.*/
  (float) MCU_SUPPLY_VOLTAGE		/*!< MCU Power Supply */
};

OneTouchTuningParams_t OneTouchTuningParamsM1 =
{
  {
    MEDIUM_FREQUENCY_TASK_RATE /*!< Frequency expressed in Hz at which the user
                                   clocks the OTT calling OTT_MF method */
  },
  (float)(SPEED_REGULATOR_BANDWIDTH),/*!< Default bandwidth of speed regulator.*/
  1.0f,                       /*!< Duration of measurement window for speed and
                                   current Iq, expressed in seconds.*/
  POLE_PAIR_NUM,              /*!< Number of motor poles pairs.*/
  (int16_t)NOMINAL_CURRENT,   /*!< Maximum positive value of motor
                                   torque. This value represents
                                   actually the maximum Iq current
                                   expressed in digit.*/
  10.0f,                      /*!< Current regulation stabilization time in seconds.*/
  0.6f,                       /*!< OTT lower speed percentage.*/
  0.8f,                       /*!< OTT higher speed percentage.*/
  20.0f,                      /*!< Speed stabilization time in seconds.*/
  10.0f,                      /*!< Timeout for speed stabilization.*/
  0.90f,                      /*!< Speed margin percentage to validate speed ctrl.*/
  MOTOR_MAX_SPEED_RPM,        /*!< Nominal speed set by the user expressed in RPM.*/
  MP_KP,                      /*!< Initial KP factor of the speed regulator to be tuned.*/
  MP_KI,                      /*!< Initial KI factor of the speed regulator to be tuned.*/
  0.1f,                       /*!< Initial antiwindup factor of the speed regulator to be tuned.*/
  (float)(RSHUNT),            /*!< Value of shunt resistor.*/
  (float)(AMPLIFICATION_GAIN) /*!< Current sensing amplification gain.*/
};

RampExtMngrParams_t rampExtMngrHFParamsM1 =
{
 .hFrequencyHz = TF_REGULATION_RATE /*!< Execution frequency expressed in Hz */
};

#endif /* __SYSTEMNDRIVE_PARAMS_H */
/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
