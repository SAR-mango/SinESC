
/**
  ******************************************************************************
  * @file    mc_parameters.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides definitions of HW parameters specific to the
  *          configuration of the subsystem.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "parameters_conversion.h"

#include "r3_1_f30x_pwm_curr_fdbk.h"

/* USER CODE BEGIN Additional include */
#include "esc.h"
/* USER CODE END Additional include */

#define FREQ_RATIO 1                /* Dummy value for single drive */
#define FREQ_RELATION HIGHEST_FREQ  /* Dummy value for single drive */

/**
  * @brief  Current sensor parameters Motor 1 - three shunt 1 ADC (STM32F302x8)
  */
const R3_1_Params_t R3_1_ParamsM1 =
{
/* Dual MC parameters --------------------------------------------------------*/
  .FreqRatio       = FREQ_RATIO,
  .IsHigherFreqTim = FREQ_RELATION,

/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx           = ADC1,
  .ADCConfig = {
                 MC_ADC_CHANNEL_2<<ADC_JSQR_JSQ1_Pos | MC_ADC_CHANNEL_3<<ADC_JSQR_JSQ2_Pos | 1<<ADC_JSQR_JL_Pos | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                 MC_ADC_CHANNEL_1<<ADC_JSQR_JSQ1_Pos | MC_ADC_CHANNEL_3<<ADC_JSQR_JSQ2_Pos | 1<<ADC_JSQR_JL_Pos | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                 MC_ADC_CHANNEL_3<<ADC_JSQR_JSQ1_Pos | MC_ADC_CHANNEL_1<<ADC_JSQR_JSQ2_Pos | 1<<ADC_JSQR_JL_Pos | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                 MC_ADC_CHANNEL_2<<ADC_JSQR_JSQ1_Pos | MC_ADC_CHANNEL_1<<ADC_JSQR_JSQ2_Pos | 1<<ADC_JSQR_JL_Pos | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                 MC_ADC_CHANNEL_1<<ADC_JSQR_JSQ1_Pos | MC_ADC_CHANNEL_2<<ADC_JSQR_JSQ2_Pos | 1<<ADC_JSQR_JL_Pos | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                 MC_ADC_CHANNEL_3<<ADC_JSQR_JSQ1_Pos | MC_ADC_CHANNEL_2<<ADC_JSQR_JSQ2_Pos | 1<<ADC_JSQR_JL_Pos | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
  },

  .ADCDataReg1 = {
                   &ADC1->JDR1,
                   &ADC1->JDR1,
                   &ADC1->JDR2,
                   &ADC1->JDR2,
                   &ADC1->JDR1,
                   &ADC1->JDR2,
  },
  .ADCDataReg2 = {
                   &ADC1->JDR2,
                   &ADC1->JDR2,
                   &ADC1->JDR1,
                   &ADC1->JDR1,
                   &ADC1->JDR2,
                   &ADC1->JDR1,
  },

/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = REP_COUNTER,
  .Tafter            = TW_AFTER,
  .Tbefore           = TW_BEFORE_R3_1,
  .TIMx               = TIM1,

/* PWM Driving signals initialization ----------------------------------------*/
  .LowSideOutputs = (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING,
 .pwm_en_u_port      = MC_NULL,
 .pwm_en_u_pin       = (uint16_t) 0,
 .pwm_en_v_port      = MC_NULL,
 .pwm_en_v_pin       = (uint16_t) 0,
 .pwm_en_w_port      = MC_NULL,
 .pwm_en_w_pin       = (uint16_t) 0,

/* Emergency input (BKIN2) signal initialization -----------------------------*/
  .BKIN2Mode     = INT_MODE,

/* Internal COMP settings ----------------------------------------------------*/
  .CompOCPASelection     = COMP2,
  .CompOCPAInvInput_MODE = INT_MODE,
  .CompOCPBSelection     = COMP4,
  .CompOCPBInvInput_MODE = INT_MODE,
  .CompOCPCSelection     = COMP6,
  .CompOCPCInvInput_MODE = INT_MODE,

  .CompOVPSelection      = MC_NULL,
  .CompOVPInvInput_MODE  = NONE,

/* DAC settings --------------------------------------------------------------*/
  .DAC_OCP_Threshold =  6752,
  .DAC_OVP_Threshold =  23830,
};

/* USER CODE BEGIN Additional parameters */
const ESC_Params_t ESC_ParamsM1 =
{
  .Command_TIM = TIM2,
  .Motor_TIM = TIM1,
  .ARMING_TIME = 200,
  .PWM_TURNOFF_MAX  = 500,   
  .TURNOFF_TIME_MAX = 500, 
  .Ton_max =  ESC_TON_MAX,                      /*!<  Maximum ton value for PWM (by default is 1800 us) */
  .Ton_min =  ESC_TON_MIN,                      /*!<  Minimum ton value for PWM (by default is 1080 us) */ 
  .Ton_arming = ESC_TON_ARMING,                 /*!<  Minimum value to start the arming of PWM */ 
  .delta_Ton_max = ESC_TON_MAX - ESC_TON_MIN,      
  .speed_max_valueRPM = MOTOR_MAX_SPEED_RPM,    /*!< Maximum value for speed reference from Workbench */
  .speed_min_valueRPM = 1000,                   /*!< Set the minimum value for speed reference */
  .motor = M1,
};
/* USER CODE END Additional parameters */

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
