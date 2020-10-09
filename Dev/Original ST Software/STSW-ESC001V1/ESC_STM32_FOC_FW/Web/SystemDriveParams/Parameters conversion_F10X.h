/**
  ******************************************************************************
  * @file    Parameters conversion_F10X.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file implements the Parameter conversion on the base
  *          of stdlib F10x for the first drive
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
#ifndef __PARAMETERS_CONVERSION_F10X_H
#define __PARAMETERS_CONVERSION_F10X_H

#include "Definitions.h"
#include "PMSM motor parameters.h"
#include "Drive parameters.h"
#include "Power stage parameters.h"
#include "Control stage parameters.h"
#include "stm32f10x.h"
#include "MC_Math.h"

#define SQRT_2  1.4142
#define SQRT_3  1.732

/************************* CPU & ADC PERIPHERAL CLOCK CONFIG ******************/

#ifdef CPU_CLK_72_MHZ
#define SYSCLK_FREQ_72MHz  72000000uL
#define TIM_CLOCK_DIVIDER  1
#define ADC_CLOCK_DIVIDER  RCC_PCLK2_Div6
#define ADV_TIM_CLK_MHz    72uL/TIM_CLOCK_DIVIDER
#define ADC_CLK_MHz        12uL
#define HALL_TIM_CLK       SYSCLK_FREQ_72MHz

#elif defined CPU_CLK_56_MHZ
#define SYSCLK_FREQ_56MHz  56000000uL
#define TIM_CLOCK_DIVIDER  1
#define ADC_CLOCK_DIVIDER  RCC_PCLK2_Div4
#define ADV_TIM_CLK_MHz    56uL/TIM_CLOCK_DIVIDER
#define ADC_CLK_MHz        14uL
#define HALL_TIM_CLK       SYSCLK_FREQ_56MHz
#error "The CPU frequency set in Contol stage parameters is not automatically configured in system_stm32f10x.c. Please configure it manually if you require to work with the selected CPU frequency. Once done, comment this line to avoid to show this messagge again."

#elif defined CPU_CLK_48_MHZ
#define SYSCLK_FREQ_48MHz  48000000uL
#define TIM_CLOCK_DIVIDER  1
#define ADC_CLOCK_DIVIDER  RCC_PCLK2_Div4
#define ADV_TIM_CLK_MHz    48uL/TIM_CLOCK_DIVIDER
#define ADC_CLK_MHz        12uL
#define HALL_TIM_CLK       SYSCLK_FREQ_48MHz
#error "The CPU frequency set in Contol stage parameters is not automatically configured in system_stm32f10x.c. Please configure it manually if you require to work with the selected CPU frequency. Once done, comment this line to avoid to show this messagge again."

#elif defined CPU_CLK_24_MHZ
#define SYSCLK_FREQ_24MHz  24000000uL
#define TIM_CLOCK_DIVIDER  1
#define ADC_CLOCK_DIVIDER  RCC_PCLK2_Div2
#define ADV_TIM_CLK_MHz    24uL/TIM_CLOCK_DIVIDER
#define ADC_CLK_MHz        12uL
#define HALL_TIM_CLK       SYSCLK_FREQ_24MHz
#if (defined (STM32F103x_HD) || defined(STM32F103x_MD) || defined(STM32F103x_LD))
#error "The CPU frequency set in Contol stage parameters is not automatically configured in system_stm32f10x.c. Please configure it manually if you require to work with the selected CPU frequency. Once done, comment this line to avoid to show this messagge again."
#endif
#else
#endif

#define ADC1_2  ADC1
/************************* CONTROL FREQUENCIES & DELAIES **********************/
#define TF_REGULATION_RATE 	(uint16_t) ((uint16_t)(PWM_FREQUENCY)/REGULATION_EXECUTION_RATE)
#define REP_COUNTER 			(uint16_t) ((REGULATION_EXECUTION_RATE *2u)-1u)

#define MEDIUM_FREQUENCY_TASK_RATE	(uint16_t)SPEED_LOOP_FREQUENCY_HZ

#define INRUSH_CURRLIMIT_DELAY_COUNTS  (uint16_t)(INRUSH_CURRLIMIT_DELAY_MS * \
                                  ((uint16_t)SPEED_LOOP_FREQUENCY_HZ)/1000u -1u)

#define SYS_TICK_FREQUENCY          2000
#define UI_TASK_FREQUENCY_HZ        10
#define SERIAL_COM_TIMEOUT_INVERSE  25
#define SERIAL_COM_ATR_TIME_MS 20

#define MF_TASK_OCCURENCE_TICKS  (SYS_TICK_FREQUENCY/SPEED_LOOP_FREQUENCY_HZ)-1u
#define UI_TASK_OCCURENCE_TICKS  (SYS_TICK_FREQUENCY/UI_TASK_FREQUENCY_HZ)-1u
#define SERIALCOM_TIMEOUT_OCCURENCE_TICKS (SYS_TICK_FREQUENCY/SERIAL_COM_TIMEOUT_INVERSE)-1u
#define SERIALCOM_ATR_TIME_TICKS (uint16_t)(((SYS_TICK_FREQUENCY * SERIAL_COM_ATR_TIME_MS) / 1000u) - 1u)

/*********************** SENSORLESS REV-UP PARAMETERS *************************/
#define FIRST_SLESS_ALGO_PHASE (ENABLE_SL_ALGO_FROM_PHASE-1u)  

#if OPEN_LOOP_FOC == ENABLE

#define OPEN_LOOP
#undef DEFAULT_TARGET_SPEED_RPM
#define DEFAULT_TARGET_SPEED_RPM       OPEN_LOOP_SPEED_RPM

#undef PHASE1_DURATION
#undef PHASE2_DURATION
#undef PHASE3_DURATION
#undef PHASE4_DURATION
#undef PHASE5_DURATION

#undef PHASE1_FINAL_SPEED_RPM
#undef PHASE2_FINAL_SPEED_RPM
#undef PHASE3_FINAL_SPEED_RPM
#undef PHASE4_FINAL_SPEED_RPM
#undef PHASE5_FINAL_SPEED_RPM

#undef VARIANCE_THRESHOLD
#undef CORD_VARIANCE_THRESHOLD

/* Phase 1 */
#define PHASE1_DURATION		        OPEN_LOOP_SPEED_RAMP_DURATION_MS
#define PHASE1_FINAL_SPEED_RPM	    OPEN_LOOP_SPEED_RPM

/* Phase 2 */
#define PHASE2_DURATION		      65535   /*milliseconds */
#define PHASE2_FINAL_SPEED_RPM	  PHASE1_FINAL_SPEED_RPM

/* Phase 3 */
#define PHASE3_DURATION		       65535   /*milliseconds */
#define PHASE3_FINAL_SPEED_RPM	  PHASE1_FINAL_SPEED_RPM

/* Phase 4 */
#define PHASE4_DURATION		  65535   /*milliseconds */
#define PHASE4_FINAL_SPEED_RPM	  PHASE1_FINAL_SPEED_RPM

/* Phase 5 */
#define PHASE5_DURATION		  65535    /* milliseconds */
#define PHASE5_FINAL_SPEED_RPM	  PHASE1_FINAL_SPEED_RPM

#define VARIANCE_THRESHOLD 0
#define CORD_VARIANCE_THRESHOLD 0

#endif

/* Legacy for WB 4.0 Beta */
#if !defined(OPEN_LOOP_VF)
#define OPEN_LOOP_VF FALSE
#endif
#if !defined(OPEN_LOOP_OFF)
#define OPEN_LOOP_OFF 4400
#endif
#if !defined(OPEN_LOOP_K)
#define OPEN_LOOP_K 44
#endif

/************************* OBSERVER + PLL PARAMETERS **************************/
#define MAX_BEMF_VOLTAGE  (u16)((MAX_APPLICATION_SPEED * 1.2 *\
                           MOTOR_VOLTAGE_CONSTANT*SQRT_2)/(1000u*SQRT_3))

#if (BUS_VOLTAGE_READING == ENABLE)
/*max phase voltage, 0-peak Volts*/
#define MAX_VOLTAGE (int16_t)((MCU_SUPPLY_VOLTAGE/2)/BUS_ADC_CONV_RATIO) 
#else
#define MAX_VOLTAGE (int16_t)(500/2) /* Virtual sensor conversion factor */
#endif

#ifdef ICS_SENSORS 
#define MAX_CURRENT (MCU_SUPPLY_VOLTAGE/(2*AMPLIFICATION_GAIN))
#else
#define MAX_CURRENT (MCU_SUPPLY_VOLTAGE/(2*RSHUNT*AMPLIFICATION_GAIN))
#endif

#define C1 (int32_t)((((int16_t)F1)*RS)/(LS*TF_REGULATION_RATE))
#define C2 (int32_t) GAIN1
#define C3 (int32_t)((((int16_t)F1)*MAX_BEMF_VOLTAGE)/(LS*MAX_CURRENT*TF_REGULATION_RATE))
#define C4 (int32_t) GAIN2
#define C5 (int32_t)((((int16_t)F1)*MAX_VOLTAGE)/(LS*MAX_CURRENT*TF_REGULATION_RATE))

#define PERCENTAGE_FACTOR    (uint16_t)(VARIANCE_THRESHOLD*128u)      
#define OBS_MINIMUM_SPEED        (uint16_t) (OBS_MINIMUM_SPEED_RPM/6u)
#define HFI_MINIMUM_SPEED        (uint16_t) (HFI_MINIMUM_SPEED_RPM/6u)

/*********************** OBSERVER + CORDIC PARAMETERS *************************/
#define CORD_C1 (int32_t)((((int16_t)CORD_F1)*RS)/(LS*TF_REGULATION_RATE))
#define CORD_C2 (int32_t) CORD_GAIN1
#define CORD_C3 (int32_t)((((int16_t)CORD_F1)*MAX_BEMF_VOLTAGE)/(LS*MAX_CURRENT\
                                                           *TF_REGULATION_RATE))
#define CORD_C4 (int32_t) CORD_GAIN2
#define CORD_C5 (int32_t)((((int16_t)CORD_F1)*MAX_VOLTAGE)/(LS*MAX_CURRENT*\
                                                          TF_REGULATION_RATE))
#define CORD_PERCENTAGE_FACTOR    (uint16_t)(CORD_VARIANCE_THRESHOLD*128u)      
#define CORD_MINIMUM_SPEED        (uint16_t) (CORD_MINIMUM_SPEED_RPM/6u)
/**************************   VOLTAGE CONVERSIONS  ****************************/
#define BUS_ADC_CONV_RATIO       VBUS_PARTITIONING_FACTOR

#define OVERVOLTAGE_THRESHOLD_d   (uint16_t)(OV_VOLTAGE_THRESHOLD_V*65535/\
                                  (MCU_SUPPLY_VOLTAGE/VBUS_PARTITIONING_FACTOR))
#define UNDERVOLTAGE_THRESHOLD_d  (uint16_t)((UD_VOLTAGE_THRESHOLD_V*65535)/\
                                  ((uint16_t)(MCU_SUPPLY_VOLTAGE/\
                                                           BUS_ADC_CONV_RATIO)))
#define INT_SUPPLY_VOLTAGE          (uint16_t)(65536/MCU_SUPPLY_VOLTAGE)

#define DELTA_TEMP_THRESHOLD        (OV_TEMPERATURE_THRESHOLD_C- T0_C)
#define DELTA_V_THRESHOLD           (dV_dT * DELTA_TEMP_THRESHOLD)
#define OV_TEMPERATURE_THRESHOLD_d  ((V0_V + DELTA_V_THRESHOLD)*INT_SUPPLY_VOLTAGE)

#define DELTA_TEMP_HYSTERESIS        (OV_TEMPERATURE_HYSTERESIS_C)
#define DELTA_V_HYSTERESIS           (dV_dT * DELTA_TEMP_HYSTERESIS)
#define OV_TEMPERATURE_HYSTERESIS_d  (DELTA_V_HYSTERESIS*INT_SUPPLY_VOLTAGE)
														  														  
/*************************  PWM IDLE STATES AND POLARITY  *********************/														  
#define H_ACTIVE_HIGH    0x0000u //TIM_OCPolarity_High
#define H_ACTIVE_LOW     0x0002u //TIM_OCPolarity_Low
#define L_ACTIVE_HIGH    0x0000u  //TIM_OCNPolarity_High
#define L_ACTIVE_LOW     0x0008u  //TIM_OCNPolarity_Low 

#define DOUT_ACTIVE_HIGH   DOutputActiveHigh
#define DOUT_ACTIVE_LOW    DOutputActiveLow

#define EMSTOP_ACTIVE_HIGH    TIM_BreakPolarity_High
#define EMSTOP_ACTIVE_LOW     TIM_BreakPolarity_Low

#if (HIGH_SIDE_IDLE_STATE == TURN_OFF)
#if (PHASE_UH_POLARITY == H_ACTIVE_HIGH)
#define IDLE_UH_POLARITY  TIM_OCIdleState_Reset
#else /* PHASE_UH_POLARITY = H_ACTIVE_LOW */
#define IDLE_UH_POLARITY  TIM_OCIdleState_Set
#endif
#else /* HIGH_SIDE_IDLE_STATE = TURN_ON */
#if	(PHASE_UH_POLARITY == H_ACTIVE_HIGH)
#define IDLE_UH_POLARITY  TIM_OCIdleState_Set
#else /* PHASE_UH_POLARITY = H_ACTIVE_LOW */
#define IDLE_UH_POLARITY  TIM_OCIdleState_Reset
#endif
#endif

#if (HIGH_SIDE_IDLE_STATE == TURN_OFF)
#if	(PHASE_VH_POLARITY == H_ACTIVE_HIGH)
#define IDLE_VH_POLARITY  TIM_OCIdleState_Reset
#else /* PHASE_VH_POLARITY = H_ACTIVE_LOW */
#define IDLE_VH_POLARITY  TIM_OCIdleState_Set
#endif
#else /* HIGH_SIDE_IDLE_STATE = TURN_ON */
#if	(PHASE_VH_POLARITY == H_ACTIVE_HIGH)
#define IDLE_VH_POLARITY  TIM_OCIdleState_Set
#else /* PHASE_VH_POLARITY = H_ACTIVE_LOW */
#define IDLE_VH_POLARITY  TIM_OCIdleState_Reset
#endif
#endif

#if (HIGH_SIDE_IDLE_STATE == TURN_OFF)
#if	(PHASE_WH_POLARITY == H_ACTIVE_HIGH)
#define IDLE_WH_POLARITY  TIM_OCIdleState_Reset
#else /* PHASE_WH_POLARITY = H_ACTIVE_LOW */
#define IDLE_WH_POLARITY  TIM_OCIdleState_Set
#endif
#else /* HIGH_SIDE_IDLE_STATE = TURN_ON */
#if	(PHASE_WH_POLARITY == H_ACTIVE_HIGH)
#define IDLE_WH_POLARITY  TIM_OCIdleState_Set
#else /* PHASE_WH_POLARITY = H_ACTIVE_LOW */
#define IDLE_WH_POLARITY  TIM_OCIdleState_Reset
#endif
#endif

#if (LOW_SIDE_IDLE_STATE == TURN_OFF)
#if	(PHASE_UL_POLARITY == L_ACTIVE_HIGH)
#define IDLE_UL_POLARITY  TIM_OCNIdleState_Reset
#else /* PHASE_UL_POLARITY = L_ACTIVE_LOW */
#define IDLE_UL_POLARITY  TIM_OCNIdleState_Set
#endif
#else /* LOW_SIDE_IDLE_STATE = TURN_ON */
#if	(PHASE_UL_POLARITY == L_ACTIVE_HIGH)
#define IDLE_UL_POLARITY  TIM_OCNIdleState_Set
#else /* PHASE_UL_POLARITY = L_ACTIVE_LOW */
#define IDLE_UL_POLARITY  TIM_OCNIdleState_Reset
#endif
#endif

#if (LOW_SIDE_IDLE_STATE == TURN_OFF)
#if	(PHASE_VL_POLARITY == L_ACTIVE_HIGH)
#define IDLE_VL_POLARITY  TIM_OCNIdleState_Reset
#else /* PHASE_VL_POLARITY = L_ACTIVE_LOW */
#define IDLE_VL_POLARITY  TIM_OCNIdleState_Set
#endif
#else /* LOW_SIDE_IDLE_STATE = TURN_ON */
#if	(PHASE_VL_POLARITY == L_ACTIVE_HIGH)
#define IDLE_VL_POLARITY  TIM_OCNIdleState_Set
#else /* PHASE_VL_POLARITY = L_ACTIVE_LOW */
#define IDLE_VL_POLARITY  TIM_OCNIdleState_Reset
#endif
#endif

#if (LOW_SIDE_IDLE_STATE == TURN_OFF)
#if	(PHASE_WL_POLARITY == L_ACTIVE_HIGH)
#define IDLE_WL_POLARITY  TIM_OCNIdleState_Reset
#else /* PHASE_WL_POLARITY = L_ACTIVE_LOW */
#define IDLE_WL_POLARITY  TIM_OCNIdleState_Set
#endif
#else /* LOW_SIDE_IDLE_STATE = TURN_ON */
#if	(PHASE_WL_POLARITY == L_ACTIVE_HIGH)
#define IDLE_WL_POLARITY  TIM_OCNIdleState_Set
#else /* PHASE_WL_POLARITY = L_ACTIVE_LOW */
#define IDLE_WL_POLARITY  TIM_OCNIdleState_Reset
#endif
#endif

#define PWM_TIM1			TIM1
#define PWM_TIM8                        TIM8
#define PWM_NO_REMAP		GPIO_NoRemap_TIM1
#define PWM_FULL_REMAP 		GPIO_FullRemap_TIM1
#define PWM_PARTIAL_REMAP 	GPIO_PartialRemap_TIM1

/**********  AUXILIARY TIMER (SINGLE SHUNT) *************/
#if (defined STM32PERFORMANCELD || defined STM32VALUELD)
#define R1_PWM_AUX_TIM                  TIM3
#else
#define R1_PWM_AUX_TIM                  TIM4
#endif

/**********  SPEED FEEDBACK TIMERS SETTING *************/
#if (defined(HALL_SENSORS) || defined(AUX_HALL_SENSORS))
  #if (HALL_TIMER_SELECTION == HALL_TIM2)
  #define HALL_TIMER 				TIM2  
  #define HALL_RCC_PERIPHERAL		RCC_APB1Periph_TIM2  
  #define HALL_IRQ_CHANNEL  		TIM2_IRQn
  #elif (HALL_TIMER_SELECTION == HALL_TIM3)
  #define HALL_TIMER 				TIM3  
  #define HALL_RCC_PERIPHERAL 	RCC_APB1Periph_TIM3
  #define HALL_IRQ_CHANNEL  		TIM3_IRQn  
  #elif (HALL_TIMER_SELECTION == HALL_TIM4)
  #define HALL_TIMER 				TIM4  
  #define HALL_RCC_PERIPHERAL 	RCC_APB1Periph_TIM4  
  #define HALL_IRQ_CHANNEL  		TIM4_IRQn
  #elif (HALL_TIMER_SELECTION == HALL_TIM5)
  #define HALL_TIMER 				TIM5  
  #define HALL_RCC_PERIPHERAL 	RCC_APB1Periph_TIM5
  #define HALL_IRQ_CHANNEL  		TIM5_IRQn  
  #else
  #error "Hall parameters not defined in Control Stage parametes.h"
  #endif
#else
  /* Dummy value to avoid compiler error */
  #undef HALL_TIMER
  #undef HALL_TIMER_REMAPPING
  #undef HALL_RCC_PERIPHERAL
  #undef HALL_IRQ_CHANNEL
  #undef H1_GPIO_PORT
  #undef H2_GPIO_PORT
  #undef H3_GPIO_PORT
  #undef H1_GPIO_PIN
  #undef H2_GPIO_PIN
  #undef H3_GPIO_PIN
  #define HALL_TIMER 				    TIM2
  #define HALL_TIMER_REMAPPING  NO_REMAP_TIM2
  #define HALL_RCC_PERIPHERAL		RCC_APB1Periph_TIM2
  #define HALL_IRQ_CHANNEL  		TIM2_IRQn
  #define H1_GPIO_PORT          GPIOA
  #define H2_GPIO_PORT          GPIOA
  #define H3_GPIO_PORT          GPIOA
  #define H1_GPIO_PIN           GPIO_Pin_0
  #define H2_GPIO_PIN           GPIO_Pin_1
  #define H3_GPIO_PIN           GPIO_Pin_2
#endif

#if (defined(ENCODER) || defined(AUX_ENCODER))
  #if (ENC_TIMER_SELECTION == ENC_TIM2)
  #define ENC_TIMER 				TIM2  
  #define ENC_RCC_PERIPHERAL		RCC_APB1Periph_TIM2  
  #define ENC_IRQ_CHANNEL  		TIM2_IRQn
  #elif (ENC_TIMER_SELECTION == ENC_TIM3)
  #define ENC_TIMER 				TIM3  
  #define ENC_RCC_PERIPHERAL 		RCC_APB1Periph_TIM3
  #define ENC_IRQ_CHANNEL  		TIM3_IRQn  
  #elif (ENC_TIMER_SELECTION == ENC_TIM4)
  #define ENC_TIMER 				TIM4  
  #define ENC_RCC_PERIPHERAL 		RCC_APB1Periph_TIM4  
  #define ENC_IRQ_CHANNEL  		TIM4_IRQn
  #elif (ENC_TIMER_SELECTION == ENC_TIM5)
  #define ENC_TIMER 				TIM5  
  #define ENC_RCC_PERIPHERAL 		RCC_APB1Periph_TIM5
  #define ENC_IRQ_CHANNEL  		TIM5_IRQn  
  #else
  #error "Encoder parameters not defined in Control Stage parametes.h"
  #endif
#else
  /* Dummy value to avoid compiler error */
  #undef  ENC_TIMER
  #undef  ENC_TIMER_REMAPPING
  #undef  ENC_RCC_PERIPHERAL
  #undef  ENC_IRQ_CHANNEL
  #undef  ENC_A_GPIO_PORT
  #undef  ENC_B_GPIO_PORT
  #undef  ENC_A_GPIO_PIN
  #undef  ENC_B_GPIO_PIN

  #define ENC_TIMER 				    TIM2
  #define ENC_TIMER_REMAPPING   NO_REMAP_TIM2
  #define ENC_RCC_PERIPHERAL		RCC_APB1Periph_TIM2
  #define ENC_IRQ_CHANNEL  		  TIM2_IRQn
  #define ENC_A_GPIO_PORT       GPIOA
  #define ENC_B_GPIO_PORT       GPIOA
  #define ENC_A_GPIO_PIN        GPIO_Pin_0
  #define ENC_B_GPIO_PIN        GPIO_Pin_1
#endif

/* Resistive Brake */
#if (ON_OVER_VOLTAGE != TURN_ON_R_BRAKE)
  /* Dummy value to avoid compiler error */
  #undef R_BRAKE_GPIO_PORT
  #undef R_BRAKE_GPIO_PIN
  
  #define R_BRAKE_GPIO_PORT     GPIOD
  #define R_BRAKE_GPIO_PIN      GPIO_Pin_13
#endif

/* Hardware over current protection bypass */
#if (HW_OV_CURRENT_PROT_BYPASS != ENABLE)
  /* Dummy value to avoid compiler error */
  #undef OV_CURR_BYPASS_GPIO_PORT
  #undef OV_CURR_BYPASS_GPIO_PIN
  
  #define OV_CURR_BYPASS_GPIO_PORT  GPIOD        
  #define OV_CURR_BYPASS_GPIO_PIN   GPIO_Pin_13
#endif

/* Inrush current limiter */
#if (INRUSH_CURRLIMIT_ENABLING != ENABLE)
  /* Dummy value to avoid compiler error */
  #undef INRUSH_CURRLIMIT_GPIO_PORT
  #undef INRUSH_CURRLIMIT_GPIO_PIN
  
  #define INRUSH_CURRLIMIT_GPIO_PORT  GPIOD
  #define INRUSH_CURRLIMIT_GPIO_PIN   GPIO_Pin_10
#endif

/* USART */
#if (SERIAL_COMMUNICATION != ENABLE)
  /* Dummy value to avoid compiler error */
  #undef USART_REMAPPING
  #undef USART_TX_GPIO_PORT
  #undef USART_TX_GPIO_PIN
  #undef USART_RX_GPIO_PORT
  #undef USART_SELECTION
  #undef USART_RX_GPIO_PIN
  
  #define USART_REMAPPING                 NO_REMAP_USART1
  #define USART_TX_GPIO_PORT              GPIOA
  #define USART_TX_GPIO_PIN               GPIO_Pin_9
  #define USART_RX_GPIO_PORT              GPIOA
  #define USART_SELECTION                 USE_USART1
  #define USART_RX_GPIO_PIN               GPIO_Pin_10
#endif

/* Current sensing topology */
#if (defined(SINGLE_SHUNT))
  /* Dummy value to avoid compiler error */
  #undef PHASE_U_CURR_ADC
  #undef PHASE_U_CURR_CHANNEL
  #undef PHASE_U_GPIO_PORT
  #undef PHASE_U_GPIO_PIN
  #undef PHASE_V_CURR_ADC
  #undef PHASE_V_CURR_CHANNEL
  #undef PHASE_V_GPIO_PORT
  #undef PHASE_V_GPIO_PIN
  #undef PHASE_W_CURR_ADC
  #undef PHASE_W_CURR_CHANNEL
  #undef PHASE_W_GPIO_PORT
  #undef PHASE_W_GPIO_PIN
  
  #define PHASE_U_CURR_ADC                ADC1_2
  #define PHASE_U_CURR_CHANNEL            ADC_Channel_11
  #define PHASE_U_GPIO_PORT               GPIOC
  #define PHASE_U_GPIO_PIN                GPIO_Pin_1
  #define PHASE_V_CURR_ADC                ADC1_2
  #define PHASE_V_CURR_CHANNEL            ADC_Channel_12
  #define PHASE_V_GPIO_PORT               GPIOC
  #define PHASE_V_GPIO_PIN                GPIO_Pin_2
  #define PHASE_W_CURR_ADC                ADC1_2
  #define PHASE_W_CURR_CHANNEL            ADC_Channel_13
  #define PHASE_W_GPIO_PORT               GPIOC
  #define PHASE_W_GPIO_PIN                GPIO_Pin_3
#elif (defined(ICS_SENSORS))
  /* Dummy value to avoid compiler error */
  #undef PHASE_W_CURR_ADC
  #undef PHASE_W_CURR_CHANNEL
  #undef PHASE_W_GPIO_PORT
  #undef PHASE_W_GPIO_PIN
  
  #define PHASE_W_CURR_ADC                ADC1_2
  #define PHASE_W_CURR_CHANNEL            ADC_Channel_13
  #define PHASE_W_GPIO_PORT               GPIOC
  #define PHASE_W_GPIO_PIN                GPIO_Pin_3
  
  #undef PHASE_CURRENTS_ADC
  #undef PHASE_CURRENTS_CHANNEL
  #undef PHASE_CURRENTS_GPIO_PORT
  #undef PHASE_CURRENTS_GPIO_PIN
  
  #define PHASE_CURRENTS_ADC              ADC3
  #define PHASE_CURRENTS_CHANNEL          ADC_Channel_12
  #define PHASE_CURRENTS_GPIO_PORT        GPIOC
  #define PHASE_CURRENTS_GPIO_PIN         GPIO_Pin_2
#else
  /* Dummy value to avoid compiler error */
  #undef PHASE_CURRENTS_ADC
  #undef PHASE_CURRENTS_CHANNEL
  #undef PHASE_CURRENTS_GPIO_PORT
  #undef PHASE_CURRENTS_GPIO_PIN
  
  #define PHASE_CURRENTS_ADC              ADC3
  #define PHASE_CURRENTS_CHANNEL          ADC_Channel_12
  #define PHASE_CURRENTS_GPIO_PORT        GPIOC
  #define PHASE_CURRENTS_GPIO_PIN         GPIO_Pin_2
#endif

#if (TEMPERATURE_READING != ENABLE)
  /* Dummy value to avoid compiler error */
  #undef TEMP_FDBK_GPIO_PORT
  #undef TEMP_FDBK_GPIO_PIN
  #define TEMP_FDBK_GPIO_PORT             GPIOA
  #define TEMP_FDBK_GPIO_PIN              GPIO_Pin_1
#endif

#if (BUS_VOLTAGE_READING != ENABLE)
  /* Dummy value to avoid compiler error */
  #undef VBUS_GPIO_PORT
  #undef VBUS_GPIO_PIN
  #define VBUS_GPIO_PORT                  GPIOA
  #define VBUS_GPIO_PIN                   GPIO_Pin_1
#endif

#if (HALL_ICx_FILTER <= 1)
#define HALL_IC_FILTER   0
#elif (HALL_ICx_FILTER <= 2)
#define HALL_IC_FILTER   1
#elif (HALL_ICx_FILTER <= 6)
#define HALL_IC_FILTER   2
#elif (HALL_ICx_FILTER <= 10)
#define HALL_IC_FILTER   3
#elif (HALL_ICx_FILTER <= 14)
#define HALL_IC_FILTER   4
#elif (HALL_ICx_FILTER <= 20)
#define HALL_IC_FILTER   5
#elif (HALL_ICx_FILTER <= 28)
#define HALL_IC_FILTER   6
#elif (HALL_ICx_FILTER <= 40)
#define HALL_IC_FILTER   7
#elif (HALL_ICx_FILTER <= 56)
#define HALL_IC_FILTER   8
#elif (HALL_ICx_FILTER <= 72)
#define HALL_IC_FILTER   9
#elif (HALL_ICx_FILTER <= 88)
#define HALL_IC_FILTER   10
#elif (HALL_ICx_FILTER <= 112)
#define HALL_IC_FILTER   11
#elif (HALL_ICx_FILTER <= 144)
#define HALL_IC_FILTER   12
#elif (HALL_ICx_FILTER <= 176)
#define HALL_IC_FILTER   13
#elif (HALL_ICx_FILTER <= 224)
#define HALL_IC_FILTER   14
#else
#define HALL_IC_FILTER   15
#endif


#if (ENC_ICx_FILTER <= 1)
#define ENC_IC_FILTER   0
#elif (ENC_ICx_FILTER <= 2)
#define ENC_IC_FILTER   1
#elif (ENC_ICx_FILTER <= 6)
#define ENC_IC_FILTER   2
#elif (ENC_ICx_FILTER <= 10)
#define ENC_IC_FILTER   3
#elif (ENC_ICx_FILTER <= 14)
#define ENC_IC_FILTER   4
#elif (ENC_ICx_FILTER <= 20)
#define ENC_IC_FILTER   5
#elif (ENC_ICx_FILTER <= 28)
#define ENC_IC_FILTER   6
#elif (ENC_ICx_FILTER <= 40)
#define ENC_IC_FILTER   7
#elif (ENC_ICx_FILTER <= 56)
#define ENC_IC_FILTER   8
#elif (ENC_ICx_FILTER <= 72)
#define ENC_IC_FILTER   9
#elif (ENC_ICx_FILTER <= 88)
#define ENC_IC_FILTER   10
#elif (ENC_ICx_FILTER <= 112)
#define ENC_IC_FILTER   11
#elif (ENC_ICx_FILTER <= 144)
#define ENC_IC_FILTER   12
#elif (ENC_ICx_FILTER <= 176)
#define ENC_IC_FILTER   13
#elif (ENC_ICx_FILTER <= 224)
#define ENC_IC_FILTER   14
#else
#define ENC_IC_FILTER   15
#endif
/*************** Encoder Alignemnt ************************/
    
/* Encoder alignment */
#define T_ALIGNMENT              ALIGNMENT_DURATION
#define ALIGNMENT_ANGLE_S16      (int16_t)  (ALIGNMENT_ANGLE_DEG*65536u/360u)

/*************** Timer for speed/position feedback   ******/
#define FULL_REMAP_TIM2			GPIO_FullRemap_TIM2
#define PARTIAL_REMAP_01_TIM2		GPIO_PartialRemap1_TIM2
#define PARTIAL_REMAP_10_TIM2		GPIO_PartialRemap2_TIM2
#define NO_REMAP_TIM2                   GPIO_NoRemap_TIMx

#define FULL_REMAP_TIM3			GPIO_FullRemap_TIM3
#define PARTIAL_REMAP_TIM3		GPIO_PartialRemap_TIM3
#define NO_REMAP_TIM3                   GPIO_NoRemap_TIMx

#define REMAP_TIM4			GPIO_Remap_TIM4
#define NO_REMAP_TIM4                   GPIO_NoRemap_TIMx

#define NO_REMAP_TIM5                   GPIO_NoRemap_TIMx

#define GPIO_AF_0  0 /* Dummy */
#define GPIO_AF_1  0 /* Dummy */
#define GPIO_AF_2  0 /* Dummy */
#define GPIO_AF_3  0 /* Dummy */
#define GPIO_AF_4  0 /* Dummy */
#define GPIO_AF_5  0 /* Dummy */
#define GPIO_AF_6  0 /* Dummy */
#define GPIO_AF_7  0 /* Dummy */

/*************** Timer for PWM generation & currenst sensing parameters  ******/
#define PWM_PERIOD_CYCLES (uint16_t)(ADV_TIM_CLK_MHz*\
                                      (unsigned long long)1000000u/((uint16_t)(PWM_FREQUENCY)))

#if ((LOW_SIDE_SIGNALS_ENABLING == ENABLE)||(LOW_SIDE_SIGNALS_ENABLING == LS_PWM_TIMER_VAL))
#define DEADTIME_NS  SW_DEADTIME_NS
#else
#define DEADTIME_NS  HW_DEAD_TIME_NS
#endif

#define DEAD_TIME_ADV_TIM_CLK_MHz (ADV_TIM_CLK_MHz * TIM_CLOCK_DIVIDER)
#define DEAD_TIME_COUNTS_1  (DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/1000uL)

#if (DEAD_TIME_COUNTS_1 <= 255)
  #define DEAD_TIME_COUNTS (uint16_t) DEAD_TIME_COUNTS_1
#elif (DEAD_TIME_COUNTS_1 <= 508)
  #define DEAD_TIME_COUNTS (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/2) /1000uL) + 128)
  #elif (DEAD_TIME_COUNTS_1 <= 1008)
    #define DEAD_TIME_COUNTS (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/8) /1000uL) + 320)
    #elif (DEAD_TIME_COUNTS_1 <= 2015)
      #define DEAD_TIME_COUNTS (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/16) /1000uL) + 384)
      #else
        #define DEAD_TIME_COUNTS 510
#endif

#define DTCOMPCNT (uint16_t)((DEADTIME_NS * ADV_TIM_CLK_MHz) / 2000)
#define TON_NS  500
#define TOFF_NS 500
#define TON  (uint16_t)((TON_NS * ADV_TIM_CLK_MHz)  / 2000)
#define TOFF (uint16_t)((TOFF_NS * ADV_TIM_CLK_MHz) / 2000)

#if (TNOISE_NS > TRISE_NS)
  #define MAX_TNTR_NS TNOISE_NS
#else
  #define MAX_TNTR_NS TRISE_NS
#endif


#define SAMPLING_TIME_NS (((CURR_SAMPLING_TIME) * 1000uL/ADC_CLK_MHz)+(7000uL/(2*ADC_CLK_MHz)))

#if (CURR_SAMPLING_TIME == 1)
#define SAMPLING_TIME_SEL  ADC_SampleTime_1Cycles5
#elif (CURR_SAMPLING_TIME == 7)
#define SAMPLING_TIME_SEL  ADC_SampleTime_7Cycles5
#elif (CURR_SAMPLING_TIME == 13)
#define SAMPLING_TIME_SEL  ADC_SampleTime_13Cycles5
#elif (CURR_SAMPLING_TIME == 28)
#define SAMPLING_TIME_SEL  ADC_SampleTime_28Cycles5
#endif

#define SAMPLING_TIME (uint16_t)(((uint16_t)(SAMPLING_TIME_NS) * ADV_TIM_CLK_MHz)/1000uL) 
#define TRISE (uint16_t)((((uint16_t)(TRISE_NS)) * ADV_TIM_CLK_MHz)/1000uL)
#define TDEAD (uint16_t)((DEADTIME_NS * ADV_TIM_CLK_MHz)/1000uL)

#define TMIN (((uint16_t)(((DEADTIME_NS+((uint16_t)(TRISE_NS))+\
			 ((uint16_t)(SAMPLING_TIME_NS)))*ADV_TIM_CLK_MHz)/1000ul))+1)
#define HTMIN (uint16_t)(TMIN >> 1)
#define TSAMPLE SAMPLING_TIME
#define TAFTER ((uint16_t)(((DEADTIME_NS+((uint16_t)(TRISE_NS)))\
												   *ADV_TIM_CLK_MHz)/1000ul))
#define TBEFORE (((uint16_t)(((((u16)(SAMPLING_TIME_NS)))\
											*ADV_TIM_CLK_MHz)/1000ul))+1)

#if (TRISE_NS > SAMPLING_TIME_NS)
  #define MAX_TRTS (2 * TRISE)
#else
  #define MAX_TRTS (2 * SAMPLING_TIME)
#endif

#define TNOISE (u16)((((u16)(TNOISE_NS)) * ADV_TIM_CLK_MHz)/1000uL)

#if (TNOISE_NS > TRISE_NS)
  #define MAX_TNTR_NS TNOISE_NS
#else
  #define MAX_TNTR_NS TRISE_NS
#endif

#define TW_AFTER ((uint16_t)(((DEADTIME_NS+MAX_TNTR_NS)*ADV_TIM_CLK_MHz)/1000ul))
#define TW_BEFORE (((uint16_t)(((((uint16_t)(SAMPLING_TIME_NS)))*ADV_TIM_CLK_MHz)/1000ul))+1u)

#if (FLUX_WEAKENING_ENABLING == ENABLE)
#define FLUX_WEAKENING
#endif
#if (MTPA_ENABLING == ENABLE)
#define IPMSM_MTPA
#endif

#if (FEED_FORWARD_CURRENT_REG_ENABLING == ENABLE)
#define FEED_FORWARD_CURRENT_REGULATION
#endif

#ifdef MAX_MODULATION_81_PER_CENT
#define START_INDEX     41
#define MAX_MODULE      26541   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*81% 
#define MMITABLE {\
32508,32255,32008,31530,31299,30852,30636,30216,30012,29617,29426,29053,28872,\
28520,28349,28015,27853,27536,27382,27081,26934,26647,26507,26234,26101,25840,\
25712,25462,25340,25101,24984,24755,24643,24422,24315,24103,24000,23796,23696,\
23500,23404,23216,23123,22941,22851,22763,22589,22504,22336,22253,22091,22011,\
21854,21776,21624,21549,21401,21329,21186,21115,20976,20908,20773,20706,20575,\
20511,20383,20320,20196,20135,20015,19955,19838,19780,19666,19609,19498,19443,\
19334,19280,19175,19122,19019,18968,18867,18817,18719\
}
#endif

#ifdef MAX_MODULATION_83_PER_CENT
#define START_INDEX     44
#define MAX_MODULE      27196   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*83% 
#define MMITABLE {\
32291,32060,31613,31397,30977,30573,30377,29996,29811,29451,29276,28934,28768,\
28444,28286,27978,27827,27533,27390,27110,26973,26705,26574,26318,26069,25948,\
25709,25592,25363,25251,25031,24923,24711,24607,24404,24304,24107,24011,23821,\
23728,23545,23456,23279,23192,23021,22854,22772,22610,22530,22374,22297,22145,\
22070,21922,21850,21707,21636,21497,21429,21294,21227,21096,21032,20904,20778,\
20717,20595,20534,20416,20357,20241,20184,20071,20015,19905,19851,19743,19690,\
19585,19533,19431,19380,19280,19182\
}
#endif


#ifdef MAX_MODULATION_85_PER_CENT
#define START_INDEX     46
#define MAX_MODULE      27851   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*85%  
#define MMITABLE {\
32324,32109,31691,31489,31094,30715,30530,30170,29995,29654,29488,29163,29005,\
28696,28397,28250,27965,27825,27552,27418,27157,26903,26779,26535,26416,26182,\
26067,25842,25623,25515,25304,25201,24997,24897,24701,24605,24415,24230,24139,\
23960,23872,23699,23614,23446,23282,23201,23042,22964,22810,22734,22584,22437,\
22365,22223,22152,22014,21945,21811,21678,21613,21484,21421,21296,21234,21112,\
21051,20932,20815,20757,20643,20587,20476,20421,20312,20205,20152,20048,19996\
,19894,19844,19744,19645\
}
#endif



#ifdef MAX_MODULATION_87_PER_CENT
#define START_INDEX     48
#define MAX_MODULE      28507   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*87%  
#define MMITABLE {\
32559,32154,31764,31575,31205,31025,30674,30335,30170,29847,\
29689,29381,29083,28937,28652,28375,28239,27974,27844,27589,\
27342,27220,26983,26866,26637,26414,26305,26090,25984,25777,\
25575,25476,25280,25184,24996,24811,24720,24542,24367,24281,\
24112,24028,23864,23703,23624,23468,23391,23240,23091,23018,\
22874,22803,22662,22524,22456,22322,22191,22126,21997,21934,\
21809,21686,21625,21505,21446,21329,21214,21157,21045,20990,\
20880,20772,20719,20613,20561,20458,20356,20306,20207,20158,\
20109\
}
#endif

#ifdef MAX_MODULATION_89_PER_CENT
#define START_INDEX     50
#define MAX_MODULE      29162   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*89%
#define MMITABLE {\
32574,32197,32014,31656,31309,31141,30811,30491,30335,30030,\
29734,29589,29306,29031,28896,28632,28375,28249,28002,27881,\
27644,27412,27299,27076,26858,26751,26541,26336,26235,26037,\
25844,25748,25561,25378,25288,25110,24936,24851,24682,24517,\
24435,24275,24118,24041,23888,23738,23664,23518,23447,23305,\
23166,23097,22962,22828,22763,22633,22505,22442,22318,22196,\
22135,22016,21898,21840,21726,21613,21557,21447,21338,21284,\
21178,21074,21022,20919,20819,20769,20670,20573\
}
#endif

#ifdef MAX_MODULATION_91_PER_CENT
#define START_INDEX     52
#define MAX_MODULE      29817   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*91%
#define MMITABLE {\
32588,32411,32066,31732,31569,31250,30940,30789,30492,30205,\
29925,29788,29519,29258,29130,28879,28634,28395,28278,28048,\
27823,27713,27497,27285,27181,26977,26777,26581,26485,26296,\
26110,26019,25840,25664,25492,25407,25239,25076,24995,24835,\
24679,24602,24450,24301,24155,24082,23940,23800,23731,23594,\
23460,23328,23263,23135,23008,22946,22822,22701,22641,22522,\
22406,22291,22234,22122,22011,21956,21848,21741,21636,21584,\
21482,21380,21330,21231,21133,21037\
}
#endif


#ifdef MAX_MODULATION_92_PER_CENT
#define START_INDEX     54
#define MAX_MODULE      30145   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*92%
#define MMITABLE {\
32424,32091,31929,31611,31302,31002,30855,30568,30289,30017,\
29884,29622,29368,29243,28998,28759,28526,28412,28187,27968,\
27753,27648,27441,27238,27040,26942,26750,26563,26470,26288,\
26110,25935,25849,25679,25513,25350,25269,25111,24955,24803,\
24727,24579,24433,24361,24219,24079,23942,23874,23740,23609,\
23479,23415,23289,23165,23042,22982,22863,22745,22629,22572,\
22459,22347,22292,22183,22075,21970,21917,21813,21711,21610,\
21561,21462,21365,21268\
}
#endif


#ifdef MAX_MODULATION_93_PER_CENT
#define START_INDEX     55
#define MAX_MODULE      30473   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*93%
#define MMITABLE {\
32437,32275,31959,31651,31353,31207,30920,30642,30371,30107,\
29977,29723,29476,29234,29116,28883,28655,28433,28324,28110,\
27900,27695,27594,27395,27201,27011,26917,26733,26552,26375,\
26202,26116,25948,25783,25621,25541,25383,25228,25076,25001,\
24854,24708,24565,24495,24356,24219,24084,24018,23887,23758,\
23631,23506,23444,23322,23202,23083,23025,22909,22795,22683,\
22627,22517,22409,22302,22250,22145,22042,21941,21890,21791,\
21693,21596,21500\
}
#endif


#ifdef MAX_MODULATION_94_PER_CENT
#define START_INDEX     56
#define MAX_MODULE      30800   //root(Vd^2+Vq^2) <= MAX_MODULE = 32767*94%
#define MMITABLE {\
32607,32293,31988,31691,31546,31261,30984,30714,30451,30322,\
30069,29822,29581,29346,29231,29004,28782,28565,28353,28249,\
28044,27843,27647,27455,27360,27174,26991,26812,26724,26550,\
26380,26213,26049,25968,25808,25652,25498,25347,25272,25125,\
24981,24839,24699,24630,24494,24360,24228,24098,24034,23908,\
23783,23660,23600,23480,23361,23245,23131,23074,22962,22851,\
22742,22635,22582,22477,22373,22271,22170,22120,22021,21924,\
21827,21732\
}
#endif


#ifdef MAX_MODULATION_95_PER_CENT
#define START_INDEX     57
#define MAX_MODULE      31128   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*95%
#define MMITABLE {\
32613,32310,32016,31872,31589,31314,31046,30784,30529,30404,\
30158,29919,29684,29456,29343,29122,28906,28695,28488,28285,\
28186,27990,27798,27610,27425,27245,27155,26980,26808,26639,\
26473,26392,26230,26072,25917,25764,25614,25540,25394,25250,\
25109,24970,24901,24766,24633,24501,24372,24245,24182,24058,\
23936,23816,23697,23580,23522,23408,23295,23184,23075,23021,\
22913,22808,22703,22600,22499,22449,22349,22251,22154,22059,\
21964\
}
#endif


#ifdef MAX_MODULATION_96_PER_CENT
#define START_INDEX     58
#define MAX_MODULE      31456   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*96%
#define MMITABLE {\
32619,32472,32184,31904,31631,31365,31106,30853,30728,30484,\
30246,30013,29785,29563,29345,29238,29028,28822,28620,28423,\
28229,28134,27946,27762,27582,27405,27231,27061,26977,26811,\
26649,26489,26332,26178,26027,25952,25804,25659,25517,25376,\
25238,25103,25035,24903,24772,24644,24518,24393,24270,24210,\
24090,23972,23855,23741,23627,23516,23461,23352,23244,23138,\
23033,22930,22828,22777,22677,22579,22481,22385,22290,22196\
}
#endif


#ifdef MAX_MODULATION_97_PER_CENT
#define START_INDEX     60
#define MAX_MODULE      31783   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*97%
#define MMITABLE {\
32483,32206,31936,31672,31415,31289,31041,30799,30563,30331,\
30105,29884,29668,29456,29352,29147,28947,28750,28557,28369,\
28183,28002,27824,27736,27563,27393,27226,27062,26901,26743,\
26588,26435,26360,26211,26065,25921,25780,25641,25504,25369,\
25236,25171,25041,24913,24788,24664,24542,24422,24303,24186,\
24129,24015,23902,23791,23681,23573,23467,23362,23258,23206,\
23105,23004,22905,22808,22711,22616,22521,22429\
}
#endif


#ifdef MAX_MODULATION_98_PER_CENT
#define START_INDEX     61
#define MAX_MODULE      32111   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*98%
#define MMITABLE {\
32494,32360,32096,31839,31587,31342,31102,30868,30639,30415,\
30196,29981,29771,29565,29464,29265,29069,28878,28690,28506,\
28325,28148,27974,27803,27635,27470,27309,27229,27071,26916,\
26764,26614,26467,26322,26180,26039,25901,25766,25632,25500,\
25435,25307,25180,25055,24932,24811,24692,24574,24458,24343,\
24230,24119,24009,23901,23848,23741,23637,23533,23431,23331,\
23231,23133,23036,22941,22846,22753,22661\
}
#endif


#ifdef MAX_MODULATION_99_PER_CENT
#define START_INDEX     62
#define MAX_MODULE      32439   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*99%
#define MMITABLE {\
32635,32375,32121,31873,31631,31394,31162,30935,30714,30497,\
30284,30076,29872,29672,29574,29380,29190,29003,28820,28641,\
28464,28291,28122,27955,27791,27630,27471,27316,27163,27012,\
26864,26718,26575,26434,26295,26159,26024,25892,25761,25633,\
25569,25444,25320,25198,25078,24959,24842,24727,24613,24501,\
24391,24281,24174,24067,23963,23859,23757,23656,23556,23458,\
23361,23265,23170,23077,22984,22893\
}
#endif


#ifdef MAX_MODULATION_100_PER_CENT
#define START_INDEX     63
#define MAX_MODULE      32767   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*100%
#define MMITABLE {\
32767,32390,32146,31907,31673,31444,31220,31001,30787,30577,30371,\
30169,29971,29777,29587,29400,29217,29037,28861,28687,28517,\
28350,28185,28024,27865,27709,27555,27404,27256,27110,26966,\
26824,26685,26548,26413,26280,26149,26019,25892,25767,25643,\
25521,25401,25283,25166,25051,24937,24825,24715,24606,24498,\
24392,24287,24183,24081,23980,23880,23782,23684,23588,23493,\
23400,23307,23215,23125\
}
#endif

/*************** PI divisor  ***************/
#if (LOG2(SP_KPDIV)==-1)
  #error "Error: SP_KPDIV defined in Drive parameters.h must be power of two."
#else
  #define SP_KPDIV_LOG LOG2(SP_KPDIV)
#endif

#if (LOG2(SP_KIDIV)==-1)
  #error "Error: SP_KIDIV defined in Drive parameters.h must be power of two."
#else
  #define SP_KIDIV_LOG LOG2(SP_KIDIV)
#endif

#if (LOG2(SP_KDDIV)==-1)
  #error "Error: SP_KDDIV defined in Drive parameters.h must be power of two."
#else
  #define SP_KDDIV_LOG LOG2(SP_KDDIV)
#endif

#if (LOG2(TF_KPDIV)==-1)
  #error "Error: TF_KPDIV defined in Drive parameters.h must be power of two."
#else
  #define TF_KPDIV_LOG LOG2(TF_KPDIV)
#endif

#if (LOG2(TF_KIDIV)==-1)
  #error "Error: TF_KIDIV defined in Drive parameters.h must be power of two."
#else
  #define TF_KIDIV_LOG LOG2(TF_KIDIV)
#endif

#if (LOG2(TF_KDDIV)==-1)
  #error "Error: TF_KDDIV defined in Drive parameters.h must be power of two."
#else
  #define TF_KDDIV_LOG LOG2(TF_KDDIV)
#endif

#if (LOG2(FW_KPDIV)==-1)
  #error "Error: FW_KPDIV defined in Drive parameters.h must be power of two."
#else
  #define FW_KPDIV_LOG LOG2(FW_KPDIV)
#endif

#if (LOG2(FW_KIDIV)==-1)
  #error "Error: FW_KIDIV defined in Drive parameters.h must be power of two."
#else
  #define FW_KIDIV_LOG LOG2(FW_KIDIV)
#endif

#define PLL_KPDIV     16384
#define PLL_KIDIV     65535

#if (LOG2(PLL_KPDIV)==-1)
  #error "Error: PLL_KPDIV defined in Drive parameters.h must be power of two."
#else
  #define PLL_KPDIV_LOG LOG2(PLL_KPDIV)
#endif

#if (LOG2(PLL_KIDIV)==-1)
  #error "Error: PLL_KIDIV defined in Drive parameters.h must be power of two."
#else
  #define PLL_KIDIV_LOG LOG2(PLL_KIDIV)
#endif

#if (LOG2(F1)==-1)
  #error "Error: F1 defined in Drive parameters.h must be power of two."
#else
  #define F1_LOG LOG2(F1)
#endif

#if (LOG2(F2)==-1)
  #error "Error: F2 defined in Drive parameters.h must be power of two."
#else
  #define F2_LOG LOG2(F2)
#endif

#if (LOG2(STO_FIFO_DEPTH_DPP)==-1)
  #error "Error: STO_FIFO_DEPTH_DPP in Drive parameters.h must be power of two."
#else
  #define STO_FIFO_DEPTH_DPP_LOG LOG2(STO_FIFO_DEPTH_DPP)
#endif

#if (LOG2(CORD_FIFO_DEPTH_DPP)==-1)
  #error "Error: CORD_FIFO_DEPTH_DPP in Drive parameters.h must be power of two."
#else
  #define CORD_FIFO_DEPTH_DPP_LOG LOG2(CORD_FIFO_DEPTH_DPP)
#endif

#if (LOG2(HFI_PID_KPDIV)==-1)
  #error "Error: HFI_PID_KPDIV defined in Drive parameters.h must be power of two."
#else
  #define HFI_PID_KPDIV_LOG LOG2(HFI_PID_KPDIV)
#endif

#if (LOG2(HFI_PID_KIDIV)==-1)
  #error "Error: HFI_PID_KIDIV defined in Drive parameters.h must be power of two."
#else
  #define HFI_PID_KIDIV_LOG LOG2(HFI_PID_KIDIV)
#endif

#if (BUS_VOLTAGE_READING == ENABLE)
 #if ((OV_VOLTAGE_PROT_ENABLING != ENABLE) || (UV_VOLTAGE_PROT_ENABLING!= ENABLE))
  #error "Configuration not supported by this version of the library. Under and Over voltage protections (motor 1) must be enabled if bus voltage sensing is enabled."
 #else
  #define BUS_VOLTAGE_MEASUREMENT
 #endif
#endif

#if ((TEMPERATURE_READING == ENABLE) && (OV_TEMPERATURE_PROT_ENABLING == ENABLE))
 #define HEAT_SINK_TEMPERATURE_MEASUREMENT
#endif

#define VIRTUAL_HEAT_SINK_TEMPERATURE_VALUE   25u

#define TEMP_SW_FILTER_BW_FACTOR      250u
#define VQD_SW_FILTER_BW_FACTOR       128u
#define VBUS_SW_FILTER_BW_FACTOR      10u

#if (LOG2(VQD_SW_FILTER_BW_FACTOR)==-1)
  #error "Error: VQD_SW_FILTER_BW_FACTOR defined in Drive parameters.h must be power of two."
#else
  #define VQD_SW_FILTER_BW_FACTOR_LOG LOG2(VQD_SW_FILTER_BW_FACTOR)
#endif

#if (VBUS_ADC_SAMPLING_TIME == 1)
#define VBUS_SAMPLING_TIME  ADC_SampleTime_1Cycles5
#elif (VBUS_ADC_SAMPLING_TIME == 7)
#define VBUS_SAMPLING_TIME  ADC_SampleTime_7Cycles5
#elif (VBUS_ADC_SAMPLING_TIME == 13)
#define VBUS_SAMPLING_TIME  ADC_SampleTime_13Cycles5
#elif (VBUS_ADC_SAMPLING_TIME == 28)
#define VBUS_SAMPLING_TIME  ADC_SampleTime_28Cycles5
#else
#define VBUS_SAMPLING_TIME  ADC_SampleTime_1Cycles5
#endif

#if (TEMP_ADC_SAMPLING_TIME == 1)
#define TEMP_SAMPLING_TIME  ADC_SampleTime_1Cycles5
#elif (TEMP_ADC_SAMPLING_TIME == 7)
#define TEMP_SAMPLING_TIME  ADC_SampleTime_7Cycles5
#elif (TEMP_ADC_SAMPLING_TIME == 13)
#define TEMP_SAMPLING_TIME  ADC_SampleTime_13Cycles5
#elif (TEMP_ADC_SAMPLING_TIME == 28)
#define TEMP_SAMPLING_TIME  ADC_SampleTime_28Cycles5
#else
#define TEMP_SAMPLING_TIME  ADC_SampleTime_1Cycles5
#endif


#if ((defined STATE_OBSERVER_PLL) || (defined STATE_OBSERVER_CORDIC))
#define NO_SPEED_SENSORS
#endif

#if LCD_JOYSTICK_FUNCTIONALITY == ENABLE
#define LCD_FUNCTIONALITY
#endif

#if (defined STM32PERFORMANCEMD) || (defined STM32PERFORMANCELD)
#define STM32PERFORMANCE
#endif
#if defined STM32PERFORMANCEHD 
#define STM32HD
#endif
#if (defined STM32VALUEMD) || (defined STM32VALUELD)
#define STM32VALUE
#endif

#if (MTPA_ENABLING == ENABLE)
#define IPMSM_MTPA
#endif

#if (FEED_FORWARD_CURRENT_REG_ENABLING == ENABLE)
#define FEED_FORWARD_CURRENT_REGULATION
#endif

#if (FLUX_WEAKENING_ENABLING == ENABLE)
#define FLUX_WEAKENING
#endif

#if defined AUX_ENCODER
#define VIEW_ENCODER_FEEDBACK
#endif

#if defined AUX_HALL_SENSORS
#define VIEW_HALL_FEEDBACK
#endif

#if (DAC_FUNCTIONALITY == DISABLE)
#undef DAC_FUNCTIONALITY
#endif

#if (SERIAL_COMMUNICATION == DISABLE)
#undef SERIAL_COMMUNICATION
#endif

/* Number of MC object */

#ifdef SINGLEDRIVE
  #define MC_NUM 1
#endif

#ifdef DUALDRIVE
  #define MC_NUM 2
#endif

#if (defined(SINGLE_SHUNT) || defined(THREE_SHUNT))
#define PQD_CONVERSION_FACTOR (int32_t)(( 1000 * 3 * MCU_SUPPLY_VOLTAGE ) /\
             ( 1.732 * RSHUNT * AMPLIFICATION_GAIN ))
#endif

#if (defined(ICS_SENSORS))
#define PQD_CONVERSION_FACTOR (int32_t)(( 1000 * 3 * MCU_SUPPLY_VOLTAGE ) /\
             ( 1.732 * AMPLIFICATION_GAIN ))
#endif

/*********************** Usart remapping  ***********************/
#define USE_USART1 0
#define USE_USART2 1
#define USE_USART3 2

#define NO_REMAP_USART1       0
#define REMAP_USART1          1
#define NO_REMAP_USART2       2
#define REMAP_USART2          3
#define NO_REMAP_USART3       4
#define FULL_REMAP_USART3     5
#define PARTIAL_REMAP_USART3  6

#define GPIO_NoRemap_USARTx 0

#if (USART_SELECTION == USE_USART1)
  #define USART                   USART1
  #define USART_CLK               RCC_APB2Periph_USART1
  #define USART_IRQ               USART1_IRQn
  #define USART_IRQHandler        USART1_IRQHandler
  #if (USART_REMAPPING == NO_REMAP_USART1)
    #define USART_GPIO_REMAP      GPIO_NoRemap_USARTx
  #else
    #define USART_GPIO_REMAP      GPIO_Remap_USART1
  #endif
#endif

#if (USART_SELECTION == USE_USART2)
  #define USART                   USART2
  #define USART_CLK               RCC_APB1Periph_USART2
  #define USART_IRQ               USART2_IRQn
  #define USART_IRQHandler        USART2_IRQHandler
  #if (USART_REMAPPING == NO_REMAP_USART2)
    #define USART_GPIO_REMAP      GPIO_NoRemap_USARTx
  #else
    #define USART_GPIO_REMAP      GPIO_Remap_USART2
  #endif
#endif

#if (USART_SELECTION == USE_USART3)
  #define USART                   USART3
  #define USART_CLK               RCC_APB1Periph_USART3
  #define USART_IRQ               USART3_IRQn
  #define USART_IRQHandler        USART3_IRQHandler
  #if (USART_REMAPPING == NO_REMAP_USART3)
    #define USART_GPIO_REMAP      GPIO_NoRemap_USARTx
  #elif (USART_REMAPPING == FULL_REMAP_USART3)
    #define USART_GPIO_REMAP      GPIO_FullRemap_USART3
  #else
    #define USART_GPIO_REMAP      GPIO_PartialRemap_USART3
  #endif
#endif

#define NVIC_USART_INIT_STR \
const NVIC_InitTypeDef NVICInitHW_str =\
{\
  USART_IRQ,  /* NVIC_IRQChannel */\
  USART_PRE_EMPTION_PRIORITY,            /* NVIC_IRQChannelPreemptionPriority */\
  USART_SUB_PRIORITY,            /* NVIC_IRQChannelSubPriority */\
  (FunctionalState)(ENABLE)        /* NVIC_IRQChannelCmd */\
};

/****** Prepares the UI configurations according the MCconfxx settings ********/
#ifdef LCD_FUNCTIONALITY
  #define LCD_ENABLE |OPT_LCD
#else
  #define LCD_ENABLE 
#endif

#ifdef SERIAL_COMMUNICATION
  #define COM_ENABLE |OPT_COM
#else
  #define COM_ENABLE
#endif

#ifdef DAC_FUNCTIONALITY
  #ifdef STM32VALUE
    #define DAC_ENABLE |OPT_DAC
    #define DAC_OPTION OPT_DAC
  #endif
  #ifdef STM32PERFORMANCE
    #define DAC_ENABLE |OPT_DACT
    #define DAC_OPTION OPT_DACT
  #endif
  #ifdef STM32HD
    #define DAC_ENABLE |OPT_DAC
    #define DAC_OPTION OPT_DAC
  #endif
  #define DAC_OP_ENABLE |UI_CFGOPT_DAC
#else
  #define DAC_ENABLE
  #define DAC_OP_ENABLE
#endif

/* Motor 1 settings */
#ifdef FLUX_WEAKENING
  #define FW_ENABLE |UI_CFGOPT_FW
#else
  #define FW_ENABLE
#endif

#ifdef DIFFERENTIAL_TERM_ENABLED
  #define DIFFTERM_ENABLE |UI_CFGOPT_SPEED_KD|UI_CFGOPT_Iq_KD|UI_CFGOPT_Id_KD
#else
  #define DIFFTERM_ENABLE
#endif

/* Sensors setting */
#ifdef STATE_OBSERVER_PLL
  #define MAIN_SCFG UI_SCODE_STO_PLL
#endif

#ifdef STATE_OBSERVER_CORDIC
  #define MAIN_SCFG UI_SCODE_STO_CR
#endif

#ifdef AUX_STATE_OBSERVER_PLL
  #define AUX_SCFG UI_SCODE_STO_PLL
#endif

#ifdef AUX_STATE_OBSERVER_CORDIC
  #define AUX_SCFG UI_SCODE_STO_CR
#endif

#ifdef ENCODER
  #define MAIN_SCFG UI_SCODE_ENC
#endif

#ifdef VIEW_ENCODER_FEEDBACK
  #define AUX_SCFG UI_SCODE_ENC
#endif

#ifdef HALL_SENSORS
  #define MAIN_SCFG UI_SCODE_HALL
#endif

#ifdef VIEW_HALL_FEEDBACK
  #define AUX_SCFG UI_SCODE_HALL
#endif

#ifndef AUX_SCFG
#define AUX_SCFG 0x0
#endif

#if defined(PLLTUNING)
  #define PLLTUNING_ENABLE |UI_CFGOPT_PLLTUNING
#else
  #define PLLTUNING_ENABLE
#endif

#if defined(PFC_ENABLED)
  #define UI_CFGOPT_PFC_ENABLE |UI_CFGOPT_PFC
#else
  #define UI_CFGOPT_PFC_ENABLE
#endif

/******************************************************************************* 
  * UI configurations settings. It can be manually overwritten if special 
  * configuartion is required. 
*******************************************************************************/

/* Base configuration of UI */
#define UI_INIT_CFG ( OPT_NONE LCD_ENABLE COM_ENABLE DAC_ENABLE )

/* Specific options of UI */
#define UI_CONFIG_M1 ( UI_CFGOPT_NONE DAC_OP_ENABLE FW_ENABLE DIFFTERM_ENABLE \
  | (MAIN_SCFG << MAIN_SCFG_POS) | (AUX_SCFG << AUX_SCFG_POS) | UI_CFGOPT_SETIDINSPDMODE PLLTUNING_ENABLE UI_CFGOPT_PFC_ENABLE | UI_CFGOPT_PLLTUNING)

#ifdef SINGLEDRIVE
  #define UI_CONFIG_M2
#endif

/* Push button definitions */
#define BUTTON_KEY_ACTIVE   Bit_RESET
#define BUTTON_KEY_INACTIVE Bit_SET
#define JOYSTIK_ACTIVE      Bit_RESET
#define JOYSTIK_INACTIVE    Bit_SET

/* Below Function prototype definitions can be removed adding the inclusion of 
Timebase.h in stm32f10x_MC_it.c */
void TB_SerialCommunicationTimeOutStop(void);
void TB_SerialCommunicationTimeOutStart(void);

#endif /*__PARAMETERS_CONVERSION_F10X_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
