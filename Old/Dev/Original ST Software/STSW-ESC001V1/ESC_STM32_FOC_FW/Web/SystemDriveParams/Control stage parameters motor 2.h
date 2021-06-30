/**
  ******************************************************************************
  * @file    Control stage parameters motor 2.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains motor parameters needed by STM32 PMSM MC FW  
  *                library v4.3.0
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
#ifndef __CONTROL_STAGE_PARAMETERS_MOTOR2_H
#define __CONTROL_STAGE_PARAMETERS_MOTOR2_H

/* Maximum modulation index */
#define MAX_MODULATION_95_PER_CENT2

/************************ DIGITAL I/O DEFINITION SECTION  *********************/
/* PWM Timer section */
#define PWM_TIMER_SELECTION2             PWM_TIM8 /* PWM_TIM1 or PWM_TIM8 */ 
#define PWM_TIMER_REMAPPING2             PWM_NO_REMAP /* PWM_NO_REMAP,  
                                                         PWM_FULL_REMAP, 
                                                         PWM_PARTIAL_REMAP */
#define PHASE_UH_GPIO_PORT2              GPIOC                                                                            
#define PHASE_UH_GPIO_PIN2               GPIO_Pin_6                                                                        
#define PHASE_VH_GPIO_PORT2              GPIOC                                                                                                                                                    
#define PHASE_VH_GPIO_PIN2               GPIO_Pin_7
#define PHASE_WH_GPIO_PORT2              GPIOC                                                                         
#define PHASE_WH_GPIO_PIN2               GPIO_Pin_8
#define PHASE_UL_GPIO_PORT2              GPIOA                                                                        
#define PHASE_UL_GPIO_PIN2               GPIO_Pin_7
#define PHASE_VL_GPIO_PORT2              GPIOB                                                                
#define PHASE_VL_GPIO_PIN2               GPIO_Pin_14
#define PHASE_WL_GPIO_PORT2              GPIOB                                                        
#define PHASE_WL_GPIO_PIN2               GPIO_Pin_15
#define EMERGENCY_STOP_GPIO_PORT2        GPIOA
#define EMERGENCY_STOP_GPIO_PIN2         GPIO_Pin_6

#define BKIN_MODE2                       NONE /* NONE, INT_MODE, EXT_MODE */
#define BKIN2_MODE2                      NONE /* NONE, INT_MODE, EXT_MODE */
#define EMERGENCY2_STOP_GPIO_PORT2       GPIOA
#define EMERGENCY2_STOP_GPIO_PIN2        GPIO_Pin_6
#define PHASE_UH_GPIO_AF2                GPIO_AF_4
#define PHASE_VH_GPIO_AF2                GPIO_AF_4
#define PHASE_WH_GPIO_AF2                GPIO_AF_4
#define PHASE_UL_GPIO_AF2                GPIO_AF_4
#define PHASE_VL_GPIO_AF2                GPIO_AF_2
#define PHASE_WL_GPIO_AF2                GPIO_AF_2
#define BRKIN_GPIO_AF2                   GPIO_AF_4
#define BRKIN2_GPIO_AF2                  GPIO_AF_2
/* Hall timer section */
#define HALL_TIMER_SELECTION2            HALL_TIM4  /* HALL_TIM2...HALL_TIM5 */ 
#define HALL_TIMER_REMAPPING2            NO_REMAP_TIM4  /* NO_REMAP, FULL_REMAP, 
                                                           PARTIAL_REMAP */
#define H1_GPIO_PORT2                    GPIOB
#define H2_GPIO_PORT2                    GPIOD
#define H3_GPIO_PORT2                    GPIOB

#define H1_GPIO_PIN2                     GPIO_Pin_6
#define H2_GPIO_PIN2                     GPIO_Pin_13
#define H3_GPIO_PIN2                     GPIO_Pin_8

/* Encoder timer selection */
#define ENC_TIMER_SELECTION2             ENC_TIM4  /* ENC_TIM2...RNC_TIM5 */
#define ENC_TIMER_REMAPPING2             NO_REMAP_TIM4  /* NO_REMAP, FULL_REMAP, 
                                                          PARTIAL_REMAP */
#define ENC_A_GPIO_PORT2                 GPIOB
#define ENC_B_GPIO_PORT2                 GPIOD
#define ENC_A_GPIO_PIN2                  GPIO_Pin_6
#define ENC_B_GPIO_PIN2                  GPIO_Pin_13

/* Digital Outputs */
#define R_BRAKE_GPIO_PORT2               GPIOF
#define R_BRAKE_GPIO_PIN2                GPIO_Pin_10
#define OV_CURR_BYPASS_GPIO_PORT2        GPIOF
#define OV_CURR_BYPASS_GPIO_PIN2         GPIO_Pin_10
#define INRUSH_CURRLIMIT_GPIO_PORT2      GPIOD
#define INRUSH_CURRLIMIT_GPIO_PIN2       GPIO_Pin_15

/************************ ANALOG I/O DEFINITION SECTION  *********************/
/** Currents reading  **/
/* Only for three shunt resistors and ICS cases */
#define ADC_1_PERIPH2                    ADC1
#define ADC_2_PERIPH2                    ADC2
#define PHASE_U_CURR_ADC2                ADC1_2
#define PHASE_U_CURR_CHANNEL2            ADC_Channel_14
#define PHASE_U_GPIO_PORT2               GPIOC
#define PHASE_U_GPIO_PIN2                GPIO_Pin_4
#define PHASE_V_CURR_ADC2                ADC1_2
#define PHASE_V_CURR_CHANNEL2            ADC_Channel_3
#define PHASE_V_GPIO_PORT2               GPIOA
#define PHASE_V_GPIO_PIN2                GPIO_Pin_3
/* Only for three shunts case */
#define PHASE_W_CURR_ADC2                ADC1_2
#define PHASE_W_CURR_CHANNEL2            ADC_Channel_8
#define PHASE_W_GPIO_PORT2               GPIOB
#define PHASE_W_GPIO_PIN2                GPIO_Pin_0       
/* Only for 1 shunt resistor case */
#define ADC_PERIPH2                      ADC1
#define PHASE_CURRENTS_CHANNEL2          ADC_Channel_1
#define PHASE_CURRENTS_GPIO_PORT2        GPIOA
#define PHASE_CURRENTS_GPIO_PIN2         GPIO_Pin_1   

/* Common */
#define ADC_AHBPERIPH2                   RCC_AHBPeriph_ADC12
#define ADC_CLOCK_WB_FREQ2               21
#define ADC_CLOCK_WB_DIV2                0
#define CURR_SAMPLING_TIME2              3  /*!< Sampling time duration  
                                                           in ADC clock cycles (1 for  
                                                           1.5, 7 for 7.5, ...) */

/** Bus and temperature readings **/
#define REGCONVADC2                      ADC1

#define VBUS_ADC2                        ADC1_2
#define VBUS_CHANNEL2                    ADC_Channel_13
#define VBUS_GPIO_PORT2                  GPIOC
#define VBUS_GPIO_PIN2                   GPIO_Pin_3   
#define VBUS_ADC_SAMPLING_TIME2          56

#define TEMP_FDBK_ADC2                    ADC1_2                  
#define TEMP_FDBK_CHANNEL2                ADC_Channel_2
#define TEMP_FDBK_GPIO_PORT2              GPIOA
#define TEMP_FDBK_GPIO_PIN2               GPIO_Pin_2  
#define TEMP_ADC_SAMPLING_TIME2           28

/* OPAMP Settings */

#define USE_INTERNAL_OPAMP2                     DISABLE

#define OPAMP1_SELECTION2                       OPAMP_Selection_OPAMP3
#define OPAMP1_INVERTINGINPUT_MODE2             INT_MODE
#define OPAMP1_INVERTINGINPUT2                  OPAMP3_InvertingInput_PGA
#define OPAMP1_INVERTINGINPUT_GPIO_PORT2        GPIOB
#define OPAMP1_INVERTINGINPUT_GPIO_PIN2         GPIO_Pin_2
#define OPAMP1_NONINVERTINGINPUT_PHA2           OPAMP1_NonInvertingInput_PA1    //dummy generated by STMCWB
#define OPAMP1_NONINVERTINGINPUT_PHA_GPIO_PORT2 GPIOA                           //dummy generated by STMCWB
#define OPAMP1_NONINVERTINGINPUT_PHA_GPIO_PIN2  GPIO_Pin_1                      //dummy generated by STMCWB
#define OPAMP1_NONINVERTINGINPUT_PHB2           OPAMP1_NonInvertingInput_PA7    //dummy generated by STMCWB
#define OPAMP1_NONINVERTINGINPUT_PHB_GPIO_PORT2 GPIOA                           //dummy generated by STMCWB
#define OPAMP1_NONINVERTINGINPUT_PHB_GPIO_PIN2  GPIO_Pin_7                      //dummy generated by STMCWB
#define OPAMP1_OUT_GPIO_PORT2                   GPIOB
#define OPAMP1_OUT_GPIO_PIN2                    GPIO_Pin_1

#define OPAMP2_SELECTION2                       OPAMP_Selection_OPAMP4
#define OPAMP2_INVERTINGINPUT_MODE2             INT_MODE
#define OPAMP2_INVERTINGINPUT2                  OPAMP4_InvertingInput_PGA
#define OPAMP2_INVERTINGINPUT_GPIO_PORT2        GPIOB
#define OPAMP2_INVERTINGINPUT_GPIO_PIN2         GPIO_Pin_10
#define OPAMP2_NONINVERTINGINPUT_PHA2           OPAMP2_NonInvertingInput_PD14   //dummy generated by STMCWB
#define OPAMP2_NONINVERTINGINPUT_PHA_GPIO_PORT2 GPIOD                           //dummy generated by STMCWB
#define OPAMP2_NONINVERTINGINPUT_PHA_GPIO_PIN2  GPIO_Pin_14                     //dummy generated by STMCWB
#define OPAMP2_NONINVERTINGINPUT_PHB2           OPAMP2_NonInvertingInput_PD14   //dummy generated by STMCWB
#define OPAMP2_NONINVERTINGINPUT_PHB_GPIO_PORT2 GPIOD                           //dummy generated by STMCWB
#define OPAMP2_NONINVERTINGINPUT_PHB_GPIO_PIN2  GPIO_Pin_14                     //dummy generated by STMCWB
#define OPAMP2_NONINVERTINGINPUT_PHC2           OPAMP2_NonInvertingInput_PD14   //dummy generated by STMCWB
#define OPAMP2_NONINVERTINGINPUT_PHC_GPIO_PORT2 GPIOD                           //dummy generated by STMCWB
#define OPAMP2_NONINVERTINGINPUT_PHC_GPIO_PIN2  GPIO_Pin_14                     //dummy generated by STMCWB
#define OPAMP2_OUT_GPIO_PORT2                   GPIOB
#define OPAMP2_OUT_GPIO_PIN2                    GPIO_Pin_12

/* Only for 1 shunt resistor case */
#define OPAMP_SELECTION2                        OPAMP_Selection_OPAMP3
#define OPAMP_INVERTINGINPUT_MODE2              INT_MODE
#define OPAMP_INVERTINGINPUT2                   OPAMP3_InvertingInput_PGA
#define OPAMP_INVERTINGINPUT_GPIO_PORT2         GPIOB
#define OPAMP_INVERTINGINPUT_GPIO_PIN2          GPIO_Pin_2
#define OPAMP_NONINVERTINGINPUT2                OPAMP3_NonInvertingInput_PB13
#define OPAMP_NONINVERTINGINPUT_GPIO_PORT2      GPIOB
#define OPAMP_NONINVERTINGINPUT_GPIO_PIN2       GPIO_Pin_13
#define OPAMP_OUT_GPIO_PORT2                    GPIOB
#define OPAMP_OUT_GPIO_PIN2                     GPIO_Pin_1

/* OPAMP common settings*/
#define OPAMP_PGAGAIN2                          OPAMP_OPAMP_PGAGain_2
#define OPAMP_PGACONNECT2                       OPAMP_PGAConnect_No

/* COMP Settings */

#define INTERNAL_OVERCURRENTPROTECTION2    DISABLE
#define OCPREF2                            23830

#define INTERNAL_OVERVOLTAGEPROTECTION2    DISABLE
#define OVPREF2                            23830

/* Only for 1 shunt resistor case */
#define OCP_SELECTION2                     COMP_Selection_COMP5
#define OCP_INVERTINGINPUT_MODE2           INT_MODE
#define OCP_INVERTINGINPUT2                COMPX_InvertingInput_VREF
#define OCP_INVERTINGINPUT_GPIO_PORT2      GPIOA
#define OCP_INVERTINGINPUT_GPIO_PIN2       GPIO_Pin_4
#define OCP_NONINVERTINGINPUT2             COMP5_NonInvertingInput_PB13
#define OCP_NONINVERTINGINPUT_GPIO_PORT2   GPIOB
#define OCP_NONINVERTINGINPUT_GPIO_PIN2    GPIO_Pin_13
#define OCP_OUTPUT_MODE2                   INT_MODE
#define OCP_OUTPUT2                        COMP_Output_TIM8BKIN2
#define OCP_OUTPUT_GPIO_PORT2              GPIOC
#define OCP_OUTPUT_GPIO_PIN2               GPIO_Pin_7
#define OCP_OUTPUT_GPIO_AF2                GPIO_AF_7
#define OCP_OUTPUTPOL2                     COMP_OutputPol_NonInverted

#define OCPA_SELECTION2                    COMP_Selection_COMP4
#define OCPA_INVERTINGINPUT_MODE2          INT_MODE
#define OCPA_INVERTINGINPUT2               COMPX_InvertingInput_VREF
#define OCPA_INVERTINGINPUT_GPIO_PORT2     GPIOA
#define OCPA_INVERTINGINPUT_GPIO_PIN2      GPIO_Pin_4
#define OCPA_NONINVERTINGINPUT2            COMP4_NonInvertingInput_PB0
#define OCPA_NONINVERTINGINPUT_GPIO_PORT2  GPIOB
#define OCPA_NONINVERTINGINPUT_GPIO_PIN2   GPIO_Pin_0
#define OCPA_OUTPUT_MODE2                  INT_MODE
#define OCPA_OUTPUT2                       COMP_Output_TIM8BKIN2
#define OCPA_OUTPUT_GPIO_PORT2             GPIOB
#define OCPA_OUTPUT_GPIO_PIN2              GPIO_Pin_1
#define OCPA_OUTPUT_GPIO_AF2               GPIO_AF_8
#define OCPA_OUTPUTPOL2                    COMP_OutputPol_NonInverted

#define OCPB_SELECTION2                    COMP_Selection_COMP5
#define OCPB_INVERTINGINPUT_MODE2          INT_MODE
#define OCPB_INVERTINGINPUT2               COMPX_InvertingInput_VREF
#define OCPB_INVERTINGINPUT_GPIO_PORT2     GPIOA
#define OCPB_INVERTINGINPUT_GPIO_PIN2      GPIO_Pin_4
#define OCPB_NONINVERTINGINPUT2            COMP5_NonInvertingInput_PB13
#define OCPB_NONINVERTINGINPUT_GPIO_PORT2  GPIOB
#define OCPB_NONINVERTINGINPUT_GPIO_PIN2   GPIO_Pin_13
#define OCPB_OUTPUT_MODE2                  INT_MODE
#define OCPB_OUTPUT2                       COMP_Output_TIM8BKIN2
#define OCPB_OUTPUT_GPIO_PORT2             GPIOC
#define OCPB_OUTPUT_GPIO_PIN2              GPIO_Pin_7
#define OCPB_OUTPUT_GPIO_AF2               GPIO_AF_7
#define OCPB_OUTPUTPOL2                    COMP_OutputPol_NonInverted

#define OCPC_SELECTION2                    COMP_Selection_COMP6
#define OCPC_INVERTINGINPUT_MODE2          INT_MODE
#define OCPC_INVERTINGINPUT2               COMPX_InvertingInput_VREF
#define OCPC_INVERTINGINPUT_GPIO_PORT2     GPIOA
#define OCPC_INVERTINGINPUT_GPIO_PIN2      GPIO_Pin_4
#define OCPC_NONINVERTINGINPUT2            COMP6_NonInvertingInput_PB11
#define OCPC_NONINVERTINGINPUT_GPIO_PORT2  GPIOB
#define OCPC_NONINVERTINGINPUT_GPIO_PIN2   GPIO_Pin_11
#define OCPC_OUTPUT_MODE2                  INT_MODE
#define OCPC_OUTPUT2                       COMP_Output_TIM8BKIN2
#define OCPC_OUTPUT_GPIO_PORT2             GPIOA
#define OCPC_OUTPUT_GPIO_PIN2              GPIO_Pin_10
#define OCPC_OUTPUT_GPIO_AF2               GPIO_AF_8
#define OCPC_OUTPUTPOL2                    COMP_OutputPol_NonInverted
                                                              
#define OVP_SELECTION2                     COMP_Selection_COMP1
#define OVP_INVERTINGINPUT_MODE2           INT_MODE
#define OVP_INVERTINGINPUT2                COMPX_InvertingInput_VREF
#define OVP_INVERTINGINPUT_GPIO_PORT2      GPIOA
#define OVP_INVERTINGINPUT_GPIO_PIN2       GPIO_Pin_4
#define OVP_NONINVERTINGINPUT2             COMP1_NonInvertingInput_PA1
#define OVP_NONINVERTINGINPUT_GPIO_PORT2   GPIOA
#define OVP_NONINVERTINGINPUT_GPIO_PIN2    GPIO_Pin_1
#define OVP_OUTPUT_MODE2                   EXT_MODE
#define OVP_OUTPUT2                        COMP_Output_TIM8BKIN
#define OVP_OUTPUT_GPIO_PORT2              GPIOA
#define OVP_OUTPUT_GPIO_PIN2               GPIO_Pin_0
#define OVP_OUTPUT_GPIO_AF2                GPIO_AF_8
#define OVP_OUTPUTPOL2                     COMP_OutputPol_NonInverted

#define HIGH_SIDE_BRAKE_STATE2             TURN_OFF /*!< TURN_OFF, TURN_ON */
#define LOW_SIDE_BRAKE_STATE2              TURN_OFF /*!< TURN_OFF, TURN_ON */

#define BKIN1_FILTER2                      0
#define BKIN2_FILTER2                      0

#define OCP_FILTER2                        COMP_Mode_HighSpeed
#define OVP_FILTER2                        COMP_Mode_HighSpeed

#define SW_OV_CURRENT_PROT_ENABLING2	   ENABLE /*!< Over-current detection 
                                                         enabling */

/* ##@@_USER_CODE_START_##@@ */
/* ##@@_USER_CODE_END_##@@ */

#endif /*__CONTROL_STAGE_PARAMETERS_MOTOR2_H*/
/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
