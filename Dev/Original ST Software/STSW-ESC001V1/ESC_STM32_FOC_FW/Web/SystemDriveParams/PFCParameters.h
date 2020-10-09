/**
  ******************************************************************************
  * @file    PFCParameters.h
  * @author  STMCWB ver.4.3.0.16265
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @project F303PFC_dual_working_230Vac.stmcx
  * @path    C:\VSS\MC\3PH\0051.15\SOFTWARE\FIRMWARE\trunk\ProtectedSources
  * @brief   This file contains PFC parameters needed by STM32 PMSM MC FW  
  *                 library v4.3.0
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
#ifndef __PFC_PARAMETERS_H
#define __PFC_PARAMETERS_H

/* Includes ------------------------------------------------------------------*/

/************************* SYSTEM CONFIGURATION ******************/

#ifdef CPU_CLK_72_MHZ
#define TIM_CLK       SYSCLK_FREQ_72MHz

#elif defined CPU_CLK_64_MHZ
#define TIM_CLK       SYSCLK_FREQ_64MHz

#elif defined CPU_CLK_56_MHZ
#define TIM_CLK       SYSCLK_FREQ_56MHz

#elif defined CPU_CLK_48_MHZ
#define TIM_CLK       SYSCLK_FREQ_48MHz

#elif defined CPU_CLK_24_MHZ
#define TIM_CLK       SYSCLK_FREQ_24MHz
#else
#endif


  /* Current reading A/D Conversions initialization --------------------------*/

#define PFC_ADC 			ADC4
#define ADCINITIALIZED      FALSE
  
#define ICHANNEL			ADC_Channel_3 /*!< ADC channel used for current sampling*/ 
#define IPORT				GPIOB /*!< GPIO port used by ICHANNEL*/
#define IPIN				GPIO_Pin_12 /*!< GPIO pin used by ICHANNEL*/ 
#define ISAMPLINGTIME		ADC_SampleTime_7Cycles5 /*!< Sampling time used to convert ICHANNEL*/
#define ISAMPLINGTIMEREAL   7.5 /*!< sampling time, this time expressed as real number*/
#define VMCHANNEL			ADC_Channel_4 /*!< ADC channel used for conversion of Mains Voltage*/
#define VMPORT				GPIOB /*!< GPIO port used by VMCHANNEL*/
#define VMPIN				GPIO_Pin_14 /*!< GPIO pin used by VMCHANNEL*/ 
#define VMSAMPLINGTIME		ADC_SampleTime_7Cycles5 /*!< Sampling time used to convert VMCHANNEL*/ 

/* PWM generation parameters --------------------------------------------------*/
#define PWMFREQ				40000 /*!< PFC PWM frequency, Hertz*/
#define TIMREMAPPING		GPIO_PartialRemap_TIM3	//GPIO_PartialRemap_TIM3      
                                         /*!< it remaps TIMx I/Os
                                                                  if dual motor control is enabled only
                                                                  GPIO_PartialRemap_TIM3 is allowed*/
#define PWMPOLARITY			TIM_OCPolarity_High    /*!< PMW output polarity, it must be 
                                        TIM_OCPolarity_High or TIM_OCPolarity_Low
										according to power device driver */
#define PWMPORT				GPIOB    /*!< PFC PWM output port (as after re-mapping). 
                                         It must be GPIOx x= A, B, ... It must be associated to TIM16 CH1*/
#define PWM_AF				GPIO_AF_1    /*!< PFC PWM alternate function output port. 
                                         It must be GPIO_AF_x x= 0, 1, .. according to the GPIO port and Pin */
#define PWMPIN				GPIO_Pin_8    /*!< PFC PWM output pin (as after re-mapping).
                                        It must be GPIO_Pin_x x= 0, 1, ...It must be associated to TIM16 CH1*/

#define ETRFAULT			DISABLE 	/*!< It enable/disable the management of 
                                        the ETR input, instantaneously 
                                        stopping PWM generation. It must be 
                                        either equal to ENABLE or DISABLE.
										If ETR is DISABLE, then an EXTI line
										should be assigned using definitions below*/
#define EXTIPOLARITY        EXTI_Trigger_Falling  /*!< EXTI input polarity, according to 
                                                  hardware protection polarity:
                                                  EXTI_Trigger_Falling,
                                                  EXTI_Trigger_Rising*/

#define PFC_BKIN_MODE		INT_MODE	   /*!< NONE, INT_MODE, EXT_MODE */
												  
#define BKINPOLARITY    	TIM_BreakPolarity_High  /*TIM_BreakPolarity_Low *1 if fault is on rising edge, 0
                                        if fault is on falling edge*/										
#define BKINPORT			GPIOB        /*!< Fault GPIO input 
                                        port (if used, after re-mapping). 
                                        It must be GPIOx x= A, B, ...*/
#define BKINPIN			    GPIO_Pin_5      /*!< Fault GPIO input pin 
                                        (if used, after re-mapping). It must be 
                                        GPIO_Pin_x x= 0, 1, ...*/
#define BKINFILTER          15      // Fdts/2, N=8
#define EXTIPORTSOURCE      GPIO_PortSourceGPIOB /*!< Fault GPIO input port, EXTI configuration*/
#define EXTIPINSOURCE       GPIO_PinSource5 /*!< Fault GPIO input pin, EXTI configuration*/
#define EXTILINE            EXTI_Line5      /*!< EXTI Line*/
#define EXTIIRQN            EXTI5_IRQn  /*!< EXTI interrupt no. */
#define ETRFILTER			0	/*!< Time filter applied to validate ETR.
                                This value defines the frequency used to sample 
                                ETR input and the length of the digital
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
/* VMAINS syncronization parameters ----------------------------------------*/										
#define SYNCPOLARITY		TIM_ICPolarity_Falling    /*!< Syncronization circuit polarity, it must be 
                                     TIM_ICPolarity_Rising or TIM_ICPolarity_Falling */
#define SYNCPORT			GPIOB    /*!< SYNC input port (as after re-mapping). 
                                         It must be GPIOx x= A, B, ...
										 It must be associated to TIM4 CH2*/
#define SYNC_AF				GPIO_AF_2    /*!< SYNC input port (as after re-mapping). 
                                         It must be GPIO_AF_x x= 0, 1, ...
										 It must be associated to TIM4 CH2*/
#define SYNCPIN				GPIO_Pin_7	 /*!< SYNC input pin (as after re-mapping).
                                        It must be GPIO_Pin_x x= 0, 1, ...
										It must be associated to TIM4 CH2*/ 
#define SYNCFILTER			93    /*!< Time filter applied to validate SYNC.
                                This value defines the frequency used to sample 
                                SYNC input and the length of the digital
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
  
  /* PFC Control Parameters --------------------------------------------------*/  
#define CURRCTRLFREQUENCY	40000 /*!< Current control frequency, Hertz */
#define VOLTCTRLFREQUENCY	100 /*!< Voltage control frequency, Hertz */
#define NOMINALCURRENT      36033 /*!< Output nominal current in s16A 20000-4.1A*/

/* Gains values for current and voltage control loops */
#define PID_CURR_KP_DEFAULT	1000      
#define PID_CURR_KI_DEFAULT	500
#define PID_CURR_KD_DEFAULT	1
#define PID_VOLT_KP_DEFAULT	30000
#define PID_VOLT_KI_DEFAULT	150
#define PID_VOLT_KD_DEFAULT	1

/* Current/Voltage control loop gains dividers*/
#define CURR_KPDIV          2048
#define CURR_KIDIV          2048
#define CURR_KDDIV          16384
#define CURRDIFFERENTIAL_TERM_ENABLING  DISABLE
#define VOLT_KPDIV          128
#define VOLT_KIDIV          4096
#define VOLT_KDDIV          16384
#define VOLTDIFFERENTIAL_TERM_ENABLING  DISABLE

#define STARTUPDURATION		300 	/*!< Duration of PFC startup, ms */
#define VOLTAGEREFERENCE	340    /*!< Boost DC Voltage reference, Volt */
#define MAINSFREQ 			500 /*!< AC Mains frequency, 0.1 Hertz */
#define MAINSFREQHYST		50				 /*!< Allowed AC Mains frequency variation, 0.1 Hertz */
#define MAINSFREQLOWTH		(MAINSFREQ - MAINSFREQHYST) /*!< Min AC Mains frequency allowed, 0.1 Hz */
#define MAINSFREQHITH		(MAINSFREQ + MAINSFREQHYST) /*!< Max AC Mains frequency allowed, 0.1 Hz */

#define VMAINSAMPLLOWTH		100    /*!< Min Mains voltage amplitude allowed, Volt */
#define VMAINSAMPLHITH		265	/*!< Max Mains voltage amplitude allowed, Volt */
#define OUTPUTPOWERACTIVATION	250 /*!< Output power for PFC activation, Watt */
#define OUTPUTPOWERDEACTIVATION 50 /*!< Output power for PFC deactivation, Watt */
#define SWOVERVOLTAGETH		360    /*!< Software OverVoltage fault threshold, Volts */
#define SWOVERCURRENTTH		45081    /*!< Software OverCurrent fault threshold, expressed in s16A */

#define PROPDELAYON         2550    /*!< PFC switch turn-on latency, ns*/            
#define PROPDELAYOFF        2550	/*!< PFC switch turn-off latency, ns*/
#define VMAINS_PARTITIONING_FACTOR	201 /*!< It expresses how 
                                                       much the VMains is attenuated  
                                                       before being converted into 
                                                       digital value */

#define PFCSHUNTRESISTOR    0.050	/*!< PFC current sensing shunt resistor, Ohm */
#define PFCAMPLGAIN         2.76	/*!< PFC current sensing amplification network gain */


#define OCP_PFC_SELECTION        COMP_Selection_COMP5                 /*!< Internal comparator used for protection. 
                                   It must be
                                   COMP_Selection_COMPx x = 1,2,3,4,5,6,7.*/
#define OCP_PFC_INVERTINGINPUT_MODE  INT_MODE       /*!< COMPx inverting input mode. It must be either 
                                   equal to EXT_MODE or INT_MODE. */
#define OCP_PFC_INVERTINGINPUT     COMPX_InvertingInput_DAC2    /*!< COMPx inverting input pin. It must be one of
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
#define OCP_PFC_INVERTINGINPUT_GPIO_PORT    GPIOA /*!< COMPx inverting input GPIO port as defined in 
                                   wInvertingInput (Just if 
                                   bInvertingInput_MODE is EXT_MODE).
                                   It must be GPIOx x= A, B, ...*/
#define OCP_PFC_INVERTINGINPUT_GPIO_PIN     GPIO_Pin_5 /*!< COMPx inverting input GPIO pin as defined in 
                                   wInvertingInput (Just if 
                                   bInvertingInput_MODE is EXT_MODE). 
                                   It must be GPIO_Pin_x x= 0, 1, ...*/
#define OCP_PFC_NONINVERTINGINPUT  COMP5_NonInvertingInput_PB13         /*!< COMPx non inverting input. It must be one of
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
													   
#define OCP_PFC_NONINVERTINGINPUT_GPIO_PORT   GPIOB /*!< COMPx non inverting input GPIO port as 
                                   defined in wNonInvertingInput.
                                   It must be GPIOx x= A, B, ...*/
#define OCP_PFC_NONINVERTINGINPUT_GPIO_PIN  GPIO_Pin_13 /*!< COMPx non inverting input GPIO pin as defined
                                   in wNonInvertingInput. 
                                   It must be GPIO_Pin_x x= 0, 1, ...*/
#define OCP_PFC_OUTPUT_MODE        INT_MODE         /*!< COMPx output. It must be either 
                                   equal to EXT_MODE or INT_MODE. */
#define OCP_PFC_OUTPUT             COMP_Output_TIM16BKIN

// OCP_PFC_Output port and pin are not used if INT_MODE is selected
#define OCP_PFC_OUTPUT_GPIO_PORT   GPIOC        /*!< COMPx output GPIO port.
                                   It must be GPIOx x= A, B, ...*/
#define OCP_PFC_OUTPUT_GPIO_PIN    INT_MODE        /*!< COMPx output GPIO pin as defined. 
                                   It must be GPIO_Pin_x x= 0, 1, ...*/
#define OCP_PFC_OUTPUT_GPIO_AF     GPIO_AF_7    /*!< COMPx output alternate functions setting.
                                   It must be one of the GPIO_AF_x (x=0,1, ...) 
                                   according to the defined GPIO port and pin.*/
#define OCP_PFC_OUTPUTPOL          COMP_OutputPol_NonInverted  /*!< COMPx output polarity. It must be one of
                                   the following:
                                   COMP_OutputPol_NonInverted,
                                   COMP_OutputPol_Inverted.*/
#define OCP_PFC_FILTER             COMP_Mode_HighSpeed
														/*!< COMPx mode it is used to define both*/
                                     
#define OCP_PFC_REF                63113     // 12A //25000 // 30000 6.6A // 22540 5A,  (5A*0.05*OPAMPgain/3.3) * 65535  
                                                                                   
/* PFC OPAMP settings --------------------------------------------------*/

#define PFC_USE_INTERNAL_OPAMP					ENABLE 	/*!< ENABLE, DISABLE */
                                                                                   
#define PFC_OPAMP_SELECTION                     OPAMP_Selection_OPAMP4     /*!< First OPAMP selection. It must be 
                                                                                either equal to 
                                                                                OPAMP_Selection_OPAMP1 or
                                                                                OPAMP_Selection_OPAMP3.*/
#define PFC_OPAMP_INVERTINGINPUT_MODE           INT_MODE                  /*!< First OPAMP inverting input mode.
                                                                                It must be either equal to EXT_MODE
                                                                                or INT_MODE.*/
#define PFC_OPAMP_INVERTINGINPUT                OPAMP4_InvertingInput_PB10   /*!< First OPAMP inverting input pin*/
                                           
  
#define PFC_OPAMP_INVERTINGINPUT_GPIO_PORT      GPIOB                          /*!< First OPAMP inverting input GPIO port
                                                                              as defined in wOPAMP_InvertingInput.
                                                                              It must be GPIOx x= A, B, ... if 
                                                                              bOPAMP_InvertingInput_MODE is 
                                                                              EXT_MODE, otherwise can be dummy.*/
#define PFC_OPAMP_INVERTINGINPUT_GPIO_PIN       GPIO_Pin_10     /*!< First OPAMP inverting input GPIO pin
                                                              as defined in wOPAMP_InvertingInput.
                                                              It must be GPIO_Pin_x x= 0, 1, ... if 
                                                               bOPAMP_InvertingInput_MODE is 
                                                               EXT_MODE, otherwise can be dummy.*/
#define PFC_OPAMP_NONINVERTINGINPUT             OPAMP4_NonInvertingInput_PB11      /*!< First OPAMP non inverting input first
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

#define PFC_OPAMP_NONINVERTINGINPUT_GPIO_PORT     GPIOB
                                                                                  /*!< First OPAMP non inverting input GPIO 
                                                                                       port as defined in 
                                                                                       wOPAMP_NonInvertingInput_PHA. 
                                                                                       It must be GPIOx x= A, B, ...*/
#define PFC_OPAMP_NONINVERTINGINPUT_GPIO_PIN      GPIO_Pin_11
                                                                                  /*!< First OPAMP non inverting input GPIO
                                                                                       pin as defined in 
                                                                                       wOPAMP_NonInvertingInput_PHA. 
                                                                                       It must be GPIO_Pin_x x= 0, 1, ...*/
#define PFC_OPAMP_OUT_GPIO_PORT                   GPIOB
                                                                                /*!< First OPAMP output GPIO port.
                                                                                     It must be GPIOx x= A, B, ...
                                                                                     Note: Output pin is PA2 for OPAMP1,
                                                                                     PB1 for OPAMP3.*/
#define PFC_OPAMP_OUT_GPIO_PIN                    GPIO_Pin_12
                                                                                /*!< First OPAMP output GPIO pin.
                                                                                     It must be GPIO_Pin_x x= 0, 1, ...
                                                                                     Note: Output pin is PA2 for OPAMP1,
                                                                                     PB1 for OPAMP3.*/
/* Common settings -----------------------------------------------------------*/
#define PFC_OPAMP_PGAGAIN                  OPAMP_OPAMP_PGAGain_2   /*!< It defines the OPAMP PGA gains.
	                                   It must be one of the following:
                                           OPAMP_OPAMP_PGAGain_2,
                                           OPAMP_OPAMP_PGAGain_4,
                                           OPAMP_OPAMP_PGAGain_8,
                                           OPAMP_OPAMP_PGAGain_16.
                                           This value is taken in account
                                           just if wOPAMPx_InvertingInput is
                                           equal to OPAMP2_InvertingInput_PGA*/
#define PFC_OPAMP_PGACONNECT               OPAMP_PGAConnect_No      /*!< It defines the OPAMP connection
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








#endif /* __PFC_PARAMETERS_H */

/* ##@@_USER_CODE_START_##@@ */
/* ##@@_USER_CODE_END_##@@ */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
