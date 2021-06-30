/**
  ******************************************************************************
  * @file    PWMnCurrFdbkClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of PWMnCurrFdbk class      
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
#ifndef __PWMNCURRFDBKCLASS_H
#define __PWMNCURRFDBKCLASS_H

/* Includes ------------------------------------------------------------------*/
#include "MC_type.h"

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */

/** @addtogroup PWMnCurrFdbk
  * @{
  */


/** @defgroup PWMnCurrFdbk_class_exported_types PWMnCurrFdbk class exported types
* @{
*/

/** 
  * @brief  Public PWMnCurrFdbk class definition 
  */
typedef struct CPWMC_t *CPWMC;

/** 
  * @brief  PWMnCurrFdbk class parameters definition  
  */
typedef const struct
{
  uint16_t hPWMperiod;            /*!< It contains the PWM period expressed in 
                                       timer clock cycles unit: 
                                       hPWMPeriod = Timer Fclk / Fpwm    */
  uint16_t hOffCalibrWaitTicks;   /*!< Wait time duration before current reading
                                       calibration expressed in number of calls
                                       of PWMC_CurrentReadingCalibr with action
                                       CRC_EXEC */
  uint16_t hDTCompCnt;            /*!< It contains half of Dead time expressed
                                       in timer clock cycles unit:
                                       hDTCompCnt = (DT(s) * Timer Fclk)/2 */
  uint16_t  Ton;                  /*!< Reserved */
  uint16_t  Toff;                 /*!> Reserved */
} PWMnCurrFdbkParams_t, *pPWMnCurrFdbkParams_t;

typedef const struct
{
  uint32_t      wSelection;   /*!< Internal comparator used for protection. 
				   It must be
                                   COMP_Selection_COMPx x = 1,2,3,4,5,6,7.*/
  uint8_t       bInvertingInput_MODE;
                              /*!< COMPx inverting input mode. It must be either 
                                   equal to EXT_MODE or INT_MODE. */
  uint32_t      wInvertingInput;
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
  GPIO_TypeDef* hInvertingInput_GPIO_PORT;
                              /*!< COMPx inverting input GPIO port as defined in 
                                   wInvertingInput (Just if 
                                   bInvertingInput_MODE is EXT_MODE).
                                   It must be GPIOx x= A, B, ...*/
  uint16_t      hInvertingInput_GPIO_PIN;
                              /*!< COMPx inverting input GPIO pin as defined in 
                                   wInvertingInput (Just if 
                                   bInvertingInput_MODE is EXT_MODE). 
                                   It must be GPIO_Pin_x x= 0, 1, ...*/
  uint32_t      wNonInvertingInput;
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
  GPIO_TypeDef* hNonInvertingInput_GPIO_PORT;
                              /*!< COMPx non inverting input GPIO port as 
                                   defined in wNonInvertingInput.
                                   It must be GPIOx x= A, B, ...*/
  uint16_t      hNonInvertingInput_GPIO_PIN;
                              /*!< COMPx non inverting input GPIO pin as defined
                                   in wNonInvertingInput. 
                                   It must be GPIO_Pin_x x= 0, 1, ...*/
  uint8_t       bOutput_MODE;
                              /*!< COMPx output. It must be either 
                                   equal to EXT_MODE or INT_MODE. */
  uint32_t      wOutput; /*!< COMPx output selection. It must be one of
                                   the following:
                                   COMP_Output_TIM1BKIN,
                                   COMP_Output_TIM1BKIN2,
                                   COMP_Output_TIM8BKIN,
                                   COMP_Output_TIM8BKIN2,
                                   COMP_Output_TIM1BKIN2_TIM8BKIN2.*/
  GPIO_TypeDef* hOutput_GPIO_PORT;
                              /*!< COMPx output GPIO port.
                                   It must be GPIOx x= A, B, ...*/
  uint16_t      hOutput_GPIO_PIN;
                              /*!< COMPx output GPIO pin as defined. 
                                   It must be GPIO_Pin_x x= 0, 1, ...*/
  uint8_t       bOutput_GPIO_AF;/*!< COMPx output alternate functions setting.
                                   It must be one of the GPIO_AF_x (x=0,1, ...) 
                                   according to the defined GPIO port and pin.*/
  uint32_t      wOutputPol;
                              /*!< COMPx output polarity. It must be one of
                                   the following:
                                   COMP_OutputPol_NonInverted,
                                   COMP_OutputPol_Inverted.*/
  uint32_t wMode;             /*!< COMPx mode it is used to define both
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
} F30XCOMPParams_t, *pF30XCOMPParams_t;

/** 
  * @brief  Current reading calibration definition  
  */
typedef enum
{
  CRC_START, /*!< Initialize the current reading calibration.*/
  CRC_EXEC   /*!< Execute the current reading calibration.*/
} CRCAction_t;
/**
* @}
*/

/** @defgroup PWMnCurrFdbk_class_exported_methods PWMnCurrFdbk class exported methods
  * @{
  */

/**
  * @brief  Initiliaze all the object variables and MCU peripherals, usually 
  *         it has to be called once right after object creation.
  *         Note: All the GPIOx port peripherals clocks are here enabled.
  *         Note: If the derived class is IHD2, R1HD2 or R3HD2 it is required
  *               to call the specific xxx_StartTimers method after the
  *               PWMC_Init call.
  * @param  this PWM 'n Current feedback object
  * @retval none
  */
void PWMC_Init(CPWMC this);

/**
* @brief  It is used to get the motor phase current in Curr_Components format 
          as read by AD converter.
* @param  this: PWM 'n Current feedback object
* @param  pStator_Currents Pointer to the struct that will receive motor current
*         of phase A and B in Curr_Components format.
* @retval none.
*/
void PWMC_GetPhaseCurrents(CPWMC this,Curr_Components* pStator_Currents);

/**
  * @brief  It converts input voltage components Valfa, beta into duty cycles 
  *         and feed it to the inverter
  * @param  this: PWM 'n Current feedback object
  * @param  Valfa_beta: Voltage Components in alfa beta reference frame
  * @retval It returns the code error 'MC_FOC_DURATION' if any, 'MC_NO_ERROR' 
  *         otherwise. These error codes are defined in MC_type.h
  */
uint16_t PWMC_SetPhaseVoltage(CPWMC this, Volt_Components Valfa_beta);

/**
  * @brief  It switch off the PWM generation, setting to inactive the outputs
  * @param  this: PWM 'n Current feedback object
  * @retval none
  */
void PWMC_SwitchOffPWM(CPWMC this);

/**
  * @brief  It switch on the PWM generation
  * @param  this: PWM 'n Current feedback object
  * @retval None
  */
void PWMC_SwitchOnPWM(CPWMC this);

/**
  * @brief  It calibrates ADC current conversions by reading the offset voltage
  *         present on ADC pins when no motor current is flowing. It's suggested
  *         to call this function before each motor start-up
  * @param  this: PWM 'n Current feedback object
  * @param  action: it can be CRC_START to initialize the offset calibration or
  *         CRC_EXEC to execute the offset calibration.
  * @retval bool It returns TRUE if the current calibration has been completed
  *         otherwise if is ongoing it returns FALSE.
  */
bool PWMC_CurrentReadingCalibr(CPWMC this, CRCAction_t action);

/**
  * @brief  It turns on low sides. This function is intended to be used for
  *         charging boot capacitors of driving section. It has to be called each 
  *         motor start-up when using high voltage drivers
  * @param  this: PWM 'n Current feedback object
  * @retval None
  */

void PWMC_TurnOnLowSides(CPWMC this);

/**
* @brief  Execute a regular conversion using ADC1. 
*         The function is not re-entrant (can't executed twice at the same time)
*         It returns 0xFFFF in case of conversion error.
* @param  this related object of class CPWMC, ADC channel to be converted
* @param  bChannel ADC channel used for the regular conversion
* @retval It returns converted value or oxFFFF for conversion error
*/
uint16_t PWMC_ExecRegularConv(CPWMC this, uint8_t bChannel);

/**
* @brief  Execute a regular conversion using ADC1 and ADC3(both ADC for regular)
*         The function is not re-entrant (can't executed twice at the same time)
*         It returns 0xFFFF in case of conversion error.
* @param  this related object of class CPWMC, ADC channel to be converted
* @param  bChannel ADC channel used for the regular conversion
* @retval It returns converted value or oxFFFF for conversion error
*/
uint16_t PWMC_ExecRegularConv_ESC(CPWMC this, uint8_t bChannel);

/**
* @brief  It sets the specified sampling time for the specified ADC channel
*         on ADC1. It must be called once for each channel utilized by user
* @param  this related object of class CPWMC
* @param  ADConv_struct struct containing ADC channel and sampling time
* @retval none
*/
void PWMC_ADC_SetSamplingTime(CPWMC this, ADConv_t ADConv_struct);

/**
* @brief  It is used to check if an overcurrent occurred since last call.
* @param  this related object of class CPWMC
* @retval uint16_t It returns MC_BREAK_IN whether an overcurrent has been 
*                  detected since last method call, MC_NO_FAULTS otherwise.
*/
uint16_t PWMC_CheckOverCurrent(CPWMC this);

/**
* @brief  It is used to set the overcurrent threshold through the DAC reference 
*         voltage.
* @param  this related object of class CPWMC
* @param  hDACVref Value of DAC reference expressed as 16bit unsigned integer.
*         Ex. 0 = 0V 65536 = VDD_DAC.
* @retval none
*/
void PWMC_OCPSetReferenceVoltage(CPWMC this,uint16_t hDACVref);

/**
* @brief  It is used to retrieve the satus of TurnOnLowSides action.
* @param  this related object of class CPWMC
* @retval bool It returns the state of TurnOnLowSides action: 
*         TRUE if TurnOnLowSides action is active, FALSE otherwise.
*/
bool PWMC_GetTurnOnLowSidesAction(CPWMC this);

/**
* @brief  It is used to set the RL Detection mode.
* @param  this related object of class CPWMC
* @retval none
*/
void PWMC_RLDetectionModeEnable(CPWMC this);

/**
* @brief  It is used to disable the RL Detection mode and set the standard PWM.
* @param  this related object of class CPWMC
* @retval none
*/
void PWMC_RLDetectionModeDisable(CPWMC this);

/**
* @brief  It is used to set the PWM dutycycle in the RL Detection mode.
* @param  this related object of class CPWMC
* @param  hDuty to be applied in u16
* @retval It returns the code error 'MC_FOC_DURATION' if any, 'MC_NO_ERROR' 
*         otherwise. These error codes are defined in MC_type.h
*/
uint16_t PWMC_RLDetectionModeSetDuty(CPWMC this, uint16_t hDuty);

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /* __PWMNCURRFDBKCLASS_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
