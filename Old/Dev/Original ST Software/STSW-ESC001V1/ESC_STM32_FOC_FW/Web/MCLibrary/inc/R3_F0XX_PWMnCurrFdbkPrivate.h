/**
  ******************************************************************************
  * @file    R3_F0XX_PWMnCurrFdbkPrivate.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private definition of R3_F0XX class      
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
#ifndef __R3_F0XX_PWMNCURRFDBKPRIVATE_H
#define __R3_F0XX_PWMNCURRFDBKPRIVATE_H

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup PWMnCurrFdbk_R3_F0XX
  * @{
  */
/** 
* @brief  Flags definition
*/
#define CURRENT_READING_ON   0x0002u /*!< Flag to indicate the active states 
                                          conditions (as START and RUN states) when
                                          the currents reading is executed, to avoid the   
                                          the regular conversions during these states. */

#define CALIB 0x0010u                /*!< This flag is used to indicate the ADC 
                                          calibration phase in order to avoid 
                                          concurrent regular conversions*/

#define REGCONVONGOING 0x0020u       /*!< This flag will be set when a regconversion 
                                          is requested by ExecRegularConv and the ADC has been 
                                          set for the conversion (GetPhaseCurrents), it will be 
                                          reset after reading conversion (CalcDutyCycles).*/

/** 
* @brief  Maximum number of "regular" conversions handled by the class
*/
#define MAX_REG_CONVERSIONS 17u /*!< Is the maximum number of "regular" (this 
                                    means not related to the current reading) 
                                    conversions that can be requested to this 
                                    class */
/**
* @}
*/

/** @defgroup R3_F0XX_private_types R3_F0XX private types
* @{
*/

/** 
  * @brief  R3_F0XX class members definition 
  */
typedef struct
{
  uint32_t wPhaseAOffset;       /*!< Offset of Phase A current sensing network  */
  uint32_t wPhaseBOffset;       /*!< Offset of Phase B current sensing network  */
  uint32_t wPhaseCOffset;       /*!< Offset of Phase C current sensing network  */
  uint16_t Half_PWMPeriod;      /*!< Half PWM Period in timer clock counts */
  uint16_t hRegConv;            /*!< Variable used to store regular conversions 
                                 *   result*/
  /* Add here extra variables required by F0xx micro ex. User conversions.*/
                                   
  bool OverCurrentFlag;         /*!< This flag is set when an overcurrent occurs.*/
  bool OverVoltageFlag;         /*!< This flag is set when an overvoltage occurs.*/
  bool BrakeActionLock;         /*!< This flag is set to avoid that brake action is 
                                 *   interrupted.*/
  uint8_t bIndex;               /*!< Number of conversions performed during the 
                                 *   calibration phase*/      
  
  uint16_t hFlags;            /*!< Flags                              
                                   CURRENT_READING_ON: 
                                   CALIB: This flag is used to indicate the ADC calibration
                                          phase in order to avoid concurrent regular conversions
                                   REGCONVONGOING: This flag will be set when a 
                                          regconversion is requested by 
                                          ExecRegularConv and the ADC has been 
                                          set for the conversion 
                                          (GetPhaseCurrents), it will be reset 
                                          after reading conversion 
                                          (CalcDutyCycles).*/
  uint8_t bRegSmpTime[18];    /*!< Array of sampling times for each channel expressed in 
                                   ADC clock cycles. E.g. 0 = 1.5 clock cycle. 
                                   See STM32F0xx reference manual for the 
                                   ramining values.*/
  
  uint16_t hRegConvValue[MAX_REG_CONVERSIONS];  /*!< Result of regular conversion.*/
  
  uint8_t bRegConvCh[MAX_REG_CONVERSIONS];      /*!< Temporary variables used to store regular 
                                                     conversion channel. Vale 255 means none.*/
  uint8_t bRegConvIndex;      /*!< Index of the element inside the regular 
                                   conversion buffer to be processed.*/
  uint8_t bRegConvRequested;  /*!< Number of the "regular" channels already 
                                   requested.*/
 
  __IO uint16_t ADC1_DMA_converted[3]; /* Buffer used for DMA data transfer after the ADC conversion */
  
  uint8_t bCalib_A_index;  /*!< Index of the A-Phase inside the DMA_ADC_BUFFER during the calibration phase*/
  uint8_t bCalib_B_index;  /*!< Index of the B-Phase inside the DMA_ADC_BUFFER during the calibration phase*/
  uint8_t bCalib_C_index;  /*!< Index of the C-Phase inside the DMA_ADC_BUFFER during the calibration phase*/
  
  
}DVars_t,*pDVars_t;

/** 
  * @brief  Redefinition of parameter structure
  */
typedef R3_F0XXParams_t DParams_t, *pDParams_t; 

/** 
  * @brief Private R3_F0XX class definition 
  */
typedef struct
{
	DVars_t DVars_str;		/*!< Derived class members container */
	pDParams_t pDParams_str;	/*!< Derived class parameters container */
}_DCR3F0XX_PWMC_t, *_DCR3F0XX_PWMC;
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__R3_F0XX_PWMNCURRFDBKPRIVATE_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
