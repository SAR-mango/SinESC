/**
  ******************************************************************************
  * @file    HALL_F4xx_SpeednPosFdbkPrivate.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private definition of HALL class      
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
#ifndef __HALL_F4xx__SPEEDNPOSFDBKPRIVATE_H
#define __HALL_F4xx__SPEEDNPOSFDBKPRIVATE_H

#define HALL_SPEED_FIFO_SIZE 	((u8)18)

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup SpeednPosFdbk_HALL
  * @{
  */

/** @defgroup HALL_private_types HALL private types
* @{
*/

typedef struct {
  uint32_t wPeriod;
  int8_t bDirection;
} PeriodMeas_s;

/** 
  * @brief  HALL class members definition 
  */
typedef struct
{
  bool SensorIsReliable;            /*!< Flag to indicate a wrong configuration 
                                         of the Hall sensor signanls.*/
  
  volatile bool RatioDec;           /*!< Flag to avoid consecutive prescaler 
                                         decrement.*/
  volatile bool RatioInc;           /*!< Flag to avoid consecutive prescaler 
                                         increment.*/
  volatile bool HallTimeOut;        /*!< Flag to indicate error on speed 
                                         measurement*/
  volatile uint8_t bFirstCapt;      /*!< Flag used to discard first capture for
                                         the speed measurement*/
  volatile uint8_t bBufferFilled;   /*!< Indicate the number of speed measuremt 
                                         present in the buffer from the start.
                                         It will be max bSpeedBufferSize and it
                                         is used to validate the start of speed
                                         averaging. If bBufferFilled is below
                                         bSpeedBufferSize the instantaneous
                                         measured speed is returned as average
                                         speed.*/
  volatile uint8_t bOVFCounter;     /*!< Count overflows if prescaler is too low
                                         */
  volatile PeriodMeas_s SensorPeriod[HALL_SPEED_FIFO_SIZE];/*!< Holding the last 
                                         captures */
  volatile uint8_t bSpeedFIFOSetIdx;/*!< Pointer of next element to be stored in
                                         the speed sensor buffer*/
  volatile uint8_t bSpeedFIFOGetIdx;/*!< Pointer of next element to be read from
                                         the speed sensor buffer*/
  
  int16_t hPrevRotorFreq; /*!< Used to store the last valid rotor electrical 
                               speed in dpp used when HALL_MAX_PSEUDO_SPEED
                               is detected */
  int8_t bSpeed;          /*!< Instantaneous direction of rotor between two 
                               captures*/
  int8_t  bNewSpeedAcquisition; /*!< Indacate that new speed information has 
                                     been stored in the buffer.*/
  
  int16_t hAvrElSpeedDpp; /*!< It is the averaged rotor electrical speed express
                               in s16degree per current control period.*/
                          
  uint8_t bHallState;     /*!< Current HALL state configuration */

  int16_t hDeltaAngle;    /*!< Delta angle at the Hall sensor signal edge between
                               current electrical rotor angle of synchronism.
                               It is in s16degrees.*/
  int16_t hMeasuredElAngle;/*!< This is the electrical angle  measured at each 
                               Hall sensor signal edge. It is considered the 
                               best measurement of electrical rotor angle.*/
  int16_t hTargetElAngle; /*!< This is the electrical angle target computed at
                               speed control frequency based on hMeasuredElAngle.*/
  int16_t hCompSpeed;     /*!< Speed compensation factor used to syncronize 
                               the current electrical angle with the target 
                               electrical angle. */
  
  uint16_t hHALLMaxRatio; /*!< Max TIM prescaler ratio defining the lowest 
                             expected speed feedback.*/
  uint16_t hSatSpeed;     /*!< Returned value if the measured speed is above the
                             maximum realistic.*/
  uint32_t wPseudoFreqConv;/*!< Conversion factor between time interval Delta T
                             between HALL sensors captures, express in timer 
                             counts, and electrical rotor speed express in dpp.
                             Ex. Rotor speed (dpp) = wPseudoFreqConv / Delta T
                             It will be ((CKTIM / 6) / (SAMPLING_FREQ)) * 65536.*/
  
  uint32_t wMaxPeriod;  /*!< Time delay between two sensor edges when the speed
                             of the rotor is the minimum realistic in the 
                             application: this allows to discriminate too low 
                             freq for instance.
                             This period shoud be expressed in timer counts and
                             it will be:
                             wMaxPeriod = ((10 * CKTIM) / 6) / MinElFreq(0.1Hz).*/
  
  uint32_t wSpeedOverflow;
                        /*!< Time delay between two sensor edges when the speed
                             of the rotor is the maximum realistic in the 
                             application: this allows discriminating glitches 
                             for instance.
                             This period shoud be expressed in timer counts and
                             it will be:
                             wSpeedOverflow = ((10 * CKTIM) / 6) / MaxElFreq(0.1Hz).*/
  
  uint16_t hHallTimeout;/*!< Max delay between two Hall sensor signal to assert
                             zero speed express in milliseconds.*/
  
  uint16_t hOvfDuration;/*!< Max counting frequency of timer (from 0 to 0x10000)
                             it will be: hOvfDuration = CKTIM /65536.*/
  uint16_t hPWMNbrPSamplingFreq; /*!< Number of current control periods inside 
                             each speed control periods it will be: 
                             (hMeasurementFrequency / hSpeedSamplingFreqHz) - 1.*/
}DVars_t,*pDVars_t;

/** 
  * @brief  Redefinition of parameter structure
  */
typedef HALLParams_t DParams_t, *pDParams_t; 

/** 
  * @brief Private HALL class definition 
  */
typedef struct
{
	DVars_t DVars_str;			/*!< Derived class members container */
	pDParams_t pDParams_str;	/*!< Derived class parameters container */
}_DCHALL_SPD_t, *_DCHALL_SPD;
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__HALL_F4xx_SPEEDNPOSFDBKPRIVATE_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
