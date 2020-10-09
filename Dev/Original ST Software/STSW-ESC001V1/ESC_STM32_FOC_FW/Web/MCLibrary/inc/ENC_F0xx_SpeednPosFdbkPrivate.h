/**
  ******************************************************************************
  * @file    ENC_F0xx_SpeednPosFdbkPrivate.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private definition of ENCODER class      
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
#ifndef __ENC_F0xx_SPEEDNPOSFDBKPRIVATE_H
#define __ENC_F0xx_SPEEDNPOSFDBKPRIVATE_H

#define ENC_DMA_PRIORITY DMA_Priority_High

#define ENC_SPEED_ARRAY_SIZE 	((uint8_t)16)    /* 2^4 */
#define ENC_MAX_OVERFLOW_NB     ((uint16_t)2048) /* 2^11*/

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup SpeednPosFdbk_ENCODER
  * @{
  */

/** @defgroup ENCODER_private_types ENCODER private types
* @{
*/

/** 
  * @brief  ENCODER class members definition 
  */
typedef struct
{
  volatile uint16_t hTimerOverflowNb;    /*!< Number of overflows occurred since
                                        last speed measurement event*/
  
  bool SensorIsReliable;            /*!< Flag to indicate sensor/decoding is not
                                         properly working.*/  
  
  uint32_t wTimerStatusRegisterCopy;   /*<! Copy of the status register
                                        at time when capture occured*/
  
  uint16_t hTIM_FLAG_CCx;               /*<! Capture Flag of the specific channel
                                        used*/
  
  uint16_t hTIM_EventSource_CCx;        /*<! Channel where to generate Capture
                                        event*/
  
  volatile uint32_t *hTIMxCCRAddress;   /*<! Address of the specific CCR of 
                                        the timer in use*/
  
  uint16_t hPreviousCapture;            /*!< Timer counter value captured during
                                        previous speed measurement event*/
  
  int32_t wDeltaCapturesBuffer[ENC_SPEED_ARRAY_SIZE]; /*!< Buffer used to store
                                        captured variations of timer counter*/
  
  volatile uint8_t bDeltaCapturesIndex; /*! <Buffer index*/
  
  uint32_t wU32MAXdivPulseNumber;       /*! <It stores U32MAX/hPulseNumber*/
  
  uint16_t hSpeedSamplingFreqHz;        /*! <Frequency (Hz) at which motor speed
                                        is to be computed. */
  
  bool TimerOverflowError;              /*!< TRUE if the number of overflow
                                        occurred is greater than 'define'
                                        ENC_MAX_OVERFLOW_NB*/
}DVars_t,*pDVars_t;

/** 
  * @brief  Redefinition of parameter structure
  */
typedef ENCODERParams_t DParams_t, *pDParams_t; 

/** 
  * @brief Private ENCODER class definition 
  */
typedef struct
{
	DVars_t DVars_str;      	/*!< Derived class members container */
	pDParams_t pDParams_str;	/*!< Derived class parameters container */
}_DCENC_SPD_t, *_DCENC_SPD;
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__ENC_F0xx_SPEEDNPOSFDBKPRIVATE_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
