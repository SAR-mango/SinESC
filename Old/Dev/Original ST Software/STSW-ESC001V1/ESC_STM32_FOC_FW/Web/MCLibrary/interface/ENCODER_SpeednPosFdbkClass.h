/**
  ******************************************************************************
  * @file    ENCODER_SpeednPosFdbkClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of ENCODER class      
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
#ifndef __ENCODER_SPEEDNPOSFDBKCLASS_H
#define __ENCODER_SPEEDNPOSFDBKCLASS_H

#define GPIO_NoRemap_TIMx ((uint32_t)(0))

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup SpeednPosFdbk_ENCODER
  * @{
  */

/** @defgroup ENCODER_class_exported_types ENCODER class exported types
* @{
*/

/** 
  * @brief  Public ENCODER class definition
  */
typedef struct CENC_SPD_t *CENC_SPD;

/** 
  * @brief  ENCODER class parameters definition
  */
typedef const struct
{
  /* SW Settings */
  uint8_t IRQnb; /*!< MC IRQ number used for TIMx capture update event.*/
  
  uint16_t hPulseNumber; /*!< Number of pulses per revolution, provided by each
                              of the two encoder signals, multiplied by 4 */
  
  FunctionalState RevertSignal; /*!< To be enabled if measured speed is opposite
                                     to real one (ENABLE/DISABLE)*/
  
  uint16_t hSpeedSamplingFreq01Hz; /*!< Frequency (01Hz) at which motor speed is to be
                                   computed. It must be equal to the frequency
                                   at which function SPD_CalcAvrgMecSpeed01Hz
                                   is called.*/
  
  uint8_t bSpeedBufferSize; /*!< Size of the buffer used to calculate the average 
                             speed. It must be <= 16.*/
  
  uint16_t hInpCaptFilter; /*!< Time filter applied to validate ENCODER sensor
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
  TIM_TypeDef* TIMx;    /*!< Timer used for ENCODER sensor management.*/
  
  uint32_t RCC_APB1Periph_TIMx; /*!< RCC Clock related to timer TIMx. 
                                     (Ex. RCC_APB1Periph_TIM2).*/
  
  uint8_t TIMx_IRQChannel; /*!< IRQ Channel related to TIMx. 
                                (Ex. TIM2_IRQChannel).*/
  
  uint32_t wTIMxRemapping; /*!< It remaps TIMx outputs. It must equal to 
                                GPIO_PartialRemap_TIMx, GPIO_FullRemap_TIMx or 
                                GPIO_NoRemap_TIMx according the HW availability*/
  
  GPIO_TypeDef* hAPort; /*!< ENCODER sensor A channel GPIO input port (if used, 
                              after re-mapping). It must be GPIOx x= A, B, ...*/
  
  uint16_t hAPin;       /*!< ENCODER sensor A channel GPIO input pin (if used, 
                              after re-mapping). It must be GPIO_Pin_x x=0,1,.*/
  
  GPIO_TypeDef* hBPort; /*!< ENCODER sensor B channel GPIO input port (if used, 
                              after re-mapping). It must be GPIOx x= A, B, ...*/
  
  uint16_t hBPin;       /*!< ENCODER sensor B channel GPIO input pin (if used, 
                              after re-mapping). It must be GPIO_Pin_x x= 0,1,.*/
  
}ENCODERParams_t, *pENCODERParams_t;
/**
  * @}
  */

/** @defgroup ENCODER_class_exported_methods ENCODER class exported methods
  * @{
  */
  
/**
  * @brief  Creates an object of the class ENCODER
  * @param  pSpeednPosFdbkParams pointer to an SpeednPosFdbk parameters structure
  * @param  pENCODERParams pointer to an ENCODER parameters structure
  * @retval CENC_SPD new instance of ENCODER object
  */
CENC_SPD ENC_NewObject(pSpeednPosFdbkParams_t pSpeednPosFdbkParams, pENCODERParams_t pENCODERParams);

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__ENCODER_SPEEDNPOSFDBKCLASS_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
