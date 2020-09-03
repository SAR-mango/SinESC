/**
  ******************************************************************************
  * @file    DigitalOutputClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of DigitalOutput class      
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
#ifndef __DIGITALOUTPUTCLASS_H
#define __DIGITALOUTPUTCLASS_H

/* Includes ------------------------------------------------------------------*/
#include "MC_type.h"

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */

/** @addtogroup DigitalOutput
  * @{
  */
  
/** @defgroup DigitalOutput_class_exported_constants DigitalOutput class exported constants
* @{
*/
#define DOutputActiveHigh       1u
#define DOutputActiveLow        0u

/**
* @}
*/

/** @defgroup DigitalOutput_class_exported_types DigitalOutput class exported types
* @{
*/

/** 
  * @brief  Public DigitalOutput class definition 
  */
typedef struct CDOUT_t *CDOUT;

/** 
  * @brief  DigitalOutput class parameters definition  
  */
typedef const struct
{
  GPIO_TypeDef* hDOutputPort;       /*!< GPIO output port. It must be equal
										 to GPIOx x= A, B, ...*/
  uint16_t hDOutputPin;             /*!< GPIO output pin. It must be equal to
                                     	GPIO_Pin_x x= 0, 1, ...*/  
  uint8_t  bDOutputPolarity;        /*!< GPIO output polarity. It must be equal 
                                     	to DOutputActiveHigh or DOutputActiveLow */ 
}DigitalOutputParams_t, *pDigitalOutputParams_t;
  
/**
* @}
*/

/** @defgroup DigitalOutput_class_exported_methods DigitalOutput class exported methods
  * @{
  */

/**
  * @brief  Creates an object of the class DigitalOutput. 
  * @param  pDigitalOutputParams pointer to an DigitalOutput parameters structure
  * @retval CDOUT new instance of DigitalOutput object
  */
CDOUT DOUT_NewObject(pDigitalOutputParams_t pDigitalOutputParams);

/**
  * @brief  Initializes object variables, port and pin. It must be called only 
  *         after PWMnCurrFdbk object initialization and DigitalOutput object 
  *         creation. 
  * @param this related object of class CDOUT.
  * @retval none.
  */
void DOUT_Init(CDOUT this);

/**
  * @brief Accordingly with selected polarity, it sets to active or inactive the
  *        digital output
  * @param this related object of class CDOUT.
  * @param OutputState_t New requested state
  * @retval none
  */
void DOUT_SetOutputState(CDOUT this, DOutputState_t State);

/**
  * @brief It returns the state of the digital output
  * @param this object of class DOUT
  * @retval OutputState_t Digital output state (ACTIVE or INACTIVE)
  */
DOutputState_t DOUT_GetOutputState(CDOUT this); 

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /* __DIGITALOUTPUTCLASS_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
