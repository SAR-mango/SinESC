/**
  ******************************************************************************
  * @file    UnidirectionalFastCom_UserInterfaceClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of UnidirectionalFastCom class      
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
#ifndef __UNIDIRECTIONALFASTCOM_USERINTERFACECLASS_H
#define __UNIDIRECTIONALFASTCOM_USERINTERFACECLASS_H

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup UserInterface_UnidirectionalFastCom
  * @{
  */

/** @defgroup UnidirectionalFastCom_class_exported_types UnidirectionalFastCom class exported types
* @{
*/

/** 
  * @brief  Public UnidirectionalFastCom class definition
  */
typedef struct CUFC_UI_t *CUFC_UI;

/** 
  * @brief  UnidirectionalFastCom class parameters definition
  */
typedef const struct
{
  /* HW Settings */
  USART_TypeDef* USARTx;
  uint32_t wUSARTRemapping;
  uint32_t wUSARTClockSource;
  GPIO_TypeDef* hTxPort;
  uint16_t hTxPin;
  uint8_t bUIIRQn;
  USART_InitTypeDef* USART_InitStructure;
  NVIC_InitTypeDef* NVIC_InitStructure;
  
  /* Functional settings */
  MC_Protocol_REG_t bDefChannel1; /*!< Code of default variables to be sent Ch1.*/
  MC_Protocol_REG_t bDefChannel2; /*!< Code of default variables to be sent Ch2.*/
  uint8_t bDefMotor;              /*!< Default motor selected. */
  uint8_t bCh1ByteNum;            /*!< Number of bytes transmitted for Ch1 */
  uint8_t bCh2ByteNum;            /*!< Number of bytes transmitted for Ch2 */
  uint8_t bChNum;                 /*!< Number of channel to be transmitted. */
} UnidirectionalFastComParams_t, *pUnidirectionalFastComParams_t;
/**
  * @}
  */

/** @defgroup UnidirectionalFastCom_class_exported_methods UnidirectionalFastCom class exported methods
  * @{
  */
  
/**
  * @brief  Creates an object of the class UnidirectionalFastCom
  * @param  pUserInterfaceParams pointer to an UserInterface parameters structure
  * @param  pUnidirectionalFastComParams pointer to an UnidirectionalFastCom parameters structure
  * @retval CUFC_UI new instance of UnidirectionalFastCom object
  */
CUFC_UI UFC_NewObject(pUserInterfaceParams_t pUserInterfaceParams, pUnidirectionalFastComParams_t pUnidirectionalFastComParams);

/**
  * @brief  Initialization of the class UnidirectionalFastCom. It initialize all
  *         HW and private vars.
  * @param  this related object of class CUFC_UI
  * @retval none
  */
void UFC_Init(CUFC_UI this);

/**
  * @brief  Starts the fast unidirectional communication.
  * @param  this related object of class CUFC_UI
  * @retval none
  */
void UFC_StartCom(CUFC_UI this);

/**
  * @brief  Starts the fast unidirectional communication.
  * @param  this related object of class CUFC_UI
  * @retval none
  */
void UFC_StopCom(CUFC_UI this);

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__UNIDIRECTIONALFASTCOM_USERINTERFACECLASS_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
