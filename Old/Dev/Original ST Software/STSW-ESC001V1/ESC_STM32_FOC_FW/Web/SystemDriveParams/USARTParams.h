/**
  ******************************************************************************
  * @file    USARTParams.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains USART constant parameters definition
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
#ifndef __USART_PARAMS_H
#define __USART_PARAMS_H
#include "Parameters conversion.h"

#include "USART_PhysicalLayerCommunication_Class.h"
#include "UIIRQHandlerClass.h"

/* For backward compatibility */
#if (!defined(USART_SPEED))
#define USART_SPEED 115200
#endif

#define MC_PROTOCOL_REG_UNDEFINED 1
#if (SERIAL_COM_CHANNEL2 == MC_PROTOCOL_REG_UNDEFINED)
  #define SERIAL_COM_CH_NBRS 1
#else
  #define SERIAL_COM_CH_NBRS 2
#endif
#undef MC_PROTOCOL_REG_UNDEFINED

NVIC_USART_INIT_STR

const USART_InitTypeDef USARTInitHW_str = 
{
  USART_SPEED,                         //USART_BaudRate
  USART_WordLength_8b,            //USART_WordLength
  USART_StopBits_1,               //USART_StopBits
  USART_Parity_No,                //USART_Parity
  USART_Mode_Rx | USART_Mode_Tx,  //USART_Mode
  USART_HardwareFlowControl_None  //USART_HardwareFlowControl
};

const USART_InitTypeDef UFCInitHW_str = 
{
  USART_SPEED,                         //USART_BaudRate
  USART_WordLength_8b,            //USART_WordLength
  USART_StopBits_1,               //USART_StopBits
  USART_Parity_No,                //USART_Parity
  USART_Mode_Tx,  //USART_Mode
  USART_HardwareFlowControl_None  //USART_HardwareFlowControl
};

USARTParams_t USARTParams_str = 
{
  USART,                 // USART
  USART_GPIO_REMAP,       // USART_REMAP GPIO_NoRemap_USART1 or GPIO_Remap_USART1 ...
  USART_CLK,              // USART_CLK
  USART_RX_GPIO_PORT,    // USART_RxPort
  USART_RX_GPIO_PIN,     // USART_RxPin
  USART_TX_GPIO_PORT,    // USART_TxPort
  USART_TX_GPIO_PIN,     // USART_TxPin
  UI_IRQ_USART,           // IRQ Number
  (USART_InitTypeDef*)(&USARTInitHW_str),       // USART_InitStructure
  (NVIC_InitTypeDef*)(&NVICInitHW_str)         // NVIC_InitStructure
};

UnidirectionalFastComParams_t UFCParams_str =
{
  /* HW Settings */
  USART,                 // USART
  USART_GPIO_REMAP,       // USART_REMAP GPIO_NoRemap_USART1 or GPIO_Remap_USART1 ...
  USART_CLK,              // USART_CLK
  USART_TX_GPIO_PORT,    // USART_TxPort
  USART_TX_GPIO_PIN,     // USART_TxPin
  UI_IRQ_USART,           // IRQ Number
  (USART_InitTypeDef*)(&UFCInitHW_str),       // USART_InitStructure
  (NVIC_InitTypeDef*)(&NVICInitHW_str),       // NVIC_InitStructure
  /* Functional settings */
  SERIAL_COM_CHANNEL1, /*!< Code of default variables to be sent Ch1.*/
  SERIAL_COM_CHANNEL2, /*!< Code of default variables to be sent Ch2.*/
  SERIAL_COM_MOTOR,    /*!< Default motor selected. */
  1,                   /*!< Number of bytes transmitted for Ch1 */
  1,                   /*!< Number of bytes transmitted for Ch2 */
  SERIAL_COM_CH_NBRS   /*!< Number of channel to be transmitted. */
};

FCPParams_t FrameParams_str =
{
  5000,             // TimeOutRXByte
  1000,             // TimeOutTXByte
  MC_NULL,             // RXTimeOutEvent
  MC_NULL              // TXTimeOutEvent
};

MCPParams_t MCPParams = 
{
  &FrameParams_str // pFCPParams
};

#endif /* __USART_PARAMS_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
