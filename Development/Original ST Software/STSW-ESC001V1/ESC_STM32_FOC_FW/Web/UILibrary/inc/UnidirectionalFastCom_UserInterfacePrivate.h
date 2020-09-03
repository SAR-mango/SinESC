/**
  ******************************************************************************
  * @file    UnidirectionalFastCom_UserInterfacePrivate.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private definition of UnidirectionalFastCom class      
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
#ifndef __UNIDIRECTIONALFASTCOM_USERINTERFACEPRIVATE_H
#define __UNIDIRECTIONALFASTCOM_USERINTERFACEPRIVATE_H

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup UserInterface_UnidirectionalFastCom
  * @{
  */

/** @defgroup UnidirectionalFastCom_private_types UnidirectionalFastCom private types
* @{
*/

/** 
  * @brief  UnidirectionalFastCom class members definition 
  */
typedef struct
{
  bool comON;  /*!< True to establish the communication false to stop it */
  MC_Protocol_REG_t bChannel[2]; /*!< Codes of variables to be sent. */
  uint8_t bChTransmitted;        /*!< Current channel to be transmitted. */
  uint8_t bByteTransmitted;      /*!< Current byte to be transmitted. */
  int32_t wBuffer;               /*!< Transmission buffer 4 bytes. */
  uint8_t bChByteNum[2];         /*!< Number of bytes transmitted. */
  uint8_t bChNum;                /*!< Number of channel to be transmitted. */
} DVars_t,*pDVars_t;

/** 
  * @brief  Redefinition of parameter structure
  */
typedef UnidirectionalFastComParams_t DParams_t, *pDParams_t; 

/** 
  * @brief Private UnidirectionalFastCom class definition 
  */
typedef struct
{
  DVars_t DVars_str;			/*!< Derived class members container */
  pDParams_t pDParams_str;	/*!< Derived class parameters container */
} _DCUFC_UI_t, *_DCUFC_UI;
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__UNIDIRECTIONALFASTCOM_USERINTERFACEPRIVATE_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
