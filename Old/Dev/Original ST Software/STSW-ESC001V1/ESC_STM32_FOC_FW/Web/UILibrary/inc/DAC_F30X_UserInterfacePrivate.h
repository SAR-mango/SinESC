/**
  ******************************************************************************
  * @file    DAC_F30X_UserInterfacePrivate.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private definition of DAC class for STM32F30x
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
#ifndef __DAC_F30X_USERINTERFACEPRIVATE_H
#define __DAC_F30X_USERINTERFACEPRIVATE_H

/** @addtogroup STM32_PMSM_UI_Library
  * @{
  */
  
/** @addtogroup DAC_UserInterface
  * @{
  */

/** @defgroup DAC_private_types DAC private types
* @{
*/

#define DAC_CH_NBR 2
#define DAC_CH_USER 2

/** 
  * @brief  DAC class members definition 
  */
typedef struct
{
  MC_Protocol_REG_t bChannel_variable[DAC_CH_NBR]; /*!< Array of codes of 
                                                        variables to be provided
                                                        in out to the related 
                                                        channel.*/
  int16_t hUserValue[DAC_CH_USER];                 /*!< Array of user defined 
                                                        DAC values.*/
}DVars_t,*pDVars_t;

/** 
  * @brief  Redefinition of parameter structure
  */
typedef DAC_F30XParams_t DParams_t, *pDParams_t; 

/** 
  * @brief Private DAC class definition 
  */
typedef struct
{
	DVars_t DVars_str;			/*!< Derived class members container */
	pDParams_t pDParams_str;	/*!< Derived class parameters container */
}_DCDAC_UI_t, *_DCDAC_UI;
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__DAC_F30X_USERINTERFACEPRIVATE_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
