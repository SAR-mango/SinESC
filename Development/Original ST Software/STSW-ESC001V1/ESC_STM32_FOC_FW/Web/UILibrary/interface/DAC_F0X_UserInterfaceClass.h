/**
  ******************************************************************************
  * @file    DAC_F0X_UserInterfaceClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of DAC class for STM32F0x
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
#ifndef __DAC_F0X_USERINTERFACECLASS_H
#define __DAC_F0X_USERINTERFACECLASS_H

/** @addtogroup STM32_PMSM_UI_Library
  * @{
  */
  
/** @addtogroup DAC_UserInterface
  * @{
  */

/** @defgroup DAC_class_exported_types DAC class exported types
* @{
*/

/** 
  * @brief  Public DAC class definition
  */
/* Note: It is declared a common CDAC_UI class for both CDAC and CDAC_F0X to 
         avoid duplicate pools. This should be safe untill the difference 
         between the two classes are just in the parametres struct.*/
#ifndef __CDAC_UI
#define __CDAC_UI
typedef struct CDAC_UI_t *CDAC_UI;
#endif

/** 
  * @brief  DAC class parameters definition
  */
typedef const struct
{
  uint16_t hDAC_CH1_ENABLED;  /*!< Set to ENABLE to assign the channel 1 to the 
                                   DAC object otherwise set DISABLE */
  uint16_t hDAC_CH2_ENABLED;  /*!< Set to ENABLE to assign the channel 2 to the 
                                   DAC object otherwise set DISABLE */
}
DAC_F0XParams_t, *pDAC_F0XParams_t;
/**
  * @}
  */

/** @defgroup DAC_class_exported_methods DAC class exported methods
  * @{
  */
  
/**
  * @brief  Creates an object of the class DAC for STM32F0x
  * @param  pUserInterfaceParams pointer to an UserInterface parameters structure
  * @param  pDACParams pointer to an DAC parameters structure
  * @retval CDAC_F0X_UI new instance of DAC object
  */
CDAC_UI DAC_F0X_NewObject(pUserInterfaceParams_t pUserInterfaceParams, pDAC_F0XParams_t pDACParams);
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__DAC_F0X_USERINTERFACECLASS_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
