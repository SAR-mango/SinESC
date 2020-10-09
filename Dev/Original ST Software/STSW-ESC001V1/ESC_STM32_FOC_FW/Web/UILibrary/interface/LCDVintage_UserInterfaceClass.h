/**
  ******************************************************************************
  * @file    LCDVintage_UserInterfaceClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of LCDVintage class      
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
#ifndef __LCDVINTAGE_USERINTERFACECLASS_H
#define __LCDVINTAGE_USERINTERFACECLASS_H

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup UserInterface_LCDVintage
  * @{
  */

/** @defgroup LCDVintage_class_exported_types LCDVintage class exported types
* @{
*/

/** 
  * @brief  Public LCDVintage class definition
  */
typedef struct CLCDV_UI_t *CLCDV_UI;

/** 
  * @brief  LCDVintage class parameters definition
  */
typedef const struct
{
	unsigned int param1; /*!< Example of parameter */
}LCDVintageParams_t, *pLCDVintageParams_t;
/**
  * @}
  */

/** @defgroup LCDVintage_class_exported_methods LCDVintage class exported methods
  * @{
  */
  
/**
  * @brief  Creates an object of the class LCDVintage
  * @param  pUserInterfaceParams pointer to an UserInterface parameters structure
  * @param  pLCDVintageParams pointer to an LCDVintage parameters structure
  * @retval CLCDV_UI new instance of LCDVintage object
  */
CLCDV_UI LCDV_NewObject(pUserInterfaceParams_t pUserInterfaceParams, pLCDVintageParams_t pLCDVintageParams);

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__LCDVINTAGE_USERINTERFACECLASS_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
