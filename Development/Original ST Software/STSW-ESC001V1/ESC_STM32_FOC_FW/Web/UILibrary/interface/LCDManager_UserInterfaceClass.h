/**
  ******************************************************************************
  * @file    LCDManager_UserInterfaceClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of LCDManager class      
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
#ifndef __LCDMANAGER_USERINTERFACECLASS_H
#define __LCDMANAGER_USERINTERFACECLASS_H

/** @addtogroup STM32_PMSM_UI_Library
  * @{
  */
  
/** @addtogroup LCD_Manager_UserInterface
  * @{
  */

/** @defgroup LCDManager_class_exported_types LCDManager class exported types
* @{
*/

/** 
  * @brief  Public LCDManager class definition
  */
typedef struct CLCD_UI_t *CLCD_UI;

/** 
  * @brief  LCDManager class parameters definition
  */
typedef const void LCDManagerParams_t, *pLCDManagerParams_t;
/**
  * @}
  */

/** @defgroup LCDManager_class_exported_methods LCDManager class exported methods
  * @{
  */
  
/**
  * @brief  Creates an object of the class LCDManager
  * @param  pUserInterfaceParams pointer to an UserInterface parameters structure
  * @param  pLCDManagerParams pointer to an LCDManager parameters structure
  * @retval CLCD_UI new instance of LCDManager object
  */
CLCD_UI LCD_NewObject(pUserInterfaceParams_t pUserInterfaceParams, 
                      pLCDManagerParams_t pLCDManagerParams);

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

/**
  * @brief  It is used to store the LCDI Imported functions.
  * @param  void** List of imported functions.
  * @retval none.
  */
void LCD_SetLCDIImportedFunctions(void* const* ImportedFunctions);

#endif /*__LCDMANAGER_USERINTERFACECLASS_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
