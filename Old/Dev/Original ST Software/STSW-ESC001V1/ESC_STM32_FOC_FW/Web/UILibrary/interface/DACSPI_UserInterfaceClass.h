/**
  ******************************************************************************
  * @file    DACSPI_UserInterfaceClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of DACSPI class      
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
#ifndef __DACSPI_USERINTERFACECLASS_H
#define __DACSPI_USERINTERFACECLASS_H

/** @addtogroup STM32_PMSM_UI_Library
  * @{
  */
  
/** @addtogroup DAC_SPI_UserInterface
  * @{
  */

/** @defgroup DACSPI_class_exported_types DACSPI class exported types
* @{
*/

/** 
  * @brief  Public DACSPI class definition
  */
typedef struct CDACS_UI_t *CDACS_UI;

/** 
  * @brief  DACSPI class parameters definition
  */
typedef const void DACSPIParams_t, *pDACSPIParams_t;
/**
  * @}
  */

/** @defgroup DACSPI_class_exported_methods DACSPI class exported methods
  * @{
  */
  
/**
  * @brief  Creates an object of the class DACSPI
  * @param  pUserInterfaceParams pointer to an UserInterface parameters structure
  * @param  pDACSPIParams pointer to an DACSPI parameters structure
  * @retval CDACS_UI new instance of DACSPI object
  */
CDACS_UI DACS_NewObject(pUserInterfaceParams_t pUserInterfaceParams, pDACSPIParams_t pDACSPIParams);

/**
  * @brief  Example of public method of the class DACSPI
  * @param  this related object of class CDACS_UI
  * @retval none
  */
void DACS_Func(CDACS_UI this);
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__DACSPI_USERINTERFACECLASS_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
