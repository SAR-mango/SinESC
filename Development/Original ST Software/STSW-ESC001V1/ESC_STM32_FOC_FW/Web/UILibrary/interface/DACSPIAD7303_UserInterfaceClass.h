/**
  ******************************************************************************
  * @file    DACSPIAD7303_UserInterfaceClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of DACSPIAD7303 class      
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
#ifndef __DACSPIAD7303_USERINTERFACECLASS_H
#define __DACSPIAD7303_USERINTERFACECLASS_H

/** @addtogroup STM32_PMSM_UI_Library
  * @{
  */
  
/** @addtogroup DAC_SPI_AD7303_UserInterface
  * @{
  */

/** @defgroup DACSPIAD7303_class_exported_types DACSPIAD7303 class exported types
* @{
*/

/** 
  * @brief  Public DACSPIAD7303 class definition
  */
typedef struct CDACX_UI_t *CDACX_UI;

/** 
  * @brief  DACSPIAD7303 class parameters definition
  */
typedef const void DACSPIAD7303Params_t, *pDACSPIAD7303Params_t;
/**
  * @}
  */

/** @defgroup DACSPIAD7303_class_exported_methods DACSPIAD7303 class exported methods
  * @{
  */
  
/**
  * @brief  Creates an object of the class DACSPIAD7303
  * @param  pUserInterfaceParams pointer to an UserInterface parameters structure
  * @param  pDACSPIAD7303Params pointer to an DACSPIAD7303 parameters structure
  * @retval CDACX_UI new instance of DACSPIAD7303 object
  */
CDACX_UI DACX_NewObject(pUserInterfaceParams_t pUserInterfaceParams, pDACSPIAD7303Params_t pDACSPIAD7303Params);

/**
  * @brief  Example of public method of the class DACSPIAD7303
  * @param  this related object of class CDACX_UI
  * @retval none
  */
void DACX_Func(CDACX_UI this);
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__DACSPIAD7303_USERINTERFACECLASS_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
