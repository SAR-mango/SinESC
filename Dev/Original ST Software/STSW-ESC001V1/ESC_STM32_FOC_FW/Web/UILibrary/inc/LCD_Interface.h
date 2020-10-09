/**
  ******************************************************************************
  * @file    LCD_Interface.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   Header of "LCD Interface" module
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
#ifndef __LCD_INTERFACE_H
#define __LCD_INTERFACE_H

/* Exported Defines ----------------------------------------------------------*/

#ifdef __cplusplus
 extern "C" {
#endif
   
#define this _this
#include "UserInterfaceClass.h"
#undef this

void LCDI_Init(CUI oUI, CUI oDAC, const char* s_fwVer);
void LCDI_UpdateAll(CUI oUI);
void LCDI_UpdateMeasured(CUI oUI);
void LCDI_Polling(void);
void LCDI_SetUIImportedFunctions(void* const* ImportedFunctions);

#ifdef __cplusplus
 }
#endif

#endif /* __LCD_INTERFACE_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
