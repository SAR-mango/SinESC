/**
  ******************************************************************************
  * @file    UIExportedFunctions.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains the definitions of UI exported functions      
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
#ifndef __UIEXPORTEDFUNCTIONS_H
#define __UIEXPORTEDFUNCTIONS_H

enum {
EF_UI_GetReg,
EF_UI_ExecSpeedRamp,
EF_UI_SetReg,
EF_UI_ExecCmd,
EF_UI_GetSelectedMCConfig,
EF_UI_SetRevupData,
EF_UI_GetRevupData,
EF_UI_DACChannelConfig,
EF_UI_SetCurrentReferences,
EF_UI_NUMBERS
};

typedef int32_t (*pUI_GetReg_t) (CUI,MC_Protocol_REG_t);
typedef bool (*pUI_ExecSpeedRamp_t)(CUI,int32_t,uint16_t);
typedef bool (*pUI_SetReg_t)(CUI,MC_Protocol_REG_t,int32_t);
typedef bool (*pUI_ExecCmd_t)(CUI,uint8_t);
typedef uint32_t (*pUI_GetSelectedMCConfig_t)(CUI);
typedef bool (*pUI_SetRevupData_t)(CUI,uint8_t,uint16_t,int16_t,int16_t);
typedef bool (*pUI_GetRevupData_t)(CUI,uint8_t,uint16_t*,int16_t*,int16_t*);
typedef void (*pUI_DACChannelConfig_t)(CUI,DAC_Channel_t,MC_Protocol_REG_t);
typedef void (*pUI_SetCurrentReferences_t)(CUI,int16_t,int16_t); 

#endif /*__UIEXPORTEDFUNCTIONS_H*/
/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
