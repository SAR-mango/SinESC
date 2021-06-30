/**
  ******************************************************************************
  * @file    LCDManager_UserInterfaceClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private impelementation of LCDManager class      
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

/* Includes ------------------------------------------------------------------*/
#include "UserInterfaceClass.h"
#include "UserInterfacePrivate.h"
#include "LCDManager_UserInterfaceClass.h"
#include "LCDManager_UserInterfacePrivate.h"
#include "LCDIExportedFunctions.h"
#include "MC_type.h"

#ifdef MC_CLASS_DYNAMIC
  #include "stdlib.h" /* Used for dynamic allocation */
#else
  #define MAX_LCD_UI_NUM 1u
  _DCLCD_UI_t LCD_UIpool[MAX_LCD_UI_NUM];
  unsigned char LCD_UI_Allocated = 0u;
#endif
        
#define DCLASS_PARAM ((_DCLCD_UI)(((_CUI) this)->DerivedClass))->pDParams_str
#define DCLASS_VARS  &(((_DCLCD_UI)(((_CUI) this)->DerivedClass))->DVars_str)
#define  CLASS_VARS  &(((_CUI)this)->Vars_str)
#define  CLASS_PARAM (((_CUI)this)->pParams_str)
        
static void LCD_Init(CUI this, CUI oDAC, const char* s_fwVer);
static void LCD_Exec(CUI this);
static void LCD_UpdateAll(CUI this);
static void LCD_UpdateMeasured(CUI this);

static void* const* g_ImportedFunctions = MC_NULL;

/**
  * @brief  Creates an object of the class LCDManager
  * @param  pUserInterfaceParams pointer to an UserInterface parameters structure
  * @param  pLCDManagerParams pointer to an LCDManager parameters structure
  * @retval CLCD_UI new instance of LCDManager object
  */
CLCD_UI LCD_NewObject(pUserInterfaceParams_t pUserInterfaceParams, pLCDManagerParams_t pLCDManagerParams)
{
	_CUI _oUserInterface;
	_DCLCD_UI _oLCDManager;

	_oUserInterface = (_CUI)UI_NewObject(pUserInterfaceParams);

	#ifdef MC_CLASS_DYNAMIC
		_oLCDManager = (_DCLCD_UI)calloc(1u,sizeof(_DCLCD_UI_t));
	#else
		if (LCD_UI_Allocated  < MAX_LCD_UI_NUM)
		{
			_oLCDManager = &LCD_UIpool[LCD_UI_Allocated++];
		}
		else
		{
			_oLCDManager = MC_NULL;
		}
	#endif
  
	_oLCDManager->pDParams_str = pLCDManagerParams;
	_oUserInterface->DerivedClass = (void*)_oLCDManager;
        
        _oUserInterface->Methods_str.pUI_LCDInit = &LCD_Init;
        _oUserInterface->Methods_str.pUI_LCDExec = &LCD_Exec;
        _oUserInterface->Methods_str.pUI_LCDUpdateAll = &LCD_UpdateAll;
        _oUserInterface->Methods_str.pUI_LCDUpdateMeasured = &LCD_UpdateMeasured;
  
	return ((CLCD_UI)_oUserInterface);
}

/** @addtogroup STM32F10x_PMSM_UI_Library
  * @{
  */
  
/** @addtogroup LCD_Manager_UserInterface
  * @{
  */

/** @defgroup LCDManager_class_private_methods LCDManager class private methods
* @{
*/

/**
  * @brief  Initialization of LCD object. It must be called after the UI_Init.
  * @param  this related object of class CUI. It must be a LCDx_UI object casted
  *         to CUI otherwise the method will have no effect.
  * @param  oDAC related DAC object upcasted to CUI. It can be MC_NULL.
  * @param  s_fwVer String contating firmware version.
  * @retval none.
  */
static void LCD_Init(CUI this, CUI oDAC, const char* s_fwVer)
{
  pLCDI_Init_t pLCDI_Init = (pLCDI_Init_t)(g_ImportedFunctions[EF_LCDI_Init]);
  (*pLCDI_Init)((CUI)this, oDAC, s_fwVer);
}

/**
  * @brief  Execute the LCD execution and refreshing. It must be called 
  *         periodically.
  * @param  this related object of class CUI. It must be a LCDx_UI object casted
  *         to CUI otherwise the method will have no effect.
  * @retval none.
  */
static void LCD_Exec(CUI this)
{
  pLCDI_Polling_t pLCDI_Polling = (pLCDI_Polling_t)(g_ImportedFunctions[EF_LCDI_Polling]);  
  (*pLCDI_Polling)();
}

/**
  * @brief  It is used to force a refresh of all LCD values.
  * @param  this related object of class CUI. It must be a LCDx_UI object casted
  *         to CUI otherwise the method will have no effect.
  * @retval none.
  */
static void LCD_UpdateAll(CUI this)
{
  pLCDI_UpdateAll_t pLCDI_UpdateAll = (pLCDI_UpdateAll_t)(g_ImportedFunctions[EF_LCDI_UpdateAll]);
  (*pLCDI_UpdateAll)((CUI)this);
}

/**
  * @brief  It is used to force a refresh of only measured LCD values.
  * @param  this related object of class CUI. It must be a LCDx_UI object casted
  *         to CUI otherwise the method will have no effect.
  * @retval none.
  */
static void LCD_UpdateMeasured(CUI this)
{
  pLCDI_UpdateMeasured_t pLCDI_UpdateMeasured = (pLCDI_UpdateMeasured_t)(g_ImportedFunctions[EF_LCDI_UpdateMeasured]);
  (*pLCDI_UpdateMeasured)((CUI)this);
}

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
void LCD_SetLCDIImportedFunctions(void* const* ImportedFunctions)
{
  g_ImportedFunctions = ImportedFunctions;
}

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
