/**
  ******************************************************************************
  * @file    GateDriverCtrlClass.c
  * @author  IMS Systems Lab and Technical Marketing - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of GateDriverCtrl class
  ******************************************************************************
  * <br>
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  */

/* Includes ------------------------------------------------------------------*/
#include "GateDriverCtrlClass.h"
#include "GateDriverCtrlPrivate.h"
#include "MCLibraryConf.h"
#include "MC_type.h"

#ifdef MC_CLASS_DYNAMIC
#include "stdlib.h" /* Used for dynamic allocation */
#else
_CGDC_t GDCpool[MAX_GDC_NUM];
unsigned char GDC_Allocated = 0u;
#endif

#define CLASS_VARS   &((_CGDC)this)->Vars_str
#define CLASS_PARAMS  ((_CGDC)this)->pParams_str

/**
  * @brief  Creates an object of the class GateDriverCtrl
  * @param  pGateDriverCtrlParams pointer to an GateDriverCtrl parameters structure
  * @retval CGDC new instance of GateDriverCtrl object
  */
CGDC GDC_NewObject(pGateDriverCtrlParams_t pGateDriverCtrlParams)
{
  _CGDC _oGDC;
  
  #ifdef MC_CLASS_DYNAMIC
    _oGDC = (_CGDC)calloc(1u,sizeof(_CGDC_t));
  #else
    if (GDC_Allocated  < MAX_GDC_NUM)
    {
      _oGDC = &GDCpool[GDC_Allocated++];
    }
    else
    {
      _oGDC = MC_NULL;
    }
  #endif
  
  _oGDC->pParams_str = (pParams_t)pGateDriverCtrlParams;
  
  return ((CGDC)_oGDC);
}

/**
  * @brief  Example of public method of the class GateDriverCtrl
  * @param  this related object of class CGDC
  * @retval none
  */
void GDC_Func(CGDC this)
{
  ((_CGDC)this)->Vars_str.base_vars = 0u;
}

/**
  * @brief  Initiliaze all the object variables and MCU peripherals. Usually 
  *         it has to be called once right after object creation.
  * @param  this Gate driver controller object
  * @retval none
  */
void GDC_Init(CGDC this)
{
  ((_CGDC)this)->Methods_str.pGDC_Init(this);
}

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
