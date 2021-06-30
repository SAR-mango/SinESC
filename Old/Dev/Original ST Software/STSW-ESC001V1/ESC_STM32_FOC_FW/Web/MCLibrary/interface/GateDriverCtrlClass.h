/**
  ******************************************************************************
  * @file    GateDriverCtrlClass.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GATEDRIVERCTRLCLASS_H
#define __GATEDRIVERCTRLCLASS_H

/* Includes ------------------------------------------------------------------*/
#include "MC_type.h"

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */

/** @addtogroup GateDriverCtrl
  * @{
  */
  
/** @defgroup GateDriverCtrl_class_exported_types GateDriverCtrl class exported types
* @{
*/

/** 
  * @brief  Public GateDriverCtrl class definition 
  */
typedef struct CGDC_t *CGDC;

/** 
  * @brief  GateDriverCtrl class parameters definition  
  */
typedef const struct
{
  int32_t paramA; /*!< Example of parameter */
} GateDriverCtrlParams_t, *pGateDriverCtrlParams_t;
  
/**
* @}
*/

/** @defgroup GateDriverCtrl_class_exported_methods GateDriverCtrl class exported methods
  * @{
  */

/**
  * @brief  Creates an object of the class GateDriverCtrl
  * @param  pGateDriverCtrlParams pointer to an GateDriverCtrl parameters structure
  * @retval CGDC new instance of GateDriverCtrl object
  */
CGDC GDC_NewObject(pGateDriverCtrlParams_t pGateDriverCtrlParams);

/**
  * @brief  Example of public method of the class GateDriverCtrl
  * @param  this related object of class CGDC
  * @retval none
  */
void GDC_Func(CGDC this);

/**
  * @brief  Initiliaze all the object variables and MCU peripherals. Usually 
  *         it has to be called once right after object creation.
  * @param  this Gate driver controller object
  * @retval none
  */
void GDC_Init(CGDC this);
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /* __GATEDRIVERCTRLCLASS_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
