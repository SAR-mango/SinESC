/**
  ******************************************************************************
  * @file    GateDriverCtrlPrivate.h
  * @author  IMS Systems Lab and Technical Marketing - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private definition of GateDriverCtrl class      
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
#ifndef __GATEDRIVERCTRLPRIVATE_H
#define __GATEDRIVERCTRLPRIVATE_H

/** @addtogroup STM32F10x_PMSM_MC_Library
  * @{
  */

/** @addtogroup GateDriverCtrl
  * @{
  */

/** @defgroup GateDriverCtrl_class_private_types GateDriverCtrl class private types
* @{
*/

/** 
  * @brief  GateDriverCtrl class members definition
  */
typedef struct
{
	unsigned int base_vars; /*!< Example of member */
}Vars_t,*pVars_t;

/** 
  * @brief  Redefinition of parameter structure
  */
typedef GateDriverCtrlParams_t Params_t, *pParams_t;

/**
  * @brief Virtual methods container
  */
typedef struct
{
	void (*pGDC_Init)(CGDC this); /*!< Example of virtual function pointer */
}Methods_t,*pMethods_t;

/** 
  * @brief  Private GateDriverCtrl class definition 
  */
typedef struct
{
	Methods_t Methods_str;	/*!< Virtual methods container */
	Vars_t Vars_str; 		/*!< Class members container */
	pParams_t pParams_str;	/*!< Class parameters container */
	void *DerivedClass;		/*!< Pointer to derived class */
}_CGDC_t, *_CGDC;
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__GATEDRIVERCTRLPRIVATE_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
