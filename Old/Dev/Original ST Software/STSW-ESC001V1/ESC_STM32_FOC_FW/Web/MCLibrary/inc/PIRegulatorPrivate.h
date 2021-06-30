/**
  ******************************************************************************
  * @file    PIRegulatorPrivate.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private definition of Base class      
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
#ifndef __PIDREGULATORPRIVATE_H
#define __PIDREGULATORPRIVATE_H

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */

/** @addtogroup PI_regulator
  * @{
  */

/** @defgroup PI_regulator_class_private_types PI regulator class private types
* @{
*/

/** 
  * @brief  PI regulator class members definition
  */
typedef struct
{
  int16_t       hKpGain;
  int16_t       hKiGain;
  int32_t       wIntegralTerm;
  int32_t       wUpperIntegralLimit;
  int32_t       wLowerIntegralLimit;
  int16_t       hUpperOutputLimit;  
  int16_t       hLowerOutputLimit;    
  uint16_t      hKpDivisor;
  uint16_t      hKiDivisor;
  uint16_t      hKpDivisorPOW2;
  uint16_t      hKiDivisorPOW2;
}Vars_t,*pVars_t;

/** 
  * @brief  Redefinition of parameter structure
  */
typedef PIParams_t Params_t, *pParams_t; 

/*
typedef struct
{
  void (*pPIRegulator)(CPI oPI);
}Methods_t,*pMethods_t;
No virtual methods available for PI class */ 

/** 
* @brief  Private PI regulator class definition 
*/
typedef struct
{
  Vars_t Vars_str;           /*!< Class members container */
  pParams_t pParams_str;     /*!< Class parameters container */
  /*Methods_t Methods_str;   No virtual methods available for PI class */   
  void *DerivedClass;        /*!< Pointer to derived class */
}_CPI_t, *_CPI;

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */
#endif /*__SPEEDSENSORPRIVATE_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
