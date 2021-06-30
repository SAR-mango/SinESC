/**
  ******************************************************************************
  * @file    PIRegulatorClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of PI Regulator class      
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
#ifndef __PIREGULATORCLASS_H
#define __PIREGULATORCLASS_H

/* Includes ------------------------------------------------------------------*/
#include "MC_type.h"

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */

/** @addtogroup PI_regulator
  * @{
  */

/** @defgroup PI_regulator_Class_exported_types PI regulator class exported types
  * @{
  */

/** 
  * @brief  Public PI regulator class definition  
  */
typedef struct CPI_t  *CPI;

/** 
  * @brief  PI regulator class parameters structure definition  
  */
typedef const struct
{
  int16_t hDefKpGain;          /*!< Default Kp gain, used to initialize Kp gain
                                     software variable*/
  int16_t hDefKiGain;          /*!< Default Ki gain, used to initialize Ki gain
                                     software variable*/
  uint16_t hKpDivisor;           /*!< Kp gain divisor, used in conjuction with 
                                     Kp gain allows obtaining fractional values. 
                                     If FULL_MISRA_C_COMPLIANCY is not defined 
                                     the divisor is implemented through 
                                     algebrical right shifts to speed up PI 
                                     execution. Only in this case this parameter
                                     specifies the number of right shifts to be 
                                     executed */
  uint16_t hKiDivisor;           /*!< Ki gain divisor, used in conjuction with 
                                     Ki gain allows obtaining fractional values. 
                                     If FULL_MISRA_C_COMPLIANCY is not defined 
                                     the divisor is implemented through 
                                     algebrical right shifts to speed up PI 
                                     execution. Only in this case this parameter
                                     specifies the number of right shifts to be 
                                     executed */
  int32_t  wDefMaxIntegralTerm; /*!< Upper limit used to saturate the integral 
                                     term given by Ki / Ki divisor * integral of
                                     process variable error */ 
  int32_t  wDefMinIntegralTerm; /*!< Lower limit used to saturate the integral 
                                     term given by Ki / Ki divisor * integral of
                                     process variable error */    
  int16_t  hDefMaxOutput;       /*!< Upper limit used to saturate the PI output 
                                     */ 
  int16_t  hDefMinOutput;       /*!< Lower limit used to saturate the PI output 
                                     */ 
  uint16_t hKpDivisorPOW2;      /*!< Kp gain divisor expressed as power of 2.
                                     E.g. if gain divisor is 512 the value 
                                     must be 9 because 2^9 = 512 */
  uint16_t hKiDivisorPOW2;      /*!< Ki gain divisor expressed as power of 2.
                                     E.g. if gain divisor is 512 the value 
                                     must be 9 because 2^9 = 512 */
} PIParams_t, *pPIParams_t;

/**
  * @}
  */

/** @defgroup PI_Regulator_Class_Exported_Methods PI regulator class exported methods
  * @{
  */

/**
  * @brief  Creates an object of the class PI regulator
  * @param  pPIParams_t pointer to a PI parameters structure
  * @retval CPI new instance of PI object
  */
CPI  PI_NewObject(pPIParams_t pPIParams);

/**
  * @brief  Initiliaze all the object variables, usually it has to be called 
  *         once right after object creation
  * @param  CPI PI regulator object
  * @retval None
  */
void PI_ObjectInit(CPI this);

/**
  * @brief  This function compute the output of a PI regulator sum of its 
  *         proportional and integral terms
  * @param  CPI PI regulator object
  * @param  int32_t Present process variable error, intended as the reference 
  *         value minus the present process variable value
  * @retval int16_t PI output
  */
int16_t PI_Controller(CPI this, int32_t wProcessVarError); 

/**
  * @brief  It updates the Kp gain 
  * @param  CPI PI object
  * @param  int16_t New Kp gain
  * @retval None
  */
void PI_SetKP(CPI this, int16_t hKpGain);

/**
  * @brief  It updates the Ki gain 
  * @param  CPI PI object
  * @param  int16_t New Ki gain
  * @retval None
  */
void PI_SetKI(CPI this, int16_t hKiGain);

/**
  * @brief  It returns the Kp gain of the passed PI object
  * @param  CPI PI regulator object
  * @retval int16_t Kp gain 
  */
int16_t PI_GetKP(CPI this);

/**
  * @brief  It returns the Ki gain of the passed PI object
  * @param  CPI PI regulator object
  * @retval int16_t Ki gain 
  */
int16_t PI_GetKI(CPI this);

/**
  * @brief  It returns the Kp gain divisor of the passed PI object
  * @param  CPI PI regulator object
  * @retval int16_t Kp gain 
  */
uint16_t PI_GetKPDivisor(CPI this);

/**
  * @brief  It updates the Kp divisor
  * @param  CPI PI regulator object
  * @param  uint16_t New Kp divisor expressed as power of 2.
            E.g. if gain divisor is 512 the value 
            must be 9 because 2^9 = 512
  * @retval None
  */
void PI_SetKPDivisorPOW2(CPI this, uint16_t hKpDivisorPOW2);

/**
  * @brief  It returns the Ki gain divisor of the passed PI object
  * @param  CPI PI regulator object
  * @retval int16_t Ki gain 
  */
uint16_t PI_GetKIDivisor(CPI this);

/**
  * @brief  It updates the Ki divisor
  * @param  CPI PI regulator object
  * @param  uint16_t New Ki divisor expressed as power of 2.
            E.g. if gain divisor is 512 the value 
            must be 9 because 2^9 = 512
  * @retval None
  */
void PI_SetKIDivisorPOW2(CPI this, uint16_t hKiDivisorPOW2);

/**
  * @brief  It set a new value into the PI integral term 
  * @param  CPI PI regulator object
  * @param  int32_t New integral term value
  * @retval None
  */
void PI_SetIntegralTerm(CPI this, int32_t wIntegralTermValue);

/**
  * @brief  It set a new value for lower integral term limit  
  * @param  CPI PI regulator object
  * @param  int32_t New lower integral term limit value
  * @retval None
  */
void PI_SetLowerIntegralTermLimit(CPI this, int32_t wLowerLimit);

/**
  * @brief  It set a new value for upper integral term limit  
  * @param  CPI PI regulator object
  * @param  int32_t New upper integral term limit value
  * @retval None
  */
void PI_SetUpperIntegralTermLimit(CPI this, int32_t wUpperLimit);

/**
  * @brief  It set a new value for lower output limit  
  * @param  CPI PI regulator object
  * @param  int32_t New lower output limit value
  * @retval None
  */
void PI_SetLowerOutputLimit(CPI this, int16_t hLowerLimit);

/**
  * @brief  It set a new value for upper output limit  
  * @param  CPI PI regulator object
  * @param  int32_t New upper output limit value
  * @retval None
  */
void PI_SetUpperOutputLimit(CPI this, int16_t hUpperLimit);

/**
  * @brief  It returns the Default Kp gain of the passed PI object
  * @param  CPI PI regulator object
  * @retval int16_t Kp gain 
  */
int16_t PI_GetDefaultKP(CPI this);

/**
  * @brief  It returns the Default Ki gain of the passed PI object
  * @param  CPI PI regulator object
  * @retval int16_t Ki gain 
  */
int16_t PI_GetDefaultKI(CPI this);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /*__PIDREGULATORCLASS_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
