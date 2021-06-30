/**
  ******************************************************************************
  * @file    PID_PIRegulatorClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of Derived class      
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
#ifndef __PID_PICLASS_H
#define __PID_PICLASS_H

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup PI_regulator_PID
  * @{
  */

/** @defgroup PID_class_exported_types PID regulator class exported types
* @{
*/

/** 
  * @brief  Public PID class definition
  */
typedef struct CPID_PI_t *CPID_PI;

/** 
  * @brief  PID class parameters definition
  */
typedef const struct
{
  int16_t hDefKdGain;           /*!< Default Kd gain, used to initialize Kd 
                                     gain software variable */
  uint16_t hKdDivisor;          /*!< Kd gain divisor, used in conjuction with 
                                     Kd gain allows obtaining fractional values. 
                                     If FULL_MISRA_C_COMPLIANCY is not defined 
                                     the divisor is implemented through 
                                     algebrical right shifts to speed up PI 
                                     execution. Only in this case this parameter
                                     specifies the number of right shifts to be 
                                     executed */
  uint16_t hKdDivisorPOW2;      /*!< Kd gain divisor expressed as power of 2.
                                     E.g. if gain divisor is 512 the value 
                                     must be 9 because 2^9 = 512 */
}PIDParams_t, *pPIDParams_t;
/**
  * @}
  */

/** @defgroup PID_class_exported_methods PID class exported methods
  * @{
  */
  
/**
  * @brief  Creates an object of the derived class PID
  * @param  pPIParams_t pointer to PI regulator class parameters structure
  * @param  pPIDParams_t pointer to a PID Derived class parameters structure
  * @retval CPID_PI new instance of derived class object
  */
CPID_PI PID_NewObject(pPIParams_t pPIParam,
                                  pPIDParams_t pPIDParam);
/**
  * @brief  It initiliazes all the object variables, usually it has to be called 
  *         once right after object creation
  * @param  CPID_PI PID regulator object
  * @retval None
  */
void PID_ObjectInit(CPID_PI this);

/**
  * @brief  It set a new value into the PID Previous error variable required to 
  *         compute derivative term
  * @param  CPID_PI PID regulator object
  * @param  int32_t New integral term value
  * @retval None
  */
void PID_SetPrevError(CPID_PI this, int32_t wPrevProcessVarError);

/**
  * @brief  It updates the Kd gain 
  * @param  CPID_PI PID regulator object
  * @param  int16_t New Kd gain
  * @retval None
  */
void PID_SetKD(CPID_PI this, int16_t hKdGain);

/**
  * @brief  It returns the Kd gain of the PID object passed 
  * @param  CPID_PI PID regulator object
  * @retval int16_t Kd gain
  */
int16_t PID_GetKD(CPID_PI this);

/**
  * @brief  It returns the Kd gain divisor of the PID object passed 
  * @param  CPID_PI PID regulator object
  * @retval int16_t Kd gain
  */
uint16_t PID_GetKDDivisor(CPID_PI this);

/**
  * @brief  This function compute the output of a PID regulator sum of its 
  *         proportional, integral and derivative terms
  * @param  CPID_PI PID regulator object
  * @param  Present process variable error, intended as the 
  *         reference value minus the present process variable value
  * @retval int16_t PID output
  */
int16_t PID_Controller(CPID_PI  this, int32_t wProcessVarError); 
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__PID_PICLASS_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
