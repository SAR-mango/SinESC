/**
  ******************************************************************************
  * @file    MotorPowerMeasurementPrivate.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private definition of MotorPowerMeasurement class      
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
#ifndef __MOTORPOWERMEASUREMENTPRIVATE_H
#define __MOTORPOWERMEASUREMENTPRIVATE_H

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */

/** @addtogroup MotorPowerMeasurement
  * @{
  */

/** @defgroup MotorPowerMeasurement_class_private_defines MotorPowerMeasurement class private defines
* @{
*/

#define MPM_BUFFER_LENGHT 128u /*!< Length of buffer used to store the 
                                   instantaneous measurements of motor power.*/

/**
  * @}
  */

/** @defgroup MotorPowerMeasurement_class_private_types MotorPowerMeasurement class private types
* @{
*/

/** 
  * @brief  MotorPowerMeasurement class members definition
  */
typedef struct
{
  int16_t hMeasBuffer[MPM_BUFFER_LENGHT]; /*!< Buffer used by MPM object to 
                                               store the instantaneous 
                                                measurements of motor power. */
  uint16_t hNextMeasBufferIndex; /*!< Index of the buffer that will contain the
                                      next motor power measurement. */
  uint16_t hLastMeasBufferIndex; /*!< Index of the buffer that contains the last
                                      motor power measurement. */
  int16_t hAvrgElMotorPowerW; /*!< The average measured motor power expressed in
                                   watt. */
}Vars_t,*pVars_t;

/**
  * @brief Virtual methods container
  */
typedef struct
{
  void (*pPQD_Init)(CMPM this, pMPMInitStruct_t pMPMInitStruct);
  void (*pPQD_Clear)(CMPM this);
  int16_t (*pPQD_CalcElMotorPower)(CMPM this);
}Methods_t,*pMethods_t;

/** 
  * @brief  Private MotorPowerMeasurement class definition 
  */
typedef struct
{
	Methods_t Methods_str;	/*!< Virtual methods container */
	Vars_t Vars_str; 		/*!< Class members container */
	void *DerivedClass;		/*!< Pointer to derived class */
}_CMPM_t, *_CMPM;
/**
  * @}
  */

/** @defgroup MotorPowerMeasurement_class_methods_exported_to_derived_classes MotorPowerMeasurement class methods exported to private implementation of derived classes
  * @{
  */

/**
  * @brief  Creates an object of the class MotorPowerMeasurement
  * @param  pMotorPowerMeasurementParams pointer to an MotorPowerMeasurement 
  *         parameters structure
  * @retval CMPM new instance of MotorPowerMeasurement object
  */
CMPM MPM_NewObject(void);

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__MOTORPOWERMEASUREMENTPRIVATE_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
