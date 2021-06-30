/**
  ******************************************************************************
  * @file    TemperatureSensorPrivate.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private definition of TemperatureSensor class      
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
#ifndef __TEMPERATURESENSORPRIVATE_H
#define __TEMPERATURESENSORPRIVATE_H

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */

/** @addtogroup TemperatureSensor
  * @{
  */

/** @defgroup TemperatureSensor_class_private_types TemperatureSensor class private types
  * @{
  */

/** 
  * @brief  TemperatureSensor class members definition
  */
  typedef struct
  {
    uint16_t hAvTemp_d;        /*! It contains latest available average Vbus
                                    expressed in u16Celsius */
    uint16_t hFaultState;      /*! It contains latest Fault code (MC_NO_ERROR, 
                                    or MC_OVER_TEMP) */
  }Vars_t,*pVars_t;

/** 
  * @brief  Redefinition of parameter structure
  */
typedef TempSensorParams_t Params_t, *pParams_t;

/**
  * @brief  Virtual methods container
  */
typedef struct
{
  void (*pTSNS_Init)(CTSNS this,CPWMC oPWMnCurrentSensor); 
  void (*pTSNS_Clear)(CTSNS this);
  uint16_t (*pTSNS_CalcAvTemp)(CTSNS this);   
  int16_t (*pTSNS_GetAvTemp_C)(CTSNS this);
}Methods_t,*pMethods_t;

/** 
  * @brief  Private TemperatureSensor class definition 
  */
typedef struct
{
  Methods_t Methods_str;	/*!< Virtual methods container */
  Vars_t Vars_str; 		/*!< Class members container */
  pParams_t pParams_str;	/*!< Class parameters container */
  void *DerivedClass;		/*!< Pointer to derived class */
}_CTSNS_t, *_CTSNS;

/**
  * @}
  */

/** @defgroup TempSensor_class_methods_exported_to_derived_classes TempSensor class methods exported to private implementation of derived classes
  * @{
  */
/**
  * @brief  Creates an object of the class temperature sensor
  * @param  pTempSensorParams pointer to a temperature sensor parameters 
  *         structure
  * @retval CTSNS new instance of temperature sensor object
  */
CTSNS TSNS_NewObject(pTempSensorParams_t pTempSensorParams);

/**
  * @}
  */
/**
  * @}
  */

/**
  * @}
  */

#endif /*__TEMPERATURESENSORPRIVATE_H*/
/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/

