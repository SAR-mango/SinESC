/**
  ******************************************************************************
  * @file    BusVoltageSensorPrivate.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private definition of BusVoltageSensor class      
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
#ifndef __BUSVOLTAGESENSORPRIVATE_H
#define __BUSVOLTAGESENSORPRIVATE_H

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */

/** @addtogroup BusVoltageSensor
  * @{
  */

/** @defgroup BusVoltageSensor_class_private_types BusVoltageSensor class private types
  * @{
  */

/** 
  * @brief  BusVoltageSensor class members definition
  */
  typedef struct
  {
    uint16_t hLatestConv;      /*!< It contains latest Vbus converted value 
                                    expressed in u16Volts format */
    uint16_t hAvBusVoltage_d;  /*! It contains latest available average Vbus
                                    expressed in digit */
    uint16_t hFaultState;      /*! It contains latest Fault code (MC_NO_ERROR, 
                                    MC_OVER_VOLT or MC_UNDER_VOLT) */
  }Vars_t,*pVars_t;

/** 
  * @brief  Redefinition of parameter structure
  */
typedef BusVoltageSensorParams_t Params_t, *pParams_t;

/**
  * @brief  Virtual methods container
  */
typedef struct
{
  void (*pVBS_Init)(CVBS this,CPWMC oPWMnCurrentSensor); 
  void (*pVBS_Clear)(CVBS this);
  uint16_t (*pVBS_CalcAvVbus)(CVBS this);   
}Methods_t,*pMethods_t;

/** 
  * @brief  Private BusVoltageSensor class definition 
  */
typedef struct
{
  Methods_t Methods_str;	/*!< Virtual methods container */
  Vars_t Vars_str; 		/*!< Class members container */
  pParams_t pParams_str;	/*!< Class parameters container */
  void *DerivedClass;		/*!< Pointer to derived class */
}_CVBS_t, *_CVBS;


/**
  * @}
  */
  
/** @defgroup BusVoltageSensor_class_methods_exported_to_derived_classes BusVoltageSensor class methods exported to private implementation of derived classes
  * @{
  */
/**
  * @brief  Creates an object of the class BusVoltageSensor
  * @param  pBusVoltageSensorParams pointer to a BusVoltageSensor parameters 
  *         structure
  * @retval CVBS new instance of BusVoltageSensor object
  */
CVBS VBS_NewObject(pBusVoltageSensorParams_t pBusVoltageSensorParams);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /*__BUSVOLTAGESENSORPRIVATE_H*/
/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/

