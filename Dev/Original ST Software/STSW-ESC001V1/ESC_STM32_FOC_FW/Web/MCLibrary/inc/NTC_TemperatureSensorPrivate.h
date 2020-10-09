/**
  ******************************************************************************
  * @file    NTC_TemperatureSensorPrivate.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private definition of NTC class      
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
#ifndef __NTC_TEMPERATURESENSORPRIVATE_H
#define __NTC_TEMPERATURESENSORPRIVATE_H

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup TemperatureSensor_NTC
  * @{
  */

/** @defgroup NTC_private_types NTC private types
* @{
*/

/** 
  * @brief  NTC class members definition 
  */
typedef struct
{
  CPWMC oPWMnCurrentSensor;     /*! CPWMC object to be used for regular 
                                    conversions*/
  bool IsOverTempFaultActive;   /*! Flag to keep the information about present 
                                    over temperature fault situation */
}DVars_t,*pDVars_t;

/** 
  * @brief  Redefinition of parameter structure
  */
typedef NTCParams_t DParams_t, *pDParams_t; 

/** 
  * @brief Private NTC class definition 
  */
typedef struct
{
	DVars_t DVars_str;			/*!< Derived class members container */
	pDParams_t pDParams_str;	/*!< Derived class parameters container */
}_CNTC_TSNS_t, *_CNTC_TSNS;
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__NTC_TEMPERATURESENSORPRIVATE_H*/
/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
