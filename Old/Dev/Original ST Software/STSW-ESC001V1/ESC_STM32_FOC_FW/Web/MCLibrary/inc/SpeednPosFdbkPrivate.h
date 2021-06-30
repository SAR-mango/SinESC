/**
  ******************************************************************************
  * @file    SpeednPosFdbkPrivate.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private definition of SpeednPosFdbk class      
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
#ifndef __SPEEDNPOSFDBKPRIVATE_H
#define __SPEEDNPOSFDBKPRIVATE_H

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */

/** @addtogroup SpeednPosFdbk
  * @{
  */

/** @defgroup SpeednPosFdbk_class_private_types SpeednPosFdbk class private types
* @{
*/

/** 
  * @brief  SpeednPosFdbk class members definition
  */
typedef struct
{
        int16_t hElAngle;
        int16_t hMecAngle;
        int16_t hAvrMecSpeed01Hz;
        int16_t hElSpeedDpp;
        int16_t hMecAccel01HzP;
        uint8_t bSpeedErrorNumber;
        uint8_t bElToMecRatio;
}Vars_t,*pVars_t;

/** 
  * @brief  Redefinition of parameter structure
  */
typedef SpeednPosFdbkParams_t Params_t, *pParams_t;

/**
  * @brief Virtual methods container
  */
typedef struct
{
	void (*pIRQ_Handler)(void *this, unsigned char flag);
        void (*pSPD_Init)(CSPD this);
        void (*pSPD_Clear)(CSPD this);
        int16_t (*pSPD_CalcAngle)(CSPD this, void *pInputVars_str);
        bool (*pSPD_CalcAvrgMecSpeed01Hz)(CSPD this, int16_t *pMecSpeed01Hz);
        void (*pSPD_SetMecAngle)(CSPD this, int16_t hMecAngle);
}Methods_t,*pMethods_t;

/** 
  * @brief  Private SpeednPosFdbk class definition 
  */
typedef struct
{
	Methods_t Methods_str;	/*!< Virtual methods container */
	Vars_t Vars_str; 		/*!< Class members container */
	pParams_t pParams_str;	/*!< Class parameters container */
	void *DerivedClass;		/*!< Pointer to derived class */
}_CSPD_t, *_CSPD;
/**
  * @}
  */
  
/** @defgroup SpeednPosFdbk_class_methods_exported_to_derived_classes SpeednPosFdbk class methods exported to private implementation of derived classes
  * @{
  */
/**
  * @brief  Creates an object of the class SpeednPosFdbk
  * @param  pSpeednPosFdbkParams pointer to an SpeednPosFdbk parameters structure
  * @retval CSPD new instance of SpeednPosFdbk object
  */
CSPD SPD_NewObject(pSpeednPosFdbkParams_t pSpeednPosFdbkParams);

/**
  * @}
  */
/**
  * @}
  */

/**
  * @}
  */

#endif /*__SPEEDNPOSFDBKPRIVATE_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
