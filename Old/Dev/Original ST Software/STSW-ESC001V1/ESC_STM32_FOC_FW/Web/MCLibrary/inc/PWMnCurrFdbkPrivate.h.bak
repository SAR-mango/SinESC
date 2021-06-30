/**
  ******************************************************************************
  * @file    PWMnCurrFdbkPrivate.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private definition of PWMnCurrFdbk class      
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
#ifndef __PWMNCURRFDBKPRIVATE_H
#define __PWMNCURRFDBKPRIVATE_H
#include "PWMnCurrFdbkClass.h"
/** @addtogroup STM32_PMSM_MC_Library
* @{
*/

/** @addtogroup PWMnCurrFdbk
* @{
*/

/** @defgroup PWMnCurrFdbk_Class_Private_Constants PWMnCurrFdbk class private constants
* @{
*/
/** 
* @brief  Space vector sectors constant definitions
*/
#define SECTOR_1	0u
#define SECTOR_2	1u
#define SECTOR_3	2u
#define SECTOR_4	3u
#define SECTOR_5	4u
#define SECTOR_6	5u

/**
* @}
*/

/** @defgroup PWMnCurrFdbk_class_private_types PWMnCurrFdbk class private types
* @{
*/

/** 
* @brief  CompareStruct_t, structure contating compare registers value 
*/

/** 
* @brief  PWMnCurrFdbk class members definition
*/
typedef struct
{
  uint16_t  hT_Sqrt3;   /*!< Contains a constant utilized by SVPWM algorithm */
  uint16_t  hSector;    /*!< Contains the space vector sector number */
  uint16_t  hCntPhA;
  uint16_t  hCntPhB;
  uint16_t  hCntPhC;
  uint16_t  SWerror;
  bool bTurnOnLowSidesAction; /*!< TRUE if TurnOnLowSides action is active,
                                   FALSE otherwise. */
  uint16_t  hOffCalibrWaitTimeCounter; /*!< Counter to wait fixed time before
                                            motor current measurement offset 
                                            calibration. */
  uint8_t   bMotor;     /*!< Motor reference number */
  bool      RLDetectionMode; /*!< TRUE if enabled, FALSE if disabled. */
  int16_t   hIa; /* Last Ia measurement. */
  int16_t   hIb; /* Last Ib measurement. */
  int16_t   hIc; /* Last Ic measurement. */
  uint16_t  DTTest;    /* Reserved */
  uint16_t  DTCompCnt; /* Reserved */
} Vars_t,*pVars_t;

/** 
* @brief  Redefinition of parameter structure
*/
typedef PWMnCurrFdbkParams_t Params_t, *pParams_t;

/**
* @brief Virtual methods container
*/
typedef struct
{
  void *(*pIRQ_Handler)(void *this, unsigned char flag);
  void (*pPWMC_Init)(CPWMC this);
  void (*pPWMC_GetPhaseCurrents)(CPWMC this, Curr_Components* pStator_Currents); 
  void (*pPWMC_SwitchOffPWM)(CPWMC this); 
  void (*pPWMC_SwitchOnPWM )(CPWMC this); 
  void (*pPWMC_CurrentReadingCalibr)(CPWMC this); 
  void (*pPWMC_TurnOnLowSides)(CPWMC this); 
  void (*pPWMC_SetSamplingTime)(CPWMC this, ADConv_t ADConv_struct); 
  uint16_t (*pPWMC_SetADCSampPointSect1)(CPWMC this);        
  uint16_t (*pPWMC_SetADCSampPointSect2)(CPWMC this);       
  uint16_t (*pPWMC_SetADCSampPointSect3)(CPWMC this);       
  uint16_t (*pPWMC_SetADCSampPointSect4)(CPWMC this);       
  uint16_t (*pPWMC_SetADCSampPointSect5)(CPWMC this);       
  uint16_t (*pPWMC_SetADCSampPointSect6)(CPWMC this);               
  uint16_t (*pPWMC_ExecRegularConv)(CPWMC this, uint8_t bChannel);
  uint16_t (*pPWMC_IsOverCurrentOccurred)(CPWMC this);
  void (*pPWMC_OCPSetReferenceVoltage)(CPWMC this,uint16_t hDACVref);
  void (*pRLDetectionModeEnable)(CPWMC this);
  void (*pRLDetectionModeDisable)(CPWMC this);
  uint16_t (*pRLDetectionModeSetDuty)(CPWMC this, uint16_t hDuty);

}Methods_t,*pMethods_t;

/** 
* @brief  Private PWMnCurrFdbk class definition 
*/
typedef struct
{
  Methods_t Methods_str;	/*!< Virtual methods container */
  Vars_t Vars_str; 		/*!< Class members container */
  pParams_t pParams_str;	/*!< Class parameters container */
  void *DerivedClass;		/*!< Pointer to derived class */
  
}_CPWMC_t, *_CPWMC;
/**
* @}
*/

/** @defgroup PWMnCurrFdbk_class_methods_exported_to_derived_classes PWMnCurrFdbk class methods exported to private implementation of derived classes
  * @{
  */
/**
  * @brief  Creates an object of the class PWMnCurrFdbk
  * @param  pPWMnCurrFdbkParams pointer to a PWMnCurrFdbk parameters structure
  * @retval CPWMC new instance of PWMnCurrFdbk object
  */
CPWMC PWMC_NewObject(pPWMnCurrFdbkParams_t pPWMnCurrFdbkParams);

/**
  * @}
  */
/**
* @}
*/

/**
* @}
*/

#endif /*__PWMNCURRFDBKPRIVATE_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
