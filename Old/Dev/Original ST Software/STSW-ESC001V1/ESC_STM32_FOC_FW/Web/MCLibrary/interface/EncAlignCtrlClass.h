/**
  ******************************************************************************
  * @file    EncAlignCtrlClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of EncAlignCtrl class      
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
#ifndef __ENCALIGNCTRLCLASS_H
#define __ENCALIGNCTRLCLASS_H

/* Includes ------------------------------------------------------------------*/
#include "MC_type.h"
#include "SpeednTorqCtrlClass.h"
#include "VirtualSpeedSensor_SpeednPosFdbkClass.h"
#include "ENCODER_SpeednPosFdbkClass.h"

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */

/** @addtogroup EncAlignCtrl
  * @{
  */
  
/** @defgroup EncAlignCtrl_class_exported_types EncAlignCtrl class exported types
* @{
*/

/** 
  * @brief  Public EncAlignCtrl class definition 
  */
typedef struct CEAC_t *CEAC;

/** 
  * @brief  EncAlignCtrl class parameters definition  
  */
typedef const struct
{
  uint16_t hEACFrequencyHz; /*!< Frequency expressed in Hz at which the user 
                                 clocks the EAC calling EAC_Exec method */
  int16_t hFinalTorque;     /*!< Motor torque reference imposed by STC at the 
                                 end of programmed alignment. This value 
                                 represents actually the Iq current expressed in
                                 digit.*/
  int16_t hElAngle;        /*!< Electrical angle of programmed alignment 
                                 expressed in s16degrees.*/
  uint16_t hDurationms;     /*!< Duration of the programmed alignment expressed
                                 in milliseconds.*/
  uint8_t bElToMecRatio;    /*!< Coefficient used to transform electrical to
                                 mechanical quantities and viceversa. It usually
                                 coincides with motor pole pairs number*/
} EncAlignCtrlParams_t, *pEncAlignCtrlParams_t;
  
/**
* @}
*/

/** @defgroup EncAlignCtrl_class_exported_methods EncAlignCtrl class exported methods
  * @{
  */

/**
  * @brief  Creates an object of the class EncAlignCtrl
  * @param  pEncAlignCtrlParams pointer to an EncAlignCtrl parameters structure
  * @retval CEAC new instance of EncAlignCtrl object
  */
CEAC EAC_NewObject(pEncAlignCtrlParams_t pEncAlignCtrlParams);

/**
  * @brief  Initializes all the object variables, usually it has to be called 
  *         once right after object creation. It is also used to assign the 
  *         speed and torque controller, the virtual speed sensor objects and 
  *         the encoder object to be used by encoder alignment controller.
  * @param  this related object of class CEAC.
  * @param  oSTC the speed and torque controller used by the EAC.
  * @param  oVSS the virtual speed sensor used by the EAC.
  * @param  oENC the encoder object used by the EAC.
  * @retval none.
  */
void EAC_Init(CEAC this, CSTC oSTC, CVSS_SPD oVSS, CENC_SPD oENC );

/**
  * @brief  It is called to start the encoder alignment procedure.
  *	       	It configure the VSS with the required angle and sets the STC to 
  *         execute the required torque ramp.
  * @param  this related object of class CEAC.
  * @retval none.
  */
void EAC_StartAlignment(CEAC this);

/**
  * @brief  It clocks the encoder alignment controller and must be called with a 
  *         frequency equal to the one settled in the parameters 
  *         hEACFrequencyHz. Calling this method the EAC is possible to verify 
  *         if the alignment duration has been finished. At the end of alignment 
  *         the encoder object it is set to the defined electrical angle.
  *         Note: STC, VSS, ENC are not clocked by EAC_Exec.
  * @param  this related object of class CEAC.
  * @retval bool It returns TRUE when the programmed alignment has been 
  *         completed. 
  */
bool EAC_Exec(CEAC this);

/**
  * @brief  This function returns TRUE if the encoder has been aligned at least 
  *         one time, FALSE if hasn't been never aligned.
  * @param  this related object of class CEAC.
  * @retval bool It returns TRUE if the encoder has been aligned at least 
  *         one time, FALSE if hasn't been never aligned. 
  */
bool EAC_IsAligned(CEAC this);

/**
  * @brief  This function is used to program a restart after an encoder 
  *         alignment.
  * @param  this related object of class CEAC.
  * @param  restart Set to TRUE if a restart is programmed else FALSE
  * @retval none. 
  */
void EAC_SetRestartState(CEAC this, bool restart);

/**
  * @brief  This function is used to verify if a restart after an encoder 
  *         alignment has been requested.
  * @param  this related object of class CEAC.
  * @retval bool It is TRUE if a restart is programmed else FALSE.
  */
bool EAC_GetRestartState(CEAC this);
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /* __ENCALIGNCTRLCLASS_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
