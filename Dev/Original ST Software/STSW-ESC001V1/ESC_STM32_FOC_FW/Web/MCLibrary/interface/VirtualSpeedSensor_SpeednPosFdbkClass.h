/**
  ******************************************************************************
  * @file    VirtualSpeedSensor_SpeednPosFdbkClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of VirtualSpeedSensor class      
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
#ifndef __VIRTUALSPEEDSENSOR_SPEEDNPOSFDBKCLASS_H
#define __VIRTUALSPEEDSENSOR_SPEEDNPOSFDBKCLASS_H

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup SpeednPosFdbk_VirtualSpeedSensor
  * @{
  */

/** @defgroup VirtualSpeedSensor_class_exported_types VirtualSpeedSensor class exported types
* @{
*/

/** 
  * @brief  Public VirtualSpeedSensor class definition
  */
typedef struct CVSS_SPD_t *CVSS_SPD;

/** 
  * @brief  VirtualSpeedSensor class parameters definition
  */
typedef const struct
{
  uint16_t hSpeedSamplingFreqHz; /*!< Frequency (Hz) at which motor speed is to 
                             be computed. It must be equal to the frequency
                             at which function SPD_CalcAvrgMecSpeed01Hz
                             is called.*/
  int16_t hTransitionSteps; /*< Number of steps to perform the transition phase
                             from CVSS_SPD to other CSPD; if the Transition PHase
                             should last TPH milliseconds, and the FOC Execution
                             Frequency is set to FEF kHz, then
                             hTransitionSteps = TPH * FEF*/
}VirtualSpeedSensorParams_t, *pVirtualSpeedSensorParams_t;
/**
  * @}
  */

/** @defgroup VirtualSpeedSensor_class_exported_methods VirtualSpeedSensor class exported methods
  * @{
  */
  
/**
  * @brief  Creates an object of the class VirtualSpeedSensor
  * @param  pSpeednPosFdbkParams pointer to an SpeednPosFdbk parameters structure
  * @param  pVirtualSpeedSensorParams pointer to an VirtualSpeedSensor parameters structure
  * @retval CVSS_SPD new instance of VirtualSpeedSensor object
  */
CVSS_SPD VSS_NewObject(pSpeednPosFdbkParams_t pSpeednPosFdbkParams, pVirtualSpeedSensorParams_t pVirtualSpeedSensorParams);

/**
  * @brief  Set the mechanical acceleration of virtual sensor. This acceleration
            is defined starting from current mechanical speed, final mechanical
            speed expressed in 0.1Hz and duration expressed in milliseconds.
  * @param  this related object of class CSPD.
  * @param  hFinalMecSpeed01Hz mechanical speed expressed in 0.1Hz assumed by 
            the virtual sensor at the end of the duration.
  * @param  hDurationms Duration expressed in ms. It can be 0 to apply 
            instantaneous the final speed. 
  * @retval none
  */
void  VSPD_SetMecAcceleration(CSPD this, int16_t  hFinalMecSpeed01Hz, 
                              uint16_t hDurationms);

/**
  * @brief  Checks if the ramp executed after a VSPD_SetMecAcceleration command
  *         has been completed.
  * @param  this related object of class CSPD.
  * @retval bool TRUE if the ramp is completed, otherwise FALSE.
  */
bool VSPD_RampCompleted(CSPD this);

/**
  * @brief  Get the final speed of last setled ramp of virtual sensor expressed 
            in 0.1Hz.
  * @param  this related object of class CSTC.
  * @param  hFinalMecSpeed01Hz mechanical speed expressed in 0.1Hz assumed by 
            the virtual sensor at the end of the duration.
  * @param  hDurationms Duration expressed in ms. It can be 0 to apply 
            instantaneous the final speed. 
  * @retval none
  */
int16_t  VSPD_GetLastRampFinalSpeed(CSPD this);

/**
  * @brief  Set the command to Start the transition phase from CVSS_SPD to other CSPD.
            Transition is to be considered ended when Sensor information is
            declared 'Reliable' or if function returned value is FALSE
  * @param  this related object of class CSPD.
  * @param  bool TRUE to Start the transition phase, FALSE has no effect
  * @retval bool TRUE if Transition phase is enabled (started or not), FALSE if
            transition has been triggered but it's actually disabled
            (parameter hTransitionSteps = 0)
  */
bool VSPD_SetStartTransition(CSPD this, bool bCommand);

/**
  * @brief  Return the status of the transition phase.
  * @param  this related object of class CSPD.
  * @retval bool TRUE if Transition phase is ongoing, FALSE otherwise.
  */
bool VSPD_IsTransitionOngoing(CSPD this);

/**
  * @brief  It could be used to set istantaneous information on rotor electrical
  *         angle.
  * @param  this related object of class CSPD
  * @param  hElAngle istantaneous measure of rotor electrical angle (s16degrees)
  * @retval none
  */
void VSPD_SetElAngle(CSPD this, int16_t hElAngle);

/**
  * @brief  It could be used to set istantaneous information on rotor electrical
  *         angle copied by state observer;
  * @param  this related object of class CSPD
  * @retval none
  */
void VSPD_SetCopyObserver(CSPD this);

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__VIRTUALSPEEDSENSOR_SPEEDNPOSFDBKCLASS_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
