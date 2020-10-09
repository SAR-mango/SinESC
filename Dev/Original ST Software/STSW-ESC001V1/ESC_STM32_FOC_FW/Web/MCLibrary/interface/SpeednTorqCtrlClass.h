/**
  ******************************************************************************
  * @file    SpeednTorqCtrlClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of SpeednTorqCtrl class      
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
#ifndef __SPEEDNTORQCTRLCLASS_H
#define __SPEEDNTORQCTRLCLASS_H

/* Includes ------------------------------------------------------------------*/
#include "MC_type.h"
#include "PIRegulatorClass.h"
#include "SpeednPosFdbkClass.h"

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */

/** @addtogroup SpeednTorqCtrl
  * @{
  */
  
/** @defgroup SpeednTorqCtrl_class_exported_types SpeednTorqCtrl class exported types
* @{
*/

/** 
  * @brief  Public SpeednTorqCtrl class definition 
  */
typedef struct CSTC_t *CSTC;

/** 
  * @brief  SpeednTorqCtrl class parameters definition  
  */
typedef const struct
{
  uint16_t hSTCFrequencyHz;             /*!< Frequency on which the user updates 
                                             the torque reference calling 
                                             STC_CalcTorqueReference method 
                                             expressed in Hz */
  uint16_t hMaxAppPositiveMecSpeed01Hz; /*!< Application maximum positive value
                                             of rotor speed. It's expressed in 
                                             tenth of mechanical Hertz.*/
  uint16_t hMinAppPositiveMecSpeed01Hz; /*!< Application minimum positive value
                                             of rotor speed. It's expressed in
                                             tenth of mechanical Hertz.*/
  int16_t hMaxAppNegativeMecSpeed01Hz;  /*!< Application maximum negative value
                                             of rotor speed. It's expressed in 
                                             tenth of mechanical Hertz.*/
  int16_t hMinAppNegativeMecSpeed01Hz;  /*!< Application minimum negative value
                                             of rotor speed. It's expressed in
                                             tenth of mechanical Hertz.*/
  uint16_t hMaxPositiveTorque;          /*!< Maximum positive value of motor 
                                             torque. This value represents 
                                             actually the maximum Iq current 
                                             expressed in digit.*/
  int16_t hMinNegativeTorque;           /*!< Minimum negative value of motor 
                                             torque. This value represents 
                                             actually the maximum Iq current 
                                             expressed in digit.*/
  STC_Modality_t bModeDefault;          /*!< Default STC modality.*/
  int16_t hMecSpeedRef01HzDefault;      /*!< Default mechanical rotor speed 
                                             reference expressed in tenths of 
                                             HZ.*/
  int16_t hTorqueRefDefault;            /*!< Default motor torque reference. 
                                             This value represents actually the
                                             Iq current reference expressed in
                                             digit.*/
  int16_t hIdrefDefault;                /*!< Default Id current reference expressed
                                             in digit.*/
}SpeednTorqCtrlParams_t, *pSpeednTorqCtrlParams_t;
  
/**
* @}
*/

/** @defgroup SpeednTorqCtrl_class_exported_methods SpeednTorqCtrl class exported methods
  * @{
  */

/**
  * @brief  Creates an object of the class SpeednTorqCtrl
  * @param  pSpeednTorqCtrlParams pointer to an SpeednTorqCtrl parameters structure
  * @retval CSTC new instance of SpeednTorqCtrl object
  */
CSTC STC_NewObject(pSpeednTorqCtrlParams_t pSpeednTorqCtrlParams);

/**
  * @brief  Initializes all the object variables, usually it has to be called 
  *         once right after object creation.
  * @param  this related object of class CSTC.
  * @param  oPI the PI object used as controller for the speed regulation.
  *         It can be equal to MC_NULL if the STC is initialized in torque mode
  *         and it will never be configured in speed mode.
  * @param  oSPD the speed sensor used to perform the speed regulation.
  *         It can be equal to MC_NULL if the STC is used only in torque 
  *         mode.
  * @retval none.
  */
void STC_Init(CSTC this, CPI oPI, CSPD oSPD);

/**
  * @brief  It should be called before each motor restart. If STC is set in
            speed mode, this method resets the integral term of speed regulator.
  * @param this related object of class CSTC.
  * @retval none.
  */
void STC_Clear(CSTC this);

/**
  * @brief  Get the current mechanical rotor speed reference expressed in tenths
  *         of HZ. 
  * @param  this related object of class CSTC.
  * @retval int16_t current mechanical rotor speed reference expressed in tenths
  *         of HZ. 
  */
int16_t STC_GetMecSpeedRef01Hz(CSTC this);

/**
  * @brief  Get the current motor torque reference. This value represents 
  *         actually the Iq current reference expressed in digit.
  *         To convert current expressed in digit to current expressed in Amps
  *         is possible to use the formula: 
  *         Current(Amp) = [Current(digit) * Vdd micro] / [65536 * Rshunt * Aop]
  * @param  this related object of class CSTC.
  * @retval int16_t current motor torque reference. This value represents 
  *         actually the Iq current expressed in digit.
  */
int16_t STC_GetTorqueRef(CSTC this);

/**
  * @brief  Set the modality of the speed and torque controller. Two modality 
  *         are available Torque mode and Speed mode.
  *         In Torque mode is possible to set directly the motor torque 
  *         reference or execute a motor torque ramp. This value represents 
  *         actually the Iq current reference expressed in digit.
  *         In Speed mode is possible to set the mechanical rotor speed
  *         reference or execute a speed ramp. The required motor torque is
  *         automatically calculated by the STC.
  *         This command interrupts the execution of any previous ramp command 
  *         maintaining the last value of Iq.
  * @param  this related object of class CSTC.
  * @param  bMode modality of STC. It can be one of these two settings: 
  *         STC_TORQUE_MODE to enable the Torque mode or STC_SPEED_MODE to
  *         enable the Speed mode.
  * @retval none
  */
void STC_SetControlMode(CSTC this, STC_Modality_t bMode);

/**
  * @brief  Get the modality of the speed and torque controller. 
  * @param  this related object of class CSTC.
  * @retval STC_Modality_t It returns the modality of STC. It can be one of 
  *         these two values: STC_TORQUE_MODE or STC_SPEED_MODE.
  */
STC_Modality_t STC_GetControlMode(CSTC this);

/**
  * @brief  Starts the execution of a ramp using new target and duration. This
  *         command interrupts the execution of any previous ramp command.
  *         The generated ramp will be in the modality previously set by
  *         STC_SetControlMode method.
  * @param  this related object of class CSTC  
  * @param  hTargetFinal final value of command. This is different accordingly
  *         the STC modality. 
  *         If STC is in Torque mode hTargetFinal is the value of motor torque
  *         reference at the end of the ramp. This value represents actually the
  *         Iq current expressed in digit.
  *         To convert current expressed in Amps to current expressed in digit
  *         is possible to use the formula:
  *         Current(digit) = [Current(Amp) * 65536 * Rshunt * Aop]  /  Vdd micro
  *         If STC is in Speed mode hTargetFinal is the value of mechanical
  *         rotor speed reference at the end of the ramp expressed in tenths of
  *         HZ.
  * @param  hDurationms the duration of the ramp expressed in milliseconds. It
  *         is possible to set 0 to perform an instantaneous change in the value.
  * @retval bool It return false if the absolute value of hTargetFinal is out of
  *         the boundary of the application (Above max application speed or max
  *         application torque or below min application speed depending on
  *         current modality of TSC) in this case the command is ignored and the
  *         previous ramp is not interrupted, otherwise it returns true. 
  */
bool STC_ExecRamp(CSTC this, int16_t hTargetFinal, uint32_t hDurationms);

/**
  * @brief  This command interrupts the execution of any previous ramp command.
  *         If STC has been set in Torque mode the last value of Iq is
  *         maintained.
  *         If STC has been set in Speed mode the last value of mechanical
  *         rotor speed reference is maintained.
  * @param  this related object of class CSTC.
  * @retval none
  */
void STC_StopRamp(CSTC this);

/**
  * @brief  It is used to compute the new value of motor torque reference. It
  *         must be called at fixed time equal to hSTCFrequencyHz. It is called
  *         passing as parameter the speed sensor used to perform the speed
  *         regulation.
  * @param  this related object of class CSTC.
  * @retval int16_t motor torque reference. This value represents actually the
  *         Iq current expressed in digit.
  *         To convert current expressed in Amps to current expressed in digit
  *         is possible to use the formula:
  *         Current(digit) = [Current(Amp) * 65536 * Rshunt * Aop]  /  Vdd micro
  */
int16_t STC_CalcTorqueReference(CSTC this);

/**
  * @brief  Get the Default mechanical rotor speed reference expressed in tenths
  *         of HZ.
  * @param  this related object of class CSTC.
  * @retval int16_t It returns the Default mechanical rotor speed. reference 
  *         expressed in tenths of HZ.
  */
int16_t STC_GetMecSpeedRef01HzDefault(CSTC this);

/**
  * @brief  Get the Application maximum positive value of rotor speed. It's 
            expressed in tenth of mechanical Hertz.
  * @param  this related object of class CSTC.
  * @retval uint16_t It returns the application maximum positive value of rotor
            speed expressed in tenth of mechanical Hertz.
  */
uint16_t STC_GetMaxAppPositiveMecSpeed01Hz(CSTC this);

/**
  * @brief  Get the Application minimum negative value of rotor speed. It's 
            expressed in tenth of mechanical Hertz.
  * @param  this related object of class CSTC.
  * @retval uint16_t It returns the application minimum negative value of rotor
            speed expressed in tenth of mechanical Hertz.
  */
int16_t STC_GetMinAppNegativeMecSpeed01Hz(CSTC this);

/**
  * @brief  Check if the settled speed or torque ramp has been completed.
  * @param  this related object of class CSTC.
  * @retval bool It returns TRUE if the ramp is completed, FALSE otherwise.
  */
bool STC_RampCompleted(CSTC this);

/**
  * @brief  Stop the execution of speed ramp.
  * @param  this related object of class CSTC.
  * @retval bool It returns TRUE if the command is executed, FALSE otherwise.
  */
bool STC_StopSpeedRamp(CSTC this);

/**
  * @brief It sets in real time the speed sensor utilized by the FOC. 
  * @param this related object of class CSTC
  * @param oSPD Speed sensor object to be set.
  * @retval none
  */
void STC_SetSpeedSensor(CSTC this, CSPD oSPD);

/**
  * @brief It returns the speed sensor utilized by the FOC. 
  * @param this related object of class CSTC
  * @retval CSPD speed sensor utilized by the FOC.
  */
CSPD STC_GetSpeedSensor(CSTC this);

/**
  * @brief It returns the default values of Iqdref. 
  * @param this related object of class CSTC
  * @retval default values of Iqdref.
  */
Curr_Components STC_GetDefaultIqdref(CSTC this);

/**
  * @brief  Change the nominal current .
  * @param  this related object of class CSTC.
  * @param  hNominalCurrent This value represents actually the maximum Iq current 
            expressed in digit.
  * @retval none
  */
void STC_SetNominalCurrent(CSTC this, uint16_t hNominalCurrent);

/**
  * @brief  Force the speed reference to the curren speed. It is used
  *         at the START_RUN state to initialize the speed reference.
  * @param  this related object of class CSTC  
  * @retval none
  */
void STC_ForceSpeedReferenceToCurrentSpeed(CSTC this);

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /* __SPEEDNTORQCTRLCLASS_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
