/**
  ******************************************************************************
  * @file    MCInterfaceClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of MCInterface class      
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
#ifndef __MCINTERFACECLASS_H
#define __MCINTERFACECLASS_H

/* Includes ------------------------------------------------------------------*/
#include "MC_type.h"

/** @addtogroup STM32_PMSM_MC_Application
  * @{
  */

/** @addtogroup MCInterface
  * @{
  */
  
/** @defgroup MCInterface_class_exported_types MCInterface class exported types
* @{
*/

typedef enum {
	MCI_BUFFER_EMPTY,                  /*!< If no buffered command has been 
                                             called.*/
	MCI_COMMAND_NOT_ALREADY_EXECUTED,  /*!< If the buffered command condition 
                                             hasn't already occurred.*/
	MCI_COMMAND_EXECUTED_SUCCESFULLY,  /*!< If the buffered command has been 
                                             executed successfully.*/
	MCI_COMMAND_EXECUTED_UNSUCCESFULLY /*!< If the buffered command has been 
                                             executed unsuccessfully.*/
} CommandState_t ;

typedef enum {
  MCI_NOCOMMANDSYET,        /*!< No command has been set by the user.*/
  MCI_EXECSPEEDRAMP,        /*!< ExecSpeedRamp command coming from the user.*/
  MCI_EXECTORQUERAMP,       /*!< ExecTorqueRamp command coming from the user.*/
  MCI_SETCURRENTREFERENCES, /*!< SetCurrentReferences command coming from the 
                                 user.*/
} UserCommands_t;

/** 
  * @brief  Public MCInterface class definition 
  */
typedef struct CMCI_t *CMCI;

/** 
  * @brief  MCInterface class parameters definition  
  */
typedef const void MCInterfaceParams_t, *pMCInterfaceParams_t;
  
/**
* @}
*/

/** @defgroup MCInterface_class_exported_methods MCInterface class exported methods
  * @{
  */

/**
  * @brief  This is a buffered command to set a motor speed ramp. This commands
  *         don't become active as soon as it is called but it will be executed
  *         when the oSTM state is START_RUN or RUN. User can check the status
  *         of the command calling the MCI_IsCommandAcknowledged method.
  * @param  this related object of class CMCI.
  * @param  hFinalSpeed is the value of mechanical rotor speed reference at the
  *         end of the ramp expressed in tenths of HZ.
  * @param  hDurationms the duration of the ramp expressed in milliseconds. It 
  *         is possible to set 0 to perform an instantaneous change in the 
  *         value.
  * @retval none.
  */
void MCI_ExecSpeedRamp(CMCI this,  int16_t hFinalSpeed, uint16_t hDurationms);

/**
  * @brief  This is a buffered command to set a motor torque ramp. This commands
  *         don't become active as soon as it is called but it will be executed
  *         when the oSTM state is START_RUN or RUN. User can check the status
  *         of the command calling the MCI_IsCommandAcknowledged method.
  * @param  this related object of class CMCI.
  * @param  hFinalTorque is the value of motor torque reference at the end of 
  *         the ramp. This value represents actually the Iq current expressed in
  *         digit.
  *         To convert current expressed in Amps to current expressed in digit
  *         is possible to use the formula:
  *         Current (digit) = [Current(Amp) * 65536 * Rshunt * Aop] / Vdd micro.
  * @param  hDurationms the duration of the ramp expressed in milliseconds. It
  *         is possible to set 0 to perform an instantaneous change in the 
  *         value.
  * @retval none.
  */
void MCI_ExecTorqueRamp(CMCI this,  int16_t hFinalTorque, uint16_t hDurationms);

/**
  * @brief  This is a buffered command to set directly the motor current 
  *         references Iq and Id. This commands don't become active as soon as 
  *         it is called but it will be executed when the oSTM state is 
  *         START_RUN or RUN. User can check the status of the command calling 
  *         the MCI_IsCommandAcknowledged method.
  * @param  this related object of class CMCI.
  * @param  Iqdref current references on qd reference frame in Curr_Components
  *         format.
  * @retval none.
  */
void MCI_SetCurrentReferences(CMCI this, Curr_Components Iqdref);

/**
  * @brief  This is a user command used to begin the start-up procedure. 
  *         If the state machine is in IDLE state the command is executed 
  *         instantaneously otherwise the command is discarded. User must take
  *         care of this possibility by checking the return value. 
  *         Before calling MCI_StartMotor it is mandatory to execute one of 
  *         these commands:\n
  *         MCI_ExecSpeedRamp\n
  *         MCI_ExecTorqueRamp\n
  *         MCI_SetCurrentReferences\n	
  *         Otherwise the behaviour in run state will be unpredictable.\n
  *         <B>Note:</B> The MCI_StartMotor command is used just to begin the 
  *         start-up procedure moving the state machine from IDLE state to 
  *         IDLE_START. The command MCI_StartMotor is not blocking the execution
  *         of project until the motor is really running; to do this, the user 
  *         have to check the state machine and verify that the RUN state (or 
  *         any other state) has been reached. 
  * @param  this related object of class CMCI.
  * @retval bool It returns TRUE if the command is successfully executed 
  *         otherwise it return FALSE.
  */
bool MCI_StartMotor(CMCI this);

/**
  * @brief  This is a user command used to begin the stop motor procedure. 
  *         If the state machine is in RUN or START states the command is 
  *         executed instantaneously otherwise the command is discarded. User 
  *         must take care of this possibility by checking the return value.\n
  *         <B>Note:</B> The MCI_StopMotor command is used just to begin the 
  *         stop motor procedure moving the state machine to ANY_STOP.
  *         The command MCI_StopMotor is not blocking the execution of project
  *         until the motor is really stopped; to do this, the user have to 
  *         check the state machine and verify that the IDLE state has been 
  *         reached again.
  * @param  this related object of class CMCI.
  * @retval bool It returns TRUE if the command is successfully executed 
  *         otherwise it return FALSE.
  */
bool MCI_StopMotor(CMCI this);

/**
  * @brief  This is a user command used to indicate that the user has seen the 
  *         error condition. If is possible, the command is executed 
  *         instantaneously otherwise the command is discarded. User must take 
  *         care of this possibility by checking the return value.
  * @param  this related object of class CMCI.
  * @retval bool It returns TRUE if the command is successfully executed 
  *         otherwise it return FALSE.
  */
bool MCI_FaultAcknowledged(CMCI this);

/**
  * @brief  This is a user command used to begin the encoder alignment procedure. 
  *         If the state machine is in IDLE state the command is executed 
  *         instantaneously otherwise the command is discarded. User must take
  *         care of this possibility by checking the return value.\n
  *         <B>Note:</B> The MCI_EncoderAlign command is used just to begin the 
  *         encoder alignment procedure moving the state machine from IDLE state 
  *         to IDLE_ALIGNMENT. The command MCI_EncoderAlign is not blocking the 
  *         execution of project until the encoder is really calibrated; to do 
  *         this, the user have to check the state machine and verify that the 
  *         IDLE state has been reached again.
  * @param  this related object of class CMCI.
  * @retval bool It returns TRUE if the command is successfully executed 
  *         otherwise it return FALSE.
  */
bool MCI_EncoderAlign(CMCI this);

/**
  * @brief  It returns information about the state of the last buffered command.
  * @param  this related object of class CMCI.
  * @retval CommandState_t  It can be one of the following codes:
  *         - MCI_BUFFER_EMPTY if no buffered command has been called.
  *         - MCI_COMMAND_NOT_ALREADY_EXECUTED if the buffered command 
  *         condition hasn't already occurred.
  *         - MCI_COMMAND_EXECUTED_SUCCESFULLY if the buffered command has 
  *         been executed successfully. In this case calling this function reset
  *         the command state to BC_BUFFER_EMPTY.
  *         - MCI_COMMAND_EXECUTED_UNSUCCESFULLY if the buffered command has 
  *         been executed unsuccessfully. In this case calling this function 
  *         reset the command state to BC_BUFFER_EMPTY.
  */
CommandState_t  MCI_IsCommandAcknowledged(CMCI this);

/**
  * @brief  It returns information about the state of the related oSTM object.
  * @param  this related object of class CMCI.
  * @retval State_t It returns the current state of the related oSTM object.
  */
State_t  MCI_GetSTMState(CMCI this);

/**
  * @brief It returns a 16 bit fields containing information about faults
  *        historically occurred since the state machine has been moved into 
  *        FAULT_NOW state.
  * \n\link Fault_generation_error_codes Returned error codes are listed here \endlink
  * @param this object of class CSTM.
  * @retval uint16_t  16 bit fields with information about the faults 
  *         historically occurred since the state machine has been moved into 
  *         FAULT_NOW state.
  * \n\link Fault_generation_error_codes Returned error codes are listed here \endlink   
  */
uint16_t MCI_GetOccurredFaults(CMCI this);

/**
  * @brief It returns a 16 bit fields containing information about faults
  *        currently present.
  * \n\link Fault_generation_error_codes Returned error codes are listed here \endlink
  * @param this object of class CSTM.
  * @retval uint16_t  16 bit fields with information about about currently 
  *         present faults.
  * \n\link Fault_generation_error_codes Returned error codes are listed here \endlink   
  */
uint16_t MCI_GetCurrentFaults(CMCI this);

/**
  * @brief  It returns information about the current mechanical rotor speed
  *         reference expressed in tenths of HZ.
  * @param  this related object of class CMCI.
  * @retval int16_t current mechanical rotor speed reference expressed in tenths
  *         of HZ.
  */
int16_t MCI_GetMecSpeedRef01Hz(CMCI this);

/**
  * @brief  It returns information about last computed average mechanical speed,
  *         expressed in 01Hz (tenth of Hertz).
  * @param  this related object of class CMCI.
  * @retval int16_t rotor average mechanical speed (01Hz).
  */
int16_t MCI_GetAvrgMecSpeed01Hz(CMCI this);

/**
  * @brief  It returns information about current motor measured torque. This
  *         value represents actually the Iq current expressed in digit.
  *         To convert current expressed in digit to current expressed in Amps
  *         is possible to use the formula: 
  *         Current(Amp) = [Current(digit) * Vdd micro] / [65536 * Rshunt * Aop]
  * @param  this related object of class CMCI.
  * @retval int16_t current motor measured torque. This value represents 
  *         actually the Iq ref current expressed in digit.
  */
int16_t MCI_GetTorque(CMCI this);

/**
  * @brief  It returns the motor phase current amplitude (0-to-peak) in s16A
  *         To convert s16A into Ampere following formula must be used: 
  *         Current(Amp) = [Current(s16A) * Vdd micro] / [65536 * Rshunt * Aop]
  * @param  this related object of class CMCI
  * @retval int16_t Motor phase current (0-to-peak) in s16A
  */
int16_t MCI_GetPhaseCurrentAmplitude(CMCI this);

/**
  * @brief  It returns the applied motor phase voltage amplitude (0-to-peak) in 
  *         s16V. To convert s16V into Volts following formula must be used: 
  *         PhaseVoltage(V) = [PhaseVoltage(s16A) * Vbus(V)] /[sqrt(3) *32767]
  * @param  this related object of class CMCI
  * @retval int16_t Motor phase voltage (0-to-peak) in s16V
  */
int16_t MCI_GetPhaseVoltageAmplitude(CMCI this);

/**
  * @brief  It returns the modality of the speed and torque controller. 
  * @param  this related object of class CMCI.
  * @retval STC_Modality_t It returns the modality of STC. It can be one of 
  *         these two values: STC_TORQUE_MODE or STC_SPEED_MODE.
  */
STC_Modality_t MCI_GetControlMode(CMCI this);

/**
  * @brief  It returns the motor direction imposed by the last command 
  *         (MCI_ExecSpeedRamp, MCI_ExecTorqueRamp or MCI_SetCurrentReferences).
  * @param  this related object of class CMCI.
  * @retval int16_t It returns 1 or -1 according the sign of hFinalSpeed, 
  *         hFinalTorque or Iqdref.qI_Component1 of the last command.
  */
int16_t MCI_GetImposedMotorDirection(CMCI this);

/**
  * @brief  It returns information about the last ramp final speed sent by the 
  *         user expressed in tenths of HZ.
  * @param  this related object of class CMCI.
  * @retval int16_t last ramp final speed sent by the user expressed in tehts
  *         of HZ.
  */
int16_t MCI_GetLastRampFinalSpeed(CMCI this);

/**
  * @brief  Check if the settled speed or torque ramp has been completed.
  * @param  this related object of class CMCI.
  * @retval bool It returns TRUE if the ramp is completed, FALSE otherwise.
  */
bool MCI_RampCompleted(CMCI this);

/**
  * @brief  Stop the execution of speed ramp.
  * @param  this related object of class CMCI.
  * @retval bool It returns TRUE if the command is executed, FALSE otherwise.
  */
bool MCI_StopSpeedRamp(CMCI this);

/**
  * @brief  It returns speed sensor reliability with reference to the sensor 
  *         actually used for reference frame transformation
  * @param  this related object of class CMCI.
  * @retval bool It returns TRUE if the speed sensor utilized for reference 
  *         frame transformation and (in speed control mode) for speed 
  *         regulation is reliable, FALSE otherwise
  */
bool MCI_GetSpdSensorReliability(CMCI this);

/**
  * @brief  It returns the last computed average mechanical speed, expressed in
  *         01Hz (tenth of Hertz) and related to the sensor actually used by FOC 
  *         algorithm
  * @param  this related object of class CMCI.
  * @retval int16_t rotor average mechanical speed (01Hz)
  */
int16_t MCI_GetAvrgMecSpeed01Hz(CMCI this);

/**
  * @brief  Get the current mechanical rotor speed reference expressed in tenths
  *         of HZ. 
  * @param  this related object of class CMCI.
  * @retval int16_t current mechanical rotor speed reference expressed in tenths
  *         of HZ. 
  */
int16_t MCI_GetMecSpeedRef01Hz(CMCI this);

/**
  * @brief  It returns stator current Iab in Curr_Components format
  * @param  this related object of class CMCI.
  * @retval Curr_Components Stator current Iab
  */
Curr_Components MCI_GetIab(CMCI this);

/**
  * @brief  It returns stator current Ialphabeta in Curr_Components format
  * @param  this related object of class CMCI.
  * @retval Curr_Components Stator current Ialphabeta
  */
Curr_Components MCI_GetIalphabeta(CMCI this);

/**
  * @brief  It returns stator current Iqd in Curr_Components format
  * @param  this related object of class CMCI.
  * @retval Curr_Components Stator current Iqd
  */
Curr_Components MCI_GetIqd(CMCI this);

/**
  * @brief  It returns stator current IqdHF in Curr_Components format
  * @param  this related object of class CMCI.
  * @retval Curr_Components Stator current IqdHF if HFI is selected as main
  *         sensor. Otherwise it returns { 0, 0}.
  */
Curr_Components MCI_GetIqdHF(CMCI this);

/**
  * @brief  It returns stator current Iqdref in Curr_Components format
  * @param  this related object of class CMCI.
  * @retval Curr_Components Stator current Iqdref
  */
Curr_Components MCI_GetIqdref(CMCI this);

/**
  * @brief  It returns stator current Vqd in Volt_Components format
  * @param  this related object of class CMCI.
  * @retval Curr_Components Stator current Vqd
  */
Volt_Components MCI_GetVqd(CMCI this);
 
/**
  * @brief  It returns stator current Valphabeta in Volt_Components format
  * @param  this related object of class CMCI.
  * @retval Curr_Components Stator current Valphabeta
  */
Volt_Components MCI_GetValphabeta(CMCI this);

/**
  * @brief  It returns the rotor electrical angle actually used for reference  
  *         frame transformation
  * @param  this related object of class CMCI.
  * @retval int16_t Rotor electrical angle in dpp format
  */
int16_t MCI_GetElAngledpp(CMCI this);

/**
  * @brief  It returns the reference eletrical torque, fed to derived class for
  *         Iqref and Idref computation
  * @param  this related object of class CMCI.
  * @retval int16_t Teref 
  */
int16_t MCI_GetTeref(CMCI this);

/**
  * @brief  It returns the motor phase current amplitude (0-to-peak) in s16A
  *         To convert s16A into Ampere following formula must be used: 
  *         Current(Amp) = [Current(s16A) * Vdd micro] / [65536 * Rshunt * Aop]
  * @param  this related object of class CMCI.
  * @retval int16_t Motor phase current (0-to-peak) in s16A
  */
int16_t MCI_GetPhaseCurrentAmplitude(CMCI this);

/**
  * @brief  It returns the applied motor phase voltage amplitude (0-to-peak) in 
  *         s16V. To convert s16V into Volts following formula must be used: 
  *         PhaseVoltage(V) = [PhaseVoltage(s16A) * Vbus(V)] /[sqrt(3) *32767]
  * @param  this related object of class CMCI.
  * @retval int16_t Motor phase voltage (0-to-peak) in s16V
  */
int16_t MCI_GetPhaseVoltageAmplitude(CMCI this);

/**
  * @brief  When bDriveInput is set to INTERNAL, Idref should is normally managed
  *         by FOC_CalcCurrRef. Neverthless, this method allows forcing changing 
  *         Idref value. Method call has no effect when either flux weakening 
  *         region is entered or MTPA is enabled
  * @param  this related object of class CMCI.
  * @param  int16_t New target Id value
  * @retval none
  */
void MCI_SetIdref(CMCI this, int16_t hNewIdref);

/**
  * @brief  It re-initializes Iqdref variables with their default values. 
  * @param  this related object of class CMCI.
  * @retval none
  */
void MCI_Clear_Iqdref(CMCI this);

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /* __MCINTERFACECLASS_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
