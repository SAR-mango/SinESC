/**
  ******************************************************************************
  * @file    MCInterfaceClass.c
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

/* Includes ------------------------------------------------------------------*/
#include "MC_Math.h"
#include "SpeednTorqCtrlClass.h"
#include "StateMachineClass.h"

#include "MCInterfaceClass.h"
#include "MCInterfaceClassPrivate.h"
#include "MCInterfacePrivate.h"

#ifdef MC_CLASS_DYNAMIC
  #include "stdlib.h" /* Used for dynamic allocation */
#else
  #include "MC_type.h"
  #define MAX_MCI_NUM 2u

  _CMCI_t MCIpool[MAX_MCI_NUM];
  unsigned char MCI_Allocated = 0u;
#endif
  
#define CLASS_VARS   &((_CMCI)this)->Vars_str

/* This macro converts the exported enum from the state machine to the related 
   bit field.*/
#define BC(state) (1u<<((uint16_t)((uint8_t)(state))))

/* Private function prototypes */
     
/**
  * @brief  Creates an object of the class MCInterface
  * @param  pMCInterfaceParams pointer to an MCInterface parameters structure
  * @retval CMCI new instance of MCInterface object
  */
CMCI MCI_NewObject(pMCInterfaceParams_t pMCInterfaceParams)
{
  _CMCI _oMCI;
  
  #ifdef MC_CLASS_DYNAMIC
    _oMCI = (_CMCI)calloc(1u,sizeof(_CMCI_t));
  #else
    if (MCI_Allocated  < MAX_MCI_NUM)
    {
      _oMCI = &MCIpool[MCI_Allocated++];
    }
    else
    {
      _oMCI = MC_NULL;
    }
  #endif
  
  _oMCI->pParams_str = (pParams_t)pMCInterfaceParams;
  
  return ((CMCI)_oMCI);
}

/**
  * @brief  Initializes all the object variables, usually it has to be called 
  *         once right after object creation. It is also used to assign the 
  *         state machine object, the speed and torque controller, and the FOC
  *         drive object to be used by MC Interface.
  * @param  this related object of class CMCI.
  * @param  oSTM the state machine object used by the MCI.
  * @param  oSTC the speed and torque controller used by the MCI.
  * @param  pFOCVars pointer to FOC vars to be used by MCI.
  * @retval none.
  */
void MCI_Init(CMCI this, CSTM oSTM, CSTC oSTC, pFOCVars_t pFOCVars)
{
  pVars_t pVars = CLASS_VARS;

  pVars->oSTM = oSTM;
  pVars->oSTC = oSTC;
  pVars->pFOCVars = pFOCVars;

  /* Buffer related initialization */
  pVars->lastCommand = MCI_NOCOMMANDSYET;
  pVars->hFinalSpeed = 0;
  pVars->hFinalTorque = 0;
  pVars->hDurationms = 0;
  pVars->CommandState = MCI_BUFFER_EMPTY;
}

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
void MCI_ExecSpeedRamp(CMCI this,  int16_t hFinalSpeed, uint16_t hDurationms)
{
  pVars_t pVars = CLASS_VARS;

  pVars->lastCommand = MCI_EXECSPEEDRAMP;
  pVars->hFinalSpeed = hFinalSpeed;
  pVars->hDurationms = hDurationms;
  pVars->CommandState = MCI_COMMAND_NOT_ALREADY_EXECUTED;
  pVars->LastModalitySetByUser = STC_SPEED_MODE;
}

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
void MCI_ExecTorqueRamp(CMCI this,  int16_t hFinalTorque, uint16_t hDurationms)
{
  pVars_t pVars = CLASS_VARS;
  
  pVars->lastCommand = MCI_EXECTORQUERAMP;
  pVars->hFinalTorque = hFinalTorque;
  pVars->hDurationms = hDurationms;
  pVars->CommandState = MCI_COMMAND_NOT_ALREADY_EXECUTED;
  pVars->LastModalitySetByUser = STC_TORQUE_MODE;
}

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
void MCI_SetCurrentReferences(CMCI this, Curr_Components Iqdref)
{
  pVars_t pVars = CLASS_VARS;
  
  pVars->lastCommand = MCI_SETCURRENTREFERENCES;
  pVars->Iqdref.qI_Component1 = Iqdref.qI_Component1;
  pVars->Iqdref.qI_Component2 = Iqdref.qI_Component2;
  pVars->CommandState = MCI_COMMAND_NOT_ALREADY_EXECUTED;
  pVars->LastModalitySetByUser = STC_TORQUE_MODE;
}

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
bool MCI_StartMotor(CMCI this)
{
  pVars_t pVars = CLASS_VARS;
  bool RetVal = STM_NextState(((_CMCI)this)->Vars_str.oSTM,IDLE_START);
  
  if (RetVal == TRUE)
  {
    pVars->CommandState = MCI_COMMAND_NOT_ALREADY_EXECUTED;
  }
  
  return RetVal;
}

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
bool MCI_StopMotor(CMCI this)
{
  return STM_NextState(((_CMCI)this)->Vars_str.oSTM,ANY_STOP);
}

/**
  * @brief  This is a user command used to indicate that the user has seen the 
  *         error condition. If is possible, the command is executed 
  *         instantaneously otherwise the command is discarded. User must take 
  *         care of this possibility by checking the return value.
  * @param  this related object of class CMCI.
  * @retval bool It returns TRUE if the command is successfully executed 
  *         otherwise it return FALSE.
  */
bool MCI_FaultAcknowledged(CMCI this)
{
  return STM_FaultAcknowledged(((_CMCI)this)->Vars_str.oSTM);
}

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
bool MCI_EncoderAlign(CMCI this)
{
  return STM_NextState(((_CMCI)this)->Vars_str.oSTM,IDLE_ALIGNMENT);
}

/**
  * @brief  This is usually a method managed by task. It must be called 
  *         periodically in order to check the status of the related oSTM object
  *         and eventually to execute the buffered command if the condition 
  *         occurs.
  * @param  this related object of class CMCI.
  * @retval none.
  */
void MCI_ExecBufferedCommands(CMCI this)
{
  pVars_t pVars = CLASS_VARS;
  
  if (this)
  {
    if (pVars->CommandState == MCI_COMMAND_NOT_ALREADY_EXECUTED)
    {
      bool commandHasBeenExecuted = FALSE;
      switch (pVars->lastCommand)
      {
      case MCI_EXECSPEEDRAMP:
        {
          pVars->pFOCVars->bDriveInput = INTERNAL;
          STC_SetControlMode(pVars->oSTC, STC_SPEED_MODE);
          commandHasBeenExecuted = STC_ExecRamp(pVars->oSTC, 
            pVars->hFinalSpeed, pVars->hDurationms);
        }
        break;
      case MCI_EXECTORQUERAMP:
        {
          pVars->pFOCVars->bDriveInput = INTERNAL;
          STC_SetControlMode(pVars->oSTC, STC_TORQUE_MODE);
          commandHasBeenExecuted = STC_ExecRamp(pVars->oSTC,
            pVars->hFinalTorque, pVars->hDurationms);
        }
        break;
      case MCI_SETCURRENTREFERENCES:
        {
          pVars->pFOCVars->bDriveInput = EXTERNAL;
          pVars->pFOCVars->Iqdref = pVars->Iqdref;
          commandHasBeenExecuted = TRUE;
        }
        break;
      default:
        break;
      }
      
      if (commandHasBeenExecuted)
      {
        pVars->CommandState = MCI_COMMAND_EXECUTED_SUCCESFULLY;
      }
      else
      {
        pVars->CommandState = MCI_COMMAND_EXECUTED_UNSUCCESFULLY;
      }
    }
  }
}

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
CommandState_t  MCI_IsCommandAcknowledged(CMCI this)
{
  pVars_t pVars = CLASS_VARS;
  CommandState_t retVal = pVars->CommandState;
  
  if ((retVal == MCI_COMMAND_EXECUTED_SUCCESFULLY) | 
      (retVal == MCI_COMMAND_EXECUTED_UNSUCCESFULLY))
  {
    pVars->CommandState = MCI_BUFFER_EMPTY;
  }
  return retVal;
}

/**
  * @brief  It returns information about the state of the related oSTM object.
  * @param  this related object of class CMCI.
  * @retval State_t It returns the current state of the related oSTM object.
  */
State_t  MCI_GetSTMState(CMCI this)
{
  return STM_GetState(((_CMCI)this)->Vars_str.oSTM);
}

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
uint16_t MCI_GetOccurredFaults(CMCI this)
{
  return (uint16_t)(STM_GetFaultState(((_CMCI)this)->Vars_str.oSTM));
}

/**
  * @brief It returns a 16 bit fields containing information about faults
  *        currently present.
  * \n\link Fault_generation_error_codes Returned error codes are listed here \endlink
  * @param this object of class CSTM.
  * @retval uint16_t  16 bit fields with information about about currently 
  *         present faults.
  * \n\link Fault_generation_error_codes Returned error codes are listed here \endlink   
  */
uint16_t MCI_GetCurrentFaults(CMCI this)
{
  return (uint16_t)(STM_GetFaultState(((_CMCI)this)->Vars_str.oSTM) >> 16);
}

/**
  * @brief  It returns the modality of the speed and torque controller. 
  * @param  this related object of class CMCI.
  * @retval STC_Modality_t It returns the modality of STC. It can be one of 
  *         these two values: STC_TORQUE_MODE or STC_SPEED_MODE.
  */
STC_Modality_t MCI_GetControlMode(CMCI this)
{
  pVars_t pVars = CLASS_VARS;
  
  return pVars->LastModalitySetByUser;
}

/**
  * @brief  It returns the motor direction imposed by the last command 
  *         (MCI_ExecSpeedRamp, MCI_ExecTorqueRamp or MCI_SetCurrentReferences).
  * @param  this related object of class CMCI.
  * @retval int16_t It returns 1 or -1 according the sign of hFinalSpeed, 
  *         hFinalTorque or Iqdref.qI_Component1 of the last command.
  */
int16_t MCI_GetImposedMotorDirection(CMCI this)
{
  int16_t retVal = 1;
  pVars_t pVars = CLASS_VARS;
  
  switch (pVars->CommandState)
  {
  case MCI_EXECSPEEDRAMP:
    if (pVars->hFinalSpeed < 0)
    {
      retVal = -1;
    }
    break;
  case MCI_EXECTORQUERAMP:
    if (pVars->hFinalTorque < 0)
    {
      retVal = -1;
    }
    break;
  case MCI_SETCURRENTREFERENCES:
    if (pVars->Iqdref.qI_Component1 < 0)
    {
      retVal = -1;
    }
    break;
  default:
    break;
  }
  return retVal;
}

/**
  * @brief  It returns information about the last ramp final speed sent by the 
  *         user expressed in tenths of HZ.
  * @param  this related object of class CMCI.
  * @retval int16_t last ramp final speed sent by the user expressed in tehts
  *         of HZ.
  */
int16_t MCI_GetLastRampFinalSpeed(CMCI this)
{
  int16_t hRetVal = 0;
  pVars_t pVars = CLASS_VARS;
  /* Examine the last buffered commands */
  if (pVars->lastCommand == MCI_EXECSPEEDRAMP)
  {
    hRetVal = pVars->hFinalSpeed;
  }
  return hRetVal;
}

/**
  * @brief  Check if the settled speed or torque ramp has been completed.
  * @param  this related object of class CMCI.
  * @retval bool It returns TRUE if the ramp is completed, FALSE otherwise.
  */
bool MCI_RampCompleted(CMCI this)
{
  bool retVal = FALSE;
  pVars_t pVars = CLASS_VARS;
  
  if((STM_GetState(((_CMCI)this)->Vars_str.oSTM))==RUN)
  {  
    retVal = STC_RampCompleted(pVars->oSTC);
  }
  return retVal;
}

/**
  * @brief  Stop the execution of speed ramp.
  * @param  this related object of class CMCI.
  * @retval bool It returns TRUE if the command is executed, FALSE otherwise.
  */
bool MCI_StopSpeedRamp(CMCI this)
{
  pVars_t pVars = CLASS_VARS;
  return STC_StopSpeedRamp(pVars->oSTC);
}

/**
  * @brief  It returns speed sensor reliability with reference to the sensor 
  *         actually used for reference frame transformation
  * @param  this related object of class CMCI.
  * @retval bool It returns TRUE if the speed sensor utilized for reference 
  *         frame transformation and (in speed control mode) for speed 
  *         regulation is reliable, FALSE otherwise
  */
bool MCI_GetSpdSensorReliability(CMCI this)
{
  pVars_t pVars = CLASS_VARS;
  CSPD oSPD = STC_GetSpeedSensor(pVars->oSTC);
  return(SPD_Check(oSPD));
}

/**
  * @brief  It returns the last computed average mechanical speed, expressed in
  *         01Hz (tenth of Hertz) and related to the sensor actually used by FOC 
  *         algorithm
  * @param  this related object of class CMCI.
  * @retval int16_t rotor average mechanical speed (01Hz)
  */
int16_t MCI_GetAvrgMecSpeed01Hz(CMCI this)
{
  pVars_t pVars = CLASS_VARS;
  CSPD oSPD = STC_GetSpeedSensor(pVars->oSTC);
  return(SPD_GetAvrgMecSpeed01Hz(oSPD));
}

/**
  * @brief  Get the current mechanical rotor speed reference expressed in tenths
  *         of HZ. 
  * @param  this related object of class CMCI.
  * @retval int16_t current mechanical rotor speed reference expressed in tenths
  *         of HZ. 
  */
int16_t MCI_GetMecSpeedRef01Hz(CMCI this)
{
  pVars_t pVars = CLASS_VARS;
  return(STC_GetMecSpeedRef01Hz(pVars->oSTC));
}


/**
  * @brief  It returns stator current Iab in Curr_Components format
  * @param  this related object of class CMCI.
  * @retval Curr_Components Stator current Iab
  */
Curr_Components MCI_GetIab(CMCI this)
{
  pVars_t pVars = CLASS_VARS;
  return (pVars->pFOCVars->Iab);
}

/**
  * @brief  It returns stator current Ialphabeta in Curr_Components format
  * @param  this related object of class CMCI.
  * @retval Curr_Components Stator current Ialphabeta
  */
Curr_Components MCI_GetIalphabeta(CMCI this)
{
  pVars_t pVars = CLASS_VARS;
  return (pVars->pFOCVars->Ialphabeta);
}

/**
  * @brief  It returns stator current Iqd in Curr_Components format
  * @param  this related object of class CMCI.
  * @retval Curr_Components Stator current Iqd
  */
Curr_Components MCI_GetIqd(CMCI this)
{
  pVars_t pVars = CLASS_VARS;
  return (pVars->pFOCVars->Iqd);
}

/**
  * @brief  It returns stator current IqdHF in Curr_Components format
  * @param  this related object of class CMCI.
  * @retval Curr_Components Stator current IqdHF if HFI is selected as main
  *         sensor. Otherwise it returns { 0, 0}.
  */
Curr_Components MCI_GetIqdHF(CMCI this)
{
  pVars_t pVars = CLASS_VARS;
  return (pVars->pFOCVars->IqdHF);
}

/**
  * @brief  It returns stator current Iqdref in Curr_Components format
  * @param  this related object of class CMCI.
  * @retval Curr_Components Stator current Iqdref
  */
Curr_Components MCI_GetIqdref(CMCI this)
{
  pVars_t pVars = CLASS_VARS;
  return (pVars->pFOCVars->Iqdref);
}

/**
  * @brief  It returns stator current Vqd in Volt_Components format
  * @param  this related object of class CMCI.
  * @retval Curr_Components Stator current Vqd
  */
Volt_Components MCI_GetVqd(CMCI this)
{
  pVars_t pVars = CLASS_VARS;
  return (pVars->pFOCVars->Vqd);
}
 
/**
  * @brief  It returns stator current Valphabeta in Volt_Components format
  * @param  this related object of class CMCI.
  * @retval Curr_Components Stator current Valphabeta
  */
Volt_Components MCI_GetValphabeta(CMCI this)
{
  pVars_t pVars = CLASS_VARS;
  return (pVars->pFOCVars->Valphabeta);
}

/**
  * @brief  It returns the rotor electrical angle actually used for reference  
  *         frame transformation
  * @param  this related object of class CMCI.
  * @retval int16_t Rotor electrical angle in dpp format
  */
int16_t MCI_GetElAngledpp(CMCI this)
{
  pVars_t pVars = CLASS_VARS;
  return (pVars->pFOCVars->hElAngle);
}

/**
  * @brief  It returns the reference eletrical torque, fed to derived class for
  *         Iqref and Idref computation
  * @param  this related object of class CMCI.
  * @retval int16_t Teref 
  */
int16_t MCI_GetTeref(CMCI this)
{
  pVars_t pVars = CLASS_VARS;
  return (pVars->pFOCVars->hTeref);
}

/**
  * @brief  It returns the motor phase current amplitude (0-to-peak) in s16A
  *         To convert s16A into Ampere following formula must be used: 
  *         Current(Amp) = [Current(s16A) * Vdd micro] / [65536 * Rshunt * Aop]
  * @param  this related object of class CMCI.
  * @retval int16_t Motor phase current (0-to-peak) in s16A
  */
int16_t MCI_GetPhaseCurrentAmplitude(CMCI this)
{
  pVars_t pVars = CLASS_VARS;
  Curr_Components Local_Curr;
  int32_t wAux1,wAux2;
  
  Local_Curr = pVars->pFOCVars->Ialphabeta;
  wAux1 = (int32_t)(Local_Curr.qI_Component1) * Local_Curr.qI_Component1;
  wAux2 = (int32_t)(Local_Curr.qI_Component2) * Local_Curr.qI_Component2;
  
  wAux1 += wAux2;
  wAux1 = MCM_Sqrt(wAux1);
  
  if (wAux1 > S16_MAX)
  {
    wAux1 = (int32_t)S16_MAX;
  }
  
  return((int16_t)wAux1);
}

/**
  * @brief  It returns the applied motor phase voltage amplitude (0-to-peak) in 
  *         s16V. To convert s16V into Volts following formula must be used: 
  *         PhaseVoltage(V) = [PhaseVoltage(s16A) * Vbus(V)] /[sqrt(3) *32767]
  * @param  this related object of class CMCI.
  * @retval int16_t Motor phase voltage (0-to-peak) in s16V
  */
int16_t MCI_GetPhaseVoltageAmplitude(CMCI this)
{
  pVars_t pVars = CLASS_VARS;
  Volt_Components Local_Voltage;
  int32_t wAux1,wAux2;
  
  Local_Voltage = pVars->pFOCVars->Valphabeta;
  wAux1 = (int32_t)(Local_Voltage.qV_Component1) * Local_Voltage.qV_Component1;
  wAux2 = (int32_t)(Local_Voltage.qV_Component2) * Local_Voltage.qV_Component2;
  
  wAux1 += wAux2;
  wAux1 = MCM_Sqrt(wAux1);
  
  if (wAux1 > S16_MAX)
  {
    wAux1 = (int32_t)S16_MAX;
  }
  
  return((int16_t)wAux1);
}

/**
  * @brief  When bDriveInput is set to INTERNAL, Idref should is normally managed
  *         by FOC_CalcCurrRef. Neverthless, this method allows forcing changing 
  *         Idref value. Method call has no effect when either flux weakening 
  *         region is entered or MTPA is enabled
  * @param  this related object of class CMCI.
  * @param  int16_t New target Id value
  * @retval none
  */
void MCI_SetIdref(CMCI this, int16_t hNewIdref)
{
  pVars_t pVars = CLASS_VARS;
  pVars->pFOCVars->Iqdref.qI_Component2 = hNewIdref;
  pVars->pFOCVars->UserIdref = hNewIdref;
}

/**
  * @brief  It re-initializes Iqdref variables with their default values. 
  * @param  this related object of class CMCI.
  * @retval none
  */
void MCI_Clear_Iqdref(CMCI this)
{
  pVars_t pVars = CLASS_VARS;
  pVars->pFOCVars->Iqdref = STC_GetDefaultIqdref(pVars->oSTC);
}

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
