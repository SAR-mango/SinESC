/**
  ******************************************************************************
  * @file    SelfComCtrlClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of SelfComCtrl class      
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
#ifndef __SELFCOMCTRLCLASS_H
#define __SELFCOMCTRLCLASS_H

/* Includes ------------------------------------------------------------------*/
#include "MC_type.h"
#include "PWMnCurrFdbkClass.h"
#include "RampExtMngrClass.h"
#include "BusVoltageSensorClass.h"
#include "StateMachineClass.h"
#include "SpeednPosFdbkClass.h"
#include "VirtualSpeedSensor_SpeednPosFdbkClass.h"
#include "OpenLoopClass.h"
#include "CircleLimitationClass.h"
#include "PIRegulatorClass.h"
#include "RevupCtrlClass.h"
#include "SpeednTorqCtrlClass.h"

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */

/** @addtogroup SelfComCtrl
  * @{
  */
    
/** @defgroup SelfComCtrl_class_exported_types SelfComCtrl class exported types
* @{
*/

/** 
  * @brief  Public SelfComCtrl class definition 
  */
typedef struct CSCC_t *CSCC;

/** 
  * @brief SelfComCtrl class init structure type definition
  */
typedef struct
{
  CPWMC oPWMC;          /*!< Current feedback and PWM object used.*/
  CVBS  oVBS;           /*!< Bus voltage sensor used.*/
  pFOCVars_t pFOCVars;  /*!< Related structure of FOC vars.*/
  CSTM  oSTM;           /*!< State macchine of related MC.*/
  CSPD oVSS;            /*!< VSS used.*/
  CCLM oCLM;            /*!< Circle limitation used.*/
  CPI oPIDIq;           /*!< Iq PID used.*/
  CPI oPIDId;           /*!< Id PID used.*/
  CRUC oRUC;            /*!< RUC used.*/
  CSPD oSTO;            /*!< State Observer used.*/
  CSTC oSTC;            /*!< Speed and torque controller used.*/
  COTT oOTT;            /*!< One touch tuning object used.*/
} SCC_Init_t, *pSCC_Init_t;

/** 
  * @brief  SelfComCtrl class parameters definition  
  */
typedef const struct
{
  RampExtMngrParams_t rampExtMngrParams; /*!< Ramp manager used by SCC.*/
  float fRshunt;                         /*!< Value of shunt resistor.*/
  float fAmplificationGain;              /*!< Current sensing amplification gain.*/
  float fVbusConvFactor;                 /*!< Bus voltage conversion factor.*/
  float fVbusPartitioningFactor;         /*!< Bus voltage partitioning factor. 
                                              (Vmcu / Bus voltage conversion factor).*/
  float fRVNK;                           /*!< Power stage calibration factor. 
                                              (Measured sperimentally).*/
  float fRSMeasCurrLevelMax;             /*!< Maxium level of DC current used
                                              for RS measurement.*/
  uint16_t hDutyRampDuration;            /*!< Duration of voltage ramp executed 
                                              for the duty cycle determination
                                              stage.*/
  uint16_t hAlignmentDuration;           /*!< Duration of the alignment stage.*/
  uint16_t hRSDetectionDuration;         /*!< Duration of R detetction stage.*/
  float fLdLqRatio;                      /*!< Ld vs Lq ratio.*/
  float fCurrentBW;                      /*!< Bandwidth of current regulator.*/
  bool bPBCharacterization;              /*!< Set to TRUE for characterization
                                              of power board, otherwise FALSE.*/
  int32_t wNominalSpeed;                 /*!< Nominal speed set by the user expressed in RPM.*/
  uint16_t hPWMFreqHz;                   /*!< PWM frequency used for the test.*/
  uint8_t bFOCRepRate;                   /*!< FOC repetition rate used for the test.*/
  float fMCUPowerSupply;				 /*!< MCU Power Supply */
} SelfComCtrlParams_t, *pSelfComCtrlParams_t;

/** 
* @brief  SCC_State_t enum type definition, it lists all the possible SCC state
          machine states.
*/
typedef enum
{
  SCC_IDLE,
  SCC_DUTY_DETECTING_PHASE,
  SCC_ALIGN_PHASE,
  SCC_RS_DETECTING_PHASE_RAMP,
  SCC_RS_DETECTING_PHASE,
  SCC_LS_DETECTING_PHASE,
  SCC_WAIT_RESTART,
  SCC_RESTART_SCC,
  SCC_KE_DETECTING_PHASE,
  SCC_STOP,
  SCC_CALIBRATION_END
} SCC_State_t;
  
/**
* @}
*/

/** @defgroup SelfComCtrl_class_exported_methods SelfComCtrl class exported methods
  * @{
  */

/**
  * @brief  Creates an object of the class SelfComCtrl
  * @param  pSelfComCtrlParams pointer to an SelfComCtrl parameters structure
  * @retval CSCC new instance of SelfComCtrl object
  */
CSCC SCC_NewObject(pSelfComCtrlParams_t pSelfComCtrlParams);

/**
  * @brief  Initializes all the object variables, usually it has to be called 
  *         once right after object creation.
  * @param  this related object of class CSCC.
  * @param  pSCC_Init pointer to the SCC init structure.
  * @retval none.
  */
void SCC_Init(CSCC this, pSCC_Init_t pSCC_Init);

/**
  * @brief  It should be called before each motor restart.
  * @param this related object of class CSCC.
  * @retval bool It return FALSE if function fails to start the SCC.
  */
bool SCC_Start(CSCC this);

/**
  * @brief  It should be called before each motor stop.
  * @param this related object of class CSCC.
  * @retval none.
  */
void SCC_Stop(CSCC this);

/**
  * @brief  It feed the required phase voltage to the inverter.
  * @param  this related object of class CSCC.
  * @retval It returns the code error 'MC_FOC_DURATION' if any, 'MC_NO_ERROR' 
  *         otherwise. These error codes are defined in MC_type.h
  */
uint16_t SCC_SetPhaseVoltage(CSCC this);

/**
  * @brief  Medium frequency task.
  * @param  this related object of class CSCC.
  * @retval none
  */
void SCC_MF(CSCC this);
    
/**
  * @brief  It returns the state of Selfcommissioning procedure.
  * @param  this related object of class CSCC.
  * @retval uint8_t It returns the state of Selfcommissioning procedure.
  */
uint8_t SCC_GetState(CSCC this);

/**
  * @brief  It returns the number of states of Selfcommissioning procedure.
  * @param  this related object of class CSCC.
  * @retval uint8_t It returns the number of states of Selfcommissioning procedure.
  */
uint8_t SCC_GetSteps(CSCC this);

/**
  * @brief  It returns the measured Rs.
  * @param  this related object of class CSCC.
  * @retval uint32_t It returns the measured Rs, it is a floating point number
  *         codified into a 32bit integer.
  */
uint32_t SCC_GetRs(CSCC this);

/**
  * @brief  It returns the measured Ls.
  * @param  this related object of class CSCC.
  * @retval uint32_t It returns the measured Ls, it is a floating point number
  *         codified into a 32bit integer.
  */
uint32_t SCC_GetLs(CSCC this);

/**
  * @brief  It returns the measured Ke.
  * @param  this related object of class CSCC.
  * @retval uint32_t It returns the measured Ke, it is a floating point number
  *         codified into a 32bit integer.
  */
uint32_t SCC_GetKe(CSCC this);

/**
  * @brief  It returns the measured VBus.
  * @param  this related object of class CSCC.
  * @retval uint32_t It returns the measured Vbus, it is a floating point number
  *         codified into a 32bit integer.
  */
uint32_t SCC_GetVbus(CSCC this);

/**
  * @brief  It returns the nominal speed estimated from Ke.
  * @param  this related object of class CSCC.
  * @retval uint32_t It returns the nominal speed estimated from Ke, it is a 
  *         floating point number codified into a 32bit integer.
  */
uint32_t SCC_GetEstNominalSpeed(CSCC this);

/**
  * @brief  Call this method before start motor to force new motor profile.
  * @param  this related object of class CSCC.
  * @retval none
  */
void SCC_ForceProfile(CSCC this);

/**
  * @brief  Call this method to force end of motor profile.
  * @param  this related object of class CSCC.
  * @retval none
  */
void SCC_StopProfile(CSCC this);

/**
  * @brief  Sets the number of motor poles pairs.
  *         This function shall be called before the start 
  *         of the MP procedure.
  * @param  this related object of class CSCC.
  * @param  bPP Number of motor poles pairs to be set.
  * @retval none
  */
void SCC_SetPolesPairs(CSCC this, uint8_t bPP);

/**
  * @brief  Change the current used for RL determination. 
            Usually is the nominal current of the motor.
  *         This function shall be called before the start 
  *         of the MP procedure.
  * @param  this related object of class CSCC.
  * @param  fCurrent Current used for RL determination.
  * @retval none
  */
void SCC_SetNominalCurrent(CSCC this, float fCurrent);

/**
  * @brief  Get the nominal current used for RL determination. 
  * @param  this related object of class CSCC.
  * @retval float Nominal current used for RL determination.
  */
float SCC_GetNominalCurrent(CSCC this);

/**
  * @brief  Set the Ld/Lq ratio.
  *         This function shall be called before the start 
  *         of the MP procedure.
  * @param  this related object of class CSCC.
  * @param  fLdLqRatio New value of Lq/Lq ratio used by MP for tuning of 
            current regulators.
  * @retval none
  */
void SCC_SetLdLqRatio(CSCC this, float fLdLqRatio);

/**
  * @brief  Get the Ld/Lq ratio.
  * @param  this related object of class CSCC.
  * @retval float New value of Lq/Lq ratio used by MP for tuning of 
            current regulators.
  */
float SCC_GetLdLqRatio(CSCC this);

/**
  * @brief  Set the nominal speed according motor datasheet.
  *         This function shall be called before the start 
  *         of the MP procedure.
  * @param  this related object of class CSCC.
  * @param  wNominalSpeed Nominal speed expressed in RPM.
  * @retval none
  */
void SCC_SetNominalSpeed(CSCC this, int32_t wNominalSpeed);

/**
  * @brief  Get the last nominal speed set by SCC_SetNominalSpeed.
  *         Note that this is not the estimated one.
  * @param  this related object of class CSCC.
  * @retval int32_t Nominal speed expressed in RPM.
  */
int32_t SCC_GetNominalSpeed(CSCC this);

/**
  * @brief  Get the estimated maximum speed that can be
  *         sustatined in the startup open loop acceleration.
  *         This function must be called only after that the
  *         MP procedure is completed succesfully.
  * @param  this related object of class CSCC.
  * @retval int32_t Estimated maximum open loop speed expressed in RPM.
  */
int32_t SCC_GetEstMaxOLSpeed(CSCC this);

/**
  * @brief  Get the estimated maximum acceleration that can be
  *         sustatined in the startup using the estimated 
  *         startup current. You can retireve the max startup
  *         current using the SCC_GetStartupCurrentX function.
  *         This function must be called only after that the
  *         MP procedure is completed succesfully.
  * @param  this related object of class CSCC.
  * @retval int32_t Estimated maximum open loop acceleration
  *         espressed in RPM/s.
  */
int32_t SCC_GetEstMaxAcceleration(CSCC this);

/**
  * @brief  Get the estimated maximum statup current that
  *         can be applied to the selected motor.
  *         This function must be called only after that the
  *         MP procedure is completed succesfully.
  * @param  this related object of class CSCC.
  * @retval int16_t Estimated maximum open loop current
  *         espressed in s16int.
  */
int16_t SCC_GetStartupCurrentS16(CSCC this);

/**
  * @brief  Get the estimated maximum statup current that
  *         can be applied to the selected motor.
  *         This function must be called only after that the
  *         MP procedure is completed succesfully.
  * @param  this related object of class CSCC.
  * @retval int16_t Estimated maximum open loop current
  *         espressed in Ampere.
  */
float SCC_GetStartupCurrentAmp(CSCC this);

/**
  * @brief  Set the bandwidth used to tune the current regulators.
  *         This function shall be called before the start 
  *         of the MP procedure.
  * @param  this related object of class CSCC.
  * @param  fCurrentBW Bandwidth used to tune the current regulators expressed in rad/s.
  * @retval none
  */
void SCC_SetCurrentBandwidth(CSCC this, float fCurrentBW);

/**
  * @brief  Get the bandwidth used to tune the current regulators.
  * @param  this related object of class CSCC.
  *         This function must be called only after that the
  *         MP procedure is completed succesfully.
  * @retval float Bandwidth used to tune the current regulators expressed in 
  *         rad/s.
  */
float SCC_GetCurrentBandwidth(CSCC this);

/**
  * @brief  Get the PWM frequency used by the test.
  * @param  this related object of class CSCC.
  * @retval uint16_t PWM frequency used by the test expressed in Hz.
  */
uint16_t SCC_GetPWMFrequencyHz(CSCC this);

/**
  * @brief  Get the FOC repetition rate. It is the number of PWM
  *         periods elapsed before executing one FOC control cycle. 
  * @param  this related object of class CSCC.
  * @retval uint8_t FOC repetition used by the test.
  */
uint8_t SCC_GetFOCRepRate(CSCC this);

/**
  * @brief  Check overcurrent during RL detetction and restart the procedure 
  *         with less current.
  * @param  this related object of class CSCC.
  * @retval none.
  */
void SCC_CheckOC_RL(CSCC this);

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /* __SELFCOMCTRLCLASS_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
