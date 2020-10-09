/**
  ******************************************************************************
  * @file    MCTuningClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of MCTuning class      
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
#ifndef __MCTUNINGCLASS_H
#define __MCTUNINGCLASS_H

/* Includes ------------------------------------------------------------------*/
#include "MC_type.h"

/** @addtogroup STM32_PMSM_MC_Application
  * @{
  */

/** @addtogroup MCTuning
  * @{
  */
  
/** @defgroup MCTuning_class_exported_types MCTuning class exported types
* @{
*/

/** 
  * @brief  Public MCTuning class definition 
  */
typedef struct CMCT_t *CMCT;

/** 
  * @brief  MCTuning class parameters definition  
  */
typedef const void MCTuningParams_t, *pMCTuningParams_t;

/** 
  * @brief  Public OpenLoop class definition 
  */
#ifndef __OPENLOOPCLASS_H
typedef struct COL_t *COL;
#endif

/** 
  * @brief  Public PI regulator class definition  
  */
#ifndef __PIREGULATORCLASS_H
typedef struct CPI_t  *CPI;
#endif
/** 
  * @brief  Public PID class definition
  */
#ifndef __PID_PICLASS_H
typedef struct CPID_PI_t *CPID_PI;
#endif

/** 
  * @brief  Public PWMnCurrFdbk class definition 
  */
#ifndef __PWMNCURRFDBKCLASS_H
typedef struct CPWMC_t *CPWMC;
#endif

/** 
  * @brief  Public RevupCtrl class definition 
  */
#ifndef __REVUPCTRLCLASS_H
typedef struct CRUC_t *CRUC;
#endif

/** 
  * @brief  Public SpeednPosFdbk class definition 
  */
#ifndef __SPEEDNPOSFDBKCLASS_H
typedef struct CSPD_t *CSPD;
#endif

/** 
  * @brief  Public STO class definition
  */
#ifndef __STO_SPEEDNPOSFDBKCLASS_H
typedef struct CSTO_SPD_t *CSTO_SPD;
#endif

/** 
  * @brief  Public STO_CORDIC class definition
  */
#ifndef __STO_CORDIC_SPEEDNPOSFDBKCLASS_H
typedef struct CSTO_CR_SPD_t *CSTO_CR_SPD;
#endif

/** 
  * @brief  Public SpeednTorqCtrl class definition 
  */
#ifndef __SPEEDNTORQCTRLCLASS_H
typedef struct CSTC_t *CSTC;
#endif

/** 
  * @brief  Public StateMachine class definition 
  */
#ifndef __STATEMACHINECLASS_H
typedef struct CSTM_t *CSTM;
#endif

/** 
  * @brief  Public temperature sensor class definition 
  */
#ifndef __TEMPERATURESENSORCLASS_H
typedef struct CTSNS_t *CTSNS;
#endif

/** 
  * @brief  Public BusVoltageSensor class definition 
  */
#ifndef __BUSVOLTAGESENSORCLASS_H
typedef struct CVBS_t *CVBS;
#endif

/** 
  * @brief  Public DigitalOutput class definition 
  */
#ifndef __DIGITALOUTPUTCLASS_H
typedef struct CDOUT_t *CDOUT;
#endif

/** 
  * @brief  Public MotorPowerMeasurement class definition 
  */
#ifndef __MOTORPOWERMEASUREMENTCLASS_H
typedef struct CMPM_t *CMPM;
#endif

/** 
  * @brief  Public FluxWeakeningCtrl class definition 
  */
#ifndef __FLUXWEAKENINGCTRLCLASS_H
typedef struct CFW_t *CFW;
#endif

/** 
  * @brief  Public FeedForwardCtrl class definition 
  */
#ifndef __FEEDFORWARDCTRLCLASS_H
typedef struct CFF_t *CFF;
#endif

/** 
  * @brief  Public HiFreqInj_FPU_Ctrl class definition 
  */
#ifndef __HFINJCTRLCLASS_H
typedef struct CHFI_FP_t *CHFI_FP;
#endif

/** 
  * @brief  Public SelfComCtrl class definition 
  */
#ifndef __SELFCOMCTRLCLASS_H
typedef struct CSCC_t *CSCC;
#endif

/** 
  * @brief  Public OneTouchTuning class definition 
  */
#ifndef __ONETOUCHTUNINGCLASS_H
typedef struct COTT_t *COTT;
#endif

/**
* @}
*/

/** @defgroup MCTuning_class_exported_methods MCTuning class exported methods
  * @{
  */

/**
  * @brief  It returns the FluxWeakeningCtrl object
  * \n\link FW_class_exported_methodsT Methods listed here can be applied \endlink
  * @param  this related object of class CMCT
  * @retval CFW, instance of FluxWeakeningCtrl Class
  */
CFW MCT_GetFluxWeakeningCtrl(CMCT this);

/**
  * @brief  It returns the FeedForwardCtrl object
  * \n\link FF_class_exported_methodsT Methods listed here can be applied \endlink
  * @param  this related object of class CMCT
  * @retval CFF, instance of FeedForwardCtrl Class
  */
CFF MCT_GetFeedForwardCtrl(CMCT this);

/**
  * @brief  It returns the HFICtrl object
  * \n\link HFI_class_exported_methodsT Methods listed here can be applied \endlink
  * @param  this related object of class CMCT
  * @retval CHFI_FP, instance of HFICtrl Class
  */
CHFI_FP MCT_GetHFICtrl(CMCT this);

/**
  * @brief  It returns the speed control loop PI(D) object
  * \n\link PI_Regulator_Class_Exported_MethodsT Methods listed here can be applied \endlink
  * \n\link PID_class_exported_methodsT Methods listed here can be applied if
  *         the object belongs to derived class PID \endlink
  * @param  this related object of class CMCT
  * @retval CPI, instance of PIRegulator Class
  */
CPI MCT_GetSpeedLoopPID(CMCT this);

/**
  * @brief  It returns the Iq current control loop PI(D) object
  * \n\link PI_Regulator_Class_Exported_MethodsT Methods listed here can be applied \endlink
  * \n\link PID_class_exported_methodsT Methods listed here can be applied if
  *         the object belongs to derived class PID \endlink
  * @param  this related object of class CMCT
  * @retval CPI, instance of PIRegulator Class
  */
CPI MCT_GetIqLoopPID(CMCT this);

/**
  * @brief  It returns the Id current control loop PI(D) object
  * \n\link PI_Regulator_Class_Exported_MethodsT Methods listed here can be applied \endlink
  * \n\link PID_class_exported_methodsT Methods listed here can be applied if
  *         the object belongs to derived class PID \endlink
  * @param  this related object of class CMCT
  * @retval CPI, instance of PIRegulator Class
  */
CPI MCT_GetIdLoopPID(CMCT this);

/**
  * @brief  It returns the Flux Weakening control loop PI(D) object
  * \n\link PI_Regulator_Class_Exported_MethodsT Methods listed here can be applied \endlink
  * \n\link PID_class_exported_methodsT Methods listed here can be applied if
  *         the object belongs to derived class PID \endlink
  * @param  this related object of class CMCT
  * @retval CPI, instance of PI RegulatorClass; MC_NULL if Flux Weakening is not
  *         enabled
  */
CPI MCT_GetFluxWeakeningLoopPID(CMCT this);

/**
  * @brief  It returns the PWMnCurrFdbk object
  * \n\link PWMnCurrFdbk_class_exported_methodsT Methods listed here can be applied \endlink
  * @param  this related object of class CMCT
  * @retval CPWMC, instance of PWMnCurrFdbk Class
  */
CPWMC MCT_GetPWMnCurrFdbk(CMCT this);

/**
  * @brief  It returns the Rev-up controller object
  * \n\link RevupCtrl_class_exported_methodsT Methods listed here can be applied \endlink
  * @param  this related object of class CMCT
  * @retval CRUC, instance of RevupCtrl Class
  */
CRUC MCT_GetRevupCtrl(CMCT this);

/**
  * @brief  It returns the Main Speed'n Position sensor object.
  *         Main position sensor is considered the one used to execute FOC
  * \n\link SpeednPosFdbk_class_exported_methodsT Methods listed here can be applied \endlink
  * \n\link STO_class_exported_methodsT Methods listed here can be applied if
  *         the object belongs to derived class STO (StateObserver) \endlink
  * @param  this related object of class CMCT
  * @retval CSPD, instance of SpeednPosFdbk Class
  */
CSPD MCT_GetSpeednPosSensorMain(CMCT this);

/**
  * @brief  It returns the Auxiliary Speed'n Position sensor object.
  *         Auxiliary position sensor is considered the one used to backup/tune
  *         the main one
  * \n\link SpeednPosFdbk_class_exported_methodsT Methods listed here can be applied \endlink
  * \n\link STO_class_exported_methodsT Methods listed here can be applied if
  *         the object belongs to derived class STO (StateObserver) \endlink
  * @param  this related object of class CMCT
  * @retval CSPD, instance of SpeednPosFdbk Class; MC_NULL if no auxiliary sensor
  *         is active
  */
CSPD MCT_GetSpeednPosSensorAuxiliary(CMCT this);

/**
  * @brief  It returns the Virtual Speed'n Position sensor object.
  *         Virtual position sensor is considered the one used to rev-up the
  *         motor during the start-up procedure required by the state-observer
  *         sensorless algorithm
  * \n\link SpeednPosFdbk_class_exported_methodsT Methods listed here can be applied \endlink
  * @param  this related object of class CMCT
  * @retval CSPD, instance of SpeednPosFdbk Class; MC_NULL if no auxiliary sensor
  *         is active
  */
CSPD MCT_GetSpeednPosSensorVirtual(CMCT this);

/**
  * @brief  It returns the Speed'n Torque Controller object.
  * \n\link SpeednTorqCtrl_class_exported_methodsT Methods listed here can be applied \endlink
  * @param  this related object of class CMCT
  * @retval CSTC, instance of SpeednTorqCtrl Class
  */
CSTC MCT_GetSpeednTorqueController(CMCT this);

/**
  * @brief  It returns the State Machine object.
  * \n\link StateMachine_class_exported_methodsT Methods listed here can be applied \endlink
  * @param  this related object of class CMCT
  * @retval CSTM, instance of StateMachine Class
  */
CSTM MCT_GetStateMachine(CMCT this);

/**
  * @brief  It returns the Temperature sensor object.
  * \n\link Temperature_sensor_class_exported_methodsT Methods listed here can be applied \endlink
  * @param  this related object of class CMCT
  * @retval CTSNS, instance of TemperatureSensor Class
  */
CTSNS MCT_GetTemperatureSensor(CMCT this);

/**
  * @brief  It returns the Bus Voltage sensor object
  * \n\link BusVoltageSensor_class_exported_methodsT Methods listed here can be applied \endlink
  * @param  this related object of class CMCT
  * @retval CVBS, instance of BusVoltageSensor Class
  */
CVBS MCT_GetBusVoltageSensor(CMCT this);

/**
  * @brief  It returns the Brake resistor object
  * \n\link DigitalOutput_class_exported_methodsT Methods listed here can be applied \endlink
  * @param  this related object of class CMCT
  * @retval CDOUT, instance of DigitalOutput Class; MC_NULL if no Brake resistor
  *         is active
  */
CDOUT MCT_GetBrakeResistor(CMCT this);

/**
  * @brief  It returns the NTC Relay object
  * \n\link DigitalOutput_class_exported_methodsT Methods listed here can be applied \endlink
  * @param  this related object of class CMCT
  * @retval CDOUT, instance of DigitalOutput Class; MC_NULL if no Relay
  *         is active
  */
CDOUT MCT_GetNTCRelay(CMCT this);

/**
  * @brief  It returns the MotorPowerMeasurement object
  * \n\link MotorPowerMeasurement_class_exported_methodsT Methods listed here can be applied \endlink
  * @param  this related object of class CMCT
  * @retval CMPM, instance of MotorPowerMeasurement Class; MC_NULL if no MPM
  *         is active
  */
CMPM MCT_GetMotorPowerMeasurement(CMCT this);

/**
  * @brief  It returns the Selfcommissioning object
  * \n\link Selfcommisioning_class_exported_methodsT Methods listed here can be applied \endlink
  * @param  this related object of class CMCT
  * @retval CSCC, instance of Selfcommissioning Class; MC_NULL if no SCC
  *         is active
  */
CSCC MCT_GetSelfCommissioning(CMCT this);

/**
  * @brief  It returns the One touch tuning object
  * \n\link OneTouchTuning_class_exported_methodsT Methods listed here can be applied \endlink
  * @param  this related object of class CMCT
  * @retval COTT, instance of One touch tuning Class; MC_NULL if no OTT
  *         is active
  */
COTT MCT_GetOneTouchTuning(CMCT this);

/** @defgroup FW_class_exported_methodsT FluxWeakeningCtrl class exported methods
  * @{
  */

/**
  * @brief  Use this method to set a new value for the voltage reference used by 
  *         flux weakening algorithm
  * @param  this related object of class CFW
  * @param  uint16_t New target voltage value, expressend in tenth of percentage
  *         points of available voltage
* @retval none
  */
void FW_SetVref(CFW this, uint16_t hNewVref);

/**
  * @brief  It returns the present value of target voltage used by flux 
  *         weakening algorihtm
  * @param  this related object of class CFW
  * @retval int16_t Present target voltage value expressed in tenth of 
  *         percentage points of available voltage
  */
uint16_t FW_GetVref(CFW this);

/**
  * @brief  It returns the present value of voltage actually used by flux 
  *         weakening algorihtm
  * @param  this related object of class CFW
  * @retval int16_t Present averaged phase stator voltage value, expressed 
  *         in s16V (0-to-peak), where 
  *         PhaseVoltage(V) = [PhaseVoltage(s16A) * Vbus(V)] /[sqrt(3) *32767]
  */
int16_t FW_GetAvVAmplitude(CFW this);

/**
  * @brief  It returns the measure of present voltage actually used by flux 
  *         weakening algorihtm as percentage of available voltage.
  * @param  this related object of class CFW
  * @retval uint16_t Present averaged phase stator voltage value, expressed in 
  *         tenth of percentage points of available voltage
  */
uint16_t FW_GetAvVPercentage(CFW this);

/**
* @}
*/

/** @defgroup FF_class_exported_methodsT FeedForwardCtrl class exported methods
  * @{
  */

/**
  * @brief  Use this method to set new values for the constants utilized by 
  *         feed-forward algorithm
  * @param  this related object of class CFF
  * @param  IMFF_TuningStruct_t New values for the constants utilized by 
  *         feed-forward algorithm
* @retval none
  */
void FF_SetFFConstants(CFF this, FF_TuningStruct_t sNewConstants);

/**
  * @brief  Use this method to get present values for the constants utilized by 
  *         feed-forward algorithm
  * @param  this related object of class CFF
  * @retval IMFF_TuningStruct_t new values for the constants utilized by 
  *         feed-forward algorithm
  */
FF_TuningStruct_t FF_GetFFConstants(CFF this);

/**
  * @brief  Use this method to get present values for the Vqd feed-forward 
  *         components
  * @param  this related object of class CFF
  * @retval Volt_Components Vqd feed-forward components
  */
Volt_Components FF_GetVqdff(CFF this);

/**
  * @brief  Use this method to get values of the averaged output of qd axes 
  *         currents PI regulators
  * @param  this related object of class CFF
  * @retval Volt_Components Averaged output of qd axes currents PI regulators
  */
Volt_Components FF_GetVqdAvPIout(CFF this);

/**
* @}
*/

/** @defgroup HFI_class_exported_methodsT HFICtrl class exported methods
  * @{
  */

/**
  * @brief  It returns the rotor angle lock value
  * @param  this related object of class CHFI_FP
  * @retval int16_t Rotor angle lock value
  */
int16_t HFI_FP_GetRotorAngleLock(CHFI_FP this);

/**
  * @brief  It returns the saturation difference measured during the last 
  *         north/south identification stage.
  * @param  this related object of class CHFI_FP
  * @retval int16_t Saturation difference measured during the last north/south
  *         identification stage. 
  */
int16_t HFI_FP_GetSaturationDifference(CHFI_FP this);

/**
  * @brief  It return the quantity that shall be put in the DAC to tune the HFI
  * @param  this related object of class CHFI_FP
  * @retval int16_t HFI current
  */
int16_t HFI_FP_GetCurrent(CHFI_FP this);

/**
  * @brief  It returns the Track PI
  * @param  this related object of class CHFI_FP
  * @retval CPI Track PI 
  */
CPI HFI_FP_GetPITrack(CHFI_FP this);

/**
  * @brief  It set the min saturation difference used to validate the
  *         north/south identification stage.
  * @param  this related object of class CHFI_FP
  * @param  hMinSaturationDifference Min Saturation difference used to validate 
  *         the north/south identification stage.
  *         identification stage. 
  * @retval none
  */
void HFI_FP_SetMinSaturationDifference(CHFI_FP this, int16_t hMinSaturationDifference);

/**
* @}
*/

/** @defgroup OL_class_exported_methodsT OLCtrl class exported methods
  * @{
  */

/**
  * @brief  It allows changing applied open loop phase voltage.
  * @param  this related object of class COL
  * @param  hNewVoltage New voltage value to be applied by the open loop.
  * @retval None
  */
void OL_UpdateVoltage(COL this, int16_t hNewVoltage);

/**
* @}
*/

/** @defgroup PI_Regulator_Class_Exported_MethodsT PI regulator class exported 
  *                                               methods
  * @{
  */

/**
  * @brief  It updates the Kp gain 
  * @param  CPI PI object
  * @param  int16_t New Kp gain
  * @retval None
  */
void PI_SetKP(CPI this, int16_t hKpGain);

/**
  * @brief  It updates the Ki gain 
  * @param  CPI PI object
  * @param  int16_t New Ki gain
  * @retval None
  */
void PI_SetKI(CPI this, int16_t hKiGain);

/**
  * @brief  It returns the Kp gain of the passed PI object
  * @param  CPI PI regulator object
  * @retval int16_t Kp gain 
  */
int16_t PI_GetKP(CPI this);

/**
  * @brief  It returns the Kp gain divisor of the passed PI object
  * @param  CPI PI regulator object
  * @retval int16_t Kp gain 
  */
uint16_t PI_GetKPDivisor(CPI this);

/**
  * @brief  It returns the Ki gain of the passed PI object
  * @param  CPI PI regulator object
  * @retval int16_t Ki gain 
  */
int16_t PI_GetKI(CPI this);

/**
  * @brief  It returns the Ki gain divisor of the passed PI object
  * @param  CPI PI regulator object
  * @retval int16_t Ki gain 
  */
uint16_t PI_GetKIDivisor(CPI this);

/**
  * @brief  It returns the Default Kp gain of the passed PI object
  * @param  CPI PI regulator object
  * @retval int16_t Kp gain 
  */
int16_t PI_GetDefaultKP(CPI this);

/**
  * @brief  It returns the Default Ki gain of the passed PI object
  * @param  CPI PI regulator object
  * @retval int16_t Ki gain 
  */
int16_t PI_GetDefaultKI(CPI this);

/**
  * @brief  It set a new value into the PI integral term 
  * @param  CPI PI regulator object
  * @param  int32_t New integral term value
  * @retval None
  */
void PI_SetIntegralTerm(CPI this, int32_t wIntegralTermValue);

/**
* @}
*/

/** @defgroup PID_class_exported_methodsT PID class exported methods
  * @{
  */

/**
  * @brief  It set a new value into the PID Previous error variable required to 
  *         compute derivative term
  * @param  this regulator object
  * @param  wPrevProcessVarError New integral term value
  * @retval None
  */
void PID_SetPrevError(CPID_PI this, int32_t wPrevProcessVarError);

/**
  * @brief  It updates the Kd gain 
  * @param  this PID regulator object
  * @param  hKdGain New Kd gain
  * @retval None
  */
void PID_SetKD(CPID_PI this, int16_t hKdGain);

/**
  * @brief  It returns the Kd gain of the PID object passed 
  * @param  this PID regulator object
  * @retval int16_t Kd gain
  */
int16_t PID_GetKD(CPID_PI this);
/**
* @}
*/

/** @defgroup PWMnCurrFdbk_class_exported_methodsT PWMnCurrFdbk class exported methods
  * @{
  */

/**
* @brief  Execute a regular conversion using ADC1. 
*         The function is not re-entrant (can't executed twice at the same time)
*         It returns 0xFFFF in case of conversion error.
* @param  this related object of class CPWMC, ADC channel to be converted
* @param  bChannel ADC channel used for the regular conversion
* @retval It returns converted value or oxFFFF for conversion error
*/
uint16_t PWMC_ExecRegularConv(CPWMC this, uint8_t bChannel);

/**
* @brief  It sets the specified sampling time for the specified ADC channel
*         on ADC1. It must be called once for each channel utilized by user
* @param  this related object of class CPWMC
* @param  ADConv_struct struct containing ADC channel and sampling time
* @retval none
*/
void PWMC_ADC_SetSamplingTime(CPWMC this, ADConv_t ADConv_struct);

/**
* @}
*/

/** @defgroup RevupCtrl_class_exported_methodsT RevupCtrl class exported methods
  * @{
  */

/**
  * @brief  It is used to modify the default value of duration of a specific 
  *         rev up phase.
  *         Note: The module can be also compiled commenting the 
  *         define RUC_ALLOWS_TUNING to optimize the flash memory occupation
  *         and the RAM usage if the tuning is not required in this case this
  *         function has no effect.
  * @param  this related object of class CRUC.
  * @param  bPhase is the rev up phase, zero based, to be modified.
  * @param  hDurationms is the new value of duration for that phase.
  * @retval none. 
  */
void RUC_SetPhaseDurationms(CRUC this, uint8_t bPhase, uint16_t hDurationms);

/**
  * @brief  It is used to modify the default value of mechanical speed at the
  *         end of a specific rev up phase.
  *         Note: The module can be also compiled commenting the 
  *         define RUC_ALLOWS_TUNING to optimize the flash memory occupation
  *         and the RAM usage if the tuning is not required in this case this
  *         function has no effect.
  * @param  this related object of class CRUC.
  * @param  bPhase is the rev up phase, zero based, to be modified.
  * @param  hFinalMecSpeed01Hz is the new value of mechanical speed at the end
  *         of that phase expressed in 0.1Hz.
  * @retval none. 
  */
void RUC_SetPhaseFinalMecSpeed01Hz(CRUC this, uint8_t bPhase, 
                                   int16_t hFinalMecSpeed01Hz);

/**
  * @brief  It is used to modify the default value of motor torque at the end of
  *         a specific rev up phase.
  *         Note: The module can be also compiled commenting the
  *         define RUC_ALLOWS_TUNING to optimize the flash memory occupation
  *         and the RAM usage if the tuning is not required in this case this
  *         function has no effect.
  * @param  this related object of class CRUC.
  * @param  bPhase is the rev up phase, zero based, to be modified.
  * @param  hFinalTorque is the new value of motor torque at the end of that
  *         phase. This value represents actually the Iq current expressed in
  *         digit.
  * @retval none. 
  */
void RUC_SetPhaseFinalTorque(CRUC this, uint8_t bPhase, int16_t hFinalTorque);

/**
  * @brief  It is used to read the current value of duration of a specific rev
  *         up phase.
  *         Note: The module can be also compiled commenting the 
  *         define RUC_ALLOWS_TUNING to optimize the flash memory occupation
  *         and the RAM usage if the tuning is not required in this case this
  *         function has no effect.
  * @param  this related object of class CRUC.
  * @param  bPhase is the rev up phase, zero based, to be read.
  * @retval uint16_t The current value of duration for that phase expressed in
  *         milliseconds. 
  */
uint16_t RUC_GetPhaseDurationms(CRUC this, uint8_t bPhase);

/**
  * @brief  It is used to read the current value of mechanical speed at the end
  *         of a specific rev up phase.
  *         Note: The module can be also compiled commenting the 
  *         define RUC_ALLOWS_TUNING to optimize the flash memory occupation
  *         and the RAM usage if the tuning is not required in this case this
  *         function has no effect.
  * @param  this related object of class CRUC.
  * @param  bPhase is the rev up phase, zero based, to be read.
  * @retval int16_t The current value of mechanical speed at the end of that
  *         phase expressed in 0.1Hz. 
  */
int16_t RUC_GetPhaseFinalMecSpeed01Hz(CRUC this, uint8_t bPhase);

/**
  * @brief  It is used to read the current value of motor torque at the end of a
  *         specific rev up phase.
  *         Note: The module can be also compiled commenting the
  *         define RUC_ALLOWS_TUNING to optimize the flash memory occupation
  *         and the RAM usage if the tuning is not required in this case this
  *         function has no effect.
  * @param  this related object of class CRUC.
  * @param  bPhase is the rev up phase, zero based, to be read.
  * @retval int16_t The current value of motor torque at the end of that phase.
  *         This value represents actually the Iq current expressed in digit.
  */
int16_t RUC_GetPhaseFinalTorque(CRUC this, uint8_t bPhase);

/**
  * @brief  It is used to get information about the number of phases relative to 
  *         the programmed rev up.
  *         Note: The module can be also compiled commenting the
  *         define RUC_ALLOWS_TUNING to optimize the flash memory occupation
  *         and the RAM usage if the tuning is not required in this case this
  *         function has no effect.
  * @param  this related object of class CRUC.
  * @retval uint8_t The number of phases relative to the programmed rev up.
  */
uint8_t RUC_GetNumberOfPhases(CRUC this);

/**
* @}
*/

/** @defgroup SpeednPosFdbk_class_exported_methodsT SpeednPosFdbk class exported methods
  * @{
  */

/**
  * @brief  It returns the last computed rotor electrical angle, expressed in
  *         s16degrees. 1 s16degree = 360°/65536
  * @param  this related object of class CSPD
  * @retval int16_t rotor electrical angle (s16degrees)
  */
int16_t SPD_GetElAngle(CSPD this);

/**
  * @brief  It returns the last computed rotor mechanical angle, expressed in
  *         s16degrees. Mechanical angle frame is based on parameter bElToMecRatio
  *         and, if occasionally provided - through function SPD_SetMecAngle -
  *         of a measured mechanical angle, on information computed thereof.
  * @param  this related object of class CSPD
  * @retval int16_t rotor mechanical angle (s16degrees)
  */
int16_t SPD_GetMecAngle(CSPD this);

/**
  * @brief  It returns the last computed average mechanical speed, expressed in
  *         01Hz (tenth of Hertz).
  * @param  this related object of class CSPD
  * @retval int16_t rotor average mechanical speed (01Hz)
  */
int16_t SPD_GetAvrgMecSpeed01Hz(CSPD this);

/**
  * @brief  It returns the last computed electrical speed, expressed in Dpp.
  *         1 Dpp = 1 s16Degree/control Period. The control period is the period
  *         on which the rotor electrical angle is computed (through function
  *         SPD_CalcElectricalAngle).
  * @param  this related object of class CSPD
  * @retval int16_t rotor electrical speed (Dpp)
  */
int16_t SPD_GetElSpeedDpp(CSPD this);

/**
  * @brief  It returns the result of the last reliability check performed.
  *         Reliability is measured with reference to parameters
  *         hMaxReliableElSpeed01Hz, hMinReliableElSpeed01Hz,
  *         bMaximumSpeedErrorsNumber and/or specific parameters of the derived
  *         TRUE = sensor information is reliable
  *         FALSE = sensor information is not reliable
  * @param  this related object of class CSPD
  * @retval bool sensor reliability state
  */
bool SPD_Check(CSPD this);

/**
  * @brief  This method returns the average mecchanical rotor speed expressed in
  *         "S16Speed". It means that:\n
  *         - it is zero for zero speed,\n
  *         - it become S16_MAX when the average mecchanical speed is equal to 
  *           hMaxReliableMecSpeed01Hz,\n
  *         - it becomes -S16_MAX when the average mecchanical speed is equal to
  *         -hMaxReliableMecSpeed01Hz.
  * @param  this related object of class CSPD
  * @retval int16_t The average mecchanical rotor speed expressed in
  *         "S16Speed".
  */
int16_t SPD_GetS16Speed(CSPD this);

/**
  * @brief  This method returns the coefficient used to transform electrical to
  *         mechanical quantities and viceversa. It usually coincides with motor
  *         pole pairs number.
  * @param  this related object of class CSPD
  * @retval uint8_t The motor pole pairs number.
  */
uint8_t SPD_GetElToMecRatio(CSPD this);

/**
  * @brief  This method sets the coefficient used to transform electrical to
  *         mechanical quantities and viceversa. It usually coincides with motor
  *         pole pairs number.
  * @param  this related object of class CSPD
  * @param  bPP The motor pole pairs number to be set.
  */
void SPD_SetElToMecRatio(CSPD this, uint8_t bPP);

/**
* @}
*/

/** @defgroup STO_class_exported_methodsT STO class exported methods
  * @{
  */

/**
  * @brief  It exports estimated Bemf alpha-beta in Volt_Components format
  * @param  this related object of class CSTO_SPD
  * @retval Volt_Components Bemf alpha-beta 
  */
Volt_Components STO_GetEstimatedBemf(CSTO_SPD this);

/**
  * @brief  It exports the stator current alpha-beta as estimated by state 
  *         observer
  * @param  this related object of class CSTO_SPD
  * @retval Curr_Components State observer estimated stator current Ialpha-beta 
  */
Curr_Components STO_GetEstimatedCurrent(CSTO_SPD this);

/**
  * @brief  It exports current observer gains through parameters hC2 and hC4
  * @param  this related object of class CSTO_SPD
  * @param  pC2 pointer to int16_t used to return parameters hC2
  * @param  pC4 pointer to int16_t used to return parameters hC4
  * @retval none 
  */
void STO_GetObserverGains(CSTO_SPD this, int16_t *pC2, int16_t *pC4);

/**
  * @brief  It allows setting new values for observer gains
  * @param  this related object of class CSTO_SPD
  * @param  hC2 new value for observer gain hC1
  * @param  hC4 new value for observer gain hC2
  * @retval none 
  */
void STO_SetObserverGains(CSTO_SPD this, int16_t hC1, int16_t hC2);

/**
  * @brief  It exports current PLL gains through parameters pPgain and pIgain
  * @param  this related object of class CSTO_SPD
  * @param  pPgain pointer to int16_t used to return PLL proportional gain
  * @param  pIgain pointer to int16_t used to return PLL integral gain
  * @retval none 
  */
void STO_GetPLLGains(CSTO_SPD this, int16_t *pPgain, int16_t *pIgain);

/**
  * @brief  It allows setting new values for PLL gains
  * @param  this related object of class CSTO_SPD
  * @param  hPgain new value for PLL proportional gain 
  * @param  hIgain new value for PLL integral gain 
  * @retval none 
  */
void STO_SetPLLGains(CSTO_SPD this, int16_t hPgain, int16_t hIgain);

/**
  * @brief  It resets integral term of PLL
  * @param  this related object of class CSTO_SPD 
  * @retval none 
  */
void STO_ResetPLL(CSTO_SPD this);

/**
  * @brief  It exports estimated Bemf squared level
  * @param  this related object of class CSTO_SPD
  * @retval int32_t 
  */
int32_t STO_GetEstimatedBemfLevel(CSTO_SPD this);

/**
  * @brief  It exports observed Bemf squared level
  * @param  this related object of class CSTO_SPD
  * @retval int32_t 
  */
int32_t STO_GetObservedBemfLevel(CSTO_SPD this);

/**
  * @brief  It enables/disables the bemf consistency check
  * @param  this related object of class CSTO_SPD
  * @param  bSel boolean; TRUE enables check; FALSE disables check
  * @retval int32_t 
  */
void STO_BemfConsistencyCheckSwitch(CSTO_SPD this, bool bSel);

/**
  * @brief  It returns the result of the Bemf consistency check
  * @param  this related object of class CSTO_SPD
  * @retval bool Bemf consistency state
  */
bool STO_IsBemfConsistent(CSTO_SPD this);

/**
* @}
*/

/** @defgroup STO_CORDIC_class_exported_methods STO_CORDIC class exported methods
  * @{
  */
/**
  * @brief  It exports estimated Bemf alpha-beta in Volt_Components format
  * @param  this related object of class CSTO_CR_SPD
  * @retval Volt_Components Bemf alpha-beta 
  */
Volt_Components STO_CR_GetEstimatedBemf(CSTO_CR_SPD this);

/**
  * @brief  It exports the stator current alpha-beta as estimated by state 
  *         observer
  * @param  this related object of class CSTO_CR_SPD
  * @retval Curr_Components State observer estimated stator current Ialpha-beta 
  */
Curr_Components STO_CR_GetEstimatedCurrent(CSTO_CR_SPD this);

/**
  * @brief  It exports current observer gains through parameters hC2 and hC4
  * @param  this related object of class CSTO_CR_SPD
  * @param  pC2 pointer to int16_t used to return parameters hC2
  * @param  pC4 pointer to int16_t used to return parameters hC4
  * @retval none 
  */
void STO_CR_GetObserverGains(CSTO_CR_SPD this, int16_t *pC2, int16_t *pC4);

/**
  * @brief  It allows setting new values for observer gains
  * @param  this related object of class CSTO_CR_SPD
  * @param  hC1 new value for observer gain hC1
  * @param  hC2 new value for observer gain hC2
  * @retval none 
  */
void STO_CR_SetObserverGains(CSTO_CR_SPD this, int16_t hC1, int16_t hC2);

/**
  * @brief  It exports estimated Bemf squared level
  * @param  this related object of class CSTO_CR_SPD
  * @retval int32_t 
  */
int32_t STO_CR_GetEstimatedBemfLevel(CSTO_CR_SPD this);

/**
  * @brief  It exports observed Bemf squared level
  * @param  this related object of class CSTO_CR_SPD
  * @retval int32_t 
  */
int32_t STO_CR_GetObservedBemfLevel(CSTO_CR_SPD this);

/**
  * @brief  It enables/disables the bemf consistency check
  * @param  this related object of class CSTO_CR_SPD
  * @param  bSel boolean; TRUE enables check; FALSE disables check
  */
void STO_CR_BemfConsistencyCheckSwitch(CSTO_CR_SPD this, bool bSel);

/**
  * @brief  It returns the result of the Bemf consistency check
  * @param  this related object of class CSTO_CR_SPD
  * @retval bool Bemf consistency state
  */
bool STO_CR_IsBemfConsistent(CSTO_CR_SPD this);

/**
* @}
*/

/** @defgroup VSS_class_exported_methods VSS class exported methods
  * @{
  */

/**
  * @brief  Set the mechanical acceleration of virtual sensor. This acceleration
            is defined starting from current mechanical speed, final mechanical
            speed expressed in 0.1Hz and duration expressed in milliseconds.
  * @param  this related object of class CSTC.
  * @param  hFinalMecSpeed01Hz mechanical speed expressed in 0.1Hz assumed by 
            the virtual sensor at the end of the duration.
  * @param  hDurationms Duration expressed in ms. It can be 0 to apply 
            instantaneous the final speed. 
  * @retval none
  */
void  VSPD_SetMecAcceleration(CSPD this, int16_t  hFinalMecSpeed01Hz, 
                              uint16_t hDurationms);

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
* @}
*/

/** @defgroup SpeednTorqCtrl_class_exported_methodsT SpeednTorqCtrl class exported methods
  * @{
  */

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
  * @brief  Get the modality of the speed and torque controller. 
  * @param  this related object of class CSTC.
  * @retval STC_Modality_t It returns the modality of STC. It can be one of 
  *         these two values: STC_TORQUE_MODE or STC_SPEED_MODE.
  */
STC_Modality_t STC_GetControlMode(CSTC this);

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
* @}
*/

/** @defgroup StateMachine_class_exported_methodsT StateMachine class exported methods
  * @{
  */

/**
  * @brief  Returns the current state machine state
  * @param  this object of class CSTM
  * @retval State_t Current state machine state
  */
State_t STM_GetState(CSTM this);

/**
  * @brief It returns two 16 bit fields containing information about both faults
  *        currently present and faults historically occurred since the state 
  *        machine has been moved into state
  * \n\link Fault_generation_error_codes Returned error codes are listed here \endlink
  * @param this object of class CSTM.
  * @retval uint32_t  Two 16 bit fields: in the most significant half are stored
  *         the information about currently present faults. In the least 
  *         significant half are stored the information about the faults 
  *         historically occurred since the state machine has been moved into 
  *         FAULT_NOW state
  * \n\link Fault_generation_error_codes Returned error codes are listed here \endlink   
  */
uint32_t STM_GetFaultState(CSTM this);

/**
* @}
*/

/** @defgroup Temperature_sensor_class_exported_methodsT Temperature sensor class
  *                                                     exported methods
  * @{
  */

/**
  * @brief  It returns latest averaged temperature measurement expressed in 
  *         Celsius degrees
  * @param  this related object of class CTSNS
  * @retval int16_t Latest averaged temperature measurement in Celsius degrees
  */
int16_t TSNS_GetAvTemp_C(CTSNS this);

/**
  * @brief  It returns MC_OVER_TEMP or MC_NO_ERROR depending on
  *         temperature sensor output measurement and protection threshold values
  * @param  this related object of class CTSNS
  * @retval uint16_t Fault code error
  */
uint16_t TSNS_CheckTemp(CTSNS this);

/**
* @}
*/

/** @defgroup BusVoltageSensor_class_exported_methodsT BusVoltageSensor class exported methods
  * @{
  */

/**
  * @brief  It returns latest averaged Vbus measurement expressed in Volts
  * @param  this related object of class CVBS
  * @retval uint16_t Latest averaged Vbus measurement in Volts
  */
uint16_t VBS_GetAvBusVoltage_V(CVBS this);


/**
  * @brief  It returns MC_OVER_VOLT, MC_UNDER_VOLT or MC_NO_ERROR depending on
  *         bus voltage measurement and protection threshold values
  * @param  this related object of class CVBS
  * @retval uint16_t Fault code error
  */
uint16_t VBS_CheckVbus(CVBS this);

/**
* @}
*/

/** @defgroup DigitalOutput_class_exported_methodsT DigitalOutput class exported methods
  * @{
  */

/**
  * @brief It returns the state of the digital output
  * @param this object of class DOUT
  * @retval OutputState_t Digital output state (ACTIVE or INACTIVE)
  */
DOutputState_t DOUT_GetOutputState(CDOUT this);

/**
* @}
*/

/** @defgroup MotorPowerMeasurement_class_exported_methodsT MotorPowerMeasurement class exported methods
  * @{
  */

/**
  * @brief  This method is used to get the last measured motor power 
  *         (instantaneous value) expressed in watt.
  * @param this related object of class CMPM.
  * @retval int16_t The last measured motor power (instantaneous value) 
  *         expressed in watt.
  */
int16_t MPM_GetElMotorPowerW(CMPM this);

/**
  * @brief  This method is used to get the average measured motor power 
  *         expressed in watt.
  * @param this related object of class CMPM.
  * @retval int16_t The average measured motor power expressed in watt.
  */
int16_t MPM_GetAvrgElMotorPowerW(CMPM this);

/**
* @}
*/

/** @defgroup Selfcommisioning_class_exported_methodsT Selfcommissioning class exported methods
  * @{
  */

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
* @}
*/

/** @defgroup OneTouchTuning_class_exported_methodsT One touch tuning class exported methods
  * @{
  */

/**
  * @brief  Call this method before start motor to force new OTT procedure.
  * @param  this related object of class COTT.
  * @retval none.
  */
void OTT_ForceTuning(COTT this);

/**
  * @brief  It returns the nominal speed estimated by OTT.
  * @param  this related object of class COTT.
  * @retval uint32_t It returns the nominal speed estimated by OTT, it is a 
  *         floating point number codified into a 32bit integer.
  */
uint32_t OTT_GetNominalSpeed(COTT this);

/**
  * @brief  It returns the number of states of OTT.
  * @param  this related object of class COTT.
  * @retval uint8_t It returns the number of states of Selfcommissioning procedure.
  */
uint8_t OTT_GetSteps(COTT this);

/**
  * @brief  It returns the state of OTT.
  * @param  this related object of class COTT.
  * @retval uint8_t It returns the state of OTT.
  */
uint8_t OTT_GetState(COTT this);

/**
  * @brief  It returns TRUE if OTT procedure has been completed, FALSE otherwise.
  * @param  this related object of class COTT.
  * @retval bool It returns TRUE if OTT procedure has been completed, FALSE otherwise.
  */
bool OTT_IsSpeedPITuned(COTT this);

/**
  * @brief  It returns the nominal speed estimated by OTT in RPM.
  * @param  this related object of class COTT.
  * @retval float It returns the nominal speed estimated by OTT in RPM.
  */
float OTT_fGetNominalSpeedRPM(COTT this);

/**
  * @brief  Sets the number of motor poles pairs.
  * @param  this related object of class COTT.
  * @param  bPP Number of motor poles pairs to be set.
  * @retval none
  */
void OTT_SetPolesPairs(COTT this, uint8_t bPP);

/**
  * @brief  Change the nominal current .
  * @param  this related object of class COTT.
  * @param  hNominalCurrent This value represents actually the maximum Iq current 
            expressed in digit.
  * @retval none
  */
void OTT_SetNominalCurrent(COTT this, uint16_t hNominalCurrent);

/**
  * @brief  Change the speed regulator bandwidth.
  * @param  this related object of class COTT.
  * @param  fBW Current regulator bandwidth espressed in rad/s.
  * @retval none
  */
void OTT_SetSpeedRegulatorBandwidth(COTT this, float fBW);

/**
  * @brief  Get the speed regulator bandwidth.
  * @param  this related object of class COTT.
  * @retval float Current regulator bandwidth espressed in rad/s.
  */
float OTT_GetSpeedRegulatorBandwidth(COTT this);

/**
  * @brief  Get the measured inertia of the motor.
  * @param  this related object of class COTT.
  * @retval float Measured inertia of the motor expressed in Kgm^2.
  */
float OTT_GetJ(COTT this);

/**
  * @brief  Get the measured friction of the motor.
  * @param  this related object of class COTT.
  * @retval float Measured friction of the motor expressed in Nms.
  */
float OTT_GetF(COTT this);

/**
  * @brief  Return true if the motor has been already profiled.
  * @param  this related object of class COTT.
  * @retval bool TRUE if the if the motor has been already profiled, 
  *         FALSE otherwise.
  */
bool OTT_IsMotorAlreadyProfiled(COTT this);

/**
* @}
*/

/**
* @}
*/

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /* __MCTUNINGCLASS_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
