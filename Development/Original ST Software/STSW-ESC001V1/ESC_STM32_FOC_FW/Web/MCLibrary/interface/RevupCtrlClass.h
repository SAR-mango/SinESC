/**
  ******************************************************************************
  * @file    RevupCtrlClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of RevupCtrl class      
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
#ifndef __REVUPCTRLCLASS_H
#define __REVUPCTRLCLASS_H

/* Includes ------------------------------------------------------------------*/
#include "MC_type.h"
#include "SpeednTorqCtrlClass.h"
#include "VirtualSpeedSensor_SpeednPosFdbkClass.h"
#include "STO_SpeednPosFdbkClass.h"
#include "PWMnCurrFdbkClass.h"

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */

/** @addtogroup RevupCtrl
  * @{
  */
  
/** @defgroup RevupCtrl_class_exported_types RevupCtrl class exported types
* @{
*/

/** 
  * @brief  Parameters related to each rev up phase. 
  */
typedef const struct
{
	uint16_t hDurationms;       /*!< Duration of the rev up phase expressed in
                                   milliseconds.*/
	int16_t hFinalMecSpeed01Hz; /*!< Mechanical speed expressed in 0.1Hz assumed
                                   by VSS at the end of the rev up phase.*/
	int16_t hFinalTorque;       /*!< Motor torque reference imposed by STC at the
                                   end of rev up phase. This value represents
                                   actually the Iq current expressed in digit.*/
  void* pNext;                /*!< Pointer of the next element of phase params.
                                   It must be MC_NULL for the last element.*/
} RUCPhasesParams_t, *pRUCPhasesParams_t;

/** 
  * @brief  Public RevupCtrl class definition 
  */
typedef struct CRUC_t *CRUC;

/** 
  * @brief  RevupCtrl class parameters definition  
  */
typedef const struct
{
  uint16_t hRUCFrequencyHz; /*!< Frequency expressed in Hz at which the user
                                 clocks the RUC calling RUC_Exec method */
  int16_t hStartingMecAngle;/*!< Starting angle of programmed rev up.*/
  pRUCPhasesParams_t pPhaseParam;/*!< Pointer of the first element of 
                                 phase params that constitute the programmed 
                                 rev up. Each element is of type 
                                 RUCPhasesParams_t.*/
  uint8_t bFirstAccelerationStage;/*< Indicate here in which of the rev-up phases
                                     the final acceleration is started. The 
                                     sensor-less algorithm is re-initialized 
                                     (cleared) when this phase begins. 
                               NOTE: The rev up phases counting starts from 0.
                                     First phase should be indicated with zero */
  uint16_t hMinStartUpValidSpeed;/*!< Minimum mechanical speed (expressed in 
                                      01Hz required to validate the start-up*/
  uint16_t hMinStartUpFlySpeed;  /*!< Minimum mechanical speed (expressed in 
                                      01Hz required to validate the on the fly*/
  uint8_t bOTFvalidation;        /*!< Minimum number of consecutive times the observer
                                      is reliable to validate OTF startup*/
  int16_t hFinalRevUpCurrent;    /*!< Final revup current. */
  uint16_t hOTFSection1Duration; /*!< On-the-fly section1 duration, millisecond.*/   
  bool OTFStartupEnabled;        /*!< ENABLE for OTF strartup otherwise FALSE.*/
} RevupCtrlParams_t, *pRevupCtrlParams_t;  
/**
* @}
*/

/** @defgroup RevupCtrl_class_exported_methods RevupCtrl class exported methods
  * @{
  */

/**
  * @brief  Creates an object of the class RevupCtrl
  * @param  pRevupCtrlParams pointer to an RevupCtrl parameters structure
  * @retval CRUC new instance of RevupCtrl object
  */
CRUC RUC_NewObject(pRevupCtrlParams_t pRevupCtrlParams);

/**
  * @brief  Initializes all the object variables, usually it has to be called
  *         once right after object creation. It is also used to assign the
  *         speed and torque controller and the virtual speed sensor objects to
  *         be used by rev up controller.
  * @param  this related object of class CSTC.
  * @param  oSTC the speed and torque controller used by the RUC.
  * @param  oVSS the virtual speed sensor used by the RUC.
  * @param  oSNSL the sensorles state observer used by the OTF Startup.
  * @param  oPWM the PWM object used by the OTF Startup.
  * @retval none.
  */
void RUC_Init(CRUC this, CSTC oSTC, CVSS_SPD oVSS, CSTO_SPD oSNSL, CPWMC oPWM);

/**
  * @brief  It should be called before each motor restart. It initialize
  *         internal state of RUC sets first commands to VSS and STC and calls 
  *         the Clear method of VSS.
  *         It also sets the Speed and Torque controller in TORQUE mode.
  * @param  this related object of class CSTC.
  * @param  hMotorDirection If it is "1" the programmed revup sequence is 
  *         performed. If it is "-1" the revup sequence is performed with
  *         opposite values of targets (speed, torque).
  * @retval none.
  */
void RUC_Clear(CRUC this, int16_t hMotorDirection);

/**
  * @brief  It clocks the rev up controller and must be called with a frequency
  *         equal to the one set in the parameters hRUCFrequencyHz. Calling this
  *         method the rev up controller perform the programmed sequence.
  *         Note: STC and VSS aren’t clocked by RUC_Exec.
  * @param  this related object of class CRUC.
  * @retval bool It returns FALSE when the programmed rev up has been completed. 
  */
bool RUC_Exec(CRUC this);

/**
  * @brief  Return information about current state of programmer rev up sequence.
  * @param  this related object of class CRUC.
  * @retval bool It returns TRUE when the programmed rev up has been completed. 
  */
bool RUC_Completed(CRUC this);

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
  * @brief  It is used to know if the programmed "first acceleration stage" has
  *         been reached inside the rev up sequence. Is intended that the stages
  *         previous to this are reserved for alignments. When is reached the
  *         first stage of acceleration the observer should be cleared once.
  *         NOTE: The rev up stage are zero-based indexed so that the fist stage 
  *         is the number zero.
  * @param  this related object of class CRUC.
  * @retval bool It returns TRUE if the first acceleration stage has been 
  *         reached and FALSE otherwise.
  */
bool RUC_FirstAccelerationStageReached(CRUC this);

/**
  * @brief  It returns the state of low side switches.
  * @param  this related object of class CRUC.
  * @retval bool Status of the LS
  */
bool RUC_Get_SCLowsideOTF_Status(CRUC this);

/**
  * @brief  It is used to stop the programmed RevUp at the current speed.
  * @param  this related object of class CRUC.
  * @retval none.
  */
void RUC_Stop(CRUC this);

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /* __REVUPCTRLCLASS_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
