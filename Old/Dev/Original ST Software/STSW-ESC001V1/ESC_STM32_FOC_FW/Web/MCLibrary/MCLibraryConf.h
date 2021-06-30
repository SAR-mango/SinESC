/**
  ******************************************************************************
  * @file    MCLibraryConf.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains general configurations of the MC library      
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
#ifndef __MCLIBRARYCONF_H
#define __MCLIBRARYCONF_H

/* Includes ------------------------------------------------------------------*/

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */
  
/** @defgroup MCLibraryConf_definitions MCLibraryConf definitions
* @{
*/

/** @defgroup MCLibraryConf_dynamic_static MC Classes dynamic vs static memory allocation
* @{
*/

/** 
  * @brief Uncomment #define MC_CLASS_DYNAMIC to enable MC Classes dynamic memory
  *        allocation; on the contrary, static memory allocation is enabled: in
  *        that case, the pool size of each class is to be defined.
  * \n\link MCLibraryConf_Static Size definitions listed here \endlink
  */
/*#define MC_CLASS_DYNAMIC*/

#ifdef MC_LIBRARY_DUAL

/** @defgroup MCLibraryConf_Static Static compilation, pool sizes
* @{
*/

/** 
  * @brief  PIRegulatorClass pool dimension
  */
#define MAX_PI_NUM 16u

/** 
  * @brief  PID_PIRegulatorClass pool dimension
  */
#define MAX_PID_PI_NUM 16u

/** 
  * @brief  DigitalOutputClass pool dimension
  */
#define MAX_DOUT_NUM 5u

/** 
  * @brief  SpeednPosFdbkClass pool dimension
  */
#define MAX_SPD_NUM 6u

/** 
  * @brief  STO_SpeednPosFdbkClass pool dimension
  */
#define MAX_STO_SPD_NUM 2u

/** 
  * @brief  STO_CORDIC_SpeednPosFdbkClass pool dimension
  */
#define MAX_STO_CR_SPD_NUM 2u

/** 
  * @brief  HALL_SpeednPosFdbkClass pool dimension
  */
#define MAX_HALL_SPD_NUM 2u

/** 
  * @brief  ENCODER_SpeednPosFdbkClass pool dimension
  */
#define MAX_ENC_SPD_NUM 2u

/** 
  * @brief  VirtualSpeedSensor_SpeednPosFdbkClass pool dimension
  */
#define MAX_VSS_SPD_NUM 2u

/** 
  * @brief  EncAlignCtrlClass pool dimension
  */
#define MAX_EAC_NUM 2u

/** 
  * @brief  PWMCurrFdbkClass pool dimension
  */
#define MAX_PWMC_NUM 2u

/** 
  * @brief  Derived PWMCurrFdbkClass pool dimension
  */
#define MAX_DRV_PWMC_NUM 2u

/** 
  * @brief  RevupCtrlClass pool dimension
  */
#define MAX_RUC_NUM 2u

/** 
  * @brief  SpeednTorqCtrlClass pool dimension
  */
#define MAX_STC_NUM 2u

/** 
  * @brief  StateMachineClass pool dimension
  */
#define MAX_STM_NUM 4u

/** 
  * @brief  BusVoltageSensorClass pool dimension
  */
#define MAX_VBS_NUM 2u

/** 
  * @brief  Rdivider_BusVoltageSensorClass pool dimension
  */
#define MAX_RVBS_VBS_NUM 2u

/** 
  * @brief  Virtual_BusVoltageSensorClass pool dimension
  */
#define MAX_VVBS_VBS_NUM 2u

/** 
  * @brief  TemperatureSensorClass pool dimension
  */
#define MAX_TSNS_NUM 2u

/** 
  * @brief  NTC_TemperatureSensorClass pool dimension
  */
#define MAX_NTC_TSNS_NUM 2u

/** 
  * @brief  Virtual_TemperatureSensorClass pool dimension
  */
#define MAX_VTS_TSNS_NUM 2u

/** 
  * @brief  OpenLoopFOC pool dimension
  */
#define MAX_OL_NUM 2u

/** 
  * @brief  MotorPowerMeasurementClass pool dimension
  */
#define MAX_MPM_NUM 2u

/** 
  * @brief  PQD_MotorPowerMeasurementClass pool dimension
  */
#define MAX_PQD_MPM_NUM 2u

/** 
  * @brief  InrushCurrentLimiterClass pool dimension
  */
#define MAX_ICL_NUM 2u

/** 
  * @brief  CircleLimitationClass pool dimension
  */
#define MAX_CLM_NUM 2u

/** 
  * @brief  FeedForwardCtrlClass pool dimension
  */
#define MAX_FF_NUM 2u

/** 
  * @brief  FluxWeakeningCtrlClass pool dimension
  */
#define MAX_FW_NUM 2u

/** 
  * @brief  HFInjCtrlClass pool dimension
  */
#define MAX_HFI_NUM 2u

/** 
  * @brief  HiFreqInj_FPU_SpeednPosFdbkClass pool dimension
  */
#define MAX_HFI_FP_SPD_NUM 2u

/** 
  * @brief  MTPACtrlClass pool dimension
  */
#define MAX_MTPA_NUM 2u

/** 
  * @brief  GateDriverCtrlClass pool dimension
  */
#define MAX_GDC_NUM 2u

/** 
  * @brief  GAP_GateDriverCtrlClass pool dimension
  */
#define MAX_GAP_GDC_NUM 2u

#else

/** @defgroup MCLibraryConf_Static Static compilation, pool sizes for single drive
* @{
*/

/** 
  * @brief  PIRegulatorClass pool dimension
  */
#define MAX_PI_NUM 8u

/** 
  * @brief  PID_PIRegulatorClass pool dimension
  */
#define MAX_PID_PI_NUM 8u

/** 
  * @brief  DigitalOutputClass pool dimension
  */
#define MAX_DOUT_NUM 5u

/** 
  * @brief  SpeednPosFdbkClass pool dimension
  */
#define MAX_SPD_NUM 3u

/** 
  * @brief  STO_SpeednPosFdbkClass pool dimension
  */
#define MAX_STO_SPD_NUM 1u

/** 
  * @brief  STO_CORDIC_SpeednPosFdbkClass pool dimension
  */
#define MAX_STO_CR_SPD_NUM 1u

/** 
  * @brief  HALL_SpeednPosFdbkClass pool dimension
  */
#define MAX_HALL_SPD_NUM 1u

/** 
  * @brief  ENCODER_SpeednPosFdbkClass pool dimension
  */
#define MAX_ENC_SPD_NUM 1u

/** 
  * @brief  VirtualSpeedSensor_SpeednPosFdbkClass pool dimension
  */
#define MAX_VSS_SPD_NUM 1u

/** 
  * @brief  EncAlignCtrlClass pool dimension
  */
#define MAX_EAC_NUM 1u

/** 
  * @brief  PWMCurrFdbkClass pool dimension
  */
#define MAX_PWMC_NUM 1u

/** 
  * @brief  Derived PWMCurrFdbkClass pool dimension
  */
#define MAX_DRV_PWMC_NUM 1u

/** 
  * @brief  RevupCtrlClass pool dimension
  */
#define MAX_RUC_NUM 1u

/** 
  * @brief  SpeednTorqCtrlClass pool dimension
  */
#define MAX_STC_NUM 1u

/** 
  * @brief  StateMachineClass pool dimension
  */
#define MAX_STM_NUM 2u

/** 
  * @brief  BusVoltageSensorClass pool dimension
  */
#define MAX_VBS_NUM 1u

/** 
  * @brief  Rdivider_BusVoltageSensorClass pool dimension
  */
#define MAX_RVBS_VBS_NUM 1u

/** 
  * @brief  Virtual_BusVoltageSensorClass pool dimension
  */
#define MAX_VVBS_VBS_NUM 1u

/** 
  * @brief  TemperatureSensorClass pool dimension
  */
#define MAX_TSNS_NUM 1u

/** 
  * @brief  NTC_TemperatureSensorClass pool dimension
  */
#define MAX_NTC_TSNS_NUM 1u

/** 
  * @brief  Virtual_TemperatureSensorClass pool dimension
  */
#define MAX_VTS_TSNS_NUM 1u

/** 
  * @brief  OpenLoopFOC pool dimension
  */
#define MAX_OL_NUM 1u

/** 
  * @brief  MotorPowerMeasurementClass pool dimension
  */
#define MAX_MPM_NUM 1u

/** 
  * @brief  PQD_MotorPowerMeasurementClass pool dimension
  */
#define MAX_PQD_MPM_NUM 1u

/** 
  * @brief  InrushCurrentLimiterClass pool dimension
  */
#define MAX_ICL_NUM 1u

/** 
  * @brief  CircleLimitationClass pool dimension
  */
#define MAX_CLM_NUM 1u

/** 
  * @brief  FeedForwardCtrlClass pool dimension
  */
#define MAX_FF_NUM 1u

/** 
  * @brief  FluxWeakeningCtrlClass pool dimension
  */
#define MAX_FW_NUM 1u

/** 
  * @brief  HFInjCtrlClass pool dimension
  */
#define MAX_HFI_NUM 1u

/** 
  * @brief  HiFreqInj_FPU_SpeednPosFdbkClass pool dimension
  */
#define MAX_HFI_FP_SPD_NUM 1u

/** 
  * @brief  MTPACtrlClass pool dimension
  */
#define MAX_MTPA_NUM 1u

/** 
  * @brief  GateDriverCtrlClass pool dimension
  */
#define MAX_GDC_NUM 1u

/** 
  * @brief  GAP_GateDriverCtrlClass pool dimension
  */
#define MAX_GAP_GDC_NUM 1u

#endif

/** 
  * @brief  SelfComCtrlClass pool dimension
  */
#define MAX_SCC_NUM 1u

/** 
  * @brief  OneTouchTuningClass pool dimension
  */
#define MAX_OTT_NUM 1u

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* __MCLIBRARYCONF_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
