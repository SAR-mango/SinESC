/**
  ******************************************************************************
  * @file    MCTasks.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file implementes tasks definition
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

/* Pre-compiler coherency check */
#define MC_APPLICATION_CHK
#include "CrossCheck.h" 
#undef MC_APPLICATION_CHK

#include "PIRegulatorClass.h"
#include "PID_PIRegulatorClass.h"
#include "DigitalOutputClass.h"
#include "MC_type.h"
#include "MC_Math.h"
#include "MCIRQHandlerClass.h"

#include "SpeednPosFdbkClass.h"
#include "STO_SpeednPosFdbkClass.h"
#include "HiFreqInj_FPU_SpeednPosFdbkClass.h"
#include "ENCODER_SpeednPosFdbkClass.h"
#include "HALL_SpeednPosFdbkClass.h"
#include "HALL_F30X_SpeednPosFdbkClass.h"
#include "STO_CORDIC_SpeednPosFdbkClass.h"
#include "VirtualSpeedSensor_SpeednPosFdbkClass.h"
#include "EncAlignCtrlClass.h"

#include "HiFreqInj_FPU_CtrlClass.h"
#include "OpenLoopClass.h"
#include "FluxWeakeningCtrlClass.h"
#include "FeedForwardCtrlClass.h"
#include "MTPACtrlClass.h"
#include "CircleLimitationClass.h"

#include "PWMnCurrFdbkClass.h"

/* PWMC derived class includes */
#include "R3_LM1_PWMnCurrFdbkClass.h"
#include "R3_HD2_PWMnCurrFdbkClass.h"
#include "R1_LM1_PWMnCurrFdbkClass.h"
#include "R1_VL1_PWMnCurrFdbkClass.h"
#include "R1_HD2_PWMnCurrFdbkClass.h"
#include "ICS_LM1_PWMnCurrFdbkClass.h"
#include "ICS_HD2_PWMnCurrFdbkClass.h"
#include "R3_F2XX_PWMnCurrFdbkClass.h"
#include "R1_F2XX_PWMnCurrFdbkClass.h"
#include "ICS_F2XX_PWMnCurrFdbkClass.h"
#include "R3_F4XX_PWMnCurrFdbkClass.h"
#include "R1_F4XX_PWMnCurrFdbkClass.h"
#include "ICS_F4XX_PWMnCurrFdbkClass.h"
#include "R1_F0XX_PWMnCurrFdbkClass.h"
#include "R3_F0XX_PWMnCurrFdbkClass.h"
#include "R3_1_F30X_PWMnCurrFdbkClass.h"
#include "R3_2_F30X_PWMnCurrFdbkClass.h"
#include "R3_4_F30X_PWMnCurrFdbkClass.h"
#include "R1_F30X_PWMnCurrFdbkClass.h"
#include "ICS_F30X_PWMnCurrFdbkClass.h"

#include "StateMachineClass.h"
#include "SpeednTorqCtrlClass.h"
#include "RevupCtrlClass.h"

#include "BusVoltageSensorClass.h"
#include "Rdivider_BusVoltageSensorClass.h"
#include "Virtual_BusVoltageSensorClass.h"

#include "DigitalOutputClass.h"

#include "TemperatureSensorClass.h"
#include "NTC_TemperatureSensorClass.h"
#include "Virtual_TemperatureSensorClass.h"

#include "MotorPowerMeasurementClass.h"
#include "PQD_MotorPowerMeasurementClass.h"

#include "InrushCurrentLimiterClass.h"

#include "OneTouchTuningClass.h"
#include "SelfComCtrlClass.h"

#include "MCInterfaceClass.h"
#include "MCInterfaceClassPrivate.h"
#include "MCTuningClass.h"
#include "MCTuningClassPrivate.h"
#include "RampExtMngrClass.h"

#if (defined(OTF_STARTUP))
#define CHARGE_BOOT_CAP_ENABLING        DISABLE
#define CHARGE_BOOT_CAP_ENABLING2       DISABLE
#else
#define CHARGE_BOOT_CAP_ENABLING        ENABLE
#define CHARGE_BOOT_CAP_ENABLING2       ENABLE
#endif
#define CHARGE_BOOT_CAP_MS  10
#define CHARGE_BOOT_CAP_MS2 10
#define OFFCALIBRWAIT_MS     0
#define OFFCALIBRWAIT_MS2    0
#define STOPPERMANENCY_MS  400
#define STOPPERMANENCY_MS2 400
#define CHARGE_BOOT_CAP_TICKS  (uint16_t)((SYS_TICK_FREQUENCY * CHARGE_BOOT_CAP_MS)/ 1000)
#define CHARGE_BOOT_CAP_TICKS2 (uint16_t)((SYS_TICK_FREQUENCY * CHARGE_BOOT_CAP_MS2)/ 1000)
#define OFFCALIBRWAITTICKS     (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS)/ 1000)
#define OFFCALIBRWAITTICKS2    (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS2)/ 1000)
#define STOPPERMANENCY_TICKS   (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS)/ 1000)
#define STOPPERMANENCY_TICKS2  (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS2)/ 1000)

#include "SystemNDriveParams.h"

#include "MCTasks.h"
#include "MC.h"

#define MC_TUNING_INTERFACE

#if defined(MOTOR_PROFILER)
#define ONE_TOUCH_TUNING
#endif

/* Private define ------------------------------------------------------------*/
#define AUX_SPEED_FDBK_M1 (defined(VIEW_HALL_FEEDBACK) || defined(VIEW_ENCODER_FEEDBACK) || defined(AUX_STATE_OBSERVER_PLL) || defined(AUX_STATE_OBSERVER_CORDIC))
#define AUX_SPEED_FDBK_M2 (defined(VIEW_HALL_FEEDBACK2) || defined(VIEW_ENCODER_FEEDBACK2) || defined(AUX_STATE_OBSERVER_PLL2) || defined(AUX_STATE_OBSERVER_CORDIC2))

/* Un-Comment this macro define in order to activate the smooth 
   braking action on over voltage */

/* #define SMOOTH_BRAKING_ACTION_ON_OVERVOLTAGE */

/* Private variables----------------------------------------------------------*/
FOCVars_t FOCVars[NBR_OF_MOTORS];

CMCI oMCInterface[NBR_OF_MOTORS];
#ifdef MC_TUNING_INTERFACE
CMCT oMCTuning[NBR_OF_MOTORS];
#endif
CSTM oSTM[NBR_OF_MOTORS];
CSTC oSTC[NBR_OF_MOTORS];
CPI oPIDSpeed[NBR_OF_MOTORS],oPIDIq[NBR_OF_MOTORS],oPIDId[NBR_OF_MOTORS];
CVBS oBusSensor[NBR_OF_MOTORS];

#ifdef SMOOTH_BRAKING_ACTION_ON_OVERVOLTAGE
#ifdef SINGLEDRIVE
static uint16_t nominalBusd[1] = {0u};
static uint16_t ovthd[1] = {OVERVOLTAGE_THRESHOLD_d};
#else
static uint16_t nominalBusd[2] = {0u,0u};
static uint16_t ovthd[2] = {OVERVOLTAGE_THRESHOLD_d,OVERVOLTAGE_THRESHOLD_d2};
#endif
#endif

CTSNS oTemperatureSensor[NBR_OF_MOTORS];
CPWMC oCurrSensor[NBR_OF_MOTORS];  
CSPD oSpeedSensor[NBR_OF_MOTORS]; 
CDOUT oR_Brake[NBR_OF_MOTORS];
CDOUT oOCPDisabling[NBR_OF_MOTORS];
CMPM oMPM[NBR_OF_MOTORS];
CCLM oCLM[NBR_OF_MOTORS];

#if AUX_SPEED_FDBK_M1
CSPD oSpeedSensorAux_M1;              /* only if auxiliary speed sensor on M1*/
#endif 
#if ((defined NO_SPEED_SENSORS)||(defined ENCODER)||(defined VIEW_ENCODER_FEEDBACK))
CSPD oVSS_M1;                         /* only if M1 is sensorless*/
#endif
#if (defined NO_SPEED_SENSORS)
CRUC oRUC_M1;                         /* only if M1 is sensorless*/
#endif
#if ((defined ENCODER)||(defined VIEW_ENCODER_FEEDBACK))
CEAC oEAC_M1;
#endif
#if defined(FLUX_WEAKENING) || (defined(DUALDRIVE) && defined(FLUX_WEAKENING2))
CFW oFW[NBR_OF_MOTORS];     /* only if M1 or M2 has FW */
CPI oPIFW[NBR_OF_MOTORS];   /* only if M1 or M2 has FW */
#endif
#if defined(FEED_FORWARD_CURRENT_REGULATION) || (defined(DUALDRIVE) && defined(FEED_FORWARD_CURRENT_REGULATION2))
CFF oFF[NBR_OF_MOTORS];     /* only if M1 or M2 has FF */
#endif
#if defined(IPMSM_MTPA) || (defined(DUALDRIVE) && defined(IPMSM_MTPA2))
CMTPA oMTPA[2] = {MC_NULL,MC_NULL}; /* only if M1 or M2 has MTPA */
#endif
#if defined(OPEN_LOOP) || (defined(DUALDRIVE) && defined(OPEN_LOOP2))
COL oOL[2] = {MC_NULL,MC_NULL};     /* only if M1 or M2 has OPEN LOOP */
#endif
#if (defined(SINGLEDRIVE) && defined(HFINJECTION))
CHFI_FP oHFI[NBR_OF_MOTORS] = {MC_NULL};/* only if M1 has HFI */
#elif (defined(DUALDRIVE) && (defined(HFINJECTION) || defined(HFINJECTION2)))
CHFI_FP oHFI[NBR_OF_MOTORS] = {MC_NULL,MC_NULL};/* only if M1 or M2 has HFI */
#endif
#if defined(MOTOR_PROFILER)
CSCC oSCC[1] = {MC_NULL};
#endif
#if defined(ONE_TOUCH_TUNING)
COTT oOTT[1] = {MC_NULL};
#endif

#ifdef DUALDRIVE
#if AUX_SPEED_FDBK_M2
CSPD oSpeedSensorAux_M2;              /* only if auxiliary speed sensor on M2*/  
#endif
#if ((defined NO_SPEED_SENSORS2)||(defined ENCODER2)||(defined VIEW_ENCODER_FEEDBACK2))
CSPD oVSS_M2;                         /* only if M2 is sensorless*/
#endif
#if (defined NO_SPEED_SENSORS2)
CRUC oRUC_M2;                         /* only if M2 is sensorless*/
#endif
#if ((defined ENCODER2)||(defined VIEW_ENCODER_FEEDBACK2))
CEAC oEAC_M2;
#endif
#endif

#if (INRUSH_CURRLIMIT_ENABLING == ENABLE)
CICL oICL_M1;
CDOUT oICLDOUT_M1;
#endif

#if (INRUSH_CURRLIMIT_ENABLING2 == ENABLE)
CICL oICL_M2;
CDOUT oICLDOUT_M2;
#endif

CREMNG oREMNG[NBR_OF_MOTORS];   /*!< Ramp manager used to modify the Iq ref 
                                     during the start-up switch over.*/

static volatile uint16_t hMFTaskCounterM1 = 0;
static volatile uint16_t hBootCapDelayCounterM1 = 0;
static volatile uint16_t hStopPermanencyCounterM1 = 0;
#ifdef DUALDRIVE
static volatile uint16_t hMFTaskCounterM2 = 0;
static volatile uint16_t hBootCapDelayCounterM2 = 0;
static volatile uint16_t hStopPermanencyCounterM2 = 0;
#endif

static uint8_t UDC_Channel = 0u;
static uint16_t UDC_ConvertedValue = 0u;
static volatile UDRC_State_t UDC_State = UDRC_STATE_IDLE;

#if (defined NO_SPEED_SENSORS)
static bool SWO_transitionStartM1 = FALSE;
#endif
#ifdef DUALDRIVE
#if (defined NO_SPEED_SENSORS2)
static bool SWO_transitionStartM2 = FALSE;
#endif
#endif

static uint8_t bMCBootCompleted = 0;

/* Private functions ---------------------------------------------------------*/
static void TSK_MediumFrequencyTaskM1(void);
static void FOC_Clear(uint8_t bMotor);
static void FOC_InitAdditionalMethods(uint8_t bMotor);
static void FOC_CalcCurrRef(uint8_t bMotor);
#if !defined(MOTOR_PROFILER)
static uint16_t FOC_CurrController(uint8_t bMotor);
#endif

void TSK_SetChargeBootCapDelayM1(uint16_t hTickCount);
bool TSK_ChargeBootCapDelayHasElapsedM1(void);
static void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount);
static bool TSK_StopPermanencyTimeHasElapsedM1(void);

void TSK_SafetyTask_PWMOFF(uint8_t motor);
void TSK_SafetyTask_RBRK(uint8_t motor);
void TSK_SafetyTask_LSON(uint8_t motor);

#ifdef DUALDRIVE
static void TSK_MediumFrequencyTaskM2(void);

void TSK_SetChargeBootCapDelayM2(uint16_t hTickCount);
bool TSK_ChargeBootCapDelayHasElapsedM2(void);
static void TSK_SetStopPermanencyTimeM2(uint16_t SysTickCount);
static bool TSK_StopPermanencyTimeHasElapsedM2(void);

#define FOC_ARRAY_LENGHT 2
static void *FOC_array[FOC_ARRAY_LENGHT]={MC_NULL,MC_NULL};
static uint8_t FOC_array_head = 0; // Next obj to be executed
static uint8_t FOC_array_tail = 0; // Last arrived
#endif

/****************************** USE ONLY FOR SDK 4.0 EXAMPLES *************/  
#if defined(EXAMPLE_SPEEDMONITOR)
   void ARR_TIM5_update(CSPD xCSPD);
#endif
/**************************************************************************/   

/**
  * @brief  It initializes the whole MC core according to user defined 
  *         parameters.
  * @param  oMCIList pointer to the vector of MCInterface objects that will be
  *         created and initialized. The vector must have length equal to the 
  *         number of motor drives.
  * @param  oMCTList pointer to the vector of MCTuning objects that will be
  *         created and initialized. The vector must have length equal to the 
  *         number of motor drives.
  * @retval None
  */
void MCboot(CMCI oMCIList[NBR_OF_MOTORS],CMCT oMCTList[NBR_OF_MOTORS])
{
#if defined(FLUX_WEAKENING)
  FWInit_t FWInitStructureM1; /* only if M1 has FW */
#endif
#if defined(FEED_FORWARD_CURRENT_REGULATION)
  FFInit_t FFInitStructureM1; /* only if M1 has FF */
#endif
#if defined(HFINJECTION) 
  HFI_FP_Init_t HFI_FP_InitStructureM1; /* only if M1 has HFI */
#endif
#if defined(MOTOR_PROFILER)
  static SCC_Init_t SCC_InitStrM1; /* only if M1 has MOTOR_PROFILER */
#endif
#if defined(ONE_TOUCH_TUNING)
  OTT_Init_t OTT_InitStrM1; /* only if M1 has ONE TOUCH TUNING */
#endif
  
  PQD_MPMInitStruct_t PQD_MPMInitStruct;
  
#ifdef MC_TUNING_INTERFACE
  MCTuningInitStruct_t MCTInitStruct;
#endif

#ifdef DUALDRIVE
#if defined(FLUX_WEAKENING2)
  FWInit_t FWInitStructureM2; /* only if M2 has FW */
#endif
#if defined(FEED_FORWARD_CURRENT_REGULATION2)
  FFInit_t FFInitStructureM2; /* only if M2 has FF */
#endif
#if defined(HFINJECTION2) 
  HFI_FP_Init_t HFI_FP_InitStructureM2; /* only if M2 has HFI */
#endif
#endif
  
  bMCBootCompleted = 0;
  
  oCLM[M1] = CLM_NewObject(&CircleLimitationParamsM1);

#if defined(FLUX_WEAKENING)
  oFW[M1] = FW_NewObject(&FWParamsM1); /* only if M1 has FW */
#endif
  
#if defined(FEED_FORWARD_CURRENT_REGULATION)
  oFF[M1] = FF_NewObject(&FFParamsM1); /* only if M1 has FF */
#endif
  
#if defined(OPEN_LOOP)
  oOL[M1] = OL_NewObject(&OpenLoop_ParamsM1); /* only if M1 has OPEN LOOP */
#endif
  
#if defined(HFINJECTION) 
  oHFI[M1] = HFI_FP_NewObject(&HiFreqInj_FPU_ParamsM1); /* only if M1 has HFI */
#endif
  
#if defined(MOTOR_PROFILER)
  oSCC[M1] = SCC_NewObject(&SelfComCtrlParamsM1); /* only if M1 has MOTOR_PROFILER */
#endif
  
#if defined(ONE_TOUCH_TUNING)
  oOTT[M1] = OTT_NewObject(&OneTouchTuningParamsM1); /* only if M1 has ONE TOCUH TUNING */
#endif

  /* PWMC derived class object instantiation M1 */
#if ((defined STM32PERFORMANCE)&&(defined THREE_SHUNT)&&(defined SINGLEDRIVE))
  oCurrSensor[M1] = (CPWMC)R3LM1_NewObject(&PWMnCurrFdbkParamsM1, &R3_LM1ParamsSD);
#elif ((defined STM32PERFORMANCE)&&(defined SINGLE_SHUNT)&&(defined SINGLEDRIVE))
  oCurrSensor[M1] = (CPWMC)R1LM1_NewObject(&PWMnCurrFdbkParamsM1, &R1_LM1ParamsSD);
#elif ((defined STM32PERFORMANCE)&&(defined ICS_SENSORS)&&(defined SINGLEDRIVE))
  oCurrSensor[M1] = (CPWMC)ILM1_NewObject(&PWMnCurrFdbkParamsM1, &ICS_LM1ParamsSD);  
#elif ((defined STM32VALUE)&&(defined SINGLE_SHUNT)&&(defined SINGLEDRIVE))  
  oCurrSensor[M1] = (CPWMC)R1VL1_NewObject(&PWMnCurrFdbkParamsM1, &R1_VL1ParamsSD);
#elif ((defined STM32HD)&&(defined SINGLE_SHUNT))
  oCurrSensor[M1] = (CPWMC)R1HD2_NewObject(&PWMnCurrFdbkParamsM1, &R1_DDParamsM1);
#elif ((defined STM32HD)&&(defined THREE_SHUNT))
  oCurrSensor[M1] = (CPWMC)R3HD2_NewObject(&PWMnCurrFdbkParamsM1, &R3_DDParamsM1);
#elif ((defined STM32HD)&&(defined ICS_SENSORS))
  oCurrSensor[M1] = (CPWMC)IHD2_NewObject(&PWMnCurrFdbkParamsM1, &ICS_DDParamsM1);
#elif ((defined STM32F2XX)&&(defined THREE_SHUNT))
  oCurrSensor[M1] = (CPWMC)R3F2XX_NewObject(&PWMnCurrFdbkParamsM1, &R3_DDParamsM1);
#elif ((defined STM32F2XX)&&(defined SINGLE_SHUNT))
  oCurrSensor[M1] = (CPWMC)R1F2XX_NewObject(&PWMnCurrFdbkParamsM1, &R1_DDParamsM1);
#elif ((defined STM32F2XX)&&(defined ICS_SENSORS))
  oCurrSensor[M1] = (CPWMC)IF2XX_NewObject(&PWMnCurrFdbkParamsM1, &ICS_DDParamsM1);
#elif ((defined STM32F40XX || defined STM32F446xC_xE)&&(defined THREE_SHUNT))
  oCurrSensor[M1] = (CPWMC)R3F4XX_NewObject(&PWMnCurrFdbkParamsM1, &R3_DDParamsM1);
#elif ((defined STM32F40XX || defined STM32F446xC_xE)&&(defined SINGLE_SHUNT))
  oCurrSensor[M1] = (CPWMC)R1F4XX_NewObject(&PWMnCurrFdbkParamsM1, &R1_DDParamsM1);
#elif ((defined STM32F40XX || defined STM32F446xC_xE)&&(defined ICS_SENSORS))
  oCurrSensor[M1] = (CPWMC)IF4XX_NewObject(&PWMnCurrFdbkParamsM1, &ICS_DDParamsM1);
#elif ((defined STM32F0XX)&&(defined SINGLE_SHUNT))
  oCurrSensor[M1] = (CPWMC)R1F0XX_NewObject(&PWMnCurrFdbkParamsM1, &R1_SDParams);
#elif ((defined STM32F0XX)&&(defined THREE_SHUNT))
  oCurrSensor[M1] = (CPWMC)R3F0XX_NewObject(&PWMnCurrFdbkParamsM1, &R3_F0XXParamsSD);
#elif ((defined STM32F302X8) && (defined THREE_SHUNT))
  oCurrSensor[M1] = (CPWMC)R3_1_F3XX_NewObject(&PWMnCurrFdbkParamsM1, &R3_1_F30XParamsM1);
#elif ((defined STM32F30X)&&(defined THREE_SHUNT_INDEPENDENT_RESOURCES))
  oCurrSensor[M1] = (CPWMC)R3_4_F3XX_NewObject(&PWMnCurrFdbkParamsM1, &R3_4_F30XParamsM1);
#elif ((defined STM32F30X)&&(defined THREE_SHUNT_SHARED_RESOURCES))
  oCurrSensor[M1] = (CPWMC)R3_2_F3XX_NewObject(&PWMnCurrFdbkParamsM1, &R3_2_F30XParamsM1);
#elif ((defined STM32F30X)&&(defined SINGLE_SHUNT))
  oCurrSensor[M1] = (CPWMC)R1F3XX_NewObject(&PWMnCurrFdbkParamsM1,&R1_F30XParamsM1);
#elif ((defined STM32F30X)&&(defined ICS_SENSORS))
  oCurrSensor[M1] = (CPWMC)IF3XX_NewObject(&PWMnCurrFdbkParamsM1, &ICS_DDParamsM1);
#else  
#error "Invalid boot: configuration not valid or to be assigned"
#endif

#if defined(IPMSM_MTPA)
  oMTPA[M1] = MTPA_NewObject(&MTPAParamsM1); /* only if M1 has MTPA */
#endif

#if ((HW_OV_CURRENT_PROT_BYPASS == ENABLE) && (ON_OVER_VOLTAGE == TURN_ON_LOW_SIDES))
  oOCPDisabling[M1] = DOUT_NewObject(&DOUT_OCPDisablingParamsM1); 
  DOUT_Init(oOCPDisabling[M1]);
#endif   
  
#ifdef DUALDRIVE
  
  oCLM[M2] = CLM_NewObject(&CircleLimitationParamsM2);

#if defined(FLUX_WEAKENING2)
  oFW[M2] = FW_NewObject(&FWParamsM2); /* only if M2 has FW */
#endif
  
#if defined(FEED_FORWARD_CURRENT_REGULATION2)
  oFF[M2] = FF_NewObject(&FFParamsM2); /* only if M2 has FF */
#endif
  
#if defined(IPMSM_MTPA2)
  oMTPA[M2] = MTPA_NewObject(&MTPAParamsM2); /* only if M2 has MTPA */
#endif
  
#if defined(OPEN_LOOP2)
  oOL[M2] = OL_NewObject(&OpenLoop_ParamsM2); /* only if M2 has OPEN LOOP */
#endif
  
#if defined(HFINJECTION2) 
  oHFI[M2] = HFI_FP_NewObject(&HiFreqInj_FPU_ParamsM2); /* only if M2 has HFI */
#endif

  /* PWMC derived class object instantiation M2 */
#if ((defined STM32HD)&&(defined SINGLE_SHUNT2))
  oCurrSensor[M2] = (CPWMC)R1HD2_NewObject(&PWMnCurrFdbkParamsM2, &R1_DDParamsM2);
#elif ((defined STM32HD)&&(defined THREE_SHUNT2))
  oCurrSensor[M2] = (CPWMC)R3HD2_NewObject(&PWMnCurrFdbkParamsM2, &R3_DDParamsM2);
#elif ((defined STM32HD)&&(defined ICS_SENSORS2))
  oCurrSensor[M2] = (CPWMC)IHD2_NewObject(&PWMnCurrFdbkParamsM2, &ICS_DDParamsM2);
#elif ((defined STM32F2XX)&&(defined SINGLE_SHUNT2))
  oCurrSensor[M2] = (CPWMC)R1F2XX_NewObject(&PWMnCurrFdbkParamsM2, &R1_DDParamsM2);
#elif ((defined STM32F2XX)&&(defined THREE_SHUNT2))
  oCurrSensor[M2] = (CPWMC)R3F2XX_NewObject(&PWMnCurrFdbkParamsM2, &R3_DDParamsM2);
#elif ((defined STM32F2XX)&&(defined ICS_SENSORS2))
  oCurrSensor[M2] = (CPWMC)IF2XX_NewObject(&PWMnCurrFdbkParamsM2, &ICS_DDParamsM2);
#elif ((defined STM32F40XX || defined STM32F446xC_xE)&&(defined SINGLE_SHUNT2))
  oCurrSensor[M2] = (CPWMC)R1F4XX_NewObject(&PWMnCurrFdbkParamsM2, &R1_DDParamsM2);
#elif ((defined STM32F40XX || defined STM32F446xC_xE)&&(defined THREE_SHUNT2))
  oCurrSensor[M2] = (CPWMC)R3F4XX_NewObject(&PWMnCurrFdbkParamsM2, &R3_DDParamsM2);
#elif ((defined STM32F40XX || defined STM32F446xC_xE)&&(defined ICS_SENSORS2))
  oCurrSensor[M2] = (CPWMC)IF4XX_NewObject(&PWMnCurrFdbkParamsM2, &ICS_DDParamsM2);
#elif ((defined STM32F30X)&&(defined THREE_SHUNT_INDEPENDENT_RESOURCES2))
  oCurrSensor[M2] = (CPWMC)R3_4_F3XX_NewObject(&PWMnCurrFdbkParamsM2, &R3_4_F30XParamsM2);
#elif ((defined STM32F30X)&&(defined THREE_SHUNT_SHARED_RESOURCES2))
  oCurrSensor[M2] = (CPWMC)R3_2_F3XX_NewObject(&PWMnCurrFdbkParamsM2, &R3_2_F30XParamsM2);
#elif ((defined STM32F30X)&&(defined SINGLE_SHUNT2))
  oCurrSensor[M2] = (CPWMC)R1F3XX_NewObject(&PWMnCurrFdbkParamsM2,&R1_F30XParamsM2);
#elif ((defined STM32F30X)&&(defined ICS_SENSORS2))
  oCurrSensor[M2] = (CPWMC)IF3XX_NewObject(&PWMnCurrFdbkParamsM2, &ICS_DDParamsM2);
#else
#error "Invalid boot: configuration not valid or to be assigned"
#endif
  
#if ((HW_OV_CURRENT_PROT_BYPASS2 == ENABLE) && (ON_OVER_VOLTAGE2 == TURN_ON_LOW_SIDES))
  oOCPDisabling[M2] = DOUT_NewObject(&DOUT_OCPDisablingParamsM2); 
  DOUT_Init(oOCPDisabling[M2]);
#endif 
  
#endif /* Dual */
  
  /* Moved here to avoid that the second motor deletes the setting done in the init of the first motor. */
  PWMC_Init(oCurrSensor[M1]);
  
#ifdef DUALDRIVE
  
  PWMC_Init(oCurrSensor[M2]);
  
#endif
  
  /* PWMC derived class start timers */
#if ((defined STM32HD) && (defined THREE_SHUNT))
  R3HD2_StartTimers();
#elif ((defined STM32HD) && (defined SINGLE_SHUNT))
  R1HD2_StartTimers();
#elif ((defined STM32HD) && (defined ICS_SENSORS))
  IHD2_StartTimers();
#elif ((defined STM32F2XX) && (defined THREE_SHUNT))
  R3F2XX_StartTimers();
#elif ((defined STM32F2XX) && (defined SINGLE_SHUNT))
  R1F2XX_StartTimers();
#elif ((defined STM32F2XX) && (defined ICS_SENSORS))
  IF2XX_StartTimers();
#elif (((defined STM32F40XX) || (defined STM32F446xC_xE)) && (defined THREE_SHUNT))
  R3F4XX_StartTimers();
#elif (((defined STM32F40XX) || (defined STM32F446xC_xE)) && (defined SINGLE_SHUNT))
  R1F4XX_StartTimers();
#elif (((defined STM32F40XX) || (defined STM32F446xC_xE)) && (defined ICS_SENSORS))
  IF4XX_StartTimers();
#elif ((defined STM32F30X) && (defined THREE_SHUNT_INDEPENDENT_RESOURCES))
  R3_4_F3XX_StartTimers();
#elif ((defined STM32F30X) && (defined THREE_SHUNT_SHARED_RESOURCES))
  R3_2_F3XX_StartTimers();
#elif ((defined STM32F30X) && (defined SINGLE_SHUNT))
  R1F3XX_StartTimers();
#elif ((defined STM32F30X) && (defined ICS_SENSORS))
  IF3XX_StartTimers();  
#endif
  
  oSTM[M1] = STM_NewObject();
  STM_Init(oSTM[M1]);
  
  oPIDSpeed[M1] = (CPI)PI_NewObject(&PISpeedParamsM1);
  PI_ObjectInit(oPIDSpeed[M1]);  

#if (defined HFINJECTION)
  oSpeedSensor[M1] = (CSPD)HFI_FP_SPD_NewObject(&SpeednPosFdbkParamsM1,&HiFreqInjParamsM1); /* only if M1 has HFI */
#elif (defined STATE_OBSERVER_PLL)
  oSpeedSensor[M1] = (CSPD)STO_NewObject(&SpeednPosFdbkParamsM1,&STOParamsM1); /* speed sensor STO, ENC, HALL xNew and xParamsM1*/
#elif (defined STATE_OBSERVER_CORDIC)
  oSpeedSensor[M1] = (CSPD)STO_CR_NewObject(&SpeednPosFdbkParamsM1,&STO_CORDICParamsM1); /* speed sensor STO, ENC, HALL xNew and xParamsM1*/  
#elif (defined ENCODER)
  oSpeedSensor[M1] = (CSPD)ENC_NewObject(&SpeednPosFdbkParamsM1,&ENCParamsM1); /* speed sensor STO, ENC, HALL xNew and xParamsM1*/
#elif (defined HALL_SENSORS)
  #if (defined(STM32F30X))
    oSpeedSensor[M1] = (CSPD)HALL_F30X_NewObject(&SpeednPosFdbkParamsM1,&HALL_F30XParamsM1); /* speed sensor STO, ENC, HALL xNew and xParamsM1*/  
  #else /* Any other micros */
    oSpeedSensor[M1] = (CSPD)HALL_NewObject(&SpeednPosFdbkParamsM1,&HALLParamsM1); /* speed sensor STO, ENC, HALL xNew and xParamsM1*/  
  #endif
#endif  
  SPD_Init(oSpeedSensor[M1]);
  
#if AUX_SPEED_FDBK_M1 
#if (defined AUX_STATE_OBSERVER_PLL)
  oSpeedSensorAux_M1 = (CSPD)STO_NewObject(&SpeednPosFdbkParamsM1,&STOParamsM1); /* aux speed sensor STO, ENC, HALL xNew and xParamsM1*/
#elif (defined AUX_STATE_OBSERVER_CORDIC)
  oSpeedSensorAux_M1 = (CSPD)STO_CR_NewObject(&SpeednPosFdbkParamsM1,&STO_CORDICParamsM1); /* aux speed sensor STO, ENC, HALL xNew and xParamsM1*/  
#elif (defined VIEW_ENCODER_FEEDBACK)
  oSpeedSensorAux_M1 = (CSPD)ENC_NewObject(&SpeednPosFdbkParamsM1,&ENCParamsM1); /* aux speed sensor STO, ENC, HALL xNew and xParamsM1*/
#elif (defined VIEW_HALL_FEEDBACK)
  #if (defined(STM32F30X))
    oSpeedSensorAux_M1 = (CSPD)HALL_F30X_NewObject(&SpeednPosFdbkParamsM1,&HALL_F30XParamsM1); /* aux speed sensor STO, ENC, HALL xNew and xParamsM1*/  
  #else /* Any other micros */
    oSpeedSensorAux_M1 = (CSPD)HALL_NewObject(&SpeednPosFdbkParamsM1,&HALLParamsM1); /* aux speed sensor STO, ENC, HALL xNew and xParamsM1*/  
  #endif  
#endif  
  SPD_Init(oSpeedSensorAux_M1);
#endif
  
  oSTC[M1] = STC_NewObject(&SpeednTorqCtrlParamsM1);
  STC_Init(oSTC[M1],oPIDSpeed[M1],oSpeedSensor[M1]);

#if ((defined NO_SPEED_SENSORS)||(defined ENCODER)||(defined VIEW_ENCODER_FEEDBACK))
  oVSS_M1 = (CSPD)VSS_NewObject(&SpeednPosFdbkParamsM1,&VirtualSpeedSensorParamsM1);
  SPD_Init(oVSS_M1);
#endif
#if (defined NO_SPEED_SENSORS)
  oRUC_M1 = RUC_NewObject(&RevupCtrlParamsM1);        /* only if sensorless*/
  RUC_Init(oRUC_M1,oSTC[M1],(CVSS_SPD)oVSS_M1, (CSTO_SPD)oSpeedSensor[M1], (CPWMC) oCurrSensor[M1]);        /* only if sensorless*/
#endif
#if ((defined ENCODER)||(defined VIEW_ENCODER_FEEDBACK))
  oEAC_M1 = (CEAC)EAC_NewObject(&EncAlignCtrlParamsM1);
#endif
#if (defined ENCODER)
  EAC_Init(oEAC_M1,oSTC[M1],(CVSS_SPD)oVSS_M1,(CENC_SPD)oSpeedSensor[M1]);
#endif  
#if (defined VIEW_ENCODER_FEEDBACK)
  EAC_Init(oEAC_M1,oSTC[M1],(CVSS_SPD)oVSS_M1,(CENC_SPD)oSpeedSensorAux_M1);
#endif
  
  oPIDIq[M1] = (CPI)PI_NewObject(&PIIqParamsM1);
  PI_ObjectInit(oPIDIq[M1]); 
  
  oPIDId[M1] = (CPI)PI_NewObject(&PIIdParamsM1);
  PI_ObjectInit(oPIDId[M1]);   
  
#if defined BUS_VOLTAGE_MEASUREMENT
  oBusSensor[M1] = (CVBS)RVBS_NewObject(&RealBusVoltageSensorParamsM1,
                                       &RdividerParamsM1); /* powerboard configuration: Rdivider or Virtual*/  
#else
  oBusSensor[M1] = (CVBS)VVBS_NewObject(&VirtualBusVoltageSensorParamsM1,
                                       &VirtualBusParamsM1); /* powerboard configuration: Rdivider or Virtual*/    
#endif
  VBS_Init(oBusSensor[M1],oCurrSensor[M1]);
  
  oMPM[M1] = (CMPM)PQD_NewObject(&PQD_MotorPowerMeasurementParamsM1);
  PQD_MPMInitStruct.pFOCVars = &FOCVars[M1];
  PQD_MPMInitStruct.oVBS = oBusSensor[M1];
  MPM_Init(oMPM[M1],(pMPMInitStruct_t)&PQD_MPMInitStruct);
  
#if ON_OVER_VOLTAGE == TURN_ON_R_BRAKE
  oR_Brake[M1] = DOUT_NewObject(&R_BrakeParamsM1); 
  DOUT_Init(oR_Brake[M1]);
#endif 

  
#if defined HEAT_SINK_TEMPERATURE_MEASUREMENT
  oTemperatureSensor[M1] = (CTSNS)NTC_NewObject(&RealTempSensorParamsM1,
                                               &NTCParamsM1); /* powerboard configuration: NTC or Virtual*/
#else
  oTemperatureSensor[M1] = (CTSNS)VTS_NewObject(&VirtualTempSensorParamsM1,
                                               &VirtualTParamsM1); /* powerboard configuration: NTC or Virtual*/
#endif
  TSNS_Init(oTemperatureSensor[M1],oCurrSensor[M1]);  
  
#if defined(FLUX_WEAKENING)
  oPIFW[M1] = (CPI)PI_NewObject(&PIFluxWeakeningParamsM1);  /* only if M1 has FW */
  PI_ObjectInit(oPIFW[M1]);                                 /* PI_ObjectInit or PID_ObjectInit*/
  FWInitStructureM1.oFluxWeakeningPI = oPIFW[M1];           /* only if M1 has FW */
  FWInitStructureM1.oSpeedPI = oPIDSpeed[M1];                /* only if M1 has FW */
  FW_Init(oFW[M1],&FWInitStructureM1);                      /* only if M1 has FW */
#endif
  
#if defined(FEED_FORWARD_CURRENT_REGULATION)
  FFInitStructureM1.oVBS = oBusSensor[M1]; /* only if M1 has FF */
  FFInitStructureM1.oPI_q = oPIDIq[M1];    /* only if M1 has FF */
  FFInitStructureM1.oPI_d = oPIDId[M1];    /* only if M1 has FF */
  FF_Init(oFF[M1],&FFInitStructureM1);     /* only if M1 has FF */
#endif
  
#if defined(OPEN_LOOP)
  OL_Init(oOL[M1], oVSS_M1);     /* only if M1 has open loop */
#endif
  
#if defined(HFINJECTION) 
  HFI_FP_InitStructureM1.oHFI_FP_SpeedSensor = (CHFI_FP_SPD)(oSpeedSensor[M1]); /* only if M1 has HFI */
  HFI_FP_InitStructureM1.pFOCVarsPtr = &FOCVars[M1];                            /* only if M1 has HFI */
  HFI_FP_InitStructureM1.oPI_q = oPIDIq[M1];                                    /* only if M1 has HFI */
  HFI_FP_InitStructureM1.oPI_d = oPIDId[M1];                                    /* only if M1 has HFI */
  HFI_FP_InitStructureM1.oVbusSensor = oBusSensor[M1];                          /* only if M1 has HFI */
  HFI_FP_Init(oHFI[M1], &HFI_FP_InitStructureM1);                               /* only if M1 has HFI */
#endif
  
#if defined(MOTOR_PROFILER)
  SCC_InitStrM1.oPWMC = oCurrSensor[M1];
  SCC_InitStrM1.oVBS = oBusSensor[M1];
  SCC_InitStrM1.pFOCVars = &FOCVars[M1];
  SCC_InitStrM1.oSTM = oSTM[M1];
  SCC_InitStrM1.oVSS = oVSS_M1;
  SCC_InitStrM1.oCLM = oCLM[M1];
  SCC_InitStrM1.oPIDIq = oPIDIq[M1];
  SCC_InitStrM1.oPIDId = oPIDId[M1];
  SCC_InitStrM1.oRUC = oRUC_M1;
  SCC_InitStrM1.oSTO = oSpeedSensor[M1];
  SCC_InitStrM1.oSTC = oSTC[M1];
  SCC_InitStrM1.oOTT = oOTT[M1];
  SCC_Init(oSCC[M1], &SCC_InitStrM1);
#endif
  
#if defined(ONE_TOUCH_TUNING)
  OTT_InitStrM1.oSpeedSensor = oSpeedSensor[M1]; /* only if M1 has ONE TOUCH TUNING */
  OTT_InitStrM1.pFOCVarsPtr = &FOCVars[M1];      /* only if M1 has ONE TOUCH TUNING */
  OTT_InitStrM1.oPIDSpeed = oPIDSpeed[M1];       /* only if M1 has ONE TOUCH TUNING */
  OTT_InitStrM1.oSTC = oSTC[M1];                 /* only if M1 has ONE TOUCH TUNING */
  OTT_Init(oOTT[M1],&OTT_InitStrM1);
#endif
  
  oREMNG[M1] = REMNG_NewObject(&rampExtMngrHFParamsM1);
  REMNG_Init(oREMNG[M1]);
  
  FOC_Clear(M1);
  FOCVars[M1].bDriveInput = EXTERNAL;
  FOCVars[M1].Iqdref = STC_GetDefaultIqdref(oSTC[M1]);
  FOCVars[M1].UserIdref = STC_GetDefaultIqdref(oSTC[M1]).qI_Component2;
  
  oMCInterface[M1] = MCI_NewObject(MC_NULL);
  MCI_Init(oMCInterface[M1], oSTM[M1], oSTC[M1], &FOCVars[M1]);
  MCI_ExecSpeedRamp(oMCInterface[M1], 
  STC_GetMecSpeedRef01HzDefault(oSTC[M1]),0); /*First command to STC*/
  
  oMCIList[M1] = oMCInterface[M1];

#ifdef MC_TUNING_INTERFACE
  oMCTuning[M1] = MCT_NewObject(MC_NULL);
  
  MCTInitStruct.oPIDSpeed = oPIDSpeed[M1];
  MCTInitStruct.oPIDIq = oPIDIq[M1];
  MCTInitStruct.oPIDId = oPIDId[M1];
#if defined(FLUX_WEAKENING)
  MCTInitStruct.oPIDFluxWeakening = oPIFW[M1]; /* only if M1 has FW */
#else
  MCTInitStruct.oPIDFluxWeakening = MC_NULL; /* if M1 doesn't has FW */
#endif
  MCTInitStruct.oPWMnCurrFdbk = oCurrSensor[M1];
#if (defined NO_SPEED_SENSORS)  
  MCTInitStruct.oRevupCtrl = oRUC_M1;              /* only if M1 is sensorless*/
#else
  MCTInitStruct.oRevupCtrl = MC_NULL;              /* only if M1 is not sensorless*/
#endif
  MCTInitStruct.oSpeedSensorMain = oSpeedSensor[M1];
#if (AUX_SPEED_FDBK_M1)
  MCTInitStruct.oSpeedSensorAux = oSpeedSensorAux_M1; /* only if M1 has auxiliary sensor oSpeedSensorAux_M1*/
#else
  MCTInitStruct.oSpeedSensorAux = MC_NULL;            /* only if M1 has auxiliary sensor oSpeedSensorAux_M1*/
#endif
#if (defined NO_SPEED_SENSORS)
  MCTInitStruct.oSpeedSensorVirtual = oVSS_M1;  /* only if M1 is sensorless*/
#endif
  MCTInitStruct.oSpeednTorqueCtrl = oSTC[M1];
  MCTInitStruct.oStateMachine = oSTM[M1];
  MCTInitStruct.oTemperatureSensor = oTemperatureSensor[M1];
  MCTInitStruct.oBusVoltageSensor = oBusSensor[M1];
  MCTInitStruct.oBrakeDigitalOutput = MC_NULL;   /* brake is defined, oBrakeM1*/
  MCTInitStruct.oNTCRelay = MC_NULL;             /* relay is defined, oRelayM1*/
  MCTInitStruct.oMPM = oMPM[M1];
#if defined(FLUX_WEAKENING)
  MCTInitStruct.oFW = oFW[M1];
#else
  MCTInitStruct.oFW = MC_NULL;
#endif
#if defined(FEED_FORWARD_CURRENT_REGULATION)
  MCTInitStruct.oFF = oFF[M1];
#else
  MCTInitStruct.oFF = MC_NULL;
#endif
#if defined(HFINJECTION)
  MCTInitStruct.oHFI = oHFI[M1];
#else
  MCTInitStruct.oHFI = MC_NULL;
#endif
#if defined(MOTOR_PROFILER)
  MCTInitStruct.oSCC = oSCC[M1];
#else
  MCTInitStruct.oSCC = MC_NULL;
#endif
#if defined (ONE_TOUCH_TUNING)
  MCTInitStruct.oOTT = oOTT[M1];
#else
  MCTInitStruct.oOTT = MC_NULL;
#endif

  MCT_Init(oMCTuning[M1],MCTInitStruct);
  
  oMCTList[M1] = oMCTuning[M1];
#endif

#ifdef DUALDRIVE
  oSTM[M2] = STM_NewObject();
  STM_Init(oSTM[M2]);
  
  oPIDSpeed[M2] = (CPI)PI_NewObject(&PISpeedParamsM2);
  PI_ObjectInit(oPIDSpeed[M2]); 

#if (defined HFINJECTION2)
  oSpeedSensor[M2] = (CSPD)HFI_FP_SPD_NewObject(&SpeednPosFdbkParamsM2,&HiFreqInjParamsM2); /* only if M2 has HFI */
#elif (defined STATE_OBSERVER_PLL2) 
  oSpeedSensor[M2] = (CSPD)STO_NewObject(&SpeednPosFdbkParamsM2,&STOParamsM2); /* speed sensor STO, ENC, HALL xNew and xParamsM2*/
#elif (defined STATE_OBSERVER_CORDIC2) 
  oSpeedSensor[M2] = (CSPD)STO_CR_NewObject(&SpeednPosFdbkParamsM2,&STO_CORDICParamsM2); /* speed sensor STO, ENC, HALL xNew and xParamsM2*/  
#elif (defined ENCODER2)
  oSpeedSensor[M2] = (CSPD)ENC_NewObject(&SpeednPosFdbkParamsM2,&ENCParamsM2); /* speed sensor STO, ENC, HALL xNew and xParamsM2*/
#elif (defined HALL_SENSORS2)
  #if (defined(STM32F30X))
    oSpeedSensor[M2] = (CSPD)HALL_F30X_NewObject(&SpeednPosFdbkParamsM2,&HALL_F30XParamsM2); /* speed sensor STO, ENC, HALL xNew and xParamsM2*/  
  #else /* Any other micros */
    oSpeedSensor[M2] = (CSPD)HALL_NewObject(&SpeednPosFdbkParamsM2,&HALLParamsM2); /* speed sensor STO, ENC, HALL xNew and xParamsM2*/  
  #endif  
#else
#error "Invalid boot: configuration not valid or to be assigned"
#endif
  
  SPD_Init(oSpeedSensor[M2]);
  
#if AUX_SPEED_FDBK_M2 
#if (defined AUX_STATE_OBSERVER_PLL2)
  oSpeedSensorAux_M2 = (CSPD)STO_NewObject(&SpeednPosFdbkParamsM2,&STOParamsM2); /* aux speed sensor STO, ENC, HALL xNew and xParamsM2*/
#elif (defined AUX_STATE_OBSERVER_CORDIC2)
  oSpeedSensorAux_M2 = (CSPD)STO_CR_NewObject(&SpeednPosFdbkParamsM2,&STO_CORDICParamsM2); /* aux speed sensor STO, ENC, HALL xNew and xParamsM2*/  
#elif (defined VIEW_ENCODER_FEEDBACK2)
  oSpeedSensorAux_M2 = (CSPD)ENC_NewObject(&SpeednPosFdbkParamsM2,&ENCParamsM2); /* aux speed sensor STO, ENC, HALL xNew and xParamsM2*/
#elif (defined VIEW_HALL_FEEDBACK2)
  #if (defined(STM32F30X))
    oSpeedSensorAux_M2 = (CSPD)HALL_F30X_NewObject(&SpeednPosFdbkParamsM2,&HALL_F30XParamsM2); /* aux speed sensor STO, ENC, HALL xNew and xParamsM2*/  
  #else /* Any other micros */
    oSpeedSensorAux_M2 = (CSPD)HALL_NewObject(&SpeednPosFdbkParamsM2,&HALLParamsM2); /* aux speed sensor STO, ENC, HALL xNew and xParamsM2*/
  #endif
#endif  
  SPD_Init(oSpeedSensorAux_M2);
#endif
  
  oSTC[M2] = STC_NewObject(&SpeednTorqCtrlParamsM2);
  STC_Init(oSTC[M2],oPIDSpeed[M2],oSpeedSensor[M2]);

#if ((defined NO_SPEED_SENSORS2)||(defined ENCODER2)||(defined VIEW_ENCODER_FEEDBACK2))
  oVSS_M2 = (CSPD)VSS_NewObject(&SpeednPosFdbkParamsM2,&VirtualSpeedSensorParamsM2);
  SPD_Init(oVSS_M2);                               
#endif
#if (defined NO_SPEED_SENSORS2)  
  oRUC_M2 = RUC_NewObject(&RevupCtrlParamsM2);        /* only if sensorless*/
  RUC_Init(oRUC_M2,oSTC[M2],(CVSS_SPD)oVSS_M2,(CSTO_SPD)oSpeedSensor[M2],(CPWMC)oCurrSensor[M2]);        /* only if sensorless*/
#endif
#if ((defined ENCODER2)||(defined VIEW_ENCODER_FEEDBACK2))  
  oEAC_M2 = (CEAC)EAC_NewObject(&EncAlignCtrlParamsM2);
#endif
#if (defined ENCODER2)
  EAC_Init(oEAC_M2,oSTC[M2],(CVSS_SPD)oVSS_M2,(CENC_SPD)oSpeedSensor[M2]);
#endif
#if (defined VIEW_ENCODER_FEEDBACK2)
  EAC_Init(oEAC_M2,oSTC[M2],(CVSS_SPD)oVSS_M2,(CENC_SPD)oSpeedSensorAux_M2);
#endif  
  
  oPIDIq[M2] = (CPI)PI_NewObject(&PIIqParamsM2);
  PI_ObjectInit(oPIDIq[M2]);
  
  oPIDId[M2] = (CPI)PI_NewObject(&PIIdParamsM2);
  PI_ObjectInit(oPIDId[M2]);   

#if defined BUS_VOLTAGE_MEASUREMENT2
  oBusSensor[M2] = (CVBS)RVBS_NewObject(&RealBusVoltageSensorParamsM2,
                                       &RdividerParamsM2); /* powerboard configuration: Rdivider or Virtual*/  
#else
  oBusSensor[M2] = (CVBS)VVBS_NewObject(&VirtualBusVoltageSensorParamsM2,
                                       &VirtualBusParamsM2); /* powerboard configuration: Rdivider or Virtual*/    
#endif  
  VBS_Init(oBusSensor[M2],oCurrSensor[M1]);
  
  oMPM[M2] = (CMPM)PQD_NewObject(&PQD_MotorPowerMeasurementParamsM2);
  PQD_MPMInitStruct.pFOCVars = &FOCVars[M2];
  PQD_MPMInitStruct.oVBS = oBusSensor[M2];
  MPM_Init(oMPM[M2],(pMPMInitStruct_t)&PQD_MPMInitStruct);
  
#if ON_OVER_VOLTAGE2 == TURN_ON_R_BRAKE
  oR_Brake[M2] = DOUT_NewObject(&R_BrakeParamsM2); 
  DOUT_Init(oR_Brake[M2]);
#endif
  
#if defined HEAT_SINK_TEMPERATURE_MEASUREMENT2
  oTemperatureSensor[M2] = (CTSNS)NTC_NewObject(&RealTempSensorParamsM2,
                                               &NTCParamsM2); /* powerboard configuration: NTC or Virtual*/
#else
  oTemperatureSensor[M2] = (CTSNS)VTS_NewObject(&VirtualTempSensorParamsM2,
                                               &VirtualTParamsM2); /* powerboard configuration: NTC or Virtual*/
#endif
  
  TSNS_Init(oTemperatureSensor[M2],oCurrSensor[M1]);  
  
#if defined(FLUX_WEAKENING2)
  oPIFW[M2] = (CPI)PI_NewObject(&PIFluxWeakeningParamsM2);  /* only if M2 has FW */
  PI_ObjectInit(oPIFW[M2]);                                 /* PI_ObjectInit or PID_ObjectInit*/
  FWInitStructureM2.oFluxWeakeningPI = oPIFW[M2];           /* only if M2 has FW */
  FWInitStructureM2.oSpeedPI = oPIDSpeed[M2];                /* only if M2 has FW */
  FW_Init(oFW[M2],&FWInitStructureM2);                      /* only if M2 has FW */
#endif
  
#if defined(FEED_FORWARD_CURRENT_REGULATION2)
  FFInitStructureM2.oVBS = oBusSensor[M2]; /* only if M2 has FF */
  FFInitStructureM2.oPI_q = oPIDIq[M2];    /* only if M2 has FF */
  FFInitStructureM2.oPI_d = oPIDId[M2];    /* only if M2 has FF */
  FF_Init(oFF[M2],&FFInitStructureM2);    /* only if M2 has FF */
#endif
  
#if defined(OPEN_LOOP2)
  OL_Init(oOL[M2], oVSS_M2);     /* only if M2 has open loop */
#endif
  
#if defined(HFINJECTION2) 
  HFI_FP_InitStructureM2.oHFI_FP_SpeedSensor = (CHFI_FP_SPD)(oSpeedSensor[M2]); /* only if M2 has HFI */
  HFI_FP_InitStructureM2.pFOCVarsPtr = &FOCVars[M2];                            /* only if M2 has HFI */
  HFI_FP_InitStructureM2.oPI_q = oPIDIq[M2];                                    /* only if M2 has HFI */
  HFI_FP_InitStructureM2.oPI_d = oPIDId[M2];                                    /* only if M2 has HFI */
  HFI_FP_InitStructureM2.oVbusSensor = oBusSensor[M2];                          /* only if M1 has HFI */
  HFI_FP_Init(oHFI[M2], &HFI_FP_InitStructureM2);                               /* only if M2 has HFI */
#endif
  
  oREMNG[M2] = REMNG_NewObject(&rampExtMngrHFParamsM2);
  REMNG_Init(oREMNG[M2]);  
  
  FOC_Clear(M2);
  FOCVars[M2].bDriveInput = EXTERNAL;
  FOCVars[M2].Iqdref = STC_GetDefaultIqdref(oSTC[M2]);
  FOCVars[M2].UserIdref = STC_GetDefaultIqdref(oSTC[M2]).qI_Component2;
    
  oMCInterface[M2] = MCI_NewObject(MC_NULL);
  MCI_Init(oMCInterface[M2], oSTM[M2], oSTC[M2], &FOCVars[M2]);
  MCI_ExecSpeedRamp(oMCInterface[M2], 
  STC_GetMecSpeedRef01HzDefault(oSTC[M2]),0); /*First command to STC*/
  
  oMCIList[M2] = oMCInterface[M2]; 
    
#ifdef MC_TUNING_INTERFACE
  oMCTuning[M2] = MCT_NewObject(MC_NULL);
  
  MCTInitStruct.oPIDSpeed = oPIDSpeed[M2];
  MCTInitStruct.oPIDIq = oPIDIq[M2];
  MCTInitStruct.oPIDId = oPIDId[M2];
#if defined(FLUX_WEAKENING2)
  MCTInitStruct.oPIDFluxWeakening = oPIFW[M2]; /* only if M2 has FW */
#else
  MCTInitStruct.oPIDFluxWeakening = MC_NULL; /* if M2 doesn't has FW */
#endif
  MCTInitStruct.oPWMnCurrFdbk = oCurrSensor[M2];
#if (defined NO_SPEED_SENSORS2)  
  MCTInitStruct.oRevupCtrl = oRUC_M2;              /* only if M2 is sensorless*/
#else
  MCTInitStruct.oRevupCtrl = MC_NULL;              /* only if M2 is not sensorless*/
#endif
  MCTInitStruct.oSpeedSensorMain = oSpeedSensor[M2];
#if (AUX_SPEED_FDBK_M2)
  MCTInitStruct.oSpeedSensorAux = oSpeedSensorAux_M2; /* only if M2 has auxiliary sensor oSpeedSensorAux_M2*/
#else
  MCTInitStruct.oSpeedSensorAux = MC_NULL;            /* only if M2 has auxiliary sensor oSpeedSensorAux_M2*/
#endif

#if (defined NO_SPEED_SENSORS2)
  MCTInitStruct.oSpeedSensorVirtual = oVSS_M2;  /* only if M2 is sensorless*/
#endif
  MCTInitStruct.oSpeednTorqueCtrl = oSTC[M2];
  MCTInitStruct.oStateMachine = oSTM[M2];
  MCTInitStruct.oTemperatureSensor = oTemperatureSensor[M2];
  MCTInitStruct.oBusVoltageSensor = oBusSensor[M2];
  MCTInitStruct.oBrakeDigitalOutput = MC_NULL;   /* brake is defined, oBrakeM2*/
  MCTInitStruct.oNTCRelay = MC_NULL;             /* relay is defined, oRelayM2*/
  MCTInitStruct.oMPM = oMPM[M2];
#if defined(FLUX_WEAKENING2)
  MCTInitStruct.oFW = oFW[M2];
#else
  MCTInitStruct.oFW = MC_NULL;
#endif
#if defined(FEED_FORWARD_CURRENT_REGULATION2)
  MCTInitStruct.oFF = oFF[M2];
#else
  MCTInitStruct.oFF = MC_NULL;
#endif
#if defined(HFINJECTION2)
  MCTInitStruct.oHFI = oHFI[M2];
#else
  MCTInitStruct.oHFI = MC_NULL;
#endif
  MCTInitStruct.oSCC = MC_NULL;
  
  MCT_Init(oMCTuning[M2],MCTInitStruct);
  
  oMCTList[M2] = oMCTuning[M2];
#endif
#endif  
  
#if (INRUSH_CURRLIMIT_ENABLING == ENABLE)
  oICLDOUT_M1 = DOUT_NewObject(&ICLDOUTParamsM1);
  DOUT_Init(oICLDOUT_M1);
  oICL_M1 = ICL_NewObject(&InrushCurrentLimiterParamsM1);
  ICL_Init(oICL_M1, oBusSensor[M1], oICLDOUT_M1);
  STM_NextState(oSTM[M1],ICLWAIT);
#endif 
#if (INRUSH_CURRLIMIT_ENABLING2 == ENABLE)
  oICLDOUT_M2 = DOUT_NewObject(&ICLDOUTParamsM2);
  DOUT_Init(oICLDOUT_M2);
  oICL_M2 = ICL_NewObject(&InrushCurrentLimiterParamsM2);
  ICL_Init(oICL_M2, oBusSensor[M2], oICLDOUT_M2);
  STM_NextState(oSTM[M2],ICLWAIT);
#endif
  
  bMCBootCompleted = 1;
}

/**
  * @brief  It executes MC tasks: safety task and medium frequency for all
  *         drive instances. It have to be clocked with Systick frequnecy.
  * @param  None
  * @retval None
  */
void MC_Scheduler(void)
{
  if (bMCBootCompleted == 1)
  {
    TIM_ITConfig(PWM_TIMER_SELECTION, TIM_IT_Break, (FunctionalState)(ENABLE));
    
    if(hMFTaskCounterM1 > 0u)
    {
      hMFTaskCounterM1--;
    }   
    else
    {
      TSK_MediumFrequencyTaskM1();
      /****************************** USE ONLY FOR SDK 4.0 EXAMPLES *************/  
#if defined(EXAMPLE_SPEEDMONITOR)
      ARR_TIM5_update(oSpeedSensor[M1]); 
#endif
      /**************************************************************************/    
      hMFTaskCounterM1 = MF_TASK_OCCURENCE_TICKS;
    } 
    
#ifdef DUALDRIVE
    if(hMFTaskCounterM2 > 0u)
    {
      hMFTaskCounterM2--;
    }   
    else
    {
      TSK_MediumFrequencyTaskM2();
      hMFTaskCounterM2 = MF_TASK_OCCURENCE_TICKS2;
    }
#endif
    
    if(hBootCapDelayCounterM1 > 0u)
    {
      hBootCapDelayCounterM1--;    
    } 
    if(hStopPermanencyCounterM1 > 0u)
    {
      hStopPermanencyCounterM1--;    
    }
#ifdef DUALDRIVE
    if(hBootCapDelayCounterM2 > 0u)
    {
      hBootCapDelayCounterM2--;    
    }
    if(hStopPermanencyCounterM2 > 0u)
    {
      hStopPermanencyCounterM2--;    
    }
#endif
  }
  else
  {
    TIM_ITConfig(PWM_TIMER_SELECTION, TIM_IT_Break, (FunctionalState)(DISABLE));
  }
}

/**
  * @brief  It executes some of the control duties on Motor 1 accordingly with  
  *         the present state of its state machine. In particular, duties 
  *         requiring a specific timing (e.g. speed controller) are here 
  *         executed
  * @param  None
  * @retval void
  */
void TSK_MediumFrequencyTaskM1(void)
{
  State_t StateM1;
  bool IsSpeedReliable;
  int16_t wAux = 0;
  
#if (INRUSH_CURRLIMIT_ENABLING == ENABLE)
  ICLState_t ICLState = ICL_Exec(oICL_M1);
#endif
  
#if (defined(VIEW_HALL_FEEDBACK)||defined(VIEW_ENCODER_FEEDBACK)||defined(AUX_STATE_OBSERVER_PLL)||defined(AUX_STATE_OBSERVER_CORDIC))
  bool IsSpeedReliableAux = SPD_CalcAvrgMecSpeed01Hz(oSpeedSensorAux_M1,&wAux);  /*  Only if Aux sensored is enabled */
#endif  
  
  IsSpeedReliable = SPD_CalcAvrgMecSpeed01Hz(oSpeedSensor[M1],&wAux);
  
  MPM_CalcElMotorPower(oMPM[M1]);
  
  StateM1 = STM_GetState(oSTM[M1]);
  switch(StateM1)
  {
#if (INRUSH_CURRLIMIT_ENABLING == ENABLE)
  case ICLWAIT:
    if (ICLState == ICL_INACTIVE)
    {
      /* If ICL Inactive move to IDLE */
      STM_NextState(oSTM[M1],IDLE);
    }
    break;
#endif
    
#if ((defined ENCODER)||(defined VIEW_ENCODER_FEEDBACK)) /*  only for encoder*/
  case IDLE:
    if (EAC_GetRestartState(oEAC_M1))
    {
      EAC_SetRestartState(oEAC_M1,FALSE); /* Reset restart flag*/
      STM_NextState(oSTM[M1],IDLE_START);
    }
    break;
#endif
    
  case IDLE_START:
#if defined(ONE_TOUCH_TUNING)
    OTT_Clear(oOTT[M1]);
#endif
#if ((defined ENCODER)||(defined VIEW_ENCODER_FEEDBACK)) /*  only for encoder*/
    if (EAC_IsAligned(oEAC_M1) == FALSE)
    {
      EAC_SetRestartState(oEAC_M1,TRUE); /* Set restart flag. Run after align*/
      STM_NextState(oSTM[M1],IDLE_ALIGNMENT);
      break;
    }
#endif
#if (CHARGE_BOOT_CAP_ENABLING == ENABLE)
    PWMC_TurnOnLowSides(oCurrSensor[M1]); 
    TSK_SetChargeBootCapDelayM1(CHARGE_BOOT_CAP_TICKS);
    STM_NextState(oSTM[M1],CHARGE_BOOT_CAP);
#else
    PWMC_CurrentReadingCalibr(oCurrSensor[M1],CRC_START);
    STM_NextState(oSTM[M1],OFFSET_CALIB);
#endif
    break;
  
#if (CHARGE_BOOT_CAP_ENABLING == ENABLE)
  case CHARGE_BOOT_CAP:
    if (TSK_ChargeBootCapDelayHasElapsedM1())
    {
      PWMC_CurrentReadingCalibr(oCurrSensor[M1],CRC_START);
      STM_NextState(oSTM[M1],OFFSET_CALIB);
    }
    break;
#endif
    
  case OFFSET_CALIB:
    if (PWMC_CurrentReadingCalibr(oCurrSensor[M1],CRC_EXEC))
    {
      STM_NextState(oSTM[M1],CLEAR);
    }
    break;
    
  case CLEAR:
#if (defined NO_SPEED_SENSORS)      
    FOCVars[M1].bDriveInput = EXTERNAL;                               /* only for sensorless */
    STC_SetSpeedSensor(oSTC[M1],oVSS_M1);                             /* only for sensorless */
    RUC_Clear(oRUC_M1,MCI_GetImposedMotorDirection(oMCInterface[M1]));/* only for sensorless */
    SWO_transitionStartM1 = FALSE;                                    /* only for sensorless */
#endif
    
#if (defined HFINJECTION)
    STC_SetSpeedSensor(oSTC[M1],oSpeedSensor[M1]);
#endif

    SPD_Clear(oSpeedSensor[M1]); 		/*  only for sensorless/hall/encoder main*/
#if (defined(AUX_STATE_OBSERVER_PLL) || defined(AUX_STATE_OBSERVER_CORDIC) || defined(VIEW_HALL_FEEDBACK) || defined(VIEW_ENCODER_FEEDBACK))
    SPD_Clear(oSpeedSensorAux_M1); 		/*  only for sensorless/hall main/auxiliary*/
#endif
    if(STM_NextState(oSTM[M1], START) == TRUE)
    {
      FOC_Clear(M1);
      
#if defined(MOTOR_PROFILER)
      SCC_Start(oSCC[M1]);
#endif
      
      PWMC_SwitchOnPWM(oCurrSensor[M1]);
    }
    break;
    
#if ((defined ENCODER)||(defined VIEW_ENCODER_FEEDBACK)) /*  only for encoder*/
    case IDLE_ALIGNMENT:
#if (CHARGE_BOOT_CAP_ENABLING == ENABLE)
    PWMC_TurnOnLowSides(oCurrSensor[M1]); 
    TSK_SetChargeBootCapDelayM1(CHARGE_BOOT_CAP_TICKS);
    STM_NextState(oSTM[M1],ALIGN_CHARGE_BOOT_CAP);
#else
    PWMC_CurrentReadingCalibr(oCurrSensor[M1],CRC_START);
    STM_NextState(oSTM[M1],ALIGN_OFFSET_CALIB);
#endif
    break;
  
#if (CHARGE_BOOT_CAP_ENABLING == ENABLE)
  case ALIGN_CHARGE_BOOT_CAP:
    if (TSK_ChargeBootCapDelayHasElapsedM1())
    {
      PWMC_CurrentReadingCalibr(oCurrSensor[M1],CRC_START);
      STM_NextState(oSTM[M1],ALIGN_OFFSET_CALIB);
    }
    break;
#endif
    
  case ALIGN_OFFSET_CALIB:
    if (PWMC_CurrentReadingCalibr(oCurrSensor[M1],CRC_EXEC))
    {
      STM_NextState(oSTM[M1],ALIGN_CLEAR);
    }
    break;
    
  case ALIGN_CLEAR:
    FOCVars[M1].bDriveInput = EXTERNAL;
    STC_SetSpeedSensor(oSTC[M1],oVSS_M1);
    EAC_StartAlignment(oEAC_M1);
    
    if(STM_NextState(oSTM[M1], ALIGNMENT) == TRUE)
    {
      FOC_Clear(M1);
      PWMC_SwitchOnPWM(oCurrSensor[M1]); 
    }
    break;
#endif
    
  case START:
    {    
#if (defined NO_SPEED_SENSORS)  /* only for sensor-less control */  
      
      int16_t hForcedMecSpeed01Hz;
      Curr_Components IqdRef;
      bool StartUpTransitionEnded;
      bool StartUpDoTransition;
      
      if(!RUC_Exec(oRUC_M1))
      {
        #if ((OPEN_LOOP_FOC == ENABLE) || defined (MOTOR_PROFILER))
          /* No error generated when OPEN LOOP or MOTOR_PROFILER is enabled. */
        #else
          STM_FaultProcessing(oSTM[M1], MC_START_UP, 0);  /*Time allowed for startup has ended*/
        #endif
      }
      else
      {
        if (SWO_transitionStartM1 == FALSE)
        {
          IqdRef.qI_Component1 = STC_CalcTorqueReference(oSTC[M1]);
          IqdRef.qI_Component2 = FOCVars[M1].UserIdref;
          FOCVars[M1].Iqdref = IqdRef;
        }
      }
      
      StartUpTransitionEnded = SPD_CalcAvrgMecSpeed01Hz(oVSS_M1,&hForcedMecSpeed01Hz);
      
      #if (OPEN_LOOP_FOC == ENABLE)
      {
        int16_t hOLFinalMecSpeed01Hz = MCI_GetLastRampFinalSpeed(oMCInterface[M1]);
        if (hOLFinalMecSpeed01Hz != VSPD_GetLastRampFinalSpeed(oVSS_M1))
        {
          VSPD_SetMecAcceleration(oVSS_M1,hOLFinalMecSpeed01Hz,OPEN_LOOP_SPEED_RAMP_DURATION_MS);
        }
        OL_Calc(oOL[M1]);
      }
      #endif
      
#if (defined(STATE_OBSERVER_PLL))
      StartUpDoTransition = VSPD_SetStartTransition(oVSS_M1,STO_IsObserverConverged((CSTO_SPD)oSpeedSensor[M1],hForcedMecSpeed01Hz));
#elif (defined(STATE_OBSERVER_CORDIC))
      StartUpDoTransition = VSPD_SetStartTransition(oVSS_M1,STO_CR_IsObserverConverged((CSTO_CR_SPD)oSpeedSensor[M1],hForcedMecSpeed01Hz));
#endif
      
      if (VSPD_IsTransitionOngoing(oVSS_M1))
      {
        if (SWO_transitionStartM1 == FALSE)
        {
          int16_t Iq = 0;
          Curr_Components StatorCurrent = MCM_Park(FOCVars[M1].Ialphabeta, SPD_GetElAngle(oSpeedSensor[M1]));
          Iq = StatorCurrent.qI_Component1;
          
          REMNG_Init(oREMNG[M1]);
          REMNG_ExecRamp(oREMNG[M1], FOCVars[M1].Iqdref.qI_Component1, 0);
          REMNG_ExecRamp(oREMNG[M1], Iq, TRANSITION_DURATION);
          
          SWO_transitionStartM1 = TRUE;
        }
      }
      else
      {
        if (SWO_transitionStartM1 == TRUE)
        {
          SWO_transitionStartM1 = FALSE;
        }
      }
      
      if (StartUpDoTransition == FALSE)
      {
        StartUpTransitionEnded = TRUE;
      }
      
      if (StartUpTransitionEnded == TRUE)
      {   
        PI_SetIntegralTerm(oPIDSpeed[M1],(int32_t)FOCVars[M1].Iqdref.qI_Component1*PI_GetKIDivisor(oPIDSpeed[M1]));
       
        STM_NextState(oSTM[M1], START_RUN);
      }
#endif
    
#if ((defined ENCODER)||(defined HALL_SENSORS))
      STM_NextState(oSTM[M1], START_RUN); /* only for sensored*/
#endif
      
#if (defined HFINJECTION)
      /* HFI validation check */
      if (HFI_FP_Start(oHFI[M1]) == TRUE)
      {
        STM_NextState(oSTM[M1], START_RUN);
      }
#endif
    }
    break;
#if ((defined ENCODER)||(defined VIEW_ENCODER_FEEDBACK))
  case ALIGNMENT:
    if(!EAC_Exec(oEAC_M1))
    {
      Curr_Components IqdRef;
      IqdRef.qI_Component1 = 0;
      IqdRef.qI_Component2 = STC_CalcTorqueReference(oSTC[M1]);
      FOCVars[M1].Iqdref = IqdRef;	
    }
    else
    {
      PWMC_SwitchOffPWM(oCurrSensor[M1]);
      STC_SetControlMode(oSTC[M1], STC_SPEED_MODE);
      STC_SetSpeedSensor(oSTC[M1],oSpeedSensor[M1]);
      STM_NextState(oSTM[M1], ANY_STOP);
    }    
    break;    
#endif
  case START_RUN:
#if defined(ONE_TOUCH_TUNING)
    OTT_SR(oOTT[M1]);
#endif
        
#if (defined NO_SPEED_SENSORS) /* only for sensor-less control */
    STC_SetSpeedSensor(oSTC[M1], oSpeedSensor[M1]); /*Observer has converged*/
#endif
    
#if (defined HFINJECTION)
    if (!HFI_FP_STMRUN(oHFI[M1]))
    {
      STM_FaultProcessing(oSTM[M1], MC_SPEED_FDBK, 0);
    }
    else      
#endif 
    {
      FOC_InitAdditionalMethods(M1);    
      FOC_CalcCurrRef(M1);
      STM_NextState(oSTM[M1], RUN);
    }
    
    STC_ForceSpeedReferenceToCurrentSpeed(oSTC[M1]); /* Init the reference speed to current speed */
    MCI_ExecBufferedCommands(oMCInterface[M1]); /* Exec the speed ramp after changing of the speed sensor */
    
    break;
  case RUN:
    MCI_ExecBufferedCommands(oMCInterface[M1]);
    FOC_CalcCurrRef(M1);
#if (defined HFINJECTION)
    if (STC_GetSpeedSensor(oSTC[M1]) == oSpeedSensor[M1])
    {
      int16_t hObsSpeed01Hz = SPD_GetAvrgMecSpeed01Hz((CSPD)oSpeedSensorAux_M1);
      int16_t hHFISpeedDpp = SPD_GetElSpeedDpp(oSpeedSensor[M1]);
      int32_t wtemp = (int32_t)hHFISpeedDpp * (int32_t)hObsSpeed01Hz;
      
      if(!IsSpeedReliable)
      {
        STM_FaultProcessing(oSTM[M1], MC_SPEED_FDBK, 0);
      }      
      
      if (wtemp < 0)
      {
        STO_SetPLL((CSTO_SPD)oSpeedSensorAux_M1, hHFISpeedDpp,
        SPD_GetElAngle(oSpeedSensor[M1]));      
      }      
      
      if (!HFI_FP_SPD_AccelerationStageReached((CHFI_FP_SPD)oSpeedSensor[M1]))
      {
        STO_SetPLL((CSTO_SPD)oSpeedSensorAux_M1, SPD_GetElSpeedDpp(oSpeedSensor[M1]),
                   SPD_GetElAngle(oSpeedSensor[M1]));        
      } 
      else
      {
        if (STO_IsObserverConverged((CSTO_SPD)oSpeedSensorAux_M1,SPD_GetAvrgMecSpeed01Hz(oSpeedSensor[M1])))
        {
          STC_SetSpeedSensor(oSTC[M1], oSpeedSensorAux_M1);
          HFI_FP_DisHFGeneration(oHFI[M1]);           
        }
      }
    }
    else  /* STO - HFI */
    {
      int16_t hObsSpeed01Hz = SPD_GetAvrgMecSpeed01Hz((CSPD)oSpeedSensorAux_M1);
      
      if(!IsSpeedReliableAux)
      {
        STM_FaultProcessing(oSTM[M1], MC_SPEED_FDBK, 0);
      }       
      
      if (HFI_FP_SPD_Restart((CHFI_FP_SPD)oSpeedSensor[M1],hObsSpeed01Hz))
      {
        if (HFI_FP_Restart(oHFI[M1]) == TRUE)
        {
          if (HFI_FP_SPD_IsConverged((CHFI_FP_SPD)oSpeedSensor[M1],hObsSpeed01Hz))
          {
            SPD_Clear(oSpeedSensor[M1]);
            HFI_FP_SPD_SetElAngle((CHFI_FP_SPD)oSpeedSensor[M1],SPD_GetElAngle((CSPD)oSpeedSensorAux_M1));          
            HFI_FP_SPD_SetElSpeedDpp((CHFI_FP_SPD)oSpeedSensor[M1],-SPD_GetElSpeedDpp((CSPD)oSpeedSensorAux_M1));            
            HFI_FP_SPD_SetAvrgMecSpeed01Hz((CHFI_FP_SPD)oSpeedSensor[M1],-SPD_GetAvrgMecSpeed01Hz((CSPD)oSpeedSensorAux_M1));
            HFI_FP_SPD_SetHFState((CHFI_FP_SPD)oSpeedSensor[M1],TRUE);
            HFI_FP_STMRUN(oHFI[M1]);
            STC_SetSpeedSensor(oSTC[M1],oSpeedSensor[M1]);
          }
        }
      }
      else
      {
        HFI_FP_DisHFGeneration(oHFI[M1]);
      }   
    }
#else
    
    if(!IsSpeedReliable)
    {
#ifdef SPEED_FEEDBACK_CHECK
      STM_FaultProcessing(oSTM[M1], MC_SPEED_FDBK, 0);
#endif
    }
    
#endif
    
#if defined(ONE_TOUCH_TUNING)
    OTT_MF(oOTT[M1]);
#endif
    
    break;
    
  case ANY_STOP:
    
#if defined(MOTOR_PROFILER)
    SCC_Stop(oSCC[M1]);
    OTT_Stop(oOTT[M1]);
#endif
      
    PWMC_SwitchOffPWM(oCurrSensor[M1]);    
    FOC_Clear(M1);
    MPM_Clear(oMPM[M1]);
    TSK_SetStopPermanencyTimeM1(STOPPERMANENCY_TICKS);
    STM_NextState(oSTM[M1], STOP);
    break;
  case STOP:
    if(TSK_StopPermanencyTimeHasElapsedM1())
    {
      STM_NextState(oSTM[M1], STOP_IDLE);
    }
    break;
  case STOP_IDLE:
#if (defined NO_SPEED_SENSORS)    
    STC_SetSpeedSensor(oSTC[M1],oVSS_M1);  	/*  sensor-less */
    SPD_Clear(oVSS_M1); /* Reset measured speed in IDLE */
#endif
    
#if (defined HFINJECTION)    
    HFI_FP_DisHFGeneration(oHFI[M1]);
#endif    
    
#if (INRUSH_CURRLIMIT_ENABLING == ENABLE)
    STM_NextState(oSTM[M1], ICLWAIT);
#else
    STM_NextState(oSTM[M1], IDLE);
#endif
    break;
  default:
    break;    
  }
  
#if defined(MOTOR_PROFILER)
    SCC_MF(oSCC[M1]);
#endif

}

/**
  * @brief  It re-initializes the current and voltage variables. Moreover
  *         it clears qd currents PI controllers, voltage sensor and SpeednTorque 
  *         controller. It must be called before each motor restart. 
  *         It does not clear speed sensor. 
  * @param  bMotor related motor it can be M1 or M2
  * @retval none
  */
void FOC_Clear(uint8_t bMotor)
{
  Curr_Components Inull = {(int16_t)0, (int16_t)0};
  Volt_Components Vnull = {(int16_t)0, (int16_t)0};
  
  FOCVars[bMotor].Iab = Inull;
  FOCVars[bMotor].Ialphabeta = Inull;
  FOCVars[bMotor].Iqd = Inull;
  FOCVars[bMotor].Iqdref = Inull;
  FOCVars[bMotor].hTeref = (int16_t)0;
  FOCVars[bMotor].Vqd = Vnull;
  FOCVars[bMotor].Valphabeta = Vnull;   
  FOCVars[bMotor].hElAngle = (int16_t)0;
  
  PI_SetIntegralTerm(oPIDIq[bMotor], (int32_t)0);
  PI_SetIntegralTerm(oPIDId[bMotor], (int32_t)0);
  
  STC_Clear(oSTC[bMotor]);
  
  PWMC_SwitchOffPWM(oCurrSensor[bMotor]);
  
#if defined(FLUX_WEAKENING) || (defined(DUALDRIVE) && defined(FLUX_WEAKENING2))
  if (oFW[bMotor])
  {
    FW_Clear(oFW[bMotor]);
  }
#endif
  
#if defined(FEED_FORWARD_CURRENT_REGULATION) || (defined(DUALDRIVE) && defined(FEED_FORWARD_CURRENT_REGULATION2))
  if (oFF[bMotor])
  {
    FF_Clear(oFF[bMotor]);
  }
#endif
  
#if defined(HFINJECTION) || (defined(DUALDRIVE) && defined(HFINJECTION2))
  if (oHFI[bMotor])
  {
    HFI_FP_Clear(oHFI[bMotor]);
  }
#endif
}

/**
  * @brief  Use this method to initialize additional methods (if any) in 
  *         START_TO_RUN state
  * @param  bMotor related motor it can be M1 or M2 
  * @retval none
  */
void FOC_InitAdditionalMethods(uint8_t bMotor)
{
#if defined(FEED_FORWARD_CURRENT_REGULATION) || (defined(DUALDRIVE) && defined(FEED_FORWARD_CURRENT_REGULATION2))
  if (oFF[M1])
  {
    FF_InitFOCAdditionalMethods(oFF[M1]);
  }
#endif
}

/**
  * @brief  It computes the new values of Iqdref (current references on qd 
  *         reference frame) based on the required electrical torque information
  *         provided by oTSC object (internally clocked). 
  *         If implemented in the derived class it executes flux weakening and/or
  *         MTPA algorithm(s). It must be called with the periodicity specified  
  *         in oTSC parameters
  * @param  bMotor related motor it can be M1 or M2 
  * @retval none
  */
void FOC_CalcCurrRef(uint8_t bMotor)
{
  if(FOCVars[bMotor].bDriveInput == INTERNAL)
  {
    FOCVars[bMotor].hTeref = STC_CalcTorqueReference(oSTC[bMotor]);
    
    FOCVars[bMotor].Iqdref.qI_Component1 = FOCVars[bMotor].hTeref;
    
#if defined(IPMSM_MTPA) || (defined(DUALDRIVE) && defined(IPMSM_MTPA2))
    if (oMTPA[bMotor])
    {
      FOCVars[bMotor].Iqdref = MTPA_CalcCurrRef(oMTPA[bMotor], FOCVars[bMotor].Iqdref);
    }
#endif
    
#if defined(FLUX_WEAKENING) || (defined(DUALDRIVE) && defined(FLUX_WEAKENING2))
    if (oFW[bMotor])
    {
      FOCVars[bMotor].Iqdref = FW_CalcCurrRef(oFW[bMotor],FOCVars[bMotor].Iqdref);
    }
#endif
    
#if defined(FEED_FORWARD_CURRENT_REGULATION) || (defined(DUALDRIVE) && defined(FEED_FORWARD_CURRENT_REGULATION2))
    if (oFF[bMotor])
    {
      FF_VqdffComputation(oFF[bMotor], oSpeedSensor[bMotor], FOCVars[bMotor].Iqdref);
    }
#endif
  }
}

/**
  * @brief  It set a counter intended to be used for counting the delay required 
  *         for drivers boot capacitors charging of motor 1
  * @param  hTickCount number of ticks to be counted
  * @retval void
  */
void TSK_SetChargeBootCapDelayM1(uint16_t hTickCount)
{
   hBootCapDelayCounterM1 = hTickCount; 
}

/**
  * @brief  Use this function to know whether the time required to charge boot 
  *         capacitors of motor 1 has elapsed
  * @param  none
  * @retval bool TRUE if time has elapsed, FALSE otherwise
  */
bool TSK_ChargeBootCapDelayHasElapsedM1(void)
{
  bool retVal = FALSE;
  if (hBootCapDelayCounterM1 == 0)
  {
    retVal = TRUE;
  }
  return (retVal);
}

/**
  * @brief  It set a counter intended to be used for counting the permanency 
  *         time in STOP state of motor 1
  * @param  hTickCount number of ticks to be counted
  * @retval void
  */
void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount)
{
  hStopPermanencyCounterM1 = hTickCount;  
}

/**
  * @brief  Use this function to know whether the permanency time in STOP state 
  *         of motor 1 has elapsed
  * @param  none
  * @retval bool TRUE if time is elapsed, FALSE otherwise
  */
bool TSK_StopPermanencyTimeHasElapsedM1(void)
{
  bool retVal = FALSE;
  if (hStopPermanencyCounterM1 == 0)
  {
    retVal = TRUE;
  }
  return (retVal);
}

#ifdef DUALDRIVE
/**
  * @brief  It executes some of the control duties on Motor 2 accordingly with  
  *         the present state of its state machine. In particular, duties 
  *         requiring a specific timing (e.g. speed controller(s)) are here 
  *         executed
  * @param  None
  * @retval void
  */
void TSK_MediumFrequencyTaskM2(void)
{
  State_t StateM2;
  bool IsSpeedReliable;
  int16_t wAux = 0;
  
#if (INRUSH_CURRLIMIT_ENABLING2 == ENABLE)
  ICLState_t ICLState = ICL_Exec(oICL_M2);
#endif
  
#if (defined(VIEW_HALL_FEEDBACK2)||defined(VIEW_ENCODER_FEEDBACK2)||defined(AUX_STATE_OBSERVER_PLL2)||defined(AUX_STATE_OBSERVER_CORDIC2))
  bool IsSpeedReliableAux = SPD_CalcAvrgMecSpeed01Hz(oSpeedSensorAux_M2,&wAux);  /*  Only if Aux sensored is enabled */
#endif  
  
  IsSpeedReliable = SPD_CalcAvrgMecSpeed01Hz(oSpeedSensor[M2],&wAux);  
  
  MPM_CalcElMotorPower(oMPM[M2]);
  
  StateM2 = STM_GetState(oSTM[M2]);
  switch(StateM2)
  {
#if (INRUSH_CURRLIMIT_ENABLING2 == ENABLE)
  case ICLWAIT:
    if (ICLState == ICL_INACTIVE)
    {
      /* If ICL Inactive move to IDLE */
      STM_NextState(oSTM[M2],IDLE);
    }
    break;
#endif
    
  case IDLE_START:
#if (CHARGE_BOOT_CAP_ENABLING2 == ENABLE)
    PWMC_TurnOnLowSides(oCurrSensor[M2]); 
    TSK_SetChargeBootCapDelayM2(CHARGE_BOOT_CAP_TICKS2);
    STM_NextState(oSTM[M2],CHARGE_BOOT_CAP);
#else
    PWMC_CurrentReadingCalibr(oCurrSensor[M2],CRC_START);
    STM_NextState(oSTM[M2],OFFSET_CALIB);
#endif
    break;
    
#if (CHARGE_BOOT_CAP_ENABLING2 == ENABLE)
  case CHARGE_BOOT_CAP:
    if (TSK_ChargeBootCapDelayHasElapsedM2())
    {
      PWMC_CurrentReadingCalibr(oCurrSensor[M2],CRC_START);
      STM_NextState(oSTM[M2],OFFSET_CALIB);
    }
    break;
#endif
    
  case OFFSET_CALIB:
    if (PWMC_CurrentReadingCalibr(oCurrSensor[M2],CRC_EXEC))
    {
      STM_NextState(oSTM[M2],CLEAR);
    }
    break;
    
  case CLEAR:
#if (defined NO_SPEED_SENSORS2)      
    FOCVars[M2].bDriveInput = EXTERNAL;                               /*  only for sensorless*/
    STC_SetSpeedSensor(oSTC[M2],oVSS_M2);                             /*  only for sensorless */        
    RUC_Clear(oRUC_M2,MCI_GetImposedMotorDirection(oMCInterface[M2]));/*  only for sensorless */
    SWO_transitionStartM2 = FALSE;                                    /* only for sensorless */
#endif
    
#if (defined HFINJECTION2)
    STC_SetSpeedSensor(oSTC[M2],oSpeedSensor[M2]);
#endif
    
    SPD_Clear(oSpeedSensor[M2]); 		/*  only for sensorless/hall/encoder main*/
#if (defined(AUX_STATE_OBSERVER_PLL2) || defined(AUX_STATE_OBSERVER_CORDIC2) || defined(VIEW_HALL_FEEDBACK2) || defined(VIEW_ENCODER_FEEDBACK2))
    SPD_Clear(oSpeedSensorAux_M2); 		/*  only for sensorless/hall main/auxiliary*/
#endif
    if(STM_NextState(oSTM[M2], START) == TRUE)
    {
      FOC_Clear(M2);
      PWMC_SwitchOnPWM(oCurrSensor[M2]); 
    }
    break;
#if ((defined ENCODER2)||(defined VIEW_ENCODER_FEEDBACK2)) /*  only for encoder*/
  case IDLE_ALIGNMENT:
#if (CHARGE_BOOT_CAP_ENABLING2 == ENABLE)
    PWMC_TurnOnLowSides(oCurrSensor[M2]); 
    TSK_SetChargeBootCapDelayM2(CHARGE_BOOT_CAP_TICKS);
    STM_NextState(oSTM[M2],ALIGN_CHARGE_BOOT_CAP);
#else
    PWMC_CurrentReadingCalibr(oCurrSensor[M2],CRC_START);
    STM_NextState(oSTM[M2],ALIGN_OFFSET_CALIB);
#endif
    break;
    
#if (CHARGE_BOOT_CAP_ENABLING2 == ENABLE)
  case ALIGN_CHARGE_BOOT_CAP:
    if (TSK_ChargeBootCapDelayHasElapsedM2())
    {
      PWMC_CurrentReadingCalibr(oCurrSensor[M2],CRC_START);
      STM_NextState(oSTM[M2],ALIGN_OFFSET_CALIB);
    }
    break;
#endif
    
  case ALIGN_OFFSET_CALIB:
    if (PWMC_CurrentReadingCalibr(oCurrSensor[M2],CRC_EXEC))
    {
      STM_NextState(oSTM[M2],ALIGN_CLEAR);
    }
    break;
    
  case ALIGN_CLEAR:
    FOCVars[M2].bDriveInput = EXTERNAL;
    STC_SetSpeedSensor(oSTC[M2],oVSS_M2);;
    EAC_StartAlignment(oEAC_M2);
    
    if(STM_NextState(oSTM[M2], ALIGNMENT) == TRUE)
    {
      FOC_Clear(M2);
      PWMC_SwitchOnPWM(oCurrSensor[M2]); 
    }
    break;
#endif
    
  case START:
    {
#if (defined NO_SPEED_SENSORS2)  /* only for sensor-less control */
      
      int16_t hForcedMecSpeed01Hz;
      Curr_Components IqdRef;
      bool StartUpTransitionEnded;
      bool StartUpDoTransition;
      
      if(!RUC_Exec(oRUC_M2))
      {
        #if (OPEN_LOOP_FOC2 == ENABLE)
          /* No error generated when OPEN LOOP is enabled. */
        #else
          STM_FaultProcessing(oSTM[M2], MC_START_UP, 0);  /*Time allowed for startup has ended*/
        #endif
      }
      else
      {
        if (SWO_transitionStartM2 == FALSE)
        {
          IqdRef.qI_Component1 = STC_CalcTorqueReference(oSTC[M2]);
          IqdRef.qI_Component2 = FOCVars[M2].UserIdref;
          FOCVars[M2].Iqdref = IqdRef;
        }
      }
      
      StartUpTransitionEnded = SPD_CalcAvrgMecSpeed01Hz(oVSS_M2,&hForcedMecSpeed01Hz);
      
      #if (OPEN_LOOP_FOC2 == ENABLE)
      {
        int16_t hOLFinalMecSpeed01Hz = MCI_GetLastRampFinalSpeed(oMCInterface[M2]);
        if (hOLFinalMecSpeed01Hz != VSPD_GetLastRampFinalSpeed(oVSS_M2))
        {
          VSPD_SetMecAcceleration(oVSS_M2,hOLFinalMecSpeed01Hz,OPEN_LOOP_SPEED_RAMP_DURATION_MS2);
        }
        OL_Calc(oOL[M2]);
      }
      #endif
      
#if (defined(STATE_OBSERVER_PLL2))
      StartUpDoTransition = VSPD_SetStartTransition(oVSS_M2,STO_IsObserverConverged((CSTO_SPD)oSpeedSensor[M2],hForcedMecSpeed01Hz));
#elif (defined(STATE_OBSERVER_CORDIC2))
      StartUpDoTransition = VSPD_SetStartTransition(oVSS_M2,STO_CR_IsObserverConverged((CSTO_CR_SPD)oSpeedSensor[M2],hForcedMecSpeed01Hz));
#endif
      
      if (VSPD_IsTransitionOngoing(oVSS_M2))
      {
        if (SWO_transitionStartM2 == FALSE)
        {
          int16_t Iq = 0;
          Curr_Components StatorCurrent = MCM_Park(FOCVars[M2].Ialphabeta, SPD_GetElAngle(oSpeedSensor[M2]));
          Iq = StatorCurrent.qI_Component1;
          
          REMNG_Init(oREMNG[M2]);
          REMNG_ExecRamp(oREMNG[M2], FOCVars[M2].Iqdref.qI_Component1, 0);
          REMNG_ExecRamp(oREMNG[M2], Iq, TRANSITION_DURATION2);
          
          SWO_transitionStartM2 = TRUE;
        }
      }
      else
      {
        if (SWO_transitionStartM2 == TRUE)
        {
          SWO_transitionStartM2 = FALSE;
        }
      }
      
      if (StartUpDoTransition == FALSE)
      {
        StartUpTransitionEnded = TRUE;
      }      
      
      if (StartUpTransitionEnded == TRUE)
      {
        {      
          PI_SetIntegralTerm(oPIDSpeed[M2],(int32_t)FOCVars[M2].Iqdref.qI_Component1*PI_GetKIDivisor(oPIDSpeed[M2]));
        }        
        STM_NextState(oSTM[M2], START_RUN);
      }
#endif
      
#if ((defined ENCODER2)||(defined HALL_SENSORS2))
      STM_NextState(oSTM[M2], START_RUN); /* only for sensored*/
#endif
      
#if (defined HFINJECTION2)
      /* HFI validation check */
      if (HFI_FP_Start(oHFI[M2]) == TRUE)
      {
        STM_NextState(oSTM[M2], START_RUN);
      }
#endif
    }
    break;
#if ((defined ENCODER2)||(defined VIEW_ENCODER_FEEDBACK2)) /*  only for encoder*/
  case ALIGNMENT:
    if(!EAC_Exec(oEAC_M2))
    {
      Curr_Components IqdRef;
      IqdRef.qI_Component1 = 0;
      IqdRef.qI_Component2 = STC_CalcTorqueReference(oSTC[M2]);
      FOCVars[M2].Iqdref = IqdRef;	
    }
    else
    {
      PWMC_SwitchOffPWM(oCurrSensor[M2]);
      STC_SetControlMode(oSTC[M2], STC_SPEED_MODE);
      STC_SetSpeedSensor(oSTC[M2], oSpeedSensor[M2]);
      STM_NextState(oSTM[M2], ANY_STOP);
    }    
    break;    
#endif    
  case START_RUN:
#if (defined NO_SPEED_SENSORS2) /* only for sensor-less control */
    STC_SetSpeedSensor(oSTC[M2],oSpeedSensor[M2]);  /*Observer has converged*/
#endif
    
#if (defined HFINJECTION2)
    if (!HFI_FP_STMRUN(oHFI[M2]))
    {
      STM_FaultProcessing(oSTM[M2], MC_SPEED_FDBK, 0);
    }
    else
#endif
    {
      FOC_InitAdditionalMethods(M2);    
      FOC_CalcCurrRef(M2);
      STM_NextState(oSTM[M2], RUN);
    }
    
    STC_ForceSpeedReferenceToCurrentSpeed(oSTC[M2]); /* Init the reference speed to current speed */
    MCI_ExecBufferedCommands(oMCInterface[M2]); /* Exec the speed ramp after changing of the speed sensor */
    
    break;
  case RUN:
    MCI_ExecBufferedCommands(oMCInterface[M2]);
    FOC_CalcCurrRef(M2);
#if (defined HFINJECTION2)
    if (STC_GetSpeedSensor(oSTC[M2]) == oSpeedSensor[M2])
    {
      int16_t hObsSpeed01Hz = SPD_GetAvrgMecSpeed01Hz((CSPD)oSpeedSensorAux_M2);
      int16_t hHFISpeedDpp = SPD_GetElSpeedDpp(oSpeedSensor[M2]);
      int32_t wtemp = (int32_t)hHFISpeedDpp * (int32_t)hObsSpeed01Hz;
      
      if(!IsSpeedReliable)
      {
        STM_FaultProcessing(oSTM[M2], MC_SPEED_FDBK, 0);
      } 
      
      if (wtemp < 0)
      {
        STO_SetPLL((CSTO_SPD)oSpeedSensorAux_M2, hHFISpeedDpp,
        SPD_GetElAngle(oSpeedSensor[M2]));      
      }      
      
      if (!HFI_FP_SPD_AccelerationStageReached((CHFI_FP_SPD)oSpeedSensor[M2]))
      {
        STO_SetPLL((CSTO_SPD)oSpeedSensorAux_M2, SPD_GetElSpeedDpp(oSpeedSensor[M2]),
                   SPD_GetElAngle(oSpeedSensor[M2]));        
      } 
      else
      {
        if (STO_IsObserverConverged((CSTO_SPD)oSpeedSensorAux_M2,SPD_GetAvrgMecSpeed01Hz(oSpeedSensor[M2])))
        {
          STC_SetSpeedSensor(oSTC[M2], oSpeedSensorAux_M2);
          HFI_FP_DisHFGeneration(oHFI[M2]);           
        }
      }
    }
    else  /* STO - HFI */
    {
      int16_t hObsSpeed01Hz = SPD_GetAvrgMecSpeed01Hz((CSPD)oSpeedSensorAux_M2);
      
      if(!IsSpeedReliableAux)
      {
        STM_FaultProcessing(oSTM[M2], MC_SPEED_FDBK, 0);
      }
      
      if (HFI_FP_SPD_Restart((CHFI_FP_SPD)oSpeedSensor[M2],hObsSpeed01Hz))
      {
        if (HFI_FP_Restart(oHFI[M2]) == TRUE)
        {
          if (HFI_FP_SPD_IsConverged((CHFI_FP_SPD)oSpeedSensor[M2],hObsSpeed01Hz))
          {
            SPD_Clear(oSpeedSensor[M2]);
            HFI_FP_SPD_SetElAngle((CHFI_FP_SPD)oSpeedSensor[M2],SPD_GetElAngle((CSPD)oSpeedSensorAux_M2));
            HFI_FP_SPD_SetElSpeedDpp((CHFI_FP_SPD)oSpeedSensor[M2],-SPD_GetElSpeedDpp((CSPD)oSpeedSensorAux_M2));
            HFI_FP_SPD_SetAvrgMecSpeed01Hz((CHFI_FP_SPD)oSpeedSensor[M2],-SPD_GetAvrgMecSpeed01Hz((CSPD)oSpeedSensorAux_M2));
            HFI_FP_SPD_SetHFState((CHFI_FP_SPD)oSpeedSensor[M2],TRUE);
            HFI_FP_STMRUN(oHFI[M2]);
            STC_SetSpeedSensor(oSTC[M2],oSpeedSensor[M2]);
          }
        }
      }
      else
      {
        HFI_FP_DisHFGeneration(oHFI[M2]);
      }   
    }
#else
    
    if(!IsSpeedReliable)
    {
#ifdef SPEED_FEEDBACK_CHECK2
      STM_FaultProcessing(oSTM[M2], MC_SPEED_FDBK, 0);
#endif
    }
    
#endif
    break;
  case ANY_STOP:
    PWMC_SwitchOffPWM(oCurrSensor[M2]); 
    FOC_Clear(M2);
    MPM_Clear(oMPM[M2]);
    TSK_SetStopPermanencyTimeM2(40);
    STM_NextState(oSTM[M2], STOP);
    break;
  case STOP:
    if(TSK_StopPermanencyTimeHasElapsedM2())
    {
      STM_NextState(oSTM[M2], STOP_IDLE);
    }
    break;
  case STOP_IDLE:
#if (defined NO_SPEED_SENSORS2)    
    STC_SetSpeedSensor(oSTC[M2],oVSS_M2);  	/*  only for sensor-less */
    SPD_Clear(oVSS_M2); /* Reset measured speed in IDLE */
#endif
    
#if (defined HFINJECTION2)
    HFI_FP_DisHFGeneration(oHFI[M2]);
#endif
    
#if (INRUSH_CURRLIMIT_ENABLING2 == ENABLE)
    STM_NextState(oSTM[M2], ICLWAIT);
#else
    STM_NextState(oSTM[M2], IDLE);
#endif
    break;
  default:
    break;    
  }
}

/**
  * @brief  It set a counter intended to be used for counting the delay required 
  *         for drivers boot capacitors charging of motor 2
  * @param  hTickCount number of ticks to be counted
  * @retval void
  */
void TSK_SetChargeBootCapDelayM2(uint16_t hTickCount)
{
   hBootCapDelayCounterM2 = hTickCount;    
}

/**
  * @brief  Use this function to know whether the time required to charge boot 
  *         capacitors of motor 2 has elapsed
  * @param  none
  * @retval bool TRUE if time has elapsed, FALSE otherwise
  */
bool TSK_ChargeBootCapDelayHasElapsedM2(void)
{
  bool retVal = FALSE;
  if (hBootCapDelayCounterM2 == 0)
  {
    retVal = TRUE;
  }
  return (retVal);
}

/**
  * @brief  It set a counter intended to be used for counting the permanency 
  *         time in STOP state of motor 2
  * @param  hTickCount number of ticks to be counted
  * @retval void
  */
void TSK_SetStopPermanencyTimeM2(uint16_t hTickCount)
{
  hStopPermanencyCounterM2 = hTickCount;  
}

/**
  * @brief  Use this function to know whether the permanency time in STOP state 
  *         of motor 2 has elapsed
  * @param  none
  * @retval bool TRUE if time is elapsed, FALSE otherwise
  */
bool TSK_StopPermanencyTimeHasElapsedM2(void)
{
  bool retVal = FALSE;
  if (hStopPermanencyCounterM2 == 0)
  {
    retVal = TRUE;
  }
  return (retVal);
}
#endif

#if !defined(MOTOR_PROFILER)
#if defined (CCMRAM_ENABLED)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif
/**
  * @brief  Accordingly with the present state(s) of the state machine(s), it 
  *         executes those motor control duties requiring a high frequency rate
  *         and a precise timing (e.g. FOC current control loop)
  * @param  None
  * @retval uint8_t It return the motor instance number of last executed FOC.
  */
uint8_t TSK_HighFrequencyTask(void)
{ 
  uint8_t bMotorNbr = 0;
  uint16_t hFOCreturn;
  
#ifdef SINGLEDRIVE

#if (defined NO_SPEED_SENSORS)
  uint16_t hState;  /*  only if sensorless main*/
#endif
#if ((defined STATE_OBSERVER_PLL)||(defined AUX_STATE_OBSERVER_PLL))
  Observer_Inputs_t STO_Inputs; /*  only if sensorless main/aux*/
#endif
#if (defined STATE_OBSERVER_PLL)
  bool IsAccelerationStageReached;
#endif
#if ((defined STATE_OBSERVER_CORDIC)||(defined AUX_STATE_OBSERVER_CORDIC))
  STO_CR_Observer_Inputs_t STO_CR_Inputs; /*  only if sensorless main/aux*/
#endif
  
#if ((defined ENCODER)||(defined HALL_SENSORS)||(defined HFINJECTION))
  SPD_CalcAngle(oSpeedSensor[M1],MC_NULL);   /* if not sensorless then 2nd parameter is MC_NULL*/    
#endif
#if ((defined VIEW_ENCODER_FEEDBACK)||(defined VIEW_HALL_FEEDBACK))    
  SPD_CalcAngle(oSpeedSensorAux_M1,MC_NULL);  /*  only if auxiliary sensor is enabled */  
#endif

#if ((defined STATE_OBSERVER_PLL)||(defined AUX_STATE_OBSERVER_PLL))
  STO_Inputs.Valfa_beta = FOCVars[M1].Valphabeta;  /* only if sensorless*/
#endif
#if ((defined STATE_OBSERVER_CORDIC)||(defined AUX_STATE_OBSERVER_CORDIC))
  STO_CR_Inputs.Valfa_beta = FOCVars[M1].Valphabeta;  /* only if sensorless*/
#endif  
  
#if (defined NO_SPEED_SENSORS)
  if (SWO_transitionStartM1 == TRUE)
  {
    if (!REMNG_RampCompleted(oREMNG[M1]))
    {
      FOCVars[M1].Iqdref.qI_Component1 = REMNG_Calc(oREMNG[M1]);
    }
  }
#endif
  
#if (defined(OTF_STARTUP))
  if(!RUC_Get_SCLowsideOTF_Status(oRUC_M1))
  {
    hFOCreturn = FOC_CurrController(M1);
  }
  else
  {
    hFOCreturn = MC_NO_ERROR;
  }
#else
  hFOCreturn = FOC_CurrController(M1);
#endif

  if(hFOCreturn == MC_FOC_DURATION)
  {
    STM_FaultProcessing(oSTM[M1], MC_FOC_DURATION, 0); 
  }
  else
  {
#if ((defined STATE_OBSERVER_PLL)||(defined AUX_STATE_OBSERVER_PLL))
    STO_Inputs.Ialfa_beta = FOCVars[M1].Ialphabeta; /*  only if sensorless*/
    
    STO_Inputs.Vbus = VBS_GetAvBusVoltage_d(oBusSensor[M1]); /*  only for sensorless*/
#endif    
#if ((defined STATE_OBSERVER_CORDIC)||(defined AUX_STATE_OBSERVER_CORDIC))
    STO_CR_Inputs.Ialfa_beta = FOCVars[M1].Ialphabeta; /*  only if sensorless*/
    
    STO_CR_Inputs.Vbus = VBS_GetAvBusVoltage_d(oBusSensor[M1]); /*  only for sensorless*/    
#endif    
#if (defined STATE_OBSERVER_PLL)
    IsAccelerationStageReached = RUC_FirstAccelerationStageReached(oRUC_M1);
    
    SPD_CalcAngle(oSpeedSensor[M1],&STO_Inputs);   /* if not sensorless then 2nd parameter is MC_NULL*/

    STO_CalcAvrgElSpeedDpp((CSTO_SPD) oSpeedSensor[M1]); /*  Only in case of Sensor-less */    
    
    if (IsAccelerationStageReached == FALSE)
    {
      STO_ResetPLL((CSTO_SPD)oSpeedSensor[M1]);
    }   
    
    hState = STM_GetState(oSTM[M1]);
    if((hState == START) || (hState == START_RUN)) /*  only for sensor-less*/
    {
      int16_t hObsAngle;
      hObsAngle = SPD_GetElAngle(oSpeedSensor[M1]);
      SPD_CalcAngle(oVSS_M1,&hObsAngle);
    }
#endif
#if (defined STATE_OBSERVER_CORDIC)
    SPD_CalcAngle(oSpeedSensor[M1],&STO_CR_Inputs);   /* if not sensorless then 2nd parameter is MC_NULL*/
    
    STO_CR_CalcAvrgElSpeedDpp((CSTO_CR_SPD) oSpeedSensor[M1]); /*  Only in case of Sensor-less */
    
    hState = STM_GetState(oSTM[M1]);
    if((hState == START) || (hState == START_RUN)) /*  only for sensor-less*/
    {
      int16_t hObsAngle;
      hObsAngle = SPD_GetElAngle(oSpeedSensor[M1]);      
      SPD_CalcAngle(oVSS_M1,&hObsAngle);  
    }
#endif    
#if (defined(AUX_STATE_OBSERVER_PLL))
    SPD_CalcAngle(oSpeedSensorAux_M1,&STO_Inputs);   /* if not sensorless then 2nd parameter is MC_NULL*/
    
    STO_CalcAvrgElSpeedDpp((CSTO_SPD) oSpeedSensorAux_M1); /*  Only in case of Sensor-less auxiliary*/
#endif
#if (defined(AUX_STATE_OBSERVER_CORDIC))
    SPD_CalcAngle(oSpeedSensorAux_M1,&STO_CR_Inputs);   /* if not sensorless then 2nd parameter is MC_NULL*/
    
    STO_CR_CalcAvrgElSpeedDpp((CSTO_CR_SPD) oSpeedSensorAux_M1); /*  Only in case of Sensor-less auxiliary*/
#endif        
  }
#else //DUALDRIVE

#if ((defined NO_SPEED_SENSORS)||(defined NO_SPEED_SENSORS2))
  uint16_t hState;  /*  only if, at least, one of the two motors is sensorless main*/
#endif
#if ((defined STATE_OBSERVER_PLL)||(defined AUX_STATE_OBSERVER_PLL)||(defined STATE_OBSERVER_PLL2)||(defined AUX_STATE_OBSERVER_PLL2))
  Observer_Inputs_t STO_Inputs; /*  only if sensorless main/aux*/
#endif
#if ((defined STATE_OBSERVER_PLL)||(defined STATE_OBSERVER_PLL2))
  bool IsAccelerationStageReached;
#endif
#if ((defined STATE_OBSERVER_CORDIC)||(defined AUX_STATE_OBSERVER_CORDIC)||(defined STATE_OBSERVER_CORDIC2)||(defined AUX_STATE_OBSERVER_CORDIC2))
  STO_CR_Observer_Inputs_t STO_CR_Inputs; /*  only if sensorless main/aux*/
#endif  
  
  bMotorNbr = *((uint8_t*)(FOC_array[FOC_array_head]));
  
  if (bMotorNbr == M1)
  {
#if ((defined STATE_OBSERVER_PLL)||(defined AUX_STATE_OBSERVER_PLL))
  STO_Inputs.Valfa_beta = FOCVars[bMotorNbr].Valphabeta;        /* only if motor0 is sensorless*/
#endif
#if ((defined STATE_OBSERVER_CORDIC)||(defined AUX_STATE_OBSERVER_CORDIC))
  STO_CR_Inputs.Valfa_beta = FOCVars[bMotorNbr].Valphabeta;     /* only if motor0 is sensorless*/
#endif
    
#if ((defined ENCODER)||(defined HALL_SENSORS))
    SPD_CalcAngle(oSpeedSensor[bMotorNbr],MC_NULL);   /* if not sensorless then 2nd parameter is MC_NULL*/
#endif
#if ((defined VIEW_ENCODER_FEEDBACK)||(defined VIEW_HALL_FEEDBACK))
    SPD_CalcAngle(oSpeedSensorAux_M1,MC_NULL);  /*  only if auxiliary sensor is enabled */
#endif
    
#if (defined NO_SPEED_SENSORS)
    if (SWO_transitionStartM1 == TRUE)
    {
      if (!REMNG_RampCompleted(oREMNG[M1]))
      {
        FOCVars[M1].Iqdref.qI_Component1 = REMNG_Calc(oREMNG[M1]);
      }
    }
#endif
    
#if (defined(OTF_STARTUP))
    if(!RUC_Get_SCLowsideOTF_Status(oRUC_M1))
    {
      hFOCreturn = FOC_CurrController(M1);
    }
    else
    {
      hFOCreturn = MC_NO_ERROR;
    } 
#else
    hFOCreturn = FOC_CurrController(M1);
#endif
    
  }
  else
  {
#if ((defined STATE_OBSERVER_PLL2)||(defined AUX_STATE_OBSERVER_PLL2))
    STO_Inputs.Valfa_beta = FOCVars[bMotorNbr].Valphabeta;      /* only if motor1 is sensorless*/
#endif
#if ((defined STATE_OBSERVER_CORDIC2)||(defined AUX_STATE_OBSERVER_CORDIC2))
    STO_CR_Inputs.Valfa_beta = FOCVars[bMotorNbr].Valphabeta;   /* only if motor1 is sensorless*/
#endif    
    
#if ((defined ENCODER2)||(defined HALL_SENSORS2))
    SPD_CalcAngle(oSpeedSensor[bMotorNbr],MC_NULL);   /* if not sensorless then 2nd parameter is MC_NULL*/
#endif    
#if ((defined VIEW_ENCODER_FEEDBACK2)||(defined VIEW_HALL_FEEDBACK2))
    SPD_CalcAngle(oSpeedSensorAux_M2,MC_NULL);  /*  only if auxiliary sensor is enabled */
#endif
    
#if (defined NO_SPEED_SENSORS2)
    if (SWO_transitionStartM2 == TRUE)
    {
      if (!REMNG_RampCompleted(oREMNG[M2]))
      {
        FOCVars[M2].Iqdref.qI_Component1 = REMNG_Calc(oREMNG[M2]);
      }
    }
#endif
    
#if (defined(OTF_STARTUP2))
    if(!RUC_Get_SCLowsideOTF_Status(oRUC_M2))
    {
      hFOCreturn = FOC_CurrController(M2);
    }
    else
    {
      hFOCreturn = MC_NO_ERROR;
    } 
#else
    hFOCreturn = FOC_CurrController(M2);
#endif
  }
  
  if(hFOCreturn == MC_FOC_DURATION)
  {
    STM_FaultProcessing(oSTM[bMotorNbr], MC_FOC_DURATION, 0); 
  }
  else
  {
    if (bMotorNbr == 0)
    {
#if ((defined STATE_OBSERVER_PLL)||(defined AUX_STATE_OBSERVER_PLL))
      STO_Inputs.Ialfa_beta = FOCVars[bMotorNbr].Ialphabeta;  /* only if motor0 is sensorless*/  
      
      STO_Inputs.Vbus = VBS_GetAvBusVoltage_d(oBusSensor[bMotorNbr]); /*  only for sensorless*/
#endif
#if ((defined STATE_OBSERVER_CORDIC)||(defined AUX_STATE_OBSERVER_CORDIC))
      STO_CR_Inputs.Ialfa_beta = FOCVars[bMotorNbr].Ialphabeta;  /* only if motor0 is sensorless*/  
      
      STO_CR_Inputs.Vbus = VBS_GetAvBusVoltage_d(oBusSensor[bMotorNbr]); /*  only for sensorless*/
#endif      
#if (defined STATE_OBSERVER_PLL)
      
      IsAccelerationStageReached = RUC_FirstAccelerationStageReached(oRUC_M1);
      
      SPD_CalcAngle(oSpeedSensor[bMotorNbr],&STO_Inputs);   /* if not sensorless then 2nd parameter is MC_NULL*/
      
      STO_CalcAvrgElSpeedDpp((CSTO_SPD) oSpeedSensor[bMotorNbr]); /*  Only in case of Sensor-less */
      
      if (IsAccelerationStageReached == FALSE)
      {
        STO_ResetPLL((CSTO_SPD)oSpeedSensor[bMotorNbr]);
      }    
      
      hState = STM_GetState(oSTM[bMotorNbr]);
      if((hState == START) || (hState == START_RUN)) /*  only for sensor-less*/
      {
        int16_t hObsAngle;
        hObsAngle = SPD_GetElAngle(oSpeedSensor[bMotorNbr]);        
        SPD_CalcAngle(oVSS_M1,&hObsAngle);  
      }
#endif
#if (defined STATE_OBSERVER_CORDIC)
      SPD_CalcAngle(oSpeedSensor[bMotorNbr],&STO_CR_Inputs);   /* if not sensorless then 2nd parameter is MC_NULL*/
      
      STO_CR_CalcAvrgElSpeedDpp((CSTO_CR_SPD) oSpeedSensor[bMotorNbr]); /*  Only in case of Sensor-less */
      
      hState = STM_GetState(oSTM[bMotorNbr]);
      if((hState == START) || (hState == START_RUN)) /*  only for sensor-less*/
      {
        int16_t hObsAngle;
        hObsAngle = SPD_GetElAngle(oSpeedSensor[bMotorNbr]);        
        SPD_CalcAngle(oVSS_M1,&hObsAngle);  
      }
#endif      
#if (defined(AUX_STATE_OBSERVER_PLL))
      SPD_CalcAngle(oSpeedSensorAux_M1,&STO_Inputs);   /* if not sensorless then 2nd parameter is MC_NULL*/
      
      STO_CalcAvrgElSpeedDpp((CSTO_SPD) oSpeedSensorAux_M1); /*  Only in case of Sensor-less */
#endif
#if (defined(AUX_STATE_OBSERVER_CORDIC))
      SPD_CalcAngle(oSpeedSensorAux_M1,&STO_CR_Inputs);   /* if not sensorless then 2nd parameter is MC_NULL*/
      
      STO_CR_CalcAvrgElSpeedDpp((CSTO_CR_SPD) oSpeedSensorAux_M1); /*  Only in case of Sensor-less */
#endif       
    }
    else
    {
#if ((defined STATE_OBSERVER_PLL2)||(defined AUX_STATE_OBSERVER_PLL2))
      STO_Inputs.Ialfa_beta = FOCVars[bMotorNbr].Ialphabeta;  /* only if motor1 is sensorless*/  
      
      STO_Inputs.Vbus = VBS_GetAvBusVoltage_d(oBusSensor[bMotorNbr]); /*  only for sensorless*/
#endif
#if ((defined STATE_OBSERVER_CORDIC2)||(defined AUX_STATE_OBSERVER_CORDIC2))
      STO_CR_Inputs.Ialfa_beta = FOCVars[bMotorNbr].Ialphabeta;  /* only if motor1 is sensorless*/  
      
      STO_CR_Inputs.Vbus = VBS_GetAvBusVoltage_d(oBusSensor[bMotorNbr]); /*  only for sensorless*/
#endif      
#if (defined STATE_OBSERVER_PLL2)
      
      IsAccelerationStageReached = RUC_FirstAccelerationStageReached(oRUC_M2);
      
      SPD_CalcAngle(oSpeedSensor[bMotorNbr],&STO_Inputs);   /* if not sensorless then 2nd parameter is MC_NULL*/
      
      STO_CalcAvrgElSpeedDpp((CSTO_SPD) oSpeedSensor[bMotorNbr]); /*  Only in case of Sensor-less */
      
      if (IsAccelerationStageReached == FALSE)
      {
        STO_ResetPLL((CSTO_SPD)oSpeedSensor[bMotorNbr]);
      }       
      
      hState = STM_GetState(oSTM[bMotorNbr]);
      if((hState == START) || (hState == START_RUN)) /*  only for sensor-less*/
      {
        int16_t hObsAngle;
        hObsAngle = SPD_GetElAngle(oSpeedSensor[bMotorNbr]);
        SPD_CalcAngle(oVSS_M2,&hObsAngle);
      }      
#endif
#if (defined STATE_OBSERVER_CORDIC2)
      SPD_CalcAngle(oSpeedSensor[bMotorNbr],&STO_CR_Inputs);   /* if not sensorless then 2nd parameter is MC_NULL*/
      
      STO_CR_CalcAvrgElSpeedDpp((CSTO_CR_SPD) oSpeedSensor[bMotorNbr]); /*  Only in case of Sensor-less */
      
      hState = STM_GetState(oSTM[bMotorNbr]);
      if((hState == START) || (hState == START_RUN)) /*  only for sensor-less*/
      {
        int16_t hObsAngle;
        hObsAngle = SPD_GetElAngle(oSpeedSensor[bMotorNbr]);        
        SPD_CalcAngle(oVSS_M2,&hObsAngle);
      }      
#endif      
#if (defined(AUX_STATE_OBSERVER_PLL2))
      SPD_CalcAngle(oSpeedSensorAux_M2,&STO_Inputs);   /* if not sensorless then 2nd parameter is MC_NULL*/
      
      STO_CalcAvrgElSpeedDpp((CSTO_SPD) oSpeedSensorAux_M2); /*  Only in case of Sensor-less */
#endif
#if (defined(AUX_STATE_OBSERVER_CORDIC2))
      SPD_CalcAngle(oSpeedSensorAux_M2,&STO_CR_Inputs);   /* if not sensorless then 2nd parameter is MC_NULL*/
      
      STO_CR_CalcAvrgElSpeedDpp((CSTO_CR_SPD) oSpeedSensorAux_M2); /*  Only in case of Sensor-less */
#endif      
    }    
  }
  
  FOC_array_head++;
  if (FOC_array_head == FOC_ARRAY_LENGHT)
  {
    FOC_array_head = 0;
  }
  
#endif
  
  return bMotorNbr;
}
#endif

#if defined(MOTOR_PROFILER)
#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif
/**
  * @brief  Motor control profiler HF task
  * @param  None
  * @retval uint8_t It return always 0.
  */
uint8_t TSK_HighFrequencyTask(void)
{
  Curr_Components Iab;
  
  if (SWO_transitionStartM1 == TRUE)
  {
    if (!REMNG_RampCompleted(oREMNG[M1]))
    {
      FOCVars[M1].Iqdref.qI_Component1 = REMNG_Calc(oREMNG[M1]);
    }
  }
  
  PWMC_GetPhaseCurrents(oCurrSensor[M1], &Iab);
  FOCVars[M1].Iab = Iab;
  SCC_SetPhaseVoltage(oSCC[M1]);
  
  return 0; /* Single motor only */
}
#endif

#if !defined(MOTOR_PROFILER)
#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif
/**
  * @brief It executes the core of FOC drive that is the controllers for Iqd 
  *        currents regulation. Reference frame transformations are carried out 
  *        accordingly to the active speed sensor. It must be called periodically
  *        when new motor currents have been converted
  * @param this related object of class CFOC.
  * @retval int16_t It returns MC_NO_FAULTS if the FOC has been ended before 
  *         next PWM Update event, MC_FOC_DURATION otherwise
  */
#pragma inline
uint16_t FOC_CurrController(uint8_t bMotor)
{
  Curr_Components Iab, Ialphabeta, Iqd;
#if defined(HFINJECTION) || (defined(DUALDRIVE) && defined(HFINJECTION2))
  Curr_Components IqdHF = {0,0};
#endif
  Volt_Components Valphabeta, Vqd;
  int16_t hElAngledpp;
  uint16_t hCodeError;
  
  hElAngledpp = SPD_GetElAngle(STC_GetSpeedSensor(oSTC[bMotor]));
  
  PWMC_GetPhaseCurrents(oCurrSensor[bMotor], &Iab);
  
  Ialphabeta = MCM_Clarke(Iab);
  
  Iqd = MCM_Park(Ialphabeta, hElAngledpp);
  
#if defined(HFINJECTION) || (defined(DUALDRIVE) && defined(HFINJECTION2))
  if (oHFI[bMotor])
  {
    IqdHF = Iqd; /* Stores the Iqd not filtered */
    Iqd = HFI_FP_PreProcessing(oHFI[bMotor], Iqd);
  }
#endif
  
  Vqd.qV_Component1 = PI_Controller(oPIDIq[bMotor], 
            (int32_t)(FOCVars[bMotor].Iqdref.qI_Component1) - Iqd.qI_Component1);
  
  Vqd.qV_Component2 = PI_Controller(oPIDId[bMotor], 
            (int32_t)(FOCVars[bMotor].Iqdref.qI_Component2) - Iqd.qI_Component2);  
  
#if defined(FEED_FORWARD_CURRENT_REGULATION) || (defined(DUALDRIVE) && defined(FEED_FORWARD_CURRENT_REGULATION2))
  if (oFF[bMotor])
  {
    Vqd = FF_VqdConditioning(oFF[bMotor],Vqd);
  }
#endif
  
  FOCVars[bMotor].Vqd = Vqd;
  
#if defined(HFINJECTION) || (defined(DUALDRIVE) && defined(HFINJECTION2))
  if (oHFI[bMotor])
  {
    Vqd = HFI_FP_VqdConditioning(oHFI[bMotor], Vqd);
  }
#endif
  
#if defined(OPEN_LOOP) || (defined(DUALDRIVE) && defined(OPEN_LOOP2))
  if (oOL[bMotor])
  {
    Vqd = OL_VqdConditioning(oOL[bMotor]);
  }
#endif
  
  Vqd = Circle_Limitation(oCLM[bMotor], Vqd);
  
  Valphabeta = MCM_Rev_Park(Vqd, hElAngledpp);
  
  hCodeError = PWMC_SetPhaseVoltage(oCurrSensor[bMotor], Valphabeta);
  
  FOCVars[bMotor].Iab = Iab;
  FOCVars[bMotor].Ialphabeta = Ialphabeta;  
  FOCVars[bMotor].Iqd = Iqd;    
  FOCVars[bMotor].Valphabeta = Valphabeta;
  FOCVars[bMotor].hElAngle = hElAngledpp;

#if defined(HFINJECTION) || (defined(DUALDRIVE) && defined(HFINJECTION2))
  FOCVars[bMotor].IqdHF = IqdHF;
#endif
  
#if defined(FLUX_WEAKENING) || (defined(DUALDRIVE) && defined(FLUX_WEAKENING2))
  if (oFW[bMotor])
  {
    FW_DataProcess(oFW[bMotor], Vqd);
  }
#endif
  
#if defined(FEED_FORWARD_CURRENT_REGULATION) || (defined(DUALDRIVE) && defined(FEED_FORWARD_CURRENT_REGULATION2))
  if (oFF[bMotor])
  {
    FF_DataProcess(oFF[bMotor]);
  }
#endif
  
#if defined(HFINJECTION) || (defined(DUALDRIVE) && defined(HFINJECTION2))
  if (oHFI[bMotor])
  {
    HFI_FP_DataProcess(oHFI[bMotor], IqdHF);
  }
#endif
  
  return(hCodeError);
}
#endif

/**
* @brief  This function requests a user-defined regular conversion. All user 
*         defined conversion requests must be performed inside routines with the 
*         same priority level. If previous regular conversion request is pending
*         this function has no effect, for this reason is better to call the 
*         MC_RegularConvState and check if the state is UDRC_STATE_IDLE before 
*         to call MC_RequestRegularConv.
* @param  bChannel ADC channel used for the regular conversion.
* @param  bSamplTime Sampling time selection, ADC_SampleTime_nCycles defined in 
*         stm32fxxx_adc.h see ADC_sampling_times.
*/
void MC_RequestRegularConv(uint8_t bChannel, uint8_t bSamplTime)
{
  ADConv_t ADConv_struct;
  if (UDC_State == UDRC_STATE_IDLE)
  {
    ADConv_struct.Channel = bChannel;
    ADConv_struct.SamplTime = bSamplTime;
    PWMC_ADC_SetSamplingTime(oCurrSensor[M1],ADConv_struct);
    UDC_State = UDRC_STATE_REQUESTED;
    UDC_Channel = bChannel;
  }
}

/**
* @brief  Get the last user-defined regular conversion.
* @retval uint16_t It returns converted value or oxFFFF for conversion error.
*         This function returns a valid result if the state returned by 
*         MC_RegularConvState is UDRC_STATE_EOC.
*/
uint16_t MC_GetRegularConv(void)
{
  uint16_t hRetVal = 0xFFFFu;
  if (UDC_State == UDRC_STATE_EOC)
  {
    hRetVal = UDC_ConvertedValue;
    UDC_State = UDRC_STATE_IDLE;
  }
  return hRetVal;
}

/**
* @brief  Use this function to know the status of the last requested regular 
*         conversion.
* @retval UDRC_State_t The state of the last user-defined regular conversion. 
*         It can be one of the following values:
*         UDRC_STATE_IDLE no regular conversion request pending.
*         UDRC_STATE_REQUESTED regular conversion has been requested and not 
*         completed.
*         UDRC_STATE_EOC regular conversion has been completed but not readed 
*         from the user.
*/
UDRC_State_t MC_RegularConvState(void)
{
  return UDC_State;
}

/**
  * @brief  It executes safety checks (e.g. bus voltage and temperature) for all
  *         drive instances. Faults flags are also here updated     
  * @param  None
  * @retval None
  */
void TSK_SafetyTask(void)
{  
#if defined(SINGLEDRIVE)
  
    #if defined(MOTOR_PROFILER)
      SCC_CheckOC_RL(oSCC[M1]);
    #endif

    #if (ON_OVER_VOLTAGE == TURN_OFF_PWM)
        TSK_SafetyTask_PWMOFF(M1);
    #elif (ON_OVER_VOLTAGE == TURN_ON_R_BRAKE)
        TSK_SafetyTask_RBRK(M1);
    #elif (ON_OVER_VOLTAGE == TURN_ON_LOW_SIDES)
        TSK_SafetyTask_LSON(M1);
    #endif

#elif defined(DUALDRIVE)

    /* First drive */
    #if (ON_OVER_VOLTAGE == TURN_OFF_PWM)
        TSK_SafetyTask_PWMOFF(M1);
    #elif (ON_OVER_VOLTAGE == TURN_ON_R_BRAKE)
        TSK_SafetyTask_RBRK(M1);
    #elif (ON_OVER_VOLTAGE == TURN_ON_LOW_SIDES)
        TSK_SafetyTask_LSON(M1);
    #endif
    
    /* Second drive */
    #if (ON_OVER_VOLTAGE2 == TURN_OFF_PWM)
        TSK_SafetyTask_PWMOFF(M2);
    #elif (ON_OVER_VOLTAGE2 == TURN_ON_R_BRAKE)
        TSK_SafetyTask_RBRK(M2);
    #elif (ON_OVER_VOLTAGE2 == TURN_ON_LOW_SIDES)
        TSK_SafetyTask_LSON(M2);
    #endif

#endif

    /* User conversion execution */
  if (UDC_State == UDRC_STATE_REQUESTED)
  {
    UDC_ConvertedValue = PWMC_ExecRegularConv (oCurrSensor[M1], UDC_Channel);
    UDC_State = UDRC_STATE_EOC;
  }
}

/**
  * @brief  Safety task implementation if ON_OVER_VOLTAGE == TURN_OFF_PWM   
  * @param  bMotor Motor reference number defined 
  *         \link Motors_reference_number here \endlink
  * @retval None
  */
void TSK_SafetyTask_PWMOFF(uint8_t bMotor)
{
  uint16_t CodeReturn;

  CodeReturn = TSNS_CalcAvTemp(oTemperatureSensor[bMotor]); /* Clock temperature sensor and check for fault. It returns MC_OVER_TEMP or MC_NO_ERROR */
  CodeReturn |= PWMC_CheckOverCurrent(oCurrSensor[bMotor]); /* Clock current sensor and check for fault. It return MC_BREAK_IN or MC_NO_FAULTS (for STM32F30x can return MC_OVER_VOLT in case of HW Overvoltage) */
  CodeReturn |= VBS_CalcAvVbus(oBusSensor[bMotor]); /* Clock the bus voltage sensor and check for fault. It returns MC_OVER_VOLT or MC_UNDER_VOLT or MC_NO_ERROR */
  
  STM_FaultProcessing(oSTM[bMotor], CodeReturn, ~CodeReturn); /* Update the STM according error code */
  
  switch (STM_GetState(oSTM[bMotor])) /* Acts on PWM outputs in case of faults */
  {
  case FAULT_NOW:   
#if defined(MOTOR_PROFILER)
    SCC_Stop(oSCC[M1]);
    OTT_Stop(oOTT[M1]);
#endif
    PWMC_SwitchOffPWM(oCurrSensor[bMotor]);
    FOC_Clear(bMotor);
    MPM_Clear(oMPM[bMotor]);
    break;
  case FAULT_OVER:
#if defined(MOTOR_PROFILER)
    SCC_Stop(oSCC[M1]);
    OTT_Stop(oOTT[M1]);
#endif
    PWMC_SwitchOffPWM(oCurrSensor[bMotor]);
    break;
  default:
    break;    
  }

#ifdef SMOOTH_BRAKING_ACTION_ON_OVERVOLTAGE
  {
    /* Smooth braking action on overvoltage */
    uint16_t busd = (uint16_t)(VBS_GetAvBusVoltage_d(oBusSensor[bMotor]));
         
    if ((STM_GetState(oSTM[bMotor]) == IDLE)||
       ((STM_GetState(oSTM[bMotor]) == RUN)&&(FOCVars[bMotor].Iqdref.qI_Component1>0)))      
    {
      nominalBusd[bMotor] = busd;
    }
    else
    {
      if((STM_GetState(oSTM[bMotor]) == RUN)&&(FOCVars[bMotor].Iqdref.qI_Component1<0))
      {
        if (busd > ((ovthd[bMotor] + nominalBusd[bMotor]) >> 1))
        {
          FOCVars[bMotor].Iqdref.qI_Component1 = 0;
          FOCVars[bMotor].Iqdref.qI_Component2 = 0;
        }            
      }
    }
  }
#endif
}

/**
  * @brief  Safety task implementation if ON_OVER_VOLTAGE == TURN_ON_R_BRAKE   
  * @param  motor Motor reference number defined 
  *         \link Motors_reference_number here \endlink
  * @retval None
  */
void TSK_SafetyTask_RBRK(uint8_t bMotor)
{
  uint16_t CodeReturn;
  uint16_t BusVoltageFaultsFlag;
  
  /* Brake resistor management */
  BusVoltageFaultsFlag = VBS_CalcAvVbus(oBusSensor[bMotor]); /* Clock the bus voltage sensor and check for fault. It returns MC_OVER_VOLT or MC_UNDER_VOLT or MC_NO_ERROR */
  
  if (BusVoltageFaultsFlag == MC_OVER_VOLT)
  {
    DOUT_SetOutputState(oR_Brake[bMotor], ACTIVE); 
  }
  else
  {
    DOUT_SetOutputState(oR_Brake[bMotor], INACTIVE);    
  }
  
  CodeReturn = TSNS_CalcAvTemp(oTemperatureSensor[bMotor]); /* Clock temperature sensor and check for fault. It returns MC_OVER_TEMP or MC_NO_ERROR */
  CodeReturn |= PWMC_CheckOverCurrent(oCurrSensor[bMotor]); /* Clock current sensor and check for fault. It return MC_BREAK_IN or MC_NO_FAULTS (for STM32F30x can return MC_OVER_VOLT in case of HW Overvoltage) */
  CodeReturn |= (BusVoltageFaultsFlag & MC_UNDER_VOLT); /* MC_UNDER_VOLT generates fault, MC_OVER_VOLT doesn't generate fault */
    
  STM_FaultProcessing(oSTM[bMotor], CodeReturn, ~CodeReturn); /* Update the STM according error code */
  
  switch (STM_GetState(oSTM[bMotor]))
  {
  case FAULT_NOW:   
#if defined(MOTOR_PROFILER)
    SCC_Stop(oSCC[M1]);
    OTT_Stop(oOTT[M1]);
#endif
    PWMC_SwitchOffPWM(oCurrSensor[bMotor]);
    FOC_Clear(bMotor);
    MPM_Clear(oMPM[bMotor]);
    break;
  case FAULT_OVER:
#if defined(MOTOR_PROFILER)
    SCC_Stop(oSCC[M1]);
    OTT_Stop(oOTT[M1]);
#endif
    PWMC_SwitchOffPWM(oCurrSensor[bMotor]);
    break;
  default:
    break;    
  }
}

/**
  * @brief  Safety task implementation if ON_OVER_VOLTAGE == TURN_ON_LOW_SIDES   
  * @param  motor Motor reference number defined 
  *         \link Motors_reference_number here \endlink
  * @retval None
  */
void TSK_SafetyTask_LSON(uint8_t bMotor)
{
  uint16_t CodeReturn;
  uint16_t OverVoltageFlag;
  
  bool TurnOnLowSideAction = PWMC_GetTurnOnLowSidesAction(oCurrSensor[bMotor]);
  
  CodeReturn = TSNS_CalcAvTemp(oTemperatureSensor[bMotor]); /* Clock temperature sensor and check for fault. It returns MC_OVER_TEMP or MC_NO_ERROR */
  CodeReturn |= PWMC_CheckOverCurrent(oCurrSensor[bMotor]); /* Clock current sensor and check for fault. It return MC_BREAK_IN or MC_NO_FAULTS (for STM32F30x can return MC_OVER_VOLT in case of HW Overvoltage) */
  OverVoltageFlag = VBS_CalcAvVbus(oBusSensor[bMotor]);     /* Clock the bus voltage sensor and check for fault. It returns MC_OVER_VOLT or MC_UNDER_VOLT or MC_NO_ERROR */
  CodeReturn |= OverVoltageFlag;                       /* Over/Under voltage code errors generate faults. */
  STM_FaultProcessing(oSTM[bMotor], CodeReturn, ~CodeReturn); /* Update the STM according error code */
  
  if ((OverVoltageFlag == MC_OVER_VOLT) && (TurnOnLowSideAction == FALSE))
  {
    /* Start turn on low side action */
    PWMC_SwitchOffPWM(oCurrSensor[bMotor]); /* Required before PWMC_TurnOnLowSides */
#if (HW_OV_CURRENT_PROT_BYPASS == ENABLE)
    DOUT_SetOutputState(oOCPDisabling[bMotor], ACTIVE); /* Disable the OCP */
#endif     
    PWMC_TurnOnLowSides(oCurrSensor[bMotor]); /* Turn on Low side switches */
  }
  else
  {
    switch (STM_GetState(oSTM[bMotor])) /* Is state equal to FAULT_NOW or FAULT_OVER */
    {
    case IDLE:
      {
        /* After a OV occurs the turn on low side action become active. It is released just after a fault acknowledge -> state == IDLE */
        if (TurnOnLowSideAction == TRUE)
        {
          /* End of TURN_ON_LOW_SIDES action */
#if (HW_OV_CURRENT_PROT_BYPASS == ENABLE)
          DOUT_SetOutputState(oOCPDisabling[bMotor], INACTIVE); /* Re-enable the OCP */
#endif   
          PWMC_SwitchOffPWM(oCurrSensor[bMotor]);  /* Switch off the PWM */
        }        
      }
      break;
    case FAULT_NOW:
      {
        if (TurnOnLowSideAction == FALSE)
        {
          /* Switching off the PWM if fault occurs must be done just if TURN_ON_LOW_SIDES action is not in place */
          PWMC_SwitchOffPWM(oCurrSensor[bMotor]);
          FOC_Clear(bMotor);
          MPM_Clear(oMPM[bMotor]);
        }
      }
      break;
    case FAULT_OVER:
      {
        if (TurnOnLowSideAction == FALSE)
        {
          /* Switching off the PWM if fault occurs must be done just if TURN_ON_LOW_SIDES action is not in place */
          PWMC_SwitchOffPWM(oCurrSensor[bMotor]);
        }
      }
      break;
    default:
      break;    
    }
  }
}

#ifdef DUALDRIVE
#if defined (CCMRAM_ENABLED)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif
/**
  * @brief  This function is called by TIMx_UP_IRQHandler in case of dual MC and
  *         it allows to reserve half PWM period in advance the FOC execution on
  *         ADC ISR
  * @param  oDrive pointer to a CFOC object
  * @retval None
  */
void TSK_DualDriveFIFOUpdate(void *oDrive)
{
  FOC_array[FOC_array_tail] = oDrive;
  FOC_array_tail++;
  if (FOC_array_tail == FOC_ARRAY_LENGHT)
  {
    FOC_array_tail = 0;
  }
}
#endif

/**
  * @brief  This function returns the reference of the MCInterface relative to 
  *         the selected drive. 
  * @param  bMotor Motor reference number defined 
  *         \link Motors_reference_number here \endlink
  * @retval CMCI Reference to MCInterface relative to the selected drive.
  *         Note: it can be MC_NULL if MCInterface of selected drive is not
  *         allocated.
  */
CMCI GetMCI(uint8_t bMotor)
{
  CMCI retVal = MC_NULL;
  if ((oMCInterface != MC_NULL) && (bMotor < MC_NUM))
  {
    retVal = oMCInterface[bMotor];
  }
  return retVal;
}

/**
  * @brief  This function returns the reference of the MCTuning relative to 
  *         the selected drive.
  * @param  bMotor Motor reference number defined 
  *         \link Motors_reference_number here \endlink
  * @retval CMCI Reference to MCInterface relative to the selected drive.
  *         Note: it can be MC_NULL if MCInterface of selected drive is not
  *         allocated.
  */
CMCT GetMCT(uint8_t bMotor)
{
  CMCT retVal = MC_NULL;
  if ((oMCTuning != MC_NULL) && (bMotor < MC_NUM))
  {
    retVal = oMCTuning[bMotor];
  }
  return retVal;
}

/**
  * @brief  It is executed when a general hardware failure has been detected by
  *         the microcontroller and is used to put the system in safety 
  *         condition.
  * @param  None
  * @retval None
  */
void TSK_HardwareFaultTask(void)
{
#if defined(MOTOR_PROFILER)
  SCC_Stop(oSCC[M1]);
  OTT_Stop(oOTT[M1]);
#endif
  PWMC_SwitchOffPWM(oCurrSensor[M1]);  
  STM_FaultProcessing(oSTM[M1], MC_SW_ERROR, 0);
  
#ifdef DUALDRIVE
  PWMC_SwitchOffPWM(oCurrSensor[M2]);
  STM_FaultProcessing(oSTM[M2], MC_SW_ERROR, 0);
#endif
}

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
