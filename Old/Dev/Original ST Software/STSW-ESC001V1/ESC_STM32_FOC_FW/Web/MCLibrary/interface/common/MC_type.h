/**
  ******************************************************************************
  * @file    MC_type.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   Standard lib MC class definition
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
#ifndef __MC_TYPE_H
#define __MC_TYPE_H

/* Includes ------------------------------------------------------------------*/
/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */
  
/** @defgroup MCLibraryConf_MISRACompliancy MC Classes MISRA Compliancy Test
* @{
*/

/** 
  * @brief Uncomment #define MISRA_C_2004_BUILD to build the library including 
  *        "stm32fxxx_MisraCompliance.h" instead of "stm32fxxx.h". 
  *        This will allow to build the library in 'strict ISO/ANSI C' and in 
  *        compliance with MISRA C 2004 rules (check project options)
  */
/*#define MISRA_C_2004_BUILD*/

/**
  * @}
  */

#if (defined(STM32F10X_MD) || defined (STM32F10X_LD) || defined(STM32F10X_MD_VL) || defined(STM32F10X_LD_VL) || defined(STM32F10X_HD))
  #ifdef MISRA_C_2004_BUILD
    #include "stm32f10x_MisraCompliance.h"
    #define FULL_MISRA_C_COMPLIANCY
  #else
    #include "stm32f10x.h"
  #endif
#elif defined(STM32F2XX)
  #ifdef MISRA_C_2004_BUILD
    #include "stm32f2xx_MisraCompliance.h"
    #define FULL_MISRA_C_COMPLIANCY
  #else
    #include "stm32f2xx.h"
  #endif
#elif (defined(STM32F40XX) || defined (STM32F446xx))
  #ifdef MISRA_C_2004_BUILD
    #include "stm32f40x_MisraCompliance.h"
    #define FULL_MISRA_C_COMPLIANCY
  #else
    #include "stm32f4xx.h"
  #endif
#elif defined(STM32F0XX)
  #ifdef MISRA_C_2004_BUILD
    #include "stm32f0xx_MisraCompliance.h"
    #define FULL_MISRA_C_COMPLIANCY
  #else
    #include "stm32f0xx.h"
  #endif
#elif defined(STM32F30X)
  #ifdef MISRA_C_2004_BUILD
    #include "stm32f30x_MisraCompliance.h"
    #define FULL_MISRA_C_COMPLIANCY
  #else
    #include "stm32f30x.h"
  #endif
#endif

#ifndef __cplusplus
typedef enum {FALSE,TRUE} bool;
#endif

/** @addtogroup MC_Type
  * @{
  */
  
/** @defgroup MCType_exported_constant Exported constant definitions
* @{
*/

/**
* @brief Not initialized pointer
*/
#define MC_NULL    (uint16_t)(0x0000u)

/**
  * @}
  */

/** @defgroup Motors_reference_number Motor reference number
* @{
*/
#define M1      (uint8_t)(0x0)  /*!< Motor 1.*/
#define M2      (uint8_t)(0x1)  /*!< Motor 2.*/
#define M_NONE  (uint8_t)(0xFF) /*!< None motor.*/
/**
  * @}
  */

/** @defgroup Standard_types_boundary_values Standard types boundary values
* @{
*/
#define U8_MAX      ((uint8_t)255)
#define S8_MAX      ((int8_t)127)
#define S8_MIN      ((int8_t)-127)
#define U16_MAX     ((uint16_t)65535u)
#define S16_MAX     ((int16_t)32767)
#define S16_MIN     ((int16_t)-32767)
#define S16_STD_MIN ((int16_t)-32768)
#define U32_MAX     ((uint32_t)4294967295uL)
#define S32_MAX     ((int32_t)2147483647)
#define S32_MIN     ((int32_t)-2147483647)
/**
  * @}
  */
  
/** @defgroup Fault_generation_error_codes Fault generation error codes definition
* @{
*/
typedef enum
{
  MC_NO_ERROR = (uint16_t)(0x0000u),      /*!<No error.*/
  MC_NO_FAULTS = (uint16_t)(0x0000u),     /*!<No error.*/
  MC_FOC_DURATION = (uint16_t)(0x0001u),  /*!<Error: FOC rate to high.*/
  MC_OVER_VOLT = (uint16_t)(0x0002u),     /*!<Error: Software over voltage.*/
  MC_UNDER_VOLT = (uint16_t)(0x0004u),    /*!<Error: Software under voltage.*/
  MC_OVER_TEMP = (uint16_t)(0x0008u),     /*!<Error: Software over temperature.*/
  MC_START_UP = (uint16_t)(0x0010u),      /*!<Error: Startup failed.*/
  MC_SPEED_FDBK = (uint16_t)(0x0020u),    /*!<Error: Speed feedback.*/
  MC_BREAK_IN = (uint16_t)(0x0040u),      /*!<Error: Emergency input (Over current).*/
  MC_SW_ERROR = (uint16_t)(0x0080u)       /*!<Software Error.*/
} FaultCondition_t;
/**
  * @}
  */
  
/** @defgroup MCType_exported_types Exported types
* @{
*/

/** 
  * @brief  Two components stator current type definition 
  */
typedef struct 
{
  int16_t qI_Component1;
  int16_t qI_Component2;
} Curr_Components;

/** 
  * @brief  Two components stator voltage type definition 
  */
typedef struct 
{
  int16_t qV_Component1;
  int16_t qV_Component2;
} Volt_Components;

/** 
  * @brief  ADConv_t type definition, it is used by PWMC_ADC_SetSamplingTime method of PWMnCurrFdbk class for user defined A/D regular conversions 
  */
typedef struct
{ 
  uint8_t Channel;   /*!< Integer channel number, from 0 to 15 */
  uint8_t SamplTime; /*!< Sampling time selection, ADC_SampleTime_nCycles5 
                          in case of STM32F10x, n= 1, 3, ...; */
} ADConv_t;

/** 
  * @brief  SensorType_t type definition, it's used in BusVoltageSensor and TemperatureSensor classes parameters structures
  *		    to specify whether the sensor is real or emulated by SW
  */	
typedef enum
{
REAL_SENSOR, VIRTUAL_SENSOR
} SensorType_t;

/** 
  * @brief  DOutputState_t type definition, it's used by DOUT_SetOutputState method of DigitalOutput class to specify the 
  *			required output state
  */
typedef enum
{
 INACTIVE, ACTIVE
} DOutputState_t;

/** 
  * @brief  State_t enum type definition, it lists all the possible state machine states  
  */
typedef enum
{
ICLWAIT = 12,         /*!< Persistent state, the system is waiting for ICL 
                           deactivation. Is not possible to run the motor if 
                           ICL is active. Until the ICL is active the state is 
                           forced to ICLWAIT, when ICL become inactive the state 
                           is moved to IDLE */
IDLE = 0,             /*!< Persistent state, following state can be IDLE_START 
                           if a start motor command has been given or 
                           IDLE_ALIGNMENT if a start alignment command has been 
                           given */
IDLE_ALIGNMENT = 1,   /*!< "Pass-through" state containg the code to be executed 
                           only once after encoder alignment command. 
                           Next states can be ALIGN_CHARGE_BOOT_CAP or 
                           ALIGN_OFFSET_CALIB according the configuration. It 
                           can also be ANY_STOP if a stop motor command has been 
                           given. */
ALIGN_CHARGE_BOOT_CAP = 13,/*!< Persistent state where the gate driver boot 
                           capacitors will be charged. Next states will be 
                           ALIGN_OFFSET_CALIB. It can also be ANY_STOP if a stop 
                           motor command has been given. */
ALIGN_OFFSET_CALIB = 14,/*!< Persistent state where the offset of motor currents 
                           measurements will be calibrated. Next state will be 
                           ALIGN_CLEAR. It can also be ANY_STOP if a stop motor 
                           command has been given. */
ALIGN_CLEAR = 15,     /*!< "Pass-through" state in which object is cleared and 
                           set for the startup.
                           Next state will be ALIGNMENT. It can also be ANY_STOP 
                           if a stop motor command has been given. */
ALIGNMENT = 2,        /*!< Persistent state in which the encoder are properly 
                           aligned to set mechanical angle, following state can 
                           only be ANY_STOP */
IDLE_START = 3,       /*!< "Pass-through" state containg the code to be executed
                           only once after start motor command. 
                           Next states can be CHARGE_BOOT_CAP or OFFSET_CALIB 
                           according the configuration. It can also be ANY_STOP 
                           if a stop motor command has been given. */
CHARGE_BOOT_CAP = 16, /*!< Persistent state where the gate driver boot 
                           capacitors will be charged. Next states will be 
                           OFFSET_CALIB. It can also be ANY_STOP if a stop motor 
                           command has been given. */
OFFSET_CALIB = 17,    /*!< Persistent state where the offset of motor currents 
                           measurements will be calibrated. Next state will be 
                           CLEAR. It can also be ANY_STOP if a stop motor 
                           command has been given. */
CLEAR = 18,           /*!< "Pass-through" state in which object is cleared and 
                           set for the startup.
                           Next state will be START. It can also be ANY_STOP if 
                           a stop motor command has been given. */
START = 4,            /*!< Persistent state where the motor start-up is intended 
                           to be executed. The following state is normally 
                           START_RUN as soon as first validated speed is 
                           detected. Another possible following state is 
                           ANY_STOP if a stop motor command has been executed */
START_RUN = 5,        /*!< "Pass-through" state, the code to be executed only 
                           once between START and RUN states it’s intended to be 
                           here executed. Following state is normally  RUN but 
                           it can also be ANY_STOP  if a stop motor command has 
                           been given */
RUN = 6,              /*!< Persistent state with running motor. The following 
                           state is normally ANY_STOP when a stop motor command 
                           has been executed */
ANY_STOP = 7,         /*!< "Pass-through" state, the code to be executed only 
                           once between any state and STOP it’s intended to be 
                           here executed. Following state is normally STOP */
STOP = 8,             /*!< Persistent state. Following state is normally 
                           STOP_IDLE as soon as conditions for moving state 
                           machine are detected */
STOP_IDLE = 9,        /*!< "Pass-through" state, the code to be executed only
                           once between STOP and IDLE it’s intended to be here 
                           executed. Following state is normally IDLE */
FAULT_NOW = 10,       /*!< Persistent state, the state machine can be moved from
                           any condition directly to this state by 
                           STM_FaultProcessing method. This method also manage 
                           the passage to the only allowed following state that 
                           is FAULT_OVER */
FAULT_OVER = 11       /*!< Persistent state where the application is intended to
                          stay when the fault conditions disappeared. Following 
                          state is normally STOP_IDLE, state machine is moved as 
                          soon as the user has acknowledged the fault condition. 
                      */
} State_t;


/** 
  * @brief  STC_Modality_t type definition, it's used by STC_SetControlMode and STC_GetControlMode methods in
  *         SpeednTorqCtrl class to specify the control modality type
  */
typedef enum
{
	STC_TORQUE_MODE, /*!<Torque mode.*/
	STC_SPEED_MODE   /*!<Speed mode.*/
} STC_Modality_t;


/** 
  * @brief IMFF_PMSM class, structure type definition for feed-forward constants
  *        tuning 
  */
typedef struct
{
  int32_t wConst_1D;
  int32_t wConst_1Q;
  int32_t wConst_2;  
} IMFF_TuningStruct_t, FF_TuningStruct_t;

/** 
  * @brief  Current references source type, internal or external to FOCDriveClass 
  */
typedef enum
{
  INTERNAL, EXTERNAL
} CurrRefSource_t ;

/** 
  * @brief  FOC variables structure
  */
typedef struct
{
  Curr_Components Iab;         /*!< Stator current on stator reference frame abc */
  Curr_Components Ialphabeta;  /*!< Stator current on stator reference frame 
                                    alfa-beta*/
  Curr_Components IqdHF;       /*!< Stator current on stator reference frame 
                                    alfa-beta*/                                     
  Curr_Components Iqd;         /*!< Stator current on rotor reference frame qd */ 
  Curr_Components Iqdref;      /*!< Stator current on rotor reference frame qd */ 
  int16_t UserIdref;           /*!< User value for the Idref stator current */ 
  Volt_Components Vqd;         /*!< Phase voltage on rotor reference frame qd */ 
  Volt_Components Valphabeta;  /*!< Phase voltage on stator reference frame 
                                   alpha-beta*/ 
  int16_t hTeref;              /*!< Reference torque */ 
  int16_t hElAngle;            /*!< Electrical angle used for reference frame 
                                    transformation  */
  uint16_t hCodeError;         /*!< error message */
  CurrRefSource_t bDriveInput; /*!< It specifies whether the current reference 
                                    source must be INTERNAL or EXTERNAL*/
} FOCVars_t, *pFOCVars_t;

/** 
  * @brief  Low side or enabling signal definition  
  */
#define LS_DISABLED   0
#define LS_PWM_TIMER  1
#define ES_GPIO       2

typedef enum
{
  LS_DISABLED_VAL = LS_DISABLED,    /*!< Low side signals and enabling signals always off.
                                         It is equivalent to DISABLED. */
  LS_PWM_TIMER_VAL = LS_PWM_TIMER,  /*!< Low side PWM signals are generated by timer. It is
                                         equivalent to ENABLED. */
  ES_GPIO_VAL = ES_GPIO             /*!< Enabling signals are managed by GPIOs (L6230 mode).*/
} LowSideOutputsFunction_t, *pLowSideOutputsFunction_t;

/** 
  * @brief  MPInfo structure (used only for serial communication)
  */
typedef struct
{
  uint8_t* data;
  uint8_t len;
} MPInfo_t, *pMPInfo_t;

/**
  * @}
  */

/** @defgroup MCType_UserInterfaceRelated_exported_definitions UserInterface related exported definitions
* @{
*/

#define OPT_NONE    0x00 /*!<No UI option selected.*/
#define OPT_LCD     0x01 /*!<Bit field indicating that the UI uses LCD manager.*/
#define OPT_COM     0x02 /*!<Bit field indicating that the UI uses serial communication.*/
#define OPT_DAC     0x04 /*!<Bit field indicating that the UI uses real DAC.*/
#define OPT_DACT    0x08 /*!<Bit field indicating that the UI uses RC Timer DAC.*/
#define OPT_DACS    0x10 /*!<Bit field indicating that the UI uses SPI communication.*/
#define OPT_DACF3   0x40 /*!<Bit field indicating that the UI uses DAC for STM32F3.*/
#define OPT_DACF072 0x80 /*!<Bit field indicating that the UI uses DAC for STM32F072.*/

/**
  * @}
  */
  
#define MAIN_SCFG_POS (28)
#define AUX_SCFG_POS (24)

#define MAIN_SCFG_VALUE(x) (((x)>>MAIN_SCFG_POS)&0x0F)
#define AUX_SCFG_VALUE(x)  (((x)>>AUX_SCFG_POS)&0x0F)

/** @defgroup MCType_UserInterfaceRelated_exported_definitions UserInterface related exported definitions
* @{
*/

#define PFC_SWE             0x0001u /*!<PFC Software error.*/
#define PFC_HW_PROT         0x0002u /*!<PFC hardware protection.*/
#define PFC_SW_OVER_VOLT    0x0004u /*!<PFC software over voltage.*/
#define PFC_SW_OVER_CURRENT 0x0008u /*!<PFC software over current.*/
#define PFC_SW_MAINS_FREQ   0x0010u /*!<PFC mains frequency error.*/
#define PFC_SW_MAIN_VOLT    0x0020u /*!<PFC mains voltage error.*/

/**
  * @}
  */

/** @defgroup MCType_DACUsedAsReferenceForProtectionRelated_exported_definitions DAC channel used as reference for protection exported definitions
* @{
*/

#define AO_DISABLED 0x00u /*!<Analog output disabled.*/
#define AO_DEBUG    0x01u /*!<Analog output debug.*/
#define VREF_OCPM1  0x02u /*!<Voltage reference for over current protection of motor 1.*/
#define VREF_OCPM2  0x03u /*!<Voltage reference for over current protection of motor 2.*/
#define VREF_OCPM12 0x04u /*!<Voltage reference for over current protection of both motors.*/
#define VREF_OVPM12 0x05u /*!<Voltage reference for over voltage protection of both motors.*/

/**
  * @}
  */

/** @defgroup MCType_Utils_exported_definitions Utillity fuinctions definitions
* @{
*/

#define RPM2MEC01HZ(rpm) (int16_t)((int32_t)(rpm)/6)

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /* __MC_TYPE_H */
/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
