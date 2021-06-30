/**
  ******************************************************************************
  * @file    MC.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   Access point to the MC API.
  * To use any of the MC API function in a user code follow the steps:
  *         1) #include "MC.h"
  *         2) Create the CMCI local reference: CMCI oMCI
  *         3) Get reference of MCI via: oMCI = GetMCI(M1) or oMCI = GetMCI(M1)
  *         4) Call any MCI function:
  *               e.g. MCI_ExecSpeedRamp(oMCI,speed,duration);
  * To use any MCT function in a user code follow the steps:
  *         1) #include "MC.h"
  *         2) Create the CMCT local reference: CMCT oMCT
  *         3) Get reference of MCT via: oMCT = GetMCT(M1) or oMCT = GetMCT(M1)
  *         4) Get reference of specific object to tune:
  *               e.g. CPI oPI = MCT_GetSpeedLoopPID(oMCT);
  *         5) Call any MCT function related to that object:
  *               e.g. PI_SetKP(oPI,newKPValue);
  * It is also available the DAC related function like:
  *               e.g. MC_SetDAC(DAC_CH0, MC_PROTOCOL_REG_OBS_EL_ANGLE);
  *               e.g. MC_SetUserDAC(DAC_USER1,value);
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
#ifndef __MC_H
#define __MC_H

/* Includes ------------------------------------------------------------------*/
#include "MCTuningClass.h"
#include "MCInterfaceClass.h"

/** @addtogroup STM32_PMSM_MC_Application
  * @{
  */

/** @addtogroup MC
  * @{
  */

/** @defgroup MC_exported_types MC exported types
  * @{
  */

typedef enum {UDRC_STATE_IDLE, UDRC_STATE_REQUESTED, UDRC_STATE_EOC} UDRC_State_t;

/** @defgroup MC_Protocol_REG Motor control protocol registers
* @{
*/
typedef enum {
  MC_PROTOCOL_REG_TARGET_MOTOR,          /* 0   */
  MC_PROTOCOL_REG_FLAGS,                 /* 1   */
  MC_PROTOCOL_REG_STATUS,                /* 2   */
  MC_PROTOCOL_REG_CONTROL_MODE,          /* 3   */
  MC_PROTOCOL_REG_SPEED_REF,             /* 4   */
  MC_PROTOCOL_REG_SPEED_KP,              /* 5   */
  MC_PROTOCOL_REG_SPEED_KI,              /* 6   */
  MC_PROTOCOL_REG_SPEED_KD,              /* 7   */
  MC_PROTOCOL_REG_TORQUE_REF,            /* 8   */
  MC_PROTOCOL_REG_TORQUE_KP,             /* 9   */
  MC_PROTOCOL_REG_TORQUE_KI,             /* 10  */
  MC_PROTOCOL_REG_TORQUE_KD,             /* 11  */
  MC_PROTOCOL_REG_FLUX_REF,              /* 12  */
  MC_PROTOCOL_REG_FLUX_KP,               /* 13  */
  MC_PROTOCOL_REG_FLUX_KI,               /* 14  */
  MC_PROTOCOL_REG_FLUX_KD,               /* 15  */
  MC_PROTOCOL_REG_OBSERVER_C1,           /* 16  */
  MC_PROTOCOL_REG_OBSERVER_C2,           /* 17  */
  MC_PROTOCOL_REG_OBSERVER_CR_C1,        /* 18  */
  MC_PROTOCOL_REG_OBSERVER_CR_C2,        /* 19  */
  MC_PROTOCOL_REG_PLL_KI,                /* 20  */
  MC_PROTOCOL_REG_PLL_KP,                /* 21  */
  MC_PROTOCOL_REG_FLUXWK_KP,             /* 22  */
  MC_PROTOCOL_REG_FLUXWK_KI,             /* 23  */
  MC_PROTOCOL_REG_FLUXWK_BUS,            /* 24  */
  MC_PROTOCOL_REG_BUS_VOLTAGE,           /* 25  */
  MC_PROTOCOL_REG_HEATS_TEMP,            /* 26  */
  MC_PROTOCOL_REG_MOTOR_POWER,           /* 27  */
  MC_PROTOCOL_REG_DAC_OUT1,              /* 28  */
  MC_PROTOCOL_REG_DAC_OUT2,              /* 29  */
  MC_PROTOCOL_REG_SPEED_MEAS,            /* 30  */
  MC_PROTOCOL_REG_TORQUE_MEAS,           /* 31  */
  MC_PROTOCOL_REG_FLUX_MEAS,             /* 32  */
  MC_PROTOCOL_REG_FLUXWK_BUS_MEAS,       /* 33  */
  MC_PROTOCOL_REG_RUC_STAGE_NBR,         /* 34  */
  MC_PROTOCOL_REG_I_A,                   /* 35  */
  MC_PROTOCOL_REG_I_B,                   /* 36  */
  MC_PROTOCOL_REG_I_ALPHA,               /* 37  */
  MC_PROTOCOL_REG_I_BETA,                /* 38  */
  MC_PROTOCOL_REG_I_Q,                   /* 39  */
  MC_PROTOCOL_REG_I_D,                   /* 40  */
  MC_PROTOCOL_REG_I_Q_REF,               /* 41  */
  MC_PROTOCOL_REG_I_D_REF,               /* 42  */
  MC_PROTOCOL_REG_V_Q,                   /* 43  */
  MC_PROTOCOL_REG_V_D,                   /* 44  */
  MC_PROTOCOL_REG_V_ALPHA,               /* 45  */
  MC_PROTOCOL_REG_V_BETA,                /* 46  */
  MC_PROTOCOL_REG_MEAS_EL_ANGLE,         /* 47  */
  MC_PROTOCOL_REG_MEAS_ROT_SPEED,        /* 48  */
  MC_PROTOCOL_REG_OBS_EL_ANGLE,          /* 49  */
  MC_PROTOCOL_REG_OBS_ROT_SPEED,         /* 50  */
  MC_PROTOCOL_REG_OBS_I_ALPHA,           /* 51  */
  MC_PROTOCOL_REG_OBS_I_BETA,            /* 52  */
  MC_PROTOCOL_REG_OBS_BEMF_ALPHA,        /* 53  */
  MC_PROTOCOL_REG_OBS_BEMF_BETA,         /* 54  */
  MC_PROTOCOL_REG_OBS_CR_EL_ANGLE,       /* 55  */
  MC_PROTOCOL_REG_OBS_CR_ROT_SPEED,      /* 56  */
  MC_PROTOCOL_REG_OBS_CR_I_ALPHA,        /* 57  */
  MC_PROTOCOL_REG_OBS_CR_I_BETA,         /* 58  */
  MC_PROTOCOL_REG_OBS_CR_BEMF_ALPHA,     /* 59  */
  MC_PROTOCOL_REG_OBS_CR_BEMF_BETA,      /* 60  */
  MC_PROTOCOL_REG_DAC_USER1,             /* 61  */
  MC_PROTOCOL_REG_DAC_USER2,             /* 62  */
  MC_PROTOCOL_REG_MAX_APP_SPEED,         /* 63  */
  MC_PROTOCOL_REG_MIN_APP_SPEED,         /* 64  */
  MC_PROTOCOL_REG_IQ_SPEEDMODE,          /* 65  */
  MC_PROTOCOL_REG_EST_BEMF_LEVEL,        /* 66  */
  MC_PROTOCOL_REG_OBS_BEMF_LEVEL,        /* 67  */
  MC_PROTOCOL_REG_EST_CR_BEMF_LEVEL,     /* 68  */
  MC_PROTOCOL_REG_OBS_CR_BEMF_LEVEL,     /* 69  */
  MC_PROTOCOL_REG_FF_1Q,                 /* 70  */
  MC_PROTOCOL_REG_FF_1D,                 /* 71  */
  MC_PROTOCOL_REG_FF_2,                  /* 72  */
  MC_PROTOCOL_REG_FF_VQ,                 /* 73  */
  MC_PROTOCOL_REG_FF_VD,                 /* 74  */
  MC_PROTOCOL_REG_FF_VQ_PIOUT,           /* 75  */
  MC_PROTOCOL_REG_FF_VD_PIOUT,           /* 76  */
  MC_PROTOCOL_REG_PFC_STATUS,            /* 77  */
  MC_PROTOCOL_REG_PFC_FAULTS,            /* 78  */
  MC_PROTOCOL_REG_PFC_DCBUS_REF,         /* 79  */
  MC_PROTOCOL_REG_PFC_DCBUS_MEAS,        /* 80  */
  MC_PROTOCOL_REG_PFC_ACBUS_FREQ,        /* 81  */
  MC_PROTOCOL_REG_PFC_ACBUS_RMS,         /* 82  */
  MC_PROTOCOL_REG_PFC_I_KP,              /* 83  */
  MC_PROTOCOL_REG_PFC_I_KI,              /* 84  */
  MC_PROTOCOL_REG_PFC_I_KD,              /* 85  */
  MC_PROTOCOL_REG_PFC_V_KP,              /* 86  */
  MC_PROTOCOL_REG_PFC_V_KI,              /* 87  */
  MC_PROTOCOL_REG_PFC_V_KD,              /* 88  */
  MC_PROTOCOL_REG_PFC_STARTUP_DURATION,  /* 89  */
  MC_PROTOCOL_REG_PFC_ENABLED,           /* 90  */
  MC_PROTOCOL_REG_RAMP_FINAL_SPEED,      /* 91  */
  MC_PROTOCOL_REG_RAMP_DURATION,         /* 92  */
  MC_PROTOCOL_REG_HFI_EL_ANGLE,          /* 93  */
  MC_PROTOCOL_REG_HFI_ROT_SPEED,         /* 94  */
  MC_PROTOCOL_REG_HFI_CURRENT,           /* 95  */
  MC_PROTOCOL_REG_HFI_INIT_ANG_PLL,      /* 96  */
  MC_PROTOCOL_REG_HFI_INIT_ANG_SAT_DIFF, /* 97  */
  MC_PROTOCOL_REG_HFI_PI_PLL_KP,         /* 98  */
  MC_PROTOCOL_REG_HFI_PI_PLL_KI,         /* 99  */
  MC_PROTOCOL_REG_HFI_PI_TRACK_KP,       /* 100 */
  MC_PROTOCOL_REG_HFI_PI_TRACK_KI,       /* 101 */
  MC_PROTOCOL_REG_SC_CHECK,              /* 102 */
  MC_PROTOCOL_REG_SC_STATE,              /* 103 */
  MC_PROTOCOL_REG_SC_RS,                 /* 104 */
  MC_PROTOCOL_REG_SC_LS,                 /* 105 */
  MC_PROTOCOL_REG_SC_KE,                 /* 106 */
  MC_PROTOCOL_REG_SC_VBUS,               /* 107 */
  MC_PROTOCOL_REG_SC_MEAS_NOMINALSPEED,  /* 108 */
  MC_PROTOCOL_REG_SC_STEPS,              /* 109 */
  MC_PROTOCOL_REG_SPEED_KP_DIV,          /* 110 */
  MC_PROTOCOL_REG_SPEED_KI_DIV,          /* 111 */
  MC_PROTOCOL_REG_UID,                   /* 112 */
  MC_PROTOCOL_REG_HWTYPE,                /* 113 */
  MC_PROTOCOL_REG_CTRBDID,               /* 114 */
  MC_PROTOCOL_REG_PWBDID,                /* 115 */
  MC_PROTOCOL_REG_SC_PP,                 /* 116 */
  MC_PROTOCOL_REG_SC_CURRENT,            /* 117 */
  MC_PROTOCOL_REG_SC_SPDBANDWIDTH,       /* 118 */
  MC_PROTOCOL_REG_SC_LDLQRATIO,          /* 119 */
  MC_PROTOCOL_REG_SC_NOMINAL_SPEED,      /* 120 */
  MC_PROTOCOL_REG_SC_CURRBANDWIDTH,      /* 121 */
  MC_PROTOCOL_REG_SC_J,                  /* 122 */
  MC_PROTOCOL_REG_SC_F,                  /* 123 */
  MC_PROTOCOL_REG_SC_MAX_CURRENT,        /* 124 */
  MC_PROTOCOL_REG_SC_STARTUP_SPEED,      /* 125 */
  MC_PROTOCOL_REG_SC_STARTUP_ACC,        /* 126 */
  MC_PROTOCOL_REG_SC_PWM_FREQUENCY,      /* 127 */
  MC_PROTOCOL_REG_SC_FOC_REP_RATE,       /* 128 */
  MC_PROTOCOL_REG_PWBDID2,               /* 129 */
  MC_PROTOCOL_REG_SC_COMPLETED,          /* 130 */
  MC_PROTOCOL_REG_UNDEFINED
} MC_Protocol_REG_t;
/**
  * @}
  */

/** @defgroup DAC_Channels DAC channels
* @{
*/
typedef enum {
	DAC_CH0,
	DAC_CH1,
	DAC_CH2,
	DAC_CH3,
	DAC_MAX_Channel_nbr
} DAC_Channel_t;
/**
  * @}
  */

/** @defgroup DAC_UserChannel DAC user channels
* @{
*/
typedef enum {
	DAC_USER1,
	DAC_USER2
} DAC_UserChannel_t;
/**
  * @}
  */

/**
  * @}
  */
  
/** @defgroup MC_exported_methods MC exported methods
  * @{
  */

/**
  * @brief  This function returns the reference of the MCInterface relative to 
  *         the selected drive. 
  * @param  bMotor Motor reference number defined 
  *         \link Motors_reference_number here \endlink
  * @retval CMCI Reference to MCInterface relative to the selected drive.
  *         Note: it can be MC_NULL if MCInterface of selected drive is not
  *         allocated.
  */
CMCI GetMCI(uint8_t bMotor);

/**
  * @brief  This function returns the reference of the MCTuning relative to 
  *         the selected drive.
  * @param  bMotor Motor reference number defined 
  *         \link Motors_reference_number here \endlink
  * @retval CMCI Reference to MCInterface relative to the selected drive.
  *         Note: it can be MC_NULL if MCInterface of selected drive is not
  *         allocated.
  */
CMCT GetMCT(uint8_t bMotor);

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
void MC_RequestRegularConv(uint8_t bChannel, uint8_t bSamplTime);

/**
* @brief  Get the last user-defined regular conversion.
* @retval uint16_t It returns converted value or oxFFFF for conversion error.
*         This function returns a valid result if the state returned by 
*         MC_RegularConvState is UDRC_STATE_EOC.
*/
uint16_t MC_GetRegularConv(void);

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
UDRC_State_t MC_RegularConvState(void);

/**
  * @brief  This method is used to set up the DAC outputs. The selected
  *         variables will be provided in the related output channels.
  * @param  bChannel the DAC channel to be programmed. It must be one of the 
  *         exported channels decribed \link DAC_Channels here\endlink.
  *         E.g. DAC_CH0.
  * @param  bVariable the variables to be provided in out through the selected
  *         channel. It must be one of the exported UI register described
  *         \link MC_Protocol_REG here\endlink. E.g. MC_PROTOCOL_REG_I_A.
  * @retval none.
  */
void MC_SetDAC(DAC_Channel_t bChannel, MC_Protocol_REG_t bVariable);

/**
  * @brief  This method is used to set the value of the "User DAC channel".
  * @param  bUserChNumber the "User DAC channel" to be programmed. One of
  *         the channels described \link DAC_UserChannel here\endlink can be
  *         used. E.g. DAC_USER1.
  * @param  hValue 16bit signed value to be put in output.
  * @retval none.
  */
void MC_SetUserDAC(DAC_UserChannel_t bUserChNumber, int16_t hValue);

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /* __MC_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
