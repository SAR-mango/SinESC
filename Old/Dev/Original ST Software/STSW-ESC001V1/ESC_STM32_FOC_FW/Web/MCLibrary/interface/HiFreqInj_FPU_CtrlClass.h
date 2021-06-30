/**
  ******************************************************************************
  * @file    HiFreqInj_FPU_CtrlClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of HiFreqInj_FPU_Ctrl class      
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
#ifndef __HFINJCTRLCLASS_H
#define __HFINJCTRLCLASS_H

/* Includes ------------------------------------------------------------------*/
#include "MC_type.h"
#include "PIRegulatorClass.h"
#include "BusVoltageSensorClass.h"
#include "SpeednPosFdbkClass.h"
#include "HiFreqInj_FPU_SpeednPosFdbkClass.h"
#include "arm_math.h"

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */

/** @addtogroup HiFreqInj_FPU_Ctrl
  * @{
  */
  
/** @defgroup HiFreqInj_FPU_Ctrl_class_exported_types HiFreqInj_FPU_Ctrl class exported types
* @{
*/

/** 
  * @brief  Public HiFreqInj_FPU_Ctrl class definition 
  */
typedef struct CHFI_FP_t *CHFI_FP;

/** 
  * @brief HiFreqInj_FPU_Ctrl class init structure type definition
  */
typedef struct
{
  CHFI_FP_SPD  oHFI_FP_SpeedSensor;
  pFOCVars_t pFOCVarsPtr;
  CPI   oPI_q;                 /*!< retaled PI controller used for q-axes current 
                                    regulation */
  CPI   oPI_d;                 /*!< related PI controller used for q-axes current 
                                    regulation */
  CVBS oVbusSensor;            /*!< related bus voltage sensor */
} HFI_FP_Init_t;

/** 
  * @brief  HiFreqInj_FPU_Ctrl class parameters definition  
  */
typedef const struct
{
  /* Digital filter */
  float32_t Notch_DIIcoefficients[5]; /*!< Notch filter coefficients */
  float32_t LPI_DIIcoefficients[5];   /*!< Low pass filter coefficients */
  float32_t HPI_DIIcoefficients[5];   /*!< High pass filter coefficients */
  float32_t PLL_DIIcoefficients[5];   /*!< Low pass filter coefficients */
  
  /* HFI params */
  int16_t hHiFrFreq;            /*!< High frequency signal, Hertz */
  int16_t hHiFrAmplVolts;       /*!< High frequency signal amplitude, volts */
  int16_t hPWMFr;               /*!< PWM frequency, Hertz*/
  int16_t hIdhPhase;            /*!< Reserved */
  PIParams_t PIParams;          /*!< It contains the parameters of the PI*/
  
  /* init scan parameters */
  float32_t fDefPLL_KP;    /*!< Default KP for init scan PLL */
  float32_t fDefPLL_KI;    /*!< Default KI for init scan PLL */
  int16_t hLockFreq;    /*!< rotation freq of the initial scan, expressed in dpp */
  uint8_t hScanRotationsNo; /*!< Number of initial rotor scans */
  int16_t hHiFrAmplScanVolts; /*!< High frequency signal amplification during scan, volts */
  uint16_t hWaitBeforeNS; /*!< Wait time before NS detection, expressed in number of half HFI period*/
  uint16_t hWaitAfterNS;  /*!< Wait time after NS detection, expressed in number of half HFI period */
  uint8_t hNSMaxDetPoints;  /*!< NS det points, < 31*/
  uint8_t hNSDetPointsSkip; /*!< NS det points skipped*/
  int16_t hDefMinSaturationDifference; /*!< Minimum saturation difference to validate NS detection.*/
  bool debugmode;        /*!< HFI in debug mode just angle detection */
  bool RevertDirection;  /*!< TRUE reverts the detected direction. */
  uint16_t hWaitTrack;   /*!< Reserved */
  uint16_t hWaitSynch;   /*!< Reserved */
  int16_t hStepAngle;    /*!< Reserved */
  int16_t hMaxangleDiff; /*!< Reserved */
  float32_t fRestartTime; /*!< Reserved */
} HiFreqInj_FPU_CtrlParams_t, *pHiFreqInj_FPU_CtrlParams_t;
  
/**
* @}
*/

/** @defgroup HiFreqInj_FPU_Ctrl_class_exported_methods HiFreqInj_FPU_Ctrl class exported methods
  * @{
  */

/**
  * @brief  Creates an object of the class HiFreqInj_FPU_Ctrl
  * @param  pHiFreqInj_FPU_CtrlParams pointer to an HiFreqInj_FPU_Ctrl parameters structure
  * @retval CHFI_FP new instance of HiFreqInj_FPU_Ctrl object
  */
CHFI_FP HFI_FP_NewObject(pHiFreqInj_FPU_CtrlParams_t pHiFreqInj_FPU_CtrlParams);

/**
  * @brief  Initializes all the object variables, usually it has to be called 
  *         once right after object creation.
  * @param  this related object of class CHFI_FP.
  * @param  pHFI_FP_Init HFI init strutcture.
  * @retval none.
  */
void HFI_FP_Init(CHFI_FP this, HFI_FP_Init_t* pHFI_FP_Init);

/**
  * @brief  It should be called before each motor restart and clears the HFI 
  *         internal variables
  * @param  this related object of class CHFI_FP.
  * @retval none
  */
void HFI_FP_Clear(CHFI_FP this);

/**
  * @brief  It filters motor phase currents from HFI components
  * @param  this related object of class CHFI_FP.
  * @param  IqdHF current components, as measured by current sensing stage.
  * @retval Iqd current components, filtered from HFI
  */
Curr_Components HFI_FP_PreProcessing(CHFI_FP this, Curr_Components IqdHF);

/**
  * @brief  It adds the HFI voltage in the qd reference frame
  * @param  this related object of class CHFI_FP.
  * @param  Vqd voltage components as calculated by current regulation stage.
  * @retval Vqd voltage components, added with HFI voltage
  */
Volt_Components HFI_FP_VqdConditioning(CHFI_FP this, Volt_Components Vqd);

/**
  * @brief  It synthesizes the HFI signal, and processes its results
  * @param  this related object of class CHFI_FP.
  * @param  IqdHF current components, qd phase currents (not filtered)
  * @retval none
  */
void HFI_FP_DataProcess(CHFI_FP this, Curr_Components IqdHF);

/**
  * @brief  Disable HF voltage gen when STO takes over
  * @param  this related object of class CHFI_FP
  * @retval none
  */
void HFI_FP_DisHFGeneration(CHFI_FP this);

/**
  * @brief  Gives information about HFI initial angle measurement stage completion
  * @param  this related object of class CHFI_FP
  * @retval bool; TRUE if HFI has locked on rotor angle;
  *         FALSE if HFI ended with an error;
  */
bool HFI_FP_STMRUN(CHFI_FP this);

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
  * @brief  It return the quantity that shall be put in the DAC to tune the HFI
  * @param  this related object of class CHFI_FP
  * @retval int16_t HFI current
  */
int16_t HFI_FP_GetCurrent(CHFI_FP this);

/**
  * @brief  It restarts the HF injection before the swap between STO-HFI;
  *         To be called continuously to check if the HFI insertion phase 
  *         has been completed.
  * @param  this related object of class CHFI_FP
  * @retval bool. TRUE if HFI insertion has been completed;
  *         FALSE if HFI insertion has not been completed yet 
  */
bool HFI_FP_Restart(CHFI_FP this);

/**
  * @brief  It starts the HF injection; to be called 
  *         continuously to check the status of angle detection phase.
  * @param  this related object of class CHFI_FP
  * @retval bool. TRUE if HFI angle detection has been completed;
  *         FALSE if HFI angle detection has not been completed yet
  */
bool HFI_FP_Start(CHFI_FP this);

/**
  * @brief  It updates the Kp gain of PLL PI
  * @param  this related object of class CHFI_FP
  * @param  float32_t New Kp gain
  * @retval None
  */
void HFI_FP_PLLSetKP(CHFI_FP this, float32_t fKpGain);

/**
  * @brief  It updates the Ki gain of PLL PI
  * @param  this related object of class CHFI_FP
  * @param  float32_t New Ki gain
  * @retval None
  */
void HFI_FP_PLLSetKI(CHFI_FP this, float32_t fKiGain);

/**
  * @brief  It returns the Kp gain of PLL PI
  * @param  this related object of class CHFI_FP
  * @retval float32_t Kp gain 
  */
float32_t HFI_FP_PLLGetKP(CHFI_FP this);

/**
  * @brief  It returns the Ki gain of PLL PI
  * @param  this related object of class CHFI_FP
  * @retval float32_t Ki gain 
  */
float32_t HFI_FP_PLLGetKI(CHFI_FP this);

/**
  * @brief  It returns the Track PI
  * @param  this related object of class CHFI_FP
  * @retval CPI Track PI 
  */
CPI HFI_FP_GetPITrack(CHFI_FP this);

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /* __HFINJCTRLCLASS_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
