/**
  ******************************************************************************
  * @file    HiFreqInj_FPU_SpeednPosFdbkClass.c
  * @author  IMS Systems Lab and Technical Marketing - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private impelementation of HiFreqInj_FPU class      
  ******************************************************************************
  * <br>
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  */

/* Includes ------------------------------------------------------------------*/
#include "SpeednPosFdbkClass.h"
#include "SpeednPosFdbkPrivate.h"
#include "HiFreqInj_FPU_SpeednPosFdbkClass.h"
#include "HiFreqInj_FPU_SpeednPosFdbkPrivate.h"
#include "MCLibraryConf.h"
#include "MC_type.h"

#ifdef MC_CLASS_DYNAMIC
  #include "stdlib.h" /* Used for dynamic allocation */
#else
  _DCHFI_FP_SPD_t HFI_FP_SPDpool[MAX_HFI_FP_SPD_NUM];
  unsigned char HFI_FP_SPD_Allocated = 0u;
#endif

#define DCLASS_PARAMS ((_DCHFI_FP_SPD)(((_CSPD) this)->DerivedClass))->pDParams_str
#define DCLASS_VARS  &(((_DCHFI_FP_SPD)(((_CSPD) this)->DerivedClass))->DVars_str)
#define  CLASS_VARS  &(((_CSPD)this)->Vars_str)
#define  CLASS_PARAMS (((_CSPD)this)->pParams_str)

static void HFI_FP_SPD_Init(CSPD this);
static void HFI_FP_SPD_Clear(CSPD this);
static int16_t HFI_FP_SPD_CalcElAngle(CSPD this, void *pInputVars_str);
static bool HFI_FP_SPD_CalcAvrgMecSpeed01Hz(CSPD this, int16_t *hMecSpeed01Hz);
static void HFI_FP_SPD_SetMecAngle(CSPD this, int16_t hMecAngle);
static void HFI_FP_SPD_InitSpeedBuffer(CHFI_FP_SPD this, int16_t hVal);

/**
  * @brief  Creates an object of the class HiFreqInj_FPU
  * @param  pSpeednPosFdbkParams pointer to an SpeednPosFdbk parameters structure
  * @param  pHiFreqInj_FPUParams pointer to an HiFreqInj_FPU parameters structure
  * @retval CHFI_FP_SPD new instance of HiFreqInj_FPU object
  */
CHFI_FP_SPD HFI_FP_SPD_NewObject(pSpeednPosFdbkParams_t pSpeednPosFdbkParams, pHiFreqInj_FPUParams_t pHiFreqInj_FPUParams)
{
  _CSPD _oSpeednPosFdbk;
  _DCHFI_FP_SPD _oHiFreqInj_FPU;
  
  _oSpeednPosFdbk = (_CSPD)SPD_NewObject(pSpeednPosFdbkParams);
  
#ifdef MC_CLASS_DYNAMIC
  _oHiFreqInj_FPU = (_DCHFI_FP_SPD)calloc(1u,sizeof(_DCHFI_FP_SPD_t));
#else
  if (HFI_FP_SPD_Allocated  < MAX_HFI_FP_SPD_NUM)
  {
    _oHiFreqInj_FPU = &HFI_FP_SPDpool[HFI_FP_SPD_Allocated++];
  }
  else
  {
    _oHiFreqInj_FPU = MC_NULL;
  }
#endif
  
  _oHiFreqInj_FPU->pDParams_str = pHiFreqInj_FPUParams;
  _oSpeednPosFdbk->DerivedClass = (void*)_oHiFreqInj_FPU;
  
  _oSpeednPosFdbk->Methods_str.pSPD_Init = &HFI_FP_SPD_Init;
  _oSpeednPosFdbk->Methods_str.pSPD_Clear = &HFI_FP_SPD_Clear;
  _oSpeednPosFdbk->Methods_str.pSPD_CalcAngle = &HFI_FP_SPD_CalcElAngle;
  _oSpeednPosFdbk->Methods_str.pSPD_CalcAvrgMecSpeed01Hz = 
    &HFI_FP_SPD_CalcAvrgMecSpeed01Hz;
  _oSpeednPosFdbk->Methods_str.pSPD_SetMecAngle = &HFI_FP_SPD_SetMecAngle;        
  
  return ((CHFI_FP_SPD)_oSpeednPosFdbk);
}

/** @addtogroup STM32F10x_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup SpeednPosFdbk_HiFreqInj_FPU
  * @{
  */

/** @defgroup HiFreqInj_FPU_class_private_methods HiFreqInj_FPU class private methods
* @{
*/

/**
  * @brief  Software initialization of VSS object.
  * @param  this related object of class CSPD
  * @param  InputVars_str not used by VSS.
  * @retval none
  */
static void HFI_FP_SPD_Init(CSPD this)
{
  HFI_FP_SPD_Clear(this);
}

/**
* @brief  Software initializzation of VSS object to be performed at each restart
*         of the motor.
* @param  this related object of class CSPD
* @retval none
*/
static void HFI_FP_SPD_Clear(CSPD this)
{
  pVars_t pVars = CLASS_VARS;
  pDVars_t pDVars = DCLASS_VARS;  
  pVars->hElAngle = 0;
  pVars->hMecAngle = 0;
  pVars->hAvrMecSpeed01Hz = 0;
  pVars->hElSpeedDpp = 0;
  pVars->hMecAccel01HzP = 0;
  pVars->bSpeedErrorNumber = 0u;
  
  pDVars->hHiFrTrSpeed = 0;
  pDVars->Oriented = FALSE;
  HFI_FP_SPD_InitSpeedBuffer((CHFI_FP_SPD)this,0); 
}

/**
* @brief  Update the rotor electrical angle integrating the last setled 
*         instantaneous electrical speed express in dpp.
* @param  this related object of class CSPD.
* @retval int16_t Measured electrical angle in s16degree format.
*/
static int16_t HFI_FP_SPD_CalcElAngle(CSPD this, void *pInputVars_str)
{
  pVars_t pVars = CLASS_VARS;
  pDVars_t pDVars = DCLASS_VARS;
  int16_t hElAngle = pVars->hElAngle - pDVars->hHiFrTrSpeed;
  pVars->hElAngle = hElAngle;
  return hElAngle;
}

/**
  * @brief  This method must be called with the same periodicity
  *         on which speed control is executed.
  *         This method compute and store rotor istantaneous el speed (express 
  *         in dpp considering the measurement frequency) in order to provide it
  *         to SPD_CalcElAngle function and SPD_GetElAngle. 
  *         Then compute store and return - through parameter 
  *         hMecSpeed01Hz - the rotor average mech speed, expressed in 01Hz.
  *         Then return the reliability state of the sensor (allways TRUE).
  * @param  this related object of class CSPD
  * @param  hMecSpeed01Hz pointer to int16_t, used to return the rotor average
  *         mechanical speed (01Hz)
  * @retval TRUE = sensor information is reliable
  *         FALSE = sensor information is not reliable
  */
static bool HFI_FP_SPD_CalcAvrgMecSpeed01Hz(CSPD this, int16_t *hMecSpeed01Hz)
{
  pParams_t pParams = CLASS_PARAMS;
  pDParams_t pDParams = DCLASS_PARAMS;
  pVars_t pVars = CLASS_VARS;
  pDVars_t pDVars = DCLASS_VARS;
  int32_t wAvrSpeed_dpp = (int32_t)0;
  int32_t wAux;
  uint8_t i, bSpeedBufferSize01Hz = pDParams->bSpeedBufferSize01Hz;
  bool retVal = FALSE;
  
  for (i=0u; i<bSpeedBufferSize01Hz; i++)
  {
    wAvrSpeed_dpp -= (int32_t)(pDVars->hSpeed_Buffer[i]);
  }
  
  wAvrSpeed_dpp = wAvrSpeed_dpp / (int16_t)bSpeedBufferSize01Hz;  
  
  /*Computation of Mechanical speed 01Hz*/
  wAux = wAvrSpeed_dpp * (int32_t)(pParams->hMeasurementFrequency);
  wAux = wAux * (int32_t) 10;
  wAux = wAux/(int32_t)65536;
  wAux = wAux /(int16_t)(pParams->bElToMecRatio);
  
  *hMecSpeed01Hz = (int16_t)wAux;
  
  pVars->hAvrMecSpeed01Hz = (int16_t)wAux;
  
  pVars->hElSpeedDpp = (int16_t)wAvrSpeed_dpp;
  
  if (pDVars->Oriented == TRUE)
  {
    retVal = TRUE;
  }
  
  return retVal;
}

/**
  * @brief  It is used to set istantaneous information on mechanical and
  *         electrical angle.
  * @param  this related object of class CSPD
  * @param  hMecAngle istantaneous measure of rotor mechanical angle
  * @retval none
  */
static void HFI_FP_SPD_SetMecAngle(CSPD this, int16_t hMecAngle)
{
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif
/**
  * @brief  Set electrical speed
  * @param  this related object of class CHFI_FP_SPD
  * @retval none
  */
void HFI_FP_SPD_SetElSpeedDpp(CHFI_FP_SPD this,int16_t hHiFrTrSpeed)
{
  pDParams_t pDParams = DCLASS_PARAMS;
  pDVars_t pDVars = DCLASS_VARS;
  uint8_t bBuffer_index = pDVars->bSpeed_Buffer_Index;
  
  bBuffer_index++;
  if (bBuffer_index == pDParams->bSpeedBufferSize01Hz) 
  {
    bBuffer_index = 0u;
  } 
  
  pDVars->hSpeed_Buffer[bBuffer_index] = hHiFrTrSpeed;
  pDVars->hHiFrTrSpeed = hHiFrTrSpeed;  
  pDVars->bSpeed_Buffer_Index = bBuffer_index;  
}

/**
  * @brief  Set electrical angle
  * @param  this related object of class CHFI_FP_SPD
  * @retval none
  */
void HFI_FP_SPD_SetElAngle(CHFI_FP_SPD this,int16_t hElAngle)
{
  pVars_t pVars = CLASS_VARS;
  pVars->hElAngle = hElAngle;
}

/**
  * @brief  Check that the motor speed has reached the minimum threshold
  *         for swapping speed sensor from HF to STO
  * @param  this related object of class CHFI_FP_SPD
  * @retval bool It returns TRUE if the threshold has been reached, FALSE otherw
  */
bool HFI_FP_SPD_AccelerationStageReached(CHFI_FP_SPD this)
{
  pDParams_t pDParams = DCLASS_PARAMS;
  pVars_t pVars = CLASS_VARS;
  bool retVal = FALSE;
  int16_t hAvrMecSpeed01Hz = pVars->hAvrMecSpeed01Hz; 
  
  if (hAvrMecSpeed01Hz < 0)
  {
    hAvrMecSpeed01Hz = -hAvrMecSpeed01Hz;
  }  
  
  if (hAvrMecSpeed01Hz > pDParams->hSwapMecSpeed01Hz)
  {
    retVal = TRUE;
  }
  
  return (retVal);
}

/**
  * @brief  Informs the speed sensor that the angle detection has been completed
  * NOT TO BE CALLED CONTINUOUSLY but only when state changes
  * @param  this related object of class CHFI_FP_SPD
  * @param  TRUE if angle detection phase has been completed
  * @retval none
  */
void HFI_FP_SPD_SetHFState(CHFI_FP_SPD this, bool StateOriented)
{
  pVars_t pVars = CLASS_VARS;
  pDVars_t pDVars = DCLASS_VARS;
  pDVars->Oriented = StateOriented;
  pVars->bSpeedErrorNumber = 0u;
}

/**
  * @brief  Set mechanical speed
  * @param  this related object of class CHFI_FP_SPD
  * @retval none
  */
void HFI_FP_SPD_SetAvrgMecSpeed01Hz(CHFI_FP_SPD this,int16_t hHiFrTrSpeed)
{
  pVars_t pVars = CLASS_VARS;
  pVars->hAvrMecSpeed01Hz = (int16_t)hHiFrTrSpeed;
  HFI_FP_SPD_InitSpeedBuffer((CHFI_FP_SPD)this,hHiFrTrSpeed);
}

/**
  * @brief  It clears the estimated speed buffer 
  * @param  this related object of class CSTO_SPD
  * @retval none
  */
static void HFI_FP_SPD_InitSpeedBuffer(CHFI_FP_SPD this,int16_t hVal)
{
  pDParams_t pDParams = DCLASS_PARAMS;
  pDVars_t pDVars = DCLASS_VARS;
  uint8_t b_i;
  
  uint8_t bSpeedBufferSize01Hz = pDParams->bSpeedBufferSize01Hz;
  
  /*init speed buffer*/
  for (b_i = 0u; b_i<bSpeedBufferSize01Hz; b_i++)
  {
    pDVars->hSpeed_Buffer[b_i] = hVal;
  }
  pDVars->bSpeed_Buffer_Index = 0u;
  
  return;
}

/**
  * @brief  It checks if the HFI shall be restarted acconding the speed 
  *         measurement done by the OBS_PLL.
  * @param  this related object of class CHFI_FP_SPD.
  * @param  hObsSpeed01Hz Speed measured by OBS_PLL expressed in tenths of 
  *         mechanical Hertz.
  * @retval bool It returns TRUE if the HFI shall be restarted according the 
  *         speed measurement done by the OBS_PLL.
  */
bool HFI_FP_SPD_Restart(CHFI_FP_SPD this,int16_t hObsSpeed01Hz)
{
  pDParams_t pDParams = DCLASS_PARAMS;
  bool retVal = FALSE;
  
  if (hObsSpeed01Hz < 0)
  {
    hObsSpeed01Hz = -hObsSpeed01Hz;
  }  
  
  if (hObsSpeed01Hz < pDParams->hHFIRestart01Hz)
  {
    retVal = TRUE;
  }
  
  return (retVal);
}

/**
  * @brief  It checks if the speed threshold to switch from OBS_PLL to HFI has
  *         been reached.
  * @param  this related object of class CHFI_FP_SPD.
  * @param  hObsSpeed01Hz Speed measured by OBS_PLL expressed in tenths of 
  *         mechanical Hertz.
  * @retval bool It returns TRUE if the speed threshold to switch from OBS_PLL
  *         to HFI has been reached.
  */
bool HFI_FP_SPD_IsConverged(CHFI_FP_SPD this,int16_t hObsSpeed01Hz)
{
  pDParams_t pDParams = DCLASS_PARAMS;
  bool retVal = FALSE;
  
  if (hObsSpeed01Hz < 0)
  {
    hObsSpeed01Hz = -hObsSpeed01Hz;
  }  
  
  if (hObsSpeed01Hz < pDParams->hSTOHFI01Hz)
  {
    retVal = TRUE;
  }
  
  return (retVal);
}

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */


/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
