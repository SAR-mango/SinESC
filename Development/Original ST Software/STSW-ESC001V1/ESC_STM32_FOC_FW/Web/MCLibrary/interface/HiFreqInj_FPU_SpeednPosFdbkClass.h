/**
  ******************************************************************************
  * @file    HiFreqInj_FPU_SpeednPosFdbkClass.h
  * @author  IMS Systems Lab and Technical Marketing - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of HiFreqInj_FPU class      
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HIFREQINJ_FPU_SPEEDNPOSFDBKCLASS_H
#define __HIFREQINJ_FPU_SPEEDNPOSFDBKCLASS_H

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup SpeednPosFdbk_HiFreqInj_FPU
  * @{
  */

/** @defgroup HiFreqInj_FPU_class_exported_types HiFreqInj_FPU class exported types
* @{
*/

/** 
  * @brief  Public HiFreqInj_FPU class definition
  */
typedef struct CHFI_FP_SPD_t *CHFI_FP_SPD;

/** 
  * @brief  HiFreqInj_FPU class parameters definition
  */
typedef const struct
{
  uint8_t bSpeedBufferSize01Hz;/*!< Depth of FIFO used to average 
                                    estimated speed exported by 
                                    SPD_GetAvrgMecSpeed01Hz. It 
                                    must be an integer number between 1 
                                    and 64 */
  int16_t hSwapMecSpeed01Hz; /* Speed threshold to reset PLL.*/
  int16_t hSTOHFI01Hz;       /* Speed threshold to switch speed and position 
                                sensor from STO_PLL to HFI.*/
  int16_t hHFISTO01Hz;       /* Speed threshold to switch speed and position
                                sensor from HFI to STO_PLL.*/
  int16_t hHFIRestart01Hz;   /* Speed threshold to re-enable the HF injection.*/
} HiFreqInj_FPUParams_t, *pHiFreqInj_FPUParams_t;
/**
  * @}
  */

/** @defgroup HiFreqInj_FPU_class_exported_methods HiFreqInj_FPU class exported methods
  * @{
  */
  
/**
  * @brief  Creates an object of the class HiFreqInj_FPU
  * @param  pSpeednPosFdbkParams pointer to an SpeednPosFdbk parameters structure
  * @param  pHiFreqInj_FPUParams pointer to an HiFreqInj_FPU parameters structure
  * @retval CHFI_FP_SPD new instance of HiFreqInj_FPU object
  */
CHFI_FP_SPD HFI_FP_SPD_NewObject(pSpeednPosFdbkParams_t pSpeednPosFdbkParams, pHiFreqInj_FPUParams_t pHiFreqInj_FPUParams);

/**
  * @brief  Set electrical speed
  * @param  this related object of class CHFI_FP_SPD.
  * @retval none
  */
void HFI_FP_SPD_SetElSpeedDpp(CHFI_FP_SPD this,int16_t hHiFrTrSpeed);

/**
  * @brief  Set electrical angle
  * @param  this related object of class CHFI_FP_SPD.
  * @retval none
  */
void HFI_FP_SPD_SetElAngle(CHFI_FP_SPD this,int16_t hElAngle);

/**
  * @brief  Check that the motor speed has reached the minimum threshold
  *         for swapping speed sensor from HF to STO.
  * @param  this related object of class CHFI_FP_SPD.
  * @retval bool It returns TRUE if the threshold has been reached, FALSE otherw
  */
bool HFI_FP_SPD_AccelerationStageReached(CHFI_FP_SPD this);

/**
  * @brief  Informs the speed sensor that the angle detection is good.
  * @param  this related object of class CHFI_FP_SPD.
  * @param  TRUE if angle detection is good.
  * @retval none
  */
void HFI_FP_SPD_SetHFState(CHFI_FP_SPD this, bool StateOriented);
    
/**
  * @brief  Set mechanical speed.
  * @param  this related object of class CHFI_FP_SPD.
  * @retval none
  */
void HFI_FP_SPD_SetAvrgMecSpeed01Hz(CHFI_FP_SPD this,int16_t hHiFrTrSpeed);

/**
  * @brief  It checks if the HFI shall be restarted acconding the speed 
  *         measurement done by the OBS_PLL.
  * @param  this related object of class CHFI_FP_SPD.
  * @param  hObsSpeed01Hz Speed measured by OBS_PLL expressed in tenths of 
  *         mechanical Hertz.
  * @retval bool It returns TRUE if the HFI shall be restarted according the 
  *         speed measurement done by the OBS_PLL.
  */
bool HFI_FP_SPD_Restart(CHFI_FP_SPD this,int16_t hObsSpeed01Hz);

/**
  * @brief  It checks if the speed threshold to switch from OBS_PLL to HFI has
  *         been reached.
  * @param  this related object of class CHFI_FP_SPD.
  * @param  hObsSpeed01Hz Speed measured by OBS_PLL expressed in tenths of 
  *         mechanical Hertz.
  * @retval bool It returns TRUE if the speed threshold to switch from OBS_PLL
  *         to HFI has been reached.
  */
bool HFI_FP_SPD_IsConverged(CHFI_FP_SPD this,int16_t hObsSpeed01Hz);

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__HIFREQINJ_FPU_SPEEDNPOSFDBKCLASS_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
