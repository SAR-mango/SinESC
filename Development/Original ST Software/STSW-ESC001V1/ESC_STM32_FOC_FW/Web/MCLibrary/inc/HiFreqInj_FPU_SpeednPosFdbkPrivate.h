/**
  ******************************************************************************
  * @file    HiFreqInj_FPU_SpeednPosFdbkPrivate.h
  * @author  IMS Systems Lab and Technical Marketing - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private definition of HiFreqInj_FPU class      
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
#ifndef __HIFREQINJ_FPU_SPEEDNPOSFDBKPRIVATE_H
#define __HIFREQINJ_FPU_SPEEDNPOSFDBKPRIVATE_H

/** @addtogroup STM32F10x_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup SpeednPosFdbk_HiFreqInj_FPU
  * @{
  */

/** @defgroup HiFreqInj_FPU_private_types HiFreqInj_FPU private types
* @{
*/

/** 
  * @brief  HiFreqInj_FPU class members definition 
  */
typedef struct
{
  int16_t hHiFrTrSpeed; /*!< High Frequency track speed */
  bool Oriented; /*!< true if HF has detected the angle*/
  int16_t hSpeed_Buffer[64];    /*!< Estimated speed FIFO, it contains latest 
                                     bSpeedBufferSize speed measurements*/
  uint8_t bSpeed_Buffer_Index;  /*!< Position of latest speed estimation in 
                                     estimated speed FIFO */  
}DVars_t,*pDVars_t;

/** 
  * @brief  Redefinition of parameter structure
  */
typedef HiFreqInj_FPUParams_t DParams_t, *pDParams_t; 

/** 
  * @brief Private HiFreqInj_FPU class definition 
  */
typedef struct
{
	DVars_t DVars_str;			/*!< Derived class members container */
	pDParams_t pDParams_str;	/*!< Derived class parameters container */
}_DCHFI_FP_SPD_t, *_DCHFI_FP_SPD;
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__HIFREQINJ_FPU_SPEEDNPOSFDBKPRIVATE_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
