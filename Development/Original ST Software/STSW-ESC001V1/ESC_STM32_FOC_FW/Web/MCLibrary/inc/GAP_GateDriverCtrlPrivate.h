/**
  ******************************************************************************
  * @file    GAP_GateDriverCtrlPrivate.h
  * @author  IMS Systems Lab and Technical Marketing - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private definition of GAP class      
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
#ifndef __GAP_GATEDRIVERCTRLPRIVATE_H
#define __GAP_GATEDRIVERCTRLPRIVATE_H

/** @addtogroup STM32F10x_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup GateDriverCtrl_GAP
  * @{
  */

/* Low-level functions dependent on the microcontroler stdlib */
void GAP_Init(CGDC this);
void GAP_SD_Deactivate(CGDC this);
void GAP_SD_Activate(CGDC this);
void GAP_CS_Deactivate(CGDC this);
void GAP_CS_Activate(CGDC this);
uint16_t GAP_SPI_Send(CGDC this, uint16_t value);

/** @defgroup GAP_private_types GAP private types
* @{
*/

/** 
  * @brief  GAP class members definition 
  */
typedef struct
{
  uint32_t wGAP_ErrorsNow[MAX_DEVICES_NUMBER];
                       /*!< Bitfield with extra error codes that is currently
                            active. The error code avalilable
                            are listed here (TBF).*/
  uint32_t wGAP_ErrorsOccurred[MAX_DEVICES_NUMBER];
                       /*!< Bitfield with extra error codes that occurs and is
                            over. The error code avalilable
                            are listed here (TBF).*/
} DVars_t,*pDVars_t;

/** 
  * @brief  Redefinition of parameter structure
  */
typedef GAPParams_t DParams_t, *pDParams_t; 

/** 
  * @brief Private GAP class definition 
  */
typedef struct
{
	DVars_t DVars_str;			/*!< Derived class members container */
	pDParams_t pDParams_str;	/*!< Derived class parameters container */
}_DCGAP_GDC_t, *_DCGAP_GDC;
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__GAP_GATEDRIVERCTRLPRIVATE_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
