/**
  ******************************************************************************
  * @file    GAP_GateDriverCtrlClass.h
  * @author  IMS Systems Lab and Technical Marketing - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of GAP class      
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
#ifndef __GAP_GATEDRIVERCTRLCLASS_H
#define __GAP_GATEDRIVERCTRLCLASS_H

#define GPIO_NoRemap_SPI ((uint32_t)(0))

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup GateDriverCtrl_GAP
  * @{
  */

#define MAX_DEVICES_NUMBER 7
    
/** @defgroup GAP_class_error_code GAP class error code
* @{
*/
#define GAP_ERROR_CLEAR        (uint32_t)(0x00000000)
#define GAP_ERROR_CODE_UVLOD   (uint32_t)(0x00000001)
#define GAP_ERROR_CODE_OVLOD   (uint32_t)(0x00000002)
#define GAP_ERROR_CODE_REGERRL (uint32_t)(0x00000004)
#define GAP_ERROR_CODE_SPI_ERR (uint32_t)(0x00000008)
#define GAP_ERROR_CODE_DT_ERR  (uint32_t)(0x00000010)
#define GAP_ERROR_CODE_CFG     (uint32_t)(0x00000020)
#define GAP_ERROR_CODE_GATE    (uint32_t)(0x00000100)
#define GAP_ERROR_CODE_ASC     (uint32_t)(0x00000200)
#define GAP_ERROR_CODE_REGERRR (uint32_t)(0x00000400)
#define GAP_ERROR_CODE_TWN     (uint32_t)(0x00010000)
#define GAP_ERROR_CODE_TSD     (uint32_t)(0x00020000)
#define GAP_ERROR_CODE_UVLOL   (uint32_t)(0x00040000)
#define GAP_ERROR_CODE_UVLOH   (uint32_t)(0x00080000)
#define GAP_ERROR_CODE_SENSE   (uint32_t)(0x00100000)
#define GAP_ERROR_CODE_DESAT   (uint32_t)(0x00200000)
#define GAP_ERROR_CODE_OVLOL   (uint32_t)(0x00400000)
#define GAP_ERROR_CODE_OVLOH   (uint32_t)(0x00800000)
#define GAP_ERROR_CODE_SPI_CRC (uint32_t)(0x40000000)
#define GAP_ERROR_CODE_DEVICES_NOT_PROGRAMMABLE (uint32_t)(0x80000000)    
/**
  * @}
  */
    
/** @defgroup GAP_class_exported_defines GAP class exported defines
* @{
*/
#define GAP_CFG1_CRC_SPI           (uint8_t)(0x80)
#define GAP_CFG1_UVLOD             (uint8_t)(0x40)
#define GAP_CFG1_SD_FLAG           (uint8_t)(0x20)
#define GAP_CFG1_DIAG_EN           (uint8_t)(0x10)
#define GAP_CFG1_DT_DISABLE        (uint8_t)(0x00)
#define GAP_CFG1_DT_250NS          (uint8_t)(0x04)
#define GAP_CFG1_DT_800NS          (uint8_t)(0x08)
#define GAP_CFG1_DT_1200NS         (uint8_t)(0x0C)
#define GAP_CFG1_INFILTER_DISABLE  (uint8_t)(0x00)
#define GAP_CFG1_INFILTER_210NS    (uint8_t)(0x01)
#define GAP_CFG1_INFILTER_560NS    (uint8_t)(0x02)
#define GAP_CFG1_INFILTER_70NS     (uint8_t)(0x03)

#define GAP_CFG2_SENSETH_100MV     (uint8_t)(0x00)
#define GAP_CFG2_SENSETH_125MV     (uint8_t)(0x20)
#define GAP_CFG2_SENSETH_150MV     (uint8_t)(0x40)
#define GAP_CFG2_SENSETH_175MV     (uint8_t)(0x60)
#define GAP_CFG2_SENSETH_200MV     (uint8_t)(0x80)
#define GAP_CFG2_SENSETH_250MV     (uint8_t)(0xA0)
#define GAP_CFG2_SENSETH_300MV     (uint8_t)(0xC0)
#define GAP_CFG2_SENSETH_400MV     (uint8_t)(0xE0)
#define GAP_CFG2_DESATCURR_250UA   (uint8_t)(0x00)
#define GAP_CFG2_DESATCURR_500UA   (uint8_t)(0x08)
#define GAP_CFG2_DESATCURR_750UA   (uint8_t)(0x10)
#define GAP_CFG2_DESATCURR_1000UA  (uint8_t)(0x18)
#define GAP_CFG2_DESATTH_3V        (uint8_t)(0x00)
#define GAP_CFG2_DESATTH_4V        (uint8_t)(0x01)
#define GAP_CFG2_DESATTH_5V        (uint8_t)(0x02)
#define GAP_CFG2_DESATTH_6V        (uint8_t)(0x03)
#define GAP_CFG2_DESATTH_7V        (uint8_t)(0x04)
#define GAP_CFG2_DESATTH_8V        (uint8_t)(0x05)
#define GAP_CFG2_DESATTH_9V        (uint8_t)(0x06)
#define GAP_CFG2_DESATTH_10V       (uint8_t)(0x07)

#define GAP_CFG3_2LTOTH_7_0V       (uint8_t)(0x00)
#define GAP_CFG3_2LTOTH_7_5V       (uint8_t)(0x10)
#define GAP_CFG3_2LTOTH_8_0V       (uint8_t)(0x20)
#define GAP_CFG3_2LTOTH_8_5V       (uint8_t)(0x30)
#define GAP_CFG3_2LTOTH_9_0V       (uint8_t)(0x40)
#define GAP_CFG3_2LTOTH_9_5V       (uint8_t)(0x50)
#define GAP_CFG3_2LTOTH_10_0V      (uint8_t)(0x60)
#define GAP_CFG3_2LTOTH_10_5V      (uint8_t)(0x70)
#define GAP_CFG3_2LTOTH_11_0V      (uint8_t)(0x80)
#define GAP_CFG3_2LTOTH_11_5V      (uint8_t)(0x90)
#define GAP_CFG3_2LTOTH_12_0V      (uint8_t)(0xA0)
#define GAP_CFG3_2LTOTH_12_5V      (uint8_t)(0xB0)
#define GAP_CFG3_2LTOTH_13_0V      (uint8_t)(0xC0)
#define GAP_CFG3_2LTOTH_13_5V      (uint8_t)(0xD0)
#define GAP_CFG3_2LTOTH_14_0V      (uint8_t)(0xE0)
#define GAP_CFG3_2LTOTH_14_5V      (uint8_t)(0xF0)
#define GAP_CFG3_2LTOTIME_DISABLE  (uint8_t)(0x00)
#define GAP_CFG3_2LTOTIME_0_75_US  (uint8_t)(0x01)
#define GAP_CFG3_2LTOTIME_1_00_US  (uint8_t)(0x02)
#define GAP_CFG3_2LTOTIME_1_50_US  (uint8_t)(0x03)
#define GAP_CFG3_2LTOTIME_2_00_US  (uint8_t)(0x04)
#define GAP_CFG3_2LTOTIME_2_50_US  (uint8_t)(0x05)
#define GAP_CFG3_2LTOTIME_3_00_US  (uint8_t)(0x06)
#define GAP_CFG3_2LTOTIME_3_50_US  (uint8_t)(0x07)
#define GAP_CFG3_2LTOTIME_3_75_US  (uint8_t)(0x08)
#define GAP_CFG3_2LTOTIME_4_00_US  (uint8_t)(0x09)
#define GAP_CFG3_2LTOTIME_4_25_US  (uint8_t)(0x0A)
#define GAP_CFG3_2LTOTIME_4_50_US  (uint8_t)(0x0B)
#define GAP_CFG3_2LTOTIME_4_75_US  (uint8_t)(0x0C)
#define GAP_CFG3_2LTOTIME_5_00_US  (uint8_t)(0x0D)
#define GAP_CFG3_2LTOTIME_5_25_US  (uint8_t)(0x0E)
#define GAP_CFG3_2LTOTIME_5_50_US  (uint8_t)(0x0F)

#define GAP_CFG4_OVLO              (uint8_t)(0x20)
#define GAP_CFG4_UVLOLATCH         (uint8_t)(0x10)
#define GAP_CFG4_UVLOTH_VH_DISABLE (uint8_t)(0x00)
#define GAP_CFG4_UVLOTH_VH_10V     (uint8_t)(0x01)
#define GAP_CFG4_UVLOTH_VH_12V     (uint8_t)(0x02)
#define GAP_CFG4_UVLOTH_VH_14V     (uint8_t)(0x03)
#define GAP_CFG4_UVLOTH_VL_DISABLE (uint8_t)(0x00)
#define GAP_CFG4_UVLOTH_VL_N3V     (uint8_t)(0x04)
#define GAP_CFG4_UVLOTH_VL_N5V     (uint8_t)(0x08)
#define GAP_CFG4_UVLOTH_VL_N7V     (uint8_t)(0x0C)

#define GAP_CFG5_2LTO_ON_FAULT     (uint8_t)(0x08)
#define GAP_CFG5_CLAMP_EN          (uint8_t)(0x04)
#define GAP_CFG5_DESAT_EN          (uint8_t)(0x02)
#define GAP_CFG5_SENSE_EN          (uint8_t)(0x01)

#define GAP_STATUS1_OVLOH          (uint8_t)(0x80)
#define GAP_STATUS1_OVLOL          (uint8_t)(0x40)
#define GAP_STATUS1_DESAT          (uint8_t)(0x20)
#define GAP_STATUS1_SENSE          (uint8_t)(0x10)
#define GAP_STATUS1_UVLOH          (uint8_t)(0x08)
#define GAP_STATUS1_UVLOL          (uint8_t)(0x04)
#define GAP_STATUS1_TSD            (uint8_t)(0x02)
#define GAP_STATUS1_TWN            (uint8_t)(0x01)

#define GAP_STATUS2_REGERRR        (uint8_t)(0x04)
#define GAP_STATUS2_ASC            (uint8_t)(0x02)
#define GAP_STATUS2_GATE           (uint8_t)(0x01)

#define GAP_STATUS3_CFG            (uint8_t)(0x20)
#define GAP_STATUS3_DT_ERR         (uint8_t)(0x10)
#define GAP_STATUS3_SPI_ERR        (uint8_t)(0x08)
#define GAP_STATUS3_REGERRL        (uint8_t)(0x04)
#define GAP_STATUS3_OVLOD          (uint8_t)(0x02)
#define GAP_STATUS3_UVLOD          (uint8_t)(0x01)

#define GAP_TEST1_GOFFCHK          (uint8_t)(0x10)
#define GAP_TEST1_GONCHK           (uint8_t)(0x08)
#define GAP_TEST1_DESCHK           (uint8_t)(0x04)
#define GAP_TEST1_SNSCHK           (uint8_t)(0x02)
#define GAP_TEST1_RCHK             (uint8_t)(0x01)

#define GAP_DIAG_SPI_REGERR        (uint8_t)(0x80)
#define GAP_DIAG_UVLOD_OVLOD       (uint8_t)(0x40)
#define GAP_DIAG_UVLOH_UVLOL       (uint8_t)(0x20)
#define GAP_DIAG_OVLOH_OVLOL       (uint8_t)(0x10)
#define GAP_DIAG_DESAT_SENSE       (uint8_t)(0x08)
#define GAP_DIAG_ASC_DT_ERR        (uint8_t)(0x04)
#define GAP_DIAG_TSD               (uint8_t)(0x02)
#define GAP_DIAG_TWN               (uint8_t)(0x01)
#define GAP_DIAG_NONE              (uint8_t)(0x00)

/**
  * @}
  */

/** @defgroup GAP_class_private_enum GAP class private enum
  * @{
  */

typedef enum {
  CFG1 = 0x0C,
  CFG2 = 0x1D,
  CFG3 = 0x1E,
  CFG4 = 0x1F,
  CFG5 = 0x19,
  STATUS1 = 0x02,
  STATUS2 = 0x01,
  STATUS3 = 0x0A,
  TEST1 = 0x11,
  DIAG1 = 0x05,
  DIAG2 = 0x06
} GAP_Registers_t;

/**
  * @}
  */


/** @defgroup GAP_class_testModes GAP class test modes
  * @{
  */

typedef enum {
  SENSE_RESISTOR_CHK,
  SENSE_COMPARATOR_CHK,
  GON_CHK,
  GOFF_CHK,
  DESAT_CHK
} GAP_TestMode_t;

/**
  * @}
  */

/** @defgroup GAP_class_exported_types GAP class exported types
* @{
*/

/** 
  * @brief  Public GAP class definition
  */
typedef struct CGAP_GDC_t *CGAP_GDC;

/** 
  * @brief  GAP class parameters definition
  */
typedef const struct
{
  uint8_t bCFG1;  /*!< Configuration value for CFG1 register. */
  uint8_t bCFG2;  /*!< Configuration value for CFG2 register. */
  uint8_t bCFG3;  /*!< Configuration value for CFG3 register. */
  uint8_t bCFG4;  /*!< Configuration value for CFG4 register. */
  uint8_t bCFG5;  /*!< Configuration value for CFG5 register. */
  uint8_t bDIAG1; /*!< Configuration value for DIAG1 register. */
  uint8_t bDIAG2; /*!< Configuration value for DIAG2 register. */
} GAPDeviceParams_t, *pGAPDeviceParams_t;

typedef const struct
{
  uint8_t bDeviceNum; /*!< Number of GAP used in the daisy chain */
  pGAPDeviceParams_t deviceParams[MAX_DEVICES_NUMBER]; /*!< Device parameters pointers */
  
  /* SPI related parameters */
  SPI_TypeDef*  SPIx;     /*!< It contains the the SPI used for communication
                               with GAP devices. It must be SPI1 or SPI2. */
  uint32_t wSPIRemapping; /*!< It defines the SPI remapping and must equal to 
                                           GPIO_Remap_SPI1 or 
                                           GPIO_NoRemap_SPI */
  GPIO_TypeDef* hSCKPort; /*!< GPIO port used by SPI SCK. It must
                                           be equal to GPIOx x= A, B, ...*/
  uint16_t      hSCKPin;  /*!< GPIO pin used by SPI SCK. It must be
                                          equal to GPIO_Pin_x x= 0, 1, ...*/
  GPIO_TypeDef* hMISOPort; /*!< GPIO port used by SPI MISO. It must
                                           be equal to GPIOx x= A, B, ...*/
  uint16_t      hMISOPin; /*!< GPIO pin used by SPI MISO. It must be 
                                          equal to GPIO_Pin_x x= 0, 1, ...*/
  GPIO_TypeDef* hMOSIPort; /*!< GPIO port used by SPI MOSI. It must
                                           be equal to GPIOx x= A, B, ...*/
  uint16_t      hMOSIPin; /*!< GPIO pin used by SPI MOSI. It must be 
                                          equal to GPIO_Pin_x x= 0, 1, ...*/
  GPIO_TypeDef* hNCSPort; /*!< GPIO port used by NCS. It must
                                           be equal to GPIOx x= A, B, ...*/
  uint16_t      hNCSPin;  /*!< GPIO pin used by NCS. It must be 
                                          equal to GPIO_Pin_x x= 0, 1, ...*/
  GPIO_TypeDef* hNSDPort; /*!< GPIO port used by NSD. It must
                                           be equal to GPIOx x= A, B, ...*/
  uint16_t      hNSDPin;  /*!< GPIO pin used by NSD. It must be 
                                          equal to GPIO_Pin_x x= 0, 1, ...*/
} GAPParams_t, *pGAPParams_t;
/**
  * @}
  */

/** @defgroup GAP_class_exported_methods GAP class exported methods
  * @{
  */
  
/**
  * @brief  Creates an object of the class GAP
  * @param  pGateDriverCtrlParams pointer to an GateDriverCtrl parameters structure
  * @param  pGAPParams pointer to an GAP parameters structure
  * @retval CGAP_GDC new instance of GAP object
  */
CGAP_GDC GAP_NewObject(pGateDriverCtrlParams_t pGateDriverCtrlParams, pGAPParams_t pGAPParams);

/**
  * @brief  Check errors of GAP devices
  * @param  this related object of class CGAP_GDC
  * @param  errorNow buffer of returned bitfields containing error flags 
  *         coming from GAP device that are currently active.\n
  *         \link GAP_class_error_code Code errors\endlink.\n
  *         The buffer have to be provided from the caller.
  * @param  errorOccurred buffer of returned bitfields containing error flags 
  *         coming from GAP device that are over.\n
  *         \link GAP_class_error_code Code errors\endlink.\n
  *         The buffer have to be provided from the caller.
  * @retval bool It returns false if an error occurs, otherwise return true.
  */
bool GAP_CheckErrors(CGDC this,uint32_t* errorNow,uint32_t* errorOccurred);

/**
  * @brief  Clears the fault state of GAP devices.
  * @param  this related object of class CGDC.
  * @retval none.
  */
void GAP_FaultAck(CGDC this);

/**
  * @brief  Programs the GAP devices with the settled parameters.
  * @param  this related object of class CGDC.
  * @retval bool It returns false if an error occurs, otherwise return true.
  */
bool GAP_Configuration(CGDC this);

/**
  * @brief  Read all data in the daisy chain related to register reg.
  * @param  this related object of class CGDC
  * @param  pDataRead Pointer to the buffer in which will be stored the readed
  *         data. The buffer have to be provided from the caller.
  * @param  reg Register to be read. It must be one of the register defined in
  *         \link GAP_class_private_enum GAP_Registers_t\endlink.
  * @retval bool It returns false if an error occurs, otherwise return true.
  */
bool GAP_ReadRegs(CGDC this, uint8_t* pDataRead, GAP_Registers_t reg);

void GAP_StartConfig(CGDC this);
void GAP_StopConfig(CGDC this);

/**
  * @brief  Write data in the daisy chain in to the register reg.
  * @param  this related object of class CGDC
  * @param  pDataWrite Pointer to the buffer in which are stored the data
  *         to be written in the register reg. The buffer have to be provided 
  *         from the caller.
  * @param  reg Register to be write. It must be one of the register defined in
  *         \link GAP_class_private_enum GAP_Registers_t\endlink.
  * @retval bool It returns false if an error occurs, otherwise return true.
  */
bool GAP_WriteRegs(CGDC this, uint8_t* pDataWrite, GAP_Registers_t reg);

/**
  * @brief  ** This function should be called just in IDLE state. **\n
  *         It performs the selected test on each GAP devices.
  * @param  this related object of class CGDC
  * @param  testMode Test mode to be executed. It shall be one of the
  *         test modes present in the
  *         \link GAP_class_testModes GAP class test modes\endlink.
  * @retval bool It returns TRUE if an error occurs, otherwise return FALSE.
  */
bool GAP_Test(CGDC this, GAP_TestMode_t testMode);

/**
  * @brief  Reset all the registers to the default and releases all the failure flag.
  * @param  this related object of class CGDC.
  * @retval none.
  */
void GAP_GlobalReset(CGDC this);
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__GAP_GATEDRIVERCTRLCLASS_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
