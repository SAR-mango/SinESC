/**
  ******************************************************************************
  * @file    GAP_GateDriverCtrlClass.c
  * @author  IMS Systems Lab and Technical Marketing - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private impelementation of GAP class      
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
#include "GateDriverCtrlClass.h"
#include "GateDriverCtrlPrivate.h"
#include "GAP_GateDriverCtrlClass.h"
#include "GAP_GateDriverCtrlPrivate.h"
#include "MCLibraryConf.h"
#include "MC_type.h"

#ifdef MC_CLASS_DYNAMIC
#include "stdlib.h" /* Used for dynamic allocation */
#else
_DCGAP_GDC_t GAP_GDCpool[MAX_GAP_GDC_NUM];
unsigned char GAP_GDC_Allocated = 0u;
#endif

#define DCLASS_PARAMS ((_DCGAP_GDC)(((_CGDC) this)->DerivedClass))->pDParams_str
#define DCLASS_VARS  &(((_DCGAP_GDC)(((_CGDC) this)->DerivedClass))->DVars_str)
#define CLASS_VARS   &(((_CGDC)this)->Vars_str)
#define CLASS_PARAMS  (((_CGDC)this)->pParams_str)

/** @defgroup GAP_class_private_defines GAP class private defines
  * @{
  */

#define GAP_STARTCONFIG 0x2A
#define GAP_STOPCONFIG  0x3A
#define GAP_WRITEREG    0x80
#define GAP_READREG     0xA0
#define GAP_RESETREG    0xC0
#define GAP_RESETSTATUS 0xD0
#define GAP_GLOBALRESET 0xEA
#define GAP_SLPEEP      0xF5
#define GAP_NOP         0x00

#define CFG1_REG_MASK    (uint8_t)(0xFF)
#define CFG2_REG_MASK    (uint8_t)(0xFF)
#define CFG3_REG_MASK    (uint8_t)(0xFF)
#define CFG4_REG_MASK    (uint8_t)(0x3F)
#define CFG5_REG_MASK    (uint8_t)(0x0F)
#define STATUS1_REG_MASK (uint8_t)(0xFF)
#define STATUS2_REG_MASK (uint8_t)(0x07)
#define STATUS3_REG_MASK (uint8_t)(0x1F) // It should be 0x3F
#define TEST1_REG_MASK   (uint8_t)(0x1F)
#define DIAG1_REG_MASK   (uint8_t)(0xFF)
#define DIAG2_REG_MASK   (uint8_t)(0xFF)

#define GAP_ERROR_CODE_FROM_DEVICE_MASK (uint32_t)(0xFF000000)

#define WAITTIME      5000 // 860u
#define WAITTRCHK     50   // 8.6u
#define WAITTSENSECHK 50   // 8.6u
#define WAITTGCHK     50   // 8.6u
#define WAITTDESATCHK 50   // 8.6u

/**
  * @}
  */
    
/** @defgroup GAP_class_private_functions GAP class private functions
  * @{
  */

static uint16_t GAP_CRCCalculate(uint8_t data, uint8_t crcInitialValue);
static bool GAP_CRCCheck(uint8_t* out, uint16_t dataIn);
static void wait(uint16_t count);
static uint8_t GAP_regMask(GAP_Registers_t reg);

/**
  * @brief  Checks if the GAP devices are programmed with the settled parameters.
  * @param  this related object of class CGDC.
  * @retval True if the GAP devices are already programmed with the settled 
  *         parameters.
  */
bool GAP_IsDevicesProgrammed(CGDC this);

/**
  * @brief  Programs the GAP devices with the settled parameters.
  * @param  this related object of class CGDC.
  * @retval bool It returns false if an error occurs, otherwise return true.
  */
bool GAP_DevicesConfiguration(CGDC this);

/**
  * @brief  Reset selected status register.
  * @param  this related object of class CGAP_GDC
  * @param  reg Register to be reset. It must be one of the STATUS register 
   *        defined in \link GAP_class_private_enum GAP_Registers_t\endlink.
  * @retval bool It returns false if an error occurs, otherwise return true.
  */
bool GAP_ResetStatus(CGDC this, GAP_Registers_t reg);
    
/**
  * @}
  */

volatile uint16_t waitCnt;

/**
  * @brief  Creates an object of the class GAP
  * @param  pGateDriverCtrlParams pointer to an GateDriverCtrl parameters structure
  * @param  pGAPParams pointer to an GAP parameters structure
  * @retval CGAP_GDC new instance of GAP object
  */
CGAP_GDC GAP_NewObject(pGateDriverCtrlParams_t pGateDriverCtrlParams, pGAPParams_t pGAPParams)
{
	_CGDC _oGateDriverCtrl;
	_DCGAP_GDC _oGAP;
	int i,l;

	_oGateDriverCtrl = (_CGDC)GDC_NewObject(pGateDriverCtrlParams);

	#ifdef MC_CLASS_DYNAMIC
		_oGAP = (_DCGAP_GDC)calloc(1u,sizeof(_DCGAP_GDC_t));
	#else
		if (GAP_GDC_Allocated  < MAX_GAP_GDC_NUM)
		{
			_oGAP = &GAP_GDCpool[GAP_GDC_Allocated++];
		}
		else
		{
			_oGAP = MC_NULL;
		}
	#endif
  
	_oGAP->pDParams_str = pGAPParams;
	_oGateDriverCtrl->DerivedClass = (void*)_oGAP;
  
	_oGateDriverCtrl->Methods_str.pGDC_Init = &GAP_Init;
        
        /* Init of private data */
        l = pGAPParams->bDeviceNum;
        for (i = 0; i < l; i++)
        {
          _oGAP->DVars_str.wGAP_ErrorsNow[i] = GAP_ERROR_CLEAR;
          _oGAP->DVars_str.wGAP_ErrorsOccurred[i] = GAP_ERROR_CLEAR;
        }
        
	return ((CGAP_GDC)_oGateDriverCtrl);
}

/** @addtogroup STM32F10x_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup GateDriverCtrl_GAP
  * @{
  */

/** @defgroup GAP_class_private_methods GAP class private methods
* @{
*/

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
bool GAP_CheckErrors(CGDC this,uint32_t* errorNow,uint32_t* errorOccurred)
{
  uint32_t errorFromDevices[MAX_DEVICES_NUMBER];
  bool retVal = FALSE;
  if ((errorNow)&&(errorOccurred))
  {
    pDVars_t pDVars = DCLASS_VARS;
    pDParams_t pDParams = DCLASS_PARAMS;
    uint8_t i,l;
    uint8_t regRead[MAX_DEVICES_NUMBER];
    
//    /* If current error is device not programmable try to re-configure before
//       read the status registers */
//    if ((pDVars->wGAP_ErrorsNow[0] & GAP_ERROR_CODE_DEVICES_NOT_PROGRAMMABLE) ||
//        (pDVars->wGAP_ErrorsNow[0] & GAP_ERROR_CODE_SPI_CRC))
//    {
//      if (GAP_Configuration(this))
//      {
//        pDVars->wGAP_ErrorsNow[0] &= ~GAP_ERROR_CODE_DEVICES_NOT_PROGRAMMABLE;
//        pDVars->wGAP_ErrorsNow[0] &= ~GAP_ERROR_CODE_SPI_CRC;
//      }
//    }
    
    
    l = pDParams->bDeviceNum;
    retVal = GAP_ReadRegs(this,regRead,STATUS1);
    for (i = 0; i < l; i++)
    {
      errorFromDevices[i] = (regRead[i] << 16);
    }
    retVal = GAP_ReadRegs(this,regRead,STATUS2);
    for (i = 0; i < l; i++)
    {
      /* Clear GATE bit from STATUS2 - no error if 1 */
      regRead[i] &= 0xFE;
      errorFromDevices[i] |= (regRead[i] << 8);
    }
    retVal = GAP_ReadRegs(this,regRead,STATUS3);
    for (i = 0; i < l; i++)
    {
      errorFromDevices[i] |= regRead[i];
    }
        
    for (i = 0; i < l; i++)
    {
      pDVars->wGAP_ErrorsNow[i] &= GAP_ERROR_CODE_FROM_DEVICE_MASK;
      pDVars->wGAP_ErrorsNow[i] |= errorFromDevices[i];
      pDVars->wGAP_ErrorsOccurred[i] |= pDVars->wGAP_ErrorsNow[i];
      errorNow[i] = pDVars->wGAP_ErrorsNow[i];
      errorOccurred[i] = pDVars->wGAP_ErrorsOccurred[i];
    }
  }
  return retVal;
}

/**
  * @brief  Clears the fault state of GAP devices.
  * @param  this related object of class CGDC.
  * @retval none.
  */
void GAP_FaultAck(CGDC this)
{
  uint8_t i,l;
  uint16_t value;
  pDParams_t pDParams = DCLASS_PARAMS;
  pDVars_t pDVars = DCLASS_VARS;
  GAP_SD_Activate(this);
  value = GAP_CRCCalculate(GAP_RESETSTATUS, 0xFF);
  GAP_CS_Activate(this);
  l = pDParams->bDeviceNum;
  for (i = 0; i < l; i++)
  {
    GAP_SPI_Send(this, value);
  }
  GAP_CS_Deactivate(this);
  GAP_SD_Deactivate(this);
  wait(WAITTIME);      
  for (i = 0; i < l; i++)
  {
    pDVars->wGAP_ErrorsOccurred[i] = GAP_ERROR_CLEAR;
  }
}

/**
  * @brief  Programs the GAP devices with the settled parameters.
  * @param  this related object of class CGDC.
  * @retval bool It returns false if at least one device is not programmable,
  *         otherwise return true.
  */
bool GAP_Configuration(CGDC this)
{
  bool retVal;
  
  /* Configure devices with settled parameters */
  GAP_DevicesConfiguration(this);
  
  /* Verify if device has been programmed */
  retVal = GAP_IsDevicesProgrammed(this);
  
  if (!retVal)
  {
    /* At least one device is not programmable */
    pDVars_t pDVars = DCLASS_VARS;
    pDVars->wGAP_ErrorsNow[0] |= GAP_ERROR_CODE_DEVICES_NOT_PROGRAMMABLE;
    pDVars->wGAP_ErrorsOccurred[0] |= GAP_ERROR_CODE_DEVICES_NOT_PROGRAMMABLE;
  }
  return retVal;
}

/**
  * @brief  Checks if the GAP devices are programmed with the settled parameters.
  * @param  this related object of class CGDC.
  * @retval True if the GAP devices are already programmed with the settled 
  *         parameters.
  */
bool GAP_IsDevicesProgrammed(CGDC this)
{
  bool retVal = TRUE;
  pDParams_t pDParams = DCLASS_PARAMS;
  uint8_t i, l = pDParams->bDeviceNum;
  uint8_t readRegValues[MAX_DEVICES_NUMBER];
  GAP_ReadRegs(this,readRegValues,CFG1);
  for (i = 0; i < l; i++)
  {
    if ((pDParams->deviceParams[i]->bCFG1 & CFG1_REG_MASK) != readRegValues[i])
    {
      retVal = FALSE;
      break;
    }
  }
  if (retVal)
  {
    GAP_ReadRegs(this,readRegValues,CFG2);
    for (i = 0; i < l; i++)
    {
      if ((pDParams->deviceParams[i]->bCFG2 & CFG2_REG_MASK) != readRegValues[i])
      {
        retVal = FALSE;
        break;
      }
    }
  }
  if (retVal)
  {
    GAP_ReadRegs(this,readRegValues,CFG3);
    for (i = 0; i < l; i++)
    {
      if ((pDParams->deviceParams[i]->bCFG3 & CFG3_REG_MASK) != readRegValues[i])
      {
        retVal = FALSE;
        break;
      }
    }
  }
  if (retVal)
  {
    GAP_ReadRegs(this,readRegValues,CFG4);
    for (i = 0; i < l; i++)
    {
      if ((pDParams->deviceParams[i]->bCFG4 & CFG4_REG_MASK) != readRegValues[i])
      {
        retVal = FALSE;
        break;
      }
    }
  }
  if (retVal)
  {
    GAP_ReadRegs(this,readRegValues,CFG5);
    for (i = 0; i < l; i++)
    {
      if ((pDParams->deviceParams[i]->bCFG5 & CFG5_REG_MASK) != readRegValues[i])
      {
        retVal = FALSE;
        break;
      }
    }
  }
  if (retVal)
  {
    GAP_ReadRegs(this,readRegValues,DIAG1);
    for (i = 0; i < l; i++)
    {
      if ((pDParams->deviceParams[i]->bDIAG1 & DIAG1_REG_MASK)  != readRegValues[i])
      {
        retVal = FALSE;
        break;
      }
    }
  }
  if (retVal)
  {
    GAP_ReadRegs(this,readRegValues,DIAG2);
    for (i = 0; i < l; i++)
    {
      if ((pDParams->deviceParams[i]->bDIAG2 & DIAG2_REG_MASK) != readRegValues[i])
      {
        retVal = FALSE;
        break;
      }
    }
  }
  return retVal;
}
  
/**
  * @brief  Programs the GAP devices with the settled parameters.
  * @param  this related object of class CGDC.
  * @retval bool It returns false if an error occurs, otherwise return true.
  */
bool GAP_DevicesConfiguration(CGDC this)
{
  bool retVal = TRUE;
  pDParams_t pDParams = DCLASS_PARAMS;
  uint8_t i, l = pDParams->bDeviceNum;
  uint8_t writeRegValues[MAX_DEVICES_NUMBER];
  
  GAP_StartConfig(this);
  
  /* Global Reset before programming */
  GAP_GlobalReset(this);
  
  for (i = 0; i < l; i++)
  {
    writeRegValues[i] = pDParams->deviceParams[i]->bCFG1;
  }
  retVal = GAP_WriteRegs(this,writeRegValues,CFG1);
  
  if (retVal)
  {
    for (i = 0; i < l; i++)
    {
      writeRegValues[i] = pDParams->deviceParams[i]->bCFG2;
    }
    retVal = GAP_WriteRegs(this,writeRegValues,CFG2);
  }
  
  if (retVal)
  {
    for (i = 0; i < l; i++)
    {
      writeRegValues[i] = pDParams->deviceParams[i]->bCFG3;
    }
    retVal = GAP_WriteRegs(this,writeRegValues,CFG3);
  }
  if (retVal)
  {
    for (i = 0; i < l; i++)
    {
      writeRegValues[i] = pDParams->deviceParams[i]->bCFG4;
    }
    retVal = GAP_WriteRegs(this,writeRegValues,CFG4);
  }
  if (retVal)
  {
    for (i = 0; i < l; i++)
    {
      writeRegValues[i] = pDParams->deviceParams[i]->bCFG5;
    }
    retVal = GAP_WriteRegs(this,writeRegValues,CFG5);
  }
  if (retVal)
  {
    for (i = 0; i < l; i++)
    {
      writeRegValues[i] = pDParams->deviceParams[i]->bDIAG1;
    }
    retVal = GAP_WriteRegs(this,writeRegValues,DIAG1);
  }
  if (retVal)
  {
    for (i = 0; i < l; i++)
    {
      writeRegValues[i] = pDParams->deviceParams[i]->bDIAG2;
    }
    retVal = GAP_WriteRegs(this,writeRegValues,DIAG2);
  }
  
  GAP_StopConfig(this);
  
  /* Fault reset */
  GAP_FaultAck(this);
  
  return retVal;
}

/**
  * @brief  Calculate CRC from data and create 16bit value with data as MSB and
  *         CRC as LSB.
  * @param  data 8bit value used to calculate CRC.
  * @retval uint16_t It returns the 16bit value with data as MSB and
  *         CRC as LSB.
  */
uint16_t GAP_CRCCalculate(uint8_t data, uint8_t crcInitialValue)
{
  uint8_t crc =  crcInitialValue;
  uint8_t poly = 0x07;
  uint8_t crc_temp;
  uint8_t crctab[8];
  uint8_t i,j;
  
  uint16_t value;
  value = data;
  value <<= 8;
  
  for (j=0; j<8; j++) {crctab[j] = (crc >> j) & 0x1;}
  for (j=0; j<8; j++)
  {
    crc_temp = (crctab[7] << 7) + (crctab[6] << 6) + (crctab[5] << 5) + (crctab[4] << 4) + (crctab[3] << 3) + (crctab[2] << 2) + (crctab[1] << 1) + (crctab[0]);
    crctab[0] = ((data >> (7-j)) & 0x1 ) ^ crctab[7];
    for (i=1; i<8; i++)
    {
      crctab[i] = (crctab[0] & ((poly >> i)&0x1)) ^ ((crc_temp>>(i-1))&0x1);
    }
  }
  crc = (crctab[7] << 7) + (crctab[6] << 6) + (crctab[5] << 5) + (crctab[4] << 4) + (crctab[3] << 3) + (crctab[2] << 2) + (crctab[1] << 1) + (crctab[0] << 0);
  crc ^= 0xFF;
  
  value |= crc;
  return value;
}

/**
  * @brief  Verify the CRC from dataIn and extract the 8bit data value (out).
  * @param  out Reference for the extracted 8bit data value.
  * @param  dataIn 16bit value with data as MSB and CRC as LSB.
  * @retval bool It returns TRUE if CRC is correct, FALSE otherwise.
  */
bool GAP_CRCCheck(uint8_t* out, uint16_t dataIn)
{
  bool retVal = FALSE;
  uint8_t data = (uint8_t)(dataIn >> 8);
  uint8_t receivedCRC = (uint8_t)(dataIn);
  uint8_t crc = (uint8_t)(GAP_CRCCalculate(data,0xFF)) ^ 0xFF;
  if (crc == receivedCRC)
  {
    *out = data;
    retVal = TRUE;
  }
  return retVal;
}

/**
  * @brief  Wait a time interval proportional to count.
  * @param  count Number of count to be waited
  * @retval none
  */
void wait(uint16_t count)
{
  for (waitCnt = 0; waitCnt < count; waitCnt++)
  {
  }
}

/**
  * @brief  Return the register mask starting from it address.
  * @param  reg Register address.
  * @retval uint8_t Mask to be and-ed bit wise to data to filter it.
  */
uint8_t GAP_regMask(GAP_Registers_t reg)
{
  uint8_t retVal;
  switch (reg)
  {
  case CFG1:
    {
      retVal = CFG1_REG_MASK;
    }
    break;
  case CFG2:
    {
      retVal = CFG2_REG_MASK;
    }
    break;
  case CFG3:
    {
      retVal = CFG3_REG_MASK;
    }
    break;
  case CFG4:
    {
      retVal = CFG4_REG_MASK;
    }
    break;
  case CFG5:
    {
      retVal = CFG5_REG_MASK;
    }
    break;
  case STATUS1:
    {
      retVal = STATUS1_REG_MASK;
    }
    break;
  case STATUS2:
    {
      retVal = STATUS2_REG_MASK;
    }
    break;
  case STATUS3:
    {
      retVal = STATUS3_REG_MASK;
    }
    break;
  case TEST1:
    {
      retVal = TEST1_REG_MASK;
    }
    break;
  case DIAG1:
    {
      retVal = DIAG1_REG_MASK;
    }
    break;
  case DIAG2:
    {
      retVal = DIAG2_REG_MASK;
    }
    break;
  default:
    {
      retVal = 0x00;
    }
    break;
  }
  return retVal;
}

uint8_t device;
uint8_t dataReceived;

/**
  * @brief  Read all data in the daisy chain related to register reg.
  * @param  this related object of class CGAP_GDC
  * @param  pDataRead Pointer to the buffer in which will be stored the readed
  *         data. The buffer have to be provided from the caller.
  * @param  reg Register to be read. It must be one of the register defined in
  *         \link GAP_class_private_enum GAP_Registers_t\endlink.
  * @retval bool It returns false if an error occurs, otherwise return true.
  */
bool GAP_ReadRegs(CGDC this, uint8_t* pDataRead, GAP_Registers_t reg)
{
  pDParams_t pDParams = DCLASS_PARAMS;
  pDVars_t pDVars = DCLASS_VARS;
  bool retVal = FALSE;
  if (pDataRead)
  {
    uint16_t value;
		uint8_t i;
    value = GAP_CRCCalculate(GAP_READREG | reg, 0xFF);
    GAP_CS_Activate(this);
    for (i = 0; i < pDParams->bDeviceNum; i++)
    {
      GAP_SPI_Send(this, value);
    }
    GAP_CS_Deactivate(this);
    wait(WAITTIME);
    value = GAP_CRCCalculate(GAP_NOP, 0xFF);
    GAP_CS_Activate(this);
    retVal = TRUE;
    for (i = 0; i < pDParams->bDeviceNum; i++)
    {
      device = pDParams->bDeviceNum - i - 1;
      if (pDParams->deviceParams[device]->bCFG1 & GAP_CFG1_CRC_SPI)
      {
        uint8_t data;
        if (GAP_CRCCheck(&data,GAP_SPI_Send(this, value)))
        {
          pDataRead[device] = data & GAP_regMask(reg);
        }
        else
        {
          pDataRead[device] = 0x00;
          pDVars->wGAP_ErrorsNow[0] |= GAP_ERROR_CODE_SPI_CRC;
          pDVars->wGAP_ErrorsOccurred[0] |= GAP_ERROR_CODE_SPI_CRC;
          retVal = FALSE;
        }
      }
      else
      {
        dataReceived = (uint8_t)(GAP_SPI_Send(this, value) >> 8) & GAP_regMask(reg);
        pDataRead[device] = dataReceived;
      }
    }
    GAP_CS_Deactivate(this);
  }
  return retVal;
}

void GAP_StartConfig(CGDC this)
{
  pDParams_t pDParams = DCLASS_PARAMS;
  uint16_t value;
  uint8_t i;
  
  GAP_SD_Activate(this);
  
  value = GAP_CRCCalculate(GAP_STARTCONFIG, 0xFF);
  GAP_CS_Activate(this);
  
  for (i = 0; i < pDParams->bDeviceNum; i++)
  {
    GAP_SPI_Send(this, value);
  }
  GAP_CS_Deactivate(this);
  wait(WAITTIME);
}

void GAP_StopConfig(CGDC this)
{
  pDParams_t pDParams = DCLASS_PARAMS;
  uint16_t value;
  uint8_t i;
  
  value = GAP_CRCCalculate(GAP_STOPCONFIG, 0xFF);
  GAP_CS_Activate(this);
  for (i = 0; i < pDParams->bDeviceNum; i++)
  {
    GAP_SPI_Send(this, value);
  }
  GAP_CS_Deactivate(this);
  wait(WAITTIME);
  
  GAP_SD_Deactivate(this);
}
  
/**
  * @brief  Write data in the daisy chain in to the register reg.
  * @param  this related object of class CGAP_GDC
  * @param  pDataWrite Pointer to the buffer in which are stored the data
  *         to be written in the register reg. The buffer have to be provided 
  *         from the caller.
  * @param  reg Register to be write. It must be one of the register defined in
  *         \link GAP_class_private_enum GAP_Registers_t\endlink.
  * @retval bool It returns false if an error occurs, otherwise return true.
  */
bool GAP_WriteRegs(CGDC this, uint8_t* pDataWrite, GAP_Registers_t reg)
{
  pDParams_t pDParams = DCLASS_PARAMS;
  bool retVal = FALSE;
  if (pDataWrite)
  {
    uint8_t i;
    uint16_t value;
    uint8_t crc;
    
    value = GAP_CRCCalculate(GAP_WRITEREG | reg, 0xFF);
    crc = (uint8_t)(value);
    GAP_CS_Activate(this);
    for (i = 0; i < pDParams->bDeviceNum; i++)
    {
      GAP_SPI_Send(this, value);
    }
    GAP_CS_Deactivate(this);
    wait(WAITTIME);
    
    GAP_CS_Activate(this);
    retVal = TRUE;
    for (i = 0; i < pDParams->bDeviceNum; i++)
    {
      value = GAP_CRCCalculate(pDataWrite[i], crc ^ 0xFF);
      GAP_SPI_Send(this, value);
    }
    GAP_CS_Deactivate(this);
    wait(WAITTIME);
    
  }
  return retVal;
}

/**
  * @brief  Reset all the registers to the default and releases all the failure flag.
  * @param  this related object of class CGDC.
  * @retval none.
  */
void GAP_GlobalReset(CGDC this)
{
  pDParams_t pDParams = DCLASS_PARAMS;
  uint8_t i;
  uint16_t value;
  value = GAP_CRCCalculate(GAP_GLOBALRESET, 0xFF);
  GAP_CS_Activate(this);
  for (i = 0; i < pDParams->bDeviceNum; i++)
  {
    GAP_SPI_Send(this, value);
  }
  GAP_CS_Deactivate(this);
  wait(WAITTIME);
}

/**
  * @brief  Reset selected status register.
  * @param  this related object of class CGAP_GDC
  * @param  reg Register to be reset. It must be one of the STATUS register 
   *        defined in \link GAP_class_private_enum GAP_Registers_t\endlink.
  * @retval bool It returns false if an error occurs, otherwise return true.
  */
bool GAP_ResetStatus(CGDC this, GAP_Registers_t reg)
{
  pDParams_t pDParams = DCLASS_PARAMS;
  bool retVal = FALSE;
  uint8_t i;
  uint16_t value;
  GAP_SD_Activate(this);
    
  value = GAP_CRCCalculate(GAP_RESETREG | reg, 0xFF);
  GAP_CS_Activate(this);
  for (i = 0; i < pDParams->bDeviceNum; i++)
  {
    GAP_SPI_Send(this, value);
  }
  GAP_CS_Deactivate(this);
  
  GAP_SD_Deactivate(this);
  return retVal;
}

/**
  * @brief  ** This function should be called just in IDLE state. **\n
  *         It performs the selected test on each GAP devices.
  * @param  this related object of class CGDC
  * @param  testMode Test mode to be executed. It shall be one of the
  *         test modes present in the
  *         \link GAP_class_testModes GAP class test modes\endlink.
  * @retval bool It returns TRUE if an error occurs, otherwise return FALSE.
  */
bool GAP_Test(CGDC this, GAP_TestMode_t testMode)
{
  bool retVal = FALSE;
  pDParams_t pDParams = DCLASS_PARAMS;
  uint8_t testModeData;
  uint8_t clr[MAX_DEVICES_NUMBER];
  uint8_t data[MAX_DEVICES_NUMBER];
  uint8_t i, l = pDParams->bDeviceNum;
  uint16_t waitCnt;
  uint8_t statusMask;
  bool invertResult = FALSE;
  
  switch (testMode)
  {
  case GOFF_CHK:
    {
      testModeData = GAP_TEST1_GOFFCHK;
      waitCnt = WAITTGCHK;
      statusMask = GAP_STATUS1_DESAT;
    }
    break;
  case GON_CHK:
    {
      testModeData = GAP_TEST1_GONCHK;
      waitCnt = WAITTGCHK;
      statusMask = GAP_STATUS1_TSD;
    }
    break;
  case GAP_TEST1_DESCHK:
    {
      testModeData = DESAT_CHK;
      waitCnt = WAITTDESATCHK;
      statusMask = GAP_STATUS1_DESAT;
      invertResult = TRUE;
    }
    break;
  case SENSE_RESISTOR_CHK:
    {
      testModeData = GAP_TEST1_RCHK;
      waitCnt = WAITTRCHK;
      statusMask = GAP_STATUS1_SENSE;
    }
    break;
  case SENSE_COMPARATOR_CHK:
    {
      testModeData = GAP_TEST1_SNSCHK;
      waitCnt = WAITTSENSECHK;
      statusMask = GAP_STATUS1_SENSE;
      invertResult = TRUE;
    }
    break;
  default:
    {
      // Error
      retVal = TRUE;
    }
    break;
  }
  
  for (i = 0; i < l; i++)
  {
    data[i] = testModeData;
    clr[i] = 0x00;
  }
  GAP_WriteRegs(this, data, TEST1);
  wait(waitCnt);
  GAP_ReadRegs(this, data, STATUS1);
  /* Clear TEST1 regs */
  GAP_WriteRegs(this, clr, TEST1);
  /* Clear STATUS1 regs */
  GAP_ResetStatus(this, STATUS1);
  
  for (i = 0; i < l; i++)
  {
    if (invertResult)
    {
      if ((data[i] & statusMask) == 0)
      {
        retVal = TRUE;
      }
    }
    else
    {
      if ((data[i] & statusMask) != 0)
      {
        retVal = TRUE;
      }
    }
  }
  
  return retVal;
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
