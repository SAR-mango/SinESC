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

#define DCLASS_PARAMS ((_DCGAP_GDC)(((_CGDC) this)->DerivedClass))->pDParams_str
#define DCLASS_VARS  &(((_DCGAP_GDC)(((_CGDC) this)->DerivedClass))->DVars_str)
#define CLASS_VARS   &(((_CGDC)this)->Vars_str)
#define CLASS_PARAMS  (((_CGDC)this)->pParams_str)

static uint16_t GPIOPin2Source(uint16_t GPIO_Pin);

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
* @brief  It initializes SPI and GPIO required by GAP driver
* @param  this: related object of class CGDC
* @retval none
*/
void GAP_Init(CGDC this)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef   SPI_InitStructure;
  uint8_t bGPIOAF;
  pDParams_t pDParams = DCLASS_PARAMS;
  
  // Enable GPIOA-GPIOF clock to be removed at the end because is done inside PWMC Classes*/
  RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA | 
                         RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC | 
                           RCC_AHBPeriph_GPIOD | RCC_AHBPeriph_GPIOE | 
                             RCC_AHBPeriph_GPIOF, ENABLE);
  
  /* Peripheral clocks enabling ----------------------------------------------*/
  if (pDParams->SPIx == SPI1)
  {
    /* Enable SPI1 clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    bGPIOAF = GPIO_AF_5; // SPI1 mapped only in AF5
  }
  else if (pDParams->SPIx == SPI2)
  {
    /* Enable SPI2 clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    // SPI2 is mapped in AF5 or AF6. AF5 default PB13 SCK PB15 MISO PB15 MOSI. Note !!!!!  AF6 if SCK on PF9 or PF10 to be changed manually below
    bGPIOAF = GPIO_AF_5;
  }
  else if (pDParams->SPIx == SPI3)
  {
    /* Enable SPI3 clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
    bGPIOAF = GPIO_AF_6; // SPI3 mapped only in AF6
  }
  else
  {
    /* Runtime error */
  }
  
  /* GPIOs configurations ----------------------------------------------------*/
  GPIO_StructInit(&GPIO_InitStructure);
  
  /****** SPI alternate function remapping ******/  
  GPIO_PinAFConfig(pDParams->hSCKPort, GPIOPin2Source(pDParams->hSCKPin), bGPIOAF);
  GPIO_PinAFConfig(pDParams->hMISOPort, GPIOPin2Source(pDParams->hMISOPin), bGPIOAF);
  GPIO_PinAFConfig(pDParams->hMOSIPort, GPIOPin2Source(pDParams->hMOSIPin), bGPIOAF);
  
  /****** Configure SPI SCK GPIO as AF ****/
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = pDParams->hSCKPin;  
  GPIO_Init(pDParams->hSCKPort, &GPIO_InitStructure);
  GPIO_PinLockConfig(pDParams->hSCKPort, pDParams->hSCKPin);
  
  /****** Configure SPI MISO GPIO as AF ****/
  GPIO_InitStructure.GPIO_Pin = pDParams->hMISOPin;  
  GPIO_Init(pDParams->hMISOPort, &GPIO_InitStructure);
  GPIO_PinLockConfig(pDParams->hMISOPort, pDParams->hMISOPin);
  
  /****** Configure SPI MOSI GPIO as AF ****/
  GPIO_InitStructure.GPIO_Pin = pDParams->hMOSIPin;  
  GPIO_Init(pDParams->hMOSIPort, &GPIO_InitStructure);
  GPIO_PinLockConfig(pDParams->hMOSIPort, pDParams->hMOSIPin);
  
  /****** Configure NCS GPIO as Out PP ****/
  GAP_CS_Deactivate(this);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Pin = pDParams->hNCSPin;  
  GPIO_Init(pDParams->hNCSPort, &GPIO_InitStructure);
  GPIO_PinLockConfig(pDParams->hNCSPort, pDParams->hNCSPin);
  
  /****** Configure NSD GPIO as Out PP ****/
  GAP_SD_Deactivate(this);
  GPIO_InitStructure.GPIO_Pin = pDParams->hNSDPin;  
  GPIO_Init(pDParams->hNSDPort, &GPIO_InitStructure);
  GPIO_PinLockConfig(pDParams->hNSDPort, pDParams->hNSDPin);
  
  /* SPI Config --------------------------------------------------------------*/
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(pDParams->SPIx, &SPI_InitStructure);
  
  /* Enable SPI */
  SPI_Cmd(pDParams->SPIx, ENABLE);
}

/**
* @brief  This function sends a 16bit value through the configured SPI and 
*         returns the 16-bit value received during communication.
* @param  this: related object of class CGDC
* @param  value: Value to be sent through SPI
* @retval uint16_t Received 16bit value
*/
uint16_t GAP_SPI_Send(CGDC this, uint16_t value)
{
  pDParams_t pDParams = DCLASS_PARAMS;
  SPI_TypeDef*  SPIx = pDParams->SPIx;
  /* Wait for SPI Tx buffer empty */
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET);
  /* Send SPI data */
  SPI_I2S_SendData16(SPIx, value);
  /* Wait for SPIz data reception */
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET);
  /* Read SPIz received data */
  return SPI_I2S_ReceiveData16(SPIx);
}

/**
* @brief  Deactivate SD pin
* @param  this: related object of class CGDC
* @retval none
*/
void GAP_SD_Deactivate(CGDC this)
{
  pDParams_t pDParams = DCLASS_PARAMS;
  pDParams->hNSDPort->BSRR = pDParams->hNSDPin;
}

/**
* @brief  Activate SD pin
* @param  this: related object of class CGDC
* @retval none
*/
void GAP_SD_Activate(CGDC this)
{
  pDParams_t pDParams = DCLASS_PARAMS;
  pDParams->hNSDPort->BRR = pDParams->hNSDPin;
}

/**
* @brief  Deactivate CS pin
* @param  this: related object of class CGDC
* @retval none
*/
void GAP_CS_Deactivate(CGDC this)
{
  pDParams_t pDParams = DCLASS_PARAMS;
  pDParams->hNCSPort->BSRR = pDParams->hNCSPin;
}

/**
* @brief  Activate CS pin
* @param  this: related object of class CGDC
* @retval none
*/
void GAP_CS_Activate(CGDC this)
{
  pDParams_t pDParams = DCLASS_PARAMS;
  pDParams->hNCSPort->BRR = pDParams->hNCSPin;
}

/**
  * @brief  It is an internal function used to compute the GPIO Source 
  *         value starting from GPIO pin value. The GPIO Source value 
  *         is used for AF remapping.
  * @param  GPIO_Pin Pin value to be converted.
  * @retval uint16_t The GPIO pin source value converted.
  */
static uint16_t GPIOPin2Source(uint16_t GPIO_Pin)
{
  uint16_t hRetVal;
  switch (GPIO_Pin)
  {
  case GPIO_Pin_0:
    {
      hRetVal = GPIO_PinSource0;
      break;
    }
  case GPIO_Pin_1:
    {
      hRetVal = GPIO_PinSource1;
      break;
    }
  case GPIO_Pin_2:
    {
      hRetVal = GPIO_PinSource2;
      break;
    }
  case GPIO_Pin_3:
    {
      hRetVal = GPIO_PinSource3;
      break;
    }
  case GPIO_Pin_4:
    {
      hRetVal = GPIO_PinSource4;
      break;
    }
  case GPIO_Pin_5:
    {
      hRetVal = GPIO_PinSource5;
      break;
    }
  case GPIO_Pin_6:
    {
      hRetVal = GPIO_PinSource6;
      break;
    }
  case GPIO_Pin_7:
    {
      hRetVal = GPIO_PinSource7;
      break;
    }
  case GPIO_Pin_8:
    {
      hRetVal = GPIO_PinSource8;
      break;
    }
  case GPIO_Pin_9:
    {
      hRetVal = GPIO_PinSource9;
      break;
    }
  case GPIO_Pin_10:
    {
      hRetVal = GPIO_PinSource10;
      break;
    }
  case GPIO_Pin_11:
    {
      hRetVal = GPIO_PinSource11;
      break;
    }
  case GPIO_Pin_12:
    {
      hRetVal = GPIO_PinSource12;
      break;
    }
  case GPIO_Pin_13:
    {
      hRetVal = GPIO_PinSource13;
      break;
    }
  case GPIO_Pin_14:
    {
      hRetVal = GPIO_PinSource14;
      break;
    }
  case GPIO_Pin_15:
    {
      hRetVal = GPIO_PinSource15;
      break;
    }
  default:
    {
      hRetVal = 0u;
      break;
    }
  }
  return hRetVal;
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
