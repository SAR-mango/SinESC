/**
  ******************************************************************************
  * @file    UnidirectionalFastCom_UserInterfaceClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private impelementation of UnidirectionalFastCom class      
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

/* Includes ------------------------------------------------------------------*/
#include "UserInterfaceClass.h"
#include "UserInterfacePrivate.h"
#include "UnidirectionalFastCom_UserInterfaceClass.h"
#include "UnidirectionalFastCom_UserInterfacePrivate.h"
#include "MC_type.h"
#include "UIIRQHandlerPrivate.h"

#define GPIO_AF_USART1 GPIO_AF_7
#define GPIO_AF_USART2 GPIO_AF_7
#define GPIO_AF_USART3 GPIO_AF_7
#define GPIO_AF_UART4 GPIO_AF_7
#define GPIO_AF_UART5 GPIO_AF_7

#ifdef MC_CLASS_DYNAMIC
  #include "stdlib.h" /* Used for dynamic allocation */
#else
  #define MAX_UFC_UI_NUM 1u
  _DCUFC_UI_t UFC_UIpool[MAX_UFC_UI_NUM];
  unsigned char UFC_UI_Allocated = 0u;
#endif

#define DCLASS_PARAM ((_DCUFC_UI)(((_CUI) this)->DerivedClass))->pDParams_str
#define DCLASS_VARS  &(((_DCUFC_UI)(((_CUI) this)->DerivedClass))->DVars_str)
#define  CLASS_VARS  &(((_CUI)this)->Vars_str)
#define  CLASS_PARAM (((_CUI)this)->pParams_str)
  
/* Private function prototypes -----------------------------------------------*/
static uint16_t USART_GPIOPin2Source(uint16_t GPIO_Pin);
static uint8_t USART_AF(USART_TypeDef* USARTx);

/* Private function prototypes -----------------------------------------------*/  
void* UFC_IRQ_Handler(void* this,unsigned char flags, unsigned short rx_data);

/**
  * @brief  Creates an object of the class UnidirectionalFastCom
  * @param  pUserInterfaceParams pointer to an UserInterface parameters structure
  * @param  pUnidirectionalFastComParams pointer to an UnidirectionalFastCom parameters structure
  * @retval CUFC_UI new instance of UnidirectionalFastCom object
  */
CUFC_UI UFC_NewObject(pUserInterfaceParams_t pUserInterfaceParams, pUnidirectionalFastComParams_t pUnidirectionalFastComParams)
{
  _CUI _oUserInterface;
  _DCUFC_UI _oUnidirectionalFastCom;
  
  _oUserInterface = (_CUI)UI_NewObject(pUserInterfaceParams);
  
#ifdef MC_CLASS_DYNAMIC
  _oUnidirectionalFastCom = (_DCUFC_UI)calloc(1u,sizeof(_DCUFC_UI_t));
#else
  if (UFC_UI_Allocated  < MAX_UFC_UI_NUM)
  {
    _oUnidirectionalFastCom = &UFC_UIpool[UFC_UI_Allocated++];
  }
  else
  {
    _oUnidirectionalFastCom = MC_NULL;
  }
#endif
  
  _oUnidirectionalFastCom->pDParams_str = pUnidirectionalFastComParams;
  _oUserInterface->DerivedClass = (void*)_oUnidirectionalFastCom;
  
  _oUserInterface->Methods_str.pIRQ_Handler = &UFC_IRQ_Handler;
  Set_UI_IRQ_Handler(pUnidirectionalFastComParams->bUIIRQn, (_CUIIRQ)_oUserInterface);
  
  return ((CUFC_UI)_oUserInterface);
}

/** @addtogroup STM32F10x_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup UserInterface_UnidirectionalFastCom
  * @{
  */

/** @defgroup UnidirectionalFastCom_class_private_methods UnidirectionalFastCom class private methods
* @{
*/

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

/**
  * @brief  Initialization of the class UnidirectionalFastCom. It initialize all
  *         HW and private vars.
  * @param  this related object of class CUFC_UI
  * @retval none
  */
void UFC_Init(CUFC_UI this)
{
  pVars_t pVars = CLASS_VARS;
  pDVars_t pDVars = DCLASS_VARS;
  pDParams_t pDParams = DCLASS_PARAM;
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* Init Vars */
  pVars->bSelectedDrive = pDParams->bDefMotor;
  pDVars->bChannel[0] = pDParams->bDefChannel1;
  pDVars->bChannel[1] = pDParams->bDefChannel2;
  pDVars->bChByteNum[0] = pDParams->bCh1ByteNum;
  pDVars->bChByteNum[1] = pDParams->bCh2ByteNum;
  pDVars->comON = FALSE;
  pDVars->bChTransmitted = 0;
  pDVars->bByteTransmitted = 0;
  pDVars->bChNum = pDParams->bChNum;
  
  /* HW Init */
  
  /* Enable USART clock: UASRT1 -> APB2, USART2-5 -> APB1 */
  if (pDParams->wUSARTClockSource == RCC_APB2Periph_USART1)
  {
    RCC_APB2PeriphClockCmd(pDParams->wUSARTClockSource, ENABLE);
  }
  else
  {
    RCC_APB1PeriphClockCmd(pDParams->wUSARTClockSource, ENABLE);
  }  
  
  /* USART Init structure */
  /* Configure USART */
  USART_Init(pDParams->USARTx, pDParams->USART_InitStructure);
  
  GPIO_PinAFConfig(pDParams->hTxPort, USART_GPIOPin2Source(pDParams->hTxPin), USART_AF(pDParams->USARTx));
  
  /* Configure Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = pDParams->hTxPin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(pDParams->hTxPort, &GPIO_InitStructure);
  
  if (pDParams->NVIC_InitStructure->NVIC_IRQChannelCmd == ENABLE)
  {
    /* Enable the USARTy Interrupt */
    NVIC_Init(pDParams->NVIC_InitStructure);
  }
  
  /* Enable the USART */
  USART_Cmd(pDParams->USARTx, ENABLE);
}

/*******************************************************************************
* Function Name  : USART_IRQ_Handler
* Description    : Interrupt function for the serial communication
* Input          : none 
* Return         : none
*******************************************************************************/
void* UFC_IRQ_Handler(void* this,unsigned char flags, unsigned short rx_data)
{
  pDVars_t pDVars = DCLASS_VARS;
  pDParams_t pDParams = DCLASS_PARAM;
  void* pRetVal = MC_NULL;
  uint8_t txData = 0;
  uint8_t* pBuff;
  
  if (flags == 1) // Flag 1 = TX
  {
    if (pDVars->comON)
    {
      if (pDVars->bByteTransmitted == 0)
      {
        /* First byte to be transmitted, read value and populate the buffer */
        pDVars->wBuffer = UI_GetReg(this, pDVars->bChannel[pDVars->bChTransmitted]) >> 8;
      }
      
      pBuff = (uint8_t*)(&(pDVars->wBuffer));
      txData = pBuff[pDVars->bByteTransmitted];
      
      /* Write one byte to the transmit data register */
      USART_SendData(pDParams->USARTx, txData);
      
      pDVars->bByteTransmitted++;
      if (pDVars->bByteTransmitted == pDVars->bChByteNum[pDVars->bChTransmitted])
      {
        pDVars->bByteTransmitted = 0;
        pDVars->bChTransmitted++;
        if (pDVars->bChTransmitted == pDVars->bChNum)
        {
          pDVars->bChTransmitted = 0;
        }
      }
    }
  }
  
  return pRetVal;
}

/**
  * @brief  Starts the fast unidirectional communication.
  * @param  this related object of class CUFC_UI
  * @retval none
  */
void UFC_StartCom(CUFC_UI this)
{
  pDVars_t pDVars = DCLASS_VARS;
  pDParams_t pDParams = DCLASS_PARAM;
  pDVars->comON = TRUE;
  USART_SendData(pDParams->USARTx, ' ');
  /* Enable USART Transmit interrupts */
  USART_ITConfig(pDParams->USARTx, USART_IT_TXE, ENABLE);
}

/**
  * @brief  Starts the fast unidirectional communication.
  * @param  this related object of class CUFC_UI
  * @retval none
  */
void UFC_StopCom(CUFC_UI this)
{
  pDVars_t pDVars = DCLASS_VARS;
  pDParams_t pDParams = DCLASS_PARAM;
  pDVars->comON = FALSE;
  /* Disable USART Transmit interrupts */
  USART_ITConfig(pDParams->USARTx, USART_IT_TXE, DISABLE);
}

/**
  * @brief  It is an internal function used to compute the GPIO Source 
  *         value starting from GPIO pin value. The GPIO Source value 
  *         is used for AF remapping.
  * @param  GPIO_Pin Pin value to be converted.
  * @retval uint16_t The GPIO pin source value converted.
  */
static uint16_t USART_GPIOPin2Source(uint16_t GPIO_Pin)
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

static uint8_t USART_AF(USART_TypeDef* USARTx)
{
  uint8_t hRetVal = 0u;
  if (USARTx == USART1)
  {
    hRetVal = GPIO_AF_USART1;
  }
  else if (USARTx == USART2)
  {
    hRetVal = GPIO_AF_USART2;
  }
  else if (USARTx == USART3)
  {
    hRetVal = GPIO_AF_USART3;
  }
  else if (USARTx == UART4)
  {
    hRetVal = GPIO_AF_UART4;
  }
  else if (USARTx == UART5)
  {
    hRetVal = GPIO_AF_UART5;
  }
  return hRetVal;
}

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
