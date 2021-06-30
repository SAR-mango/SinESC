/**
  ******************************************************************************
  * @file    ENC_F0xx_SpeednPosFdbkClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private implementation of ENCODER class for F0xx
  *          microcontroller family
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
#include "SpeednPosFdbkClass.h"
#include "SpeednPosFdbkPrivate.h"
#include "ENCODER_SpeednPosFdbkClass.h"
#include "ENC_F0xx_SpeednPosFdbkPrivate.h"
#include "MCIRQHandlerPrivate.h"
#include "MCLibraryConf.h"
#include "MCLibraryISRPriorityConf.h"
#include "MC_type.h"

/* Private Defines -----------------------------------------------------------*/
#define DCLASS_PARAM ((_DCENC_SPD)(((_CSPD) this)->DerivedClass))->pDParams_str
#define DCLASS_VARS  &(((_DCENC_SPD)(((_CSPD) this)->DerivedClass))->DVars_str)
#define  CLASS_VARS  &(((_CSPD)this)->Vars_str)
#define  CLASS_PARAM (((_CSPD)this)->pParams_str)

/* Direct address of the registers used by DMA */
#define TIM2_SR_Address   0x40000010u
#define TIM3_SR_Address   0x40000410u
#define TIM4_SR_Address   0x40000810u
#define TIM5_SR_Address   0x40000C10u

#ifdef MC_CLASS_DYNAMIC
	#include "stdlib.h" /* Used for dynamic allocation */
#else
	_DCENC_SPD_t ENC_SPDpool[MAX_ENC_SPD_NUM];
	unsigned char ENC_SPD_Allocated = 0u;
#endif

static void ENC_IRQHandler(void *this, unsigned char flag);
static void ENC_Init(CSPD this);
static void ENC_Clear(CSPD this);
static int16_t ENC_CalcAngle(CSPD this, void *pInputVars_str);
static bool ENC_CalcAvrgMecSpeed01Hz(CSPD this, int16_t *pMecSpeed01Hz);
static void ENC_SetMecAngle(CSPD this, int16_t hMecAngle);
static uint16_t F0XX_GPIOPin2Source(uint16_t GPIO_Pin);

/**
  * @brief  Creates an object of the class ENCODER
  * @param  pSpeednPosFdbkParams pointer to an SpeednPosFdbk parameters structure
  * @param  pENCODERParams pointer to an ENCODER parameters structure
  * @retval CENC_SPD new instance of ENCODER object
  */
CENC_SPD ENC_NewObject(pSpeednPosFdbkParams_t pSpeednPosFdbkParams, pENCODERParams_t pENCODERParams)
{
	_CSPD _oSpeednPosFdbk;
	_DCENC_SPD _oENCODER;

	_oSpeednPosFdbk = (_CSPD)SPD_NewObject(pSpeednPosFdbkParams);

	#ifdef MC_CLASS_DYNAMIC
		_oENCODER = (_DCENC_SPD)calloc(1u,sizeof(_DCENC_SPD_t));
	#else
		if (ENC_SPD_Allocated  < MAX_ENC_SPD_NUM)
		{
			_oENCODER = &ENC_SPDpool[ENC_SPD_Allocated++];
		}
		else
		{
			_oENCODER = MC_NULL;
		}
	#endif
  
	_oENCODER->pDParams_str = pENCODERParams;
	_oSpeednPosFdbk->DerivedClass = (void*)_oENCODER;
	
	_oSpeednPosFdbk->Methods_str.pIRQ_Handler = &ENC_IRQHandler;
	Set_IRQ_Handler(pENCODERParams->IRQnb, (_CMCIRQ)_oSpeednPosFdbk);

        _oSpeednPosFdbk->Methods_str.pSPD_Init = &ENC_Init;
        _oSpeednPosFdbk->Methods_str.pSPD_Clear = &ENC_Clear;
        _oSpeednPosFdbk->Methods_str.pSPD_CalcAngle = &ENC_CalcAngle;
        _oSpeednPosFdbk->Methods_str.pSPD_CalcAvrgMecSpeed01Hz = 
          &ENC_CalcAvrgMecSpeed01Hz;
        _oSpeednPosFdbk->Methods_str.pSPD_SetMecAngle =
          &ENC_SetMecAngle;
        
	return ((CENC_SPD)_oSpeednPosFdbk);
}

/** @addtogroup STM32F10x_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup SpeednPosFdbk_ENCODER
  * @{
  */

/** @defgroup ENCODER_class_private_methods ENCODER class private methods
* @{
*/

/**
  * @brief  It initializes the hardware peripherals (TIMx, GPIO and NVIC) 
            required for the speed position sensor management using ENCODER 
            sensors.
  * @param  this related object of class CSPD
  * @retval none
  */
static void ENC_Init(CSPD this)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_ICInitTypeDef TIM_ICInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;
  
  pDParams_t pDParams_str = DCLASS_PARAM;
  pDVars_t pDVars_str = DCLASS_VARS;
  TIM_TypeDef* TIMx = pDParams_str->TIMx;
  DMA_Channel_TypeDef*  DMA_Channelx = 0;
  uint16_t hSignalPolarity = TIM_ICPolarity_Rising;
  uint8_t  bGPIOAF = 0u;
  
  uint16_t hTIM_Channel_x = TIM_Channel_3;  
  uint16_t hTIM_IT_CCx = TIM_IT_CC3;
  uint16_t hTIM_DMA_CCx = TIM_DMA_CC3;
  uint32_t wTimerStatusRegisterAddress = 0u;
  uint8_t bBufferSize;
  uint8_t bIndex;
  
  /*Selection of appropriate capture channel, DMA channel, timer status register*/
  pDVars_str->hTIMxCCRAddress = &(TIMx->CCR3);
  pDVars_str->hTIM_FLAG_CCx = TIM_FLAG_CC3;
  pDVars_str->hTIM_EventSource_CCx = TIM_EventSource_CC3;
  
  if (pDParams_str->RevertSignal)
  {
    hSignalPolarity = TIM_ICPolarity_Falling;
  }
  
  if (TIMx == TIM2)
  {
    wTimerStatusRegisterAddress = (uint32_t)TIM2_SR_Address;
    DMA_Channelx = DMA1_Channel1;
    bGPIOAF = GPIO_AF_2;
  }
  else if (TIMx == TIM3)
  {
    hTIM_Channel_x = TIM_Channel_4;
    hTIM_DMA_CCx = TIM_DMA_CC4;
    pDVars_str->hTIM_FLAG_CCx = TIM_FLAG_CC4;
    hTIM_IT_CCx = TIM_IT_CC4;
    pDVars_str->hTIM_EventSource_CCx = TIM_EventSource_CC4;
    pDVars_str->hTIMxCCRAddress = &(TIMx->CCR4);
    
    wTimerStatusRegisterAddress = (uint32_t)TIM3_SR_Address;
    DMA_Channelx = DMA1_Channel3;
    bGPIOAF = GPIO_AF_1;    
  }
  else
  {
    ;
  }  
    
  /* TIMx clock enable */
  RCC_APB1PeriphClockCmd(pDParams_str->RCC_APB1Periph_TIMx, ENABLE);
  
  /* DMAx clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);  
  
  GPIO_StructInit(&GPIO_InitStructure);
  /* Configure Encoder input A+*/
  GPIO_InitStructure.GPIO_Pin = pDParams_str->hAPin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(pDParams_str->hAPort, &GPIO_InitStructure);
  
  
  /* Configure Encoder input B+*/
  GPIO_InitStructure.GPIO_Pin = pDParams_str->hBPin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(pDParams_str->hBPort, &GPIO_InitStructure);

  GPIO_PinAFConfig(pDParams_str->hAPort, F0XX_GPIOPin2Source(pDParams_str->hAPin), bGPIOAF);
  GPIO_PinAFConfig(pDParams_str->hBPort, F0XX_GPIOPin2Source(pDParams_str->hBPin), bGPIOAF);

  
  /* Timer configuration in Encoder mode, Input Capture, DMA request */
  TIM_DeInit(TIMx);
  
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);  
  TIM_TimeBaseStructure.TIM_Prescaler = 0u;
  TIM_TimeBaseStructure.TIM_Period = (uint32_t)(pDParams_str->hPulseNumber) - 1u;  
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
  TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);
    
  TIM_ICStructInit(&TIM_ICInitStructure);  
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICFilter = pDParams_str->hInpCaptFilter;
  TIM_ICInit(TIMx, &TIM_ICInitStructure);  
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInit(TIMx, &TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_Channel = hTIM_Channel_x;
  TIM_ICInit(TIMx, &TIM_ICInitStructure);
  TIM_DMACmd(TIMx,hTIM_DMA_CCx,ENABLE);

  TIM_EncoderInterfaceConfig(TIMx, TIM_EncoderMode_TI12, 
                             TIM_ICPolarity_Rising, hSignalPolarity);

  
  /* DMA Configuration for status register transfer*/ 
  DMA_DeInit(DMA_Channelx);
  DMA_InitStructure.DMA_PeripheralBaseAddr = wTimerStatusRegisterAddress;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(&(pDVars_str->wTimerStatusRegisterCopy));
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 1u;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = ENC_DMA_PRIORITY;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA_Channelx, &DMA_InitStructure);
  
  /* Enable the TIMx Update Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = pDParams_str->TIMx_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPriority = TIMx_PRE_EMPTION_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
      
  /* Clear all pending interrupts */
  TIM_ClearFlag(TIMx, TIM_FLAG_Update);
  TIM_ClearFlag(TIMx, pDVars_str->hTIM_FLAG_CCx);

  /*Configure IRQ related to ENCODER*/
  TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE);
  TIM_ITConfig(TIMx, hTIM_IT_CCx, ENABLE);
  
  /* Reset counter */
  TIMx->CNT = 0u;
  
  /*Calculations of convenience*/
  pDVars_str->wU32MAXdivPulseNumber = U32_MAX/(uint32_t)(pDParams_str->hPulseNumber);
  pDVars_str->hSpeedSamplingFreqHz = pDParams_str->hSpeedSamplingFreq01Hz/ 10u;
  
  /*Enable the DMA channel*/
  DMA_Cmd(DMA_Channelx, ENABLE);
  
  /* Enable the counting timer*/  
  TIM_Cmd(TIMx, ENABLE);

  /* Erase speed buffer */
  bBufferSize = pDParams_str->bSpeedBufferSize;
  
  for (bIndex=0u;bIndex<bBufferSize;bIndex++)
  {
    pDVars_str->wDeltaCapturesBuffer[bIndex]=0;
  }  
}

/**
* @brief  Clear software FIFO where are "pushed" rotor angle variations captured
*         This function must be called before starting the motor to initialize
*	        the speed measurement process.
* @param  this related object of class CSPD
* @retval none
*/
static void ENC_Clear(CSPD this)
{
  pDVars_t pDVars_str = DCLASS_VARS;
    
  pDVars_str->SensorIsReliable = TRUE;
}

/**
* @brief  It calculates the rotor electrical and mechanical angle, on the basis
*         of the instantaneous value of the timer counter.
* @param  this related object of class CSPD
* @param  pInputVars_str Not used in this derived class implementation of
*						 SPD_CalcElAngle
* @retval int16_t Measured electrical angle in s16degree format.
*/
static int16_t ENC_CalcAngle(CSPD this, void *pInputVars_str)
{
  pDParams_t pDParams_str = DCLASS_PARAM;
  pDVars_t pDVars_str = DCLASS_VARS;
  int32_t wtemp1;
  int32_t wtemp2;
  int16_t htemp1;
  int16_t htemp2;
  
  wtemp1 = (int32_t)(pDParams_str->TIMx->CNT) *
    (int32_t)(pDVars_str->wU32MAXdivPulseNumber);
  
  /*Computes and stores the rotor electrical angle*/
  wtemp2 = wtemp1 * (int32_t)((_CSPD)this)->pParams_str->bElToMecRatio;
  htemp1 = (int16_t)(wtemp2/65536);  

  ((_CSPD)this)->Vars_str.hElAngle = htemp1;
  
  /*Computes and stores the rotor mechanical angle*/
  htemp2 = (int16_t)(wtemp1/65536);
  
  ((_CSPD)this)->Vars_str.hMecAngle = htemp2;
  
  /*Returns rotor electrical angle*/  
  return(htemp1);
}

/**
  * @brief  This method must be called with the periodicity defined by parameter
  *         hSpeedSamplingFreq01Hz. The method generates a capture event on
  *         a channel, computes & stores average mechanical speed [01Hz] (on the
  *         basis of the buffer filled by CCx IRQ), computes & stores average 
  *         mechanical acceleration [01Hz/SpeedSamplingFreq], computes & stores
  *         the instantaneous electrical speed [dpp], updates the index of the
  *         speed buffer, then checks & stores & returns the reliability state
  *         of the sensor.
  * @param  this related object of class CSPD
  * @param  hMecSpeed01Hz pointer to int16_t, used to return the rotor average
  *         mechanical speed (01Hz)
  * @retval TRUE = sensor information is reliable
  *         FALSE = sensor information is not reliable
  */
static bool ENC_CalcAvrgMecSpeed01Hz(CSPD this, int16_t *pMecSpeed01Hz)
{
  pDParams_t pDParams_str = DCLASS_PARAM;
  pDVars_t pDVars_str = DCLASS_VARS;
  TIM_TypeDef* TIMx = pDParams_str->TIMx;
  int32_t wOverallAngleVariation = 0;
  int32_t wtemp1;
  int32_t wtemp2;
  uint8_t bBufferIndex = 0u;
  bool bReliability = TRUE;
  uint8_t bBufferSize = pDParams_str->bSpeedBufferSize;
  
  /*Generates a capture event on CCx*/
  TIMx->EGR = pDVars_str->hTIM_EventSource_CCx;
  
  /*Computes & returns average mechanical speed [01Hz], var wtemp1*/
  for (bBufferIndex=0u;bBufferIndex<bBufferSize;bBufferIndex++)
  {
    wOverallAngleVariation += pDVars_str->wDeltaCapturesBuffer[bBufferIndex];
  }
  wtemp1 = wOverallAngleVariation * (int32_t)(pDParams_str->hSpeedSamplingFreq01Hz);
  wtemp2 = (int32_t)(pDParams_str->hPulseNumber)*
    (int32_t)(pDParams_str->bSpeedBufferSize);
  wtemp1 /= wtemp2;  
  *pMecSpeed01Hz = (int16_t)(wtemp1);
  
  /*Computes & stores average mechanical acceleration [01Hz/SpeedSamplingFreq]*/
  ((_CSPD)this)->Vars_str.hMecAccel01HzP = (int16_t)(wtemp1 - 
              ((_CSPD)this)->Vars_str.hAvrMecSpeed01Hz);
    
  /*Stores average mechanical speed [01Hz]*/
  ((_CSPD)this)->Vars_str.hAvrMecSpeed01Hz = (int16_t)wtemp1;
  
  /*Computes & stores the instantaneous electrical speed [dpp], var wtemp1*/
  wtemp1 = pDVars_str->wDeltaCapturesBuffer[pDVars_str->bDeltaCapturesIndex] *
    (int32_t)(pDVars_str->hSpeedSamplingFreqHz) *
      (int32_t)((_CSPD)this)->pParams_str->bElToMecRatio;
  wtemp1 /= (int32_t)(pDParams_str->hPulseNumber);
  wtemp1 *= (int32_t)U16_MAX;  
  wtemp1 /= (int32_t)(((_CSPD)this)->pParams_str->hMeasurementFrequency); 

  ((_CSPD)this)->Vars_str.hElSpeedDpp = (int16_t)wtemp1;
  
  /*Buffer index update*/
  pDVars_str->bDeltaCapturesIndex++;
  if (pDVars_str->bDeltaCapturesIndex == pDParams_str->bSpeedBufferSize)
  {
    pDVars_str->bDeltaCapturesIndex = 0u;
  }
  
  /*Checks the reliability status, then stores and returns it*/
  if (pDVars_str->TimerOverflowError)
  {
    bReliability = FALSE;
    pDVars_str->SensorIsReliable = FALSE;
    
  }

  return(bReliability);
}

/**
  * @brief  It could be used to set istantaneous information on rotor mechanical
  *         angle. As a consequence, timer counted is computed and updated.
  * @param  this related object of class CSPD
  * @param  hMecAngle istantaneous measure of rotor mechanical angle (s16degrees)
  * @retval none
  */
static void ENC_SetMecAngle(CSPD this, int16_t hMecAngle)
{
  pDParams_t pDParams_str = DCLASS_PARAM;
  TIM_TypeDef* TIMx = pDParams_str->TIMx;
  
  uint16_t hAngleCounts;
  uint16_t hMecAngleuint;
  
  if (hMecAngle < 0)
  {
    hMecAngle *= -1;
    hMecAngleuint = 65535u - (uint16_t)hMecAngle;
  }
  else
  {
    hMecAngleuint = (uint16_t)hMecAngle;
  }
  
  hAngleCounts = (uint16_t)(((uint32_t)hMecAngleuint *
                    (uint32_t)pDParams_str->hPulseNumber)/65535u);
   
  TIMx->CNT = (uint16_t)(hAngleCounts);
}

/**
  * @brief  Example of private method of the class ENCODER to implement an MC IRQ function
  * @param  this related object
  * @param  flag used to distinguish between various IRQ sources
  * @retval none
  */
static void ENC_IRQHandler(void *this, unsigned char flag)
{
  pDVars_t pDVars_str = DCLASS_VARS;
  pDParams_t pDParams_str = DCLASS_PARAM;
  TIM_TypeDef* TIMx = pDParams_str->TIMx;
  uint16_t hOverflowsOccurred;
  uint32_t wNewCapture;
  
  /*TIMx Capture IRQ*/
  if ((TIMx->SR & pDVars_str->hTIM_FLAG_CCx) == pDVars_str->hTIM_FLAG_CCx) 
  {
    wNewCapture = *(pDVars_str->hTIMxCCRAddress);
    /* Computation of overflows occurred (pending flags at capture time to account)*/
    if ((pDVars_str->wTimerStatusRegisterCopy & TIM_FLAG_Update) == TIM_FLAG_Update)
    {
      wNewCapture = TIMx->CNT;
      hOverflowsOccurred = pDVars_str->hTimerOverflowNb + 1u;
      pDVars_str->hTimerOverflowNb = 0u;
    }
    else
    {
      hOverflowsOccurred = pDVars_str->hTimerOverflowNb;
      pDVars_str->hTimerOverflowNb = 1u;
    }
    
    /*Calculation of delta angle*/
    if ((TIMx->CR1 & TIM_CounterMode_Down) == TIM_CounterMode_Down)  
    {/* encoder timer down-counting*/
      pDVars_str->wDeltaCapturesBuffer[pDVars_str->bDeltaCapturesIndex]= 
        (int32_t)(wNewCapture) - (int32_t)(pDVars_str->hPreviousCapture) -
          ((int32_t)(hOverflowsOccurred) - 1) * (int32_t)(pDParams_str->hPulseNumber);
    }
    else  
    {/* encoder timer up-counting*/
      pDVars_str->wDeltaCapturesBuffer[pDVars_str->bDeltaCapturesIndex]= 
        (int32_t)(wNewCapture) - (int32_t)(pDVars_str->hPreviousCapture) +
          ((int32_t)(hOverflowsOccurred) - 1) * (int32_t)(pDParams_str->hPulseNumber);
    }
    
    /*last captured value update*/
    pDVars_str->hPreviousCapture = (uint16_t)(wNewCapture);
  }
  
  /* TIMx Update IRQ */
  if ((TIMx->SR & TIM_FLAG_Update) == TIM_FLAG_Update)  
  {
    /* Clear the interrupt pending flag */
    TIM_ClearFlag(TIMx, TIM_FLAG_Update);
    
    /*Updates the number of overflows occurred*/      
    if (pDVars_str->hTimerOverflowNb != ENC_MAX_OVERFLOW_NB)  
    {
      (pDVars_str->hTimerOverflowNb)+=1u;
    }
    else
    {
      pDVars_str->TimerOverflowError = TRUE;
    }
  }
}

/**
  * @brief  It is an internal function used to compute the GPIO Source 
  *         value starting from GPIO pin value. The GPIO Source value 
  *         is used for AF remapping.
  * @param  GPIO_Pin Pin value to be converted.
  * @retval uint16_t The GPIO pin source value converted.
  */
static uint16_t F0XX_GPIOPin2Source(uint16_t GPIO_Pin)
{ 
 uint16_t GPIO_Sourcex = 0u;
 
 while (GPIO_Pin != 0x01u)
 {
   GPIO_Pin = GPIO_Pin >> 1u;
   GPIO_Sourcex++;
 }
 return GPIO_Sourcex;

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
