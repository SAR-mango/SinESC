/**
  ******************************************************************************
  * @file    HALL_F4xx_SpeednPosFdbkClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private implementation of HALL_SpeednPosFdbk for
  *          F4xx microcontroller family
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
#include "HALL_SpeednPosFdbkClass.h"
#include "HALL_F4xx_SpeednPosFdbkPrivate.h"
#include "MCIRQHandlerPrivate.h"
#include "MCLibraryConf.h"
#include "MCLibraryISRPriorityConf.h"
#include "MC_type.h"

/* Private Typedef -----------------------------------------------------------*/
/* Private Defines -----------------------------------------------------------*/
#define DCLASS_PARAM ((_DCHALL_SPD)(((_CSPD) this)->DerivedClass))->pDParams_str
#define DCLASS_VARS  &(((_DCHALL_SPD)(((_CSPD) this)->DerivedClass))->DVars_str)
#define  CLASS_VARS  &(((_CSPD)this)->Vars_str)
#define  CLASS_PARAM (((_CSPD)this)->pParams_str)

/* Lower threshold to reques a decrease of clock prescaler */
#define LOW_RES_THRESHOLD   ((u16)0x5500u)

#define HALL_COUNTER_RESET  ((u16) 0u)

#define S16_120_PHASE_SHIFT (s16)(65536/3)
#define S16_60_PHASE_SHIFT  (s16)(65536/6)

#define STATE_0 (u8)0
#define STATE_1 (u8)1
#define STATE_2 (u8)2
#define STATE_3 (u8)3
#define STATE_4 (u8)4
#define STATE_5 (u8)5
#define STATE_6 (u8)6
#define STATE_7 (u8)7

#define NEGATIVE          (s8)-1
#define POSITIVE          (s8)1
#define NEGATIVE_SWAP     (s8)-2
#define POSITIVE_SWAP     (s8)2
#define HALL_ERROR        (s8)127

/* With digit-per-PWM unit (here 2*PI rad = 0xFFFF): */
#define HALL_MAX_PSEUDO_SPEED        ((s16)-32768)

#define CCER_CC1E_Set               ((u16)0x0001)
#define CCER_CC1E_Reset             ((u16)0xFFFE)

/* #define HALL_MTPA */

#ifdef MC_CLASS_DYNAMIC
	#include "stdlib.h" /* Used for dynamic allocation */
#else
	_DCHALL_SPD_t HALL_SPDpool[MAX_HALL_SPD_NUM];
	unsigned char HALL_SPD_Allocated = 0u;
#endif

static void HALL_IRQHandler(void *this, unsigned char flag);
static void HALL_Init(CSPD this);
static void HALL_Clear(CSPD this);
static int16_t HALL_CalcElAngle(CSPD this, void *pInputVars_str);
static bool HALL_CalcAvrgMecSpeed01Hz(CSPD this, int16_t *hMecSpeed01Hz);
static int16_t HALL_CalcAvrgElSpeedDpp(CSPD this);
static void HALL_Init_Electrical_Angle(CSPD this);
static void HALL_SetMecAngle(CSPD this, int16_t hMecAngle);
static uint16_t F4XX_GPIOPin2Source(uint16_t GPIO_Pin);

/**
  * @brief  Creates an object of the class HALL
  * @param  pSpeednPosFdbkParams pointer to an SpeednPosFdbk parameters structure
  * @param  pHALLParams pointer to an HALL parameters structure
  * @retval CHALL_SPD new instance of HALL object
  */
CHALL_SPD HALL_NewObject(pSpeednPosFdbkParams_t pSpeednPosFdbkParams, pHALLParams_t pHALLParams)
{
	_CSPD _oSpeednPosFdbk;
	_DCHALL_SPD _oHALL;

	_oSpeednPosFdbk = (_CSPD)SPD_NewObject(pSpeednPosFdbkParams);

	#ifdef MC_CLASS_DYNAMIC
		_oHALL = (_DCHALL_SPD)calloc(1u,sizeof(_DCHALL_SPD_t));
	#else
		if (HALL_SPD_Allocated  < MAX_HALL_SPD_NUM)
		{
			_oHALL = &HALL_SPDpool[HALL_SPD_Allocated++];
		}
		else
		{
			_oHALL = MC_NULL;
		}
	#endif
  
	_oHALL->pDParams_str = pHALLParams;
	_oSpeednPosFdbk->DerivedClass = (void*)_oHALL;
	
	_oSpeednPosFdbk->Methods_str.pIRQ_Handler = &HALL_IRQHandler;
	Set_IRQ_Handler(pHALLParams->IRQnb, (_CMCIRQ)_oSpeednPosFdbk);
  
  _oSpeednPosFdbk->Methods_str.pSPD_Init = &HALL_Init;
  _oSpeednPosFdbk->Methods_str.pSPD_Clear = &HALL_Clear;
  _oSpeednPosFdbk->Methods_str.pSPD_CalcAngle = &HALL_CalcElAngle;
  _oSpeednPosFdbk->Methods_str.pSPD_CalcAvrgMecSpeed01Hz = 
    &HALL_CalcAvrgMecSpeed01Hz;
  _oSpeednPosFdbk->Methods_str.pSPD_SetMecAngle = &HALL_SetMecAngle;

	return ((CHALL_SPD)_oSpeednPosFdbk);
}

/** @addtogroup STM32F10x_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup SpeednPosFdbk_HALL
  * @{
  */

/** @defgroup HALL_class_private_methods HALL class private methods
* @{
*/

/**
  * @brief  It initializes the hardware peripherals (TIMx, GPIO and NVIC) 
            required for the speed position sensor management using HALL 
            sensors.
  * @param  this related object of class CSPD
  * @retval none
  */
static void HALL_Init(CSPD this)
{
  TIM_TimeBaseInitTypeDef TIM_HALLTimeBaseInitStructure;
  TIM_ICInitTypeDef TIM_HALLICInitStructure;
  NVIC_InitTypeDef NVIC_InitHALLStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  pDVars_t pDVars_str = DCLASS_VARS;
  pDParams_t pDParams_str = DCLASS_PARAM;
  pParams_t pParams_str = CLASS_PARAM;
  TIM_TypeDef* TIMx = pDParams_str->TIMx;
  uint16_t hMinReliableElSpeed01Hz = pParams_str->hMinReliableMecSpeed01Hz *
    pParams_str->bElToMecRatio;
  uint16_t hMaxReliableElSpeed01Hz = pParams_str->hMaxReliableMecSpeed01Hz *
    pParams_str->bElToMecRatio;
  uint8_t bSpeedBufferSize;
  uint8_t bIndex;
  uint8_t  bGPIOAF = 0u;
  
  /* Adjustment factor: minimum measurable speed is x time less than the minimum
  reliable speed */
  hMinReliableElSpeed01Hz /= 4u;
  
  /* Adjustment factor: maximum measurable speed is x time greather than the
  maximum reliable speed */
  hMaxReliableElSpeed01Hz *= 2u;
  
  /* SW Init */
  if (hMinReliableElSpeed01Hz == 0u)
  {
    /* Set fixed to 150 ms */
    pDVars_str->hHallTimeout = 150u; 
    
    /* Set fixed to 164*/
    pDVars_str->hHALLMaxRatio = 164u;
    
    pDVars_str->wMaxPeriod = 10000000u;
  }
  else
  {
    /* Set accordingly the min reliable speed */
    pDVars_str->hHallTimeout = 10000u / (6u * hMinReliableElSpeed01Hz);
    pDVars_str->hHALLMaxRatio = (uint16_t)((pDVars_str->hHallTimeout * 
                               (pDParams_str->wTIMClockFreq/1000uL)) / 65536uL);   
    pDVars_str->wMaxPeriod = ((10u * pDParams_str->wTIMClockFreq) / 6u)
      / hMinReliableElSpeed01Hz;
  }

  pDVars_str->hSatSpeed = hMaxReliableElSpeed01Hz;
  
  pDVars_str->wPseudoFreqConv = ((pDParams_str->wTIMClockFreq / 6u) 
                     / (pParams_str->hMeasurementFrequency)) * 65536u;
    
  pDVars_str->wSpeedOverflow = ((10u * pDParams_str->wTIMClockFreq) / 6u) 
                     / hMaxReliableElSpeed01Hz;
    
  pDVars_str->hOvfDuration = (uint16_t)(pDParams_str->wTIMClockFreq / 65536u);
  
  pDVars_str->hPWMNbrPSamplingFreq = (pParams_str ->hMeasurementFrequency / 
                pDParams_str->hSpeedSamplingFreqHz) - 1u;
  
  /* Reset speed reliability */
  pDVars_str->SensorIsReliable = TRUE;
  
  /* HW Init */

  /* TIMx clock enable */  
  RCC_APB1PeriphClockCmd(pDParams_str->RCC_APB1Periph_TIMx, ENABLE);
  
    /* Enable all GPIO clocks */
  RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA | 
                          RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | 
                          RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE | 
                          RCC_AHB1Periph_GPIOF | RCC_AHB1Periph_GPIOG |
                          RCC_AHB1Periph_GPIOH | RCC_AHB1Periph_GPIOI , ENABLE);   
  
  if(pDParams_str->TIMx == TIM2)
  {
    bGPIOAF = GPIO_AF_TIM2;
  }
  else if (pDParams_str->TIMx == TIM3)
  {
    bGPIOAF = GPIO_AF_TIM3;
  }
  else if (pDParams_str->TIMx == TIM4)  
  {
    bGPIOAF = GPIO_AF_TIM4;
  }
  else if (TIMx == TIM5)
  {
    bGPIOAF = GPIO_AF_TIM5;    
  }
  else 
  {
    ;
  }
  
  GPIO_StructInit(&GPIO_InitStructure);
  /* Configure Hall sensors H1 input */
  GPIO_InitStructure.GPIO_Pin = pDParams_str->hH1Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(pDParams_str->hH1Port, &GPIO_InitStructure);

  /* Configure Hall sensors H2 input */
  GPIO_InitStructure.GPIO_Pin = pDParams_str->hH2Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(pDParams_str->hH2Port, &GPIO_InitStructure);

  /* Configure Hall sensors H3 input */
  GPIO_InitStructure.GPIO_Pin = pDParams_str->hH3Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(pDParams_str->hH3Port, &GPIO_InitStructure);
   
  GPIO_PinAFConfig(pDParams_str->hH1Port, F4XX_GPIOPin2Source(pDParams_str->hH1Pin), bGPIOAF);
  GPIO_PinAFConfig(pDParams_str->hH2Port, F4XX_GPIOPin2Source(pDParams_str->hH2Pin), bGPIOAF);
  GPIO_PinAFConfig(pDParams_str->hH3Port, F4XX_GPIOPin2Source(pDParams_str->hH3Pin), bGPIOAF);
  
  /* Timer configuration in Clear on capture mode */
  TIM_DeInit(TIMx);
  
  TIM_TimeBaseStructInit(&TIM_HALLTimeBaseInitStructure);
  /* Set full 16-bit working range */
  TIM_HALLTimeBaseInitStructure.TIM_Period = U16_MAX;
  TIM_HALLTimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIMx,&TIM_HALLTimeBaseInitStructure);
  
  TIM_ICStructInit(&TIM_HALLICInitStructure);
  TIM_HALLICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_HALLICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
  TIM_HALLICInitStructure.TIM_ICFilter = pDParams_str->hInpCaptFilter;
  
  TIM_ICInit(TIMx,&TIM_HALLICInitStructure);
  
  /* Force input capture from TRC */
  /* Disable the Channel 1: Reset the CC1E Bit */
  TIMx->CCER &= CCER_CC1E_Reset;
  TIMx->CCMR1 |= 0x03u; /* CCS1 = 0b11; */
  TIMx->CCER |= CCER_CC1E_Set;
  
  /* Force the TIMx prescaler with immediate access (no need of an update event) 
  */ 
  TIM_PrescalerConfig(TIMx, (u16) pDVars_str->hHALLMaxRatio, 
                      TIM_PSCReloadMode_Immediate);
  TIM_InternalClockConfig(TIMx);
  
  /* Enables the XOR of channel 1, channel2 and channel3 */
  TIM_SelectHallSensor(TIMx, ENABLE);
  
  TIM_SelectInputTrigger(TIMx, TIM_TS_TI1F_ED);
  TIM_SelectSlaveMode(TIMx,TIM_SlaveMode_Reset);
  
  /* Source of Update event is only counter overflow/underflow */
  TIM_UpdateRequestConfig(TIMx, TIM_UpdateSource_Regular);
  
  /* Enable the TIMx IRQChannel*/
  NVIC_InitHALLStructure.NVIC_IRQChannel =  pDParams_str->TIMx_IRQChannel;  
  NVIC_InitHALLStructure.NVIC_IRQChannelPreemptionPriority = 
    TIMx_PRE_EMPTION_PRIORITY;
  NVIC_InitHALLStructure.NVIC_IRQChannelSubPriority = TIMx_SUB_PRIORITY;
  NVIC_InitHALLStructure.NVIC_IRQChannelCmd = ENABLE;
  
  NVIC_Init(&NVIC_InitHALLStructure);
  
  /* Clear the TIMx's pending flags */
  TIMx->SR = 0u;
  
  /* Selected input capture and Update (overflow) events generate interrupt */
  TIM_ITConfig(TIMx, TIM_IT_CC1, ENABLE);
  TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE);
  
  TIM_SetCounter(TIMx, HALL_COUNTER_RESET);
  TIM_Cmd(TIMx, ENABLE);
  
  /* Erase speed buffer */
  bSpeedBufferSize = pDParams_str->bSpeedBufferSize;
  
  for (bIndex = 0u; bIndex < bSpeedBufferSize; bIndex++)
  {
    pDVars_str->SensorPeriod[bIndex].wPeriod = U32_MAX;
    pDVars_str->SensorPeriod[bIndex].bDirection = POSITIVE;
  }
}

/**
* @brief  Clear software FIFO where are "pushed" latest speed information
*         This function must be called before starting the motor to initialize
*	        the speed measurement process.
* @param  this related object of class CSPD
* @retval none
*/
static void HALL_Clear(CSPD this)
{
  pVars_t pVars_str = CLASS_VARS;
  pDVars_t pDVars_str = DCLASS_VARS;
  pDParams_t pDParams_str = DCLASS_PARAM;
  TIM_TypeDef* TIMx = pDParams_str->TIMx;
  
  /* Mask interrupts to insure a clean intialization */
  TIM_ITConfig(TIMx, TIM_IT_CC1, DISABLE);
  
  pDVars_str->RatioDec = FALSE;
  pDVars_str->RatioInc = FALSE;
  pDVars_str->HallTimeOut = FALSE;
  
  /* Reset speed reliability */
  pDVars_str->SensorIsReliable = TRUE;
  
  /* Acceleration measurement not implemented.*/
  pVars_str->hMecAccel01HzP = 0;
  
  pDVars_str->bFirstCapt = 0u;
  pDVars_str->bBufferFilled = 0u;
  pDVars_str->bOVFCounter = 0u;  

  pDVars_str->hCompSpeed = 0;
    
  pDVars_str->bSpeed = POSITIVE;
    
  /* Initialize pointers to speed buffer */
  pDVars_str->bSpeedFIFOSetIdx = 0u;
  pDVars_str->bSpeedFIFOGetIdx = 0u;
  
  /* Clear new speed acquisitions flag */
  pDVars_str->bNewSpeedAcquisition = 0;
  
  /* Re-initialize partly the timer */
  TIMx->PSC = pDVars_str->hHALLMaxRatio;
  
  TIM_SetCounter(TIMx, HALL_COUNTER_RESET);
  
  TIM_Cmd(TIMx, ENABLE);
  
  TIM_ITConfig(TIMx, TIM_IT_CC1, ENABLE);
  
  HALL_Init_Electrical_Angle(this);
}

/**
* @brief  Update the rotor electrical angle integrating the last measured 
*         instantaneous electrical speed express in dpp.
* @param  this related object of class CSPD.
* @param  pInputVars_str Not used in this derived class implementation of
*						 SPD_CalcElAngle
* @retval int16_t Measured electrical angle in s16degree format.
*/
static int16_t HALL_CalcElAngle(CSPD this, void *pInputVars_str)
{
  pVars_t pVars_str = CLASS_VARS;
  pDVars_t pDVars_str = DCLASS_VARS;
    
  if (pVars_str->hElSpeedDpp != HALL_MAX_PSEUDO_SPEED)
  {
    pDVars_str->hMeasuredElAngle += pVars_str->hElSpeedDpp;
    pDVars_str->hTargetElAngle += pVars_str->hElSpeedDpp;
    pVars_str->hElAngle += pVars_str->hElSpeedDpp + pDVars_str->hCompSpeed;
    pDVars_str->hPrevRotorFreq = pVars_str->hElSpeedDpp;
  }
  else
  {
    pVars_str->hElAngle += pDVars_str->hPrevRotorFreq;
  }
      
  return pVars_str->hElAngle;
}


/**
  * @brief  This method must be called - at least - with the same periodicity
  *         on which speed control is executed.
  *         This method compute and store rotor istantaneous el speed (express 
  *         in dpp considering the measurement frequency) in order to provide it
  *         to HALL_CalcElAngle function and SPD_GetElAngle. 
  *         Then compute rotor average el speed (express in dpp considering the 
  *         measurement frequency) based on the buffer filled by IRQ, then - as 
  *         a consequence - compute, store and return - through parameter 
  *         hMecSpeed01Hz - the rotor average mech speed, expressed in 01Hz.
  *         Then check, store and return the reliability state of
  *         the sensor; in this function the reliability is measured with 
  *         reference to specific parameters of the derived
  *         sensor (HALL) through internal variables managed by IRQ.
  * @param  this related object of class CSPD
  * @param  hMecSpeed01Hz pointer to int16_t, used to return the rotor average
  *         mechanical speed (01Hz)
  * @retval TRUE = sensor information is reliable
  *         FALSE = sensor information is not reliable
  */
static bool HALL_CalcAvrgMecSpeed01Hz(CSPD this, int16_t *hMecSpeed01Hz)
{
  pVars_t pVars_str = CLASS_VARS;
  pParams_t pParams_str = CLASS_PARAM;
  pDVars_t pDVars_str = DCLASS_VARS;
  pDParams_t pDParams_str = DCLASS_PARAM;
  TIM_TypeDef* TIMx = pDParams_str->TIMx;
  PeriodMeas_s PeriodMeasAux;
    
  /* If speed is not reliable the El and Mec speed is set to 0 */
  if (pDVars_str->HallTimeOut == TRUE)
  {
    pVars_str->hElSpeedDpp = 0;
    *hMecSpeed01Hz = 0;
  }
  else  
  {
    /* Computing the rotor istantaneous el speed */
    PeriodMeasAux = pDVars_str->SensorPeriod[pDVars_str->bSpeedFIFOGetIdx];
    
    if(PeriodMeasAux.bDirection != HALL_ERROR)
    {
      /* No errors have been detected during rotor speed information 
      extrapolation */
      if ( TIMx->PSC >= pDVars_str->hHALLMaxRatio )
      {                           
        /* At start-up or very low freq */
        /* Based on current prescaler value only */
        pVars_str->hElSpeedDpp = 0;
        *hMecSpeed01Hz = 0;
      }
      else
      {
        if( PeriodMeasAux.wPeriod > pDVars_str->wMaxPeriod) 
        {
          /* Speed is too low */
          pVars_str->hElSpeedDpp = 0;
          *hMecSpeed01Hz = 0;
        }
        else
        {  
          /*Avoid u32 DIV Overflow*/
          if ( PeriodMeasAux.wPeriod > (u32)pDVars_str->wSpeedOverflow )
          {
            pVars_str->hElSpeedDpp = (s16)((u16) (pDVars_str->wPseudoFreqConv /
                                                  PeriodMeasAux.wPeriod));
            pVars_str->hElSpeedDpp *= PeriodMeasAux.bDirection;
            
            #ifdef HALL_MTPA
            {
              pDVars_str->hCompSpeed = 0;
            }
            #else
            {
              pDVars_str->hTargetElAngle = pDVars_str->hMeasuredElAngle;
              pDVars_str->hDeltaAngle = pDVars_str->hMeasuredElAngle - pVars_str->hElAngle;
              pDVars_str->hCompSpeed = (int16_t)
                ((int32_t)(pDVars_str->hDeltaAngle)/
                 (int32_t)(pDVars_str->hPWMNbrPSamplingFreq));
            }
            #endif
            
            *hMecSpeed01Hz = HALL_CalcAvrgElSpeedDpp(this);
                        
            /* Converto el_dpp to Mec01Hz */
            *hMecSpeed01Hz = (int16_t)((*hMecSpeed01Hz * 
                              (int32_t)pParams_str->hMeasurementFrequency * 10)/
                              (65536 * (int32_t)pParams_str->bElToMecRatio));
            
          }
          else
          {
            pVars_str->hElSpeedDpp = HALL_MAX_PSEUDO_SPEED;
            *hMecSpeed01Hz = (int16_t)pDVars_str->hSatSpeed;
          }
        }
      }
    }              
  }
  
  pVars_str->hAvrMecSpeed01Hz = *hMecSpeed01Hz;
    
  return (pDVars_str->SensorIsReliable);
}

/**
* @brief  Example of private method of the class HALL to implement an MC IRQ function
* @param  this related object
* @param  flag used to distinguish between various IRQ sources
*         0: Capture Event
*         1: Update Event
* @retval none
*/
static void HALL_IRQHandler(void *this, unsigned char flag)
{
  pVars_t pVars_str = CLASS_VARS;
  pDVars_t pDVars_str = DCLASS_VARS;
  pDParams_t pDParams_str = DCLASS_PARAM;
  TIM_TypeDef* TIMx = pDParams_str->TIMx;
  uint8_t bPrevHallState;
  uint32_t wCaptBuf;
  uint16_t hPrscBuf;
  uint32_t wHighSpeedCapture;
  
  /* Check for the source of IRQ - Capture or Update Event - */
  if ( flag == 0u )
  {    
    /* A capture event generated this interrupt */
    uint8_t bAux;    
    
    bPrevHallState = pDVars_str->bHallState;
    
    pDVars_str->bHallState  = GPIO_ReadInputDataBit(pDParams_str->hH3Port, pDParams_str->hH3Pin)<<2;
    bAux = GPIO_ReadInputDataBit(pDParams_str->hH2Port, pDParams_str->hH2Pin)<<1;
    pDVars_str->bHallState |= bAux;
    pDVars_str->bHallState |= GPIO_ReadInputDataBit(pDParams_str->hH1Port, pDParams_str->hH1Pin);
    
    if (pDParams_str->bSensorPlacement == DEGREES_120) 
    {
      switch(pDVars_str->bHallState)
      {
      case STATE_5:
        if (bPrevHallState == STATE_5)
        {
          /* a speed reversal occured */
          if(pDVars_str->bSpeed<0)
          {
            pDVars_str->bSpeed = POSITIVE_SWAP;
          }
          else
          {
            pDVars_str->bSpeed = NEGATIVE_SWAP;
          }
        }
        else   
        {
          if (bPrevHallState == STATE_4)
          {
            pDVars_str->bSpeed = POSITIVE;
          }
          else
          {
            if (bPrevHallState == STATE_1)
            {
              pDVars_str->bSpeed = NEGATIVE;
            }
          }
        }
        /* Update angle */
        if(pDVars_str->bSpeed<0)
        {
          pDVars_str->hMeasuredElAngle = (s16)(pDParams_str->hPhaseShift+S16_60_PHASE_SHIFT);
        }
        else if(pDVars_str->bSpeed!= HALL_ERROR)
        {
          pDVars_str->hMeasuredElAngle = pDParams_str->hPhaseShift;  
        }
        else
        {
        }
        break;

      case STATE_1:
        if (bPrevHallState == STATE_1)
        {
          /* a speed reversal occured */
          if(pDVars_str->bSpeed<0)
          {
            pDVars_str->bSpeed = POSITIVE_SWAP;
          }
          else
          {
            pDVars_str->bSpeed = NEGATIVE_SWAP;
          }
        }
        else   
        {
          if (bPrevHallState == STATE_5)
          {
            pDVars_str->bSpeed = POSITIVE;
          }
          else
          {
            if (bPrevHallState == STATE_3)
            {
              pDVars_str->bSpeed = NEGATIVE;
            }
          }
        }
        /* Update angle */
        if(pDVars_str->bSpeed<0)
        {
          pDVars_str->hMeasuredElAngle = (s16)(pDParams_str->hPhaseShift+S16_120_PHASE_SHIFT);
        }
        else if(pDVars_str->bSpeed!= HALL_ERROR)
        {
          pDVars_str->hMeasuredElAngle = pDParams_str->hPhaseShift+S16_60_PHASE_SHIFT;  
        }
        else
        {
        }
        break;
        
      case STATE_3:
        if (bPrevHallState == STATE_3)
        {
          /* a speed reversal occured */
          if(pDVars_str->bSpeed<0)
          {
            pDVars_str->bSpeed = POSITIVE_SWAP;
          }
          else
          {
            pDVars_str->bSpeed = NEGATIVE_SWAP;
          }
        }
        else
          if (bPrevHallState == STATE_1)
          {
            pDVars_str->bSpeed = POSITIVE;
          }
          else if (bPrevHallState == STATE_2)
          {
            pDVars_str->bSpeed = NEGATIVE;
          }
          else
          {
          }
        /* Update of the electrical angle */
        if(pDVars_str->bSpeed<0)
        {
          pDVars_str->hMeasuredElAngle = (s16)(pDParams_str->hPhaseShift+S16_120_PHASE_SHIFT+
                                    S16_60_PHASE_SHIFT);
        }
        else if(pDVars_str->bSpeed!= HALL_ERROR)
        {
          pDVars_str->hMeasuredElAngle =(s16)(pDParams_str->hPhaseShift + S16_120_PHASE_SHIFT);
        }
        else
        {
        }
        break;  

      case STATE_2:
        if (bPrevHallState == STATE_2)
        {
          /* a speed reversal occured */
          if(pDVars_str->bSpeed<0)
          {
            pDVars_str->bSpeed = POSITIVE_SWAP;
          }
          else
          {
            pDVars_str->bSpeed = NEGATIVE_SWAP;
          }
        }
        else
          if (bPrevHallState == STATE_3)
          {
            pDVars_str->bSpeed = POSITIVE;
          }
          else if (bPrevHallState == STATE_6)
          {
            pDVars_str->bSpeed = NEGATIVE;
          }
          else
          {
          }
        /* Update of the electrical angle */
        if(pDVars_str->bSpeed<0)
        {
          pDVars_str->hMeasuredElAngle = (s16)(pDParams_str->hPhaseShift-S16_120_PHASE_SHIFT);
        }
        else if(pDVars_str->bSpeed!= HALL_ERROR)
        {
          pDVars_str->hMeasuredElAngle =(s16)(pDParams_str->hPhaseShift + S16_120_PHASE_SHIFT
                                     + S16_60_PHASE_SHIFT);
        }
        else
        {
        }
        break;  
        
      case STATE_6: 
        if (bPrevHallState == STATE_6)
        {
          if(pDVars_str->bSpeed<0)
          {
            pDVars_str->bSpeed = POSITIVE_SWAP;
          }
          else
          {
            pDVars_str->bSpeed = NEGATIVE_SWAP;
          }
        }
        
        if (bPrevHallState == STATE_2)
        {
          pDVars_str->bSpeed = POSITIVE; 
        }
        else if (bPrevHallState == STATE_4)
        {
          pDVars_str->bSpeed = NEGATIVE;
        }
        else
        {
        }
        if(pDVars_str->bSpeed<0)
        {
          pDVars_str->hMeasuredElAngle =(s16)(pDParams_str->hPhaseShift - S16_60_PHASE_SHIFT);  
        }
        else if(pDVars_str->bSpeed!= HALL_ERROR)
        {
          pDVars_str->hMeasuredElAngle =(s16)(pDParams_str->hPhaseShift - S16_120_PHASE_SHIFT); 
        }
        else
        {
        }
        break;

      case STATE_4: 
        if (bPrevHallState == STATE_4)
        {
          if(pDVars_str->bSpeed<0)
          {
            pDVars_str->bSpeed = POSITIVE_SWAP;
          }
          else
          {
            pDVars_str->bSpeed = NEGATIVE_SWAP;
          }
        }
        
        if (bPrevHallState == STATE_6)
        {
          pDVars_str->bSpeed = POSITIVE; 
        }
        else if (bPrevHallState == STATE_5)
        {
          pDVars_str->bSpeed = NEGATIVE;
        }
        else
        {
        }
        if(pDVars_str->bSpeed<0)
        {
          pDVars_str->hMeasuredElAngle =(s16)(pDParams_str->hPhaseShift);  
        }
        else if(pDVars_str->bSpeed!= HALL_ERROR)
        {
          pDVars_str->hMeasuredElAngle =(s16)(pDParams_str->hPhaseShift - S16_60_PHASE_SHIFT); 
        }
        else
        {
        }
        break;
        
      default:
        pDVars_str->bSpeed = HALL_ERROR;
        
        /* Bad hall sensor configutarion so update the speed reliability */
        pDVars_str->SensorIsReliable = FALSE;
        
        break;
      }
    }
    else if (pDParams_str->bSensorPlacement == DEGREES_60)
    {
      switch(pDVars_str->bHallState)
      {
      case STATE_1:
        if (bPrevHallState == STATE_1)
        {
          if(pDVars_str->bSpeed<0)
          {
            pDVars_str->bSpeed = POSITIVE_SWAP;
          }
          else
          {
            pDVars_str->bSpeed = NEGATIVE_SWAP;
          }
        }
        else          
          if (bPrevHallState == STATE_0)
          {
            pDVars_str->bSpeed = POSITIVE;
          }
          else if (bPrevHallState == STATE_3)
          {
            pDVars_str->bSpeed = NEGATIVE;              
          }
          else
          {
          }
        if(pDVars_str->bSpeed<0)
        {
          pDVars_str->hMeasuredElAngle = (s16)(pDParams_str->hPhaseShift+S16_60_PHASE_SHIFT);
        }
        else if(pDVars_str->bSpeed!= HALL_ERROR)
        {
          pDVars_str->hMeasuredElAngle = (s16)(pDParams_str->hPhaseShift);
        }
        else
        {
        }
        break;
        
      case STATE_3:
        if (bPrevHallState == STATE_3)
        {
          if(pDVars_str->bSpeed<0)
          {
            pDVars_str->bSpeed = POSITIVE_SWAP;
          }
          else
          {
            pDVars_str->bSpeed = NEGATIVE_SWAP;
          }
        }
        else          
          if (bPrevHallState == STATE_1)
          {
            pDVars_str->bSpeed = POSITIVE;
          }
          else if (bPrevHallState == STATE_7)
          {
            pDVars_str->bSpeed = NEGATIVE;              
          }
          else
          {
          }
        if(pDVars_str->bSpeed<0)
        {
          pDVars_str->hMeasuredElAngle = (s16)(pDParams_str->hPhaseShift+S16_120_PHASE_SHIFT);
        }
        else if(pDVars_str->bSpeed!= HALL_ERROR)
        {
          pDVars_str->hMeasuredElAngle = (s16)(pDParams_str->hPhaseShift+S16_60_PHASE_SHIFT);
        }
        else
        {
        }
        break;

      case STATE_7:
        if (bPrevHallState == STATE_7)
        {
          if(pDVars_str->bSpeed<0)
          {
            pDVars_str->bSpeed = POSITIVE_SWAP;
          }
          else
          {
            pDVars_str->bSpeed = NEGATIVE_SWAP;
          }
        }
        else          
          if (bPrevHallState == STATE_3)
          {
            pDVars_str->bSpeed = POSITIVE;
          }
          else if (bPrevHallState == STATE_6)
          {
            pDVars_str->bSpeed = NEGATIVE;              
          }
          else
          {
          }
        if(pDVars_str->bSpeed<0)
        {
          pDVars_str->hMeasuredElAngle = (s16)(pDParams_str->hPhaseShift+S16_120_PHASE_SHIFT 
                                      + S16_60_PHASE_SHIFT);
        }
        else if(pDVars_str->bSpeed!= HALL_ERROR)
        {
          pDVars_str->hMeasuredElAngle = (s16)(pDParams_str->hPhaseShift+S16_120_PHASE_SHIFT);
        }
        else
        {
        }
        break;
        
      case STATE_6:
        if (bPrevHallState == STATE_6)
        {
          if(pDVars_str->bSpeed<0)
          {
            pDVars_str->bSpeed = POSITIVE_SWAP;
          }
          else
          {
            pDVars_str->bSpeed = NEGATIVE_SWAP;
          }
        } 
        else
          if (bPrevHallState == STATE_7)
          {
            pDVars_str->bSpeed = POSITIVE;           
          }
          else if (bPrevHallState == STATE_4)
          {
            pDVars_str->bSpeed = NEGATIVE;
          }
          else
          {
          }
        if(pDVars_str->bSpeed<0)
        {
          pDVars_str->hMeasuredElAngle = (s16)(pDParams_str->hPhaseShift-S16_120_PHASE_SHIFT);
        }
        else if(pDVars_str->bSpeed!= HALL_ERROR)
        {
          pDVars_str->hMeasuredElAngle =(s16)(pDParams_str->hPhaseShift + S16_120_PHASE_SHIFT+
                                     S16_60_PHASE_SHIFT);
        }
        else
        {
        }
        break;  
        
      case STATE_4:
        if (bPrevHallState == STATE_4)
        {
          if(pDVars_str->bSpeed<0)
          {
            pDVars_str->bSpeed = POSITIVE_SWAP;
          }
          else
          {
            pDVars_str->bSpeed = NEGATIVE_SWAP;
          }
        } 
        else
          if (bPrevHallState == STATE_6)
          {
            pDVars_str->bSpeed = POSITIVE;           
          }
          else if (bPrevHallState == STATE_0)
          {
            pDVars_str->bSpeed = NEGATIVE;
          }
          else
          {
          }
        if(pDVars_str->bSpeed<0)
        {
          pDVars_str->hMeasuredElAngle = (s16)(pDParams_str->hPhaseShift-S16_60_PHASE_SHIFT);
        }
        else if(pDVars_str->bSpeed!= HALL_ERROR)
        {
          pDVars_str->hMeasuredElAngle =(s16)(pDParams_str->hPhaseShift - S16_120_PHASE_SHIFT);
        }
        else
        {
        }
        break;
        
      case STATE_0:
        if (bPrevHallState == STATE_0)
        {
          if(pDVars_str->bSpeed<0)
          {
            pDVars_str->bSpeed = POSITIVE_SWAP;
          }
          else
          {
            pDVars_str->bSpeed = NEGATIVE_SWAP;
          }
        } 
        else
          if (bPrevHallState == STATE_4)
          {
            pDVars_str->bSpeed = POSITIVE;
          }
          else if (bPrevHallState == STATE_1)
          {
            pDVars_str->bSpeed = NEGATIVE;
          }
          else
          {
          }
        
        if(pDVars_str->bSpeed<0)
        {
          pDVars_str->hMeasuredElAngle =(s16)(pDParams_str->hPhaseShift );  
        }
        else if(pDVars_str->bSpeed!= HALL_ERROR)
        {
          pDVars_str->hMeasuredElAngle =(s16)(pDParams_str->hPhaseShift - S16_60_PHASE_SHIFT);  
        }
        else
        {
        }
        break;
        
      default:
        pDVars_str->bSpeed = HALL_ERROR;
        
        /* Bad hall sensor configutarion so update the speed reliability */
        pDVars_str->SensorIsReliable = FALSE;
        
        break;
      }                         
    }
    else
    {
    }
    
    #ifdef HALL_MTPA
    {
      pVars_str->hElAngle = pDVars_str->hMeasuredElAngle;
    }
    #endif
    
    /* Discard first capture */
    if (pDVars_str->bFirstCapt == 0u)
    {
      pDVars_str->bFirstCapt++;
      TIM_GetCapture1(TIMx);
    }
    else
    {      
      /* used to validate the average speed measurement */
      if (pDVars_str->bBufferFilled < pDParams_str->bSpeedBufferSize)
      {
        pDVars_str->bBufferFilled++;
      }
            
      /* Store the latest speed acquisition */    
      wHighSpeedCapture = TIM_GetCapture1(TIMx);
      wCaptBuf = wHighSpeedCapture;
      hPrscBuf = TIMx->PSC;
            
      /* Add the numbers of overflow to the counter */
      wCaptBuf += (uint32_t)pDVars_str->bOVFCounter * 0x10000uL;
      
      if (pDVars_str->bOVFCounter != 0u)
      {
        /* Adjust the capture using prescaler */
        uint16_t hAux;
        hAux = hPrscBuf + 1u;
        wCaptBuf *= hAux;
        
        if (pDVars_str->RatioInc)
        {
          pDVars_str->RatioInc = FALSE;	/* Previous capture caused overflow */
          /* Don't change prescaler (delay due to preload/update mechanism) */
        }
        else
        {
          if ((TIMx->PSC) < pDVars_str->hHALLMaxRatio) /* Avoid OVF w/ very low freq */
          {
            (TIMx->PSC)++; /* To avoid OVF during speed decrease */
            pDVars_str->RatioInc = TRUE;	  /* new prsc value updated at next capture only */
          }
        }
      }
      else
      {
        /* If prsc preload reduced in last capture, store current register + 1 */
        if (pDVars_str->RatioDec)  /* and don't decrease it again */
        {
          /* Adjust the capture using prescaler */
          uint16_t hAux;
          hAux = hPrscBuf + 2u;
          wCaptBuf *= hAux;
          
          pDVars_str->RatioDec = FALSE;
        }
        else  /* If prescaler was not modified on previous capture */
        {
          /* Adjust the capture using prescaler */
          uint16_t hAux = hPrscBuf + 1u;
          wCaptBuf *= hAux;
          
          if (wHighSpeedCapture < LOW_RES_THRESHOLD)/* If capture range correct */
          {
            if(TIMx->PSC > 0u) /* or prescaler cannot be further reduced */
            {
              (TIMx->PSC)--;	/* Increase accuracy by decreasing prsc */
              /* Avoid decrementing again in next capt.(register preload delay) */
              pDVars_str->RatioDec = TRUE;
            }
          }
        }
      }
            
      /* Store into the buffer */
      pDVars_str->SensorPeriod[pDVars_str->bSpeedFIFOSetIdx].wPeriod = wCaptBuf;
      pDVars_str->SensorPeriod[pDVars_str->bSpeedFIFOSetIdx].bDirection = pDVars_str->bSpeed;
            
      /* Reset the number of overflow occurred */
      pDVars_str->bOVFCounter = 0u;
      
      /* Timeout Flag is cleared when receiving an IC */
      pDVars_str->HallTimeOut = FALSE;
      
      /* Update pointers to speed buffer */
      pDVars_str->bSpeedFIFOGetIdx = pDVars_str->bSpeedFIFOSetIdx;
      pDVars_str->bSpeedFIFOSetIdx++;
      if (pDVars_str->bSpeedFIFOSetIdx == pDParams_str->bSpeedBufferSize)
      {
        pDVars_str->bSpeedFIFOSetIdx = 0u;        
      }
      
      /* Indicate new speed acquisitions */
      pDVars_str->bNewSpeedAcquisition = 1;
    }
  }
  else if (flag == 1u)
  {
    uint16_t hMaxTimerOverflow;
  	/* an update event occured for this interrupt request generation */
    pDVars_str->bOVFCounter++;
    
    hMaxTimerOverflow = (uint16_t)(((uint32_t)pDVars_str->hHallTimeout * pDVars_str->hOvfDuration)
      /(((uint32_t)TIMx->PSC + 1u) * 1000u));
    if (pDVars_str->bOVFCounter >= hMaxTimerOverflow)
    {
      pDVars_str->HallTimeOut = TRUE;
      
      /* Set rotor speed to zero */
      pVars_str->hElSpeedDpp = 0;
      
      /* Reset the electrical angle according the hall sensor configuration */
      HALL_Init_Electrical_Angle(this);
      
      /* Reset the overflow counter */
      pDVars_str->bOVFCounter = 0u;
    }    
  }
  else
  {
  }
}

/**
* @brief  Compute and returns the average rotor electrical speed express in dpp
* @param  this related object of class CSPD
* @retval int16_t the average rotor electrical speed express in dpp
*/
static int16_t HALL_CalcAvrgElSpeedDpp(CSPD this)
{
  pDVars_t pDVars_str = DCLASS_VARS;
  pDParams_t pDParams_str = DCLASS_PARAM;
  TIM_TypeDef* TIMx = pDParams_str->TIMx;
  uint8_t bIndex;
  int32_t wElBuffer,wFreqBuffer;
  int16_t hAvrgElSpeed;
  uint8_t bSpeedBufferSize;
  
  if (pDVars_str->bNewSpeedAcquisition == 1)
  {
    
    if (pDVars_str->bBufferFilled < pDParams_str->bSpeedBufferSize)
    {
      uint32_t wAux;
      /*Disable capture interrupts to have presc and capture of the same period*/
      TIMx->DIER &= (u16)~TIM_IT_CC1; /* NB:Std libray not used for perf issues*/
      
      wAux = pDVars_str->wPseudoFreqConv
               /pDVars_str->SensorPeriod[pDVars_str->bSpeedFIFOGetIdx].wPeriod;
      hAvrgElSpeed = (int16_t)(wAux);
      hAvrgElSpeed *= pDVars_str->SensorPeriod[pDVars_str->bSpeedFIFOGetIdx].bDirection;
      
      TIMx->DIER |= TIM_IT_CC1;   /* NB:Std libray not used for perf issue*/
    }
    else
    {
      wElBuffer = 0;
      bSpeedBufferSize = pDParams_str->bSpeedBufferSize;
      for (bIndex = 0u; bIndex < bSpeedBufferSize; bIndex++ )
      {
        /*Disable capture interrupts to have presc and capture of the same period*/
        TIMx->DIER &= (u16)~TIM_IT_CC1; /* NB:Std libray not used for perf issues*/
        
        wFreqBuffer = (int32_t)pDVars_str->SensorPeriod[bIndex].wPeriod;      
        wFreqBuffer *= pDVars_str->SensorPeriod[bIndex].bDirection;
        
        TIMx->DIER |= TIM_IT_CC1;   /* NB:Std libray not used for perf issue*/
        wElBuffer += wFreqBuffer;	/* Sum the whole periods FIFO */
      }
      
      wElBuffer /= (int32_t)(bSpeedBufferSize);        /* Average value */
      hAvrgElSpeed = (int16_t)((int32_t)(pDVars_str->wPseudoFreqConv)/wElBuffer);
    }
    
    pDVars_str->hAvrElSpeedDpp = hAvrgElSpeed;
        
    /* Clear new speed acquisitions flag */
    pDVars_str->bNewSpeedAcquisition = 0;
  }
  
  return pDVars_str->hAvrElSpeedDpp;
}

/**
* @brief  Read the logic level of the three Hall sensor and individuates in this 
*         way the position of the rotor (+/- 30°). Electrical angle is then 
*         initialized.
* @param  this related object of class CSPD
* @retval none
*/
static void HALL_Init_Electrical_Angle(CSPD this)
{
  pVars_t pVars_str = CLASS_VARS;
  pDVars_t pDVars_str = DCLASS_VARS;
  pDParams_t pDParams_str = DCLASS_PARAM;
  
  u8 bAux;
  
  pDVars_str->bHallState  = GPIO_ReadInputDataBit(pDParams_str->hH3Port, pDParams_str->hH3Pin)<<2;
  bAux = GPIO_ReadInputDataBit(pDParams_str->hH2Port, pDParams_str->hH2Pin)<<1;
  pDVars_str->bHallState |= bAux;
  pDVars_str->bHallState |= GPIO_ReadInputDataBit(pDParams_str->hH1Port, pDParams_str->hH1Pin);
  
  if (pDParams_str->bSensorPlacement == DEGREES_120) 
  {
    switch(pDVars_str->bHallState)
    {
    case STATE_5:
      pVars_str->hElAngle = (s16)(pDParams_str->hPhaseShift+S16_60_PHASE_SHIFT/2);
      break;
    case STATE_1:
      pVars_str->hElAngle =(s16)(pDParams_str->hPhaseShift+S16_60_PHASE_SHIFT+
                               S16_60_PHASE_SHIFT/2);
      break;
    case STATE_3:
      pVars_str->hElAngle =(s16)(pDParams_str->hPhaseShift+S16_120_PHASE_SHIFT+
                               S16_60_PHASE_SHIFT/2);      
      break;
    case STATE_2:
      pVars_str->hElAngle =(s16)(pDParams_str->hPhaseShift-S16_120_PHASE_SHIFT-
                               S16_60_PHASE_SHIFT/2);      
      break;
    case STATE_6:
      pVars_str->hElAngle =(s16)(pDParams_str->hPhaseShift-S16_60_PHASE_SHIFT-
                               S16_60_PHASE_SHIFT/2);          
      break;
    case STATE_4:
      pVars_str->hElAngle =(s16)(pDParams_str->hPhaseShift-S16_60_PHASE_SHIFT/2);          
      break;    
    default:
      /* Bad hall sensor configutarion so update the speed reliability */
      pDVars_str->SensorIsReliable = FALSE;
      break;
    }
  }
  else if (pDParams_str->bSensorPlacement == DEGREES_60)
  {
    switch(pDVars_str->bHallState)
    {  
    case STATE_1:
      pVars_str->hElAngle =(s16)(pDParams_str->hPhaseShift+S16_60_PHASE_SHIFT/2);
      break;
    case STATE_3:
      pVars_str->hElAngle =(s16)(pDParams_str->hPhaseShift+S16_60_PHASE_SHIFT+
                               S16_60_PHASE_SHIFT/2);
      break;
    case STATE_7:
      pVars_str->hElAngle =(s16)(pDParams_str->hPhaseShift+S16_120_PHASE_SHIFT+
                               S16_60_PHASE_SHIFT/2);      
      break;
    case STATE_6:
      pVars_str->hElAngle =(s16)(pDParams_str->hPhaseShift-S16_120_PHASE_SHIFT-
                               S16_60_PHASE_SHIFT/2);      
      break;
    case STATE_4:
      pVars_str->hElAngle =(s16)(pDParams_str->hPhaseShift-S16_60_PHASE_SHIFT-
                               S16_60_PHASE_SHIFT/2);          
      break;
    case STATE_0:
      pVars_str->hElAngle =(s16)(pDParams_str->hPhaseShift-S16_60_PHASE_SHIFT/2);          
      break;    
    default:   
      /* Bad hall sensor configutarion so update the speed reliability */
      pDVars_str->SensorIsReliable = FALSE;
      break;
    }
  }
  else
  {
  }
  
  /* Initialize the measured angle */
  pDVars_str->hMeasuredElAngle = pVars_str->hElAngle;
  
}

/**
  * @brief  It could be used to set istantaneous information on rotor mechanical
  *         angle.
  *         Note: Mechanical angle management is not implemented in this 
  *         version of Hall sensor class.
  * @param  this related object of class CSPD
  * @param  hMecAngle istantaneous measure of rotor mechanical angle
  * @retval none
  */
static void HALL_SetMecAngle(CSPD this, int16_t hMecAngle)
{
}

/**
  * @brief  It is an internal function used to compute the GPIO Source 
  *         value starting from GPIO pin value. The GPIO Source value 
  *         is used for AF remapping.
  * @param  GPIO_Pin Pin value to be converted.
  * @retval uint16_t The GPIO pin source value converted.
  */
static uint16_t F4XX_GPIOPin2Source(uint16_t GPIO_Pin)
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
