/**
  ******************************************************************************
  * @file    ICS_F4XX_PWMnCurrFdbkClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private implementation of current sensor class in 
  *          case of dual motors and in case ICS and STM32F103 High Density is 
  *          used           
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
#include "PWMnCurrFdbkClass.h"
#include "PWMnCurrFdbkPrivate.h"
#include "ICS_F4XX_PWMnCurrFdbkClass.h"
#include "ICS_F4XX_PWMnCurrFdbkPrivate.h"
#include "MCIRQHandlerClass.h"
#include "MCIRQHandlerPrivate.h"
#include "MCLibraryConf.h"
#include "MCLibraryISRPriorityConf.h"
#include "MC_type.h"

/* ADC SMPx mask */
#define SMPR1_SMP_Set              ((uint32_t) (0x00000007u))
#define SMPR2_SMP_Set              ((uint32_t) (0x00000007u))
/* Bit Banding */
#define ADC1_CR2_SWSTART_BB 0x42240178u

/* ADC registers reset values */
#define ADC_GENERAL_RESET_VALUE    ((uint32_t) (0x00000000u))
#define ADC_HTR_RESET_VALUE        ((uint32_t) (0x00000FFFu))

#define TIMxCCER_MASK              ((uint16_t)  ~0x1555u)
#define TIMxCCER_MASK_CH123        ((uint16_t)  0x555u)

#define TIMx_CC4E_BIT              ((uint16_t)  0x1000u) 

#define CONV_STARTED               ((uint32_t) (0x8))
#define CONV_FINISHED              ((uint32_t) (0xC))
#define FLAGS_CLEARED              ((uint32_t) (0x0))
#define ADC_SR_MASK                ((uint32_t) (0xC))

#define ADC_RIGHT_ALIGNMENT 3u

#define NB_CONVERSIONS 16u

#define CLASS_VARS   ((_CPWMC)this)->Vars_str
#define DCLASS_PARAMS ((_DCIF4XX_PWMC)(((_CPWMC) this)->DerivedClass))->pDParams_str
#define DCLASS_VARS  ((_DCIF4XX_PWMC)(((_CPWMC) this)->DerivedClass))->DVars_str

#ifdef MC_CLASS_DYNAMIC
#include "stdlib.h" /* Used for dynamic allocation */
#else
_DCIF4XX_PWMC_t IF4XX_PWMCpool[MAX_DRV_PWMC_NUM];
unsigned char IF4XX_PWMC_Allocated = 0u;
#endif

/*#define DEBUG*/

#ifdef DEBUG
static volatile _CPWMC oPWMCdbg;
static volatile _DCIF4XX_PWMC oIF4XXdbg;
#endif

static void IF4XX_Init(CPWMC this);
static void IF4XX_TIMxInit(TIM_TypeDef* TIMx, CPWMC this);
static void IF4XX_CurrentReadingCalibration(CPWMC this);
static void IF4XX_GetPhaseCurrents(CPWMC this,Curr_Components* pStator_Currents);
static void IF4XX_TurnOnLowSides(CPWMC this);
static void IF4XX_SwitchOnPWM(CPWMC this);
static void IF4XX_SwitchOffPWM(CPWMC this);
static uint16_t IF4XX_WriteTIMRegisters(CPWMC this);   
static void *IF4XX_IRQHandler(void *this, unsigned char flag);
static uint16_t IF4XX_ExecRegularConv(CPWMC this, uint8_t bChannel);
static void IF4XX_ADC_SetSamplingTime(CPWMC this, ADConv_t ADConv_struct);
static void IF4XX_HFCurrentsCalibration(CPWMC this,Curr_Components* pStator_Currents);
static uint16_t IF4XX_IsOverCurrentOccurred(CPWMC this);
static uint16_t F4XX_GPIOPin2Source(uint16_t GPIO_Pin);
static void IF4XX_ADC_Deinit(ADC_TypeDef* Local_ADC);
                              
/**
* @brief  Creates an object of the class ICS_F4XX
* @param  pPWMnCurrFdbkParams pointer to an PWMnCurrFdbk parameters structure
* @param  pICS_DDParams pointer to an ICS_DD parameters structure
* @retval CIF4XX_PWMC new instance of ICS_F4XX object
*/
CIF4XX_PWMC IF4XX_NewObject(pPWMnCurrFdbkParams_t pPWMnCurrFdbkParams, 
                                    pICS_DDParams_t pICS_DDParams)
{
  _CPWMC _oPWMnCurrFdbk;
  _DCIF4XX_PWMC _oICS_F4XX;
  
  _oPWMnCurrFdbk = (_CPWMC)PWMC_NewObject(pPWMnCurrFdbkParams);
  
#ifdef MC_CLASS_DYNAMIC
  _oICS_F4XX = (_DCIF4XX_PWMC)calloc(1u,sizeof(_DCIF4XX_PWMC_t));
#else
  if (IF4XX_PWMC_Allocated  < MAX_DRV_PWMC_NUM)
  {
    _oICS_F4XX = &IF4XX_PWMCpool[IF4XX_PWMC_Allocated++];
  }
  else
  {
    _oICS_F4XX = MC_NULL;
  }
#endif
  
  _oICS_F4XX->pDParams_str = pICS_DDParams;
  _oPWMnCurrFdbk->DerivedClass = (void*)_oICS_F4XX;
  
  _oPWMnCurrFdbk->Methods_str.pIRQ_Handler = &IF4XX_IRQHandler;
  
  Set_IRQ_Handler(pICS_DDParams->IRQnb, (_CMCIRQ)_oPWMnCurrFdbk);
  
  _oPWMnCurrFdbk->Methods_str.pPWMC_Init = &IF4XX_Init;
  _oPWMnCurrFdbk->Methods_str.pPWMC_GetPhaseCurrents = &IF4XX_GetPhaseCurrents;
  _oPWMnCurrFdbk->Methods_str.pPWMC_SwitchOffPWM = &IF4XX_SwitchOffPWM;
  _oPWMnCurrFdbk->Methods_str.pPWMC_SwitchOnPWM = &IF4XX_SwitchOnPWM;        
  _oPWMnCurrFdbk->Methods_str.pPWMC_CurrentReadingCalibr = 
                                                 &IF4XX_CurrentReadingCalibration;         
  _oPWMnCurrFdbk->Methods_str.pPWMC_TurnOnLowSides = &IF4XX_TurnOnLowSides;         
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect1 = 
                                                      &IF4XX_WriteTIMRegisters;        
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect2 = 
                                                      &IF4XX_WriteTIMRegisters; 
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect3 = 
                                                      &IF4XX_WriteTIMRegisters;        
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect4 = 
                                                      &IF4XX_WriteTIMRegisters;         
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect5 = 
                                                      &IF4XX_WriteTIMRegisters;        
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect6 = 
                                                      &IF4XX_WriteTIMRegisters; 
  _oPWMnCurrFdbk->Methods_str.pPWMC_ExecRegularConv= &IF4XX_ExecRegularConv;
  
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetSamplingTime= &IF4XX_ADC_SetSamplingTime;
  
  _oPWMnCurrFdbk->Methods_str.pPWMC_IsOverCurrentOccurred = 
    &IF4XX_IsOverCurrentOccurred;
  
  return ((CIF4XX_PWMC)_oPWMnCurrFdbk);
}

/** @addtogroup STM32F10x_PMSM_MC_Library
* @{
*/

/** @addtogroup PWMnCurrFdbk_ICS_F4XX
* @{
*/

/** @defgroup ICS_F4XX_class_private_methods ICS_F4XX class private methods
* @{
*/

/**
* @brief  It initializes TIMx, ADC, GPIO and NVIC for current reading 
*         in ICS configuration using STM32F4XX
* @param  this: related object of class CIF4XX_PWMC
* @retval none
*/
static void IF4XX_Init(CPWMC this)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  ADC_InitTypeDef ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  pVars_t pVars_str = &CLASS_VARS;
  pDVars_t pDVars_str = &DCLASS_VARS;  
  pDParams_t pDParams_str = DCLASS_PARAMS; 
  uint8_t bGPIOAF =  GPIO_AF_TIM1;
  
  pDVars_str->Half_PWMPeriod = ((((_CPWMC) this)->pParams_str->hPWMperiod)/2u);
  pVars_str->bMotor = (pDParams_str->bInstanceNbr==1u?M1:M2);
    
  /* Peripheral clocks enabling ---------------------------------------------*/
  
  RCC->AHB1ENR |= RCC_AHB1Periph_CRC;
  
  /* Enable ADC1 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  /* Enable ADC2 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE); 
  /* Enable GPIOA-GPIOI clock */
   RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA | 
                         RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | 
                           RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE | 
                             RCC_AHB1Periph_GPIOF | RCC_AHB1Periph_GPIOG |
                             RCC_AHB1Periph_GPIOH | RCC_AHB1Periph_GPIOI, ENABLE);  
   
  /* Enable the CCS */
  RCC_ClockSecuritySystemCmd((FunctionalState)(ENABLE));
  
  if(pDParams_str->TIMx == TIM1)
  {
    /* Enable TIM1 clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  }
  else
  {
    /* Enable TIM8 clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
    /* Used for GPIO AF remap */
    bGPIOAF = GPIO_AF_TIM8;
  }
	
	IF4XX_TIMxInit(pDParams_str->TIMx, this);
  
  /* GPIOs configurations --------------------------------------------------*/
  GPIO_StructInit(&GPIO_InitStructure);
  
  /****** Configure phase A ADC channel GPIO as analog input ****/
  GPIO_InitStructure.GPIO_Pin = pDParams_str->hIaPin;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(pDParams_str->hIaPort,
            &GPIO_InitStructure);
  GPIO_PinLockConfig(pDParams_str->hIaPort, pDParams_str->hIaPin);
  
  /****** Configure phase B ADC channel GPIO as analog input ****/
  GPIO_InitStructure.GPIO_Pin = pDParams_str->hIbPin;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_Init(pDParams_str->hIbPort, &GPIO_InitStructure);
  GPIO_PinLockConfig(pDParams_str->hIbPort, pDParams_str->hIbPin);
  
  /****** Configure TIMx Channel 1, 2 and 3 Outputs ******/ 
  GPIO_PinAFConfig(pDParams_str->hCh1Port, F4XX_GPIOPin2Source(pDParams_str->hCh1Pin), bGPIOAF);
  GPIO_PinAFConfig(pDParams_str->hCh2Port, F4XX_GPIOPin2Source(pDParams_str->hCh2Pin), bGPIOAF);
  GPIO_PinAFConfig(pDParams_str->hCh3Port, F4XX_GPIOPin2Source(pDParams_str->hCh3Pin), bGPIOAF);
   
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  GPIO_InitStructure.GPIO_Pin = pDParams_str->hCh1Pin;
  GPIO_Init(pDParams_str->hCh1Port, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = pDParams_str->hCh2Pin;
  GPIO_Init(pDParams_str->hCh2Port, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = pDParams_str->hCh3Pin;
  GPIO_Init(pDParams_str->hCh3Port, &GPIO_InitStructure);
  
  GPIO_PinLockConfig(pDParams_str->hCh1Port, pDParams_str->hCh1Pin);
  GPIO_PinLockConfig(pDParams_str->hCh2Port, pDParams_str->hCh2Pin);
  GPIO_PinLockConfig(pDParams_str->hCh3Port, pDParams_str->hCh3Pin);

  /****** Configure TIMx Channel 1N, 2N and 3N Outputs, if enabled ******/    
  if ((pDParams_str->LowSideOutputs)== LS_PWM_TIMER) 
  { 
    GPIO_PinAFConfig(pDParams_str->hCh1NPort, F4XX_GPIOPin2Source(pDParams_str->hCh1NPin), bGPIOAF);
    GPIO_PinAFConfig(pDParams_str->hCh2NPort, F4XX_GPIOPin2Source(pDParams_str->hCh2NPin), bGPIOAF);
    GPIO_PinAFConfig(pDParams_str->hCh3NPort, F4XX_GPIOPin2Source(pDParams_str->hCh3NPin), bGPIOAF);
    
    GPIO_InitStructure.GPIO_Pin = pDParams_str->hCh1NPin;  
    GPIO_Init(pDParams_str->hCh1NPort, &GPIO_InitStructure);  
    GPIO_InitStructure.GPIO_Pin = pDParams_str->hCh2NPin;  
    GPIO_Init(pDParams_str->hCh2NPort, &GPIO_InitStructure);    
    GPIO_InitStructure.GPIO_Pin = pDParams_str->hCh3NPin;  
    GPIO_Init(pDParams_str->hCh3NPort, &GPIO_InitStructure);
    
    GPIO_PinLockConfig(pDParams_str->hCh1NPort, pDParams_str->hCh1NPin);
    GPIO_PinLockConfig(pDParams_str->hCh2NPort, pDParams_str->hCh2NPin);
    GPIO_PinLockConfig(pDParams_str->hCh3NPort, pDParams_str->hCh3NPin);
  }  else if ((pDParams_str->LowSideOutputs)== ES_GPIO)
  {
    /* Only "active high" polarity is supported */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    
    GPIO_InitStructure.GPIO_Pin = pDParams_str->hCh1NPin;
    GPIO_Init(pDParams_str->hCh1NPort, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = pDParams_str->hCh2NPin;
    GPIO_Init(pDParams_str->hCh2NPort, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = pDParams_str->hCh3NPin;
    GPIO_Init(pDParams_str->hCh3NPort, &GPIO_InitStructure);
    
    GPIO_PinLockConfig(pDParams_str->hCh1NPort, pDParams_str->hCh1NPin);
    GPIO_PinLockConfig(pDParams_str->hCh2NPort, pDParams_str->hCh2NPin);
    GPIO_PinLockConfig(pDParams_str->hCh3NPort, pDParams_str->hCh3NPin);
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  }
  else
  {
  }
  
  /****** Configure TIMx BKIN input, if enabled ******/
  if ((pDParams_str->EmergencyStop)!= DISABLE)  
  {
    GPIO_PinAFConfig(pDParams_str->hBKINPort, F4XX_GPIOPin2Source(pDParams_str->hBKINPin), bGPIOAF);
    GPIO_InitStructure.GPIO_Pin = pDParams_str->hBKINPin;  
    GPIO_Init(pDParams_str->hBKINPort, &GPIO_InitStructure); 
    GPIO_PinLockConfig(pDParams_str->hBKINPort, pDParams_str->hBKINPin);
  }
  
  if(pDParams_str->TIMx == TIM1)
  {   
    /* TIM1 Counter Clock stopped when the core is halted */
    DBGMCU_APB2PeriphConfig(DBGMCU_TIM1_STOP, ENABLE);
  }
  else
  {
    /* TIM8 Counter Clock stopped when the core is halted */
    DBGMCU_APB2PeriphConfig(DBGMCU_TIM8_STOP, ENABLE);
  }
    
  /* ADC1 and ADC2 registers configuration ---------------------------------*/
  /* ADC1 and ADC2 registers reset */  
  IF4XX_ADC_Deinit(ADC1);
  IF4XX_ADC_Deinit(ADC2);

 /* Enable the ADC Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = (uint8_t) ADC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 
    ADC_PRE_EMPTION_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = ADC_SUB_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  if(pDParams_str->TIMx==TIM1)
  {
    /* Enable the TIM1 Update interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = (uint8_t) TIM1_UP_TIM10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIMx_UP_PRE_EMPTION_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = TIMx_UP_SUB_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);   
  }
  else
  {
    /* Enable the TIM1 Update interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = (uint8_t) TIM8_UP_TIM13_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIMx_UP_PRE_EMPTION_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = TIMx_UP_SUB_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);     
  }
  
  /* ADC1 and ADC2 registers configuration ---------------------------------*/
  /* Enable ADC1 and ADC2 */
  ADC_Cmd(ADC1, ENABLE);
  ADC_Cmd(ADC2, ENABLE);

  /* ADC Common Init only for first instance */
  if (pDParams_str->bInstanceNbr == 1u)
  {
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = pDParams_str ->wADC_Clock_Divider;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);
  }
  
  /* ADC Init */
  ADC_StructInit(&ADC_InitStructure);
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Left;
  ADC_InitStructure.ADC_NbrOfConversion = 1u;
  ADC_Init(ADC1, &ADC_InitStructure);
  ADC_Init(ADC2, &ADC_InitStructure);
  
  ADC_InjectedDiscModeCmd(ADC1, ENABLE);
  ADC_InjectedDiscModeCmd(ADC2, ENABLE);
 
/* It is used only to configure the sampling time to the corresponding channel*/
  ADC_InjectedChannelConfig(ADC1, pDParams_str->bIaChannel, 1u,
                                                pDParams_str->b_IaSamplingTime);
  ADC_InjectedChannelConfig(ADC2, pDParams_str->bIbChannel, 1u,
                                                pDParams_str->b_IbSamplingTime);    

  /* ADC1 Injected conversions end interrupt enabling */
  ADC_ClearFlag(ADC1, ADC_FLAG_JEOC); 	 
  ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE); 
  
  /* To pre-compute the following variables is used the configuration already 
     performed on ADC1 and ADC2. This means that ADC configurations run from here 
     on out will be overwritten during the context switching.*/
  if(pDParams_str->TIMx == TIM1)
  {
    /* The following two variables are pre-computed and used to disable/enable 
       the ADC injected external trigger during the context switching. */
    pDVars_str->wADCTriggerUnSet = ADC1->CR2 & 0xFFC0FFFFu; /* JEXTEN = 00b (Disable), JEXTSEL = 0000b (TIM1_CC4) */
    pDVars_str->wADCTriggerSet   = pDVars_str->wADCTriggerUnSet | 0x00100000u; /* JEXTEN = 01b (Enable), JEXTSEL = 0000b (TIM1_CC4) */
  }
  else
  {    
    /* The following two variables are pre-computed and used to disable/enable 
       the ADC injected external trigger during the context switching. */
    pDVars_str->wADCTriggerUnSet = ADC1->CR2 & 0xFFC0FFFFu; /* JEXTEN = 00b (Disable), JEXTSEL = 0000b (TIM1_CC4 "dummy") */
    pDVars_str->wADCTriggerSet   = pDVars_str->wADCTriggerUnSet | 0x001E0000u; /* JEXTEN = 01b (Enable), JEXTSEL = 1110b (TIM8_CC4) */
  }
}

/**
* @brief  It initializes TIMx peripheral for PWM generation
* @param 'TIMx': Timer to be initialized
* @param 'this': related object of class CIF4XX_PWMC
* @retval none
*/
static void IF4XX_TIMxInit(TIM_TypeDef* TIMx, CPWMC this)
{
  TIM_TimeBaseInitTypeDef TIMx_TimeBaseStructure;
  TIM_OCInitTypeDef TIMx_OCInitStructure;
  TIM_BDTRInitTypeDef TIMx_BDTRInitStructure;
  pDVars_t pDVars_str = &DCLASS_VARS;  
  pDParams_t pDParams_str =DCLASS_PARAMS; 
  
  /* TIMx Peripheral Configuration -------------------------------------------*/
  /* TIMx Registers reset */
  TIM_DeInit(TIMx);
  TIM_TimeBaseStructInit(&TIMx_TimeBaseStructure);
  /* Time Base configuration */
  TIMx_TimeBaseStructure.TIM_Prescaler = (uint16_t)(pDParams_str->bTim_Clock_Divider) - 1u;  
  TIMx_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
  TIMx_TimeBaseStructure.TIM_Period = pDVars_str->Half_PWMPeriod;
  TIMx_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV2;
  TIMx_TimeBaseStructure.TIM_RepetitionCounter = pDParams_str->
                                                            bRepetitionCounter;
  TIM_TimeBaseInit(TIMx, &TIMx_TimeBaseStructure);
  
  /* Channel 1, 2,3 and 4 Configuration in PWM mode */
  TIM_OCStructInit(&TIMx_OCInitStructure);  
  TIMx_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
  TIMx_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
  TIMx_OCInitStructure.TIM_Pulse = (uint32_t)(pDVars_str->Half_PWMPeriod)/2u; /* dummy value */
  
  /* Channel 1 */
  TIMx_OCInitStructure.TIM_OCPolarity = pDParams_str->hCh1Polarity;      
  TIMx_OCInitStructure.TIM_OCIdleState = pDParams_str->hCh1IdleState;    
  if ((pDParams_str->LowSideOutputs)== LS_PWM_TIMER) 
  {
    TIMx_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable; 
    TIMx_OCInitStructure.TIM_OCNPolarity = pDParams_str->hCh1NPolarity; 
    TIMx_OCInitStructure.TIM_OCNIdleState = pDParams_str->hCh1NIdleState;     
  }    
  else
  {
    TIMx_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  }    
  TIM_OC1Init(TIMx, &TIMx_OCInitStructure); 
  
  
  /* Channel 2 */
  TIMx_OCInitStructure.TIM_OCPolarity = pDParams_str->hCh2Polarity;      
  TIMx_OCInitStructure.TIM_OCIdleState = pDParams_str->hCh2IdleState;    
  if ((pDParams_str->LowSideOutputs)== LS_PWM_TIMER) 
  {
  TIMx_OCInitStructure.TIM_OCNPolarity = pDParams_str->hCh2NPolarity; 
  TIMx_OCInitStructure.TIM_OCNIdleState = pDParams_str->hCh2NIdleState;         
  }
  TIM_OC2Init(TIMx, &TIMx_OCInitStructure); 
  
  
  /* Channel 3 */
  TIMx_OCInitStructure.TIM_OCPolarity = pDParams_str->hCh3Polarity;      
  TIMx_OCInitStructure.TIM_OCIdleState = pDParams_str->hCh3IdleState;    
  if ((pDParams_str->LowSideOutputs)== LS_PWM_TIMER) 
  {
  TIMx_OCInitStructure.TIM_OCNPolarity = pDParams_str->hCh3NPolarity; 
  TIMx_OCInitStructure.TIM_OCNIdleState = pDParams_str->hCh3NIdleState;         
  }
  TIM_OC3Init(TIMx, &TIMx_OCInitStructure);   
  
    /* Channel 4 */
  TIMx_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  TIMx_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;      
  TIMx_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset; 
  TIMx_OCInitStructure.TIM_Pulse = (uint32_t)(DCLASS_VARS.Half_PWMPeriod)+1u;
  TIM_OC4Init(TIMx, &TIMx_OCInitStructure); 
  
  /* Enables the TIMx Preload on CC1 Register */
  TIM_OC1PreloadConfig(TIMx, TIM_OCPreload_Enable);
  /* Enables the TIMx Preload on CC2 Register */
  TIM_OC2PreloadConfig(TIMx, TIM_OCPreload_Enable);
  /* Enables the TIMx Preload on CC3 Register */
  TIM_OC3PreloadConfig(TIMx, TIM_OCPreload_Enable);
  /* Enables the TIMx Preload on CC4 Register */
  TIM_OC4PreloadConfig(TIMx, TIM_OCPreload_Enable); 
  
  TIM_BDTRStructInit(&TIMx_BDTRInitStructure);
  /* Dead Time */
  TIMx_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
  TIMx_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
  TIMx_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1; 
  TIMx_BDTRInitStructure.TIM_DeadTime = (pDParams_str->hDeadTime)/2u;
  /* BKIN, if enabled */
  if ((pDParams_str->EmergencyStop)!= DISABLE)  
  {
    TIMx_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
    TIMx_BDTRInitStructure.TIM_BreakPolarity = pDParams_str->hBKINPolarity;
    TIMx_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;
    TIM_ClearITPendingBit(TIMx, TIM_IT_Break);
    TIM_ITConfig(TIMx, TIM_IT_Break, ENABLE);
  }  
  TIM_BDTRConfig(TIMx, &TIMx_BDTRInitStructure);
 
  TIM_SelectInputTrigger(TIMx,TIM_TS_ITR1);
  TIM_SelectSlaveMode(TIMx,TIM_SlaveMode_Trigger);

  /* Prepare timer for synchronization */
  TIM_GenerateEvent(TIMx,TIM_EventSource_Update);
      
  if (pDParams_str->bFreqRatio == 2u) 
  {
    if (pDParams_str->bIsHigherFreqTim == HIGHER_FREQ)
    {
      if (pDParams_str->bRepetitionCounter == 3u)
      {
        /* Set TIMx repetition counter to 1 */
        TIMx->RCR =0x01u; 
        TIM_GenerateEvent(TIMx, TIM_EventSource_Update);
        /* Repetition counter will be set to 3 at next Update */
        TIMx->RCR =0x03u; 
      }
    }
    
    TIM_SetCounter(TIMx, (uint32_t)(pDVars_str->Half_PWMPeriod)-1u);     
  }
  else /* bFreqRatio equal to 1 or 3 */
  {
    if (pDParams_str->bInstanceNbr == 1u)
    {
      TIM_SetCounter(TIMx, (uint32_t)(pDVars_str->Half_PWMPeriod)-1u);
    }
  }
}

/**
* @brief  It perform the start of all the timers required by the control. 
          It utilizes TIM2 as temporary timer to achieve synchronization between 
          PWM signals.
          When this function is called, TIM1 and/or TIM8 must be in frozen state
          with CNT, ARR, REP RATE and trigger correctly set (these setting are 
          usually performed in the Init method accordingly with the configuration)
* @param  none
* @retval none
*/
 void IF4XX_StartTimers(void)
{
 TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;   
  
  /* Temporary Enable TIM2 clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  TIM_DeInit(TIM2);
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  /* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0u;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = 1u; /* dummy */ 
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  
  TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);  
     
  TIM_Cmd(TIM2, ENABLE);
  
  TIM_DeInit(TIM2);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, DISABLE); 
}


/**
* @brief  It stores into 'this' object variables the voltage present on Ia and 
*         Ib current feedback analog channels when no current is flowin into the
*         motor
* @param  this: related object of class CIF4XX_PWMC
* @retval none
*/
static void IF4XX_CurrentReadingCalibration(CPWMC this)
{
  pDVars_t pDVars_str = &DCLASS_VARS; 
  TIM_TypeDef*  LocalTIMx = DCLASS_PARAMS->TIMx;    
  uint16_t htempCCER, haux;
  
  pDVars_str-> wPhaseAOffset = 0u;
  pDVars_str-> wPhaseBOffset = 0u; 
  
  pDVars_str->bIndex=0u;
  
  /* Force inactive level on TIMx CHy and TIMx CHyN */ 
  htempCCER =  DCLASS_PARAMS->TIMx->CCER;
  haux = htempCCER & TIMxCCER_MASK;
  haux |= TIMx_CC4E_BIT;
  LocalTIMx->CCER = haux;
  
  /* Change function to be executed in ADCx_ISR */ 
  ((_CPWMC) this)->Methods_str.pPWMC_GetPhaseCurrents = &IF4XX_HFCurrentsCalibration;
  
  IF4XX_SwitchOnPWM(this);
  
  /* Wait for NB_CONVERSIONS to be executed */
  while (pDVars_str->bIndex < (NB_CONVERSIONS))
  {
    if (LocalTIMx->DIER & TIM_IT_Update)
    {}
    else
    {
      pDVars_str->bIndex = NB_CONVERSIONS;
    }
  }  
  
  IF4XX_SwitchOffPWM( this);
  
  pDVars_str->wPhaseAOffset >>=3; 
  pDVars_str->wPhaseBOffset >>=3; 
 
  /* It over write TIMx CCRy wrongly written by FOC during calibration so as to 
   force 50% duty cycle on the three inverer legs */
  /* Disable TIMx preload */ 
  LocalTIMx->CCMR1 &= 0xF7F7u;
  LocalTIMx->CCMR2 &= 0xF7F7u;
  LocalTIMx->CCR1 = pDVars_str->Half_PWMPeriod;
  LocalTIMx->CCR2 = pDVars_str->Half_PWMPeriod;
  LocalTIMx->CCR3 = pDVars_str->Half_PWMPeriod;
  
  /* Enable TIMx preload */
  LocalTIMx->CCMR1 |= 0x0808u;
  LocalTIMx->CCMR2 |= 0x0808u;
  
  /* Set back TIMx CCER register */ 
  LocalTIMx->CCER = htempCCER;

  /* Change back function to be executed in ADCx_ISR */ 
  ((_CPWMC) this)->Methods_str.pPWMC_GetPhaseCurrents = &IF4XX_GetPhaseCurrents;
}

/**
* @brief  It computes and return latest converted motor phase currents motor
* @param  this: related object of class CIF4XX_PWMC
* @retval Ia and Ib current in Curr_Components format
*/
static void IF4XX_GetPhaseCurrents(CPWMC this,Curr_Components* pStator_Currents)
{
  int32_t wAux;
  uint16_t hReg;
  /* Derived class members container */
  pDVars_t pDVars_str = &DCLASS_VARS;  

  /* Reset the SOFOC flag to indicate the start of FOC algorithm*/
  pDVars_str->hFlags &= (~SOFOC); 
  
 /* Ia = (hPhaseAOffset)-(PHASE_A_ADC_CHANNEL vale)  */
  hReg = (uint16_t)((ADC1->JDR1)<<1);
  wAux = (int32_t)(hReg)-(int32_t)(pDVars_str->wPhaseAOffset);
  
 /* Saturation of Ia */
  if (wAux < S16_MIN)
  {
    pStator_Currents->qI_Component1= S16_MIN;
  }  
  else  if (wAux > S16_MAX)
        { 
          pStator_Currents->qI_Component1= S16_MAX;
        }
        else
        {
          pStator_Currents->qI_Component1= (int16_t)wAux;
        }
                     
 /* Ib = (hPhaseBOffset)-(PHASE_B_ADC_CHANNEL value) */
  hReg = (uint16_t)((ADC2->JDR1)<<1);
  wAux = (int32_t)(hReg)-(int32_t)(pDVars_str->wPhaseBOffset);
  
 /* Saturation of Ib */
  if (wAux < S16_MIN)
  {
    pStator_Currents->qI_Component2= S16_MIN;
  }  
  else  if (wAux > S16_MAX)
        { 
          pStator_Currents->qI_Component2= S16_MAX;
        }
        else
        {
          pStator_Currents->qI_Component2= (int16_t)wAux;
        }
}


/**
* @brief  It sum up injected conversion data into wPhaseXOffset. It is called 
*         only during current calibration 
* @param  this: related object of class CIF4XX_PWMC
* @retval It always returns {0,0} in Curr_Components format
*/
static void IF4XX_HFCurrentsCalibration(CPWMC this,Curr_Components* pStator_Currents)
{ 
  /* Derived class members container */
  pDVars_t pDVars_str = &DCLASS_VARS; 

 /* Reset the SOFOC flag to indicate the start of FOC algorithm*/
  pDVars_str->hFlags &= (~SOFOC); 
    
  if (pDVars_str->bIndex < NB_CONVERSIONS)
  {
    pDVars_str-> wPhaseAOffset += ADC1->JDR1;
    pDVars_str-> wPhaseBOffset += ADC2->JDR1; 
    pDVars_str->bIndex++;
  }
}

/**
  * @brief  It turns on low sides switches. This function is intended to be 
  *         used for charging boot capacitors of driving section. It has to be 
  *         called each motor start-up when using high voltage drivers
  * @param  this: related object of class CIF4XX_PWMC
  * @retval none
  */
static void IF4XX_TurnOnLowSides(CPWMC this)
{
  pDParams_t pLocalDParams = DCLASS_PARAMS;
  TIM_TypeDef*  LocalTIMx = DCLASS_PARAMS->TIMx;  
  pDVars_t pDVars_str = &DCLASS_VARS;  

  /*Turn on the three low side switches */
  LocalTIMx->CCR1 = 0u;
  LocalTIMx->CCR2 = 0u;
  LocalTIMx->CCR3 = 0u;
  /*Disable ADC trigger */
  LocalTIMx->CCMR2 =0x7068u;
  LocalTIMx->CCR4 = (uint32_t)(pDVars_str->Half_PWMPeriod)+1u;
  
  pDVars_str->hFlags &= (~SOFOC);
  
  TIM_ClearFlag(LocalTIMx,TIM_FLAG_Update);
  while (TIM_GetFlagStatus(LocalTIMx,TIM_FLAG_Update) == RESET)
  {}
  
  /* Main PWM Output Enable */
  TIM_CtrlPWMOutputs(LocalTIMx, ENABLE);
  if ((pLocalDParams->LowSideOutputs)== ES_GPIO)
  {
    GPIO_WriteBit(pLocalDParams->hCh1NPort, pLocalDParams->hCh1NPin, Bit_SET);
    GPIO_WriteBit(pLocalDParams->hCh2NPort, pLocalDParams->hCh2NPin, Bit_SET);
    GPIO_WriteBit(pLocalDParams->hCh3NPort, pLocalDParams->hCh3NPin, Bit_SET);
  }
  return; 
}


/**
* @brief  It enables PWM generation on the proper Timer peripheral acting on MOE
*         bit
* @param  this: related object of class CIF4XX_PWMC
* @retval none
*/
static void IF4XX_SwitchOnPWM(CPWMC this)
{  
  pDParams_t pLocalDParams = DCLASS_PARAMS;
  TIM_TypeDef*  LocalTIMx = DCLASS_PARAMS->TIMx;  
  pDVars_t pDVars_str = &DCLASS_VARS;  
 
  /* It clears ADCs JSTRT and JEOC bits */
  ADC1->SR &= ~ADC_SR_MASK;
  ADC2->SR &= ~ADC_SR_MASK;
  
  /* Clear Update Flag */
  TIM_ClearFlag(LocalTIMx, TIM_FLAG_Update);
  
  /* Enable TIMx preload and ADC trigger on next update */
  LocalTIMx->CCMR2 = 0x7868u;
  LocalTIMx->CCR4 = (uint32_t)(pDVars_str->Half_PWMPeriod)-5u;
  
  TIM_ITConfig(LocalTIMx, TIM_IT_Update, ENABLE);
  
  /* Main PWM Output Disable */
  TIM_CtrlPWMOutputs(LocalTIMx, ENABLE);
  if ((pLocalDParams->LowSideOutputs)== ES_GPIO)
  {
    GPIO_WriteBit(pLocalDParams->hCh1NPort, pLocalDParams->hCh1NPin, Bit_SET);
    GPIO_WriteBit(pLocalDParams->hCh2NPort, pLocalDParams->hCh2NPin, Bit_SET);
    GPIO_WriteBit(pLocalDParams->hCh3NPort, pLocalDParams->hCh3NPin, Bit_SET);
  }
  pDVars_str->hFlags &= (~SOFOC);
  return; 
}


/**
* @brief  It disables PWM generation on the proper Timer peripheral acting on 
*         MOE bit
* @param  this: related object of class CIF4XX_PWMC
* @retval none
*/
static void IF4XX_SwitchOffPWM(CPWMC this)
{ 
  pDParams_t pLocalDParams = DCLASS_PARAMS;
  TIM_TypeDef*  LocalTIMx = DCLASS_PARAMS->TIMx;
  pDVars_t pDVars_str = &DCLASS_VARS;  
  
    /* Disable UPDATE ISR */
  TIM_ITConfig(LocalTIMx, TIM_IT_Update, DISABLE);  
  
  LocalTIMx->CCER &= (uint16_t)(~TIMxCCER_MASK_CH123);
  
  while (TIM_GetFlagStatus(LocalTIMx,TIM_FLAG_Update)==RESET)
  {
    if (LocalTIMx->DIER & TIM_IT_Update)
    { break;}
  }
   
    /* Main PWM Output Disable */
  TIM_CtrlPWMOutputs(LocalTIMx, DISABLE);
  if ((pLocalDParams->LowSideOutputs)== ES_GPIO)
  {
    GPIO_WriteBit(pLocalDParams->hCh1NPort, pLocalDParams->hCh1NPin, Bit_RESET);
    GPIO_WriteBit(pLocalDParams->hCh2NPort, pLocalDParams->hCh2NPin, Bit_RESET);
    GPIO_WriteBit(pLocalDParams->hCh3NPort, pLocalDParams->hCh3NPin, Bit_RESET);
  }
  LocalTIMx->CCER |= TIMxCCER_MASK_CH123; 
  
  LocalTIMx->CCMR2 =0x7068u;
  LocalTIMx->CCR4 = (uint32_t)(pDVars_str->Half_PWMPeriod)+1u;  
  
  return; 
}

/**
* @brief  It stores into 'this' object variables the voltage present on Ia and 
*         Ib current feedback analog channels when no current is flowin into the
*         motor
* @param  this: related object of class CIF4XX_PWMC
* @retval none
*/
static uint16_t IF4XX_WriteTIMRegisters(CPWMC this)
{
  uint16_t hAux;
  TIM_TypeDef*  LocalTIMx = DCLASS_PARAMS->TIMx;
  /* Derived class members container */
  pDVars_t pDVars_str = &DCLASS_VARS;  
    
  LocalTIMx->CCR1 = ((_CPWMC) this)->Vars_str.hCntPhA;
  LocalTIMx->CCR2 = ((_CPWMC) this)->Vars_str.hCntPhB;
  LocalTIMx->CCR3 =((_CPWMC) this)->Vars_str.hCntPhC;
  
  /* Disable TIMx preload */  
  LocalTIMx->CCMR2 =0x7068u;
  LocalTIMx->CCR4 = (uint32_t)(pDVars_str->Half_PWMPeriod)+1u;
  /* Enable TIMx preload */
  LocalTIMx->CCMR2 = 0x7868u;
  LocalTIMx->CCR4 = (uint32_t)(pDVars_str->Half_PWMPeriod)-5u;
  
   /* Limit for update event */
  /* Check the status of SOFOC flag. If it is set, an update event has occurred 
  and thus the FOC rate is too high */
  if ((pDVars_str->hFlags & SOFOC) != 0u)
  {
    hAux = MC_FOC_DURATION;
  }
  else
  {
    hAux = MC_NO_ERROR;
  }
  return hAux;
}



/**
* @brief  It contains the TIMx Update event interrupt
* @param  this: related object of class CIF4XX_PWMC
* @retval none
*/
static void *IF4XX_IRQHandler(void *this, unsigned char flag)
{
  uint32_t wADCInjFlags;
  pVars_t pVars_str = &CLASS_VARS;
  pDParams_t pDParams_str = DCLASS_PARAMS;
  TIM_TypeDef*  LocalTIMx = DCLASS_PARAMS->TIMx; 
  pDVars_t pDVars_str = &DCLASS_VARS;  
  
 wADCInjFlags = (ADC1-> SR) & ADC_SR_MASK;
 
if (wADCInjFlags == CONV_STARTED)
 {
   do
   {
      wADCInjFlags = (ADC1-> SR) & ADC_SR_MASK;
   }
   while (wADCInjFlags != CONV_FINISHED);
 }
else if (wADCInjFlags == FLAGS_CLEARED)
{
  while ((LocalTIMx->CNT) < (pDParams_str->Tw))
  {}
  wADCInjFlags = (ADC1-> SR) & ADC_SR_MASK;
  
  if (wADCInjFlags == CONV_STARTED)
 {
   do
   {
      wADCInjFlags = (ADC1-> SR) & ADC_SR_MASK;
   }
   while (wADCInjFlags != CONV_FINISHED);
 }
}
else {}

/* Set the SOFOC flag to indicate the execution of Update IRQ*/
pDVars_str->hFlags |= SOFOC;

/* Switch Context */
/* It re-initilize AD converter in run time when using dual MC */
  ADC1->CR2 = pDVars_str->wADCTriggerSet;
  ADC2->CR2 = pDVars_str->wADCTriggerSet;
/* Change channels keeping equal to 1 element the sequencer lenght */ 
ADC1->JSQR = (uint32_t)(pDParams_str->bIaChannel)<<15;
ADC2->JSQR = (uint32_t)(pDParams_str->bIbChannel)<<15;  

return &(pVars_str->bMotor);
}

/**
* @brief  Execute a regular conversion using ADC1. 
*         The function is not re-entrant (can't executed twice at the same time)
* @param  this related object of class CIF4XX_PWMC
* @retval It returns converted value or oxFFFF for conversion error
*/
static uint16_t IF4XX_ExecRegularConv(CPWMC this, uint8_t bChannel)
{
  ADC1->SQR3 = bChannel;
    
  /* Clear EOC flag of ADC1 */
  ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
  
  /* It starts software triggered regular conversion
  through bit banding access. It is equivalent to 
  ADC_SoftwareStartConv(ADC1);*/
  *(uint32_t *)(ADC1_CR2_SWSTART_BB)=(uint32_t)(0x1u);
  
  /* Wait end of conversion */
  while ((ADC1->SR & ADC_FLAG_EOC) == 0u)
  {
  }
  
  return (ADC_GetConversionValue(ADC1));
}

/**
* @brief  It sets the specified sampling time for the specified ADC channel
*         on ADC1. It must be called once for each channel utilized by user
* @param  ADC channel, sampling time
* @retval none
*/
static void IF4XX_ADC_SetSamplingTime(CPWMC this, ADConv_t ADConv_struct)
{ 
   uint32_t tmpreg1 = 0u, tmpreg2 = 0u, tmpreg3=0u, tmpreg4 = SMPR1_SMP_Set;
   
  /* if ADC_Channel_10 ... ADC_Channel_17 is selected */
  if (ADConv_struct.Channel> ADC_Channel_9)
  {
    /* Get the old register value */
    tmpreg1 = ADC1->SMPR1;
    /* Calculate the mask to clear */
    tmpreg3 = (uint32_t) (ADConv_struct.Channel) - 10u;
    tmpreg3 = tmpreg3 *3u;
    tmpreg2 =  tmpreg4 << (tmpreg3);
    /* Clear the old discontinuous mode channel count */
    tmpreg1 &= ~tmpreg2;
    /* Calculate the mask to set */
    tmpreg2 = (uint32_t)(ADConv_struct.SamplTime) << (tmpreg3);
    /* Set the discontinuous mode channel count */
    tmpreg1 |= tmpreg2;
    /* Store the new register value */
    ADC1->SMPR1 = tmpreg1;
  }
  else /* ADC_Channel include in ADC_Channel_[0..9] */
  {
    /* Get the old register value */
    tmpreg1 = ADC1->SMPR2;
    tmpreg3 = (uint32_t) (ADConv_struct.Channel) * 3u;
    /* Calculate the mask to clear */
    tmpreg4 = SMPR2_SMP_Set;
    tmpreg2 =  tmpreg4 << (tmpreg3);
    /* Clear the old discontinuous mode channel count */
    tmpreg1 &= ~tmpreg2;
    /* Calculate the mask to set */
    tmpreg2 = (uint32_t)(ADConv_struct.SamplTime) << (tmpreg3);
    /* Set the discontinuous mode channel count */
    tmpreg1 |= tmpreg2;
    /* Store the new register value */
    ADC1->SMPR2 = tmpreg1;
  }
}

/**
* @brief  It is used to check if an overcurrent occurred since last call.
* @param  this related object of class CPWMC
* @retval uint16_t It returns MC_BREAK_IN whether an overcurrent has been 
*                  detected since last method call, MC_NO_FAULTS otherwise.
*/
static uint16_t IF4XX_IsOverCurrentOccurred(CPWMC this)
{
  TIM_TypeDef*  LocalTIMx = DCLASS_PARAMS->TIMx;
  uint16_t retVal = MC_NO_FAULTS;
  if ((LocalTIMx->SR & TIM_FLAG_Break) != 0u)
  {
    retVal = MC_BREAK_IN;
    LocalTIMx->SR = (u16)~TIM_FLAG_Break;
  }
  return retVal;
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

static void IF4XX_ADC_Deinit(ADC_TypeDef * Local_ADC)
{
  Local_ADC->SR = ADC_GENERAL_RESET_VALUE;
  Local_ADC->CR1 = ADC_GENERAL_RESET_VALUE;
  Local_ADC->CR2 = ADC_GENERAL_RESET_VALUE;
  Local_ADC->HTR = ADC_HTR_RESET_VALUE;
  Local_ADC->LTR = ADC_GENERAL_RESET_VALUE;
  Local_ADC->SQR1 = ADC_GENERAL_RESET_VALUE;
  Local_ADC->SQR2 = ADC_GENERAL_RESET_VALUE;
  Local_ADC->SQR3 = ADC_GENERAL_RESET_VALUE;
  Local_ADC->JSQR = ADC_GENERAL_RESET_VALUE;
  ADC->CSR = ADC_GENERAL_RESET_VALUE;
  ADC->CCR = ADC_GENERAL_RESET_VALUE;  
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
