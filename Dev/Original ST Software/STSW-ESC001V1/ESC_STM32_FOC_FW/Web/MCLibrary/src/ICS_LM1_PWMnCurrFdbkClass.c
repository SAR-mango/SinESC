/**
  ******************************************************************************
  * @file    ICS_LM1_PWMnCurrFdbkClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private impelementation of derived class for 
  *          single motor ICS current reading with STM32F103x Low and Medium 
  *          Density
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

#if (defined(STM32F10X_MD)||defined(STM32F10X_LD)||(STM32F10X_HD))

/* Includes ------------------------------------------------------------------*/
#include "PWMnCurrFdbkClass.h"
#include "PWMnCurrFdbkPrivate.h"
#include "ICS_LM1_PWMnCurrFdbkClass.h"
#include "ICS_LM1_PWMnCurrFdbkPrivate.h"
#include "MCLibraryConf.h"
#include "MCLibraryISRPriorityConf.h"
#include "MC_type.h"

/* ADC1 Data Register address */
#define ADC1_DR_Address     0x4001244Cu

#define NB_CONVERSIONS 16u

#define ADC_RIGHT_ALIGNMENT 3u

#define DCLASS_PARAMS ((_DCILM1_PWMC)(((_CPWMC) this)->DerivedClass))->pDParams_str
#define DCLASS_VARS  ((_DCILM1_PWMC)(((_CPWMC) this)->DerivedClass))->DVars_str

/* ADC SMPx mask */
#define SMPR1_SMP_Set              ((uint32_t) (0x00000007u))
#define SMPR2_SMP_Set              ((uint32_t) (0x00000007u))
#define ADC1_CR2_EXTTRIG_SWSTART_BB             0x42248158u

#ifdef MC_CLASS_DYNAMIC
#include "stdlib.h" /* Used for dynamic allocation */
#else
_DCILM1_PWMC_t ILM1_PWMCpool[MAX_DRV_PWMC_NUM];
unsigned char ILM1_PWMC_Allocated = 0u;
#endif

/*#define DEBUG*/

#ifdef DEBUG
static volatile _CPWMC oPWMCdbg;
static volatile _DCILM1_PWMC oILM1dbg;
#endif

static void ILM1_Init(CPWMC this);
static void ILM1_TIM1Init(CPWMC this);
static void ILM1_CurrentReadingCalibration(CPWMC this);
static void ILM1_GetPhaseCurrents(CPWMC this,Curr_Components* pStator_Currents);
static void ILM1_TurnOnLowSides(CPWMC this);
static void ILM1_SwitchOnPWM(CPWMC this);
static void ILM1_SwitchOffPWM(CPWMC this);
static uint16_t ILM1_WriteTIMRegisters(CPWMC this);   
static void ILM1_ADC_SetSamplingTime(CPWMC this, ADConv_t ADConv_struct);
static uint16_t ILM1_ExecRegularConv(CPWMC this, uint8_t bChannel);
static uint16_t ILM1_IsOverCurrentOccurred(CPWMC this);

/**
* @brief  Creates an object of the class ICS_LM1
* @param  pPWMnCurrFdbkParams pointer to an PWMnCurrFdbk parameters structure
* @param  pICS_LM1Params pointer to an ICS_LM1 parameters structure
* @retval CILM1_PWMC new instance of ICS_LM1 object
*/
CILM1_PWMC ILM1_NewObject(pPWMnCurrFdbkParams_t pPWMnCurrFdbkParams, 
                                    pICS_LM1Params_t pICS_LM1Params)
{
  _CPWMC _oPWMnCurrFdbk;
  _DCILM1_PWMC _oICS_LM1;
  
  _oPWMnCurrFdbk = (_CPWMC)PWMC_NewObject(pPWMnCurrFdbkParams);
  
#ifdef MC_CLASS_DYNAMIC
  _oICS_LM1 = (_DCILM1_PWMC)calloc(1u,sizeof(_DCILM1_PWMC_t));
#else
  if (ILM1_PWMC_Allocated  < MAX_DRV_PWMC_NUM)
  {
    _oICS_LM1 = &ILM1_PWMCpool[ILM1_PWMC_Allocated++];
  }
  else
  {
    _oICS_LM1 = MC_NULL;
  }
#endif
  
  _oICS_LM1->pDParams_str = pICS_LM1Params;
  _oPWMnCurrFdbk->DerivedClass = (void*)_oICS_LM1;
  
  _oPWMnCurrFdbk->Methods_str.pPWMC_Init = &ILM1_Init;
  _oPWMnCurrFdbk->Methods_str.pPWMC_GetPhaseCurrents = &ILM1_GetPhaseCurrents;
  _oPWMnCurrFdbk->Methods_str.pPWMC_SwitchOffPWM = &ILM1_SwitchOffPWM;
  _oPWMnCurrFdbk->Methods_str.pPWMC_SwitchOnPWM = &ILM1_SwitchOnPWM;        
  _oPWMnCurrFdbk->Methods_str.pPWMC_CurrentReadingCalibr = 
                                                 &ILM1_CurrentReadingCalibration;         
  _oPWMnCurrFdbk->Methods_str.pPWMC_TurnOnLowSides = &ILM1_TurnOnLowSides;         
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect1 = 
                                                      &ILM1_WriteTIMRegisters;        
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect2 = 
                                                      &ILM1_WriteTIMRegisters; 
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect3 = 
                                                      &ILM1_WriteTIMRegisters;        
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect4 = 
                                                      &ILM1_WriteTIMRegisters;         
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect5 = 
                                                      &ILM1_WriteTIMRegisters;        
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect6 = 
                                                      &ILM1_WriteTIMRegisters; 
  _oPWMnCurrFdbk->Methods_str.pPWMC_ExecRegularConv= &ILM1_ExecRegularConv;
  
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetSamplingTime= &ILM1_ADC_SetSamplingTime;
  
  _oPWMnCurrFdbk->Methods_str.pPWMC_IsOverCurrentOccurred = 
    &ILM1_IsOverCurrentOccurred;

  return ((CILM1_PWMC)_oPWMnCurrFdbk);
}


/** @addtogroup STM32F10x_PMSM_MC_Library
* @{
*/

/** @addtogroup PWMnCurrFdbk_ICS_LM1
* @{
*/

/** @defgroup ICS_LM1_class_private_methods ICS_LM1 class private methods
* @{
*/

/**
* @brief  It initializes TIM1, ADC, GPIO, DMA1 and NVIC for current reading 
*         in ICS configuration using STM32F103x High Density
* @param  this: related object of class CILM1_PWMC
* @retval none
*/
static void ILM1_Init(CPWMC this)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  ADC_InitTypeDef ADC_InitStructure;
  uint16_t hAux;
  DMA_InitTypeDef DMA_InitStructure;
  pDParams_t pLocalDParams = DCLASS_PARAMS;  
  DVars_t *pLocalVars_Str = &DCLASS_VARS;
  
  pLocalVars_Str->Half_PWMPeriod = ((((_CPWMC) this)->pParams_str->hPWMperiod)/2u)-1u;

  /* Peripheral clocks enabling ---------------------------------------------*/
  
  RCC->AHBENR |= RCC_AHBPeriph_CRC;
  
  /* ADCCLK = PCLK2 */
  RCC_ADCCLKConfig(pLocalDParams ->wADC_Clock_Divider);
  /* Enable ADC1 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  /* Enable ADC2 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE); 
  /* Enable GPIOA-GPIOF clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | 
                         RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | 
                           RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | 
                             RCC_APB2Periph_GPIOF, ENABLE);  
  
  /* Enable the CCS */
  RCC_ClockSecuritySystemCmd((FunctionalState)(ENABLE));
  
  /* Enable TIM1 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  /* Enable DMA1 clock */  
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	ILM1_TIM1Init(this);
  
  /* GPIOs configurations --------------------------------------------------*/
  GPIO_StructInit(&GPIO_InitStructure);
  
  /****** Configure phase A ADC channel GPIO as analog input ****/
  GPIO_InitStructure.GPIO_Pin = pLocalDParams->hIaPin;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(pLocalDParams->hIaPort,
            &GPIO_InitStructure);
  GPIO_PinLockConfig(pLocalDParams->hIaPort, pLocalDParams->hIaPin);
  
  /****** Configure phase B ADC channel GPIO as analog input ****/
  GPIO_InitStructure.GPIO_Pin = pLocalDParams->hIbPin;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(pLocalDParams->hIbPort, &GPIO_InitStructure);
  GPIO_PinLockConfig(pLocalDParams->hIbPort, pLocalDParams->hIbPin);
  
    
    /****** Timer1 alternate function full remapping ******/  
    if((pLocalDParams-> wTIM1Remapping) != GPIO_NoRemap_TIM1)
    {   
      GPIO_PinRemapConfig(pLocalDParams->wTIM1Remapping, ENABLE);  
    }  
  
  /****** Configure TIM1 Channel 1, 2 and 3 Outputs ******/ 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  hAux = (pLocalDParams->hCh1Pin) | (pLocalDParams->hCh2Pin);  
  hAux = hAux | (pLocalDParams->hCh3Pin);  
  GPIO_InitStructure.GPIO_Pin = hAux; 
  GPIO_Init(pLocalDParams->hCh1Port, &GPIO_InitStructure);
  GPIO_PinLockConfig(pLocalDParams->hCh1Port, hAux);
  
  /****** Configure TIM1 Channel 1N, 2N and 3N Outputs, if enabled ******/    
  if ((pLocalDParams->LowSideOutputs)== LS_PWM_TIMER) 
  { 
    GPIO_InitStructure.GPIO_Pin = pLocalDParams->hCh1NPin;  
    GPIO_Init(pLocalDParams->hCh1NPort, &GPIO_InitStructure);  
    GPIO_PinLockConfig(pLocalDParams->hCh1NPort, pLocalDParams->hCh1NPin);
    
    hAux = (pLocalDParams->hCh2NPin) | (pLocalDParams->hCh3NPin);
    GPIO_InitStructure.GPIO_Pin = hAux; 
    GPIO_Init(pLocalDParams->hCh2NPort, &GPIO_InitStructure);    
    GPIO_PinLockConfig(pLocalDParams->hCh2NPort, hAux);
  }  else if ((pLocalDParams->LowSideOutputs)== ES_GPIO)
  {
    /* Only "active high" polarity is supported */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = pLocalDParams->hCh1NPin;  
    GPIO_Init(pLocalDParams->hCh1NPort, &GPIO_InitStructure);  
    GPIO_PinLockConfig(pLocalDParams->hCh1NPort, pLocalDParams->hCh1NPin);
    
    hAux = (pLocalDParams->hCh2NPin) | (pLocalDParams->hCh3NPin);
    GPIO_InitStructure.GPIO_Pin = hAux; 
    GPIO_Init(pLocalDParams->hCh2NPort, &GPIO_InitStructure);    
    GPIO_PinLockConfig(pLocalDParams->hCh2NPort, hAux);
  }
  else
  {
  }
  
  if ((pLocalDParams->EmergencyStop)!= DISABLE)  
  {
    /****** Configure TIMx BKIN input, if enabled ******/   
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = pLocalDParams->hBKINPin;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(pLocalDParams->hBKINPort, &GPIO_InitStructure); 
    GPIO_PinLockConfig(pLocalDParams->hBKINPort, pLocalDParams->hBKINPin);
  }
  
  /* TIM1 Counter Clock stopped when the core is halted */
  DBGMCU_Config(DBGMCU_TIM1_STOP, ENABLE);
  
  /* ADC1 and ADC2 registers configuration ---------------------------------*/
  /* ADC1 and ADC2 registers reset */  
  ADC_DeInit(ADC1);
  ADC_DeInit(ADC2);
  
  /* Enable ADC1 and ADC2 */
  ADC_Cmd(ADC1, ENABLE);
  ADC_Cmd(ADC2, ENABLE);
  
  /* ADC Init */
  ADC_StructInit(&ADC_InitStructure);
  ADC_InitStructure.ADC_Mode = ADC_Mode_InjecSimult;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Left;
  ADC_InitStructure.ADC_NbrOfChannel = 1u;
  ADC_Init(ADC1, &ADC_InitStructure);
  ADC_Init(ADC2, &ADC_InitStructure);
  
  /* Enable external trigger (it will be SW) for ADC1 and ADC2 regular 
  conversions */ 
  ADC_ExternalTrigConvCmd(ADC1, ENABLE);
  ADC_ExternalTrigConvCmd(ADC2, ENABLE);
  
  /* ADC1 Injected conversions configuration */     
  ADC_InjectedSequencerLengthConfig(ADC1,1u); 
  ADC_InjectedSequencerLengthConfig(ADC2,1u);
  
  /* Start calibration of ADC1 and ADC2 */
  ADC_StartCalibration(ADC1);
  ADC_StartCalibration(ADC2);
  
  /* Wait for the end of ADCs calibration */
  while (ADC_GetCalibrationStatus(ADC1) & ADC_GetCalibrationStatus(ADC2))
  {
  }
  
  /*Enable external trigger fo injected conv of ADC2 */
  ADC_ExternalTrigInjectedConvCmd(ADC2,ENABLE);  

 /* DMA Event related to ADC regular conversion*/
  /* DMA1 channel1 configuration */
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(&(pLocalVars_Str->hRegConv));
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 1u;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  /* Enable DMA1 Channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);
  /* Enable ADC1 EOC DMA */
  ADC_DMACmd(ADC1,ENABLE);  
  
  /* Enable the ADC Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = (uint8_t) ADC1_2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 
    ADC_PRE_EMPTION_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = ADC_SUB_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/**
* @brief  It initializes TIM1 peripheral for PWM generation
* @param 'TIMx': Timer to be initialized
* @param 'this': related object of class CILM1_PWMC
* @retval none
*/
static void ILM1_TIM1Init(CPWMC this)
{
  TIM_TimeBaseInitTypeDef TIMx_TimeBaseStructure;
  TIM_OCInitTypeDef TIMx_OCInitStructure;
  TIM_BDTRInitTypeDef TIMx_BDTRInitStructure;
  pDParams_t pLocalDParams = DCLASS_PARAMS;
  DVars_t *pLocalVars_Str = &DCLASS_VARS;
  
  pLocalVars_Str->Half_PWMPeriod = ((((_CPWMC) this)->pParams_str->hPWMperiod)/2u)-1u;
  
  /* TIM1 Peripheral Configuration -------------------------------------------*/
  /* TIM1 Registers reset */
  TIM_DeInit(TIM1);
  TIM_TimeBaseStructInit(&TIMx_TimeBaseStructure);
  /* Time Base configuration */
  TIMx_TimeBaseStructure.TIM_Prescaler = (uint16_t)(pLocalDParams->bTim_Clock_Divider) - 1u;
  TIMx_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
  TIMx_TimeBaseStructure.TIM_Period =  pLocalVars_Str->Half_PWMPeriod;
  TIMx_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV2;
  TIMx_TimeBaseStructure.TIM_RepetitionCounter = pLocalDParams->
                                                             bRepetitionCounter;
  TIM_TimeBaseInit(TIM1, &TIMx_TimeBaseStructure);
  
  /* Channel 1, 2,3 and 4 Configuration in PWM mode */
  TIM_OCStructInit(&TIMx_OCInitStructure);  
  TIMx_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
  TIMx_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
  TIMx_OCInitStructure.TIM_Pulse =  (pLocalVars_Str->Half_PWMPeriod)/2u; /* dummy value */
  
  /* Channel 1 */
  TIMx_OCInitStructure.TIM_OCPolarity = pLocalDParams->hCh1Polarity;      
  TIMx_OCInitStructure.TIM_OCIdleState = pLocalDParams->hCh1IdleState;    
  if ((pLocalDParams->LowSideOutputs)== LS_PWM_TIMER)
  {
    TIMx_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable; 
    TIMx_OCInitStructure.TIM_OCNPolarity = pLocalDParams->hCh1NPolarity; 
    TIMx_OCInitStructure.TIM_OCNIdleState = pLocalDParams->hCh1NIdleState;     
  }    
  else
  {
    TIMx_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  }    
  TIM_OC1Init(TIM1, &TIMx_OCInitStructure); 
  
  
  /* Channel 2 */
  TIMx_OCInitStructure.TIM_OCPolarity = pLocalDParams->hCh2Polarity;      
  TIMx_OCInitStructure.TIM_OCIdleState = pLocalDParams->hCh2IdleState;    
  if ((pLocalDParams->LowSideOutputs)== LS_PWM_TIMER)
  {
  TIMx_OCInitStructure.TIM_OCNPolarity = pLocalDParams->hCh2NPolarity; 
  TIMx_OCInitStructure.TIM_OCNIdleState = pLocalDParams->hCh2NIdleState;         
  }
  TIM_OC2Init(TIM1, &TIMx_OCInitStructure); 
  
  
  /* Channel 3 */
  TIMx_OCInitStructure.TIM_OCPolarity = pLocalDParams->hCh3Polarity;      
  TIMx_OCInitStructure.TIM_OCIdleState = pLocalDParams->hCh3IdleState;    
  if ((pLocalDParams->LowSideOutputs)== LS_PWM_TIMER)
  {
  TIMx_OCInitStructure.TIM_OCNPolarity = pLocalDParams->hCh3NPolarity; 
  TIMx_OCInitStructure.TIM_OCNIdleState = pLocalDParams->hCh3NIdleState;         
  }
  TIM_OC3Init(TIM1, &TIMx_OCInitStructure);   
 
  
  /* Enables the TIM1 Preload on CC1 Register */
  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
  /* Enables the TIM1 Preload on CC2 Register */
  TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
  /* Enables the TIM1 Preload on CC3 Register */
  TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable); 
  
  /* Dead Time */
  TIM_BDTRStructInit(&TIMx_BDTRInitStructure);
  TIMx_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
  TIMx_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
  TIMx_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1; 
  TIMx_BDTRInitStructure.TIM_DeadTime = (pLocalDParams->hDeadTime)/2u;
  /* BKIN, if enabled */
  if ((pLocalDParams->EmergencyStop)!= DISABLE)  
  {
    TIMx_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
    TIMx_BDTRInitStructure.TIM_BreakPolarity = pLocalDParams->hBKINPolarity;
    TIMx_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;
    TIM_ClearITPendingBit(TIM1, TIM_IT_Break);
    TIM_ITConfig(TIM1, TIM_IT_Break, ENABLE);
  }  
  TIM_BDTRConfig(TIM1, &TIMx_BDTRInitStructure);
  
  /* Trigger Output signal is Update Event */
  TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);
  
  /* TIM1 counter enable */
  TIM_Cmd(TIM1, ENABLE); 
}

/**
* @brief  It stores into 'this' object variables the voltage present on Ia and 
*         Ib current feedback analog channels when no current is flowin into the
*         motor
* @param  this: related object of class CILM1_PWMC
* @retval none
*/
static void ILM1_CurrentReadingCalibration(CPWMC this)
{
  uint8_t bIndex;
  uint32_t wPhaseAOffset=0u, wPhaseBOffset=0u; 
  pDParams_t pLocalDParams = DCLASS_PARAMS; 
  DVars_t *pLocalVars_Str = &DCLASS_VARS;
  
  ADC_InjectedChannelConfig(ADC1, pLocalDParams->hIaChannel, 1u, 
                                            pLocalDParams->b_IaSamplingTime);
  ADC_InjectedChannelConfig(ADC2, pLocalDParams->hIbChannel, 1u, 
                                              pLocalDParams->b_IbSamplingTime); 
  
  /* ADC1 Injected end of conversions interrupt disabling */
  ADC_ITConfig(ADC1, ADC_IT_JEOC, DISABLE);
  
  /* ADC1 Injected conversions trigger is given by software and enabled */ 
  ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_None);  
  ADC_ExternalTrigInjectedConvCmd(ADC1,ENABLE); 
  
  /* ADC Channel used for current reading are read 
  in order to get zero currents ADC values*/ 
  for(bIndex = NB_CONVERSIONS; bIndex !=0u; bIndex--)
  {
    /* Clear the ADC1 JEOC pending flag */
    ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);   
    ADC_SoftwareStartInjectedConvCmd(ADC1,ENABLE);
    while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_JEOC)) { }
    
    wPhaseAOffset += (ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_1));
    wPhaseBOffset += (ADC_GetInjectedConversionValue(ADC2,ADC_InjectedChannel_1));
  }
  
  pLocalVars_Str->hPhaseAOffset = (uint16_t)(wPhaseAOffset>>3);
  pLocalVars_Str->hPhaseBOffset = (uint16_t)(wPhaseBOffset>>3);
      
  ADC_ClearFlag(ADC1, ADC_FLAG_JEOC); 	  
  
  /* ADC1 Injected conversions trigger is TIM1 Trigger Output */ 
  ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_TRGO); 
  
  /* ADC1 Injected conversions end interrupt enabling */
  ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE); 
}

/**
* @brief  It computes and return latest converted motor phase currents motor
* @param  this: related object of class CILM1_PWMC
* @retval Ia and Ib current in Curr_Components format
*/
static void ILM1_GetPhaseCurrents(CPWMC this,Curr_Components* pStator_Currents)
{
  int32_t wAux;
  uint16_t hReg;
  DVars_t *pLocalVars_Str = &DCLASS_VARS;

  /* Clear TIMx Update Flag necessary to detect FOC duration SW error */
  TIM1->SR &= (uint16_t)(~TIM_FLAG_Update); 

 /* Ia = (hPhaseAOffset)-(PHASE_A_ADC_CHANNEL vale)  */
  hReg = (uint16_t)((ADC1->JDR1)<<1);
  wAux = (int32_t)(hReg)-(int32_t)(pLocalVars_Str->hPhaseAOffset);
 
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
  wAux = (int32_t)(hReg)-(int32_t)(pLocalVars_Str->hPhaseBOffset);
  
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
  * @brief  It turns on low sides switches. This function is intended to be 
  *         used for charging boot capacitors of driving section. It has to be 
  *         called each motor start-up when using high voltage drivers
  * @param  this: related object of class CILM1_PWMC
  * @retval none
  */
static void ILM1_TurnOnLowSides(CPWMC this)
{    
  pDParams_t pLocalDParams = DCLASS_PARAMS;
  
  ((_CPWMC) this)->Vars_str.hCntPhA = 
    ((_CPWMC) this)->Vars_str.hCntPhB =
      ((_CPWMC) this)->Vars_str.hCntPhC = 0u;
    
  ILM1_WriteTIMRegisters(this);
  
  TIM_ClearFlag(TIM1,TIM_FLAG_Update);
  while (TIM_GetFlagStatus(TIM1,TIM_FLAG_Update) == RESET)
  {}
  
  /* Main PWM Output Enable */
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
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
* @param  this: related object of class CILM1_PWMC
* @retval none
*/
static void ILM1_SwitchOnPWM(CPWMC this)
{
  pDParams_t pLocalDParams = DCLASS_PARAMS;
  
  /* Main PWM Output Enable */
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
  if ((pLocalDParams->LowSideOutputs)== ES_GPIO)
  {
    GPIO_WriteBit(pLocalDParams->hCh1NPort, pLocalDParams->hCh1NPin, Bit_SET);
    GPIO_WriteBit(pLocalDParams->hCh2NPort, pLocalDParams->hCh2NPin, Bit_SET);
    GPIO_WriteBit(pLocalDParams->hCh3NPort, pLocalDParams->hCh3NPin, Bit_SET);
  }
  ADC_ExternalTrigInjectedConvCmd(ADC1,ENABLE);
  return; 
}


/**
* @brief  It disables PWM generation on the proper Timer peripheral acting on 
*         MOE bit
* @param  this: related object of class CILM1_PWMC
* @retval none
*/
static void ILM1_SwitchOffPWM(CPWMC this)
{ 
  pDParams_t pLocalDParams = DCLASS_PARAMS;
  /* Main PWM Output Disable */
  TIM_CtrlPWMOutputs(TIM1, DISABLE);
  if ((pLocalDParams->LowSideOutputs)== ES_GPIO)
  {
    GPIO_WriteBit(pLocalDParams->hCh1NPort, pLocalDParams->hCh1NPin, Bit_RESET);
    GPIO_WriteBit(pLocalDParams->hCh2NPort, pLocalDParams->hCh2NPin, Bit_RESET);
    GPIO_WriteBit(pLocalDParams->hCh3NPort, pLocalDParams->hCh3NPin, Bit_RESET);
  }
  ADC_ExternalTrigInjectedConvCmd(ADC1,DISABLE);  
  
  return; 
}

/**
* @brief  It stores into 'this' object variables the voltage present on Ia and 
*         Ib current feedback analog channels when no current is flow into the
*         motor
* @param  this: related object of class CILM1_PWMC
* @retval none
*/
static uint16_t ILM1_WriteTIMRegisters(CPWMC this)
{
  uint16_t hAux;
  
    TIM1->CCR1 = ((_CPWMC) this)->Vars_str.hCntPhA;
    TIM1->CCR2 = ((_CPWMC) this)->Vars_str.hCntPhB;
    TIM1->CCR3 = ((_CPWMC) this)->Vars_str.hCntPhC; 
      
  if ((TIM1->SR & (u32)TIM_FLAG_Update) == TIM_FLAG_Update)
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
* @brief  Execute a regular conversion using ADC1. 
*         The function is not re-entrant (can't executed twice at the same time)
*         It returns 0xFFFF in case of conversion error.
* @param  this related object of class CILM1_PWMC
* @retval It returns converted value or oxFFFF for conversion error
*/
static uint16_t ILM1_ExecRegularConv(CPWMC this, uint8_t bChannel)
{
  pDVars_t pDVars_str = &DCLASS_VARS;
  uint32_t tmpflag = 0u;

  ADC1->SQR3 = bChannel;
  
  DMA1_Channel1->CMAR=(uint32_t)(&(pDVars_str->hRegConv));
  
  /* Reset DMA1_CH1 TC Flag */
  DMA1->IFCR = DMA1_FLAG_TC1;
  
  /* It starts software triggered regular conversion
  through bit banding access. It is equivalent to 
  ADC1->CR2 |= EXTTRIG_SWSTART_Set; */
  *(uint32_t *)(ADC1_CR2_EXTTRIG_SWSTART_BB)=(uint32_t)(0x1u);
  
  /* Wait until end of regular conversion */
  while (tmpflag == 0u)
  {
    tmpflag = (DMA1->ISR & DMA1_FLAG_TC1);
  }
 
  return (pDVars_str->hRegConv);
}

/**
* @brief  It sets the specified sampling time for the specified ADC channel
*         on ADC1. It must be called once for each channel utilized by user
* @param  ADC channel, sampling time
* @retval none
*/
static void ILM1_ADC_SetSamplingTime(CPWMC this, ADConv_t ADConv_struct)
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
static uint16_t ILM1_IsOverCurrentOccurred(CPWMC this)
{
  uint16_t retVal = MC_NO_FAULTS;
  if ((TIM1->SR & TIM_FLAG_Break) != 0u)
  {
    retVal = MC_BREAK_IN;
    TIM1->SR = (u16)~TIM_FLAG_Break;
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


#endif

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
