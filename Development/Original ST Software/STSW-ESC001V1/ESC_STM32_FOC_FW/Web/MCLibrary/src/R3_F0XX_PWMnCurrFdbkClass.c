/**
  ******************************************************************************
  * @file    R3_F0XX_PWMnCurrFdbkClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private impelementation of R3_F0XX class      
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
#include "R3_F0XX_PWMnCurrFdbkClass.h"
#include "R3_F0XX_PWMnCurrFdbkPrivate.h"
#include "MCIRQHandlerClass.h"
#include "MCIRQHandlerPrivate.h"
#include "MCLibraryConf.h"
#include "MCLibraryISRPriorityConf.h"
#include "MC_type.h"

#ifdef MC_CLASS_DYNAMIC
  #include "stdlib.h" /* Used for dynamic allocation */
#else
  _DCR3F0XX_PWMC_t R3F0XX_PWMCpool[MAX_DRV_PWMC_NUM];
  unsigned char R3F0XX_PWMC_Allocated = 0u;
#endif

#define DCLASS_PARAMS ((_DCR3F0XX_PWMC)(((_CPWMC) this)->DerivedClass))->pDParams_str
#define  CLASS_PARAMS (((_CPWMC)this)->pParams_str)
  
#define DCLASS_VARS  &(((_DCR3F0XX_PWMC)(((_CPWMC) this)->DerivedClass))->DVars_str)
#define  CLASS_VARS  &(((_CPWMC)this)->Vars_str)
  
#define TIMxCCER_MASK_CH123        ((uint32_t)  0x00000555u) 

#define NB_CONVERSIONS 16u 
#define PWM_PERIOD      pDVars->Half_PWMPeriod
  
#define ADC1_DR_Address   0x40012440  /* Value from Memory Mapping Section of the STM32F030 DataSheets */
  
#define CCMR2_CH4_DISABLE 0x8FFFu
#define CCMR2_CH4_PWM1    0x6000u
  
#define ADC_CLEAR_TRIG_EDGE_Mask  (~ADC_ExternalTrigConvEdge_RisingFalling) /* 32 bit Mask */  
  
/* Private Structures  -------------------------------------------------------*/
  


/* Function Prototypes -------------------------------------------------- */
static void* R3F0XX_IRQHandler(void *this, unsigned char flag);
static void R3F0XX_Init(CPWMC this);
static void R3F0XX_TIMxInit(CPWMC this);
static void R3F0XX_CurrentReadingCalibration(CPWMC this);
static void R3F0XX_HFCurrentsCalibration(CPWMC this,Curr_Components* pStator_Currents);
static void R3F0XX_GetPhaseCurrents(CPWMC this,Curr_Components* pStator_Currents);
static uint16_t R3F0XX_SetADCSampPointSect1(CPWMC this);
static uint16_t R3F0XX_SetADCSampPointSect2(CPWMC this);
static uint16_t R3F0XX_SetADCSampPointSect3(CPWMC this);
static uint16_t R3F0XX_SetADCSampPointSect4(CPWMC this);
static uint16_t R3F0XX_SetADCSampPointSect5(CPWMC this);
static uint16_t R3F0XX_SetADCSampPointSect6(CPWMC this);
static uint16_t R3F0XX_SetADCSampPointCalibration(CPWMC this);
static uint16_t R3F0XX_WriteTIMRegisters(CPWMC this); 
static void R3F0XX_TurnOnLowSides(CPWMC this);
static void R3F0XX_SwitchOnPWM(CPWMC this);
static void R3F0XX_SwitchOffPWM(CPWMC this);
static uint16_t R3F0XX_ExecRegularConv(CPWMC this, uint8_t bChannel);
static void R3F0XX_ADC_SetSamplingTime(CPWMC this, ADConv_t ADConv_struct);
static uint16_t R3F0XX_IsOverCurrentOccurred(CPWMC this);
static uint16_t F0XX_GPIOPin2Source(uint16_t GPIO_Pin);
static void F0X_ChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel, uint8_t ADC_SampleTime);

static void R3F0XX_ADC_DMA_Config_AB(CPWMC this,ADC_TypeDef* ADCx, uint32_t ADC_Ext_Trig_Edge);
static void R3F0XX_ADC_DMA_Config_BC(CPWMC this,ADC_TypeDef* ADCx, uint32_t ADC_Ext_Trig_Edge);
static void R3F0XX_ADC_DMA_Config_AC(CPWMC this,ADC_TypeDef* ADCx, uint32_t ADC_Ext_Trig_Edge);

/* Private Variables -------------------------------------------------------- */

/* -------------------------------------------------------------------------- */

/**
  * @brief  Creates an object of the class R3_F0XX
  * @param  pPWMnCurrFdbkParams pointer to an PWMnCurrFdbk parameters structure
  * @param  pR3_F0XXParams pointer to an R3_F0XX parameters structure
  * @retval CR3F0XX_PWMC new instance of R3_F0XX object
  */
CR3F0XX_PWMC R3F0XX_NewObject(pPWMnCurrFdbkParams_t pPWMnCurrFdbkParams, pR3_F0XXParams_t pR3_F0XXParams)
{
  _CPWMC _oPWMnCurrFdbk;
  _DCR3F0XX_PWMC _oR3_F0XX;
  
  _oPWMnCurrFdbk = (_CPWMC)PWMC_NewObject(pPWMnCurrFdbkParams);
  
#ifdef MC_CLASS_DYNAMIC
  _oR3_F0XX = (_DCR3F0XX_PWMC)calloc(1u,sizeof(_DCR3F0XX_PWMC_t));
#else
  if (R3F0XX_PWMC_Allocated  < MAX_DRV_PWMC_NUM)
  {
    _oR3_F0XX = &R3F0XX_PWMCpool[R3F0XX_PWMC_Allocated++];
  }
  else
  {
    _oR3_F0XX = MC_NULL;
  }
#endif
  
  _oR3_F0XX->pDParams_str = pR3_F0XXParams;
  _oPWMnCurrFdbk->DerivedClass = (void*)_oR3_F0XX;
  
  _oPWMnCurrFdbk->Methods_str.pIRQ_Handler = &R3F0XX_IRQHandler;
  Set_IRQ_Handler(pR3_F0XXParams->IRQnb, (_CMCIRQ)_oPWMnCurrFdbk);
  
  _oPWMnCurrFdbk->Methods_str.pPWMC_Init = &R3F0XX_Init;
  _oPWMnCurrFdbk->Methods_str.pPWMC_GetPhaseCurrents = &R3F0XX_GetPhaseCurrents;
  _oPWMnCurrFdbk->Methods_str.pPWMC_SwitchOffPWM = &R3F0XX_SwitchOffPWM;
  _oPWMnCurrFdbk->Methods_str.pPWMC_SwitchOnPWM = &R3F0XX_SwitchOnPWM;        
  _oPWMnCurrFdbk->Methods_str.pPWMC_CurrentReadingCalibr = 
    &R3F0XX_CurrentReadingCalibration;         
  _oPWMnCurrFdbk->Methods_str.pPWMC_TurnOnLowSides = &R3F0XX_TurnOnLowSides;         
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect1 = 
    &R3F0XX_SetADCSampPointSect1;        
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect2 = 
    &R3F0XX_SetADCSampPointSect2; 
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect3 = 
    &R3F0XX_SetADCSampPointSect3;        
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect4 = 
    &R3F0XX_SetADCSampPointSect4;         
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect5 = 
    &R3F0XX_SetADCSampPointSect5;        
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect6 = 
    &R3F0XX_SetADCSampPointSect6; 
  _oPWMnCurrFdbk->Methods_str.pPWMC_ExecRegularConv= &R3F0XX_ExecRegularConv;
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetSamplingTime= &R3F0XX_ADC_SetSamplingTime;
  _oPWMnCurrFdbk->Methods_str.pPWMC_IsOverCurrentOccurred = 
    &R3F0XX_IsOverCurrentOccurred;
  
  return ((CR3F0XX_PWMC)_oPWMnCurrFdbk);
}

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup PWMnCurrFdbk_R3_F0XX
  * @{
  */

/** @defgroup R3_F0XX_class_private_methods R3_F0XX class private methods
* @{
*/

/**
* @brief  It initializes TIM1, ADC1, GPIO, DMA1 and NVIC for three shunt current 
*         reading configuration using STM32F0x.
* @param  this related object of class CPWMC
* @retval none
*/
static void R3F0XX_Init(CPWMC this)
{
  /* Declaration to access variables and parameters */
  pDVars_t pDVars = DCLASS_VARS;
  pDParams_t pDParams = DCLASS_PARAMS;
  
  /* Peripheral Initialization structures declaration */ 
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  ADC_InitTypeDef   ADC_InitStructure;
  DMA_InitTypeDef   DMA_InitStructure;

  
  pDVars->Half_PWMPeriod = ((((_CPWMC) this)->pParams_str->hPWMperiod)/2u);
  
  /* Peripheral clocks enabling BEGIN --------------------------------------*/
  
  RCC->AHBENR |= RCC_AHBPeriph_CRC;
  
  /* ADC Clock mode configuration */
  ADC_ClockModeConfig(ADC1,ADC_ClockMode_AsynClk);
  /* ADC1 Periph clock enable */ 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, (FunctionalState)ENABLE);
  
  /* Enable GPIOA-GPIOF clock */
  RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA | 
                         RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC | 
                         RCC_AHBPeriph_GPIOD | RCC_AHBPeriph_GPIOE | 
                         RCC_AHBPeriph_GPIOF,(FunctionalState) ENABLE);    
  /* Enable DMA1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,(FunctionalState) ENABLE);
  
  /* Enable TIM1 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_DBGMCU, (FunctionalState)ENABLE);
  
  /* Enable the CCS */
  RCC_ClockSecuritySystemCmd((FunctionalState)(ENABLE));
  
  /* Peripheral clocks enabling END ----------------------------------------*/
	
	R3F0XX_TIMxInit(this);
  
  /* GPIOs configurations --------------------------------------------------*/
  
  /* Fills each GPIO_InitStructure member with its default value.*/
  GPIO_StructInit(&GPIO_InitStructure);

  /****** Configure phase A ADC channel GPIO as analog input ****/
    GPIO_InitStructure.GPIO_Pin = pDParams->hIaPin;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_Init(pDParams->hIaPort,&GPIO_InitStructure);
    GPIO_PinLockConfig(pDParams->hIaPort, pDParams->hIaPin);
	
  /****** Configure phase B ADC channel GPIO as analog input ****/
    GPIO_InitStructure.GPIO_Pin = pDParams->hIbPin;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_Init(pDParams->hIbPort, &GPIO_InitStructure);
    GPIO_PinLockConfig(pDParams->hIbPort, pDParams->hIbPin);
    
  /****** Configure phase C ADC channel GPIO as analog input ****/
    GPIO_InitStructure.GPIO_Pin = pDParams->hIcPin;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_Init(pDParams->hIcPort, &GPIO_InitStructure);
    GPIO_PinLockConfig(pDParams->hIcPort, pDParams->hIcPin);   
  
 /****** Configure TIM1 Channel 1, 2 and 3 Outputs on GPIO ports ******/  
 
 /* Fills each GPIO_InitStructure member with its default value.*/
    GPIO_StructInit(&GPIO_InitStructure);   
 
 /* Common GPIO_InitStructure members values for TIM1 Channel 1,2,3 configuration*/
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  

 /* TIM1 Channel 1 GPIO pin Initialization*/  
    GPIO_InitStructure.GPIO_Pin = pDParams->hCh1Pin;
    GPIO_Init(pDParams->hCh1Port, &GPIO_InitStructure);
  
 /* TIM1 Channel 2 GPIO pin Initialization*/  
    GPIO_InitStructure.GPIO_Pin = pDParams->hCh2Pin;
    GPIO_Init(pDParams->hCh2Port, &GPIO_InitStructure);
    
 /* TIM1 Channel 2 GPIO pin Initialization*/  
    GPIO_InitStructure.GPIO_Pin = pDParams->hCh3Pin;
    GPIO_Init(pDParams->hCh3Port, &GPIO_InitStructure);

 /* Alternate Function Configuration of GPIO pin set as TIM1 Channel 1,2,3 Outputs */     
    GPIO_PinAFConfig(pDParams->hCh1Port, F0XX_GPIOPin2Source(pDParams->hCh1Pin), pDParams->bCh1AF);
    GPIO_PinAFConfig(pDParams->hCh2Port, F0XX_GPIOPin2Source(pDParams->hCh2Pin), pDParams->bCh2AF);
    GPIO_PinAFConfig(pDParams->hCh3Port, F0XX_GPIOPin2Source(pDParams->hCh3Pin), pDParams->bCh3AF);
  
 /****** Configure TIMx Channel 1N, 2N and 3N Outputs on GPIO pin , if enabled ******/    
  if ((pDParams->LowSideOutputs)== LS_PWM_TIMER) 
  { 
    
    GPIO_InitStructure.GPIO_Pin = pDParams->hCh1NPin;  
    GPIO_Init(pDParams->hCh1NPort, &GPIO_InitStructure);  
    
    GPIO_InitStructure.GPIO_Pin = pDParams->hCh2NPin;  
    GPIO_Init(pDParams->hCh2NPort, &GPIO_InitStructure); 
    
    GPIO_InitStructure.GPIO_Pin = pDParams->hCh3NPin;  
    GPIO_Init(pDParams->hCh3NPort, &GPIO_InitStructure);
    
    GPIO_PinAFConfig(pDParams->hCh1NPort, F0XX_GPIOPin2Source(pDParams->hCh1NPin),pDParams->bCh1NAF);
    GPIO_PinAFConfig(pDParams->hCh2NPort, F0XX_GPIOPin2Source(pDParams->hCh2NPin),pDParams->bCh2NAF);
    GPIO_PinAFConfig(pDParams->hCh3NPort, F0XX_GPIOPin2Source(pDParams->hCh3NPin),pDParams->bCh3NAF);
    
    GPIO_PinLockConfig(pDParams->hCh1NPort, pDParams->hCh1NPin);
    GPIO_PinLockConfig(pDParams->hCh2NPort, pDParams->hCh2NPin);
    GPIO_PinLockConfig(pDParams->hCh3NPort, pDParams->hCh3NPin);
  }  
  else if ((pDParams->LowSideOutputs)== ES_GPIO)
  {
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    
    GPIO_InitStructure.GPIO_Pin = pDParams->hCh1NPin;
    GPIO_Init(pDParams->hCh1NPort, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = pDParams->hCh2NPin;
    GPIO_Init(pDParams->hCh2NPort, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = pDParams->hCh3NPin;
    GPIO_Init(pDParams->hCh3NPort, &GPIO_InitStructure);
    
    GPIO_PinLockConfig(pDParams->hCh1NPort, pDParams->hCh1NPin);
    GPIO_PinLockConfig(pDParams->hCh2NPort, pDParams->hCh2NPin);
    GPIO_PinLockConfig(pDParams->hCh3NPort, pDParams->hCh3NPin);
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  }   

  /* ***** Configure TIMx BKIN external input (GPIO pin configuration) ***** */
 
    GPIO_PinAFConfig(pDParams->hBKINPort, F0XX_GPIOPin2Source(pDParams->hBKINPin), pDParams->bBKINAF);
    GPIO_InitStructure.GPIO_Pin = pDParams->hBKINPin;  
    GPIO_Init(pDParams->hBKINPort, &GPIO_InitStructure); 
    GPIO_PinLockConfig(pDParams->hBKINPort, pDParams->hBKINPin);
    
    /* Clear TIMx break flag. */
    TIM_ClearFlag(TIM1,TIM_FLAG_Break);
  
  /* TIM1 Counter Clock stopped when the core is halted */
  DBGMCU_APB2PeriphConfig(DBGMCU_TIM1_STOP, (FunctionalState)ENABLE);
    
  /* GPIOs configurations END ---------------------------------------------*/
  
  /* ADC1 configurations BEGIN --------------------------------------------*/
  
  /* Init ADC peripherals ************************************************ */
  
  ADC_DeInit(ADC1);
  
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ContinuousConvMode = (FunctionalState)DISABLE; 
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_TRGO;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Left;
  ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
  ADC_Init(ADC1, &ADC_InitStructure); 

  /* Convert the ADC1 Channels with the ADC_SampleTime_X_Cycles sampling time 
   * NOTE that the order of sequence channels scan is defined by the number of the channel and NOT by the
   * order of activation of the single ADC_Channel_X   */  
  
  F0X_ChannelConfig(ADC1, pDParams->bIaChannel, pDParams->b_IaSamplingTime);
  F0X_ChannelConfig(ADC1, pDParams->bIbChannel, pDParams->b_IbSamplingTime);
  F0X_ChannelConfig(ADC1, pDParams->bIcChannel, pDParams->b_IcSamplingTime);

  
  /* ADC Calibration */
  ADC_GetCalibrationFactor(ADC1);
  
  ADC_DMARequestModeConfig(ADC1,ADC_DMAMode_Circular);
  
  /* Enables the ADC peripheral */
  ADC_Cmd(ADC1, (FunctionalState)ENABLE);
  
    /* Wait ADC Ready */
  while (ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY)==RESET)
  {} 
  
  /* FOC Starts after DMA1_Channel1 TC - dual sampling  */
  /* NVIC DMA1 Channel1 Interrupt configuration*/
  NVIC_InitStructure.NVIC_IRQChannel = (uint8_t) DMA1_Channel1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 1u; /* ADC_PRE_EMPTION_PRIORITY */
  NVIC_InitStructure.NVIC_IRQChannelCmd = (FunctionalState)ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* DMA1 Configuration -------------------------------------------------- */
  
  /* DMA1 Channel1 Config */
  DMA_DeInit(DMA1_Channel1);
  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)pDVars->ADC1_DMA_converted;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 3;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
 
  /* Enables the DMA1 Channel1 peripheral */
  DMA_Cmd(DMA1_Channel1, (FunctionalState) ENABLE);
  
  /* Enables the TIM1 BRK interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_UP_TRG_COM_IRQn; /* TIM1_BRK_UP_TRG_COM_IRQHandler*/
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0u; /* TIMx_UP_PRE_EMPTION_PRIORITY*/
  NVIC_InitStructure.NVIC_IRQChannelCmd = (FunctionalState)ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Clear the flags */
  pDVars->OverVoltageFlag = FALSE;
  pDVars->OverCurrentFlag = FALSE; 
  
  /* Init of "regular" conversion registers */ 
  pDVars->bRegConvRequested = 0u;
  pDVars->bRegConvIndex = 0u;    
}

/**
* @brief  It initializes TIMx peripheral for PWM generation
* @param  TIMx Timer to be initialized
* @param  this related object of class CPWMC
* @retval none
*/
static void R3F0XX_TIMxInit(CPWMC this)
{
  TIM_TimeBaseInitTypeDef TIMx_TimeBaseStructure;
  TIM_OCInitTypeDef       TIMx_OCInitStructure;
  TIM_BDTRInitTypeDef     TIMx_BDTRInitStructure;
  

  pDVars_t pDVars = DCLASS_VARS;  
  pDParams_t pDParams =DCLASS_PARAMS; 
  
  /* TIMx Registers reset */
  TIM_DeInit(TIM1);
  TIM_TimeBaseStructInit(&TIMx_TimeBaseStructure);
  
  /* Time Base configuration */
  TIMx_TimeBaseStructure.TIM_Prescaler = (uint16_t)(pDParams->bTim_Clock_Divider) - 1u;
  TIMx_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
  TIMx_TimeBaseStructure.TIM_Period = pDVars->Half_PWMPeriod;
  TIMx_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV2;
  TIMx_TimeBaseStructure.TIM_RepetitionCounter = pDParams->bRepetitionCounter;

  TIM_TimeBaseInit(TIM1, &TIMx_TimeBaseStructure);
  TIM_Cmd(TIM1, (FunctionalState)ENABLE);
  
  /* OutCompare TIM1 Structure Initialization ------------------------------- */
  
  /* Channel 1, 2,3 and 4 Configuration in PWM mode ------------------------- */  
  TIM_OCStructInit(&TIMx_OCInitStructure);

	TIMx_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  
  TIMx_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
  TIMx_OCInitStructure.TIM_Pulse = (uint32_t)(pDVars->Half_PWMPeriod)/2u; /* dummy value */

  /* Channel 1 - PWM Phase A*/
  TIMx_OCInitStructure.TIM_OCPolarity =  pDParams->hCh1Polarity;      
  TIMx_OCInitStructure.TIM_OCIdleState = pDParams->hCh1IdleState;
  if ((pDParams-> LowSideOutputs)== LS_PWM_TIMER)
  {
    TIMx_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable; 
    TIMx_OCInitStructure.TIM_OCNPolarity  = pDParams->hCh1NPolarity; 
    TIMx_OCInitStructure.TIM_OCNIdleState = pDParams->hCh1NIdleState;     
  }    
  else
  {
    TIMx_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  }    
  TIM_OC1Init(TIM1, &TIMx_OCInitStructure); 
  
  /* Channel 2  - PWM Phase B*/
  TIMx_OCInitStructure.TIM_OCPolarity = pDParams->hCh2Polarity;      
  TIMx_OCInitStructure.TIM_OCIdleState = pDParams->hCh2IdleState;    
  if ((pDParams-> LowSideOutputs)== LS_PWM_TIMER)
  {                             
  TIMx_OCInitStructure.TIM_OCNPolarity = pDParams->hCh2NPolarity; 
  TIMx_OCInitStructure.TIM_OCNIdleState = pDParams->hCh2NIdleState;         
  }
  TIM_OC2Init(TIM1, &TIMx_OCInitStructure); 
  
  /* Channel 3  - PWM Phase C*/
  TIMx_OCInitStructure.TIM_OCPolarity = pDParams->hCh3Polarity;      
  TIMx_OCInitStructure.TIM_OCIdleState = pDParams->hCh3IdleState;    
  if ((pDParams-> LowSideOutputs)== LS_PWM_TIMER)
  {
  TIMx_OCInitStructure.TIM_OCNPolarity  = pDParams->hCh3NPolarity; 
  TIMx_OCInitStructure.TIM_OCNIdleState = pDParams->hCh3NIdleState;         
  }
  TIM_OC3Init(TIM1, &TIMx_OCInitStructure);   
  
  /* Channel 4  - Trigger for ADC Three-Shunt single ADC current reading */
  TIMx_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; 
  TIMx_OCInitStructure.TIM_OutputState =TIM_OutputState_Disable;
  TIMx_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;      
  TIMx_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset; 
  TIMx_OCInitStructure.TIM_Pulse = 0xFFFFu;
  TIM_OC4Init(TIM1, &TIMx_OCInitStructure);
  
  /* ------------------------------------------------------------------------ */
  
  /* Enables the TIMx Preload on CC1 Register */
  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
  /* Enables the TIMx Preload on CC2 Register */
  TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
  /* Enables the TIMx Preload on CC3 Register */
  TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
  /* Enables the TIMx Preload on CC4 Register */
  TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable); 
  
  /* Set channel 4 as TRGO */
  TIM_SelectOutputTrigger(TIM1,TIM_TRGOSource_OC4Ref);
  
  /* Set to default values the structure members */
  TIM_BDTRStructInit(&TIMx_BDTRInitStructure);
  
  /* Dead Time */
  TIMx_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
  TIMx_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Disable;
  TIMx_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1; 
  TIMx_BDTRInitStructure.TIM_DeadTime = (pDParams->hDeadTime)/2u;
 
  /* BKIN configuration , if enabled */
  if ((pDParams->EmergencyStop)!= DISABLE) 
  {
    TIMx_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
    
    /* External Mode of Break-In Activation */
    /* Set from the power stage */
    TIMx_BDTRInitStructure.TIM_BreakPolarity = pDParams->hBKINPolarity;
    
    TIMx_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;
  }
  TIM_BDTRConfig(TIM1,&TIMx_BDTRInitStructure);
  
  TIM_ClearITPendingBit(TIM1, TIM_IT_Break);
  TIM_ITConfig(TIM1, TIM_IT_Break, (FunctionalState)ENABLE);
  
  /* Prepare timer for synchronization */
  TIM_GenerateEvent(TIM1,TIM_EventSource_Update);
  
  TIM_SetCounter(TIM1, (uint32_t)(pDVars->Half_PWMPeriod)-1u);
}

/**
* @brief  It stores into 'this' object variables the voltage present on the  
*         current feedback analog channel when no current is flowin into the
*         motor
* @param  this related object of class CPWMC
* @retval none
*/
static void R3F0XX_CurrentReadingCalibration(CPWMC this)
{
  pVars_t pVars = CLASS_VARS;
  pDVars_t pDVars = DCLASS_VARS;
  pDParams_t pDParams = DCLASS_PARAMS;
 
  TIM_TypeDef*  TIMx = TIM1;
  
  uint16_t hCalibrationPeriodCounter;
  uint16_t hMaxPeriodsNumber;
  
  /* Set the CALIB flags to indicate the ADC calibartion phase*/
  pDVars->hFlags |= CALIB;

  pDVars-> wPhaseAOffset = 0u;
  pDVars-> wPhaseBOffset = 0u; 
  pDVars-> wPhaseCOffset = 0u; 
  
  pDVars->bIndex = 0u;
  
  /* ADC sequence conversion detection to a correct DMA Buffer reading */ 
  if(pDParams->bIaChannel < pDParams->bIbChannel)
  {
    if(pDParams->bIbChannel < pDParams->bIcChannel)
    {
      pDVars->bCalib_A_index = 0u;
      pDVars->bCalib_B_index = 1u;
      pDVars->bCalib_C_index = 2u;      
    }
    else
    {
      if(pDParams->bIaChannel < pDParams->bIcChannel)
      {
        pDVars->bCalib_A_index = 0u;
        pDVars->bCalib_B_index = 2u;
        pDVars->bCalib_C_index = 1u; 
      }
      else
      {
        pDVars->bCalib_A_index = 1u;
        pDVars->bCalib_B_index = 2u;
        pDVars->bCalib_C_index = 0u;       
      }
    }    
  }
  else
  {
    if(pDParams->bIaChannel < pDParams->bIcChannel)
    {
        pDVars->bCalib_A_index = 1u;
        pDVars->bCalib_B_index = 0u;
        pDVars->bCalib_C_index = 2u;       
    }
    else
    {
      if(pDParams->bIbChannel < pDParams->bIcChannel)
      {
        pDVars->bCalib_A_index = 2u;
        pDVars->bCalib_B_index = 0u;
        pDVars->bCalib_C_index = 1u; 
      }
      else
      {
        pDVars->bCalib_A_index = 2u;
        pDVars->bCalib_B_index = 1u;
        pDVars->bCalib_C_index = 0u;         
      }   
    }    
  }
  
  /* It forces inactive level on TIMx CHy and CHyN */
  TIMx->CCER &= (~TIMxCCER_MASK_CH123);
  
  /* Offset calibration for A B c phases */
  /* Change function to be executed in ADCx_ISR */ 
  ((_CPWMC) this)->Methods_str.pPWMC_GetPhaseCurrents     = &R3F0XX_HFCurrentsCalibration;
  ((_CPWMC) this)->Methods_str.pPWMC_SetADCSampPointSect1 = &R3F0XX_SetADCSampPointCalibration;
  ((_CPWMC) this)->Methods_str.pPWMC_SetADCSampPointSect2 = &R3F0XX_SetADCSampPointCalibration;
  ((_CPWMC) this)->Methods_str.pPWMC_SetADCSampPointSect3 = &R3F0XX_SetADCSampPointCalibration;
  ((_CPWMC) this)->Methods_str.pPWMC_SetADCSampPointSect4 = &R3F0XX_SetADCSampPointCalibration;
  ((_CPWMC) this)->Methods_str.pPWMC_SetADCSampPointSect5 = &R3F0XX_SetADCSampPointCalibration;
  ((_CPWMC) this)->Methods_str.pPWMC_SetADCSampPointSect6 = &R3F0XX_SetADCSampPointCalibration;

  R3F0XX_SwitchOnPWM(this);
  
  /* Wait for NB_CONVERSIONS to be executed */
  hMaxPeriodsNumber=(NB_CONVERSIONS+1u)*(((uint16_t)(pDParams->bRepetitionCounter)+1u)>>1);
  
  TIMx->SR = (uint16_t)~TIM_FLAG_CC1;
  
  hCalibrationPeriodCounter = 0u;
  while (pDVars->bIndex < NB_CONVERSIONS)
  {
    if (TIMx->SR & TIM_FLAG_CC1)
    {
      TIMx->SR = (uint16_t)~TIM_FLAG_CC1;       /* Cancel Flag */
      hCalibrationPeriodCounter++;
      if (hCalibrationPeriodCounter >= hMaxPeriodsNumber)
      {
        if (pDVars->bIndex < NB_CONVERSIONS)
        {
          pVars->SWerror = 1u;
          break;
        }
      }
    }
    else
    {

    }
  }
  
  R3F0XX_SwitchOffPWM(this);

  /* Mean Value of PhaseCurrents Offset calculation by 4bit shifting operation 
  *  instead division by NB_CONVERSIONS value fixed to 16. */
  
  pDVars->wPhaseAOffset>>=4; 
  pDVars->wPhaseBOffset>>=4; 
  pDVars->wPhaseCOffset>>=4; 
  
  /* Change back function to be executed in ADCx_ISR */ 
  ((_CPWMC) this)->Methods_str.pPWMC_GetPhaseCurrents     = &R3F0XX_GetPhaseCurrents;
  ((_CPWMC) this)->Methods_str.pPWMC_SetADCSampPointSect1 = &R3F0XX_SetADCSampPointSect1;
  ((_CPWMC) this)->Methods_str.pPWMC_SetADCSampPointSect2 = &R3F0XX_SetADCSampPointSect2;
  ((_CPWMC) this)->Methods_str.pPWMC_SetADCSampPointSect3 = &R3F0XX_SetADCSampPointSect3;
  ((_CPWMC) this)->Methods_str.pPWMC_SetADCSampPointSect4 = &R3F0XX_SetADCSampPointSect4;
  ((_CPWMC) this)->Methods_str.pPWMC_SetADCSampPointSect5 = &R3F0XX_SetADCSampPointSect5;
  ((_CPWMC) this)->Methods_str.pPWMC_SetADCSampPointSect6 = &R3F0XX_SetADCSampPointSect6;

  /* It over write TIMx CCRy wrongly written by FOC during calibration so as to 
     force 50% duty cycle on the three inverer legs */
  /* Disable TIMx preload */  
  
  TIMx->CCMR1 &= 0xF7F7u;
  TIMx->CCMR2 &= 0xF7F7u;
  TIMx->CCR1 = pDVars->Half_PWMPeriod;
  TIMx->CCR2 = pDVars->Half_PWMPeriod;
  TIMx->CCR3 = pDVars->Half_PWMPeriod;
  
  /* Enable TIMx preload */
  TIMx->CCMR1 |= 0x0808u;
  TIMx->CCMR2 |= 0x0808u;
  
  /* It re-enable drive of TIMx CHy and CHyN by TIMx CHyRef*/
  TIMx->CCER |= TIMxCCER_MASK_CH123;
  
  pDVars->BrakeActionLock = FALSE;
  
  /* Reset the CALIB flags to indicate the end of ADC calibartion phase*/
  pDVars->hFlags &= (~CALIB);
  
}

/**
* @brief  It computes and return latest converted motor phase currents motor
* @param  this related object of class CPWMC
* @retval Curr_Components Ia and Ib current in Curr_Components format
*/
static void R3F0XX_GetPhaseCurrents(CPWMC this,Curr_Components* pStator_Currents)
{ 
  uint8_t bSector;
  uint32_t wuAux;
  int32_t wAux;
  uint16_t hReg1,hReg2;  

  pVars_t pVars = CLASS_VARS;
  pDVars_t pDVars = DCLASS_VARS;
  pDParams_t pDParams = DCLASS_PARAMS;  
  
  /* Clear the flag to indicate the start of FOC algorithm*/
  TIM_ClearFlag(TIM1, (uint16_t)(TIM_FLAG_Update));  
   
  bSector = (uint8_t) pVars->hSector;
  
  switch (bSector)
  {
  case SECTOR_4:
  case SECTOR_5: 
    /* Current on Phase C is not accessible     */
    
    /* Phase Currents conversion sequence detection. It depends on the value of
       ADC_Channel. It permits to read from ADC1_Converted DMA Buffer the correct
       value of Pase Currents */
    if(pDParams->bIaChannel < pDParams->bIbChannel)
    {
      hReg1 = pDVars->ADC1_DMA_converted[0]; 
      hReg2 = pDVars->ADC1_DMA_converted[1];    
    }
    else
    {
      hReg1 = pDVars->ADC1_DMA_converted[1];
      hReg2 = pDVars->ADC1_DMA_converted[0];
    }  
    /* ----------------------------------------------------------------------- */      
    
    /* Ia = PhaseAOffset - ADC converted value) ------------------------------*/
        
    wAux = (int32_t)(pDVars->wPhaseAOffset)-(int32_t)(hReg1);
    
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
    
    /* Ib = PhaseBOffset - ADC converted value) ------------------------------*/
    
    wAux = (int32_t)(pDVars->wPhaseBOffset)-(int32_t)(hReg2);
      
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
    break;
 
  case SECTOR_6:
  case SECTOR_1:  
    /* Current on Phase A is not accessible     */
    
    /* Phase Currents conversion sequence detection. It depends on the value of
       ADC_Channel. It permits to read from ADC1_Converted DMA Buffer the correct
       value of Pase Currents */
    if(pDParams->bIbChannel < pDParams->bIcChannel)
    {
      hReg1 = pDVars->ADC1_DMA_converted[0];
      hReg2 = pDVars->ADC1_DMA_converted[1];  
    }
    else
    {
      hReg1 = pDVars->ADC1_DMA_converted[1];
      hReg2 = pDVars->ADC1_DMA_converted[0]; 
    }
    /* ----------------------------------------------------------------------- */
    
    /* Ib = (PhaseBOffset - ADC converted value) ------------------------------*/ 
    wAux = (int32_t)(pDVars->wPhaseBOffset)-(int32_t)(hReg1);
    
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
    
    wAux = (int32_t)(pDVars->wPhaseCOffset)-(int32_t)(hReg2); 
    /* Ia = -Ic -Ib ----------------------------------------------------------*/
    wAux =-wAux - (int32_t)pStator_Currents->qI_Component2;           /* Ia  */         

    /* Saturation of Ia */
    if (wAux> S16_MAX)
    {
      pStator_Currents->qI_Component1 = S16_MAX;
    }
    else  if (wAux <S16_MIN)
    {
      pStator_Currents->qI_Component1 = S16_MIN;
    }
    else
    {  
      pStator_Currents->qI_Component1 = (int16_t)wAux;
    }
    break;
 
  case SECTOR_2:
  case SECTOR_3:
   /* Current on Phase B is not accessible     */
    
   /* Phase Currents conversion sequence detection. It depends on the value of
    ADC_Channel. It permits to read from ADC1_Converted DMA Buffer the correct
    value of Pase Currents */
    if(pDParams->bIaChannel < pDParams->bIcChannel)
    {
      hReg1 = pDVars->ADC1_DMA_converted[0];
      hReg2 = pDVars->ADC1_DMA_converted[1];  
    }
    else
    {
      hReg1 = pDVars->ADC1_DMA_converted[1];
      hReg2 = pDVars->ADC1_DMA_converted[0]; 
    }
    /* ----------------------------------------------------------------------- */
    
    
    /* Ia = PhaseAOffset - ADC converted value) ------------------------------*/    
    wAux = (int32_t)(pDVars->wPhaseAOffset)-(int32_t)(hReg1);
    
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

    /* Ic = PhaseCOffset - ADC converted value) ------------------------------*/    
    wAux = (int32_t)(pDVars->wPhaseCOffset)-(int32_t)(hReg2);

    /* Ib = -Ic -Ia */
    wAux = -wAux -  (int32_t)pStator_Currents->qI_Component1;           /* Ib  */     

    /* Saturation of Ib */
    if (wAux> S16_MAX)
    {
      pStator_Currents->qI_Component2=S16_MAX;
    }
    else  if (wAux <S16_MIN)
    {  
      pStator_Currents->qI_Component2 = S16_MIN;
    }
    else  
    {
      pStator_Currents->qI_Component2 = (int16_t)wAux;
    }                     
    break;
    
  default:
    break;
  }  
  
  /* Regular Conversion during CURRENT_READING_ON state (START, RUN, ..)  */
  
  if (pDVars->bRegConvRequested != 0u)
  {
    /* Exec regular conversion @ bRegConvIndex */
    
    uint8_t bRegConvCh = pDVars->bRegConvCh[pDVars->bRegConvIndex];
    
  /* Stop of ADC conversion to allow writing of the CHSELR register */       
    ADC_StopOfConversion(ADC1);
    
    /* Wait Stop */
    while (ADC_GetFlagStatus(ADC1,ADC_FLAG_ADSTART) == SET)
    {
    }       
    
    /* Set Sampling time and channel of ADC for Regular Conversion */   
    wuAux = 1u;
    ADC1->CHSELR = wuAux <<  bRegConvCh;
    ADC1->SMPR = (uint32_t)(pDVars->bRegSmpTime[bRegConvCh]) & 0x00000007u;
   
    /* If external trigger is disabled the StartOfConversion(ADC1) starts 
    the regular A/D conversion immediately */
    
    /* Disabling External Trigger of ADC */
    ADC1->CFGR1 &= ADC_CLEAR_TRIG_EDGE_Mask;
    
    /* Clear EOC */
    ADC_ClearFlag(ADC1,ADC_FLAG_EOC);
    
    /* Disabling ADC DMA request  */
    ADC_DMACmd(ADC1, (FunctionalState) DISABLE);
    
     /* Start ADC for regular conversion */
    ADC_StartOfConversion(ADC1);  
 
    /* Activation of the regular conversion ongoing flag used into WriteTIMRegisters method */
    pDVars->hFlags |= REGCONVONGOING;
  }
}

/**
* @brief  Configure the ADC for the current sampling during calibration.
*         It means set the sampling point via TIMx_Ch4 value and polarity
*         ADC sequence length and channels.
*         And call the WriteTIMRegisters method.
* @param  this related object of class CPWMC
* @retval none
*/
static uint16_t R3F0XX_SetADCSampPointCalibration(CPWMC this)
{
  pDVars_t pDVars = DCLASS_VARS;
  pDParams_t pDParams = DCLASS_PARAMS;
  
  TIM_TypeDef*  TIMx = TIM1;
  
  TIMx->CCR4 = (uint32_t)(PWM_PERIOD) - 1u;
  
  /* Stop of ADC conversion to allow writing of the CHSELR register */ 
  ADC_StopOfConversion(ADC1);
  
  /* Wait Stop */
  while (ADC_GetFlagStatus(ADC1,ADC_FLAG_ADSTART) == SET)
  {
  }       
  
  ADC1->CHSELR = 0x00000000; 
  F0X_ChannelConfig(ADC1, pDParams->bIaChannel, pDParams->b_IaSamplingTime);
  F0X_ChannelConfig(ADC1, pDParams->bIbChannel, pDParams->b_IbSamplingTime);
  F0X_ChannelConfig(ADC1, pDParams->bIcChannel, pDParams->b_IcSamplingTime);
  
  /* Enabling and Configuration of External Trigger of ADC*/
  ADC1->CFGR1 &= ADC_CLEAR_TRIG_EDGE_Mask;
  ADC1->CFGR1 |= ADC_ExternalTrigConvEdge_Rising;
  
  /* Clear EOC */
  ADC_ClearFlag(ADC1,ADC_FLAG_EOC);
    
  /* Enable ADC DMA request*/
  ADC_DMACmd(ADC1, (FunctionalState) ENABLE);
  
  /* Start ADC*/
  ADC_StartOfConversion(ADC1); 
  
  return R3F0XX_WriteTIMRegisters(this);
}

/**
* @brief  Configure the ADC for the current sampling related to sector 1.
*         It means set the sampling point via TIM1_Ch4 value,
*         ADC sequence and channels.
*         And call the WriteTIMRegisters method.
* @param  this related object of class CPWMC
* @retval none
*/
static uint16_t R3F0XX_SetADCSampPointSect1(CPWMC this)
{
  /* Class Structures definition --------------------------------------------*/
  pVars_t pVars = CLASS_VARS;
  pDVars_t pDVars = DCLASS_VARS;
  pDParams_t pDParams = DCLASS_PARAMS;
  
  /* Variables --------------------------------------------------------------*/
  uint16_t hCntSmp, hDeltaDuty;
  
  /* Peripherals structures -------------------------------------------------*/
  TIM_TypeDef*  TIMx = TIM1;
  

  /* Check if sampling AB in the middle of PWM is possible */
  if (((uint16_t)(PWM_PERIOD-pVars->hCntPhA) > pDParams->hTafter) &&
      ((uint16_t)(PWM_PERIOD-pVars->hCntPhB) > pDParams->hTafter) &&
      ((uint16_t)(PWM_PERIOD-pVars->hCntPhC) > pDParams->hTafter))
  {
    TIMx->CCR4 = (uint32_t)(PWM_PERIOD) - 1u;
    
    ((_CPWMC)this)->Vars_str.hSector = SECTOR_4; /* Dummy just for the GetPhaseCurrent */
    
    /* This function set the size of DMA buffer and the ADC channels to convert and their 
        sampling time. The order of the sequence of conversion it's not configurable 
        but depends on the number of ADC channels. */
    
    R3F0XX_ADC_DMA_Config_AB(this, ADC1, ADC_ExternalTrigConvEdge_Rising); 
    
  }
  else
  {
    uint32_t wADC_Trig_Edge = ADC_ExternalTrigConvEdge_Rising;
  
    if ((uint16_t)(PWM_PERIOD-pVars->hCntPhA) > pDParams->hTafter)
    {
      hCntSmp = PWM_PERIOD - 1u;
    }
    else
    {
      hDeltaDuty = (uint16_t)(pVars->hCntPhA - pVars->hCntPhB);
      
      /* Definition of crossing point */
      if (hDeltaDuty > (uint16_t)(PWM_PERIOD-pVars->hCntPhA)*2u)
      {
        hCntSmp = pVars->hCntPhA - pDParams->hTbefore;
      }
      else
      {
        hCntSmp = pVars->hCntPhA + pDParams->hTafter;
        
        if (hCntSmp >= PWM_PERIOD)
        { 
          wADC_Trig_Edge= ADC_ExternalTrigConvEdge_Falling; 
          
          hCntSmp = (2u * PWM_PERIOD) - hCntSmp - 1u;
        }
      }
    }
    
    R3F0XX_ADC_DMA_Config_BC(this,ADC1,wADC_Trig_Edge);
         
   /* Set TIMx_CH4 value */
    TIMx->CCR4 = hCntSmp;
  }
  return R3F0XX_WriteTIMRegisters(this);
}

/**
* @brief  Configure the ADC for the current sampling related to sector 2.
*         It means set the sampling point via TIM1_Ch4 value and polarity
*         ADC sequence length and channels.
*         And call the WriteTIMRegisters method.
* @param  this related object of class CPWMC
* @retval none
*/
static uint16_t R3F0XX_SetADCSampPointSect2(CPWMC this)
{
  
  /* Class Structures definition --------------------------------------------*/
  pVars_t pVars = CLASS_VARS;
  pDVars_t pDVars = DCLASS_VARS;
  pDParams_t pDParams = DCLASS_PARAMS;
  
  /* Variables --------------------------------------------------------------*/
  uint16_t hCntSmp, hDeltaDuty;
  
  /* Peripherals structures -------------------------------------------------*/
  TIM_TypeDef*  TIMx = TIM1;
  
  /* Check if sampling AB in the middle of PWM is possible */
  if (((uint16_t)(PWM_PERIOD-pVars->hCntPhA) > pDParams->hTafter) &&
      ((uint16_t)(PWM_PERIOD-pVars->hCntPhB) > pDParams->hTafter) &&
      ((uint16_t)(PWM_PERIOD-pVars->hCntPhC) > pDParams->hTafter))
  {
    TIMx->CCR4 = (uint32_t)(PWM_PERIOD) - 1u;
    
    ((_CPWMC)this)->Vars_str.hSector = SECTOR_4; /* Dummy just for the GetPhaseCurrent */
    
    R3F0XX_ADC_DMA_Config_AB(this,ADC1,ADC_ExternalTrigConvEdge_Rising);
    
  }
  else
  {
    uint32_t wADC_Trig_Edge = ADC_ExternalTrigConvEdge_Rising ;
  
    if ((uint16_t)(PWM_PERIOD-pVars->hCntPhA) > pDParams->hTafter)
    {
      hCntSmp = PWM_PERIOD - 1u;
    }
    else
    {
      hDeltaDuty = (uint16_t)(pVars->hCntPhB - pVars->hCntPhA);
      
      /* Definition of crossing point */
      if (hDeltaDuty > (uint16_t)(PWM_PERIOD-pVars->hCntPhB)*2u)
      {
        hCntSmp = pVars->hCntPhB - pDParams->hTbefore;
      }
      else
      {
        hCntSmp = pVars->hCntPhB + pDParams->hTafter;
        
        if (hCntSmp >= PWM_PERIOD)
        { 
        
          wADC_Trig_Edge= ADC_ExternalTrigConvEdge_Falling;
          
          
          hCntSmp = (2u * PWM_PERIOD) - hCntSmp - 1u;
        }
      }
    }
    R3F0XX_ADC_DMA_Config_AC(this,ADC1,wADC_Trig_Edge);
         
   /* Set TIMx_CH4 value */
    TIMx->CCR4 = hCntSmp;
  }
  return R3F0XX_WriteTIMRegisters(this);
}

/**
* @brief  Configure the ADC for the current sampling related to sector 3.
*         It means set the sampling point via TIM1_Ch4 value and polarity
*         ADC sequence length and channels.
*         And call the WriteTIMRegisters method.
* @param  this related object of class CPWMC
* @retval none
*/
static uint16_t R3F0XX_SetADCSampPointSect3(CPWMC this)
{
  
  /* Class Structures definition --------------------------------------------*/
  pVars_t pVars = CLASS_VARS;
  pDVars_t pDVars = DCLASS_VARS;
  pDParams_t pDParams = DCLASS_PARAMS;
  
  /* Variables --------------------------------------------------------------*/
  uint16_t hCntSmp, hDeltaDuty;
  
  /* Peripherals structures -------------------------------------------------*/
  TIM_TypeDef*  TIMx = TIM1;

  
  /* Check if sampling AB in the middle of PWM is possible */
  if (((uint16_t)(PWM_PERIOD-pVars->hCntPhA) > pDParams->hTafter) &&
      ((uint16_t)(PWM_PERIOD-pVars->hCntPhB) > pDParams->hTafter) &&
      ((uint16_t)(PWM_PERIOD-pVars->hCntPhC) > pDParams->hTafter))
  {
    TIMx->CCR4 = (uint32_t)(PWM_PERIOD) - 1u;
    
    ((_CPWMC)this)->Vars_str.hSector = SECTOR_4; /* Dummy just for the GetPhaseCurrent */
    
    R3F0XX_ADC_DMA_Config_AB(this,ADC1,ADC_ExternalTrigConvEdge_Rising);
    
  }
  else
  {
        uint32_t wADC_Trig_Edge = ADC_ExternalTrigConvEdge_Rising ;
    
    if ((uint16_t)(PWM_PERIOD-pVars->hCntPhB) > pDParams->hTafter)
    {
      hCntSmp = PWM_PERIOD - 1u;
    }
    else
    {
      hDeltaDuty = (uint16_t)(pVars->hCntPhB - pVars->hCntPhC);
      
      /* Definition of crossing point */
      if (hDeltaDuty > (uint16_t)(PWM_PERIOD-pVars->hCntPhB)*2u)
      {
        hCntSmp = pVars->hCntPhB - pDParams->hTbefore;
      }
      else
      {
        hCntSmp = pVars->hCntPhB + pDParams->hTafter;
        
        if (hCntSmp >= PWM_PERIOD)
        { 
          wADC_Trig_Edge= ADC_ExternalTrigConvEdge_Falling;
          
          hCntSmp = (2u * PWM_PERIOD) - hCntSmp - 1u;
        }
      }
    }
    R3F0XX_ADC_DMA_Config_AC(this,ADC1,wADC_Trig_Edge);
         
  /* Set TIMx_CH4 value */
    TIMx->CCR4 = hCntSmp;
  } 
  return R3F0XX_WriteTIMRegisters(this);
}

/**
* @brief  Configure the ADC for the current sampling related to sector 4.
*         It means set the sampling point via TIM1_Ch4 value and polarity
*         ADC sequence length and channels.
*         And call the WriteTIMRegisters method.
* @param  this related object of class CPWMC
* @retval none
*/
static uint16_t R3F0XX_SetADCSampPointSect4(CPWMC this)
{
  
  /* Class Structures definition --------------------------------------------*/
  pVars_t pVars = CLASS_VARS;
  pDVars_t pDVars = DCLASS_VARS;
  pDParams_t pDParams = DCLASS_PARAMS;
  
  /* Variables --------------------------------------------------------------*/
  uint16_t hCntSmp, hDeltaDuty;
  
  /* Peripherals structures -------------------------------------------------*/
  TIM_TypeDef*  TIMx = TIM1;

  
  /* Check if sampling AB in the middle of PWM is possible */
  if (((uint16_t)(PWM_PERIOD-pVars->hCntPhA) > pDParams->hTafter) &&
      ((uint16_t)(PWM_PERIOD-pVars->hCntPhB) > pDParams->hTafter) &&
      ((uint16_t)(PWM_PERIOD-pVars->hCntPhC) > pDParams->hTafter))
  {
    TIMx->CCR4 = (uint32_t)(PWM_PERIOD) - 1u;
    
    ((_CPWMC)this)->Vars_str.hSector = SECTOR_4; /* Dummy just for the GetPhaseCurrent */
    
    R3F0XX_ADC_DMA_Config_AB(this,ADC1,ADC_ExternalTrigConvEdge_Rising);
    
  }
  else
  {
    uint32_t wADC_Trig_Edge = ADC_ExternalTrigConvEdge_Rising;

    if ((uint16_t)(PWM_PERIOD-pVars->hCntPhC) > pDParams->hTafter)
    {
      hCntSmp = PWM_PERIOD - 1u;
    }
    else
    {
      hDeltaDuty = (uint16_t)(pVars->hCntPhC - pVars->hCntPhB);
      
      /* Definition of crossing point */
      if (hDeltaDuty > (uint16_t)(PWM_PERIOD-pVars->hCntPhC)*2u)
      {
        hCntSmp = pVars->hCntPhC- pDParams->hTbefore;
      }
      else
      {
        hCntSmp = pVars->hCntPhC + pDParams->hTafter;
        
        if (hCntSmp >= PWM_PERIOD)
        { 
          wADC_Trig_Edge= ADC_ExternalTrigConvEdge_Falling;
          
          hCntSmp = (2u * PWM_PERIOD) - hCntSmp - 1u;
        }
      }
    }
    R3F0XX_ADC_DMA_Config_AB(this,ADC1,wADC_Trig_Edge);
         
   /* Set TIMx_CH4 value */
    TIMx->CCR4 = hCntSmp;
  } 
  return R3F0XX_WriteTIMRegisters(this);
}

/**
* @brief  Configure the ADC for the current sampling related to sector 5.
*         It means set the sampling point via TIM1_Ch4 value and polarity
*         ADC sequence length and channels.
*         And call the WriteTIMRegisters method.
* @param  this related object of class CPWMC
* @retval none
*/
static uint16_t R3F0XX_SetADCSampPointSect5(CPWMC this)
{
  
  /* Class Structures definition --------------------------------------------*/
  pVars_t pVars = CLASS_VARS;
  pDVars_t pDVars = DCLASS_VARS;
  pDParams_t pDParams = DCLASS_PARAMS;
  
  /* Variables --------------------------------------------------------------*/
  uint16_t hCntSmp, hDeltaDuty;
  
  /* Peripherals structures -------------------------------------------------*/
  TIM_TypeDef*  TIMx = TIM1;
  
  /* Check if sampling AB in the middle of PWM is possible */
  if (((uint16_t)(PWM_PERIOD-pVars->hCntPhA) > pDParams->hTafter) &&
      ((uint16_t)(PWM_PERIOD-pVars->hCntPhB) > pDParams->hTafter) &&
      ((uint16_t)(PWM_PERIOD-pVars->hCntPhC) > pDParams->hTafter))
  {
    TIMx->CCR4 = (uint32_t)(PWM_PERIOD) - 1u;
    
    ((_CPWMC)this)->Vars_str.hSector = SECTOR_4; /* Dummy just for the GetPhaseCurrent */
    
    R3F0XX_ADC_DMA_Config_AB(this,ADC1,ADC_ExternalTrigConvEdge_Rising);
    
  }
  else
  {
    
    uint32_t wADC_Trig_Edge = ADC_ExternalTrigConvEdge_Rising;
  
    if ((uint16_t)(PWM_PERIOD-pVars->hCntPhC) > pDParams->hTafter)
    {
      hCntSmp = PWM_PERIOD - 1u;
    }
    else
    {
      hDeltaDuty = (uint16_t)(pVars->hCntPhC - pVars->hCntPhA);
      
      /* Definition of crossing point */
      if (hDeltaDuty > (uint16_t)(PWM_PERIOD-pVars->hCntPhC)*2u)
      {
        hCntSmp = pVars->hCntPhC - pDParams->hTbefore;
      }
      else
      {
        hCntSmp = pVars->hCntPhC + pDParams->hTafter;
        
        if (hCntSmp >= PWM_PERIOD)
        { 
          wADC_Trig_Edge= ADC_ExternalTrigConvEdge_Falling;
          
          hCntSmp = (2u * PWM_PERIOD) - hCntSmp - 1u;
        }
      }
    }
    R3F0XX_ADC_DMA_Config_AB(this,ADC1,wADC_Trig_Edge);
         
   /* Set TIMx_CH4 value */
    TIMx->CCR4 = hCntSmp;
  } 
  return R3F0XX_WriteTIMRegisters(this);
}

/**
* @brief  Configure the ADC for the current sampling related to sector 6.
*         It means set the sampling point via TIM1_Ch4 value and polarity
*         ADC sequence length and channels.
*         And call the WriteTIMRegisters method.
* @param  this related object of class CPWMC
* @retval none
*/
static uint16_t R3F0XX_SetADCSampPointSect6(CPWMC this)
{
  
  /* Class Structures definition --------------------------------------------*/
  pVars_t pVars = CLASS_VARS;
  pDVars_t pDVars = DCLASS_VARS;
  pDParams_t pDParams = DCLASS_PARAMS;
  
  /* Variables --------------------------------------------------------------*/
  uint16_t hCntSmp, hDeltaDuty;
  
  /* Peripherals structures -------------------------------------------------*/
  TIM_TypeDef*  TIMx = TIM1;
  
  /* Check if sampling AB in the middle of PWM is possible */
  if (((uint16_t)(PWM_PERIOD-pVars->hCntPhA) > pDParams->hTafter) &&
      ((uint16_t)(PWM_PERIOD-pVars->hCntPhB) > pDParams->hTafter) &&
      ((uint16_t)(PWM_PERIOD-pVars->hCntPhC) > pDParams->hTafter))
  {
    TIMx->CCR4 = (uint32_t)(PWM_PERIOD) - 1u;
    
    ((_CPWMC)this)->Vars_str.hSector = SECTOR_4; /* Dummy just for the GetPhaseCurrent */
    
    R3F0XX_ADC_DMA_Config_AB(this,ADC1,ADC_ExternalTrigConvEdge_Rising);
    
  }
  else
  {
    uint32_t wADC_Trig_Edge = ADC_ExternalTrigConvEdge_Rising;    
  
    if ((uint16_t)(PWM_PERIOD-pVars->hCntPhA) > pDParams->hTafter)
    {
      hCntSmp = PWM_PERIOD - 1u;
    }
    else
    {
      hDeltaDuty = (uint16_t)(pVars->hCntPhA - pVars->hCntPhC);
      
      /* Definition of crossing point */
      if (hDeltaDuty > (uint16_t)(PWM_PERIOD-pVars->hCntPhA)*2u)
      {
        hCntSmp = pVars->hCntPhA - pDParams->hTbefore;
      }
      else
      {
        hCntSmp = pVars->hCntPhA + pDParams->hTafter;
        
        if (hCntSmp >= PWM_PERIOD)
        { 

          wADC_Trig_Edge= ADC_ExternalTrigConvEdge_Falling;
          
          hCntSmp = (2u * PWM_PERIOD) - hCntSmp - 1u;
        }
      }
    }
    R3F0XX_ADC_DMA_Config_BC(this,ADC1,wADC_Trig_Edge);
         
   /* Set TIMx_CH4 value */
    TIMx->CCR4 = hCntSmp;
  } 
  return R3F0XX_WriteTIMRegisters(this);
}

/**
* @brief  Write the dutycycle into timer regiters and check for FOC duration.
* @param  this: related object of class CPWMC
* @retval none
*/
static uint16_t R3F0XX_WriteTIMRegisters(CPWMC this)
{ 
  TIM_TypeDef*  TIMx = TIM1;
  uint16_t hAux;
  uint32_t hCCR4Aux;

  pVars_t pVars = CLASS_VARS;
  pDVars_t pDVars = DCLASS_VARS;
  
  TIM1->CCR1 = pVars->hCntPhA;
  TIM1->CCR2 = pVars->hCntPhB;
  TIM1->CCR3 = pVars->hCntPhC;
  
  /* Disabling of External Trigger of ADC */
  hCCR4Aux = (uint16_t)(TIMx->CCR4);
  TIM_OC4PreloadConfig(TIMx, TIM_OCPreload_Disable);
  TIMx->CCR4 = 0xFFFFu;
  TIM_OC4PreloadConfig(TIMx, TIM_OCPreload_Enable);
  TIMx->CCR4 = hCCR4Aux;   
  
  if ((TIM1->SR & TIM_FLAG_Update) == TIM_FLAG_Update)   
  {
    hAux = MC_FOC_DURATION;
  }
  else
  {
    hAux = MC_NO_ERROR;
  }
  
  if (((_CPWMC) this)->Vars_str.SWerror == 1u)
  {
    hAux = MC_FOC_DURATION;
    ((_CPWMC) this)->Vars_str.SWerror = 0u;
  }
  
  
  if ((pDVars->hFlags & REGCONVONGOING) == 0u)
  {

  }
  else
  {
    /* Reading of ADC Converted Value */
    pDVars->hRegConvValue[pDVars->bRegConvIndex] = (uint16_t)(ADC1->DR);
        
    /* Clear regular conversion ongoing flag */
    pDVars->hFlags &= (uint16_t)~REGCONVONGOING;
    
    /* Prepare next conversion */
    pDVars->bRegConvIndex++;
    
    /* Reset of bRegConvIndex to restart the cicle of Regular conversions */
    if (pDVars->bRegConvIndex >= pDVars->bRegConvRequested)
    {
      pDVars->bRegConvIndex = 0u;
    }
    
  }
 return hAux;
}

/**
* @brief  Implementaion of PWMC_GetPhaseCurrents to be performed during 
*         calibration. It sum up ADC conversion data into wPhaseAOffset and
*         wPhaseBOffset to compute the offset introduced in the current feedback
*         network. It is required to proper configure ADC inputs before to enable
*         the offset computation.
* @param  this: related object of class CPWMC
* @retval It always returns {0,0} in Curr_Components format
*/
static void R3F0XX_HFCurrentsCalibration(CPWMC this,Curr_Components* pStator_Currents)
{ 
  TIM_TypeDef*  TIMx = TIM1;
  pDVars_t pDVars = DCLASS_VARS;
  
  /* Clear the flag to indicate the start of FOC algorithm*/
  TIM_ClearFlag(TIMx, (uint16_t)(TIM_FLAG_Update));
  
  if (pDVars->bIndex < NB_CONVERSIONS)
  {
    pDVars->wPhaseAOffset += pDVars->ADC1_DMA_converted[pDVars->bCalib_A_index];
    pDVars->wPhaseBOffset += pDVars->ADC1_DMA_converted[pDVars->bCalib_B_index];
    pDVars->wPhaseCOffset += pDVars->ADC1_DMA_converted[pDVars->bCalib_C_index];
    pDVars->bIndex++;
  }
  
  if (pDVars->bRegConvRequested != 0u)
  {
    
    uint8_t bRegConvCh;
    
    ADC_ClearFlag(ADC1,ADC_FLAG_EOC);
    
    /* Disabling External Trigger of ADC */
    ADC1->CFGR1 &= ADC_CLEAR_TRIG_EDGE_Mask;
    
    /* Disable ADC DMA request*/
    ADC_DMACmd(ADC1,(FunctionalState)DISABLE);
    
    /* Exec regular conversion @ bRegConvIndex */
    bRegConvCh = pDVars->bRegConvCh[pDVars->bRegConvIndex];
    
    /* Stop ADC to allow writing of CHSELR register of ADC */
    ADC_StopOfConversion(ADC1);
    
    /* Wait Stop */
    while (ADC_GetFlagStatus(ADC1,ADC_FLAG_ADSTART) == SET)
    {
    }  
    
    /* Set Sampling time and channel */
    ADC1->CHSELR = 0x00000000;
    ADC1->CHSELR = (uint32_t) 1u <<  bRegConvCh;
    ADC1->SMPR = (uint32_t)(pDVars->bRegSmpTime[bRegConvCh]) & 0x00000007u;
    
    /* Start ADC for regular conversion */
    ADC_StartOfConversion(ADC1);
    
    /* Activation of the regular conversion ongoing flag used into WriteTIMRegisters method */
    pDVars->hFlags |= REGCONVONGOING;
  }
}


/**
* @brief  It turns on low sides switches. This function is intended to be 
*         used for charging boot capacitors of driving section. It has to be 
*         called each motor start-up when using high voltage drivers
* @param  this related object of class CPWMC
* @retval none
*/
static void R3F0XX_TurnOnLowSides(CPWMC this)
{
  pDParams_t pDParams = DCLASS_PARAMS;
  TIM_TypeDef*  TIMx = TIM1;
  
  /* Clear Update Flag */
  TIM_ClearFlag(TIMx, (uint16_t)(TIM_FLAG_Update));
  
  /*Turn on the three low side switches */
  TIMx->CCR1 = 0u;
  TIMx->CCR2 = 0u;
  TIMx->CCR3 = 0u;
  
  /* Wait until next update */
  while (TIM_GetFlagStatus(TIMx,TIM_FLAG_Update)==RESET)
  {}
  
  /* Main PWM Output Enable */
  TIMx->BDTR |= TIM_BDTR_MOE;

  if ((pDParams->LowSideOutputs)== ES_GPIO)
  {
    GPIO_WriteBit(pDParams->hCh1NPort, pDParams->hCh1NPin, Bit_SET);
    GPIO_WriteBit(pDParams->hCh2NPort, pDParams->hCh2NPin, Bit_SET);
    GPIO_WriteBit(pDParams->hCh3NPort, pDParams->hCh3NPin, Bit_SET);
  }
  return;   
}

/**
* @brief  This function enables the PWM outputs
* @param  this related object of class CPWMC
* @retval none
*/
static void R3F0XX_SwitchOnPWM(CPWMC this)
{
  TIM_TypeDef* TIMx = TIM1;
  
  pDParams_t pDParams = DCLASS_PARAMS;
  pDVars_t pDVars = DCLASS_VARS;

  /* Wait for a new PWM period */
  TIM_ClearFlag(TIMx, (uint16_t)(TIM_FLAG_Update));
  
  while (TIM_GetFlagStatus(TIMx,TIM_FLAG_Update) == RESET)
  {}
  
  /* Clear Update Flag */
  TIM_ClearFlag(TIMx, (uint16_t)(TIM_FLAG_Update));

  /* Set all duty to 50% */
  TIMx->CCR1 = (uint32_t)(pDVars->Half_PWMPeriod) >> 1;
  TIMx->CCR2 = (uint32_t)(pDVars->Half_PWMPeriod) >> 1;
  TIMx->CCR3 = (uint32_t)(pDVars->Half_PWMPeriod) >> 1;
  TIMx->CCR4 = (uint32_t)(pDVars->Half_PWMPeriod) - 5u;
  
  while (TIM_GetFlagStatus(TIMx,TIM_FLAG_Update) == RESET)
  {}
  
  /* Main PWM Output Enable */
  TIMx->BDTR |= TIM_OSSIState_Enable; 
  TIMx->BDTR |= TIM_BDTR_MOE;
  
  if ((pDParams->LowSideOutputs)== ES_GPIO)
  {
    if ((TIMx->CCER & TIMxCCER_MASK_CH123) != 0u)
    {
      GPIO_WriteBit(pDParams->hCh1NPort, pDParams->hCh1NPin, Bit_SET);
      GPIO_WriteBit(pDParams->hCh2NPort, pDParams->hCh2NPin, Bit_SET);
      GPIO_WriteBit(pDParams->hCh3NPort, pDParams->hCh3NPin, Bit_SET);
    }
    else
    {
      /* It is executed during calibration phase the EN signal shall stay off */
      GPIO_WriteBit(pDParams->hCh1NPort, pDParams->hCh1NPin, Bit_RESET);
      GPIO_WriteBit(pDParams->hCh2NPort, pDParams->hCh2NPin, Bit_RESET);
      GPIO_WriteBit(pDParams->hCh3NPort, pDParams->hCh3NPin, Bit_RESET);
    }
  }

  /* If Calibration is TRUE */
  if ((pDVars->hFlags & CALIB) != 0u)
  {
    /* Calibration */
	  
    /* Configuration of DMA and ADC to next conversions */    
    /* It's possible write the CHSELR resgister because the ADC conversion 
       is stopped by the R3F0XX_SwitchOffPWM function */
    
    ADC1->CHSELR = 0x00000000; 
    F0X_ChannelConfig(ADC1, pDParams->bIaChannel, pDParams->b_IaSamplingTime);
    F0X_ChannelConfig(ADC1, pDParams->bIbChannel, pDParams->b_IbSamplingTime);
    F0X_ChannelConfig(ADC1, pDParams->bIcChannel, pDParams->b_IcSamplingTime);
    
    /* Setting of the DMA Buffer Size.*/
    /* NOTE. This register (CNDTRx) must not be written when the DMAy Channel x is ENABLED */    
    DMA_Cmd(DMA1_Channel1,(FunctionalState)DISABLE);
    /* Write the Buffer size on CNDTR register */
    DMA1_Channel1->CNDTR = 3u;

    /* DMA Enabling */
    DMA_Cmd(DMA1_Channel1,(FunctionalState)ENABLE);
    
    /* Clear EOC */
    ADC_ClearFlag(ADC1,ADC_FLAG_EOC);
    
    /* Enable ADC DMA request*/
    ADC_DMACmd(ADC1, (FunctionalState)ENABLE);
    
    /* External Trigger of ADC Enabling */
    ADC1->CFGR1 &= ADC_CLEAR_TRIG_EDGE_Mask;
    ADC1->CFGR1 |= ADC_ExternalTrigConvEdge_Rising;
    
    /* Start of ADC */
    ADC_StartOfConversion(ADC1);
  }
  else
  {
  /* Configuration of DMA and ADC to next conversions */
  /* It's possible write the CHSELR resgister because the ADC conversion 
     is stopped by the R3F0XX_SwitchOffPWM function */
    
  /* Selection of ADC channels to convert */  
  ADC1->CHSELR = 0x00000000;
  F0X_ChannelConfig(ADC1, pDParams->bIaChannel, pDParams->b_IaSamplingTime);
  F0X_ChannelConfig(ADC1, pDParams->bIbChannel, pDParams->b_IbSamplingTime);
  
  /* Setting of the DMA Buffer Size.*/
  /* NOTE. This register (CNDTRx) must not be written 
     when the DMAy Channel x is ENABLED*/
    
  /* DMA Disabling*/
    DMA_Cmd(DMA1_Channel1,(FunctionalState)DISABLE);
  
  /* Write the Buffer size on CNDTR register - Two currents to convert */
  DMA1_Channel1->CNDTR = 2u;
  
  /* DMA Enabling*/
  DMA_Cmd(DMA1_Channel1,(FunctionalState)ENABLE);
  
  /* Clear EOC */
  ADC_ClearFlag(ADC1,ADC_FLAG_EOC);
  
  /* Enable ADC DMA request*/
  ADC_DMACmd(ADC1, (FunctionalState)ENABLE);
  
  /* External Trigger of ADC Enabling */
  ADC1->CFGR1 &= ADC_CLEAR_TRIG_EDGE_Mask;
  ADC1->CFGR1 |= ADC_ExternalTrigConvEdge_Rising;
  
  /* Start of ADC */
  ADC_StartOfConversion(ADC1);
  
  }
  /* Clear Pending Interrupt Bits */  
  DMA_ClearITPendingBit(DMA1_IT_TC1|DMA1_IT_GL1|DMA1_IT_HT1);
  
  /* DMA Interrupt Event configuration */
  DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, (FunctionalState)ENABLE);
  
  /* Set the RUN flag to indicate the NOT IDLE condition */
  pDVars->hFlags |= CURRENT_READING_ON;
  
  return; 
}
 

/**
* @brief  It disables PWM generation on the proper Timer peripheral acting on 
*         MOE bit and reset the TIM status
* @param  this related object of class CPWMC
* @retval none
*/
static void R3F0XX_SwitchOffPWM(CPWMC this)
{
  pDParams_t pDParams = DCLASS_PARAMS;
  pDVars_t pDVars = DCLASS_VARS;
  TIM_TypeDef* TIMx = TIM1;

  /* Main PWM Output Disable */
  if (pDVars->BrakeActionLock == TRUE)
  {
  }
  else
  {
    TIMx->BDTR &= ~((uint32_t)(TIM_OSSIState_Enable));
    
    if ((pDParams->LowSideOutputs)== ES_GPIO)
    {
      GPIO_WriteBit(pDParams->hCh1NPort, pDParams->hCh1NPin, Bit_RESET);
      GPIO_WriteBit(pDParams->hCh2NPort, pDParams->hCh2NPin, Bit_RESET);
      GPIO_WriteBit(pDParams->hCh3NPort, pDParams->hCh3NPin, Bit_RESET);
    }
  }
  TIMx->BDTR &= (uint32_t)~TIM_BDTR_MOE;
  
  /* Disabling of DMA Interrupt Event configured */
  DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, (FunctionalState)DISABLE);
  
  ADC_StopOfConversion(ADC1);
    
  /* Disable ADC DMA request*/
  ADC_DMACmd(ADC1, (FunctionalState)DISABLE);  
  
  /* Clear Transmission Complete Flag  of DMA1 Channel1 */
  DMA_ClearFlag(DMA1_FLAG_TC1);
  
  /* Clear EOC */
  ADC_ClearFlag(ADC1,ADC_FLAG_EOC);  
    
  /* Reset the CURRENT_READING_ON flag to indicate the IDLE condition for the ADC Regular 
     Conversions */
  pDVars->hFlags &= (~CURRENT_READING_ON);

  return;  
}

/**
  * @brief  R3_F0XX implement MC IRQ functions
  * @param  this related object
  * @param  flag 
  *         @arg        0 TIM1 Update IRQ
  *         @arg        1 DMA TC IRQ
  *         @arg        2 TIM1 Break-in IRQ - OverCurrent

  * @retval void* It returns always MC_NULL
  */
static void* R3F0XX_IRQHandler(void* this, unsigned char flag)
{ 
  pDVars_t pDVars = DCLASS_VARS;
  pDParams_t pDParams = DCLASS_PARAMS;
  
  switch(flag)
  {
  case 0: /* TIM1 Update IRQ*/
    {
      /* Not used - The TIM1 Update IRQ is disabled for 3-Shunt mode*/     
    }
    break;
  case 1: /* DMA TC IRQ */
    {     
      /* Not used - This IRQ is disabled for 3-Shunt mode*/      
    }
    break;
  case 2:    /* TIM1 Break-in IRQ - OverCurrent */
    {
        if ((pDParams->LowSideOutputs)== ES_GPIO)
        {
          GPIO_WriteBit(pDParams->hCh1NPort, pDParams->hCh1NPin, Bit_RESET);
          GPIO_WriteBit(pDParams->hCh2NPort, pDParams->hCh2NPin, Bit_RESET);
          GPIO_WriteBit(pDParams->hCh3NPort, pDParams->hCh3NPin, Bit_RESET);
        }
      pDVars->OverCurrentFlag = TRUE;
    }
    break;
  default:
    break;
  }
  return MC_NULL;
}

/**
* @brief  Execute a regular conversion. 
*         The function is not re-entrant (can't executed twice at the same time)
*         It returns 0xFFFF in case of conversion error.
* @param  this related object of class CPWMC, ADC channel to be converted
* @param  bChannel ADC channel used for the regular conversion
* @retval uint16_t It returns converted value or oxFFFF for conversion error */

static uint16_t R3F0XX_ExecRegularConv(CPWMC this, uint8_t bChannel)
{
  
  pDVars_t pDVars = DCLASS_VARS;
  
  uint16_t hRetVal = 0xFFFFu;
  uint8_t i;
  bool bRegChFound = FALSE;
  uint32_t wAux; 
  
  if (bChannel < 18u)
  {
    /* Check if the channel has been already requested */
    for (i = 0u; i < pDVars->bRegConvRequested; i++)
    {
      if (pDVars->bRegConvCh[i] == bChannel)
      {
        hRetVal = pDVars->hRegConvValue[i];
        bRegChFound = TRUE;
        break;
      }
    }
    if (bRegChFound == FALSE)
    {
      if (pDVars->bRegConvRequested < MAX_REG_CONVERSIONS)
      {
        /* Add new channel to the list */
        pDVars->bRegConvCh[pDVars->bRegConvRequested] = bChannel;
        i = pDVars->bRegConvRequested;        
        pDVars->bRegConvRequested++;
        
      }
    }
   
   /* ADC Regular conversions when the System state is IDLE (No Calibration and not currents reading) */ 
   if ((pDVars->hFlags & CALIB) == 0u) /* If it is not Calibration */
    {
      if ((pDVars->hFlags & CURRENT_READING_ON) == 0u) 
      {
        
       ADC_DMACmd(ADC1,(FunctionalState)DISABLE);
        
        /* ADC STOP condition requested to write CHSELR is true because of the ADCSTOP is set by hardware 
           at the end of A/D conversion if the external Trigger of ADC is disabled.*/
        
        /*By default it is ADSTART = 0, then at the first time the CFGR1 can be written. */        
        
        /* Disabling External Trigger of ADC */
        ADC1->CFGR1 &= ADC_CLEAR_TRIG_EDGE_Mask;                
        
        /* Set Sampling time and channel */
        wAux = 1u;
        ADC1->CHSELR = wAux <<  bChannel;
        ADC1->SMPR = (uint32_t)(pDVars->bRegSmpTime[bChannel]) & 0x00000007u;
              
        /* Clear EOC */
        ADC_ClearFlag(ADC1,ADC_FLAG_EOC);
        
        /* Start ADC conversion */          
        ADC_StartOfConversion(ADC1);
        
        /* Wait EOC */
        while (ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC) == RESET)
        {
        }
        
        /* Read the "Regular" conversion (Not related to current sampling) */
        hRetVal = ADC_GetConversionValue(ADC1);
        pDVars->hRegConvValue[i] = hRetVal;      
                 
      }
    }
  } 
  return hRetVal;
}

/**
* @brief  It sets the specified sampling time for the specified ADC channel
*         on ADC1. It must be called once for each channel utilized by user
* @param  this related object of class CPWMC
* @param  ADConv_struct struct containing ADC channel and sampling time
* @retval none
*/
static void R3F0XX_ADC_SetSamplingTime(CPWMC this, ADConv_t ADConv_struct)
{
  pDVars_t pDVars = DCLASS_VARS;
  
  if (ADConv_struct.Channel < 18u)
  {
    if (ADConv_struct.SamplTime < 0x07) /* register min_value:0x00 max_value:0x07 */
    {
      pDVars->bRegSmpTime[ADConv_struct.Channel] = ADConv_struct.SamplTime;
    }
  } 
}

/**
* @brief  It is used to check if an overcurrent occurred since last call.
* @param  this related object of class CPWMC
* @retval uint16_t It returns MC_BREAK_IN whether an overcurrent has been 
*                  detected since last method call, MC_NO_FAULTS otherwise.
*/
static uint16_t R3F0XX_IsOverCurrentOccurred(CPWMC this)
{
  pDVars_t pDVars = DCLASS_VARS;
  
  uint16_t retVal = MC_NO_FAULTS;
  
  if (pDVars->OverVoltageFlag == TRUE)
  {
    retVal = MC_OVER_VOLT;
    pDVars->OverVoltageFlag = FALSE;
  }
  
  if (pDVars->OverCurrentFlag == TRUE )
  {
    retVal |= MC_BREAK_IN;
    pDVars->OverCurrentFlag = FALSE;
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
static uint16_t F0XX_GPIOPin2Source(uint16_t GPIO_Pin)
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
  * @brief  Configures for the selected ADC and its sampling time.
  * @param  ADCx: where x can be 1 to select the ADC peripheral.
  * @param  ADC_Channel: the ADC channel to configure. 
  *          This parameter can be any combination of the following values:
  *            @arg ADC_Channel_0: ADC Channel0 selected
  *            @arg ADC_Channel_1: ADC Channel1 selected
  *            @arg ADC_Channel_2: ADC Channel2 selected
  *            @arg ADC_Channel_3: ADC Channel3 selected
  *            @arg ADC_Channel_4: ADC Channel4 selected
  *            @arg ADC_Channel_5: ADC Channel5 selected
  *            @arg ADC_Channel_6: ADC Channel6 selected
  *            @arg ADC_Channel_7: ADC Channel7 selected
  *            @arg ADC_Channel_8: ADC Channel8 selected
  *            @arg ADC_Channel_9: ADC Channel9 selected
  *            @arg ADC_Channel_10: ADC Channel10 selected, not available for STM32F031 devices
  *            @arg ADC_Channel_11: ADC Channel11 selected, not available for STM32F031 devices
  *            @arg ADC_Channel_12: ADC Channel12 selected, not available for STM32F031 devices
  *            @arg ADC_Channel_13: ADC Channel13 selected, not available for STM32F031 devices
  *            @arg ADC_Channel_14: ADC Channel14 selected, not available for STM32F031 devices
  *            @arg ADC_Channel_15: ADC Channel15 selected, not available for STM32F031 devices
  *            @arg ADC_Channel_16: ADC Channel16 selected
  *            @arg ADC_Channel_17: ADC Channel17 selected
  *            @arg ADC_Channel_18: ADC Channel18 selected, not available for STM32F030 devices
  * @param  ADC_SampleTime: The sample time value to be set for the selected channel. 
  *          This parameter can be one of the following values:
  *            @arg ADC_SampleTime_1_5Cycles: Sample time equal to 1.5 cycles  
  *            @arg ADC_SampleTime_7_5Cycles: Sample time equal to 7.5 cycles
  *            @arg ADC_SampleTime_13_5Cycles: Sample time equal to 13.5 cycles
  *            @arg ADC_SampleTime_28_5Cycles: Sample time equal to 28.5 cycles
  *            @arg ADC_SampleTime_41_5Cycles: Sample time equal to 41.5 cycles
  *            @arg ADC_SampleTime_55_5Cycles: Sample time equal to 55.5 cycles
  *            @arg ADC_SampleTime_71_5Cycles: Sample time equal to 71.5 cycles
  *            @arg ADC_SampleTime_239_5Cycles: Sample time equal to 239.5 cycles
  * @retval None
  */
static void F0X_ChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel, uint8_t ADC_SampleTime)
{
  uint32_t wAux = 1u;
  
  /* ADC1 Channel and sampling time config for Phase-current reading */
  ADC1->CHSELR |= wAux << ADC_Channel;
  ADC1->SMPR = (uint32_t)(ADC_SampleTime) & 0x00000007u;
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

/**
  * @brief  Configures ADC1 channels and DMA1_Channel1 to convert only A and B phase    
  * @param  this related object of class CR3F0XX_PWMC
  * @param  DMAy_Channelx: where y can be 1 to select the DMA and
  *         x can be 1 to 7 for DMA1 to select the DMA Channel.
  * @param  ADCx: where x can be 1 to select the ADC1 peripheral.
  * @retval none
  */
static void R3F0XX_ADC_DMA_Config_AB(CPWMC this,ADC_TypeDef* ADCx, uint32_t ADC_Ext_Trig_Edge)
{
  pDParams_t pDParams = DCLASS_PARAMS;

  /* ADSTART is set to Zero automatically after the Regular conversion 
     because the External trigger is disabled; then at this point the ADC 
     is STOPPED and it's possible to write on the CHSELR and on the CFGR registers of ADC */
  
  /* Selection of ADC channel to convert */
  ADCx->CHSELR = 0x00000000;
  F0X_ChannelConfig(ADCx, pDParams->bIaChannel, pDParams->b_IaSamplingTime);
  F0X_ChannelConfig(ADCx, pDParams->bIbChannel, pDParams->b_IbSamplingTime);
  
  /* Enabling and Configuration of External Trigger of ADC*/
  ADCx->CFGR1 &= ADC_CLEAR_TRIG_EDGE_Mask;
  ADCx->CFGR1 |= ADC_Ext_Trig_Edge;
  
  /* Clear EOC */
  ADC_ClearFlag(ADC1,ADC_FLAG_EOC);
    
  ADC_DMACmd(ADC1, (FunctionalState) ENABLE);
  
  /* Start ADC. It will wait the Trigger event configured into CFGR1*/
  ADC_StartOfConversion(ADCx);  

}

/**
  * @brief  Configures ADC1 channels and DMA1_Channel1 to convert only B and C phase    
  * @param  this related object of class CR3F0XX_PWMC
  * @param  DMAy_Channelx: where y can be 1 to select the DMA and
  *         x can be 1 to 7 for DMA1 to select the DMA Channel.
  * @param  ADCx: where x can be 1 to select the ADC1 peripheral.
  * @retval none
  */
static void R3F0XX_ADC_DMA_Config_BC(CPWMC this,ADC_TypeDef* ADCx, uint32_t ADC_Ext_Trig_Edge)
{
  pDParams_t pDParams = DCLASS_PARAMS;
  
  /* ADSTART is set to Zero automatically after the Regular conversion 
  because the External trigger is disabled; then at this point the ADC 
  is STOPPED and it's possible to write on the CHSELR and on the CFGR registers of ADC */
  
  ADCx->CHSELR = 0x00000000;
  F0X_ChannelConfig(ADCx, pDParams->bIbChannel, pDParams->b_IbSamplingTime);
  F0X_ChannelConfig(ADCx, pDParams->bIcChannel, pDParams->b_IcSamplingTime);
  
  /* Enabling and Configuration of External Trigger of ADC*/
  ADCx->CFGR1 &= ADC_CLEAR_TRIG_EDGE_Mask;
  ADCx->CFGR1 |= ADC_Ext_Trig_Edge;
  
  ADC_DMACmd(ADC1, (FunctionalState) ENABLE);
             
  /* Start ADC. It will wait the Trigger event configured into CFGR1*/
  ADC_StartOfConversion(ADCx); 
}

/**
  * @brief  Configures ADC1 channels and DMA1_Channel1 to convert only B and C phase    
  * @param  this related object of class CR3F0XX_PWMC
  * @param  DMAy_Channelx: where y can be 1 to select the DMA and
  *         x can be 1 to 7 for DMA1 to select the DMA Channel.
  * @param  ADCx: where x can be 1 to select the ADC1 peripheral.
  * @retval none
  */
static void R3F0XX_ADC_DMA_Config_AC(CPWMC this,ADC_TypeDef* ADCx, uint32_t ADC_Ext_Trig_Edge)
{
  pDParams_t pDParams = DCLASS_PARAMS;
  
 /* ADSTART is set to Zero automatically after the Regular conversion 
  because the External trigger is disabled; then at this point the ADC 
  is STOPPED and it's possible to write on the CHSELR and on the CFGR registers of ADC */   
  
  ADCx->CHSELR = 0x00000000;
  F0X_ChannelConfig(ADCx, pDParams->bIaChannel, pDParams->b_IaSamplingTime);
  F0X_ChannelConfig(ADCx, pDParams->bIcChannel, pDParams->b_IcSamplingTime);
  
  
  /* Enabling and Configuration of External Trigger of ADC*/
  ADCx->CFGR1 &= ADC_CLEAR_TRIG_EDGE_Mask;
  ADCx->CFGR1 |= ADC_Ext_Trig_Edge;
  
  /* Clear EOC */
  ADC_ClearFlag(ADC1,ADC_FLAG_EOC);
    
  ADC_DMACmd(ADC1, (FunctionalState) ENABLE);
             
  /* Start ADC. It will wait the Trigger event configured into CFGR1*/
  ADC_StartOfConversion(ADCx); 
}






/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/






