/**
  ******************************************************************************
  * @file    R1_VL1_PWMnCurrFdbkClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private implementation of R1_VL1_PWMnCurrFdbk IRQ      
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
#include "R1_VL1_PWMnCurrFdbkClass.h"
#include "R1_VL1_PWMnCurrFdbkPrivate.h"
#include "MCIRQHandlerClass.h"
#include "MCIRQHandlerPrivate.h"
#include "MCLibraryConf.h"
#include "MCLibraryISRPriorityConf.h"
#include "MC_type.h"

/* Private Defines -----------------------------------------------------------*/
#define CLASS_VARS   ((_CPWMC)this)->Vars_str

#define PWM_PERIOD (((_CPWMC) this)->pParams_str->hPWMperiod)/2u

/* Direct address of the registers used by DMA */
#define TIM1_CCR1_Address   0x40012C34u
#define TIM1_CCR2_Address   0x40012C38u
#define TIM1_CCR3_Address   0x40012C3Cu
#define TIM3_CCR4_Address   0x40000440u
#define TIM4_CCR3_Address   0x4000083Cu

#define ADC1_DR_Address     0x4001244Cu

#define NB_CONVERSIONS 16u

#define DCLASS_PARAMS ((_DCR1VL1_PWMC)(((_CPWMC) this)->DerivedClass))->pDParams_str

#define REGULAR         ((uint8_t)0u)
#define BOUNDARY_1      ((uint8_t)1u)  /* Two small, one big */
#define BOUNDARY_2      ((uint8_t)2u)  /* Two big, one small */
#define BOUNDARY_3      ((uint8_t)3u)  /* Three equal        */

#define INVERT_NONE 0u
#define INVERT_A 1u
#define INVERT_B 2u
#define INVERT_C 3u

#define SAMP_NO 0u
#define SAMP_IA 1u
#define SAMP_IB 2u
#define SAMP_IC 3u
#define SAMP_NIA 4u
#define SAMP_NIB 5u
#define SAMP_NIC 6u
#define SAMP_OLDA 7u
#define SAMP_OLDB 8u
#define SAMP_OLDC 9u

#define CH1NORMAL           0x0060u
#define CH2NORMAL           0x6000u
#define CH3NORMAL           0x0060u
#define CH4NORMAL           0x7000u

#define CCMR1_PRELOAD_DISABLE_MASK 0xF7F7u
#define CCMR2_PRELOAD_DISABLE_MASK 0xFFF7u

#define CCMR1_PRELOAD_ENABLE_MASK 0x0808u
#define CCMR2_PRELOAD_ENABLE_MASK 0x0008u

/* DMA ENABLE mask */
#define CCR_ENABLE_Set          ((uint32_t)0x00000001u)
#define CCR_ENABLE_Reset        ((uint32_t)0xFFFFFFFEu)

#define CR2_JEXTSEL_Reset       ((uint32_t)0xFFFF8FFFu)
#define CR2_JEXTTRIG_Set        ((uint32_t)0x00008000u)
#define CR2_JEXTTRIG_Reset      ((uint32_t)0xFFFF7FFFu)

#define TIM_DMA_ENABLED_CC1 0x0200u
#define TIM_DMA_ENABLED_CC2 0x0400u
#define TIM_DMA_ENABLED_CC3 0x0800u

#define CR2_ADON_Set                ((uint32_t)0x00000001u)

/* ADC SMPx mask */
#define SMPR1_SMP_Set              ((uint32_t) (0x00000007u))
#define SMPR2_SMP_Set              ((uint32_t) (0x00000007u))
#define CR2_EXTTRIG_SWSTART_Set     ((u32)0x00500000)

#define ADC1_CR2_EXTTRIG_SWSTART_BB 0x42248158u

#if (defined(STM32F10X_MD_VL)||defined(STM32F10X_LD_VL))
  #define ADCx_IRQn      ADC1_IRQn
  #define TIMx_UP_IRQn  TIM1_UP_TIM16_IRQn
#elif (defined(STM32F10X_MD)||defined(STM32F10X_LD)||defined(STM32F10X_HD))
  #define ADCx_IRQn ADC1_2_IRQn
  #define TIMx_UP_IRQn  TIM1_UP_IRQn
#endif

/* Constant values -----------------------------------------------------------*/
static const uint8_t REGULAR_SAMP_CUR1[6] = {SAMP_NIC,SAMP_NIC,SAMP_NIA,SAMP_NIA,SAMP_NIB,SAMP_NIB};
static const uint8_t REGULAR_SAMP_CUR2[6] = {SAMP_IA,SAMP_IB,SAMP_IB,SAMP_IC,SAMP_IC,SAMP_IA};
static const uint8_t BOUNDR1_SAMP_CUR2[6] = {SAMP_IB,SAMP_IB,SAMP_IC,SAMP_IC,SAMP_IA,SAMP_IA};
static const uint8_t BOUNDR2_SAMP_CUR1[6] = {SAMP_IA,SAMP_IB,SAMP_IB,SAMP_IC,SAMP_IC,SAMP_IA};
static const uint8_t BOUNDR2_SAMP_CUR2[6] = {SAMP_IC,SAMP_IA,SAMP_IA,SAMP_IB,SAMP_IB,SAMP_IC};

#ifdef MC_CLASS_DYNAMIC
	#include "stdlib.h" /* Used for dynamic allocation */
#else
	_DCR1VL1_PWMC_t R1VL1_PWMCpool[MAX_DRV_PWMC_NUM];
	unsigned char R1VL1_PWMC_Allocated = 0u;
#endif

static void* R1VL1_IRQHandler(void *this, unsigned char flag);
static void R1VL1_Init(CPWMC this);
static void R1VL1_TIMxInit(TIM_TypeDef* TIMx, TIM_TypeDef* TIMx_2, CPWMC this);
static void R1VL1_CurrentReadingCalibration(CPWMC this);
static void R1VL1_GetPhaseCurrents(CPWMC this,Curr_Components* pStator_Currents);
static void R1VL1_TurnOnLowSides(CPWMC this);
static void R1VL1_SwitchOnPWM(CPWMC this);
static void R1VL1_SwitchOffPWM(CPWMC this);
static void R1VL1_1ShuntMotorVarsInit(CPWMC this);
static void R1VL1_1ShuntMotorVarsRestart(CPWMC this);
static uint16_t R1VL1_CalcDutyCycles(CPWMC this);
static uint16_t R1VL1_ExecRegularConv(CPWMC this, uint8_t bChannel);
static void R1VL1_ADC_SetSamplingTime(CPWMC this, ADConv_t ADConv_struct);
static uint16_t R1VL1_IsOverCurrentOccurred(CPWMC this);
static void R1VL1_StartTimers(void);

/**
  * @brief  Creates an object of the class R1_VL1
  * @param  pPWMnCurrFdbkParams pointer to an PWMnCurrFdbk parameters structure
  * @param  pR1_VL1Params pointer to an R1_VL1 parameters structure
  * @retval CR1VL1_PWMC new instance of R1_VL1 object
  */
CR1VL1_PWMC R1VL1_NewObject(pPWMnCurrFdbkParams_t pPWMnCurrFdbkParams, pR1_VL1Params_t pR1_VL1Params)
{
	_CPWMC _oPWMnCurrFdbk;
	_DCR1VL1_PWMC _oR1_VL1;

	_oPWMnCurrFdbk = (_CPWMC)PWMC_NewObject(pPWMnCurrFdbkParams);

	#ifdef MC_CLASS_DYNAMIC
		_oR1_VL1 = (_DCR1VL1_PWMC)calloc(1u,sizeof(_DCR1VL1_PWMC_t));
	#else
		if (R1VL1_PWMC_Allocated  < MAX_DRV_PWMC_NUM)
		{
			_oR1_VL1 = &R1VL1_PWMCpool[R1VL1_PWMC_Allocated++];
		}
		else
		{
			_oR1_VL1 = MC_NULL;
		}
	#endif
  
	_oR1_VL1->pDParams_str = pR1_VL1Params;
	_oPWMnCurrFdbk->DerivedClass = (void*)_oR1_VL1;
	
	_oPWMnCurrFdbk->Methods_str.pIRQ_Handler = &R1VL1_IRQHandler;
	Set_IRQ_Handler(pR1_VL1Params->IRQnb, (_CMCIRQ)_oPWMnCurrFdbk);
  
  _oPWMnCurrFdbk->Methods_str.pPWMC_Init = &R1VL1_Init;
  _oPWMnCurrFdbk->Methods_str.pPWMC_GetPhaseCurrents = &R1VL1_GetPhaseCurrents;
  _oPWMnCurrFdbk->Methods_str.pPWMC_SwitchOffPWM = &R1VL1_SwitchOffPWM;
  _oPWMnCurrFdbk->Methods_str.pPWMC_SwitchOnPWM = &R1VL1_SwitchOnPWM;        
  _oPWMnCurrFdbk->Methods_str.pPWMC_CurrentReadingCalibr = 
                                                 &R1VL1_CurrentReadingCalibration;         
  _oPWMnCurrFdbk->Methods_str.pPWMC_TurnOnLowSides = &R1VL1_TurnOnLowSides;         
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect1 = 
                                                      &R1VL1_CalcDutyCycles;        
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect2 = 
                                                      &R1VL1_CalcDutyCycles; 
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect3 = 
                                                      &R1VL1_CalcDutyCycles;        
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect4 = 
                                                      &R1VL1_CalcDutyCycles;         
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect5 = 
                                                      &R1VL1_CalcDutyCycles;        
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect6 = 
                                                      &R1VL1_CalcDutyCycles; 
  _oPWMnCurrFdbk->Methods_str.pPWMC_ExecRegularConv= &R1VL1_ExecRegularConv;
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetSamplingTime= &R1VL1_ADC_SetSamplingTime;
  _oPWMnCurrFdbk->Methods_str.pPWMC_IsOverCurrentOccurred = 
    &R1VL1_IsOverCurrentOccurred;
  
	return ((CR1VL1_PWMC)_oPWMnCurrFdbk);
}

/** @addtogroup STM32F10x_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup PWMnCurrFdbk_R1_VL1
  * @{
  */

/** @defgroup R1_VL1_class_private_methods R1_VL1 class private methods
* @{
*/

/**
* @brief  It initializes TIM1, ADC, GPIO, DMA1 and NVIC for single shunt current 
*         reading configuration using STM32 High Density.
* @param  this related object of class CPWMC
* @retval none
*/
static void R1VL1_Init(CPWMC this)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;
  uint16_t hAux;
  ADC_InitTypeDef ADC_InitStructure;
  pDVars_t pDVars_str;
  pDParams_t pDParams_str;
  TIM_TypeDef* AuxTIM;
  
  pDVars_str =   &(((_DCR1VL1_PWMC)(((_CPWMC) this)->DerivedClass))->DVars_str);
  pDParams_str =  ((_DCR1VL1_PWMC)(((_CPWMC) this)->DerivedClass))->pDParams_str;
  AuxTIM = pDParams_str->AuxTIM;
  
  R1VL1_1ShuntMotorVarsInit(this);
        
  /* Peripheral clocks enabling ---------------------------------------------*/
  
  RCC->AHBENR |= RCC_AHBPeriph_CRC;
  
  /* ADCCLK = PCLK2 */
  RCC_ADCCLKConfig(pDParams_str->wADC_Clock_Divider);
  
  /* Enable GPIOA-GPIOF clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | 
                         RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | 
                           RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | 
                             RCC_APB2Periph_GPIOF | RCC_APB2Periph_GPIOG, ENABLE);  
  
  /* Enable ADC1 clock - Used in any case for regular MC conversion */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  
  /* Enable the CCS */
  RCC_ClockSecuritySystemCmd((FunctionalState)(ENABLE));
    
  /* Set for enabling ADC1_2 IRQ */
  NVIC_InitStructure.NVIC_IRQChannel = (uint8_t) ADCx_IRQn;
  
  /* Enable the ADC Interrupt */
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = ADC_PRE_EMPTION_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = ADC_SUB_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
    
  /* Enable TIM1 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  
  /* Enable AUX_TIM clock and Debug MODE*/
  if (AuxTIM == TIM3)
  {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    DBGMCU_Config(DBGMCU_TIM3_STOP, ENABLE);
  }
  else
  {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    DBGMCU_Config(DBGMCU_TIM4_STOP, ENABLE);
  }
  
  /* Enable DMA clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  
  /* Set timer in Debug MODE */
  /* TIM1 Counter Clock stopped when the core is halted */
  DBGMCU_Config(DBGMCU_TIM1_STOP, ENABLE);
  
  /****** Timer1 alternate function full remapping ******/  
  if(pDParams_str->wTIM1Remapping != GPIO_NoRemap_TIM1)
  {   
    GPIO_PinRemapConfig(pDParams_str->wTIM1Remapping, ENABLE);  
  }  
  
  R1VL1_TIMxInit(TIM1, AuxTIM, this);
  
  /* DMA & NVIC Settings */
  
  /* DMA Event related to TIM1 Channel 4 */
  /* DMA1 Channel4 configuration ----------------------------------------------*/
  DMA_DeInit(DMA1_Channel4);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)TIM1_CCR1_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(pDVars_str->hDmaBuff);
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = 2u;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel4, &DMA_InitStructure);
  /* Disable DMA1 Channel4 */
  DMA_Cmd(DMA1_Channel4, ENABLE); 
  
  /* DMA Event related to AUX_TIM */
  /* DMA channel configuration */
  if (AuxTIM == TIM3)
  {
    DMA_DeInit(DMA1_Channel3);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)TIM3_CCR4_Address;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(pDVars_str->hCCDmaBuffCh4);
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = 3u;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel3, &DMA_InitStructure);
    /* Enable DMA Channel */
    DMA_Cmd(DMA1_Channel3, ENABLE);
  }
  else
  {
    DMA_DeInit(DMA1_Channel5);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)TIM4_CCR3_Address;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(pDVars_str->hCCDmaBuffCh4);
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = 3u;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);
    /* Enable DMA1 Channel5 */
    DMA_Cmd(DMA1_Channel5, ENABLE);
  }
  
  /* DMA Event related to ADC regular conversion*/
  /* DMA1 channel1 configuration */
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(&(pDVars_str->hRegConv));
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
  
  /* Enable the TIM1 Update interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = (uint8_t) TIMx_UP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIMx_UP_PRE_EMPTION_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = TIMx_UP_SUB_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);   
  
  if (pDParams_str->bRepetitionCounter > 1u)
  {
    /* Only if REP RATE > 1 */
    /* Enable the DMA1_CH4 TC interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = (uint8_t) DMA1_Channel4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = DMAx_TC_PRE_EMPTION_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = DMAx_TC_SUB_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    /* Enable DMA1 CH4 TC IRQ */
    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
    
    pDVars_str->bDMATot = (pDParams_str->bRepetitionCounter+1u)/2u;
  }
  else
  {
    /* REP RATE = 1 */
    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, DISABLE);
    pDVars_str->bDMATot = 0u;
  }
        
  /* GPIOs configurations --------------------------------------------------*/
  GPIO_StructInit(&GPIO_InitStructure);
  
  /****** Configure phase ADC channel GPIO as analog input ****/
  GPIO_InitStructure.GPIO_Pin = pDParams_str->hIPin;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(pDParams_str->hIPort, &GPIO_InitStructure);
  GPIO_PinLockConfig(pDParams_str->hIPort, pDParams_str->hIPin);
  
  /****** Configure TIMx Channel 1, 2 and 3 Outputs ******/ 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  hAux = (pDParams_str->hCh1Pin) | pDParams_str->hCh2Pin;  
  hAux = hAux | pDParams_str->hCh3Pin;  
  GPIO_InitStructure.GPIO_Pin = hAux; 
  GPIO_Init(pDParams_str->hCh1Port, &GPIO_InitStructure);
  GPIO_PinLockConfig(pDParams_str->hCh1Port, hAux);
  
  /****** Configure TIMx Channel 1N, 2N and 3N Outputs, if enabled ******/    
  if ((pDParams_str->LowSideOutputs)== LS_PWM_TIMER) 
  { 
    GPIO_InitStructure.GPIO_Pin = pDParams_str->hCh1NPin;  
    GPIO_Init(pDParams_str->hCh1NPort, &GPIO_InitStructure);  
    GPIO_PinLockConfig(pDParams_str->hCh1NPort, pDParams_str->hCh1NPin);
    
    hAux = (pDParams_str->hCh2NPin) | pDParams_str->hCh3NPin;
    GPIO_InitStructure.GPIO_Pin = hAux; 
    GPIO_Init(pDParams_str->hCh2NPort, &GPIO_InitStructure);    
    GPIO_PinLockConfig(pDParams_str->hCh2NPort, hAux);
  }  else if ((pDParams_str->LowSideOutputs)== ES_GPIO)
  {
    /* Only "active high" polarity is supported */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = pDParams_str->hCh1NPin;  
    GPIO_Init(pDParams_str->hCh1NPort, &GPIO_InitStructure);  
    GPIO_PinLockConfig(pDParams_str->hCh1NPort, pDParams_str->hCh1NPin);
    
    hAux = (pDParams_str->hCh2NPin) | (pDParams_str->hCh3NPin);
    GPIO_InitStructure.GPIO_Pin = hAux; 
    GPIO_Init(pDParams_str->hCh2NPort, &GPIO_InitStructure);    
    GPIO_PinLockConfig(pDParams_str->hCh2NPort, hAux);
  }
  else
  {
  }
      
  if ((pDParams_str->EmergencyStop)!= DISABLE)  
  {
    /****** Configure TIMx BKIN input, if enabled ******/   
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = pDParams_str->hBKINPin;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(pDParams_str->hBKINPort, &GPIO_InitStructure); 
    GPIO_PinLockConfig(pDParams_str->hBKINPort, pDParams_str->hBKINPin);
  }
  
  ADC_StructInit(&ADC_InitStructure);
  /* ADC registers configuration -----------------------------------*/
  /* ADC registers reset */  
  ADC_DeInit(ADC1);
  
  /* Enable ADC */
  ADC_Cmd(ADC1, ENABLE);
  
  /* ADC Init */
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Left;
  ADC_InitStructure.ADC_NbrOfChannel = 1u;
  ADC_Init(ADC1, &ADC_InitStructure);
  
  /* Enable external trigger (it will be SW) for ADC1 regular conversions */ 
  ADC_ExternalTrigConvCmd(ADC1, ENABLE);
    
  /* Start calibration of ADC1 */
  ADC_StartCalibration(ADC1);
  
  /* Wait for the end of ADC calibration */
  while (ADC_GetCalibrationStatus(ADC1))
  {
  }
  
  /* Enable Discontinuos mode */
  ADC_InjectedDiscModeCmd(ADC1,ENABLE);
  
  /* ADC Injected conversions configuration */     
  ADC_InjectedSequencerLengthConfig(ADC1,2u); 

  ADC_InjectedChannelConfig(ADC1,
  pDParams_str->hIChannel, 1u, pDParams_str->b_ISamplingTime);
  ADC_InjectedChannelConfig(ADC1,
  pDParams_str->hIChannel, 2u, pDParams_str->b_ISamplingTime);
  
  /* Enable ADC1 EOC DMA */
  ADC_DMACmd(ADC1,ENABLE);
  
  R1VL1_1ShuntMotorVarsRestart(this);
  
  /*  Set AUX_TIM channel start value and enable DMA */
  if (AuxTIM == TIM3)
  {
    TIM3->CCR4 = (PWM_PERIOD >> 1) - pDParams_str->hTbefore;
    TIM_DMACmd(TIM3, TIM_DMA_CC4, ENABLE);
  }
  else
  {
    TIM4->CCR3 = (PWM_PERIOD >> 1) - pDParams_str->hTbefore;
    TIM_DMACmd(TIM4, TIM_DMA_CC3, ENABLE);
  }
  
  /* Neglect first JEOC */
  ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_None);
  ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);    
  ADC_SoftwareStartInjectedConvCmd(ADC1, ENABLE);
  while (ADC_GetFlagStatus(ADC1,ADC_FLAG_JEOC)==RESET)
  {
  }
  ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);
  
  /* Disabling the Injectec conversion for ADC1*/
  ADC_ExternalTrigInjectedConvCmd(ADC1,DISABLE);
  
  /* Select the Injected conversion trigger */
  if (AuxTIM == TIM3)
  {
    ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T3_CC4);
  }
  else
  {
    ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T4_TRGO);
  }
  
  ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);
  
  R1VL1_StartTimers();
}

/**
* @brief  It initializes TIMx and TIMx_2 peripheral for PWM generation, 
          active vector insertion and adc triggering.
* @param  TIMx Timer to be initialized
* @param  TIMx_2 Auxiliary timer to be initialized used for adc triggering
* @param  this related object of class CPWMC
* @retval none
*/
static void R1VL1_TIMxInit(TIM_TypeDef* TIMx, TIM_TypeDef* TIMx_2, CPWMC this)
{
  TIM_TimeBaseInitTypeDef TIMx_TimeBaseStructure;
  TIM_OCInitTypeDef TIMx_OCInitStructure;
  TIM_BDTRInitTypeDef TIMx_BDTRInitStructure;
  pDParams_t pDParams_str;
  
  pDParams_str =  ((_DCR1VL1_PWMC)(((_CPWMC) this)->DerivedClass))->pDParams_str;
  
  /* TIMx Peripheral Configuration -------------------------------------------*/
  /* TIMx Registers reset */
  TIM_DeInit(TIMx);
  TIM_DeInit(TIMx_2);
  TIM_TimeBaseStructInit(&TIMx_TimeBaseStructure);
  /* Time Base configuration */
  TIMx_TimeBaseStructure.TIM_Prescaler = (uint16_t)(pDParams_str->bTim_Clock_Divider) - 1u;
  TIMx_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned3;
  TIMx_TimeBaseStructure.TIM_Period = PWM_PERIOD;
  TIMx_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV2;
  TIMx_TimeBaseStructure.TIM_RepetitionCounter = pDParams_str->bRepetitionCounter;
  TIM_TimeBaseInit(TIMx, &TIMx_TimeBaseStructure);
  TIMx_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIMx_TimeBaseStructure.TIM_Period = (PWM_PERIOD * 2u) - 1u;
  TIM_TimeBaseInit(TIMx_2, &TIMx_TimeBaseStructure);
    
  /* Channel 1, 2,3 Configuration in PWM mode */
  TIM_OCStructInit(&TIMx_OCInitStructure);  
  TIMx_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
  TIMx_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
  TIMx_OCInitStructure.TIM_Pulse = 0x0u; /* dummy value */
  
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
    TIMx_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIMx_OCInitStructure.TIM_OCNPolarity = pDParams_str->hCh2NPolarity; 
    TIMx_OCInitStructure.TIM_OCNIdleState = pDParams_str->hCh2NIdleState;         
  }
  else
  {
    TIMx_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  }
  TIM_OC2Init(TIMx, &TIMx_OCInitStructure); 
    
  /* Channel 3 */
  TIMx_OCInitStructure.TIM_OCPolarity = pDParams_str->hCh3Polarity;      
  TIMx_OCInitStructure.TIM_OCIdleState = pDParams_str->hCh3IdleState; 
  if ((pDParams_str->LowSideOutputs)== LS_PWM_TIMER)
  {
    TIMx_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIMx_OCInitStructure.TIM_OCNPolarity = pDParams_str->hCh3NPolarity; 
    TIMx_OCInitStructure.TIM_OCNIdleState = pDParams_str->hCh3NIdleState;         
  }
  else
  {
    TIMx_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  }
  TIM_OC3Init(TIMx, &TIMx_OCInitStructure);   
  
  /* Channel 4 Configuration in PWM mode */
  TIMx_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;  
  TIMx_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable; 
  TIMx_OCInitStructure.TIM_Pulse = PWM_PERIOD-pDParams_str->hHTMin;
  TIMx_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
  TIMx_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM_OC4Init(TIMx, &TIMx_OCInitStructure);
    
  /* Dead Time */
  TIM_BDTRStructInit(&TIMx_BDTRInitStructure);
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
  
  /* Disable update interrupt */
  TIM_ITConfig(TIMx, TIM_IT_Update, DISABLE);
  
  TIM_SelectOutputTrigger(TIMx, TIM_TRGOSource_Update);
  TIM_SelectOutputTrigger(TIMx_2, TIM_TRGOSource_OC3Ref); /* Actualy used when AUX = TIM4 */
  
  /* TIMx_2 channel Init */
  TIMx_OCInitStructure.TIM_Pulse = PWM_PERIOD >> 2u - pDParams_str->hTMin - pDParams_str->hTbefore;
  if (TIMx_2 == TIM3)
  {
    TIMx_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; /* Is not possible to disable it */
    TIM_OC4Init(TIMx_2, &TIMx_OCInitStructure); 
  }
  else
  {
    TIMx_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable; /* Enable here for sampling point debug */
    TIM_OC3Init(TIMx_2, &TIMx_OCInitStructure); 
  }
  
  TIM_SelectInputTrigger(TIMx,TIM_TS_ITR1);
  TIM_SelectSlaveMode(TIMx,TIM_SlaveMode_Trigger);
  
  TIM_SelectInputTrigger(TIMx_2,TIM_TS_ITR1);
  TIM_SelectSlaveMode(TIMx_2,TIM_SlaveMode_Trigger);
  
  /* Prepare timer for synchronization */
  TIM_GenerateEvent(TIMx,TIM_EventSource_Update);
  TIM_GenerateEvent(TIMx_2,TIM_EventSource_Update);
}

/**
* @brief  It perform the start of all the timers needed by the control. 
          Each timer must be already prepared to be started by temporary timer.
          It utilizes TIM2 as temporary timer to achieve synchronization.
          Each timer must be in frozen state with CNT, ARR, REP RATE and trigger
          correctly set (these setting are usually performed in the Init method
          according the configuration)
* @retval none
*/
static void R1VL1_StartTimers(void)
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
* @brief  It stores into 'this' object variables the voltage present on the  
*         current feedback analog channel when no current is flowin into the
*         motor
* @param  this related object of class CPWMC
* @retval none
*/
static void R1VL1_CurrentReadingCalibration(CPWMC this)
{
  ADConv_t ADConv_struct;
  uint8_t bIndex = 0u;
  uint32_t wPhaseOffset = 0u;
  pDVars_t pDVars_str;
  pDParams_t pDParams_str;
  
  pDVars_str =   &(((_DCR1VL1_PWMC)(((_CPWMC) this)->DerivedClass))->DVars_str);
  pDParams_str =  ((_DCR1VL1_PWMC)(((_CPWMC) this)->DerivedClass))->pDParams_str;
    
  /* Set the CALIB flags to indicate the ADC calibartion phase*/
  pDVars_str->hFlags |= CALIB;
  
  ADConv_struct.Channel = pDParams_str->hIChannel;
  ADConv_struct.SamplTime = pDParams_str->b_ISamplingTime;
  
  R1VL1_ADC_SetSamplingTime(this,ADConv_struct);
  
  /* ADC Channel used for current reading are read 
  in order to get zero currents ADC values*/   
  while (bIndex< NB_CONVERSIONS)
  {
    ADC1->SQR3 = pDParams_str->hIChannel;
        
    /* Reset DMA1_CH1 TC Flag */
    DMA_ClearFlag(DMA1_FLAG_TC1);
          
    /* It starts software triggered regular conversion
    through bit banding access. It is equivalent to 
    ADC1->CR2 |= EXTTRIG_SWSTART_Set;    */
    *(uint32_t *)(ADC1_CR2_EXTTRIG_SWSTART_BB)=(uint32_t)(0x1u);
    
    /* Wait until end of regular conversion */
    while (DMA_GetFlagStatus(DMA1_FLAG_TC1)==RESET)
    {}    
        
    wPhaseOffset += (pDVars_str->hRegConv);
    bIndex++;
  }
  
  pDVars_str->hPhaseOffset = (uint16_t)(wPhaseOffset/NB_CONVERSIONS);
  
  /* Reset the CALIB flags to indicate the end of ADC calibartion phase*/
  pDVars_str->hFlags &= (~CALIB);
  
}

/**
* @brief  First initialization of class members
* @param  this related object of class CPWMC
* @retval none
*/
static void R1VL1_1ShuntMotorVarsInit(CPWMC this)
{
  pDVars_t pDVars_str;
  pDParams_t pDParams_str;
  
  pDVars_str =   &(((_DCR1VL1_PWMC)(((_CPWMC) this)->DerivedClass))->DVars_str);
  pDParams_str =  ((_DCR1VL1_PWMC)(((_CPWMC) this)->DerivedClass))->pDParams_str;
  
  /* Init motor vars */
  pDVars_str->hPhaseOffset=0u;
  pDVars_str->bInverted_pwm=INVERT_NONE;
  pDVars_str->bInverted_pwm_new=INVERT_NONE;
  pDVars_str->hFlags &= (~STBD3);
  pDVars_str->hFlags &= (~DSTEN);
  
  /* After reset value of DMA buffers */
  pDVars_str->hDmaBuff[0] = PWM_PERIOD + 1u;
  pDVars_str->hDmaBuff[1] = PWM_PERIOD >> 1;
    
  /* After reset value of dvDutyValues */
  CLASS_VARS.hCntPhA = PWM_PERIOD >> 1;
  CLASS_VARS.hCntPhB = PWM_PERIOD >> 1;
  CLASS_VARS.hCntPhC = PWM_PERIOD >> 1;
  
  /* Default value of DutyValues */
  pDVars_str->hCntSmp1 = (PWM_PERIOD >> 1) - pDParams_str->hTbefore;
  pDVars_str->hCntSmp2 = (PWM_PERIOD >> 1) + pDParams_str->hTafter;
  
  /* Default value of sampling point */
  pDVars_str->hCCDmaBuffCh4[0] = pDVars_str->hCntSmp2; /* Second point */
  pDVars_str->hCCDmaBuffCh4[1] = (PWM_PERIOD * 2u) - 1u;         /* Update */
  pDVars_str->hCCDmaBuffCh4[2] = pDVars_str->hCntSmp1; /* First point */
  
  TIM_DMACmd(TIM1, TIM_DMA_CC4, DISABLE);
}

/**
* @brief  Initialization of class members after each motor start
* @param  this related object of class CPWMC
* @retval none
*/
static void R1VL1_1ShuntMotorVarsRestart(CPWMC this)
{
  pDVars_t pDVars_str;
  pDParams_t pDParams_str;
  
  pDVars_str =   &(((_DCR1VL1_PWMC)(((_CPWMC) this)->DerivedClass))->DVars_str);
  pDParams_str =  ((_DCR1VL1_PWMC)(((_CPWMC) this)->DerivedClass))->pDParams_str;
  
  /* Default value of DutyValues */
  pDVars_str->hCntSmp1 = (PWM_PERIOD >> 1) - pDParams_str->hTbefore;
  pDVars_str->hCntSmp2 = (PWM_PERIOD >> 1) + pDParams_str->hTafter;
  
  /* Default value of sampling point */
  pDVars_str->hCCDmaBuffCh4[0] = pDVars_str->hCntSmp2; /* Second point */
  pDVars_str->hCCDmaBuffCh4[2] = pDVars_str->hCntSmp1; /* First point */
  
  /* After start value of DMA buffers */
  pDVars_str->hDmaBuff[0] = PWM_PERIOD + 1u;
  pDVars_str->hDmaBuff[1]= PWM_PERIOD >> 1;
  
  /* After start value of dvDutyValues */
  CLASS_VARS.hCntPhA = PWM_PERIOD >> 1;
  CLASS_VARS.hCntPhB = PWM_PERIOD >> 1;
  CLASS_VARS.hCntPhC = PWM_PERIOD >> 1;
  
  /* Set the default previous value of Phase A,B,C current */
  pDVars_str->hCurrAOld=0;
  pDVars_str->hCurrBOld=0;
  pDVars_str->hCurrCOld=0;
    
  TIM_DMACmd(TIM1, TIM_DMA_CC4, DISABLE);
}

/**
* @brief  It computes and return latest converted motor phase currents motor
* @param  this related object of class CPWMC
* @retval Curr_Components Ia and Ib current in Curr_Components format
*/
static void R1VL1_GetPhaseCurrents(CPWMC this,Curr_Components* pStator_Currents)
{  
  int32_t wAux;
  int16_t hCurrA = 0, hCurrB = 0, hCurrC = 0;
  uint8_t bCurrASamp = 0u, bCurrBSamp = 0u, bCurrCSamp = 0u;
  pDVars_t pDVars_str;
  
  pDVars_str =   &(((_DCR1VL1_PWMC)(((_CPWMC) this)->DerivedClass))->DVars_str);
  
  /* Disabling the Injectec conversion for ADCx after EOC*/
  /* Stdlib replaced: ADC_ExternalTrigInjectedConvCmd(ADC1,DISABLE);*/
  ADC1->CR2 &= CR2_JEXTTRIG_Reset;
  
  /* Reset the bSOFOC flags to indicate the start of FOC algorithm*/
  pDVars_str->hFlags &= (~SOFOC);
  
  /* First sampling point */
  wAux = (int32_t)(ADC1->JDR2);
  wAux *= 2;
  wAux -= (int32_t)(pDVars_str->hPhaseOffset);
  
  /* Check saturation */
  if (wAux > S16_MIN)
  {
    if (wAux < S16_MAX)
    {
    }
    else
    {
      wAux = S16_MAX;
    }
  }
  else
  {
    wAux = S16_MIN;
  }  
  
  switch (pDVars_str->sampCur1)
  {
  case SAMP_IA:
    hCurrA = (int16_t)(wAux);
    bCurrASamp = 1u;
    break;
  case SAMP_IB:
    hCurrB = (int16_t)(wAux);
    bCurrBSamp = 1u;
    break;
  case SAMP_IC:
    hCurrC = (int16_t)(wAux);
    bCurrCSamp = 1u;
    break;
  case SAMP_NIA:
    wAux = -wAux;
    hCurrA = (int16_t)(wAux);
    bCurrASamp = 1u;
    break;
  case SAMP_NIB:
    wAux = -wAux;
    hCurrB = (int16_t)(wAux);
    bCurrBSamp = 1u;
    break;
  case SAMP_NIC:
    wAux = -wAux;
    hCurrC = (int16_t)(wAux);
    bCurrCSamp = 1u;
    break;
  case SAMP_OLDA:
    hCurrA = pDVars_str->hCurrAOld;
    bCurrASamp = 1u;
    break;
  case SAMP_OLDB:
    hCurrB = pDVars_str->hCurrBOld;
    bCurrBSamp = 1u;
    break;
  default:
    break;
  }
  
  /* Second sampling point */
  wAux = (int32_t)(ADC1->JDR1);
  wAux *= 2;
  wAux -= (int32_t)(pDVars_str->hPhaseOffset);
  
  /* Check saturation */
  if (wAux > S16_MIN)
  {
    if (wAux < S16_MAX)
    {
    }
    else
    {
      wAux = S16_MAX;
    }
  }
  else
  {
    wAux = S16_MIN;
  }
  
  switch (pDVars_str->sampCur2)
  {
  case SAMP_IA:
    hCurrA = (int16_t)(wAux);
    bCurrASamp = 1u;
    break;
  case SAMP_IB:
    hCurrB = (int16_t)(wAux);
    bCurrBSamp = 1u;
    break;
  case SAMP_IC:
    hCurrC = (int16_t)(wAux);
    bCurrCSamp = 1u;
    break;
  case SAMP_NIA:
    wAux = -wAux; 
    hCurrA = (int16_t)(wAux);
    bCurrASamp = 1u;
    break;
  case SAMP_NIB:
    wAux = -wAux; 
    hCurrB = (int16_t)(wAux);
    bCurrBSamp = 1u;
    break;
  case SAMP_NIC:
    wAux = -wAux; 
    hCurrC = (int16_t)(wAux);
    bCurrCSamp = 1u;
    break;
  default:
    break;
  }
    
  /* Computation of the third value */
  if (bCurrASamp == 0u)
  {
    wAux = -((int32_t)(hCurrB)) -((int32_t)(hCurrC));
    
    /* Check saturation */
    if (wAux > S16_MIN)
    {
      if (wAux < S16_MAX)
      {
      }
      else
      {
        wAux = S16_MAX;
      }
    }
    else
    {
      wAux = S16_MIN;
    }  
    
    hCurrA = (int16_t)wAux; 
  }
  if (bCurrBSamp == 0u)
  {
    wAux = -((int32_t)(hCurrA)) -((int32_t)(hCurrC));
    
    /* Check saturation */
    if (wAux > S16_MIN)
    {
      if (wAux < S16_MAX)
      {
      }
      else
      {
        wAux = S16_MAX;
      }
    }
    else
    {
      wAux = S16_MIN;
    }  
    
    hCurrB = (int16_t)wAux;
  }
  if (bCurrCSamp == 0u)
  {
    wAux = -((int32_t)(hCurrA)) -((int32_t)(hCurrB));
    
    /* Check saturation */
    if (wAux > S16_MIN)
    {
      if (wAux < S16_MAX)
      {
      }
      else
      {
        wAux = S16_MAX;
      }
    }
    else
    {
      wAux = S16_MIN;
    }  
    
    hCurrC = (int16_t)wAux;
  }
  
  /* hCurrA, hCurrB, hCurrC values are the sampled values */
    
  pDVars_str->hCurrAOld = hCurrA;
  pDVars_str->hCurrBOld = hCurrB;
  pDVars_str->hCurrCOld = hCurrC;
  
  pStator_Currents->qI_Component1 = hCurrA;
  pStator_Currents->qI_Component2 = hCurrB;
}

/**
* @brief  It turns on low sides switches. This function is intended to be 
*         used for charging boot capacitors of driving section. It has to be 
*         called each motor start-up when using high voltage drivers
* @param  this related object of class CPWMC
* @retval none
*/
static void R1VL1_TurnOnLowSides(CPWMC this)
{
  pDParams_t pLocalDParams = DCLASS_PARAMS;
  TIM1->CCR1 = 0u;
  TIM1->CCR2 = 0u;
  TIM1->CCR3 = 0u;
  
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
* @brief  This function enables the update event and the single shunt distortion
* @param  this related object of class CPWMC
* @retval none
*/
static void R1VL1_SwitchOnPWM(CPWMC this)
{
  pDVars_t pDVars_str;
  pDParams_t pLocalDParams = DCLASS_PARAMS;
  
  pDVars_str =   &(((_DCR1VL1_PWMC)(((_CPWMC) this)->DerivedClass))->DVars_str);
  
  /* Main PWM Output Enable */
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
  if ((pLocalDParams->LowSideOutputs)== ES_GPIO)
  {
    GPIO_WriteBit(pLocalDParams->hCh1NPort, pLocalDParams->hCh1NPin, Bit_SET);
    GPIO_WriteBit(pLocalDParams->hCh2NPort, pLocalDParams->hCh2NPin, Bit_SET);
    GPIO_WriteBit(pLocalDParams->hCh3NPort, pLocalDParams->hCh3NPin, Bit_SET);
  }
  
  /* Enable UPDATE ISR */
  /* Clear Update Flag */
  TIM_ClearFlag(TIM1, TIM_FLAG_Update);
  TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
  
  /* Enabling distortion for single shunt */
  pDVars_str->hFlags |= DSTEN;
  return; 
}

/**
* @brief  It disables PWM generation on the proper Timer peripheral acting on 
*         MOE bit, disables the single shunt distortion and reset the TIM status
* @param  this related object of class CPWMC
* @retval none
*/
static void R1VL1_SwitchOffPWM(CPWMC this)
{
  pDVars_t pDVars_str;
  pDParams_t pLocalDParams = DCLASS_PARAMS;
  
  pDVars_str =   &(((_DCR1VL1_PWMC)(((_CPWMC) this)->DerivedClass))->DVars_str);
  
  /* Main PWM Output Disable */
  TIM_CtrlPWMOutputs(TIM1, DISABLE);
  if ((pLocalDParams->LowSideOutputs)== ES_GPIO)
  {
    GPIO_WriteBit(pLocalDParams->hCh1NPort, pLocalDParams->hCh1NPin, Bit_RESET);
    GPIO_WriteBit(pLocalDParams->hCh2NPort, pLocalDParams->hCh2NPin, Bit_RESET);
    GPIO_WriteBit(pLocalDParams->hCh3NPort, pLocalDParams->hCh3NPin, Bit_RESET);
  }
  
  /* Disable UPDATE ISR */
  TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);
    
  /* Disabling distortion for single */
  pDVars_str->hFlags &= (~DSTEN);

  while (TIM_GetFlagStatus(TIM1,TIM_FLAG_Update)==RESET)
  {}
  /* Disabling all DMA previous setting */
  TIM_DMACmd(TIM1, TIM_DMA_CC4, DISABLE);  
  
  /* Set all duty to 50% */
  TIM1->CCR1 = PWM_PERIOD >> 1;
  TIM1->CCR2 = PWM_PERIOD >> 1;
  TIM1->CCR3 = PWM_PERIOD >> 1;    
    
  return; 
}

/**
* @brief  Implementation of the single shunt algorithm to setup the 
*         TIM1 register and DMA buffers values for the next PWM period.
* @param  this related object of class CPWMC
* @retval uint16_t It returns MC_FOC_DURATION if the TIMx update occurs 
          before the end of FOC algorithm else returns MC_NO_ERROR
*/
static uint16_t R1VL1_CalcDutyCycles(CPWMC this)
{
  int16_t hDeltaDuty_0;
  int16_t hDeltaDuty_1;
  uint16_t hDutyV_0 = 0u;
  uint16_t hDutyV_1 = 0u;
  uint16_t hDutyV_2 = 0u;
  uint8_t bSector;
  uint8_t bStatorFluxPos;
  uint16_t hAux;
  pDVars_t pDVars_str;
  pDParams_t pDParams_str;
    
  pDVars_str =   &(((_DCR1VL1_PWMC)(((_CPWMC) this)->DerivedClass))->DVars_str);
  pDParams_str =  ((_DCR1VL1_PWMC)(((_CPWMC) this)->DerivedClass))->pDParams_str;
  
  bSector = (uint8_t)(((_CPWMC)this)->Vars_str.hSector);
  
  if ((pDVars_str->hFlags & DSTEN) != 0u)
  { 
    switch (bSector)
    {
    case SECTOR_1:
      hDutyV_2 = CLASS_VARS.hCntPhA;
      hDutyV_1 = CLASS_VARS.hCntPhB;
      hDutyV_0 = CLASS_VARS.hCntPhC;
      break;
    case SECTOR_2:
      hDutyV_2 = CLASS_VARS.hCntPhB;
      hDutyV_1 = CLASS_VARS.hCntPhA;
      hDutyV_0 = CLASS_VARS.hCntPhC;
      break;
    case SECTOR_3:
      hDutyV_2 = CLASS_VARS.hCntPhB;
      hDutyV_1 = CLASS_VARS.hCntPhC;
      hDutyV_0 = CLASS_VARS.hCntPhA;
      break;
    case SECTOR_4:
      hDutyV_2 = CLASS_VARS.hCntPhC;
      hDutyV_1 = CLASS_VARS.hCntPhB;
      hDutyV_0 = CLASS_VARS.hCntPhA;
      break;
    case SECTOR_5:
      hDutyV_2 = CLASS_VARS.hCntPhC;
      hDutyV_1 = CLASS_VARS.hCntPhA;
      hDutyV_0 = CLASS_VARS.hCntPhB;
      break;
    case SECTOR_6:
      hDutyV_2 = CLASS_VARS.hCntPhA;
      hDutyV_1 = CLASS_VARS.hCntPhC;
      hDutyV_0 = CLASS_VARS.hCntPhB;
      break;
    default:
      break;
    }
    
    /* Compute delta duty */
    hDeltaDuty_0 = (int16_t)(hDutyV_1) - (int16_t)(hDutyV_0);
    hDeltaDuty_1 = (int16_t)(hDutyV_2) - (int16_t)(hDutyV_1);
    
    /* Check region */
    if ((uint16_t)hDeltaDuty_0<=pDParams_str->hTMin)
    {
      if ((uint16_t)hDeltaDuty_1<=pDParams_str->hTMin)
      {
        bStatorFluxPos = BOUNDARY_3;
      }
      else
      {
        bStatorFluxPos = BOUNDARY_2;
      }
    } 
    else 
    {
      if ((uint16_t)hDeltaDuty_1>pDParams_str->hTMin)
      {
        bStatorFluxPos = REGULAR;
      }
      else
      {
        bStatorFluxPos = BOUNDARY_1;
      }
    }
        
    if (bStatorFluxPos == REGULAR)
    {
      pDVars_str->bInverted_pwm_new = INVERT_NONE;
    }
    else if (bStatorFluxPos == BOUNDARY_1) /* Adjust the lower */
    {
      switch (bSector)
      {
      case SECTOR_5:
      case SECTOR_6:
        if (CLASS_VARS.hCntPhA - pDParams_str->hHTMin - hDutyV_0 > pDParams_str->hTMin)
        {
          pDVars_str->bInverted_pwm_new = INVERT_A;
          CLASS_VARS.hCntPhA -=pDParams_str->hHTMin;
          if (CLASS_VARS.hCntPhA < hDutyV_1)
          {
            hDutyV_1 = CLASS_VARS.hCntPhA;
          }
        }
        else
        {
          bStatorFluxPos = BOUNDARY_3;
          if ((pDVars_str->hFlags & STBD3) == 0u)
          {
            pDVars_str->bInverted_pwm_new = INVERT_A;
            CLASS_VARS.hCntPhA -=pDParams_str->hHTMin;
            pDVars_str->hFlags |= STBD3;
          } 
          else
          {
            pDVars_str->bInverted_pwm_new = INVERT_B;
            CLASS_VARS.hCntPhB -=pDParams_str->hHTMin;
            pDVars_str->hFlags &= (~STBD3);
          }
        }
        break;
      case SECTOR_2:
      case SECTOR_1:
        if (CLASS_VARS.hCntPhB - pDParams_str->hHTMin - hDutyV_0 > pDParams_str->hTMin)
        {
          pDVars_str->bInverted_pwm_new = INVERT_B;
          CLASS_VARS.hCntPhB -=pDParams_str->hHTMin;
          if (CLASS_VARS.hCntPhB < hDutyV_1)
          {
            hDutyV_1 = CLASS_VARS.hCntPhB;
          }
        }
        else
        {
          bStatorFluxPos = BOUNDARY_3;
          if ((pDVars_str->hFlags & STBD3) == 0u)
          {
            pDVars_str->bInverted_pwm_new = INVERT_A;
            CLASS_VARS.hCntPhA -=pDParams_str->hHTMin;
            pDVars_str->hFlags |= STBD3;
          } 
          else
          {
            pDVars_str->bInverted_pwm_new = INVERT_B;
            CLASS_VARS.hCntPhB -=pDParams_str->hHTMin;
            pDVars_str->hFlags &= (~STBD3);
          }
        }
        break;
      case SECTOR_4:
      case SECTOR_3:
        if (CLASS_VARS.hCntPhC - pDParams_str->hHTMin - hDutyV_0 > pDParams_str->hTMin)
        {
          pDVars_str->bInverted_pwm_new = INVERT_C;
          CLASS_VARS.hCntPhC -=pDParams_str->hHTMin;
          if (CLASS_VARS.hCntPhC < hDutyV_1)
          {
            hDutyV_1 = CLASS_VARS.hCntPhC;
          }
        }
        else
        {
          bStatorFluxPos = BOUNDARY_3;
          if ((pDVars_str->hFlags & STBD3) == 0u)
          {
            pDVars_str->bInverted_pwm_new = INVERT_A;
            CLASS_VARS.hCntPhA -=pDParams_str->hHTMin;
            pDVars_str->hFlags |= STBD3;
          } 
          else
          {
            pDVars_str->bInverted_pwm_new = INVERT_B;
            CLASS_VARS.hCntPhB -=pDParams_str->hHTMin;
            pDVars_str->hFlags &= (~STBD3);
          }
        }
        break;
      default:
        break;
      }
    }
    else if (bStatorFluxPos == BOUNDARY_2) /* Adjust the middler */
    {
      switch (bSector)
      {
      case SECTOR_4:
      case SECTOR_5: /* Invert B */
        pDVars_str->bInverted_pwm_new = INVERT_B;
        CLASS_VARS.hCntPhB -=pDParams_str->hHTMin;
        if (CLASS_VARS.hCntPhB > 0xEFFFu)
        {
          CLASS_VARS.hCntPhB = 0u;
        }
        break;
      case SECTOR_2:
      case SECTOR_3: /* Invert A */
        pDVars_str->bInverted_pwm_new = INVERT_A;
        CLASS_VARS.hCntPhA -=pDParams_str->hHTMin;
        if (CLASS_VARS.hCntPhA > 0xEFFFu)
        {
          CLASS_VARS.hCntPhA = 0u;
        }
        break;
      case SECTOR_6:
      case SECTOR_1: /* Invert C */
        pDVars_str->bInverted_pwm_new = INVERT_C;
        CLASS_VARS.hCntPhC -=pDParams_str->hHTMin;
        if (CLASS_VARS.hCntPhC > 0xEFFFu)
        {
          CLASS_VARS.hCntPhC = 0u;
        }
        break;
      default:
        break;
      }
    }
    else
    {
      if ((pDVars_str->hFlags & STBD3) == 0u)
      {
        pDVars_str->bInverted_pwm_new = INVERT_A;
        CLASS_VARS.hCntPhA -=pDParams_str->hHTMin;
        pDVars_str->hFlags |= STBD3;
      } 
      else
      {
        pDVars_str->bInverted_pwm_new = INVERT_B;
        CLASS_VARS.hCntPhB -=pDParams_str->hHTMin;
        pDVars_str->hFlags &= (~STBD3);
      }
    }
        
    if (bStatorFluxPos == REGULAR) /* Regular zone */
    {
      /* First point */
      if ((hDutyV_1 - hDutyV_0 - pDParams_str->hDeadTime)> pDParams_str->hMaxTrTs)
      {
        pDVars_str->hCntSmp1 = hDutyV_0 + hDutyV_1 + pDParams_str->hDeadTime;
        pDVars_str->hCntSmp1 >>= 1;
      }
      else
      {
        pDVars_str->hCntSmp1 = hDutyV_1 - pDParams_str->hTbefore;
      }
      /* Second point */
      if ((hDutyV_2 - hDutyV_1 - pDParams_str->hDeadTime)> pDParams_str->hMaxTrTs)
      {
        pDVars_str->hCntSmp2 = hDutyV_1 + hDutyV_2 + pDParams_str->hDeadTime;
        pDVars_str->hCntSmp2 >>= 1;
      }
      else
      {
        pDVars_str->hCntSmp2 = hDutyV_2 - pDParams_str->hTbefore;
      }
    }
    
    if (bStatorFluxPos == BOUNDARY_1) /* Two small, one big */
    {      
      /* First point */
      if ((hDutyV_1 - hDutyV_0 - pDParams_str->hDeadTime)> pDParams_str->hMaxTrTs)
      {
        pDVars_str->hCntSmp1 = hDutyV_0 + hDutyV_1 + pDParams_str->hDeadTime;
        pDVars_str->hCntSmp1 >>= 1;
      }
      else
      {
        pDVars_str->hCntSmp1 = hDutyV_1 - pDParams_str->hTbefore;
      }
      /* Second point */
      pDVars_str->hCntSmp2 = PWM_PERIOD + pDParams_str->hHTMin - pDParams_str->hTSample;
    }
    
    if (bStatorFluxPos == BOUNDARY_2) /* Two big, one small */
    {
      /* First point */
      if ((hDutyV_2 - hDutyV_1 - pDParams_str->hDeadTime)>= pDParams_str->hMaxTrTs)
      {
        pDVars_str->hCntSmp1 = hDutyV_1 + hDutyV_2 + pDParams_str->hDeadTime;
        pDVars_str->hCntSmp1 >>= 1;
      }
      else
      {
        pDVars_str->hCntSmp1 = hDutyV_2 - pDParams_str->hTbefore;
      }
      /* Second point */
      pDVars_str->hCntSmp2 = PWM_PERIOD + pDParams_str->hHTMin - pDParams_str->hTSample;
    }
    
    if (bStatorFluxPos == BOUNDARY_3)  
    {
      /* First point */
      pDVars_str->hCntSmp1 = hDutyV_0-pDParams_str->hTbefore; /* Dummy trigger */
      /* Second point */
      pDVars_str->hCntSmp2 = PWM_PERIOD + pDParams_str->hHTMin - pDParams_str->hTSample;
    }
  }
  else
  {
    pDVars_str->bInverted_pwm_new = INVERT_NONE;
    bStatorFluxPos = REGULAR;
  }
    
  /* Update Timer Ch 1,2,3 (These value are required before update event) */
    
  pDVars_str->hFlags |= EOFOC;
  /* Check if DMA transition has been completed */
  if (pDVars_str->bDMACur == 0u)
  {    
    /* Preload Enable */
    TIM1->CCMR1 |= CCMR1_PRELOAD_ENABLE_MASK;
    TIM1->CCMR2 |= CCMR2_PRELOAD_ENABLE_MASK;
    
    TIM1->CCR1 = CLASS_VARS.hCntPhA;
    TIM1->CCR2 = CLASS_VARS.hCntPhB;
    TIM1->CCR3 = CLASS_VARS.hCntPhC;

    /* Update ADC Trigger DMA buffer */    
    pDVars_str->hCCDmaBuffCh4[0] = pDVars_str->hCntSmp2; /* Second point */
    pDVars_str->hCCDmaBuffCh4[2] = pDVars_str->hCntSmp1; /* First point */
  }
    
  /* Limit for update event */
  
  /* Check the status of bSOFOC flags if is set the next update event has been 
  occurred so an error will be reported*/
  if ((pDVars_str->hFlags & SOFOC) != 0u)
  {
    hAux = MC_FOC_DURATION;
  }
  else
  {
    hAux = MC_NO_ERROR;
  }
  
  /* The following instruction can be executed after Update handler 
     before the get phase current (Second EOC) */
      
  /* Set the current sampled */
   if (bStatorFluxPos == REGULAR) /* Regual zone */
  {
    pDVars_str->sampCur1 = REGULAR_SAMP_CUR1[bSector];
    pDVars_str->sampCur2 = REGULAR_SAMP_CUR2[bSector];
  }
  
  if (bStatorFluxPos == BOUNDARY_1) /* Two small, one big */
  {
    pDVars_str->sampCur1 = REGULAR_SAMP_CUR1[bSector];
    pDVars_str->sampCur2 = BOUNDR1_SAMP_CUR2[bSector];
  }
  
  if (bStatorFluxPos == BOUNDARY_2) /* Two big, one small */
  {
    pDVars_str->sampCur1 = BOUNDR2_SAMP_CUR1[bSector];
    pDVars_str->sampCur2 = BOUNDR2_SAMP_CUR2[bSector];
  }
  
  if (bStatorFluxPos == BOUNDARY_3)  
  {
    if (pDVars_str->bInverted_pwm_new == INVERT_A)
    {
      pDVars_str->sampCur1 = SAMP_OLDB;
      pDVars_str->sampCur2 = SAMP_IA;
    }
    if (pDVars_str->bInverted_pwm_new == INVERT_B)
    {
      pDVars_str->sampCur1 = SAMP_OLDA;
      pDVars_str->sampCur2 = SAMP_IB;
    }
  }
    
  /* Limit for the Get Phase current (Second EOC Handler) */
      
  return (hAux);
}

/**
  * @brief  R1_VL1 implement MC IRQ function TIMER Update and DMA TC
  * @param  this related object
  * @param  flag used to indicate which IRQ has been occurred
  *			0 Means TIM1 Update IRQ occurred
  *			1 Not used
  *			2 Means DAC TC IRQ occurred
  * @retval void* It returns always MC_NULL
  */
static void* R1VL1_IRQHandler(void* this, unsigned char flag)
{   
  uint8_t bInverted_pwm_new;
  pDVars_t pDVars_str;
  
  pDVars_str =   &(((_DCR1VL1_PWMC)(((_CPWMC) this)->DerivedClass))->DVars_str);
  
  switch (flag) /* Case 1 is not used */
  {
  case 0: /* TIM1 Update IRQ */
    {      
      /* Critical point start */
      
      /* Enabling the Injectec conversion for ADCx*/
      /* Stdlib replaced: ADC_ExternalTrigInjectedConvCmd(ADC1,ENABLE); */
      ADC1->CR2 |= CR2_JEXTTRIG_Set;
      
      /* Critical point stop */
      
      /* TMP var to speedup the execution */
      bInverted_pwm_new = pDVars_str->bInverted_pwm_new;
      
      if (bInverted_pwm_new != pDVars_str->bInverted_pwm)  
      {
        /* Set the DMA destination */
        switch (bInverted_pwm_new)
        {
        case INVERT_A:
          DMA1_Channel4->CPAR = TIM1_CCR1_Address;
          /*Stdlib replaced: TIM_DMACmd(TIM1, TIM_DMA_CC4, ENABLE);*/
          TIM1->DIER |= TIM_DMA_CC4;
          break;
          
        case INVERT_B:
          DMA1_Channel4->CPAR = TIM1_CCR2_Address;
          /*Stdlib replaced: TIM_DMACmd(TIM1, TIM_DMA_CC4, ENABLE);*/
          TIM1->DIER |= TIM_DMA_CC4;
          break;
          
        case INVERT_C:
          DMA1_Channel4->CPAR = TIM1_CCR3_Address;
          /*Stdlib replaced: TIM_DMACmd(TIM1, TIM_DMA_CC4, ENABLE);*/
          TIM1->DIER |= TIM_DMA_CC4;
          break;
          
        default:
          /*Stdlib replaced: TIM_DMACmd(TIM1, TIM_DMA_CC4, DISABLE);*/
          TIM1->DIER &= (u16)~TIM_DMA_CC4;
          break;
        }  
      }
      
      /* Clear of End of FOC Flags */
      pDVars_str->hFlags &= (~EOFOC);
      
      /* Preload Disable */
      TIM1->CCMR1 &= CCMR1_PRELOAD_DISABLE_MASK;
      TIM1->CCMR2 &= CCMR2_PRELOAD_DISABLE_MASK;
      
      switch (bInverted_pwm_new)
      {
      case INVERT_A:
        pDVars_str->hDmaBuff[1] = CLASS_VARS.hCntPhA;
        pDVars_str->bDMACur = pDVars_str->bDMATot;
        break;
        
      case INVERT_B:
        pDVars_str->hDmaBuff[1] = CLASS_VARS.hCntPhB;
        pDVars_str->bDMACur = pDVars_str->bDMATot;
        break;
        
      case INVERT_C:
        pDVars_str->hDmaBuff[1] = CLASS_VARS.hCntPhC;
        pDVars_str->bDMACur = pDVars_str->bDMATot;
        break;
        
      default:
        pDVars_str->bDMACur = 0u;
        break;
      }
      
      pDVars_str->bInverted_pwm = bInverted_pwm_new;      
    
      /* Set the bSOFOC flags to indicate the execution of Update IRQ*/
      pDVars_str->hFlags |= SOFOC;    
    }
    break;
  case 2: /* DMA TC IRQ */
    {
      pDVars_str->bDMACur--;
      if (pDVars_str->bDMACur == 0u)
      {
        if ((pDVars_str->hFlags & EOFOC) != 0u)
        {
          /* Preload Enable */
          TIM1->CCMR1 |= CCMR1_PRELOAD_ENABLE_MASK;
          TIM1->CCMR2 |= CCMR2_PRELOAD_ENABLE_MASK;
          
          /* Compare register update */
          TIM1->CCR1 = CLASS_VARS.hCntPhA;
          TIM1->CCR2 = CLASS_VARS.hCntPhB;
          TIM1->CCR3 = CLASS_VARS.hCntPhC;
          
          /* Update ADC Trigger DMA buffer */    
          pDVars_str->hCCDmaBuffCh4[0] = pDVars_str->hCntSmp2; /* Second point */
          pDVars_str->hCCDmaBuffCh4[2] = pDVars_str->hCntSmp1; /* First point */        
        }
      }
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
static uint16_t R1VL1_ExecRegularConv(CPWMC this, uint8_t bChannel)
{
  pDVars_t pDVars_str;
  uint32_t tmpflag = 0u;
  
  pDVars_str =  &(((_DCR1VL1_PWMC)(((_CPWMC) this)->DerivedClass))->DVars_str);
  
  if ((pDVars_str->hFlags & CALIB) != 0u)
  {
    pDVars_str->hRegConv = 0xFFFFu;
  }
  else
  {    
    ADC1->SQR3 = bChannel;
    
    /* Reset DMA1_CH1 TC Flag */
    DMA1->IFCR = DMA1_FLAG_TC1;
    
    /* It starts software triggered regular conversion
    through bit banding access. It is equivalent to 
    ADC1->CR2 |= EXTTRIG_SWSTART_Set;    */
    *(uint32_t *)(ADC1_CR2_EXTTRIG_SWSTART_BB)=(uint32_t)(0x1u);
    
    /* Wait until end of regular conversion */
    while (tmpflag == 0u)
    {
      tmpflag = (DMA1->ISR & DMA1_FLAG_TC1);
    }          
  }    
  return (pDVars_str->hRegConv);
}

/**
* @brief  It sets the specified sampling time for the specified ADC channel
*         on ADC1. It must be called once for each channel utilized by user
* @param  this related object of class CPWMC
* @param  ADConv_struct struct containing ADC channel and sampling time
* @retval none
*/
static void R1VL1_ADC_SetSamplingTime(CPWMC this, ADConv_t ADConv_struct)
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
static uint16_t R1VL1_IsOverCurrentOccurred(CPWMC this)
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

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
