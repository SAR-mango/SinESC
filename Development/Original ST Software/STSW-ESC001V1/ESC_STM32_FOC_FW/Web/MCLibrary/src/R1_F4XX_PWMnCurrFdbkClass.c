/**
  ******************************************************************************
  * @file    R1_F4XX_PWMnCurrFdbkClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private implementation of R1_F4XX_PWMnCurrFdbk IRQ      
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
#include "R1_F4XX_PWMnCurrFdbkClass.h"
#include "R1_F4XX_PWMnCurrFdbkPrivate.h"
#include "MCIRQHandlerClass.h"
#include "MCIRQHandlerPrivate.h"
#include "MCLibraryConf.h"
#include "MCLibraryISRPriorityConf.h"
#include "MC_type.h"

/* Private Defines -----------------------------------------------------------*/
#define CLASS_VARS    ((_CPWMC)this)->Vars_str
#define CLASS_PARAMS  ((_CPWMC)this)->pParams_str
#define DCLASS_VARS   ((_DCR1F4XX_PWMC)(((_CPWMC) this)->DerivedClass))->DVars_str
#define DCLASS_PARAMS ((_DCR1F4XX_PWMC)(((_CPWMC) this)->DerivedClass))->pDParams_str

#define PWM_PERIOD ((((_CPWMC) this)->pParams_str->hPWMperiod)/2u)

/* Direct address of the registers used by DMA */
#define TIM1_CCR1_Address   0x40010034u
#define TIM1_CCR2_Address   0x40010038u
#define TIM1_CCR3_Address   0x4001003Cu
#define TIM4_CCR2_Address   0x40000838u

#define TIM8_CCR1_Address   0x40010434u
#define TIM8_CCR2_Address   0x40010438u
#define TIM8_CCR3_Address   0x4001043Cu
#define TIM5_CCR4_Address   0x40000C40u

#define NB_CONVERSIONS 16u

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
#define CR2_JEXTTRIG_Set        ((uint32_t)0x00100000u)
#define CR2_JEXTTRIG_Reset      ((uint32_t)0xFFEFFFFFu)

#define TIM_DMA_ENABLED_CC1 0x0200u
#define TIM_DMA_ENABLED_CC2 0x0400u
#define TIM_DMA_ENABLED_CC3 0x0800u

#define CR2_ADON_Set                ((uint32_t)0x00000001u)

/* ADC SMPx mask */
#define SMPR1_SMP_Set              ((uint32_t) (0x00000007u))
#define SMPR2_SMP_Set              ((uint32_t) (0x00000007u))
#define CR2_EXTTRIG_SWSTART_Set     ((u32)0x00500000)

#define ADC1_CR2_SWSTART_BB 0x42240178u

#define TIMxCCER_MASK              ((uint16_t)  ~0x1555u)
#define TIMxCCER_MASK_CH123        ((uint16_t)  0x555u)
#define TIMx_CC4E_BIT              ((uint16_t)  0x1000u) 

/* Constant values -----------------------------------------------------------*/
static const uint8_t REGULAR_SAMP_CUR1[6] = {SAMP_NIC,SAMP_NIC,SAMP_NIA,SAMP_NIA,SAMP_NIB,SAMP_NIB};
static const uint8_t REGULAR_SAMP_CUR2[6] = {SAMP_IA,SAMP_IB,SAMP_IB,SAMP_IC,SAMP_IC,SAMP_IA};
static const uint8_t BOUNDR1_SAMP_CUR2[6] = {SAMP_IB,SAMP_IB,SAMP_IC,SAMP_IC,SAMP_IA,SAMP_IA};
static const uint8_t BOUNDR2_SAMP_CUR1[6] = {SAMP_IA,SAMP_IB,SAMP_IB,SAMP_IC,SAMP_IC,SAMP_IA};
static const uint8_t BOUNDR2_SAMP_CUR2[6] = {SAMP_IC,SAMP_IA,SAMP_IA,SAMP_IB,SAMP_IB,SAMP_IC};

#ifdef MC_CLASS_DYNAMIC
	#include "stdlib.h" /* Used for dynamic allocation */
#else
	_DCR1F4XX_PWMC_t R1F4XX_PWMCpool[MAX_DRV_PWMC_NUM];
	unsigned char R1F4XX_PWMC_Allocated = 0u;
#endif

static void* R1F4XX_IRQHandler(void *this, unsigned char flag);
static void R1F4XX_Init(CPWMC this);
static void R1F4XX_TIMxInit(TIM_TypeDef* TIMx, TIM_TypeDef* TIMx_2, CPWMC this);
static void R1F4XX_CurrentReadingCalibration(CPWMC this);
static void R1F4XX_GetPhaseCurrents(CPWMC this,Curr_Components* pStator_Currents);
static void R1F4XX_TurnOnLowSides(CPWMC this);
static void R1F4XX_SwitchOnPWM(CPWMC this);
static void R1F4XX_SwitchOffPWM(CPWMC this);
static void R1F4XX_1ShuntMotorVarsInit(CPWMC this);
static void R1F4XX_1ShuntMotorVarsRestart(CPWMC this);
static uint16_t R1F4XX_CalcDutyCycles(CPWMC this);
static uint16_t R1F4XX_ExecRegularConv(CPWMC this, uint8_t bChannel);
static void R1F4XX_ADC_SetSamplingTime(CPWMC this, ADConv_t ADConv_struct);
static void R1F4XX_HFCurrentsCalibration(CPWMC this,Curr_Components* pStator_Currents);
static uint16_t R1F4XX_IsOverCurrentOccurred(CPWMC this);
static uint16_t F4XX_GPIOPin2Source(uint16_t GPIO_Pin);
static void R1F4XX_RLDetectionModeEnable(CPWMC this);
static void R1F4XX_RLDetectionModeDisable(CPWMC this);
static uint16_t R1F4XX_RLDetectionModeSetDuty(CPWMC this, uint16_t hDuty);
static void R1F4XX_RLGetPhaseCurrents(CPWMC this,Curr_Components* pStator_Currents);
static void R1F4XX_RLTurnOnLowSides(CPWMC this);
static void R1F4XX_RLSwitchOnPWM(CPWMC this);
static void R1F4XX_RLSwitchOffPWM(CPWMC this);

/**
  * @brief  Creates an object of the class R1_F4XX
  * @param  pPWMnCurrFdbkParams pointer to an PWMnCurrFdbk parameters structure
  * @param  pR1_DDParams pointer to an R1_DD parameters structure
  * @retval CR1F4XX_PWMC new instance of R1_F4XX object
  */
CR1F4XX_PWMC R1F4XX_NewObject(pPWMnCurrFdbkParams_t pPWMnCurrFdbkParams, pR1_DDParams_t pR1_DDParams)
{
	_CPWMC _oPWMnCurrFdbk;
	_DCR1F4XX_PWMC _oR1_F4XX;

	_oPWMnCurrFdbk = (_CPWMC)PWMC_NewObject(pPWMnCurrFdbkParams);

	#ifdef MC_CLASS_DYNAMIC
		_oR1_F4XX = (_DCR1F4XX_PWMC)calloc(1u,sizeof(_DCR1F4XX_PWMC_t));
	#else
		if (R1F4XX_PWMC_Allocated  < MAX_DRV_PWMC_NUM)
		{
			_oR1_F4XX = &R1F4XX_PWMCpool[R1F4XX_PWMC_Allocated++];
		}
		else
		{
			_oR1_F4XX = MC_NULL;
		}
	#endif
  
	_oR1_F4XX->pDParams_str = pR1_DDParams;
	_oPWMnCurrFdbk->DerivedClass = (void*)_oR1_F4XX;
	
	_oPWMnCurrFdbk->Methods_str.pIRQ_Handler = &R1F4XX_IRQHandler;
	Set_IRQ_Handler(pR1_DDParams->IRQnb, (_CMCIRQ)_oPWMnCurrFdbk);
  
  _oPWMnCurrFdbk->Methods_str.pPWMC_Init = &R1F4XX_Init;
  _oPWMnCurrFdbk->Methods_str.pPWMC_GetPhaseCurrents = &R1F4XX_GetPhaseCurrents;
  _oPWMnCurrFdbk->Methods_str.pPWMC_SwitchOffPWM = &R1F4XX_SwitchOffPWM;
  _oPWMnCurrFdbk->Methods_str.pPWMC_SwitchOnPWM = &R1F4XX_SwitchOnPWM;        
  _oPWMnCurrFdbk->Methods_str.pPWMC_CurrentReadingCalibr = 
                                                 &R1F4XX_CurrentReadingCalibration;         
  _oPWMnCurrFdbk->Methods_str.pPWMC_TurnOnLowSides = &R1F4XX_TurnOnLowSides;         
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect1 = 
                                                      &R1F4XX_CalcDutyCycles;        
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect2 = 
                                                      &R1F4XX_CalcDutyCycles; 
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect3 = 
                                                      &R1F4XX_CalcDutyCycles;        
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect4 = 
                                                      &R1F4XX_CalcDutyCycles;         
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect5 = 
                                                      &R1F4XX_CalcDutyCycles;        
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect6 = 
                                                      &R1F4XX_CalcDutyCycles;
  _oPWMnCurrFdbk->Methods_str.pPWMC_ExecRegularConv= &R1F4XX_ExecRegularConv;
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetSamplingTime= &R1F4XX_ADC_SetSamplingTime;
  _oPWMnCurrFdbk->Methods_str.pPWMC_IsOverCurrentOccurred = 
    &R1F4XX_IsOverCurrentOccurred;
  
  _oPWMnCurrFdbk->Methods_str.pRLDetectionModeEnable = &R1F4XX_RLDetectionModeEnable;
  _oPWMnCurrFdbk->Methods_str.pRLDetectionModeDisable = &R1F4XX_RLDetectionModeDisable;
  _oPWMnCurrFdbk->Methods_str.pRLDetectionModeSetDuty = &R1F4XX_RLDetectionModeSetDuty;
  
  return ((CR1F4XX_PWMC)_oPWMnCurrFdbk);
}

/** @addtogroup STM32F10x_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup PWMnCurrFdbk_R1_F4XX
  * @{
  */

/** @defgroup R1_F4XX_class_private_methods R1_F4XX class private methods
* @{
*/

/**
* @brief  It initializes TIM, ADC, GPIO, DMA and NVIC for single shunt current 
*         reading configuration using STM32 High Density.
* @param  this related object of class CPWMC
* @retval none
*/
static void R1F4XX_Init(CPWMC this)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;
  uint16_t hAux;
  ADC_TypeDef* ADCx;
  ADC_InitTypeDef ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  TIM_TypeDef* TIMx;
  TIM_TypeDef* TIMx_2;
  pVars_t pVars_str = &CLASS_VARS;
  pDVars_t pDVars_str;
  pDParams_t pDParams_str;
  uint8_t bGPIOAF = GPIO_AF_TIM1;
  
  pDVars_str =   &(((_DCR1F4XX_PWMC)(((_CPWMC) this)->DerivedClass))->DVars_str);
  pDParams_str =  ((_DCR1F4XX_PWMC)(((_CPWMC) this)->DerivedClass))->pDParams_str;
  
  pVars_str->bMotor = (pDParams_str->bInstanceNbr==1u?M1:M2);
  
  R1F4XX_1ShuntMotorVarsInit(this);
  
  /* Default value of ADC */
  pDVars_str->ADCx = ADC3;
  ADCx = ADC3;
  
  /* Auxiliary TIM Used to trigger ADC3 */
  pDVars_str->TIMx_2 = TIM5;
  TIMx_2 = TIM5;
  
  /* DMA Required */
  /* TIM5_CH4 -> DMA1 Stream 1 */
  /* TIM4_CH3 -> DMA1 Stream 7 */
  /* TIM1_CH4 -> DMA2 Stream 4 */
  /* TIM8_CH4 -> DMA2 Stream 7 */
  
  /* Enable DMA1 clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
  
  /* Enable DMA2 clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  
  /* Enable the CCS */
  RCC_ClockSecuritySystemCmd((FunctionalState)(ENABLE));
  
  if (pDParams_str->bInstanceNbr == 1u)
  {    
    /* Peripheral clocks enabling ---------------------------------------------*/
    
    RCC->AHB1ENR |= RCC_AHB1Periph_CRC;

    /* Enable all GPIO clocks */
    RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA | 
                           RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | 
                             RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE | 
                               RCC_AHB1Periph_GPIOF | RCC_AHB1Periph_GPIOG |
                                 RCC_AHB1Periph_GPIOH | RCC_AHB1Periph_GPIOI , ENABLE);  
    
    /* Enable ADC1 clock - Used in any case for regular MC conversion */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); 
  }
  else
  {    
    if(pDParams_str->IsFirstR1DDInstance == FALSE)
    {
      /* ADC1 will be used since ADC3 is already used */
      pDVars_str->ADCx = ADC1;
      ADCx = ADC1;     
      
      /* Auxiliary TIM Used to trigger ADC1 */
      pDVars_str->TIMx_2 = TIM4;
      TIMx_2 = TIM4;           
    }
  }
  
  if (ADCx == ADC3)
  {
    /* Enable ADC3 clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);
  }
  
  if (TIMx_2 == TIM5) /* Used to trigger ADC3 */
  {
    /* Enable TIM5 clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
    
    /* Set timer in Debug MODE */
    /* TIM5 Counter Clock stopped when the core is halted */
    DBGMCU_APB1PeriphConfig(DBGMCU_TIM5_STOP, ENABLE);
    
    
    /* Sets the ADC Trigger for ADC3*/
    pDVars_str->hADCTrigger = ADC_ExternalTrigInjecConv_T5_TRGO;
    
    /* DMA Event related to TIM5 Channel 4 used for ADC3 trigger*/
    /* DMA1 Stream1 channel 6 configuration */
    DMA_DeInit(DMA1_Stream1);
    DMA_InitStructure.DMA_Channel = DMA_Channel_6;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)TIM5_CCR4_Address;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)(pDVars_str->hCCDmaBuffCh4);
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize = 3u;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream1, &DMA_InitStructure);
    DMA_Cmd(DMA1_Stream1, ENABLE);
  }
  
  if (TIMx_2 == TIM4) /* Used to trigger ADC1 */
  {
    /* Enable TIM4 clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    
    /* Set timer in Debug MODE */
    /* TIM4 Counter Clock stopped when the core is halted */
    DBGMCU_APB1PeriphConfig(DBGMCU_TIM4_STOP, ENABLE);
        
    /* Sets the ADC Trigger for ADC1*/
    pDVars_str->hADCTrigger = ADC_ExternalTrigInjecConv_T4_TRGO;
    
    /* DMA Event related to TIM4 Channel 2 used for ADC1 trigger*/
    /* DMA1 Stream3 channel2 configuration */
    DMA_DeInit(DMA1_Stream3);
    DMA_InitStructure.DMA_Channel = DMA_Channel_2;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)TIM4_CCR2_Address;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)(pDVars_str->hCCDmaBuffCh4);
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize = 3u;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream3, &DMA_InitStructure);
    DMA_Cmd(DMA1_Stream3, ENABLE);
  }
  
  /* Enable the ADC Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = (uint8_t) ADC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = ADC_PRE_EMPTION_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = ADC_SUB_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  if (pDParams_str->TIMx == TIM1)
  {
    /* TIM Used*/
    TIMx = TIM1;
    
    /* Enable TIM1 clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    
    /* Set timer in Debug MODE */
    /* TIM1 Counter Clock stopped when the core is halted */
    DBGMCU_APB2PeriphConfig(DBGMCU_TIM1_STOP, ENABLE);
        
    R1F4XX_TIMxInit(TIMx, TIMx_2, this);
    
    /* DMA & NVIC Settings */
    
    /* DMA Event related to TIM1 Channel 4 */
    /* DMA2 Stream4 Channel6 configuration ----------------------------------------------*/
    DMA_DeInit(DMA2_Stream4);
    DMA_InitStructure.DMA_Channel = DMA_Channel_6;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)TIM1_CCR1_Address;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)(pDVars_str->hDmaBuff);
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize = 2u;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream4, &DMA_InitStructure);
    DMA_Cmd(DMA2_Stream4, ENABLE); 
    
    /* Enable the TIM1 Update interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = (uint8_t) TIM1_UP_TIM10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIMx_UP_PRE_EMPTION_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = TIMx_UP_SUB_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);   
        
    if (pDParams_str->bRepetitionCounter > 1u)
    {
      /* Only if REP RATE > 1 */
      /* Enable the DMA2_Stream4 TC interrupt relative to TIM1_CH4 */
      NVIC_InitStructure.NVIC_IRQChannel = (uint8_t) DMA2_Stream4_IRQn;
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = DMAx_TC_PRE_EMPTION_PRIORITY;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = DMAx_TC_SUB_PRIORITY;
      NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&NVIC_InitStructure);
      
      /* Enable DMA2_Stream4 TC IRQ */
      DMA_ITConfig(DMA2_Stream4, DMA_IT_TC, ENABLE);
      
      pDVars_str->bDMATot = (pDParams_str->bRepetitionCounter+1u)/2u;
    }
    else
    {
      /* REP RATE = 1 */
      DMA_ITConfig(DMA2_Stream4, DMA_IT_TC, DISABLE);
      pDVars_str->bDMATot = 0u;
    }
  }
  else
  {
    /* TIM Used*/
    TIMx = TIM8;
        
    /* Enable TIM8 clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
        
    /* Set timer in Debug MODE */
    /* TIM8 Counter Clock stopped when the core is halted */
    DBGMCU_APB2PeriphConfig(DBGMCU_TIM8_STOP, ENABLE);
    
    R1F4XX_TIMxInit(TIMx, TIMx_2, this);
    
    /* DMA & NVIC Settings */
    
    /* DMA Event related to TIM8 Channel 4 */
    /* DMA2 Stream7 Channel7 configuration ----------------------------------------------*/
    DMA_DeInit(DMA2_Stream7);
    DMA_InitStructure.DMA_Channel = DMA_Channel_7;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)TIM8_CCR1_Address;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)(pDVars_str->hDmaBuff);
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize = 2u;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream7, &DMA_InitStructure);
    DMA_Cmd(DMA2_Stream7, ENABLE);  
    
    /* Enable the TIM8 Update interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = (uint8_t) TIM8_UP_TIM13_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIMx_UP_PRE_EMPTION_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = TIMx_UP_SUB_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    if (pDParams_str->bRepetitionCounter > 1u)
    {
      /* Only if REP RATE > 1 */
      /* Enable the DMA2_CH2 TC interrupt */
      NVIC_InitStructure.NVIC_IRQChannel = (uint8_t) DMA2_Stream7_IRQn;
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = DMAx_TC_PRE_EMPTION_PRIORITY;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = DMAx_TC_SUB_PRIORITY;
      NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&NVIC_InitStructure);
      
      /* Enable DMA2 Stream7 TC IRQ */
      DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);
      
      pDVars_str->bDMATot = (pDParams_str->bRepetitionCounter+1u)/2u;
    }
    else
    {
      /* REP RATE = 1 */
      DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, DISABLE);
      pDVars_str->bDMATot = 0u;
    }
    
    /* Used for GPIO AF remap */
    bGPIOAF = GPIO_AF_TIM8;
  }
        
  /* GPIOs configurations --------------------------------------------------*/
  GPIO_StructInit(&GPIO_InitStructure);
  
  /****** Configure phase ADC channel GPIO as analog input ****/
  GPIO_InitStructure.GPIO_Pin = pDParams_str->hIPin;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(pDParams_str->hIPort, &GPIO_InitStructure);
  GPIO_PinLockConfig(pDParams_str->hIPort, pDParams_str->hIPin);
  
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
    /* Clear TIMx break flag. */
    TIM_ClearFlag(TIMx,TIM_FLAG_Break);
  }
  
  /* ADC registers configuration -----------------------------------*/  
  /* Enable ADC */
  ADC_Cmd(ADCx, ENABLE);
  
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
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Left;
  ADC_InitStructure.ADC_NbrOfConversion = 1u;
  ADC_Init(ADCx, &ADC_InitStructure);
      
  if (pDParams_str->bInstanceNbr == 1u)
  {
    /* Init ADC1 - Used in any case for regular conversion */
    ADC_Cmd(ADC1, ENABLE);
    ADC_Init(ADC1, &ADC_InitStructure);
  }
  
  /* Enable Discontinuos mode */
  ADC_InjectedDiscModeCmd(ADCx,ENABLE);
  
  /* ADC Injected conversions configuration */     
  ADC_InjectedSequencerLengthConfig(ADCx,2u); 

  ADC_InjectedChannelConfig(ADCx,
  pDParams_str->hIChannel, 1u, pDParams_str->b_ISamplingTime);
  ADC_InjectedChannelConfig(ADCx,
  pDParams_str->hIChannel, 2u, pDParams_str->b_ISamplingTime);
    
  R1F4XX_1ShuntMotorVarsRestart(this);
  
  /*  Set TIMx_2 CCx start value */
  if (TIMx_2 == TIM4)
  {
    hAux = (PWM_PERIOD >> 1) - pDParams_str->hTbefore;
    TIMx_2->CCR2 = (uint32_t)(hAux);
    TIM_DMACmd(TIMx_2, TIM_DMA_CC2, ENABLE);
  }
  if (TIMx_2 == TIM5)
  {
    hAux = (PWM_PERIOD >> 1) - pDParams_str->hTbefore;
    TIMx_2->CCR4 = (uint32_t)(hAux);
    TIM_DMACmd(TIMx_2, TIM_DMA_CC4, ENABLE);
  }
    
  /* Disabling the Injectec conversion for ADC1*/
  ADC_ExternalTrigInjectedConvEdgeConfig(ADCx,ADC_ExternalTrigInjecConvEdge_None);
  
  /* Select the Injected conversion trigger */
  ADC_ExternalTrigInjectedConvConfig(ADCx, pDVars_str->hADCTrigger);
  
  ADC_ITConfig(ADCx, ADC_IT_JEOC, ENABLE);
}

/**
* @brief  It initializes TIMx and TIMx_2 peripheral for PWM generation, 
          active vector insertion and adc triggering.
* @param  TIMx Timer to be initialized
* @param  TIMx_2 Auxiliary timer to be initialized used for adc triggering
* @param  this related object of class CPWMC
* @retval none
*/
static void R1F4XX_TIMxInit(TIM_TypeDef* TIMx, TIM_TypeDef* TIMx_2, CPWMC this)
{
  uint16_t hAux;
  TIM_TimeBaseInitTypeDef TIMx_TimeBaseStructure;
  TIM_OCInitTypeDef TIMx_OCInitStructure;
  TIM_BDTRInitTypeDef TIMx_BDTRInitStructure;
  pDVars_t pDVars_str;
  pDParams_t pDParams_str;
  
  pDVars_str =   &(((_DCR1F4XX_PWMC)(((_CPWMC) this)->DerivedClass))->DVars_str);
  pDParams_str =  ((_DCR1F4XX_PWMC)(((_CPWMC) this)->DerivedClass))->pDParams_str;
  
  /* TIMx Peripheral Configuration -------------------------------------------*/
  /* TIMx Registers reset */
  TIM_DeInit(TIMx);
  TIM_DeInit(TIMx_2);
  TIM_TimeBaseStructInit(&TIMx_TimeBaseStructure);
  /* Time Base configuration */
  TIMx_TimeBaseStructure.TIM_Prescaler = (uint16_t)(pDParams_str->bTim_Clock_Divider) - 1u;  
  TIMx_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned3;
  hAux = PWM_PERIOD;
  TIMx_TimeBaseStructure.TIM_Period = (uint32_t)(hAux);
  TIMx_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV2;
  TIMx_TimeBaseStructure.TIM_RepetitionCounter = pDParams_str->bRepetitionCounter;
  TIM_TimeBaseInit(TIMx, &TIMx_TimeBaseStructure);
  TIMx_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  hAux = PWM_PERIOD - 1u;
  TIMx_TimeBaseStructure.TIM_Period = (uint32_t)(hAux);
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
  hAux = PWM_PERIOD-pDParams_str->hHTMin;
  TIMx_OCInitStructure.TIM_Pulse = (uint32_t)(hAux);
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

  /* TIMx_2 Init */
  hAux = PWM_PERIOD >> 2u - pDParams_str->hTMin - pDParams_str->hTbefore;
  TIMx_OCInitStructure.TIM_Pulse =  (uint32_t)(hAux);
  TIMx_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable; /* Enable here for sampling point debug */
  if (TIMx_2 == TIM4)
  {
    TIM_SelectOutputTrigger(TIMx_2, TIM_TRGOSource_OC2Ref);
    TIM_OC2Init(TIMx_2, &TIMx_OCInitStructure);
    pDVars_str->pTIMx_2_CCR = &(TIM4->CCR2);
  }
  if (TIMx_2 == TIM5)
  {
    TIM_SelectOutputTrigger(TIMx_2, TIM_TRGOSource_OC4Ref);
    TIM_OC4Init(TIMx_2, &TIMx_OCInitStructure);
    pDVars_str->pTIMx_2_CCR = &(TIM5->CCR4);
  }
      
  TIM_SelectInputTrigger(TIMx,TIM_TS_ITR1);
  TIM_SelectSlaveMode(TIMx,TIM_SlaveMode_Trigger);    
  
  TIM_SelectSlaveMode(TIMx_2,TIM_SlaveMode_Trigger);
  if (TIMx_2 == TIM4)
  {
    TIM_SelectInputTrigger(TIMx_2,TIM_TS_ITR1);
  }
  if (TIMx_2 == TIM5)
  {
    TIM_SelectInputTrigger(TIMx_2,TIM_TS_ITR0);
  }

  /* Prepare timer for synchronization */
  TIM_GenerateEvent(TIMx,TIM_EventSource_Update);
  TIM_GenerateEvent(TIMx_2,TIM_EventSource_Update);
      
  if (pDParams_str->bFreqRatio == 2u) 
  {
    if (pDParams_str->bIsHigherFreqTim == HIGHER_FREQ)
    {
      if (pDParams_str->bRepetitionCounter == 3u)
      {
        /* Set TIM1 repetition counter to 1 */
        TIMx->RCR =0x01u; 
        TIM_GenerateEvent(TIMx, TIM_EventSource_Update);
        /* Repetition counter will be set to 3 at next Update */
        TIMx->RCR =0x03u; 
      }
    }
    hAux = PWM_PERIOD-1u;
    TIM_SetCounter(TIMx, (uint32_t)(hAux));
    hAux = PWM_PERIOD/2u;
    TIM_SetCounter(TIMx_2, (uint32_t)(hAux));         
  }
  else /* bFreqRatio equal to 1 or 3 */
  {
    if (pDParams_str->bInstanceNbr == 1u)
    {
      hAux = PWM_PERIOD-1u;
      TIM_SetCounter(pDParams_str ->TIMx, (uint32_t)(hAux));
      hAux = PWM_PERIOD/2u;
      TIM_SetCounter(pDVars_str->TIMx_2, (uint32_t)(hAux));
    }
  }
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
void R1F4XX_StartTimers(void)
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
static void R1F4XX_CurrentReadingCalibration(CPWMC this)
{
  pDVars_t pDVars_str;			
  pDParams_t pDParams_str;	
  TIM_TypeDef*  LocalTIMx;
  uint16_t htempCCER, haux;
  
  pDVars_str =   &(((_DCR1F4XX_PWMC)(((_CPWMC) this)->DerivedClass))->DVars_str);
  pDParams_str =  ((_DCR1F4XX_PWMC)(((_CPWMC) this)->DerivedClass))->pDParams_str;

  LocalTIMx = pDParams_str->TIMx;
  
  pDVars_str->wPhaseOffset = 0u;
  
  pDVars_str->bIndex=0u;
  
  /* Force inactive level on TIMx CHy and TIMx CHyN */ 
  htempCCER =  pDParams_str->TIMx->CCER;
  haux = htempCCER & TIMxCCER_MASK;
  haux |= TIMx_CC4E_BIT;
  LocalTIMx->CCER = haux;
  
  /* Change function to be executed in ADCx_ISR */ 
  ((_CPWMC) this)->Methods_str.pPWMC_GetPhaseCurrents = &R1F4XX_HFCurrentsCalibration;
  
  R1F4XX_SwitchOnPWM(this);
  
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
  
  R1F4XX_SwitchOffPWM( this);
  
  pDVars_str->wPhaseOffset = pDVars_str->wPhaseOffset/NB_CONVERSIONS;
  pDVars_str->wPhaseOffset <<= 1;
 
  /* Set back TIMx CCER register */ 
  LocalTIMx->CCER = htempCCER;
  /* Change back function to be executed in ADCx_ISR */ 
  ((_CPWMC) this)->Methods_str.pPWMC_GetPhaseCurrents = &R1F4XX_GetPhaseCurrents;  
}

/**
* @brief  First initialization of class members
* @param  this related object of class CPWMC
* @retval none
*/
static void R1F4XX_1ShuntMotorVarsInit(CPWMC this)
{
  uint16_t hAux;
  pDVars_t pDVars_str;
  pDParams_t pDParams_str;
  
  pDVars_str =   &(((_DCR1F4XX_PWMC)(((_CPWMC) this)->DerivedClass))->DVars_str);
  pDParams_str =  ((_DCR1F4XX_PWMC)(((_CPWMC) this)->DerivedClass))->pDParams_str;
  
  /* Init motor vars */
  pDVars_str->wPhaseOffset=0u;
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
  pDVars_str->hCCDmaBuffCh4[0] = (uint32_t)(pDVars_str->hCntSmp2) >> 1u; /*  Second point */
  hAux = PWM_PERIOD - 1u;
  pDVars_str->hCCDmaBuffCh4[1] = (uint32_t)(hAux); /* Update */
  pDVars_str->hCCDmaBuffCh4[2] = (uint32_t)(pDVars_str->hCntSmp1) >> 1u; /* First point */
  
  TIM_DMACmd(pDParams_str->TIMx, TIM_DMA_CC4, DISABLE);
}

/**
* @brief  Initialization of class members after each motor start
* @param  this related object of class CPWMC
* @retval none
*/
static void R1F4XX_1ShuntMotorVarsRestart(CPWMC this)
{
  pDVars_t pDVars_str;
  pDParams_t pDParams_str;
  
  pDVars_str =   &(((_DCR1F4XX_PWMC)(((_CPWMC) this)->DerivedClass))->DVars_str);
  pDParams_str =  ((_DCR1F4XX_PWMC)(((_CPWMC) this)->DerivedClass))->pDParams_str;
  
  /* Default value of DutyValues */
  pDVars_str->hCntSmp1 = (PWM_PERIOD >> 1) - pDParams_str->hTbefore;
  pDVars_str->hCntSmp2 = (PWM_PERIOD >> 1) + pDParams_str->hTafter;
  
  /* Default value of sampling point */
  pDVars_str->hCCDmaBuffCh4[0] = (uint32_t)(pDVars_str->hCntSmp2) >> 1u; /*  Second point */
  pDVars_str->hCCDmaBuffCh4[2] = (uint32_t)(pDVars_str->hCntSmp1) >> 1u; /* First point */
  
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
    
  TIM_DMACmd(pDParams_str->TIMx, TIM_DMA_CC4, DISABLE);
}

/**
* @brief  It computes and return latest converted motor phase currents motor
* @param  this related object of class CPWMC
* @retval Curr_Components Ia and Ib current in Curr_Components format
*/
static void R1F4XX_GetPhaseCurrents(CPWMC this,Curr_Components* pStator_Currents)
{  
  int32_t wAux;
  int16_t hCurrA = 0, hCurrB = 0, hCurrC = 0;
  uint8_t bCurrASamp = 0u, bCurrBSamp = 0u, bCurrCSamp = 0u;
  pDVars_t pDVars_str;
  
  pDVars_str =   &(((_DCR1F4XX_PWMC)(((_CPWMC) this)->DerivedClass))->DVars_str);
  
  /* Disabling the Injectec conversion for ADCx after EOC*/
  /* Stdlib replaced: ADC_ExternalTrigInjectedConvCmd(pDVars_str->ADCx,DISABLE);*/
  pDVars_str->ADCx->CR2 &= CR2_JEXTTRIG_Reset;
  
  /* Reset the bSOFOC flags to indicate the start of FOC algorithm*/
  pDVars_str->hFlags &= (~SOFOC);
  
  /* First sampling point */
  wAux = (int32_t)(pDVars_str->ADCx->JDR1);
  wAux *= 2;
  wAux -= (int32_t)(pDVars_str->wPhaseOffset);
  
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
  wAux = (int32_t)(pDVars_str->ADCx->JDR2);
  wAux *= 2;
  wAux -= (int32_t)(pDVars_str->wPhaseOffset);
  
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
* @brief  It sum up injected conversion data into wPhaseOffset. It is called 
*         only during current calibration 
* @param  this related object
* @retval Curr_Components It always returns {0,0} in Curr_Components format
*/
static void R1F4XX_HFCurrentsCalibration(CPWMC this,Curr_Components* pStator_Currents)
{
  pDVars_t pDVars_str;
  
  pDVars_str =   &(((_DCR1F4XX_PWMC)(((_CPWMC) this)->DerivedClass))->DVars_str);
  
  /* Disabling the Injectec conversion for ADCx after EOC*/
  /* Stdlib replaced: ADC_ExternalTrigInjectedConvCmd(pDVars_str->ADCx,DISABLE);*/
  pDVars_str->ADCx->CR2 &= CR2_JEXTTRIG_Reset;
  
  /* Reset the SOFOC flag to indicate the start of FOC algorithm*/
  pDVars_str->hFlags &= (~SOFOC); 
    
  if (pDVars_str->bIndex < NB_CONVERSIONS)
  {
    pDVars_str->wPhaseOffset += pDVars_str->ADCx->JDR1;
    pDVars_str->bIndex++;
  }
}

/**
* @brief  It turns on low sides switches. This function is intended to be 
*         used for charging boot capacitors of driving section. It has to be 
*         called each motor start-up when using high voltage drivers
* @param  this related object of class CPWMC
* @retval none
*/
static void R1F4XX_TurnOnLowSides(CPWMC this)
{
  pDParams_t pDParams_str;
  
  pDParams_str =  ((_DCR1F4XX_PWMC)(((_CPWMC) this)->DerivedClass))->pDParams_str;

  pDParams_str->TIMx->CCR1 = 0u;
  pDParams_str->TIMx->CCR2 = 0u;
  pDParams_str->TIMx->CCR3 = 0u;
  
  TIM_ClearFlag(pDParams_str->TIMx,TIM_FLAG_Update);
  while (TIM_GetFlagStatus(pDParams_str->TIMx,TIM_FLAG_Update) == RESET)
  {}
  
  /* Main PWM Output Enable */
  TIM_CtrlPWMOutputs(pDParams_str->TIMx, ENABLE);
  if ((pDParams_str->LowSideOutputs)== ES_GPIO)
  {
    GPIO_WriteBit(pDParams_str->hCh1NPort, pDParams_str->hCh1NPin, Bit_SET);
    GPIO_WriteBit(pDParams_str->hCh2NPort, pDParams_str->hCh2NPin, Bit_SET);
    GPIO_WriteBit(pDParams_str->hCh3NPort, pDParams_str->hCh3NPin, Bit_SET);
  }
  return; 
}

/**
* @brief  This function enables the update event and the single shunt distortion
* @param  this related object of class CPWMC
* @retval none
*/
static void R1F4XX_SwitchOnPWM(CPWMC this)
{
  uint16_t hAux;
  pDVars_t pDVars_str;
  pDParams_t pDParams_str;
  
  pDVars_str =   &(((_DCR1F4XX_PWMC)(((_CPWMC) this)->DerivedClass))->DVars_str);
  pDParams_str =  ((_DCR1F4XX_PWMC)(((_CPWMC) this)->DerivedClass))->pDParams_str;
  
  /* Set all duty to 50% */
  hAux = PWM_PERIOD >> 1u;
  pDParams_str->TIMx->CCR1 = (uint32_t)(hAux);
  pDParams_str->TIMx->CCR2 = (uint32_t)(hAux);
  pDParams_str->TIMx->CCR3 = (uint32_t)(hAux);
  
  TIM_ClearFlag(pDParams_str->TIMx, TIM_FLAG_Update);
  while (TIM_GetFlagStatus(pDParams_str->TIMx,TIM_FLAG_Update) == RESET)
  {}
  
  /* Enable UPDATE ISR */
  TIM_ClearFlag(pDParams_str->TIMx, TIM_FLAG_Update);
  TIM_ITConfig(pDParams_str->TIMx, TIM_IT_Update, ENABLE);
  
  /* Enabling distortion for single shunt */
  pDVars_str->hFlags |= DSTEN;
  
  /* Main PWM Output Enable */
  TIM_CtrlPWMOutputs(pDParams_str->TIMx, ENABLE);
  if ((pDParams_str->LowSideOutputs)== ES_GPIO)
  {
    if ((pDParams_str->TIMx->CCER & TIMxCCER_MASK_CH123) != 0u)
    {
      GPIO_WriteBit(pDParams_str->hCh1NPort, pDParams_str->hCh1NPin, Bit_SET);
      GPIO_WriteBit(pDParams_str->hCh2NPort, pDParams_str->hCh2NPin, Bit_SET);
      GPIO_WriteBit(pDParams_str->hCh3NPort, pDParams_str->hCh3NPin, Bit_SET);
    }
    else
    {
      /* It is executed during calibration phase the EN signal shall stay off */
      GPIO_WriteBit(pDParams_str->hCh1NPort, pDParams_str->hCh1NPin, Bit_RESET);
      GPIO_WriteBit(pDParams_str->hCh2NPort, pDParams_str->hCh2NPin, Bit_RESET);
      GPIO_WriteBit(pDParams_str->hCh3NPort, pDParams_str->hCh3NPin, Bit_RESET);
    }
  }
  return; 
}

/**
* @brief  It disables PWM generation on the proper Timer peripheral acting on 
*         MOE bit, disables the single shunt distortion and reset the TIM status
* @param  this related object of class CPWMC
* @retval none
*/
static void R1F4XX_SwitchOffPWM(CPWMC this)
{
  uint16_t hAux;
  pDVars_t pDVars_str;
  pDParams_t pDParams_str;
  
  pDVars_str =   &(((_DCR1F4XX_PWMC)(((_CPWMC) this)->DerivedClass))->DVars_str);
  pDParams_str =  ((_DCR1F4XX_PWMC)(((_CPWMC) this)->DerivedClass))->pDParams_str;
  
  /* Main PWM Output Disable */
  TIM_CtrlPWMOutputs(pDParams_str->TIMx, DISABLE);
  if ((pDParams_str->LowSideOutputs)== ES_GPIO)
  {
    GPIO_WriteBit(pDParams_str->hCh1NPort, pDParams_str->hCh1NPin, Bit_RESET);
    GPIO_WriteBit(pDParams_str->hCh2NPort, pDParams_str->hCh2NPin, Bit_RESET);
    GPIO_WriteBit(pDParams_str->hCh3NPort, pDParams_str->hCh3NPin, Bit_RESET);
  }
  
  /* Disable UPDATE ISR */
  TIM_ITConfig(pDParams_str->TIMx, TIM_IT_Update, DISABLE);
    
  /* Disabling distortion for single */
  pDVars_str->hFlags &= (~DSTEN);
  
  while (TIM_GetFlagStatus(pDParams_str->TIMx,TIM_FLAG_Update)==RESET)
  {
    if (pDParams_str->TIMx->DIER & TIM_IT_Update)
    { break;}
  }
    
    
  /* Disabling all DMA previous setting */
  TIM_DMACmd(pDParams_str->TIMx, TIM_DMA_CC4, DISABLE);  
  
  /* Set all duty to 50% */
  hAux = PWM_PERIOD >> 1u;
  pDParams_str->TIMx->CCR1 = (uint32_t)(hAux);
  pDParams_str->TIMx->CCR2 = (uint32_t)(hAux);
  pDParams_str->TIMx->CCR3 = (uint32_t)(hAux);    
    
  return; 
}

/**
* @brief  Implementation of the single shunt algorithm to setup the 
*         TIM1 register and DMA buffers values for the next PWM period.
* @param  this related object of class CPWMC
* @retval uint16_t It returns MC_FOC_DURATION if the TIMx update occurs 
          before the end of FOC algorithm else returns MC_NO_ERROR
*/
static uint16_t R1F4XX_CalcDutyCycles(CPWMC this)
{
  int16_t hDeltaDuty_0;
  int16_t hDeltaDuty_1;
  uint16_t hDutyV_0 = 0u;
  uint16_t hDutyV_1 = 0u;
  uint16_t hDutyV_2 = 0u;
  uint8_t bSector;
  uint8_t bStatorFluxPos;
  TIM_TypeDef* TIMx;
  uint16_t hAux;
  pDVars_t pDVars_str;
  pDParams_t pDParams_str;
    
  pDVars_str =   &(((_DCR1F4XX_PWMC)(((_CPWMC) this)->DerivedClass))->DVars_str);
  pDParams_str =  ((_DCR1F4XX_PWMC)(((_CPWMC) this)->DerivedClass))->pDParams_str;
  
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
    TIMx = pDParams_str->TIMx;
    
    /* Preload Enable */
    TIMx->CCMR1 |= CCMR1_PRELOAD_ENABLE_MASK;
    TIMx->CCMR2 |= CCMR2_PRELOAD_ENABLE_MASK;
    
    TIMx->CCR1 = CLASS_VARS.hCntPhA;
    TIMx->CCR2 = CLASS_VARS.hCntPhB;
    TIMx->CCR3 = CLASS_VARS.hCntPhC;
    
    /* Update ADC Trigger DMA buffer */    
    hAux = pDVars_str->hCntSmp2 >> 1; /* Second point */
    pDVars_str->hCCDmaBuffCh4[0] = (uint32_t)(hAux); /* Second point */
    hAux = pDVars_str->hCntSmp1 >> 1; /* First point */
    pDVars_str->hCCDmaBuffCh4[2] = (uint32_t)(hAux); /* First point */
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
  * @brief  R1_F4XX implement MC IRQ function TIMER Update and DMA TC
  * @param  this related object
  * @param  flag used to indicate which IRQ has been occurred
  *			0 Means TIM1 Update IRQ occurred
  *			1 Means TIM8 Update IRQ occurred
  *			2 Means DAC TC IRQ occurred
  * @retval void* Pointer to drive object related to PWMnCurr object. 
            It is set in the init function and returned by the MC TIMx update 
            IRQ handler
  */
static void* R1F4XX_IRQHandler(void *this, unsigned char flag)
{ 
  uint16_t hAux;  
  uint8_t bInverted_pwm_new;
  pVars_t pVars_str = &CLASS_VARS;
  pDVars_t pDVars_str;
  pDParams_t pDParams_str;
  
  pDVars_str =   &(((_DCR1F4XX_PWMC)(((_CPWMC) this)->DerivedClass))->DVars_str);
  pDParams_str =  ((_DCR1F4XX_PWMC)(((_CPWMC) this)->DerivedClass))->pDParams_str;
  
  switch (flag)
  {
  case 0: /* TIM1 Update IRQ */
    {      
      /* Critical point start */
      
      /* Update timer for first sampling */
      *(pDVars_str->pTIMx_2_CCR) = pDVars_str->hCCDmaBuffCh4[2];
      
      /* Enabling the Injectec conversion for ADCx*/
      /* Stdlib replaced: ADC_ExternalTrigInjectedConvCmd(pDVars_str->ADCx,ENABLE); */
      pDVars_str->ADCx->CR2 |= CR2_JEXTTRIG_Set;
      
      /* Critical point stop */
      
      /* TMP var to speedup the execution */
      bInverted_pwm_new = pDVars_str->bInverted_pwm_new;
      
      if (bInverted_pwm_new != pDVars_str->bInverted_pwm)  
      {                      
        /* Set the DMA destination */
        switch (bInverted_pwm_new)
        {
        case INVERT_A:
          DMA2_Stream4->CR &= ~(uint32_t)DMA_SxCR_EN; /* Disable DMA to access PAR */
          DMA2_Stream4->PAR = TIM1_CCR1_Address;
          DMA2_Stream4->CR |= (uint32_t)DMA_SxCR_EN; /* Enable DMA */
          DMA_ClearFlag(DMA2_Stream4,DMA_FLAG_TCIF4); /* Clear TC flag set due DMA disabling */ 
          /* NVIC->ICPR[1] = 0x10000000;  Clear NVIC pending FLAG */
          /* Stdlib replaced: TIM_DMACmd(pDVars_str->TIMx, TIM_DMA_CC4, ENABLE);*/
          pDParams_str->TIMx->DIER |= TIM_DMA_CC4;          
          break;
          
        case INVERT_B:
          DMA2_Stream4->CR &= ~(uint32_t)DMA_SxCR_EN; /* Disable DMA to access PAR */
          DMA2_Stream4->PAR = TIM1_CCR2_Address;
          DMA2_Stream4->CR |= (uint32_t)DMA_SxCR_EN; /* Enable DMA */
          DMA_ClearFlag(DMA2_Stream4,DMA_FLAG_TCIF4); /* Clear TC flag set due DMA disabling */
          /* NVIC->ICPR[1] = 0x10000000;  Clear NVIC pending FLAG */
          /* Stdlib replaced: TIM_DMACmd(pDVars_str->TIMx, TIM_DMA_CC4, ENABLE);*/
          pDParams_str->TIMx->DIER |= TIM_DMA_CC4;
          break;
          
        case INVERT_C:
          DMA2_Stream4->CR &= ~(uint32_t)DMA_SxCR_EN; /* Disable DMA to access PAR */
          DMA2_Stream4->PAR = TIM1_CCR3_Address;
          DMA2_Stream4->CR |= (uint32_t)DMA_SxCR_EN; /* Enable DMA */
          DMA_ClearFlag(DMA2_Stream4,DMA_FLAG_TCIF4); /* Clear TC flag set due DMA disabling */
          /* NVIC->ICPR[1] = 0x10000000;  Clear NVIC pending FLAG */
          /* Stdlib replaced: TIM_DMACmd(pDVars_str->TIMx, TIM_DMA_CC4, ENABLE);*/
          pDParams_str->TIMx->DIER |= TIM_DMA_CC4;
          break;
          
        default:
          /* Stdlib replaced: TIM_DMACmd(pDVars_str->TIMx, TIM_DMA_CC4, DISABLE);*/
          pDParams_str->TIMx->DIER &= (u16)~TIM_DMA_CC4;
          
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
  case 1: /* TIM8 Update IRQ */
    {      
      /* Critical point start */
      
      /* Update timer for first sampling */
      *(pDVars_str->pTIMx_2_CCR) = pDVars_str->hCCDmaBuffCh4[2];
      
      /* Enabling the Injectec conversion for ADCx*/
      /* Stdlib replaced: ADC_ExternalTrigInjectedConvCmd(pDVars_str->ADCx,ENABLE); */
      pDVars_str->ADCx->CR2 |= CR2_JEXTTRIG_Set;
            
      /* Critical point stop */
      
      /* TMP var to speedup the execution */
      bInverted_pwm_new = pDVars_str->bInverted_pwm_new;
      
      if (bInverted_pwm_new != pDVars_str->bInverted_pwm)  
      {                      
        /* Set the DMA destination */
        switch (bInverted_pwm_new)
        {
        case INVERT_A:
          DMA2_Stream7->CR &= ~(uint32_t)DMA_SxCR_EN; /* Disable DMA to access PAR */
          DMA2_Stream7->PAR = TIM8_CCR1_Address;
          DMA2_Stream7->CR |= (uint32_t)DMA_SxCR_EN; /* Enable DMA */
          DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7); /* Clear TC flag set due DMA disabling */
          /* NVIC->ICPR[2] = 0x00000040;  Clear NVIC pending FLAG */
          /* Stdlib replaced: TIM_DMACmd(pDVars_str->TIMx, TIM_DMA_CC4, ENABLE);*/
          pDParams_str->TIMx->DIER |= TIM_DMA_CC4;          
          break;
          
        case INVERT_B:
          DMA2_Stream7->CR &= ~(uint32_t)DMA_SxCR_EN; /* Disable DMA to access PAR */
          DMA2_Stream7->PAR = TIM8_CCR2_Address;
          DMA2_Stream7->CR |= (uint32_t)DMA_SxCR_EN; /* Enable DMA */
          DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7); /* Clear TC flag set due DMA disabling */
          /* NVIC->ICPR[2] = 0x00000040;  Clear NVIC pending FLAG */
          /* Stdlib replaced: TIM_DMACmd(pDVars_str->TIMx, TIM_DMA_CC4, ENABLE);*/
          pDParams_str->TIMx->DIER |= TIM_DMA_CC4;
          break;
          
        case INVERT_C:
          DMA2_Stream7->CR &= ~(uint32_t)DMA_SxCR_EN; /* Disable DMA to access PAR */
          DMA2_Stream7->PAR = TIM8_CCR3_Address;
          DMA2_Stream7->CR |= (uint32_t)DMA_SxCR_EN; /* Enable DMA */
          DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7); /* Clear TC flag set due DMA disabling */
          /* NVIC->ICPR[2] = 0x00000040;  Clear NVIC pending FLAG */
          /* Stdlib replaced: TIM_DMACmd(pDVars_str->TIMx, TIM_DMA_CC4, ENABLE);*/
          pDParams_str->TIMx->DIER |= TIM_DMA_CC4;
          break;
          
        default:
          /* Stdlib replaced: TIM_DMACmd(pDVars_str->TIMx, TIM_DMA_CC4, DISABLE);*/
          pDParams_str->TIMx->DIER &= (u16)~TIM_DMA_CC4;
          break;
        }  
      }
            
      /* Clear of End of FOC Flags */
      pDVars_str->hFlags &= (~EOFOC);
      
      /* Preload Disable */
      TIM8->CCMR1 &= CCMR1_PRELOAD_DISABLE_MASK;
      TIM8->CCMR2 &= CCMR2_PRELOAD_DISABLE_MASK;
            
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
  case 2: /* DAC TC IRQ */
    {
      pDVars_str->bDMACur--;
      if (pDVars_str->bDMACur == 0u)
      {
        if ((pDVars_str->hFlags & EOFOC) != 0u)
        {
          /* Preload Enable */
          pDParams_str->TIMx->CCMR1 |= CCMR1_PRELOAD_ENABLE_MASK;
          pDParams_str->TIMx->CCMR2 |= CCMR2_PRELOAD_ENABLE_MASK;
          
          /* Compare register update */
          pDParams_str->TIMx->CCR1 = CLASS_VARS.hCntPhA;
          pDParams_str->TIMx->CCR2 = CLASS_VARS.hCntPhB;
          pDParams_str->TIMx->CCR3 = CLASS_VARS.hCntPhC;
          
          /* Update ADC Trigger DMA buffer */    
          hAux = pDVars_str->hCntSmp2 >> 1; /* Second point */
          pDVars_str->hCCDmaBuffCh4[0] = (uint32_t)(hAux); /* Second point */
          hAux = pDVars_str->hCntSmp1 >> 1; /* First point */
          pDVars_str->hCCDmaBuffCh4[2] = (uint32_t)(hAux); /* First point */        
        }
      }
    }
    break;
  default:
    break;
  }
  
  return &(pVars_str->bMotor);
}

/**
* @brief  Execute a regular conversion. 
*         The function is not re-entrant (can't executed twice at the same time)
*         It returns 0xFFFF in case of conversion error.
* @param  this related object of class CPWMC
* @param  bChannel ADC channel used for the regular conversion
* @retval uint16_t It returns converted value or oxFFFF for conversion error */
static uint16_t R1F4XX_ExecRegularConv(CPWMC this, uint8_t bChannel)
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
* @param  this related object of class CPWMC
* @param  ADConv_struct struct containing ADC channel and sampling time
* @retval none
*/
static void R1F4XX_ADC_SetSamplingTime(CPWMC this, ADConv_t ADConv_struct)
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
static uint16_t R1F4XX_IsOverCurrentOccurred(CPWMC this)
{
  TIM_TypeDef*  LocalTIMx = ((_DCR1F4XX_PWMC)(((_CPWMC) this)->DerivedClass))->
    pDParams_str->TIMx;
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
* @brief  It is used to set the PWM mode for R/L detection.
* @param  this related object of class CPWMC
* @param  hDuty to be applied in u16
* @retval none
*/
static void R1F4XX_RLDetectionModeEnable(CPWMC this)
{
  pVars_t pVars = &CLASS_VARS;
  pDParams_t pDParams = DCLASS_PARAMS;
  TIM_TypeDef*  TIMx = pDParams->TIMx;
  
  if (pVars->RLDetectionMode == FALSE)
  {
    /*  Channel1 configuration */
    TIM_SelectOCxM(TIMx, TIM_Channel_1, TIM_OCMode_PWM1);
    TIM_CCxCmd(TIMx, TIM_Channel_1, TIM_CCx_Enable);
    TIM_CCxNCmd(TIMx, TIM_Channel_1, TIM_CCxN_Disable);
    
    TIM_SetCompare1(TIMx, 0u);
    
    /*  Channel2 configuration */
    if ((pDParams-> LowSideOutputs)== LS_PWM_TIMER)
    {
      TIM_SelectOCxM(TIMx, TIM_Channel_2, TIM_OCMode_Active);
      TIM_CCxCmd(TIMx, TIM_Channel_2, TIM_CCx_Disable);
      TIM_CCxNCmd(TIMx, TIM_Channel_2, TIM_CCxN_Enable);
    }
    else if ((pDParams->LowSideOutputs)== ES_GPIO)
    {
      TIM_SelectOCxM(TIMx, TIM_Channel_2, TIM_OCMode_Inactive);
      TIM_CCxCmd(TIMx, TIM_Channel_2, TIM_CCx_Enable);
      TIM_CCxNCmd(TIMx, TIM_Channel_2, TIM_CCxN_Disable);
    }
    else
    {
    }
    
    /*  Channel3 configuration */
    TIM_SelectOCxM(TIMx, TIM_Channel_3, TIM_OCMode_PWM2);
    TIM_CCxCmd(TIMx, TIM_Channel_3, TIM_CCx_Disable);
    TIM_CCxNCmd(TIMx, TIM_Channel_3, TIM_CCxN_Disable);
  }
  
  ((_CPWMC)this)->Methods_str.pPWMC_GetPhaseCurrents = &R1F4XX_RLGetPhaseCurrents;
  ((_CPWMC)this)->Methods_str.pPWMC_TurnOnLowSides = &R1F4XX_RLTurnOnLowSides;
  ((_CPWMC)this)->Methods_str.pPWMC_SwitchOnPWM = &R1F4XX_RLSwitchOnPWM;
  ((_CPWMC)this)->Methods_str.pPWMC_SwitchOffPWM = &R1F4XX_RLSwitchOffPWM;
  
  pVars->RLDetectionMode = TRUE;
}

/**
* @brief  It is used to disable the PWM mode in 6-step.
* @param  this related object of class CPWMC
* @retval none
*/
static void R1F4XX_RLDetectionModeDisable(CPWMC this)
{
  pVars_t pVars = &CLASS_VARS;
  pDParams_t pDParams = DCLASS_PARAMS;
  TIM_TypeDef*  TIMx = pDParams->TIMx;
  
  if (pVars->RLDetectionMode == TRUE)
  {
    uint16_t hAux = PWM_PERIOD;
    /*  Channel1 configuration */
    TIM_SelectOCxM(TIMx, TIM_Channel_1, TIM_OCMode_PWM1);
    TIM_CCxCmd(TIMx, TIM_Channel_1, TIM_CCx_Enable);
    
    if ((pDParams-> LowSideOutputs)== LS_PWM_TIMER)
    {
      TIM_CCxNCmd(TIMx, TIM_Channel_1, TIM_CCxN_Enable);
    }
    else if ((pDParams->LowSideOutputs)== ES_GPIO)
    {
      TIM_CCxNCmd(TIMx, TIM_Channel_1, TIM_CCxN_Disable);
    }
    else
    {
    }
    
    TIM_SetCompare1(TIMx, (uint32_t)(hAux) >> 1);
    
    /*  Channel2 configuration */
    TIM_SelectOCxM(TIMx, TIM_Channel_2, TIM_OCMode_PWM1);
    TIM_CCxCmd(TIMx, TIM_Channel_2, TIM_CCx_Enable);
    
    if ((pDParams-> LowSideOutputs)== LS_PWM_TIMER)
    {
      TIM_CCxNCmd(TIMx, TIM_Channel_2, TIM_CCxN_Enable);
    }
    else if ((pDParams->LowSideOutputs)== ES_GPIO)
    {
      TIM_CCxNCmd(TIMx, TIM_Channel_2, TIM_CCxN_Disable);
    }
    else
    {
    }
    
    TIM_SetCompare2(TIMx, (uint32_t)(hAux) >> 1);
    
    /*  Channel3 configuration */
    TIM_SelectOCxM(TIMx, TIM_Channel_3, TIM_OCMode_PWM1);
    TIM_CCxCmd(TIMx, TIM_Channel_3, TIM_CCx_Enable);
    
    if ((pDParams-> LowSideOutputs)== LS_PWM_TIMER)
    {
      TIM_CCxNCmd(TIMx, TIM_Channel_3, TIM_CCxN_Enable);
    }
    else if ((pDParams->LowSideOutputs)== ES_GPIO)
    {
      TIM_CCxNCmd(TIMx, TIM_Channel_3, TIM_CCxN_Disable);
    }
    else
    {
    }
    
    TIM_SetCompare3(TIMx, (uint32_t)(hAux) >> 1);
    
    ((_CPWMC)this)->Methods_str.pPWMC_GetPhaseCurrents = &R1F4XX_GetPhaseCurrents;
    ((_CPWMC)this)->Methods_str.pPWMC_TurnOnLowSides = &R1F4XX_TurnOnLowSides;
    ((_CPWMC)this)->Methods_str.pPWMC_SwitchOnPWM = &R1F4XX_SwitchOnPWM;
    ((_CPWMC)this)->Methods_str.pPWMC_SwitchOffPWM = &R1F4XX_SwitchOffPWM;
    
    pVars->RLDetectionMode = FALSE;
  }
}

/**
* @brief  It is used to set the PWM dutycycle in 6-step mode.
* @param  this related object of class CPWMC
* @param  hDuty to be applied in u16
* @retval It returns the code error 'MC_FOC_DURATION' if any, 'MC_NO_ERROR' 
*         otherwise. These error codes are defined in MC_type.h
*/
static uint16_t R1F4XX_RLDetectionModeSetDuty(CPWMC this, uint16_t hDuty)
{
  pVars_t pVars = &CLASS_VARS;
  pDParams_t pDParams = DCLASS_PARAMS;
  TIM_TypeDef*  TIMx = pDParams->TIMx;
  uint16_t hAux = PWM_PERIOD;
  
  uint32_t val = ((uint32_t)(hAux) * (uint32_t)(hDuty)) >> 16;
  pVars->hCntPhA = (uint16_t)(val);
    
  TIMx->CCR1 = ((_CPWMC) this)->Vars_str.hCntPhA;
    
  /* Limit for update event */
  /* Check the status flag. If an update event has occurred before to set new
  values of regs the FOC rate is too high */
  if (TIMx->SR & TIM_FLAG_Update)
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
  return hAux;
}

/**
* @brief  It computes and return latest converted motor phase currents motor
*         during RL detection phase
* @param  this: related object of class CR3F30X_PWMC
* @retval Ia and Ib current in Curr_Components format
*/
static void R1F4XX_RLGetPhaseCurrents(CPWMC this,Curr_Components* pStator_Currents)
{
  pDVars_t pDVars = &DCLASS_VARS;
  int32_t wAux;
  int16_t hCurrB = 0;
    
  /* Reset the bSOFOC flags to indicate the start of FOC algorithm*/
  pDVars->hFlags &= (~SOFOC);
  
  /* First sampling point */
  wAux = (int32_t)(pDVars->ADCx->JDR1);
  wAux *= 2;
  wAux -= (int32_t)(pDVars->wPhaseOffset);
  
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
  
  hCurrB = -(int16_t)(wAux);
  
  pStator_Currents->qI_Component1 = hCurrB;
  pStator_Currents->qI_Component2 = hCurrB;
}

/**
  * @brief  It turns on low sides switches. This function is intended to be 
  *         used for charging boot capacitors of driving section. It has to be 
  *         called each motor start-up when using high voltage drivers.
  *         This function is specific for RL detection phase.
  * @param  this: related object of class CR3F30X_PWMC
  * @retval none
  */
static void R1F4XX_RLTurnOnLowSides(CPWMC this)
{  
  pDParams_t pDParams = DCLASS_PARAMS;
  TIM_TypeDef* TIMx = DCLASS_PARAMS->TIMx;
  
  /*Turn on the phase A low side switch */
  TIMx->CCR1 = 0u;

  /* Clear Update Flag */
  TIM_ClearFlag(TIMx, (uint16_t)(TIM_FLAG_Update));
  
  /* Wait until next update */
  while (TIM_GetFlagStatus(TIMx,TIM_FLAG_Update)==RESET)
  {}
  
  /* Main PWM Output Enable */
  TIMx->BDTR |= TIM_BDTR_MOE;
  
  if ((pDParams->LowSideOutputs)== ES_GPIO)
  {
    GPIO_WriteBit(pDParams->hCh1NPort, pDParams->hCh1NPin, Bit_SET);
    GPIO_WriteBit(pDParams->hCh2NPort, pDParams->hCh2NPin, Bit_RESET);
    GPIO_WriteBit(pDParams->hCh3NPort, pDParams->hCh3NPin, Bit_RESET);
  }
  return; 
}


/**
* @brief  It enables PWM generation on the proper Timer peripheral
*         This function is specific for RL detection phase.
* @param  this: related object of class CR3F30X_PWMC
* @retval none
*/
static void R1F4XX_RLSwitchOnPWM(CPWMC this)
{
  TIM_TypeDef* TIMx = DCLASS_PARAMS->TIMx;
  pDParams_t pDParams = DCLASS_PARAMS;
  pDVars_t pDVars = &DCLASS_VARS;
  
  /* wait for a new PWM period */
  TIM_ClearFlag(TIMx, (uint16_t)(TIM_FLAG_Update));
  while (TIM_GetFlagStatus(TIMx,TIM_FLAG_Update) == RESET)
  {}
  /* Clear Update Flag */
  TIM_ClearFlag(TIMx, (uint16_t)(TIM_FLAG_Update));
  
  TIMx->CCR1 = 0u;
  
  while (TIM_GetFlagStatus(TIMx,TIM_FLAG_Update) == RESET)
  {}
  
  /* Main PWM Output Enable */
  TIM_CtrlPWMOutputs(TIMx, ENABLE);
  
  if ((pDParams->LowSideOutputs)== ES_GPIO)
  {
    if ((TIMx->CCER & TIMxCCER_MASK_CH123) != 0u)
    {
      GPIO_WriteBit(pDParams->hCh1NPort, pDParams->hCh1NPin, Bit_SET);
      GPIO_WriteBit(pDParams->hCh2NPort, pDParams->hCh2NPin, Bit_SET);
      GPIO_WriteBit(pDParams->hCh3NPort, pDParams->hCh3NPin, Bit_RESET);
    }
    else
    {
      /* It is executed during calibration phase the EN signal shall stay off */
      GPIO_WriteBit(pDParams->hCh1NPort, pDParams->hCh1NPin, Bit_RESET);
      GPIO_WriteBit(pDParams->hCh2NPort, pDParams->hCh2NPin, Bit_RESET);
      GPIO_WriteBit(pDParams->hCh3NPort, pDParams->hCh3NPin, Bit_RESET);
    }
  }
  
  /* Preload Enable */
  TIMx->CCMR1 |= CCMR1_PRELOAD_ENABLE_MASK;
  TIMx->CCMR2 |= CCMR2_PRELOAD_ENABLE_MASK;
  
  /* ADC Injected conversions configuration */     
  ADC_InjectedSequencerLengthConfig(pDVars->ADCx,1u);
    
  TIM_SelectOutputTrigger(pDVars->TIMx_2, TIM_TRGOSource_Update);
      
  /* Enabling the Injectec conversion for ADCx*/
  /* Stdlib replaced: ADC_ExternalTrigInjectedConvCmd(pDVars_str->ADCx,ENABLE); */
  pDVars->ADCx->CR2 |= CR2_JEXTTRIG_Set;
  
  return; 
}

/**
* @brief  It disables PWM generation on the proper Timer peripheral acting on 
*         MOE bit, disables the single shunt distortion and reset the TIM status
* @param  this related object of class CPWMC
* @retval none
*/
static void R1F4XX_RLSwitchOffPWM(CPWMC this)
{
  uint16_t hAux;
  pDVars_t pDVars_str;
  pDParams_t pDParams_str;
  
  pDVars_str =   &(((_DCR1F4XX_PWMC)(((_CPWMC) this)->DerivedClass))->DVars_str);
  pDParams_str =  ((_DCR1F4XX_PWMC)(((_CPWMC) this)->DerivedClass))->pDParams_str;
  
  /* Main PWM Output Disable */
  TIM_CtrlPWMOutputs(pDParams_str->TIMx, DISABLE);
  if ((pDParams_str->LowSideOutputs)== ES_GPIO)
  {
    GPIO_WriteBit(pDParams_str->hCh1NPort, pDParams_str->hCh1NPin, Bit_RESET);
    GPIO_WriteBit(pDParams_str->hCh2NPort, pDParams_str->hCh2NPin, Bit_RESET);
    GPIO_WriteBit(pDParams_str->hCh3NPort, pDParams_str->hCh3NPin, Bit_RESET);
  }
  
  /* Disable UPDATE ISR */
  TIM_ITConfig(pDParams_str->TIMx, TIM_IT_Update, DISABLE);
    
  /* Disabling distortion for single */
  pDVars_str->hFlags &= (~DSTEN);
  
  while (TIM_GetFlagStatus(pDParams_str->TIMx,TIM_FLAG_Update)==RESET)
  {
    if (pDParams_str->TIMx->DIER & TIM_IT_Update)
    { break;}
  }
  
  /* Disabling the Injectec conversion for ADCx after EOC*/
  /* Stdlib replaced: ADC_ExternalTrigInjectedConvCmd(pDVars_str->ADCx,DISABLE);*/
  pDVars_str->ADCx->CR2 &= CR2_JEXTTRIG_Reset;
    
  /* Disabling all DMA previous setting */
  TIM_DMACmd(pDParams_str->TIMx, TIM_DMA_CC4, DISABLE);  
  
  /* Set all duty to 50% */
  hAux = PWM_PERIOD >> 1u;
  pDParams_str->TIMx->CCR1 = (uint32_t)(hAux);
  pDParams_str->TIMx->CCR2 = (uint32_t)(hAux);
  pDParams_str->TIMx->CCR3 = (uint32_t)(hAux);    
  
  /* ADC Injected conversions configuration */     
  ADC_InjectedSequencerLengthConfig(pDVars_str->ADCx,2u);
  
  ADC_InjectedChannelConfig(pDVars_str->ADCx,
  pDParams_str->hIChannel, 1u, pDParams_str->b_ISamplingTime);
  ADC_InjectedChannelConfig(pDVars_str->ADCx,
  pDParams_str->hIChannel, 2u, pDParams_str->b_ISamplingTime);
  
  if (pDVars_str->TIMx_2 == TIM4)
  {
    TIM_SelectOutputTrigger(pDVars_str->TIMx_2, TIM_TRGOSource_OC2Ref);
  }
  if (pDVars_str->TIMx_2 == TIM5)
  {
    TIM_SelectOutputTrigger(pDVars_str->TIMx_2, TIM_TRGOSource_OC4Ref);
  }
    
  return; 
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
