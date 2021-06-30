/**
  ******************************************************************************
  * @file    R1_F30X_PWMnCurrFdbkClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains implementation of current sensor class to be
  *          instantiated when a single shunt current sensing topology is 
  *          used.
  *          It is specifically designed for STM32F30x microcontrollers.
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
#include "R1_F30X_PWMnCurrFdbkClass.h"
#include "R1_F30X_PWMnCurrFdbkPrivate.h"
#include "MCIRQHandlerClass.h"
#include "MCIRQHandlerPrivate.h"
#include "MCLibraryConf.h"
#include "MCLibraryISRPriorityConf.h"
#include "MC_type.h"

#define TIMxCCER_MASK_CH123        ((uint32_t)  0x00000555u)

#define CC12_PRELOAD_ENABLE_MASK 0x0808u
#define CC3_PRELOAD_ENABLE_MASK 0x0008u

#define CC1_PRELOAD_DISABLE_MASK 0xFFFFFFF7u
#define CC2_PRELOAD_DISABLE_MASK 0xFFFFF7FFu
#define CC3_PRELOAD_DISABLE_MASK 0xFFFFFFF7u

#define TIMxCCR56_PRELOAD_DISABLE_MASK 0xFFFFF7F7u
#define TIMxCCR56_PRELOAD_ENABLE_MASK 0x808u

#define NB_CONVERSIONS 16u

#define CLASS_VARS   ((_CPWMC)this)->Vars_str
#define DCLASS_PARAMS ((_DCR1F30X_PWMC)(((_CPWMC) this)->DerivedClass))->pDParams_str
#define DCLASS_VARS  ((_DCR1F30X_PWMC)(((_CPWMC) this)->DerivedClass))->DVars_str

#define PWM_PERIOD pLocalVars_Str->Half_PWMPeriod

/* DEFINES FOR SINGLE SHUNT*/

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

/* Constant values -----------------------------------------------------------*/
static const uint8_t REGULAR_SAMP_CUR1[6] = {SAMP_NIC,SAMP_NIC,SAMP_NIA,SAMP_NIA,SAMP_NIB,SAMP_NIB};
static const uint8_t REGULAR_SAMP_CUR2[6] = {SAMP_IA,SAMP_IB,SAMP_IB,SAMP_IC,SAMP_IC,SAMP_IA};
static const uint8_t BOUNDR1_SAMP_CUR2[6] = {SAMP_IB,SAMP_IB,SAMP_IC,SAMP_IC,SAMP_IA,SAMP_IA};
static const uint8_t BOUNDR2_SAMP_CUR1[6] = {SAMP_IA,SAMP_IB,SAMP_IB,SAMP_IC,SAMP_IC,SAMP_IA};
static const uint8_t BOUNDR2_SAMP_CUR2[6] = {SAMP_IC,SAMP_IA,SAMP_IA,SAMP_IB,SAMP_IB,SAMP_IC};

/* END */

#ifdef MC_CLASS_DYNAMIC
	#include "stdlib.h" /* Used for dynamic allocation */
#else
	_DCR1F30X_PWMC_t R1F30X_PWMCpool[MAX_DRV_PWMC_NUM];
	unsigned char R1F30X_PWMC_Allocated = 0u;
#endif

/** 
  * @brief  BDTR structure definition 
  * @note   This extend the STD lib structure to set also
	*         BKIN2 enable/disable
	*         BKIN2 polarity
	*         BKIN  Filter
	*         BKIN2 Filter
  *         It must be used with the fucntion TIM_BDTRConfig_MC
  */
typedef struct
{

  uint16_t TIM_OSSRState;        /*!< Specifies the Off-State selection used in Run mode.
                                      This parameter can be a value of @ref TIM_OSSR_Off_State_Selection_for_Run_mode_state */

  uint16_t TIM_OSSIState;        /*!< Specifies the Off-State used in Idle state.
                                      This parameter can be a value of @ref TIM_OSSI_Off_State_Selection_for_Idle_mode_state */

  uint16_t TIM_LOCKLevel;        /*!< Specifies the LOCK level parameters.
                                      This parameter can be a value of @ref TIM_Lock_level */ 

  uint16_t TIM_DeadTime;         /*!< Specifies the delay time between the switching-off and the
                                      switching-on of the outputs.
                                      This parameter can be a number between 0x00 and 0xFF  */

  uint32_t TIM_Break;            /*!< Specifies whether the TIM Break input is enabled or not. 
                                      This parameter can be a value of @ref TIM_Break1_Input_enable_disable */

  uint32_t TIM_BreakPolarity;    /*!< Specifies the TIM Break Input pin polarity.
                                      This parameter can be a value of @ref TIM_Break_Polarity */

  uint16_t TIM_AutomaticOutput;  /*!< Specifies whether the TIM Automatic Output feature is enabled or not. 
                                      This parameter can be a value of @ref TIM_AOE_Bit_Set_Reset */
  uint32_t TIM_Break2;           /*!< Specifies whether the TIM Break input is enabled or not. 
                                      This parameter can be a value of @ref TIM_Break2_Input_enable_disable */
  uint32_t TIM_Break2Polarity;   /*!< specifies the Break2 polarity.
                                      This parameter can be one of the following values:
                                      @arg TIM_Break2Polarity_Low: Break2 input is active low
                                      @arg TIM_Break2Polarity_High: Break2 input is active high */
  uint8_t TIM_Break1Filter;      /*!< Specifies the Break1 filter value.
                                      This parameter must be a value between 0x00 and 0x0F */
  uint8_t TIM_Break2Filter;      /*!< Specifies the Break2 filter value.
                                      This parameter must be a value between 0x00 and 0x0F */
} TIM_BDTRInitTypeDef_MC;

/* These function overloads the TIM_BDTRConfig and TIM_BDTRStructInit
   of the standard library */
static void TIM_BDTRConfig_MC(TIM_TypeDef* TIMx, TIM_BDTRInitTypeDef_MC *TIM_BDTRInitStruct);
static void TIM_BDTRStructInit_MC(TIM_BDTRInitTypeDef_MC* TIM_BDTRInitStruct);

static void* R1F30X_IRQHandler(void *this, unsigned char flag);
static void R1F30X_Init(CPWMC this);
static void R1F30X_TIMxInit(TIM_TypeDef* TIMx,CPWMC this);
static void R1F30X_CurrentReadingCalibration(CPWMC this);
static void R1F30X_GetPhaseCurrents(CPWMC this,Curr_Components* pStator_Currents);
static void R1F30X_TurnOnLowSides(CPWMC this);
static void R1F30X_SwitchOnPWM(CPWMC this);
static void R1F30X_SwitchOffPWM(CPWMC this);
static void R1F30X_1ShuntMotorVarsInit(CPWMC this);
static void R1F30X_1ShuntMotorVarsRestart(CPWMC this);
static uint16_t R1F30X_CalcDutyCycles(CPWMC this);
static uint16_t R1F30X_ExecRegularConv(CPWMC this, uint8_t bChannel);
static void R1F30X_ADC_SetSamplingTime(CPWMC this, ADConv_t ADConv_struct);
static void R1F30X_HFCurrentsCalibration(CPWMC this,Curr_Components* pStator_Currents);
static void R1F30X_CurrentReadingCalibration(CPWMC this);
static uint16_t R1F30X_IsOverCurrentOccurred(CPWMC this);
static void R1F30X_SetAOReferenceVoltage(uint32_t DAC_Channel, uint16_t hDACVref);
static uint16_t F30X_GPIOPin2Source(uint16_t GPIO_Pin);
static void R1F30X_RLDetectionModeEnable(CPWMC this);
static void R1F30X_RLDetectionModeDisable(CPWMC this);
static uint16_t R1F30X_RLDetectionModeSetDuty(CPWMC this, uint16_t hDuty);
static void R1F30X_RLGetPhaseCurrents(CPWMC this,Curr_Components* pStator_Currents);
static void R1F30X_RLTurnOnLowSides(CPWMC this);
static void R1F30X_RLSwitchOnPWM(CPWMC this);
static void R1F30X_RLSwitchOffPWM(CPWMC this);

/**
  * @brief  Creates an object of the class R1_F30X
  * @param  pPWMnCurrFdbkParams pointer to an PWMnCurrFdbk parameters structure
  * @param  pR1_DDParams pointer to an R1_DD parameters structure
  * @retval CR1F30X_PWMC new instance of R1_F30X object
  */
CR1F30X_PWMC R1F3XX_NewObject(pPWMnCurrFdbkParams_t pPWMnCurrFdbkParams, pR1_F30XParams_t pR1_F30XParams)
{
	_CPWMC _oPWMnCurrFdbk;
	_DCR1F30X_PWMC _oR1_F30X;

	_oPWMnCurrFdbk = (_CPWMC)PWMC_NewObject(pPWMnCurrFdbkParams);

	#ifdef MC_CLASS_DYNAMIC
		_oR1_F30X = (_DCR1F30X_PWMC)calloc(1u,sizeof(_DCR1F30X_PWMC_t));
	#else
		if (R1F30X_PWMC_Allocated  < MAX_DRV_PWMC_NUM)
		{
			_oR1_F30X = &R1F30X_PWMCpool[R1F30X_PWMC_Allocated++];
		}
		else
		{
			_oR1_F30X = MC_NULL;
		}
	#endif
  
  _oR1_F30X->pDParams_str = pR1_F30XParams;
  _oPWMnCurrFdbk->DerivedClass = (void*)_oR1_F30X;

  _oPWMnCurrFdbk->Methods_str.pIRQ_Handler = &R1F30X_IRQHandler;

  Set_IRQ_Handler(pR1_F30XParams->IRQnb, (_CMCIRQ)_oPWMnCurrFdbk);
  
  _oPWMnCurrFdbk->Methods_str.pPWMC_Init = &R1F30X_Init;
  _oPWMnCurrFdbk->Methods_str.pPWMC_GetPhaseCurrents = &R1F30X_GetPhaseCurrents;
  _oPWMnCurrFdbk->Methods_str.pPWMC_SwitchOffPWM = &R1F30X_SwitchOffPWM;
  _oPWMnCurrFdbk->Methods_str.pPWMC_SwitchOnPWM = &R1F30X_SwitchOnPWM;        
  _oPWMnCurrFdbk->Methods_str.pPWMC_CurrentReadingCalibr = 
                                                 &R1F30X_CurrentReadingCalibration;         
  _oPWMnCurrFdbk->Methods_str.pPWMC_TurnOnLowSides = &R1F30X_TurnOnLowSides;         
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect1 = 
                                                      &R1F30X_CalcDutyCycles;        
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect2 = 
                                                      &R1F30X_CalcDutyCycles; 
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect3 = 
                                                      &R1F30X_CalcDutyCycles;        
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect4 = 
                                                      &R1F30X_CalcDutyCycles;         
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect5 = 
                                                      &R1F30X_CalcDutyCycles;        
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect6 = 
                                                      &R1F30X_CalcDutyCycles;
  _oPWMnCurrFdbk->Methods_str.pPWMC_ExecRegularConv= &R1F30X_ExecRegularConv;
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetSamplingTime= &R1F30X_ADC_SetSamplingTime;
  _oPWMnCurrFdbk->Methods_str.pPWMC_IsOverCurrentOccurred = 
    &R1F30X_IsOverCurrentOccurred;
  
  _oPWMnCurrFdbk->Methods_str.pRLDetectionModeEnable = &R1F30X_RLDetectionModeEnable;
  
  _oPWMnCurrFdbk->Methods_str.pRLDetectionModeDisable = &R1F30X_RLDetectionModeDisable;
  
  _oPWMnCurrFdbk->Methods_str.pRLDetectionModeSetDuty = &R1F30X_RLDetectionModeSetDuty;
  
  /* Deinit of ADC to be done here to avoid that the second motor deletes the setting done in the init of the first motor. */
  ADC_DeInit(pR1_F30XParams->ADCx);
  ADC_DeInit(pR1_F30XParams->regconvADCx);
  
  return ((CR1F30X_PWMC)_oPWMnCurrFdbk);
}

/** @addtogroup STM32F10x_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup PWMnCurrFdbk_R1_F30X
  * @{
  */

/** @defgroup R1_F30X_class_private_methods R1_F30X class private methods
* @{
*/

/**
* @brief  It initializes TIMx, ADC, GPIO, DMA1 and NVIC for current reading 
*         in ICS configuration using STM32F103x High Density
* @param  this: related object of class CR1F30X_PWMC
* @retval none
*/
static void R1F30X_Init(CPWMC this)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  ADC_InitTypeDef ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  pVars_t pVars_str = &CLASS_VARS;
  pDVars_t pDVars_str = &DCLASS_VARS;  
  pDParams_t pDParams_str = DCLASS_PARAMS; 
  pR1_F30XOPAMPParams_t pDOPAMPParams_str = pDParams_str->pOPAMPParams;
  pF30XCOMPParams_t pDOCP_COMPParams_str = pDParams_str->pOCP_COMPParams;
  pF30XCOMPParams_t pDOVP_COMPParams_str = pDParams_str->pOVP_COMPParams;
  ADC_TypeDef* ADCx = pDParams_str->ADCx;
  ADC_InjectedInitTypeDef ADC_InjectedInitStructure;
  DMA_InitTypeDef DMA_InitStructure;
  
  R1F30X_1ShuntMotorVarsInit(this);
  
  pDVars_str->ADCx = ADCx;
  pDVars_str->TIMx = pDParams_str->TIMx;
  pVars_str->bMotor = (pDParams_str->bInstanceNbr==1u?M1:M2);
  
  /* Peripheral clocks enabling ---------------------------------------------*/
  
  RCC->AHBENR |= RCC_AHBPeriph_CRC;
  
  /* ADC Periph clock enable */ 
  RCC_AHBPeriphClockCmd(pDParams_str->wAHBPeriph, ENABLE);
  
  /* Enable GPIOA-GPIOI clock */
  RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA | 
                         RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC | 
                           RCC_AHBPeriph_GPIOD | RCC_AHBPeriph_GPIOE | 
                             RCC_AHBPeriph_GPIOF, ENABLE);     
  
  /* Enable the CCS */
  RCC_ClockSecuritySystemCmd((FunctionalState)(ENABLE));
  
  /* Enable TIM1 - TIM8 clock */
  if(pDParams_str->TIMx == TIM1)
  {
    /* Enable TIM1 clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    /* Clock source is APB high speed clock*/
    RCC_TIMCLKConfig(RCC_TIM1CLK_HCLK);
    
    /* Enable DMA1 clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);    
    
    /* DMA Event related to TIM1 Channel 4 */
    /* DMA1 Channel4 configuration ----------------------------------------------*/
    DMA_DeInit(DMA1_Channel4);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(TIM1->CCR1)); /*dummy*/
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
    pDVars_str->DistortionDMAy_Chx = DMA1_Channel4;
    
    /* DMA Event related to TIM1 update */
    /* DMA1 Channel5 configuration ----------------------------------------------*/
    DMA_DeInit(DMA1_Channel5);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(TIM1->CCMR1)); /*dummy*/
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(&(pDVars_str->wPreloadDisableActing));
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = 1u;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);
    /* Disable DMA1 Channel5 */
    DMA_Cmd(DMA1_Channel5, DISABLE);
    pDVars_str->PreloadDMAy_Chx = DMA1_Channel5;    
  }
  else
  {
    /* Enable TIM8 clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
		/* Clock source is APB high speed clock*/
		RCC_TIMCLKConfig(RCC_TIM8CLK_HCLK);
                
    /* Enable DMA2 clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);    
    
    /* DMA Event related to TIM8 Channel 4 */
    /* DMA2 Channel2 configuration ----------------------------------------------*/
    DMA_DeInit(DMA2_Channel2);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(TIM8->CCR1));
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
    DMA_Init(DMA2_Channel2, &DMA_InitStructure);
    /* Disable DMA2 Channel2 */
    DMA_Cmd(DMA2_Channel2, ENABLE);
    pDVars_str->DistortionDMAy_Chx = DMA2_Channel2;
    
    /* DMA Event related to TIM8 update */
    /* DMA2 Channel1 configuration ----------------------------------------------*/
    DMA_DeInit(DMA2_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(TIM8->CCMR1)); /*dummy*/
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(&(pDVars_str->wPreloadDisableActing));
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = 1u;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA2_Channel1, &DMA_InitStructure);
    /* Disable DMA2 Channel1 */
    DMA_Cmd(DMA2_Channel1, DISABLE);
    pDVars_str->PreloadDMAy_Chx = DMA2_Channel1;
  }
	
	R1F30X_TIMxInit(pDParams_str->TIMx, this);
  
  /* GPIOs configurations --------------------------------------------------*/
  GPIO_StructInit(&GPIO_InitStructure);
  
  if (!pDOPAMPParams_str)
  {  
  /****** Configure phase ADC channel GPIO as analog input ****/
    GPIO_InitStructure.GPIO_Pin = pDParams_str->hIPin;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_Init(pDParams_str->hIPort,
              &GPIO_InitStructure);
    GPIO_PinLockConfig(pDParams_str->hIPort, pDParams_str->hIPin);
  }
  
  /****** Configure TIMx Channel 1, 2 and 3 Outputs ******/ 
  GPIO_PinAFConfig(pDParams_str->hCh1Port, F30X_GPIOPin2Source(pDParams_str->hCh1Pin), pDParams_str->bCh1AF);
  GPIO_PinAFConfig(pDParams_str->hCh2Port, F30X_GPIOPin2Source(pDParams_str->hCh2Pin), pDParams_str->bCh2AF);
  GPIO_PinAFConfig(pDParams_str->hCh3Port, F30X_GPIOPin2Source(pDParams_str->hCh3Pin), pDParams_str->bCh3AF);
  
  GPIO_StructInit(&GPIO_InitStructure);
  
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
    GPIO_PinAFConfig(pDParams_str->hCh1NPort, F30X_GPIOPin2Source(pDParams_str->hCh1NPin), pDParams_str->bCh1NAF);
    GPIO_PinAFConfig(pDParams_str->hCh2NPort, F30X_GPIOPin2Source(pDParams_str->hCh2NPin), pDParams_str->bCh2NAF);
    GPIO_PinAFConfig(pDParams_str->hCh3NPort, F30X_GPIOPin2Source(pDParams_str->hCh3NPin), pDParams_str->bCh3NAF);
    
    GPIO_InitStructure.GPIO_Pin = pDParams_str->hCh1NPin;  
    GPIO_Init(pDParams_str->hCh1NPort, &GPIO_InitStructure);  
    GPIO_InitStructure.GPIO_Pin = pDParams_str->hCh2NPin;  
    GPIO_Init(pDParams_str->hCh2NPort, &GPIO_InitStructure);    
    GPIO_InitStructure.GPIO_Pin = pDParams_str->hCh3NPin;  
    GPIO_Init(pDParams_str->hCh3NPort, &GPIO_InitStructure);
    
    GPIO_PinLockConfig(pDParams_str->hCh1NPort, pDParams_str->hCh1NPin);
    GPIO_PinLockConfig(pDParams_str->hCh2NPort, pDParams_str->hCh2NPin);
    GPIO_PinLockConfig(pDParams_str->hCh3NPort, pDParams_str->hCh3NPin);
  }  
	else if ((pDParams_str->LowSideOutputs)== ES_GPIO)
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
  if ((pDParams_str->bBKINMode) == EXT_MODE)
  {
    GPIO_PinAFConfig(pDParams_str->hBKINPort, F30X_GPIOPin2Source(pDParams_str->hBKINPin), pDParams_str->bBKINAF);
    GPIO_InitStructure.GPIO_Pin = pDParams_str->hBKINPin;  
    GPIO_Init(pDParams_str->hBKINPort, &GPIO_InitStructure); 
    GPIO_PinLockConfig(pDParams_str->hBKINPort, pDParams_str->hBKINPin);
  }
  
  /****** Configure TIMx BKIN2 input, if enabled ******/
  if ((pDParams_str->bBKIN2Mode) == EXT_MODE)
  {
    GPIO_PinAFConfig(pDParams_str->hBKIN2Port, F30X_GPIOPin2Source(pDParams_str->hBKIN2Pin), pDParams_str->bBKIN2AF);
    GPIO_InitStructure.GPIO_Pin = pDParams_str->hBKIN2Pin;  
    GPIO_Init(pDParams_str->hBKIN2Port, &GPIO_InitStructure); 
    GPIO_PinLockConfig(pDParams_str->hBKIN2Port, pDParams_str->hBKIN2Pin);
  }
  
  if (pDOPAMPParams_str)
  {
    OPAMP_InitTypeDef OPAMP_InitStruct;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_Pin = pDOPAMPParams_str->hOPAMP_NonInvertingInput_GPIO_PIN;
    GPIO_Init(pDOPAMPParams_str->hOPAMP_NonInvertingInput_GPIO_PORT, &GPIO_InitStructure);
    GPIO_PinLockConfig(pDOPAMPParams_str->hOPAMP_NonInvertingInput_GPIO_PORT,
                       pDOPAMPParams_str->hOPAMP_NonInvertingInput_GPIO_PIN);
    if (pDOPAMPParams_str->bOPAMP_InvertingInput_MODE == EXT_MODE)
    {
      GPIO_InitStructure.GPIO_Pin = pDOPAMPParams_str->hOPAMP_InvertingInput_GPIO_PIN;
      GPIO_Init(pDOPAMPParams_str->hOPAMP_InvertingInput_GPIO_PORT, &GPIO_InitStructure);
      GPIO_PinLockConfig(pDOPAMPParams_str->hOPAMP_InvertingInput_GPIO_PORT,
                         pDOPAMPParams_str->hOPAMP_InvertingInput_GPIO_PIN);
    }
    GPIO_InitStructure.GPIO_Pin = pDOPAMPParams_str->hOPAMP_Output_GPIO_PIN;
    GPIO_Init(pDOPAMPParams_str->hOPAMP_Output_GPIO_PORT, &GPIO_InitStructure);
    GPIO_PinLockConfig(pDOPAMPParams_str->hOPAMP_Output_GPIO_PORT,
                       pDOPAMPParams_str->hOPAMP_Output_GPIO_PIN);
    
    OPAMP_InitStruct.OPAMP_InvertingInput = pDOPAMPParams_str->wOPAMP_InvertingInput;
    OPAMP_InitStruct.OPAMP_NonInvertingInput = pDOPAMPParams_str->wOPAMP_NonInvertingInput;
    OPAMP_Init(pDOPAMPParams_str->wOPAMP_Selection,&OPAMP_InitStruct);
    OPAMP_PGAConfig(pDOPAMPParams_str->wOPAMP_Selection,
                    pDOPAMPParams_str->wOPAMP_PGAGain,
                    pDOPAMPParams_str->OPAMP_PGAConnect);
    OPAMP_Cmd(pDOPAMPParams_str->wOPAMP_Selection,ENABLE);
    
    OPAMP_LockConfig(pDOPAMPParams_str->wOPAMP_Selection);
  }
  
  /* Over current protection */
  if (pDOCP_COMPParams_str)
  {
    COMP_InitTypeDef COMP_InitStruct;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    if (!pDOPAMPParams_str)
    {    
      /* NonInverting input*/
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
      GPIO_InitStructure.GPIO_Pin = pDOCP_COMPParams_str->hNonInvertingInput_GPIO_PIN;
      GPIO_Init(pDOCP_COMPParams_str->hNonInvertingInput_GPIO_PORT, &GPIO_InitStructure);
      GPIO_PinLockConfig(pDOCP_COMPParams_str->hNonInvertingInput_GPIO_PORT,
                         pDOCP_COMPParams_str->hNonInvertingInput_GPIO_PIN);
    }
    
    /* Inverting input*/
    if (pDOCP_COMPParams_str->bInvertingInput_MODE == EXT_MODE)
    {
      GPIO_InitStructure.GPIO_Pin = pDOCP_COMPParams_str->hInvertingInput_GPIO_PIN;
      GPIO_Init(pDOCP_COMPParams_str->hInvertingInput_GPIO_PORT, &GPIO_InitStructure);
      GPIO_PinLockConfig(pDOCP_COMPParams_str->hInvertingInput_GPIO_PORT,
                         pDOCP_COMPParams_str->hInvertingInput_GPIO_PIN);
    }
    else
    {
      if (pDOCP_COMPParams_str->wInvertingInput == COMP_InvertingInput_DAC1OUT1)
      {
        R1F30X_SetAOReferenceVoltage(DAC_Channel_1, (uint16_t)(pDParams_str->hDAC_OCP_Threshold));
      }
      else if (pDOCP_COMPParams_str->wInvertingInput == COMP_InvertingInput_DAC1OUT2)
      {
        R1F30X_SetAOReferenceVoltage(DAC_Channel_2, (uint16_t)(pDParams_str->hDAC_OCP_Threshold));
      }
      else
      {
      }
    }
    
    /* Wait to stabilize DAC voltage */
    {
      volatile uint16_t waittime = 0u;
      for(waittime=0u;waittime<1000u;waittime++)
      {
      }
    }    
    
    /* Output */
    if (pDOCP_COMPParams_str->bOutput_MODE == EXT_MODE)
    {
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
      GPIO_InitStructure.GPIO_Pin = pDOCP_COMPParams_str->hOutput_GPIO_PIN;
      GPIO_Init(pDOCP_COMPParams_str->hOutput_GPIO_PORT, &GPIO_InitStructure);
      GPIO_PinAFConfig(pDOCP_COMPParams_str->hOutput_GPIO_PORT,
                       F30X_GPIOPin2Source(pDOCP_COMPParams_str->hOutput_GPIO_PIN),
                       pDOCP_COMPParams_str->bOutput_GPIO_AF);
      GPIO_PinLockConfig(pDOCP_COMPParams_str->hOutput_GPIO_PORT,
                         pDOCP_COMPParams_str->hOutput_GPIO_PIN);
    }
    
    COMP_InitStruct.COMP_InvertingInput = pDOCP_COMPParams_str->wInvertingInput;
    COMP_InitStruct.COMP_NonInvertingInput = pDOCP_COMPParams_str->wNonInvertingInput;
    COMP_InitStruct.COMP_Output = pDOCP_COMPParams_str->wOutput;
    COMP_InitStruct.COMP_OutputPol = pDOCP_COMPParams_str->wOutputPol;
    COMP_InitStruct.COMP_BlankingSrce = COMP_BlankingSrce_None; 
    COMP_InitStruct.COMP_Hysteresis = COMP_Hysteresis_Low;
    COMP_InitStruct.COMP_Mode = pDOCP_COMPParams_str->wMode;
    COMP_Init(pDOCP_COMPParams_str->wSelection,&COMP_InitStruct);
    COMP_Cmd(pDOCP_COMPParams_str->wSelection,ENABLE);
    COMP_LockConfig(pDOCP_COMPParams_str->wSelection);
  }
  
  /* Over voltage protection */
  if (pDOVP_COMPParams_str)
  {
    COMP_InitTypeDef COMP_InitStruct;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);    
    
    /* NonInverting input*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_Pin = pDOVP_COMPParams_str->hNonInvertingInput_GPIO_PIN;
    GPIO_Init(pDOVP_COMPParams_str->hNonInvertingInput_GPIO_PORT, &GPIO_InitStructure);
    GPIO_PinLockConfig(pDOVP_COMPParams_str->hNonInvertingInput_GPIO_PORT,
                       pDOVP_COMPParams_str->hNonInvertingInput_GPIO_PIN);
    
    /* Inverting input*/
    if (pDOVP_COMPParams_str->bInvertingInput_MODE == EXT_MODE)
    {
      GPIO_InitStructure.GPIO_Pin = pDOVP_COMPParams_str->hInvertingInput_GPIO_PIN;
      GPIO_Init(pDOVP_COMPParams_str->hInvertingInput_GPIO_PORT, &GPIO_InitStructure);
      GPIO_PinLockConfig(pDOVP_COMPParams_str->hInvertingInput_GPIO_PORT,
                         pDOVP_COMPParams_str->hInvertingInput_GPIO_PIN);
    }
    else
    {
      if (pDOVP_COMPParams_str->wInvertingInput == COMP_InvertingInput_DAC1OUT1)
      {
        R1F30X_SetAOReferenceVoltage(DAC_Channel_1, (uint16_t)(pDParams_str->hDAC_OVP_Threshold));
      }
      else if (pDOVP_COMPParams_str->wInvertingInput == COMP_InvertingInput_DAC1OUT2)
      {
        R1F30X_SetAOReferenceVoltage(DAC_Channel_2, (uint16_t)(pDParams_str->hDAC_OVP_Threshold));
      }
      else
      {
      }
    }

    /* Wait to stabilize DAC voltage */
    {
      volatile uint16_t waittime = 0u;
      for(waittime=0u;waittime<1000u;waittime++)
      {
      }
    }    
    
    /* Output */
    if (pDOVP_COMPParams_str->bOutput_MODE == EXT_MODE)
    {
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
      GPIO_InitStructure.GPIO_Pin = pDOVP_COMPParams_str->hOutput_GPIO_PIN;
      GPIO_Init(pDOVP_COMPParams_str->hOutput_GPIO_PORT, &GPIO_InitStructure);
      GPIO_PinAFConfig(pDOVP_COMPParams_str->hOutput_GPIO_PORT,
                       F30X_GPIOPin2Source(pDOVP_COMPParams_str->hOutput_GPIO_PIN),
                       pDOVP_COMPParams_str->bOutput_GPIO_AF);
      GPIO_PinLockConfig(pDOVP_COMPParams_str->hOutput_GPIO_PORT,
                         pDOVP_COMPParams_str->hOutput_GPIO_PIN);
    }
    
    COMP_InitStruct.COMP_InvertingInput = pDOVP_COMPParams_str->wInvertingInput;
    COMP_InitStruct.COMP_NonInvertingInput = pDOVP_COMPParams_str->wNonInvertingInput;
    COMP_InitStruct.COMP_Output = pDOVP_COMPParams_str->wOutput;
    COMP_InitStruct.COMP_OutputPol = pDOVP_COMPParams_str->wOutputPol;
    COMP_InitStruct.COMP_BlankingSrce = COMP_BlankingSrce_None; 
    COMP_InitStruct.COMP_Hysteresis = COMP_Hysteresis_Low;
    COMP_InitStruct.COMP_Mode = pDOVP_COMPParams_str->wMode;
    COMP_Init(pDOVP_COMPParams_str->wSelection,&COMP_InitStruct);
    COMP_Cmd(pDOVP_COMPParams_str->wSelection,ENABLE);
    COMP_LockConfig(pDOVP_COMPParams_str->wSelection);
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
  
  /* ADC initializations */
  
  if (pDParams_str->bInstanceNbr == 1u)
  {
    if (pDParams_str->regconvADCx != ADCx)
    {
      if ((pDParams_str->regconvADCx == ADC1) ||
          (pDParams_str->regconvADCx == ADC2))
      {
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);
        
        /* Common init */
        ADC_CommonStructInit(&ADC_CommonInitStructure);
        ADC_CommonInitStructure.ADC_Mode=ADC_Mode_Independent;
        ADC_CommonInitStructure.ADC_Clock=pDParams_str->wADC_Clock_Divider;
        ADC_CommonInitStructure.ADC_DMAAccessMode=ADC_DMAAccessMode_Disabled;
        ADC_CommonInitStructure.ADC_DMAMode=ADC_DMAMode_OneShot;
        ADC_CommonInitStructure.ADC_TwoSamplingDelay= 0u;
        ADC_CommonInit(ADC1, &ADC_CommonInitStructure);        
      }
      
      else if ((pDParams_str->regconvADCx == ADC3) ||
               (pDParams_str->regconvADCx == ADC4))
      {
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC34, ENABLE);
        
        /* Common init */
        ADC_CommonStructInit(&ADC_CommonInitStructure);
        ADC_CommonInitStructure.ADC_Mode=ADC_Mode_Independent;
        ADC_CommonInitStructure.ADC_Clock=pDParams_str->wADC_Clock_Divider;
        ADC_CommonInitStructure.ADC_DMAAccessMode=ADC_DMAAccessMode_Disabled;
        ADC_CommonInitStructure.ADC_DMAMode=ADC_DMAMode_OneShot;
        ADC_CommonInitStructure.ADC_TwoSamplingDelay= 0u;
        ADC_CommonInit(ADC3, &ADC_CommonInitStructure);        
      }
      else
      {
      }
    }  
  }
  
  ADC_CommonStructInit(&ADC_CommonInitStructure);
  ADC_CommonInitStructure.ADC_Mode=ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Clock=pDParams_str->wADC_Clock_Divider;
  ADC_CommonInitStructure.ADC_DMAAccessMode=ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_DMAMode=ADC_DMAMode_OneShot;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0u;

#if defined(STM32F302x8)
  NVIC_InitStructure.NVIC_IRQChannel = (uint8_t) ADC1_IRQn;
  ADC_CommonInit(ADC1, &ADC_CommonInitStructure);
#else
  /* Enable the ADC Interrupt */
  if (ADCx == ADC3)
  {
    NVIC_InitStructure.NVIC_IRQChannel = (uint8_t) ADC3_IRQn;
    ADC_CommonInit(ADC3, &ADC_CommonInitStructure); /*ADC3 here stands for ADC3&4*/
  }
  else if (ADCx == ADC4)
  {
    NVIC_InitStructure.NVIC_IRQChannel = (uint8_t) ADC4_IRQn;
    ADC_CommonInit(ADC3, &ADC_CommonInitStructure); /*ADC3 here stands for ADC3&4*/
  }
  else
  {
    NVIC_InitStructure.NVIC_IRQChannel = (uint8_t) ADC1_2_IRQn;
    ADC_CommonInit(ADC1, &ADC_CommonInitStructure); /*ADC1 here stands for ADC1&2*/ 
  }
  #endif
  
  ADC_VoltageRegulatorCmd(ADCx, ENABLE);  
  
  /* Wait for Regulator Startup time, once for both */
  {
    uint16_t waittime = 0u;
    for(waittime=0u;waittime<65000u;waittime++)
    {
    }
  }    
    
  ADC_SelectCalibrationMode(ADCx,ADC_CalibrationMode_Single);    
  ADC_StartCalibration(ADCx);
  while (ADC_GetCalibrationStatus(ADCx)== SET )
  {
  }
  
  if (pDParams_str->bInstanceNbr == 1u)
  {
    if (pDParams_str->regconvADCx != ADCx)
    {
      ADC_VoltageRegulatorCmd(pDParams_str->regconvADCx, ENABLE);  
      
      /* Wait for Regulator Startup time, once for both */
      {
        uint16_t waittime = 0u;
        for(waittime=0u;waittime<65000u;waittime++)
        {
        }
      }    
      
      ADC_SelectCalibrationMode(pDParams_str->regconvADCx,ADC_CalibrationMode_Single);    
      ADC_StartCalibration(pDParams_str->regconvADCx);
      while (ADC_GetCalibrationStatus(pDParams_str->regconvADCx)== SET )
      {
      }
    }
  }
  
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 
    ADC_PRE_EMPTION_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = ADC_SUB_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 
    
  /* ADC registers configuration ---------------------------------*/
  /* Enable ADC*/
  ADC_Cmd(ADCx, ENABLE);
  
  if (pDParams_str->bInstanceNbr == 1u)
  {
    if (pDParams_str->regconvADCx != ADCx)
    {
      ADC_Cmd(pDParams_str->regconvADCx, ENABLE);
      
      /* Configure the ADCx for reg conversions */
      ADC_StructInit(&ADC_InitStructure);
      ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Disable;
      ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; 
      ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0; /*dummy*/
      ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;    
      ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Left;
      ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;
      ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;
      ADC_InitStructure.ADC_NbrOfRegChannel = 1u;    
      ADC_Init(pDParams_str->regconvADCx, &ADC_InitStructure);    
    }
  }
    
  /* Configure the ADCx for reg conversions */
  ADC_StructInit(&ADC_InitStructure);
  ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Disable;
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; 
  ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0; /*dummy*/
  ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;    
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Left;
  ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;
  ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;
  ADC_InitStructure.ADC_NbrOfRegChannel = 1u;    
  ADC_Init(ADCx, &ADC_InitStructure);
  
  /* Configure the ADCx for injected conversions */
  ADC_InjectedStructInit(&ADC_InjectedInitStructure);  
  if(pDParams_str->TIMx == TIM1)
  {   
    /* ADC inj conv trig comes from TIM1_TRGO2 */
    ADC_InjectedInitStructure.ADC_ExternalTrigInjecConvEvent = ADC_ExternalTrigInjecConvEvent_8;
  }
  else
  {
    /* ADC inj conv trig comes from TIM8_TRGO2 */
    ADC_InjectedInitStructure.ADC_ExternalTrigInjecConvEvent = ADC_ExternalTrigInjecConvEvent_10;
  }  
  ADC_InjectedInitStructure.ADC_ExternalTrigInjecEventEdge = ADC_ExternalTrigInjecEventEdge_RisingEdge;     
  ADC_InjectedInitStructure.ADC_NbrOfInjecChannel = 2u;                                                             
  ADC_InjectedInitStructure.ADC_InjecSequence1 = pDParams_str->bIChannel; 
  ADC_InjectedInitStructure.ADC_InjecSequence2 = pDParams_str->bIChannel;
  ADC_InjectedInitStructure.ADC_InjecSequence3 = pDParams_str->bIChannel;
  ADC_InjectedInitStructure.ADC_InjecSequence4 = pDParams_str->bIChannel;  
  ADC_InjectedInit(ADCx,&ADC_InjectedInitStructure);
  
  ADC_InjectedChannelSampleTimeConfig(ADCx, pDParams_str->bIChannel, pDParams_str->b_ISamplingTime);    
  pDVars_str->wADC_JSQR = ADCx->JSQR;  

  ADC_SelectQueueOfContextMode(ADCx,ENABLE);
  
  ADC_InjectedDiscModeCmd(ADCx,ENABLE);
  
  ADC_StartInjectedConversion(ADCx);
    
  /* ADCx Injected conversions end interrupt enabling */
  ADC_ClearFlag(ADCx, ADC_FLAG_JEOS);
  ADC_ITConfig(ADCx, ADC_IT_JEOS, ENABLE);  
  
  if(pDParams_str->TIMx==TIM1)
  {
    /* Enable the TIM1 BRK interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = (uint8_t) TIM1_BRK_TIM15_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIMx_BRK_PRE_EMPTION_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = TIMx_BRK_SUB_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
  }
  else
  {
#if !defined(STM32F302x8)
    /* Enable the TIM8 BRK interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = (uint8_t) TIM8_BRK_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIMx_BRK_PRE_EMPTION_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = TIMx_BRK_SUB_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif
  }
  
  /* Clear the flags */
  pDVars_str->OverVoltageFlag = FALSE;
  pDVars_str->OverCurrentFlag = FALSE;  
}

/**
* @brief  It initializes TIMx peripheral for PWM generation
* @param 'TIMx': Timer to be initialized
* @param 'this': related object of class CR3F30X_PWMC
* @retval none
*/
static void R1F30X_TIMxInit(TIM_TypeDef* TIMx, CPWMC this)
{
  TIM_TimeBaseInitTypeDef TIMx_TimeBaseStructure;
  TIM_OCInitTypeDef TIMx_OCInitStructure;
  TIM_BDTRInitTypeDef_MC TIMx_BDTRInitStructure;
  pDVars_t pDVars_str = &DCLASS_VARS;  
  pDParams_t pDParams_str =DCLASS_PARAMS;  
  
  /* TIMx Peripheral Configuration -------------------------------------------*/
  /* TIMx Registers reset */
  TIM_DeInit(TIMx);
  TIM_TimeBaseStructInit(&TIMx_TimeBaseStructure);
  /* Time Base configuration */
  TIMx_TimeBaseStructure.TIM_Prescaler = (uint16_t)(pDParams_str->bTim_Clock_Divider) - 1u;
  TIMx_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned3;
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
  if ((pDParams_str-> LowSideOutputs)== LS_PWM_TIMER)
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
  if ((pDParams_str-> LowSideOutputs)== LS_PWM_TIMER)
  {
  TIMx_OCInitStructure.TIM_OCNPolarity = pDParams_str->hCh2NPolarity; 
  TIMx_OCInitStructure.TIM_OCNIdleState = pDParams_str->hCh2NIdleState;         
  }
  TIM_OC2Init(TIMx, &TIMx_OCInitStructure); 
  
  
  /* Channel 3 */
  TIMx_OCInitStructure.TIM_OCPolarity = pDParams_str->hCh3Polarity;      
  TIMx_OCInitStructure.TIM_OCIdleState = pDParams_str->hCh3IdleState;    
  if ((pDParams_str-> LowSideOutputs)== LS_PWM_TIMER)
  {
  TIMx_OCInitStructure.TIM_OCNPolarity = pDParams_str->hCh3NPolarity; 
  TIMx_OCInitStructure.TIM_OCNIdleState = pDParams_str->hCh3NIdleState;         
  }
  TIM_OC3Init(TIMx, &TIMx_OCInitStructure); 

  /* Channel 4 Configuration in PWM mode for active vector*/
  TIMx_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;  
  TIMx_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable; 
  TIMx_OCInitStructure.TIM_Pulse =  (uint32_t)(pDVars_str->Half_PWMPeriod) - pDParams_str->hHTMin;
  TIMx_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
  TIMx_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM_OC4Init(TIMx, &TIMx_OCInitStructure); 
  
    /* Channel 5 for first ADC trigger*/
  TIMx_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;      
  TIMx_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIMx_OCInitStructure.TIM_Pulse = (uint32_t)(pDVars_str->Half_PWMPeriod) + 1u;
  TIM_OC5Init(TIMx, &TIMx_OCInitStructure);
  
    /* Channel 6 for second ADC trigger*/
  TIM_OC6Init(TIMx, &TIMx_OCInitStructure);  
  
  /* Enables the TIMx Preload on CC1 Register */
  TIM_OC1PreloadConfig(TIMx, TIM_OCPreload_Enable);
  /* Enables the TIMx Preload on CC2 Register */
  TIM_OC2PreloadConfig(TIMx, TIM_OCPreload_Enable);
  /* Enables the TIMx Preload on CC3 Register */
  TIM_OC3PreloadConfig(TIMx, TIM_OCPreload_Enable);
  /* Enables the TIMx Preload on CC4 Register */
  TIM_OC4PreloadConfig(TIMx, TIM_OCPreload_Enable);
  /* Enables the TIMx Preload on CC5 Register */
  TIM_OC5PreloadConfig(TIMx, TIM_OCPreload_Enable);
  /* Enables the TIMx Preload on CC6 Register */
  TIM_OC6PreloadConfig(TIMx, TIM_OCPreload_Enable);

  /* TIM output trigger 2 for ADC */
  TIM_SelectOutputTrigger2(TIMx, TIM_TRGO2Source_OC5RefRising_OC6RefRising);
  
  TIM_BDTRStructInit_MC(&TIMx_BDTRInitStructure);
  /* Dead Time */
  TIMx_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
  TIMx_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Disable;
  TIMx_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1; 
  TIMx_BDTRInitStructure.TIM_DeadTime = (pDParams_str->hDeadTime)/2u;
  
  /* Always enable BKIN for safety fature */
  TIMx_BDTRInitStructure.TIM_Break = TIM_Break1_Enable;
  if ((pDParams_str->bBKINMode) == EXT_MODE)
  {
    /* Set from the power stage */
    TIMx_BDTRInitStructure.TIM_BreakPolarity = pDParams_str->hBKINPolarity;
  }
  else
  {
    /* Internal - always active high */
    TIMx_BDTRInitStructure.TIM_BreakPolarity = TIM_Break1Polarity_High;
  }
  TIMx_BDTRInitStructure.TIM_Break1Filter = pDParams_str->bBKINFilter;
  TIMx_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;
  TIM_ClearITPendingBit(TIMx, TIM_IT_Break);
  TIM_ITConfig(TIMx, TIM_IT_Break, ENABLE);
  
  if ((pDParams_str->bBKIN2Mode) != NONE)
  {
    TIMx_BDTRInitStructure.TIM_Break2 = TIM_Break2_Enable;
    if ((pDParams_str->bBKIN2Mode) == EXT_MODE)
    {
      /* Set from the power stage */
      TIMx_BDTRInitStructure.TIM_Break2Polarity = pDParams_str->hBKIN2Polarity;
    }
    else
    {
      /* Internal - always active high */
      TIMx_BDTRInitStructure.TIM_Break2Polarity = TIM_Break2Polarity_High;
    }
    TIMx_BDTRInitStructure.TIM_Break2Filter = pDParams_str->bBKIN2Filter;
    TIMx_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;
    TIMx->SR = ~(((uint32_t)(TIM_IT_Break)<<1)); /* Clear BKIN2F. Not possible with stdlib V1.0.1 */
    TIM_ITConfig(TIMx, TIM_IT_Break, ENABLE);
  }
  TIM_BDTRConfig_MC(TIMx, &TIMx_BDTRInitStructure);
 
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
      if(pDParams_str->bRepetitionCounter == 1u)
      {
        TIM_SetCounter(TIMx, (uint32_t)(pDVars_str->Half_PWMPeriod)-1u);
      }  
      else if (pDParams_str->bRepetitionCounter == 3u)
      {
        /* Set TIMx repetition counter to 1 */
        TIMx->RCR =0x01u;
        TIM_GenerateEvent(TIMx, TIM_EventSource_Update);
        /* Repetition counter will be set to 3 at next Update */
        TIMx->RCR =0x03u; 
      } 
    }
  }

  pDVars_str->wPreloadDisableCC1 = TIMx->CCMR1 & CC1_PRELOAD_DISABLE_MASK;
  pDVars_str->wPreloadDisableCC2 = TIMx->CCMR1 & CC2_PRELOAD_DISABLE_MASK;
  pDVars_str->wPreloadDisableCC3 = TIMx->CCMR2 & CC3_PRELOAD_DISABLE_MASK;
}

/**
* @brief  First initialization of class members
* @param  this related object of class CPWMC
* @retval none
*/
static void R1F30X_1ShuntMotorVarsInit(CPWMC this)
{  
  pDVars_t pDVars_str = &DCLASS_VARS; 
  pDParams_t pDParams_str =  DCLASS_PARAMS;
  
  /* Init motor vars */
  pDVars_str->bInverted_pwm_new=INVERT_NONE;
  pDVars_str->hFlags &= (~STBD3); /*STBD3 cleared*/
  pDVars_str->hFlags &= (~DSTEN); /*DSTEN cleared*/
  
  pDVars_str->Half_PWMPeriod = ((((_CPWMC) this)->pParams_str->hPWMperiod)/2u);
  
  /* After reset, value of DMA buffers for distortion*/
  pDVars_str->hDmaBuff[0] =  pDVars_str->Half_PWMPeriod + 1u;
  pDVars_str->hDmaBuff[1] =  pDVars_str->Half_PWMPeriod >> 1; /*dummy*/
 
  /* Default value of sampling points */ 
  pDVars_str->hCntSmp1 = (pDVars_str->Half_PWMPeriod >> 1) + (pDParams_str->hTafter);
  pDVars_str->hCntSmp2 = pDVars_str->Half_PWMPeriod - 1u;
}

/**
* @brief  Initialization of class members after each motor start
* @param  this related object of class CPWMC
* @retval none
*/
static void R1F30X_1ShuntMotorVarsRestart(CPWMC this)
{
  pDVars_t pDVars_str = &DCLASS_VARS;
  pDParams_t pDParams_str =  DCLASS_PARAMS;
  
  /* Default value of sampling points */
  pDVars_str->hCntSmp1 = (pDVars_str->Half_PWMPeriod >> 1) + (pDParams_str->hTafter);
  pDVars_str->hCntSmp2 = pDVars_str->Half_PWMPeriod - 1u;
  
  pDVars_str->bInverted_pwm_new=INVERT_NONE;
  pDVars_str->hFlags &= (~STBD3); /*STBD3 cleared*/
  
  /* Set the default previous value of Phase A,B,C current */
  pDVars_str->hCurrAOld=0;
  pDVars_str->hCurrBOld=0;
  
  pDVars_str->hDmaBuff[0] =  pDVars_str->Half_PWMPeriod + 1u;
  pDVars_str->hDmaBuff[1] =  pDVars_str->Half_PWMPeriod >> 1; /*dummy*/ 
  
  pDVars_str->BrakeActionLock = FALSE;
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
 void R1F3XX_StartTimers(void)
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
* @param  this: related object of class CR3F30X_PWMC
* @retval none
*/
static void R1F30X_CurrentReadingCalibration(CPWMC this)
{
  Vars_t *pBaseVars = &((_CPWMC) this)->Vars_str;
  pDVars_t pDVars_str = &DCLASS_VARS;
  pDParams_t pDParams_str = DCLASS_PARAMS;

  TIM_TypeDef* TIMx = pDParams_str->TIMx;
  uint16_t hCalibrationPeriodCounter;
  uint16_t hMaxPeriodsNumber;  
  pDVars_str->wPhaseOffset = 0u;  
  pDVars_str->bIndex=0u;

  /* Disable distortion*/
  pDVars_str->hFlags &= (~DSTEN); /*DSTEN cleared*/
  
  /* It forces inactive level on TIMx CHy and CHyN */
  TIMx->CCER &= (~TIMxCCER_MASK_CH123);
   
  /* Offset calibration  */
  /* Change function to be executed in ADCx_ISR */ 
  ((_CPWMC) this)->Methods_str.pPWMC_GetPhaseCurrents = &R1F30X_HFCurrentsCalibration;
  
  R1F30X_SwitchOnPWM(this);
  
  /* Wait for NB_CONVERSIONS to be executed */
  hMaxPeriodsNumber=(NB_CONVERSIONS+1u)*((uint16_t)(pDParams_str->bRepetitionCounter)+1u);
  TIMx->SR = (uint16_t)~TIM_FLAG_CC1;
  hCalibrationPeriodCounter = 0u;
  while (pDVars_str->bIndex < NB_CONVERSIONS)
  {
    if (TIMx->SR & TIM_FLAG_CC1)
    {
      TIMx->SR = (uint16_t)~TIM_FLAG_CC1;
      hCalibrationPeriodCounter++;
      if (hCalibrationPeriodCounter >= hMaxPeriodsNumber)
      {
        if (pDVars_str->bIndex < NB_CONVERSIONS)
        {
          pBaseVars->SWerror = 1u;
          break;
        }
      }
    }
  }
  
  R1F30X_SwitchOffPWM(this);
  
  pDVars_str->wPhaseOffset >>=4;  

  /* Change back function to be executed in ADCx_ISR */ 
  ((_CPWMC) this)->Methods_str.pPWMC_GetPhaseCurrents = &R1F30X_GetPhaseCurrents;
  
  /* It re-enable drive of TIMx CHy and CHyN by TIMx CHyRef*/
  TIMx->CCER |= TIMxCCER_MASK_CH123;
  
  /* Enable distortion*/
  pDVars_str->hFlags |= DSTEN; /*DSTEN set*/
  
  R1F30X_1ShuntMotorVarsRestart(this);  
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif
/**
* @brief  It computes and return latest converted motor phase currents motor
* @param  this: related object of class CR3F30X_PWMC
* @retval Ia and Ib current in Curr_Components format
*/
static void R1F30X_GetPhaseCurrents(CPWMC this,Curr_Components* pStator_Currents)
{
  int32_t wAux;
  int16_t hCurrA = 0, hCurrB = 0, hCurrC = 0;
  uint8_t bCurrASamp = 0u, bCurrBSamp = 0u, bCurrCSamp = 0u;
  
  pDVars_t pDVars_str = &DCLASS_VARS;
  
  TIM_TypeDef* TIMx = pDVars_str->TIMx;  
  TIMx->CCMR1 |= CC12_PRELOAD_ENABLE_MASK;
  TIMx->CCMR2 |= CC3_PRELOAD_ENABLE_MASK;  
  
  /* Reset the update flag to indicate the start of FOC algorithm*/
  pDVars_str->TIMx->SR &= (uint32_t)~TIM_FLAG_Update;
  
  /* First sampling point */
  wAux = (int32_t)(pDVars_str->ADCx->JDR1);

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
  
  pStator_Currents->qI_Component1 = hCurrA;
  pStator_Currents->qI_Component2 = hCurrB;
}

/**
* @brief  Implementaion of PWMC_GetPhaseCurrents to be performed during 
*         calibration. It sum up injected conversion data into wPhaseCOffset
*         to compute the offset introduced in the current feedback
*         network. It is requied to proper configure ADC input before to enable
*         the offset computation.
* @param  this: related object of class CPWMC
* @retval It always returns {0,0} in Curr_Components format
*/
static void R1F30X_HFCurrentsCalibration(CPWMC this,Curr_Components* pStator_Currents)
{
  /* Derived class members container */
  pDVars_t pDVars_str = &DCLASS_VARS;
  pDParams_t pDParams_str = DCLASS_PARAMS;
  TIM_TypeDef* TIMx = pDVars_str->TIMx;
  
  /* Reset the update flag to indicate the start of FOC algorithm*/
  pDVars_str->TIMx->SR = (uint16_t)~TIM_FLAG_Update;  
  
  if (pDVars_str->bIndex < NB_CONVERSIONS)
  {
    pDVars_str->wPhaseOffset += pDVars_str->ADCx->JDR2;
    pDVars_str->bIndex++;
    
    /* fill the queue*/
    /* Preload Disable */
    TIMx->CCMR3 &= TIMxCCR56_PRELOAD_DISABLE_MASK;
    TIMx->CCR5 = 0u;
    TIMx->CCR6 = 0u;
    /* Preload enable */
    TIMx->CCMR3 |= TIMxCCR56_PRELOAD_ENABLE_MASK; 
    
    TIMx->CCR5 = ((uint32_t)(pDVars_str->Half_PWMPeriod) >> 1) + (uint32_t)(pDParams_str->hTafter);    
    
    TIMx->CCR6 = (uint32_t)(pDVars_str->Half_PWMPeriod) - 1u;
  }
}

/**
  * @brief  It turns on low sides switches. This function is intended to be 
  *         used for charging boot capacitors of driving section. It has to be 
  *         called each motor start-up when using high voltage drivers
  * @param  this: related object of class CR3F30X_PWMC
  * @retval none
  */
static void R1F30X_TurnOnLowSides(CPWMC this)
{  
  pDParams_t pDParams_str = DCLASS_PARAMS;
  TIM_TypeDef* TIMx = DCLASS_PARAMS->TIMx;
  
  /*Turn on the three low side switches */
  TIMx->CCR1 = 0u;
  TIMx->CCR2 = 0u;
  TIMx->CCR3 = 0u;
  
  /* Clear Update Flag */
  TIM_ClearFlag(TIMx, (uint16_t)(TIM_FLAG_Update));
  
  /* Wait until next update */
  while (TIM_GetFlagStatus(TIMx,TIM_FLAG_Update)==RESET)
  {}
  
  /* Main PWM Output Enable */
  TIMx->BDTR |= TIM_BDTR_MOE;
  
  if ((pDParams_str->LowSideOutputs)== ES_GPIO)
  {
    GPIO_WriteBit(pDParams_str->hCh1NPort, pDParams_str->hCh1NPin, Bit_SET);
    GPIO_WriteBit(pDParams_str->hCh2NPort, pDParams_str->hCh2NPin, Bit_SET);
    GPIO_WriteBit(pDParams_str->hCh3NPort, pDParams_str->hCh3NPin, Bit_SET);
  }
  return; 
}


/**
* @brief  It enables PWM generation on the proper Timer peripheral acting on MOE
*         bit
* @param  this: related object of class CR3F30X_PWMC
* @retval none
*/
static void R1F30X_SwitchOnPWM(CPWMC this)
{
  TIM_TypeDef* TIMx = DCLASS_PARAMS->TIMx;
  pDParams_t pDParams_str = DCLASS_PARAMS;
  pDVars_t pDVars_str = &DCLASS_VARS;
  
  TIMx->DIER &= (uint16_t)~TIM_DMA_CC4;
  TIMx->DIER &= (uint16_t)~TIM_DMA_Update;
  pDVars_str->PreloadDMAy_Chx->CCR &= (uint16_t)(~DMA_CCR_EN);
  pDVars_str->DistortionDMAy_Chx->CCR &= (uint16_t)(~DMA_CCR_EN); 
  pDVars_str->DistortionDMAy_Chx->CNDTR = 2u;
  
  /* Enables the TIMx Preload on CC1 Register */
  TIM_OC1PreloadConfig(TIMx, TIM_OCPreload_Enable);
  /* Enables the TIMx Preload on CC2 Register */
  TIM_OC2PreloadConfig(TIMx, TIM_OCPreload_Enable);
  /* Enables the TIMx Preload on CC3 Register */
  TIM_OC3PreloadConfig(TIMx, TIM_OCPreload_Enable);
  
  /* TIM output trigger 2 for ADC */
  TIM_SelectOutputTrigger2(TIMx, TIM_TRGO2Source_OC5RefRising_OC6RefRising); 

  /* wait for a new PWM period */
  TIM_ClearFlag(TIMx, (uint16_t)(TIM_FLAG_Update));
  while (TIM_GetFlagStatus(TIMx,TIM_FLAG_Update) == RESET)
  {}
  TIM_ClearFlag(TIMx, (uint16_t)(TIM_FLAG_Update));
  
  /* Set all duty to 50% */
  /* Set ch5 ch6 for triggering */
  /* Clear Update Flag */
  /* TIM ch4 DMA request enable */  
  
  pDVars_str->hDmaBuff[1] = pDVars_str->Half_PWMPeriod >> 1; 
  TIMx->CCR1 = (uint32_t)(pDVars_str->Half_PWMPeriod) >> 1;
  TIMx->CCR2 = (uint32_t)(pDVars_str->Half_PWMPeriod) >> 1;
  TIMx->CCR3 = (uint32_t)(pDVars_str->Half_PWMPeriod) >> 1;

  while (TIM_GetFlagStatus(TIMx,TIM_FLAG_Update) == RESET)
  {}
  
  /* Main PWM Output Enable */  
  
  TIMx->BDTR |= TIM_OSSIState_Enable; 
  TIMx->BDTR |= TIM_BDTR_MOE; 
  
  TIMx->CCR5 = ((uint32_t)(pDVars_str->Half_PWMPeriod) >> 1) + (uint32_t)(pDParams_str->hTafter);  
  TIMx->CCR6 = (uint32_t)(pDVars_str->Half_PWMPeriod) - 1u; 
  
  TIMx->DIER |= TIM_DMA_CC4;
  pDVars_str->DistortionDMAy_Chx->CCR |= DMA_CCR_EN;  
  
  if ((pDParams_str->LowSideOutputs)== ES_GPIO)
  {
    if ((TIMx->CCER & TIMxCCER_MASK_CH123) != 0u)
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
*         MOE bit
* @param  this: related object of class CR3F30X_PWMC
* @retval none
*/
static void R1F30X_SwitchOffPWM(CPWMC this)
{
  pDVars_t pDVars_str = &DCLASS_VARS;
  TIM_TypeDef* TIMx = pDVars_str->TIMx;
  ADC_TypeDef* ADCx = pDVars_str->ADCx;
  pDParams_t pDParams_str = DCLASS_PARAMS;
  
  /* Main PWM Output Disable */
  if (DCLASS_VARS.BrakeActionLock == TRUE)
  {
  }
  else
  {
    TIMx->BDTR &= ~((uint32_t)(TIM_OSSIState_Enable));    
  }
  TIMx->BDTR &= (uint32_t)~TIM_BDTR_MOE;
  
  if ((pDParams_str->LowSideOutputs)== ES_GPIO)
  {
    GPIO_WriteBit(pDParams_str->hCh1NPort, pDParams_str->hCh1NPin, Bit_RESET);
    GPIO_WriteBit(pDParams_str->hCh2NPort, pDParams_str->hCh2NPin, Bit_RESET);
    GPIO_WriteBit(pDParams_str->hCh3NPort, pDParams_str->hCh3NPin, Bit_RESET);
  }  
  
  /* ADC_ITConfig(ADCx, ADC_IT_JEOS, DISABLE);*/
  ADCx->IER &= (~(uint32_t)ADC_IT_JEOS);
  
  /* Flushing JSQR queue of context by setting JADSTP = 1 (JQM)=1 */
  ADCx->CR |= ADC_CR_JADSTP;
  
  /* CC5 CC6 Preload Disable */
  TIMx->CCMR3 &= TIMxCCR56_PRELOAD_DISABLE_MASK;  
  
  TIMx->CCR5 = (uint32_t)(pDVars_str->Half_PWMPeriod) + 1u;
  TIMx->CCR6 = (uint32_t)(pDVars_str->Half_PWMPeriod) + 1u;
  
  /* Preload enable */
  TIMx->CCMR3 |= TIMxCCR56_PRELOAD_ENABLE_MASK;
  
  /* Re-enable ADC triggering*/
  pDVars_str->ADCx->JSQR = pDVars_str->wADC_JSQR;
  /*ADC_StartInjectedConversion(ADCx);*/
  ADCx->CR |= ADC_CR_JADSTART;  
  /*ADC_ClearFlag(ADCx, ADC_FLAG_JEOS);*/
  ADCx->ISR = (uint32_t)ADC_FLAG_JEOS;

  /* ADC_ITConfig(ADCx, ADC_IT_JEOS, ENABLE);*/
  ADCx->IER |= ADC_FLAG_JEOS;
  
  /* Disable TIMx DMA requests enable */
  TIMx->DIER &= (uint16_t)~TIM_DMA_CC4;
  TIMx->DIER &= (uint16_t)~TIM_DMA_Update;
  
  /* Disable DMA channels*/
  pDVars_str->PreloadDMAy_Chx->CCR &= (uint16_t)(~DMA_CCR_EN); 
  
  return; 
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif
/**
* @brief  It contains the TIMx Update event interrupt
* @param  this: related object of class CR3F30X_PWMC
* @retval none
*/
static void *R1F30X_IRQHandler(void *this, unsigned char flag)
{
  pVars_t pVars_str = &CLASS_VARS;
  if (flag == 2u)
  {
    /* L6230 management */
    DCLASS_VARS.OverCurrentFlag = TRUE;
  }
  else if (flag == 3u)
  {
    DCLASS_PARAMS->TIMx->BDTR |= TIM_OSSIState_Enable; 
    DCLASS_VARS.OverVoltageFlag = TRUE;
    DCLASS_VARS.BrakeActionLock = TRUE;
  }
  else
  {
  }  
  return &(pVars_str->bMotor);
}

/**
* @brief  Execute a regular conversion using ADCx. 
*         The function is not re-entrant (can't executed twice at the same time)
* @param  this related object of class CR3F30X_PWMC
* @retval It returns converted value or oxFFFF for conversion error
*/
static uint16_t R1F30X_ExecRegularConv(CPWMC this, uint8_t bChannel)
{
  pDVars_t pDVars_str = &DCLASS_VARS;
  pDParams_t pDParams_str = DCLASS_PARAMS;
  ADC_TypeDef* ADCx = pDParams_str->regconvADCx;
  
  ADCx->SQR1 = (uint32_t)(bChannel) << 6;
  
  ADCx->DR;
  ADCx->CR = ADC_CR_ADSTART;
  
  /* Wait until end of regular conversion */
  while ((ADCx->ISR & ADC_ISR_EOC) == 0u)
  {
  }
  
  pDVars_str->hRegConv = (uint16_t)(ADCx->DR);
  return (pDVars_str->hRegConv);
}

/**
* @brief  It sets the specified sampling time for the specified ADC channel
*         on ADC1. It must be called once for each channel utilized by user
* @param  ADC channel, sampling time
* @retval none
*/
static void R1F30X_ADC_SetSamplingTime(CPWMC this, ADConv_t ADConv_struct)
{ 
  pDParams_t pDParams_str = DCLASS_PARAMS;
  uint32_t tmpreg2 = 0u;
  uint8_t ADC_Channel = ADConv_struct.Channel;
  uint8_t ADC_SampleTime = ADConv_struct.SamplTime;
  
  /* Channel sampling configuration */
  /* if ADC_Channel_10 ... ADC_Channel_18 is selected */
  if (ADC_Channel > ADC_Channel_9)
  {
    uint32_t wAux,wAux2;
    /* Get the old register value */
    /* Calculate the mask to clear */
    wAux = ADC_SMPR2_SMP10;
    wAux2 = 3u * ((uint32_t)(ADC_Channel) - 10u);
    tmpreg2 =  wAux << wAux2;
    /* Clear the old channel sample time */
    pDParams_str->regconvADCx->SMPR2 &= ~tmpreg2;
    /* Calculate the mask to set */
    wAux = (uint32_t)(ADC_SampleTime);
    pDParams_str->regconvADCx->SMPR2 |=  wAux << wAux2;
    
  }
  else /* ADC_Channel include in ADC_Channel_[0..9] */
  {
    uint32_t wAux,wAux2;
    /* Get the old register value */
    /* Calculate the mask to clear */
    wAux = ADC_SMPR1_SMP1;
    wAux2 = 3u * ((uint32_t)(ADC_Channel) - 1u);
    tmpreg2 =  wAux << wAux2;
    /* Clear the old channel sample time */
    pDParams_str->regconvADCx->SMPR1 &= ~tmpreg2;
    /* Calculate the mask to set */
    wAux = (uint32_t)(ADC_SampleTime);
    wAux2 = 3u * ((uint32_t)(ADC_Channel));
    pDParams_str->regconvADCx->SMPR1 |= wAux << wAux2;
  }
}
/**
* @brief  It is used to check if an overcurrent occurred since last call.
* @param  this related object of class CPWMC
* @retval uint16_t It returns MC_BREAK_IN whether an overcurrent has been 
*                  detected since last method call, MC_NO_FAULTS otherwise.
*/
static uint16_t R1F30X_IsOverCurrentOccurred(CPWMC this)
{
  pDVars_t pDVars_str = &DCLASS_VARS;

  uint16_t retVal = MC_NO_FAULTS;
  
  if (pDVars_str->OverVoltageFlag == TRUE)
  {
    retVal = MC_OVER_VOLT;
    pDVars_str->OverVoltageFlag = FALSE;
  }
  
  if (pDVars_str->OverCurrentFlag == TRUE)
  {
    retVal |= MC_BREAK_IN;
    pDVars_str->OverCurrentFlag = FALSE;
  }
  
  return retVal;  
}

/**
* @brief  It is used to configure the analog output used for protection 
*         thresholds.
* @param  DAC_Channel: the selected DAC channel. 
*          This parameter can be:
*            @arg DAC_Channel_1: DAC Channel1 selected
*            @arg DAC_Channel_2: DAC Channel2 selected
* @param  hDACVref Value of DAC reference expressed as 16bit unsigned integer.
*         Ex. 0 = 0V 65536 = VDD_DAC.
* @param  Output It enable/disable the DAC output to the external pin.
             It must be either equal to ENABLE or DISABLE.
* @retval none
*/
static void R1F30X_SetAOReferenceVoltage(uint32_t DAC_Channel, uint16_t hDACVref)
{
  DAC_InitTypeDef DAC_InitStructure;
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
  
  /* DAC Configuration */
  DAC_InitStructure.DAC_Trigger = DAC_Trigger_Software;
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
  DAC_InitStructure.DAC_Buffer_Switch = DAC_BufferSwitch_Enable;
  DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude=DAC_TriangleAmplitude_1;
  DAC_Init(DAC1, DAC_Channel, &DAC_InitStructure);
  
  if (DAC_Channel == DAC_Channel_2)
  {
    DAC_SetChannel2Data(DAC1, DAC_Align_12b_L,hDACVref);
  }
  else
  {
    DAC_SetChannel1Data(DAC1, DAC_Align_12b_L,hDACVref);
  }  

  /* Enable DAC Channel */
  DAC_SoftwareTriggerCmd(DAC1, DAC_Channel,ENABLE);
  DAC_Cmd(DAC1, DAC_Channel, ENABLE);
}

/**
  * @brief  It is an internal function used to compute the GPIO Source 
  *         value starting from GPIO pin value. The GPIO Source value 
  *         is used for AF remapping.
  * @param  GPIO_Pin Pin value to be converted.
  * @retval uint16_t The GPIO pin source value converted.
  */
static uint16_t F30X_GPIOPin2Source(uint16_t GPIO_Pin)
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
  * @brief  Fills each TIM_BDTRInitStruct member with its default value.
  * @param  TIM_BDTRInitStruct: pointer to a TIM_BDTRInitTypeDef structure which
  *         will be initialized.
  * @retval None
  */
static void TIM_BDTRStructInit_MC(TIM_BDTRInitTypeDef_MC* TIM_BDTRInitStruct)
{
  /* Set the default configuration */
  TIM_BDTRInitStruct->TIM_OSSRState = TIM_OSSRState_Disable;
  TIM_BDTRInitStruct->TIM_OSSIState = TIM_OSSIState_Disable;
  TIM_BDTRInitStruct->TIM_LOCKLevel = TIM_LOCKLevel_OFF;
  TIM_BDTRInitStruct->TIM_DeadTime = 0x00u;
  TIM_BDTRInitStruct->TIM_Break = TIM_Break_Disable;
  TIM_BDTRInitStruct->TIM_BreakPolarity = TIM_BreakPolarity_Low;
  TIM_BDTRInitStruct->TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;
  TIM_BDTRInitStruct->TIM_Break2 = TIM_Break2_Disable;
  TIM_BDTRInitStruct->TIM_Break2Polarity = TIM_Break2Polarity_Low;
  TIM_BDTRInitStruct->TIM_Break1Filter = 0x00u;
  TIM_BDTRInitStruct->TIM_Break2Filter = 0x00u;
}

/**
  * @brief  Configures the Break feature, dead time, Lock level, OSSI/OSSR State
  *         and the AOE(automatic output enable).
  * @param  TIMx: where x can be  1, 8, 15, 16 or 17 to select the TIM 
  * @param  TIM_BDTRInitStruct: pointer to a TIM_BDTRInitTypeDef_MC structure that
  *         contains the BDTR Register configuration  information for the TIM peripheral.
  * @retval None
  */
static void TIM_BDTRConfig_MC(TIM_TypeDef* TIMx, TIM_BDTRInitTypeDef_MC *TIM_BDTRInitStruct)
{
  /* Check the parameters */
#ifndef MISRA_C_2004_BUILD
  assert_param(IS_TIM_LIST6_PERIPH(TIMx));
  assert_param(IS_TIM_OSSR_STATE(TIM_BDTRInitStruct->TIM_OSSRState));
  assert_param(IS_TIM_OSSI_STATE(TIM_BDTRInitStruct->TIM_OSSIState));
  assert_param(IS_TIM_LOCK_LEVEL(TIM_BDTRInitStruct->TIM_LOCKLevel));
  assert_param(IS_TIM_BREAK_STATE(TIM_BDTRInitStruct->TIM_Break));
  assert_param(IS_TIM_BREAK_POLARITY(TIM_BDTRInitStruct->TIM_BreakPolarity));
  assert_param(IS_TIM_AUTOMATIC_OUTPUT_STATE(TIM_BDTRInitStruct->TIM_AutomaticOutput));
  assert_param(IS_TIM_BREAK2_STATE(TIM_BDTRInitStruct->TIM_Break2));
  assert_param(IS_TIM_BREAK2_POLARITY(TIM_BDTRInitStruct->TIM_Break2Polarity));
  assert_param(IS_TIM_BREAK1_FILTER(TIM_BDTRInitStruct->TIM_Break1Filter));
  assert_param(IS_TIM_BREAK2_FILTER(TIM_BDTRInitStruct->TIM_Break2Filter));
#endif

  /* Set the Lock level, the Break enable Bit and the Polarity, the OSSR State,
     the OSSI State, the dead time value and the Automatic Output Enable Bit */
  TIMx->BDTR = (uint32_t)TIM_BDTRInitStruct->TIM_OSSRState | TIM_BDTRInitStruct->TIM_OSSIState |
             TIM_BDTRInitStruct->TIM_LOCKLevel | TIM_BDTRInitStruct->TIM_DeadTime |
             TIM_BDTRInitStruct->TIM_Break | TIM_BDTRInitStruct->TIM_BreakPolarity |
             TIM_BDTRInitStruct->TIM_AutomaticOutput|TIM_BDTRInitStruct->TIM_Break2 |
	     TIM_BDTRInitStruct->TIM_Break2Polarity|((uint32_t)TIM_BDTRInitStruct->TIM_Break1Filter << 16) |
	           ((uint32_t)TIM_BDTRInitStruct->TIM_Break2Filter << 20);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif
/**
* @brief  Implementation of the single shunt algorithm to setup the 
*         TIM1 register and DMA buffers values for the next PWM period.
* @param  this related object of class CPWMC
* @retval uint16_t It returns MC_FOC_DURATION if the TIMx update occurs 
          before the end of FOC algorithm else returns MC_NO_ERROR
*/
static uint16_t R1F30X_CalcDutyCycles(CPWMC this)
{
  int16_t hDeltaDuty_0;
  int16_t hDeltaDuty_1;
  uint16_t hDutyV_0 = 0u;
  uint16_t hDutyV_1 = 0u;
  uint16_t hDutyV_2 = 0u;
  uint8_t bSector;
  uint8_t bStatorFluxPos;
  uint16_t hAux;
  
  pDVars_t pDVars_str = &DCLASS_VARS;
  pDParams_t pDParams_str = DCLASS_PARAMS;
  TIM_TypeDef* TIMx = pDVars_str->TIMx;
  
  bSector = (uint8_t)(CLASS_VARS.hSector);
  
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
        
    if (bStatorFluxPos == BOUNDARY_1) /* Adjust the lower */
    {
      switch (bSector)
      {
      case SECTOR_5:
      case SECTOR_6:
        if (CLASS_VARS.hCntPhA - pDParams_str->hCHTMin - hDutyV_0 > pDParams_str->hTMin)
        {
          pDVars_str->bInverted_pwm_new = INVERT_A;
          CLASS_VARS.hCntPhA -=pDParams_str->hCHTMin;
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
            CLASS_VARS.hCntPhA -=pDParams_str->hCHTMin;
            pDVars_str->hFlags |= STBD3;
          } 
          else
          {
            pDVars_str->bInverted_pwm_new = INVERT_B;
            CLASS_VARS.hCntPhB -=pDParams_str->hCHTMin;
            pDVars_str->hFlags &= (~STBD3);
          }
        }
        break;
      case SECTOR_2:
      case SECTOR_1:
        if (CLASS_VARS.hCntPhB - pDParams_str->hCHTMin - hDutyV_0 > pDParams_str->hTMin)
        {
          pDVars_str->bInverted_pwm_new = INVERT_B;
          CLASS_VARS.hCntPhB -=pDParams_str->hCHTMin;
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
            CLASS_VARS.hCntPhA -=pDParams_str->hCHTMin;
            pDVars_str->hFlags |= STBD3;
          } 
          else
          {
            pDVars_str->bInverted_pwm_new = INVERT_B;
            CLASS_VARS.hCntPhB -=pDParams_str->hCHTMin;
            pDVars_str->hFlags &= (~STBD3);
          }
        }
        break;
      case SECTOR_4:
      case SECTOR_3:
        if (CLASS_VARS.hCntPhC - pDParams_str->hCHTMin - hDutyV_0 > pDParams_str->hTMin)
        {
          pDVars_str->bInverted_pwm_new = INVERT_C;
          CLASS_VARS.hCntPhC -=pDParams_str->hCHTMin;
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
            CLASS_VARS.hCntPhA -=pDParams_str->hCHTMin;
            pDVars_str->hFlags |= STBD3;
          } 
          else
          {
            pDVars_str->bInverted_pwm_new = INVERT_B;
            CLASS_VARS.hCntPhB -=pDParams_str->hCHTMin;
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
        CLASS_VARS.hCntPhB -=pDParams_str->hCHTMin;
        if (CLASS_VARS.hCntPhB > 0xEFFFu)
        {
          CLASS_VARS.hCntPhB = 0u;
        }
        break;
      case SECTOR_2:
      case SECTOR_3: /* Invert A */
        pDVars_str->bInverted_pwm_new = INVERT_A;
        CLASS_VARS.hCntPhA -=pDParams_str->hCHTMin;
        if (CLASS_VARS.hCntPhA > 0xEFFFu)
        {
          CLASS_VARS.hCntPhA = 0u;
        }
        break;
      case SECTOR_6:
      case SECTOR_1: /* Invert C */
        pDVars_str->bInverted_pwm_new = INVERT_C;
        CLASS_VARS.hCntPhC -=pDParams_str->hCHTMin;
        if (CLASS_VARS.hCntPhC > 0xEFFFu)
        {
          CLASS_VARS.hCntPhC = 0u;
        }
        break;
      default:
        break;
      }
    }
    else if (bStatorFluxPos == BOUNDARY_3)
    {
      if ((pDVars_str->hFlags & STBD3) == 0u)
      {
        pDVars_str->bInverted_pwm_new = INVERT_A;
        CLASS_VARS.hCntPhA -=pDParams_str->hCHTMin;
        pDVars_str->hFlags |= STBD3;
      } 
      else
      {
        pDVars_str->bInverted_pwm_new = INVERT_B;
        CLASS_VARS.hCntPhB -=pDParams_str->hCHTMin;
        pDVars_str->hFlags &= (~STBD3);
      }
    }
    else
    {
    }
        
    if (bStatorFluxPos == REGULAR) /* Regular zone */
    {
      /* First point */
      pDVars_str->hCntSmp1 = hDutyV_1 - pDParams_str->hTbefore;
      
      /* Second point */
      pDVars_str->hCntSmp2 = hDutyV_2 - pDParams_str->hTbefore;
    }
    
    if (bStatorFluxPos == BOUNDARY_1) /* Two small, one big */
    {      
      /* First point */
      pDVars_str->hCntSmp1 = hDutyV_1 - pDParams_str->hTbefore;
      
      /* Second point */
      pDVars_str->hCntSmp2 = pDVars_str->Half_PWMPeriod - pDParams_str->hHTMin - pDParams_str->hTSample;
    }
    
    if (bStatorFluxPos == BOUNDARY_2) /* Two big, one small */
    {
      /* First point */
      pDVars_str->hCntSmp1 = hDutyV_2 - pDParams_str->hTbefore;
      
      /* Second point */
      pDVars_str->hCntSmp2 = pDVars_str->Half_PWMPeriod - pDParams_str->hHTMin - pDParams_str->hTSample;
    }
    
    if (bStatorFluxPos == BOUNDARY_3)  
    {
      /* First point */
      pDVars_str->hCntSmp1 = hDutyV_0-pDParams_str->hTbefore; /* Dummy trigger */
      /* Second point */
      pDVars_str->hCntSmp2 = pDVars_str->Half_PWMPeriod - pDParams_str->hHTMin - pDParams_str->hTSample;
    }
  }
  else
  {
    pDVars_str->bInverted_pwm_new = INVERT_NONE;
    bStatorFluxPos = REGULAR;
  }
  
  /* Update Timer Ch4 for active vector*/  
  /* Update Timer Ch 5,6 for ADC triggering and books the queue*/
  TIMx->CCMR3 &= TIMxCCR56_PRELOAD_DISABLE_MASK;
  TIMx->CCR5 = 0x0u;
  TIMx->CCR6 = 0xFFFFu;
  TIMx->CCMR3 |= TIMxCCR56_PRELOAD_ENABLE_MASK; 
  
  TIMx->CCR5 = pDVars_str->hCntSmp1;
  TIMx->CCR6 = pDVars_str->hCntSmp2;    
  
  if (bStatorFluxPos == REGULAR)
  {
    /*TIM_SelectOutputTrigger2(TIMx, TIM_TRGO2Source_OC5RefRising_OC6RefRising); */
    TIMx->CR2 &= ((uint32_t)0xFFEFFFFFu);
    
    switch (pDVars_str->bInverted_pwm_new)
    {
    case INVERT_A:
      pDVars_str->hDmaBuff[1] = CLASS_VARS.hCntPhA;

      break;      
    case INVERT_B:
      pDVars_str->hDmaBuff[1] = CLASS_VARS.hCntPhB;

      break;      
    case INVERT_C:
      pDVars_str->hDmaBuff[1] = CLASS_VARS.hCntPhC;

      break;      
    default:      
      break;
    }
  }
  else
  {
    TIMx->DIER &= (uint16_t)~TIM_DMA_Update;
    
    /* Set the DMA destinations */
    switch (pDVars_str->bInverted_pwm_new)
    {
    case INVERT_A:
      pDVars_str->DistortionDMAy_Chx->CPAR = (uint32_t)(&(TIMx->CCR1));
      pDVars_str->PreloadDMAy_Chx->CPAR = (uint32_t)(&(TIMx->CCMR1));
      pDVars_str->hDmaBuff[1] = CLASS_VARS.hCntPhA;
      pDVars_str->wPreloadDisableActing = pDVars_str->wPreloadDisableCC1;   
      break;
      
    case INVERT_B:
      pDVars_str->DistortionDMAy_Chx->CPAR = (uint32_t)(&(TIMx->CCR2));
      pDVars_str->PreloadDMAy_Chx->CPAR = (uint32_t)(&(TIMx->CCMR1));
      pDVars_str->hDmaBuff[1] = CLASS_VARS.hCntPhB;
      pDVars_str->wPreloadDisableActing = pDVars_str->wPreloadDisableCC2;     
      break;
      
    case INVERT_C:
      pDVars_str->DistortionDMAy_Chx->CPAR = (uint32_t)(&(TIMx->CCR3));
      pDVars_str->PreloadDMAy_Chx->CPAR = (uint32_t)(&(TIMx->CCMR2));
      pDVars_str->hDmaBuff[1] = CLASS_VARS.hCntPhC;
      pDVars_str->wPreloadDisableActing = pDVars_str->wPreloadDisableCC3; 
      break;
      
    default:      
      break;
    } 
    
    /*TIM_SelectOutputTrigger2(TIMx, TIM_TRGO2Source_OC5RefRising_OC6RefFalling); */
    TIMx->CR2 |= ((uint32_t)0x100000u);
    
    /*active vector*/
    pDVars_str->PreloadDMAy_Chx->CCR &= (uint16_t)(~DMA_CCR_EN);
    pDVars_str->PreloadDMAy_Chx->CNDTR = 1u;
    pDVars_str->PreloadDMAy_Chx->CCR |= DMA_CCR_EN;    

    TIMx->DIER |= TIM_DMA_Update;
  }
  
  pDVars_str->ADCx->JSQR = pDVars_str->wADC_JSQR;

  /* Update Timer Ch 1,2,3 (These value are required before update event) */
  TIMx->CCR1 = CLASS_VARS.hCntPhA;
  TIMx->CCR2 = CLASS_VARS.hCntPhB;
  TIMx->CCR3 = CLASS_VARS.hCntPhC;
  
  /*End of FOC*/
  /*check software error*/
  if ((TIMx->SR & TIM_IT_Update) == SET)
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
* @brief  It is used to set the PWM mode for R/L detection.
* @param  this related object of class CPWMC
* @param  hDuty to be applied in u16
* @retval none
*/
static void R1F30X_RLDetectionModeEnable(CPWMC this)
{
  pVars_t pVars_str = &CLASS_VARS;
  pDParams_t pDParams_str = DCLASS_PARAMS;
  TIM_TypeDef*  TIMx = pDParams_str->TIMx;
  
  if (pVars_str->RLDetectionMode == FALSE)
  {
    /*  Channel1 configuration */
    TIM_SelectOCxM(TIMx, TIM_Channel_1, TIM_OCMode_PWM1);
    TIM_CCxCmd(TIMx, TIM_Channel_1, TIM_CCx_Enable);
    TIM_CCxNCmd(TIMx, TIM_Channel_1, TIM_CCxN_Disable);
    
    TIM_SetCompare1(TIMx, 0u);
    
    /*  Channel2 configuration */
    if ((pDParams_str-> LowSideOutputs)== LS_PWM_TIMER)
    {
      TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_Active);
      TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
      TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);
    }
    else if ((pDParams_str->LowSideOutputs)== ES_GPIO)
    {
      TIM_SelectOCxM(TIMx, TIM_Channel_2, TIM_OCMode_Inactive);
      TIM_CCxCmd(TIMx, TIM_Channel_2, TIM_CCx_Enable);
      TIM_CCxNCmd(TIMx, TIM_Channel_2, TIM_CCxN_Disable);
    }
    else
    {
    }
    
    /*  Channel3 configuration */
    TIM_CCxCmd(TIMx, TIM_Channel_3, TIM_CCx_Disable);
    TIM_CCxNCmd(TIMx, TIM_Channel_3, TIM_CCxN_Disable);
  }
  
  ((_CPWMC)this)->Methods_str.pPWMC_GetPhaseCurrents = &R1F30X_RLGetPhaseCurrents;
  ((_CPWMC)this)->Methods_str.pPWMC_TurnOnLowSides = &R1F30X_RLTurnOnLowSides;
  ((_CPWMC)this)->Methods_str.pPWMC_SwitchOnPWM = &R1F30X_RLSwitchOnPWM;
  ((_CPWMC)this)->Methods_str.pPWMC_SwitchOffPWM = &R1F30X_RLSwitchOffPWM;
  
  pVars_str->RLDetectionMode = TRUE;
}

/**
* @brief  It is used to disable the PWM mode in 6-step.
* @param  this related object of class CPWMC
* @retval none
*/
static void R1F30X_RLDetectionModeDisable(CPWMC this)
{
  pVars_t pVars_str = &CLASS_VARS;
  pDVars_t pDVars_str = &DCLASS_VARS;
  pDParams_t pDParams_str = DCLASS_PARAMS;
  TIM_TypeDef*  TIMx = pDParams_str->TIMx;
  
  if (pVars_str->RLDetectionMode == TRUE)
  {
    /*  Channel1 configuration */
    TIM_SelectOCxM(TIMx, TIM_Channel_1, TIM_OCMode_PWM1);
    TIM_CCxCmd(TIMx, TIM_Channel_1, TIM_CCx_Enable);
    
    if ((pDParams_str-> LowSideOutputs)== LS_PWM_TIMER)
    {
      TIM_CCxNCmd(TIMx, TIM_Channel_1, TIM_CCxN_Enable);
    }
    else if ((pDParams_str->LowSideOutputs)== ES_GPIO)
    {
      TIM_CCxNCmd(TIMx, TIM_Channel_1, TIM_CCxN_Disable);
    }
    else
    {
    }
    
    TIM_SetCompare1(TIMx, (uint32_t)(pDVars_str->Half_PWMPeriod) >> 1);
    
    /*  Channel2 configuration */
    TIM_SelectOCxM(TIMx, TIM_Channel_2, TIM_OCMode_PWM1);
    TIM_CCxCmd(TIMx, TIM_Channel_2, TIM_CCx_Enable);
    
    if ((pDParams_str-> LowSideOutputs)== LS_PWM_TIMER)
    {
      TIM_CCxNCmd(TIMx, TIM_Channel_2, TIM_CCxN_Enable);
    }
    else if ((pDParams_str->LowSideOutputs)== ES_GPIO)
    {
      TIM_CCxNCmd(TIMx, TIM_Channel_2, TIM_CCxN_Disable);
    }
    else
    {
    }
    
    TIM_SetCompare2(TIMx, (uint32_t)(pDVars_str->Half_PWMPeriod) >> 1);
    
    /*  Channel3 configuration */
    TIM_SelectOCxM(TIMx, TIM_Channel_3, TIM_OCMode_PWM1);
    TIM_CCxCmd(TIMx, TIM_Channel_3, TIM_CCx_Enable);
    
    if ((pDParams_str-> LowSideOutputs)== LS_PWM_TIMER)
    {
      TIM_CCxNCmd(TIMx, TIM_Channel_3, TIM_CCxN_Enable);
    }
    else if ((pDParams_str->LowSideOutputs)== ES_GPIO)
    {
      TIM_CCxNCmd(TIMx, TIM_Channel_3, TIM_CCxN_Disable);
    }
    else
    {
    }
    
    TIM_SetCompare3(TIMx, (uint32_t)(pDVars_str->Half_PWMPeriod) >> 1);
    
    ((_CPWMC)this)->Methods_str.pPWMC_GetPhaseCurrents = &R1F30X_GetPhaseCurrents;
    ((_CPWMC)this)->Methods_str.pPWMC_TurnOnLowSides = &R1F30X_TurnOnLowSides;
    ((_CPWMC)this)->Methods_str.pPWMC_SwitchOnPWM = &R1F30X_SwitchOnPWM;
    ((_CPWMC)this)->Methods_str.pPWMC_SwitchOffPWM = &R1F30X_SwitchOffPWM;
    
    pVars_str->RLDetectionMode = FALSE;
  }
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif
/**
* @brief  It is used to set the PWM dutycycle in 6-step mode.
* @param  this related object of class CPWMC
* @param  hDuty to be applied in u16
* @retval It returns the code error 'MC_FOC_DURATION' if any, 'MC_NO_ERROR' 
*         otherwise. These error codes are defined in MC_type.h
*/
static uint16_t R1F30X_RLDetectionModeSetDuty(CPWMC this, uint16_t hDuty)
{
  Vars_t *pVars_Str = &CLASS_VARS;
  pDVars_t pDVars_str = &DCLASS_VARS;
  pDParams_t pDParams_str = DCLASS_PARAMS;
  DVars_t *pLocalVars_Str = &DCLASS_VARS;
  TIM_TypeDef*  TIMx = pDParams_str->TIMx;
  uint16_t hAux;
  
  uint32_t val = ((uint32_t)(PWM_PERIOD) * (uint32_t)(hDuty)) >> 16;
  pVars_Str->hCntPhA = (uint16_t)(val);
    
  TIMx->CCR1 = ((_CPWMC) this)->Vars_str.hCntPhA;
  
  pDVars_str->ADCx->JSQR = pDVars_str->wADC_JSQR & (uint32_t)(0xFFFFFFFCu); /* Re enable ADC trig 1 trigger*/
    
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

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif
/**
* @brief  It computes and return latest converted motor phase currents motor
*         during RL detection phase
* @param  this: related object of class CR3F30X_PWMC
* @retval Ia and Ib current in Curr_Components format
*/
static void R1F30X_RLGetPhaseCurrents(CPWMC this,Curr_Components* pStator_Currents)
{
  pDVars_t pDVars_str = &DCLASS_VARS;
  int32_t wAux;
  int16_t hCurrA = 0, hCurrB = 0;
  
  /* Reset the update flag to indicate the start of algorithm*/
  pDVars_str->TIMx->SR &= (uint32_t)~TIM_FLAG_Update;
  
  wAux = (int32_t)(pDVars_str->ADCx->JDR1);

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
  
  hCurrA = (int16_t)(wAux);
  hCurrB = -hCurrA;
  
  pStator_Currents->qI_Component1 = -hCurrA;
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
static void R1F30X_RLTurnOnLowSides(CPWMC this)
{  
  pDParams_t pDParams_str = DCLASS_PARAMS;
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
  
  if ((pDParams_str->LowSideOutputs)== ES_GPIO)
  {
    GPIO_WriteBit(pDParams_str->hCh1NPort, pDParams_str->hCh1NPin, Bit_SET);
    GPIO_WriteBit(pDParams_str->hCh2NPort, pDParams_str->hCh2NPin, Bit_RESET);
    GPIO_WriteBit(pDParams_str->hCh3NPort, pDParams_str->hCh3NPin, Bit_RESET);
  }
  return; 
}


/**
* @brief  It enables PWM generation on the proper Timer peripheral
*         This function is specific for RL detection phase.
* @param  this: related object of class CR3F30X_PWMC
* @retval none
*/
static void R1F30X_RLSwitchOnPWM(CPWMC this)
{
  TIM_TypeDef* TIMx = DCLASS_PARAMS->TIMx;
  pDParams_t pDParams_str = DCLASS_PARAMS;
  
  /* Enables the TIMx Preload on CC1 Register */
  TIM_OC1PreloadConfig(TIMx, TIM_OCPreload_Enable);
  /* Enables the TIMx Preload on CC2 Register */
  TIM_OC2PreloadConfig(TIMx, TIM_OCPreload_Enable);
  
  /* TIM output trigger 2 for ADC */
  TIM_SelectOutputTrigger2(TIMx, TIM_TRGO2Source_Update);

  /* wait for a new PWM period */
  TIM_ClearFlag(TIMx, (uint16_t)(TIM_FLAG_Update));
  while (TIM_GetFlagStatus(TIMx,TIM_FLAG_Update) == RESET)
  {}
  TIM_ClearFlag(TIMx, (uint16_t)(TIM_FLAG_Update));
  
  /* Set duty to 0% */
  TIMx->CCR1 = 0u;
  
  while (TIM_GetFlagStatus(TIMx,TIM_FLAG_Update) == RESET)
  {}
  
  /* Main PWM Output Enable */  
  
  TIMx->BDTR |= TIM_OSSIState_Enable; 
  TIMx->BDTR |= TIM_BDTR_MOE; 
  
  if ((pDParams_str->LowSideOutputs)== ES_GPIO)
  {
    GPIO_WriteBit(pDParams_str->hCh1NPort, pDParams_str->hCh1NPin, Bit_SET);
    GPIO_WriteBit(pDParams_str->hCh2NPort, pDParams_str->hCh2NPin, Bit_SET);
    GPIO_WriteBit(pDParams_str->hCh3NPort, pDParams_str->hCh3NPin, Bit_RESET);
  }
  return; 
}


/**
* @brief  It disables PWM generation on the proper Timer peripheral acting on 
*         MOE bit
*         This function is specific for RL detection phase.
* @param  this: related object of class CR3F30X_PWMC
* @retval none
*/
static void R1F30X_RLSwitchOffPWM(CPWMC this)
{
  pDVars_t pDVars_str = &DCLASS_VARS;
  TIM_TypeDef* TIMx = pDVars_str->TIMx;
  ADC_TypeDef* ADCx = pDVars_str->ADCx;
  pDParams_t pDParams_str = DCLASS_PARAMS;
  
  /* Main PWM Output Disable */
  if (DCLASS_VARS.BrakeActionLock == TRUE)
  {
  }
  else
  {
    TIMx->BDTR &= ~((uint32_t)(TIM_OSSIState_Enable));    
  }
  TIMx->BDTR &= (uint32_t)~TIM_BDTR_MOE;
  
  if ((pDParams_str->LowSideOutputs)== ES_GPIO)
  {
    GPIO_WriteBit(pDParams_str->hCh1NPort, pDParams_str->hCh1NPin, Bit_RESET);
    GPIO_WriteBit(pDParams_str->hCh2NPort, pDParams_str->hCh2NPin, Bit_RESET);
    GPIO_WriteBit(pDParams_str->hCh3NPort, pDParams_str->hCh3NPin, Bit_RESET);
  }

  /* TIM output trigger 2 for ADC */
  TIM_SelectOutputTrigger2(TIMx, TIM_TRGO2Source_Reset);  
  
  /* ADC_ITConfig(ADCx, ADC_IT_JEOS, DISABLE);*/
  ADCx->IER &= (~(uint32_t)ADC_IT_JEOS);
  
  /* Flushing JSQR queue of context by setting JADSTP = 1 (JQM)=1 */
  ADCx->CR |= ADC_CR_JADSTP;
    
  /* Re-enable ADC triggering*/
  pDVars_str->ADCx->JSQR = pDVars_str->wADC_JSQR;
  /*ADC_StartInjectedConversion(ADCx);*/
  ADCx->CR |= ADC_CR_JADSTART;  
  /*ADC_ClearFlag(ADCx, ADC_FLAG_JEOS);*/
  ADCx->ISR = (uint32_t)ADC_FLAG_JEOS;

  /* ADC_ITConfig(ADCx, ADC_IT_JEOS, ENABLE);*/
  ADCx->IER |= ADC_FLAG_JEOS;
  
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
