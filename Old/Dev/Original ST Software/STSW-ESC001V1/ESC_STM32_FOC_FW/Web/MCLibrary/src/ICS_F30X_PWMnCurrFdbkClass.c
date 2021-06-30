/**
  ******************************************************************************
  * @file    ICS_F30X_PWMnCurrFdbkClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file will contains implementation of current sensor class to be
  *          instantiated when an insulated current sensing topology is 
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
#include "ICS_F30X_PWMnCurrFdbkClass.h"
#include "ICS_F30X_PWMnCurrFdbkPrivate.h"
#include "MCIRQHandlerClass.h"
#include "MCIRQHandlerPrivate.h"
#include "MCLibraryConf.h"
#include "MCLibraryISRPriorityConf.h"
#include "MC_type.h"

/* ADC SMPx mask */
#define SMPR1_SMP_Set              ((uint32_t) (0x00000007u))
#define SMPR2_SMP_Set              ((uint32_t) (0x00000007u))

/* ADC registers reset values */
#define ADC_HTR_RESET_VALUE        ((uint32_t) (0x00000FFFu))

#define TIMxCCER_MASK              ((uint16_t)  ~0x1555u)
#define TIMxCCER_MASK_CH123        ((uint16_t)  0x555u)

#define TIMx_CC4E_BIT              ((uint16_t)  0x1000u) 

#define CONV_STARTED               ((uint32_t) (0x8))
#define CONV_FINISHED              ((uint32_t) (0xC))
#define FLAGS_CLEARED              ((uint32_t) (0x0))

#define ADC_RIGHT_ALIGNMENT 3u

#define NB_CONVERSIONS 16u

#define CLASS_VARS   ((_CPWMC)this)->Vars_str
#define DCLASS_PARAMS ((_DCIF30X_PWMC)(((_CPWMC) this)->DerivedClass))->pDParams_str
#define DCLASS_VARS  ((_DCIF30X_PWMC)(((_CPWMC) this)->DerivedClass))->DVars_str

#ifdef MC_CLASS_DYNAMIC
#include "stdlib.h" /* Used for dynamic allocation */
#else
_DCIF30X_PWMC_t IF30X_PWMCpool[MAX_DRV_PWMC_NUM];
unsigned char IF30X_PWMC_Allocated = 0u;
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

static void IF3XX_Init(CPWMC this);
static void IF3XX_TIMxInit(TIM_TypeDef* TIMx, CPWMC this);
static void IF3XX_CurrentReadingCalibration(CPWMC this);
static void IF3XX_GetPhaseCurrents(CPWMC this,Curr_Components* pStator_Currents);
static void IF3XX_TurnOnLowSides(CPWMC this);
static void IF3XX_SwitchOnPWM(CPWMC this);
static void IF3XX_SwitchOffPWM(CPWMC this);
static uint16_t IF3XX_WriteTIMRegisters(CPWMC this);   
static void *IF3XX_IRQHandler(void *this, unsigned char flag);
static uint16_t IF3XX_ExecRegularConv(CPWMC this, uint8_t bChannel);
static void IF3XX_ADC_SetSamplingTime(CPWMC this, ADConv_t ADConv_struct);
static void IF3XX_HFCurrentsCalibration(CPWMC this,Curr_Components* pStator_Currents);
static uint16_t IF3XX_IsOverCurrentOccurred(CPWMC this);
static uint16_t F3XX_GPIOPin2Source(uint16_t GPIO_Pin);
                              
/**
* @brief  Creates an object of the class ICS_F30X
* @param  pPWMnCurrFdbkParams pointer to an PWMnCurrFdbk parameters structure
* @param  pICS_DDParams pointer to an ICS_DD parameters structure
* @retval CIF30X_PWMC new instance of ICS_F30X object
*/
CIF30X_PWMC IF3XX_NewObject(pPWMnCurrFdbkParams_t pPWMnCurrFdbkParams, 
                                    pICS_F30XParams_t pICS_DDParams)
{
  _CPWMC _oPWMnCurrFdbk;
  _DCIF30X_PWMC _oICS_F30X;
  
  _oPWMnCurrFdbk = (_CPWMC)PWMC_NewObject(pPWMnCurrFdbkParams);
  
#ifdef MC_CLASS_DYNAMIC
  _oICS_F30X = (_DCIF30X_PWMC)calloc(1u,sizeof(_DCIF30X_PWMC_t));
#else
  if (IF30X_PWMC_Allocated  < MAX_DRV_PWMC_NUM)
  {
    _oICS_F30X = &IF30X_PWMCpool[IF30X_PWMC_Allocated++];
  }
  else
  {
    _oICS_F30X = MC_NULL;
  }
#endif
  
  _oICS_F30X->pDParams_str = pICS_DDParams;
  _oPWMnCurrFdbk->DerivedClass = (void*)_oICS_F30X;
  
  _oPWMnCurrFdbk->Methods_str.pIRQ_Handler = &IF3XX_IRQHandler;
  
  Set_IRQ_Handler(pICS_DDParams->IRQnb, (_CMCIRQ)_oPWMnCurrFdbk);
  
  _oPWMnCurrFdbk->Methods_str.pPWMC_Init = &IF3XX_Init;
  _oPWMnCurrFdbk->Methods_str.pPWMC_GetPhaseCurrents = &IF3XX_GetPhaseCurrents;
  _oPWMnCurrFdbk->Methods_str.pPWMC_SwitchOffPWM = &IF3XX_SwitchOffPWM;
  _oPWMnCurrFdbk->Methods_str.pPWMC_SwitchOnPWM = &IF3XX_SwitchOnPWM;        
  _oPWMnCurrFdbk->Methods_str.pPWMC_CurrentReadingCalibr = 
                                                 &IF3XX_CurrentReadingCalibration;         
  _oPWMnCurrFdbk->Methods_str.pPWMC_TurnOnLowSides = &IF3XX_TurnOnLowSides;         
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect1 = 
                                                      &IF3XX_WriteTIMRegisters;        
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect2 = 
                                                      &IF3XX_WriteTIMRegisters; 
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect3 = 
                                                      &IF3XX_WriteTIMRegisters;        
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect4 = 
                                                      &IF3XX_WriteTIMRegisters;         
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect5 = 
                                                      &IF3XX_WriteTIMRegisters;        
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect6 = 
                                                      &IF3XX_WriteTIMRegisters; 
  _oPWMnCurrFdbk->Methods_str.pPWMC_ExecRegularConv= &IF3XX_ExecRegularConv;
  
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetSamplingTime= &IF3XX_ADC_SetSamplingTime;
  
  _oPWMnCurrFdbk->Methods_str.pPWMC_IsOverCurrentOccurred = 
    &IF3XX_IsOverCurrentOccurred;
  
  return ((CIF30X_PWMC)_oPWMnCurrFdbk);
}

/** @addtogroup STM32_PMSM_MC_Library
* @{
*/

/** @addtogroup PWMnCurrFdbk_ICS_F30X
* @{
*/

/** @defgroup ICS_F30X_class_private_methods ICS_F30X class private methods
* @{
*/

/**
* @brief  It initializes TIMx, ADC, GPIO and NVIC for current reading 
*         in ICS configuration using STM32F4XX
* @param  this: related object of class CIF3XX_PWMC
* @retval none
*/
static void IF3XX_Init(CPWMC this)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  ADC_InitTypeDef ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  pVars_t pVars_str = &CLASS_VARS;
  pDVars_t pDVars_str = &DCLASS_VARS;  
  pDParams_t pDParams_str = DCLASS_PARAMS; 
  
  pDVars_str->Half_PWMPeriod = ((((_CPWMC) this)->pParams_str->hPWMperiod)/2u);
  pVars_str->bMotor = (pDParams_str->bInstanceNbr==1u?M1:M2);
    
  /* Peripheral clocks enabling ---------------------------------------------*/
  
  RCC->AHBENR |= RCC_AHBPeriph_CRC;
  
  /* ADC Periph clock enable */ 
  RCC_AHBPeriphClockCmd(pDParams_str->wAHBPeriph, ENABLE);
  
  /* Enable GPIOA-GPIOF clock */
  RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA | 
                         RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC | 
                           RCC_AHBPeriph_GPIOD | RCC_AHBPeriph_GPIOE | 
                             RCC_AHBPeriph_GPIOF, ENABLE); 
   
  if(pDParams_str->TIMx == TIM1)
  {
    /* Enable TIM1 clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    
    /* Clock source is APB high speed clock*/
    RCC_TIMCLKConfig(RCC_TIM1CLK_HCLK);
  }
  else
  {
    /* Enable TIM8 clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
    
    /* Clock source is APB high speed clock*/
    RCC_TIMCLKConfig(RCC_TIM8CLK_HCLK);
  }
  
  /* Enable the CCS */
  RCC_ClockSecuritySystemCmd((FunctionalState)(ENABLE));
	
	IF3XX_TIMxInit(pDParams_str->TIMx, this);
  
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
  GPIO_PinAFConfig(pDParams_str->hCh1Port, F3XX_GPIOPin2Source(pDParams_str->hCh1Pin), pDParams_str->bCh1AF);
  GPIO_PinAFConfig(pDParams_str->hCh2Port, F3XX_GPIOPin2Source(pDParams_str->hCh2Pin), pDParams_str->bCh2AF);
  GPIO_PinAFConfig(pDParams_str->hCh3Port, F3XX_GPIOPin2Source(pDParams_str->hCh3Pin), pDParams_str->bCh3AF);
   
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
    GPIO_PinAFConfig(pDParams_str->hCh1NPort, F3XX_GPIOPin2Source(pDParams_str->hCh1NPin), pDParams_str->bCh1NAF);
    GPIO_PinAFConfig(pDParams_str->hCh2NPort, F3XX_GPIOPin2Source(pDParams_str->hCh2NPin), pDParams_str->bCh2NAF);
    GPIO_PinAFConfig(pDParams_str->hCh3NPort, F3XX_GPIOPin2Source(pDParams_str->hCh3NPin), pDParams_str->bCh3NAF);
    
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
  if ((pDParams_str->bBKINMode) == EXT_MODE)
  {
    GPIO_PinAFConfig(pDParams_str->hBKINPort, F3XX_GPIOPin2Source(pDParams_str->hBKINPin), pDParams_str->bBKINAF);
    GPIO_InitStructure.GPIO_Pin = pDParams_str->hBKINPin;  
    GPIO_Init(pDParams_str->hBKINPort, &GPIO_InitStructure); 
    GPIO_PinLockConfig(pDParams_str->hBKINPort, pDParams_str->hBKINPin);
  }
  
  /****** Configure TIMx BKIN2 input, if enabled ******/
  if ((pDParams_str->bBKIN2Mode) == EXT_MODE)
  {
    GPIO_PinAFConfig(pDParams_str->hBKIN2Port, F3XX_GPIOPin2Source(pDParams_str->hBKIN2Pin), pDParams_str->bBKIN2AF);
    GPIO_InitStructure.GPIO_Pin = pDParams_str->hBKIN2Pin;  
    GPIO_Init(pDParams_str->hBKIN2Port, &GPIO_InitStructure); 
    GPIO_PinLockConfig(pDParams_str->hBKIN2Port, pDParams_str->hBKIN2Pin);
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
  
  /* Common init */
  ADC_CommonStructInit(&ADC_CommonInitStructure);
  ADC_CommonInitStructure.ADC_Mode=ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Clock=pDParams_str->wADC_Clock_Divider;
  ADC_CommonInitStructure.ADC_DMAAccessMode=ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_DMAMode=ADC_DMAMode_OneShot;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay= 0u;
  ADC_CommonInit(ADC1, &ADC_CommonInitStructure);
  
  ADC_VoltageRegulatorCmd(ADC1, ENABLE);
  ADC_VoltageRegulatorCmd(ADC2, ENABLE);
  
  /* Wait for Regulator Startup time, once for both */
  {
    uint16_t waittime = 0u;
    for(waittime=0u;waittime<65000u;waittime++)
    {
    }
  }    
  
  ADC_SelectCalibrationMode(ADC1,ADC_CalibrationMode_Single);    
  ADC_StartCalibration(ADC1);
  while (ADC_GetCalibrationStatus(ADC1)== SET )
  {
  }
  
  ADC_SelectCalibrationMode(ADC2,ADC_CalibrationMode_Single);
  ADC_StartCalibration(ADC2);
  while (ADC_GetCalibrationStatus(ADC2)== SET )
  {
  }

 /* Enable the ADC Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = (uint8_t) ADC1_2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 
    ADC_PRE_EMPTION_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = ADC_SUB_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  if(pDParams_str->TIMx==TIM1)
  {
    /* Enable the TIM1 Update interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = (uint8_t) TIM1_UP_TIM16_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIMx_UP_PRE_EMPTION_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = TIMx_UP_SUB_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);   
  }
  else
  {
    /* Enable the TIM1 Update interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = (uint8_t) TIM8_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIMx_UP_PRE_EMPTION_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = TIMx_UP_SUB_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);     
  }
  
  /* ADC1 and ADC2 registers configuration ---------------------------------*/
  /* Enable ADC1 and ADC2 */
  ADC_Cmd(ADC1, ENABLE);
  ADC_Cmd(ADC2, ENABLE);
  
  /* Configure the ADC_x1&2 for reg conversions */
  ADC_StructInit(&ADC_InitStructure);
  ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Disable;
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; 
  ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;
  ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;    
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Left;
  ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;
  ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;
  ADC_InitStructure.ADC_NbrOfRegChannel = 1u;
  
  ADC_Init(ADC1, &ADC_InitStructure);
  ADC_Init(ADC2, &ADC_InitStructure);
  
  ADC_InjectedDiscModeCmd(ADC1, ENABLE);
  ADC_InjectedDiscModeCmd(ADC2, ENABLE);
  
  /* It is used only to configure the sampling time to the corresponding channel*/
  ADC_InjectedChannelSampleTimeConfig(ADC1, pDParams_str->bIaChannel, pDParams_str->b_IaSamplingTime);
  ADC_InjectedChannelSampleTimeConfig(ADC2, pDParams_str->bIbChannel, pDParams_str->b_IbSamplingTime);
  

  /* ADC1 Injected conversions end interrupt enabling */
  ADC_ClearFlag(ADC1, ADC_FLAG_JEOC); 	 
  ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);
  
  /* Change channels keeping equal to 1 element the sequencer lenght */ 
  ADC1->JSQR = (uint32_t)(pDParams_str->bIaChannel)<<8 | ADC_ExternalTrigInjecEventEdge_RisingEdge;
  ADC2->JSQR = (uint32_t)(pDParams_str->bIbChannel)<<8 | ADC_ExternalTrigInjecEventEdge_RisingEdge;  
  
  ADC1->CR |= ADC_CR_JADSTART;
  ADC2->CR |= ADC_CR_JADSTART;
}

/**
* @brief  It initializes TIMx peripheral for PWM generation
* @param 'TIMx': Timer to be initialized
* @param 'this': related object of class CIF3XX_PWMC
* @retval none
*/
static void IF3XX_TIMxInit(TIM_TypeDef* TIMx, CPWMC this)
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
  
  /* Set channel 4 as TRGO */
  TIM_SelectOutputTrigger(TIMx,TIM_TRGOSource_OC4Ref);
  
  TIM_BDTRStructInit_MC(&TIMx_BDTRInitStructure);
  /* Dead Time */
  TIMx_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
  TIMx_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
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
 void IF3XX_StartTimers(void)
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
* @param  this: related object of class CIF3XX_PWMC
* @retval none
*/
static void IF3XX_CurrentReadingCalibration(CPWMC this)
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
  ((_CPWMC) this)->Methods_str.pPWMC_GetPhaseCurrents = &IF3XX_HFCurrentsCalibration;
  
  IF3XX_SwitchOnPWM(this);
  
  /* Wait for NB_CONVERSIONS to be executed */
  while (pDVars_str->bIndex < (NB_CONVERSIONS))
  {
  }  
  
  IF3XX_SwitchOffPWM( this);
  
  pDVars_str->wPhaseAOffset >>=4; 
  pDVars_str->wPhaseBOffset >>=4; 
 
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
  ((_CPWMC) this)->Methods_str.pPWMC_GetPhaseCurrents = &IF3XX_GetPhaseCurrents;
}

/**
* @brief  It computes and return latest converted motor phase currents motor
* @param  this: related object of class CIF3XX_PWMC
* @retval Ia and Ib current in Curr_Components format
*/
static void IF3XX_GetPhaseCurrents(CPWMC this,Curr_Components* pStator_Currents)
{
  int32_t wAux;
  uint16_t hReg;
  /* Derived class members container */
  pDVars_t pDVars_str = &DCLASS_VARS;  

  /* Reset the SOFOC flag to indicate the start of FOC algorithm*/
  pDVars_str->hFlags &= (~SOFOC); 
  
 /* Ia = (hPhaseAOffset)-(PHASE_A_ADC_CHANNEL vale)  */
  hReg = (uint16_t)(ADC1->JDR1);
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
  hReg = (uint16_t)(ADC2->JDR1);
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
* @param  this: related object of class CIF3XX_PWMC
* @retval It always returns {0,0} in Curr_Components format
*/
static void IF3XX_HFCurrentsCalibration(CPWMC this,Curr_Components* pStator_Currents)
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
  * @param  this: related object of class CIF3XX_PWMC
  * @retval none
  */
static void IF3XX_TurnOnLowSides(CPWMC this)
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
* @param  this: related object of class CIF3XX_PWMC
* @retval none
*/
static void IF3XX_SwitchOnPWM(CPWMC this)
{  
  pDParams_t pLocalDParams = DCLASS_PARAMS;
  TIM_TypeDef*  LocalTIMx = DCLASS_PARAMS->TIMx;  
  pDVars_t pDVars_str = &DCLASS_VARS;  
 
  /* It clears ADCs JSTRT and JEOC bits */
  ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);
  ADC_ClearFlag(ADC2, ADC_FLAG_JEOC);
  
  /* Clear Update Flag */
  TIM_ClearFlag(LocalTIMx, TIM_FLAG_Update);
  
  /* Enable TIMx preload and ADC trigger on next update */
  LocalTIMx->CCMR2 = 0x7868u;
  LocalTIMx->CCR4 = (uint32_t)(pDVars_str->Half_PWMPeriod)-5u;
    
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
* @param  this: related object of class CIF3XX_PWMC
* @retval none
*/
static void IF3XX_SwitchOffPWM(CPWMC this)
{ 
  pDParams_t pLocalDParams = DCLASS_PARAMS;
  TIM_TypeDef*  LocalTIMx = DCLASS_PARAMS->TIMx;
  pDVars_t pDVars_str = &DCLASS_VARS;  
  
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
* @param  this: related object of class CIF3XX_PWMC
* @retval none
*/
static uint16_t IF3XX_WriteTIMRegisters(CPWMC this)
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
* @param  this: related object of class CIF3XX_PWMC
* @retval none
*/
static void *IF3XX_IRQHandler(void *this, unsigned char flag)
{
  pVars_t pVars_str = &CLASS_VARS;
  pDParams_t pDParams_str = DCLASS_PARAMS;
  pDVars_t pDVars_str = &DCLASS_VARS;  
  
/* Set the SOFOC flag to indicate the execution of Update IRQ*/
pDVars_str->hFlags |= SOFOC;

/* Switch Context */
/* It re-initilize AD converter in run time when using dual MC */
/* Removed because the trigger is set writing directly the JSQR */

/* Change channels keeping equal to 1 element the sequencer lenght */ 
ADC1->JSQR = (uint32_t)(pDParams_str->bIaChannel)<<8 | ADC_ExternalTrigInjecEventEdge_RisingEdge;
ADC2->JSQR = (uint32_t)(pDParams_str->bIbChannel)<<8 | ADC_ExternalTrigInjecEventEdge_RisingEdge;  

return &(pVars_str->bMotor);
}

/**
* @brief  Execute a regular conversion using ADC1. 
*         The function is not re-entrant (can't executed twice at the same time)
* @param  this related object of class CIF3XX_PWMC
* @retval It returns converted value or oxFFFF for conversion error
*/
static uint16_t IF3XX_ExecRegularConv(CPWMC this, uint8_t bChannel)
{
  ADC1->SQR1 = bChannel << 6;
    
  /* Clear EOC flag of ADC1 */
  ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
  
  /* It starts software regular conversion */
  ADC_StartConversion(ADC1);
  
  /* Wait end of conversion */
  while ((ADC1->ISR & ADC_FLAG_EOC) == 0u)
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
static void IF3XX_ADC_SetSamplingTime(CPWMC this, ADConv_t ADConv_struct)
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
static uint16_t IF3XX_IsOverCurrentOccurred(CPWMC this)
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
static uint16_t F3XX_GPIOPin2Source(uint16_t GPIO_Pin)
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
