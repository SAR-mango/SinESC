/**
  ******************************************************************************
  * @file    stm32f4xx_MC_it.c 
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine, related to Motor Control
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
/* Pre-compiler coherency check */
#define PROJECT_CHK
#include "CrossCheck.h" 
#undef PROJECT_CHK
#include "MCIRQHandlerClass.h"
#include "UIIRQHandlerClass.h"
#include "MCInterfaceClass.h"
#include "MCTuningClass.h"
#include "MC_type.h"
#include "MCTasks.h"
#include "UITask.h"
#include "Timebase.h"
#include "Parameters conversion.h"
#include "Parameters conversion motor 2.h"

/** @addtogroup STM32F4xx_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles ADC1/ADC2 interrupt request.
  * @param  None
  * @retval None
  */
void ADC_IRQHandler(void)
{
  if((ADC1->SR & ADC_FLAG_JEOC) == ADC_FLAG_JEOC)
  {    
    // Clear Flags
    ADC1->SR &= ~(u32)(ADC_FLAG_JEOC | ADC_FLAG_JSTRT);
  
#ifdef DAC_FUNCTIONALITY
    UI_DACUpdate(TSK_HighFrequencyTask());  /*GUI, this section is present only if DAC is enabled*/
#else
    TSK_HighFrequencyTask();          /*GUI, this section is present only if DAC is disabled*/
#endif  
  }
  else
  {
    // Clear Flags
    ADC3->SR &= ~(u32)(ADC_FLAG_JEOC | ADC_FLAG_JSTRT);
    
#ifdef DAC_FUNCTIONALITY
    UI_DACUpdate(TSK_HighFrequencyTask());  /*GUI, this section is present only if DAC is enabled*/
#else
    TSK_HighFrequencyTask();          /*GUI, this section is present only if DAC is disabled*/
#endif  
  }
}

/**
  * @brief  This function handles first motor TIMx Update interrupt request.
  * @param  None
  * @retval None
  */
void TIMx_UP_M1_IRQHandler(void)
{
  PWM_TIMER_SELECTION->SR = (uint16_t)(~TIM_FLAG_Update);
#ifdef SINGLEDRIVE
  Exec_IRQ_Handler(MC_IRQ_PWMNCURRFDBK_1,TIMx_UP_M1_IRQFlag);
#else
  TSK_DualDriveFIFOUpdate(Exec_IRQ_Handler(MC_IRQ_PWMNCURRFDBK_1,TIMx_UP_M1_IRQFlag));
#endif
}

#if defined(DUALDRIVE)
/**
  * @brief  This function handles second motor TIMx Update interrupt request.
  * @param  None
  * @retval None
  */
void TIMx_UP_M2_IRQHandler(void)
{
  PWM_TIMER_SELECTION2->SR = (uint16_t)(~TIM_FLAG_Update);
#ifdef SINGLEDRIVE
  Exec_IRQ_Handler(MC_IRQ_PWMNCURRFDBK_2,TIMx_UP_M2_IRQFlag);
#else
  TSK_DualDriveFIFOUpdate(Exec_IRQ_Handler(MC_IRQ_PWMNCURRFDBK_2,TIMx_UP_M2_IRQFlag));
#endif
}
#endif

/**
  * @brief  This function handles first motor DMAx TC interrupt request. 
  *         Required only for R1 with rep rate > 1
  * @param  None
  * @retval None
  */
void DMAx_R1_M1_IRQHandler(void)
{
  if (DMA_GetFlagStatus(DMAx_R1_M1_Stream,DMAx_R1_M1_Flag) == SET)
  {
    DMA_ClearFlag(DMAx_R1_M1_Stream,DMAx_R1_M1_Flag);
    Exec_IRQ_Handler(MC_IRQ_PWMNCURRFDBK_1,2);
  } 
}

#if defined(DUALDRIVE)
/**
  * @brief  This function handles second motor DMAx TC interrupt request. 
  *         Required only for R1 with rep rate > 1
  * @param  None
  * @retval None
  */
void DMAx_R1_M2_IRQHandler(void)
{
  if (DMA_GetFlagStatus(DMAx_R1_M2_Stream,DMAx_R1_M2_Flag) == SET)
  {
    DMA_ClearFlag(DMAx_R1_M2_Stream,DMAx_R1_M2_Flag);
    Exec_IRQ_Handler(MC_IRQ_PWMNCURRFDBK_2,2);
  } 
}
#endif

#if ((defined ENCODER) || (defined AUX_ENCODER) || (defined HALL_SENSORS) \
                                                  || (defined AUX_HALL_SENSORS))
/**
  * @brief  This function handles TIMx global interrupt request for M1 Speed Sensor.
  * @param  None
  * @retval None
  */
void SPD_TIM_M1_IRQHandler(void)
{
#if ((defined HALL_SENSORS) || (defined AUX_HALL_SENSORS))
  if ((HALL_TIMER->SR & TIM_FLAG_Update) != 0)
  {
    HALL_TIMER->SR = (u16)~TIM_FLAG_Update;
    Exec_IRQ_Handler(MC_IRQ_SPEEDNPOSFDBK_1,1);
  }
  else
  {
    HALL_TIMER->SR = (u16)~TIM_FLAG_CC1;
    Exec_IRQ_Handler(MC_IRQ_SPEEDNPOSFDBK_1,0);
  }
#else
  { 
    Exec_IRQ_Handler(MC_IRQ_SPEEDNPOSFDBK_1,0);
  }
#endif
}
#endif

#if defined(DUALDRIVE)
#if ((defined ENCODER2) || (defined AUX_ENCODER2) || (defined HALL_SENSORS2) \
                                                  || (defined AUX_HALL_SENSORS2))
/**
  * @brief  This function handles TIMx global interrupt request for M2 Speed Sensor.
  * @param  None
  * @retval None
  */
void SPD_TIM_M2_IRQHandler(void)
{
#if ((defined HALL_SENSORS2) || (defined AUX_HALL_SENSORS2))
  if ((HALL_TIMER2->SR & TIM_FLAG_Update) != 0)
  {
    HALL_TIMER2->SR = (u16)~TIM_FLAG_Update;
    Exec_IRQ_Handler(MC_IRQ_SPEEDNPOSFDBK_2,1);
  }
  else
  {
    HALL_TIMER2->SR = (u16)~TIM_FLAG_CC1;
    Exec_IRQ_Handler(MC_IRQ_SPEEDNPOSFDBK_2,0);
  }
#else
  { 
    Exec_IRQ_Handler(MC_IRQ_SPEEDNPOSFDBK_2,0);
  }
#endif
}
#endif
#endif

#ifdef SERIAL_COMMUNICATION
/*Start here***********************************************************/
/*GUI, this section is present only if serial communication is enabled*/
/**
  * @brief  This function handles USART interrupt request.
  * @param  None
  * @retval None
  */
void USART_IRQHandler(void)
{
  typedef void* (*pExec_UI_IRQ_Handler_t) (unsigned char bIRQAddr, 
                                                            unsigned char flag);
  uint16_t hUSART_SR = USART->SR;
  uint16_t hUSART_DR = USART->DR;
  
  if (hUSART_SR & USART_SR_ORE) /* Overrun error occurs before SR access */
  {
    /* Send Overrun message */
    UI_SerialCommunicationTimeOutStop();
    Exec_UI_IRQ_Handler(UI_IRQ_USART,2,0); /* Flag 2 = Send overrun error*/
  }
  else if (USART->SR & USART_SR_ORE) // Overrun error occurs after SR access and before DR access
  {
    /* Send Overrun message */
    UI_SerialCommunicationTimeOutStop();
    Exec_UI_IRQ_Handler(UI_IRQ_USART,2,0); /* Flag 2 = Send overrun error */
    USART->DR;
  }
  else if (hUSART_SR & USART_SR_RXNE) /* Valid data received */
  {
    uint16_t retVal;
    retVal = *(uint16_t*)(Exec_UI_IRQ_Handler(UI_IRQ_USART,0,hUSART_DR)); /* Flag 0 = RX */
    if (retVal == 1)
    {
      UI_SerialCommunicationTimeOutStart();
    }
    if (retVal == 2)
    {
      UI_SerialCommunicationTimeOutStop();
    }
  }
    
  if(USART_GetITStatus(USART, USART_IT_TXE) != RESET)
  {   
    Exec_UI_IRQ_Handler(UI_IRQ_USART,1,0); /* Flag 1 = TX */
  }
  
}
/*End here***********************************************************/
#endif

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
