/**
  ******************************************************************************
  * @file    stm32f0xx_MC_it.c 
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

/** @addtogroup STM32F0xx_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles curren regulation interrupt request.
  * @param  None
  * @retval None
  */
void CURRENT_REGULATION_IRQHandler(void)
{  
#if defined(SINGLE_SHUNT)
  /* Clear Flags */
  DMA1->IFCR = (DMA1_FLAG_GL2|DMA1_FLAG_TC2|DMA1_FLAG_HT2);
#elif defined (THREE_SHUNT)
  /* Clear Flags */
  DMA_ClearITPendingBit(DMA1_IT_TC1|DMA1_IT_GL1|DMA1_IT_HT1);
#endif
  
#ifdef DAC_FUNCTIONALITY
  UI_DACUpdate(TSK_HighFrequencyTask());  /*GUI, this section is present only if DAC is enabled*/
#else
  TSK_HighFrequencyTask();          /*GUI, this section is present only if DAC is disabled*/
#endif      
}

/**
  * @brief  This function handles first motor TIMx Update, Break-in interrupt request.
  * @param  None
  * @retval None
  */
void TIMx_UP_BRK_M1_IRQHandler(void)
{
  if(TIM_GetFlagStatus(PWM_TIMER_SELECTION,TIM_FLAG_Update)== SET)
  {
    TIM_ClearFlag(PWM_TIMER_SELECTION,TIM_FLAG_Update);
    Exec_IRQ_Handler(MC_IRQ_PWMNCURRFDBK_1,TIMx_UP_M1_IRQFlag); 
  }
  else if(TIM_GetFlagStatus(PWM_TIMER_SELECTION,TIM_FLAG_Break)== SET)
  {
    PWM_TIMER_SELECTION->SR = (uint16_t)(~TIM_FLAG_Break);
    Exec_IRQ_Handler(MC_IRQ_PWMNCURRFDBK_1,TIMx_BRK_M1_IRQFlag);
  }
}

/**
  * @brief  This function handles first motor DMAx TC interrupt request. 
  *         Required only for R1 with rep rate > 1
  * @param  None
  * @retval None
  */
void DMAx_R1_M1_IRQHandler(void)
{
  if (DMA_GetFlagStatus(DMAx_R1_M1_TC_Flag) == SET)
  {
    DMA_ClearFlag(DMAx_R1_M1_TC_Flag);
    Exec_IRQ_Handler(MC_IRQ_PWMNCURRFDBK_1,DMAx_TC_M1_IRQFlag);
  } 
}

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
    HALL_TIMER->SR = (uint16_t)~TIM_FLAG_Update;
    Exec_IRQ_Handler(MC_IRQ_SPEEDNPOSFDBK_1,1);
  }
  else
  {
    HALL_TIMER->SR = (uint16_t)~TIM_FLAG_CC1;
    Exec_IRQ_Handler(MC_IRQ_SPEEDNPOSFDBK_1,0);
  }
#else
  { 
    Exec_IRQ_Handler(MC_IRQ_SPEEDNPOSFDBK_1,0);
  }
#endif
}
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
  if (USART_GetFlagStatus(USART, USART_FLAG_ORE) == SET) /* Overrun error occurs */
  {
    /* Send Overrun message */
    Exec_UI_IRQ_Handler(UI_IRQ_USART,2,0); /* Flag 2 = Send overrun error*/
    USART_ClearFlag(USART,USART_FLAG_ORE); /* Clear overrun flag */
  }
  else if (USART_GetITStatus(USART, USART_IT_TXE) != RESET)
  {   
    Exec_UI_IRQ_Handler(UI_IRQ_USART,1,0); /* Flag 1 = TX */
  }  
  else /* Valid data have been received */
  {
    Exec_UI_IRQ_Handler(UI_IRQ_USART,0,0); /* Flag 0 = RX */
    USART_RequestCmd(UI_IRQ_USART,USART_Request_RXFRQ,(FunctionalState)(ENABLE)); /* Clear RXNE flag */
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
