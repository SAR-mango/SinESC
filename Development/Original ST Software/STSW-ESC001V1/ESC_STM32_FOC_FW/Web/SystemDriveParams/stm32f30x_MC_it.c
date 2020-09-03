/**
  ******************************************************************************
  * @file    stm32f30x_MC_it.c 
  * @author  STMCWB ver.4.3.0.16508
  * @version 4.3.0
  * @date    2017-21-03 11:47:54
  * @project SDK43x-STM32F303-STEVAL_ESC001V1.stmcx
  * @path    C:\Users\giuseppe scuderi-sl\Desktop\STSW-ESC001V1\ESC_STMCWB_prj
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

#if defined(PFC_ENABLED)
  #include "PFCApplication.h"
#endif

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif
/**
  * @brief  This function handles ADC1/ADC2 interrupt request.
  * @param  None
  * @retval None
  */
void ADC1_2_IRQHandler(void)
{
  // Shared IRQ management - begin
  
  
  // Clear Flags Single or M1
  ADC1->ISR = (uint32_t)ADC_FLAG_JEOS;
  
  // FIFO Update Single or M1
  

  // Highfrequency task Single or M1
  TSK_HighFrequencyTask();
  
  // Shared IRQ management - middle
  
  
  // Clear Flags M2
  
  
  // FIFO Update M2
  

  // Highfrequency task M2
  
  
  // Shared IRQ management - end
  
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif
/**
  * @brief  This function handles ADC3 interrupt request.
  * @param  None
  * @retval None
  */
void ADC3_IRQHandler(void)
{
  // Clear Flags
  
  
  // FIFO Update ADC3
  

  // Highfrequency task ADC3
  
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif
/**
  * @brief  This function handles ADC4 interrupt request.
  * @param  None
  * @retval None
  */
void ADC4_IRQHandler(void)
{
  // Clear Flags
  
  
  // FIFO Update ADC4
  

  // Highfrequency task ADC4
  
}

/**
  * @brief  This function handles first motor TIMx Update interrupt request.
  * @param  None
  * @retval None 
  */
void TIMx_UP_M1_IRQHandler(void)
{
#if defined(PFC_ENABLED)
  if(TIM16->SR & TIM_FLAG_Break)
  {
    TIM16->SR = (uint16_t)(~TIM_FLAG_Break);
    PFC_OCP_Processing();
  } else
  {
#endif
  PWM_TIMER_SELECTION->SR = (uint16_t)(~TIM_FLAG_Update);
#ifdef SINGLEDRIVE
  Exec_IRQ_Handler(MC_IRQ_PWMNCURRFDBK_1,TIMx_UP_M1_IRQFlag);
#else
  TSK_DualDriveFIFOUpdate(Exec_IRQ_Handler(MC_IRQ_PWMNCURRFDBK_1,TIMx_UP_M1_IRQFlag));
#endif

#if defined(PFC_ENABLED)
}
#endif
}

#if (defined(PFC_ENABLED) && defined(MC1USETIM8) && defined(SINGLEDRIVE))
/**
  * @brief  This function handles PFC function if motor 1 use TIM8.
  * @param  None
  * @retval None 
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
  if(TIM16->SR & TIM_FLAG_Break)
  {
    TIM16->SR = (uint16_t)(~TIM_FLAG_Break);
    PFC_OCP_Processing();
  } 
}
#endif

void TIMx_BRK_M1_IRQHandler(void)
{
  if (PWM_TIMER_SELECTION->SR & TIM_FLAG_Break)
  {
    PWM_TIMER_SELECTION->SR = (uint16_t)(~TIM_FLAG_Break);
    Exec_IRQ_Handler(MC_IRQ_PWMNCURRFDBK_1,3);
  }
  if (PWM_TIMER_SELECTION->SR & TIM_FLAG_Break2)
  {
    PWM_TIMER_SELECTION->SR = (uint16_t)(~TIM_FLAG_Break2);
    Exec_IRQ_Handler(MC_IRQ_PWMNCURRFDBK_1,2);
  }
  /* Systick is not executed due low priority so is necessary to call MC_Scheduler here.*/
  MC_Scheduler();
  
}

#if defined(DUALDRIVE)
/**
  * @brief  This function handles second motor TIMx Update interrupt request.
  * @param  None
  * @retval None 
  */
void TIMx_UP_M2_IRQHandler(void)
{
#if defined(PFC_ENABLED)
  if(TIM16->SR & TIM_FLAG_Break)
  {
    TIM16->SR = (uint16_t)(~TIM_FLAG_Break);
    PFC_OCP_Processing();
  } else
  {
#endif
  PWM_TIMER_SELECTION2->SR = (uint16_t)(~TIM_FLAG_Update);
#ifdef SINGLEDRIVE
  Exec_IRQ_Handler(MC_IRQ_PWMNCURRFDBK_2,TIMx_UP_M2_IRQFlag);
#else
  TSK_DualDriveFIFOUpdate(Exec_IRQ_Handler(MC_IRQ_PWMNCURRFDBK_2,TIMx_UP_M2_IRQFlag));
#endif
#if defined(PFC_ENABLED)
}
#endif
}

void TIMx_BRK_M2_IRQHandler(void)
{
  if (PWM_TIMER_SELECTION2->SR & TIM_FLAG_Break)
  {
    PWM_TIMER_SELECTION2->SR = (uint16_t)(~TIM_FLAG_Break);
    Exec_IRQ_Handler(MC_IRQ_PWMNCURRFDBK_2,3);
  }
  if (PWM_TIMER_SELECTION2->SR & TIM_FLAG_Break2)
  {
    PWM_TIMER_SELECTION2->SR = (uint16_t)(~TIM_FLAG_Break2);
    Exec_IRQ_Handler(MC_IRQ_PWMNCURRFDBK_2,2);
  }
  /* Systick is not executed due low priority so is necessary to call MC_Scheduler here.*/
  MC_Scheduler();
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

  if (USART_GetFlagStatus(USART, USART_FLAG_ORE) == SET) /* Overrun error occurs */
  {
    /* Send Overrun message */
    Exec_UI_IRQ_Handler(UI_IRQ_USART,2,0); /* Flag 2 = Send overrun error*/
    USART_ClearFlag(USART,USART_FLAG_ORE); /* Clear overrun flag */
    UI_SerialCommunicationTimeOutStop();
  }
  else if (USART_GetITStatus(USART, USART_IT_TXE) != RESET)
  {   
    Exec_UI_IRQ_Handler(UI_IRQ_USART,1,0); /* Flag 1 = TX */
  }  
  else /* Valid data have been received */
  {
    uint16_t retVal;
    retVal = *(uint16_t*)(Exec_UI_IRQ_Handler(UI_IRQ_USART,0,USART_ReceiveData(USART))); /* Flag 0 = RX */
    if (retVal == 1)
    {
      UI_SerialCommunicationTimeOutStart();
    }
    if (retVal == 2)
    {
      UI_SerialCommunicationTimeOutStop();
    }
  }
}
/*End here***********************************************************/
#endif

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  TSK_HardwareFaultTask();
  
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
#if defined(LCD_FUNCTIONALITY)
    UI_LCDRefresh();
#else 
  #if defined(SERIAL_COMMUNICATION)
    {
      typedef void* (*pExec_UI_IRQ_Handler_t) (unsigned char bIRQAddr, 
                                               unsigned char flag);
      
      if (USART_GetFlagStatus(USART, USART_FLAG_ORE) == SET) /* Overrun error occurs */
      {
        /* Send Overrun message */
        Exec_UI_IRQ_Handler(UI_IRQ_USART,2,0); /* Flag 2 = Send overrun error*/
        USART_ClearFlag(USART,USART_FLAG_ORE); /* Clear overrun flag */
        UI_SerialCommunicationTimeOutStop();
      }
      else if (USART_GetITStatus(USART, USART_IT_TXE) != RESET)
      {   
        Exec_UI_IRQ_Handler(UI_IRQ_USART,1,0); /* Flag 1 = TX */
      }  
      else if (USART_GetITStatus(USART, USART_IT_RXNE) != RESET) /* Valid data have been received */
      {
        uint16_t retVal;
        retVal = *(uint16_t*)(Exec_UI_IRQ_Handler(UI_IRQ_USART,0,USART_ReceiveData(USART))); /* Flag 0 = RX */
        if (retVal == 1)
        {
          UI_SerialCommunicationTimeOutStart();
        }
        if (retVal == 2)
        {
          UI_SerialCommunicationTimeOutStop();
        }
      }
      else
      {
      }
    }  
  #endif
#endif
  }
}

/* ##@@_USER_CODE_START_##@@ */
/* ##@@_USER_CODE_END_##@@ */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
