/**
  ******************************************************************************
  * @file    main.c 
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   Main program body
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

#include "MCTuningClass.h"
#include "MCInterfaceClass.h"

#if defined(PFC_ENABLED)
  #include "PFCInit.h"
  #include "PFCApplication.h"
#endif

#include "MCTasks.h"
#include "Parameters conversion.h"
#ifdef DUALDRIVE
#include "Parameters conversion motor 2.h"
#endif
#include "Timebase.h"
#include "UITask.h"
#include "MCLibraryISRPriorityConf.h"

#include <stdio.h>

/******************************************************************************/  
/* Electronic Speed Controller (STEVAL-ESC001V1)*/
#include "STEVAL_ESC001V1.h"
/******************************************************************************/  

#if (defined(USE_STM32303C_EVAL))
#include "stm32303c_eval.h"
#elif USE_EVAL
#include "stm32_eval.h"
#endif

#ifdef USE_STGAP1S
#include "GAPApplication.h"
#endif

#ifdef STSPIN32F0
void STSPIN32F0_Init(void);
#endif

#define INTERNAL 0
#define EXTERNAL 1
#define STM32F3_64MHZ_INT  ((CLOCK_SOURCE == INTERNAL) && defined(STM32F30X) && defined(CPU_CLK_64_MHZ))

#if STM32F3_64MHZ_INT
void STM32F3_64MHz_Internal(void);
#endif 

#define FIRMWARE_VERS "STM32 FOC SDK\0Ver.4.3.0"
const char s_fwVer[32] = FIRMWARE_VERS;

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

#if defined(EXAMPLE_SPEEDMONITOR)
  void speedmonitor_start(void);
#endif
#if defined(EXAMPLE_POTENTIOMETER)
 void potentiometer_start(void);  
#endif   
#if defined(EXAMPLE_RAMP)
  void ramp_start(void);
#endif   
#if defined(EXAMPLE_PI)
  void NewPIval_start(void);
#endif    
#if defined(EXAMPLE_CONTROLMODE)
 void TqSpeedMode_start(void);
#endif 
    
/* Private function prototypes -----------------------------------------------*/

void SysTick_Configuration(void);

/* Private variables ---------------------------------------------------------*/

CMCI oMCI[MC_NUM];
CMCT oMCT[MC_NUM];  
uint32_t wConfig[MC_NUM] = {UI_CONFIG_M1,UI_CONFIG_M2};


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/  

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{    
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */
  
#if !defined(STM32F0XX)
  /*NVIC Priority group configuration.
    Default option is NVIC_PriorityGroup_3. 
  */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
#endif
  
#ifdef USE_STGAP1S
  GAPboot();
#endif

#if STM32F3_64MHZ_INT
 STM32F3_64MHz_Internal();
#endif

#ifdef STSPIN32F0
  STSPIN32F0_Init();
#endif
    
  /*MCInterface and MCTuning boot*/
  MCboot(oMCI,oMCT);
  
/******************************************************************************/  
 /*Initialization of Electronic Speed Controller (STEVAL-ESC001V1)*/
  ESCboot();
/******************************************************************************/    
  
  #if defined(PFC_ENABLED)
    PFC_Boot(oMCT[0],(CMCT)MC_NULL, (int16_t *)MC_NULL);
  #endif
    
  /*Systick configuration.*/
  SysTick_Configuration();
  
  /* Start here ***************************************************************/
  /* GUI, this section is present only if LCD, DAC or serial communication is */
  /* enabled.                                                                 */
#if (defined(LCD_FUNCTIONALITY) | defined(DAC_FUNCTIONALITY) | defined(SERIAL_COMMUNICATION))
  UI_TaskInit(UI_INIT_CFG,wConfig,MC_NUM,oMCI,oMCT,s_fwVer);
#endif  
  
#ifdef ENABLE_START_STOP_BUTTON
  /* Init Key input (Start/Stop button) */
  {
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = START_STOP_GPIO_PIN;
    GPIO_Init(START_STOP_GPIO_PORT, &GPIO_InitStructure);
  }
#endif
  /* End here******************************************************************/  
  
  while(1)
  {        
#ifdef SERIAL_COMMUNICATION
    /* Start here ***********************************************************/
    /* GUI, this section is present only if serial communication is enabled.*/
    if (UI_SerialCommunicationTimeOutHasElapsed())
    {
      // Send timeout message
      Exec_UI_IRQ_Handler(UI_IRQ_USART,3,0); // Flag 3 = Send timeout error*/
    }
    if (UI_SerialCommunicationATRTimeHasElapsed())
    {
      // Send ATR message
      Exec_UI_IRQ_Handler(UI_IRQ_USART,4,0); // Flag 4 = Send ATR message*/
    }
    /* End here**************************************************************/
#endif

#if (defined(LCD_FUNCTIONALITY) || defined(ENABLE_START_STOP_BUTTON))
    /* Start here ***********************************************************/
    /* GUI, this section is present only if LCD or start/stop button is enabled. */
    if (UI_IdleTimeHasElapsed())
    {  
      UI_SetIdleTime(UI_TASK_OCCURENCE_TICKS);
      
#ifdef LCD_FUNCTIONALITY
      UI_LCDRefresh();
#endif
      
#ifdef ENABLE_START_STOP_BUTTON
      {
        /* Chek status of Start/Stop button and performs debounce management */
        static uint16_t hKeyButtonDebounceCounter = 0u;
        if ((GPIO_ReadInputDataBit(START_STOP_GPIO_PORT, START_STOP_GPIO_PIN) == START_STOP_POLARITY) &&
            (hKeyButtonDebounceCounter == 0))
        {

#ifdef SINGLEDRIVE
          /* Queries the STM and a command start or stop depending on the state. */
          /* It can be added to MCI functionality */
          if (MCI_GetSTMState(oMCI[M1]) == IDLE)
          {
            MCI_StartMotor(oMCI[M1]);
          }
          else
          {
            MCI_StopMotor(oMCI[M1]);
          }
#endif
        
#ifdef DUALDRIVE
          /* Stop both motors */
          MCI_StopMotor(oMCI[M1]);
          MCI_StopMotor(oMCI[M2]);
#endif

          hKeyButtonDebounceCounter = 4u; /* Debounce time xx * LCD Clock */
        }
        if (hKeyButtonDebounceCounter > 0)
        {
          hKeyButtonDebounceCounter--;
        }
      }
#endif
      
    }
    /* End here**************************************************************/  
#endif

/********************************   EXAMPLE AREA ******************************/
#if defined(EXAMPLE_POTENTIOMETER)
   potentiometer_start();  
#endif   
#if defined(EXAMPLE_RAMP)
   ramp_start();
#endif   
#if defined(EXAMPLE_PI)
   NewPIval_start();
#endif    
#if defined(EXAMPLE_CONTROLMODE)
   TqSpeedMode_start();
#endif
#if defined(EXAMPLE_SPEEDMONITOR)
   speedmonitor_start();
#endif
/*****************************************************************************/
   
#ifdef USE_STGAP1S
   GAPSchedule();
#endif
   
/******************************************************************************/   
/* Main routine for Electronic Speed Controller (STEVAL-ESC001V1) */
  PWM_FC_control(); 
/******************************************************************************/   
   
  }
}
/**
  * @brief  Configures the SysTick.
  * @param  None
  * @retval None
  */
void SysTick_Configuration(void)
{
  /* Setup SysTick Timer for 500 usec interrupts  */
  if (SysTick_Config((SystemCoreClock) / SYS_TICK_FREQUENCY))
  { 
    /* Capture error */ 
    while (1);
  }
  
  NVIC_SetPriority(SysTick_IRQn, SYSTICK_PRIORITY);
  NVIC_SetPriority(PendSV_IRQn, PENDSV_PRIORITY);
}

#if STM32F3_64MHZ_INT
void STM32F3_64MHz_Internal()
{
#warning "Internal"
  /* Cleaning of Source clock register */
  RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
  
  /* Disable PLL */
  RCC_PLLCmd((FunctionalState)DISABLE);
  
  /* Wait untill PLL is cleared */
  while((RCC->CR & RCC_CR_PLLRDY) == 1)
  {
  }
  
  /* Setting of system clock to 64 MHz */
  RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_16); //8/2*16 = 64MHz
  
  /* Enable PLL */
  RCC_PLLCmd((FunctionalState)ENABLE);
  
  /* Wait till PLL is ready */
  while((RCC->CR & RCC_CR_PLLRDY) == 0)
  {
  }
  
  /* Select PLL as system clock source */
  RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;    
  
  /* Wait till PLL is used as system clock source */
  while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)RCC_CFGR_SWS_PLL)
  {
  }
  
  /* HSE disabling */
  RCC_HSEConfig(RCC_HSE_OFF);
  
  /* Wait the disabling of HSE */
  while(RCC_GetFlagStatus(RCC_FLAG_HSERDY)==1)
  {
  }
}
#endif

#ifdef STSPIN32F0
void STSPIN32F0_Init(void)
{
	/** This function is dedicated to the manual setting for STSPIN32F0. **/
	
	/** Setting of internal clock source **/
	
  /* Cleaning of Source clock register */
  RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
  
  /* Disable PLL */
  RCC_PLLCmd((FunctionalState)DISABLE);
  
  /* Wait untill PLL is cleared */
  while((RCC->CR & RCC_CR_PLLRDY) == 1)
  {
  }
  
  /* Setting of system clock to 48 MHz */
  RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_CFGR_PLLMUL12); //8/2*12 = 48MHz
  
  /* Enable PLL */
  RCC_PLLCmd((FunctionalState)ENABLE);
  
  /* Wait till PLL is ready */
  while((RCC->CR & RCC_CR_PLLRDY) == 0)
  {
  }
  
  /* Select PLL as system clock source */
  RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;    
  
  /* Wait till PLL is used as system clock source */
  while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)RCC_CFGR_SWS_PLL)
  {
  }
  
  /* HSE disabling */
  RCC_HSEConfig(RCC_HSE_OFF);
  
  /* Wait the disabling of HSE */
  while(RCC_GetFlagStatus(RCC_FLAG_HSERDY)==1)
  {
  }
  
  /** Setting for OC protection th **/
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF,(FunctionalState) ENABLE);
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOF, &GPIO_InitStructure);
  GPIOF->BSRR = GPIO_Pin_7;
  GPIOF->BRR = GPIO_Pin_6;
}
#endif

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
