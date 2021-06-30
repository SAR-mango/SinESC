/**
  ******************************************************************************
  * @file    STEVAL_ESC001V1.c
  * @author  IMS Systems Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   ESC Eval board      
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
#include "MC.h"
#include "STEVAL_ESC001V1.h"
#include "Parameters conversion.h"
#include "Timebase.h"

/* Define --------------------------------------------------------------------*/
#define SM_ARMING       0x00
#define SM_ARMED        0x01 
#define SM_POSITIVE_RUN 0x02
#define SM_STOP         0x03
#define COUNT_MAX_SEC  1
#define STOP_DURATION_SEC 2
#define COUNT_MAX (COUNT_MAX_SEC * USER_TIMEBASE_FREQUENCY_HZ)
#define STOP_DURATION  (STOP_DURATION_SEC * USER_TIMEBASE_FREQUENCY_HZ)
#define USER_TIMEBASE_FREQUENCY_HZ        400
#define USER_TIMEBASE_OCCURENCE_TICKS  (SYS_TICK_FREQUENCY/USER_TIMEBASE_FREQUENCY_HZ)-1u

#define FILTER_DEEP 4                                  /*!<  PWM Low pass filter variable */
#define PWM_ESC                                        /*!<  If undef enables the WB tool (ESC function disables) */

ADC_InitTypeDef ADC_InitStructure;
ADC_CommonInitTypeDef ADC_CommonInitStructure;

uint8_t  PWM_fail_counter = 0;
uint8_t  PWM_fail_counter_prev = 0;
uint32_t PWM_TURNOFF_counter = 0;
uint32_t TURNOFF_TIME_counter = 0;
uint32_t PWM_TURNOFF_MAX  = 750;   
uint32_t TURNOFF_TIME_MAX = 750;
                                   
static uint8_t User_State = SM_ARMING;
bool cmd_status = FALSE;
static uint16_t UserCnt = 0;
uint32_t ARMING_counter = 0;

bool buffer_completed = FALSE;                        /*!<  PWM filter variable */
uint32_t PWM_tmp_buffer[FILTER_DEEP];                 /*!<  PWM filter variable */
uint32_t index_filter = 1;                            /*!<  PWM filter variable */
uint32_t PWM_filtered = 0;                            /*!<  PWM filter variable */
uint32_t PWM_sum_filt = 0;                            /*!<  PWM filter variable */

uint16_t speed_max_valueRPM = MOTOR_MAX_SPEED_RPM;    /*!< Maximum value for speed reference from Workbench */
uint16_t speed_min_valueRPM = 1000;                   /*!< Set the minimum value for speed reference */
                                             
uint32_t Ton_max = 133206;                            /*!<  Maximum ton value for PWM (by default is for 400Hz PWM) */
uint32_t Ton_min =  77638;                            /*!<  Minimum ton value for PWM (by default is for 400Hz PWM) */
uint32_t delta_Ton_max = 0;  
uint32_t new_speed = 0;
uint32_t ARMING_TIME = 500;   
__IO uint32_t CaptureNumber = 0;
uint32_t IC1ReadValue1 = 0, IC1ReadValue2 = 0;
uint32_t Ton_value = 0;                               /*!<  Stores the last ton value after the digital filter */
__IO uint32_t Capture = 0;                            /*!<  Stores the last ton value before the digital filter */
uint32_t Ton_value_previous = 0;

/*Create the CMCI local reference: CMCI oMCI*/
static CMCI oMCI;


/**
  * @brief  Boot function to initialize the ESC board.
  * @retval none.
  */
void ESCboot(void)
{
   /* Get reference of MCI*/ 
   oMCI = GetMCI(M1);
   
   /* Enable GPIOA-GPIOI clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC34, (FunctionalState)ENABLE); 
  
  /* Init ADC peripherals and related IRQ handler*/
  ADC_DeInit(ADC3);
  
  /* Common init */
  ADC_CommonStructInit(&ADC_CommonInitStructure);
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Clock = 65536;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_OneShot;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay= 0u;
  ADC_CommonInit(ADC3, &ADC_CommonInitStructure);
  
  ADC_VoltageRegulatorCmd(ADC3, (FunctionalState)ENABLE);
  
  /* Wait for Regulator Startup time, once for both */
  {
    uint16_t waittime = 0u;
    for(waittime=0u;waittime<65000u;waittime++)
    {
	waittime=waittime;
    }
  }    
  
  ADC_SelectCalibrationMode(ADC3,ADC_CalibrationMode_Single);    
  ADC_StartCalibration(ADC3);
  while (ADC_GetCalibrationStatus(ADC3)== SET )
  {
  }
  /* Configure the ADC3_for reg conversions */
  ADC_StructInit(&ADC_InitStructure);
  ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Disable;
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; 
  ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;
  ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;    
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Left;
  ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;
  ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;
  ADC_InitStructure.ADC_NbrOfRegChannel = 1u;
  
  ADC_Init(ADC3, &ADC_InitStructure);
    
   /* Enable ADC3 */
  ADC_Cmd(ADC3, (FunctionalState)ENABLE);
  
  /* Enable the PWM function for input signal */
  PWM_FC_Init();
  
  delta_Ton_max = Ton_max - Ton_min; 
  
}

/**
  * @brief  Configures the PWM input.
  * @param  None
  * @retval None
  */
void PWM_FC_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_ICInitTypeDef  TIM_ICInitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;  

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_1);

  /* Enable the TIM3 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd =(FunctionalState)ENABLE;
  NVIC_Init(&NVIC_InitStructure);  
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, (FunctionalState)ENABLE);  
  
  /* TIM2 configuration: PWM Input mode ------------------------
     The external signal is connected to TIM2 CH1 pin (PA.15), 
     The Rising edge is used as active edge,
     The TIM2 CCR2 is used to compute the frequency value 
     The TIM2 CCR1 is used to compute the duty cycle value
  ------------------------------------------------------------ */

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;

  TIM_PWMIConfig(TIM2, &TIM_ICInitStructure);

  /* TIM enable counter */
  TIM_Cmd(TIM2, (FunctionalState)ENABLE);

  /* Enable the CC2 Interrupt Request */
  TIM_ITConfig(TIM2, TIM_IT_CC1, (FunctionalState)ENABLE);

}

/**
  * @brief  This function handles TIM2 global interrupt request.
  * @param  None
  * @retval None
  */

void TIM2_IRQHandler(void)
{
  uint32_t tmpccer = 0;
  
  PWM_fail_counter++;
  if(PWM_fail_counter == 0) 
     PWM_fail_counter = 1;
       
  if(TIM_GetITStatus(TIM2, TIM_IT_CC1) == SET) 
  {
    /* Clear TIM1 Capture compare interrupt pending bit */
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
   
    if(CaptureNumber == 0)
    {
      /* Detect the rising edge and set the next for falling */
      tmpccer &= (uint32_t)~(TIM_CCER_CC1P | TIM_CCER_CC1NP);
      tmpccer |= (uint32_t)(TIM_ICPolarity_Falling | (uint32_t)TIM_CCER_CC1E); 
      TIM2->CCER =  tmpccer;    
      IC1ReadValue1 = TIM_GetCapture1(TIM2);      
      CaptureNumber = 1;
    }
    else
    { 
      /* Detect the falling edge and set the next for rising */
      tmpccer &= (uint32_t)~(TIM_CCER_CC1P | TIM_CCER_CC1NP);
      tmpccer |= (uint32_t)(TIM_ICPolarity_Rising | (uint32_t)TIM_CCER_CC1E); 
      TIM2->CCER =  tmpccer; 
      IC1ReadValue2 = TIM_GetCapture1(TIM2);       
      CaptureNumber = 0;
    }
    
    if(CaptureNumber == 0)
    {
      /* Capture computation */
      if (IC1ReadValue2 > IC1ReadValue1)
      {
        Capture = (IC1ReadValue2 - IC1ReadValue1); 
        
        Ton_value = TIMCapture_filter(Capture);
      }
  }
 }
}

void pwm_stop()
{
  MCI_StopMotor(oMCI);
  User_State = SM_ARMING;
  UserCnt = 0;  
  ARMING_counter = 0;
}

/*This is the main function to use in the main.c in order to start the current example */
void pwm_start()
{ 
 if(TB_UserTimebaseHasElapsed())
 { 
  /* User defined code */
  switch (User_State)
  {
   case SM_ARMING:
    {
      Ton_value_previous = 0;
      if((Ton_value > Ton_min) && (Ton_value < (Ton_max +5000))) 
      {
        ARMING_counter++;
        if(ARMING_counter > ARMING_TIME)    
         {
           User_State = SM_ARMED;   
           ARMING_counter  = 0;
           PWM_TURNOFF_counter = 0;                   
           PWM_fail_counter = 0;
           PWM_fail_counter_prev = 0;   
           buffer_completed = FALSE;
         }
      }
      else 
       {
         User_State = SM_ARMING;          
         ARMING_counter  = 0;
       }
      }
    break;  
   case SM_ARMED:
    {
          /* Next state */
          /* This command sets what will be the first speed ramp after the 
          MCI_StartMotor command. It requires as first parameter the oMCI[0], as 
          second parameter the target mechanical speed in thenth of Hz and as
          third parameter the speed ramp duration in milliseconds. */
          MCI_ExecSpeedRamp(oMCI, speed_min_valueRPM/6, 0);
          
          /* This is a user command used to start the motor. The speed ramp shall be
          pre programmed before the command.*/
          cmd_status = MCI_StartMotor(oMCI);
          
          /* It verifies if the command  "MCI_StartMotor" is successfully executed 
          otherwise it tries to restart the procedure */
          if(cmd_status==FALSE)    
           {
            User_State = SM_ARMING;                       // Command NOT executed
           }
          else User_State = SM_POSITIVE_RUN;              // Command executed
 
          UserCnt = 0;
    }
    break;  
   case SM_POSITIVE_RUN:
    {  
       if(PWM_fail_counter == PWM_fail_counter_prev)
       {
         PWM_TURNOFF_counter++;
 
         if(PWM_TURNOFF_counter > PWM_TURNOFF_MAX)
         {
          User_State = SM_STOP; 
          Ton_value_previous = 0;
         }
       }
       else 
       {
         PWM_TURNOFF_counter = 0;

       }

       if(Ton_value > 0 && Ton_value < Ton_min)
       {
         TURNOFF_TIME_counter ++;
         if(TURNOFF_TIME_counter > TURNOFF_TIME_MAX)
         {
          User_State = SM_STOP; 
          Ton_value_previous = 0;          
          TURNOFF_TIME_counter = 0;
         }         
       }
       else 
       {
         TURNOFF_TIME_counter = 0;
       }
                        
       if(Ton_value<=Ton_max && Ton_value > Ton_min)
       {
         new_speed = (Ton_value-Ton_min) * speed_max_valueRPM / delta_Ton_max;  
       }
                 
       if(new_speed < speed_min_valueRPM)
        new_speed = speed_min_valueRPM;
       if(new_speed > speed_max_valueRPM)
        new_speed = speed_max_valueRPM;        
  
    if(MCI_GetSTMState(oMCI)== RUN)       
     {
       MCI_ExecSpeedRamp(oMCI, new_speed/6, 0); 
     }
     
       PWM_fail_counter_prev = PWM_fail_counter;
    }
    break;
   case SM_STOP:
    {
       /* This is a user command to stop the motor */
       MCI_StopMotor(oMCI);
        
       /* After the time "STOP_DURATION" the motor will be restarted */
       if (UserCnt >= STOP_DURATION)
          {
            /* Next state */ 
            User_State = SM_ARMING;
            UserCnt = 0;   
            Ton_value  = 0;
            ARMING_counter = 0;
            Ton_value_previous = 0;
            buffer_completed = FALSE;
          }
          else
          {
            UserCnt++;
          }
    }
    break;  
  }
  TB_SetUserTimebaseTime(USER_TIMEBASE_OCCURENCE_TICKS);
 }
}

/**
  * @brief  This function handles PWM control from input signal
  * @param  None
  * @retval None
  */
void PWM_FC_control()
{
#ifdef PWM_ESC  
  if(MCI_GetSTMState(oMCI)== FAULT_OVER)
  {
   MCI_FaultAcknowledged(oMCI);
   User_State = SM_STOP;   
   ARMING_counter  = 0;
  }
 
  pwm_start();
#endif
}

uint32_t TIMCapture_filter(uint32_t capture_value)
{ 
  if(buffer_completed == FALSE)
  {
     PWM_tmp_buffer[index_filter] = capture_value;    
     PWM_sum_filt = 0;
     for(uint32_t i = 1; i <= index_filter;i++)
     {
       PWM_sum_filt = PWM_sum_filt + PWM_tmp_buffer[i];
     }
     PWM_filtered = PWM_sum_filt/index_filter;
     index_filter++;
     
      if(index_filter >= FILTER_DEEP) 
       {
         index_filter = 1;
         buffer_completed = TRUE;
       }
  }  
  else
  {
     index_filter++;
     if(index_filter >= FILTER_DEEP)
     {
      index_filter = 1;
     }
     
     PWM_sum_filt = 0;
     PWM_tmp_buffer[index_filter] = capture_value;
     uint32_t PWM_max = 0;
     for(uint32_t i = 1; i < FILTER_DEEP;i++)
     {
       uint32_t val = PWM_tmp_buffer[i];
       if (val > PWM_max)
       {
         PWM_max = val;
       }
       PWM_sum_filt = (uint32_t) (PWM_sum_filt +val);
     }
     PWM_sum_filt -= PWM_max;
     PWM_filtered = PWM_sum_filt/(FILTER_DEEP-2);
  }
  if(PWM_filtered==0) PWM_filtered = 1;
  
return(PWM_filtered);
}

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
