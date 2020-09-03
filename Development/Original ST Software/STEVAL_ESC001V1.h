/**
  ******************************************************************************
  * @file    STEVAL_ESC001V1.h
  * @author  SRA - MC Team
  * @version 1.1.2
  * @date    28-Jan-2019 15:29
  * @brief   ESC001V1 Eval board      
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

/* Define to prevent recursive inclusion -------------------------------------*/

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */

/** @addtogroup ESCApplication
  * @{
  */
  
/** @defgroup ESCApplication_exported_functions ESCApplication exported functions
  * @{
  */


/**
  * @brief  Boot function to initialize the ESC board.
  * @retval none.
  */
void ESCboot(void);
void PWM_FC_Init(void);
void PWM_FC_control(void);
void TIM2_IRQHandler(void);
void pwm_start(void);
void Init_PWM_Channel(void);
uint32_t TIMCapture_filter(uint32_t);


/**
  * @}
  */
  
/* Exported types ------------------------------------------------------------*/
/**
  * @brief  State_t enum type definition, it lists all the possible state machine states
  */
typedef enum
{
  SM_ARMING = 0,
  SM_ARMED = 1,
  SM_POSITIVE_RUN = 2,
  SM_STOP = 3
} User_State_t;

typedef enum
{
  ESC_NOERROR = 0,
  ESC_NOSIGNAL = 1,
  ESC_PWM_BELOW_MIN = 2
} ESC_State_t;

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
