/**
  ******************************************************************************
  * @file    STEVAL_ESC001V1.h
  * @author  IMS Systems Lab  - MC Team
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
void pwm_stop(void);
uint32_t TIMCapture_filter(uint32_t);


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
