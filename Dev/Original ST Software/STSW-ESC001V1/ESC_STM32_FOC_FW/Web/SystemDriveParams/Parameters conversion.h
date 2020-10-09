/**
  ******************************************************************************
  * @file    Parameters conversion.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file includes the proper Parameter conversion on the base
  *          of stdlib for the first drive
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
#ifndef __PARAMETERS_CONVERSION_H
#define __PARAMETERS_CONVERSION_H

#if (defined(STM32F10X_MD) || defined (STM32F10X_LD) || defined(STM32F10X_MD_VL) || defined(STM32F10X_LD_VL) || defined(STM32F10X_HD))
  #include "Parameters conversion_F10x.h"
#elif defined(STM32F2XX)
  #include "Parameters conversion_F2xx.h"
#elif defined(STM32F30X)
  #include "Parameters conversion_F30x.h"
#elif (defined(STM32F40XX) || defined(STM32F446xx))
  #include "Parameters conversion_F4xx.h"
#elif defined(STM32F0XX)
  #include "Parameters conversion_F0xx.h"
#endif

/* Common parameters conversions */
#if defined(OTF_STARTUP)
#define OTF_STARTUP_EN TRUE
#else
#define OTF_STARTUP_EN FALSE
#endif

#if defined(MOTOR_PROFILER)
 #undef MAX_APPLICATION_SPEED
 #define MAX_APPLICATION_SPEED 50000
 #undef F1
 #define F1 0
 #undef CORD_F1
 #define CORD_F1 0
#else
  #if !defined(BUS_VOLTAGE_CONVERSION_FACTOR)
   #define BUS_VOLTAGE_CONVERSION_FACTOR 0.0f
  #endif
  #if !defined(CALIBRATION_FACTOR)
   #define CALIBRATION_FACTOR            0.0f
  #endif
  #if !defined(CURRENT_REGULATOR_BANDWIDTH)
   #define CURRENT_REGULATOR_BANDWIDTH   0.0f
  #endif
  #if !defined(SPEED_REGULATOR_BANDWIDTH)
   #define SPEED_REGULATOR_BANDWIDTH     0.0f
  #endif
  #if !defined(MP_KP)
    #define MP_KP 0.0f
  #endif
  #if !defined(MP_KI)
    #define MP_KI 0.0f
  #endif
#endif

#if !defined(DC_CURRENT_RS_MEAS)
  #define DC_CURRENT_RS_MEAS 0.0f
#endif
#if !defined(LDLQ_RATIO)
  #define LDLQ_RATIO 0.0f
#endif

#if (START_STOP_BTN == ENABLE)
  #define ENABLE_START_STOP_BUTTON
#endif

#define DIN_ACTIVE_LOW Bit_RESET
#define DIN_ACTIVE_HIGH Bit_SET

#define USE_EVAL (defined(USE_STM32446E_EVAL) || defined(USE_STM324xG_EVAL) || defined(USE_STM32F4XX_DUAL))

#if !defined(UID)
#define UID 0
#endif

#if !defined(CTRBDID)
#define CTRBDID 0
#endif

#if !defined(PWBDID)
#define PWBDID 0
#endif

#endif /*__PARAMETERS_CONVERSION_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
