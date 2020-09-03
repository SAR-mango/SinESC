/**
  ******************************************************************************
  * @file    CrossCheck.h 
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file performs the coherency test between control stage 
  *			 parameters values and the compiler options.
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
#ifndef __CROSSCHECK_H
#define __CROSSCHECK_H

#include "Control stage parameters.h"
#include "Drive parameters.h"

/* Backward compatibility */
#if   defined(STM32F103x_LD)
#define STM32PERFORMANCELD
#elif defined(STM32F103x_MD)
#define STM32PERFORMANCEMD
#elif defined(STM32F103x_HD)
#define STM32PERFORMANCEHD
#elif defined(STM32F100x_LD)
#define STM32VALUELD
#elif defined(STM32F100x_MD)
#define STM32VALUEMD
#elif defined(STM32F2xx)
#define CS_STM32F2xx
#elif defined(STM32F4xx)
#define CS_STM32F4xx
#elif (defined(STM32F05xx) || defined(STM32F051x) || defined(STM32F050x) || defined(STM32F030x) || defined(STM32F072x))
#define CS_STM32F0xx
#if (defined(STM32F05xx))
#define STM32F051x
#endif
#elif defined(STM32F30x)
#define CS_STM32F30x /* Legacy */
#define STM32F303xC
#elif  (defined(STM32F303xB) || defined(STM32F303xC) || defined(STM32F302xB) || defined(STM32F302xC))
#define CS_STM32F30x
#endif

#if (defined(PROJECT_CHK))
  #define ENABLE   1
  #define DISABLE  0
  
  #if ((defined(STM32PERFORMANCELD) || defined(STM32VALUELD)) && (LCD_JOYSTICK_FUNCTIONALITY == ENABLE))
    #error "Invalid configuration: MCU definition in Control stage parameters.h, LD devices not compatible with LCD manager"
  #endif
    
  #undef ENABLE
  #undef DISABLE

  #if (defined(STM32PERFORMANCEMD) && defined(STM32F10X_MD))
  #elif (defined (STM32PERFORMANCELD) && (defined(STM32F10X_LD)||(STM32F10X_MD)))
  #elif (defined(STM32VALUEMD) && defined(STM32F10X_MD_VL))
  #elif (defined(STM32VALUELD) && (defined(STM32F10X_LD_VL)||(STM32F10X_MD_VL)))
  #elif (defined(STM32PERFORMANCEHD) && defined(STM32F10X_HD))
  #elif (defined(CS_STM32F2xx) && defined(STM32F2XX))
  #elif (defined(CS_STM32F4xx) && defined(STM32F40XX))
  #elif (defined(STM32F446xC_xE) && defined(STM32F446xx))
  #elif (defined(CS_STM32F0xx) && defined(STM32F0XX))
  #elif (defined(STSPIN32F0) && defined(STM32F0XX))
  #elif ((defined(STM32F303xB) || (defined(STM32F303xC))) && defined(STM32F303X))
  #elif ((defined(STM32F302xB) || (defined(STM32F302xC))) && defined(STM32F302X))
  #elif (defined(STM32F302x8) && defined(STM32F302X8))
  #else
//    #error "Invalid configuration: MCU definition in Control stage parameters.h not compatible with selected board. Please use the ST Motor Control Workbench to configure the firmware according your system."
  #endif
  
  #if (((defined STM32VALUEMD) || (defined STM32VALUELD)) && !defined(CPU_CLK_24_MHZ))
  #error "Invalid configuration: MCU frequency definition in Control stage parameters.h not compatible with selected MCU"
  #endif

  #if (defined(STM32F050x))
    #warning "Please verify the target MCU selected in the IDE project setting. Once done, comment this line to avoid to show the messagge again."
  #endif

  #if (PWM_FREQUENCY < 4500)
    #error "Invalid configuration: PWM frequency settled in Drive parameters.h is not supported by the firmware"
  #endif

#endif

#if (defined(MC_APPLICATION_CHK))
  #if (defined(STM32PERFORMANCEMD) && defined(STM32F10X_MD))
  #elif (defined (STM32PERFORMANCELD) && (defined(STM32F10X_LD)||(STM32F10X_MD)))
  #elif (defined(STM32VALUEMD) && defined(STM32F10X_MD_VL))
  #elif (defined(STM32VALUELD) && (defined(STM32F10X_LD_VL)||(STM32F10X_MD_VL)))
  #elif (defined(STM32PERFORMANCEHD) && defined(STM32F10X_HD))
  #elif ((defined(STM32PERFORMANCEMD) || defined (STM32PERFORMANCELD) || defined(STM32VALUEMD) || defined(STM32VALUELD) || defined(STM32PERFORMANCEHD)) && defined(STM32F10X_HD))
  #elif (defined(CS_STM32F2xx) && defined(STM32F2XX))
  #elif (defined(CS_STM32F4xx) && defined(STM32F40XX))
  #elif (defined(STM32F446xC_xE) && defined(STM32F446xx))
  #elif (defined(CS_STM32F0xx) && defined(STM32F0XX))
  #elif (defined(STSPIN32F0) && defined(STM32F0XX))
  #elif (defined(CS_STM32F30x) && defined(STM32F30X))
  #elif (defined(STM32F302x8) && defined(STM32F302X8))
  #else
//    #error "Invalid configuration: MCU definition in Control stage parameters.h not compatible with selected board. Please use the ST Motor Control Workbench to configure the firmware according your system."
  #endif
  
  #if (defined(DUALDRIVE) && (!defined(STM32F10X_HD) && !defined(STM32F40XX) && !defined(STM32F446xx) && !defined(STM32F2XX) && !defined(STM32F303xB) && !defined(STM32F303xC)))
  #error "Invalid configuration: DUALDRIVE definition in Drive parameters.h is not supported by selected MCU"
  #endif

  #define ENABLE   1
  #define DISABLE  0

  #if (defined(MOTOR_PROFILER))
    #if (!defined(STM32F4xx) && !defined(STM32F446xC_xE) && !defined(STM32F303xB) && !defined(STM32F303xC) && !defined(STM32F302xB) && !defined(STM32F302xC) && !defined(STM32F302X8))
      #error "Invalid configuration: MOTOR_PROFILER can be enabled only with STM32F4 or STM32F3 devices."
    #endif
    #if (defined(DUALDRIVE))
      #error "Invalid configuration: MOTOR_PROFILER can't be enabled for DUALDRIVE configuration."
    #endif
    #if (!defined(STATE_OBSERVER_PLL))
      #error "Invalid configuration: STATE_OBSERVER_PLL must be enabled for MOTOR_PROFILER configuration."
    #endif
    #if (FLUX_WEAKENING_ENABLING == ENABLE)
      #error "Invalid configuration: FLUX_WEAKENING must be disabled for MOTOR_PROFILER configuration."
    #endif
    #if (defined(OTF_STARTUP))
      #error "Invalid configuration: OTF_STARTUP must be disabled for MOTOR_PROFILER configuration."
    #endif
    #if (USE_INTERNAL_OPAMP == ENABLE)
      #error "Invalid configuration: EMBEDDED PGA must be disabled for MOTOR_PROFILER configuration."
    #endif
  #endif
  
  #if (defined(OTF_STARTUP) && (!defined(STATE_OBSERVER_PLL) && !defined(STATE_OBSERVER_CORDIC)))
      #error "Invalid configuration: OTF_STARTUP can be enabled only when STATE_OBSERVER_PLL or STATE_OBSERVER_CORDIC are enabled as main speed and position sensor in Drive parameters.h"
  #endif

  #if ((OPEN_LOOP_FOC == ENABLE)&&(!defined(STATE_OBSERVER_PLL))&&(!defined(STATE_OBSERVER_CORDIC)))
    #error "Invalid configuration: OPEN_LOOP configuration shall be enabled with Sensor-less enabled in Drive parameters.h"
  #endif

#if defined(DUALDRIVE)
  #include "Drive parameters motor 2.h"

  #if ((OPEN_LOOP_FOC2 == ENABLE)&&(!defined(STATE_OBSERVER_PLL2))&&(!defined(STATE_OBSERVER_CORDIC2)))
    #error "Invalid configuration: OPEN_LOOP configuration shall be enabled with Sensor-less enabled in Drive parameters motor 2.h"
  #endif
#endif

  #undef ENABLE
  #undef DISABLE

#endif
    
#if (defined(LCD_CHK))
  #if (defined(STM32PERFORMANCELD) || defined(STM32VALUELD) || defined(STM32F10X_LD) || defined(STM32F10X_LD_VL))
  #error "Invalid configuration: MCU definition in Control stage parameters.h, LD devices not compatible with LCD manager"
  #endif

  #if (defined(STM32F10X_MD) && !defined(STM32PERFORMANCEMD))
  #error "Invalid configuration: MCU definition in Control stage parameters.h not compatible with selected board"
  #endif

  #if (defined(STM32F10X_MD_VL) && !defined(STM32VALUEMD))
  #error "Invalid configuration: MCU definition in Control stage parameters.h not compatible with selected board"
  #endif

  #if (defined(STM32F10X_HD) && !defined(STM32PERFORMANCEHD))
  #error "Invalid configuration: MCU definition in Control stage parameters.h not compatible with selected board"
  #endif

  #if (defined(DUALDRIVE) && (!defined(STM32F10X_HD) && !defined(STM32F40XX) && !defined(STM32F2XX) && !defined(STM32F303xB) && !defined(STM32F303xC)))
  #error "Invalid configuration: DUALDRIVE definition in Drive parameters.h is not supported by selected MCU"
  #endif

  #if (defined(STM32VALUE) && !defined(CPU_CLK_24_MHZ))
  #error "Invalid configuration: MCU frequency definition in Control stage parameters.h not compatible with selected MCU"
  #endif

  #if (defined(STM32VALUE) && defined(PFC_ENABLED))
  #error "Invalid configuration: PFC can't be enabled using selected MCU"
  #endif

  #if (defined(STM32F0XX) && !defined(LCD_REDUCED_FUNCTIONS))
  #error "Invalid configuration: LCD_REDUCED_FUNCTIONS must be defined with selected MCU"
  #endif

  #if (defined(LCD_REDUCED_FUNCTIONS) && defined(DUALDRIVE))
  #error "Invalid configuration: LCD_REDUCED_FUNCTIONS not compatible with DUALDRIVE"
  #endif

  #if (defined(LCD_REDUCED_FUNCTIONS) && defined(PFC_ENABLED))
  #error "Invalid configuration: LCD_REDUCED_FUNCTIONS not compatible with PFC_ENABLED"
  #endif
#endif

#endif /* __TIMEBASE_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
