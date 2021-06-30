/**
  ******************************************************************************
  * @file    MC_Math.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of MC Math module  
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
#ifndef __MC_MATH_H
#define __MC_MATH_H

/* Includes ------------------------------------------------------------------*/
#include "MC_type.h"

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */

/** @addtogroup MC_Math
  * @{
  */

/** @defgroup MC_Math_exported_macro MC Mathemical exported macro
  * @{
  */

/** 
  * @brief  Macro to compute logarithm of two
  */
#define LOG2(x) \
((x) == 65535 ? 16 : \
((x) == 2*2*2*2*2*2*2*2*2*2*2*2*2*2*2 ? 15 : \
((x) == 2*2*2*2*2*2*2*2*2*2*2*2*2*2 ? 14 : \
((x) == 2*2*2*2*2*2*2*2*2*2*2*2*2 ? 13 : \
((x) == 2*2*2*2*2*2*2*2*2*2*2*2 ? 12 : \
((x) == 2*2*2*2*2*2*2*2*2*2*2 ? 11 : \
((x) == 2*2*2*2*2*2*2*2*2*2 ? 10 : \
((x) == 2*2*2*2*2*2*2*2*2 ? 9 : \
((x) == 2*2*2*2*2*2*2*2 ? 8 : \
((x) == 2*2*2*2*2*2*2 ? 7 : \
((x) == 2*2*2*2*2*2 ? 6 : \
((x) == 2*2*2*2*2 ? 5 : \
((x) == 2*2*2*2 ? 4 : \
((x) == 2*2*2 ? 3 : \
((x) == 2*2 ? 2 : \
((x) == 2 ? 1 : \
((x) == 1 ? 0 : -1)))))))))))))))))

/**
  * @}
  */

/** @defgroup MC_Math_exported_types MC Mathemical exported types
  * @{
  */

/** 
  * @brief  Trigonometrical functions type definition 
  */
  typedef struct
  {
    int16_t hCos;
    int16_t hSin;
  } Trig_Components;

/**
  * @}
  */

/** @defgroup MC_Math_Exported_Functions MC Math exported functions
  * @{
  */

/**
  * @brief  This function transforms stator currents Ia and qIb (which are 
  *         directed along axes each displaced by 120 degrees) into currents 
  *         Ialpha and Ibeta in a stationary qd reference frame.
  *                               Ialpha = Ia
  *                       Ibeta = -(2*Ib+Ia)/sqrt(3)
  * @param  Curr_Input: stator current Ia and Ib in Curr_Components format
  * @retval Stator current Ialpha and Ibeta in Curr_Components format
  */
Curr_Components MCM_Clarke(Curr_Components Curr_Input);

/**
  * @brief  This function transforms stator currents Ialpha and Ibeta, which 
  *         belong to a stationary qd reference frame, to a rotor flux 
  *         synchronous reference frame (properly oriented), so as Iq and Id.
  *                   Id= Ialpha *sin(theta)+qIbeta *cos(Theta)
  *                   Iq=qIalpha *cos(Theta)-qIbeta *sin(Theta)            
  * @param  Curr_Input: stator current Ialpha and Ibeta in Curr_Components format
  * @param  Theta: rotating frame angular position in q1.15 format
  * @retval Stator current Iq and Id in Curr_Components format
  */
Curr_Components MCM_Park(Curr_Components Curr_Input, int16_t Theta);

/**
  * @brief  This function transforms stator voltage qVq and qVd, that belong to 
  *         a rotor flux synchronous rotating frame, to a stationary reference 
  *         frame, so as to obtain qValpha and qVbeta:
  *                  Valfa= Vq*Cos(theta)+ Vd*Sin(theta)
  *                  Vbeta=-Vq*Sin(theta)+ Vd*Cos(theta)     
  * @param  Curr_Input: stator voltage Vq and Vd in Volt_Components format
  * @param  Theta: rotating frame angular position in q1.15 format
  * @retval Stator voltage Valpha and Vbeta in Volt_Components format
  */
Volt_Components MCM_Rev_Park(Volt_Components Volt_Input, int16_t Theta);


/**
  * @brief  This function returns cosine and sine functions of the angle fed in 
  *         input
  * @param  hAngle: angle in q1.15 format
  * @retval Trig_Components Cos(angle) and Sin(angle) in Trig_Components format
  */
Trig_Components MCM_Trig_Functions(int16_t hAngle);

/**
  * @brief  It calculates the square root of a non-negative s32. It returns 0 
  *         for negative s32.
  * @param  Input int32_t number
  * @retval int32_t Square root of Input (0 if Input<0)
  */
int32_t MCM_Sqrt(int32_t wInput);

/**
  * @brief  This function codify a floting point number into the relative
  *         32bit integer.
  * @param  float Floting point number to be coded.
  * @retval uint32_t Coded 32bit integer.
  */
uint32_t MCM_floatToIntBit(float x);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#endif /*__MC_MATH_H*/
/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
