/**
  ******************************************************************************
  * @file    SpeednTorqCtrlPrivate.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private definition of SpeednTorqCtrl class      
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
#ifndef __SPEEDNTORQCTRLPRIVATE_H
#define __SPEEDNTORQCTRLPRIVATE_H

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */

/** @addtogroup SpeednTorqCtrl
  * @{
  */

/** @defgroup SpeednTorqCtrl_class_private_types SpeednTorqCtrl class private types
* @{
*/

/** 
  * @brief  SpeednTorqCtrl class members definition
  */
typedef struct
{
	STC_Modality_t bMode;   /*!< Modality of STC. It can be one of these two
                                     settings: STC_TORQUE_MODE to enable the
                                     Torque mode or STC_SPEED_MODE to enable the
                                     Speed mode.*/
	int16_t hTargetFinal;	/*!< Backup of hTargetFinal to be applied in the
                                     last step.*/ 
	int32_t wSpeedRef01HzExt; /*!< Current mechanical rotor speed reference 
                                     expressed in tenths of HZ multiplied by 
                                     65536.*/
	int32_t wTorqueRef;     /*!< Current motor torque reference. This value
                                     represents actually the Iq current 
                                     expressed in digit multiplied by 65536.*/
	uint32_t wRampRemainingStep;/*!< Number of steps remaining to complete the
                                     ramp.*/
	CPI oPISpeed;		/*!< The regulator used to perform the speed
                                     control loop.*/
        CSPD oSPD;              /*!< The speed sensor used to perform the speed 
                                     regulation.*/
	int32_t wIncDecAmount;	/*!< Increment/decrement amount to be applied to
                                     the reference value at each
                                     CalcTorqueReference.*/
        uint16_t hMaxPositiveTorque; /*!< Maximum positive value of motor 
                                     torque. This value represents 
                                     actually the maximum Iq current 
                                     expressed in digit.*/
        int16_t hMinNegativeTorque;  /*!< Minimum negative value of motor 
                                     torque. This value represents 
                                     actually the maximum Iq current 
                                     expressed in digit.*/
}Vars_t,*pVars_t;

/** 
  * @brief  Redefinition of parameter structure
  */
typedef SpeednTorqCtrlParams_t Params_t, *pParams_t;

/** 
  * @brief  Private SpeednTorqCtrl class definition 
  */
typedef struct
{
	Vars_t Vars_str; 		/*!< Class members container */
	pParams_t pParams_str;	/*!< Class parameters container */
}_CSTC_t, *_CSTC;
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__SPEEDNTORQCTRLPRIVATE_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
