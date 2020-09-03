/**
  ******************************************************************************
  * @file    EncAlignCtrlPrivate.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private definition of EncAlignCtrl class      
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
#ifndef __ENCALIGNCTRLPRIVATE_H
#define __ENCALIGNCTRLPRIVATE_H

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */

/** @addtogroup EncAlignCtrl
  * @{
  */

/** @defgroup EncAlignCtrl_class_private_types EncAlignCtrl class private types
* @{
*/

/** 
  * @brief  EncAlignCtrl class members definition
  */
typedef struct
{
  CSTC oSTC;                /*!< Speed and torque controller object used by 
                                 EAC.*/
	CVSS_SPD oVSS;            /*!< Virtual speed sensor object used by EAC.*/
	CENC_SPD oENC;            /*!< Encoder object used by EAC.*/
	uint16_t hRemainingTicks; /*!< Number of clock events remaining to complete 
                                 the alignment.*/
        bool EncAligned;   /*!< This flag is TRUE if the encoder has been 
                                aligned at least one time, FALSE if hasn't been
                                never aligned.*/
        bool EncRestart;   /*!< This flag is used to force a restart of the 
                                motorafter the encoder alignment. It is TRUE
                                if a restart is programmed else FALSE*/
}Vars_t,*pVars_t;

/** 
  * @brief  Redefinition of parameter structure
  */
typedef EncAlignCtrlParams_t Params_t, *pParams_t;

/** 
  * @brief  Private EncAlignCtrl class definition 
  */
typedef struct
{
	Vars_t Vars_str; 		/*!< Class members container */
	pParams_t pParams_str;	/*!< Class parameters container */
}_CEAC_t, *_CEAC;
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__ENCALIGNCTRLPRIVATE_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
