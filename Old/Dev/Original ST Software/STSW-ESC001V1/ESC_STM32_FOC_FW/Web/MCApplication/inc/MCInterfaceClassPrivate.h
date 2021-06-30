/**
  ******************************************************************************
  * @file    MCInterfaceClassPrivate.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private interface of MCInterface class      
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
#ifndef __MCINTERFACECLASSPRIVATE_H
#define __MCINTERFACECLASSPRIVATE_H

/** @addtogroup STM32F10x_PMSM_MC_Interface
  * @{
  */

/** @addtogroup MCInterface
  * @{
  */

/** @defgroup MCInterface_class_private_methods MCInterface class private methods
  * @{
  */

/**
  * @brief  Creates an object of the class MCInterface
  * @param  pMCInterfaceParams pointer to an MCInterface parameters structure
  * @retval CMCI new instance of MCInterface object
  */
CMCI MCI_NewObject(pMCInterfaceParams_t pMCInterfaceParams);

/**
  * @brief  Initializes all the object variables, usually it has to be called 
  *         once right after object creation. It is also used to assign the 
  *         state machine object, the speed and torque controller, and the FOC
  *         drive object to be used by MC Interface.
  * @param  this related object of class CMCI.
  * @param  oSTM the state machine object used by the MCI.
  * @param  oSTC the speed and torque controller used by the MCI.
  * @param  pFOCVars pointer to FOC vars to be used by MCI.
  * @retval none.
  */
void MCI_Init(CMCI this, CSTM oSTM, CSTC oSTC, pFOCVars_t pFOCVars);

/**
  * @brief  This is usually a method managed by task. It must be called 
  *         periodically in order to check the status of the related oSTM object
  *         and eventually to execute the buffered command if the condition 
  *         occurs.
  * @param  this related object of class CMCI.
  * @retval none.
  */
void MCI_ExecBufferedCommands(CMCI this);
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__MCINTERFACECLASSPRIVATE_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
