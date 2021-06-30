/**
  ******************************************************************************
  * @file    GAPApplication.h
  * @author  IMS Systems Lab and Technical Marketing - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   Interface of GAPApplication      
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
#ifndef __GAPAPPLICATION_H
#define __GAPAPPLICATION_H

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */

/** @addtogroup GAPApplication
  * @{
  */
  
/** @defgroup GAPApplication_exported_functions GAPApplication exported functions
  * @{
  */


/**
  * @brief  Boot function to initialize the GAP controller.
  * @retval none.
  */
void GAPboot(void);

/**
  * @brief  Schedule function to be called with periodicity.
  * @retval none.
  */
void GAPSchedule(void);

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /* __GAPAPPLICATION_H */
/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
