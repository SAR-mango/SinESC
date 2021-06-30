/**
  ******************************************************************************
  * @file    DigitalOutputClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of DigitalOutput class implementation
  *          based on stdlib STM32F4xx
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
#include "DigitalOutputClass.h"
#include "DigitalOutputPrivate.h"
#include "MCLibraryConf.h"
#include "MC_type.h"

#define CLASS_VARS   ((_CDOUT)this)->Vars_str
#define CLASS_PARAMS  ((_CDOUT)this)->pParams_str  

#ifdef MC_CLASS_DYNAMIC
  #include "stdlib.h" /* Used for dynamic allocation */
#else
  _CDOUT_t DOUTpool[MAX_DOUT_NUM];
  unsigned char DOUT_Allocated = 0u;
#endif

/**
  * @brief  Creates an object of the class DigitalOutput
  * @param  pDigitalOutputParams pointer to an DigitalOutput parameters structure
  * @retval CDOUT new instance of DigitalOutput object
  */
CDOUT DOUT_NewObject(pDigitalOutputParams_t pDigitalOutputParams)
{
  _CDOUT _oDOUT;
  
  #ifdef MC_CLASS_DYNAMIC
    _oDOUT = (_CDOUT)calloc(1u,sizeof(_CDOUT_t));
  #else
    if (DOUT_Allocated  < MAX_DOUT_NUM)
    {
      _oDOUT = &DOUTpool[DOUT_Allocated++];
    }
    else
    {
      _oDOUT = MC_NULL;
    }
  #endif
  
  _oDOUT->pParams_str = (pParams_t)pDigitalOutputParams;
  
  return ((CDOUT)_oDOUT);
}

/**
* @brief  Initializes object variables, port and pin. It must be called only 
*         after PWMnCurrFdbk object initialization and DigitalOutput object 
*         creation. 
* @param this related object of class CDOUT.
* @retval none.
*/
void DOUT_Init(CDOUT this)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* GPIOs configurations --------------------------------------------------*/
  GPIO_StructInit(&GPIO_InitStructure);
  
  /* Configure GPIO port and pin, clock it's supposed to be enabled in 
  PWMnCurrentFdbk object initialization */  
  GPIO_InitStructure.GPIO_Pin = CLASS_PARAMS->hDOutputPin;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(CLASS_PARAMS->hDOutputPort, &GPIO_InitStructure);
  DOUT_SetOutputState(this, INACTIVE);
}

/**
* @brief Accordingly with selected polarity, it sets to active or inactive the
*        digital output
* @param this related object of class CDOUT.
* @param OutputState_t New requested state
* @retval none
*/
void DOUT_SetOutputState(CDOUT this, DOutputState_t State)
{
  
  if(State == ACTIVE)
  {
    if(CLASS_PARAMS->bDOutputPolarity == DOutputActiveHigh)
    {
      GPIO_SetBits(CLASS_PARAMS->hDOutputPort,CLASS_PARAMS->hDOutputPin);
    }
    else
    {
      GPIO_ResetBits(CLASS_PARAMS->hDOutputPort,CLASS_PARAMS->hDOutputPin);     
    }
  }
  else
    if(CLASS_PARAMS->bDOutputPolarity == DOutputActiveHigh)
    {
      GPIO_ResetBits(CLASS_PARAMS->hDOutputPort,CLASS_PARAMS->hDOutputPin);
    }
    else
    {
      GPIO_SetBits(CLASS_PARAMS->hDOutputPort,CLASS_PARAMS->hDOutputPin);     
    }   
  CLASS_VARS.OutputState = State;
}

/**
* @brief It returns the state of the digital output
* @param this object of class DOUT
* @retval OutputState_t Digital output state (ACTIVE or INACTIVE)
*/
DOutputState_t DOUT_GetOutputState(CDOUT this)
{
  return(CLASS_VARS.OutputState);
}


/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
