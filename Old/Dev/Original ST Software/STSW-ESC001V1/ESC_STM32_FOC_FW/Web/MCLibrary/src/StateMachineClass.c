/**
  ******************************************************************************
  * @file    StateMachineClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of StateMachine class
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
#include "StateMachineClass.h"
#include "StateMachinePrivate.h"
#include "MCLibraryConf.h"
#include "MC_type.h"

#ifdef MC_CLASS_DYNAMIC
  #include "stdlib.h" /* Used for dynamic allocation */
#else
  _CSTM_t STMpool[MAX_STM_NUM];
  unsigned char STM_Allocated = 0u;
#endif

#define CLASS_VARS   ((_CSTM)this)->Vars_str
#define CLASS_PARAMS  ((_CSTM)this)->pParams_str  
  
/**
  * @brief  Creates an object of the class StateMachine
  * @param  pStateMachineParams pointer to an StateMachine parameters structure
  * @retval CSTM new instance of StateMachine object
  */
CSTM STM_NewObject(void)
{
  _CSTM _oSTM;
  
  #ifdef MC_CLASS_DYNAMIC
    _oSTM = (_CSTM)calloc(1u,sizeof(_CSTM_t));
  #else
    if (STM_Allocated  < MAX_STM_NUM)
    {
      _oSTM = &STMpool[STM_Allocated++];
    }
    else
    {
      _oSTM = MC_NULL;
    }
  #endif
  
  return ((CSTM)_oSTM);
}


/**
  * @brief  Initializes all the object variables, usually it has to be called 
  *         once right after object creation.
  * @param this related object of class CSTM.
  * @retval none.
  */
void STM_Init(CSTM this)
{
  pVars_t pVars = &CLASS_VARS;
  
  pVars->bState = IDLE;
  pVars->hFaultNow = MC_NO_FAULTS;     
  pVars->hFaultOccurred = MC_NO_FAULTS;  
}     

/**
  * @brief It submits the request for moving the state machine into the state 
  *        specified by bState (FAULT_NOW and FAUL_OVER are not handled by this 
  *        method). Accordingly with the current state, the command is really
  *        executed (state machine set to bState) or discarded (no state 
  *        changes).
  *        If requested state can't be reached the return value is FALSE and the 
  *        MC_SW_ERROR is raised, but if requested state is IDLE_START, 
  *        IDLE_ALIGNMENT or ANY_STOP, that corresponds with the user actions: 
  *        Start Motor, Encoder Alignemnt and Stop Motor, the MC_SW_ERROR is
  *        not raised.
  * @param this related object of class CSTM.
  * @param bState New requested state
  * @retval bool It returns TRUE if the state has been really set equal to 
  *         bState, FALSE if the requested state can't be reached
  */
bool STM_NextState(CSTM this, State_t bState)
{
  bool bChangeState = FALSE;
  pVars_t pVars = &CLASS_VARS;
  State_t bCurrentState = pVars->bState;
  State_t bNewState = bCurrentState;
  
  switch(bCurrentState)
  {
  case ICLWAIT:
    if (bState == IDLE)
    {
      bNewState = bState;
      bChangeState = TRUE;
    }  
    break;
  case IDLE:
    if((bState == IDLE_START) || (bState == IDLE_ALIGNMENT) 
                              || (bState == ICLWAIT))
    {
      bNewState = bState;
      bChangeState = TRUE;
    }
    break;
    
  case IDLE_ALIGNMENT:
    if((bState == ANY_STOP) || (bState == ALIGN_CHARGE_BOOT_CAP) 
                            || (bState == ALIGN_OFFSET_CALIB))
    {
      bNewState = bState;
      bChangeState = TRUE;
    }
    break;
    
  case ALIGN_CHARGE_BOOT_CAP:
    if ((bState == ALIGN_OFFSET_CALIB) || (bState == ANY_STOP))
    {
      bNewState = bState;
      bChangeState = TRUE;
    }
    break;
    
    case ALIGN_OFFSET_CALIB:
      if ((bState == ALIGN_CLEAR) || (bState == ANY_STOP))
      {
        bNewState = bState;
        bChangeState = TRUE;
      }
      break;
      
  case ALIGN_CLEAR:
    if ((bState == ALIGNMENT) || (bState == ANY_STOP))
    {
      bNewState = bState;
      bChangeState = TRUE;
    }
    break;      
    
  case ALIGNMENT:
    if(bState == ANY_STOP)
    {
      bNewState = bState;
      bChangeState = TRUE;
    }
    break;  
    
  case IDLE_START:
    if((bState == ANY_STOP) || (bState == CHARGE_BOOT_CAP) ||
       (bState == START) ||
       (bState == OFFSET_CALIB) || (bState == IDLE_ALIGNMENT))
    {
      bNewState = bState;
      bChangeState = TRUE;
    }
    break;
    
  case CHARGE_BOOT_CAP:
    if ((bState == OFFSET_CALIB) || (bState == ANY_STOP))
    {
      bNewState = bState;
      bChangeState = TRUE;
    }
    break;
    
    case OFFSET_CALIB:
      if ((bState == CLEAR) || (bState == ANY_STOP))
      {
        bNewState = bState;
        bChangeState = TRUE;
      }
      break;
      
  case CLEAR:
    if ((bState == START) || (bState == ANY_STOP))
    {
      bNewState = bState;
      bChangeState = TRUE;
    }
    break;
    
  case START:
    if((bState == START_RUN) || (bState == ANY_STOP))
    {
      bNewState = bState;
      bChangeState = TRUE;
    }
    break;
    
  case START_RUN:
    if((bState == RUN) || (bState == ANY_STOP))
    {
      bNewState = bState;
      bChangeState = TRUE;
    }
    break;       
    
  case RUN:
    if(bState == ANY_STOP)
    {
      bNewState = bState;
      bChangeState = TRUE;
    }
    break;          
    
  case ANY_STOP:
    if(bState == STOP)
    {
      bNewState = bState;
      bChangeState = TRUE;
    }
    break;      
    
  case STOP:
    if(bState == STOP_IDLE)
    {
      bNewState = bState;
      bChangeState = TRUE;
    }
    break;       
    
  case STOP_IDLE:
    if((bState == IDLE)||(bState == ICLWAIT))
    {
      bNewState = bState;
      bChangeState = TRUE;
    }
    break;      
  default:
    break;
  }
  
  if (bChangeState)
  {
    pVars->bState = bNewState;
  }
  else
  {
    if (!((bState == IDLE_START) || (bState == IDLE_ALIGNMENT) 
                   || (bState == ANY_STOP)))
    {
      /* If new state is not a user command START/STOP raise a software error */
      STM_FaultProcessing(this, MC_SW_ERROR, 0u);
    }
  }
  
  return(bChangeState); 
}

/**
  * @brief It clocks both HW and SW faults processing and update the state 
  *        machine accordingly with hSetErrors, hResetErrors and present state. 
  *        Refer to State_t description for more information about fault states.
  * @param this object of class CSTM
  * @param hSetErrors Bit field reporting faults currently present
  * @param hResetErrors Bit field reporting faults to be cleared
  * @retval State_t New state machine state after fault processing
  */
State_t STM_FaultProcessing(CSTM this, uint16_t hSetErrors, uint16_t 
                                                                  hResetErrors)
{
  pVars_t pVars = &CLASS_VARS;  
  State_t LocalState =  pVars->bState;
  uint16_t hAux;
  
  /* Set corrent errors */
  hAux = (uint16_t)(pVars->hFaultNow);
  hAux |= hSetErrors;  
  pVars->hFaultNow = (FaultCondition_t)(hAux);
  hAux = (uint16_t)(pVars->hFaultOccurred);
  hAux |= hSetErrors; 
  pVars->hFaultOccurred = (FaultCondition_t)(hAux);
  /* Reset current errors */
  hAux = (uint16_t)(pVars->hFaultNow);
  hAux &= (uint16_t)(~hResetErrors);
  pVars->hFaultNow = (FaultCondition_t)(hAux);
 
  if(LocalState == FAULT_NOW)
  {
    if (pVars->hFaultNow == MC_NO_FAULTS)
    {
      pVars->bState = FAULT_OVER;
      LocalState = FAULT_OVER;
    }
  }  
  else
  {
    if (pVars->hFaultNow != MC_NO_FAULTS)
    {
      pVars->bState = FAULT_NOW;
      LocalState = FAULT_NOW;
    }
  }
  
  return (LocalState);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif
/**
  * @brief  Returns the current state machine state
  * @param  this object of class CSTM
  * @retval State_t Current state machine state
  */
State_t STM_GetState(CSTM this)
{
  return(CLASS_VARS.bState);
}


/**
  * @brief It reports to the state machine that the fault state has been 
  *        acknowledged by the user. If the state machine is in FAULT_OVER state
  *        then it is moved into STOP_IDLE and the bit field variable containing 
  *        information about the faults historically occured is cleared. 
  *        The method call is discarded if the state machine is not in FAULT_OVER
  * @param this object of class CSTM
  * @retval bool TRUE if the state machine has been moved to IDLE, FALSE if the 
  *        method call had no effects 
  */
bool STM_FaultAcknowledged(CSTM this)
{
 bool bToBeReturned = FALSE;
 pVars_t pVars = &CLASS_VARS;   

 if(pVars->bState == FAULT_OVER)
 {
  pVars->bState = STOP_IDLE;
  pVars->hFaultOccurred = MC_NO_FAULTS;  
  bToBeReturned = TRUE;
 }

 return(bToBeReturned);  
}


/**
  * @brief It returns two 16 bit fields containing information about both faults
  *        currently present and faults historically occurred since the state 
  *        machine has been moved into state
  * @param this object of class CSTM.
  * @retval uint32_t  Two 16 bit fields: in the most significant half are stored
  *         the information about currently present faults. In the least 
  *         significant half are stored the information about the faults 
  *         historically occurred since the state machine has been moved into 
  *         FAULT_NOW state
  */
uint32_t STM_GetFaultState(CSTM this)
{
 uint32_t LocalFaultState;
 pVars_t pVars = &CLASS_VARS;   
 
 LocalFaultState = (uint32_t)(pVars->hFaultOccurred);
 LocalFaultState |= (uint32_t)(pVars->hFaultNow)<<16;
   
 return LocalFaultState;
}

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
