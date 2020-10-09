/**
  ******************************************************************************
  * @file    PWMnCurrFdbkClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of PI Regulator class      
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
#include "PWMnCurrFdbkClass.h"
#include "PWMnCurrFdbkPrivate.h"
#include "MCLibraryConf.h"
#include "MC_type.h"

#ifdef MC_CLASS_DYNAMIC
#include "stdlib.h" /* Used for dynamic allocation */
#else
_CPWMC_t PWMCpool[MAX_PWMC_NUM];
unsigned char PWMC_Allocated = 0u;
#endif

#define CLASS_VARS   &((_CPWMC)this)->Vars_str
#define CLASS_PARAMS  ((_CPWMC)this)->pParams_str
#define CLASS_METHODS ((_CPWMC)this)->Methods_str

#define SQRT3FACTOR (uint16_t) 0xDDB4 /* = (16384 * 1.732051 * 2)*/

/**
* @brief  Creates an object of the class PWMnCurrFdbk
* @param  pPWMnCurrFdbkParams pointer to an PWMnCurrFdbk parameters structure
* @retval CPWMC new instance of PWMnCurrFdbk object
*/
CPWMC PWMC_NewObject(pPWMnCurrFdbkParams_t pPWMnCurrFdbkParams)
{
  _CPWMC _oPWMC;
  
#ifdef MC_CLASS_DYNAMIC
  _oPWMC = (_CPWMC)calloc(1u,sizeof(_CPWMC_t));
#else
  if (PWMC_Allocated  < MAX_PWMC_NUM)
  {
    _oPWMC = &PWMCpool[PWMC_Allocated++];
  }
  else
  {
    _oPWMC = MC_NULL;
  }
#endif
  
  _oPWMC->pParams_str = (pParams_t)pPWMnCurrFdbkParams;
  
  _oPWMC->Methods_str.pPWMC_OCPSetReferenceVoltage = MC_NULL;
  
  return ((CPWMC)_oPWMC);
}

/**
  * @brief  Initiliaze all the object variables and MCU peripherals, usually 
  *         it has to be called once right after object creation.
  *         Note: All the GPIOx port peripherals clocks are here enabled.
  *         Note: If the derived class is IHD2, R1HD2 or R3HD2 it is required
  *               to call the specific xxx_StartTimers method after the
  *               PWMC_Init call.
  * @param  this PWM 'n Current feedback object
  * @retval none
  */
void PWMC_Init(CPWMC this)
{
  pVars_t pVars = CLASS_VARS;
  pParams_t pParams = CLASS_PARAMS;
  pVars->hT_Sqrt3= (pParams->hPWMperiod*SQRT3FACTOR)/16384u; 
  pVars->bTurnOnLowSidesAction = FALSE;
  (CLASS_METHODS.pPWMC_Init)(this);
  
  pVars->DTTest = 0u;
  pVars->DTCompCnt = pParams->hDTCompCnt;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif
/**
* @brief  It is used to get the motor phase current in Curr_Components format 
          as read by AD converter.
* @param  this: PWM 'n Current feedback object
* @param  pStator_Currents Pointer to the struct that will receive motor current
*         of phase A and B in Curr_Components format.
* @retval none.
*/
void PWMC_GetPhaseCurrents(CPWMC this,Curr_Components* pStator_Currents)
{
  int16_t hIa, hIb;
  pVars_t pVars = CLASS_VARS;
  (CLASS_METHODS.pPWMC_GetPhaseCurrents)(this, pStator_Currents);
  hIa = pStator_Currents->qI_Component1;
  hIb = pStator_Currents->qI_Component2;
  pVars->hIa = hIa;
  pVars->hIb = hIb;
  pVars->hIc = -hIa - hIb;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif  
/**
  * @brief  It converts input voltage components Valfa, beta into duty cycles 
  *         and feed it to the inverter
  * @param  this: PWM 'n Current feedback object
  * @param  Valfa_beta: Voltage Components in alfa beta reference frame
  * @retval It returns the code error 'MC_FOC_DURATION' if any, 'MC_NO_ERROR' 
  *         otherwise. These error codes are defined in MC_type.h
  */
uint16_t PWMC_SetPhaseVoltage(CPWMC this, Volt_Components Valfa_beta)
{
  pVars_t pVars = CLASS_VARS;
  int32_t wX, wY, wZ, wUAlpha, wUBeta, wTimePhA, wTimePhB, wTimePhC;
  uint16_t (*pSetADCSamplingPoint)(CPWMC Localthis);
    
  wUAlpha = Valfa_beta.qV_Component1 * (int32_t)pVars->hT_Sqrt3;
  wUBeta = -(Valfa_beta.qV_Component2 * 
             (int32_t)(CLASS_PARAMS->hPWMperiod))*2;
 
  wX = wUBeta;
  wY = (wUBeta + wUAlpha)/2;
  wZ = (wUBeta - wUAlpha)/2;
  
  /* Sector calculation from wX, wY, wZ */
  if (wY<0)
  {
    if (wZ<0)
    {
      pVars->hSector = SECTOR_5;
      wTimePhA = (int32_t)(CLASS_PARAMS->hPWMperiod)/4 + ((wY - wZ)/(int32_t)262144);  
      wTimePhB = wTimePhA + wZ/131072;
      wTimePhC = wTimePhA - wY/131072;
      pSetADCSamplingPoint = CLASS_METHODS.pPWMC_SetADCSampPointSect5;    
    }
    else /* wZ >= 0 */
      if (wX<=0)
      {
        pVars->hSector = SECTOR_4;
        wTimePhA = (int32_t)(CLASS_PARAMS->hPWMperiod)/4 + ((wX - wZ)/(int32_t)262144);       
        wTimePhB = wTimePhA + wZ/131072;
        wTimePhC = wTimePhB - wX/131072;
        pSetADCSamplingPoint = CLASS_METHODS.pPWMC_SetADCSampPointSect4;      
        
      }
      else /* wX > 0 */
      {
        pVars->hSector = SECTOR_3;
        wTimePhA = (int32_t)(CLASS_PARAMS->hPWMperiod)/4 + ((wY - wX)/(int32_t)262144);  
        wTimePhC = wTimePhA - wY/131072;
        wTimePhB = wTimePhC + wX/131072;
        pSetADCSamplingPoint = CLASS_METHODS.pPWMC_SetADCSampPointSect3;            
        
      }
  }
  else /* wY > 0 */
  {
    if (wZ>=0)
    {
      pVars->hSector = SECTOR_2;
      wTimePhA = (int32_t)(CLASS_PARAMS->hPWMperiod)/4 + ((wY - wZ)/(int32_t)262144);            
      wTimePhB = wTimePhA + wZ/131072;
      wTimePhC = wTimePhA - wY/131072;             
      pSetADCSamplingPoint = CLASS_METHODS.pPWMC_SetADCSampPointSect2;    
    }
    else /* wZ < 0 */
      if (wX<=0)
      {  
        pVars->hSector = SECTOR_6;
        wTimePhA = (int32_t)(CLASS_PARAMS->hPWMperiod)/4 + ((wY - wX)/(int32_t)262144);  
        wTimePhC = wTimePhA - wY/131072;
        wTimePhB = wTimePhC + wX/131072;
        pSetADCSamplingPoint = CLASS_METHODS.pPWMC_SetADCSampPointSect6;            
      }
      else /* wX > 0 */
      {
        pVars->hSector = SECTOR_1;
        wTimePhA = (int32_t)(CLASS_PARAMS->hPWMperiod)/4 + ((wX - wZ)/(int32_t)262144);       
        wTimePhB = wTimePhA + wZ/131072;
        wTimePhC = wTimePhB - wX/131072;
        pSetADCSamplingPoint = CLASS_METHODS.pPWMC_SetADCSampPointSect1;
        
      }
  }
  
  pVars->hCntPhA = (uint16_t)wTimePhA;
  pVars->hCntPhB = (uint16_t)wTimePhB;
  pVars->hCntPhC = (uint16_t)wTimePhC;
  
  if (pVars->DTTest == 1u)
  {
  /* Dead time compensation */
  if (pVars->hIa > 0)
  {
    pVars->hCntPhA += pVars->DTCompCnt;
  }
  else
  {
    pVars->hCntPhA -= pVars->DTCompCnt;
  }
  
  if (pVars->hIb > 0)
  {
    pVars->hCntPhB += pVars->DTCompCnt;
  }
  else
  {
    pVars->hCntPhB -= pVars->DTCompCnt;
  }
  
  if (pVars->hIc > 0)
  {
    pVars->hCntPhC += pVars->DTCompCnt;
  }
  else
  {
    pVars->hCntPhC -= pVars->DTCompCnt;
  }
  }
  
  return(pSetADCSamplingPoint(this));
}

/**
* @brief  It switch off the PWM generation, setting to inactive the outputs
* @param  this: PWM 'n Current feedback object
* @retval none
*/
void PWMC_SwitchOffPWM(CPWMC this)
{
  pVars_t pVars = CLASS_VARS;
  pVars->bTurnOnLowSidesAction = FALSE;
  (CLASS_METHODS.pPWMC_SwitchOffPWM)(this);
}


/**
* @brief  It switch on the PWM generation
* @param  this: PWM 'n Current feedback object
* @retval None
*/
void PWMC_SwitchOnPWM(CPWMC this)
{
  pVars_t pVars = CLASS_VARS;
  pVars->bTurnOnLowSidesAction = FALSE;
  (CLASS_METHODS.pPWMC_SwitchOnPWM)(this);
}

/**
* @brief  It calibrates ADC current conversions by reading the offset voltage
*         present on ADC pins when no motor current is flowing. It's suggested
*         to call this function before each motor start-up
* @param  this: PWM 'n Current feedback object
* @param  action: it can be CRC_START to initialize the offset calibration or
*         CRC_EXEC to execute the offset calibration.
* @retval bool It returns TRUE if the current calibration has been completed
*         otherwise if is ongoing it returns FALSE.
*/
bool PWMC_CurrentReadingCalibr(CPWMC this, CRCAction_t action)
{
  pVars_t pVars = CLASS_VARS;
  bool retVal = FALSE;
  if (action == CRC_START)
  {
    PWMC_SwitchOffPWM(this);
    pVars->hOffCalibrWaitTimeCounter = CLASS_PARAMS->hOffCalibrWaitTicks;
    if (CLASS_PARAMS->hOffCalibrWaitTicks == 0u)
    {
      (CLASS_METHODS.pPWMC_CurrentReadingCalibr)(this);
      retVal = TRUE;
    }
  }
  else if (action == CRC_EXEC)
  {
    if (pVars->hOffCalibrWaitTimeCounter > 0u)
    {
      pVars->hOffCalibrWaitTimeCounter--;
      if (pVars->hOffCalibrWaitTimeCounter == 0u)
      {
        (CLASS_METHODS.pPWMC_CurrentReadingCalibr)(this);
        retVal = TRUE;
      }
    }
    else
    {
      retVal = TRUE;
    }
  }
  else
  {
  }
  return retVal;
}

/**
* @brief  It switch on low sides. This function is intended to be used for
*         charging boot capacitors of driving section. It has to be called each 
*         motor start-up when using high voltage drivers
* @param  this: PWM 'n Current feedback object
* @retval None
*/
void PWMC_TurnOnLowSides(CPWMC this)
{
  pVars_t pVars = CLASS_VARS;
  pVars->bTurnOnLowSidesAction = TRUE;
  (CLASS_METHODS.pPWMC_TurnOnLowSides)(this);
}

/**
* @brief  Execute a regular conversion. User must guarantee by 
*         design (i.e. properly selecting tasks priorities) that this function 
*         can only be interrupted by TIMx_UP_ISR and ADC1_2_ISR. 
*         The function is not re-entrant (can't executed twice at the same time)
*         It returns 0xFFFF in case of conversion error.
* @param  this related object of class CPWMC, ADC channel to be converted
* @param  bChannel ADC channel used for the regular conversion
* @retval It returns converted value or oxFFFF for conversion error
*/
uint16_t PWMC_ExecRegularConv(CPWMC this, uint8_t bChannel)
{ 
  uint16_t hConvValue;
  hConvValue=(CLASS_METHODS.pPWMC_ExecRegularConv)(this, bChannel);
  return(hConvValue);
}

/**
* @brief  Execute a regular conversion. User must guarantee by 
*         design (i.e. properly selecting tasks priorities) that this function 
*         can only be interrupted by TIMx_UP_ISR and ADC1_2_ISR. 
*         The function is not re-entrant (can't executed twice at the same time)
*         It returns 0xFFFF in case of conversion error.
* @param  this related object of class CPWMC, ADC channel to be converted
* @param  bChannel ADC channel used for the regular conversion
* @retval It returns converted value or oxFFFF for conversion error
*/
uint16_t PWMC_ExecRegularConv_ESC(CPWMC this, uint8_t bChannel)
{ 
  uint16_t hConvValue;
  hConvValue=(CLASS_METHODS.pPWMC_ExecRegularConv_ESC)(this, bChannel);
  return(hConvValue);
}

/**
* @brief  It sets the specified sampling time for the specified ADC channel
*         on ADC1. It must be called once for each channel utilized by user
* @param  this related object of class CPWMC
* @param  ADConv_struct struct containing ADC channel and sampling time
* @retval none
*/
void PWMC_ADC_SetSamplingTime(CPWMC this, ADConv_t ADConv_struct)
{
  (CLASS_METHODS.pPWMC_SetSamplingTime)(this, ADConv_struct);
}

/**
* @brief  It is used to check if an overcurrent occurred since last call.
* @param  this related object of class CPWMC
* @retval uint16_t It returns MC_BREAK_IN whether an overcurrent has been 
*                  detected since last method call, MC_NO_FAULTS otherwise.
*/
uint16_t PWMC_CheckOverCurrent(CPWMC this)
{
  return (CLASS_METHODS.pPWMC_IsOverCurrentOccurred)(this);
}

/**
* @brief  It is used to set the overcurrent threshold through the DAC reference 
*         voltage.
* @param  this related object of class CPWMC
* @param  hDACVref Value of DAC reference expressed as 16bit unsigned integer.
*         Ex. 0 = 0V 65536 = VDD_DAC.
* @retval none
*/
void PWMC_OCPSetReferenceVoltage(CPWMC this,uint16_t hDACVref)
{
  if (CLASS_METHODS.pPWMC_OCPSetReferenceVoltage)
  {
    (CLASS_METHODS.pPWMC_OCPSetReferenceVoltage)(this, hDACVref);
  }
}

/**
* @brief  It is used to retrieve the satus of TurnOnLowSides action.
* @param  this related object of class CPWMC
* @retval bool It returns the state of TurnOnLowSides action: 
*         TRUE if TurnOnLowSides action is active, FALSE otherwise.
*/
bool PWMC_GetTurnOnLowSidesAction(CPWMC this)
{
  pVars_t pVars = CLASS_VARS;
  return pVars->bTurnOnLowSidesAction;
}

/**
* @brief  It is used to set the RL Detection mode.
* @param  this related object of class CPWMC
* @retval none
*/
void PWMC_RLDetectionModeEnable(CPWMC this)
{
  if (CLASS_METHODS.pRLDetectionModeEnable)
  {
    (CLASS_METHODS.pRLDetectionModeEnable)(this);
  }
}

/**
* @brief  It is used to disable the RL Detection mode and set the standard PWM.
* @param  this related object of class CPWMC
* @retval none
*/
void PWMC_RLDetectionModeDisable(CPWMC this)
{
  if (CLASS_METHODS.pRLDetectionModeDisable)
  {
    (CLASS_METHODS.pRLDetectionModeDisable)(this);
  }
}

/**
* @brief  It is used to set the PWM dutycycle in the RL Detection mode.
* @param  this related object of class CPWMC
* @param  hDuty to be applied in u16
* @retval It returns the code error 'MC_FOC_DURATION' if any, 'MC_NO_ERROR' 
*         otherwise. These error codes are defined in MC_type.h
*/
uint16_t PWMC_RLDetectionModeSetDuty(CPWMC this, uint16_t hDuty)
{
  uint16_t hRetVal = MC_FOC_DURATION;
  if (CLASS_METHODS.pRLDetectionModeSetDuty)
  {
    hRetVal = (CLASS_METHODS.pRLDetectionModeSetDuty)(this, hDuty);
  }
  return hRetVal;
}

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
