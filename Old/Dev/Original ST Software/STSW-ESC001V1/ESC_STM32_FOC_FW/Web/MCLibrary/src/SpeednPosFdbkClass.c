/**
  ******************************************************************************
  * @file    SpeednPosFdbkClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains source code of SpeednPosFdbk class      
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
#include "SpeednPosFdbkClass.h"
#include "SpeednPosFdbkPrivate.h"
#include "MCLibraryConf.h"
#include "MC_type.h"

#ifdef MC_CLASS_DYNAMIC
  #include "stdlib.h" /* Used for dynamic allocation */
#else
  _CSPD_t SPDpool[MAX_SPD_NUM];
  unsigned char SPD_Allocated = 0u;
#endif

/**
  * @brief  Creates an object of the class SpeednPosFdbk
  * @param  pSpeednPosFdbkParams pointer to an SpeednPosFdbk parameters structure
  * @retval CSPD new instance of SpeednPosFdbk object
  */
CSPD SPD_NewObject(pSpeednPosFdbkParams_t pSpeednPosFdbkParams)
{
  _CSPD _oSPD;
  
  #ifdef MC_CLASS_DYNAMIC
    _oSPD = (_CSPD)calloc(1u,sizeof(_CSPD_t));
  #else
    if (SPD_Allocated  < MAX_SPD_NUM)
    {
      _oSPD = &SPDpool[SPD_Allocated++];
    }
    else
    {
      _oSPD = MC_NULL;
    }
  #endif
  
  _oSPD->pParams_str = (pParams_t)pSpeednPosFdbkParams;
  
  return ((CSPD)_oSPD);
}

/**
  * @brief  Initiliazes all the object variables and MCU peripherals, usually
  *         it has to be called once right after object creation
  * @param  this related object of class CSPD
  * @retval none
  */
void SPD_Init(CSPD this)
{
  ((_CSPD)this)->Vars_str.bElToMecRatio = ((_CSPD)this)->pParams_str->bElToMecRatio;
  ((_CSPD)this)->Methods_str.pSPD_Init(this);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif
/**
  * @brief  It returns the last computed rotor electrical angle, expressed in
  *         s16degrees. 1 s16degree = 360°/65536
  * @param  this related object of class CSPD
  * @retval int16_t rotor electrical angle (s16degrees)
  */
int16_t SPD_GetElAngle(CSPD this)
{
  return(((_CSPD)this)->Vars_str.hElAngle);
}

/**
  * @brief  It returns the last computed rotor mechanical angle, expressed in
  *         s16degrees. Mechanical angle frame is based on parameter bElToMecRatio
  *         and, if occasionally provided - through function SPD_SetMecAngle -
  *         of a measured mechanical angle, on information computed thereof.
  *         Note: both Hall sensor and Sensor-less do not implement either 
  *         mechanical angle computation or acceleration computation. 
  * @param  this related object of class CSPD
  * @retval int16_t rotor mechanical angle (s16degrees)
  */
int16_t SPD_GetMecAngle(CSPD this)
{
  return(((_CSPD)this)->Vars_str.hMecAngle);
}

/**
  * @brief  It returns the last computed average mechanical speed, expressed in
  *         01Hz (tenth of Hertz).
  * @param  this related object of class CSPD
  * @retval int16_t rotor average mechanical speed (01Hz)
  */
int16_t SPD_GetAvrgMecSpeed01Hz(CSPD this)
{
  return(((_CSPD)this)->Vars_str.hAvrMecSpeed01Hz);
}

/**
  * @brief  It returns the last computed electrical speed, expressed in Dpp.
  *         1 Dpp = 1 s16Degree/control Period. The control period is the period
  *         on which the rotor electrical angle is computed (through function
  *         SPD_CalcElectricalAngle).
  * @param  this related object of class CSPD
  * @retval int16_t rotor electrical speed (Dpp)
  */
int16_t SPD_GetElSpeedDpp(CSPD this)
{
  return(((_CSPD)this)->Vars_str.hElSpeedDpp);
}

/**
  * @brief  It could be used to set istantaneous information on rotor mechanical
  *         angle. As a consequence, the offset relationship between electrical
  *         angle and mechanical angle is computed.
  *         Note: both Hall sensor and Sensor-less do not implement either 
  *         mechanical angle computation or acceleration computation.
  * @param  this related object of class CSPD
  * @param  hMecAngle istantaneous measure of rotor mechanical angle (s16degrees)
  * @retval none
  */
void SPD_SetMecAngle(CSPD this, int16_t hMecAngle)
{
  ((_CSPD)this)->Methods_str.pSPD_SetMecAngle(this,hMecAngle);  
}

/**
  * @brief  It restore all the functional variables to initial values.
  * @param  this related object of class CSPD
  * @retval none
  */
void SPD_Clear(CSPD this)
{  
  /* Clear previous speed error conditions */
  ((_CSPD)this)->Vars_str.bSpeedErrorNumber = 0u;
  
  ((_CSPD)this)->Methods_str.pSPD_Clear(this);
}

/**
  * @brief  It returns the result of the last reliability check performed.
  *         Reliability is measured with reference to parameters
  *         hMaxReliableElSpeed01Hz, hMinReliableElSpeed01Hz,
  *         bMaximumSpeedErrorsNumber and/or specific parameters of the derived
  *         TRUE = sensor information is reliable
  *         FALSE = sensor information is not reliable
  * @param  this related object of class CSPD
  * @retval bool sensor reliability state
  */
bool SPD_Check(CSPD this)
{
  bool SpeedSensorReliability = TRUE;
  if (((_CSPD)this)->Vars_str.bSpeedErrorNumber ==
      ((_CSPD)this)->pParams_str->bMaximumSpeedErrorsNumber)
  {
    SpeedSensorReliability = FALSE;
  }
  return(SpeedSensorReliability);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif
/**
  * @brief  This method must be called with the same periodicity on which FOC
  *         is executed. It computes and returns the rotor electrical angle,
  *         expressed in s16Degrees. 1 s16Degree = 360°/65536
  * @param  this related object of class CSPD
  * @param  pInputVars_str pointer to input structure. For derived 
			class STO input structure type is Observer_Inputs_t, for HALL and 
			ENCODER this parameter will not be used (thus it can be equal to 
			MC_NULL).
  * @retval int16_t rotor electrical angle (s16Degrees)
  */
int16_t SPD_CalcAngle(CSPD this, void *pInputVars_str)
{
  return(((_CSPD)this)->Methods_str.pSPD_CalcAngle(this, pInputVars_str));
}

/**
  * @brief  This method must be called - at least - with the same periodicity
  *         on which speed control is executed. It computes and returns - through
  *         parameter pMecSpeed01Hz - the rotor average mechanical speed,
  *         expressed in 01Hz. It computes and returns the reliability state of
  *         the sensor; reliability is measured with reference to parameters
  *         hMaxReliableElSpeed01Hz, hMinReliableElSpeed01Hz,
  *         bMaximumSpeedErrorsNumber and/or specific parameters of the derived
  *         TRUE = sensor information is reliable
  *         FALSE = sensor information is not reliable
  * @param  this related object of class CSPD
  * @param  pMecSpeed01Hz pointer to int16_t, used to return the rotor average
  *         mechanical speed (01Hz)
  * @retval none
  */
bool SPD_CalcAvrgMecSpeed01Hz(CSPD this, int16_t *pMecSpeed01Hz)
{
  bool SpeedSensorReliability = TRUE;
  uint8_t bSpeedErrorNumber;
  uint8_t bMaximumSpeedErrorsNumber = ((_CSPD)this)->pParams_str->bMaximumSpeedErrorsNumber;
  
  if (((_CSPD)this)->Methods_str.pSPD_CalcAvrgMecSpeed01Hz(this,pMecSpeed01Hz) 
      == FALSE)
  {
    /* If derived class speed error check is false the reliabilty of the base
       class is assert */
    bSpeedErrorNumber = bMaximumSpeedErrorsNumber;
    SpeedSensorReliability = FALSE;
  }
  else
  {
    bool SpeedError = FALSE;
    uint16_t hAbsMecSpeed01Hz, hAbsMecAccel01HzP;
    int16_t hAux;
    
    bSpeedErrorNumber = ((_CSPD)this)->Vars_str.bSpeedErrorNumber;
    
    /* Compute absoulte value of mechanical speed */
    if (*pMecSpeed01Hz < 0)
    {
      hAux = -(*pMecSpeed01Hz);
      hAbsMecSpeed01Hz = (uint16_t)(hAux);
    }
    else
    {
      hAbsMecSpeed01Hz = (uint16_t)(*pMecSpeed01Hz);
    }
    
    if (hAbsMecSpeed01Hz > ((_CSPD)this)->pParams_str->hMaxReliableMecSpeed01Hz)
    {
      SpeedError = TRUE;
    }
    
    if (hAbsMecSpeed01Hz < ((_CSPD)this)->pParams_str->hMinReliableMecSpeed01Hz)
    {
      SpeedError = TRUE;
    }
    
    /* Compute absoulte value of mechanical acceleration */
    if (((_CSPD)this)->Vars_str.hMecAccel01HzP < 0)
    {
      hAux = -(((_CSPD)this)->Vars_str.hMecAccel01HzP);
      hAbsMecAccel01HzP = (uint16_t)(hAux);
    }
    else
    {
      hAbsMecAccel01HzP = (uint16_t)(((_CSPD)this)->Vars_str.hMecAccel01HzP);
    }
    
    if ( hAbsMecAccel01HzP >
        ((_CSPD)this)->pParams_str->hMaxReliableMecAccel01HzP)
    {
      SpeedError = TRUE;
    }
    
    if (SpeedError == TRUE)
    {
      if (bSpeedErrorNumber < bMaximumSpeedErrorsNumber)
      {
        bSpeedErrorNumber++;
      }
    }
    else
    {
      if (bSpeedErrorNumber < bMaximumSpeedErrorsNumber)
      {
        bSpeedErrorNumber = 0u;
      }
    }
    
    if (bSpeedErrorNumber == bMaximumSpeedErrorsNumber)
    { 
      SpeedSensorReliability = FALSE; 
    }
  }
  
  ((_CSPD)this)->Vars_str.bSpeedErrorNumber = bSpeedErrorNumber;
  
  return(SpeedSensorReliability);
}

/**
  * @brief  This method returns the average mechanical rotor speed expressed in
  *         "S16Speed". It means that:\n
  *         - it is zero for zero speed,\n
  *         - it become S16_MAX when the average mechanical speed is equal to 
  *           hMaxReliableMecSpeed01Hz,\n
  *         - it becomes -S16_MAX when the average mechanical speed is equal to
  *         -hMaxReliableMecSpeed01Hz.
  * @param  this related object of class CSPD
  * @retval int16_t The average mechanical rotor speed expressed in
  *         "S16Speed".
  */
int16_t SPD_GetS16Speed(CSPD this)
{
  int32_t wAux = (int32_t)((_CSPD)this)->Vars_str.hAvrMecSpeed01Hz;
  wAux *= S16_MAX;
  wAux /= (int16_t)((_CSPD)this)->pParams_str->hMaxReliableMecSpeed01Hz;
  return (int16_t)wAux;
}

/**
  * @brief  This method returns the coefficient used to transform electrical to
  *         mechanical quantities and viceversa. It usually coincides with motor
  *         pole pairs number.
  * @param  this related object of class CSPD
  * @retval uint8_t The motor pole pairs number.
  */
uint8_t SPD_GetElToMecRatio(CSPD this)
{
  return (((_CSPD)this)->Vars_str.bElToMecRatio);
}

/**
  * @brief  This method sets the coefficient used to transform electrical to
  *         mechanical quantities and viceversa. It usually coincides with motor
  *         pole pairs number.
  * @param  this related object of class CSPD
  * @param  bPP The motor pole pairs number to be set.
  */
void SPD_SetElToMecRatio(CSPD this, uint8_t bPP)
{
  ((_CSPD)this)->Vars_str.bElToMecRatio = bPP;
}

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
