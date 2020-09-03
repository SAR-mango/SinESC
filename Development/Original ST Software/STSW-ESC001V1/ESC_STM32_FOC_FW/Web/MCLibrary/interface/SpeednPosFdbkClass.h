/**
  ******************************************************************************
  * @file    SpeednPosFdbkClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of SpeednPosFdbk class      
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
#ifndef __SPEEDNPOSFDBKCLASS_H
#define __SPEEDNPOSFDBKCLASS_H

/* Includes ------------------------------------------------------------------*/
#include "MC_type.h"

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */

/** @addtogroup SpeednPosFdbk
  * @{
  */
  
/** @defgroup SpeednPosFdbk_class_exported_types SpeednPosFdbk class exported types
* @{
*/

/** 
  * @brief  Public SpeednPosFdbk class definition 
  */
typedef struct CSPD_t *CSPD;

/** 
  * @brief  SpeednPosFdbk class parameters definition  
  */
typedef const struct
{
  uint8_t bElToMecRatio;  /*!< Coefficient used to transform electrical to
                               mechanical quantities and viceversa. It usually
                               coincides with motor pole pairs number*/
  uint16_t hMaxReliableMecSpeed01Hz; /*!< Maximum value of measured speed that is
                                        considered to be valid. It's expressed
                                        in tenth of mechanical Hertz.*/
  uint16_t hMinReliableMecSpeed01Hz; /*!< Minimum value of measured speed that is
                                        considered to be valid. It's expressed
                                        in tenth of mechanical Hertz.*/
  uint8_t bMaximumSpeedErrorsNumber; /*!< Maximum value of not valid measurements
                                        before an error is reported.*/
  uint16_t hMaxReliableMecAccel01HzP; /*!< Maximum value of measured acceleration
                                        that is considered to be valid. It's
                                        expressed in 01HzP (tenth of Hertz per
                                        speed calculation period)*/
  uint16_t hMeasurementFrequency;  /*!< Frequency on which the user will request
                                    a measurement of the rotor electrical angle.
                                    It's also used to convert measured speed from
                                    tenth of Hz to dpp and viceversa.*/
}SpeednPosFdbkParams_t, *pSpeednPosFdbkParams_t;
  
/**
* @}
*/

/** @defgroup SpeednPosFdbk_class_exported_methods SpeednPosFdbk class exported methods
  * @{
  */

/**
  * @brief  Creates an object of the class SpeednPosFdbk
  * @param  pSpeednPosFdbkParams pointer to an SpeednPosFdbk parameters structure
  * @retval CSPD new instance of SpeednPosFdbk object
  */
CSPD SPD_NewObject(pSpeednPosFdbkParams_t pSpeednPosFdbkParams);

/**
  * @brief  Initiliazes all the object variables and MCU peripherals, usually
  *         it has to be called once right after object creation
  * @param  this related object of class CSPD
  * @retval none
  */
void SPD_Init(CSPD this);

/**
  * @brief  It returns the last computed rotor electrical angle, expressed in
  *         s16degrees. 1 s16degree = 360°/65536
  * @param  this related object of class CSPD
  * @retval int16_t rotor electrical angle (s16degrees)
  */
int16_t SPD_GetElAngle(CSPD this);

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
int16_t SPD_GetMecAngle(CSPD this);

/**
  * @brief  It returns the last computed average mechanical speed, expressed in
  *         01Hz (tenth of Hertz).
  * @param  this related object of class CSPD
  * @retval int16_t rotor average mechanical speed (01Hz)
  */
int16_t SPD_GetAvrgMecSpeed01Hz(CSPD this);

/**
  * @brief  It returns the last computed electrical speed, expressed in Dpp.
  *         1 Dpp = 1 s16Degree/control Period. The control period is the period
  *         on which the rotor electrical angle is computed (through function
  *         SPD_CalcElectricalAngle).
  * @param  this related object of class CSPD
  * @retval int16_t rotor electrical speed (Dpp)
  */
int16_t SPD_GetElSpeedDpp(CSPD this);

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
void SPD_SetMecAngle(CSPD this, int16_t hMecAngle);

/**
  * @brief  It restore all the functional variables to initial values.
  * @param  this related object of class CSPD
  * @retval none
  */
void SPD_Clear(CSPD this);

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
bool SPD_Check(CSPD this);

/**
  * @brief  This method must be called with the same periodicity on which FOC
  *         is executed. It computes and returns the rotor electrical angle,
  *         expressed in s16Degrees. 1 s16Degree = 360°/65536. It computes also
  *         the rotor mechanical angle
  * @param  this related object of class CSPD
  * @param  pInputVars_str pointer to input structure. For derived 
			class STO input structure type is Observer_Inputs_t, for HALL and 
			ENCODER this parameter will not be used (thus it can be equal to 
			MC_NULL).
  * @retval int16_t rotor electrical angle (s16Degrees)
  */
int16_t SPD_CalcAngle(CSPD this, void *pInputVars_str);

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
bool SPD_CalcAvrgMecSpeed01Hz(CSPD this, int16_t *pMecSpeed01Hz);

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
int16_t SPD_GetS16Speed(CSPD this);

/**
  * @brief  This method returns the coefficient used to transform electrical to
  *         mechanical quantities and viceversa. It usually coincides with motor
  *         pole pairs number.
  * @param  this related object of class CSPD
  * @retval uint8_t The motor pole pairs number.
  */
uint8_t SPD_GetElToMecRatio(CSPD this);

/**
  * @brief  This method sets the coefficient used to transform electrical to
  *         mechanical quantities and viceversa. It usually coincides with motor
  *         pole pairs number.
  * @param  this related object of class CSPD
  * @param  bPP The motor pole pairs number to be set.
  */
void SPD_SetElToMecRatio(CSPD this, uint8_t bPP);

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /* __SPEEDNPOSFDBKCLASS_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
