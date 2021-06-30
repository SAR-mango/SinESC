/**
  ******************************************************************************
  * @file    LCDVintage_UserInterfaceClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private impelementation of LCDVintage class      
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
#include "UserInterfaceClass.h"
#include "UserInterfacePrivate.h"
#include "LCDVintage_UserInterfaceClass.h"
#include "LCDVintage_UserInterfacePrivate.h"
#include "MC_type.h"
#include "Timebase.h"
#include "Parameters conversion.h"
#ifdef DUALDRIVE
#include "Parameters conversion motor 2.h"
#endif

#if (defined(USE_STM32303C_EVAL) || defined (P_NUCLEO_IHM001))
  #include "stm32f30x.h"
  #include "stm32303c_eval.h"
  #include "stm32303c_eval_lcd.h"
#else
  #include "stm32_eval.h"
#endif

/* Constants --------------------------------------------------------*/
#define CONTROL_MODE_SPEED_MENU  (uint8_t) 0
#define REF_SPEED_MENU           (uint8_t) 1
#define MOTOR_SPD_MENU           (uint8_t) 26
#define MOTOR_TRQ_MENU           (uint8_t) 27

#define P_SPEED_MENU             (uint8_t) 2
#define I_SPEED_MENU             (uint8_t) 3
#define D_SPEED_MENU             (uint8_t) 4

#define P_TORQUE_MENU            (uint8_t) 5
#define I_TORQUE_MENU            (uint8_t) 6
#define D_TORQUE_MENU            (uint8_t) 7

#define P_FLUX_MENU              (uint8_t) 8
#define I_FLUX_MENU              (uint8_t) 9
#define D_FLUX_MENU              (uint8_t) 10

#define POWER_STAGE_MENU         (uint8_t) 11

#define CONTROL_MODE_TORQUE_MENU (uint8_t) 12
#define IQ_REF_MENU              (uint8_t) 13
#define ID_REF_MENU              (uint8_t) 14

#define FAULT_MENU               (uint8_t) 15

#define WAIT_MENU                (uint8_t) 16

#ifdef OBSERVER_GAIN_TUNING
#define K1_MENU				   (uint8_t) 17
#define K2_MENU				   (uint8_t) 18
#define P_PLL_MENU			   (uint8_t) 19
#define I_PLL_MENU			   (uint8_t) 20
#endif

#if (DAC_FUNCTIONALITY == ENABLE)
#define DAC_PB0_MENU		           (uint8_t) 21
#define DAC_PB1_MENU                       (uint8_t) 22
#endif

#ifdef FLUX_WEAKENING
#define P_VOLT_MENU             (uint8_t)23
#define I_VOLT_MENU             (uint8_t)24
#define TARGET_VOLT_MENU        (uint8_t)25
#endif

#define BLINKING_TIME   5  /* 5 * timebase_display_5 ms */

#define VISUALIZATION_1   (uint8_t)1
#define VISUALIZATION_2   (uint8_t)2
#define VISUALIZATION_3   (uint8_t)3
#define VISUALIZATION_4   (uint8_t)4
#define VISUALIZATION_5   (uint8_t)5
#define VISUALIZATION_6   (uint8_t)6
#define VISUALIZATION_7   (uint8_t)7
#define VISUALIZATION_8   (uint8_t)8
#define VISUALIZATION_9   (uint8_t)9
#define VISUALIZATION_10  (uint8_t)10
#ifdef FLUX_WEAKENING
#define VISUALIZATION_11  (uint8_t)11
#endif
#define CHAR_0            (uint8_t)0 /*First character of the line starting from the left */
#define CHAR_1            (uint8_t)1 
#define CHAR_2            (uint8_t)2
#define CHAR_3            (uint8_t)3
#define CHAR_4            (uint8_t)4
#define CHAR_5            (uint8_t)5
#define CHAR_6            (uint8_t)6
#define CHAR_7            (uint8_t)7
#define CHAR_8            (uint8_t)8
#define CHAR_9            (uint8_t)9
#define CHAR_10           (uint8_t)10
#define CHAR_11           (uint8_t)11
#define CHAR_12           (uint8_t)12
#define CHAR_13           (uint8_t)13
#define CHAR_14           (uint8_t)14
#define CHAR_15           (uint8_t)15
#define CHAR_16           (uint8_t)16
#define CHAR_17           (uint8_t)17

#ifdef OBSERVER_GAIN_TUNING 
#define CHAR_18           (uint8_t)18
#endif

#if (DAC_FUNCTIONALITY == ENABLE)
#define CHAR_19           (uint8_t)19
#endif

/* Key ------------------------------------------------------------*/
#define  NOKEY      (uint8_t)0
#define  SEL        (uint8_t)1
#define  RIGHT      (uint8_t)2
#define  LEFT       (uint8_t)3
#define  UP         (uint8_t)4
#define  DOWN       (uint8_t)5
#define  KEY_HOLD   (uint8_t)6

#define KEY_UP_PORT UP_BUTTON_GPIO_PORT
#define KEY_UP_BIT  UP_BUTTON_PIN

#define KEY_DOWN_PORT DOWN_BUTTON_GPIO_PORT
#define KEY_DOWN_BIT  DOWN_BUTTON_PIN

#define KEY_RIGHT_PORT RIGHT_BUTTON_GPIO_PORT
#define KEY_RIGHT_BIT  RIGHT_BUTTON_PIN

#define KEY_LEFT_PORT LEFT_BUTTON_GPIO_PORT
#define KEY_LEFT_BIT  LEFT_BUTTON_PIN

#define KEY_SEL_PORT SEL_BUTTON_GPIO_PORT
#define KEY_SEL_BIT  SEL_BUTTON_PIN

#define USER_BUTTON_PORT KEY_BUTTON_GPIO_PORT
#define USER_BUTTON_BIT  KEY_BUTTON_PIN

#define  SEL_FLAG        (uint8_t)0x02
#define  RIGHT_FLAG      (uint8_t)0x04
#define  LEFT_FLAG       (uint8_t)0x08
#define  UP_FLAG         (uint8_t)0x10
#define  DOWN_FLAG       (uint8_t)0x20

/* Variable increment and decrement */
#define SPEED_INC_DEC     (uint16_t)10
#define SPEED_INC_DEC_DURATION (uint16_t)100
#define KP_GAIN_INC_DEC   (uint16_t)250
#define KI_GAIN_INC_DEC   (uint16_t)25
#define KD_GAIN_INC_DEC   (uint16_t)100

#ifdef FLUX_WEAKENING
#define KP_VOLT_INC_DEC   (uint8_t)50
#define KI_VOLT_INC_DEC   (uint8_t)10
#define VOLT_LIM_INC_DEC  (uint8_t)5 
#endif

#define TORQUE_INC_DEC    (uint16_t)250
#define FLUX_INC_DEC      (uint16_t)250

#define K1_INC_DEC        (int16_t)(250)
#define K2_INC_DEC        (int16_t)(5000)

#define PLL_IN_DEC        (uint16_t)(25)

/* Private typedef */
typedef enum InputKey_e
{
  KEY_NONE           = 0,
  KEY_JOYSTICK_UP    = 1,
  KEY_JOYSTICK_DOWN  = 2,
  KEY_JOYSTICK_LEFT  = 3,
  KEY_JOYSTICK_RIGHT = 4,
  KEY_JOYSTICK_SEL   = 5,
  KEY_USER_BUTTON    = 6
} InputKey;

/* Private variables ---------------------------------------------------------*/
volatile static uint16_t hTimebase_Blinking;
static uint8_t bPrevious_Visualization = 0;
static uint8_t bPresent_Visualization;
static uint8_t bMenu_index;
static uint8_t bKey;
static uint8_t bPrevious_key;
static uint8_t bKey_Flag;

#ifdef MC_CLASS_DYNAMIC
  #include "stdlib.h" /* Used for dynamic allocation */
#else
  #define MAX_LCDV_UI_NUM 1u
  _DCLCDV_UI_t LCDV_UIpool[MAX_LCDV_UI_NUM];
  unsigned char LCDV_UI_Allocated = 0u;
#endif

#define DCLASS_PARAM ((_DCLCDV_UI)(((_CUI) this)->DerivedClass))->pDParams_str
#define DCLASS_VARS  &(((_DCLCDV_UI)(((_CUI) this)->DerivedClass))->DVars_str)
#define  CLASS_VARS  &(((_CUI)this)->Vars_str)
#define  CLASS_PARAM (((_CUI)this)->pParams_str)

static void LCDV_Init(CUI this, CUI oDAC, const char* s_fwVer);
static void LCDV_Exec(CUI this);
static void LCDV_UpdateAll(CUI this);
static void LCDV_UpdateMeasured(CUI this);
static void Display_LCD(CUI this);
static void KEYS_process(CUI this);

static void Display_5DigitSignedNumber(uint8_t Line, uint8_t bFirstchar, int16_t number);
static void Display_3DigitUnsignedNumber(uint8_t Line, uint8_t bFirstchar, int16_t number);
static void Display_1DigitUnsignedNumber(uint8_t Line, uint8_t bFirstchar, uint8_t number);
static void Display_5dot_line(uint16_t Line, uint16_t Column);
static void DisplayChar(uint16_t Line, uint16_t Column, uint8_t Ascii);
static bool JOY_Pressed(InputKey inKey);
static uint8_t ComputeVisualization(uint8_t bLocal_MenuIndex, State_t State);
static uint8_t LCDV_DACIDToSel(MC_Protocol_REG_t ID);
void LCDV_DACSelUpdate(CUI this, int8_t bInc, DAC_Channel_t bCh);

#define GUI_DAC_ChID_LAST_ELEMENT 0xFF

const char * const GUI_DAC_ChTxt[] = {
  "Ia               ","Ib               ",
  "Ialpha           ","Ibeta            ","Iq               ",
  "Id               ","Iq ref           ","Id ref           ",
  "Vq               ","Vd               ","Valpha           ",
  "Vbeta            ","Meas. El Angle   ","Meas. Rotor Speed",
  "HFI El Angle     ","HFI Rotor Speed  ","HFI debug current","HFI debug angle  ",
  "Obs. El Ang.(PLL)","Obs. Rot.Spd(PLL)","Obs. Ialpha (PLL)",
  "Obs. Ibeta  (PLL)","Obs. Bemf a.(PLL)","Obs. Bemf b.(PLL)",
  "Exp. Bemf l.(PLL)","Obs. Bemf l.(PLL)",
  "Obs. El Ang. (CR)","Obs. Rot.Spd (CR)","Obs. Ialpha  (CR)",
  "Obs. Ibeta   (CR)","Obs. Bemf a. (CR)","Obs. Bemf b. (CR)",
  "Exp. Bemf l. (CR)","Obs. Bemf l. (CR)",
  "User 1           ","User 2           ", MC_NULL};

const MC_Protocol_REG_t GUI_DAC_ChID[] = {
  MC_PROTOCOL_REG_I_A,
  MC_PROTOCOL_REG_I_B,
  MC_PROTOCOL_REG_I_ALPHA,
  MC_PROTOCOL_REG_I_BETA,
  MC_PROTOCOL_REG_I_Q,
  MC_PROTOCOL_REG_I_D,
  MC_PROTOCOL_REG_I_Q_REF,
  MC_PROTOCOL_REG_I_D_REF,
  MC_PROTOCOL_REG_V_Q,
  MC_PROTOCOL_REG_V_D,
  MC_PROTOCOL_REG_V_ALPHA,
  MC_PROTOCOL_REG_V_BETA,
  MC_PROTOCOL_REG_MEAS_EL_ANGLE,
  MC_PROTOCOL_REG_MEAS_ROT_SPEED,
  MC_PROTOCOL_REG_HFI_EL_ANGLE,
  MC_PROTOCOL_REG_HFI_ROT_SPEED,
  MC_PROTOCOL_REG_HFI_CURRENT,
  MC_PROTOCOL_REG_HFI_INIT_ANG_PLL,
  MC_PROTOCOL_REG_OBS_EL_ANGLE,
  MC_PROTOCOL_REG_OBS_ROT_SPEED,
  MC_PROTOCOL_REG_OBS_I_ALPHA,
  MC_PROTOCOL_REG_OBS_I_BETA,
  MC_PROTOCOL_REG_OBS_BEMF_ALPHA,
  MC_PROTOCOL_REG_OBS_BEMF_BETA,
  MC_PROTOCOL_REG_EST_BEMF_LEVEL,
  MC_PROTOCOL_REG_OBS_BEMF_LEVEL,
  MC_PROTOCOL_REG_OBS_CR_EL_ANGLE,
  MC_PROTOCOL_REG_OBS_CR_ROT_SPEED,
  MC_PROTOCOL_REG_OBS_CR_I_ALPHA,
  MC_PROTOCOL_REG_OBS_CR_I_BETA,
  MC_PROTOCOL_REG_OBS_CR_BEMF_ALPHA,
  MC_PROTOCOL_REG_OBS_CR_BEMF_BETA,
  MC_PROTOCOL_REG_EST_CR_BEMF_LEVEL,
  MC_PROTOCOL_REG_OBS_CR_BEMF_LEVEL,
  MC_PROTOCOL_REG_DAC_USER1,
  MC_PROTOCOL_REG_DAC_USER2,
  (MC_Protocol_REG_t)(GUI_DAC_ChID_LAST_ELEMENT)
};

/**
  * @brief  Creates an object of the class LCDVintage
  * @param  pUserInterfaceParams pointer to an UserInterface parameters structure
  * @param  pLCDVintageParams pointer to an LCDVintage parameters structure
  * @retval CLCDV_UI new instance of LCDVintage object
  */
CLCDV_UI LCDV_NewObject(pUserInterfaceParams_t pUserInterfaceParams, pLCDVintageParams_t pLCDVintageParams)
{
	_CUI _oUserInterface;
	_DCLCDV_UI _oLCDVintage;

	_oUserInterface = (_CUI)UI_NewObject(pUserInterfaceParams);

	#ifdef MC_CLASS_DYNAMIC
		_oLCDVintage = (_DCLCDV_UI)calloc(1u,sizeof(_DCLCDV_UI_t));
	#else
		if (LCDV_UI_Allocated  < MAX_LCDV_UI_NUM)
		{
			_oLCDVintage = &LCDV_UIpool[LCDV_UI_Allocated++];
		}
		else
		{
			_oLCDVintage = MC_NULL;
		}
	#endif
  
	_oLCDVintage->pDParams_str = pLCDVintageParams;
	_oUserInterface->DerivedClass = (void*)_oLCDVintage;
  
	_oUserInterface->Methods_str.pUI_LCDInit = &LCDV_Init;
        _oUserInterface->Methods_str.pUI_LCDExec = &LCDV_Exec;
        _oUserInterface->Methods_str.pUI_LCDUpdateAll = &LCDV_UpdateAll;
        _oUserInterface->Methods_str.pUI_LCDUpdateMeasured = &LCDV_UpdateMeasured;

	return ((CLCDV_UI)_oUserInterface);
}

/** @addtogroup STM32F10x_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup UserInterface_LCDVintage
  * @{
  */

/** @defgroup LCDVintage_class_private_methods LCDVintage class private methods
* @{
*/

/**
  * @brief  Initialization of LCD object. It must be called after the UI_Init.
  * @param  this related object of class CUI. It must be a LCDx_UI object casted
  *         to CUI otherwise the method will have no effect.
  * @param  oDAC related DAC object upcasted to CUI. It can be MC_NULL.
  * @param  s_fwVer String contating firmware version.
  * @retval none.
  */
static void LCDV_Init(CUI this, CUI oDAC, const char* s_fwVer)
{
  pVars_t pVars = CLASS_VARS;
  pDVars_t pDVars = DCLASS_VARS;
  CMCT oMCT = pVars->pMCT[pVars->bSelectedDrive];
  CSTC oSTC = MCT_GetSpeednTorqueController(oMCT);
	uint8_t *ptr;
  
  /* Initialize the LCD */
  LCD_HW_Init();
  
  LCD_Clear(White);
  
  LCD_SetBackColor(White);
  LCD_SetTextColor(Black);
  
  /* Initialize Joystick */
  STM_EVAL_JOYInit();
  
  /* Welcome message */
  ptr = " STM32 Motor Control";
  
  LCD_DisplayStringLine(Line0, ptr);
  
  ptr = "  PMSM FOC ver 4.0  ";
  LCD_DisplayStringLine(Line1, ptr);
  
  ptr = " <> Move  ^| Change ";
  LCD_DisplayStringLine(Line9, ptr);
  
  /* Initialize vars */
  pDVars->oDAC = oDAC;
  pDVars->bDAC_CH0_ID = MC_PROTOCOL_REG_UNDEFINED;
  pDVars->bDAC_CH1_ID = MC_PROTOCOL_REG_UNDEFINED;
  pDVars->bDAC_CH0_Sel = 0;
  pDVars->bDAC_CH1_Sel = 0;
  pDVars->bDAC_Size = LCDV_DACIDToSel((MC_Protocol_REG_t)(GUI_DAC_ChID_LAST_ELEMENT));
  
  pDVars->Iqdref = STC_GetDefaultIqdref(oSTC);
}

/**
  * @brief  Execute the LCD execution and refreshing. It must be called 
  *         periodically.
  * @param  this related object of class CUI. It must be a LCDx_UI object casted
  *         to CUI otherwise the method will have no effect.
  * @retval none.
  */
static void LCDV_Exec(CUI this)
{
  pDVars_t pDVars = DCLASS_VARS;
  uint8_t bSel,bSize = pDVars->bDAC_Size;
  MC_Protocol_REG_t ID;
  
  Display_LCD(this);
  KEYS_process(this);
  
  /* Update selected DAC variable */
  ID = UI_GetDAC(pDVars->oDAC, DAC_CH0);
  if (pDVars->bDAC_CH0_ID != ID)
  {
    pDVars->bDAC_CH0_ID = ID;
    bSel = LCDV_DACIDToSel(ID);
    if (bSel < bSize)
    {
      pDVars->bDAC_CH0_Sel = bSel;
    }
  }
  ID = UI_GetDAC(pDVars->oDAC, DAC_CH1);
  if (pDVars->bDAC_CH1_ID != ID)
  {
    pDVars->bDAC_CH1_ID = ID;
    bSel = LCDV_DACIDToSel(ID);
    if (bSel < bSize)
    {
      pDVars->bDAC_CH1_Sel = bSel;
    }
  }
}

/**
  * @brief  It is used to force a refresh of all LCD values.
  * @param  this related object of class CUI. It must be a LCDx_UI object casted
  *         to CUI otherwise the method will have no effect.
  * @retval none.
  */
static void LCDV_UpdateAll(CUI this)
{
}

/**
  * @brief  It is used to force a refresh of only measured LCD values.
  * @param  this related object of class CUI. It must be a LCDx_UI object casted
  *         to CUI otherwise the method will have no effect.
  * @retval none.
  */
static void LCDV_UpdateMeasured(CUI this)
{
}
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

static uint8_t LCDV_DACIDToSel(MC_Protocol_REG_t ID)
{
  uint8_t retVal;
  for (retVal = 0; retVal < GUI_DAC_ChID_LAST_ELEMENT; retVal++)
  {
    if ((GUI_DAC_ChID[retVal] == ID) ||
        (GUI_DAC_ChID[retVal] == GUI_DAC_ChID_LAST_ELEMENT) ||
        (retVal == 0xFF))
    {
      break;
    }
  }
  return retVal;
}

void LCDV_DACSelUpdate(CUI this, int8_t bInc, DAC_Channel_t bCh)
{
  pDVars_t pDVars = DCLASS_VARS;
  uint8_t bSize= pDVars->bDAC_Size;
  int8_t bSel;

  if (bCh == DAC_CH0)
  {
    bSel = (int8_t)(pDVars->bDAC_CH0_Sel);
  }
  else if (bCh == DAC_CH1)
  {
    bSel = (int8_t)(pDVars->bDAC_CH1_Sel);
  }
  
  bSel += bInc;
  
  if (bSel < 0)
  {
    bSel += bSize;
  }
  if (bSel >= bSize)
  {
    bSel -= bSize;
  }
  
  if (bCh == DAC_CH0)
  {
    pDVars->bDAC_CH0_Sel = (uint8_t)(bSel);
  }
  else if (bCh == DAC_CH1)
  {
    pDVars->bDAC_CH1_Sel = (uint8_t)(bSel);
  }
  
  UI_SetDAC(pDVars->oDAC, bCh, GUI_DAC_ChID[bSel]);
}

/*******************************************************************************
* Function Name  : Display_LCD
* Description    : Display routine for LCD management
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Display_LCD(CUI this)
{
  pVars_t pVars = CLASS_VARS;
  pDVars_t pDVars = DCLASS_VARS;
  CMCT oMCT = pVars->pMCT[pVars->bSelectedDrive];
  CMCI oMCI = pVars->pMCI[pVars->bSelectedDrive];
  CPI oSpeedLoopPID = MCT_GetSpeedLoopPID(oMCT);
  CPI oIqLoopPID = MCT_GetIqLoopPID(oMCT);
  CPI oIdLoopPID = MCT_GetIdLoopPID(oMCT);
  CPI oFluxWeakeningLoopPID = MCT_GetFluxWeakeningLoopPID(oMCT);
  CFW oFWCtrl = MCT_GetFluxWeakeningCtrl(oMCT);
  CVBS oVBS = MCT_GetBusVoltageSensor(oMCT);
  CTSNS oTSNS = MCT_GetTemperatureSensor(oMCT);
  State_t State = MCI_GetSTMState(oMCI);
  uint16_t hCurrentFault = MCI_GetCurrentFaults(oMCI);
  uint16_t hOccurredFault = MCI_GetOccurredFaults(oMCI);
  
  bPrevious_Visualization = bPresent_Visualization;
  
  if (((bMenu_index == CONTROL_MODE_SPEED_MENU) ||
       (bMenu_index == CONTROL_MODE_TORQUE_MENU)) &&
      (State != START))
  {
    if (MCI_GetControlMode(oMCI) == STC_SPEED_MODE)
    {
      bMenu_index = CONTROL_MODE_SPEED_MENU;
    }
    else
    {
      bMenu_index = CONTROL_MODE_TORQUE_MENU;
    }
  }
  
  bPresent_Visualization = ComputeVisualization(bMenu_index, State);
  
  switch(bPresent_Visualization)
  {
    uint8_t *ptr;
    int16_t temp;
    
  case VISUALIZATION_1:
    if (bPresent_Visualization != bPrevious_Visualization)
    {         
      LCD_ClearLine(Line3); 
      
      LCD_ClearLine(Line4); 
      
      ptr = " Target     Measured";
      LCD_DisplayStringLine(Line5,ptr); 
      
      ptr = "       (rpm)        ";
      LCD_DisplayStringLine(Line7,ptr); 
      
      LCD_ClearLine(Line6);        
      
      LCD_ClearLine(Line8);
      
      ptr = " <> Move  ^| Change ";          
      LCD_DisplayStringLine(Line9, ptr); 
    }
    
    if (bMenu_index == MOTOR_SPD_MENU)
    {
      LCD_SetTextColor(Red);
    }
    else
    {
      LCD_SetTextColor(Blue);
    }
    
    ptr = "      Motor ";
    LCD_DisplayStringLine(Line2,ptr);
    
    temp = pVars->bSelectedDrive;
    Display_1DigitUnsignedNumber(Line2, CHAR_12, temp + 1); /* Zero based */
    
    if(bMenu_index == CONTROL_MODE_SPEED_MENU)
    {
      LCD_SetTextColor(Red);
    }
    else
    {
      LCD_SetTextColor(Blue);
    }
    
    ptr = " Speed control mode";        
    LCD_DisplayStringLine(Line3,ptr);
    
    if(bMenu_index == REF_SPEED_MENU)
    {
      LCD_SetTextColor(Red);
    }
    else
    {
      LCD_SetTextColor(Blue);
    }
    
    /* Compute target speed in rpm */
    temp = (int16_t)(MCI_GetLastRampFinalSpeed(oMCI) * 6);                 
    Display_5DigitSignedNumber(Line7, CHAR_0, temp);
    
    LCD_SetTextColor(Blue);
    
    /* Compute measured speed in rpm */
    temp = (int16_t)(MCI_GetAvrgMecSpeed01Hz(oMCI) * 6);
    Display_5DigitSignedNumber(Line7, CHAR_13, temp); 
    
    break;
    
  case VISUALIZATION_2:
    if (bPresent_Visualization != bPrevious_Visualization)
    {           
      ptr = "       Speed        ";
      LCD_DisplayStringLine(Line2,ptr);
      
      ptr = "    P     I     D   ";
      LCD_DisplayStringLine(Line3,ptr); 
      
      LCD_ClearLine(Line4);
      LCD_ClearLine(Line5);
      
      ptr = " Target        (rpm)";
      LCD_DisplayStringLine(Line6,ptr); 
      
      ptr = " Measured      (rpm)";
      LCD_DisplayStringLine(Line7,ptr);
      
      LCD_ClearLine(Line8);
      
      ptr = " <> Move  ^| Change ";          
      LCD_DisplayStringLine(Line9, ptr); 
    }
    
    switch(bMenu_index)
    {
    case P_SPEED_MENU:
      LCD_SetTextColor(Red);            
      temp = PI_GetKP(oSpeedLoopPID);
      Display_5DigitSignedNumber(Line4, CHAR_1, temp);
      LCD_SetTextColor(Blue);
      
      temp = PI_GetKI(oSpeedLoopPID);
      Display_5DigitSignedNumber(Line4, CHAR_7, temp);
      
#ifdef DIFFERENTIAL_TERM_ENABLED            
      temp = PID_GetKD((CPID_PI)oSpeedLoopPID);
      Display_5DigitSignedNumber(Line4, CHAR_13, temp);
#else
      Display_5dot_line(Line4, 18);
#endif         
      
      break;
      
    case I_SPEED_MENU:                                 
      temp = PI_GetKP(oSpeedLoopPID);
      Display_5DigitSignedNumber(Line4, CHAR_1, temp);
      
      LCD_SetTextColor(Red);   
      temp = PI_GetKI(oSpeedLoopPID);
      Display_5DigitSignedNumber(Line4, CHAR_7, temp);
      LCD_SetTextColor(Blue);
      
#ifdef DIFFERENTIAL_TERM_ENABLED            
      temp = PID_GetKD((CPID_PI)oSpeedLoopPID);
      Display_5DigitSignedNumber(Line4, CHAR_13, temp);
#else        
      Display_5dot_line(Line4, 18);
#endif
      break;
      
#ifdef DIFFERENTIAL_TERM_ENABLED
    case D_SPEED_MENU:
      temp = PI_GetKP(oSpeedLoopPID);
      Display_5DigitSignedNumber(Line4, CHAR_1, temp);
      
      temp = PI_GetKI(oSpeedLoopPID);
      Display_5DigitSignedNumber(Line4, CHAR_7, temp);
      
      LCD_SetTextColor(Red);
      temp = PID_GetKD((CPID_PI)oSpeedLoopPID);
      Display_5DigitSignedNumber(Line4, CHAR_13, temp);
      LCD_SetTextColor(Blue);
      
      break;
#endif
    default:
      break;
    }
    //Independently from the menu, this visualization must display current 
    //and measured speeds
    
    /* Display target speed in rpm */
    temp = (int16_t)(MCI_GetLastRampFinalSpeed(oMCI) * 6);          
    Display_5DigitSignedNumber(Line6, CHAR_9, temp);
    
    /* Compute measured speed in rpm */
    temp = (int16_t)(MCI_GetAvrgMecSpeed01Hz(oMCI) * 6);
    Display_5DigitSignedNumber(Line7, CHAR_9, temp);         
    break;
    
  case VISUALIZATION_3:
    if (bPresent_Visualization != bPrevious_Visualization)
    {           
      ptr = "       Torque       ";
      LCD_DisplayStringLine(Line2,ptr);
      
      ptr = "    P     I     D   ";
      LCD_DisplayStringLine(Line3,ptr); 
      
      LCD_ClearLine(Line4);
      LCD_ClearLine(Line5);
      
      ptr = " Target         (Iq)";
      LCD_DisplayStringLine(Line6,ptr); 
      
      ptr = " Measured       (Iq)";
      LCD_DisplayStringLine(Line7,ptr);
      
      LCD_ClearLine(Line8);
      
      ptr = " <> Move  ^| Change ";          
      LCD_DisplayStringLine(Line9, ptr); 
    }
    
    switch(bMenu_index)
    {
    case P_TORQUE_MENU:
      LCD_SetTextColor(Red);            
      temp = PI_GetKP(oIqLoopPID);;
      Display_5DigitSignedNumber(Line4, CHAR_1, temp);
      LCD_SetTextColor(Blue);
      
      temp = PI_GetKI(oIqLoopPID);;
      Display_5DigitSignedNumber(Line4, CHAR_7, temp);
      
#ifdef DIFFERENTIAL_TERM_ENABLED            
      temp = PID_GetKD((CPID_PI)oIqLoopPID);
      Display_5DigitSignedNumber(Line4, CHAR_13, temp);
#else        
      Display_5dot_line(Line4, 18);
#endif       
      break;
      
    case I_TORQUE_MENU:                                 
      temp = PI_GetKP(oIqLoopPID);;
      Display_5DigitSignedNumber(Line4, CHAR_1, temp);
      
      LCD_SetTextColor(Red);   
      temp = PI_GetKI(oIqLoopPID);;
      Display_5DigitSignedNumber(Line4, CHAR_7, temp);
      LCD_SetTextColor(Blue);
      
#ifdef DIFFERENTIAL_TERM_ENABLED             
      temp = PID_GetKD((CPID_PI)oIqLoopPID);
      Display_5DigitSignedNumber(Line4, CHAR_13, temp);
#else        
      Display_5dot_line(Line4, 18);
#endif
      break;
      
#ifdef DIFFERENTIAL_TERM_ENABLED 
    case D_TORQUE_MENU:
      temp = PI_GetKP(oIqLoopPID);;
      Display_5DigitSignedNumber(Line4, CHAR_1, temp);
      
      temp = PI_GetKI(oIqLoopPID);;
      Display_5DigitSignedNumber(Line4, CHAR_7, temp);
      
      LCD_SetTextColor(Red);
      temp = PID_GetKD((CPID_PI)oIqLoopPID);
      Display_5DigitSignedNumber(Line4, CHAR_13, temp);
      LCD_SetTextColor(Blue);
      
      break;
#endif
    default:
      break;
    }
    //Independently from the menu, this visualization must display current 
    //and measured Iq
    
    temp = MCI_GetIqdref(oMCI).qI_Component1;
    Display_5DigitSignedNumber(Line6, CHAR_9, temp);
    
    temp = MCI_GetIqd(oMCI).qI_Component1;
    Display_5DigitSignedNumber(Line7, CHAR_9, temp);        
    break;
    
  case VISUALIZATION_4:
    if (bPresent_Visualization != bPrevious_Visualization)
    {           
      ptr = "        Flux        ";
      LCD_DisplayStringLine(Line2,ptr);
      
      ptr = "    P     I     D   ";
      LCD_DisplayStringLine(Line3,ptr); 
      
      LCD_ClearLine(Line4);
      LCD_ClearLine(Line5);
      
      ptr = " Target         (Id)";
      LCD_DisplayStringLine(Line6,ptr); 
      
      ptr = " Measured       (Id)";
      LCD_DisplayStringLine(Line7,ptr);
      
      LCD_ClearLine(Line8);
      
      ptr = " <> Move  ^| Change ";          
      LCD_DisplayStringLine(Line9, ptr); 
    }
    
    switch(bMenu_index)
    {
    case P_FLUX_MENU:
      LCD_SetTextColor(Red);            
      temp = PI_GetKP(oIdLoopPID);;
      Display_5DigitSignedNumber(Line4, CHAR_1, temp);
      LCD_SetTextColor(Blue);
      
      temp = PI_GetKI(oIdLoopPID);;
      Display_5DigitSignedNumber(Line4, CHAR_7, temp);
      
#ifdef DIFFERENTIAL_TERM_ENABLED            
      temp = PID_GetKD((CPID_PI)oIdLoopPID);
      Display_5DigitSignedNumber(Line4, CHAR_13, temp);
#else        
      Display_5dot_line(Line4, 18);
#endif       
      break;
      
    case I_FLUX_MENU:                                 
      temp = PI_GetKP(oIdLoopPID);;
      Display_5DigitSignedNumber(Line4, CHAR_1, temp);
      
      LCD_SetTextColor(Red);   
      temp = PI_GetKI(oIdLoopPID);;
      Display_5DigitSignedNumber(Line4, CHAR_7, temp);
      LCD_SetTextColor(Blue);
      
#ifdef DIFFERENTIAL_TERM_ENABLED             
      temp = PID_GetKD((CPID_PI)oIdLoopPID);
      Display_5DigitSignedNumber(Line4, CHAR_13, temp);
#else        
      Display_5dot_line(Line4, 18);
#endif
      break;
      
#ifdef DIFFERENTIAL_TERM_ENABLED 
    case D_FLUX_MENU:
      temp = PI_GetKP(oIdLoopPID);;
      Display_5DigitSignedNumber(Line4, CHAR_1, temp);
      
      temp = PI_GetKI(oIdLoopPID);;
      Display_5DigitSignedNumber(Line4, CHAR_7, temp);
      
      LCD_SetTextColor(Red);
      temp = PID_GetKD((CPID_PI)oIdLoopPID);
      Display_5DigitSignedNumber(Line4, CHAR_13, temp);
      LCD_SetTextColor(Blue);
      
      break;
#endif
    default:
      break;
    }
    //Independently from the menu, this visualization must display current 
    //and measured Id
    
    temp = MCI_GetIqdref(oMCI).qI_Component2;          
    Display_5DigitSignedNumber(Line6, CHAR_9, temp);
    
    temp = MCI_GetIqd(oMCI).qI_Component2;
    Display_5DigitSignedNumber(Line7, CHAR_9, temp);   
    break;
    
#ifdef FLUX_WEAKENING      
  case VISUALIZATION_11:
    if (bPresent_Visualization != bPrevious_Visualization)
    {           
      ptr = "Flux Weakening Ctrl ";
      LCD_DisplayStringLine(Line2,ptr);
      
      ptr = "    P     I         ";
      LCD_DisplayStringLine(Line3,ptr); 
      
      LCD_ClearLine(Line4);
      LCD_ClearLine(Line5);
      
      ptr = " Target        (Vs%)";
      LCD_DisplayStringLine(Line6,ptr); 
      
      ptr = " Measured      (Vs%)";
      LCD_DisplayStringLine(Line7,ptr);
      
      LCD_ClearLine(Line8);
      
      ptr = " <> Move  ^| Change ";          
      LCD_DisplayStringLine(Line9, ptr); 
    }
    
    switch(bMenu_index)
    {
    case P_VOLT_MENU:
      LCD_SetTextColor(Red);            
      temp = PI_GetKP(oFluxWeakeningLoopPID);
      Display_5DigitSignedNumber(Line4, CHAR_1, temp);
      
      LCD_SetTextColor(Blue);            
      temp = PI_GetKI(oFluxWeakeningLoopPID);
      Display_5DigitSignedNumber(Line4, CHAR_7, temp);
      temp = FW_GetVref(oFWCtrl);
      Display_5DigitSignedNumber(Line6, CHAR_9, temp);
      LCD_DrawRect(161,97,1,2);            
      
      Display_5dot_line(Line4, 18);
      
      break;
      
    case I_VOLT_MENU:                                 
      temp = PI_GetKP(oFluxWeakeningLoopPID);
      Display_5DigitSignedNumber(Line4, CHAR_1, temp);
      temp = FW_GetVref(oFWCtrl);
      Display_5DigitSignedNumber(Line6, CHAR_9, temp);            
      
      LCD_SetTextColor(Red);   
      temp = PI_GetKI(oFluxWeakeningLoopPID);
      Display_5DigitSignedNumber(Line4, CHAR_7, temp);
      LCD_SetTextColor(Blue);
      LCD_DrawRect(161,97,1,2); 
      
      Display_5dot_line(Line4, 18);
      
      break;
      
    case TARGET_VOLT_MENU:
      LCD_SetTextColor(Red);            
      temp = FW_GetVref(oFWCtrl);
      Display_5DigitSignedNumber(Line6, CHAR_9, temp);
      LCD_DrawRect(161,97,1,2);
      
      LCD_SetTextColor(Blue);
      temp = PI_GetKP(oFluxWeakeningLoopPID);
      Display_5DigitSignedNumber(Line4, CHAR_1, temp);
      temp = PI_GetKI(oFluxWeakeningLoopPID);
      Display_5DigitSignedNumber(Line4, CHAR_7, temp);
      
      Display_5dot_line(Line4, 18);
      
      break;              
      
    default:
      break;
    }
    //Independently from the menu, this visualization must display current 
    //and measured voltage level
    
    //Compute applied voltage in int16_t
    temp = FW_GetAvVPercentage(oFWCtrl);
    Display_5DigitSignedNumber(Line7, CHAR_9, temp);
    LCD_DrawRect(185,97,1,2);
    break;
#endif      
    
  case VISUALIZATION_5:
    if (bPresent_Visualization != bPrevious_Visualization)
    {           
      LCD_ClearLine(Line2);
      
      ptr = " Power Stage Status ";          
      LCD_DisplayStringLine(Line3, ptr); 
      
      LCD_ClearLine(Line4);
      
      ptr = "  DC bus =     Volt ";          
      LCD_DisplayStringLine(Line5, ptr); 
      
      LCD_ClearLine(Line6);
      
      ptr = "  T =      Celsius  ";          
      LCD_DisplayStringLine(Line7, ptr); 
      
      LCD_ClearLine(Line8);
      
      ptr = " <> Move            ";          
      LCD_DisplayStringLine(Line9, ptr); 
    }
    
    temp = VBS_GetAvBusVoltage_V(oVBS);
    Display_3DigitUnsignedNumber(Line5, CHAR_11, temp);
    
    temp = TSNS_GetAvTemp_C(oTSNS);
    Display_3DigitUnsignedNumber(Line7, CHAR_6, temp);
    
    break;    
    
  case VISUALIZATION_6:
    if (bPresent_Visualization != bPrevious_Visualization)
    {                      
      LCD_ClearLine(Line3); 
      
      ptr = "     Target Measured";
      LCD_DisplayStringLine(Line4,ptr);
      
      ptr = "Iq                  ";
      LCD_DisplayStringLine(Line5,ptr); 
      
      ptr = "Id                  ";
      LCD_DisplayStringLine(Line6,ptr);
      
      ptr = "Speed (rpm)         ";
      LCD_DisplayStringLine(Line7,ptr);
      
      LCD_ClearLine(Line8);
      
      ptr = " <> Move  ^| Change ";          
      LCD_DisplayStringLine(Line9, ptr); 
    }
    if (bMenu_index == MOTOR_TRQ_MENU)
    {
      LCD_SetTextColor(Red);
    }
    else
    {
      LCD_SetTextColor(Blue);
    }
    
    ptr = "      Motor ";
    LCD_DisplayStringLine(Line2,ptr);
    
    temp = pVars->bSelectedDrive;
    Display_1DigitUnsignedNumber(Line2, CHAR_12, temp + 1); /* Zero based */
    
    if (bMenu_index == CONTROL_MODE_TORQUE_MENU)
    {
      LCD_SetTextColor(Red);
    }
    else
    {
      LCD_SetTextColor(Blue);
    }
    ptr = "Torque control mode ";        
    LCD_DisplayStringLine(Line3,ptr);
    
    if (bMenu_index == IQ_REF_MENU)
    {
      LCD_SetTextColor(Red);
    }
    else
    {
      LCD_SetTextColor(Blue);
    }
    temp = pDVars->Iqdref.qI_Component1;
    Display_5DigitSignedNumber(Line5, CHAR_5, temp);
    
    if (bMenu_index == ID_REF_MENU)
    {
      LCD_SetTextColor(Red);
    }
    else
    {
      LCD_SetTextColor(Blue);
    }
    temp = pDVars->Iqdref.qI_Component2; 
    Display_5DigitSignedNumber(Line6, CHAR_5, temp);
    
    LCD_SetTextColor(Blue);
    
    temp = MCI_GetIqd(oMCI).qI_Component1;
    Display_5DigitSignedNumber(Line5, CHAR_13, temp);
    
    temp = MCI_GetIqd(oMCI).qI_Component2;        
    Display_5DigitSignedNumber(Line6, CHAR_13, temp);
    
    /* Compute measured speed in rpm */
    temp = (int16_t)(MCI_GetAvrgMecSpeed01Hz(oMCI) * 6); 
    Display_5DigitSignedNumber(Line7, CHAR_13, temp);
    break;
    
  case VISUALIZATION_7:
    if (bPresent_Visualization != bPrevious_Visualization)
    {  
      LCD_SetTextColor(Red);
      
      ptr = "      Motor ";
      LCD_DisplayStringLine(Line2,ptr);
      
      temp = pVars->bSelectedDrive;
      Display_1DigitUnsignedNumber(Line2, CHAR_12, temp + 1); /* Zero based */
      
      ptr = "    !!! FAULT !!!   ";
      LCD_DisplayStringLine(Line3,ptr);
      LCD_SetTextColor(Blue);
      
      if ( (hOccurredFault & MC_UNDER_VOLT) == MC_UNDER_VOLT)
      {           
        ptr = " Bus Under Voltage  ";
        LCD_DisplayStringLine(Line4, ptr);                                   
      }
      else if ( (hOccurredFault & MC_BREAK_IN) ==  MC_BREAK_IN)
      {
        ptr = "   Over Current    ";
        LCD_DisplayStringLine(Line4, ptr); 
      }
      else if ( (hOccurredFault & MC_OVER_TEMP) ==  MC_OVER_TEMP)
      {
        ptr = "   Over Heating    ";
        LCD_DisplayStringLine(Line4, ptr);                             
      }
      else if ( (hOccurredFault & MC_OVER_VOLT) ==  MC_OVER_VOLT)
      {
        ptr = "  Bus Over Voltage  ";
        LCD_DisplayStringLine(Line4, ptr);               
      }
      else if ( (hOccurredFault & MC_START_UP) ==  MC_START_UP)
      {
        ptr = "  Start-up failed   ";
        LCD_DisplayStringLine(Line4, ptr);    
      }      
      else if ( (hOccurredFault & MC_SPEED_FDBK) ==  MC_SPEED_FDBK)
      {
        ptr = "Error on speed fdbck";
        LCD_DisplayStringLine(Line4, ptr);     
      }  
      LCD_ClearLine(Line5);
      LCD_ClearLine(Line7);  
    } 
    
    if ((hCurrentFault & ( MC_OVER_TEMP | MC_UNDER_VOLT | MC_OVER_VOLT)) == 0) 
    { 
      LCD_ClearLine(Line6);
      ptr = "   Press 'Key' to   ";
      LCD_DisplayStringLine(Line8,ptr);
      
      ptr = "   return to menu   ";
      LCD_DisplayStringLine(Line9,ptr);
    }
    else
    {
      if ((hCurrentFault & (MC_UNDER_VOLT | MC_OVER_VOLT)) == 0)    
      {
        /* Under or over voltage */
        if (bPresent_Visualization != bPrevious_Visualization)
        { 
          LCD_ClearLine(Line6);
        }
        temp = TSNS_GetAvTemp_C(oTSNS); 
        ptr = "       T =";  
        LCD_DisplayStringLine(Line6, ptr);
        Display_3DigitUnsignedNumber(Line6, CHAR_11, temp);
        DisplayChar(Line6, CHAR_14, ' ');
        DisplayChar(Line6, CHAR_15, 'C');
      }
      else 
      {           
        if (bPresent_Visualization != bPrevious_Visualization)
        { 
          LCD_ClearLine(Line6);         
        }
        ptr = "  DC bus =";            
        LCD_DisplayStringLine(Line6, ptr);
        temp = VBS_GetAvBusVoltage_V(oVBS);
        Display_3DigitUnsignedNumber(Line6, CHAR_11, temp); 
        DisplayChar(Line6, CHAR_14, ' ');
        DisplayChar(Line6, CHAR_15, 'V');             
      }          
      LCD_ClearLine(Line8);
      LCD_ClearLine(Line9);
    }
    break;
    
  case VISUALIZATION_8:  
    if (bPresent_Visualization != bPrevious_Visualization)
    {  
      LCD_ClearLine(Line2);
      
      ptr = " Motor is stopping  ";
      LCD_DisplayStringLine(Line3,ptr);
      
      ptr = "   please wait...   ";
      LCD_DisplayStringLine(Line4,ptr);
      
      LCD_ClearLine(Line5);
      LCD_ClearLine(Line6);
      LCD_ClearLine(Line7);
      LCD_ClearLine(Line8);
      LCD_ClearLine(Line9);
    } 
    break;
    
#ifdef OBSERVER_GAIN_TUNING      
  case VISUALIZATION_9 :
    if (bPresent_Visualization != bPrevious_Visualization)
    {           
      ptr = "   Observer Gains   ";
      LCD_DisplayStringLine(Line2,ptr);
      
      ptr = "     K1       K2    ";
      LCD_DisplayStringLine(Line3,ptr); 
      
      LCD_ClearLine(Line4);
      
      ptr = "      PLL Gains     ";
      LCD_DisplayStringLine(Line5,ptr); 
      
      ptr = "     P        I     ";
      LCD_DisplayStringLine(Line6,ptr);
      
      LCD_ClearLine(Line7);
      
      LCD_ClearLine(Line8);
      
      ptr = " <> Move  ^| Change ";          
      LCD_DisplayStringLine(Line9, ptr); 
    }
    
    switch(bMenu_index)
    {
    case K1_MENU:
      LCD_SetTextColor(Red);
      temp = UI_GetReg(this, MC_PROTOCOL_REG_OBSERVER_C1);
      Display_5DigitSignedNumber(Line4, CHAR_3, temp);
      
      LCD_SetTextColor(Blue);
      temp = UI_GetReg(this, MC_PROTOCOL_REG_OBSERVER_C2);
      Display_5DigitSignedNumber(Line4, CHAR_12, temp);
      
      temp = UI_GetReg(this, MC_PROTOCOL_REG_PLL_KP);
      Display_5DigitSignedNumber(Line7, CHAR_3, temp);  
      
      temp = UI_GetReg(this, MC_PROTOCOL_REG_PLL_KI);
      Display_5DigitSignedNumber(Line7, CHAR_12, temp);
      break;
      
    case K2_MENU:              
      temp = UI_GetReg(this, MC_PROTOCOL_REG_OBSERVER_C1);
      Display_5DigitSignedNumber(Line4, CHAR_3, temp);
      
      LCD_SetTextColor(Red);  
      temp = UI_GetReg(this, MC_PROTOCOL_REG_OBSERVER_C2);
      Display_5DigitSignedNumber(Line4, CHAR_12, temp);
      
      LCD_SetTextColor(Blue);           
      temp = UI_GetReg(this, MC_PROTOCOL_REG_PLL_KP);
      Display_5DigitSignedNumber(Line7, CHAR_3, temp);  
      
      temp = UI_GetReg(this, MC_PROTOCOL_REG_PLL_KI);
      Display_5DigitSignedNumber(Line7, CHAR_12, temp);
      break;
      
    case P_PLL_MENU:
      temp = UI_GetReg(this, MC_PROTOCOL_REG_OBSERVER_C1);
      Display_5DigitSignedNumber(Line4, CHAR_3, temp);
      
      temp = UI_GetReg(this, MC_PROTOCOL_REG_OBSERVER_C2);
      Display_5DigitSignedNumber(Line4, CHAR_12, temp);
      
      LCD_SetTextColor(Red);           
      temp = UI_GetReg(this, MC_PROTOCOL_REG_PLL_KP);
      Display_5DigitSignedNumber(Line7, CHAR_3, temp);  
      
      LCD_SetTextColor(Blue); 
      temp = UI_GetReg(this, MC_PROTOCOL_REG_PLL_KI);
      Display_5DigitSignedNumber(Line7, CHAR_12, temp);
      break;
      
    case I_PLL_MENU :
      temp = UI_GetReg(this, MC_PROTOCOL_REG_OBSERVER_C1);
      Display_5DigitSignedNumber(Line4, CHAR_3, temp);
      
      temp = UI_GetReg(this, MC_PROTOCOL_REG_OBSERVER_C2);
      Display_5DigitSignedNumber(Line4, CHAR_12, temp);
      
      temp = UI_GetReg(this, MC_PROTOCOL_REG_PLL_KP);
      Display_5DigitSignedNumber(Line7, CHAR_3, temp);  
      
      LCD_SetTextColor(Red);
      temp = UI_GetReg(this, MC_PROTOCOL_REG_PLL_KI);
      Display_5DigitSignedNumber(Line7, CHAR_12, temp);                  
      LCD_SetTextColor(Blue); 
      break;  
    default:
      break;
    }
    break;
#endif
    
#if (DAC_FUNCTIONALITY == ENABLE)      
  case VISUALIZATION_10:
    if (bPresent_Visualization != bPrevious_Visualization)
    {           
      LCD_ClearLine(Line2);
      
      ptr = "    Signal on PB0   ";
      LCD_DisplayStringLine(Line3,ptr); 
      
      LCD_ClearLine(Line4);
      
      LCD_ClearLine(Line5);
      
      ptr = "    Signal on PB1   ";
      LCD_DisplayStringLine(Line6,ptr);
      
      LCD_ClearLine(Line7);
      
      LCD_ClearLine(Line8);
      
      ptr = " <> Move  ^| Change ";          
      LCD_DisplayStringLine(Line9, ptr); 
    }
    
    switch(bMenu_index)
    {
    case DAC_PB0_MENU:
      LCD_SetTextColor(Red);
      ptr = (uint8_t*)(GUI_DAC_ChTxt[pDVars->bDAC_CH0_Sel]);
      LCD_DisplayStringLine(Line4, ptr);
      
      LCD_SetTextColor(Blue);
      ptr = (uint8_t*)(GUI_DAC_ChTxt[pDVars->bDAC_CH1_Sel]);
      LCD_DisplayStringLine(Line7, ptr);
      break;
      
    case DAC_PB1_MENU:
      ptr = (uint8_t*)(GUI_DAC_ChTxt[pDVars->bDAC_CH0_Sel]);
      LCD_DisplayStringLine(Line4, ptr);
      
      LCD_SetTextColor(Red);
      ptr = (uint8_t*)(GUI_DAC_ChTxt[pDVars->bDAC_CH1_Sel]);
      LCD_DisplayStringLine(Line7, ptr);
      LCD_SetTextColor(Blue);
      break;
      
    default:
      break;
    }
    break;      
#endif
  default:
    break;      
  }
}
          
/*******************************************************************************
* Function Name  : Display_5DigitSignedNumber
* Description    : It Displays a 5 digit signed number in the specified line, 
*                  starting from a specified element of LCD display matrix 
* Input          : Line, starting point in LCD dysplay matrix, 5 digit signed
*                  number 
* Output         : None
* Return         : None
*******************************************************************************/
static void Display_5DigitSignedNumber(uint8_t Line, uint8_t bFirstchar, int16_t number)
{ 
  uint32_t i;
  uint16_t h_aux=1;

  if (number<0)     
  {
    DisplayChar(Line,(uint16_t)(bFirstchar), '-');
    number = -number;
  }
  else 
  {
    DisplayChar(Line,(uint16_t)(bFirstchar), ' ');
  }
      
  for (i=0; i<4; i++)
  {
    DisplayChar(Line, (uint16_t)(bFirstchar+5-i),
                                        (uint8_t)(((number%(10*h_aux))/h_aux)+0x30));          
    h_aux *= 10;
  }
  DisplayChar(Line,(uint16_t)(bFirstchar+1), (uint8_t)(((number/10000))+0x30));
}

/*******************************************************************************
* Function Name  : Display_3DigitUnsignedNumber
* Description    : It Displays a 3 digit unsigned number in the specified line, 
*                  starting from a specified element of LCD display matrix 
* Input          : Line, starting point in LCD dysplay matrix, 3 digit unsigned
*                  number 
* Output         : None
* Return         : None
*******************************************************************************/
static void Display_3DigitUnsignedNumber(uint8_t Line, uint8_t bFirstchar, int16_t number)
{
  DisplayChar(Line, (bFirstchar), (uint8_t)(((number%1000)/100)+0x30));
  DisplayChar(Line, (bFirstchar+1), (uint8_t)(((number%100)/10)+0x30));
  DisplayChar(Line, (bFirstchar+2), (uint8_t)((number%10)+0x30));
}

/*******************************************************************************
* Function Name  : Display_1DigitUnsignedNumber
* Description    : It Displays a 1 digit unsigned number in the specified line, 
*                  starting from a specified element of LCD display matrix 
* Input          : Line, starting point in LCD dysplay matrix, 1 digit unsigned
*                  number 
* Output         : None
* Return         : None
*******************************************************************************/
static void Display_1DigitUnsignedNumber(uint8_t Line, uint8_t bFirstchar, uint8_t number)
{
  DisplayChar(Line, bFirstchar, (uint8_t)((number%10)+0x30));
}

static void Display_5dot_line(uint16_t Line, uint16_t Column)
{
  uint8_t i;
  for(i = 0; i < 5; i++)
  {
    DisplayChar(Line, (uint16_t)(Column-i), '-');
  }
}

/*******************************************************************************
* Function Name  : DisplayChar
* Description    : Wrapper function for the Std.lib.  LCD_DisplayChar
* Input          : Line, Column, Ascii
*                  number 
* Output         : None
* Return         : None
*******************************************************************************/
static void DisplayChar(uint16_t Line, uint16_t Column, uint8_t Ascii)
{
  uint16_t xpos = 16*Column;
  if (LCD_GetXAxesDirection() == LCD_X_AXES_INVERTED)
  {
    xpos = 320-xpos;
  }
  LCD_DisplayChar(Line, xpos, Ascii);
}


/*******************************************************************************
* Function Name  : ComputeVisualization
* Description    : Starting from the value of the bMenuIndex, this function 
*                  extract the information about the present menu to be 
*                  displayed on LCD
* Input          : bMenuIndex variable, State 
* Output         : Present visualization
* Return         : None
*******************************************************************************/

uint8_t ComputeVisualization(uint8_t bLocal_MenuIndex, State_t State)
{  
  uint8_t bTemp;
  
  switch(bLocal_MenuIndex)
  {
  case CONTROL_MODE_SPEED_MENU:
    bTemp = VISUALIZATION_1;
    break;
  case REF_SPEED_MENU:
    bTemp = VISUALIZATION_1;
    break;
  case MOTOR_SPD_MENU:
    bTemp = VISUALIZATION_1;
    break;
    
  case P_SPEED_MENU:
    bTemp = VISUALIZATION_2; 
    break;
  case I_SPEED_MENU :
    bTemp = VISUALIZATION_2; 
    break;
#ifdef DIFFERENTIAL_TERM_ENABLED
  case D_SPEED_MENU:
    bTemp = VISUALIZATION_2; 
    break;
#endif        
    
  case P_TORQUE_MENU:
    bTemp = VISUALIZATION_3; 
    break; 
  case I_TORQUE_MENU :
    bTemp = VISUALIZATION_3; 
    break; 
#ifdef DIFFERENTIAL_TERM_ENABLED
  case D_TORQUE_MENU :
    bTemp = VISUALIZATION_3; 
    break; 
#endif        
    
  case P_FLUX_MENU :
    bTemp = VISUALIZATION_4; 
    break; 
  case I_FLUX_MENU :
    bTemp = VISUALIZATION_4; 
    break; 
#ifdef DIFFERENTIAL_TERM_ENABLED
  case D_FLUX_MENU :
    bTemp = VISUALIZATION_4; 
    break; 
#endif
    
#ifdef FLUX_WEAKENING
  case P_VOLT_MENU :
    bTemp = VISUALIZATION_11; 
    break;
  case I_VOLT_MENU:
    bTemp = VISUALIZATION_11;
  case TARGET_VOLT_MENU :
    bTemp = VISUALIZATION_11;        
    break;
#endif      
    
  case POWER_STAGE_MENU :
    bTemp = VISUALIZATION_5;
    break;
    
  case CONTROL_MODE_TORQUE_MENU:
    bTemp = VISUALIZATION_6;
    break;
  case IQ_REF_MENU :
    bTemp = VISUALIZATION_6;
    break;
  case ID_REF_MENU:
    bTemp = VISUALIZATION_6;
    break;
  case MOTOR_TRQ_MENU:
    bTemp = VISUALIZATION_6;
    break;
    
#ifdef OBSERVER_GAIN_TUNING  
  case K1_MENU:
    bTemp = VISUALIZATION_9;
    break;   
  case K2_MENU:
    bTemp = VISUALIZATION_9;
    break;
  case P_PLL_MENU:
    bTemp = VISUALIZATION_9;
    break;
  case I_PLL_MENU:
    bTemp = VISUALIZATION_9;
    break;
#endif
    
#if (DAC_FUNCTIONALITY == ENABLE)
  case DAC_PB0_MENU:
    bTemp = VISUALIZATION_10;
    break;   
  case DAC_PB1_MENU:
    bTemp = VISUALIZATION_10;
    break;
#endif             
  default:
    bTemp = VISUALIZATION_1;
    break;      
  }    
  
  if ((State == FAULT_NOW) || (State == FAULT_OVER))
  {
    bTemp = VISUALIZATION_7;
  }
  else if (State == STOP)
  {
    bTemp = VISUALIZATION_8;
  }  
  
  return (bTemp);
}

#if (defined(USE_STM3210C_EVAL) || defined(USE_STM322xG_EVAL) || defined(USE_STM324xG_EVAL))
static bool JOY_Pressed(InputKey inKey)
{
  bool retVal = FALSE;
  uint8_t Button = JOY_SEL;
	JOYState_TypeDef joystate;
  switch (inKey)
  {
  case KEY_JOYSTICK_UP:
    Button = JOY_UP;
    break;
  case KEY_JOYSTICK_DOWN:
    Button = JOY_DOWN;
    break;
  case KEY_JOYSTICK_LEFT:
    Button = JOY_LEFT;
    break;
  case KEY_JOYSTICK_RIGHT:
    Button = JOY_RIGHT;
    break;
  case KEY_JOYSTICK_SEL:
    Button = JOY_SEL;
    break;
  }
  joystate = IOE_JoyStickGetState();
  if (joystate == Button)
  {
    retVal = TRUE;
  }
  else
  {
    retVal = FALSE;
  }
  return (retVal);
}
#else
static bool JOY_Pressed(InputKey inKey)
{
  bool retVal = FALSE;
  Button_TypeDef Button = BUTTON_SEL;
  switch (inKey)
  {
  case KEY_JOYSTICK_UP:
    Button = BUTTON_UP;            
    break;
  case KEY_JOYSTICK_DOWN:
    Button = BUTTON_DOWN;      
    break;
  case KEY_JOYSTICK_LEFT:
    Button = BUTTON_LEFT;      
    break;
  case KEY_JOYSTICK_RIGHT:
    Button = BUTTON_RIGHT;      
    break;
  case KEY_JOYSTICK_SEL:
    Button = BUTTON_SEL;          
    break;
  }
  if (STM_EVAL_PBGetState(Button) == JOYSTIK_ACTIVE)
  {
    retVal = TRUE;
  }
  else
  {
    retVal = FALSE;
  }
  return (retVal);
}
#endif

/*******************************************************************************
* Function Name  : KEYS_Read
* Description    : Reads key from demoboard.
* Input          : None
* Output         : None
* Return         : Return RIGHT, LEFT, SEL, UP, DOWN, KEY_HOLD or NOKEY
*******************************************************************************/
uint8_t KEYS_Read ( void )
{
  /* "RIGHT" key is pressed */
  if (JOY_Pressed(KEY_JOYSTICK_RIGHT))
  {
    if (bPrevious_key == RIGHT) 
    {
      return KEY_HOLD;
    }
    else
    {
      bPrevious_key = RIGHT;
      return RIGHT;
    }
  }
  /* "LEFT" key is pressed */
  else if (JOY_Pressed(KEY_JOYSTICK_LEFT))
  {
    if (bPrevious_key == LEFT) 
    {
      return KEY_HOLD;
    }
    else
    {
      bPrevious_key = LEFT;
      return LEFT;
    }
  }
  /* "SEL" key is pressed */
  if (JOY_Pressed(KEY_JOYSTICK_SEL))
  {
    if (bPrevious_key == SEL) 
    {
      return KEY_HOLD;
    }
    else
    {
      if ( (TB_DebounceDelay_IsElapsed() == FALSE) && (bKey_Flag & SEL_FLAG == SEL_FLAG) )
      {
        return NOKEY;
      }
      else
      {
      if ( (TB_DebounceDelay_IsElapsed() == TRUE) && ( (bKey_Flag & SEL_FLAG) == 0) ) 
      {
        bKey_Flag |= SEL_FLAG;
        TB_Set_DebounceDelay_500us(100); // 50 ms debounce
      }
      else if ( (TB_DebounceDelay_IsElapsed() == TRUE) && ((bKey_Flag & SEL_FLAG) == SEL_FLAG) )
      {
        bKey_Flag &= (uint8_t)(~SEL_FLAG);
        bPrevious_key = SEL;
        return SEL;
      }
      return NOKEY;
      }
    }
  }
  /* "SEL" key is pressed */
  else if(!GPIO_ReadInputDataBit(USER_BUTTON_PORT, USER_BUTTON_BIT))
  {
    if (bPrevious_key == SEL) 
    {
      return KEY_HOLD;
    }
    else
    {
      if ( (TB_DebounceDelay_IsElapsed() == FALSE) && (bKey_Flag & SEL_FLAG == SEL_FLAG) )
      {
        return NOKEY;
      }
      else
      {
      if ( (TB_DebounceDelay_IsElapsed() == TRUE) && ( (bKey_Flag & SEL_FLAG) == 0) ) 
      {
        bKey_Flag |= SEL_FLAG;
        TB_Set_DebounceDelay_500us(100); // 50 ms debounce
      }
      else if ( (TB_DebounceDelay_IsElapsed() == TRUE) && ((bKey_Flag & SEL_FLAG) == SEL_FLAG) )
      {
        bKey_Flag &= (uint8_t)(~SEL_FLAG);
        bPrevious_key = SEL;
        return SEL;
      }
      return NOKEY;
      }
    }
  }
   /* "UP" key is pressed */
  else if (JOY_Pressed(KEY_JOYSTICK_UP))
  {
    if (bPrevious_key == UP) 
    {
      return KEY_HOLD;
    }
    else
    {
      bPrevious_key = UP;
      return UP;
    }
  }
  /* "DOWN" key is pressed */
  else if (JOY_Pressed(KEY_JOYSTICK_DOWN))
  {
    if (bPrevious_key == DOWN) 
    {
      return KEY_HOLD;
    }
    else
    {
      bPrevious_key = DOWN;
      return DOWN;
    }
  }
  
  /* No key is pressed */
  else
  {
    bPrevious_key = NOKEY;
    return NOKEY;
  }
}



/*******************************************************************************
* Function Name  : KEYS_process
* Description    : Process key 
* Input          : Key code
* Output         : None
* Return         : None
*******************************************************************************/
static void KEYS_process(CUI this)
{
  pVars_t pVars = CLASS_VARS;
  pDVars_t pDVars = DCLASS_VARS;
  CMCT oMCT = pVars->pMCT[pVars->bSelectedDrive];
  CMCI oMCI = pVars->pMCI[pVars->bSelectedDrive];
  CPI oSpeedLoopPID = MCT_GetSpeedLoopPID(oMCT);
  CPI oIqLoopPID = MCT_GetIqLoopPID(oMCT);
  CPI oIdLoopPID = MCT_GetIdLoopPID(oMCT);
  CPI oFluxWeakeningLoopPID = MCT_GetFluxWeakeningLoopPID(oMCT);
  CFW oFWCtrl = MCT_GetFluxWeakeningCtrl(oMCT);
  State_t State = MCI_GetSTMState(oMCI);
  CSTC oSTC = MCT_GetSpeednTorqueController(oMCT);
  
  bKey = KEYS_Read();    // read key pushed (if any...)
  
  if (bKey == SEL)
  {
    /* Queries the STM and send start, stop or fault ack depending on the state. */
    switch (MCI_GetSTMState(oMCI))
    {
    case FAULT_OVER:
      {
        MCI_FaultAcknowledged(oMCI);
      }
      break;
    case IDLE:
      {
        MCI_StartMotor(oMCI);
      }
      break;
    default:
      {
        MCI_StopMotor(oMCI);
      }
      break;
    }
  }

  
  if (bPresent_Visualization == VISUALIZATION_7)
  {
    /* Fault condition */
#ifdef DUALDRIVE
    switch(bKey)
    {
    case UP:
    case DOWN:
      {
        uint8_t bSelDrv = pVars->bSelectedDrive;
        if (bSelDrv == 0)
        {
          pVars->bSelectedDrive = 1;
          bPresent_Visualization--; /* Forces display update */
        }
        else
        {
          pVars->bSelectedDrive = 0;
          bPresent_Visualization--; /* Forces display update */
        }
      }
      break;  
    default:
      break;
    }
#endif
  }
  else
  {
    /* Manage Joy input if no fault */
    switch (bMenu_index)
    { 
    case MOTOR_SPD_MENU:
    case MOTOR_TRQ_MENU:
      switch(bKey)
      {
      case UP:
      case DOWN:
#ifdef DUALDRIVE
        {
          uint8_t bSelDrv = pVars->bSelectedDrive;
          if (bSelDrv == 0)
          {
            pVars->bSelectedDrive = 1;
          }
          else
          {
            pVars->bSelectedDrive = 0;
          }
        }
#endif
        break;
        
      case RIGHT:
        if (MCI_GetControlMode(oMCI) == STC_SPEED_MODE)
        {
          bMenu_index = CONTROL_MODE_SPEED_MENU;
        }
        else
        {
          bMenu_index = CONTROL_MODE_TORQUE_MENU;
        }
        break;
        
      case LEFT:
#if (DAC_FUNCTIONALITY == ENABLE)              
        bMenu_index = DAC_PB1_MENU;
#elif defined OBSERVER_GAIN_TUNING
        bMenu_index = I_PLL_MENU;          
#else              
        bMenu_index = POWER_STAGE_MENU;
#endif              
        break;
        
      default:
        break;
      }
      break;
      
    case CONTROL_MODE_SPEED_MENU:
      switch(bKey)
      {
      case UP:
      case DOWN:
        if (UI_GetReg(this, MC_PROTOCOL_REG_CONTROL_MODE) == STC_SPEED_MODE)
        {
          UI_SetReg(this, MC_PROTOCOL_REG_CONTROL_MODE, STC_TORQUE_MODE);
        }
        else
        {
          UI_SetReg(this, MC_PROTOCOL_REG_CONTROL_MODE, STC_SPEED_MODE);
        }
        break;
        
      case RIGHT:
        bMenu_index = REF_SPEED_MENU;
        break;
        
      case LEFT:
        bMenu_index = MOTOR_SPD_MENU;
        break;
        
      default:
        break;
      }
      break;
      
    case REF_SPEED_MENU:
      switch(bKey)
      {
      case UP:
        if (MCI_GetLastRampFinalSpeed(oMCI) <= (MAX_APPLICATION_SPEED / 6))
        {
          MCI_ExecSpeedRamp(oMCI, MCI_GetLastRampFinalSpeed(oMCI) + SPEED_INC_DEC, SPEED_INC_DEC_DURATION);
        }
        break;
        
      case DOWN:
        if (MCI_GetLastRampFinalSpeed(oMCI) >= - (MAX_APPLICATION_SPEED / 6))
        {
          MCI_ExecSpeedRamp(oMCI, MCI_GetLastRampFinalSpeed(oMCI) - SPEED_INC_DEC, SPEED_INC_DEC_DURATION);
        }
        break;
        
      case RIGHT:
        bMenu_index = P_SPEED_MENU;
        break;
        
      case LEFT:
        bMenu_index = CONTROL_MODE_SPEED_MENU;         
        break;
        
      default:
        break;
      }
      break;          
      
    case CONTROL_MODE_TORQUE_MENU:
      switch(bKey)
      {
      case UP:
      case DOWN:
        if (UI_GetReg(this, MC_PROTOCOL_REG_CONTROL_MODE) == STC_SPEED_MODE)
        {
          UI_SetReg(this, MC_PROTOCOL_REG_CONTROL_MODE, STC_TORQUE_MODE);
        }
        else
        {
          UI_SetReg(this, MC_PROTOCOL_REG_CONTROL_MODE, STC_SPEED_MODE);
        }
        break;
        
      case RIGHT:
        bMenu_index = IQ_REF_MENU;
        break;
        
      case LEFT:
        bMenu_index = MOTOR_TRQ_MENU;
        break;
        
      default:
        break;
      }
      break;  
      
    case IQ_REF_MENU:
      switch(bKey)
      {
      case UP:
        if (MCI_GetIqdref(oMCI).qI_Component1 <= NOMINAL_CURRENT - TORQUE_INC_DEC)
        {
          pDVars->Iqdref.qI_Component1 += TORQUE_INC_DEC;
          MCI_SetCurrentReferences(oMCI,pDVars->Iqdref);
        }
        break;
        
      case DOWN:
        if (MCI_GetIqdref(oMCI).qI_Component1 >= -NOMINAL_CURRENT + TORQUE_INC_DEC)
        {
          pDVars->Iqdref.qI_Component1 -= TORQUE_INC_DEC;
          MCI_SetCurrentReferences(oMCI,pDVars->Iqdref);
        }
        break;
        
      case RIGHT:
        bMenu_index = ID_REF_MENU;
        break;
        
      case LEFT:
        if(State == IDLE)
        {
          bMenu_index = CONTROL_MODE_TORQUE_MENU;
        }
        else
        {
#if (DAC_FUNCTIONALITY == ENABLE)              
          bMenu_index = DAC_PB1_MENU;
#elif defined OBSERVER_GAIN_TUNING
          bMenu_index = I_PLL_MENU;          
#else              
          bMenu_index = POWER_STAGE_MENU;
#endif 
        }
        break;
        
      default:
        break;
      }
      break;    
      
    case ID_REF_MENU:
      switch(bKey)
      {
      case UP:
        if (MCI_GetIqdref(oMCI).qI_Component2 <= NOMINAL_CURRENT - FLUX_INC_DEC)
        {
          pDVars->Iqdref.qI_Component2 += FLUX_INC_DEC;
          MCI_SetCurrentReferences(oMCI,pDVars->Iqdref);
        }
        break;
        
      case DOWN:
        if (MCI_GetIqdref(oMCI).qI_Component2 >= FLUX_INC_DEC - NOMINAL_CURRENT)
        {
          pDVars->Iqdref.qI_Component2 -= FLUX_INC_DEC;
          MCI_SetCurrentReferences(oMCI,pDVars->Iqdref);
        }
        break;
        
      case RIGHT:
        bMenu_index = P_TORQUE_MENU;
        break;
        
      case LEFT:
        bMenu_index = IQ_REF_MENU;
        break;
        
      default:
        break;
      }
      break;
      
    case P_SPEED_MENU:    
      switch(bKey)
      {
      case UP:
        if (PI_GetKP(oSpeedLoopPID) <= S16_MAX-KP_GAIN_INC_DEC)
        {
          PI_SetKP(oSpeedLoopPID,PI_GetKP(oSpeedLoopPID) + KP_GAIN_INC_DEC);
        }
        break;
        
      case DOWN:
        if (PI_GetKP(oSpeedLoopPID) >= KP_GAIN_INC_DEC)
        {
          PI_SetKP(oSpeedLoopPID,PI_GetKP(oSpeedLoopPID) - KP_GAIN_INC_DEC);
        }
        break;
        
      case RIGHT:
        bMenu_index = I_SPEED_MENU;
        break;
        
      case LEFT:
        bMenu_index = REF_SPEED_MENU;             
        break;
        
      default:
        break;
      }
      break;
      
    case I_SPEED_MENU:    
      switch(bKey)
      {
      case UP:
        if (PI_GetKI(oSpeedLoopPID) <= S16_MAX-KI_GAIN_INC_DEC)
        {
          PI_SetKI(oSpeedLoopPID,PI_GetKI(oSpeedLoopPID) + KI_GAIN_INC_DEC);
        }
        break;
        
      case DOWN:
        if (PI_GetKI(oSpeedLoopPID) >= KI_GAIN_INC_DEC)
        {
          PI_SetKI(oSpeedLoopPID,PI_GetKI(oSpeedLoopPID) - KI_GAIN_INC_DEC);
        }
        break;
        
      case RIGHT:
#ifdef DIFFERENTIAL_TERM_ENABLED                 
        bMenu_index = D_SPEED_MENU;
#else
        bMenu_index = P_TORQUE_MENU;
#endif                
        break;
        
      case LEFT:
        bMenu_index = P_SPEED_MENU;
        break;
        
      default:
        break;
      }
      break;        
      
#ifdef DIFFERENTIAL_TERM_ENABLED       
    case D_SPEED_MENU:    
      switch(bKey)
      {
      case UP:
        if (PID_GetKD((CPID_PI)oSpeedLoopPID) <= S16_MAX-KD_GAIN_INC_DEC)
        {
          PID_SetKD((CPID_PI)oSpeedLoopPID,PID_GetKD((CPID_PI)oSpeedLoopPID) + KD_GAIN_INC_DEC);
        }
        break;
        
      case DOWN:
        if (PID_GetKD((CPID_PI)oSpeedLoopPID) >= KD_GAIN_INC_DEC)
        {
          PID_SetKD((CPID_PI)oSpeedLoopPID,PID_GetKD((CPID_PI)oSpeedLoopPID) - KD_GAIN_INC_DEC);
        }
        break;
        
      case RIGHT:                
        bMenu_index = P_TORQUE_MENU;               
        break;
        
      case LEFT:
        bMenu_index = I_SPEED_MENU;
        break;
        
      default:
        break;
      }
      break;   
#endif
      
    case P_TORQUE_MENU:    
      switch(bKey)
      {
      case UP:
        if (PI_GetKP(oIqLoopPID) <= S16_MAX - KP_GAIN_INC_DEC)
        {
          PI_SetKP(oIqLoopPID,PI_GetKP(oIqLoopPID) + KP_GAIN_INC_DEC);
        }
        break;
        
      case DOWN:
        if (PI_GetKP(oIqLoopPID) >= KP_GAIN_INC_DEC)
        {
          PI_SetKP(oIqLoopPID,PI_GetKP(oIqLoopPID) - KP_GAIN_INC_DEC);
        }
        break;
        
      case RIGHT:
        bMenu_index = I_TORQUE_MENU;
        break;
        
      case LEFT:         
        if (MCI_GetControlMode(oMCI) == STC_SPEED_MODE)
        {
#ifdef DIFFERENTIAL_TERM_ENABLED              
          bMenu_index = D_SPEED_MENU;
#else
          bMenu_index = I_SPEED_MENU; 
#endif                
        }
        else
        {
          bMenu_index = ID_REF_MENU;
        }              
        break;
        
      default:
        break;
      }
      break;
      
    case I_TORQUE_MENU:    
      switch(bKey)
      {
      case UP:
        if (PI_GetKI(oIqLoopPID) <= S16_MAX - KI_GAIN_INC_DEC)
        {
          PI_SetKI(oIqLoopPID,PI_GetKI(oIqLoopPID) + KI_GAIN_INC_DEC);
        }
        break;
        
      case DOWN:
        if (PI_GetKI(oIqLoopPID) >= KI_GAIN_INC_DEC)
        {
          PI_SetKI(oIqLoopPID,PI_GetKI(oIqLoopPID) - KI_GAIN_INC_DEC);
        }
        break;
        
      case RIGHT:
#ifdef DIFFERENTIAL_TERM_ENABLED                 
        bMenu_index = D_TORQUE_MENU;
#else
        bMenu_index = P_FLUX_MENU;
#endif                
        break;
        
      case LEFT:
        bMenu_index = P_TORQUE_MENU;
        break;
        
      default:
        break;
      }
      break;
      
#ifdef DIFFERENTIAL_TERM_ENABLED       
    case D_TORQUE_MENU:    
      switch(bKey)
      {
      case UP:
        if (PID_GetKD((CPID_PI)oIqLoopPID) <= S16_MAX - KD_GAIN_INC_DEC)
        {
          PID_SetKD((CPID_PI)oIqLoopPID,PID_GetKD((CPID_PI)oIqLoopPID) + KD_GAIN_INC_DEC);
        }
        break;
        
      case DOWN:
        if (PID_GetKD((CPID_PI)oIqLoopPID) >= KD_GAIN_INC_DEC)
        {
          PID_SetKD((CPID_PI)oIqLoopPID,PID_GetKD((CPID_PI)oIqLoopPID) + KD_GAIN_INC_DEC);
        }
        break;
        
      case RIGHT:                
        bMenu_index = P_FLUX_MENU;               
        break;
        
      case LEFT:
        bMenu_index = I_TORQUE_MENU;
        break;
        
      default:
        break;
      }
      break;   
#endif 
    case P_FLUX_MENU:    
      switch(bKey)
      {
      case UP:
        if (PI_GetKP(oIdLoopPID) <= S16_MAX-KP_GAIN_INC_DEC)
        {
          PI_SetKP(oIdLoopPID,PI_GetKP(oIdLoopPID) + KP_GAIN_INC_DEC);
        }
        break;
        
      case DOWN:
        if (PI_GetKP(oIdLoopPID) >= KP_GAIN_INC_DEC)
        {
          PI_SetKP(oIdLoopPID,PI_GetKP(oIdLoopPID) - KP_GAIN_INC_DEC);
        }
        break;
        
      case RIGHT:
        bMenu_index = I_FLUX_MENU;
        break;
        
      case LEFT:         
#ifdef  DIFFERENTIAL_TERM_ENABLED
        bMenu_index = D_TORQUE_MENU;
#else
        bMenu_index = I_TORQUE_MENU;
#endif              
        break;
        
      default:
        break;
      }
      break;
      
    case I_FLUX_MENU:
      switch(bKey)
      {
      case UP:
        if (PI_GetKI(oIdLoopPID) <= S16_MAX-KI_GAIN_INC_DEC)
        {
          PI_SetKI(oIdLoopPID,PI_GetKI(oIdLoopPID) + KI_GAIN_INC_DEC);
        }
        break;
        
      case DOWN:
        if (PI_GetKI(oIdLoopPID) >= KI_GAIN_INC_DEC)
        {
          PI_SetKI(oIdLoopPID,PI_GetKI(oIdLoopPID) - KI_GAIN_INC_DEC);
        }
        break;
        
      case RIGHT:
#ifdef DIFFERENTIAL_TERM_ENABLED                 
        bMenu_index = D_FLUX_MENU;
#elif defined FLUX_WEAKENING
        bMenu_index = P_VOLT_MENU;
#else                
        bMenu_index = POWER_STAGE_MENU;
#endif                
        break;
        
      case LEFT:
        bMenu_index = P_FLUX_MENU;
        break;
        
      default:
        break;
      }
      break;
      
#ifdef DIFFERENTIAL_TERM_ENABLED       
    case D_FLUX_MENU:
      switch(bKey)
      {
      case UP:
        if (PID_GetKD((CPID_PI)oIdLoopPID) <= S16_MAX - KD_GAIN_INC_DEC)
        {
          PID_SetKD((CPID_PI)oIdLoopPID,PID_GetKD((CPID_PI)oIdLoopPID) + KD_GAIN_INC_DEC);
        }
        break;
        
      case DOWN:
        if (PID_GetKD((CPID_PI)oIdLoopPID) >= KD_GAIN_INC_DEC)
        {
          PID_SetKD((CPID_PI)oIdLoopPID,PID_GetKD((CPID_PI)oIdLoopPID) - KD_GAIN_INC_DEC);
        }
        break;
        
      case RIGHT:
#ifdef FLUX_WEAKENING
        bMenu_index = P_VOLT_MENU;
#else
        bMenu_index = POWER_STAGE_MENU;
#endif                
        break;
        
      case LEFT:              
        bMenu_index = I_FLUX_MENU;
        break;
        
      default:
        break;
      }
      break;   
#endif
      
#ifdef FLUX_WEAKENING        
    case P_VOLT_MENU:
      switch(bKey)
      {
      case UP:
        if (PI_GetKP(oFluxWeakeningLoopPID) <= 32500)
        {
          PI_SetKP(oFluxWeakeningLoopPID,PI_GetKP(oFluxWeakeningLoopPID) + KP_VOLT_INC_DEC);
        }
        break;
        
      case DOWN:
        if (PI_GetKP(oFluxWeakeningLoopPID) >= KP_VOLT_INC_DEC)
        {
          PI_SetKP(oFluxWeakeningLoopPID,PI_GetKP(oFluxWeakeningLoopPID) - KP_VOLT_INC_DEC);
        }
        break;
        
      case RIGHT:
        bMenu_index = I_VOLT_MENU;
        break;
        
      case LEFT:
#ifdef DIFFERENTIAL_TERM_ENABLED            
        bMenu_index = D_FLUX_MENU;
#else              
        bMenu_index = I_FLUX_MENU;
#endif              
        break;
        
      default:
        break;
      }
      break;
      
    case I_VOLT_MENU:
      switch(bKey)
      {
      case UP:
        if (PI_GetKI(oFluxWeakeningLoopPID) <= 32500)
        {
          PI_SetKI(oFluxWeakeningLoopPID,PI_GetKI(oFluxWeakeningLoopPID) + KI_VOLT_INC_DEC);
        }
        break;
        
      case DOWN:
        if (PI_GetKI(oFluxWeakeningLoopPID) >= KI_VOLT_INC_DEC)
        {
          PI_SetKI(oFluxWeakeningLoopPID,PI_GetKI(oFluxWeakeningLoopPID) - KI_VOLT_INC_DEC);
        }
        break;
        
      case RIGHT:
        bMenu_index = TARGET_VOLT_MENU;
        break;
        
      case LEFT:
        bMenu_index = P_VOLT_MENU;
        break;
        
      default:
        break;
      }
      break;
      
    case TARGET_VOLT_MENU:
      switch(bKey)
      {
      case UP:
        if (FW_GetVref(oFWCtrl) <= (1000-VOLT_LIM_INC_DEC))
        {
          FW_SetVref(oFWCtrl, FW_GetVref(oFWCtrl) + VOLT_LIM_INC_DEC);
        }
        break;
        
      case DOWN:
        if (FW_GetVref(oFWCtrl) >= VOLT_LIM_INC_DEC)
        {
          FW_SetVref(oFWCtrl, FW_GetVref(oFWCtrl) - VOLT_LIM_INC_DEC);
        }
        break;
        
      case RIGHT:
        bMenu_index = POWER_STAGE_MENU;
        break;
        
      case LEFT:
        bMenu_index = I_VOLT_MENU;
        break;
        
      default:
        break;
      }
      break;        
      
#endif        
      
    case POWER_STAGE_MENU:
      switch(bKey)
      {
      case RIGHT:  
#ifdef OBSERVER_GAIN_TUNING
        bMenu_index = K1_MENU;
#elif (DAC_FUNCTIONALITY == ENABLE)
        bMenu_index = DAC_PB0_MENU;
#else              
        if (MCI_GetControlMode(oMCI) == STC_SPEED_MODE)
        {
          bMenu_index = MOTOR_SPD_MENU;
        }
        else
        {
          bMenu_index = MOTOR_TRQ_MENU;
        }
#endif              
        break;
        
      case LEFT:
#ifdef FLUX_WEAKENING
        bMenu_index = TARGET_VOLT_MENU;              
#elif defined DIFFERENTIAL_TERM_ENABLED            
        bMenu_index = D_FLUX_MENU;
#else
        bMenu_index = I_FLUX_MENU;                          
#endif              
        break;
        
      default:
        break;
      }
      break; 
      
#ifdef OBSERVER_GAIN_TUNING
    case K1_MENU:    
      switch(bKey)
      {
      case UP:
        if (UI_GetReg(this, MC_PROTOCOL_REG_OBSERVER_C1) <= -K1_INC_DEC)
        {
          UI_SetReg(this, MC_PROTOCOL_REG_OBSERVER_C1,
                    UI_GetReg(this, MC_PROTOCOL_REG_OBSERVER_C1) +
                      K1_INC_DEC);
        }
        break;  
        
      case DOWN:
        if (UI_GetReg(this, MC_PROTOCOL_REG_OBSERVER_C1) >= -600000)
        {
          UI_SetReg(this, MC_PROTOCOL_REG_OBSERVER_C1,
                    UI_GetReg(this, MC_PROTOCOL_REG_OBSERVER_C1) -
                      K1_INC_DEC);
        }
        break;
        
      case RIGHT:
        bMenu_index = K2_MENU;
        break;
        
      case LEFT:         
        bMenu_index = POWER_STAGE_MENU;
        break;
        
      default:
        break;
      }
      break;
      
    case K2_MENU:    
      switch(bKey)
      {
      case UP:
        if (UI_GetReg(this, MC_PROTOCOL_REG_OBSERVER_C1) <= S16_MAX - K2_INC_DEC)
        {
          UI_SetReg(this, MC_PROTOCOL_REG_OBSERVER_C2,
                    UI_GetReg(this, MC_PROTOCOL_REG_OBSERVER_C2) +
                      K2_INC_DEC);
        }
        break;  
        
      case DOWN:
        if (UI_GetReg(this, MC_PROTOCOL_REG_OBSERVER_C1) >= S16_MIN + K2_INC_DEC)
        {
          UI_SetReg(this, MC_PROTOCOL_REG_OBSERVER_C2,
                    UI_GetReg(this, MC_PROTOCOL_REG_OBSERVER_C2) -
                      K2_INC_DEC);
        }
        break;
        
      case RIGHT:
        bMenu_index = P_PLL_MENU;
        break;
        
      case LEFT:         
        bMenu_index = K1_MENU;
        break;
        
      default:
        break;
      }
      break;
      
    case P_PLL_MENU:    
      switch(bKey)
      {
      case UP:
        if (UI_GetReg(this, MC_PROTOCOL_REG_PLL_KP) <= 32000)
        {
          UI_SetReg(this, MC_PROTOCOL_REG_PLL_KP,
                    UI_GetReg(this, MC_PROTOCOL_REG_PLL_KP) +
                      PLL_IN_DEC);
        } 
        break;  
        
      case DOWN:
        if (UI_GetReg(this, MC_PROTOCOL_REG_PLL_KP) > 0)
        {
          UI_SetReg(this, MC_PROTOCOL_REG_PLL_KP,
                    UI_GetReg(this, MC_PROTOCOL_REG_PLL_KP) -
                      PLL_IN_DEC);
        } 
        break;
        
      case RIGHT:
        bMenu_index = I_PLL_MENU;
        break;
        
      case LEFT:         
        bMenu_index = K2_MENU;
        break;
        
      default:
        break;
      }
      break;
      
    case I_PLL_MENU:    
      switch(bKey)
      {
      case UP:
        if (UI_GetReg(this, MC_PROTOCOL_REG_PLL_KI) <= 32000)
        {
          UI_SetReg(this, MC_PROTOCOL_REG_PLL_KI,
                    UI_GetReg(this, MC_PROTOCOL_REG_PLL_KI) +
                      PLL_IN_DEC);
        } 
        break;  
        
      case DOWN:
        if (UI_GetReg(this, MC_PROTOCOL_REG_PLL_KI) > 0)
        {
          UI_SetReg(this, MC_PROTOCOL_REG_PLL_KI,
                    UI_GetReg(this, MC_PROTOCOL_REG_PLL_KI) -
                      PLL_IN_DEC);
        } 
        break;
        
      case RIGHT:
#if (DAC_FUNCTIONALITY == ENABLE)
        bMenu_index = DAC_PB0_MENU;  
#else
        if (MCI_GetControlMode(oMCI) == STC_SPEED_MODE)
        {
          bMenu_index = MOTOR_SPD_MENU;
        }
        else
        {
          bMenu_index = MOTOR_TRQ_MENU;
        }
#endif              
        break;
        
      case LEFT:         
        bMenu_index = P_PLL_MENU;
        break;
        
      default:
        break;
      }
      break;          
#endif
      
#if (DAC_FUNCTIONALITY == ENABLE)  
    case DAC_PB0_MENU:    
      switch(bKey)
      {
      case UP:
        LCDV_DACSelUpdate(this, 1, DAC_CH0);
        break;
        
      case DOWN:
        LCDV_DACSelUpdate(this, -1, DAC_CH0);
        break;  
        
      case RIGHT:                
        bMenu_index = DAC_PB1_MENU;               
        break;
        
      case LEFT:
#ifdef OBSERVER_GAIN_TUNING              
        bMenu_index = I_PLL_MENU;
#else
        bMenu_index = POWER_STAGE_MENU;
#endif              
        break;
        
      default:
        break;
      }
      break;   
      
    case DAC_PB1_MENU:    
      switch(bKey)
      {
      case UP:
        LCDV_DACSelUpdate(this, 1, DAC_CH1);
        break;
        
      case DOWN:
        LCDV_DACSelUpdate(this, -1, DAC_CH1);
        break;  
        
      case RIGHT:
        if (MCI_GetControlMode(oMCI) == STC_SPEED_MODE)
        {
          bMenu_index = MOTOR_SPD_MENU;
        }
        else
        {
          bMenu_index = MOTOR_TRQ_MENU;
        }
        break;
        
      case LEFT:             
        bMenu_index = DAC_PB0_MENU;
        break;
        
      default:
        break;
      }
      break; 
#endif 
      
    default:
      break; 
    }
  }
}

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
