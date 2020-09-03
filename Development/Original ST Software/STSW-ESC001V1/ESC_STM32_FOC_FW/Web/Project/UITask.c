/**
  ******************************************************************************
  * @file    UITask.c 
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file implementes user interface tasks definition 
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
/* Pre-compiler coherency check */
#define PROJECT_CHK
#include "CrossCheck.h" 
#undef PROJECT_CHK

#include "UITask.h"
#include "MCLibraryISRPriorityConf.h"
#include "USARTParams.h"
#include "DACParams.h"
#include "UIExportedFunctions.h"
#include "Parameters conversion.h"

#if !defined(DEFAULT_DAC_CHANNEL_1)
#define DEFAULT_DAC_CHANNEL_1 MC_PROTOCOL_REG_I_A
#endif
#if !defined(DEFAULT_DAC_CHANNEL_2)
#define DEFAULT_DAC_CHANNEL_2 MC_PROTOCOL_REG_I_B
#endif

#ifdef DUALDRIVE
  #include "Parameters conversion motor 2.h"
#endif

#define OPT_DACX  0x20 /*!<Bit field indicating that the UI uses SPI AD7303 DAC.*/

CUI oLCD = MC_NULL;

CUSART_COM oUSART = MC_NULL;
CFCP oFCP = MC_NULL;
CMCP_UI oMCP = MC_NULL;

CUI oDAC = MC_NULL;

CUFC_UI oUFC = MC_NULL;

static volatile uint16_t  bUITaskCounter;
static volatile uint16_t  bCOMTimeoutCounter;
static volatile uint16_t  bCOMATRTimeCounter = SERIALCOM_ATR_TIME_TICKS;

#if defined(STM32F10X_MD)
#define VECT_TABLE_BASE 0x08010000
#endif

#if defined(STM32F10X_MD_VL)
#define VECT_TABLE_BASE 0x08013800
#endif

#if defined(STM32F10X_HD)
#define VECT_TABLE_BASE 0x08070000
#endif

#if defined(STM32F2XX)
#define VECT_TABLE_BASE 0x080F0000
#endif


#if defined(STM32F40XX)
#define VECT_TABLE_BASE 0x080F0000
#endif

#if defined(STM32F446xC_xE)
#define VECT_TABLE_BASE 0x08070000
#endif

#if defined(STM32F0XX)
#if defined(STM32F072x)
#define VECT_TABLE_BASE 0x08018000
#else
#define VECT_TABLE_BASE 0x08007E00
#endif
#endif

#if defined(STM32F30X)
#define VECT_TABLE_BASE 0x08030000
#endif

/* Setup the exported functions see UIExportedFunctions.h enum. */
void* const exportedFunctions[EF_UI_NUMBERS] =
{
  (void*)(&UI_GetReg),
  (void*)(&UI_ExecSpeedRamp),
  (void*)(&UI_SetReg),
  (void*)(&UI_ExecCmd),
  (void*)(&UI_GetSelectedMCConfig),
  (void*)(&UI_SetRevupData),
  (void*)(&UI_GetRevupData),
  (void*)(&UI_SetDAC),
  (void*)(&UI_SetCurrentReferences)
};

void UI_TaskInit(uint8_t cfg, uint32_t* pUICfg, uint8_t bMCNum, CMCI oMCIList[],
                 CMCT oMCTList[],const char* s_fwVer)
{  
#if (defined(DAC_FUNCTIONALITY))
  #if (DAC_OPTION == OPT_DACT)
    if (cfg & OPT_DACT)
    {
      oDAC = (CUI)DACT_NewObject(MC_NULL,MC_NULL);
      
      UI_Init((CUI)oDAC, bMCNum, oMCIList, oMCTList, pUICfg); /* Init UI and link MC obj */
      UI_DACInit((CUI)oDAC); /* Init DAC */
      UI_SetDAC((CUI)oDAC, DAC_CH0, DEFAULT_DAC_CHANNEL_1);
      UI_SetDAC((CUI)oDAC, DAC_CH1, DEFAULT_DAC_CHANNEL_2);  
    }
  #endif
    
  #if (DAC_OPTION == OPT_DAC)
    if (cfg & OPT_DAC)
    {
      oDAC = (CUI)DAC_NewObject(MC_NULL,MC_NULL);
      
      UI_Init((CUI)oDAC, bMCNum, oMCIList, oMCTList, pUICfg); /* Init UI and link MC obj */
      UI_DACInit((CUI)oDAC); /* Init DAC */
      UI_SetDAC((CUI)oDAC, DAC_CH0, DEFAULT_DAC_CHANNEL_1);
      UI_SetDAC((CUI)oDAC, DAC_CH1, DEFAULT_DAC_CHANNEL_2);  
    }
  #endif

  #if (DAC_OPTION == OPT_DACF3)
    if (cfg & OPT_DAC)
    {
      oDAC = (CUI)DAC_F30X_NewObject(MC_NULL,&DAC_F30XParams_str);
      
      UI_Init((CUI)oDAC, bMCNum, oMCIList, oMCTList, pUICfg); /* Init UI and link MC obj */
      UI_DACInit((CUI)oDAC); /* Init DAC */
      UI_SetDAC((CUI)oDAC, DAC_CH0, DEFAULT_DAC_CHANNEL_1);
      UI_SetDAC((CUI)oDAC, DAC_CH1, DEFAULT_DAC_CHANNEL_2);  
    }
  #endif
    
  #if (DAC_OPTION == OPT_DACF072)
    if (cfg & OPT_DAC)
    {
      oDAC = (CUI)DAC_F0X_NewObject(MC_NULL,&DAC_F0XParams_str);
      
      UI_Init((CUI)oDAC, bMCNum, oMCIList, oMCTList, pUICfg); /* Init UI and link MC obj */
      UI_DACInit((CUI)oDAC); /* Init DAC */
      UI_SetDAC((CUI)oDAC, DAC_CH0, DEFAULT_DAC_CHANNEL_1);
      UI_SetDAC((CUI)oDAC, DAC_CH1, DEFAULT_DAC_CHANNEL_2);  
    }
  #endif
    
  #if (DAC_OPTION == OPT_DACS)
    if (cfg & OPT_DACS)
    {
      oDAC = (CUI)DACS_NewObject(MC_NULL,MC_NULL);
      
      UI_Init((CUI)oDAC, bMCNum, oMCIList, oMCTList, pUICfg); /* Init UI and link MC obj */
      UI_DACInit((CUI)oDAC); /* Init DAC */
      UI_SetDAC((CUI)oDAC, DAC_CH0, DEFAULT_DAC_CHANNEL_1);
      UI_SetDAC((CUI)oDAC, DAC_CH1, DEFAULT_DAC_CHANNEL_2);  
    }
  #endif
    
  #if (DAC_OPTION == OPT_DACX)
    if (cfg & OPT_DACX)
    {
      oDAC = (CUI)DACX_NewObject(MC_NULL,MC_NULL);
      
      UI_Init((CUI)oDAC, bMCNum, oMCIList, oMCTList, pUICfg); /* Init UI and link MC obj */
      UI_DACInit((CUI)oDAC); /* Init DAC */
      UI_SetDAC((CUI)oDAC, DAC_CH0, DEFAULT_DAC_CHANNEL_1);
      UI_SetDAC((CUI)oDAC, DAC_CH1, DEFAULT_DAC_CHANNEL_2);  
    }
  #endif
#endif
  
#if (defined(LCD_FUNCTIONALITY))
  if (cfg & OPT_LCD)
  {
#if (LCD_MODE == LCD_FULL)
    /* LCD FULL Version */
    #define LCDI_Code_Addr   VECT_TABLE_BASE + 0x08 /* Vect base + 0x08 */
    #define LCDI_Entry_Addr  VECT_TABLE_BASE + 0x0C /* Vect base + 0x0C */

    /* Check the presence of LCD UI code */
    uint32_t* pLCDI_Code = (uint32_t*)(LCDI_Code_Addr);
    uint32_t LCDI_Code = *pLCDI_Code;
    if (LCDI_Code == 0x1A525D)
    {
      /* Call LCD UI entry point */
      uint32_t* pLCDI_Entry = (uint32_t*)(LCDI_Entry_Addr);
      typedef void* const* (*pLCDI_Entry_t) (void* const*);
      pLCDI_Entry_t pLCDI_EntryFunc = (pLCDI_Entry_t)(*pLCDI_Entry);
      LCD_SetLCDIImportedFunctions((*pLCDI_EntryFunc)(exportedFunctions));
      
      oLCD = (CUI)LCD_NewObject(MC_NULL, MC_NULL);
    }
    else
    {
      while (1); /* Error LCDI code not correctly flashed */
    }
#endif
    
#if (LCD_MODE == LCD_LIGHT)
    /* LCD Vintage */
    oLCD = (CUI)LCDV_NewObject(MC_NULL,MC_NULL);
#endif
    
    UI_Init(oLCD, bMCNum, oMCIList, oMCTList, pUICfg); /* Init UI and link MC obj */
    UI_LCDInit(oLCD, (CUI)oDAC, s_fwVer); /* Initialize object it must be called after UI_Init (See Interface) */
    UI_LCDExec(oLCD); /* Shows welcome message */
  }
#endif

#if (defined(SERIAL_COMMUNICATION))
  if (cfg & OPT_COM)
  {
#if (SERIAL_COM_MODE == COM_BIDIRECTIONAL)
    oMCP = MCP_NewObject(MC_NULL,&MCPParams);
    oUSART = USART_NewObject(&USARTParams_str);
    oFCP = FCP_NewObject(&FrameParams_str);

    FCP_Init(oFCP, (CCOM)oUSART);
    MCP_Init(oMCP, oFCP, oDAC, s_fwVer);
    UI_Init((CUI)oMCP, bMCNum, oMCIList, oMCTList, pUICfg); /* Init UI and link MC obj */
#endif
    
#if (SERIAL_COM_MODE == COM_UNIDIRECTIONAL)
    oUFC = UFC_NewObject(MC_NULL, &UFCParams_str);
    UFC_Init(oUFC);
    UI_Init((CUI)oUFC, bMCNum, oMCIList, oMCTList, pUICfg); /* Init UI and link MC obj */
    UFC_StartCom(oUFC); /* Start transmission */
#endif
  }  
#endif
}


void UI_Scheduler(void)
{
  if(bUITaskCounter > 0u)
  {
    bUITaskCounter--;
  }
  
  if(bCOMTimeoutCounter > 1u)
  {
    bCOMTimeoutCounter--;
  }
  
  if(bCOMATRTimeCounter > 1u)
  {
    bCOMATRTimeCounter--;
  }
}

#define LCD_WELCOME_MESSAGE_DURATION_MS 1000
#define LCD_WELCOME_MESSAGE_DURATION_UI_TICKS (LCD_WELCOME_MESSAGE_DURATION_MS * UI_TASK_FREQUENCY_HZ) / 1000
static uint16_t LCD_welcome_message_UI_counter = LCD_WELCOME_MESSAGE_DURATION_UI_TICKS;

void UI_LCDRefresh(void)
{
    if (LCD_welcome_message_UI_counter == 0)
    {
      UI_LCDExec(oLCD);
      UI_LCDUpdateMeasured(oLCD);
    }
    else
    {
      LCD_welcome_message_UI_counter--;
    }
}

void UI_DACUpdate(uint8_t bMotorNbr)
{
  if (UI_GetSelectedMC((CUI)oDAC) == bMotorNbr)
  {
    UI_DACExec((CUI)oDAC); /* Exec DAC update */
  }
}

void MC_SetDAC(DAC_Channel_t bChannel, MC_Protocol_REG_t bVariable)
{
  UI_SetDAC(oDAC, bChannel, bVariable);
}

void MC_SetUserDAC(DAC_UserChannel_t bUserChNumber, int16_t hValue)
{
  UI_SetUserDAC(oDAC, bUserChNumber, hValue);
}

CUI GetLCD(void)
{
  return oLCD;
}

CMCP_UI GetMCP(void)
{
  return oMCP;
}

CUI GetDAC(void)
{
  return oDAC;
}

bool UI_IdleTimeHasElapsed(void)
{
  bool retVal = FALSE;
  if (bUITaskCounter == 0u)
  {
    retVal = TRUE;
  }
  return (retVal);
}

void UI_SetIdleTime(uint16_t SysTickCount)
{
  bUITaskCounter = SysTickCount;
}

bool UI_SerialCommunicationTimeOutHasElapsed(void)
{
  bool retVal = FALSE;
  if (bCOMTimeoutCounter == 1u)
  {
    bCOMTimeoutCounter = 0u;
    retVal = TRUE;
  }
  return (retVal);
}

bool UI_SerialCommunicationATRTimeHasElapsed(void)
{
  bool retVal = FALSE;
  if (bCOMATRTimeCounter == 1u)
  {
    bCOMATRTimeCounter = 0u;
    retVal = TRUE;
  }
  return (retVal);
}

#ifndef FREE_RTOS
void UI_SerialCommunicationTimeOutStop(void)
{
  bCOMTimeoutCounter = 0u;
}
#endif

#ifndef FREE_RTOS
void UI_SerialCommunicationTimeOutStart(void)
{
  bCOMTimeoutCounter = SERIALCOM_TIMEOUT_OCCURENCE_TICKS;
}
#endif

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
