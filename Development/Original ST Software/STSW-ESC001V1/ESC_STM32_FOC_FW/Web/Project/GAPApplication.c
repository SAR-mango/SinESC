/**
  ******************************************************************************
  * @file    GAPApplication.c
  * @author  IMS Systems Lab and Technical Marketing - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   GAPApplication      
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
#include "..\..\MCLibrary\interface\GateDriverCtrlClass.h"
#include "..\..\MCLibrary\interface\GAP_GateDriverCtrlClass.h"

#define GAP_DEVICE_NUM 7

// Parameters
#define GAP_CFG1  GAP_CFG1_SD_FLAG
#define GAP_CFG2  GAP_CFG2_DESATTH_7V  //| GAP_CFG2_SENSETH_400MV  //| GAP_CFG2_SENSETH_400MV| GAP_CFG2_DESATTH_7V
#define GAP_CFG3  GAP_CFG3_2LTOTH_7_0V | GAP_CFG3_2LTOTIME_DISABLE
#define GAP_CFG4  GAP_CFG4_UVLOTH_VH_DISABLE | GAP_CFG4_UVLOTH_VL_DISABLE
#define GAP_CFG5  GAP_CFG5_CLAMP_EN  //| GAP_CFG5_SENSE_EN  // | GAP_CFG5_2LTO_ON_FAULT
#define GAP_DIAG1  GAP_DIAG_SPI_REGERR | GAP_DIAG_UVLOD_OVLOD | GAP_DIAG_OVLOH_OVLOL | GAP_DIAG_DESAT_SENSE// | GAP_DIAG_TSD
#define GAP_DIAG2 GAP_DIAG_NONE

#define SPI                    SPI2
#define SPI_REMAP              GPIO_NoRemap_SPI
#define SPI_SCK_PORT           GPIOB  
#define SPI_SCK_PIN            GPIO_Pin_13
#define SPI_MISO_PORT          GPIOB
#define SPI_MISO_PIN           GPIO_Pin_14
#define SPI_MOSI_PORT          GPIOB
#define SPI_MOSI_PIN           GPIO_Pin_15
#define SPI_NCS_PORT           GPIOD
#define SPI_NCS_PIN            GPIO_Pin_2
#define SPI_NSD_PORT           GPIOC
#define SPI_NSD_PIN            GPIO_Pin_9

GAPDeviceParams_t GAPDeviceParams =
{
  GAP_CFG1,
  GAP_CFG2,
  GAP_CFG3,
  GAP_CFG4,
  GAP_CFG5,
  GAP_DIAG1,
  GAP_DIAG2
};

GAPParams_t GAPParams =
{
  GAP_DEVICE_NUM,
  &GAPDeviceParams,
  &GAPDeviceParams,
  &GAPDeviceParams,
  &GAPDeviceParams,
  &GAPDeviceParams,
  &GAPDeviceParams,
  &GAPDeviceParams,
  SPI,
  SPI_REMAP,
  SPI_SCK_PORT,
  SPI_SCK_PIN,
  SPI_MISO_PORT,
  SPI_MISO_PIN,
  SPI_MOSI_PORT,
  SPI_MOSI_PIN,
  SPI_NCS_PORT,
  SPI_NCS_PIN,
  SPI_NSD_PORT,
  SPI_NSD_PIN
};

GateDriverCtrlParams_t GateDriverCtrlParams = 
{
  0 // Dummy
};

/* Private variables ---------------------------------------------------------*/
CGDC oGDC;
typedef enum {TEST_IDLE,TEST_FAULT,TEST_RUN} state_t;
uint32_t GAP_errorsNow[GAP_DEVICE_NUM];
uint32_t GAP_errorsOccurred[GAP_DEVICE_NUM];
volatile bool faultGap,devicesProgrammed = FALSE;
volatile uint32_t cnt = 0;
#define LOOP 240000
volatile state_t state = TEST_IDLE;
volatile uint8_t test = 0;
volatile uint8_t configuration = 0;
volatile uint8_t checkerror = 0;
volatile uint8_t readRegs = 0;
volatile uint8_t spareValue = 0xAA;
volatile uint8_t globalCFG1[GAP_DEVICE_NUM];
volatile uint8_t globalCFG2[GAP_DEVICE_NUM];
volatile uint8_t globalCFG3[GAP_DEVICE_NUM];
volatile uint8_t globalCFG4[GAP_DEVICE_NUM];
volatile uint8_t globalCFG5[GAP_DEVICE_NUM];
volatile uint8_t globalTEST1[GAP_DEVICE_NUM];
volatile uint8_t globalDIAG1[GAP_DEVICE_NUM];
volatile uint8_t globalDIAG2[GAP_DEVICE_NUM];
volatile uint8_t globalSTATUS1[GAP_DEVICE_NUM];
volatile uint8_t globalSTATUS2[GAP_DEVICE_NUM];
volatile uint8_t globalSTATUS3[GAP_DEVICE_NUM];

/**
  * @brief  Boot function to initialize the GAP controller.
  * @retval none.
  */
void GAPboot(void)
{
  oGDC = (CGDC)GAP_NewObject(&GateDriverCtrlParams, &GAPParams);
  GDC_Init(oGDC);
  if (!GAP_Configuration(oGDC))
  {
    state = TEST_FAULT;
    
    // Trap
    while (1);
  };
}

/**
  * @brief  Schedule function to be called with periodicity.
  * @retval none.
  */ 
void GAPSchedule(void)
{
  switch (test)
  {
  case 1:
    {
      GAP_FaultAck(oGDC);
      state = TEST_IDLE;
      test = 0;
    }
    break;
  case 2:
    {
      uint8_t writeData[GAP_DEVICE_NUM];
      writeData[0] = spareValue;
      GAP_WriteRegs(oGDC,writeData,CFG1);
      test = 0;
    }
    break;
  case 3:
    {
      if (state == TEST_IDLE)
      {
        if (!GAP_Configuration(oGDC))
        {
          state = TEST_FAULT;
        } else
        {
          state = TEST_RUN;
        }
      }
      test = 0;
    }
    break;
  case 4:
    {
      if (GAP_Test(oGDC, SENSE_RESISTOR_CHK))
      {
        state = TEST_FAULT;
      };
      if (GAP_Test(oGDC, SENSE_COMPARATOR_CHK))
      {
        state = TEST_FAULT;
      };
      if (GAP_Test(oGDC, GON_CHK))
      {
        state = TEST_FAULT;
      };
      if (GAP_Test(oGDC, GOFF_CHK))
      {
        state = TEST_FAULT;
      };
      if (GAP_Test(oGDC, DESAT_CHK))
      {
        state = TEST_FAULT;
      };
      test = 0;
    }
    break;
    
  default:
    {
    }
    break;
  }
  if (cnt <= LOOP)
  {
    cnt++;
  }
  else
  {
    uint8_t i;
    
    if (checkerror)
    {
      // Check errors
      faultGap = GAP_CheckErrors(oGDC,GAP_errorsNow,GAP_errorsOccurred);
      if (faultGap)
      {
        for (i = 0; i < GAP_DEVICE_NUM; i++)
        {
          if ((GAP_errorsNow[i] != 0)||((GAP_errorsOccurred[i] != 0)))
          {
            state = TEST_FAULT;
            //Led1On();
          }
        }
      }
      else
      {
        state = TEST_FAULT;
      }
    }
    
    if (readRegs)
    {
      // Read reg
      uint8_t cfg[GAP_DEVICE_NUM];
      GAP_ReadRegs(oGDC,cfg,CFG1);
      for (i = 0; i < GAP_DEVICE_NUM; i++)
      {
        globalCFG1[i] = cfg[i];
      }
      GAP_ReadRegs(oGDC,cfg,CFG2);
      for (i = 0; i < GAP_DEVICE_NUM; i++)
      {
        globalCFG2[i] = cfg[i];
      }
      GAP_ReadRegs(oGDC,cfg,CFG3);
      for (i = 0; i < GAP_DEVICE_NUM; i++)
      {
        globalCFG3[i] = cfg[i];
      }
      GAP_ReadRegs(oGDC,cfg,CFG4);
      for (i = 0; i < GAP_DEVICE_NUM; i++)
      {
        globalCFG4[i] = cfg[i];
      }
      GAP_ReadRegs(oGDC,cfg,CFG5);
      for (i = 0; i < GAP_DEVICE_NUM; i++)
      {
        globalCFG5[i] = cfg[i];
      }
      GAP_ReadRegs(oGDC,cfg,TEST1);
      for (i = 0; i < GAP_DEVICE_NUM; i++)
      {
        globalTEST1[i] = cfg[i];
      }
      GAP_ReadRegs(oGDC,cfg,DIAG1);
      for (i = 0; i < GAP_DEVICE_NUM; i++)
      {
        globalDIAG1[i] = cfg[i];
      }
      GAP_ReadRegs(oGDC,cfg,DIAG2);
      for (i = 0; i < GAP_DEVICE_NUM; i++)
      {
        globalDIAG2[i] = cfg[i];
      }
      GAP_ReadRegs(oGDC,cfg,STATUS1);
      for (i = 0; i < GAP_DEVICE_NUM; i++)
      {
        globalSTATUS1[i] = cfg[i];
      }
      GAP_ReadRegs(oGDC,cfg,STATUS2);
      for (i = 0; i < GAP_DEVICE_NUM; i++)
      {
        globalSTATUS2[i] = cfg[i];
      }
      GAP_ReadRegs(oGDC,cfg,STATUS3);
      for (i = 0; i < GAP_DEVICE_NUM; i++)
      {
        globalSTATUS3[i] = cfg[i];
      }
      // Just one time
      readRegs = 0;
    }
    
    cnt = 0;
  }
}
/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
