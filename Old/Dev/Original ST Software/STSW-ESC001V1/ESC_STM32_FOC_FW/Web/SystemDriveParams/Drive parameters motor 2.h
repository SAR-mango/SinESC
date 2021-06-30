/**
  ******************************************************************************
  * @file    Drive parameters motor 2.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains motor parameters needed by STM32 PMSM MC FW  
  *          library v4.3.0
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
#ifndef __DRIVE_PARAMETERS_MOTOR2_H
#define __DRIVE_PARAMETERS_MOTOR2_H

/******** MAIN AND AUXILIARY SPEED/POSITION SENSOR(S) SETTINGS SECTION ********/

/*** Speed sensor selection ***/
#define STATE_OBSERVER_PLL2 
/* #define STATE_OBSERVER_CORDIC2 */
/* #define HFINJECTION2 */
/* #define ENCODER2 */
/* #define HALL_SENSORS2 */


/*** Auxiliary speed measurement selection ***/
/* #define AUX_STATE_OBSERVER_PLL2 */
/* #define AUX_STATE_OBSERVER_CORDIC2 */
/* #define AUX_HFINJECTION2 */
/* #define AUX_ENCODER2 */


/*** Speed measurement settings ***/
#define MAX_APPLICATION_SPEED2         4000 /*!< rpm, mechanical */
#define MIN_APPLICATION_SPEED2         0 /*!< rpm, mechanical,  
                                                           absolute value */
#define MEAS_ERRORS_BEFORE_FAULTS2     3 /*!< Number of speed  
                                                             measurement errors before 
                                                             main sensor goes in fault */

/*** Encoder **********************/                                                                                                           
#define ENC_MEAS_ERRORS_BEFORE_FAULTS2 3 /*!< Number of failed   
                                                        derived class specific speed 
                                                        measurements before main sensor  
                                                        goes in fault */
#define ENC_ICx_FILTER2                118 /*!< Duration of input  
capture filter in CPU clock   
                                                           cycles in case of   
                                                           quadrature encoder main 
                                                           or auxiliary sensors in use */
#define ENC_INVERT_SPEED2               DISABLE  /*!< To be enabled for  
                                                            encoder (main or aux) if  
                                                            measured speed is opposite 
                                                            to real one */        

#define ENC_AVERAGING_FIFO_DEPTH2      16 /*!< depth of the FIFO used to 
                                                              average mechanical speed in 
                                                              0.1Hz resolution */
/****** Hall sensors ************/ 
#define HALL_MEAS_ERRORS_BEFORE_FAULTS2 3 /*!< Number of failed   
                                                           derived class specific speed 
                                                           measurements before main sensor  
                                                           goes in fault */
#define HALL_ICx_FILTER2                218 /*!< Duration of input  
                                                           capture filter in CPU clock   
                                                           cycles in case of Hall sensors  
                                                           main or auxiliary sensors
                                                           in use */
#define HALL_AVERAGING_FIFO_DEPTH2      6 /*!< depth of the FIFO used to 
                                                           average mechanical speed in 
                                                           0.1Hz resolution */                                                                                                           
/****** State Observer + PLL ****/
#define VARIANCE_THRESHOLD2            0.062  /*!<Maximum accepted 
                                                            variance on speed 
                                                            estimates (percentage) */
/* State observer scaling factors F1 */                    
#define F12                            2048
#define F22                            16384

/* State observer constants */
#define GAIN12                         -3016
#define GAIN22                         18718
/*Only in case PLL is used, PLL gains */
#define PLL_KP_GAIN2                   266
#define PLL_KI_GAIN2                   11

#define OBS_MEAS_ERRORS_BEFORE_FAULTS2 3  /*!< Number of consecutive errors   
                                                           on variance test before a speed 
                                                           feedback error is reported */
#define STO_FIFO_DEPTH_DPP2            64  /*!< Depth of the FIFO used  
                                                            to average mechanical speed  
                                                            in dpp format */
#define STO_FIFO_DEPTH_01HZ2           64  /*!< Depth of the FIFO used  
                                                            to average mechanical speed  
                                                            in dpp format */
#define BEMF_CONSISTENCY_TOL2          64   /* Parameter for B-emf 
                                                            amplitude-speed consistency */
#define BEMF_CONSISTENCY_GAIN2         64   /* Parameter for B-emf 
                                                           amplitude-speed consistency */
                                                                                
/****** State Observer + CORDIC ***/
#define CORD_VARIANCE_THRESHOLD2       4  /*!<Maximum accepted 
                                                            variance on speed 
                                                            estimates (percentage) */                                                                                                                
#define CORD_F12                       2048
#define CORD_F22                       16384

/* State observer constants */
#define CORD_GAIN12                    -3016
#define CORD_GAIN22                    18718

#define CORD_MEAS_ERRORS_BEFORE_FAULTS2 3  /*!< Number of consecutive errors   
                                                           on variance test before a speed 
                                                           feedback error is reported */
#define CORD_FIFO_DEPTH_DPP2            64  /*!< Depth of the FIFO used  
                                                            to average mechanical speed  
                                                            in dpp format */
#define CORD_FIFO_DEPTH_01HZ2           64  /*!< Depth of the FIFO used  
                                                           to average mechanical speed  
                                                           in dpp format */        
#define CORD_MAX_ACCEL_DPPP2            30  /*!< Maximum instantaneous 
                                                              electrical acceleration (dpp 
                                                              per control period) */
#define CORD_BEMF_CONSISTENCY_TOL2      64  /* Parameter for B-emf 
                                                           amplitude-speed consistency */
#define CORD_BEMF_CONSISTENCY_GAIN2     64  /* Parameter for B-emf 
                                                          amplitude-speed consistency */

/****** HFI ******/                                                          
#define	HFI_FREQUENCY2                  400
#define	HFI_AMPLITUDE2                  25

#define	HFI_PID_KP_DEFAULT2             800
#define	HFI_PID_KI_DEFAULT2             400
#define	HFI_PID_KPDIV2	                16384
#define	HFI_PID_KIDIV2	                32768

#define	HFI_IDH_DELAY2	                32400

#define	HFI_PLL_KP_DEFAULT2             0.00060
#define	HFI_PLL_KI_DEFAULT2             0.00400

#define	HFI_NOTCH_0_COEFF2              0.962363
#define	HFI_NOTCH_1_COEFF2              -1.90103
#define	HFI_NOTCH_2_COEFF2              0.962363
#define	HFI_NOTCH_3_COEFF2              1.90103
#define	HFI_NOTCH_4_COEFF2              -0.924727

#define	HFI_LP_0_COEFF2                 0.009086
#define	HFI_LP_1_COEFF2                 0.018172
#define	HFI_LP_2_COEFF2                 0.009086
#define	HFI_LP_3_COEFF2                 1.712838
#define	HFI_LP_4_COEFF2                 -0.749181

#define	HFI_HP_0_COEFF2                 0.925191
#define	HFI_HP_1_COEFF2                 -1.850383
#define	HFI_HP_2_COEFF2                 0.925191
#define	HFI_HP_3_COEFF2                 1.844778
#define	HFI_HP_4_COEFF2                 -0.855987

#define	HFI_DC_0_COEFF2                 0.00146
#define	HFI_DC_1_COEFF2                 0.002921
#define	HFI_DC_2_COEFF2                 0.00146
#define	HFI_DC_3_COEFF2                 1.889033
#define	HFI_DC_4_COEFF2                 -0.894874

#define HFI_MINIMUM_SPEED_RPM2          402
#define HFI_SPD_BUFFER_DEPTH_01HZ2      64
#define	HFI_LOCKFREQ2                   41
#define	HFI_SCANROTATIONSNO2            3
#define	HFI_WAITBEFORESN2               6
#define	HFI_WAITAFTERNS2                4
#define	HFI_HIFRAMPLSCAN2               25
#define	HFI_NSMAXDETPOINTS2             20
#define	HFI_NSDETPOINTSSKIP2            10
#define	HFI_DEBUG_MODE2                 FALSE

#define HFI_STO_RPM_TH2                  OBS_MINIMUM_SPEED_RPM2
#define STO_HFI_RPM_TH2                  460
#define HFI_RESTART_RPM_TH2              (((HFI_STO_RPM_TH2) + (STO_HFI_RPM_TH2))/2)
#define HFI_NS_MIN_SAT_DIFF2             0

#define HFI_REVERT_DIRECTION2            FALSE
#define HFI_WAITTRACK2                   20
#define HFI_WAITSYNCH2                   20
#define HFI_STEPANGLE2                   3640
#define HFI_MAXANGLEDIFF2                3640
#define HFI_RESTARTTIMESEC2              0.1

/**************************    DRIVE SETTINGS SECTION   **********************/
/* Dual drive specific parameters */
#define FREQ_RATIO                      1  /* Higher PWM frequency/lower PWM frequency */  
#define FREQ_RELATION                   HIGHEST_FREQ  /* It refers to motor 1 and can be 
                                                           HIGHEST_FREQ or LOWEST frequency depending 
                                                           on motor 1 and 2 frequency relationship */
#define FREQ_RELATION2                  LOWEST_FREQ   /* It refers to motor 2 and can be 
                                                           HIGHEST_FREQ or LOWEST frequency depending 
                                                           on motor 1 and 2 frequency relationship */

/* PWM generation and current reading */
#define PWM_FREQUENCY2                 16000
                                                                                                                
#define LOW_SIDE_SIGNALS_ENABLING2     LS_PWM_TIMER
#define SW_DEADTIME_NS2                800 /*!< Dead-time to be inserted  
                                                           by FW, only if low side 
                                                           signals are enabled */
#define HIGH_SIDE_IDLE_STATE2          TURN_OFF /*!< TURN_OFF, TURN_ON */
#define LOW_SIDE_IDLE_STATE2           TURN_OFF /*!< TURN_OFF, TURN_ON */
                                                                                          
/* Torque and flux regulation loops */
#define REGULATION_EXECUTION_RATE2     1 /*!< FOC execution rate in 
                                                           number of PWM cycles */     
/* Gains values for torque and flux control loops */
#define PID_TORQUE_KP_DEFAULT2         3643       
#define PID_TORQUE_KI_DEFAULT2         531
#define PID_TORQUE_KD_DEFAULT2         100
#define PID_FLUX_KP_DEFAULT2           3643
#define PID_FLUX_KI_DEFAULT2           531
#define PID_FLUX_KD_DEFAULT2           100

/* Torque/Flux control loop gains dividers*/
#define TF_KPDIV2                      4096
#define TF_KIDIV2                      16384
#define TF_KDDIV2                      8192
#define TFDIFFERENTIAL_TERM_ENABLING2  DISABLE

/* Speed control loop */ 
#define SPEED_LOOP_FREQUENCY_HZ2        500 /*!<Execution rate of speed   
                                                      regulation loop (Hz) */
#define PID_SPEED_KP_DEFAULT2           1000
#define PID_SPEED_KI_DEFAULT2           700
#define PID_SPEED_KD_DEFAULT2           0
/* Speed PID parameter dividers */
#define SP_KPDIV2                       16
#define SP_KIDIV2                       256
#define SP_KDDIV2                       16
#define SPD_DIFFERENTIAL_TERM_ENABLING2 DISABLE

/* Default settings */
#define DEFAULT_CONTROL_MODE2           STC_SPEED_MODE /*!< STC_TORQUE_MODE or 
                                                        STC_SPEED_MODE */  
#define DEFAULT_TARGET_SPEED_RPM2       1500
#define DEFAULT_TORQUE_COMPONENT2       0
#define DEFAULT_FLUX_COMPONENT2         0

/**************************    FIRMWARE PROTECTIONS SECTION   *****************/
#define OV_VOLTAGE_PROT_ENABLING2       ENABLE
#define UV_VOLTAGE_PROT_ENABLING2       ENABLE
#define OV_VOLTAGE_THRESHOLD_V2         32 /*!< Over-voltage 
                                                         threshold */
#define UD_VOLTAGE_THRESHOLD_V2         18 /*!< Under-voltage 
                                                          threshold */
#define ON_OVER_VOLTAGE2                TURN_OFF_PWM /*!< TURN_OFF_PWM, 
                                                         TURN_ON_R_BRAKE or 
                                                         TURN_ON_LOW_SIDES */
#define R_BRAKE_SWITCH_OFF_THRES_V2     26

#define OV_TEMPERATURE_PROT_ENABLING2   ENABLE
#define OV_TEMPERATURE_THRESHOLD_C2     70  /*!< Celsius degrees */
#define OV_TEMPERATURE_HYSTERESIS_C2    10  /*!< Celsius degrees */

#define HW_OV_CURRENT_PROT_BYPASS2      DISABLE /*!< In case ON_OVER_VOLTAGE  
                                                          is set to TURN_ON_LOW_SIDES
                                                          this feature may be used to
                                                          bypass HW over-current
                                                          protection (if supported by 
                                                          power stage) */
/******************************   START-UP PARAMETERS   **********************/
/* Encoder alignment */
#define ALIGNMENT_DURATION2             700 /*!< milliseconds */
#define ALIGNMENT_ANGLE_DEG2            90 /*!< degrees [0...359] */
#define FINAL_I_ALIGNMENT2              23513 /*!< s16A */
// With ALIGNMENT_ANGLE_DEG equal to 90 degrees final alignment 
// phase current = (FINAL_I_ALIGNMENT * 1.65/ Av)/(32767 * Rshunt)  
// being Av the voltage gain between Rshunt and A/D input

/* Sensor-less rev-up sequence */
#define STARTING_ANGLE_DEG2             0  /*!< degrees [0...359] */
/* Phase 1 */
#define PHASE1_DURATION2                0 /*milliseconds */
#define PHASE1_FINAL_SPEED_RPM2         0  /* rpm */
#define PHASE1_FINAL_CURRENT2           0
/* Phase 2 */
#define PHASE2_DURATION2                0   /*milliseconds */
#define PHASE2_FINAL_SPEED_RPM2         0   /* rpm */
#define PHASE2_FINAL_CURRENT2           7234
/* Phase 3 */
#define PHASE3_DURATION2                350   /*milliseconds */
#define PHASE3_FINAL_SPEED_RPM2         630   /* rpm */
#define PHASE3_FINAL_CURRENT2           8440
/* Phase 4 */
#define PHASE4_DURATION2                1150   /*milliseconds */
#define PHASE4_FINAL_SPEED_RPM2         2700  /* rpm */
#define PHASE4_FINAL_CURRENT2           8440
/* Phase 5 */
#define PHASE5_DURATION2                0   /* milliseconds */
#define PHASE5_FINAL_SPEED_RPM2         2700   /* rpm */
#define PHASE5_FINAL_CURRENT2           8440

#define ENABLE_SL_ALGO_FROM_PHASE2      3

/* Observer start-up output conditions  */
#define OBS_MINIMUM_SPEED_RPM2         580
#define NB_CONSECUTIVE_TESTS2          2 /* Corresponding to 
                                                         former NB_CONSECUTIVE_TESTS/
                                                         (TF_REGULATION_RATE/
                                                         MEDIUM_FREQUENCY_TASK_RATE) */
#define SPEED_BAND_UPPER_LIMIT2        17 /*!< It expresses how much 
                                                            estimated speed can exceed 
                                                            forced stator electrical 
                                                            without being considered wrong. 
                                                            In 1/16 of forced speed */
#define SPEED_BAND_LOWER_LIMIT2        16  /*!< It expresses how much 
                                                             estimated speed can be below 
                                                             forced stator electrical 
                                                             without being considered wrong. 
                                                             In 1/16 of forced speed */                        
#define TRANSITION_DURATION2           100  /* Switch over duration, ms */                                                        
/******************************   ADDITIONAL FEATURES   **********************/
#define  BUS_VOLTAGE_READING2          ENABLE

#define  TEMPERATURE_READING2          ENABLE

#define  OPEN_LOOP_FOC2                 DISABLE   /*!< ENABLE for open loop */
#define  OPEN_LOOP_VOLTAGE_d2           6000      /*!< Three Phase voltage amplitude
                                                       in s16 format */
#define  OPEN_LOOP_SPEED_RPM2           100       /*!< Final forced speed in rpm */
#define  OPEN_LOOP_SPEED_RAMP_DURATION_MS2  1000  /*!< 0-to-Final speed ramp duration  */      
#define  OPEN_LOOP_VF2                  FALSE     /*!< TRUE to enable V/F mode */
#define  OPEN_LOOP_K2                   44        /*! Slope of V/F curve expressed in s16 Voltage for 
                                                      each 0.1Hz of mecchanical frequency increment. */
#define  OPEN_LOOP_OFF2                 4400      /*! Offset of V/F curve expressed in s16 Voltage 
                                                      applied when frequency is zero. */

#define  FLUX_WEAKENING_ENABLING2       ENABLE

#define  FW_VOLTAGE_REF2                985 /*!<Vs reference, tenth 
                                                        of a percent */
#define  FW_KP_GAIN2                    3000 /*!< Default Kp gain */
#define  FW_KI_GAIN2                    5000 /*!< Default Ki gain */
#define  FW_KPDIV2                      32768      
                                                /*!< Kp gain divisor.If FULL_MISRA_C_COMPLIANCY
                                                is not defined the divisor is implemented through       
                                                algebrical right shifts to speed up PIs execution. 
                                                Only in this case this parameter specifies the 
                                                number of right shifts to be executed */
#define  FW_KIDIV2                            32768
                                                /*!< Ki gain divisor.If FULL_MISRA_C_COMPLIANCY
                                                is not defined the divisor is implemented through       
                                                algebrical right shifts to speed up PIs execution. 
                                                Only in this case this parameter specifies the 
                                                number of right shifts to be executed */
/*  Feed-forward parameters */
#define FEED_FORWARD_CURRENT_REG_ENABLING2 DISABLE
#define CONSTANT1_Q2                    6510
#define CONSTANT1_D2                    6510
#define CONSTANT2_QD2                   7783

/*  Maximum Torque Per Ampere strategy parameters */
#define MTPA_ENABLING2                 DISABLE
#define IQMAX2                         25322
#define SEGDIV2                        0
#define ANGC2                          {0,0,0,0,0,0,0,0}
#define OFST2                          {0,0,0,0,0,0,0,0}

/* Inrush current limiter parameters */
#define INRUSH_CURRLIMIT_ENABLING2     DISABLE
#define INRUSH_CURRLIMIT_AT_POWER_ON2  INACTIVE  /*ACTIVE or INACTIVE*/
#define INRUSH_CURRLIMIT_CHANGE_AFTER_MS2 1000  /* milliseconds */                

/******************************** DEBUG ADD-ONs *******************************/


/* ##@@_USER_CODE_START_##@@ */
/* ##@@_USER_CODE_END_##@@ */

#endif /*__DRIVE_PARAMETERS_MOTOR2_H*/
/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
