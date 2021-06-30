#ifndef MOTOR_PROFILER
#define MOTOR_PROFILER
#define SPEED_REGULATOR_BANDWIDTH 0 // Dummy value

#if (PWBDID == 2) // X-NUCLEO-IHM07M1
#define CALIBRATION_FACTOR            1.50
#define BUS_VOLTAGE_CONVERSION_FACTOR 63.2
#define CURRENT_REGULATOR_BANDWIDTH     6000
#define MP_KP 10.00f
#define MP_KI 0.1f
#endif

#if (PWBDID == 4) // X-NUCLEO-IHM08M1
#define CALIBRATION_FACTOR            0.02
#define BUS_VOLTAGE_CONVERSION_FACTOR 63.2
#define CURRENT_REGULATOR_BANDWIDTH      6000
#define MP_KP 1.00f
#define MP_KI 0.01f
#endif

#if (PWBDID == 6) // STEVAL-IHM023V3         LV
#define CALIBRATION_FACTOR            	 0.67            
#define BUS_VOLTAGE_CONVERSION_FACTOR 	 447.903
#define CURRENT_REGULATOR_BANDWIDTH      6000
#define MP_KP 5.00f
#define MP_KI 0.05f
#endif

#if (PWBDID == 8) // STEVAL-IHM028V2    3Sh
#define CALIBRATION_FACTOR            	 0.3            
#define BUS_VOLTAGE_CONVERSION_FACTOR 	 415.800
#define CURRENT_REGULATOR_BANDWIDTH      2000
#define MP_KP 5.00f
#define MP_KI 0.05f
#endif

#if (PWBDID == 10) // STEVAL-IPM05F 3 Sh
#define CALIBRATION_FACTOR            	 0.5            
#define BUS_VOLTAGE_CONVERSION_FACTOR 	 415.800
#define CURRENT_REGULATOR_BANDWIDTH      3000
#define MP_KP 5.00f
#define MP_KI 0.05f
#endif

#if (PWBDID == 16) // STEVAL-IPM10B 3 Sh
#define CALIBRATION_FACTOR            	 0.5            
#define BUS_VOLTAGE_CONVERSION_FACTOR 	 415.800
#define CURRENT_REGULATOR_BANDWIDTH      2000
#define MP_KP 5.00f
#define MP_KI 0.05f
#endif

#if (PWBDID == 18) // STEVAL-IPM15B 3 Sh
#define CALIBRATION_FACTOR            	 0.5           
#define BUS_VOLTAGE_CONVERSION_FACTOR 	 415.800
#define CURRENT_REGULATOR_BANDWIDTH      2000
#define MP_KP 1.66f
#define MP_KI 0.01f
#endif

#if (PWBDID == 24) // STEVAL-IHM025V1    3Sh
#define CALIBRATION_FACTOR            	 0.90            
#define BUS_VOLTAGE_CONVERSION_FACTOR 	 415.800
#define CURRENT_REGULATOR_BANDWIDTH      2000
#define MP_KP 5.00f
#define MP_KI 0.05f
#endif

// STM3210B-MCKIT - MB459 3sh LV, 3sh HV respectively
#if (PWBDID == 26) ||  (PWBDID == 28)
#define CALIBRATION_FACTOR            	 0.90            
#define BUS_VOLTAGE_CONVERSION_FACTOR 	 382.800
#define CURRENT_REGULATOR_BANDWIDTH      6000
#define MP_KP 12.00f
#define MP_KI 0.12f
#endif

// STM3210B-MCKIT - MB459 1sh LV, 1sh HV respectively
// eventualmente fornire MP 1 sh solo per HV e non per LV
#if (PWBDID == 27)
#define CALIBRATION_FACTOR            	 0.64            
#define BUS_VOLTAGE_CONVERSION_FACTOR 	 382.800
#define CURRENT_REGULATOR_BANDWIDTH      8000
#define MP_KP 12.00f
#define MP_KI 0.12f
#endif

#if (PWBDID == 30) // STEVAL-IHM023V3    3Sh     HV
#define CALIBRATION_FACTOR            	 0.67            
#define BUS_VOLTAGE_CONVERSION_FACTOR 	 447.903
#define CURRENT_REGULATOR_BANDWIDTH      2000
#define MP_KP 5.00f
#define MP_KI 0.05f
#endif




#endif
/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
