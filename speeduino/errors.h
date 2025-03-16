#ifndef ERRORS_H
#define ERRORS_H

/*
 * Up to 64 different error codes may be defined (6 bits)
 */

//DTC's 0-15
#define BIT_DTC_U0002   0  // High Speed CAN Communication Bus Performance
#define BIT_DTC_P0089   1  // Fuel Pressure Regulator Performance
#define BIT_DTC_P0107   2  // Manifold Absolute Pressure/Barometric Pressure Circuit Low Input
#define BIT_DTC_P0108   3  // Manifold Absolute Pressure/Barometric Pressure Circuit High Input
#define BIT_DTC_P0112   4	 // Intake Air Temperature Circuit Low Input
#define BIT_DTC_P0113   5	 // Intake Air Temperature Circuit High Input
#define BIT_DTC_P0117   6	 // Engine Coolant Temperature Circuit Low Input
#define BIT_DTC_P0118   7	 // Engine Coolant Temperature Circuit High Input
#define BIT_DTC_P0122   8	 // Throttle/Pedal Position Sensor/Switch A Circuit Low Input
#define BIT_DTC_P0123   9	 // Throttle/Pedal Position Sensor/Switch A Circuit High Input
#define BIT_DTC_P0191	  10 // Fuel Rail Pressure Sensor Circuit Range/Performance
#define BIT_DTC_P0341   11 // Camshaft Position Sensor Circuit Range/Performance
#define BIT_DTC_P0562	  12 // System Voltage Low
#define BIT_DTC_P0563   13 // System Voltage High
#define BIT_DTC_XX1	    14 // Spare
#define BIT_DTC_XX2	    15 // Spare


#define ERR_DEFAULT_IAT_SHORT   65 //Note that the default is 25C. 65 is used due to the -40 offset
#define ERR_DEFAULT_IAT_GND     65 //Note that the default is 25C. 65 is used due to the -40 offset
#define ERR_DEFAULT_CLT_SHORT   80 //Note that the default is 40C. 80 is used due to the -40 offset
#define ERR_DEFAULT_CLT_GND     80 //Note that the default is 40C. 80 is used due to the -40 offset
#define ERR_DEFAULT_O2_Fault    147 //14.7
#define ERR_DEFAULT_TPS_Fault   50 //25%
#define ERR_DEFAULT_MAP_HIGH    240
#define ERR_DEFAULT_MAP_LOW     80


#define OBD_MAX_DTCS  16 //The number of DTCs in memory

#define OBD_ENBL_AFTERSTARTTMR 5 // 5 sec after start.

/*
 * This struct is a single byte in length and is sent to TS
 * The first 2 bits are used to define the current error (0-3)
 * The remaining 6 bits are used to give the error number
 */
struct packedError
{
  byte errorNum : 2;
  byte errorID  : 6;
};

uint8_t setDTC(uint8_t DTCref);
uint8_t checkDTC(uint8_t DTCref);
void clearAllDTCS(void);
uint8_t getNumDTCS(void);
uint16_t getNextDTC(uint8_t reset);
uint8_t reportDTCs_TS();
void DTCSetter_1000ms(void);

#endif
