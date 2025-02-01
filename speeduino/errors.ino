/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/

/*
 * Sets the next available error
 * Returns the error number or 0 if there is no more room for errors
 */
#include "globals.h"
#include "errors.h"

uint8_t OBD_Num_DTCs = 0;
uint8_t OBD_DTC_Idx = 0; 
uint16_t OBD_ActiveDTCs = 0x0000;
uint8_t OBD_RunTimer_1s = 0;
uint8_t injectorPressure_DTC_Tmr_1s = 0; 

//This list must match the order of the defined DTC's
const uint16_t OBD_DTC_List[OBD_MAX_DTCS] PROGMEM =
{0x0000,  // Spare
 0x0089,  // P0089
 0x0107,  // P0107
 0x0108,  // P0108
 0x0112,  // P0112
 0x0113,  // P0113
 0x0117,  // P0117
 0x0118,  // P0118
 0x0122,  // P0122
 0x0123,  // P0123
 0x0191,  // P0191
 0x0341,  // P0341
 0x0562,  // P0562
 0x0563,  // P0563
 0x0000,  // Spare
 0x0000   // Spare
 };

/* Sets a defined dtc as active. DTCref is one of the DTCs in a defined list 
* Returns 1 if sucessfull, 0 if list is full*/
uint8_t setDTC(uint8_t DTCref)
{
  if (BIT_CHECK(OBD_ActiveDTCs, DTCref) == true)
  {
    return 1; // DTC is already set
  }
  else if(OBD_Num_DTCs < OBD_MAX_DTCS)
  {
    BIT_SET(currentStatus.spark, BIT_SPARK_MIL); // Turn on Malfunction Indicator Lamp, for emissions DTC.
    BIT_SET(OBD_ActiveDTCs, DTCref);
    OBD_Num_DTCs++;
    return 1;
  }
  else
  {
    return 0;
  }
}

/* Checks a defined dtc as active. DTCref is one of the DTCs in a defined list 
* Returns 1 if active, 0 if not or not found*/
uint8_t checkDTC(uint8_t DTCref)
{
  if(DTCref < OBD_MAX_DTCS)
  {
    return BIT_CHECK(OBD_ActiveDTCs, DTCref);
  }
  else
  {
    return 0;
  }
}

/* Clears all DTC's */ 
void clearAllDTCS(void)
{
  OBD_ActiveDTCs = 0x0000;
  OBD_Num_DTCs = 0;
  BIT_CLEAR(currentStatus.spark, BIT_SPARK_MIL);
}

/* Returns the number of active DTCs */ 
uint8_t getNumDTCS(void)
{
  return OBD_Num_DTCs;
}
  

/* Searches through the list and returns a uint16_t of the active dtc code as per J1979
* Bits A7-A6 = Category 
* 00: P - Powertrain
* 01: C - Chassis
* 10: B - Body
* 11: U - Network 
* 
* A5-B0	Number (within category)
* 
* An example DTC of "U0158" would be decoded as follows:
* 
* Bit	    A7	A6	A5	A4	A3	A2	A1	A0	B7	B6	B5	B4	B3	B2	B1	B0
* Binary	1	  1   0	  0	  0	  0	  0	  1	  0	  1	  0	  1	  1	  0	  0	  0
* Hexadecimal	C	1	5	8
* Decoded DTC	U	0	1	5	8
* 
* Returns DTC if sucessfull, 0 if no more dtcs available*/
uint16_t getNextDTC(uint8_t reset)
{
  uint16_t OBD_ReturnDTC = 0x0000;
  
  if (reset > 0) {OBD_DTC_Idx = 0;} //start from the beginning.

  while(OBD_DTC_Idx < OBD_MAX_DTCS)
  {
    if (BIT_CHECK(OBD_ActiveDTCs, OBD_DTC_Idx) == true) //found active dtc
    {
      OBD_ReturnDTC = (uint16_t)pgm_read_word_near(OBD_DTC_List + OBD_DTC_Idx);
      OBD_DTC_Idx++; //this is remembered between calls to this function
      return OBD_ReturnDTC;
    }
    OBD_DTC_Idx++; //this is remembered between calls to this function
  }
  return OBD_ReturnDTC; //no dtc
}

byte reportDTCs_TS()
{
  packedError currentError;
  byte currentErrorNum = 0;

  if(OBD_Num_DTCs > 0)
  {
    //We alternate through the errors once per second
    //currentErrorNum = currentStatus.secl % errorCount; //Which error number will be returned. This changes once per second. 
    
    currentErrorNum = currentStatus.secl % (OBD_MAX_DTCS-1); //Which error number will be returned. This changes once per second. 

    //currentError.errorNum = currentErrorNum;
    //currentError.errorID = errorCodes[currentErrorNum];
    if (OBD_Num_DTCs <= 4) { currentError.errorNum = OBD_Num_DTCs; }
    else { currentError.errorNum = 4; } // Largest value in 2 bits is 4.
    if (BIT_CHECK(OBD_ActiveDTCs, currentErrorNum) == true) // Returns DTC if active, otherwise 0. Takes OBD_MAX_DTCS sec to cycle though all possible DTCs
    {
      currentError.errorID = (uint16_t)pgm_read_word_near(OBD_DTC_List + currentErrorNum);
    }
    else
    {
      currentError.errorID = 0;
    }
  }
  else
  {
    currentError.errorNum = 0;
    currentError.errorID = 0;
  }


  return *(byte*)&currentError; //Ugly, but this forces the cast of the currentError struct to a byte.
}

//This function checks to see if we need to set a DTC every sec
void DTCSetter_1000ms(void)
{
  if (OBD_RunTimer_1s < 5) // 5 sec timer since power on
  {
    OBD_RunTimer_1s++;
  }
  else
  {
    BIT_SET(currentStatus.OBD_DTC_Ready, OBD_READY_TIME);
  }
  
  // Syncloss is P0341 Camshaft Position Sensor Circuit Range/Performance
  if (BIT_CHECK(currentStatus.OBD_DTC_Ready, OBD_READY_BATT))
  {
    if (currentStatus.syncLossCounter > 20) // Syncloss > 20 
    {
      setDTC(BIT_DTC_P0341); // Camshaft Position Sensor Circuit Range/Performance
    }
    if ((BIT_CHECK(currentStatus.engine, BIT_ENGINE_RUN) == false) && (BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) == false))
    {
      // not running or cranking, reset syncloss after 1 sec otherwise multiple stalls and starts without cycling the controller power will eventually set the DTC
      currentStatus.syncLossCounter = 0; // reset after not running
    }
  }
  
  // P0089 Fuel pressure performance, note seperate to error codes on the fuel pressure sender.
  if (BIT_CHECK(currentStatus.OBD_DTC_Ready, OBD_READY_BATT) && BIT_CHECK(currentStatus.OBD_DTC_Ready, OBD_READY_RUNNING))
  {
    if ((currentStatus.InjectorDeltaPress < (configPage10.fPress_RefPress - 100)) || (currentStatus.InjectorDeltaPress > (configPage10.fPress_RefPress + 100))) // 1 Bar Variation
    {
      injectorPressure_DTC_Tmr_1s++; // update error timer
      
      if (injectorPressure_DTC_Tmr_1s > 3) // 3 consecutive sec out of range.
      {
        setDTC(BIT_DTC_P0089);	//Fuel Pressure Regulator Performance
      }
    }
    else
    {
      injectorPressure_DTC_Tmr_1s = 0; // ok reset timer.
    }
  }
}
