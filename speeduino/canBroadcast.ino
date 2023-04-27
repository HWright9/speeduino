/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/

/*
This is for handling the data broadcasted to various CAN dashes and instrument clusters.
*/
#if defined(NATIVE_CAN_AVAILABLE)
#include "globals.h"

uint8_t canO2TimeSinceLast;
uint8_t canO22TimeSinceLast;
uint8_t canEPBTimeSinceLast;

void sendBMWCluster()
{
  DashMessage(CAN_BMW_DME1);
  Can0.write(outMsg);
  DashMessage(CAN_BMW_DME2);
  Can0.write(outMsg);
  DashMessage(CAN_BMW_DME4);
  Can0.write(outMsg);
}

void sendVAGCluster()
{
  DashMessage(CAN_VAG_RPM);
  Can0.write(outMsg);
  DashMessage(CAN_VAG_VSS);
  Can0.write(outMsg);
}

// switch case for gathering all data to message based on CAN Id.
void DashMessage(uint16_t DashMessageID)
{
  switch (DashMessageID)
  {
    case CAN_BMW_DME1:
      outMsg.id = DashMessageID;
      outMsg.len = 8;
      outMsg.buf[0] = 0x05;  //bitfield, Bit0 = 1 = terminal 15 on detected, Bit2 = 1 = the ASC message ASC1 was received within the last 500 ms and contains no plausibility errors
      outMsg.buf[1] = 0x0C;  //Indexed Engine Torque in % of C_TQ_STND TBD do torque calculation.
      outMsg.buf[2] = 0x00;  //lsb RPM
      outMsg.buf[3] = currentStatus.RPM / 40; //msb RPM. RPM conversion is currentStatus.RPM * 6.4, but this does close enough without floats.
      outMsg.buf[4] = 0x0C;  //Indicated Engine Torque in % of C_TQ_STND TBD do torque calculation!! Use same as for byte 1
      outMsg.buf[5] = 0x15;  //Engine Torque Loss (due to engine friction, AC compressor and electrical power consumption)
      outMsg.buf[6] = 0x00;  //not used
      outMsg.buf[7] = 0x35;  //Theorethical Engine Torque in % of C_TQ_STND after charge intervention
    break;

    case CAN_BMW_DME2:
      uint8_t temp_TPS;
      uint8_t temp_BARO;
      int16_t temp_CLT;
      temp_TPS = map(currentStatus.TPS, 0, 100, 0, 254);//TPS value conversion (from 0x00 to 0xFE)
      temp_CLT = (((currentStatus.coolant - CALIBRATION_TEMPERATURE_OFFSET) + 48)*4/3); //CLT conversion (actual value to add is 48.373, but close enough)
      if (temp_CLT > 255) { temp_CLT = 255; } //CLT conversion can yield to higher values than what fits to byte, so limit the maximum value to 255.
      temp_BARO = currentStatus.baro;

      outMsg.id = DashMessageID;
      outMsg.len = 7;
      outMsg.buf[0] = 0x11;  //Multiplexed Information
      outMsg.buf[1] = temp_CLT;
      outMsg.buf[2] = temp_BARO;
      outMsg.buf[3] = 0x08;  //bitfield, Bit0 = 0 = Clutch released, Bit 3 = 1 = engine running
      outMsg.buf[4] = 0x00;  //TPS_VIRT_CRU_CAN (Not used)
      outMsg.buf[5] = temp_TPS;
      outMsg.buf[6] = 0x00;  //bitfield, Bit0 = 0 = brake not actuated, Bit1 = 0 = brake switch system OK etc...
      outMsg.buf[7] = 0x00;  //not used, but set to zero just in case.
    break;

    case 0x545:       //fuel consumption and CEl light for BMW e46/e39/e38 instrument cluster
                      //fuel consumption calculation not implemented yet. But this still needs to be sent to get rid of the CEL and EML fault lights on the dash.
      outMsg.id = DashMessageID;
      outMsg.len = 5;
      outMsg.buf[0] = 0x00;  //Check engine light (binary 10), Cruise light (binary 1000), EML (binary 10000).
      outMsg.buf[1] = 0x00;  //LSB Fuel consumption
      outMsg.buf[2] = 0x00;  //MSB Fuel Consumption
      if (currentStatus.coolant > 159) { outMsg.buf[3] = 0x08; } //Turn on overheat light if coolant temp hits 120 degrees celsius.
      else { outMsg.buf[3] = 0x00; } //Overheat light off at normal engine temps.
      outMsg.buf[4] = 0x7E; //this is oil temp
    break;

    case 0x280:       //RPM for VW instrument cluster
      int16_t temp_RPM;
      temp_RPM =  currentStatus.RPM * 4; //RPM conversion
      outMsg.id = DashMessageID;
      outMsg.len = 8;
      outMsg.buf[0] = 0x49;
      outMsg.buf[1] = 0x0E;
      outMsg.buf[2] = lowByte(temp_RPM);
      outMsg.buf[3] = highByte(temp_RPM);
      outMsg.buf[4] = 0x0E;
      outMsg.buf[5] = 0x00;
      outMsg.buf[6] = 0x1B;
      outMsg.buf[7] = 0x0E;
    break;

    case 0x5A0:       //VSS for VW instrument cluster
      uint16_t temp_VSS;
      temp_VSS =  currentStatus.vss * 133; //VSS conversion
      outMsg.id = DashMessageID;
      outMsg.len = 8;
      outMsg.buf[0] = 0xFF;
      outMsg.buf[1] = lowByte(temp_VSS);
      outMsg.buf[2] = highByte(temp_VSS);
      outMsg.buf[3] = 0x00;
      outMsg.buf[4] = 0x00;
      outMsg.buf[5] = 0x00;
      outMsg.buf[6] = 0x00;
      outMsg.buf[7] = 0xAD;
    break;

    default:
    break;
  }
}
#endif

#if defined CAN_AVR_MCP2515
#include "globals.h"
#include <mcp2515.h>

// Broadcasts Speeduino Generic data on CAN. Compatible with data dictionary v0.1
uint8_t sendCAN_Speeduino_10Hz()
{
  uint8_t canErrCode = MCP2515::ERROR_FAILTX;
  
  canTx_EngineSensor1(); canErrCode = mcp2515.sendMessage(&canMsg);
  canTx_EnginePosition1(); canErrCode = mcp2515.sendMessage(&canMsg);
  canTx_VehicleSpeed1(); canErrCode = mcp2515.sendMessage(&canMsg);
    
  return canErrCode;
}

// Recieves Speeduino data on CAN. Compatible with data dictionary v0.1
uint8_t recieveCAN_BroadCast()
{
  uint8_t canErrCode = MCP2515::ERROR_FAILTX;
  
  canErrCode = mcp2515.readMessage(&canMsg);
  
  
  if(canErrCode == MCP2515::ERROR_OK) 
  { 
      // O2 sensors using motec PLM's
    if (configPage6.egoType == 2) // O2 sensor source set to read CAN
    {
      canRx_MotecPLM_O2(&canMsg, 0x460); // Parse Motec PLM message at ID 460 for O2
      canRx_MotecPLM_O22(&canMsg, 0x461); // Parse Motec PLM message at ID 461 for 2nd O2
    }
    
    if (configPage2.vssMode == 1) // 1 is RX over CAN
    {
      canRx_EPB_Vss(&canMsg, 0x470); // Parse Electric Park Brake data on 470. This has vss gear and clutch data from EPB.
    }
      
  }
  else if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_10HZ)) // Time out variables to check if messages not recieved, sec x10
  { 
    if (canO2TimeSinceLast < 255) { canO2TimeSinceLast++; }
    if (canO22TimeSinceLast < 255) { canO22TimeSinceLast++; }
    if (canEPBTimeSinceLast < 255) { canEPBTimeSinceLast++; }
  }  
  
  // Timeout error handling
  if ((configPage6.egoType == 2) && (canO2TimeSinceLast > 10)) { canRx_EPB_Vss_Dflt(); }
  if ((configPage6.egoType == 2) && (canO22TimeSinceLast > 10)) { canRx_MotecPLM_O22_Dflt(); }
  if ((configPage2.vssMode == 1) && (canEPBTimeSinceLast > 20)) { canRx_EPB_Vss_Dflt(); }
  
  return canErrCode;
}

// Builds engine sensor status 1 msg on CAN Id 401 in struct canMsg, ready for sending
void canTx_EngineSensor1()
{
  canMsg.can_id  = 0x401;
  canMsg.can_dlc = 8;
  
  canMsg.data[0] = currentStatus.TPS;  //X * 0.5
  canMsg.data[1] = highByte(currentStatus.MAP); //X
  canMsg.data[2] = lowByte(currentStatus.MAP); //X
  canMsg.data[3] = currentStatus.baro; //X
  canMsg.data[4] = currentStatus.IAT; //X - 40
  canMsg.data[5] = currentStatus.coolant;//X - 40
  canMsg.data[6] = currentStatus.battery10; //X * 0.1
  canMsg.data[7] = currentStatus.ethanolPct; //X
  
}

void canTx_EnginePosition1()
{
  canMsg.can_id  = 0x402;
  canMsg.can_dlc = 7;
  
  canMsg.data[0] = highByte(currentStatus.RPM); //X
  canMsg.data[1] = lowByte(currentStatus.RPM); //X
  canMsg.data[2] = currentStatus.engine; //bitfield
  canMsg.data[3] = currentStatus.spark; //bitfield
  canMsg.data[4] = currentStatus.advance; //X
  canMsg.data[5] = currentStatus.idleLoad; // X
  canMsg.data[6] = currentStatus.CLIdleTarget; // X * 10
  canMsg.data[7] = 0xFF;
  
}

void canTx_VehicleSpeed1()
{
  canMsg.can_id  = 0x403;
  canMsg.can_dlc = 3;
  
  canMsg.data[0] = highByte(currentStatus.vss); //X
  canMsg.data[1] = lowByte(currentStatus.vss); //X
  canMsg.data[2] = currentStatus.gear; // Enum
  canMsg.data[3] = 0xFF;
  canMsg.data[4] = 0xFF;
  canMsg.data[5] = 0xFF;
  canMsg.data[6] = 0xFF;
  canMsg.data[7] = 0xFF;
  
}

/* Recieve MotecPLM Can message frame on defined CAN ID */
void canRx_MotecPLM_O2 (struct can_frame *canRxMsg, canid_t canRXId)
{
  if ((canRxMsg->can_id == canRXId) && (canRxMsg->can_dlc == 8)) // Check msg on correct address and data length is correct
  {
    canO2TimeSinceLast = 0; //reset timeout 
    
    //byte0 Compound ID
    
    // Check O2 data is valid using sensor status
    if (canRxMsg->data[7] == 0x00)
    {
      //byte1 and 2 Calibrated Sensor Output Value Hi:lo*1 = x.xxxLa
      uint32_t result = (canRxMsg->data[1] << 8) | canRxMsg->data[2]; //(highByte << 8) | lowByte - this is EQR from PLM
      
      if (result < 600 ) { currentStatus.O2 = 255; } // catch divide by 0 and overflow later on.
      else { currentStatus.O2 = (uint8_t)(147000 / result); }//afr
    }
    else { currentStatus.O2 = 255; } // not valid sensor reading

      
    //byte3 Heater duty cycle Byte*1 = xxx%

    //byte4 Device Internal Temperature Byte*195/10-500 = xxx.xC

    //byte5 Zp (Pump cell impedance) Byte*1 = X ohm

    //byte6 Diagnostic Field 1

    //byte7 sensor state
    /*
    switch (canRxMsg->data[7])
    {
      case (0x00):
      EQRLH_State = e_EQRState_RUN;
      break;
      
      case (0x01):
      EQRLH_State = e_EQRState_CONTROL_WAIT;
      break;

      case (0x02):
      EQRLH_State = e_EQRState_PUMP_WAIT;
      break;

      case (0x03):
      EQRLH_State = e_EQRState_WARM_UP;
      break;

      case (0x04):
      EQRLH_State = e_EQRState_NO_HEATER;
      break;

      case (0x05):
      EQRLH_State = e_EQRState_STOP;
      break;

      case (0x06):
      EQRLH_State = e_EQRState_PUMP_OFF;
      break;

      default:
      EQRLH_State = e_EQRState_STOP;
      break;
    }
    */
  }
}

// Default action when message times out
void canRx_MotecPLM_O2_Dflt(void)
{
  currentStatus.O2 = 255;
}

/* Recieve MotecPLM Can message frame on defined CAN ID */
void canRx_MotecPLM_O22 (struct can_frame *canRxMsg, canid_t canRXId)
{
  if ((canRxMsg->can_id == canRXId) && (canRxMsg->can_dlc == 8)) // Check msg on correct address and data length is correct
  {
    canO22TimeSinceLast = 0; //reset timeout 
    
    //byte0 Compound ID
    
    // Check O2 data is valid using sensor status
    if (canRxMsg->data[7] == 0x00)
    {
      //byte1 and 2 Calibrated Sensor Output Value Hi:lo*1 = x.xxxLa
      uint32_t result = (canRxMsg->data[1] << 8) | canRxMsg->data[2]; //(highByte << 8) | lowByte - this is EQR from PLM
      
      if (result < 600 ) { currentStatus.O2_2 = 255; } // catch divide by 0 and overflow later on.
      else { currentStatus.O2_2 = (uint8_t)(147000 / result); }//afr
    }
    else { currentStatus.O2_2 = 255; } // not valid sensor reading
      
    //byte3 Heater duty cycle Byte*1 = xxx%

    //byte4 Device Internal Temperature Byte*195/10-500 = xxx.xC

    //byte5 Zp (Pump cell impedance) Byte*1 = X ohm

    //byte6 Diagnostic Field 1

    //byte7 sensor state
    /*
    switch (canRxMsg->data[7])
    {
      case (0x00):
      EQRLH_State = e_EQRState_RUN;
      break;
      
      case (0x01):
      EQRLH_State = e_EQRState_CONTROL_WAIT;
      break;

      case (0x02):
      EQRLH_State = e_EQRState_PUMP_WAIT;
      break;

      case (0x03):
      EQRLH_State = e_EQRState_WARM_UP;
      break;

      case (0x04):
      EQRLH_State = e_EQRState_NO_HEATER;
      break;

      case (0x05):
      EQRLH_State = e_EQRState_STOP;
      break;

      case (0x06):
      EQRLH_State = e_EQRState_PUMP_OFF;
      break;

      default:
      EQRLH_State = e_EQRState_STOP;
      break;
    }
    */
  }
}

// Default action when message times out
void canRx_MotecPLM_O22_Dflt(void)
{
  currentStatus.O2_2 = 255;
}

/* Recieve EPB data on 470 */
void canRx_EPB_Vss (struct can_frame *canRxMsg, canid_t canRXId)
{
  if ((canRxMsg->can_id == canRXId) && (canRxMsg->can_dlc == 8)) // Check msg on correct address and data length is correct
  {
    canEPBTimeSinceLast = 0; //reset timeout 
    
    // Bytes 0 and 1 are vss X
    currentStatus.vss = (canRxMsg->data[1] << 8) | canRxMsg->data[2]; //(highByte << 8) | lowByte
    if (currentStatus.vss > 512) { currentStatus.vss = 512; } //basic error checking.

  
  // Byte 2 is gear
  // Byte 3 is reverse gearbox state, need to resolve both of them.
  currentStatus.gear = canRxMsg->data[3]; // Raw gear from EPB.
  if ( currentStatus.gear = 6) { currentStatus.gear = 0; } // 0 and 6 are both neutral in speeduino gear logic.
  if ((canRxMsg->data[4] == 0) && (currentStatus.gear >= 2 )){ currentStatus.gear = 1; } // Reverse gearbox is in reverse, limit speeduino gear to 1st
  else if (canRxMsg->data[4] >= 2){ currentStatus.gear = 0; } // All other states of reverse gearbox are neutral

  // Read clutch trigger bit as the inverse of "clutch is top travel from EPB"
  clutchTrigger = !(canRxMsg->data[4] & 0b00000010); // Bit 1 inverted.
  // Other messages not read 
  }

}

// Default action when message times out
void canRx_EPB_Vss_Dflt(void)
{
  currentStatus.vss = 0;
  currentStatus.gear = 0;
  clutchTrigger = 0;
}

#endif
