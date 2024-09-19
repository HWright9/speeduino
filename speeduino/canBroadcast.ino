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

MCP_CAN CAN0(CAN0_CS);      // Set MCP_CAN CAN0 instance CS to pin 53

/*Variables Local to this function*/
uint8_t len = 0;
uint32_t CANrxId;
uint8_t CAN_Tx_Msgdata[8]; // Used by both TX routines.
uint8_t CAN_Rx_Msgdata[8]; // Used by both RX routines.
uint8_t CAN_ErrorTmr = 0;

uint8_t canO2TimeSinceLast;
uint8_t canO22TimeSinceLast;
uint8_t canEPBTimeSinceLast;

uint8_t canMSGSeq = 0; // Message sequencer

void can0_Init(void)
{
  uint8_t canErr = CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ); // init can bus : baudrate = CAN_500KBPS, frequency MCP_8MHZ
  
  if(canErr == CAN_OK)  
  {
    /*
    CAN0.init_Mask(0,0,0x07F00000);                // Init first mask... looking at first two bytes of ID
    CAN0.init_Filt(0,0,0x04700000);                // Init first filter... Only allow  0x47x
    CAN0.init_Filt(1,0,0x04600000);                // Init second filter... Only allow  0x46x
    */
    CAN0.setMode(MCP_NORMAL);
    BIT_CLEAR(currentStatus.status4, BIT_STATUS4_CAN_ERROR);
  }
  else
  {
    BIT_SET(currentStatus.status4, BIT_STATUS4_CAN_ERROR);
    CAN_ErrorTmr = 0;
  }
    
}

void can0_Maintainance(void)
{
  if (configPage2.enableAeroSSCAN == true)
  {
    if((BIT_CHECK(currentStatus.status4, BIT_STATUS4_CAN_ERROR)) && (CAN_ErrorTmr < 254) ) { CAN_ErrorTmr++; }
    if (CAN_ErrorTmr > 2)
    {
      CAN0.abortTX(); //flush transmit buffer
      can0_Init();
    }
  }
  else 
  {
    BIT_CLEAR(currentStatus.status4, BIT_STATUS4_CAN_ERROR);
    CAN_ErrorTmr = 0;
  }
}

// Broadcasts Speeduino Generic data on CAN. Compatible with data dictionary v0.2
uint8_t sendCAN_Speeduino_100Hz(void)
{
  uint8_t canErr = CAN0.checkError();
  
  // To limit throughput each message is sent at 100hz/3 with even spacing.
  if((canErr != CAN_FAIL) && (canMSGSeq == 0)) { canErr = canTx_EngineSensor1(); }
  if((canErr != CAN_FAIL) && (canMSGSeq == 1)) { canErr = canTx_EnginePosition1(); }
  if((canErr != CAN_FAIL) && (canMSGSeq == 2)) { canErr = canTx_EngineActuator1(); }
  //if(canErrCode != CAN_FAIL) { canErr = canTx_VehicleSpeed1(); }
  
  //Error Handling
  //Serial.print(" canErr: "); Serial.print(canErr,HEX);
  if (canErr != CAN_OK)  // CAN error, may be TX or RX or Controller related
  {
    //canPrintErrors(canErr);     //debug only 
    if(canErr == CAN_FAIL) // No comms with controller
    {
      BIT_SET(currentStatus.status4, BIT_STATUS4_CAN_ERROR);
      CAN_ErrorTmr = 0;
    }
    else if ((canErr == CAN_GETTXBFTIMEOUT) || (canErr == CAN_SENDMSGTIMEOUT))
    {
      CAN0.abortTX(); // clear tx buffer
      BIT_SET(currentStatus.status5, BIT_STATUS5_CAN_RXEPBDFLT);
      BIT_SET(currentStatus.status4, BIT_STATUS4_CAN_ERROR);
      CAN_ErrorTmr = 0;
    }
    else if(canErr == CAN_CTRLERROR)
    {
      // Recoverable bus error, record error values.
      CAN0.abortTX(); // clear tx buffer
      currentStatus.status5 = CAN0.getError() & 0b11111000; // ignore the last 3 warning bits in this buffer.
      //Serial.println(" CAN_RX0BUFFOVERFLOW ");
    }
    else // some other error
    {
      BIT_SET(currentStatus.status4, BIT_STATUS4_CAN_ERROR);
      CAN_ErrorTmr = 0;
    }          
  } 
  else 
  { 
    BIT_CLEAR(currentStatus.status4, BIT_STATUS4_CAN_ERROR);
    currentStatus.status5 = 0x00 & 0b11111000; // clear error flags     
  }
  
  if (canMSGSeq < 2) { canMSGSeq++; }
  else { canMSGSeq = 0; }
    
  return canErr;
}

// Recieves Speeduino data on CAN. Compatible with data dictionary v0.1
uint8_t recieveCAN_BroadCast(void)
{
  uint8_t canErr = CAN_OK;
  
  while((digitalRead(pinCANInt) == false) && (CAN0.checkReceive() == CAN_MSGAVAIL)) // Digital read CAN INT pin on pin 2 is low
  {  
    canErr = CAN0.readMsgBuf(&CANrxId, &len, CAN_Rx_Msgdata);
    
    if ((canErr == CAN_OK) && ((CANrxId & 0x80000000) != 0x80000000))  // alternate would be CAN_NOMSG, also not extended frame
    {
        // O2 sensors using motec PLM's
      if (configPage6.egoType == 2) // O2 sensor source set to read CAN
      {
        if ((CANrxId == 0x460) && (len == 8)) { canRx_MotecPLM_O2(); }// Parse Motec PLM message at ID 460 for O2
        if ((CANrxId == 0x461) && (len == 8)) { canRx_MotecPLM_O22; } // Parse Motec PLM message at ID 461 for 2nd O2
      }
      
      if (configPage2.vssMode == 1) // 1 is RX over CAN
      {
        if ((CANrxId == 0x470) && (len == 8)) { canRx_EPB_Vss(); } // Parse Electric Park Brake data on 470. This has vss gear and clutch data from EPB.
      }
      
      if ((CANrxId == 0x472) && (len == 8)) { canRx_EPBAccelGyro1(); } // Parse Electric Park Brake data on 472. This is the 3 axis gyro
    }
  }
  
  if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_10HZ)) // Time out variables to check if messages not recieved, sec x10
  { 
    if (canO2TimeSinceLast < 255) { canO2TimeSinceLast++; }
    if (canO22TimeSinceLast < 255) { canO22TimeSinceLast++; }
    if (canEPBTimeSinceLast < 255) { canEPBTimeSinceLast++; }
    
    // Timeout error handling
    if ((configPage6.egoType == 2) && (canO2TimeSinceLast > 10)) { canRx_MotecPLM_O2_Dflt(); }
    if ((configPage6.egoType == 2) && (canO22TimeSinceLast > 10)) { canRx_MotecPLM_O22_Dflt();}
    if ((configPage2.vssMode == 1) && (canEPBTimeSinceLast > 10)) { canRx_EPB_Vss_Dflt();  canRx_EPBAccelGyro1_Dflt();}
  }  
  
  //Error handling
  if(canErr == CAN_FAIL) // No comms with controller
      {
        BIT_SET(currentStatus.status4, BIT_STATUS4_CAN_ERROR);
        CAN_ErrorTmr = 0;
      }
      
  return canErr;
}

// Builds and sends engine sensor status 1 msg on CAN Id 401
uint8_t canTx_EngineSensor1(void)
{
  CAN_Tx_Msgdata[0] = currentStatus.TPS;  //X * 0.5
  CAN_Tx_Msgdata[1] = highByte(currentStatus.MAP); //X
  CAN_Tx_Msgdata[2] = lowByte(currentStatus.MAP); //X
  CAN_Tx_Msgdata[3] = currentStatus.baro; //X
  CAN_Tx_Msgdata[4] = (uint8_t)(currentStatus.IAT + CALIBRATION_TEMPERATURE_OFFSET); //X - 40
  CAN_Tx_Msgdata[5] = (uint8_t)(currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET);//X - 40
  CAN_Tx_Msgdata[6] = currentStatus.battery10; //X * 0.1
  CAN_Tx_Msgdata[7] = currentStatus.ethanolPct; //X
  
  //CANStat = CAN0.sendMsgBuf(theaddress, 0, 8, thedata);
  return CAN0.sendMsgBuf(0x401, 0, 8, CAN_Tx_Msgdata);
}

// Builds and sends engine position status 1 msg on CAN Id 402
uint8_t canTx_EnginePosition1(void)
{
  CAN_Tx_Msgdata[0] = highByte(currentStatus.RPM); //X
  CAN_Tx_Msgdata[1] = lowByte(currentStatus.RPM); //X
  CAN_Tx_Msgdata[2] = currentStatus.engine; //bitfield
  CAN_Tx_Msgdata[3] = currentStatus.spark; //bitfield
  CAN_Tx_Msgdata[4] = currentStatus.advance; //X
  CAN_Tx_Msgdata[5] = currentStatus.idleLoad; // X
  CAN_Tx_Msgdata[6] = currentStatus.CLIdleTarget; // X * 10
  CAN_Tx_Msgdata[7] = currentStatus.engineProtectStatus; //bitfield
  
  return CAN0.sendMsgBuf(0x402, 0, 8, CAN_Tx_Msgdata);
}

// Builds and sends engine actuator status 1 msg on CAN Id 403
uint8_t canTx_EngineActuator1(void)
{
  
  CAN_Tx_Msgdata[0] = highByte(currentStatus.PW1); //X * 0.001
  CAN_Tx_Msgdata[1] = lowByte(currentStatus.PW1); //X * 0.001
  CAN_Tx_Msgdata[2] = highByte(currentStatus.PW2); //X * 0.001
  CAN_Tx_Msgdata[3] = lowByte(currentStatus.PW2); //X * 0.001
  CAN_Tx_Msgdata[4] = currentStatus.afrTarget; //X * 0.1
  CAN_Tx_Msgdata[5] = highByte(currentStatus.fuelUsedThisKey); //X * 0.001
  CAN_Tx_Msgdata[6] = lowByte(currentStatus.fuelUsedThisKey); //X * 0.001
  CAN_Tx_Msgdata[7] = 0xFF; 
  
  return CAN0.sendMsgBuf(0x403, 0, 8, CAN_Tx_Msgdata);
}

// Builds and sends vehicle speed status 1 msg on CAN Id 410
uint8_t canTx_VehicleSpeed1(void)
{
  
  CAN_Tx_Msgdata[0] = highByte(currentStatus.vss); //X
  CAN_Tx_Msgdata[1] = lowByte(currentStatus.vss); //X
  CAN_Tx_Msgdata[2] = currentStatus.gear; // Enum
  CAN_Tx_Msgdata[3] = clutchTrigger; // 0 or 1
  CAN_Tx_Msgdata[4] = 0xFF;
  CAN_Tx_Msgdata[5] = 0xFF;
  CAN_Tx_Msgdata[6] = 0xFF;
  CAN_Tx_Msgdata[7] = 0xFF;
  
  return CAN0.sendMsgBuf(0x410, 0, 8, CAN_Tx_Msgdata);
}

/* Recieve MotecPLM Can message frame on defined CAN ID */
void canRx_MotecPLM_O2 (void)
{
  canO2TimeSinceLast = 0; //reset timeout 
  
  //byte0 Compound ID
  
  // Check O2 data is valid using sensor status
  if (CAN_Rx_Msgdata[7] == 0x00)
  {
    //byte1 and 2 Calibrated Sensor Output Value Hi:lo*1 = x.xxxLa
    uint32_t result = (CAN_Rx_Msgdata[1] << 8) | CAN_Rx_Msgdata[2]; //(highByte << 8) | lowByte - this is EQR from PLM
    
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
  switch (CAN_Rx_Msgdata[7])
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

// Default action when message times out
void canRx_MotecPLM_O2_Dflt(void)
{
  currentStatus.O2 = 255;
}

/* Recieve MotecPLM Can message frame on defined CAN ID */
void canRx_MotecPLM_O22 (void)
{
  canO22TimeSinceLast = 0; //reset timeout 
  
  //byte0 Compound ID
  
  // Check O2 data is valid using sensor status
  if (CAN_Rx_Msgdata[7] == 0x00)
  {
    //byte1 and 2 Calibrated Sensor Output Value Hi:lo*1 = x.xxxLa
    uint32_t result = (CAN_Rx_Msgdata[1] << 8) | CAN_Rx_Msgdata[2]; //(highByte << 8) | lowByte - this is EQR from PLM
    
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
  switch (CAN_Rx_Msgdata[7])
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

// Default action when message times out
void canRx_MotecPLM_O22_Dflt(void)
{
  currentStatus.O2_2 = 255;
}

/* Recieve EPB data on 470 */
void canRx_EPB_Vss (void)
{
  canEPBTimeSinceLast = 0; //reset timeout 
  BIT_CLEAR(currentStatus.status5, BIT_STATUS5_CAN_RXEPBDFLT); 
  
  // Bytes 0 and 1 are vss X
  currentStatus.vss = (CAN_Rx_Msgdata[0] << 8) | CAN_Rx_Msgdata[1]; //(highByte << 8) | lowByte
  if (currentStatus.vss > 512) { currentStatus.vss = 512; } //basic error checking.


  // Byte 2 is gear
  // Byte 3 is reverse gearbox state, need to resolve both of them.
  currentStatus.gear = CAN_Rx_Msgdata[2]; // Raw gear from EPB.
  if ( currentStatus.gear == 6) { currentStatus.gear = 0; } // 0 and 6 are both neutral in speeduino gear logic.
  if ((CAN_Rx_Msgdata[3] == 0) && (currentStatus.gear >= 2 )){ currentStatus.gear = 1; } // Reverse gearbox is in reverse, limit speeduino gear to 1st
  else if (CAN_Rx_Msgdata[3] >= 2){ currentStatus.gear = 0; } // All other states of reverse gearbox are neutral

  // Read clutch trigger bit as the inverse of "clutch is top travel from EPB"
  clutchTrigger = !(CAN_Rx_Msgdata[7] & 0b01000000); // Bit 1 inverted.
  // Other messages not read 

}

// Default action when message times out
void canRx_EPB_Vss_Dflt(void)
{
  currentStatus.vss = 0;
  currentStatus.gear = 0;
  clutchTrigger = 0;
  BIT_SET(currentStatus.status5, BIT_STATUS5_CAN_RXEPBDFLT); 
}

void canRx_EPBAccelGyro1(void)
{
  // Bytes 0 and 1 are Ax
  currentStatus.longG = (CAN_Rx_Msgdata[0] << 8) | CAN_Rx_Msgdata[1]; //(highByte << 8) | lowByte 2g is 32768. -2g is -32768 
  // Bytes 2 and 3 are Ay
  currentStatus.latG = (CAN_Rx_Msgdata[2] << 8) | CAN_Rx_Msgdata[3]; //(highByte << 8) | lowByte 2g is 32768. -2g is -32768 
}

// Default action when message times out
void canRx_EPBAccelGyro1_Dflt(void)
{
  currentStatus.longG = 0;
  currentStatus.latG = 0;
}

//Just for testing
void canPrintErrors(uint8_t CANStat)
{
  switch(CANStat)
  {
    case CAN_OK:
      Serial.println(" CAN_OK ");
    break;
    
    case CAN_FAILINIT:
      Serial.println(" CAN_FAILINIT ");
    break;
    
    case CAN_FAILTX:
      Serial.println(" CAN_FAILTX ");
    break;
    
    case CAN_MSGAVAIL:
      Serial.println(" CAN_MSGAVAIL ");
    break;
    
    case CAN_NOMSG:
      Serial.println(" CAN_NOMSG ");
    break;
    
    case CAN_CTRLERROR:
      Serial.println(" CAN_CTRLERROR ");
    break;
    
    case CAN_GETTXBFTIMEOUT:
      Serial.println(" CAN_GETTXBFTIMEOUT ");
    break;
    
    case CAN_SENDMSGTIMEOUT:
      Serial.println(" CAN_SENDMSGTIMEOUT ");
    break;
    
    case CAN_FAIL:
      Serial.println(" CAN_FAIL ");
    break;
    
    default:
      Serial.println(" CAN_UNKNOWN_STAT ");
    break;
  }
  if (CANStat != CAN_FAIL) // No communication to controller
  {
    Serial.print("Rx Err Count: "); Serial.println(CAN0.errorCountRX()); //Recieve Error Count
    Serial.print("Tx Err Count: "); Serial.println(CAN0.errorCountTX()); //Transmitt Error Count
    Serial.print("CTRL Err Code: "); Serial.println(CAN0.getError(),HEX); //CAN EFLG register
  }
}

#endif
