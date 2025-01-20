/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
can_comms was originally contributed by Darren Siepka
*/

/*
secondserial_command is called when a command is received from the secondary serial port
It parses the command and calls the relevant function.

can_command is called when a command is received by the onboard/attached canbus module
It parses the command and calls the relevant function.

sendcancommand is called when a command is to be sent either to serial3 
,to the external Can interface, or to the onboard/attached can interface
*/
#include "globals.h"
#include "cancomms.h"
#include "maths.h"
#include "errors.h"
#include "utilities.h"

uint8_t currentsecondserialCommand;
uint8_t currentCanPage = 1;//Not the same as the speeduino config page numbers
uint8_t nCanretry = 0;      //no of retrys
uint8_t cancmdfail = 0;     //command fail yes/no
uint8_t canlisten = 0;
uint8_t Lbuffer[8];         //8 byte buffer to store incoming can data
uint8_t Gdata[9];
uint8_t Glow, Ghigh;
bool canCmdPending = false;

#if ( defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) )
  HardwareSerial &CANSerial = Serial3;
#elif defined(CORE_STM32)
  #ifndef HAVE_HWSERIAL2 //Hack to get the code to compile on BlackPills
    #define Serial2 Serial1
  #endif
  #if defined(STM32GENERIC) // STM32GENERIC core
    SerialUART &CANSerial = Serial2;
  #else //libmaple core aka STM32DUINO
    HardwareSerial &CANSerial = Serial2;
  #endif
#elif defined(CORE_TEENSY)
  HardwareSerial &CANSerial = Serial2;
#endif

void secondserial_Command()
{
  #if defined(CANSerial_AVAILABLE)
  if (! canCmdPending) {  currentsecondserialCommand = CANSerial.read();  }

  switch (currentsecondserialCommand)
  {
    case 'A': // sends the bytes of realtime values from the OLD CAN list
        sendcanValues(0, CAN_PACKET_SIZE, 0x31, 1); //send values to serial3
        break;

    case 'G': // this is the reply command sent by the Can interface
       byte destcaninchannel;
      if (CANSerial.available() >= 9)
      {
        canCmdPending = false;
        cancmdfail = CANSerial.read();        //0 == fail,  1 == good.
        destcaninchannel = CANSerial.read();  // the input channel that requested the data value
        if (cancmdfail != 0)
           {                                 // read all 8 bytes of data.
            for (byte Gx = 0; Gx < 8; Gx++) // first two are the can address the data is from. next two are the can address the data is for.then next 1 or two bytes of data
              {
                Gdata[Gx] = CANSerial.read();
              }
            Glow = Gdata[(configPage9.caninput_source_start_byte[destcaninchannel]&7)];
            if ((BIT_CHECK(configPage9.caninput_source_num_bytes,destcaninchannel) > 0))  //if true then num bytes is 2
               {
                if ((configPage9.caninput_source_start_byte[destcaninchannel]&7) < 8)   //you can't have a 2 byte value starting at byte 7(8 on the list)
                   {
                    Ghigh = Gdata[((configPage9.caninput_source_start_byte[destcaninchannel]&7)+1)];
                   }
            else{Ghigh = 0;}
               }
          else
               {
                 Ghigh = 0;
               }

          currentStatus.canin[destcaninchannel] = (Ghigh<<8) | Glow;
        }

        else{}  //continue as command request failed and/or data/device was not available

      }
      else
      {
        canCmdPending = true;
      }
      
        break;

    case 'k':   //placeholder for new can interface (toucan etc) commands

        break;
        
    case 'L':
        uint8_t Llength;
        while (CANSerial.available() == 0) { }
        canlisten = CANSerial.read();

        if (canlisten == 0)
        {
          //command request failed and/or data/device was not available
          break;
        }

        while (CANSerial.available() == 0) { }
        Llength= CANSerial.read();              // next the number of bytes expected value

        for (uint8_t Lcount = 0; Lcount <Llength ;Lcount++)
        {
          while (CANSerial.available() == 0){}
          // receive all x bytes into "Lbuffer"
          Lbuffer[Lcount] = CANSerial.read();
        }
        break;

    case 'n': // sends the bytes of realtime values from the NEW CAN list
        sendcanValues(0, NEW_CAN_PACKET_SIZE, 0x32, 1); //send values to serial3
        break;

    case 'r': //New format for the optimised OutputChannels over CAN
      byte Cmd;
      if (CANSerial.available() >= 6)
      {
        CANSerial.read(); //Read the $tsCanId
        Cmd = CANSerial.read();

        uint16_t offset, length;
        if( (Cmd == 0x30) || ( (Cmd >= 0x40) && (Cmd <0x50) ) ) //Send output channels command 0x30 is 48dec, 0x40(64dec)-0x4F(79dec) are external can request
        {
          byte tmp;
          tmp = CANSerial.read();
          offset = word(CANSerial.read(), tmp);
          tmp = CANSerial.read();
          length = word(CANSerial.read(), tmp);
          sendcanValues(offset, length,Cmd, 1);
          canCmdPending = false;
          //Serial.print(Cmd);
        }
        else
        {
          //No other r/ commands should be called
        }
      }
      else
      {
        canCmdPending = true;
      }
      break;

    case 's': // send the "a" stream code version
      CANSerial.print(F("Speeduino csx02019.8"));
      break;

    case 'S': // send code version
      CANSerial.print(F("Speeduino 2019.08-ser"));
      break;
      
    case 'Q': // send code version
       //for (unsigned int revn = 0; revn < sizeof( TSfirmwareVersion) - 1; revn++)
       for (unsigned int revn = 0; revn < 10 - 1; revn++)
       {
         CANSerial.write( TSfirmwareVersion[revn]);
       }
       //Serial3.print("speeduino 201609-dev");
       break;

    case 'Z': //dev use
       break;

    default:
       break;
  }
  #endif
}
void sendcanValues(uint16_t offset, uint16_t packetLength, byte cmd, byte portType)
{
    //CAN serial
    #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)|| defined(CORE_STM32) || defined (CORE_TEENSY) //ATmega2561 does not have Serial3
      if (cmd == 0x30) 
          {
           CANSerial.write("r");         //confirm cmd type
           CANSerial.write(cmd);
          }
        else if (cmd == 0x31)
          {
           CANSerial.write("A");         // confirm command type   
          }
        else if (cmd == 0x32)
          {
           CANSerial.write("n");                       // confirm command type
           CANSerial.write(cmd);                       // send command type  , 0x32 (dec50) is ascii '0'
           CANSerial.write(NEW_CAN_PACKET_SIZE);       // send the packet size the receiving device should expect.
          }
    #endif

  currentStatus.spark ^= (-currentStatus.hasSync ^ currentStatus.spark) & (1U << BIT_SPARK_SYNC); //Set the sync bit of the Spark variable to match the hasSync variable

#if defined(CANSerial_AVAILABLE)
  byte fullStatus[NEW_CAN_PACKET_SIZE];    // this must be set to the maximum number of data fullstatus must read in
  fullStatus[0] = currentStatus.secl; //secl is simply a counter that increments each second. Used to track unexpected resets (Which will reset this count to 0)
  fullStatus[1] = currentStatus.status1; //status1 Bitfield, inj1Status(0), inj2Status(1), inj3Status(2), inj4Status(3), DFCOOn(4), boostCutFuel(5), toothLog1Ready(6), toothLog2Ready(7)
  fullStatus[2] = currentStatus.engine; //Engine Status Bitfield, running(0), crank(1), ase(2), warmup(3), tpsaccaen(4), tpsacden(5), mapaccaen(6), mapaccden(7)
  fullStatus[3] = currentStatus.syncLossCounter;
  fullStatus[4] = lowByte(currentStatus.MAP); //2 bytes for MAP
  fullStatus[5] = highByte(currentStatus.MAP);
  fullStatus[6] = (byte)(currentStatus.IAT + CALIBRATION_TEMPERATURE_OFFSET); //mat
  fullStatus[7] = (byte)(currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET); //Coolant ADC
  fullStatus[8] = currentStatus.batCorrection; //Battery voltage correction (%)
  fullStatus[9] = currentStatus.battery10; //battery voltage
  fullStatus[10] = currentStatus.O2; //O2
  fullStatus[11] = currentStatus.egoCorrection; //Exhaust gas correction (%)
  fullStatus[12] = currentStatus.iatCorrection; //Air temperature Correction (%)
  fullStatus[13] = currentStatus.wueCorrection; //Warmup enrichment (%)
  fullStatus[14] = lowByte(currentStatus.RPM); //rpm HB
  fullStatus[15] = highByte(currentStatus.RPM); //rpm LB
  fullStatus[16] = currentStatus.AEamount; //acceleration enrichment (%)
  fullStatus[17] = lowByte(currentStatus.corrections); //Total GammaE (%)
  fullStatus[18] = highByte(currentStatus.corrections); //Total GammaE (%)
  fullStatus[19] = currentStatus.VE; //Current VE 1 (%)
  fullStatus[20] = currentStatus.VE2; //Current VE 2 (%)
  fullStatus[21] = currentStatus.afrTarget;
  fullStatus[22] = lowByte(currentStatus.tpsDOT); //TPS DOT
  fullStatus[23] = highByte(currentStatus.tpsDOT); //TPS DOT
  fullStatus[24] = currentStatus.advance;
  fullStatus[25] = currentStatus.TPS; // TPS (0% to 100%)
  fullStatus[26] = lowByte(currentStatus.loopsPerSecond);
  fullStatus[27] = highByte(currentStatus.loopsPerSecond);

  //The following can be used to show the amount of free memory
  currentStatus.freeRAM = freeRam();
  fullStatus[28] = lowByte(currentStatus.freeRAM); //(byte)((currentStatus.loopsPerSecond >> 8) & 0xFF);
  fullStatus[29] = highByte(currentStatus.freeRAM);

  fullStatus[30] = (byte)(currentStatus.boostTarget >> 1); //Divide boost target by 2 to fit in a byte
  fullStatus[31] = (byte)(currentStatus.boostDuty / 100);
  fullStatus[32] = currentStatus.spark; //Spark related bitfield, launchHard(0), launchSoft(1), hardLimitOn(2), softLimitOn(3), boostCutSpark(4), error(5), idleControlOn(6), sync(7)

  //rpmDOT must be sent as a signed integer
  fullStatus[33] = lowByte(currentStatus.rpmDOT);
  fullStatus[34] = highByte(currentStatus.rpmDOT);

  fullStatus[35] = currentStatus.ethanolPct; //Flex sensor value (or 0 if not used)
  fullStatus[36] = currentStatus.flexCorrection; //Flex fuel correction (% above or below 100)
  fullStatus[37] = currentStatus.flexIgnCorrection; //Ignition correction (Increased degrees of advance) for flex fuel

  fullStatus[38] = currentStatus.idleLoad;
  fullStatus[39] = currentStatus.testOutputs; // testEnabled(0), testActive(1)

  fullStatus[40] = currentStatus.O2_2; //O2
  fullStatus[41] = currentStatus.baro; //Barometer value

  fullStatus[42] = lowByte(currentStatus.canin[0]);
  fullStatus[43] = highByte(currentStatus.canin[0]);
  fullStatus[44] = lowByte(currentStatus.canin[1]);
  fullStatus[45] = highByte(currentStatus.canin[1]);
  fullStatus[46] = lowByte(currentStatus.canin[2]);
  fullStatus[47] = highByte(currentStatus.canin[2]);
  fullStatus[48] = lowByte(currentStatus.canin[3]);
  fullStatus[49] = highByte(currentStatus.canin[3]);
  fullStatus[50] = lowByte(currentStatus.canin[4]);
  fullStatus[51] = highByte(currentStatus.canin[4]);
  fullStatus[52] = lowByte(currentStatus.canin[5]);
  fullStatus[53] = highByte(currentStatus.canin[5]);
  fullStatus[54] = lowByte(currentStatus.canin[6]);
  fullStatus[55] = highByte(currentStatus.canin[6]);
  fullStatus[56] = lowByte(currentStatus.canin[7]);
  fullStatus[57] = highByte(currentStatus.canin[7]);
  fullStatus[58] = lowByte(currentStatus.canin[8]);
  fullStatus[59] = highByte(currentStatus.canin[8]);
  fullStatus[60] = lowByte(currentStatus.canin[9]);
  fullStatus[61] = highByte(currentStatus.canin[9]);
  fullStatus[62] = lowByte(currentStatus.canin[10]);
  fullStatus[63] = highByte(currentStatus.canin[10]);
  fullStatus[64] = lowByte(currentStatus.canin[11]);
  fullStatus[65] = highByte(currentStatus.canin[11]);
  fullStatus[66] = lowByte(currentStatus.canin[12]);
  fullStatus[67] = highByte(currentStatus.canin[12]);
  fullStatus[68] = lowByte(currentStatus.canin[13]);
  fullStatus[69] = highByte(currentStatus.canin[13]);
  fullStatus[70] = lowByte(currentStatus.canin[14]);
  fullStatus[71] = highByte(currentStatus.canin[14]);
  fullStatus[72] = lowByte(currentStatus.canin[15]);
  fullStatus[73] = highByte(currentStatus.canin[15]);
  
  fullStatus[74] = currentStatus.tpsADC;
  fullStatus[75] = getNextError(); // errorNum (0:1), currentError(2:7)

  fullStatus[76] = lowByte(currentStatus.PW1); //Pulsewidth 1 multiplied by 10 in ms. Have to convert from uS to mS.
  fullStatus[77] = highByte(currentStatus.PW1); //Pulsewidth 1 multiplied by 10 in ms. Have to convert from uS to mS.
  fullStatus[78] = lowByte(currentStatus.PW2); //Pulsewidth 2 multiplied by 10 in ms. Have to convert from uS to mS.
  fullStatus[79] = highByte(currentStatus.PW2); //Pulsewidth 2 multiplied by 10 in ms. Have to convert from uS to mS.
  fullStatus[80] = lowByte(currentStatus.PW3); //Pulsewidth 3 multiplied by 10 in ms. Have to convert from uS to mS.
  fullStatus[81] = highByte(currentStatus.PW3); //Pulsewidth 3 multiplied by 10 in ms. Have to convert from uS to mS.
  fullStatus[82] = lowByte(currentStatus.PW4); //Pulsewidth 4 multiplied by 10 in ms. Have to convert from uS to mS.
  fullStatus[83] = highByte(currentStatus.PW4); //Pulsewidth 4 multiplied by 10 in ms. Have to convert from uS to mS.

  fullStatus[84] = currentStatus.status3; // resentLockOn(0), nitrousOn(1), fuel2Active(2), vssRefresh(3), halfSync(4), nSquirts(6:7)
  fullStatus[85] = currentStatus.engineProtectStatus; //RPM(0), MAP(1), OIL(2), AFR(3), Unused(4:7)
  fullStatus[86] = lowByte(currentStatus.fuelLoad);
  fullStatus[87] = highByte(currentStatus.fuelLoad);
  fullStatus[88] = lowByte(currentStatus.ignLoad);
  fullStatus[89] = highByte(currentStatus.ignLoad);
  fullStatus[90] = lowByte(currentStatus.dwell); 
  fullStatus[91] = highByte(currentStatus.dwell); 
  fullStatus[92] = currentStatus.CLIdleTarget;
  fullStatus[93] = (uint8_t)(currentStatus.mapDOT); //rate of change of the map 
  fullStatus[94] = lowByte(currentStatus.vvt1Angle);
  fullStatus[95] = highByte(currentStatus.vvt1Angle);
  fullStatus[96] = currentStatus.vvt1TargetAngle;
  fullStatus[97] = currentStatus.vvt1Duty;
  fullStatus[98] = lowByte(currentStatus.flexBoostCorrection);
  fullStatus[99] = highByte(currentStatus.flexBoostCorrection);
  fullStatus[100] = currentStatus.baroCorrection;
  fullStatus[101] = currentStatus.VE;
  fullStatus[102] = currentStatus.ASEValue;
  fullStatus[103] = lowByte(currentStatus.vss); //speed reading from the speed sensor
  fullStatus[104] = highByte(currentStatus.vss);
  fullStatus[105] = currentStatus.gear; 
  fullStatus[106] = highByte(currentStatus.fuelPressure);
  fullStatus[107] = lowByte(currentStatus.fuelPressure);
  fullStatus[108] = currentStatus.oilPressure;
  fullStatus[109] = currentStatus.status4; // CanStatus (0), vvt1Error(1), vvt2Error(2), fanStatus(3), UnusedBits(4:7)
  fullStatus[110] = lowByte(currentStatus.vvt2Angle);
  fullStatus[111] = highByte(currentStatus.vvt2Angle);
  fullStatus[112] = currentStatus.vvt2TargetAngle;
  fullStatus[113] = (byte)(currentStatus.vvt2Duty);
  fullStatus[114] = currentStatus.outputsStatus;
  fullStatus[115] = (byte)(currentStatus.fuelTemp + CALIBRATION_TEMPERATURE_OFFSET); //Fuel temperature from flex sensor
  fullStatus[116] = currentStatus.fuelTempCorrection; //Fuel temperature Correction (%)
  fullStatus[117] = currentStatus.advance1; //advance 1 
  fullStatus[118] = currentStatus.advance2; //advance 2 
  fullStatus[119] = currentStatus.TS_SD_Status; //SD card status
  fullStatus[120] = lowByte(currentStatus.EGT); //2 bytes for EGT1 -WAS EMAP
  fullStatus[121] = highByte(currentStatus.EGT);
  fullStatus[122] = currentStatus.fanDuty;
  fullStatus[123] = currentStatus.ego2Correction; // ego 2 correction
  fullStatus[124] = lowByte(currentStatus.InjectorDeltaPress); // Injector Differential Pressure
  fullStatus[125] = highByte(currentStatus.InjectorDeltaPress);
  fullStatus[126] = currentStatus.injPressCorrection; // Injector Pressure Differential Correction
  fullStatus[127] = lowByte(currentStatus.longG); // Longitudinal G force
  fullStatus[128] = highByte(currentStatus.longG);
  fullStatus[129] = lowByte(currentStatus.latG); // Lateral G force
  fullStatus[130] = highByte(currentStatus.latG);
  fullStatus[131] = lowByte(currentStatus.fuelUsedThisKey); // Fuel used since speeduino powered up in L
  fullStatus[132] = highByte(currentStatus.fuelUsedThisKey);
  fullStatus[133] = currentStatus.status5; //CAN Bus error status

  if (packetLength >= NEW_CAN_PACKET_SIZE) { packetLength = NEW_CAN_PACKET_SIZE - 1; } //protection against array out of bounds.
  
  for(uint16_t x=0; x<packetLength; x++)
  {
      if (portType == 1){ CANSerial.write(fullStatus[offset+x]); }
      else if (portType == 2)
      {
        //sendto canbus transmit routine
      }
  }
#else 
  UNUSED(offset);
  UNUSED(packetLength);
  UNUSED(cmd);
  UNUSED(portType);
#endif

}

void can_Command()
{
 //int currentcanCommand = inMsg.id;
 #if defined (NATIVE_CAN_AVAILABLE)
      // currentStatus.canin[12] = (inMsg.id);
 if ( (inMsg.id == uint16_t(configPage9.obd_address + 0x100))  || (inMsg.id == 0x7DF))      
  {
    // The address is the speeduino specific ecu canbus address 
    // or the 0x7df(2015 dec) broadcast address
    if (inMsg.buf[1] == 0x01)
      {
        // PID mode 0 , realtime data stream
        obd_response(inMsg.buf[1], inMsg.buf[2], 0);     // get the obd response based on the data in byte2
        outMsg.id = (0x7E8);       //((configPage9.obd_address + 0x100)+ 8);  
        Can0.write(outMsg);       // send the 8 bytes of obd data   
      }
    if (inMsg.buf[1] == 0x22)
      {
        // PID mode 22h , custom mode , non standard data
        obd_response(inMsg.buf[1], inMsg.buf[2], inMsg.buf[3]);     // get the obd response based on the data in byte2
        outMsg.id = (0x7E8); //configPage9.obd_address+8);
        Can0.write(outMsg);       // send the 8 bytes of obd data
      }
  }
 if (inMsg.id == uint16_t(configPage9.obd_address + 0x100))      
  {
    // The address is only the speeduino specific ecu canbus address    
    if (inMsg.buf[1] == 0x09)
      {
       // PID mode 9 , vehicle information request
       if (inMsg.buf[2] == 02)
         {
          //send the VIN number , 17 char long VIN sent in 5 messages.
         }
      else if (inMsg.buf[2] == 0x0A)
         {
          //code 20: send 20 ascii characters with ECU name , "ECU -SpeeduinoXXXXXX" , change the XXXXXX ONLY as required.  
         }
      }
  }
#endif  
}  
    
// this routine sends a request(either "0" for a "G" , "1" for a "L" , "2" for a "R" to the Can interface or "3" sends the request via the actual local canbus
void sendCancommand(uint8_t cmdtype, uint16_t canaddress, uint8_t candata1, uint8_t candata2, uint16_t sourcecanAddress)
{
#if defined(CANSerial_AVAILABLE)
    switch (cmdtype)
    {
      case 0:
        CANSerial.print("G");
        CANSerial.write(canaddress);  //tscanid of speeduino device
        CANSerial.write(candata1);    // table id
        CANSerial.write(candata2);    //table memory offset
        break;

      case 1:                      //send request to listen for a can message
        CANSerial.print("L");
        CANSerial.write(canaddress);  //11 bit canaddress of device to listen for
        break;

     case 2:                                          // requests via serial3
        CANSerial.print("R");                         //send "R" to request data from the sourcecanAddress whose value is sent next
        CANSerial.write(candata1);                    //the currentStatus.current_caninchannel
        CANSerial.write(lowByte(sourcecanAddress) );       //send lsb first
        CANSerial.write(highByte(sourcecanAddress) );
        break;

     case 3:
        //send to truecan send routine
        //canaddress == speeduino canid, candata1 == canin channel dest, paramgroup == can address  to request from
        //This section is to be moved to the correct can output routine later
        #if defined(NATIVE_CAN_AVAILABLE)
        outMsg.id = (canaddress);
        outMsg.len = 8;
        CAN_Tx_Msgdata[0] = 0x0B ;  //11;   
        CAN_Tx_Msgdata[1] = 0x15;
        CAN_Tx_Msgdata[2] = candata1;
        CAN_Tx_Msgdata[3] = 0x24;
        CAN_Tx_Msgdata[4] = 0x7F;
        CAN_Tx_Msgdata[5] = 0x70;
        CAN_Tx_Msgdata[6] = 0x9E;
        CAN_Tx_Msgdata[7] = 0x4D;
        Can0.write(outMsg);
        #endif
        break;

     default:
        break;
    }
#else
  UNUSED(cmdtype);
  UNUSED(canaddress);
  UNUSED(candata1);
  UNUSED(candata2);
  UNUSED(sourcecanAddress);
#endif
}

// This routine supports the OBDII service 01 mode (sending realtime data)
void obd_Service_01(uint8_t requestedPIDlow)
{ 
//only build the PID if the mcu has onboard/attached can 

  uint16_t obdcalcA;    //used in obd calcs
  uint16_t obdcalcB;    //used in obd calcs 
  uint16_t obdcalcC;    //used in obd calcs 
  uint16_t obdcalcD;    //used in obd calcs
  uint32_t obdcalcE32;    //used in calcs 
  uint32_t obdcalcF32;    //used in calcs 
  uint16_t obdcalcG16;    //used in calcs
  uint16_t obdcalcH16;    //used in calcs  
  
  switch (requestedPIDlow)
  {
    case 0:       //PID-0x00 PIDs supported 0x01-0x20  
      CAN_Tx_Msgdata[0] =  0x06;    // sending 6 bytes
      CAN_Tx_Msgdata[1] =  0x41;    // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
      CAN_Tx_Msgdata[2] =  0x00;    // PID code
      CAN_Tx_Msgdata[3] =  B10001101;   //1-8
      CAN_Tx_Msgdata[4] =  B01111110;   //9-16
      CAN_Tx_Msgdata[5] =  B10100000;   //17-24
      CAN_Tx_Msgdata[6] =  B00010001;   //25-32
      CAN_Tx_Msgdata[7] =  0x00;   
    break;
    
    case 1:      //PID-0x01 Monitor status since DTCs cleared.
      CAN_Tx_Msgdata[0] =  0x08;                 // sending 8 bytes
      CAN_Tx_Msgdata[1] =  0x41;                 // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
      CAN_Tx_Msgdata[2] =  0x01;                 // pid code
      CAN_Tx_Msgdata[3] =  B10000001;            //A7	State of the CEL/MIL (on/off). A6-A0	Number of confirmed emissions-related DTCs available for display.
      CAN_Tx_Msgdata[4] =  B00000100;            //B6-B4	Bitmap indicating completeness of common tests (0=complete). B3	Indication of engine type 0 = Spark ignition.  B2-B0	Bitmap indicating availability of common tests.
      CAN_Tx_Msgdata[5] =  0x00; 
      CAN_Tx_Msgdata[6] =  0x00; 
      CAN_Tx_Msgdata[7] =  0x00;
    break;

    case 5:      //PID-0x05 Engine coolant temperature , range is -40 to 215 deg C , formula == A-40
      CAN_Tx_Msgdata[0] =  0x03;                 // sending 3 bytes
      CAN_Tx_Msgdata[1] =  0x41;                 // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
      CAN_Tx_Msgdata[2] =  0x05;                 // pid code
      CAN_Tx_Msgdata[3] =  (byte)(currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET);   //the data value A
      CAN_Tx_Msgdata[4] =  0x00;                 //the data value B which is 0 as unused
      CAN_Tx_Msgdata[5] =  0x00; 
      CAN_Tx_Msgdata[6] =  0x00; 
      CAN_Tx_Msgdata[7] =  0x00;
    break;
    
    case 6:        // PID-0x06 , Short term fuel trim (STFT)—Bank 1 , range is -100 (subtracting fuel) to 100 (adding fuel) , formula == A*1.28+100)
      int16_t temp_gEgo;
      // Fuel Trim is in %
      temp_gEgo = (int16_t)((currentStatus.egoCorrection * 128) / 100);
      CAN_Tx_Msgdata[0] =  0x03;    // sending 3 byte
      CAN_Tx_Msgdata[1] =  0x41;    // 
      CAN_Tx_Msgdata[2] =  0x06;    // pid code
      CAN_Tx_Msgdata[3] =  lowByte(temp_gEgo);
      CAN_Tx_Msgdata[4] =  0x00;
      CAN_Tx_Msgdata[5] =  0x00; 
      CAN_Tx_Msgdata[6] =  0x00; 
      CAN_Tx_Msgdata[7] =  0x00;
    break;
    
    case 8:        // PID-0x08 , Short term fuel trim (STFT)—Bank 2 , range is -100 (subtracting fuel) to 100 (adding fuel) , formula == A*1.28+100)
      int16_t temp_gEgo2;
      // Fuel Trim is in %
      temp_gEgo = (int16_t)((currentStatus.ego2Correction * 128) / 100);
      CAN_Tx_Msgdata[0] =  0x03;    // sending 3 byte
      CAN_Tx_Msgdata[1] =  0x41;    // 
      CAN_Tx_Msgdata[2] =  0x08;    // pid code
      CAN_Tx_Msgdata[3] =  lowByte(temp_gEgo2);
      CAN_Tx_Msgdata[4] =  0x00;
      CAN_Tx_Msgdata[5] =  0x00; 
      CAN_Tx_Msgdata[6] =  0x00; 
      CAN_Tx_Msgdata[7] =  0x00;
    break;

    case 10:        // PID-0x0A , Fuel Pressure (Gauge) , range is 0 to 765 kPa , formula == A / 3)
      uint16_t temp_fuelpressure;
      // Fuel pressure is in kPa
      temp_fuelpressure = (currentStatus.fuelPressure - currentStatus.baro) / 3;
      CAN_Tx_Msgdata[0] =  0x03;    // sending 3 byte
      CAN_Tx_Msgdata[1] =  0x41;    // 
      CAN_Tx_Msgdata[2] =  0x0A;    // pid code
      CAN_Tx_Msgdata[3] =  lowByte(temp_fuelpressure);
      CAN_Tx_Msgdata[4] =  0x00;
      CAN_Tx_Msgdata[5] =  0x00; 
      CAN_Tx_Msgdata[6] =  0x00; 
      CAN_Tx_Msgdata[7] =  0x00;
    break;

    case 11:        // PID-0x0B , MAP , range is 0 to 255 kPa , Formula == A
      CAN_Tx_Msgdata[0] =  0x03;    // sending 3 byte
      CAN_Tx_Msgdata[1] =  0x41;    // 
      CAN_Tx_Msgdata[2] =  0x0B;    // pid code
      CAN_Tx_Msgdata[3] =  lowByte(currentStatus.MAP);    // absolute map
      CAN_Tx_Msgdata[4] =  0x00;
      CAN_Tx_Msgdata[5] =  0x00; 
      CAN_Tx_Msgdata[6] =  0x00; 
      CAN_Tx_Msgdata[7] =  0x00;
    break;

    case 12:        // PID-0x0C , RPM  , range is 0 to 16383.75 rpm , Formula == 256A+B / 4
      uint16_t temp_revs; 
      temp_revs = currentStatus.RPM << 2 ;      //
      CAN_Tx_Msgdata[0] = 0x04;                        // sending 4 byte
      CAN_Tx_Msgdata[1] = 0x41;                        // 
      CAN_Tx_Msgdata[2] = 0x0C;                        // pid code
      CAN_Tx_Msgdata[3] = highByte(temp_revs);         //obdcalcB; A
      CAN_Tx_Msgdata[4] = lowByte(temp_revs);          //obdcalcD; B
      CAN_Tx_Msgdata[5] = 0x00; 
      CAN_Tx_Msgdata[6] = 0x00; 
      CAN_Tx_Msgdata[7] = 0x00;
    break;

    case 13:        //PID-0x0D , Vehicle speed , range is 0 to 255 km/h , formula == A 
      CAN_Tx_Msgdata[0] =  0x03;                       // sending 3 bytes
      CAN_Tx_Msgdata[1] =  0x41;                       // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
      CAN_Tx_Msgdata[2] =  0x0D;                       // pid code
      CAN_Tx_Msgdata[3] =  lowByte(currentStatus.vss); // A
      CAN_Tx_Msgdata[4] =  0x00;                       // B
      CAN_Tx_Msgdata[5] =  0x00; 
      CAN_Tx_Msgdata[6] =  0x00; 
      CAN_Tx_Msgdata[7] =  0x00;
    break;

    case 14:      //PID-0x0E , Ignition Timing advance, range is -64 to 63.5 BTDC , formula == A/2 - 64 
      int8_t temp_timingadvance;
      temp_timingadvance = ((currentStatus.advance + 64) << 1);
      //obdcalcA = ((timingadvance + 64) <<1) ; //((timingadvance + 64) *2)
      CAN_Tx_Msgdata[0] =  0x03;                     // sending 3 bytes
      CAN_Tx_Msgdata[1] =  0x41;                     // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
      CAN_Tx_Msgdata[2] =  0x0E;                     // pid code
      CAN_Tx_Msgdata[3] =  temp_timingadvance;       // A
      CAN_Tx_Msgdata[4] =  0x00;                     // B
      CAN_Tx_Msgdata[5] =  0x00; 
      CAN_Tx_Msgdata[6] =  0x00; 
      CAN_Tx_Msgdata[7] =  0x00;
    break;

    case 15:      //PID-0x0F , Inlet air temperature , range is -40 to 215 deg C, formula == A-40 
      CAN_Tx_Msgdata[0] =  0x03;                                                         // sending 3 bytes
      CAN_Tx_Msgdata[1] =  0x41;                                                         // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
      CAN_Tx_Msgdata[2] =  0x0F;                                                         // pid code
      CAN_Tx_Msgdata[3] =  (byte)(currentStatus.IAT + CALIBRATION_TEMPERATURE_OFFSET);   // A
      CAN_Tx_Msgdata[4] =  0x00;                                                         // B
      CAN_Tx_Msgdata[5] =  0x00; 
      CAN_Tx_Msgdata[6] =  0x00; 
      CAN_Tx_Msgdata[7] =  0x00;
   break;

   case 17:  // PID-0x11 , 
     // TPS percentage, range is 0 to 100 percent, formula == 100/256 A 
     uint16_t temp_tpsPC;
     temp_tpsPC = currentStatus.TPS;
     obdcalcA = (temp_tpsPC <<8) / 200;     // (tpsPC *256) /200 (as TPS is stored as TPS *2);
     if (obdcalcA > 255){ obdcalcA = 255;}
     CAN_Tx_Msgdata[0] =  0x03;                    // sending 3 bytes
     CAN_Tx_Msgdata[1] =  0x41;                    // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
     CAN_Tx_Msgdata[2] =  0x11;                    // pid code
     CAN_Tx_Msgdata[3] =  obdcalcA;                // A
     CAN_Tx_Msgdata[4] =  0x00;                    // B
     CAN_Tx_Msgdata[5] =  0x00; 
     CAN_Tx_Msgdata[6] =  0x00; 
     CAN_Tx_Msgdata[7] =  0x00;
   break;

   case 19:      //PID-0x13 , oxygen sensors present, A0-A3 == bank1 , A4-A7 == bank2 , 
     uint8_t O2present;
     if (configPage6.egoAlgorithm == EGO_ALGORITHM_SINGLEO2) { O2present = B00000001; }
     else if (configPage6.egoAlgorithm == EGO_ALGORITHM_DUALO2) { O2present = B00010001; }
     else { O2present = 0x00; }
     CAN_Tx_Msgdata[0] =  0x03;           // sending 3 bytes
     CAN_Tx_Msgdata[1] =  0x41;           // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
     CAN_Tx_Msgdata[2] =  0x13;           // pid code
     CAN_Tx_Msgdata[3] =  O2present;     // A
     CAN_Tx_Msgdata[4] =  0x00;           // B
     CAN_Tx_Msgdata[5] =  0x00; 
     CAN_Tx_Msgdata[6] =  0x00; 
     CAN_Tx_Msgdata[7] =  0x00;
   break;

   case 28:      // PID-0x1C obd standard
     uint16_t obdstandard;
     obdstandard = 7;              // This is OBD2 / EOBD (could also describe as 5 which is not OBD compliant)
     CAN_Tx_Msgdata[0] =  0x03;           // sending 3 bytes
     CAN_Tx_Msgdata[1] =  0x41;           // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
     CAN_Tx_Msgdata[2] =  0x1C;           // pid code
     CAN_Tx_Msgdata[3] =  obdstandard;    // A
     CAN_Tx_Msgdata[4] =  0x00;           // B
     CAN_Tx_Msgdata[5] =  0x00; 
     CAN_Tx_Msgdata[6] =  0x00; 
     CAN_Tx_Msgdata[7] =  0x00;
   break;

  case 32:      // PID-0x20 PIDs supported [0x21-0x40]
    CAN_Tx_Msgdata[0] =  0x06;          // sending 4 bytes
    CAN_Tx_Msgdata[1] =  0x41;          // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
    CAN_Tx_Msgdata[2] =  0x20;          // pid code
    CAN_Tx_Msgdata[3] =  B00011000;     // 33-40
    CAN_Tx_Msgdata[4] =  B00000000;     // 41-48
    CAN_Tx_Msgdata[5] =  B00100000;     // 49-56
    CAN_Tx_Msgdata[6] =  B00010001;     // 57-64
    CAN_Tx_Msgdata[7] = 0x00;
  break;

  case 36:      // PID-0x24 O2 sensor1, AB: fuel/air equivalence ratio, CD: voltage ,  Formula == (2/65536)(256A +B) , 8/65536(256C+D) , Range is 0 to <2 and 0 to >8V 
    obdcalcH16 = configPage2.stoich;            // configPage2.stoich(is *10 so 14.7 is 147)
    obdcalcE32 = currentStatus.O2;            // afr(is *10 so 25.5 is 255) , needs a 32bit else will overflow
    obdcalcF32 = (obdcalcE32<<8) / obdcalcH16;      //this is same as (obdcalcE32/256) / obdcalcH16 . this calculates the ratio      
    obdcalcG16 = (obdcalcF32 *32768)>>8;          
    obdcalcA = highByte(obdcalcG16);
    obdcalcB = lowByte(obdcalcG16);       

    obdcalcF32 = currentStatus.O2ADC ;             //o2ADC is wideband volts to send *100    
    obdcalcG16 = (obdcalcF32 *20971)>>8;          
    obdcalcC = highByte(obdcalcG16);
    obdcalcD = lowByte(obdcalcG16);

    CAN_Tx_Msgdata[0] =  0x06;    // sending 4 bytes
    CAN_Tx_Msgdata[1] =  0x41;    // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
    CAN_Tx_Msgdata[2] =  0x24;    // pid code
    CAN_Tx_Msgdata[3] =  obdcalcA;   // A
    CAN_Tx_Msgdata[4] =  obdcalcB;   // B
    CAN_Tx_Msgdata[5] =  obdcalcC;   // C
    CAN_Tx_Msgdata[6] =  obdcalcD;   // D
    CAN_Tx_Msgdata[7] =  0x00;
  break;

  case 37:      //O2 sensor2, AB fuel/air equivalence ratio, CD voltage ,  2/65536(256A +B) ,8/65536(256C+D) , range is 0 to <2 and 0 to >8V
    obdcalcH16 = configPage2.stoich;            // configPage2.stoich(is *10 so 14.7 is 147)
    obdcalcE32 = currentStatus.O2_2;            // afr(is *10 so 25.5 is 255) , needs a 32bit else will overflow
    obdcalcF32 = (obdcalcE32<<8) / obdcalcH16;      //this is same as (obdcalcE32/256) / obdcalcH16 . this calculates the ratio      
    obdcalcG16 = (obdcalcF32 *32768)>>8;          
    obdcalcA = highByte(obdcalcG16);
    obdcalcB = lowByte(obdcalcG16);       

    obdcalcF32 = currentStatus.O2_2ADC ;             //o2_2ADC is wideband volts to send *100    
    obdcalcG16 = (obdcalcF32 *20971)>>8;          
    obdcalcC = highByte(obdcalcG16);
    obdcalcD = lowByte(obdcalcG16);

    CAN_Tx_Msgdata[0] =  0x06;    // sending 4 bytes
    CAN_Tx_Msgdata[1] =  0x41;    // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
    CAN_Tx_Msgdata[2] =  0x25;    // pid code
    CAN_Tx_Msgdata[3] =  obdcalcA;   // A
    CAN_Tx_Msgdata[4] =  obdcalcB;   // B
    CAN_Tx_Msgdata[5] =  obdcalcC;   // C
    CAN_Tx_Msgdata[6] =  obdcalcD;   // D 
    CAN_Tx_Msgdata[7] =  0x00;
  break;

  case 51:      //PID-0x33 Absolute Barometric pressure , range is 0 to 255 kPa , formula == A
    CAN_Tx_Msgdata[0] =  0x03;                  // sending 3 bytes
    CAN_Tx_Msgdata[1] =  0x41;                  // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
    CAN_Tx_Msgdata[2] =  0x33;                  // pid code
    CAN_Tx_Msgdata[3] =  currentStatus.baro ;   // A
    CAN_Tx_Msgdata[4] =  0x00;                  // B which is 0 as unused
    CAN_Tx_Msgdata[5] =  0x00; 
    CAN_Tx_Msgdata[6] =  0x00; 
    CAN_Tx_Msgdata[7] =  0x00;
  break;
  
  case 60:      //PID-0x3C Catalyst Temperature B1 Sensor 1 -40	6,513.5 10* (256A+B)+ 40
    obdcalcA = ((uint16_t)currentStatus.EGT * 10) + 400;
    CAN_Tx_Msgdata[0] =  0x04;                  // sending 4 bytes
    CAN_Tx_Msgdata[1] =  0x41;                  // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
    CAN_Tx_Msgdata[2] =  0x3C;                  // pid code
    CAN_Tx_Msgdata[3] =  highByte(obdcalcA);    // A
    CAN_Tx_Msgdata[4] =  lowByte(obdcalcA);     // B 
    CAN_Tx_Msgdata[5] =  0x00; 
    CAN_Tx_Msgdata[6] =  0x00; 
    CAN_Tx_Msgdata[7] =  0x00;
  break;

  case 64:      // PIDs supported [41-60]  
    CAN_Tx_Msgdata[0] =  0x06;    // sending 4 bytes
    CAN_Tx_Msgdata[1] =  0x41;    // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
    CAN_Tx_Msgdata[2] =  0x40;    // pid code
    CAN_Tx_Msgdata[3] =  B11100000;    // 65-72dec
    CAN_Tx_Msgdata[4] =  B00000000;    // 73-80
    CAN_Tx_Msgdata[5] =  B11000000;   //  81-88
    CAN_Tx_Msgdata[6] =  B00010000;   //  89-96
    CAN_Tx_Msgdata[7] =  0x00;
  break;
  
  case 65:      //PID-0x41 Monitor status since DTCs cleared.
    CAN_Tx_Msgdata[0] =  0x08;                 // sending 8 bytes
    CAN_Tx_Msgdata[1] =  0x41;                 // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
    CAN_Tx_Msgdata[2] =  0x41;                 // pid code
    CAN_Tx_Msgdata[3] =  //0x00; //always 0 otherwise as per pid1
    CAN_Tx_Msgdata[4] =  B00000100;            //B6-B4	Bitmap indicating completeness of common tests. B3	Indication of engine type 0 = Spark ignition.  B2-B0	Bitmap indicating availability of common tests.
    CAN_Tx_Msgdata[5] =  0x00; 
    CAN_Tx_Msgdata[6] =  0x00; 
    CAN_Tx_Msgdata[7] =  0x00;
  break;

  case 66:      //PID-0x42 control module voltage, 256A+B / 1000 , range is 0 to 65.535v
    uint16_t temp_ecuBatt;
    temp_ecuBatt = currentStatus.battery10;   // create a 16bit temp variable to do the math
    obdcalcA = temp_ecuBatt*100;              // should be *1000 but ecuBatt is already *10
    CAN_Tx_Msgdata[0] =  0x04;                       // sending 4 bytes
    CAN_Tx_Msgdata[1] =  0x41;                       // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
    CAN_Tx_Msgdata[2] =  0x42;                       // pid code
    CAN_Tx_Msgdata[3] =  highByte(obdcalcA) ;        // A
    CAN_Tx_Msgdata[4] =  lowByte(obdcalcA) ;         // B
    CAN_Tx_Msgdata[5] =  0x00; 
    CAN_Tx_Msgdata[6] =  0x00; 
    CAN_Tx_Msgdata[7] =  0x00;
  break;
  
  case 67:      //PID-0x43 Absolute load value (%), 100*(256A+B) / 255 , range is 0 to 25,700%
    obdcalcA = (currentStatus.fuelLoad*255)/100;         
    CAN_Tx_Msgdata[0] =  0x04;                       // sending 4 bytes
    CAN_Tx_Msgdata[1] =  0x41;                       // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
    CAN_Tx_Msgdata[2] =  0x42;                       // pid code
    CAN_Tx_Msgdata[3] =  highByte(obdcalcA) ;        // A
    CAN_Tx_Msgdata[4] =  lowByte(obdcalcA) ;         // B
    CAN_Tx_Msgdata[5] =  0x00; 
    CAN_Tx_Msgdata[6] =  0x00; 
    CAN_Tx_Msgdata[7] =  0x00;
  break;

  case 81:      // PID-0x51 Fuel Type Coding
     uint16_t fuel_type;
     fuel_type = 1;              // This is Gasoline
     CAN_Tx_Msgdata[0] =  0x03;           // sending 3 bytes
     CAN_Tx_Msgdata[1] =  0x41;           // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
     CAN_Tx_Msgdata[2] =  0x51;           // pid code
     CAN_Tx_Msgdata[3] =  fuel_type;      // A
     CAN_Tx_Msgdata[4] =  0x00;           // B
     CAN_Tx_Msgdata[5] =  0x00; 
     CAN_Tx_Msgdata[6] =  0x00; 
     CAN_Tx_Msgdata[7] =  0x00;
   break;

  case 82:        //PID-0x52 Ethanol fuel % , range is 0 to 100% , formula == (100/255)A
    CAN_Tx_Msgdata[0] =  0x03;                       // sending 3 byte
    CAN_Tx_Msgdata[1] =  0x41;                       // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc. 
    CAN_Tx_Msgdata[2] =  0x52;                       // pid code
    CAN_Tx_Msgdata[3] =  currentStatus.ethanolPct;   // A
    CAN_Tx_Msgdata[4] =  0x00;
    CAN_Tx_Msgdata[5] =  0x00; 
    CAN_Tx_Msgdata[6] =  0x00; 
    CAN_Tx_Msgdata[7] =  0x00;
  break;
 

  case 92:        //PID-0x5C Engine oil temperature , range is -40 to 210 deg C , formula == A-40
    uint16_t temp_engineoiltemp;
    temp_engineoiltemp = 40;              // TEST VALUE !!!!!!!!!! 
    obdcalcA = temp_engineoiltemp+40 ;    // maybe later will be (byte)(currentStatus.EOT + CALIBRATION_TEMPERATURE_OFFSET)
    CAN_Tx_Msgdata[0] =  0x03;                // sending 3 byte
    CAN_Tx_Msgdata[1] =  0x41;                // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc. 
    CAN_Tx_Msgdata[2] =  0x5C;                // pid code
    CAN_Tx_Msgdata[3] =  obdcalcA ;           // A
    CAN_Tx_Msgdata[4] =  0x00;
    CAN_Tx_Msgdata[5] =  0x00; 
    CAN_Tx_Msgdata[6] =  0x00; 
    CAN_Tx_Msgdata[7] =  0x00;
  break;

  case 96:       //PIDs supported [61-80]  
    CAN_Tx_Msgdata[0] =  0x06;    // sending 4 bytes
    CAN_Tx_Msgdata[1] =  0x41;    // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
    CAN_Tx_Msgdata[2] =  0x60;    // pid code
    CAN_Tx_Msgdata[3] =  0x00;    // B0000 0000
    CAN_Tx_Msgdata[4] =  0x00;    // B0000 0000
    CAN_Tx_Msgdata[5] =  0x00;    // B0000 0000
    CAN_Tx_Msgdata[6] =  0x00;    // B0000 0000
    CAN_Tx_Msgdata[7] =  0x00;
  break;

  default:
    CAN_Tx_Msgdata[0] =  0x00;    // error unsupported PID requested, send all zeros
    CAN_Tx_Msgdata[1] =  0x00;    // error unsupported PID requested, send all zeros
    CAN_Tx_Msgdata[2] =  0x00;    // error unsupported PID requested, send all zeros
    CAN_Tx_Msgdata[3] =  0x00;    // error unsupported PID requested, send all zeros
    CAN_Tx_Msgdata[4] =  0x00;    // error unsupported PID requested, send all zeros
    CAN_Tx_Msgdata[5] =  0x00;    // error unsupported PID requested, send all zeros
    CAN_Tx_Msgdata[6] =  0x00;    // error unsupported PID requested, send all zeros
    CAN_Tx_Msgdata[7] =  0x00;    // error unsupported PID requested, send all zeros
  break;
  }
  
  //send the response
  CAN0.sendMsgBuf(OBD_ECU_RESP_ADDR, 0, 8, CAN_Tx_Msgdata); //((configPage9.obd_address + 0x100)+ 8);       
}

// This routine supports the OBDII service 22 mode (sending realtime data from custom pids) these are custom PIDs not listed in the SAE std.
void obd_Service_22(uint8_t requestedPIDlow, uint8_t requestedPIDhigh)
{  
  if (requestedPIDhigh == 0x77)
  {
    if ((requestedPIDlow >= 0x01) && (requestedPIDlow <= 0x10))
    {   
       // PID 0x01 (1 dec) to 0x10 (16 dec)
       // Aux data / can data IN Channel 1 - 16  
       CAN_Tx_Msgdata[0] =  0x06;                                               // sending 6 bytes
       CAN_Tx_Msgdata[1] =  0x62;                                               // Same as query, except that 40h is added to the mode value. So:62h = custom mode
       CAN_Tx_Msgdata[2] =  requestedPIDlow;                                 // PID code
       CAN_Tx_Msgdata[3] =  0x77;                                               // PID code
       CAN_Tx_Msgdata[4] =  lowByte(currentStatus.canin[requestedPIDlow]);   // A
       CAN_Tx_Msgdata[5] =  highByte(currentStatus.canin[requestedPIDlow]);  // B
       CAN_Tx_Msgdata[6] =  0x00;                                               // C
       CAN_Tx_Msgdata[7] =  0x00;                                               // D
    }
  }
  // this allows to get any value out of current status array.
  else if (requestedPIDhigh == 0x78)
  {
    int16_t tempValue;
    tempValue = ProgrammableIOGetData(requestedPIDlow);
    CAN_Tx_Msgdata[0] =  0x06;                 // sending 6 bytes
    CAN_Tx_Msgdata[1] =  0x62;                 // Same as query, except that 40h is added to the mode value. So:62h = custom mode
    CAN_Tx_Msgdata[2] =  requestedPIDlow;      // PID code
    CAN_Tx_Msgdata[3] =  0x78;                 // PID code
    CAN_Tx_Msgdata[4] =  lowByte(tempValue);   // A
    CAN_Tx_Msgdata[5] =  highByte(tempValue);  // B
    CAN_Tx_Msgdata[6] =  0x00; 
    CAN_Tx_Msgdata[7] =  0x00;
  }
  
  CAN0.sendMsgBuf(OBD_ECU_RESP_ADDR, 0, 8, CAN_Tx_Msgdata); //((configPage9.obd_address + 0x100)+ 8);      
}

// This routine supports the OBDII service 3 mode (current DTCs)
void obd_Service_03(void)
{
  //testing
  if (1) // DTC set
  {
    CAN_Tx_Msgdata[0] =  0x08;    //sending 8 bytes
    CAN_Tx_Msgdata[1] =  0x43;    // SID, 40h is added to the mode value. So:43h = DTC current
    CAN_Tx_Msgdata[2] =  0x01;    // DTC#1 High Byte TESTING should be P0158 byte A
    CAN_Tx_Msgdata[3] =  0x58;    // DTC#1 Low Byte  TESTING should be P0158 byte B
    CAN_Tx_Msgdata[4] =  0x00;    // DTC#2 High Byte 
    CAN_Tx_Msgdata[5] =  0x00;    // DTC#2 Low Byte
    CAN_Tx_Msgdata[6] =  0x00;    // DTC#3 High Byte
    CAN_Tx_Msgdata[7] =  0x00;    // DTC#4 Low Byte
                                 
    CAN0.sendMsgBuf(OBD_ECU_RESP_ADDR, 0, 8, CAN_Tx_Msgdata);
  }
}

// This routine supports the OBDII service 7 mode (DTC's during current or last drive cycle), should match mode 3 data.
void obd_Service_07(void)
{
  //testing
  if (1) // DTC set
  {
    CAN_Tx_Msgdata[0] =  0x08;    //sending 8 bytes
    CAN_Tx_Msgdata[1] =  0x43;    // SID, 40h is added to the mode value. So:43h = DTC current
    CAN_Tx_Msgdata[2] =  0x01;    // Number of DTC's
    CAN_Tx_Msgdata[3] =  0x01;    // DTC#1 High Byte TESTING should be P0158 byte A
    CAN_Tx_Msgdata[4] =  0x58;    // DTC#1 Low Byte  TESTING should be P0158 byte B
    CAN_Tx_Msgdata[5] =  0x00;    // DTC#2 High Byte 
    CAN_Tx_Msgdata[6] =  0x00;    // DTC#2 Low Byte
    CAN_Tx_Msgdata[7] =  0x00;    // zero, other DTC's in multiframe messages.
                                 
    CAN0.sendMsgBuf(OBD_ECU_RESP_ADDR, 0, 8, CAN_Tx_Msgdata);
  }
}

// This routine supports the OBDII service 22 mode (vehicle information)
void obd_Service_09(uint8_t requestedPIDlow)
{
  switch (requestedPIDlow)
  {
    case 0:       //PID-0x00 PIDs supported 01-20  
      CAN_Tx_Msgdata[0] =  0x06;    // sending 6 bytes
      CAN_Tx_Msgdata[1] =  0x49;    // Same as query, except that 40h is added to the mode value. So:49h = vehicle information
      CAN_Tx_Msgdata[2] =  0x00;    // PID code
      CAN_Tx_Msgdata[3] =  B01000000;   //1-8
      CAN_Tx_Msgdata[4] =  B00100000;   //9-16
      CAN_Tx_Msgdata[5] =  B00000000;   //17-24
      CAN_Tx_Msgdata[6] =  B00000000;   //25-32
      CAN_Tx_Msgdata[7] =  B00000000;
      
      CAN0.sendMsgBuf(OBD_ECU_RESP_ADDR, 0, 8, CAN_Tx_Msgdata);      
    break;
    
    case 2:       //PID-0x02 send the VIN number , 17 char long VIN sent via multiframe message.
         
      CAN_Tx_Msgdata[0] =  0x10;    //PCI
      CAN_Tx_Msgdata[1] =  0x14;    // length 20 bytes
      CAN_Tx_Msgdata[2] =  0x49;    // response SID Same as query, except that 40h is added to the mode value. So:49h = vehicle information
      CAN_Tx_Msgdata[3] =  0x02;    // data identifier (Same as PID); 
      CAN_Tx_Msgdata[4] =  0x01;    // Number Of Data Items (NODI)
      CAN_Tx_Msgdata[5] =  VehicleIdentificationNumber[0];
      CAN_Tx_Msgdata[6] =  VehicleIdentificationNumber[1];
      CAN_Tx_Msgdata[7] =  VehicleIdentificationNumber[2];
      CAN0.sendMsgBuf(OBD_ECU_RESP_ADDR, 0, 8, CAN_Tx_Msgdata);
      
      // Flow control... TBD
      
      CAN_Tx_Msgdata[0] =  0x21;  // First consecutive frame
      CAN_Tx_Msgdata[1] =  VehicleIdentificationNumber[3];
      CAN_Tx_Msgdata[2] =  VehicleIdentificationNumber[4];
      CAN_Tx_Msgdata[3] =  VehicleIdentificationNumber[5]; 
      CAN_Tx_Msgdata[4] =  VehicleIdentificationNumber[6];
      CAN_Tx_Msgdata[5] =  VehicleIdentificationNumber[7];
      CAN_Tx_Msgdata[6] =  VehicleIdentificationNumber[8];
      CAN_Tx_Msgdata[7] =  VehicleIdentificationNumber[9];
      CAN0.sendMsgBuf(OBD_ECU_RESP_ADDR, 0, 8, CAN_Tx_Msgdata);
      
      CAN_Tx_Msgdata[0] =  0x22;  // 2nd consecutive frame
      CAN_Tx_Msgdata[1] =  VehicleIdentificationNumber[10];
      CAN_Tx_Msgdata[2] =  VehicleIdentificationNumber[11];
      CAN_Tx_Msgdata[3] =  VehicleIdentificationNumber[12]; 
      CAN_Tx_Msgdata[4] =  VehicleIdentificationNumber[13];
      CAN_Tx_Msgdata[5] =  VehicleIdentificationNumber[14];
      CAN_Tx_Msgdata[6] =  VehicleIdentificationNumber[15];
      CAN_Tx_Msgdata[7] =  VehicleIdentificationNumber[16];
      CAN0.sendMsgBuf(OBD_ECU_RESP_ADDR, 0, 8, CAN_Tx_Msgdata);
    break;
    
    case 10:       //PID-0x0A send the ECU Name. Multiple messages.  
      CAN_Tx_Msgdata[0] =  0x10;    //PCI
      CAN_Tx_Msgdata[1] =  0x0D;    // Length 3 Bytes + 10 for the name
      CAN_Tx_Msgdata[2] =  0x49;    // Same as query, except that 40h is added to the mode value. So:49h = vehicle information
      CAN_Tx_Msgdata[3] =  0x0A;    // data identifier (Same as PID);
      CAN_Tx_Msgdata[4] =  0x01;    // Number Of Data Items (NODI)
      CAN_Tx_Msgdata[5] =  TSfirmwareVersion[0];
      CAN_Tx_Msgdata[6] =  TSfirmwareVersion[1];
      CAN_Tx_Msgdata[7] =  TSfirmwareVersion[2];
      CAN0.sendMsgBuf(OBD_ECU_RESP_ADDR, 0, 8, CAN_Tx_Msgdata);
      
      // Flow control... TBD
      
      CAN_Tx_Msgdata[0] =  0x21;  // First consecutive frame
      CAN_Tx_Msgdata[1] =  TSfirmwareVersion[3];
      CAN_Tx_Msgdata[2] =  TSfirmwareVersion[4];
      CAN_Tx_Msgdata[3] =  TSfirmwareVersion[5];
      CAN_Tx_Msgdata[4] =  TSfirmwareVersion[6];
      CAN_Tx_Msgdata[5] =  TSfirmwareVersion[7];
      CAN_Tx_Msgdata[6] =  TSfirmwareVersion[8];
      CAN_Tx_Msgdata[7] =  TSfirmwareVersion[9];
      CAN0.sendMsgBuf(OBD_ECU_RESP_ADDR, 0, 8, CAN_Tx_Msgdata);
    
    default:
    CAN_Tx_Msgdata[0] =  0x03;    // 3 Bytes
    CAN_Tx_Msgdata[1] =  0x7F;    // error unsupported PID requested
    CAN_Tx_Msgdata[2] =  0x49;    //  
    CAN_Tx_Msgdata[3] =  0x31;    
    CAN_Tx_Msgdata[4] =  0x00;    
    CAN_Tx_Msgdata[5] =  0x00;    
    CAN_Tx_Msgdata[6] =  0x00;    
    CAN_Tx_Msgdata[7] =  0x00;    
    CAN0.sendMsgBuf(OBD_ECU_RESP_ADDR, 0, 8, CAN_Tx_Msgdata);
    break;
  }
    
  
  //send the response
         
}

// This routine builds a DTC into a two Byte return parmeter
//uint16_t obd_BuildDTC(char DTC_Category, uint8_t DTC_Num1, uint8_t DTC_Num2, uint8_t DTC_Num3)
//{
//  uint16_t DTC_Word = 0x0000;
//  
//  switch (DTC_Category)
//  {
//    case 'P':  //Powertrain
//      DTC_Word = 0x0000;
//    break;
//    case 'C':  //Chassis
//      DTC_Word = 0x0001;
//    break;
//    case 'B':  //Body
//      DTC_Word = 0x0002;
//    break;
//    case 'U':  //Network
//      DTC_Word = 0x0003;
//    break;
//    
//    default: //error, undefined DTC
//      DTC_Word = 0x0000;
//      return DTC_Word;
//    break;
//  }
//  
//  DTC_Word = DTC_Word | DTC_Num3
//    
//}
