/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/

/** @file
Corrections to injection pulsewidth.
The corrections functions in this file affect the fuel pulsewidth (Either increasing or decreasing)
based on factors other than the VE lookup.

These factors include:
- Temperature (Warmup Enrichment and After Start Enrichment)
- Acceleration/Deceleration
- Flood clear mode
- etc.

Most correction functions return value 100 (like 100% == 1) for no need for correction.

There are 2 top level functions that call more detailed corrections for Fuel and Ignition respectively:
- @ref correctionsFuel() - All fuel related corrections
- @ref correctionsIgn() - All ignition related corrections
*/
//************************************************************************************************************

#include "globals.h"
#include "corrections.h"
#include "speeduino.h"
#include "timers.h"
#include "maths.h"
#include "sensors.h"


byte activateMAPDOT; //The mapDOT value seen when the MAE was activated. 
byte activateTPSDOT; //The tpsDOT value seen when the MAE was activated.

uint16_t ego_NextCycleCount;
bool idleAdvActive = false;
unsigned long knockStartTime;
byte lastKnockCount;
int16_t knockWindowMin; //The current minimum crank angle for a knock pulse to be valid
int16_t knockWindowMax;//The current maximum crank angle for a knock pulse to be valid
uint32_t dfcoStateStrtTm;
uint8_t aseTaper;
uint8_t idleAdvTaper;
uint8_t crankingEnrichTaper;
bool O2_SensorIsRich;
bool O2_SensorIsRichPrev;
bool O2_2ndSensorIsRich;
bool O2_2ndSensorIsRichPrev;
int16_t ego_FuelLoadPrev;
uint32_t ego_FreezeEndTime;
int8_t ego_Integral;
int8_t ego2_Integral;
uint32_t ego_DelaySensorTime;
uint8_t ego_IntDelayLoops;
uint8_t ego2_IntDelayLoops;
uint8_t DFCO_State;
uint16_t dfcoFuelStartIgns;
unsigned long pwLimit;


/** Initialise instances and vars related to corrections (at ECU boot-up).
 */
void initialiseCorrections()
{
  currentStatus.corrections = 100;
  currentStatus.flexIgnCorrection = 0;
  currentStatus.egoCorrection = 100; //Default value of no adjustment must be set to avoid randomness on first correction cycle after startup
  currentStatus.ego2Correction = 100; //Default value of no adjustment must be set to avoid randomness on first correction cycle after startup
  currentStatus.afrTarget = configPage2.stoich; // Init AFR Target at stoich.
  ego_NextCycleCount = 0;
  ego_FreezeEndTime = 0;
  ego_DelaySensorTime = 0;
  ego_Integral = 0;
  ego2_Integral = 0;
  O2_SensorIsRich = false;
  O2_SensorIsRichPrev = O2_SensorIsRich;
  O2_2ndSensorIsRich = false;
  O2_2ndSensorIsRichPrev = O2_2ndSensorIsRich;
  BIT_SET(currentStatus.status4, BIT_STATUS4_EGO_FROZEN);
  ego_FuelLoadPrev = currentStatus.fuelLoad;
  currentStatus.knockActive = false;
  currentStatus.battery10 = 125; //Set battery voltage to sensible value for dwell correction for "flying start" (else ignition gets suprious pulses after boot)
  DFCO_State = DFCO_OFF;  
}

/** Dispatch calculations for all fuel related corrections.
Calls all the other corrections functions and combines their results.
This is the only function that should be called from anywhere outside the file
*/
uint16_t correctionsFuel()
{
  uint32_t sumCorrections = 100;
  uint16_t result; //temporary variable to store the result of each corrections function

  //The values returned by each of the correction functions are multiplied together and then divided back to give a single 0-255 value.
  currentStatus.wueCorrection = correctionWUE();
  if (currentStatus.wueCorrection != 100) { sumCorrections = div100(sumCorrections * currentStatus.wueCorrection); }

  result = correctionCrankingASE();
  if (result != 100) { sumCorrections = div100(sumCorrections * result); }

  currentStatus.AEamount = correctionAccel();
  if (configPage2.aeApplyMode == AE_MODE_MULTIPLIER)
  {
  if (currentStatus.AEamount != 100) { sumCorrections = div100(sumCorrections * currentStatus.AEamount);}
  }

  result = correctionFloodClear();
  if (result != 100) { sumCorrections = div100(sumCorrections * result); }

  currentStatus.egoCorrection = correctionAFRClosedLoop();
  if (currentStatus.egoCorrection != 100) { sumCorrections = div100(sumCorrections * currentStatus.egoCorrection); }

  currentStatus.batCorrection = correctionBatVoltage();
  if (configPage2.battVCorMode == BATTV_COR_MODE_OPENTIME)
  {
    inj_opentime_uS = (configPage2.injOpen * currentStatus.batCorrection) / 10 ; // Apply voltage correction to injector open time.
    currentStatus.batCorrection = 100; // This is to ensure that the correction is not applied twice. There is no battery correction fator as we have instead changed the open time
  }
  if (configPage2.battVCorMode == BATTV_COR_MODE_WHOLE)
  {
    if (currentStatus.batCorrection != 100) { sumCorrections = div100(sumCorrections * currentStatus.batCorrection); }  
    inj_opentime_uS = configPage2.injOpen * 10; // Set injector open time here so it's in one place
  }

  currentStatus.iatCorrection = correctionIATDensity();
  if (currentStatus.iatCorrection != 100) { sumCorrections = div100(sumCorrections * currentStatus.iatCorrection); }

  currentStatus.baroCorrection = correctionBaro();
  if (currentStatus.baroCorrection != 100) { sumCorrections = div100(sumCorrections * currentStatus.baroCorrection); }

  currentStatus.flexCorrection = correctionFlex();
  if (currentStatus.flexCorrection != 100) { sumCorrections = div100(sumCorrections * currentStatus.flexCorrection); }

  currentStatus.fuelTempCorrection = correctionFuelTemp();
  if (currentStatus.fuelTempCorrection != 100) { sumCorrections = div100(sumCorrections * currentStatus.fuelTempCorrection); }
  
  currentStatus.injPressCorrection = correctionFuelPress();
  //if (currentStatus.injPressCorrection != 100) { sumCorrections = div100(sumCorrections * currentStatus.injPressCorrection); } - Move this to ReqFuel Calc

  currentStatus.launchCorrection = correctionLaunch();
  if (currentStatus.launchCorrection != 100) { sumCorrections = div100(sumCorrections * currentStatus.launchCorrection); }
  
  result = correctionDFCO();
  if (result != 100) { sumCorrections = div100(sumCorrections * result); }

  if(sumCorrections > 1500) { sumCorrections = 1500; } //This is the maximum allowable increase during cranking
  return (uint16_t)sumCorrections;
}


/** Warm Up Enrichment (WUE) corrections.
Uses a 2D enrichment table (WUETable) where the X axis is engine temp and the Y axis is the amount of extra fuel to add
*/
byte correctionWUE()
{
  byte WUEValue;
  //Possibly reduce the frequency this runs at (Costs about 50 loops per second)
  //if (currentStatus.coolant > (WUETable.axisX[9] - CALIBRATION_TEMPERATURE_OFFSET))
  if (currentStatus.coolant > (table2D_getAxisValue(&WUETable, 9) - CALIBRATION_TEMPERATURE_OFFSET))
  {
    //This prevents us doing the 2D lookup if we're already up to temp
    BIT_CLEAR(currentStatus.engine, BIT_ENGINE_WARMUP);
    WUEValue = table2D_getRawValue(&WUETable, 9);
  }
  else
  {
    BIT_SET(currentStatus.engine, BIT_ENGINE_WARMUP);
    WUEValue = table2D_getValue(&WUETable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET);
  }

  return WUEValue;
}

/** Cranking and Afterstart Enrichment corrections.
Additional fuel % to be added when the engine is cranking and short time after starting
*/
uint16_t correctionCrankingASE()
{
  uint16_t crankingASEValue = 100;
  
  if( BIT_CHECK(LOOP_TIMER, BIT_TIMER_10HZ) ) // Tapers only run once every 0.1 sec. 
  {
    if ( BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) == true) // Cranking 
    {
      crankingASEValue = table2D_getValue(&crankingEnrichTable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET);
      crankingASEValue = (uint16_t) crankingASEValue * 5; //multiplied by 5 to get range from 0% to 1275%
      crankingEnrichTaper = 0;
      BIT_CLEAR(currentStatus.engine, BIT_ENGINE_ASE); //Mark ASE as inactive.
      currentStatus.ASEValue = 100;
    }
    else // Running
    {
      //Calculate ASE
      if (currentStatus.runSecs < (table2D_getValue(&ASECountTable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET))) // In ASE Hold Mode
      {
        BIT_SET(currentStatus.engine, BIT_ENGINE_ASE); //Mark ASE as active.
        currentStatus.ASEValue = 100 + table2D_getValue(&ASETable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET);
        aseTaper = 0; // ASE taper only reset here to prevent re-init of ASE if the engine speed drops to crank later.
      }
      else if (aseTaper < configPage2.aseTaperTime) // In ASE taper mode
      {
        BIT_SET(currentStatus.engine, BIT_ENGINE_ASE); //Mark ASE as active.
        currentStatus.ASEValue = 100 + map(aseTaper, 0, configPage2.aseTaperTime, table2D_getValue(&ASETable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET), 0);
        aseTaper++;
      }
      else
      {
        BIT_CLEAR(currentStatus.engine, BIT_ENGINE_ASE); //Mark ASE as inactive.
        currentStatus.ASEValue = 100;
      }
      
      // Calculate Cranking as long as the taper is still got time to run.
      if ( crankingEnrichTaper < configPage10.crankingEnrichTaper )
      {
        crankingASEValue = table2D_getValue(&crankingEnrichTable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET);
        crankingASEValue = (uint16_t) crankingASEValue * 5; //multiplied by 5 to get range from 0% to 1275%
        crankingASEValue = (uint16_t) map(crankingEnrichTaper, 0, configPage10.crankingEnrichTaper, crankingASEValue, 100); //Taper from start value to 100%
        crankingEnrichTaper++;
      }
    }
    
    if (crankingASEValue < 100) { crankingASEValue = 100; } //Sanity check
    currentStatus.crankCorrection = crankingASEValue; // Save to global variable so the value is available when not in the 10Hz loop. This is only cranking enrichment at this point.
  } // End 10Hz Loop

  // Outside 10hz loop evaluate on global variables which retain values from last 10Hz loop.
  if (currentStatus.ASEValue > currentStatus.crankCorrection) { crankingASEValue = currentStatus.ASEValue;  } // Max value from cranking or ASE, also provides protection angainst values < 100
  else { crankingASEValue = currentStatus.crankCorrection; }
  
  return crankingASEValue;
}

/** Cranking Enrichment corrections.
Additional fuel % to be added when the engine is cranking
*/
uint16_t correctionCranking()
{
  uint16_t crankingValue = 100;
  //Check if we are actually cranking
  if ( BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) )
  {
    crankingValue = table2D_getValue(&crankingEnrichTable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET);
    crankingValue = (uint16_t) crankingValue * 5; //multiplied by 5 to get range from 0% to 1275%
    crankingEnrichTaper = 0;
  }
  
  //If we're not cranking, check if if cranking enrichment tapering to ASE should be done
  else if ( crankingEnrichTaper < configPage10.crankingEnrichTaper )
  {
    crankingValue = table2D_getValue(&crankingEnrichTable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET);
    crankingValue = (uint16_t) crankingValue * 5; //multiplied by 5 to get range from 0% to 1275%
    //Taper start value needs to account for ASE that is now running, so total correction does not increase when taper begins
    unsigned long taperStart = (unsigned long) crankingValue * 100 / currentStatus.ASEValue;
    crankingValue = (uint16_t) map(crankingEnrichTaper, 0, configPage10.crankingEnrichTaper, taperStart, 100); //Taper from start value to 100%
    if (crankingValue < 100) { crankingValue = 100; } //Sanity check
    if( BIT_CHECK(LOOP_TIMER, BIT_TIMER_10HZ) ) { crankingEnrichTaper++; }
  }
  return crankingValue;
}

/** After Start Enrichment calculation.
 * This is a short period (Usually <20 seconds) immediately after the engine first fires (But not when cranking)
 * where an additional amount of fuel is added (Over and above the WUE amount).
 * 
 * @return uint8_t The After Start Enrichment modifier as a %. 100% = No modification. 
 */
byte correctionASE()
{
  int16_t ASEValue;
  //Two checks are required:
  //1) Is the engine run time less than the configured ase time
  //2) Make sure we're not still cranking
  if ((currentStatus.runSecs < (table2D_getValue(&ASECountTable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET))) && 
      (BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) == false ) )
  {
    BIT_SET(currentStatus.engine, BIT_ENGINE_ASE); //Mark ASE as active.
    ASEValue = 100 + table2D_getValue(&ASETable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET);
    aseTaper = 0;
  }
  else
  {
    if ( (BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) == false) && (aseTaper < configPage2.aseTaperTime) ) //Cranking check needs to be here also, so cranking and afterstart enrichments won't run simultaneously
    {
      BIT_SET(currentStatus.engine, BIT_ENGINE_ASE); //Mark ASE as active.
      ASEValue = 100 + map(aseTaper, 0, configPage2.aseTaperTime, table2D_getValue(&ASETable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET), 0);
      if ( BIT_CHECK(LOOP_TIMER, BIT_TIMER_10HZ)) { aseTaper++; }
    }
    else
    {
      BIT_CLEAR(currentStatus.engine, BIT_ENGINE_ASE); //Mark ASE as inactive.
      ASEValue = 100;
    }
  }
  
  //Safety checks
  if(ASEValue > 255) { ASEValue = 255; }
  if(ASEValue < 0) { ASEValue = 0; }
  currentStatus.ASEValue = (byte)ASEValue;
    
  return currentStatus.ASEValue;
}

/** Acceleration enrichment correction calculation.
 * 
 * Calculates the % change of the throttle over time (%/second) and performs a lookup based on this
 * Coolant-based modifier is applied on the top of this.
 * When the enrichment is turned on, it runs at that amount for a fixed period of time (taeTime)
 * 
 * @return uint16_t The Acceleration enrichment modifier as a %. 100% = No modification.
 * 
 * As the maximum enrichment amount is +255% and maximum cold adjustment for this is 255%, the overall return value
 * from this function can be 100+(255*255/100)=750. Hence this function returns a uint16_t rather than byte.
 */
uint16_t correctionAccel()
{
  int16_t accelValue = 100;
  int16_t MAP_change = 0;
  int16_t TPS_change = 0;
  uint8_t tpsDOTTimeFiltIdx = 2;
  int16_t MAP_rateOfChange = 0;
  int16_t TPS_rateOfChange = 0;


  if(configPage2.aeMode == AE_MODE_MAP)
  {
    //Get the MAP rate change
    MAP_change = (currentStatus.MAP - MAPlast);
    currentStatus.mapDOT = ldiv(1000000, (MAP_time - MAPlast_time)).quot * MAP_change; //This is the % per second that the MAP has moved
    //currentStatus.mapDOT = 15 * MAP_change; //This is the kpa per second that the MAP has moved
  }
  else if(configPage2.aeMode == AE_MODE_TPS)
  {
    //Get the TPS rate change
   
    tpsDOTTimeFiltIdx = configPage15.tpsDOTTimeFilt; // Time base for calculating TPS Dot, 0 = 3 loops ago, 1 = 2 loops ago, 2 = previous loop.
    if((tpsDOTTimeFiltIdx >= AE_TPS_DOT_HIST_BINS) ) { tpsDOTTimeFiltIdx = (AE_TPS_DOT_HIST_BINS - 1); } // Protection for array indexing if eeprom not set. 
    TPS_change = (currentStatus.TPS - tpsHistory[tpsDOTTimeFiltIdx]); // This provides an option to calculate TPS change over a longer time period. Longer time base = better fidelity at slow signals. Shorter Time base = better fidelity at fast signals.
    TPS_rateOfChange = (TPS_READ_FREQUENCY * TPS_change) / (2*(AE_TPS_DOT_HIST_BINS - tpsDOTTimeFiltIdx)); //This is the % per second that the TPS has moved
    
    //TPS_change = (currentStatus.TPS - currentStatus.TPSlast);
    //currentStatus.tpsDOT = ldiv(1000000, (TPS_time - TPSlast_time)).quot * TPS_change; //This is the % per second that the TPS has moved
    //currentStatus.tpsDOT = (TPS_READ_FREQUENCY * TPS_change) / 2; //This is the % per second that the TPS has moved, adjusted for the 0.5% resolution of the TPS
  }
  

  //First, check whether the accel. enrichment is already running
  if( BIT_CHECK(currentStatus.engine, BIT_ENGINE_ACC) || BIT_CHECK(currentStatus.engine, BIT_ENGINE_DCC))
  {
    //If it is currently running, check whether it should still be running or whether it's reached it's end time
    if( micros_safe() >= currentStatus.AEEndTime )
    {
      //Time to turn enrichment off
      BIT_CLEAR(currentStatus.engine, BIT_ENGINE_ACC);
      BIT_CLEAR(currentStatus.engine, BIT_ENGINE_DCC);
      currentStatus.AEamount = 0;
      accelValue = 100;

      //Reset the relevant DOT value to 0
      //if(configPage2.aeMode == AE_MODE_MAP) { currentStatus.mapDOT = 0; }
      //else if(configPage2.aeMode == AE_MODE_TPS) { currentStatus.tpsDOT = 0; }
    }
    else
    {
      //Enrichment still needs to keep running. 
      //Simply return the total TAE amount
      accelValue = currentStatus.AEamount;

      //Need to check whether the accel amount has increased from when AE was turned on
      //If the accel amount HAS increased, we clear the current enrich phase and a new one will be started below
      if( (configPage2.aeMode == AE_MODE_MAP) && (abs(currentStatus.mapDOT) > activateMAPDOT) )
      {
        BIT_CLEAR(currentStatus.engine, BIT_ENGINE_ACC);
        BIT_CLEAR(currentStatus.engine, BIT_ENGINE_DCC);
      }
      else if( (configPage2.aeMode == AE_MODE_TPS) && (abs(currentStatus.tpsDOT) > activateTPSDOT) )
      {
        BIT_CLEAR(currentStatus.engine, BIT_ENGINE_ACC);
        BIT_CLEAR(currentStatus.engine, BIT_ENGINE_DCC);
      }
    }
  }

  if( !BIT_CHECK(currentStatus.engine, BIT_ENGINE_ACC) && !BIT_CHECK(currentStatus.engine, BIT_ENGINE_DCC)) //Need to check this again as it may have been changed in the above section (Both ACC and DCC are off if this has changed)
  {
    if(configPage2.aeMode == AE_MODE_MAP)
    {
      if (abs(MAP_change) <= configPage2.maeMinChange)
      {
        accelValue = 100;
        currentStatus.mapDOT = 0;
      }
      else
      {
        //If MAE isn't currently turned on, need to check whether it needs to be turned on
        if (abs(currentStatus.mapDOT) > configPage2.maeThresh)
        {
          activateMAPDOT = abs(currentStatus.mapDOT);
          currentStatus.AEEndTime = micros_safe() + ((unsigned long)configPage2.aeTime * 10000); //Set the time in the future where the enrichment will be turned off. taeTime is stored as mS / 10, so multiply it by 100 to get it in uS
          //Check if the MAP rate of change is negative or positive. Negative means decelarion.
          if (currentStatus.mapDOT < 0)
          {
            BIT_SET(currentStatus.engine, BIT_ENGINE_DCC); //Mark deceleration enleanment as active.
            accelValue = configPage2.decelAmount; //In decel, use the decel fuel amount as accelValue
          } //Deceleration
          //Positive MAP rate of change is acceleration.
          else
          {
            BIT_SET(currentStatus.engine, BIT_ENGINE_ACC); //Mark acceleration enrichment as active.
            accelValue = table2D_getValue(&maeTable, currentStatus.mapDOT / 10); //The x-axis of mae table is divided by 10 to fit values in byte.
  
            //Apply the RPM taper to the above
            //The RPM settings are stored divided by 100:
            uint16_t trueTaperMin = configPage2.aeTaperMin * 100;
            uint16_t trueTaperMax = configPage2.aeTaperMax * 100;
            if (currentStatus.RPM > trueTaperMin)
            {
              if(currentStatus.RPM > trueTaperMax) { accelValue = 0; } //RPM is beyond taper max limit, so accel enrich is turned off
              else 
              {
                int16_t taperRange = trueTaperMax - trueTaperMin;
                int16_t taperPercent = ((currentStatus.RPM - trueTaperMin) * 100UL) / taperRange; //The percentage of the way through the RPM taper range
                accelValue = percentage((100-taperPercent), accelValue); //Calculate the above percentage of the calculated accel amount. 
              }
            }
  
            //Apply AE cold coolant modifier, if CLT is less than taper end temperature
            if ( currentStatus.coolant < (int)(configPage2.aeColdTaperMax - CALIBRATION_TEMPERATURE_OFFSET) )
            {
              //If CLT is less than taper min temp, apply full modifier on top of accelValue
              if ( currentStatus.coolant <= (int)(configPage2.aeColdTaperMin - CALIBRATION_TEMPERATURE_OFFSET) )
              {
                uint16_t accelValue_uint = percentage(configPage2.aeColdPct, accelValue);
                accelValue = (int16_t) accelValue_uint;
              }
              //If CLT is between taper min and max, taper the modifier value and apply it on top of accelValue
              else
              {
                int16_t taperRange = (int16_t) configPage2.aeColdTaperMax - configPage2.aeColdTaperMin;
                int16_t taperPercent = (int)((currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET - configPage2.aeColdTaperMin) * 100) / taperRange;
                int16_t coldPct = (int16_t) 100 + percentage( (100-taperPercent), (configPage2.aeColdPct-100) );
                uint16_t accelValue_uint = (uint16_t) accelValue * coldPct / 100; //Potential overflow (if AE is large) without using uint16_t (percentage() may overflow)
                accelValue = (int16_t) accelValue_uint;
              }
            }
            accelValue = 100 + accelValue; //In case of AE, add the 100 normalisation to the calculated amount
          }
        } //MAE Threshold
      } //MAP change threshold
    } //AE Mode
    else if(configPage2.aeMode == AE_MODE_TPS)
    {
      //Check for only very small movement. This not only means we can skip the lookup, but helps reduce false triggering around 0-2% throttle openings
      if (abs(TPS_change) <= configPage2.taeMinChange)
      {
        accelValue = 100;
        currentStatus.tpsDOT = 0;
      }
      else
      {
        //If TAE isn't currently turned on, need to check whether it needs to be turned on
        if (abs(currentStatus.tpsDOT) > configPage2.taeThresh)
        {
          activateTPSDOT = abs(currentStatus.tpsDOT);
          currentStatus.AEEndTime = micros_safe() + ((unsigned long)configPage2.aeTime * 10000); //Set the time in the future where the enrichment will be turned off. taeTime is stored as mS / 10, so multiply it by 100 to get it in uS
          //Check if the TPS rate of change is negative or positive. Negative means decelarion.
          if (currentStatus.tpsDOT < 0)
          {
            BIT_SET(currentStatus.engine, BIT_ENGINE_DCC); //Mark deceleration enleanment as active.
            accelValue = configPage2.decelAmount; //In decel, use the decel fuel amount as accelValue
          } //Deceleration
          //Positive TPS rate of change is Acceleration.
          else
          {
            BIT_SET(currentStatus.engine, BIT_ENGINE_ACC); //Mark acceleration enrichment as active.
            accelValue = table2D_getValue(&taeTable, currentStatus.tpsDOT / 10); //The x-axis of tae table is divided by 10 to fit values in byte.
            //Apply the RPM taper to the above
            //The RPM settings are stored divided by 100:
            uint16_t trueTaperMin = configPage2.aeTaperMin * 100;
            uint16_t trueTaperMax = configPage2.aeTaperMax * 100;
            if (currentStatus.RPM > trueTaperMin)
            {
              if(currentStatus.RPM > trueTaperMax) { accelValue = 0; } //RPM is beyond taper max limit, so accel enrich is turned off
              else 
              {
                int16_t taperRange = trueTaperMax - trueTaperMin;
                int16_t taperPercent = ((currentStatus.RPM - trueTaperMin) * 100UL) / taperRange; //The percentage of the way through the RPM taper range
                accelValue = percentage( (100 - taperPercent), accelValue); //Calculate the above percentage of the calculated accel amount. 
              }
            }
  
            //Apply AE cold coolant modifier, if CLT is less than taper end temperature
            if ( currentStatus.coolant < (int)(configPage2.aeColdTaperMax - CALIBRATION_TEMPERATURE_OFFSET) )
            {
              //If CLT is less than taper min temp, apply full modifier on top of accelValue
              if ( currentStatus.coolant <= (int)(configPage2.aeColdTaperMin - CALIBRATION_TEMPERATURE_OFFSET) )
              {
                uint16_t accelValue_uint = percentage(configPage2.aeColdPct, accelValue);
                accelValue = (int16_t) accelValue_uint;
              }
              //If CLT is between taper min and max, taper the modifier value and apply it on top of accelValue
              else
              {
                int16_t taperRange = (int16_t) configPage2.aeColdTaperMax - configPage2.aeColdTaperMin;
                int16_t taperPercent = (int)((currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET - configPage2.aeColdTaperMin) * 100) / taperRange;
                int16_t coldPct = (int16_t)100 + percentage( (100 - taperPercent), (configPage2.aeColdPct-100) );
                uint16_t accelValue_uint = (uint16_t) accelValue * coldPct / 100; //Potential overflow (if AE is large) without using uint16_t
                accelValue = (int16_t) accelValue_uint;
              }
            }
            accelValue = 100 + accelValue; //In case of AE, add the 100 normalisation to the calculated amount
          } //Acceleration
        } //TAE Threshold
      } //TPS change threshold
    } //AE Mode
  } //AE active

  return accelValue;
}

/** Simple check to see whether we are cranking with the TPS above the flood clear threshold.
@return 100 (not cranking and thus no need for flood-clear) or 0 (Engine cranking and TPS above @ref config4.floodClear limit).
*/
byte correctionFloodClear()
{
  byte floodValue = 100;
  if( BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) )
  {
    //Engine is currently cranking, check what the TPS is
    if(currentStatus.TPS >= configPage4.floodClear)
    {
      //Engine is cranking and TPS is above threshold. Cut all fuel
      floodValue = 0;
    }
  }
  return floodValue;
}

/** Battery Voltage correction.
Uses a 2D enrichment table (WUETable) where the X axis is engine temp and the Y axis is the amount of extra fuel to add.
*/
byte correctionBatVoltage()
{
  byte batValue = 100;
  batValue = table2D_getValue(&injectorVCorrectionTable, currentStatus.battery10);
  return batValue;
}

/** Simple temperature based corrections lookup based on the inlet air temperature (IAT).
This corrects for changes in air density from movement of the temperature.
*/
byte correctionIATDensity()
{
  byte IATValue = 100;
  IATValue = table2D_getValue(&IATDensityCorrectionTable, currentStatus.IAT + CALIBRATION_TEMPERATURE_OFFSET); //currentStatus.IAT is the actual temperature, values in IATDensityCorrectionTable.axisX are temp+offset

  return IATValue;
}

/** Correction for current barometric / ambient pressure.
 * @returns A percentage value indicating the amount the fuelling should be changed based on the barometric reading. 100 = No change. 110 = 10% increase. 90 = 10% decrease
 */
byte correctionBaro()
{
  byte baroValue = 100;
  baroValue = table2D_getValue(&baroFuelTable, currentStatus.baro);

  return baroValue;
}

/** Launch control has a setting to increase the fuel load to assist in bringing up boost.
This simple check applies the extra fuel if we're currently launching
*/
byte correctionLaunch()
{
  byte launchValue = 100;
  if(currentStatus.launchingHard || currentStatus.launchingSoft) { launchValue = (100 + configPage6.lnchFuelAdd); }

  return launchValue;
}

/*
 * Returns correction for DFCO. Mostly 0 or 100.
 * DFCO_State is used by the spark side to ramp from and to DFCO Spark.
 */
byte correctionDFCO()
{
  byte DFCOValue = 100;
  byte dfcoGearEnabled = false;
  byte dfcoClutchEnabled = false;
  byte dfcoGeneralEnable = false;
  
  if (configPage2.dfcoEnabled == true)
  {
    // Gear Check
    switch (currentStatus.gear)
    {
      case 1:
        if (configPage9.dfcoEnblGear1 == true) { dfcoGearEnabled = true; }
        break;
      case 2:
        if (configPage9.dfcoEnblGear2 == true) { dfcoGearEnabled = true; }
        break;
      case 3:
        if (configPage9.dfcoEnblGear3 == true) { dfcoGearEnabled = true; }
        break;
      case 4:
        if (configPage9.dfcoEnblGear4 == true) { dfcoGearEnabled = true; }
        break;
      case 5:
        if (configPage9.dfcoEnblGear5 == true) { dfcoGearEnabled = true; }
        break;
      case 6:
        if (configPage9.dfcoEnblGear6 == true) { dfcoGearEnabled = true; }
        break;
      default:
        dfcoGearEnabled = true; // Set true here in the case the vehicle does not have VSS and/or no gear detection.
        break;
    }
    
    // Clutch Check
    if ((configPage9.dfcoDsblwClutch == true) && (clutchTrigger == true)) { dfcoClutchEnabled = false; } 
    else {  dfcoClutchEnabled = true; }
    
    if ((currentStatus.TPS < configPage4.dfcoTPSThresh ) && 
        (currentStatus.coolant >= (int)(configPage2.dfcoMinCLT - CALIBRATION_TEMPERATURE_OFFSET)) &&
        (currentStatus.vss >= configPage9.dfcoMinVss) &&
        (dfcoGearEnabled == true ) &&
        (dfcoClutchEnabled == true ))
      {
        dfcoGeneralEnable = true; // all conditions to start or force an exit of DFCO except rpm which is handled differently because of hysteresis
      }
    
    // Begin DFCO state machine
    if (DFCO_State == DFCO_OFF) //DFCO off, waiting for ramp in conditions
    {
      DFCOValue = 100;
      if ((currentStatus.RPM > (unsigned int)((configPage4.dfcoRPM * 10) + configPage4.dfcoHyster)) &&
          (dfcoGeneralEnable == true))
      {
        DFCO_State = DFCO_ENABLE_DELAY;
        dfcoStateStrtTm = runSecsX10;
      }        
    }
    
    if (DFCO_State == DFCO_ENABLE_DELAY) // Delay before ramp-in starts
    {
      DFCOValue = 100;
      if (( currentStatus.RPM < ( configPage4.dfcoRPM * 10)) || (dfcoGeneralEnable == false)) { DFCO_State = DFCO_OFF ; }
      if ((runSecsX10 - dfcoStateStrtTm) >= configPage2.dfcoStartDelay ) 
      { 
        dfcoStateStrtTm = runSecsX10;
        DFCO_State = DFCO_RAMP_IN; 
      } 
    }
    
    if (DFCO_State == DFCO_RAMP_IN) // Ramp spark to the DFCO fuel off spark to reduce torque while keeping fuel on for the delay duration.
    {
      DFCOValue = 100;
      if (( currentStatus.RPM < ( configPage4.dfcoRPM * 10)) || (dfcoGeneralEnable == false)) { DFCO_State = DFCO_OFF ; }
      if ((runSecsX10 - dfcoStateStrtTm) >= configPage9.dfcoRampInTime ) { DFCO_State = DFCO_ACTIVE; }
    }
    
    if (DFCO_State == DFCO_ACTIVE) // No Fuel, Waiting for ramp out conditions
    { 
      DFCOValue = 0;
      if (( currentStatus.RPM < ( configPage4.dfcoRPM * 10)) || (dfcoGeneralEnable == false))
      { 
        DFCO_State = DFCO_RAMP_OUT; 
        dfcoStateStrtTm = runSecsX10;
        dfcoFuelStartIgns = ignitionCount; // Save the cylinder we last fired.
      }
    }
    
    if (DFCO_State == DFCO_RAMP_OUT) //Spark blending happening here back to normal and enrichment applied to recover wall wetting and catalyst HC.
    {
      if ( (runSecsX10 - dfcoStateStrtTm) > configPage9.dfcoRampOutTime ) { DFCO_State = DFCO_OFF; DFCOValue = 100; }
      else if (configPage9.dfcoExitFuelTime == 1) { DFCOValue = configPage9.dfcoExitFuel; } // DFCO exit fuel applied for the duration of the exit ramp if selected via cal.
      else if ((ignitionCount - dfcoFuelStartIgns) < (2 * configPage2.nCylinders)) { DFCOValue = configPage9.dfcoExitFuel; } // Cannot be longer than dfcoRampOutTime, hardcoded to each cylinder firing twice
      else { DFCOValue = 100; }
    }      
  }
  else { DFCO_State = DFCO_OFF; } // Calibration disable
  
  if (DFCOValue == 0) { bitWrite(currentStatus.status1, BIT_STATUS1_DFCO, 1); } //Set DFCO enabled bit
  else { bitWrite(currentStatus.status1, BIT_STATUS1_DFCO, 0); }
    
  return DFCOValue;
}

/** Flex fuel adjustment to vary fuel based on ethanol content.
 * The amount of extra fuel required is a linear relationship based on the % of ethanol.
*/
byte correctionFlex()
{
  byte flexValue = 100;

  if (configPage2.flexEnabled == 1)
  {
    flexValue = table2D_getValue(&flexFuelTable, currentStatus.ethanolPct);
  }
  return flexValue;
}

/*
 * Fuel temperature adjustment to vary fuel based on fuel temperature reading
*/
byte correctionFuelTemp()
{
  byte fuelTempValue = 100;

  if (configPage2.flexEnabled == 1)
  {
    fuelTempValue = table2D_getValue(&fuelTempTable, currentStatus.fuelTemp + CALIBRATION_TEMPERATURE_OFFSET);
  }
  return fuelTempValue;
}

/*
 * Fuel Pressure adjustment to compensate for injector delta pressure.
 * The high side pressure reference and low side pressure reference are user calibrated
 * In the event of a fault with the sensors the fuel pressure defaults to the reference value.
*/
byte correctionFuelPress()
{
  byte fuelPressValue = 100;

  if (configPage10.injDeltaPressEnbl > 0)
  {
    int16_t fPress_InjTip = 101; // default 101kpa (atmosphere)
    int16_t fPress_InjSupply = 501; // default 501kpa (4bar or 60PSI)
    
    /* ******INJECTOR TIP PRESSURE CONFIGURATION ******** */
    if ((configPage10.fPress_InjTipRef == FPRESS_REF_MAP) && (mapErrorCount == 0)) { fPress_InjTip = currentStatus.MAP; }
    else if (configPage10.fPress_InjTipRef == FPRESS_REF_BARO) { fPress_InjTip = currentStatus.baro; }
    else { fPress_InjTip = 101; } // Fixed Default for faults
    
    /* ******INJECTOR SUPPLY PRESSURE CONFIGURATION ******** */
    if (configPage10.fPress_InjSupplyRef == FPRESS_REF_SENSOR) 
    { 
      fPress_InjSupply = currentStatus.fuelPressure;
      if (configPage10.fPress_SensorType == FPRESS_TYPE_GAUGE) { fPress_InjSupply = fPress_InjSupply + currentStatus.baro; } // If gauge pressure need to convert to Absolute to compare with MAP / BARO
    } // Fuel pressure from sensor handles its own default value when there is a fault
    else
    {
      if ((configPage10.fPress_InjSupplyRef == FPRESS_REF_MAP)  && (mapErrorCount == 0)) { fPress_InjSupply = currentStatus.MAP + configPage10.fPress_RefPress; }
      else if (configPage10.fPress_InjSupplyRef == FPRESS_REF_BARO) { fPress_InjSupply = currentStatus.baro + configPage10.fPress_RefPress; }
      else { fPress_InjSupply = 101 + configPage10.fPress_RefPress; } // Fixed uses refernce value plus 101, making the reference value gauge pressure.
      
      if (configPage10.fPress_SensorType == FPRESS_TYPE_ABS) { fPress_InjSupply = fPress_InjSupply - 101; } // If user preferrs absolute pressure units then need to modify the refPress variable to remove this.
    }
    
    // Calculate pressure accross the injectors and use lookup table for compensation factor.
    if (fPress_InjSupply > fPress_InjTip) { currentStatus.InjectorDeltaPress = fPress_InjSupply - fPress_InjTip; } // prevent underflow
    else { currentStatus.InjectorDeltaPress = 0; }
    
    fuelPressValue = table2D_getValue(&injPressTable, currentStatus.InjectorDeltaPress);
  }
  else { currentStatus.InjectorDeltaPress = configPage10.fPress_RefPress; } // Set to reference pressure
  return fuelPressValue;
}


/*
* Closed loop using Oxygen Sensors. Lookup the AFR target table and perform a Proportional + Integral fueling adjustment based on this.
*/
byte correctionAFRClosedLoop()
{
  byte ego_AdjustPct = 100;
  byte ego2_AdjustPct = 100;
  uint8_t O2_Error;
  int8_t ego_Prop;
  int8_t ego2_Prop;
  bool ego_EngineCycleCheck = false;
  
  /*Note that this should only run after the sensor warmup delay when using Include AFR option, but this is protected in the main loop where it's used so really don't need this.
   * When using Incorporate AFR option it needs to be done at all times
  */
  // Start AFR Target Determination
  if((configPage2.incorporateAFR == true) || 
     ((configPage6.egoType > 0) &&
      (currentStatus.runSecs > configPage6.egoStartdelay))) //afrTarget value lookup must be done if O2 sensor is enabled, and always if incorporateAFR is enabled
  {
    currentStatus.afrTarget = get3DTableValue(&afrTable, currentStatus.fuelLoad, currentStatus.RPM); 
  }
  else { currentStatus.afrTarget = configPage2.stoich; }
  // END AFR Target Determination    
    
  if ((currentStatus.startRevolutions >> 1) >= ego_NextCycleCount) // Crank revolutions divided by 2 is engine cycles. This check always needs to happen, to correctly align revolutions and the time delay
    {
      ego_EngineCycleCheck = true;
      // Scale the revolution counts between the two values linearly based on load value used in VE table.
      ego_NextCycleCount = (currentStatus.startRevolutions >> 1) + (uint16_t)map(currentStatus.fuelLoad, 0, (int16_t)configPage6.egoFuelLoadMax, (int16_t)configPage6.egoCountL, (int16_t)configPage6.egoCountH); 
    }
  
  //General Enable Condtions for closed loop ego. egoType of 0 means no O2 sensor and no point having O2 closed loop and cannot use include AFR from sensor since this would be 2x proportional controls.
  if( (configPage6.egoType > 0) && (configPage6.egoAlgorithm <= EGO_ALGORITHM_DUALO2) && (configPage2.includeAFR == false) ) 
  {
    //Requirements to inhibit O2 adjustment (Freeze) check this rapidly so we don't miss freeze events.
    if ((abs(currentStatus.fuelLoad - ego_FuelLoadPrev) > (int16_t)configPage9.egoFuelLoadChngMax ) || //Change in fuel load (MAP or TPS) since last time algo ran to see if we need to freeze algo due to load change.
        (currentStatus.afrTarget < configPage6.egoAFRTargetMin) || // Target too rich - good for inhibiting O2 correction using AFR Target Table
        (currentStatus.fuelLoad > (int16_t)configPage6.egoFuelLoadMax) || // Too much load
        (currentStatus.launchCorrection != 100) || // Launch Control Active
        (BIT_CHECK(currentStatus.status1, BIT_STATUS1_DFCO) == 1)) //Fuel Cut
    { 
      BIT_SET(currentStatus.status4, BIT_STATUS4_EGO_FROZEN);
      ego_FreezeEndTime = runSecsX10 + configPage9.egoFreezeDelay; // Set ego freeze condition timer
    }
       
    // Ego corrections start after checking that both the transport delay based on engine cycles and load and the sensor time based delay are satisfied.
    if ((ego_EngineCycleCheck == true) && (runSecsX10 >= ego_DelaySensorTime))
    {
      ego_DelaySensorTime = runSecsX10 + configPage6.egoSensorDelay; // Save the minimum sensor delay time for next loop
      // Read the O2 sensors before the algo runs, may be faster than the main loop.
      readO2();
      readO2_2();
      O2_Readflag = true; // Used for informing the time based O2 sensor read function that we read the O2 value here.
      ego_FuelLoadPrev = currentStatus.fuelLoad; // save last value to check for load change

      //Requirements to run Closed Loop else its reset to 100pct. These are effectively errors where closed loop cannot run.
      if( (currentStatus.coolant > (int)(configPage6.egoTemp - CALIBRATION_TEMPERATURE_OFFSET)) && 
          (currentStatus.RPM >= (unsigned int)(configPage6.egoRPM * 100)) &&
          (currentStatus.runSecs > configPage6.egoStartdelay) &&
          (currentStatus.engineProtectStatus == 0) &&      // Engine protection , fuel or ignition cut is active.     
          ((configPage15.egoResetwAFR == false) ||
           (currentStatus.afrTarget >= configPage6.egoAFRTargetMin)) && // Ignore this criteria if cal set to freeze (false).
          ((configPage15.egoResetwfuelLoad == false) ||
           (currentStatus.fuelLoad <= (int16_t)configPage6.egoFuelLoadMax))) // Ignore this criteria if cal set to freeze (false).
      {
        
        if(runSecsX10 >= ego_FreezeEndTime) // Check the algo freeze conditions are not active.
        {
          BIT_CLEAR(currentStatus.status4, BIT_STATUS4_EGO_FROZEN);
          
          /* Build the lookup table for the integrator dynamically. This saves eeprom by using a fixed axis and less variables. 
           * The calibration values are expressed as a "% of max adjustment" where the max adjustment would theoretically correct the AFR to the target in one step." 
           * The scaling in the .ini file applies the correct adjustment in g_ego % units depending on the fixed axis defined by. egoIntAFR_XBins.
           * For example 3.0 afr error is 20% g_ego. So 100%/20% = 5 % per step. If egoIntAFR_XBins axis points are changed, the scaling in the .ini file also needs to be adjusted. 
          */
          egoIntAFR_Values[0] = configPage9.egoInt_Lean2; // Corresponds with -3.0 AFR Error
          egoIntAFR_Values[1] = configPage9.egoInt_Lean1; // Corresponds with -0.3 AFR Error
          egoIntAFR_Values[2] = OFFSET_AFR_ERR; //offset value is 0 adjustment
          egoIntAFR_Values[3] = configPage9.egoInt_Rich1; // Corresponds with 0.3 AFR Error
          egoIntAFR_Values[4] = configPage9.egoInt_Rich2; // Corresponds with 3.0 AFR Error
          
          // Sensor check to check in range.
          if ((currentStatus.O2 >= configPage6.egoMin) && // Not too rich
              ((currentStatus.O2 <= configPage6.egoMax) ||
              (BIT_CHECK(currentStatus.status1, BIT_STATUS1_DFCO) == 1))) // Not too lean but ignore egoMax (lean) if in DFCO.
          {              
            //Integral Control - Sensor 1
            O2_Error = currentStatus.afrTarget - currentStatus.O2 + OFFSET_AFR_ERR; //+127 is 0 error value. Richer than target is positive.
            
            /* Tracking of rich and lean (compared to target). Used for an integrator delay which is intended to allow proportional switching control
             * time to intentionally over-correct the fuel to generate a switch from rich to lean. If the proportional control alone is not enough to switch 
             * only then will the integral will start to move to adjust the mean value after a certain amount of time.
             * This is designed to generate the correct oscillation of rich and lean pulses required for 3 way catalyst control.
             * This logic only makes sense if the AFR target is at the stoich point for the chosen fuel.           
            */ 
            if ((configPage9.egoIntDelay > 0) && (currentStatus.afrTarget == configPage2.stoich))
            {
              O2_SensorIsRichPrev = O2_SensorIsRich;
              if (O2_Error > OFFSET_AFR_ERR) 
              { 
                 O2_SensorIsRich = true;  //Positive error = rich. 
                 ego_Prop = -(int8_t)(configPage9.egoProp_Swing); // Negative swing on prop.
              } 
              else 
              { 
                O2_SensorIsRich = false; 
                ego_Prop = (int8_t)(configPage9.egoProp_Swing);  //Positive swing on prop.
              } 
           
              if (O2_SensorIsRich == O2_SensorIsRichPrev) // Increment delay loops for the integrator if switch not detected 
              {
                if (ego_IntDelayLoops < configPage9.egoIntDelay) { ego_IntDelayLoops++; } // Limit to max value.
                BIT_CLEAR(currentStatus.status4, BIT_STATUS4_EGO1_INTCORR); // unset this to indicate the integrator isnt going to move.
              }              
              else { ego_IntDelayLoops = 0; } // Switch in fuelling has been detected, reset integrator delay counter. If the switch is not detected the integrator will keep updating every loop after this delay.
            }
            else 
            { 
              ego_IntDelayLoops = configPage9.egoIntDelay; 
              ego_Prop = 0;
            }
            
            //If integrator delay is passed then update integrator
            if (ego_IntDelayLoops >= configPage9.egoIntDelay) 
            { 
              BIT_SET(currentStatus.status4, BIT_STATUS4_EGO1_INTCORR);
              ego_Integral = ego_Integral + (int8_t)(table2D_getValue(&ego_IntegralTable, O2_Error) - OFFSET_AFR_ERR); 
            } //Integrate step value from table
            
            //Integrator Limits
            if (ego_Integral < -configPage6.egoLimit) { ego_Integral = -configPage6.egoLimit; BIT_CLEAR(currentStatus.status4, BIT_STATUS4_EGO1_INTCORR); }
            if (ego_Integral > configPage6.egoLimit) { ego_Integral = configPage6.egoLimit; BIT_CLEAR(currentStatus.status4, BIT_STATUS4_EGO1_INTCORR); }
            
            //2nd check to limit total value after update with prop and output the final correction
            if ((ego_Integral + ego_Prop) < -configPage6.egoLimit) { ego_AdjustPct = 100 - configPage6.egoLimit; }
            else if ((ego_Integral + ego_Prop) > configPage6.egoLimit) { ego_AdjustPct = 100 + configPage6.egoLimit; }
            else { ego_AdjustPct = 100 + ego_Integral + ego_Prop; }
          }
          else 
          { // O2 sensor out of range
            BIT_CLEAR(currentStatus.status4, BIT_STATUS4_EGO1_INTCORR);
            // Technically we are frozen here but don't have seperate bits to indicate ego2 frozen compared to ego1 so no indication given. 
            ego_IntDelayLoops = 0; 
            if (configPage6.egoAlgorithm == EGO_ALGORITHM_DUALO2) { ego_AdjustPct = currentStatus.ego2Correction; } // If ego1 out of range or not available and we are dual O2 assume the same adjustment as ego2.  
          } 

          // Sensor2 check to check in range and if Enabled
          if ((configPage6.egoAlgorithm == EGO_ALGORITHM_DUALO2) && // 2nd Sensor Logic
              (currentStatus.O2_2 >= configPage6.egoMin) && // Not too rich
              ((currentStatus.O2_2 <= configPage6.egoMax) ||
               (BIT_CHECK(currentStatus.status1, BIT_STATUS1_DFCO) == 1))) // Not too lean but ignore egoMax (lean) if in DFCO. 
          {
            //Proportional Integral Control - Sensor 2 Re-using some variables to save RAM.
            O2_Error = currentStatus.afrTarget - currentStatus.O2_2 + OFFSET_AFR_ERR; //+127 is 0 error value. Richer than target is positive.
            
            /* Tracking of rich and lean (compared to target). Used for an integrator delay which is intended to allow proportional switching control
             * time to intentionally over-correct the fuel to generate a switch from rich to lean. If the proportional control alone is not enough to switch 
             * only then will the integral will start to move to adjust the mean value after a certain amount of time.
             * This is designed to generate the correct oscillation of rich and lean pulses required for 3 way catalyst control.
             * This logic only makes sense if the AFR target is at the stoich point for the chosen fuel.           
            */ 
            if ((configPage9.egoIntDelay > 0) && (currentStatus.afrTarget == configPage2.stoich))
            {
              O2_2ndSensorIsRichPrev = O2_2ndSensorIsRich;
              if (O2_Error > OFFSET_AFR_ERR) 
              { 
                 O2_2ndSensorIsRich = true;  //Positive error = rich. 
                 ego2_Prop = -(int8_t)(configPage9.egoProp_Swing); // Negative swing on prop.
              } 
              else 
              { 
                O2_2ndSensorIsRich = false; 
                ego2_Prop = (int8_t)(configPage9.egoProp_Swing);  //Positive swing on prop.
              } 
           
              if (O2_2ndSensorIsRich == O2_2ndSensorIsRichPrev) // Increment delay loops for the integrator if switch not detected 
              {
                if (ego2_IntDelayLoops < configPage9.egoIntDelay) { ego2_IntDelayLoops++; } // Limit to max value.
                BIT_CLEAR(currentStatus.status4, BIT_STATUS4_EGO2_INTCORR); // unset this to indicate the integrator isnt going to move.
              }              
              else { ego2_IntDelayLoops = 0; } // Switch in fuelling has been detected, reset integrator delay counter. If the switch is not detected the integrator will keep updating every loop after this delay.
            }
            else 
            { 
              ego2_IntDelayLoops = configPage9.egoIntDelay; 
              ego2_Prop = 0;
            }
                        
            //If integrator delay is passed then update integrator
            if (ego2_IntDelayLoops >= configPage9.egoIntDelay) 
            { 
               BIT_SET(currentStatus.status4, BIT_STATUS4_EGO2_INTCORR);
               ego2_Integral = ego2_Integral + (int8_t)(table2D_getValue(&ego_IntegralTable, O2_Error) - OFFSET_AFR_ERR); 
            } //Integrate step value from table
            
            //Integrator Limits
            if (ego2_Integral < -configPage6.egoLimit) { ego2_Integral = -configPage6.egoLimit; BIT_CLEAR(currentStatus.status4, BIT_STATUS4_EGO2_INTCORR); }
            if (ego2_Integral > configPage6.egoLimit) { ego2_Integral = configPage6.egoLimit; BIT_CLEAR(currentStatus.status4, BIT_STATUS4_EGO2_INTCORR); }
            
            //2nd check to limit total value after update with prop and output the final correction
            if ((ego2_Integral + ego2_Prop) < -configPage6.egoLimit) { ego2_AdjustPct = 100 - configPage6.egoLimit; }
            else if ((ego2_Integral + ego2_Prop) > configPage6.egoLimit) { ego2_AdjustPct = 100 + configPage6.egoLimit; }
            else { ego2_AdjustPct = 100 + ego2_Integral + ego2_Prop; }
          }
          else 
          { // No 2nd O2 or O2 sensor out of range
            BIT_CLEAR(currentStatus.status4, BIT_STATUS4_EGO2_INTCORR);
            // Technically we are frozen here but don't have seperate bits to indicate ego2 frozen compared to ego1 so no indication given. 
            ego2_IntDelayLoops = 0;
            ego2_AdjustPct = currentStatus.egoCorrection; // If ego2 out of range or not available, assume the same adjustment as ego1.           
          } 
        } // End Conditions to not freeze ego correction
        else { ego_AdjustPct = currentStatus.egoCorrection; ego2_AdjustPct = currentStatus.ego2Correction; BIT_CLEAR(currentStatus.status4, BIT_STATUS4_EGO1_INTCORR); BIT_CLEAR(currentStatus.status4, BIT_STATUS4_EGO2_INTCORR);} // ego frozen at last values
  	  } // End Conditions not to reset ego
  	  else 
      { //Reset closed loop. Also activate freeze delay to for when we re-enable.
        BIT_SET(currentStatus.status4, BIT_STATUS4_EGO_FROZEN);
        BIT_CLEAR(currentStatus.status4, BIT_STATUS4_EGO1_INTCORR);
        BIT_CLEAR(currentStatus.status4, BIT_STATUS4_EGO2_INTCORR);
        ego_AdjustPct = 100;
        ego2_AdjustPct = 100;      
        ego_Integral = 0;
        ego2_Integral = 0;        
        ego_IntDelayLoops = 0;
        ego2_IntDelayLoops = 0;
        ego_FreezeEndTime = runSecsX10 + configPage9.egoFreezeDelay;
      }
    } //End O2 Algorithm Run Loop check
    else
    {
      if (currentStatus.RPM >= (unsigned int)(configPage6.egoRPM * 100)) 
      { // hold last value
        ego_AdjustPct = currentStatus.egoCorrection; 
        ego2_AdjustPct = currentStatus.ego2Correction;
      } 
      else 
      {// Engine speed probably stopped or recranking so don't apply EGO during crank.
        BIT_SET(currentStatus.status4, BIT_STATUS4_EGO_FROZEN);
        BIT_CLEAR(currentStatus.status4, BIT_STATUS4_EGO1_INTCORR);
        BIT_CLEAR(currentStatus.status4, BIT_STATUS4_EGO2_INTCORR);
        ego_AdjustPct = 100;
        ego2_AdjustPct = 100;  
        ego_NextCycleCount = 0;
        ego_DelaySensorTime = 0;
        ego_Integral = 0;
        ego2_Integral = 0;
        ego_IntDelayLoops = 0;
        ego2_IntDelayLoops = 0; 
        ego_FreezeEndTime = 0;
      } 
    }
  } //End egoType
  else
  { // No O2 sensors or incorrect config to run closed loop O2
    BIT_SET(currentStatus.status4, BIT_STATUS4_EGO_FROZEN);
    BIT_CLEAR(currentStatus.status4, BIT_STATUS4_EGO1_INTCORR);
    BIT_CLEAR(currentStatus.status4, BIT_STATUS4_EGO2_INTCORR);
    ego_AdjustPct = 100;
    ego2_AdjustPct = 100;  
    ego_NextCycleCount = 0;
    ego_DelaySensorTime = 0;
    ego_Integral = 0;
    ego2_Integral = 0;
    ego_IntDelayLoops = 0;
    ego2_IntDelayLoops = 0;
    ego_FreezeEndTime = 0;
  }
  
  // This algo only returns a single byte for the bank1 correction. A 2nd ego output is needed for Bank2 which is used later in individual bank adjustments.
  currentStatus.ego2Correction = ego2_AdjustPct;  

  return ego_AdjustPct;
}

//******************************** IGNITION ADVANCE CORRECTIONS ********************************
/** Dispatch calculations for all ignition related corrections.
 * @param base_advance - Base ignition advance (deg. ?)
 * @return Advance considering all (~12) individual corrections
 */
int8_t correctionsIgn(int8_t base_advance)
{
  int8_t advance;
  advance = correctionFlexTiming(base_advance);
  advance = correctionIATretard(advance);
  advance = correctionCLTadvance(advance);
  advance = correctionDFCOEntryExit(advance);
  advance = correctionIdleAdvance(advance);
  advance = correctionSoftRevLimit(advance);
  advance = correctionNitrous(advance);
  advance = correctionSoftLaunch(advance);
  advance = correctionSoftFlatShift(advance);
  advance = correctionKnock(advance);

  //Fixed timing check must go last
  advance = correctionFixedTiming(advance);
  advance = correctionCrankingFixedTiming(advance); //This overrides the regular fixed timing, must come last

  return advance;
}
/** Correct ignition timing to configured fixed value.
 * Must be called near end to override all other corrections.
 */
int8_t correctionFixedTiming(int8_t advance)
{
  int8_t ignFixValue = advance;
  if (configPage2.fixAngEnable == 1) { ignFixValue = configPage4.FixAng; } //Check whether the user has set a fixed timing angle
  return ignFixValue;
}
/** Correct ignition timing to configured fixed value to use during craning.
 * Must be called near end to override all other corrections.
 */
int8_t correctionCrankingFixedTiming(int8_t advance)
{
  int8_t ignCrankFixValue = advance;
  if ( BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) ) { ignCrankFixValue = configPage4.CrankAng; } //Use the fixed cranking ignition angle
  return ignCrankFixValue;
}

int8_t correctionFlexTiming(int8_t advance)
{
  int16_t ignFlexValue = advance;
  if( configPage2.flexEnabled == 1 ) //Check for flex being enabled
  {
    ignFlexValue = (int16_t) table2D_getValue(&flexAdvTable, currentStatus.ethanolPct) - OFFSET_IGNITION; //Negative values are achieved with offset
    currentStatus.flexIgnCorrection = (int8_t) ignFlexValue; //This gets cast to a signed 8 bit value to allows for negative advance (ie retard) values here. 
    ignFlexValue = (int8_t) advance + currentStatus.flexIgnCorrection;
  }
  return (int8_t) ignFlexValue;
}

/** Ignition correction for inlet air temperature (IAT).
 */
int8_t correctionIATretard(int8_t advance)
{
  int8_t advanceIATadjust = table2D_getValue(&IATRetardTable, currentStatus.IAT) - 15;

  return advance - advanceIATadjust;
}

/** Ignition correction for coolant temperature (CLT).
 */
int8_t correctionCLTadvance(int8_t advance)
{
  int8_t ignCLTValue = advance;
  //Adjust the advance based on CLT.
  int8_t advanceCLTadjust = (int16_t)(table2D_getValue(&CLTAdvanceTable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET)) - 15;
  ignCLTValue = (advance + advanceCLTadjust);
  
  return ignCLTValue;
}

/** Ignition adjustment for entry to and exit from DFCO.
 */
int8_t correctionDFCOEntryExit(int8_t advance)
{
  int8_t advanceDFCOadjust = configPage9.dfcoAdv;
  int8_t ignDFCOValue = advance;
  
  //Adjust the advance based on time into or out of DFCO. The DFCO advance variable is a modifier to the base advance.
  // DFCO_State is controled by the fuel algorithm.
  if (DFCO_State == DFCO_ACTIVE) { ignDFCOValue = advance + advanceDFCOadjust; }
  else if (DFCO_State == DFCO_RAMP_IN)
  {
    ignDFCOValue = map(runSecsX10, dfcoStateStrtTm, (dfcoStateStrtTm + configPage9.dfcoRampInTime), advance, (advance + advanceDFCOadjust)); // Ramp from advance to DFCO advance at rate of DFCO delay.
  }
  else if (DFCO_State == DFCO_RAMP_OUT)
  {
    ignDFCOValue = map(runSecsX10, dfcoStateStrtTm, (dfcoStateStrtTm + configPage9.dfcoRampOutTime), (advance + advanceDFCOadjust), advance); // Ramp from DFCO advance to regular advance at rate of DFCO delay.
  }
  //Other states of DFCO do not modify advance.
  return ignDFCOValue;
}

/** Ignition Idle advance correction.
 */
int8_t correctionIdleAdvance(int8_t advance)
{

  int8_t ignIdleValue = advance;
  //Adjust the advance based on idle target rpm.
  if( (configPage2.idleAdvEnabled >= 1) && (runSecsX10 >= (configPage2.idleAdvDelay * 5)) && idleAdvActive)
  {
    //currentStatus.CLIdleTarget = (byte)table2D_getValue(&idleTargetTable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET); //All temps are offset by 40 degrees
    int idleRPMdelta = (currentStatus.CLIdleTarget - (currentStatus.RPM / 10) ) + 50;
    // Limit idle rpm delta between -500rpm - 500rpm
    if(idleRPMdelta > 100) { idleRPMdelta = 100; }
    if(idleRPMdelta < 0) { idleRPMdelta = 0; }
    if( (currentStatus.RPM < (configPage2.idleAdvRPM * 100)) && ((configPage2.vssMode == 0) || (currentStatus.vss < configPage2.idleAdvVss))
    && (((configPage2.idleAdvAlgorithm == 0) && (currentStatus.TPS < configPage2.idleAdvTPS)) || ((configPage2.idleAdvAlgorithm == 1) && (currentStatus.CTPSActive == 1))) ) // closed throttle position sensor (CTPS) based idle state
    {
      if( idleAdvTaper < configPage9.idleAdvStartDelay )
      {
        if( BIT_CHECK(LOOP_TIMER, BIT_TIMER_10HZ) ) { idleAdvTaper++; }
      }
      else
      {
        int8_t advanceIdleAdjust = (int16_t)(table2D_getValue(&idleAdvanceTable, idleRPMdelta)) - 15;
        if(configPage2.idleAdvEnabled == 1) { ignIdleValue = (advance + advanceIdleAdjust); }
        else if(configPage2.idleAdvEnabled == 2) { ignIdleValue = advanceIdleAdjust; }
      }
    }
    else { idleAdvTaper = 0; }
  }

  if ( !idleAdvActive && BIT_CHECK(currentStatus.engine, BIT_ENGINE_RUN) && (currentStatus.RPM > (((uint16_t)currentStatus.CLIdleTarget * 10) - (uint16_t)IGN_IDLE_THRESHOLD)) ) { idleAdvActive = true; } //Active only after the engine is 200 RPM below target on first time
  else if (idleAdvActive && !BIT_CHECK(currentStatus.engine, BIT_ENGINE_RUN)) { idleAdvActive = false; } //Clear flag if engine isn't running anymore

  return ignIdleValue;
}
/** Ignition soft revlimit correction.
 */
int8_t correctionSoftRevLimit(int8_t advance)
{
  byte ignSoftRevValue = advance;
  BIT_CLEAR(currentStatus.spark, BIT_SPARK_SFTLIM);

  if (configPage6.engineProtectType == PROTECT_CUT_IGN || configPage6.engineProtectType == PROTECT_CUT_BOTH) 
  {
    if (currentStatus.RPMdiv100 >= configPage4.SoftRevLim) //Softcut RPM limit
    {
      BIT_SET(currentStatus.spark, BIT_SPARK_SFTLIM);
      if( softLimitTime < configPage4.SoftLimMax )
      {
        if (configPage2.SoftLimitMode == SOFT_LIMIT_RELATIVE) { ignSoftRevValue = ignSoftRevValue - configPage4.SoftLimRetard; } //delay timing by configured number of degrees in relative mode
        else if (configPage2.SoftLimitMode == SOFT_LIMIT_FIXED) { ignSoftRevValue = configPage4.SoftLimRetard; } //delay timing to configured number of degrees in fixed mode

        if( BIT_CHECK(LOOP_TIMER, BIT_TIMER_10HZ) ) { softLimitTime++; }
      }
    }
    else if( BIT_CHECK(LOOP_TIMER, BIT_TIMER_10HZ) ) { softLimitTime = 0; } //Only reset time at runSecsX10 update rate
  }

  return ignSoftRevValue;
}
/** Ignition Nitrous oxide correction.
 */
int8_t correctionNitrous(int8_t advance)
{
  byte ignNitrous = advance;
  //Check if nitrous is currently active
  if(configPage10.n2o_enable > 0)
  {
    //Check which stage is running (if any)
    if( (currentStatus.nitrous_status == NITROUS_STAGE1) || (currentStatus.nitrous_status == NITROUS_BOTH) )
    {
      ignNitrous -= configPage10.n2o_stage1_retard;
    }
    if( (currentStatus.nitrous_status == NITROUS_STAGE2) || (currentStatus.nitrous_status == NITROUS_BOTH) )
    {
      ignNitrous -= configPage10.n2o_stage2_retard;
    }
  }

  return ignNitrous;
}
/** Ignition soft launch correction.
 */
int8_t correctionSoftLaunch(int8_t advance)
{
  byte ignSoftLaunchValue = advance;
  //SoftCut rev limit for 2-step launch control.
  if (configPage6.launchEnabled && clutchTrigger && (currentStatus.clutchEngagedRPM < ((unsigned int)(configPage6.flatSArm) * 100)) && (currentStatus.RPM > ((unsigned int)(configPage6.lnchSoftLim) * 100)) && (currentStatus.TPS >= configPage10.lnchCtrlTPS) )
  {
    currentStatus.launchingSoft = true;
    BIT_SET(currentStatus.spark, BIT_SPARK_SLAUNCH);
    ignSoftLaunchValue = configPage6.lnchRetard;
  }
  else
  {
    currentStatus.launchingSoft = false;
    BIT_CLEAR(currentStatus.spark, BIT_SPARK_SLAUNCH);
  }

  return ignSoftLaunchValue;
}
/** Ignition correction for soft flat shift.
 */
int8_t correctionSoftFlatShift(int8_t advance)
{
  int8_t ignSoftFlatValue = advance;

  if(configPage6.flatSEnable && clutchTrigger && (currentStatus.clutchEngagedRPM > ((unsigned int)(configPage6.flatSArm) * 100)) && (currentStatus.RPM > (currentStatus.clutchEngagedRPM - (configPage6.flatSSoftWin * 100) ) ) )
  {
    BIT_SET(currentStatus.spark2, BIT_SPARK2_FLATSS);
    ignSoftFlatValue = configPage6.flatSRetard;
  }
  else { BIT_CLEAR(currentStatus.spark2, BIT_SPARK2_FLATSS); }

  return ignSoftFlatValue;
}
/** Ignition knock (retard) correction.
 */
int8_t correctionKnock(int8_t advance)
{
  byte knockRetard = 0;

  //First check is to do the window calculations (Assuming knock is enabled)
  if( configPage10.knock_mode != KNOCK_MODE_OFF )
  {
    knockWindowMin = table2D_getValue(&knockWindowStartTable, currentStatus.RPMdiv100);
    knockWindowMax = knockWindowMin + table2D_getValue(&knockWindowDurationTable, currentStatus.RPMdiv100);
  }


  if( (configPage10.knock_mode == KNOCK_MODE_DIGITAL)  )
  {
    //
    if(knockCounter > configPage10.knock_count)
    {
      if(currentStatus.knockActive == true)
      {
        //Knock retard is currently 
      }
      else
      {
        //Knock needs to be activated
        lastKnockCount = knockCounter;
        knockStartTime = micros();
        knockRetard = configPage10.knock_firstStep;
      }
    }

  }

  return advance - knockRetard;
}

/** Ignition Dwell Correction.
 */
uint16_t correctionsDwell(uint16_t dwell)
{
  uint16_t tempDwell = dwell;
  uint16_t sparkDur_uS = (configPage4.sparkDur * 100); //Spark duration is in mS*10. Multiple it by 100 to get spark duration in uS
  //Pull battery voltage based dwell correction and apply if needed
  currentStatus.dwellCorrection = table2D_getValue(&dwellVCorrectionTable, currentStatus.battery10);
  if (currentStatus.dwellCorrection != 100) { tempDwell = div100(dwell) * currentStatus.dwellCorrection; }

  //Dwell limiter
  uint16_t dwellPerRevolution = tempDwell + sparkDur_uS;
  int8_t pulsesPerRevolution = 1;
  //Single channel spark mode is the only time there will be more than 1 pulse per revolution on any given output
  //For rotary ignition this also holds true in wasted spark configuration (FC/FD) resulting in 2 pulses. RX-8 however is fully sequential resulting in 1 pulse
  if( ( (configPage4.sparkMode == IGN_MODE_SINGLE) || ((configPage4.sparkMode == IGN_MODE_ROTARY) && (configPage10.rotaryType != ROTARY_IGN_RX8)) ) && (configPage2.nCylinders > 1) ) //No point in running this for 1 cylinder engines
  {
    pulsesPerRevolution = (configPage2.nCylinders >> 1);
    dwellPerRevolution = dwellPerRevolution * pulsesPerRevolution;
  }

  if(dwellPerRevolution > revolutionTime)
  {
    //Possibly need some method of reducing spark duration here as well, but this is a start
    uint16_t adjustedSparkDur = (sparkDur_uS * revolutionTime) / dwellPerRevolution;
    tempDwell = (revolutionTime / pulsesPerRevolution) - adjustedSparkDur;
  }
  return tempDwell;
}

/*********************************************************************************************/
/* Below this line corrections are for individual injectors or injector banks. */

/** Corrections are for individual injectors or injector banks. 
* all these functions modify the injecton pulswidths PW1 to PW8. None of them can modify baseFuel since that is a global correction.
*/
void correctionsFuel_Individual(void)
{  
  //Initially all the pulse widths are set the same. Adjustments are made in the functions below
  currentStatus.PW1 = currentStatus.BaseFuel;
  currentStatus.PW2 = currentStatus.BaseFuel;
  currentStatus.PW3 = currentStatus.BaseFuel;
  currentStatus.PW4 = currentStatus.BaseFuel;
  currentStatus.PW5 = currentStatus.BaseFuel;
  currentStatus.PW6 = currentStatus.BaseFuel;
  currentStatus.PW7 = currentStatus.BaseFuel;
  currentStatus.PW8 = currentStatus.BaseFuel;
  
  // Globally used variable calculatons
  //Check that the duty cycle of the chosen pulsewidth isn't too high.
  pwLimit = percentage(configPage2.dutyLim, revolutionTime); //The pulsewidth limit is determined to be the duty cycle limit (Eg 85%) by the total time it takes to perform 1 crank revolution
  //Handle multiple squirts per rev
  if (configPage2.strokes == FOUR_STROKE) { pwLimit = pwLimit * 2 / currentStatus.nSquirts; } 
  else { pwLimit = pwLimit / currentStatus.nSquirts; }
  
  // Multiplications to fuel
  correctionEGOBank2();
  correctionFuelTrim();
  correctionFuelStaging(); // Fuel staging after fuel trim to incorporate trim into stage amount and limits.
  
  // Additions and Subtractions to fuel
  correctionFuelInjOpen();
  
  // Limits
  correctionFuelPWLimit();
}

/** Staging Correction
* This sets the opposing fuel injector as the "staging" injector. To turn on with the primary injector to inject into the same cylinder (or throttle body).
* example: PW1 is the primary. PW3 is the secondary for cylinder 1.
*          PW2 is the primary. PW4 is the secondary for cylinder 2.
*/
void correctionFuelStaging(void)
{
  //Calculate staging pulsewidths if used
  //To run staged injection, the number of cylinders must be less than or equal to the injector channels (ie Assuming you're running paired injection, you need at least as many injector channels as you have cylinders, half for the primaries and half for the secondaries)
  if( (configPage10.stagingEnabled == true) && 
      ((configPage2.nCylinders <= INJ_CHANNELS) || 
       (configPage2.injType == INJ_TYPE_TBODY)) && 
      (currentStatus.BaseFuel > 0) ) //Final check is to ensure that DFCO isn't active, which would cause an overflow below (See #267)
  {
    unsigned long pwLimit_minusInjOpen = pwLimit - inj_opentime_uS; // Staging needs to calculate the limit minus open time since this is added later.
    //Scale the 'full' pulsewidth by each of the injector capacities
    uint32_t tempPW1 = (((unsigned long)currentStatus.PW1 * staged_req_fuel_mult_pri) / 100);

    if(configPage10.stagingMode == STAGING_MODE_TABLE)
    {
      uint32_t tempPW3 = (((unsigned long)currentStatus.PW1 * staged_req_fuel_mult_sec) / 100); //This is ONLY needed in in table mode. Auto mode only calculates the difference.

      byte stagingSplit = get3DTableValue(&stagingTable, currentStatus.MAP, currentStatus.RPM);
      currentStatus.PW1 = ((100 - stagingSplit) * tempPW1) / 100;
      if (currentStatus.PW1 > pwLimit_minusInjOpen) { currentStatus.PW1 = pwLimit_minusInjOpen; } // Check for injection limit

      if(stagingSplit > 0) 
      { 
        currentStatus.PW3 = (stagingSplit * tempPW3) / 100;
        if (currentStatus.PW3 > pwLimit_minusInjOpen) { currentStatus.PW3 = pwLimit_minusInjOpen; } // Also limit staging injector.        
      }
      else { currentStatus.PW3 = 0; }
    }
    else if(configPage10.stagingMode == STAGING_MODE_AUTO)
    {
      currentStatus.PW1 = tempPW1;
      //If automatic mode, the primary injectors are used all the way up to their limit (Configured by the pulsewidth limit setting)
      //If they exceed their limit, the extra duty is passed to the secondaries
      if(tempPW1 > pwLimit_minusInjOpen)
      {
        uint32_t extraPW = tempPW1 - pwLimit_minusInjOpen;
        currentStatus.PW1 = pwLimit_minusInjOpen;
        currentStatus.PW3 = ((extraPW * staged_req_fuel_mult_sec) / staged_req_fuel_mult_pri); //Convert the 'left over' fuel amount from primary injector scaling to secondary
        if (currentStatus.PW3 > pwLimit_minusInjOpen) { currentStatus.PW3 = pwLimit_minusInjOpen; }// Also limit staging injector.
      }
      else { currentStatus.PW3 = 0; } //If tempPW1 < pwLimit it means that the entire fuel load can be handled by the primaries. Simply set the secondaries to 0
    }

  //Set the 2nd channel of each stage with the same pulseWidth
  currentStatus.PW2 = currentStatus.PW1;
  currentStatus.PW4 = currentStatus.PW3;
  }
}

/** Exhaust Gas Oxygen (EGO) Correction for Bank 2
* The base fuel already includes the global correction for EGO. So EGO2 correction here is the difference between G_ego and G_ego2
* Injectors assigned on bank 1 must only align with EGO, Bank 2 must only align with EGO2. 
* Care must be taken if staging or paired injection is enabled that the ego corrections are as intended.
*/
void correctionEGOBank2(void)
{
  if( (configPage6.egoAlgorithm == EGO_ALGORITHM_DUALO2) && (configPage2.injType == INJ_TYPE_PORT) )
  {
    // need to apply the difference between the already applied global G_ego (bank1) and G_ego2 for bank 2 since, all pw already scaled with egoCorrection for bank 1.
    unsigned long pwBank2percentDiff = (100 + currentStatus.ego2Correction) - currentStatus.egoCorrection;
    
    if (pwBank2percentDiff != 100)
    {
      if( (configPage9.injBank_Inj1 == INJ_BANK2) && (channel1InjEnabled == true) ) { currentStatus.PW1 = (pwBank2percentDiff * currentStatus.PW1) / 100; }
      if( (configPage9.injBank_Inj2 == INJ_BANK2) && (channel2InjEnabled == true) ) { currentStatus.PW2 = (pwBank2percentDiff * currentStatus.PW2) / 100; }
      if( (configPage9.injBank_Inj3 == INJ_BANK2) && (channel3InjEnabled == true) ) { currentStatus.PW3 = (pwBank2percentDiff * currentStatus.PW3) / 100; }
      if( (configPage9.injBank_Inj4 == INJ_BANK2) && (channel4InjEnabled == true) ) { currentStatus.PW4 = (pwBank2percentDiff * currentStatus.PW4) / 100; }
      #if INJ_CHANNELS >= 5
      if( (configPage9.injBank_Inj5 == INJ_BANK2) && (channel5InjEnabled == true) ) { currentStatus.PW5 = (pwBank2percentDiff * currentStatus.PW5) / 100; }
      #endif
      #if INJ_CHANNELS >= 6
      if( (configPage9.injBank_Inj6 == INJ_BANK2) && (channel6InjEnabled == true) ) { currentStatus.PW6 = (pwBank2percentDiff * currentStatus.PW6) / 100; }
      #endif
      #if INJ_CHANNELS >= 7
      if( (configPage9.injBank_Inj7 == INJ_BANK2) && (channel7InjEnabled == true) ) { currentStatus.PW7 = (pwBank2percentDiff * currentStatus.PW7) / 100; }
      #endif
      #if INJ_CHANNELS >= 8
      if( (configPage9.injBank_Inj8 == INJ_BANK2) && (channel8InjEnabled == true) ) { currentStatus.PW8 = (pwBank2percentDiff * currentStatus.PW8) / 100; }
      #endif
    }
  }
}

/** Fuel Trim Correction
* Scales the fuel injection ammount via a table indexed by fuel load and rpm for each injector. 
* Care must be taken if staging or paired injection is enabled that the corrections are as intended.
*/
void correctionFuelTrim(void)
{
  if(configPage6.fuelTrimEnabled == true)
  {
    if (channel1InjEnabled == true) { currentStatus.PW1 = applyFuelTrimToPW(&trim1Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW1); }
    
    if (channel2InjEnabled == true) { currentStatus.PW2 = applyFuelTrimToPW(&trim2Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW2); }

    if (channel3InjEnabled == true) { currentStatus.PW3 = applyFuelTrimToPW(&trim3Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW3); }

    if (channel4InjEnabled == true) { currentStatus.PW4 = applyFuelTrimToPW(&trim4Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW4); }

    #if INJ_CHANNELS >= 5
    if (channel5InjEnabled == true) { currentStatus.PW5 = applyFuelTrimToPW(&trim5Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW5); }
    #endif
    
    #if INJ_CHANNELS >= 6
    if (channel6InjEnabled == true) { currentStatus.PW6 = applyFuelTrimToPW(&trim6Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW6); }
    #endif
    
    #if INJ_CHANNELS >= 7
    if (channel7InjEnabled == true) { currentStatus.PW7 = applyFuelTrimToPW(&trim7Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW7); }
    #endif
    
    #if INJ_CHANNELS >= 8
    if (channel8InjEnabled == true) { currentStatus.PW8 = applyFuelTrimToPW(&trim8Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW8); }
    #endif
  }
}

/** Injector open time correction.
* This needs to be done at the last step because the injector pulsewidth up to this point represents fuel as a linear scale for multiplication. After this addition no further multiplication can occur.
*/
void correctionFuelInjOpen(void)
{ 
  //Check each cylinder for injector cutoff and then if running apply the injector open time compensation.  
  if (currentStatus.PW1 > 0) { currentStatus.PW1 += inj_opentime_uS; }
  if (currentStatus.PW2 > 0) { currentStatus.PW2 += inj_opentime_uS; }
  if (currentStatus.PW3 > 0) { currentStatus.PW3 += inj_opentime_uS; }
  if (currentStatus.PW4 > 0) { currentStatus.PW4 += inj_opentime_uS; }
  #if INJ_CHANNELS >= 5
  if (currentStatus.PW5 > 0) { currentStatus.PW5 += inj_opentime_uS; }
  #endif
  #if INJ_CHANNELS >= 6
  if (currentStatus.PW6 > 0) { currentStatus.PW6 += inj_opentime_uS; }
  #endif
  #if INJ_CHANNELS >= 7
  if (currentStatus.PW7 > 0) { currentStatus.PW7 += inj_opentime_uS; }
  #endif
  #if INJ_CHANNELS >= 8  
  if (currentStatus.PW8 > 0) { currentStatus.PW8 += inj_opentime_uS; } 
  #endif
}

void correctionFuelPWLimit(void)
{
  //Apply the pwLimit if staging is dsiabled and engine is not cranking. Staging takes care of it's own PW limit and Cranking is a special case.
  if( (!BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK)) && (configPage10.stagingEnabled == false) ) 
  {
    if (currentStatus.PW1 > pwLimit) { currentStatus.PW1 = pwLimit; }
    if (currentStatus.PW2 > pwLimit) { currentStatus.PW2 = pwLimit; }
    if (currentStatus.PW3 > pwLimit) { currentStatus.PW3 = pwLimit; }
    if (currentStatus.PW4 > pwLimit) { currentStatus.PW4 = pwLimit; }
    #if INJ_CHANNELS >= 5
    if (currentStatus.PW5 > pwLimit) { currentStatus.PW5 = pwLimit; }
    #endif
    #if INJ_CHANNELS >= 6
    if (currentStatus.PW6 > pwLimit) { currentStatus.PW6 = pwLimit; }
    #endif
    #if INJ_CHANNELS >= 7
    if (currentStatus.PW7 > pwLimit) { currentStatus.PW7 = pwLimit; }  
    #endif
    #if INJ_CHANNELS >= 8
    if (currentStatus.PW8 > pwLimit) { currentStatus.PW8 = pwLimit; }
    #endif    
  }
}
  
