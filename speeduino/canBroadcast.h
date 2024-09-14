#ifndef CANBROADCAST_H
#define CANBROADCAST_H
#if defined(NATIVE_CAN_AVAILABLE)

//For BMW e46/e39/e38, rover and mini other CAN instrument clusters
#define CAN_BMW_ASC1 0x153 //Rx message from ACS unit that includes speed
#define CAN_BMW_DME1 0x316 //Tx message that includes RPM
#define CAN_BMW_DME2 0x329 //Tx message that includes CLT and TPS
#define CAN_BMW_DME4 0x545 //Tx message that includes CLT and TPS
#define CAN_BMW_ICL2 0x613
#define CAN_BMW_ICL3 0x615

//For VAG CAN instrument clusters
#define CAN_VAG_RPM 0x280
#define CAN_VAG_VSS 0x5A0

void sendBMWCluster();
void sendVAGCluster();
void DashMessages(uint16_t DashMessageID);
#endif

#if defined CAN_AVR_MCP2515

/*--------------- canbus config options -------------- */

/* CAN 0 */
#define CAN0_INT        2         // Set INT to pin 2
#define CAN0_CS         53        // Set CS to pin 53 on Mega
#define CAN_XTAL_20MHZ           0  // Different MCP2515 boards have different crystals.
#define CAN_XTAL_16MHZ           1 
#define CAN_XTAL_8MHZ            2

#define CAN_4K096BPS  0 
#define CAN_5KBPS     1
#define CAN_10KBPS    2
#define CAN_20KBPS    3
#define CAN_31K25BPS  4
#define CAN_33K3BPS   5
#define CAN_40KBPS    6
#define CAN_50KBPS    7
#define CAN_80KBPS    8
#define CAN_100KBPS   9
#define CAN_125KBPS   10
#define CAN_200KBPS   11
#define CAN_250KBPS   12
#define CAN_500KBPS   13
#define CAN_1000KBPS  14

/*-----------------------------------------------------*/


void can0_Init(void);
void can0_Maintainance(void);

uint8_t sendCAN_Speeduino_100Hz(void);
uint8_t recieveCAN_BroadCast(void);

uint8_t canTx_EngineSensor1(void);
uint8_t canTx_EnginePosition1(void);
uint8_t canTx_EngineActuator1(void);
uint8_t canTx_VehicleSpeed1(void);

void canRx_MotecPLM_O2(void);
void canRx_MotecPLM_O22(void);
void canRx_EPB_Vss(void);
void canRx_EPBAccelGyro1(void);

void canRx_MotecPLM_O2_Dflt(void);
void canRx_MotecPLM_O22_Dflt(void);
void canRx_EPB_Vss_Dflt(void);
void canRx_EPB_Vss_Dflt(void);


#endif

#endif // CANBROADCAST_H
