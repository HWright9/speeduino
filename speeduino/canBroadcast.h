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

uint8_t sendCAN_Speeduino_10Hz(void);

uint8_t recieveCAN_BroadCast(void);

void canTx_EngineSensor1(void);
void canTx_EnginePosition1(void);
void canTx_VehicleSpeed1(void);

void canRx_MotecPLM_O2(struct can_frame *canRxMsg, canid_t canRXId);
void canRx_MotecPLM_O22(struct can_frame *canRxMsg, canid_t canRXId);
void canRx_EPB_Vss(struct can_frame *canRxMsg, canid_t canRXId);

void canRx_MotecPLM_O2_Dflt(void);
void canRx_MotecPLM_O22_Dflt(void);
void canRx_EPB_Vss_Dflt(void);


uint8_t canO2TimeSinceLast;
uint8_t canO22TimeSinceLast;
uint8_t canEPBTimeSinceLast;
#endif

#endif // CANBROADCAST_H
