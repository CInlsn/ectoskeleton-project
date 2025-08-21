#ifndef STATEMCH_H
#define STATEMCH_H

#include "stm32h7xx_hal.h"

#define MAX_STATE_NUM 10
#define MAX_DATA_LEN 128

typedef struct PublicData {
  uint32_t DataLen;
	uint32_t DataNowIndex; // from 1
	uint8_t counter; // from 4 to 1
	uint8_t rxDataBuffer[4]; // from 0 to 4, from high bit to slow bit
	uint8_t RxData[MAX_DATA_LEN];
} PublicData;

typedef int (TransferFunc)(uint8_t input, PublicData *pd);
typedef void (HandlerFunc)(uint8_t input, PublicData *pd);

typedef struct StateNode {
	int name;
	TransferFunc *transFunc;
	HandlerFunc *handlerMap[MAX_STATE_NUM]; 
} StateNode;

void voidFunc(uint8_t input, PublicData *pd);
int TerminateTransfer(uint8_t input, PublicData *pd);

typedef struct StateMachine {
	int stateNum;  // include Terminate
	int nowState;
	StateNode *stateList[MAX_STATE_NUM];
} StateMachine;

void StateMachineInit(StateMachine *machine);
void stepStateMch(StateMachine *machine, uint8_t input);

#endif