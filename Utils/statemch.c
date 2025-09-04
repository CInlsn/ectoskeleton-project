#include "statemch.h"

PublicData pubData;

void voidFunc(uint8_t input, PublicData *pd) {
	UNUSED(input);
	UNUSED(pd);
}

int TerminateTransfer(uint8_t input, PublicData *pd) { return 0; }

StateNode TerminateNode = {
	.name = 0,
	.transFunc = TerminateTransfer,
};

void StateMachineInit(StateMachine *machine) {
	for(int i = 0; i < machine->stateNum; i++) {
		for(int j = 0; j < machine->stateNum; j++) {
			machine->stateList[i]->handlerMap[j] = voidFunc;
		}
	}
}

void stepStateMch(StateMachine *machine, uint8_t input) {
	StateNode *pNowState;
	pNowState = machine->stateList[machine->nowState];
	int next = pNowState->transFunc(input, &pubData);
	pNowState->handlerMap[next](input, &pubData);
	machine->nowState = next;
}
