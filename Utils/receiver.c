#include "receiver.h"
#include "statemch.h"

#include <string.h>
#include <stdint.h>

#define BEGIN_SIGN (uint8_t)'a'

extern StateNode TerminateNode;

__weak void ReceiveCpltCallback(uint8_t *data, uint32_t len) {};

typedef enum bdComm_RxState {
	bdComm_TERMINATE,
	bdComm_READLENSIGN,
	bdComm_READLEN,
	bdComm_READDATASIGN,
	bdComm_READDATA,
	bdComm_END,
} ReadState;

StateNode ReadlensignNode, ReadlenNode, ReaddatasignNode, ReaddataNode, EndNode;
StateMachine bdComm_stateMachine = {
	.stateNum = 6,
	.nowState = bdComm_READLENSIGN,
	.stateList = {
		[0] = &TerminateNode,
		[1] = &ReadlensignNode,
		[2] = &ReadlenNode,
		[3] = &ReaddatasignNode,
		[4] = &ReaddataNode,
		[5] = &EndNode
	}
};

int ReadlensignTransfer(uint8_t input, PublicData *pd) {
	if(input == BEGIN_SIGN) {
		return bdComm_READLEN;
	}
	return bdComm_READLENSIGN;
}

int ReadlenTransfer(uint8_t input, PublicData *pd) {
	if(pd->counter != 0) {
		return bdComm_READLEN;
	}	
	return bdComm_READDATASIGN;
}
	
int ReaddatasignTransfer(uint8_t input, PublicData *pd) {
	if(input == BEGIN_SIGN + pd->DataNowIndex) {
		if(pd->DataNowIndex == pd->DataLen + 1) {
			return bdComm_END;
		}
		return bdComm_READDATA;
	}
	return bdComm_READLENSIGN;
}
	
int ReaddataTransfer(uint8_t input, PublicData *pd) {
	if(pd->counter != 0) {
		return bdComm_READDATA;
	}
	return bdComm_READDATASIGN;
}

int EndTransfer(uint8_t input, PublicData *pd) {
	if(input == BEGIN_SIGN + pd->DataLen + 1) {
		if(pd->counter == 0) {
			return bdComm_READLENSIGN;
		}
		return bdComm_END;
	}
	return bdComm_READLENSIGN;
}

void reset(PublicData *pd) {
	memset(pd->RxData, 0, MAX_DATA_LEN);
}

void ReadlensignToReadlen(uint8_t input, PublicData *pd) {
	pd->counter = 3;
}
void ReadlenToReadlen(uint8_t input, PublicData *pd) {
	pd->rxDataBuffer[3 - pd->counter] = input;
	pd->counter--;
}
void ReadlenToReaddatasign(uint8_t input, PublicData *pd) {
	pd->rxDataBuffer[3 - pd->counter] = input;
    pd->DataLen =  pd->rxDataBuffer[0];
    pd->DataLen |= pd->rxDataBuffer[1] << 8;
    pd->DataLen |= pd->rxDataBuffer[2] << 16;
    pd->DataLen |= pd->rxDataBuffer[3] << 24;
	pd->DataNowIndex = 1;
}
void ReaddatasignToReaddata(uint8_t input, PublicData *pd) {
	pd->counter = 3;
}
void ReaddatasignToReadlensign(uint8_t input, PublicData *pd) {
	reset(pd);
}
void ReaddatasignToEnd(uint8_t input, PublicData *pd) {
	pd->counter = 3;
}
void ReaddataToReaddata(uint8_t input, PublicData *pd) {
	pd->rxDataBuffer[3 - pd->counter] = input;
	pd->counter--;
}
void ReaddataToReaddatasign(uint8_t input, PublicData *pd) {
	pd->rxDataBuffer[3 - pd->counter] = input;
	memcpy(&pd->RxData[(pd->DataNowIndex - 1) * 4], pd->rxDataBuffer, 4);
	pd->DataNowIndex++;
}
void EndToEnd(uint8_t input, PublicData *pd) {
	pd->counter--;
}
void EndToReadlensign(uint8_t input, PublicData *pd) {
	if (pd->counter != 0) {
		reset(pd);
		return;
	}
	
	ReceiveCpltCallback(pd->RxData, pd->DataLen * 4);
	reset(pd);
}

void ReceiverInit(void) {
	ReadlensignNode.name  = 1;
	ReadlenNode.name      = 2;
	ReaddatasignNode.name = 3;
	ReaddataNode.name     = 4;
	EndNode.name          = 5;
	
	ReadlensignNode.transFunc  = ReadlensignTransfer;
	ReadlenNode.transFunc      = ReadlenTransfer;
	ReaddatasignNode.transFunc = ReaddatasignTransfer;
	ReaddataNode.transFunc     = ReaddataTransfer;
	EndNode.transFunc          = EndTransfer;
	
	StateMachineInit(&bdComm_stateMachine);
	
	ReadlensignNode.handlerMap[2]  = ReadlensignToReadlen;
	ReadlenNode.handlerMap[2]      = ReadlenToReadlen;
	ReadlenNode.handlerMap[3]      = ReadlenToReaddatasign;
	ReaddatasignNode.handlerMap[1] = ReaddatasignToReadlensign;
	ReaddatasignNode.handlerMap[4] = ReaddatasignToReaddata;
	ReaddatasignNode.handlerMap[5] = ReaddatasignToEnd;
	ReaddataNode.handlerMap[3]     = ReaddataToReaddatasign;
	ReaddataNode.handlerMap[4]     = ReaddataToReaddata;
	EndNode.handlerMap[1]          = EndToReadlensign;
	EndNode.handlerMap[5]          = EndToEnd;
}

void Uint32ToUint8Array4(uint32_t f, uint8_t *arr) {
		uint32_t temp;
    temp = *(uint32_t *)&f;
    arr[3] = (temp >> 24) & 0xFF;
    arr[2] = (temp >> 16) & 0xFF;
    arr[1] = (temp >> 8) & 0xFF;
    arr[0] = temp & 0xFF;
}

void Encode(uint8_t *store, uint8_t *target, uint32_t len) {
	uint32_t size  = (int)(len / 4);
	uint32_t leave = len - size * 4;
	
	*store = BEGIN_SIGN;
	Uint32ToUint8Array4(size, store + 1);
	for(int i = 1; i <= size; i++) {
		*(store + i * 5) = BEGIN_SIGN + i;
		memcpy(store + i * 5 + 1, target + (i - 1) * 4, 4);
	}
	memset(store + (size + 1) * 5, BEGIN_SIGN + size + 1, 5);
}
