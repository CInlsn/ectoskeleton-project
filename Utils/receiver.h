#ifndef RECEIVER_H
#define RECEIVER_H

#include "statemch.h"

__weak void ReceiveCpltCallback(uint8_t *data, uint32_t len);
void ReceiverInit(void);
void Encode(uint8_t *store, uint8_t *target, uint32_t len);

#endif