/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */

#ifndef __CCROS_TRANSPORT_H__
#define __CCROS_TRANSPORT_H__

#include <stdbool.h>
#include <stdint.h>

void TransportInitUsb();
void TransportInitEthernet();
void TransportPoll();
void TransportSendLine(const char *line);
bool TransportReadLine(char *outLine, uint16_t maxLen);

#endif
