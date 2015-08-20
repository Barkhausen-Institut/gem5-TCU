#ifndef DTU_H
#define DTU_H

#include <inttypes.h>

constexpr uint64_t dtuBaseAddr = 0x1000000;
constexpr unsigned numPes = 8;

struct Endpoint
{
    uint32_t mode;
    uint32_t maxMessageSize;
    uint32_t bufferMessageCount;
    uint32_t bufferAddr;
    uint32_t bufferSize;
    uint32_t bufferReadPtr;
    uint32_t bufferWritePtr;

    uint32_t targetCoreId;
    uint32_t targetEpid;
    uint32_t messageAddr;
    uint32_t messageSize;
    uint32_t replyEpId;

    uint32_t requestLocalAddr;
    uint32_t requestRemoteAddr;
    uint32_t requestSize;

    uint32_t credits;
};

volatile uint32_t* dtuCommandPtr = (uint32_t*) dtuBaseAddr;
volatile uint32_t* dtuStatusPtr  = (uint32_t*) (dtuBaseAddr + 4);
volatile Endpoint* dtuEndpoints  = (Endpoint*) (dtuBaseAddr + 8);

#endif
