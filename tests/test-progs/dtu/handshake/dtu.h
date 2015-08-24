#ifndef DTU_H
#define DTU_H

#include <inttypes.h>

constexpr uint64_t dtuBaseAddr = 0x1000000;
constexpr unsigned numPes = 8;
typedef uint64_t reg_t;

struct Endpoint
{
    reg_t mode;
    reg_t maxMessageSize;
    reg_t bufferMessageCount;
    reg_t bufferAddr;
    reg_t bufferSize;
    reg_t bufferReadPtr;
    reg_t bufferWritePtr;

    reg_t targetCoreId;
    reg_t targetEpId;
    reg_t messageAddr;
    reg_t messageSize;
    reg_t replyEpId;

    reg_t requestLocalAddr;
    reg_t requestRemoteAddr;
    reg_t requestSize;

    reg_t credits;
};

volatile reg_t* dtuCommandPtr   = (reg_t*) dtuBaseAddr;
volatile reg_t* dtuStatusPtr    = (reg_t*) (dtuBaseAddr + sizeof(reg_t));
volatile Endpoint* dtuEndpoints = (Endpoint*) (dtuBaseAddr + sizeof(reg_t) * 2);

#endif
