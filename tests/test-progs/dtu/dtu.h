#ifndef DTU_H
#define DTU_H

#include <inttypes.h>

// TODO caution: receive buffer and messages are currently NOT allowed to cross page-boundaries
// this is because of the workaround for SE, that translates it from virtual to physical, but
// assumes that it is physically contiguous.
// for now, please use e.g. memalign to allocate them and make them at most 4096 bytes large!

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
    reg_t label;
    reg_t replyEpId;
    reg_t replyLabel;

    reg_t requestLocalAddr;
    reg_t requestRemoteAddr;
    reg_t requestSize;

    reg_t credits;
};

struct MessageHeader
{
    uint8_t flags; // if bit 0 is set its a reply, if bit 1 is set we grand credits
    uint8_t senderCoreId;
    uint8_t senderEpId;
    uint8_t replyEpId; // for a normal message this is the reply epId
                       // for a reply this is the enpoint that receives credits
    uint16_t length;

    // both should be large enough for pointers.
    uint64_t label;
    uint64_t replyLabel;
} __attribute__((packed));

enum Command : uint8_t
{
    IDLE = 0,
    START_OPERATION = 1,
    INC_READ_PTR = 2,
};

enum {
    COMMAND_OPCODE_BITS    = 2,
};

volatile reg_t* dtuCommandPtr   = (reg_t*) dtuBaseAddr;
volatile reg_t* dtuStatusPtr    = (reg_t*) (dtuBaseAddr + sizeof(reg_t));
volatile Endpoint* dtuEndpoints = (Endpoint*) (dtuBaseAddr + sizeof(reg_t) * 2);

#endif
