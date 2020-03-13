#include <malloc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "../tcu.h"
#include "m5op.h"

enum {
    MAX_MSG_SIZE    = 64,
    MSG_SIZE        = 32,
    BUF_SLOTS       = 4,
    BUF_SIZE        = MAX_MSG_SIZE * BUF_SLOTS,
};

int main()
{
    printf("Master: Hello World!\n");

    // wait a bit to allow for PE2 to setup it's endpoint
    for (volatile int i;i < 0x100; i++);

    printf("Master: Setup Endpoint 1 to send messages to PE2 endpoint 3\n");

    tcuEndpoints[1].mode = 1; // send messages
    tcuEndpoints[1].maxMessageSize = MAX_MSG_SIZE;
    tcuEndpoints[1].targetCoreId = 2;
    tcuEndpoints[1].targetEpId = 3;
    tcuEndpoints[1].messageSize = MSG_SIZE;
    tcuEndpoints[1].replyEpId = 5;
    tcuEndpoints[1].credits = MAX_MSG_SIZE * 2;
    tcuEndpoints[1].label = 0x1234;
    tcuEndpoints[1].replyLabel = 0x4567;

    printf("Master: Setup Endpoint 5 to receive messages\n");

    tcuEndpoints[5].mode = 0; // receive messages
    tcuEndpoints[5].maxMessageSize = MAX_MSG_SIZE;
    tcuEndpoints[5].bufferSize = BUF_SLOTS;

    char* data = (char*) memalign(4096, BUF_SIZE);

    uint64_t addr = reinterpret_cast<uint64_t>(data);

    tcuEndpoints[5].bufferAddr = addr;
    tcuEndpoints[5].bufferReadPtr = addr;
    tcuEndpoints[5].bufferWritePtr = addr;

    printf("Master: send Message\n");

    char* message = (char*) memalign(4096, MSG_SIZE);

    tcuEndpoints[1].messageAddr = reinterpret_cast<uint64_t>(data);

    for (int i = 0; i < MSG_SIZE; i++)
        data[i] = i + 1;

    *tcuCommandPtr = Command::START_OPERATION | (1 << COMMAND_OPCODE_BITS);

    // wait until operation finished
    while (*tcuStatusPtr);

    while (tcuEndpoints[5].bufferMessageCount == 0)
        ;

    MessageHeader *header = reinterpret_cast<MessageHeader*>(
      tcuEndpoints[5].bufferReadPtr);
    unsigned char *payload = reinterpret_cast<unsigned char*>(
      tcuEndpoints[5].bufferReadPtr + sizeof(*header));

    printf("Master: Received a reply! Its located at %#x\n", header);
    printf("Master: flags      = %#llx\n", header->flags);
    printf("Master: length     = %#llx\n", header->length);
    printf("Master: label      = %#016llx\n", header->label);

    for (int i = 0; i < (header->length + 7) / 8; i++)
        printf("Master: Message: %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x\n",
               payload[i*8+0],
               payload[i*8+1],
               payload[i*8+2],
               payload[i*8+3],
               payload[i*8+4],
               payload[i*8+5],
               payload[i*8+6],
               payload[i*8+7]);

    free(message);
    free(data);

    while (true) ;

    return 0;
}
