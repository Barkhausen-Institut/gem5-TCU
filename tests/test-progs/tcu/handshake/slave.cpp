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
    printf("Slave: Hello World!\n");

    printf("Slave: Setup Endpoint 3 to receive messages\n");

    tcuEndpoints[3].mode = 0; // receive messages
    tcuEndpoints[3].maxMessageSize = MAX_MSG_SIZE;
    tcuEndpoints[3].bufferSize = BUF_SLOTS;

    char* data = (char*) memalign(4096, BUF_SIZE);

    uint64_t addr = reinterpret_cast<uint64_t>(data);

    tcuEndpoints[3].bufferAddr = addr;
    tcuEndpoints[3].bufferReadPtr = addr;
    tcuEndpoints[3].bufferWritePtr = addr;

    while (tcuEndpoints[3].bufferMessageCount == 0)
        ;

    MessageHeader *header = reinterpret_cast<MessageHeader*>(
      tcuEndpoints[3].bufferReadPtr);
    unsigned char *payload = reinterpret_cast<unsigned char*>(
      tcuEndpoints[3].bufferReadPtr + sizeof(*header));

    printf("Slave: Received a message! Its located at %#x\n", header);
    printf("Slave: flags      = %#llx\n", header->flags);
    printf("Slave: length     = %#llx\n", header->length);
    printf("Slave: label      = %#016llx\n", header->label);
    printf("Slave: replyLabel = %#016llx\n", header->replyLabel);

    for (int i = 0; i < (header->length + 7) / 8; i++)
        printf("Slave: Message: %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x\n",
               payload[i*8+0],
               payload[i*8+1],
               payload[i*8+2],
               payload[i*8+3],
               payload[i*8+4],
               payload[i*8+5],
               payload[i*8+6],
               payload[i*8+7]);

    printf("Slave: send reply\n");

    char* reply = (char*) memalign(4096, MSG_SIZE);

    for (int i = 0; i < MSG_SIZE; i++)
        reply[i] = (i + 1) << 4;

    tcuEndpoints[3].messageAddr = reinterpret_cast<uint64_t>(reply);
    tcuEndpoints[3].messageSize = MSG_SIZE;

    *tcuCommandPtr = Command::START_OPERATION | (3 << COMMAND_OPCODE_BITS);

    // wait until operation finished
    while (*tcuStatusPtr);

    printf("Slave: done\n");

    free(data);
    free(reply);

    while (true) ;

    return 0;
}
