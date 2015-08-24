#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <inttypes.h>

#include "dtu.h"
#include "m5op.h"

int main()
{
    printf("Slave: Hello World!\n");

    printf("Slave: Setup Endpoint 3 to receive messages\n");

    dtuEndpoints[3].mode = 0; // receive messages
    dtuEndpoints[3].maxMessageSize = 64;
    dtuEndpoints[3].bufferSize = 4;

    char* data = (char*) malloc(256);

    uint64_t addr = reinterpret_cast<uint64_t>(data);

    dtuEndpoints[3].bufferAddr = addr;
    dtuEndpoints[3].bufferReadPtr = addr;
    dtuEndpoints[3].bufferWritePtr = addr;

    while (dtuEndpoints[3].bufferMessageCount == 0)
        ;

    MessageHeader *header = reinterpret_cast<MessageHeader*>(
      dtuEndpoints[3].bufferReadPtr);
    unsigned char *payload = reinterpret_cast<unsigned char*>(
      dtuEndpoints[3].bufferReadPtr + sizeof(*header));

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

    char* reply = (char*) malloc(32);

    for (int i = 0; i < 32; i++)
        reply[i] = (i + 1) << 4;

    dtuEndpoints[3].messageAddr = reinterpret_cast<uint64_t>(reply);
    dtuEndpoints[3].messageSize = 32;
    *dtuCommandPtr = 0x0d;

    // wait until operation finished
    while (*dtuStatusPtr);

    printf("Slave: done\n");

    free(data);
    free(reply);

    while (true) ;

    return 0;
}
