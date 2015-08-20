#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "dtu.h"
#include "m5op.h"

int main()
{
    printf("Master: Hello World!\n");

    // wait a bit to allow for PE2 to setup it's endpoint
    for (volatile int i;i < 0x100; i++);

    printf("Master: Setup Endpoint 1 to send messages to PE2 endpoint 3\n");

    dtuEndpoints[1].mode = 1; // send messages
    dtuEndpoints[1].maxMessageSize = 64;
    dtuEndpoints[1].targetCoreId = 2;
    dtuEndpoints[1].targetEpid = 3;
    dtuEndpoints[1].messageSize = 32;
    dtuEndpoints[1].replyEpId = 5;
    dtuEndpoints[1].credits = 128;

    printf("Master: Setup Endpoint 5 to receive messages\n");

    dtuEndpoints[5].mode = 0; // receive messages
    dtuEndpoints[5].maxMessageSize = 64;
    dtuEndpoints[5].bufferSize = 4;

    char* data = (char*) malloc(256);

    uint64_t addr = reinterpret_cast<uint64_t>(data);

    dtuEndpoints[5].bufferAddr = addr;
    dtuEndpoints[5].bufferReadPtr = addr;
    dtuEndpoints[5].bufferWritePtr = addr;

    printf("Master: send Message\n");

    char* message = (char*) malloc(32);

    dtuEndpoints[1].messageAddr = reinterpret_cast<uint64_t>(data);

    for (int i = 0; i < 32; i++)
        data[i] = i + 1;

    *dtuCommandPtr = 0x5;

    // wait until operation finished
    while (*dtuStatusPtr);

    while (dtuEndpoints[5].bufferMessageCount == 0)
        ;

    volatile unsigned char* reply = (unsigned char*) dtuEndpoints[5].bufferReadPtr;

    printf("Master: Received a reply! Its located at %#x\n", reply);

    for (int i = 0; i < 8; i++)
        printf("Master: Message: %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x\n",
               reply[i*8+0],
               reply[i*8+1],
               reply[i*8+2],
               reply[i*8+3],
               reply[i*8+4],
               reply[i*8+5],
               reply[i*8+6],
               reply[i*8+7]);

    free(message);
    free(data);

    while (true) ;

    return 0;
}
