#include <malloc.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <unistd.h>

#include "../tcu.h"

#ifndef PE_ID
#define PE_ID (-1)
#endif

enum {
	MSG_SIZE	= 64,
	BUF_SLOTS	= 16,
	BUF_SIZE	= MSG_SIZE * BUF_SLOTS,
};

void tile_printf(const char *format, ...)
{
	va_list args;

	printf("T%d: ", PE_ID);

	va_start(args, format);
	vprintf(format, args);
	va_end(args);

	printf("\n");
}

void printMessage(uint8_t* message, unsigned ep)
{
	MessageHeader* header = reinterpret_cast<MessageHeader*>(message);
	message = message + sizeof(MessageHeader);

	int start = message[0];
	bool valid = true;
	for (int i = 0; i < header->length; i++)
	{
		if (message[i] != (start + i) % 256)
		{
			valid = false;
			break;
		}
	}

	tile_printf("ep%u, Received %s of %d bytes @ %p from T%d (ep %d): %s",
	       ep,
	       header->flags & 1 ? "reply" : "message",
	       header->length,
	       header,
	       header->senderCoreId,
	       header->senderEpId,
	       valid ? "valid" : "invalid");

	if (!valid)
	{
		printf("T%u: ep%u, invalid message:", PE_ID, ep);
		for (int i = 0; i < header->length; i++)
			printf( " %2.2x", message[i]);
		printf("\n");
	}
}

int main()
{
	tile_printf("Hello World!");

	srand(PE_ID);

	tile_printf("setup endpoints");

	uint8_t* messageBuffer = (uint8_t*) memalign(4096, MSG_SIZE - sizeof(MessageHeader));
	// ringbuffers
	uint8_t* ep2_buffer = (uint8_t*) memalign(4096, BUF_SIZE);
	uint8_t* ep3_buffer = (uint8_t*) memalign(4096, BUF_SIZE);
	uint8_t* ep4_buffer = (uint8_t*) memalign(4096, BUF_SIZE);
	uint8_t* ep5_buffer = (uint8_t*) memalign(4096, BUF_SIZE);

	// endpoints 0 and 1 send messages
	tcuEndpoints[0].mode = 1;
	tcuEndpoints[0].maxMessageSize = MSG_SIZE;
	tcuEndpoints[0].messageAddr = reinterpret_cast<uint64_t>(messageBuffer);
	tcuEndpoints[0].credits = MSG_SIZE;
	tcuEndpoints[0].label = 0xDEADBEEFC0DEFEED;
	tcuEndpoints[0].replyLabel = 0xFEDCBA0987654321;
	tcuEndpoints[1].mode = 1;
	tcuEndpoints[1].maxMessageSize = MSG_SIZE;
	tcuEndpoints[1].messageAddr = reinterpret_cast<uint64_t>(messageBuffer);
	tcuEndpoints[1].credits = MSG_SIZE;
	tcuEndpoints[1].label = 0xDEADBEEFC0DEFEED;
	tcuEndpoints[1].replyLabel = 0xFEDCBA0987654321;

	// endpoints 2 and 3 receive messages
	tcuEndpoints[2].mode = 0;
	tcuEndpoints[2].maxMessageSize = MSG_SIZE;
	tcuEndpoints[2].bufferSize = BUF_SLOTS;
	tcuEndpoints[2].messageAddr = reinterpret_cast<uint64_t>(messageBuffer);
	tcuEndpoints[2].bufferAddr = reinterpret_cast<uint64_t>(ep2_buffer);
	tcuEndpoints[2].bufferReadPtr = reinterpret_cast<uint64_t>(ep2_buffer);
	tcuEndpoints[2].bufferWritePtr = reinterpret_cast<uint64_t>(ep2_buffer);
	tcuEndpoints[3].mode = 0;
	tcuEndpoints[3].maxMessageSize = MSG_SIZE;
	tcuEndpoints[3].messageAddr = reinterpret_cast<uint64_t>(messageBuffer);
	tcuEndpoints[3].bufferSize = BUF_SLOTS;
	tcuEndpoints[3].bufferAddr = reinterpret_cast<uint64_t>(ep3_buffer);
	tcuEndpoints[3].bufferReadPtr = reinterpret_cast<uint64_t>(ep3_buffer);
	tcuEndpoints[3].bufferWritePtr = reinterpret_cast<uint64_t>(ep3_buffer);

	// endpoints 4 and 5 receive replies
	tcuEndpoints[4].mode = 0;
	tcuEndpoints[4].maxMessageSize = MSG_SIZE;
	tcuEndpoints[4].bufferSize = BUF_SLOTS;
	tcuEndpoints[4].bufferAddr = reinterpret_cast<uint64_t>(ep4_buffer);
	tcuEndpoints[4].bufferReadPtr = reinterpret_cast<uint64_t>(ep4_buffer);
	tcuEndpoints[4].bufferWritePtr = reinterpret_cast<uint64_t>(ep4_buffer);
	tcuEndpoints[5].mode = 0;
	tcuEndpoints[5].maxMessageSize = MSG_SIZE;
	tcuEndpoints[5].bufferSize = BUF_SLOTS;
	tcuEndpoints[5].bufferAddr = reinterpret_cast<uint64_t>(ep5_buffer);
	tcuEndpoints[5].bufferReadPtr = reinterpret_cast<uint64_t>(ep5_buffer);
	tcuEndpoints[5].bufferWritePtr = reinterpret_cast<uint64_t>(ep5_buffer);

	while(true)
	{
		for (int ep = 2; ep <= 5; ep++)
		{
			if(tcuEndpoints[ep].bufferMessageCount > 0)
			{
				uint8_t* message =
					reinterpret_cast<uint8_t*>(tcuEndpoints[ep].bufferReadPtr);

				printMessage(message, ep);

				if (ep < 4) // send reply only if we received a message
				{
					// choose parameters randomly
					tcuEndpoints[ep].messageSize = rand() % (MSG_SIZE - sizeof(MessageHeader)) + 1;
					// fill message buffer with data
					uint8_t start = rand();
					for (unsigned i = 0; i < tcuEndpoints[ep].messageSize; i++)
						messageBuffer[i] = start + i;

					tile_printf("ep %u: Send reply of %u bytes",
					            ep,
					            tcuEndpoints[ep].messageSize);

					*tcuCommandPtr = Command::START_OPERATION | (ep << COMMAND_OPCODE_BITS);

					// wait until operation finished
					while (*tcuStatusPtr);
				}

				// increment read pointer
				*tcuCommandPtr = Command::INC_READ_PTR | (ep << COMMAND_OPCODE_BITS);
			}
		}

		if(rand() % 8 == 0)
		{
			unsigned ep = rand() % 2; // 0 or 1

			if (tcuEndpoints[ep].credits >= MSG_SIZE)
			{
				// choose parameters randomly
				tcuEndpoints[ep].messageSize = rand() % (MSG_SIZE - sizeof(MessageHeader)) + 1;
				tcuEndpoints[ep].targetCoreId = rand() % 8; // 0 to 7
				tcuEndpoints[ep].targetEpId = rand() % 2 + 2; // 2 or 3
				tcuEndpoints[ep].replyEpId = rand() % 2 + 4; // 4 or 5

				// fill message buffer with data
				uint8_t start = rand();
				for (unsigned i = 0; i < tcuEndpoints[ep].messageSize; i++)
					messageBuffer[i] = start + i;

				tile_printf("ep %d: Send message of %u bytes to T%u (ep %u, reply %d)",
						    ep,
						    tcuEndpoints[ep].messageSize,
						    tcuEndpoints[ep].targetCoreId,
						    tcuEndpoints[ep].targetEpId,
						    tcuEndpoints[ep].replyEpId);

				*tcuCommandPtr = Command::START_OPERATION | (ep << COMMAND_OPCODE_BITS);

				// wait until operation finished
				while (*tcuStatusPtr);
			}
		}
	}
}
