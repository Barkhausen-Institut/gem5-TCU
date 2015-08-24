#include <malloc.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <unistd.h>

#include "../dtu.h"

#ifndef PE_ID
#define PE_ID (-1)
#endif

enum {
	MSG_SIZE	= 64,
	BUF_SLOTS	= 16,
	BUF_SIZE	= MSG_SIZE * BUF_SLOTS,
};

void pe_printf(const char *format, ...)
{
	va_list args;

	printf("PE%d: ", PE_ID);

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

	pe_printf("ep%u, Received %s of %d bytes @ %p from PE%d (ep %d): %s",
	       ep,
	       header->flags & 1 ? "reply" : "message",
	       header->length,
	       header,
	       header->senderCoreId,
	       header->senderEpId,
	       valid ? "valid" : "invalid");

	if (!valid)
	{
		printf("PE%u: ep%u, invalid message:", PE_ID, ep);
		for (int i = 0; i < header->length; i++)
			printf( " %2.2x", message[i]);
		printf("\n");
	}
}

int main()
{
	pe_printf("Hello World!");

	srand(PE_ID);

	pe_printf("setup endpoints");

	uint8_t* messageBuffer = (uint8_t*) memalign(4096, MSG_SIZE - sizeof(MessageHeader));
	// ringbuffers
	uint8_t* ep2_buffer = (uint8_t*) memalign(4096, BUF_SIZE);
	uint8_t* ep3_buffer = (uint8_t*) memalign(4096, BUF_SIZE);
	uint8_t* ep4_buffer = (uint8_t*) memalign(4096, BUF_SIZE);
	uint8_t* ep5_buffer = (uint8_t*) memalign(4096, BUF_SIZE);

	// endpoints 0 and 1 send messages
	dtuEndpoints[0].mode = 1;
	dtuEndpoints[0].maxMessageSize = MSG_SIZE;
	dtuEndpoints[0].messageAddr = reinterpret_cast<uint64_t>(messageBuffer);
	dtuEndpoints[0].credits = MSG_SIZE;
	dtuEndpoints[0].label = 0xDEADBEEFC0DEFEED;
	dtuEndpoints[0].replyLabel = 0xFEDCBA0987654321;
	dtuEndpoints[1].mode = 1;
	dtuEndpoints[1].maxMessageSize = MSG_SIZE;
	dtuEndpoints[1].messageAddr = reinterpret_cast<uint64_t>(messageBuffer);
	dtuEndpoints[1].credits = MSG_SIZE;
	dtuEndpoints[1].label = 0xDEADBEEFC0DEFEED;
	dtuEndpoints[1].replyLabel = 0xFEDCBA0987654321;

	// endpoints 2 and 3 receive messages
	dtuEndpoints[2].mode = 0;
	dtuEndpoints[2].maxMessageSize = MSG_SIZE;
	dtuEndpoints[2].bufferSize = BUF_SLOTS;
	dtuEndpoints[2].messageAddr = reinterpret_cast<uint64_t>(messageBuffer);
	dtuEndpoints[2].bufferAddr = reinterpret_cast<uint64_t>(ep2_buffer);
	dtuEndpoints[2].bufferReadPtr = reinterpret_cast<uint64_t>(ep2_buffer);
	dtuEndpoints[2].bufferWritePtr = reinterpret_cast<uint64_t>(ep2_buffer);
	dtuEndpoints[3].mode = 0;
	dtuEndpoints[3].maxMessageSize = MSG_SIZE;
	dtuEndpoints[3].messageAddr = reinterpret_cast<uint64_t>(messageBuffer);
	dtuEndpoints[3].bufferSize = BUF_SLOTS;
	dtuEndpoints[3].bufferAddr = reinterpret_cast<uint64_t>(ep3_buffer);
	dtuEndpoints[3].bufferReadPtr = reinterpret_cast<uint64_t>(ep3_buffer);
	dtuEndpoints[3].bufferWritePtr = reinterpret_cast<uint64_t>(ep3_buffer);

	// endpoints 4 and 5 receive replies
	dtuEndpoints[4].mode = 0;
	dtuEndpoints[4].maxMessageSize = MSG_SIZE;
	dtuEndpoints[4].bufferSize = BUF_SLOTS;
	dtuEndpoints[4].bufferAddr = reinterpret_cast<uint64_t>(ep4_buffer);
	dtuEndpoints[4].bufferReadPtr = reinterpret_cast<uint64_t>(ep4_buffer);
	dtuEndpoints[4].bufferWritePtr = reinterpret_cast<uint64_t>(ep4_buffer);
	dtuEndpoints[5].mode = 0;
	dtuEndpoints[5].maxMessageSize = MSG_SIZE;
	dtuEndpoints[5].bufferSize = BUF_SLOTS;
	dtuEndpoints[5].bufferAddr = reinterpret_cast<uint64_t>(ep5_buffer);
	dtuEndpoints[5].bufferReadPtr = reinterpret_cast<uint64_t>(ep5_buffer);
	dtuEndpoints[5].bufferWritePtr = reinterpret_cast<uint64_t>(ep5_buffer);

	while(true)
	{
		for (int ep = 2; ep <= 5; ep++)
		{
			if(dtuEndpoints[ep].bufferMessageCount > 0)
			{
				uint8_t* message =
					reinterpret_cast<uint8_t*>(dtuEndpoints[ep].bufferReadPtr);

				printMessage(message, ep);

				if (ep < 4) // send reply only if we received a message
				{
					// choose parameters randomly
					dtuEndpoints[ep].messageSize = rand() % (MSG_SIZE - sizeof(MessageHeader)) + 1;
					// fill message buffer with data
					uint8_t start = rand();
					for (unsigned i = 0; i < dtuEndpoints[ep].messageSize; i++)
						messageBuffer[i] = start + i;

					pe_printf("ep %u: Send reply of %u bytes",
					          ep,
					          dtuEndpoints[ep].messageSize);

					*dtuCommandPtr = Command::START_OPERATION | (ep << COMMAND_OPCODE_BITS);

					// wait until operation finished
					while (*dtuStatusPtr);
				}

				// increment read pointer
				*dtuCommandPtr = Command::INC_READ_PTR | (ep << COMMAND_OPCODE_BITS);
			}
		}

		if(rand() % 8 == 0)
		{
			unsigned ep = rand() % 2; // 0 or 1

			if (dtuEndpoints[ep].credits >= MSG_SIZE)
			{
				// choose parameters randomly
				dtuEndpoints[ep].messageSize = rand() % (MSG_SIZE - sizeof(MessageHeader)) + 1;
				dtuEndpoints[ep].targetCoreId = rand() % 8; // 0 to 7
				dtuEndpoints[ep].targetEpId = rand() % 2 + 2; // 2 or 3
				dtuEndpoints[ep].replyEpId = rand() % 2 + 4; // 4 or 5

				// fill message buffer with data
				uint8_t start = rand();
				for (unsigned i = 0; i < dtuEndpoints[ep].messageSize; i++)
					messageBuffer[i] = start + i;

				pe_printf("ep %d: Send message of %u bytes to PE%u (ep %u, reply %d)",
						  ep,
						  dtuEndpoints[ep].messageSize,
						  dtuEndpoints[ep].targetCoreId,
						  dtuEndpoints[ep].targetEpId,
						  dtuEndpoints[ep].replyEpId);

				*dtuCommandPtr = Command::START_OPERATION | (ep << COMMAND_OPCODE_BITS);

				// wait until operation finished
				while (*dtuStatusPtr);
			}
		}
	}
}
