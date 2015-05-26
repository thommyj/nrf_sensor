#ifndef CLIENT_H
#define CLIENT_H

#define STATUS_PACKET_MAGIC   (0xAE51)
#define STATUS_PACKET_HDR_VER (0x1)

const char client0_address[5] = {0x18, 0x18, 0xE7, 0xE7, 0xE2};

struct status_packet{
        uint16_t magic;
	uint16_t sequence_nr;
	uint16_t wakeups;
	uint16_t timeouts;
	uint8_t  vdc;
	uint8_t  version;
	uint8_t  status[4];
};

#endif 
