#ifndef CLIENT_H
#define CLIENT_H

const char server_address[5] = {0xE7, 0xE7, 0x18, 0x18, 0xE1};
const char client0_address[5] = {0x18, 0x18, 0xE7, 0xE7, 0xE2};

struct status_packet{
        uint16_t magic;
	uint16_t sequence_nr;
	uint16_t wakeups;
	uint8_t  vdc;
	uint8_t  version;
	uint8_t  status[4];
};

#endif 
