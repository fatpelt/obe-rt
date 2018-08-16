#include <encoders/video/sei-timestamp.h>

#if SEI_TIMESTAMPING

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

const unsigned char ltn_uuid_sei_timestamp[] =
{
    0x59, 0x96, 0xFF, 0x28, 0x17, 0xCA, 0x41, 0x96, 0x8D, 0xE3, 0xE5, 0x3F, 0xE2, 0xF9, 0x92, 0xAE
};

unsigned char *set_timestamp_alloc()
{
	unsigned char *p = calloc(1, SEI_TIMESTAMP_PAYLOAD_LENGTH);
	if (!p)
		return NULL;

	memcpy(p, &ltn_uuid_sei_timestamp[0], sizeof(ltn_uuid_sei_timestamp));
	return p;
}

int set_timestamp_field_set(unsigned char *buffer, uint32_t nr, uint32_t value)
{
	if (nr < 1 || nr > SEI_TIMESTAMP_FIELD_COUNT)
		return -1;

	unsigned char *p = buffer;
	p += (sizeof(ltn_uuid_sei_timestamp) + ((nr - 1) * 6));
	*(p++) = (value >> 24) & 0xff;
	*(p++) = (value >> 16) & 0xff;
	*(p++) = SEI_BIT_DELIMITER;
	*(p++) = (value >>  8) & 0xff;
	*(p++) = (value >>  0) & 0xff;
	*(p++) = SEI_BIT_DELIMITER;

	return 0;
}

int ltn_uuid_find(const unsigned char *buf, unsigned int lengthBytes)
{
	for (int i = 0; i < lengthBytes - sizeof(ltn_uuid_sei_timestamp); i++) {
		if (memcmp(buf + i, &ltn_uuid_sei_timestamp[0], sizeof(ltn_uuid_sei_timestamp)) == 0) {
			return i;
		}
	}

	return -1;
}

#endif /* SEI_TIMESTAMPING */
