#ifndef SEI_TIMESTAMP_H
#define SEI_TIMESTAMP_H

#define SEI_TIMESTAMPING 1

#if SEI_TIMESTAMPING

#include <stdint.h>

/* Marker to prevent 21 consequtive zeros, its illegal. */
#define SEI_BIT_DELIMITER 0x81

extern const unsigned char ltn_uuid_sei_timestamp[16];
extern int g_sei_timestamping;

/* Format of LTN_SEI_TAG_START_TIME record:
 * All records are big endian.
 * Field#              : Description
 *         TT TT TT TT : 16 byte UUID (ltn_uuid_sei_timestamp)
 * 1       FC FC FC FC : incrementing frame counter.
 * 2       HS HS HS HS : time received from hardware seconds (timeval.ts_sec).
 * 3       HU HU HU HU : time received from hardware useconds (timeval.ts_usec).
 * 4       SS SS SS SS : time send to compressor seconds (timeval.ts_sec).
 * 5       SU SU SU SU : time send to compressor useconds (timeval.ts_usec).
 * 6       ES ES ES ES : time exit from compressor seconds (timeval.ts_sec).
 * 7       EU EU EU EU : time exit from compressor useconds (timeval.ts_usec).
 * 8       EN EN EN EN : time exit from udp transmitter (timeval.ts_sec).
 * 9       EN EN EN EN : time exit from udp transmitter (timeval.ts_usec).
 */

#define SEI_TIMESTAMP_FIELD_COUNT (9)
#define SEI_TIMESTAMP_PAYLOAD_LENGTH (sizeof(ltn_uuid_sei_timestamp) + (SEI_TIMESTAMP_FIELD_COUNT * 6))

unsigned char *set_timestamp_alloc();
int            set_timestamp_field_set(unsigned char *buffer, uint32_t nr, uint32_t value);

/* Find UUID in buffer, return buffer index or < 0 if found found. */
int ltn_uuid_find(const unsigned char *buf, unsigned int lengthBytes);

#endif /* SEI_TIMESTAMPING */

#endif /* SEI_TIMESTAMP_H */
