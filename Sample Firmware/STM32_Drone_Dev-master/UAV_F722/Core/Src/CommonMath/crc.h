/*
 * crc.h
 *
 *  Created on: Nov 21, 2024
 *      Author: Vincent
 */

#ifndef SRC_COMMONMATH_CRC_H_
#define SRC_COMMONMATH_CRC_H_

#include <stdint.h>

struct sbuf_s;

uint16_t crc16_ccitt(uint16_t crc, unsigned char a);
uint16_t crc16_ccitt_update(uint16_t crc, const void *data, uint32_t length);
void crc16_ccitt_sbuf_append(struct sbuf_s *dst, uint8_t *start);

uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a);
uint8_t crc8_dvb_s2_update(uint8_t crc, const void *data, uint32_t length);
void crc8_dvb_s2_sbuf_append(struct sbuf_s *dst, uint8_t *start);

uint8_t crc8_xor_update(uint8_t crc, const void *data, uint32_t length);
void crc8_xor_sbuf_append(struct sbuf_s *dst, uint8_t *start);

uint8_t crc8(uint8_t crc, uint8_t a);
uint8_t crc8_update(uint8_t crc, const void *data, uint32_t length);

uint8_t crc8_sum_update(uint8_t crc, const void *data, uint32_t length);

#endif /* SRC_COMMONMATH_CRC_H_ */
