/*
 * base16.h
 *
 *  Created on: 2018年3月31日
 *      Author: Ma
 */

#ifndef LIB_LIBCUBESAT_INCLUDE_UTIL_BASE16_H_
#define LIB_LIBCUBESAT_INCLUDE_UTIL_BASE16_H_

#include <stdint.h>
#include <string.h>

/**
 * Calculate length of base16-encoded data
 * @param raw_len Raw data length
 * @return Encoded string length (excluding NUL)
 */
static inline size_t base16_encoded_len(size_t raw_len) {
    return (2 * raw_len);
}

/**
 * Calculate maximum length of base16-decoded string
 * @param encoded Encoded string
 * @return Maximum length of raw data
 */
static inline size_t base16_decoded_max_len(const char *encoded) {
    return ((strlen(encoded) + 1) / 2);
}

/**
 * Base16-encode data
 *
 * The buffer must be the correct length for the encoded string.  Use
 * something like
 *
 *     char buf[ base16_encoded_len ( len ) + 1 ];
 *
 * (the +1 is for the terminating NUL) to provide a buffer of the
 * correct size.
 *
 * @param raw Raw data
 * @param len Length of raw data
 * @param encoded Buffer for encoded string
 */
extern void base16_encode(uint8_t *raw, size_t len, char *encoded);

/**
 * Base16-decode data
 *
 * The buffer must be large enough to contain the decoded data.  Use
 * something like
 *
 *     char buf[ base16_decoded_max_len ( encoded ) ];
 *
 * to provide a buffer of the correct size.
 * @param encoded Encoded string
 * @param raw Raw data
 * @return Length of raw data, or negative error
 */
extern int base16_decode(const char *encoded, uint8_t *raw);

#endif /* LIB_LIBCUBESAT_INCLUDE_UTIL_BASE16_H_ */
