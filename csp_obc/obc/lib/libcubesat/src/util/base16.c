/*
 * base16.c
 *
 *  Created on: 2018年3月31日
 *      Author: Ma
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <util/log.h>

void base16_encode(uint8_t *raw, size_t len, char *encoded) {
    uint8_t *raw_bytes = raw;
    char *encoded_bytes = encoded;
    size_t remaining = len;

    for (; remaining--; encoded_bytes += 2)
        snprintf(encoded_bytes, 3, "%02X", *(raw_bytes++));

}

int base16_decode(const char *encoded, uint8_t *raw) {
    const char *encoded_bytes = encoded;
    uint8_t *raw_bytes = raw;
    char buf[3];
    char *endp;
    size_t len;

    while (encoded_bytes[0]) {
        if (!encoded_bytes[1]) {
            printf("Base16-encoded string \"%s\" has invalid length\n",
                    encoded);
            return -22;
        }
        memcpy(buf, encoded_bytes, 2);
        buf[2] = '\0';
        *(raw_bytes++) = strtoul(buf, &endp, 16);
        if (*endp != '\0') {
            printf("Base16-encoded string \"%s\" has invalid byte \"%s\"\n",
                    encoded, buf);
            return -22;
        }
        encoded_bytes += 2;
    }
    len = (raw_bytes - raw);
    return (len);
}

