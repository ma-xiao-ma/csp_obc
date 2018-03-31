/*
 * task_user.h
 *
 *  Created on: 2015年11月16日
 *      Author: iceyuyu
 */

#ifndef SRC_TASK_H_
#define SRC_TASK_H_

#include <stdint.h>

/** Example defines */
#define MY_ADDRESS  2           // Address of local CSP node
#define MY_PORT     10          // Port to send test traffic to

#if( MY_ADDRESS == 1 )
#define ANOTHER_OBC_ADDR  2
#define CONFIG_HOSTNAME   "OBC-1"
#elif( MY_ADDRESS == 2 )
#define ANOTHER_OBC_ADDR  1
#define CONFIG_HOSTNAME   "OBC-2"
#elif( MY_ADDRESS == 3 )
#define ANOTHER_OBC_ADDR  2
#define CONFIG_HOSTNAME   "OBC-3"
#elif( MY_ADDRESS == 4 )
#define ANOTHER_OBC_ADDR  3
#define CONFIG_HOSTNAME   "OBC-4"
#endif

#define CONFIG_MODEL        "NJUST"
#define GIT_REVISION        "b0a6153"


/** Default node address */
#define NODE_OBC                1

/** Beacon Downlink address and port */
#define OBC_HK_DOWN_NODE        10
#define OBC_HK_DOWN_PORT        14

/** Port numbers */
#define OBC_PORT_FTP                7
#define OBC_PORT_TIMESYNC           8
#define OBC_PORT_LOAD_IMG           9
#define OBC_PORT_JUMP               10
#define OBC_PORT_BOOT_CONF          11
#define OBC_PORT_FS_TO_FLASH        12
#define OBC_PORT_RAM_TO_ROM         13
#define OBC_PORT_LOGD               14
#define OBC_PORT_PARAM              15
#define OBC_PORT_HK                 16
#define OBC_PORT_BOOT_COUNT         17

#define OBC_PORT_FP                 18
#define OBC_PORT_CONN_LESS_TEST     19
#define OBC_PORT_ADCS               20

#define OBC_PORT_RSH                22

typedef struct obc_ram_to_rom_s {
    uint32_t size;
    uint32_t checksum;
    uint32_t src;
    uint32_t dst;
} obc_ram_to_rom_t;

void obc_set_node(uint8_t node);
void obc_jump_ram(uint32_t addr);
void obc_load_image(const char * path);
void obc_boot_conf(uint32_t checksum, uint32_t boot_counts, const char * path);
void obc_boot_del(void);
void obc_fs_to_flash(uint32_t addr, const char * path);
void obc_ram_to_rom(uint32_t size, uint32_t checksum, uint32_t src, uint32_t dst);
int obc_boot_count_get(uint32_t *boot_count, int timeout);
int obc_boot_count_reset(uint32_t *boot_count, int timeout);

void task_initz(void);

#endif /* SRC_TASK_H_ */
