/*
 * task_server.c
 *
 *  Created on: 2018年3月31日
 *      Author: Ma
 */

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <fcntl.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include <csp/csp.h>
#include <csp/csp_endian.h>

#include <util/crc32.h>
#include <util/error.h>
#include <bsp_ds1302.h>

#include <task_user.h>

#if ENABLE_RTC
#include <dev/arm/ds1302.h>
#endif

void vWork(csp_conn_t * conn, int timeout) {

    csp_packet_t * packet = NULL;

    /* Read incoming data */
    while ((packet = csp_read(conn, timeout)) != NULL) {

        //printf("Worker reading port %u\r\n", csp_conn_dport(conn));

        switch (csp_conn_dport(conn)) {

        case OBC_PORT_TIMESYNC: {

            /* Get incoming time data */
            timestamp_t time;
            memcpy(&time, packet->data, sizeof(timestamp_t));
            time.tv_sec = csp_ntoh32(time.tv_sec);
            time.tv_nsec = csp_ntoh32(time.tv_nsec);

            /* Sync */
            if (time.tv_sec != 0)
                clock_set_time(&time);

            /* Return */
            clock_get_time(&time);

            /* Send reply */
            time.tv_sec = csp_hton32(time.tv_sec);
            time.tv_nsec = csp_hton32(time.tv_nsec);

            memcpy(packet->data, &time, sizeof(timestamp_t));

            packet->length = sizeof(timestamp_t);
            if (!csp_send(conn, packet, 0))
                csp_buffer_free(packet);

            break;
        }

#ifdef ENABLE_UFFS
        case OBC_PORT_BOOT_CONF: {

            uint32_t checksum;
            memcpy(&checksum, packet->data, sizeof(checksum));
            checksum = csp_ntoh32(checksum);

            printf("Got boot config, checksum %#"PRIX32"\r\n", checksum);

            /* If checksum is magic, delete boot conf */
            if (checksum == 0x80078007) {
                remove("/boot/boot.conf");
                printf("Deleted boot config\r\n");
                csp_buffer_free(packet);
                break;
            }

            char * path = (char *) packet->data + sizeof(checksum);

            if (!strnlen(path, 100)) {
                csp_buffer_free(packet);
                break;
            }

            printf("Inserting path %s to /boot/boot.conf\r\n", path);

            int fd = open("/boot/boot.conf", O_WRONLY | O_CREAT);
            if (fd < 0) {
                csp_buffer_free(packet);
                break;
            }

            if (write(fd, path, strlen(path)) < (int) strlen(path)) {
                close(fd);
                csp_buffer_free(packet);
                break;
            }

            char pbuf[20];
            /* Lines in /boot/boot.conf must end with newline and no linefeed! */
            sprintf(pbuf, "\n%"PRIX32"\n", checksum);
            printf("Checksum: %#"PRIX32"\r\n", checksum);
            if (write(fd, pbuf, strlen(pbuf)) < (int) strlen(pbuf)) {
                close(fd);
                csp_buffer_free(packet);
                break;
            }

            close(fd);
            csp_buffer_free(packet);
            break;
        }

#if defined(OBC_ROM)
        case OBC_PORT_LOAD_IMG: {

            char * path = (char *) packet->data;

            if (!strnlen(path, 100)) {
                csp_buffer_free(packet);
                break;
            }

            int fd = open(path, O_RDONLY);
            if (fd < 0) {
                printf("Failed to open file %s\r\n", path);
                csp_buffer_free(packet);
                break;
            }

            printf("File opened\r\n");

            void * dst = (void *) 0x50000000;

            /* Read file size */
            unsigned int size;
            struct stat st;
            if (stat(path, &st) != 0) {
                printf("Failed to stat image file\r\n");
                close(fd);
                csp_buffer_free(packet);
                break;
            } else {
                size = st.st_size;
            }
            unsigned int copied = 0;

            printf("Copying %u bytes to %p\r\n", size, dst);

            copied = read(fd, dst, size);
            if (copied != size) {
                printf("Failed to copy image\r\n");
                close(fd);
                csp_buffer_free(packet);
                break;
            }

            printf("Copied %u bytes\r\n", copied);

            /* Checking checksum */
            unsigned int checksum_ram = chksum_crc32((unsigned char *) dst, size);
            printf("Checksum RAM: 0x%x\r\n", checksum_ram);

            close(fd);

            checksum_ram = csp_hton32(checksum_ram);
            memcpy(packet->data, &checksum_ram, sizeof(uint32_t));
            packet->length = 4;

            if (!csp_send(conn, packet, 0))
                csp_buffer_free(packet);

            break;
        }
#endif // OBC_ROM
#endif // ENABLE_UFFS

        case OBC_PORT_JUMP: {

            uint32_t addr;
            memcpy(&addr, packet->data, 4);
            addr = csp_ntoh32(addr);

            /* Check for valid address */
            if ((addr < 0x50000000) || (addr > 0x50200000)) {
                printf("Invalid jump addr %p\r\n", (void *) addr);
                csp_buffer_free(packet);
                break;
            }

            /* Jump to address */
            printf("Jumping to addr %p\r\n", (void *) addr);

            void (*jump) (void) = (void *) addr;
            jump();

            break;
        }

        case OBC_PORT_RAM_TO_ROM: {

//            unsigned int checksum_ram, checksum_rom;
//
//            obc_ram_to_rom_t rxbuf;
//            memcpy(&rxbuf, packet->data, sizeof(obc_ram_to_rom_t));
//            rxbuf.size = csp_ntoh32(rxbuf.size);
//            rxbuf.checksum = csp_ntoh32(rxbuf.checksum);
//            rxbuf.src = csp_ntoh32(rxbuf.src);
//            rxbuf.dst = csp_ntoh32(rxbuf.dst);
//
//            /* No need for the packet anymore */
//            csp_buffer_free(packet);
//
//            printf("Checksum IMG: %#"PRIx32", Size %"PRIu32", src %p, dst %p\r\n", rxbuf.checksum, rxbuf.size, (void *) rxbuf.src, (void *) rxbuf.dst);
//
//            checksum_ram = chksum_crc32((unsigned char *)rxbuf.src, rxbuf.size);
//            printf("Checksum RAM: %#x\r\n", checksum_ram);
//
//            /* Validate RAM image */
//            if (checksum_ram != rxbuf.checksum) {
//                printf("Failed to validate image in RAM. Aborting\r\n");
//                break;
//            }
//
//            /* Init flash */
//            if (flash_init() != E_NO_ERR) {
//                printf("Failed to init flash\r\n");
//                break;
//            }
//
//            /* Write RAM image to ROM */
//            int ret = flash_program((void *) rxbuf.dst, (void *) rxbuf.src, rxbuf.size, 1);
//            if (ret != E_NO_ERR) {
//                printf("Failed to copy image from RAM to ROM (err: %d)\r\n", ret);
//                break;
//            }
//
//            /* Validate ROM image */
//            checksum_rom = flash_checksum((unsigned char *)rxbuf.dst, rxbuf.size);
//            printf("Checksum ROM: %#x\r\n", checksum_rom);
//            if (checksum_rom != rxbuf.checksum) {
//                /* Oh crap... */
//                printf("Failed to validate image in ROM. Aborting\r\n");
//                break;
//            }
//
//            /* Everything went well */
//            printf("Successfully copied image from RAM to ROM\r\n");
//            break;
        }

        default:
            /* If no port has matched yet, pass the call to CSP service handler */
            csp_service_handler(conn, packet);
            return;

        }

    }

}

void vTaskWorker(void * conn_param) {
    vWork((csp_conn_t *) conn_param, portMAX_DELAY);
    csp_close((csp_conn_t *) conn_param);
    vTaskDelete(NULL);
}

void vTaskServer(void * pvParameters)
{
    csp_conn_t * conn, * socket = csp_socket(0);
    csp_listen(socket, 10);
    csp_bind(socket, CSP_ANY);

    /* Get supervisor ID */
//    int sv = sv_add("Server", 60000);
//    if (sv < 0)
//        printf("Failed to get supervisor ID!\r\n");

    while (1) {

        /* Wait for incoming connection, or timeout */
        conn = csp_accept(socket, 0.2 * configTICK_RATE_HZ);

        /* Reset supervisor */
//        sv_reset(sv);

        if (conn == NULL)
            continue;

        /* Spawn new task for SW upload */
//        if (csp_conn_dport(conn) == OBC_PORT_FTP) {
//            extern void task_ftp(void * conn_param);
//            if (xTaskCreate(task_ftp, (signed char *) "FTP", 1024*4, conn, 2, NULL) != pdTRUE) {
//                printf("Failed to allocate memory for RAM upload task\r\n");
//                csp_close(conn);
//            }
//            continue;
//        }

        /* Spawn new task for RDP */
        if ((csp_conn_flags(conn) & CSP_FRDP) ||
            (csp_conn_dport(conn) == OBC_PORT_LOAD_IMG) ||
            (csp_conn_dport(conn) == OBC_PORT_RAM_TO_ROM)) {
            if (xTaskCreate(vTaskWorker, (signed char *) "WORKER", 1024*4, conn, 2, NULL) != pdTRUE) {
                printf("Failed to allocate memory for WORKER task\r\n");
                csp_close(conn);
            }
            continue;
        }

        /* Otherwise execute work directly in server task! */
        vWork(conn, 0);

        /* Close connection when no more data */
        csp_close(conn);

    }

}

//CSP_DEFINE_TASK(task_server) {
//
//    /* Create socket without any socket options */
//    csp_socket_t *sock = csp_socket(CSP_SO_NONE);
//
//    /* Bind all ports to socket */
//    csp_bind(sock, CSP_ANY);
//
//    /* Create 10 connections backlog queue */
//    csp_listen(sock, 10);
//
//    /* Pointer to current connection and packet */
//    csp_conn_t *conn;
//    csp_packet_t *packet;
//
//    /* Process incoming connections */
//    while (1) {
//
//        /* Wait for connection, 10000 ms timeout */
//        if ((conn = csp_accept(sock, 10000)) == NULL)
//            continue;
//
//        /* Read packets. Timout is 100 ms */
//        while ((packet = csp_read(conn, 100)) != NULL)
//        {
//            switch (csp_conn_dport(conn))
//            {
//                case MY_PORT:
//                    /* Process packet here */
//
////                    printf("Packet received on MY_PORT: %s\r\n", (char *) packet->data);
//
//                    log_info( "Server: Packet received on MY_PORT: %s\r\n", (char *) packet->data );
//                    csp_buffer_free(packet);
//                    break;
//
//                default:
//                    /* Let the service handler reply pings, buffer use, etc. */
//                    csp_service_handler(conn, packet);
//                    break;
//            }
//        }
//
//        /* Close current connection, and handle next */
//        csp_close(conn);
//    }
//}
