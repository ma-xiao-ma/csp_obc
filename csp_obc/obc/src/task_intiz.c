/*
 * task_intiz.c
 *
 *  Created on: 2015年11月16日
 *      Author: iceyuyu
 */

#include "bsp_usart.h"
#include "console.h"
#include "command.h"

#include "bsp_adc.h"
#include "bsp_switch.h"
#include "bsp_watchdog.h"
#include "bsp_pca9665.h"
#include "bsp_spi.h"
#include "bsp_sdio_sd.h"
#include "bsp_nor_flash.h"
#include "bsp_fsmc_sram.h"
#include "bsp_ds1302.h"
#include "bsp_ad7490.h"
#include "bsp_intadc.h"
#include "bsp_temp175.h"
//#include "bsp_camera.h"
#include "if_downlink_serial.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "ff.h"

#include "usart.h"
#include "route.h"
#include "router_io.h"
#include "obc_argvs_save.h"
#include "hk.h"
#include "contrl.h"
#include "switches.h"
#include "camera_805.h"
#include "driver_debug.h"
#include "task_monitor.h"


#include <csp/csp.h>
/* Using un-exported header file.
 * This is allowed since we are still in libcsp */
#include <csp/arch/csp_thread.h>

/** Example defines */
#define MY_ADDRESS  1           // Address of local CSP node
#define MY_PORT     10          // Port to send test traffic to


//extern unsigned char driver_debug_switch[DEBUG_ENUM_MAX+1];

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
//        while ((packet = csp_read(conn, 100)) != NULL) {
//            switch (csp_conn_dport(conn)) {
//            case MY_PORT:
//                /* Process packet here */
//                printf("Packet received on MY_PORT: %s\r\n", (char *) packet->data);
//                csp_buffer_free(packet);
//                break;
//
//            default:
//                /* Let the service handler reply pings, buffer use, etc. */
//                csp_service_handler(conn, packet);
//                break;
//            }
//        }
//
//        /* Close current connection, and handle next */
//        csp_close(conn);
//    }
//}
//
//CSP_DEFINE_TASK(task_client) {
//
//    csp_packet_t * packet;
//    csp_conn_t * conn;
//
//    while (1) {
//
//        /**
//         * Try ping
//         */
//
//        csp_sleep_ms(1000);
//
//        int result = csp_ping(MY_ADDRESS, 100, 100, CSP_O_NONE);
//        printf("Ping result %d [ms]\r\n", result);
//
//        csp_sleep_ms(1000);
//
//        /**
//         * Try data packet to server
//         */
//
//        /* Get packet buffer for data */
//        packet = csp_buffer_get(100);
//        if (packet == NULL) {
//            /* Could not get buffer element */
//            printf("Failed to get buffer element\n");
//        }
//
//        /* Connect to host HOST, port PORT with regular UDP-like protocol and 1000 ms timeout */
//        conn = csp_connect(CSP_PRIO_NORM, MY_ADDRESS, MY_PORT, 1000, CSP_O_NONE);
//        if (conn == NULL) {
//            /* Connect failed */
//            printf("Connection failed\n");
//            /* Remember to free packet buffer */
//            csp_buffer_free(packet);
//        }
//
//        /* Copy dummy data to packet */
//        char *msg = "Hello World";
//        strcpy((char *) packet->data, msg);
//
//        /* Set packet length */
//        packet->length = strlen(msg);
//
//        /* Send packet */
//        if (!csp_send(conn, packet, 1000)) {
//            /* Send failed */
//            printf("Send failed\n");
//            csp_buffer_free(packet);
//        }
//
//        /* Close connection */
//        csp_close(conn);
//
//    }
//}

//void task_initz(void)
//{
//    extern uint8_t driver_debug_switch[DEBUG_ENUM_MAX+1];
//
//    driver_debug_switch[DEBUG_HK] = 1;
//
//    /* 任务监视器初始化,线程超时时间15s */
//    supervisor_init(15000);
//
//    /*控制开关IO口初始化*/
//    bsp_InitSwitch();
//    /*姿控上电*/
//    EpsOutSwitch(OUT_EPS_S0, ENABLE);
//
////	Console_Usart_init(115200);
//    /* Initialise USART */
//    usart_stdio_id = 2;
//    usart_init(2, 115200);
//
//#if USE_SERIAL_PORT_DOWNLINK_INTERFACE
//
//	vSerialInterfaceInit();
//
//#endif
//
//	Camera_805_Init();
//
//	int_adc_init();
//
//	spi_init_dev();
////	AD7490_Init();
//	/*power related and switches*/
//	bsp_InitSPI1();
//
//	/*on-chip clock RTC*/
//	bsp_InitDS1302();
//	/* Initialise RTC */
//	struct ds1302_clock clock;
//	timestamp_t timestamp;
//
//	/* Get time from RTC */
//	ds1302_clock_read_burst(&clock);
//	ds1302_clock_to_time((time_t *) &timestamp.tv_sec, &clock);
//	timestamp.tv_nsec = 0;
//
//	/* Set time in lib-c system time*/
//	clock_set_time(&timestamp);
//
//	/*command initialize*/
//	command_init();
//
//	/*use cpu flash to store the info*/
//	obc_argvs_recover();
//
//	/*house-keeping store to SD card*/
//	hk_list_init(&hk_list);
//	hk_list_recover();
//	vTelemetryFileManage(&hk_list);
//
////#if USE_ROUTE_PROTOCOL
////
////    router_init(1, 5);
////
////    /*创建服务器任务*/
////    server_start_task(configMINIMAL_STACK_SIZE, tskIDLE_PRIORITY + 2);
////    /*创建发送处理任务*/
////    send_processing_start_task(configMINIMAL_STACK_SIZE, tskIDLE_PRIORITY + 2);
////    /*创建路由任务*/
////    router_start_task(configMINIMAL_STACK_SIZE*2, tskIDLE_PRIORITY + 1);
////
////#endif
//
//    printf("Initialising CSP\r\n");
//    csp_buffer_init(5, 300);
//
//    /* Init CSP with address MY_ADDRESS */
//    csp_init(MY_ADDRESS);
//
//    /* Start router task with 500 word stack, OS task priority 1 */
//    csp_route_start_task(500, 1);
//
//    printf("Debug enabed\r\n");
//    csp_debug_toggle_level(3);
//    csp_debug_toggle_level(4);
//    csp_debug_toggle_level(2);
//    csp_debug_toggle_level(5);
//    csp_debug_toggle_level(6);
//
//    printf("Conn table\r\n");
//    csp_conn_print_table();
//
//    printf("Route table\r\n");
//    csp_route_print_table();
//
//    printf("Interfaces\r\n");
//    csp_route_print_interfaces();
//
//    /* Server */
//    printf("Starting Server task\r\n");
//    csp_thread_handle_t handle_server;
//    csp_thread_create(task_server, "SERVER", 1000, NULL, 1, &handle_server);
//
//    /* Client */
//    printf("Starting Client task\r\n");
//    csp_thread_handle_t handle_client;
//    csp_thread_create(task_client, "SERVER", 1000, NULL, 1, &handle_client);
//
//    /*I2C(PCA9665) initialize*/
//    PCA9665_IO_Init();
//    //driver_debug_switch[DEBUG_I2C] = 1;
//    i2c_init(0, I2C_MASTER, 0x1A, 40, 5, 5, NULL); //frequency = 40Kbit/s
//    i2c_init(1, I2C_MASTER, 0x08, 60, 5, 5, i2c_rx_callback);
//    pca9665_isr_init();
//
//	/*采温芯片初始化*/
//	temp175_init();
//
//	console_init();
//
//    /* Start USART RX task */
//    extern void vTaskUsartRx(void * pvParameters);
//    xTaskCreate(vTaskUsartRx, (const signed char*) "USART", 1024*2, NULL, 4, NULL);
//
//	cmd_eps_setup();
////	cmd_dfl_setup();
//	extern void cmd_fs_setup(void);
//	cmd_fs_setup();
//	extern void cmd_i2c_setup(void);
//	cmd_i2c_setup();
//	extern void cmd_nor_setup(void);
//	cmd_nor_setup();
//	extern void cmd_sram_setup(void);
//	cmd_sram_setup();
//	extern void cmd_rtc_setup(void);
//	cmd_rtc_setup();
//	extern void cmd_ad7490_setup(void);
//	cmd_ad7490_setup();
//	extern void cmd_pwr_setup(void);
//	cmd_pwr_setup();
//	extern void cmd_switches_setup(void);
//	cmd_switches_setup();
//	extern void cmd_intadc_setup(void);
//	cmd_intadc_setup();
//	extern void cmd_ants_setup(void);
//	cmd_ants_setup();
//	extern void cmd_test_setup(void);
//	cmd_test_setup();
//	extern void cmd_isis_setup(void);
//	cmd_isis_setup();
//	extern void cmd_vu_setup(void);
//	cmd_vu_setup();
//	extern void cmd_norflash_setup(void);
//	cmd_norflash_setup();
//	extern void cmd_camera_setup(void);
//	cmd_camera_setup();
//
//	cmd_ina_temp_setup();
//
////	vTaskDelete(NULL);
//}
