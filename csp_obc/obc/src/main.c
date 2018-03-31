#include <if_downlink_serial.h>
#include "stm32f4xx.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "bsp_usart.h"
#include "console.h"
#include "command.h"

#include "bsp_adc.h"
#include "bsp_switch.h"
#include "bsp_watchdog.h"
#include "bsp_reset.h"
#include "if_downlink_vu.h"

#include "task_user.h"

#include "cube_com.h"
#include "contrl.h"
#include "hk.h"
#include "task_monitor.h"
#include "usart.h"

#include <log/log.h>
#include <csp/csp.h>
/* Using un-exported header file.
 * This is allowed since we are still in libcsp */
#include <csp/arch/csp_thread.h>
#include <csp_console.h>


#include <csp/interfaces/csp_if_i2c.h>
#include <csp/interfaces/csp_if_can.h>
#include <csp/interfaces/csp_if_kiss.h>

CSP_DEFINE_TASK(task_client) {

    csp_packet_t * packet;
    csp_conn_t * conn;

    while (1) {

        /**
         * Try ping
         */

        csp_sleep_ms(1000);

        int result = csp_ping(ANOTHER_OBC_ADDR, 100, 100, CSP_O_NONE);

        log_info( "Client: Ping result %d [ms]\r\n", result );
//        printf("Ping result %d [ms]\r\n", result);

        csp_sleep_ms(1000);

        /**
         * Try data packet to server
         */

        /* Get packet buffer for data */
        packet = csp_buffer_get(100);
        if (packet == NULL) {
            /* Could not get buffer element */
            printf("Failed to get buffer element\n");
        }

        /* Connect to host HOST, port PORT with regular UDP-like protocol and 1000 ms timeout */
        conn = csp_connect(CSP_PRIO_NORM, ANOTHER_OBC_ADDR, MY_PORT, 1000, CSP_O_NONE);
        if (conn == NULL) {
            /* Connect failed */
            printf("Connection failed\n");
            /* Remember to free packet buffer */
            csp_buffer_free(packet);
        }

        /* Copy dummy data to packet */
        char *msg = "Hello World";
        strcpy((char *) packet->data, msg);

        /* Set packet length */
        packet->length = strlen(msg);

        /* Send packet */
        if (!csp_send(conn, packet, 1000)) {
            /* Send failed */
            printf("Send failed\n");
            csp_buffer_free(packet);
        }

        /* Close connection */
        csp_close(conn);

    }
}

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    tty.init();

    extern void vTaskInit(void *pvParameters);
    xTaskCreate(vTaskInit, (const signed char *) "INIT", 1024, NULL, 3, NULL);

    extern void vTaskServer(void * pvParameters);
    /* Server */
    printf("Starting Server task\r\n");
    csp_thread_handle_t handle_server;
    int ret = csp_thread_create(vTaskServer, "SRV", 1024, NULL, 2, &handle_server);
    if (ret != CSP_ERR_NONE) {
        csp_log_error("Failed to start USART task");
    }

//    /* Client */
//    printf("Starting Client task\r\n");
//    csp_thread_handle_t handle_client;
//    ret = csp_thread_create(task_client, "CLIENT", 512, NULL, 1, &handle_client);
//    if (ret != CSP_ERR_NONE) {
//        csp_log_error("Failed to start USART task");
//    }

//    void hook_main(void);
//    hook_main();

    /* Timer uses LFCLOCK = F_OSC/2 */
    vTaskStartScheduler();

    /* Should never reach here */
    while(1);

}

