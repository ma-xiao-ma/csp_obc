/*
 * task_intiz.c
 *
 *  Created on: 2018年03月31日
 *      Author: Ma Wenli
 */
#include <csp/csp.h>
/* Using un-exported header file.
 * This is allowed since we are still in libcsp */
#include <csp/arch/csp_thread.h>

#include <log/log.h>
#include <csp/csp.h>
/* Using un-exported header file.
 * This is allowed since we are still in libcsp */
#include <csp/arch/csp_thread.h>
#include <csp_console.h>


#include <csp/interfaces/csp_if_i2c.h>
#include <csp/interfaces/csp_if_can.h>
#include <csp/interfaces/csp_if_kiss.h>

#include "console.h"
#include "command.h"
#include "usart.h"

#include <task_user.h>

static const char * kiss1_name = "KISS1";
static csp_iface_t csp_if_kiss1;
static csp_kiss_handle_t csp_kiss1_driver;

/* KISS */
void usart3_putc(char c) {
    tty.write(&c, 1);
}
void usart3_insert(char c, void * pxTaskWoken) {
    usart_insert(c, pxTaskWoken);
}

void my_usart3_rx(uint8_t * buf, int len, void * pxTaskWoken) {
    csp_kiss_rx(&csp_if_kiss1, buf, len, pxTaskWoken);
}

void init_csp(void)
{
    /* Initialise CSP */
    log_csp_init();

    csp_buffer_init(10, 300);

    csp_init(MY_ADDRESS);

    csp_set_hostname(CONFIG_HOSTNAME);
    csp_set_model(CONFIG_MODEL);
    csp_set_revision(GIT_REVISION);

    /* CAN */
    struct csp_can_config conf = {0};
    csp_can_init(CSP_CAN_MASKED, &conf);

    /* I2C */
    csp_i2c_init(MY_ADDRESS, 0, 400);

    /* UART KISS */
    usart_set_callback(&my_usart3_rx);
    csp_kiss_init(&csp_if_kiss1, &csp_kiss1_driver, usart3_putc, usart3_insert, kiss1_name);

#if( MY_ADDRESS == 1 )
    csp_route_set(CSP_DEFAULT_ROUTE, &csp_if_can, 2);
//    csp_route_set(3, &csp_if_i2c, 2);
#elif( MY_ADDRESS == 2 )
    csp_route_set(1, &csp_if_can, 1);
    csp_route_set(CSP_DEFAULT_ROUTE, &csp_if_i2c, 3);
#elif( MY_ADDRESS == 3 )
    csp_route_set(4, &csp_if_kiss1, 4);
    csp_route_set(CSP_DEFAULT_ROUTE, &csp_if_i2c, 2);
#elif( MY_ADDRESS == 4 )
    csp_route_set(CSP_DEFAULT_ROUTE, &csp_if_kiss1, 3);
#endif
//    csp_route_set(1, &csp_if_can, CSP_NODE_MAC);
//#else
//    csp_route_set(2, &csp_if_can, CSP_NODE_MAC);
//#endif
//    csp_route_set(3, &csp_if_i2c, CSP_NODE_MAC);
//    csp_route_set(4, &csp_if_i2c, CSP_NODE_MAC);

    /* CAN */
//    csp_route_set(5, &csp_if_can, CSP_NODE_MAC);
//    csp_route_set(6, &csp_if_can, CSP_NODE_MAC);


//    csp_route_set(7, &csp_if_kiss1, CSP_NODE_MAC);
//    csp_route_set(8, &csp_if_kiss1, CSP_NODE_MAC);

    /* Default route */
//    csp_route_set(CSP_DEFAULT_ROUTE, &csp_if_i2c, 4);

    /* Start router */
    csp_route_start_task(1024, 3);


    printf("Conn table\r\n");
    csp_conn_print_table();

    printf("Route table\r\n");
    csp_route_print_table();

    printf("Interfaces\r\n");
    csp_route_print_interfaces();
}

void vTaskInit(void *pvParameters)
{
    init_csp();

    /** Start USART RX task */
    extern void vTaskUsartRx(void * pvParameters);
    int ret = xTaskCreate(vTaskUsartRx, (const signed char*) "USART", 512, NULL, 3, NULL);

    if (ret != pdTRUE) {
        csp_log_error("Failed to start USART task");
    }
//
    /** Start Console */
    command_init();
    console_init();
    csp_console_init();
    console_set_hostname(CONFIG_HOSTNAME);

    ret = xTaskCreate(debug_console, (const signed char *) "CONSOLE", 512, NULL, 2, NULL);
    if (ret != pdTRUE) {
        csp_log_error("Failed to start CONSOLE task");
    }

    /** End of init */
    vTaskDelete(NULL);
}

