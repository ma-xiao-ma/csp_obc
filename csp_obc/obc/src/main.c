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


#include <csp/interfaces/csp_if_i2c.h>
#include <csp/interfaces/csp_if_can.h>
#include <csp/interfaces/csp_if_kiss.h>

/** Example defines */
#define MY_ADDRESS  2           // Address of local CSP node
#define MY_PORT     10          // Port to send test traffic to

#if( MY_ADDRESS == 2 )
#define ANOTHER_OBC_ADDR  1
#define CONFIG_HOSTNAME   "OBC-2"
#else
#define ANOTHER_OBC_ADDR  2
#define CONFIG_HOSTNAME   "OBC-1"
#endif

/***************开始任务********************/
//任务优先级
#define START_TASK_PRIO     1
//任务堆栈大小
#define START_STK_SIZE      256
//任务句柄
TaskHandle_t StartTask_Handler;
//任务函数
void start_task(void *pvParameters);
/***************看门狗任务********************/
//任务优先级
#define SUPERVISOR_TASK_PRIO     (tskIDLE_PRIORITY + 5)
//任务堆栈大小
#define SUPERVISOR_STK_SIZE      (configMINIMAL_STACK_SIZE)
//任务句柄
TaskHandle_t SupervisorTask_Handler;

/***************调试台任务********************/
//任务优先级
#define CONSOLE_TASK_PRIO     (tskIDLE_PRIORITY + 3)
//任务堆栈大小
#define CONSOLE_STK_SIZE      (configMINIMAL_STACK_SIZE * 8)
//任务句柄
TaskHandle_t ConsoleTask_Handler;

/****************电源信息采集控制任务********************/
//任务优先级
#define COLLECT_TASK_PRIO     (tskIDLE_PRIORITY + 1)
//任务堆栈大小
#define COLLECT_STK_SIZE      (configMINIMAL_STACK_SIZE * 4)
//任务句柄
TaskHandle_t CollectTask_Handler;


/****************上行ISIS通信板解包任务******************/
//任务优先级
#define ISIS_READ_TASK_PRIO     (tskIDLE_PRIORITY + 3)
//任务堆栈大小
#define ISIS_READ_STK_SIZE      (configMINIMAL_STACK_SIZE * 4)
//任务句柄
TaskHandle_t ISISReadTask_Handler;

/***************遥测下行和保存任务*******************/
//任务优先级
#define DOWN_SAVE_TASK_PRIO     (tskIDLE_PRIORITY + 2)
//任务堆栈大小
#define DOWN_SAVE_STK_SIZE      (configMINIMAL_STACK_SIZE * 4)
//任务句柄
TaskHandle_t DownSaveTask_Handler;

/***************展电池阵任务********************/
//任务优先级
#define OPEN_PANEL_TASK_PRIO     (tskIDLE_PRIORITY + 3)
//任务堆栈大小
#define OPEN_PANEL_STK_SIZE      (configMINIMAL_STACK_SIZE * 2)
//任务句柄
TaskHandle_t OpenPanelTask_Handler;

/***************展天线任务************************/
//任务优先级
#define OPEN_ANTENNA_TASK_PRIO     (tskIDLE_PRIORITY + 4)
//任务堆栈大小
#define OPEN_ANTENNA_STK_SIZE      (configMINIMAL_STACK_SIZE * 2)
//任务句柄
TaskHandle_t OpenAntennaTask_Handler;




void vWatchDogTask1( void *pvParameters __attribute__((unused)))
{
	for(;;)
	{
		vTaskDelay(100);
		bsp_WatchDogToggle();
	}
//	xTaskCreate( vWatchDogTask1, "Watchdog", configMINIMAL_STACK_SIZE,
//			( void * ) NULL, tskIDLE_PRIORITY + 3, ( TaskHandle_t * ) NULL );
}


//int main(void)
//{
//    /* 创建开始任务 */
//    xTaskCreate((TaskFunction_t )start_task,            //任务函数
//                (const char*    )"start_task",          //任务名
//                (uint16_t       )START_STK_SIZE,        //任务堆栈大小
//                (void*          )NULL,                  //传递给任务的参数
//                (UBaseType_t    )START_TASK_PRIO,       //任务优先级
//                (TaskHandle_t*  )&StartTask_Handler);   //任务句柄
//
//    vTaskStartScheduler();  //开启任务调度器
//
//    while(1){
//        reset();
//    }
//
//    return 0;
//}
//
////开始任务任务函数
//void start_task(void *pvParameters)
//{
////    taskENTER_CRITICAL();           //进入临界区
//
//    task_initz();
//    //创建监视任务
////    xTaskCreate((TaskFunction_t )supervisor_task,
////                (const char*    )"IWDG",
////                (uint16_t       )SUPERVISOR_STK_SIZE,
////                (void*          )NULL,
////                (UBaseType_t    )SUPERVISOR_TASK_PRIO,
////                (TaskHandle_t*  )&SupervisorTask_Handler);
//
//    //创建调试台任务
//    console_set_hostname("obc");
//
//    xTaskCreate((TaskFunction_t )debug_console,
//                (const char*    )"Console",
//                (uint16_t       )CONSOLE_STK_SIZE,
//                (void*          )NULL,
//                (UBaseType_t    )CONSOLE_TASK_PRIO,
//                (TaskHandle_t*  )&ConsoleTask_Handler);
//
////    //创建分系统遥测采集任务
////    xTaskCreate((TaskFunction_t )hk_collect_task,
////                (const char*    )"Collect",
////                (uint16_t       )COLLECT_STK_SIZE,
////                (void*          )NULL,
////                (UBaseType_t    )COLLECT_TASK_PRIO,
////                (TaskHandle_t*  )&CollectTask_Handler);
//
//#ifndef USE_SERIAL_PORT_DOWNLINK_INTERFACE
//    //创建上行ISIS通信板解包任务
//    xTaskCreate((TaskFunction_t )vu_isis_uplink_task,
//                (const char*    )"Uplink",
//                (uint16_t       )ISIS_READ_STK_SIZE,
//                (void*          )NULL,
//                (UBaseType_t    )ISIS_READ_TASK_PRIO,
//                (TaskHandle_t*  )&ISISReadTask_Handler);
//#endif
//
////    //创建上行解理工通信板解包任务
////    xTaskCreate((TaskFunction_t )vu_jlg_uplink_task,
////                (const char*    )"JLG",
////                (uint16_t       )ISIS_READ_STK_SIZE,
////                (void*          )NULL,
////                (UBaseType_t    )ISIS_READ_TASK_PRIO,
////                (TaskHandle_t*  )&ISISReadTask_Handler);
//
////    //遥测下行和保存任务
////    xTaskCreate((TaskFunction_t )down_save_task,
////                (const char*    )"down",
////                (uint16_t       )DOWN_SAVE_STK_SIZE,
////                (void*          )NULL,
////                (UBaseType_t    )DOWN_SAVE_TASK_PRIO,
////                (TaskHandle_t*  )&DownSaveTask_Handler);
//
////    //展电池阵任务
////    xTaskCreate((TaskFunction_t )OpenPanel_Task,
////                (const char*    )"Batt",
////                (uint16_t       )OPEN_PANEL_STK_SIZE,
////                (void*          )NULL,
////                (UBaseType_t    )OPEN_PANEL_TASK_PRIO,
////                (TaskHandle_t*  )&OpenPanelTask_Handler);
////
////    //展天线任务
////    xTaskCreate((TaskFunction_t )OpenAntenna_Task,
////                (const char*    )"ants",
////                (uint16_t       )OPEN_ANTENNA_STK_SIZE,
////                (void*          )NULL,
////                (UBaseType_t    )OPEN_ANTENNA_TASK_PRIO,
////                (TaskHandle_t*  )&OpenAntennaTask_Handler);
//
//
//
//    vTaskDelete(StartTask_Handler); //删除开始任务
//
////    taskEXIT_CRITICAL();            //退出临界区
//}

CSP_DEFINE_TASK(task_server) {

    /* Create socket without any socket options */
    csp_socket_t *sock = csp_socket(CSP_SO_NONE);

    /* Bind all ports to socket */
    csp_bind(sock, CSP_ANY);

    /* Create 10 connections backlog queue */
    csp_listen(sock, 10);

    /* Pointer to current connection and packet */
    csp_conn_t *conn;
    csp_packet_t *packet;

    /* Process incoming connections */
    while (1) {

        /* Wait for connection, 10000 ms timeout */
        if ((conn = csp_accept(sock, 10000)) == NULL)
            continue;

        /* Read packets. Timout is 100 ms */
        while ((packet = csp_read(conn, 100)) != NULL)
        {
            switch (csp_conn_dport(conn))
            {
                case MY_PORT:
                    /* Process packet here */

//                    printf("Packet received on MY_PORT: %s\r\n", (char *) packet->data);

                    log_info( "Server: Packet received on MY_PORT: %s\r\n", (char *) packet->data );
                    csp_buffer_free(packet);
                    break;

                default:
                    /* Let the service handler reply pings, buffer use, etc. */
                    csp_service_handler(conn, packet);
                    break;
            }
        }

        /* Close current connection, and handle next */
        csp_close(conn);
    }
}

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

    /* I2C */
    csp_i2c_init(MY_ADDRESS, 0, 400);
#if( MY_ADDRESS == 2 )
    csp_route_set(1, &csp_if_i2c, CSP_NODE_MAC);
#else
    csp_route_set(2, &csp_if_i2c, CSP_NODE_MAC);
#endif
    csp_route_set(3, &csp_if_i2c, CSP_NODE_MAC);
    csp_route_set(4, &csp_if_i2c, CSP_NODE_MAC);

    /* CAN */
    struct csp_can_config conf = {0};
    csp_can_init(CSP_CAN_MASKED, &conf);
    csp_route_set(5, &csp_if_can, CSP_NODE_MAC);
    csp_route_set(6, &csp_if_can, CSP_NODE_MAC);

    /* UART KISS */
    usart_set_callback(&my_usart3_rx);
    csp_kiss_init(&csp_if_kiss1, &csp_kiss1_driver, usart3_putc, usart3_insert, kiss1_name);
    csp_route_set(7, &csp_if_kiss1, CSP_NODE_MAC);
    csp_route_set(8, &csp_if_kiss1, CSP_NODE_MAC);


    /* Default route */
    csp_route_set(CSP_DEFAULT_ROUTE, &csp_if_i2c, 4);

    /* Start router */
    csp_route_start_task(1024, 3);


    printf("Conn table\r\n");
    csp_conn_print_table();

    printf("Route table\r\n");
    csp_route_print_table();

    printf("Interfaces\r\n");
    csp_route_print_interfaces();

//    /* Server */
//    printf("Starting Server task\r\n");
//    csp_thread_handle_t handle_server;
//    csp_thread_create(task_server, "SERVER", 1024, NULL, 1, &handle_server);
//
//    /* Client */
//    printf("Starting Client task\r\n");
//    csp_thread_handle_t handle_client;
//    csp_thread_create(task_client, "CLIENT", 1024, NULL, 1, &handle_client);
}

void vTaskInit(void *pvParameters)
{
    init_csp();

    /** Start USART RX task */
    extern void vTaskUsartRx(void * pvParameters);
    int ret = xTaskCreate(vTaskUsartRx, (const signed char*) "USART", 1024, NULL, 3, NULL);

    if (ret != pdTRUE) {
        csp_log_error("Failed to start USART task");
    }
//
    /** Start Console */
    command_init();
    console_init();
    console_set_hostname(CONFIG_HOSTNAME);

    ret = xTaskCreate(debug_console, (const signed char *) "CONSOLE", 1024, NULL, 2, NULL);
    if (ret != pdTRUE) {
        csp_log_error("Failed to start CONSOLE task");
    }

    /** End of init */
    vTaskDelete(NULL);
}

int main(void) {

//    /* 使用usart3当作调试串口 */
//    usart_stdio_id = 2;
//    usart_init(2, 115200);

    tty.init();

    extern void vTaskInit(void *pvParameters);
    xTaskCreate(vTaskInit, (const signed char *) "INIT", 1024 * 2, NULL, 3, NULL);

    /* Server */
    printf("Starting Server task\r\n");
    csp_thread_handle_t handle_server;

    int ret = csp_thread_create(task_server, "SERVER", 512, NULL, 1, &handle_server);

    if (ret != CSP_ERR_NONE) {
        csp_log_error("Failed to start USART task");
    }
    /* Client */
    printf("Starting Client task\r\n");
    csp_thread_handle_t handle_client;

    ret = csp_thread_create(task_client, "CLIENT", 512, NULL, 1, &handle_client);
    if (ret != CSP_ERR_NONE) {
        csp_log_error("Failed to start USART task");
    }
//    extern void vTaskServer(void * pvParameters);
//    xTaskCreate(vTaskServer, (const signed char *) "SRV", 1024*4, NULL, 2, NULL);
//
//    void hook_main(void);
//    hook_main();

    /* Timer uses LFCLOCK = F_OSC/2 */
    vTaskStartScheduler();

    /* Should never reach here */
    while(1);

}

