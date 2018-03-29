/*
 * usart.h
 *
 *  Created on: 2017年11月13日
 *      Author: Ma Wenli
 */

#ifndef CONTRL_USART_H_
#define CONTRL_USART_H_
//#include <stdint.h>
//
///**
// * Initialise UART
// *
// * @param handle usart[0,1,2,3]
// * @param fcpu CPU speed in Hz
// * @param usart_baud Baud rate in bps
// */
//void usart_init(int handle, uint32_t usart_baud);
//
///**
// * In order to catch incoming chars use the callback.
// * Only one callback per interface.
// * @param handle usart[0,1,2,3]
// * @param callback function pointer
// */
//typedef void (*usart_callback_t) (uint8_t * buf, int len, void * pxTaskWoken);
//void usart_set_callback(int handle, usart_callback_t callback);
//
///**
// * Insert a character to the RX buffer of a usart
// * @param handle usart[0,1,2,3]
// * @param c Character to insert
// */
//void usart_insert(int handle, char c, void * pxTaskWoken);
//
///**
// * Polling putchar
// *
// * @param handle usart[0,1,2,3]
// * @param c Character to transmit
// */
//void usart_putc(int handle, char c);
//
///**
// * Send char buffer on UART
// *
// * @param handle usart[0,1,2,3]
// * @param buf Pointer to data
// * @param len Length of data
// */
//void usart_putstr(int handle, char *buf, int len);
//
///**
// * Buffered getchar
// *
// * @param handle usart[0,1,2,3]
// * @return Character received
// */
//char usart_getc(int handle);
//
///**
// * Buffered getchar (not blocking)
// *
// * @param handle usart[0,1,2,3]
// * @return Character received
// */
//char usart_getc_nblock(int handle);
//
///**
// * Report number of messages waiting on the RX queue of a handle
// * @param handle usart[0,1,2,3]
// * @return Number of char's in rx buffer
// */
//int usart_messages_waiting(int handle);
//
///**
// * STDIO:
// * These functions can be used by systems which have multiple usarts
// * to get/put chars from the current stdio console.
// */
//extern int usart_stdio_id;
//
//static inline void usart_stdio_putchar(char c) {
//    if (usart_stdio_id < 0)
//        return;
//    usart_putc(usart_stdio_id, c);
//}
//
//static inline char usart_stdio_getchar(void) {
//    if (usart_stdio_id < 0)
//        return 0;
//    return usart_getc(usart_stdio_id);
//}
//
//static inline char usart_stdio_getchar_nblock(void) {
//    if (usart_stdio_id < 0)
//        return 0;
//    return usart_getc_nblock(usart_stdio_id);
//}
//
//static inline char usart_stdio_msgwaiting(void) {
//    if (usart_stdio_id < 0)
//        return 0;
//    return usart_messages_waiting(usart_stdio_id);
//}

#define TTY_BAUDRATE          921600                    /*波特率 ------------*/
#define TTY_TXBUF_SIZE        4096                       /*发送缓冲区长度 -----*/
#define TTY_RXBUF_SIZE        4096                       /*接收缓冲区长度 -----*/
#define TTY_DMA_TX_LEN        256                        /*DMA 发送缓冲区 ----*/
#define TTY_DMA_RX_LEN        256                        /*DMA 接收缓冲区 ----*/

#define TTY_USE_DMA           1                         /*启用DMA -----------*/


/* Exported Structs ---------------------------------------------------------*/

typedef struct
{
    void (*init)(void);                                     /*初始化 --------*/
    unsigned int (*write)(void *buf, unsigned int len);     /*数据写 --------*/
    unsigned int (*read) (void *buf, unsigned int len);     /*读数据 --------*/
    void (*puts)(const char *str);                          /*输入一个字符串 */
    void (*clr)(void);                                      /*清除接收缓冲区 */
    unsigned int (*buflen)(void);                           /*接收缓冲区的长度*/
    void (*printf)(const char *format, ...);                /*格式化打印 ----*/
}tty_t;


/* Exported variables ------------------------------------------------------- */
extern const tty_t tty;

char usart_getc(void);

typedef void (*usart_callback_t) (uint8_t *buf, int len, void *pxTaskWoken);
void usart_set_callback(usart_callback_t callback);

/**
 * Insert a character to the RX buffer of a usart
 * @param handle usart[0,1,2,3]
 * @param c Character to insert
 */
void usart_insert(char c, void *pxTaskWoken);

#endif /* CONTRL_USART_H_ */
