/*
 * usart.c
 *
 *  Created on: 2017年11月13日
 *      Author: Ma Wenli
 */
//#include "string.h"
//#include "stm32f4xx.h"
//
//#include "FreeRTOS.h"
//#include "semphr.h"
//#include "task.h"
//#include "usart.h"
//
//int usart_stdio_id = -1;
//
//#define USART_COUNT             3
//#define USART_RX_QUEUE_LENGTH   1024
//#define USART_RX_BUF_LENGTH     0x1000
//#define USART_TX_BUF_LENGTH     0x1000
//
//static void usart_tx_DSR(int handle);
//static void usart_rx_DSR(int handle);
//
//static xSemaphoreHandle usart_pdc_irq = NULL;
//
//static struct usart_struct {
//    USART_TypeDef * base;
//    uint32_t  periph_clk;
//    int irq;
//
//    xQueueHandle usart_rxqueue;
//    usart_callback_t rx_callback;
//
//    DMA_Stream_TypeDef * rx_dma_stream;
//    unsigned int rx_dma_all_flag;
//    int rx_dma_irq;
//    volatile int rx_dma_pending;
//    DMA_Stream_TypeDef * tx_dma_stream;
//    unsigned int tx_dma_all_flag;
//    unsigned int tx_dma_it_tcif;
//    int tx_dma_irq;
//
//    GPIO_TypeDef * rx_port;
//    uint32_t rx_port_clk;
//    uint16_t rx_pin;
//    uint8_t rx_af;
//    uint8_t rx_source;
//
//    GPIO_TypeDef * tx_port;
//    uint32_t tx_port_clk;
//    uint16_t tx_pin;
//    uint8_t tx_af;
//    uint8_t tx_source;
//
//
//
//    char rx_buf1[USART_RX_BUF_LENGTH];
//    char rx_buf2[USART_RX_BUF_LENGTH];
//    char * rx_curbuf;
//    char * rx_prevbuf;
//    int rx_prevbuf_len;
//
//
//
//
//    char tx_buf1[USART_TX_BUF_LENGTH];
//    char tx_buf2[USART_TX_BUF_LENGTH];
//    char * tx_curbuf;
//    int tx_nextbuf_len;
//
//    xSemaphoreHandle tx_sem;
//
//} usart[USART_COUNT] = {
//    {
//        .base = USART1,
//        .periph_clk = RCC_APB2Periph_USART1,
//
//        .rx_irq = USART1_IRQn,
//
//
//        .rx_port = GPIOB,
//        .rx_port_clk = RCC_AHB1Periph_GPIOB,
//        .rx_pin = GPIO_Pin_7,
//        .rx_af = GPIO_AF_USART1,
//        .rx_source = GPIO_PinSource7,
//
//        .rx_dma_stream = DMA2_Stream2,
//        .rx_dma_all_flag = DMA_FLAG_FEIF2 | DMA_FLAG_DMEIF2 | DMA_FLAG_TEIF2 | DMA_FLAG_HTIF2 | DMA_FLAG_TCIF2,
//
//        .tx_isr = usart_tx_DSR,
//        .tx_irq = DMA2_Stream7_IRQn,
//        .tx_port = GPIOB,
//        .tx_port_clk = RCC_AHB1Periph_GPIOB,
//        .tx_pin = GPIO_Pin_6,
//        .tx_af = GPIO_AF_USART1,
//        .tx_source = GPIO_PinSource6,
//
//        .tx_dma_stream = DMA2_Stream7,
//        .tx_dma_all_flag = DMA_FLAG_FEIF7 | DMA_FLAG_DMEIF7 | DMA_FLAG_TEIF7 | DMA_FLAG_HTIF7 | DMA_FLAG_TCIF7,
//        .tx_dma_it_tcif = DMA_IT_TCIF7,
//
//        .tx_nextbuf_len = 0,
//    },
//    {
//        .base = USART2,
//        .periph_clk = RCC_APB1Periph_USART2,
//
//        .rx_isr = usart_rx_DSR,
//        .rx_irq = USART2_IRQn,
//        .rx_port = GPIOA,
//        .rx_port_clk = RCC_AHB1Periph_GPIOA,
//        .rx_pin = GPIO_Pin_3,
//        .rx_af = GPIO_AF_USART2,
//        .rx_source = GPIO_PinSource3,
//
//        .rx_dma_stream = DMA1_Stream5,
//        .rx_dma_all_flag = DMA_FLAG_FEIF5 | DMA_FLAG_DMEIF5 | DMA_FLAG_TEIF5 | DMA_FLAG_HTIF5 | DMA_FLAG_TCIF5,
//
//        .tx_isr = usart_tx_DSR,
//        .tx_irq = DMA1_Stream6_IRQn,
//        .tx_port = GPIOA,
//        .tx_port_clk = RCC_AHB1Periph_GPIOA,
//        .tx_pin = GPIO_Pin_2,
//        .tx_af = GPIO_AF_USART2,
//        .tx_source = GPIO_PinSource2,
//
//        .tx_dma_stream = DMA1_Stream6,
//        .tx_dma_all_flag = DMA_FLAG_FEIF6 | DMA_FLAG_DMEIF6 | DMA_FLAG_TEIF6 | DMA_FLAG_HTIF6 | DMA_FLAG_TCIF6,
//        .tx_dma_it_tcif = DMA_IT_TCIF6,
//
//        .tx_nextbuf_len = 0,
//    },
//    {
//        .base = USART3,
//        .periph_clk = RCC_APB1Periph_USART3,
//
//        .rx_isr = usart_rx_DSR,
//        .rx_irq = USART3_IRQn,
//        .rx_port = GPIOB,
//        .rx_port_clk = RCC_AHB1Periph_GPIOB,
//        .rx_pin = GPIO_Pin_11,
//        .rx_af = GPIO_AF_USART3,
//        .rx_source = GPIO_PinSource11,
//
//        .rx_dma_stream = DMA1_Stream1,
//        .rx_dma_all_flag = DMA_FLAG_FEIF1 | DMA_FLAG_DMEIF1 | DMA_FLAG_TEIF1 | DMA_FLAG_HTIF1 | DMA_FLAG_TCIF1,
//
//        .tx_isr = usart_tx_DSR,
//        .tx_irq = DMA1_Stream3_IRQn,
//        .tx_port = GPIOB,
//        .tx_port_clk = RCC_AHB1Periph_GPIOB,
//        .tx_pin = GPIO_Pin_10,
//        .tx_af = GPIO_AF_USART3,
//        .tx_source = GPIO_PinSource10,
//
//        .tx_dma_stream = DMA1_Stream3,
//        .tx_dma_all_flag = DMA_FLAG_FEIF3 | DMA_FLAG_DMEIF3 | DMA_FLAG_TEIF3 | DMA_FLAG_HTIF3 | DMA_FLAG_TCIF3,
//        .tx_dma_it_tcif = DMA_IT_TCIF3,
//
//        .tx_nextbuf_len = 0,
//    }
//};
//
//
//static void usart_try_tx_from_isr(int handle, portBASE_TYPE * pxTaskWoken)
//{
//
//    /* 若tx_nextbuf_len不为0说明当前发送缓冲区已使用，待发送 */
//    if (usart[handle].tx_nextbuf_len == 0)
//        return;
//
//    /* 若DMA发送未完成，则返回 */
//    if (DMA_GetCmdStatus(usart[handle].tx_dma_stream) != DISABLE)
//        return;
//
//    /* 使能发送当前缓冲区已写入的数据 */
//    usart[handle].tx_dma_stream->M0AR = (uintptr_t) usart[handle].tx_curbuf;
//    usart[handle].tx_dma_stream->NDTR = usart[handle].tx_nextbuf_len;
//    usart[handle].tx_dma_stream->CR |= (uint32_t)DMA_SxCR_EN;
//
//    /* 交换缓冲区，tx_nextbuf_len置为0 */
//    usart[handle].tx_curbuf = (usart[handle].tx_curbuf == usart[handle].tx_buf1) ? usart[handle].tx_buf2 : usart[handle].tx_buf1;
//    usart[handle].tx_nextbuf_len = 0;
//
//    /* 给出串口发送锁 */
//    xSemaphoreGiveFromISR(usart[handle].tx_sem, pxTaskWoken);
//}
//
///**
// * 串口DMA发送完成中断处理程序
// *
// * @param handle 设备号
// */
//static void usart_tx_DSR(int handle)
//{
//
//    portBASE_TYPE xTaskWoken = pdFALSE;
//
//    /* 如果发送完成，则启动下次传输 */
//    if (DMA_GetITStatus(usart[handle].tx_dma_stream, usart[handle].tx_dma_it_tcif) != RESET)
//    {
//        DMA_ClearFlag(usart[handle].tx_dma_stream, usart[handle].tx_dma_all_flag);
//        usart_try_tx_from_isr(handle, &xTaskWoken);
//    }
//
////    portYIELD_FROM_ISR(xTaskWoken);
//}
//
///**
// * 串口DMA接收完成完成中断处理程序
// *
// * @param handle 设备号
// */
//static void usart_rx_DSR(int handle)
//{
//    portBASE_TYPE xTaskWoken = pdFALSE;
//
//    /* 如果是串口空闲中断 */
//    if (USART_GetITStatus(usart[handle].base, USART_IT_IDLE) != RESET)
//    {
//        /* 清除USART_IT_IDLE */
//        uint16_t CleanUpRegist = usart[handle].base->SR;
//        CleanUpRegist = usart[handle].base->DR;
//
//        /* 中断同步信号量被创建 */
//        if (usart_pdc_irq != NULL)
//        {
//            /* 获取DMA NDTR寄存器值 */
//            uint16_t rx_ndtr = DMA_GetCurrDataCounter(usart[handle].rx_dma_stream);
//
//            /* 如果缓冲区接收到数据，且上一DSR已经处理完 */
//            if (rx_ndtr != USART_RX_BUF_LENGTH && usart[handle].rx_dma_pending == 0)
//            {
//                /* 禁止DMA接收 */
//                DMA_Cmd(usart[handle].rx_dma_stream, DISABLE);
//
//                /* 计算接收长度 */
//                usart[handle].rx_prevbuf_len = USART_RX_BUF_LENGTH - rx_ndtr;
//
//                /* 交换接收缓冲区 */
//                usart[handle].rx_prevbuf = usart[handle].rx_curbuf;
//                usart[handle].rx_curbuf = (usart[handle].rx_curbuf == usart[handle].rx_buf1) ? usart[handle].rx_buf2 : usart[handle].rx_buf1;
//
//                /* 等待DMA可以设置 */
//                while (DMA_GetCmdStatus(usart[handle].rx_dma_stream));
//
//                /* 清除接收DMA标志 */
//                DMA_ClearFlag(usart[handle].rx_dma_stream, usart[handle].rx_dma_all_flag);
//
//                /* 编程并启动DMA接收 */
//                usart[handle].rx_dma_stream->M0AR = (unsigned int) usart[handle].rx_curbuf;
//                usart[handle].rx_dma_stream->NDTR = USART_RX_BUF_LENGTH;
//                usart[handle].rx_dma_stream->CR |= (uint32_t)DMA_SxCR_EN;
//
//                /* 唤醒接收处理任务 */
//                usart[handle].rx_dma_pending = 1;
//                xSemaphoreGiveFromISR(usart_pdc_irq, &xTaskWoken);
//            }
//        }
//    } /* 如果是接收寄存器非空中断 */
//    else if (USART_GetITStatus(usart[handle].base, USART_IT_RXNE) != RESET)
//    {
//        static uint8_t byte;
//
//        /* 清中断标志位 */
//        USART_ClearITPendingBit(usart[handle].base, USART_IT_RXNE);
//
//        /* 从数据寄存器读一个字节的数据 */
//        byte = (uint8_t)usart[handle].base->DR;
//
//        if(usart[handle].rx_callback != NULL) {
//            usart[handle].rx_callback(&byte, 1, &xTaskWoken);
//        } else {
//            xQueueSendToBackFromISR(usart[handle].usart_rxqueue, &byte, &xTaskWoken);
//        }
//
//    }
//
//    portYIELD_FROM_ISR(xTaskWoken);
//}
//
//void vTaskUsartRx(void * pvParameters) {
//
//    int i;
//    int handle;
//
//    /* 创建IRQ同步任务信号量，创建后信号量需要给出后才有效 */
//    if (usart_pdc_irq == NULL)
//        usart_pdc_irq = xSemaphoreCreateBinary();
//
//    /* 开启DMA接收 */
//    for (handle = 0; handle < USART_COUNT; handle++)
//    {
//        usart[handle].rx_dma_stream->M0AR = (unsigned int) usart[handle].rx_curbuf;
//        usart[handle].rx_dma_stream->NDTR = USART_RX_BUF_LENGTH;
//        usart[handle].rx_dma_stream->CR |= (uint32_t)DMA_SxCR_EN;
//    }
//
//    while(1)
//    {
//        /* 若中断同步信号给出 */
//        xSemaphoreTake(usart_pdc_irq, portMAX_DELAY);
//
//        /* 判断是哪个设备产生的处理请求 */
//        for (handle = 0; handle < USART_COUNT; handle++)
//        {
//            if (usart[handle].rx_dma_pending == 0)
//                continue;
//            if (usart[handle].rx_prevbuf_len == 0)
//                continue;
//
//            /* 中断延时处理 */
//            if (usart[handle].rx_callback != NULL) {
//                usart[handle].rx_callback((unsigned char *) usart[handle].rx_prevbuf, usart[handle].rx_prevbuf_len, NULL);
//            } else if (usart[handle].usart_rxqueue != NULL) {
//                for (i = 0; i < usart[handle].rx_prevbuf_len; i++)
//                    xQueueSendToBack(usart[handle].usart_rxqueue, usart[handle].rx_prevbuf + i, 0);
//            }
//
//            portENTER_CRITICAL();
//
//            /* Stop PDC if something happened */ /* 如果第一缓冲区处理完后第二缓冲区已经有数据接收到，则继续处理 */
//            if (USART_RX_BUF_LENGTH - usart[handle].rx_dma_stream->NDTR != 0)
//            {
//                /* 禁止DMA接收 */
//                DMA_Cmd(usart[handle].rx_dma_stream, DISABLE);
//
//                /* 计算接收长度 */
//                usart[handle].rx_prevbuf_len = USART_RX_BUF_LENGTH - usart[handle].rx_dma_stream->NDTR;
//
//                /* 交换接收缓冲区 */
//                usart[handle].rx_prevbuf = usart[handle].rx_curbuf;
//                usart[handle].rx_curbuf = (usart[handle].rx_curbuf == usart[handle].rx_buf1) ? usart[handle].rx_buf2 : usart[handle].rx_buf1;
//
//                /* 等待DMA可以设置 */
//                while (DMA_GetCmdStatus(usart[handle].rx_dma_stream));
//
//                /* 清除接收DMA标志 */
//                DMA_ClearFlag(usart[handle].rx_dma_stream, usart[handle].rx_dma_all_flag);
//
//                /* 编程并启动DMA接收 */
//                usart[handle].rx_dma_stream->M0AR = (unsigned int) usart[handle].rx_curbuf;
//                usart[handle].rx_dma_stream->NDTR = USART_RX_BUF_LENGTH;
//                usart[handle].rx_dma_stream->CR |= (uint32_t)DMA_SxCR_EN;
//
//                /* 唤醒接收处理任务 */
//                usart[handle].rx_dma_pending = 1;
//                xSemaphoreGive(usart_pdc_irq);
//
//            } else {
//
//                usart[handle].rx_dma_pending = 0;
//            }
//
//            portEXIT_CRITICAL();
//        }
//    }
//}
//
///**
// * 初始化特定串口
// *
// * @param handle 串口号
// * @param usart_baud 波特率
// */
//void usart_init(int handle, uint32_t usart_baud)
//{
//    GPIO_InitTypeDef GPIO_InitStructure = {0};
//    USART_InitTypeDef USART_InitStructure = {0};
//    NVIC_InitTypeDef NVIC_InitStructure = {0};
//    DMA_InitTypeDef DMA_InitStructure = {0};
//
//    /* 创建串口接收队列 */
//    if (usart[handle].usart_rxqueue == NULL)
//        usart[handle].usart_rxqueue = xQueueCreate(USART_RX_QUEUE_LENGTH, 1);
//
//    /* 创建串口发送锁 */
//    if (usart[handle].tx_sem == NULL)
//        vSemaphoreCreateBinary(usart[handle].tx_sem);
//
//    /* 接收和发送缓冲区为双缓冲区，赋初值 */
//    usart[handle].tx_curbuf = usart[handle].tx_buf1;
//    usart[handle].rx_curbuf = usart[handle].rx_buf1;
//
//    /* 发送接收引脚GPIO时钟使能 */
//    RCC_AHB1PeriphClockCmd(usart[handle].tx_port_clk | usart[handle].rx_port_clk, ENABLE);
//
//    /* 外设时钟使能 */
//    if (usart[handle].base == USART1 || usart[handle].base == USART6)
//    {
//        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE); //DMA外设时钟使能
//        RCC_APB2PeriphClockCmd(usart[handle].periph_clk, ENABLE); //USART外设时钟使能
//    }
//    else
//    {
//        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE); //DMA外设时钟使能
//        RCC_APB1PeriphClockCmd(usart[handle].periph_clk, ENABLE); //USART外设时钟使能
//    }
//
//    /* GPIO配置 */
//    GPIO_PinAFConfig(usart[handle].tx_port, usart[handle].tx_source, usart[handle].tx_af);/*引脚复用*/
//    GPIO_PinAFConfig(usart[handle].rx_port, usart[handle].rx_source, usart[handle].rx_af);
//
//    /* tx */
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStructure.GPIO_Pin = usart[handle].tx_pin;
//    GPIO_Init(usart[handle].tx_port, &GPIO_InitStructure);
//    /* rx */
//    GPIO_InitStructure.GPIO_Pin = usart[handle].rx_pin;
//    GPIO_Init(usart[handle].rx_port, &GPIO_InitStructure);
//
//    /* 先初始化到默认状态 */
//    USART_DeInit(usart[handle].base);
//    DMA_DeInit(usart[handle].rx_dma_stream);
//    DMA_DeInit(usart[handle].tx_dma_stream);
//
//    /* USART外设配置 */
//    USART_InitStructure.USART_BaudRate = usart_baud;
//    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//    USART_InitStructure.USART_StopBits = USART_StopBits_1;
//    USART_InitStructure.USART_Parity = USART_Parity_No;
//    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//    USART_Init(usart[handle].base, &USART_InitStructure);
//    /* 串口发送接收DMA使能 */
//    USART_DMACmd(usart[handle].base, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);
//
//    /* DMA接收流配置 */
//    DMA_InitStructure.DMA_Channel = DMA_Channel_4;
//    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&usart[handle].base->DR;
//    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)usart[handle].rx_curbuf;
//    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; //外设到内存
//    DMA_InitStructure.DMA_BufferSize = 0;
//    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址不增
//    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //内存地址增
//    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据大小1byte
//    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//内存数据大小1byte
//    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; //循环模式或者正常模式
//    DMA_InitStructure.DMA_Priority = DMA_Priority_High;//优先级高
//    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;//不开fifo
//    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
//    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//    DMA_Init(usart[handle].rx_dma_stream, &DMA_InitStructure);
//
//    /* DMA发送流配置 */
//    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)usart[handle].tx_curbuf; //内存地址
//    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral; //内存到外设
//    DMA_InitStructure.DMA_BufferSize = 0;
//    DMA_Init(usart[handle].tx_dma_stream, &DMA_InitStructure);
//
//    /* 中断配置 */
//    /* NVIC configuration */
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
//
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannel = usart[handle].rx_irq;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);
//    USART_ITConfig(usart[handle].base, USART_IT_IDLE, ENABLE);
//
//    NVIC_InitStructure.NVIC_IRQChannel = usart[handle].tx_irq;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);
//    DMA_ITConfig(usart[handle].tx_dma_stream, DMA_IT_TC, ENABLE);
//
//    /* 使能 USART */
//    USART_Cmd(usart[handle].base, ENABLE);
//}
//
//void usart_set_callback(int handle, usart_callback_t callback) {
//    usart[handle].rx_callback = callback;
//}
//
//void usart_insert(int handle, char c, void * pxTaskWoken) {
//    if (pxTaskWoken == NULL)
//        xQueueSendToBack(usart[handle].usart_rxqueue, &c, 0);
//    else
//        xQueueSendToBackFromISR(usart[handle].usart_rxqueue, &c, pxTaskWoken);
//}
//
///**
// * Polling putchar
// *
// * @param  b c Character to transmit
// */
//void usart_putc(int handle, char c) {
//
//    portBASE_TYPE xTaskWoken = pdFALSE;
//
//    /* Assume max 10 concurrent calls to usart_putc */
//    if (usart[handle].tx_nextbuf_len + 10 > USART_TX_BUF_LENGTH) {
//        xSemaphoreTake(usart[handle].tx_sem, 0);
//        xSemaphoreTake(usart[handle].tx_sem, portMAX_DELAY);
//    }
//
//    portENTER_CRITICAL();
//    usart[handle].tx_curbuf[usart[handle].tx_nextbuf_len++] = c;
//
//    if (usart[handle].tx_dma_stream->NDTR == 0)
//        usart_try_tx_from_isr(handle, &xTaskWoken);
//    portEXIT_CRITICAL();
//}
//
//void usart_putstr(int handle, char *buf, int len) {
//
//    portBASE_TYPE xTaskWoken = pdFALSE;
//
//    /* Assume max 10 concurrent calls to usart_putc */
//    if (usart[handle].tx_nextbuf_len + len + 10 > USART_TX_BUF_LENGTH) {
//        xSemaphoreTake(usart[handle].tx_sem, 0);
//        xSemaphoreTake(usart[handle].tx_sem, portMAX_DELAY);
//    }
//
//    portENTER_CRITICAL();
//    memcpy(&usart[handle].tx_curbuf[usart[handle].tx_nextbuf_len], buf, len);
//    usart[handle].tx_nextbuf_len += len;
//
//    if (usart[handle].tx_dma_stream->NDTR == 0)
//        usart_try_tx_from_isr(handle, &xTaskWoken);
//    portEXIT_CRITICAL();
//}
//
//char usart_getc(int handle) {
//    char buf;
//    xQueueReceive(usart[handle].usart_rxqueue, &buf, portMAX_DELAY);
//    return buf;
//}
//
//int usart_messages_waiting(int handle) {
//    return uxQueueMessagesWaiting(usart[handle].usart_rxqueue);
//}
//
//void DMA1_Stream3_IRQHandler(void)
//{
//    usart_tx_DSR(2);
//}
//
//void USART3_IRQHandler(void)
//{
//    usart_rx_DSR(2);
//}

/* Includes ------------------------------------------------------------------*/
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "semphr.h"

#include "ring_buf.h"
#include "stm32f4xx.h"
#include "obc_mem.h"

#include "usart.h"

static unsigned char rxbuf[TTY_TXBUF_SIZE];         /*接收缓冲区 ------------*/
static unsigned char txbuf[TTY_RXBUF_SIZE];         /*发送缓冲区 ------------*/
static ring_buf_t ringbuf_send, ringbuf_recv;       /*收发缓冲区管理 ---------*/

#if TTY_USE_DMA == 1
    static unsigned char dma_tx_buf[TTY_DMA_TX_LEN];/*DMA发送缓冲区 ---------*/
    static unsigned char dma_rx_buf[TTY_DMA_RX_LEN];/*DMA接收缓冲区 ---------*/
    static xSemaphoreHandle usart_pdc_irq = NULL;
    xQueueHandle usart_rxqueue;
#endif

/*******************************************************************************
 * 函数名称：port_conf
 * 功能描述：打印串口配置(PB10->USART3_TX, PB11->USART3_RX)
 * 输入参数：none
 * 返 回 值：none
 * 作    者：roger.luo
 ******************************************************************************/
static void port_conf(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    /*console串口引脚配置 ----------------------------------------------------*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/*******************************************************************************
 * 函数名称：DMA_Conf
 * 功能描述: 串口DMA配置(DMA1_Channel4_Stream1->USART3_RX,
 *                  DMA1_Channel4_Stream3->USART3_TX)
 * 输入参数：none
 * 返 回 值：none
 * 作    者：roger.luo
 ******************************************************************************/
#if TTY_USE_DMA == 1
static void DMA_Conf(void)
{
    DMA_InitTypeDef DMA_Structure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable DMA clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    DMA_DeInit(DMA1_Stream1);
    DMA_DeInit(DMA1_Stream3);
    while (DMA_GetCmdStatus(DMA1_Stream1) != DISABLE){}
    while (DMA_GetCmdStatus(DMA1_Stream3) != DISABLE){}
    /*配置串口3接收流 */
    DMA_Structure.DMA_Channel = DMA_Channel_4;                    /*DMA1通道4*/
    DMA_Structure.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);
    DMA_Structure.DMA_Memory0BaseAddr = (uint32_t)dma_rx_buf;
    DMA_Structure.DMA_DIR = DMA_DIR_PeripheralToMemory;           /*外设到内存*/
    DMA_Structure.DMA_BufferSize = sizeof(dma_rx_buf);
    DMA_Structure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_Structure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_Structure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_Structure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_Structure.DMA_Mode = DMA_Mode_Circular;                   /*循环模式*/
    DMA_Structure.DMA_Priority = DMA_Priority_Low;
    DMA_Structure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_Structure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_Structure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_Structure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream1, &DMA_Structure);

    /*配置串口3发送流 */
    DMA_Structure.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);
    DMA_Structure.DMA_Memory0BaseAddr = (uint32_t)dma_tx_buf;
    DMA_Structure.DMA_DIR = DMA_DIR_MemoryToPeripheral;            /*内存到外设*/
    DMA_Structure.DMA_BufferSize = sizeof(dma_tx_buf);
    DMA_Structure.DMA_Mode = DMA_Mode_Normal;                      /*正常模式 -*/
    DMA_Init(DMA1_Stream3, &DMA_Structure);

    /* Enable DMA Stream Transfer Complete interrupt */
    DMA_ITConfig(DMA1_Stream1, DMA_IT_TC, ENABLE);
    //DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, ENABLE);
    /* DMA Stream enable */
    DMA_Cmd(DMA1_Stream1, ENABLE);                                 /*使能接收流*/

    /* Enable the DMA Stream IRQ Channel */
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream3_IRQn;
    NVIC_Init(&NVIC_InitStructure);
}
#endif
/*******************************************************************************
 * 函数名称：uart_conf
 * 功能描述：TTY 串口配置
 * 输入参数：none
 * 返 回 值：none
 * 作    者：roger.luo
 ******************************************************************************/
static void uart_conf(void)
{
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    USART_DeInit(USART3);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    USART_InitStructure.USART_BaudRate = TTY_BAUDRATE;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

#if TTY_USE_DMA == 1
    USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);         /*开启DMA请求 --------*/
    USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);       /*打开空闲中断处理DMA接收 -------*/
    USART_ITConfig(USART3, USART_IT_TC,  ENABLE);        /*打开空闲中断处理DMA发送 -------*/

#else
    USART_ITConfig(USART3, USART_IT_TXE, ENABLE);        /*打开收发中断 -------*/
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
#endif
    USART_ITConfig(USART3, USART_IT_ERR, ENABLE);
    USART_Cmd(USART3, ENABLE);
    ring_buf_create(&ringbuf_send, txbuf, sizeof(txbuf));/*初始化环形缓冲区 --*/
    ring_buf_create(&ringbuf_recv, rxbuf, sizeof(rxbuf));
}

/*******************************************************************************
 * 函数名称：init
 * 功能描述：打印驱动初始化
 * 输入参数：none
 * 返 回 值：none
 * 作    者：roger.luo
 ******************************************************************************/
static void init(void)
{
    port_conf();
    uart_conf();
#if TTY_USE_DMA == 1
    DMA_Conf();
#endif

    /* 创建串口接收队列 */
    if (usart_rxqueue == NULL)
        usart_rxqueue = xQueueCreate(1000, 1);
}




/*******************************************************************************
 * 函数名称：send
 * 功能描述：向串口发送缓冲区内写入数据
 * 输入参数：buf       -  缓冲区
 *           len       -  缓冲区长度
 * 返 回 值：实际写入长度(如果此时缓冲区满,则返回len)
 * 作    者：roger.luo
 ******************************************************************************/
static unsigned int send(void *buf, unsigned int len)
{

#if TTY_USE_DMA == 1
    unsigned int ret;
    ret = ring_buf_put(&ringbuf_send, buf, len);
    USART_ITConfig(USART3, USART_IT_TC, ENABLE);
    return ret;
#else
    unsigned int ret;
    ret = ring_buf_put(&ringbuf_send, (unsigned char *)buf, len);
    USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
    return ret;
#endif
}

/*******************************************************************************
 * 函数名称：recv
 * 功能描述：读取tty接收缓冲区的数据
 * 输入参数：buf       -  缓冲区
 *           len       -  缓冲区长度
 * 返 回 值：(实际读取长度)如果接收缓冲区的有效数据大于len则返回len否则返回缓冲
 *            区有效数据的长度
 * 作    者：roger.luo
 ******************************************************************************/
unsigned int recv(void *buf, unsigned int len)
{
    return ring_buf_get(&ringbuf_recv, (unsigned char *)buf, len);
}

#if TTY_USE_DMA == 1
/*******************************************************************************
 * 函数名称：DMA1_Stream1_IRQHandler
 * 功能描述：TTY串口DMA接收完成中断
 * 输入参数：none
 * 返 回 值：none
 ******************************************************************************/
void DMA1_Stream1_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_Stream1, DMA_IT_TCIF1) != RESET)
    {
        ring_buf_put(&ringbuf_recv, dma_rx_buf, sizeof(dma_rx_buf));
        DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TCIF1);
    }
}
/*******************************************************************************
 * 函数名称：DMA1_Stream3_IRQHandler
 * 功能描述：TTY串口DMA发送完成中断
 * 输入参数：none
 * 返 回 值：none
 ******************************************************************************/
/*void DMA1_Stream3_IRQHandler(void)
{
    unsigned int len;
    if (DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF3) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TCIF3);
        if ((len = ring_buf_get(&ringbuf_send, dma_tx_buf, sizeof(dma_tx_buf))))
        {
            DMA_SetCurrDataCounter(DMA1_Stream3, len);
            DMA_Cmd(DMA1_Stream3, ENABLE);
            USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);
        }
        else sending = 0;
    }

}*/
#endif

/*******************************************************************************
 * 函数名称：USART3_IRQHandler
 * 功能描述：串口1收发中断
 * 输入参数：none
 * 返 回 值：none
 ******************************************************************************/
void USART3_IRQHandler(void)
{
#if TTY_USE_DMA == 1
    static portBASE_TYPE xTaskWoken;
    uint16_t len, retry = 0;

    if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET ||
        USART_GetITStatus(USART3, USART_IT_FE) != RESET)
    {
        /*获取DMA缓冲区内的有效数据长度 --------------------------------------*/
        len = sizeof(dma_rx_buf) - DMA_GetCurrDataCounter(DMA1_Stream1);
        DMA_Cmd(DMA1_Stream1, DISABLE);
        ring_buf_put(&ringbuf_recv, dma_rx_buf, len);    /*将数据放入接收缓冲区*/
        while (DMA_GetCmdStatus(DMA1_Stream1) != DISABLE && retry++ < 100){}
        DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1);    /*清除DMA传输完成标志,否则会进入传输完成中断*/
        /*复位DMA当前计数器值 ------------------------------------------------*/
        DMA_SetCurrDataCounter(DMA1_Stream1, sizeof(dma_rx_buf));
        DMA_Cmd(DMA1_Stream1, ENABLE);
        len = USART3->SR;                               /*清除空闲中断标志 -------*/
        len = USART3->DR;
        xTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(usart_pdc_irq, &xTaskWoken);
        portYIELD_FROM_ISR(xTaskWoken);
    }
    if (USART_GetITStatus(USART3, USART_IT_TC) != RESET)
    {
        if ((len = ring_buf_get(&ringbuf_send, dma_tx_buf, sizeof(dma_tx_buf))))
        {
            DMA_Cmd(DMA1_Stream3, DISABLE);
            DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3);
            DMA_SetCurrDataCounter(DMA1_Stream3, len);
            DMA_Cmd(DMA1_Stream3, ENABLE);
        }
        else
        {
            USART_ITConfig(USART3, USART_IT_TC, DISABLE);
        }
    }
#else
    unsigned char data;
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        data = USART_ReceiveData(USART3);
        ring_buf_put(&ringbuf_recv,&data, 1);           /*将数据放入接收缓冲区*/
    }
    if (USART_GetITStatus(USART3, USART_IT_TXE) != RESET)
    {
        if (ring_buf_get(&ringbuf_send, &data, 1))      /*从缓冲区中取出数据---*/
        {
            USART_SendData(USART3, data);
        }
        else
            USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
    }
    if (USART_GetITStatus(USART3, USART_IT_FE) != RESET)
    {
        data = USART_ReceiveData(USART3);

    }
#endif
}


/*******************************************************************************
 * 函数名称：putstr
 * 功能描述：输出一个字符串
 * 输入参数：none
 * 返 回 值：none
 ******************************************************************************/
static void putstr(const char *str)
{
    send((void *)str, strlen(str));
}

/*******************************************************************************
 * 函数名称：clear
 * 功能描述：清除接收缓冲区的数据
 * 输入参数：none
 * 返 回 值：none
 ******************************************************************************/
static void clear(void)
{
    ring_buf_clr(&ringbuf_recv);
}

/*******************************************************************************
 * 函数名称：buflen
 * 功能描述：接收缓冲区数据的有效字节数
 * 输入参数：none
 * 返 回 值：none
 ******************************************************************************/
static unsigned int buflen(void)
{
    return ring_buf_len(&ringbuf_recv);
}

/*******************************************************************************
 * 函数名称：print
 * 功能描述：格式化打印输出
 * 输入参数：none
 * 返 回 值：none
 ******************************************************************************/
static void print(const char *format, ...)
{
    va_list args;
    char buf[256];
    va_start (args, format);
    vsprintf (buf, format, args);
    va_end (args);
    putstr(buf);

}


/*外部接口定义 --------------------------------------------------------------*/
const tty_t tty =
{
    init,
    send,
    recv,
    putstr,
    clear,
    buflen,
    print
};

static usart_callback_t usart_rx_callback = NULL;

void vTaskUsartRx(void * pvParameters) {

    /* 创建IRQ同步任务信号量 */
    if (usart_pdc_irq == NULL)
        usart_pdc_irq = xSemaphoreCreateBinary();
    int len;

    while(1) {

        /* 若中断同步信号给出 */
        if( xSemaphoreTake(usart_pdc_irq, 1000) != pdTRUE )
            continue;

        if( !(len = tty.buflen()) )
        {
            vTaskDelay(10);
            continue;
        }

        uint8_t *buf = (uint8_t *)ObcMemMalloc(len);
        if( buf == NULL )
            continue;

        tty.read(buf, len);

        /* 中断延时处理 */
        if( usart_rx_callback != NULL ) {

            usart_rx_callback(buf, len, NULL);
        }
        else if( usart_rxqueue != NULL ) {

            for (int i = 0; i < len; i++)
                xQueueSendToBack(usart_rxqueue, buf + i, 0);
        }

        ObcMemFree(buf);
    }
}

char usart_getc() {
    char buf;
    xQueueReceive(usart_rxqueue, &buf, portMAX_DELAY);
    return buf;
}

int usart_messages_waiting(int handle) {
    return uxQueueMessagesWaiting(usart_rxqueue);
}

void usart_insert(char c, void * pxTaskWoken) {
    if (pxTaskWoken == NULL)
        xQueueSendToBack(usart_rxqueue, &c, 0);
    else
        xQueueSendToBackFromISR(usart_rxqueue, &c, pxTaskWoken);
}

void usart_set_callback(usart_callback_t callback) {
    usart_rx_callback = callback;
}
