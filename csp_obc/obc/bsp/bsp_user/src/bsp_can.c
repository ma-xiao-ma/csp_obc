/*
 * bsp_can.c
 *
 *  Created on: 2018年3月29日
 *      Author: Ma
 */
#include "string.h"
#include "stm32f4xx.h"

#include "error.h"
#include <csp/csp.h>

#include "bsp_can.h"

#define STD_ID_MASK ((uint32_t)0x7FF << 18)
#define EXT_ID_MASK ((uint32_t)0x3FFFF)
/**
 * CAN初始化
 *
 * Fpclk1的时钟在初始化的时候设置为42M,如果设置CAN1_Mode_Init(CAN_SJW_1tq, CAN_BS2_6tq, CAN_BS1_7tq, 6, CAN_Mode_LoopBack);
 * 则波特率为:42M/((6+7+1)*6)=500Kbps
 *
 * @param id 接收ID设置
 * @param mask 接收ID掩码设置
 * @param tsjw 重新同步跳跃时间单元.  范围:CAN_SJW_1tq ~ CAN_SJW_4tq
 * @param tbs2 时间段2的时间单元.   范围:CAN_BS2_1tq ~ CAN_BS2_8tq
 * @param tbs1 时间段1的时间单元.   范围:CAN_BS1_1tq ~ CAN_BS1_16tq
 * @param brp  波特率分频器.       范围:1 ~ 1024; tq = (brp) * tpclk1
 * @param mode CAN_Mode_Normal, 普通模式; CAN_Mode_LoopBack, 回环模式;
 * @return
 */
u8 CAN1_Mode_Init(u32 id, u32 mask, u8 tsjw, u8 tbs2, u8 tbs1, u16 brp, u8 mode)
{

    GPIO_InitTypeDef       GPIO_InitStructure = {0};
    CAN_InitTypeDef        CAN_InitStructure = {0};
    CAN_FilterInitTypeDef  CAN_FilterInitStructure = {0};
#if CAN1_RX0_INT_ENABLE
    NVIC_InitTypeDef  NVIC_InitStructure = {0};
#endif
    //使能相关时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //使能PORTA时钟

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE); //使能CAN1时钟

    //初始化GPIO
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);              //初始化PA11,PA12

    //引脚复用映射配置
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1); //GPIOA11复用为CAN1
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1); //GPIOA12复用为CAN1

    //CAN单元设置
    CAN_InitStructure.CAN_TTCM = DISABLE;   //非时间触发通信模式
    CAN_InitStructure.CAN_ABOM = DISABLE;   //软件自动离线管理
    CAN_InitStructure.CAN_AWUM = DISABLE;   //睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
    CAN_InitStructure.CAN_NART = ENABLE;    //禁止报文自动传送
    CAN_InitStructure.CAN_RFLM = DISABLE;   //报文不锁定,新的覆盖旧的
    CAN_InitStructure.CAN_TXFP = DISABLE;   //优先级由报文标识符决定
    CAN_InitStructure.CAN_Mode = mode;      //模式设置
    CAN_InitStructure.CAN_SJW = tsjw;       //重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
    CAN_InitStructure.CAN_BS1 = tbs1;       //Tbs1范围CAN_BS1_1tq ~ CAN_BS1_16tq
    CAN_InitStructure.CAN_BS2 = tbs2;       //Tbs2范围CAN_BS2_1tq ~ CAN_BS2_8tq
    CAN_InitStructure.CAN_Prescaler = brp;  //分频系数(Fdiv)为brp+1
    CAN_Init(CAN1, &CAN_InitStructure);     // 初始化CAN1

    //配置过滤器
    CAN_FilterInitStructure.CAN_FilterNumber = 0;     //过滤器0
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit; //32位
    CAN_FilterInitStructure.CAN_FilterIdHigh = (uint16_t)(id >> 16);
    CAN_FilterInitStructure.CAN_FilterIdLow = (uint16_t)id;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = (uint16_t)(mask >> 16);;//32位MASK
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = (uint16_t)mask;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;//过滤器0关联到FIFO0
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE; //激活过滤器0
    CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化

#if CAN1_RX0_INT_ENABLE

    CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);  //FIFO0消息挂号中断允许.

    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;     // 主优先级为1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif
    return 0;
}


int can_init(uint32_t id, uint32_t mask, struct csp_can_config *conf)
{
    CAN1_Mode_Init(id, mask, CAN_SJW_1tq, CAN_BS2_6tq, CAN_BS1_7tq, 6, CAN_Mode_Normal);

    return E_NO_ERR;
}

#if CAN1_RX0_INT_ENABLE //使能RX0中断
//中断服务函数
void CAN1_RX0_IRQHandler(void)
{
    static CSP_BASE_TYPE task_woken;
    CanRxMsg RxMessage;
    can_frame_t frame;

    CAN_Receive(CAN1, 0, &RxMessage);

    if( RxMessage.IDE == CAN_Id_Standard )
        frame.id = RxMessage.StdId;
    else
        frame.id = RxMessage.StdId << 18 | RxMessage.ExtId;

    frame.dlc = RxMessage.DLC;

    memcpy( frame.data, RxMessage.Data, RxMessage.DLC );

    task_woken = pdFALSE;

    if( csp_can_rx_frame(&frame, &task_woken) != CSP_ERR_NONE )
    {
        csp_log_error("CAN RX FULL!!");
    }

    for(int i=0;i<8;i++)
        printf("rxbuf[%d]: %d.\r\n", i, RxMessage.Data[i]);

    portYIELD_FROM_ISR(task_woken);
}
#endif

/**
 * csp can链路层发送接口函数
 *
 * @param id can identifier
 * @param data 待发送数据指针
 * @param dlc 待发送数据长度
 *
 * @return 返回E_NO_ERR为发送成功，否则为发送失败
 */
int can_send(can_id_t id, uint8_t * data, uint8_t dlc)
{
    u8 mbox;
    u16 i = 0;
    CanTxMsg TxMessage;

    TxMessage.StdId = (id & STD_ID_MASK) >> 18;  // 标准标识符
    TxMessage.ExtId = id & EXT_ID_MASK;     // 设置扩展标示符（29位）
    TxMessage.IDE = CAN_Id_Extended;  // 使用扩展标识符
    TxMessage.RTR = CAN_RTR_Data;     // 消息类型为数据帧，一帧8位
    TxMessage.DLC = dlc;        // 发送两帧信息

    for(i=0; i<dlc; i++)
        TxMessage.Data[i] = data[i]; // 第一帧信息

    mbox = CAN_Transmit( CAN1, &TxMessage );
    i=0;

    while( (CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_Failed) &&
            (i < 0xFFF) )
        i++; //等待发送结束

    if(i >= 0xFFF)
        return E_TIMEOUT;

    return E_NO_ERR;
}
/**
 * can发送一组数据( 固定格式:ID为0X12,标准帧,数据帧 )
 *
 * @param msg 数据指针,最大为8个字节.
 * @param len 数据长度(最大为8)
 * @return 0, 成功; 其他, 失败;
 */
u8 CAN1_Send_Msg(u8* msg,u8 len)
{
  u8 mbox;
  u16 i = 0;
  CanTxMsg TxMessage;
  TxMessage.StdId = 0x12;     // 标准标识符为0
  TxMessage.ExtId = 0x12;     // 设置扩展标示符（29位）
  TxMessage.IDE = 0;          // 使用扩展标识符
  TxMessage.RTR = 0;          // 消息类型为数据帧，一帧8位
  TxMessage.DLC = len;        // 发送两帧信息
  for(i=0;i<len;i++)
  TxMessage.Data[i] = msg[i]; // 第一帧信息
  mbox= CAN_Transmit( CAN1, &TxMessage );
  i=0;
  while( (CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_Failed) && (i<0XFFF) )i++; //等待发送结束
  if(i >= 0XFFF)return 1;
  return 0;
}

/**
 * can口接收数据查询
 *
 * @param buf 数据缓存区;
 * @return 0, 无数据被收到; 其他, 接收的数据长度;
 */
u8 CAN1_Receive_Msg(u8 *buf)
{
    u32 i;
    CanRxMsg RxMessage;

    if( CAN_MessagePending(CAN1, CAN_FIFO0) == 0)    //没有接收到数据,直接退出
        return 0;

    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);       //读取数据

    for(i=0; i < RxMessage.DLC; i++)
        buf[i]=RxMessage.Data[i];

    return RxMessage.DLC;
}

