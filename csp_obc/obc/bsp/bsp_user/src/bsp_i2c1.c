////////////////////////////////////////////////////////////////////////////////
//	功能： 软件I2C初始化、读写控制头文件
//
//	版本：V1.0
//  迭代：
//												南京理工大学微纳卫星中心
//												   2015.11.08
////////////////////////////////////////////////////////////////////////////////
#include "bsp_i2c1.h"

//读写SCL
#define I2C1_SCL_1()  GPIO_PORT_I2C1->BSRRL = I2C1_SCL_PIN		// SCL = 1
#define I2C1_SCL_0()  GPIO_PORT_I2C1->BSRRH = I2C1_SCL_PIN		// SCL = 0

//读写SDA
#define I2C1_SDA_1()  GPIO_PORT_I2C1->BSRRL = I2C1_SDA_PIN		// SDA = 1
#define I2C1_SDA_0()  GPIO_PORT_I2C1->BSRRH = I2C1_SDA_PIN		// SDA = 0

//读SDA状态
#define I2C1_SDA_READ()  ((GPIO_PORT_I2C1->IDR & I2C1_SDA_PIN) != 0)
//读SCL状态
#define I2C1_SCL_READ()  ((GPIO_PORT_I2C1->IDR & I2C1_SCL_PIN) != 0)

////////////////////////////////////////////////////////////////////////////////
//	功能说明:I2C总线初始化
//	形    参:
//	返 回 值: 无
////////////////////////////////////////////////////////////////////////////////
void bsp_InitI2C1(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_I2C1_PORT, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;

	GPIO_InitStructure.GPIO_Pin = I2C1_SCL_PIN | I2C1_SDA_PIN;
	GPIO_Init(GPIO_PORT_I2C1, &GPIO_InitStructure);

	i2c1_Stop();
}

////////////////////////////////////////////////////////////////////////////////
//	功能说明:I2C总线延迟
//	形    参:
//	返 回 值: 无
////////////////////////////////////////////////////////////////////////////////
static void i2c1_Delay(void)
{
	uint8_t i;

	for (i = 0; i < 30; i++)
	{
		__nop();__nop();
	}
}

////////////////////////////////////////////////////////////////////////////////
//	功能说明:I2C发起总线启动信号
//	形    参:
//	返 回 值: 无
////////////////////////////////////////////////////////////////////////////////
void i2c1_Start(void)
{
	I2C1_SDA_1();
	I2C1_SCL_1();
	i2c1_Delay();
	I2C1_SDA_0();
	i2c1_Delay();
	I2C1_SCL_0();
	i2c1_Delay();
}

////////////////////////////////////////////////////////////////////////////////
//	功能说明:I2C发起总线停止信号
//	形    参:
//	返 回 值: 无
////////////////////////////////////////////////////////////////////////////////
void i2c1_Stop(void)
{
	I2C1_SDA_0();
	I2C1_SCL_1();
	i2c1_Delay();
	I2C1_SDA_1();
}

////////////////////////////////////////////////////////////////////////////////
//	功能说明:CPU向I2C总线设备发送8bit数据
//	形    参:
//	返 回 值: 无
////////////////////////////////////////////////////////////////////////////////
void i2c1_SendByte(uint8_t _ucByte)
{
	uint8_t i;
	
	I2C1_SCL_0();
	

	for (i = 0; i < 8; i++)
	{
		if (_ucByte & 0x80)
		{
			I2C1_SDA_1();
		}
		else
		{
			I2C1_SDA_0();
		}
		i2c1_Delay();
		I2C1_SCL_1();
		i2c1_Delay();
		I2C1_SCL_0();
		if (i == 7)
		{
			 I2C1_SDA_1();
		}
		_ucByte <<= 1;
		i2c1_Delay();
	}
}

////////////////////////////////////////////////////////////////////////////////
//	功能说明:CPU从I2C总线设备读取8bit数据
//	形    参:
//	返 回 值: 读取的数据
////////////////////////////////////////////////////////////////////////////////
uint8_t i2c1_ReadByte(void)
{
	uint8_t i;
	uint8_t value;

	value = 0;
	for (i = 0; i < 8; i++)
	{
		value <<= 1;
		I2C1_SCL_1();
		i2c1_Delay();
		if (I2C1_SDA_READ())
		{
			value++;
		}
		I2C1_SCL_0();
		i2c1_Delay();
	}
	return value;
}

////////////////////////////////////////////////////////////////////////////////
//	功能说明:CPU产生一个时钟,并读取器件的ACK应答信号
//	形    参:
//	返 回 值: 0:正确应答;1:无器件响应
////////////////////////////////////////////////////////////////////////////////
uint8_t i2c1_WaitAck(void)
{
	uint8_t re;

	I2C1_SDA_1();
	i2c1_Delay();
	I2C1_SCL_1();
	i2c1_Delay();
	if (I2C1_SDA_READ())
	{
		re = 1;
	}
	else
	{
		re = 0;
	}
	I2C1_SCL_0();
	i2c1_Delay();
	return re;
}

////////////////////////////////////////////////////////////////////////////////
//	功能说明:CPU产生一个ACK应答信号
//	形    参:
//	返 回 值: 0:正确应答;1:无器件响应
////////////////////////////////////////////////////////////////////////////////
void i2c1_Ack(void)
{
	I2C1_SDA_0();
	i2c1_Delay();
	I2C1_SCL_1();
	i2c1_Delay();
	I2C1_SCL_0();
	i2c1_Delay();
	I2C1_SDA_1();
}

////////////////////////////////////////////////////////////////////////////////
//	功能说明:CPU产生一个NACK应答信号
//	形    参:
//	返 回 值:
////////////////////////////////////////////////////////////////////////////////
void i2c1_NAck(void)
{
	I2C1_SDA_1();
	i2c1_Delay();
	I2C1_SCL_1();
	i2c1_Delay();
	I2C1_SCL_0();
	i2c1_Delay();
}

////////////////////////////////////////////////////////////////////////////////
//	功能说明:检查I2C总线设备,CPU发送设备地址,读取设备应答判断该设备是否存在
//	形    参:_Address:设备的I2C总线地址
//	返 回 值:0:存在设备;1:未检测到设备
////////////////////////////////////////////////////////////////////////////////
uint8_t i2c1_CheckDevice(uint8_t _Address)
{
	uint8_t ucAck;

	if (I2C1_SDA_READ() && I2C1_SCL_READ())
	{
		i2c1_Start();

		i2c1_SendByte(_Address | I2C1_WR);
		ucAck = i2c1_WaitAck();

		i2c1_Stop();

		return ucAck;
	}
	return 1;
}