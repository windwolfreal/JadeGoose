#include "stdint.h"
#include "i2c.h"
#include "stm32f1xx_hal.h"

#if USE_HARDWARE_I2C == 0

//IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000) + 0x2000000 + ((addr & 0xFFFFF) << 5) + (bitnum << 2))
#define MEM_ADDR(addr) *((volatile unsigned long *)(addr))
#define BIT_ADDR(addr, bitnum) MEM_ADDR(BITBAND(addr, bitnum))

//IO口地址映射
#define GPIOA_ODR_Addr (GPIOA_BASE + 12) //0x4001080C
#define GPIOB_ODR_Addr (GPIOB_BASE + 12) //0x40010C0C
#define GPIOC_ODR_Addr (GPIOC_BASE + 12) //0x4001100C
#define GPIOD_ODR_Addr (GPIOD_BASE + 12) //0x4001140C
#define GPIOE_ODR_Addr (GPIOE_BASE + 12) //0x4001180C
#define GPIOF_ODR_Addr (GPIOF_BASE + 12) //0x40011A0C
#define GPIOG_ODR_Addr (GPIOG_BASE + 12) //0x40011E0C

#define GPIOA_IDR_Addr (GPIOA_BASE + 8) //0x40010808
#define GPIOB_IDR_Addr (GPIOB_BASE + 8) //0x40010C08
#define GPIOC_IDR_Addr (GPIOC_BASE + 8) //0x40011008
#define GPIOD_IDR_Addr (GPIOD_BASE + 8) //0x40011408
#define GPIOE_IDR_Addr (GPIOE_BASE + 8) //0x40011808
#define GPIOF_IDR_Addr (GPIOF_BASE + 8) //0x40011A08
#define GPIOG_IDR_Addr (GPIOG_BASE + 8) //0x40011E08

#define PBout(n) BIT_ADDR(GPIOB_ODR_Addr, n) //输出
#define PBin(n) BIT_ADDR(GPIOB_IDR_Addr, n)  //输入

////驱动接口，GPIO模拟IIC
//SCL-->PB6
//SDA-->PB7
#define SDA_IN() \
    // {                             \
    //     GPIOB->CRL &= 0X0FFFFFFF; \
    //     GPIOB->CRL |= 0x80000000; \
    // }
#define SDA_OUT() \
    // {                             \
    //     GPIOB->CRL &= 0X0FFFFFFF; \
    //     GPIOB->CRL |= 0x30000000; \
    // }

//IO操作函数
#define IIC_SCL PBout(6) //SCL
#define IIC_SDA PBout(7) //SDA
#define READ_SDA PBin(7) //输入SDA

#define IIC_LongDelay() BSP_DelayUs(20)
#define IIC_ShortDelay() BSP_DelayUs(10)

static void IIC_Start(void);
static void IIC_Stop(void);
static uint8_t IIC_Wait_Ack(void);
static void IIC_Ack(void);
static void IIC_NAck(void);
static void IIC_Send_Byte(uint8_t txd);
static uint8_t IIC_Read_Byte(unsigned char ack);

/**************************实现函数********************************************
*函数原型:		void GPIO_Init(void)
*功　　能:		初始化I2C对应的接口引脚。
*******************************************************************************/
static void GPIO_Init(void)
{
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

    LL_GPIO_InitTypeDef GPIO_InitStruct;

    /**I2C1 GPIO Configuration  
  PB6   ------> I2C1_SCL
  PB7   ------> I2C1_SDA 
  */
    LL_GPIO_SetOutputPin(GPIOB, IMU_I2C_SCL_Pin | IMU_I2C_SDA_Pin);
    GPIO_InitStruct.Pin = IMU_I2C_SCL_Pin | IMU_I2C_SDA_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    IIC_Send_Byte(0xff);
    IIC_NAck();
}

/**************************实现函数********************************************
*函数原型:		void IIC_Start(void)
*功　　能:		产生IIC起始信号
*******************************************************************************/
static void IIC_Start(void)
{
    SDA_OUT(); //sda线输出
    IIC_SDA = 1;
    IIC_SCL = 1;
    IIC_LongDelay();
    IIC_SDA = 0; //START:when CLK is high,DATA change form high to low
    IIC_LongDelay();
    IIC_SCL = 0; //钳住I2C总线，准备发送或接收数据
}

/**************************实现函数********************************************
*函数原型:		void IIC_Stop(void)
*功　　能:	    //产生IIC停止信号
*******************************************************************************/
static void IIC_Stop(void)
{
    SDA_OUT(); //sda线输出
    IIC_SCL = 0;
    IIC_SDA = 0; //STOP:when CLK is high DATA change form low to high
    IIC_LongDelay();
    IIC_SCL = 1;
    IIC_SDA = 1; //发送I2C总线结束信号
    IIC_LongDelay();
}

/**************************实现函数********************************************
*函数原型:		uint8_t IIC_Wait_Ack(void)
*功　　能:	    等待应答信号到来 
//返回值：1，接收应答失败
//        0，接收应答成功
*******************************************************************************/
static uint8_t IIC_Wait_Ack(void)
{
    uint8_t ucErrTime = 0;
    SDA_IN(); //SDA设置为输入
    IIC_SDA = 1;
    IIC_ShortDelay();
    IIC_SCL = 1;
    IIC_ShortDelay();
    while (READ_SDA)
    {
        ucErrTime++;
        if (ucErrTime > 50)
        {
            IIC_Stop();
            return 1;
        }
        IIC_ShortDelay();
    }
    IIC_SCL = 0; //时钟输出0
    return 0;
}

/**************************实现函数********************************************
*函数原型:		void IIC_Ack(void)
*功　　能:	    产生ACK应答
*******************************************************************************/
static void IIC_Ack(void)
{
    IIC_SCL = 0;
    SDA_OUT();
    IIC_SDA = 0;
    IIC_ShortDelay();
    IIC_SCL = 1;
    IIC_ShortDelay();
    IIC_SCL = 0;
}

/**************************实现函数********************************************
*函数原型:		void IIC_NAck(void)
*功　　能:	    产生NACK应答
*******************************************************************************/
static void IIC_NAck(void)
{
    IIC_SCL = 0;
    SDA_OUT();
    IIC_SDA = 1;
    IIC_ShortDelay();
    IIC_SCL = 1;
    IIC_ShortDelay();
    IIC_SCL = 0;
}

/**************************实现函数********************************************
*函数原型:		void IIC_Send_Byte(uint8_t txd)
*功　　能:	    IIC发送一个字节
*******************************************************************************/
static void IIC_Send_Byte(uint8_t txd)
{
    uint8_t t;
    SDA_OUT();
    IIC_SCL = 0; //拉低时钟开始数据传输
    for (t = 0; t < 8; t++)
    {
        IIC_SDA = (txd & 0x80) >> 7;
        txd <<= 1;
        IIC_ShortDelay();
        IIC_SCL = 1;
        IIC_ShortDelay();
        IIC_SCL = 0;
        IIC_ShortDelay();
    }
}

/**************************实现函数********************************************
*函数原型:		uint8_t IIC_Read_Byte(unsigned char ack)
*功　　能:	    //读1个字节，ack=1时，发送ACK，ack=0，发送nACK 
*******************************************************************************/
static uint8_t IIC_Read_Byte(unsigned char ack)
{
    unsigned char i, receive = 0;
    SDA_IN(); //SDA设置为输入
    for (i = 0; i < 8; i++)
    {
        IIC_SCL = 0;
        IIC_SDA = 1;
        IIC_ShortDelay();
        IIC_SCL = 1;
        receive <<= 1;
        if (READ_SDA)
            receive++;
        IIC_ShortDelay();
    }
    if (ack)
        IIC_Ack(); //发送ACK
    else
        IIC_NAck(); //发送nACK
    return receive;
}

/**************************实现函数********************************************
*函数原型:		unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
*功　　能:	    读取指定设备 指定寄存器的一个值
输入	I2C_Addr  目标设备地址
		addr	   寄存器地址
返回   读出来的值
*******************************************************************************/
static unsigned char I2C_ReadOneByte(unsigned char I2C_Addr, unsigned char addr)
{
    unsigned char res = 0;

    IIC_Start();

    IIC_Send_Byte(I2C_Addr); //发送写命令
    res++;
    IIC_Wait_Ack();
    IIC_Send_Byte(addr);
    res++; //发送地址
    IIC_Wait_Ack();
    //IIC_Stop();//产生一个停止条件
    IIC_Start();
    IIC_Send_Byte(I2C_Addr + 1);
    res++; //进入接收模式
    IIC_Wait_Ack();
    res = IIC_Read_Byte(0);
    IIC_Stop(); //产生一个停止条件

    return res;
}

void I2C_Init()
{
    GPIO_Init();
}

uint8_t I2C_Send(uint8_t deviceAddr, uint8_t *sendBuf, uint16_t sendLength)
{
    uint16_t count = 0;
    IIC_Start();
    IIC_Send_Byte(deviceAddr); //发送写命令
    IIC_Wait_Ack();
    for (count = 0; count < sendLength; count++)
    {
        IIC_Send_Byte(sendBuf[count]);
        IIC_Wait_Ack();
    }
    IIC_Stop(); //产生一个停止条件

    return 1; //status == 0;
}

uint8_t I2C_SendOne(uint8_t deviceAddr, uint8_t data)
{
    uint16_t count = 0;
    IIC_Start();
    IIC_Send_Byte(deviceAddr); //发送写命令
    IIC_Wait_Ack();
    IIC_Send_Byte(data);
    IIC_Wait_Ack();
    IIC_Stop(); //产生一个停止条件

    return 1; //status == 0;
}
uint8_t I2C_SendTwo(uint8_t deviceAddr, uint8_t data1, uint8_t data2)
{
    uint16_t count = 0;
    IIC_Start();
    IIC_Send_Byte(deviceAddr); //发送写命令
    IIC_Wait_Ack();
    IIC_Send_Byte(data1);
    IIC_Wait_Ack();
    IIC_Send_Byte(data2);
    IIC_Wait_Ack();
    IIC_Stop(); //产生一个停止条件

    return 1; //status == 0;
}

uint8_t I2C_Send2(uint8_t deviceAddr, uint8_t *sendBuf1, uint16_t sendLength1, uint8_t *sendBuf2, uint16_t sendLength2)
{
    uint16_t count = 0;
    IIC_Start();
    IIC_Send_Byte(deviceAddr); //发送写命令
    IIC_Wait_Ack();
    for (count = 0; count < sendLength1; count++)
    {
        IIC_Send_Byte(sendBuf1[count]);
        IIC_Wait_Ack();
    }
    for (count = 0; count < sendLength2; count++)
    {
        IIC_Send_Byte(sendBuf2[count]);
        IIC_Wait_Ack();
    }
    IIC_Stop(); //产生一个停止条件

    return 1; //status == 0;
}

uint16_t I2C_SendAndReceive(uint8_t deviceAddr, uint8_t *sendBuf, uint16_t sendLength, uint8_t *receiveBuf, uint16_t receiveLength)
{
    uint16_t count = 0;
    uint8_t temp;
    IIC_Start();
    IIC_Send_Byte(deviceAddr); //发送写命令
    IIC_Wait_Ack();
    for (count = 0; count < sendLength; count++)
    {
        IIC_Send_Byte(sendBuf[count]);
        IIC_Wait_Ack();
    }
    IIC_Start();
    IIC_Send_Byte(deviceAddr + 1); //进入接收模式
    IIC_Wait_Ack();
    count = 0;
    for (count = 0; count < receiveLength; count++)
    {

        if (count != (receiveLength - 1))
            temp = IIC_Read_Byte(1); //带ACK的读数据
        else
            temp = IIC_Read_Byte(0); //最后一个字节NACK

        receiveBuf[count] = temp;
    }
    IIC_Stop(); //产生一个停止条件
    return count;
}

#endif