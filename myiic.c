#include "stm32f10x.h"
#include "platform_config.h"
/**************************************/
typedef unsigned char  uint8;                   /* no sign  8 bit                         */
typedef signed   char  int8;                    /* sign  8 bit                            */
typedef signed   char  uchar;                    /* sign  8 bit                            */
typedef unsigned char  u8;                   /* no sign  8 bit                         */
typedef unsigned short uint16;                  /* no sign  16 bit                        */
typedef unsigned short u16;                  /* no sign  16 bit                        */
typedef unsigned short uint;                  /* no sign  16 bit                        */
typedef signed   short int16;                   /* sign  16 bit                           */
typedef unsigned int   uint32;                  /* no sign  32 bit                        */
typedef unsigned int   u32;                  /* no sign  32 bit                        */
typedef signed   int   int32;                   /* sign  32 bit                           */
typedef float          fp32;                    /* Single precision floating point(32bit) */
typedef double         fp64;                    /* Double-precision floating-point (64bit)*/
typedef unsigned char  byte;                     /* no sign  8 bit                        */
typedef unsigned   short word;                   /* no sign  16 bit                       */
								  
//////////////////////////////////////////////////////////////////////////////////
//IO方向设置
#define SDA_IN()  {CARD_IO_PORT->CRL&=0XFFFFF0FF;CARD_IO_PORT->CRL|=8<<8;}
#define SDA_OUT() {CARD_IO_PORT->CRL&=0XFFFFF0FF;CARD_IO_PORT->CRL|=6<<8;}

//IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr))
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO口地址映射
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 
//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入

//IO操作函数	 
#define IIC_SCL    PAout(4) //SCL
#define IIC_SDA    PAout(2) //SDA	 
#define READ_SDA   PAin(2)  //输入SDA  
void I2C_delay(void)
{ 
   u8 i=20; //这里可以优化速度 ，经测试最低到5还能写入
   while(i) 
   { 
     i--; 
   } 
}
//初始化IIC
void IIC_Init(void)
{					     
 //	RCC->APB2ENR|=1<<3;//先使能外设IO PORTB时钟 							 
	CARD_IO_PORT->CRL&= 0XFFFFF0FF;//PB1/11 推挽输出
	CARD_IO_PORT->CRL|= 0X00000600;
	CARD_CLK_PORT->CRL&=0XFFF0FFFF;//PB1/11 推挽输出
	CARD_CLK_PORT->CRL|=0X00020000;
	
	CARD_IO_PORT->ODR|=(CARD_IO_PIN|CARD_CLK_PIN);
  //输出高
}
//产生IIC起始信号
void IIC_Start(void)
{
	IIC_SCL=0;
	SDA_OUT();     //sda线输出
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	I2C_delay();
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	I2C_delay();
	I2C_delay();
	IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
	I2C_delay();
	I2C_delay();
}	  
//产生IIC停止信号
void IIC_Stop(void)
{
  SDA_OUT();//sda线输出
	IIC_SCL=0;
	I2C_delay();
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
	I2C_delay();
	IIC_SCL=1; 
	I2C_delay();
	IIC_SDA=1;//发送I2C总线结束信号
	I2C_delay();						   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA设置为输入  
	IIC_SDA=1;I2C_delay();  
	IIC_SCL=1;I2C_delay();	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{break;
			//不做错误处理，实际应用时很多卡无法检测到ACK
		}
	}
	IIC_SCL=0;//时钟输出0 
	I2C_delay();
	return 0;  
} 
//产生ACK应答
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	I2C_delay();
	IIC_SCL=1;
	I2C_delay();
	IIC_SCL=0;
}
//不产生ACK应答		    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	I2C_delay();
	IIC_SCL=1;
	I2C_delay();
	IIC_SCL=0;
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;  
    IIC_SCL=0;//拉低时钟开始数据传输 
	  SDA_OUT(); 	
    
    for(t=0;t<8;t++)
    {   I2C_delay();           
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		I2C_delay();   //对TEA5767这三个延时都是必须的
		IIC_SCL=1;
		I2C_delay(); 
		IIC_SCL=0;	
    }	 
		Delayus(2);
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	IIC_SCL=0;
	SDA_IN();//SDA设置为输入
	IIC_SDA=1;
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        I2C_delay();
		IIC_SCL=1;
		I2C_delay();
        receive<<=1;
        if(READ_SDA)receive++;   
		I2C_delay(); 
    }					 
    if (!ack)
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
    return receive;
}



























