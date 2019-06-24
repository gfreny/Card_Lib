//#include "24cxx.h" 
//#include "delay.h" 	

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

#define DS1_CMD_PORT CARD_CMD_PORT
#define DS1_CMD_PIN CARD_CMD_PIN
#define DS1_CLK_PORT CARD_CLK_PORT
#define DS1_CLK_PIN CARD_CLK_PIN
#define DS1_IO_PORT CARD_IO_PORT
#define DS1_IO_PIN CARD_IO_PIN
#define DS1_35V_PORT CARD_VSEL1_PORT
#define DS1_35V_PIN CARD_VSEL1_PIN
#define DS1_AUX1_PORT CARD_AUX1_PORT
#define DS1_AUX1_PIN CARD_AUX1_PIN
#define DS1_AUX2_PORT CARD_AUX2_PORT
#define DS1_AUX2_PIN CARD_AUX2_PIN

#define DS1_OFF_PORT CARD_INT_PORT
#define DS1_OFF_PIN CARD_INT_PIN

#define DS1_UARTX CARD_UARTX


#define SC1_CMDVCCH   GPIO_SetBits(DS1_CMD_PORT, DS1_CMD_PIN)
#define SC1_CMDVCCL   GPIO_ResetBits(DS1_CMD_PORT, DS1_CMD_PIN)
#define SLECLK_H	GPIO_SetBits(DS1_CLK_PORT,DS1_CLK_PIN)		  /*时钟高*/
#define SLECLK_L	GPIO_ResetBits(DS1_CLK_PORT,DS1_CLK_PIN)	  /*时钟低*/
#define SLEIO_H		GPIO_SetBits(DS1_IO_PORT,DS1_IO_PIN)		  //数据线高
#define SLEIO_L		GPIO_ResetBits(DS1_IO_PORT,DS1_IO_PIN)	  //数据线低
#define SLEIO_I		GPIO_ReadInputDataBit(DS1_IO_PORT,DS1_IO_PIN)	 //输入数据，返回值为unsigned char
#define SLERST_H	RST_Select(0x10)		   //复位高
#define SLERST_L	RST_Select(CARD_FIRST)		//复位低
#define SC1_35VH 	 GPIO_SetBits(DS1_35V_PORT, DS1_35V_PIN)
#define SC1_35VL 	 GPIO_ResetBits(DS1_35V_PORT, DS1_35V_PIN)


#define SLEPGM_H GPIO_SetBits(DS1_AUX2_PORT, DS1_AUX2_PIN)
#define SLEPGM_L GPIO_ResetBits(DS1_AUX2_PORT, DS1_AUX2_PIN)
#define SLEFUS_H GPIO_SetBits(DS1_AUX1_PORT, DS1_AUX1_PIN)
#define SLEFUS_L GPIO_ResetBits(DS1_AUX1_PORT, DS1_AUX1_PIN)
//#define SLEIO_IN_CTR 0
//#define SLEIO_OUT_CTR 1

extern volatile uint8 	Card_PowerStatus;//0为未上电，1为上电							 

extern uint8 Card_TypeSave[5];

//初始化IIC接口
void AT24CXXGPIO_Ini(void)
{ u16 i;
  //---关电
	SC1_CMDVCCH; //cmd
	SLERST_L;//rst	L
	//---关闭UART
	USART_Cmd(DS1_UARTX, DISABLE);
	IIC_Init();
	Delay_Ms(2);
	SLERST_H;
	SC1_35VH;//结合CMD进入停止模式
	
	
	SLECLK_L;
	SLEIO_L;
	//SLEPGM_L;
	//SLEFUS_L;
	SLERST_L;
	SC1_CMDVCCL; Card_PowerStatus=1;//上电
	for(i=0;i<1000;i++);
	
	SLERST_H;
	for(i=0;i<50;i++);
	SLEIO_H;
	SLECLK_H;
	for(i=0;i<50;i++);
	SLECLK_L;
	for(i=0;i<50;i++);
	SLECLK_H;
	for(i=0;i<50;i++);
	//
//	SLECLK_H;
	//for(i=0;i<50;i++);
	//SLECLK_L;


}
//在AT24CXX指定地址读出一个数据
//ReadAddr:开始读数的地址  
//返回值  :读到的数据
/*u8 AT24CXX_ReadOneByte(u16 ReadAddr)
{				  
	u8 temp=0;	  	    																 
    IIC_Start();  
    if(Card_TypeSave[0]>HC_Card_AT24C16)
	{
		IIC_Send_Byte(0XA0);	   //发送写命令
		IIC_Wait_Ack();
		
		IIC_Send_Byte(ReadAddr>>8);//发送高地址	    
	}   	
  else IIC_Send_Byte(0XA0+((ReadAddr>>7)&0X0E));   //发送器件地址0XA0,写数据  	
	IIC_Wait_Ack(); 
	
	IIC_Send_Byte(ReadAddr&0XFF);   //发送低地址
	IIC_Wait_Ack();	    
	IIC_Start();  	 
  if(Card_TypeSave[0]>HC_Card_AT24C16)	
		IIC_Send_Byte(0XA1);  
  else	//进入接收模式		
		IIC_Send_Byte(0XA1+((ReadAddr>>7)&0X0E));	
	IIC_Wait_Ack();	 
    temp=IIC_Read_Byte(0);		   
    IIC_Stop();//产生一个停止条件	    
	return temp;
}*/
u8 AT24CXX_ReadOneByte(u32 ReadAddr)
{				  
	u8 temp=0;	
  	
    IIC_Start();  
    if(Card_TypeSave[0]>HC_Card_AT24C16)
	{
		temp=((ReadAddr>>15)&0x0e);
		
		IIC_Send_Byte(0XA0+temp);	   //发送写命令
		IIC_Wait_Ack();
		
		IIC_Send_Byte(ReadAddr>>8);//发送高地址	    
	}   	
  else IIC_Send_Byte(0XA0+((ReadAddr>>7)&0X0E));   //发送器件地址0XA0,写数据  	
	IIC_Wait_Ack(); 
	
	IIC_Send_Byte(ReadAddr&0XFF);   //发送低地址
	IIC_Wait_Ack();	    
	IIC_Start();  	 
  if(Card_TypeSave[0]>HC_Card_AT24C16)	
		IIC_Send_Byte(0XA1+temp);  
  else	//进入接收模式		
		IIC_Send_Byte(0XA1+((ReadAddr>>7)&0X0E));	
	IIC_Wait_Ack();	 
    temp=IIC_Read_Byte(0);		   
    IIC_Stop();//产生一个停止条件	    
	return temp;
}
//在AT24CXX指定地址写入一个数据
//WriteAddr  :写入数据的目的地址    
//DataToWrite:要写入的数据
/*void AT24CXX_WriteOneByte(u16 WriteAddr,u8 DataToWrite)
{				   	  	    																 
    IIC_Start(); 
    if(Card_TypeSave[0]>HC_Card_AT24C16) 
	{
		IIC_Send_Byte(0XA0);	    //发送写命令
		IIC_Wait_Ack();
		IIC_Send_Byte(WriteAddr>>8);//发送高地址	  
	} 
  else IIC_Send_Byte(0XA0+((WriteAddr>>7)&0X0E));   //发送器件地址0XA0,写数据	
	IIC_Wait_Ack();	 
  IIC_Send_Byte(WriteAddr&0XFF);   //发送低地址
	IIC_Wait_Ack(); 	 										  		   
	IIC_Send_Byte(DataToWrite);     //发送字节							   
	IIC_Wait_Ack();  		    	   
  IIC_Stop();//产生一个停止条件 
	Delay_Ms(10);	 
}*/
void AT24CXX_WriteOneByte(u32 WriteAddr,u8 DataToWrite)
{		u8 temp=0;		   	  	    																 
    IIC_Start(); 
    if(Card_TypeSave[0]>HC_Card_AT24C16) 
	{
		//IIC_Send_Byte(0XA0);	    //发送写命令
		temp=((WriteAddr>>15)&0x0e);		
		IIC_Send_Byte(0XA0+temp);	   //发送写命令
		
		IIC_Wait_Ack();
		IIC_Send_Byte(WriteAddr>>8);//发送高地址	  
	} 
  else IIC_Send_Byte(0XA0+((WriteAddr>>7)&0X0E));   //发送器件地址0XA0,写数据	
	IIC_Wait_Ack();	 
  IIC_Send_Byte(WriteAddr&0XFF);   //发送低地址
	IIC_Wait_Ack(); 	 										  		   
	IIC_Send_Byte(DataToWrite);     //发送字节							   
	IIC_Wait_Ack();  		    	   
  IIC_Stop();//产生一个停止条件 
	Delay_Ms(10);	 
}
//在AT24CXX里面的指定地址开始写入长度为Len的数据
//该函数用于写入16bit或者32bit的数据.
//WriteAddr  :开始写入的地址  
//DataToWrite:数据数组首地址
//Len        :要写入数据的长度2,4
void AT24CXX_WriteLenByte(u32 WriteAddr,u32 DataToWrite,u8 Len)
{  	
	u8 t;
	for(t=0;t<Len;t++)
	{
		AT24CXX_WriteOneByte(WriteAddr+t,(DataToWrite>>(8*t))&0xff);
	}												    
}

//在AT24CXX里面的指定地址开始读出长度为Len的数据
//该函数用于读出16bit或者32bit的数据.
//ReadAddr   :开始读出的地址 
//返回值     :数据
//Len        :要读出数据的长度2,4
u32 AT24CXX_ReadLenByte(u32 ReadAddr,u16 Len)
{  	
	u8 t;
	u32 temp=0;
	for(t=0;t<Len;t++)
	{
		temp<<=8;
		temp+=AT24CXX_ReadOneByte(ReadAddr+Len-t-1); 	 				   
	}
	return temp;												    
}
//检查AT24CXX是否正常
//这里用了24XX的最后一个地址(255)来存储标志字.
//如果用其他24C系列,这个地址要修改
//返回1:检测失败
//返回0:检测成功
u8 AT24CXX_Check(void)
{
	u8 temp;
	temp=AT24CXX_ReadOneByte(255);//避免每次开机都写AT24CXX			   
	if(temp==0X55)return 0;		   
	else//排除第一次初始化的情况
	{
		AT24CXX_WriteOneByte(255,0X55);
	    temp=AT24CXX_ReadOneByte(255);	  
		if(temp==0X55)return 0;
	}
	return 1;											  
}

//在AT24CXX里面的指定地址开始读出指定个数的数据
//ReadAddr :开始读出的地址 对24c02为0~255
//pBuffer  :数据数组首地址
//NumToRead:要读出数据的个数
void AT24CXX_Read(u32 ReadAddr,u8 *pBuffer,u16 NumToRead)
{
	while(NumToRead)
	{
		*pBuffer++=AT24CXX_ReadOneByte(ReadAddr++);	
		NumToRead--;
	}
}  
//在AT24CXX里面的指定地址开始写入指定个数的数据
//WriteAddr :开始写入的地址 对24c02为0~255
//pBuffer   :数据数组首地址
//NumToWrite:要写入数据的个数
/*void AT24CXX_Write(u16 WriteAddr,u8 *pBuffer,u16 NumToWrite)
{
	while(NumToWrite--)
	{
		AT24CXX_WriteOneByte(WriteAddr,*pBuffer);
		WriteAddr++;
		pBuffer++;
	}
}*/
int16 AT24CXX_Write(u32 WriteAddr,u8 *pBuffer,u16 NumToWrite)
{
	u8 i;
	while(NumToWrite--)
	{
		AT24CXX_WriteOneByte(WriteAddr,*pBuffer);
		i=AT24CXX_ReadOneByte(WriteAddr);
		if(i!=(*pBuffer))
			return -1;
		WriteAddr++;
		pBuffer++;
	}
	return 0;
}








