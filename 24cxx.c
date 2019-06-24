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
#define SLECLK_H	GPIO_SetBits(DS1_CLK_PORT,DS1_CLK_PIN)		  /*ʱ�Ӹ�*/
#define SLECLK_L	GPIO_ResetBits(DS1_CLK_PORT,DS1_CLK_PIN)	  /*ʱ�ӵ�*/
#define SLEIO_H		GPIO_SetBits(DS1_IO_PORT,DS1_IO_PIN)		  //�����߸�
#define SLEIO_L		GPIO_ResetBits(DS1_IO_PORT,DS1_IO_PIN)	  //�����ߵ�
#define SLEIO_I		GPIO_ReadInputDataBit(DS1_IO_PORT,DS1_IO_PIN)	 //�������ݣ�����ֵΪunsigned char
#define SLERST_H	RST_Select(0x10)		   //��λ��
#define SLERST_L	RST_Select(CARD_FIRST)		//��λ��
#define SC1_35VH 	 GPIO_SetBits(DS1_35V_PORT, DS1_35V_PIN)
#define SC1_35VL 	 GPIO_ResetBits(DS1_35V_PORT, DS1_35V_PIN)


#define SLEPGM_H GPIO_SetBits(DS1_AUX2_PORT, DS1_AUX2_PIN)
#define SLEPGM_L GPIO_ResetBits(DS1_AUX2_PORT, DS1_AUX2_PIN)
#define SLEFUS_H GPIO_SetBits(DS1_AUX1_PORT, DS1_AUX1_PIN)
#define SLEFUS_L GPIO_ResetBits(DS1_AUX1_PORT, DS1_AUX1_PIN)
//#define SLEIO_IN_CTR 0
//#define SLEIO_OUT_CTR 1

extern volatile uint8 	Card_PowerStatus;//0Ϊδ�ϵ磬1Ϊ�ϵ�							 

extern uint8 Card_TypeSave[5];

//��ʼ��IIC�ӿ�
void AT24CXXGPIO_Ini(void)
{ u16 i;
  //---�ص�
	SC1_CMDVCCH; //cmd
	SLERST_L;//rst	L
	//---�ر�UART
	USART_Cmd(DS1_UARTX, DISABLE);
	IIC_Init();
	Delay_Ms(2);
	SLERST_H;
	SC1_35VH;//���CMD����ֹͣģʽ
	
	
	SLECLK_L;
	SLEIO_L;
	//SLEPGM_L;
	//SLEFUS_L;
	SLERST_L;
	SC1_CMDVCCL; Card_PowerStatus=1;//�ϵ�
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
//��AT24CXXָ����ַ����һ������
//ReadAddr:��ʼ�����ĵ�ַ  
//����ֵ  :����������
/*u8 AT24CXX_ReadOneByte(u16 ReadAddr)
{				  
	u8 temp=0;	  	    																 
    IIC_Start();  
    if(Card_TypeSave[0]>HC_Card_AT24C16)
	{
		IIC_Send_Byte(0XA0);	   //����д����
		IIC_Wait_Ack();
		
		IIC_Send_Byte(ReadAddr>>8);//���͸ߵ�ַ	    
	}   	
  else IIC_Send_Byte(0XA0+((ReadAddr>>7)&0X0E));   //����������ַ0XA0,д����  	
	IIC_Wait_Ack(); 
	
	IIC_Send_Byte(ReadAddr&0XFF);   //���͵͵�ַ
	IIC_Wait_Ack();	    
	IIC_Start();  	 
  if(Card_TypeSave[0]>HC_Card_AT24C16)	
		IIC_Send_Byte(0XA1);  
  else	//�������ģʽ		
		IIC_Send_Byte(0XA1+((ReadAddr>>7)&0X0E));	
	IIC_Wait_Ack();	 
    temp=IIC_Read_Byte(0);		   
    IIC_Stop();//����һ��ֹͣ����	    
	return temp;
}*/
u8 AT24CXX_ReadOneByte(u32 ReadAddr)
{				  
	u8 temp=0;	
  	
    IIC_Start();  
    if(Card_TypeSave[0]>HC_Card_AT24C16)
	{
		temp=((ReadAddr>>15)&0x0e);
		
		IIC_Send_Byte(0XA0+temp);	   //����д����
		IIC_Wait_Ack();
		
		IIC_Send_Byte(ReadAddr>>8);//���͸ߵ�ַ	    
	}   	
  else IIC_Send_Byte(0XA0+((ReadAddr>>7)&0X0E));   //����������ַ0XA0,д����  	
	IIC_Wait_Ack(); 
	
	IIC_Send_Byte(ReadAddr&0XFF);   //���͵͵�ַ
	IIC_Wait_Ack();	    
	IIC_Start();  	 
  if(Card_TypeSave[0]>HC_Card_AT24C16)	
		IIC_Send_Byte(0XA1+temp);  
  else	//�������ģʽ		
		IIC_Send_Byte(0XA1+((ReadAddr>>7)&0X0E));	
	IIC_Wait_Ack();	 
    temp=IIC_Read_Byte(0);		   
    IIC_Stop();//����һ��ֹͣ����	    
	return temp;
}
//��AT24CXXָ����ַд��һ������
//WriteAddr  :д�����ݵ�Ŀ�ĵ�ַ    
//DataToWrite:Ҫд�������
/*void AT24CXX_WriteOneByte(u16 WriteAddr,u8 DataToWrite)
{				   	  	    																 
    IIC_Start(); 
    if(Card_TypeSave[0]>HC_Card_AT24C16) 
	{
		IIC_Send_Byte(0XA0);	    //����д����
		IIC_Wait_Ack();
		IIC_Send_Byte(WriteAddr>>8);//���͸ߵ�ַ	  
	} 
  else IIC_Send_Byte(0XA0+((WriteAddr>>7)&0X0E));   //����������ַ0XA0,д����	
	IIC_Wait_Ack();	 
  IIC_Send_Byte(WriteAddr&0XFF);   //���͵͵�ַ
	IIC_Wait_Ack(); 	 										  		   
	IIC_Send_Byte(DataToWrite);     //�����ֽ�							   
	IIC_Wait_Ack();  		    	   
  IIC_Stop();//����һ��ֹͣ���� 
	Delay_Ms(10);	 
}*/
void AT24CXX_WriteOneByte(u32 WriteAddr,u8 DataToWrite)
{		u8 temp=0;		   	  	    																 
    IIC_Start(); 
    if(Card_TypeSave[0]>HC_Card_AT24C16) 
	{
		//IIC_Send_Byte(0XA0);	    //����д����
		temp=((WriteAddr>>15)&0x0e);		
		IIC_Send_Byte(0XA0+temp);	   //����д����
		
		IIC_Wait_Ack();
		IIC_Send_Byte(WriteAddr>>8);//���͸ߵ�ַ	  
	} 
  else IIC_Send_Byte(0XA0+((WriteAddr>>7)&0X0E));   //����������ַ0XA0,д����	
	IIC_Wait_Ack();	 
  IIC_Send_Byte(WriteAddr&0XFF);   //���͵͵�ַ
	IIC_Wait_Ack(); 	 										  		   
	IIC_Send_Byte(DataToWrite);     //�����ֽ�							   
	IIC_Wait_Ack();  		    	   
  IIC_Stop();//����һ��ֹͣ���� 
	Delay_Ms(10);	 
}
//��AT24CXX�����ָ����ַ��ʼд�볤��ΪLen������
//�ú�������д��16bit����32bit������.
//WriteAddr  :��ʼд��ĵ�ַ  
//DataToWrite:���������׵�ַ
//Len        :Ҫд�����ݵĳ���2,4
void AT24CXX_WriteLenByte(u32 WriteAddr,u32 DataToWrite,u8 Len)
{  	
	u8 t;
	for(t=0;t<Len;t++)
	{
		AT24CXX_WriteOneByte(WriteAddr+t,(DataToWrite>>(8*t))&0xff);
	}												    
}

//��AT24CXX�����ָ����ַ��ʼ��������ΪLen������
//�ú������ڶ���16bit����32bit������.
//ReadAddr   :��ʼ�����ĵ�ַ 
//����ֵ     :����
//Len        :Ҫ�������ݵĳ���2,4
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
//���AT24CXX�Ƿ�����
//��������24XX�����һ����ַ(255)���洢��־��.
//���������24Cϵ��,�����ַҪ�޸�
//����1:���ʧ��
//����0:���ɹ�
u8 AT24CXX_Check(void)
{
	u8 temp;
	temp=AT24CXX_ReadOneByte(255);//����ÿ�ο�����дAT24CXX			   
	if(temp==0X55)return 0;		   
	else//�ų���һ�γ�ʼ�������
	{
		AT24CXX_WriteOneByte(255,0X55);
	    temp=AT24CXX_ReadOneByte(255);	  
		if(temp==0X55)return 0;
	}
	return 1;											  
}

//��AT24CXX�����ָ����ַ��ʼ����ָ������������
//ReadAddr :��ʼ�����ĵ�ַ ��24c02Ϊ0~255
//pBuffer  :���������׵�ַ
//NumToRead:Ҫ�������ݵĸ���
void AT24CXX_Read(u32 ReadAddr,u8 *pBuffer,u16 NumToRead)
{
	while(NumToRead)
	{
		*pBuffer++=AT24CXX_ReadOneByte(ReadAddr++);	
		NumToRead--;
	}
}  
//��AT24CXX�����ָ����ַ��ʼд��ָ������������
//WriteAddr :��ʼд��ĵ�ַ ��24c02Ϊ0~255
//pBuffer   :���������׵�ַ
//NumToWrite:Ҫд�����ݵĸ���
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








