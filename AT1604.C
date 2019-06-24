/************************************************************************ 
* 
*�� �ļ���:	AT1604.c 
* 
*�� �ļ�����:	�߼����ܿ�at88sc1604�����⺯���� 
* 
*�� ������: 	dxs, 2007��3��8�� 
* 
*�� �汾��:	1.0 
* 
*�� �޸ļ�¼: 
* 
�ܽ�˳��
VCC--C1	  C5--GND
RST--C2	  C6--N/A
CLK--C3	  C7--I/O
FUS--C4	  C8--PGM
************************************************************************/ 

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
#define SLECLK_H	DS1_CLK_PORT->BSRR=DS1_CLK_PIN//GPIO_SetBits(DS1_CLK_PORT,DS1_CLK_PIN)		  /*ʱ�Ӹ�*/
#define SLECLK_L	DS1_CLK_PORT->BRR=DS1_CLK_PIN//GPIO_ResetBits(DS1_CLK_PORT,DS1_CLK_PIN)	  /*ʱ�ӵ�*/
#define SLEIO_H		DS1_IO_PORT->BSRR=DS1_IO_PIN//GPIO_SetBits(DS1_IO_PORT,DS1_IO_PIN)		  //�����߸�
#define SLEIO_L		DS1_IO_PORT->BRR=DS1_IO_PIN//GPIO_ResetBits(DS1_IO_PORT,DS1_IO_PIN)	  //�����ߵ�
#define SLEIO_I		(DS1_IO_PORT->IDR&DS1_IO_PIN)//GPIO_ReadInputDataBit(DS1_IO_PORT,DS1_IO_PIN)	 //�������ݣ�����ֵΪunsigned char
#define SLERST_H	RST_Select(0x10)		   //��λ��
#define SLERST_L	RST_Select(CARD_FIRST)		//��λ��
#define SC1_35VH 	 GPIO_SetBits(DS1_35V_PORT, DS1_35V_PIN)
#define SC1_35VL 	 GPIO_ResetBits(DS1_35V_PORT, DS1_35V_PIN)

#define SLEPGM_H DS1_AUX2_PORT->BSRR=DS1_AUX2_PIN//GPIO_SetBits(DS1_AUX2_PORT, DS1_AUX2_PIN)
#define SLEPGM_L DS1_AUX2_PORT->BRR=DS1_AUX2_PIN//GPIO_ResetBits(DS1_AUX2_PORT, DS1_AUX2_PIN)
#define SLEFUS_H DS1_AUX1_PORT->BSRR=DS1_AUX1_PIN//GPIO_SetBits(DS1_AUX1_PORT, DS1_AUX1_PIN)
#define SLEFUS_L DS1_AUX1_PORT->BRR=DS1_AUX1_PIN//GPIO_ResetBits(DS1_AUX1_PORT, DS1_AUX1_PIN)

#define SLEIO_IN_CTR 0
#define SLEIO_OUT_CTR 1
//���ȷֿ�
#define	FZ_BIT_ADDR	0
#define IZ_BIT_ADDR	16
#define SC_BIT_ADDR	80
#define SCAC_BIT_ADDR	96
#define CPZ_BIT_ADDR	104

#define SC1_BIT_ADDR	168
#define	S1AC_BIT_ADDR	184
#define	EZ1_BIT_ADDR	192
#define	E1AC_BIT_ADDR	208
#define AZ1_BIT_ADDR	216

#define SC2_BIT_ADDR	9776
#define	EZ2_BIT_ADDR	9792
#define	E2AC_BIT_ADDR	9808
#define AZ2_BIT_ADDR	9816

#define SC3_BIT_ADDR	11864
#define	EZ3_BIT_ADDR	11880
#define	E3AC_BIT_ADDR	11896
#define AZ3_BIT_ADDR	11904

#define SC4_BIT_ADDR	13952
#define	EZ4_BIT_ADDR	13968
#define	E4AC_BIT_ADDR	13984
#define AZ4_BIT_ADDR	13992

#define MTZ_BIT_ADDR	16040
#define FUS_BIT_ADDR	16288

void	AT1604_Reset(void);
void	AT1604_Plus(void);
void	AT1604_Inc(uint Step);
uchar	AT1604_Writebit(void);
uchar	AT1604_Erasebit(void);
uchar	AT1604_ReadByte(void);
uchar	AT1604_WriteByte(uchar OutData);
void	AT1604_CMPByte(uchar *CompareData,uchar CmpLen);
uchar	AT1604_EraseByte(void);
void	AT1604_Read(uint Addr,uint Len,uchar *DataBuf);


extern volatile uint8 	Card_PowerStatus;//0Ϊδ�ϵ磬1Ϊ�ϵ�
extern void Delayus(uint32 num);//��λΪ0.5us 
//extern volatile uint32 	systicnum ;

void AT1604_IO_ctlpp(void)
{ uint8 tmp;
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;				  	
	GPIO_InitStructure.GPIO_Pin = DS1_IO_PIN;

	{	
		GPIO_InitStructure.GPIO_Mode =GPIO_Mode_Out_PP;// GPIO_Mode_Out_OD; //GPIO_Mode_Out_PP;
		GPIO_Init(DS1_IO_PORT, &GPIO_InitStructure);
		GPIO_ResetBits(DS1_IO_PORT, DS1_IO_PIN);
	}	
	//for(tmp=0;tmp<20;tmp++);
}
void AT1604_IO_ctlod(void)
{ uint8 tmp;
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;				  	
	GPIO_InitStructure.GPIO_Pin = DS1_IO_PIN;

	{	
		GPIO_InitStructure.GPIO_Mode =GPIO_Mode_Out_OD;// GPIO_Mode_Out_OD; //GPIO_Mode_Out_PP;
		GPIO_Init(DS1_IO_PORT, &GPIO_InitStructure);
		GPIO_SetBits(DS1_IO_PORT,DS1_IO_PIN);
	}	
	
}

/*================================================================ 
* 
* �� �� ��:	void	AT1604_Reset(void)
* 
* �Ρ�����:	��
* 
* ��������:	ʹ1604����ַ��λΪ0
* 
* �� �� ֵ:	��
*
================================================================*/ 

void	AT1604_Reset(void)
{
	SC1_CMDVCCL; 
	Card_PowerStatus=1;//�ϵ�
	
	
	//SLEFUS_H;
	SLEPGM_L;
	//AT1604_IO_ctl(SLEIO_IN_CTR);//IO����
	AT1604_IO_ctlod();
	//AT1604_IO_ctl(SLEIO_OUT_CTR);//
	//SLEIO_L;
	Delayus(20);//��ʱ1us

    
  SLERST_H;
  Delayus(20);//��ʱ1us
	SLECLK_H;
	//
	Delayus(6);//��ʱ1us
	SLECLK_L;
	//AT1604_IO_ctl(SLEIO_IN_CTR);//IO����
	Delayus(20);//��ʱ1us
	Delayus(20);//��ʱ1us
	SLERST_L;
    Delayus(20);//��ʱ2us  
	return;
}

/*================================================================ 
* 
* �� �� ��:	void	AT1604_Plus(void)
* 
* �Ρ�����:	��
* 
* ��������:	ʹ1604����ַ��������1
* 
* �� �� ֵ:	��
*
================================================================*/
void	AT1604_Plus(void)
{
	SLECLK_H;
	Delayus(6);
	//_CardSetClock(1);
	SLECLK_L;
	Delayus(6);
	//_CardSetClock(0);
	return;
}

/*================================================================ 
* 
* �� �� ��:	void	AT1604_Inc(uint Step)
* 
* �Ρ�����:	uint Step	��Ҫǰ���ĵ�ַ��
* 
* ��������:	ʹ1604����ַ��������ǰStep
* 
* �� �� ֵ:	��
*
================================================================*/ 
void	AT1604_Inc(uint Step)
{
	while(Step)
	{
		AT1604_Plus();
		Step--;
	}
}

/*================================================================ 
* 
* �� �� ��:	uchar	AT1604_Writebit(void)
* 
* �Ρ�����:	��
* 
* ��������:	1604��λд����
* 
* �� �� ֵ:	0:	�ɹ�		1:	ʧ��	
*
================================================================*/ 



uchar	AT1604_Writebit(void)
{	
	uint32 tmp;
	
	SLEPGM_H;
	Delayus(6);
	AT1604_IO_ctlpp();
	SLEIO_L;//д
	Delayus(6);
	SLECLK_H;
	Delayus(6);
	//AT1604_IO_ctlod();
	//Delayus(6);
	SLEPGM_L;
	Delay_Ms(10);
	 
	AT1604_IO_ctlod();
	SLECLK_L;Delayus(6);
	if(SLEIO_I==0){return 0;}
	else{return 1;}
}
/*================================================================ 
* 
* �� �� ��:	uchar	AT1604_Erasebit(void)
* 
* �Ρ�����:	��
* 
* ��������:	1604��λ��������
* 
* �� �� ֵ:	0:	�ɹ�		1:	ʧ��
*
================================================================*/ 

uchar	AT1604_Erasebit(void)
{	
	
	uint32 tmp;
	
	SLEPGM_H;
	Delayus(6);

	AT1604_IO_ctlpp();
	SLEIO_H;//д
	Delayus(6);
	SLECLK_H;
	Delayus(6);
	
	SLEPGM_L;
	Delayus(6);
	Delay_Ms(10);
	AT1604_IO_ctlod();//����ź���
	SLECLK_L;
	
	
	
	
	 
	
	Delayus(6);
	if(SLEIO_I!=0){return 0;}
	return 1;
}


/*================================================================ 
* 
* �� �� ��:	uchar	AT1604_ReadByte(void)
* 
* �Ρ�����:	��
* 
* ��������:	��1604����һ�ֽ�����
* 
* �� �� ֵ:	��������
*
================================================================*/ 

uchar	AT1604_ReadByte(void)
{
	uchar i,cc,d;

	cc = 0;
	for (i=0;i<8;)
	{
		//INT0 = 1;		//��MCU��I0��Ϊ����״��
		
		cc = ((cc << 1)&0xfe);
		if(SLEIO_I)
			cc = cc + 1;
		//d = SLEIO_I;//INT0;
	//	cc = cc + d;
		AT1604_Plus();
		i+=1;
	}
	return cc;
}
/*================================================================ 
* 
* �� �� ��:	uchar	AT1604_WriteByte(uchar OutData)
* 
* �Ρ�����:	uchar OutData	��д�������
* 
* ��������:	��1604��дһ�ֽ�����
* 
* �� �� ֵ:	0:	�ɹ�		n>0:	��nλд����
*
================================================================*/ 

uchar	AT1604_WriteByte(uchar OutData)
{
	uchar i,cc;
	
	cc = OutData;
	for (i=0;i<8;i++)
	{
		if ( (cc & 0x80) == 0 )
			if( AT1604_Writebit() )	return i+1;
		AT1604_Plus();
		cc = cc << 1;
	}
	return 0;
}
/*================================================================ 
* 
* �� �� ��:	void	AT1604_CMPByte(uchar *CompareData,uchar CmpLen)
* 
* �Ρ�����:	uchar CompareData	�Ƚϵ�����
*		uchar CmpLen		�Ƚϵ����ݳ���
* 
* ��������:	��1604���Ƚϰ�ȫ����
* 
* �� �� ֵ:	��
*
================================================================*/ 

void	AT1604_CMPByte(uchar *CompareData,uchar CmpLen)
{
uchar i,cc;
	
	//AT1604_IO_ctl(SLEIO_OUT_CTR);//IO���
AT1604_IO_ctlpp();
//	SLECLK_H;
//	Delayus(6);
	while(CmpLen--)
	{
		cc = *(CompareData++);
		for (i=0;i<8;i++)
		{
			if ( (cc & 0x80) != 0 ) SLEIO_H;//INT0= 1;
			else SLEIO_L;//INT0 = 0;
			cc = cc << 1;
			//Delayus(10);
			SLECLK_H;
			Delayus(10);
			SLECLK_L;
			Delayus(10);
			//SLECLK_L;
		}
	}
	AT1604_IO_ctlod();
	//AT1604_IO_ctl(SLEIO_IN_CTR);//IO����
	return ;
}
/*================================================================ 
* 
* �� �� ��:	uchar	AT1604_EraseByte(void)
* 
* �Ρ�����:	��
* 
* ��������:	����1604��һ�ֽ�
* 
* �� �� ֵ:	0:	�ɹ�		1:	ʧ��
*
================================================================*/ 
uchar	AT1604_EraseByte(void)
{
	if( AT1604_Erasebit() )	return 1;
	AT1604_Inc(8);
	return 0;
}



/*================================================================ 
* 
* �� �� ��:	void	AT1604_OpenCard( void )
* 
* �Ρ�����:	��
* 
* ��������:	��1604���ϵ�
* 
* �� �� ֵ:	��
*
================================================================*/
void AT1604GPIO_Ini(void)
{
	//	��ʼ���ܽ�
	GPIO_InitTypeDef GPIO_InitStructure;
	//---�ص�
	SC1_CMDVCCH; //cmd
	SLERST_L;//rst	L
	//---�ر�UART
	USART_DeInit(DS1_UARTX);
	//USART_Cmd(DS1_UARTX, DISABLE); 
	//---����CLK
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;    //��©���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;				
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = DS1_CLK_PIN;
  	GPIO_Init(DS1_CLK_PORT, &GPIO_InitStructure);
	GPIO_ResetBits(DS1_CLK_PORT, DS1_CLK_PIN);//CLK
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;    //��©���
	//����Ĺܽ�
	GPIO_InitStructure.GPIO_Pin = DS1_AUX1_PIN;//��������
  GPIO_Init(DS1_AUX1_PORT, &GPIO_InitStructure);
 GPIO_SetBits(DS1_AUX1_PORT, DS1_AUX1_PIN); 

  GPIO_InitStructure.GPIO_Pin = DS1_AUX2_PIN;//��������
  GPIO_Init(DS1_AUX2_PORT, &GPIO_InitStructure);
  GPIO_ResetBits(DS1_AUX2_PORT, DS1_AUX2_PIN);
	//---����IO

	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;    //��©���
	GPIO_InitStructure.GPIO_Pin = DS1_IO_PIN;
  	GPIO_Init(DS1_IO_PORT, &GPIO_InitStructure);
	GPIO_SetBits(DS1_IO_PORT, DS1_IO_PIN);//IO
	
		

	
	Delay_Ms(10);
	//SLERST_H;
	SC1_35VH;//���CMD����ֹͣģʽ
	Card_PowerStatus=0;

} 

void	AT1604_OpenCard(void)
{//Beep_run(beep_able);
	//		System_WaitTime(20);
	//		Beep_run(beep_disable);
	
	
	AT1604GPIO_Ini();
	//�ϵ�
   SC1_CMDVCCL;
   Delay_Ms(50);
   Card_PowerStatus=1 ;//0Ϊδ�ϵ磬1Ϊ�ϵ�
	SLECLK_L;
	Delayus(20);
	SLECLK_H;
	Delayus(20);
	
	SLECLK_L;
	Delayus(20);
	SLECLK_H;
	Delayus(20);
	
	SLECLK_L;
	Delayus(20);
	SLECLK_H;
	Delayus(20);
	
	SLECLK_L;
	
	//SLERST_L;
	SLEFUS_H;
	SLEPGM_L;
	//INT0 = 1;
  Delay_Ms(100);

	return;
}
/*================================================================ 
* 
* �� �� ��:	void	AT1604_CloseCard( void )
* 
* �Ρ�����:	��
* 
* ��������:	��1604���µ�
* 
* �� �� ֵ:	��
*
================================================================*/ 
void	AT1604_CloseCard(void)
{
	//EX0 = 0;
	SLERST_L;
	SLEFUS_L;
	SLEPGM_L;
	SLECLK_L;
//	_EnMemcard(0);
	//INT0 = 0;
	AT1604_IO_ctlod();//IO����
	//_CardSetPower(0);
	//�µ�
   SC1_CMDVCCH;
   Delayus(200);
   Card_PowerStatus=0 ;//0Ϊδ�ϵ磬1Ϊ�ϵ�
//	IE0 = 0;
//	EX0 = 1;
	return;
}

/*================================================================ 
* 
* �� �� ��:	uchar	AT1604_ChkSC(uchar *SC)
* 
* �Ρ�����:	uchar *SC	AT88SC1604������,����Ϊ2bytes
* 
* ��������:	��֤AT88SC1604����������
* 
* �� �� ֵ:	0: �ɹ�	(return&0x0f)-1:ʣ���������Դ���	0x30:δ֪����
*
================================================================*/ 
uchar	AT1604_ChkSC(uchar *SC)
{
	uchar i;

//	EX0 = 0;
	AT1604_Reset();
	AT1604_Inc(SC_BIT_ADDR-1);
//	AT1604_Inc(SC_BIT_ADDR);
	
	AT1604_CMPByte(SC,2);
	
	for(i=8;i>0;i--)
	{
		//INT0 = 1;
		//AT1604_Plus();

		AT1604_IO_ctlod();//IO����
		if ( SLEIO_I == 0 )	AT1604_Plus();
		else 
		if( AT1604_Writebit() )	return (0x10|i);
		else
		if( AT1604_Erasebit() )	return (0x20|i);
		else	return 0;
	}
	return 0x30; 
}

/*================================================================ 
* 
* �� �� ��:	uchar	AT1604_ChkSCn(uchar Area,uchar *SCn)
* 
* �Ρ�����:	Area: ������1
*		uchar *SCn	AT88SC1604��������,����Ϊ2bytes
* 
* ��������:	��֤AT88SC1604���ķ������룬Ϊ���ȷֿ�
* 
* �� �� ֵ:	0: �ɹ�	(return&0x0f)-1:ʣ���������Դ���	0x30:δ֪����	0x40:�����Ŵ�
*
================================================================*/
uchar	AT1604_ChkSCn(uchar Area,uchar *SCn)
{
	uint i;
  
//	if( (Area<1) || (Area>4) )	return 0x40;
  
	//EX0 = 0;
	AT1604_Reset();
	i = SC1_BIT_ADDR -1;
	AT1604_Inc(i);
	
	AT1604_CMPByte(SCn,2);
	
	for(i=8;i>0;i--)
	{
		//INT0 = 1;
		AT1604_IO_ctlod();//IO����
		if ( SLEIO_I == 0 )	AT1604_Plus();
		else 
		if( AT1604_Writebit() )	return (0x10|i);
		else
		if( AT1604_Erasebit() )	return (0x20|i);
		else	return 0;
	}
	return 0x30;
}
/*===========�ۺ�===================================================== 
* 
* �� �� ��:	uchar	AT1604_Com_ChkSCSCn(uint offset,uchar *SCn)
* �ۺ�Ϊ�󲿴�����������У�飬�˶Ժ���ж������������Ƿ�˶���ȷ
* �Ρ�����:	offset: ���ż�����1������1,2,3,4�Ĳ������洢��ַλ��
*		uchar *SCn	AT88SC1604�����뼰����1����,����Ϊ2bytes
* 
* ��������:	��֤AT88SC1604�������ż�����1������1,2,3,4�Ĳ�����룬Ϊ���ȷֿ�
* 
* �� �� ֵ:	0: �ɹ�	(return&0x0f)-1:ʣ���������Դ���	0x30:δ֪����	0x40:�����Ŵ�
*
================================================================*/

uchar	AT1604_Com_ChkSCSCn(uint offset,uchar *SCn)
{
	uint i;
  
//	if( (Area<1) || (Area>4) )	return 0x40;
	//EX0 = 0;
	AT1604_Reset();
	i = offset;
	AT1604_Inc(i);
	
	AT1604_CMPByte(SCn,2);
	
	for(i=8;i>0;i--)
	{
		//INT0 = 1;
		AT1604_IO_ctlod();//IO����
		Delayus(6);
		if ( SLEIO_I == 0 )	AT1604_Plus();
		else
		{
			if( AT1604_Writebit() )	return (0x10|i);
			else
			{
				if( AT1604_Erasebit() )	return (0x20|i);
				else{AT1604_Inc(8); return 0;}
			}
		}	
	}
	return 0x30;
}

/*===============�ۺ�================================================= 
* 
* �� �� ��:	uchar	AT1604_Com_ChkUSCn(uint offset,uchar *SCn)
* 
* �Ρ�����:	offset: ����2,3,4����洢��ַλ��
*		uchar *SCn	AT88SC1604����2,3,4����,����Ϊ2bytes
* 
* ��������:	��֤AT88SC1604���ķ���2,3,4���룬Ϊ���ȷֿ�
* �ȶԺ󣬲����м����ĺ˶�
* �� �� ֵ:	0: �ɹ�		0x30:δ֪����	0x40:�����Ŵ�
*
================================================================*/
uchar	AT1604_Com_ChkUSCn(uint offset,uchar *SCn)
{
  
	//EX0 = 0;
	AT1604_Reset();
 
	AT1604_Inc(offset-1);
	
	AT1604_CMPByte(SCn,2);
	
    AT1604_IO_ctlod();//IO����
	if ( SLEIO_I )	 return 0;
	else return 0x30;
}
//�ۺϼ�������
//ͳһ�˴���������������������У��
uchar	AT1604_Com_ChkKey(uint offset,uint16 len,uchar *SCn)
{
	uint8 i;
	uint offsetbit;
	uint16 sentstatus=0;//�ظ�״̬
	offsetbit=offset*8;
	
	
	switch(offsetbit)
			{//��������
				case SC_BIT_ADDR:
				case SC1_BIT_ADDR:
				case EZ1_BIT_ADDR:
				case EZ2_BIT_ADDR:
				case EZ3_BIT_ADDR:
				case EZ4_BIT_ADDR:	
					i=AT1604_Com_ChkSCSCn(offsetbit,SCn);	
					if(i!=0)
					{	sentstatus=0x83;}
					else
					{	sentstatus=0x00;}
				break;
				//��������
				case SC2_BIT_ADDR:
				case SC3_BIT_ADDR:
				case SC4_BIT_ADDR:		
					if(AT1604_Com_ChkUSCn(offsetbit,&SCn[0])!=0)
						sentstatus=0x83;
					else
						sentstatus=0x00;
				break;
				default:
					sentstatus=0xe1;			
					break;	
			}
	return sentstatus;
}

uchar	AT1604_Com_BlowFuse(uint really)
{
	if(really==0)	//ʵ����˿
	{
		AT1604_Reset();
		SLEFUS_H;
		AT1604_Inc(FUS_BIT_ADDR);
		 SLERST_H;
     Delayus(20);//��ʱ1us
     AT1604_Writebit();
	}
	else//ģ����˿
	{
		SLEFUS_L;
	}
}
/*================================================================ 
* 
* �� �� ��:	uchar	AT1604_ChkUSCn(uchar Area,uchar *SCn)
* 
* �Ρ�����:	Area: ������2��3��4
*		uchar *SCn	AT88SC1604��������,����Ϊ2bytes
* 
* ��������:	��֤AT88SC1604���ķ������룬Ϊ���ȷֿ�
* 
* �� �� ֵ:	0: �ɹ�		0x30:δ֪����	0x40:�����Ŵ�
*
================================================================*/
uchar	AT1604_ChkUSCn(uchar Area,uchar *SCn)
{
	uint i;
  
	if( (Area<2) || (Area>4) )	return 0x40;
  
	//EX0 = 0;
	AT1604_Reset();
	if(Area==2)
		i=SC2_BIT_ADDR-1;
	else if(Area==3)
		i=SC3_BIT_ADDR-1;
	else if(Area==4)
		i=SC4_BIT_ADDR-1;
	else 
		return 0x40;

  
	AT1604_Inc(i);
	
	AT1604_CMPByte(SCn,2);
	
    AT1604_IO_ctlod();//IO����
	if ( SLEIO_I  )	 return 0;
	else return 0x30;
}
/*================================================================ 
* 
* �� �� ��:	uchar	AT1604_ChkEZn(uchar Area,uchar *EZn)
* 
* �Ρ�����:	Area: ������1��2��3��4
*		uchar *EZn	AT88SC1604������������,����Ϊ2bytes
* 
* ��������:	��֤AT88SC1604���ķ����������룬���ȷ�����
* 
* �� �� ֵ:	0: �ɹ�	(return&0x0f)-1:ʣ���������Դ���	0x30:δ֪����	0x40:�����Ŵ�
*
================================================================*/
uchar	AT1604_ChkEZn(uchar Area,uchar *EZn)
{
	uint i;
  
	//EX0 = 0;
	AT1604_Reset();
	
	if(Area==1)
		i=EZ1_BIT_ADDR-1;
	else if(Area==2)
		i=EZ2_BIT_ADDR-1;
	else if(Area==3)
		i=EZ3_BIT_ADDR-1;
	else if(Area==4)
		i=EZ4_BIT_ADDR-1;
	else 
		return 0x40;

	AT1604_Inc(i);
	
	AT1604_CMPByte(EZn,2);
	
	for(i=8;i>0;i--)
	{
		//INT0 = 1;
		AT1604_IO_ctlod();//IO����
		if ( SLEIO_I == 0 )	AT1604_Plus();
		else 
		if( AT1604_Writebit() )	return (0x10|i);
		else
		if( AT1604_Erasebit() )	return (0x20|i);
		else	return 0;
	}
	return 0x30;
}

/*================================================================ 
* 
* �� �� ��:	void	AT1604_Read(uint Addr,uint Len,uchar *DataBuf)
* 
* �Ρ�����:	Addr: ƫ�Ƶ�ַ (0--2047)
*		Len : ����
*		DataBuf	  : �������ݵĻ�������ַ
* 
* ��������:	��AT88SC1604�洢������
* 
* �� �� ֵ:	��
*
================================================================*/ 
void	AT1604_Read(uint Addr,uint Len,uchar *DataBuf)
{
	uint i;

	//EX0 = 0;
	AT1604_Reset();
	i = Addr;
	i = i << 3;
	AT1604_Inc(i);
	for (i=0;i<Len;i++)
		*(DataBuf+i) = AT1604_ReadByte();
	return;
}



/*================================================================ 
* 
* �� �� ��:	uchar	AT1604_Erase(uint Addr,uint Len)
* 
* �Ρ�����:	Addr: ƫ�Ƶ�ַ (0--2047)
*		Len : ����
* 
* ��������:	����AT88SC1604�洢������
* 
* �� �� ֵ:	0:	�ɹ�		1:	ʧ��
*
================================================================*/ 
uchar	AT1604_Erase(uint Addr,uint Len)
{
	uint i;

	//EX0 = 0;
	AT1604_Reset();
	i = Addr;
	i = i << 3;
	AT1604_Inc(i);
	for (i=0;i<Len;i++)
		if( AT1604_EraseByte() )	return 1;
	return 0;
}



/*================================================================ 
* 
* �� �� ��:	uchar	AT1604_Write(uint Addr,uint Len,uchar *DataBuf)
* 
* �Ρ�����:	Addr: ƫ�Ƶ�ַ (0--2047)
*		Len : ����
*		DataBuf	  : д���ݵĻ�������ַ
* 
* ��������:	дAT88SC102����,������øú������и�д��������Ҫ��ִ��Erase����
* 
* �� �� ֵ:	0:	�ɹ�		1:	ʧ��
*
================================================================*/ 
uchar	AT1604_Write(uint Addr,uint Len,uchar *DataBuf)
{
	uint i;

	//EX0 = 0;
	AT1604_Reset();
	i = Addr;
	i = i << 3;
	AT1604_Inc(i);
	for (i=0;i<Len;i++)
		if( AT1604_WriteByte(*(DataBuf+i)) )	return 1;
	return 0;
}



/*================================================================ 
* 
* �� �� ��:	uchar	AT1604_UpdSC(uchar *SC)
* 
* �Ρ�����:	SC:	������ 2bytes
* 
* ��������:	����AT88SC1604������,����Level 2ģʽ��SC���ɶ�,�ʸú���ûЧ�����SC�Ƿ���ȷ,��Level 1ģʽ�¿ɻض�SC���бȽ�,
*		��Level 2ģʽ��ֻ�ܸ����µ�������ϵ���AT1604_ChkSC()����Ч������Ƿ���ȷ
* 
* �� �� ֵ:	0:	�ɹ�		����:	ʧ��
*
================================================================*/ 
uchar	AT1604_UpdSC(uchar *SC)
{
	uchar i,cc;
	
	if( AT1604_Erase(SC_BIT_ADDR/8,2) )	return 2;
	
	//EX0 = 0;
	AT1604_Reset();
	AT1604_Inc(SC_BIT_ADDR);
	cc = *SC;
	for (i=0;i<8;i++)
	{
		if ( (cc & 0x80) == 0 )
			AT1604_Writebit();
		AT1604_Plus();
		cc = cc << 1;
	}
	cc = *(SC+1);
	for (i=0;i<8;i++)
	{
		if ( (cc & 0x80) == 0 )
			AT1604_Writebit();
		AT1604_Plus();
		cc = cc << 1;
	}
	
	return 0;
}

/*================================================================ 
* 
* �� �� ��:	uchar	AT1604_UpdSCAC(uchar *SCAC)
* 
* �Ρ�����:	SCAC:	�������������� 1bytes
* 
* ��������:	����AT88SC1604��������������
* 
* �� �� ֵ:	0:	�ɹ�		����:	ʧ��
*
================================================================*/ 
uchar	AT1604_UpdSCAC(uchar *SCAC)
{
	if( AT1604_Erase(SCAC_BIT_ADDR/8,1) )	return 2;
	return AT1604_Write(SCAC_BIT_ADDR/8,1,SCAC);
}



/*================================================================ 
* 
* �� �� ��:	uchar	AT1604_UpdSCn(uchar Area,uchar *SCn)
* 
* �Ρ�����:	Area: ������1��2��3��4
*		SCn:	�������� 2bytes
* 
* ��������:	����AT88SC1604��������,����Level 2ģʽ��SCn���ɶ�,�ʸú���ûЧ�����SCn�Ƿ���ȷ,��Level 1ģʽ�¿ɻض�SCn���бȽ�,
*		��Level 2ģʽ��ֻ�ܸ����µ�������ϵ���AT1604_ChkSCn()����Ч������Ƿ���ȷ
* 
* �� �� ֵ:	0:	�ɹ�		����:	ʧ��
*
================================================================*/ 
uchar	AT1604_UpdSCn(uchar Area,uchar *SCn)
{
	uint i;
	uchar cc;
  if(Area==1)
		i=SC1_BIT_ADDR/8;
	else if(Area==2)
		i=SC2_BIT_ADDR/8;
	else if(Area==3)
		i=SC3_BIT_ADDR/8;
	else if(Area==4)
		i=SC4_BIT_ADDR/8;
	else 
		return 0x40;

	if( AT1604_Erase(i,2) )	return 2;
	
	//EX0 = 0;
	AT1604_Reset();
	AT1604_Inc(i*8);
	cc = *SCn;
	for (i=0;i<8;i++)
	{
		if ( (cc & 0x80) == 0 )
			AT1604_Writebit();
		AT1604_Plus();
		cc = cc << 1;
	}
	cc = *(SCn+1);
	for (i=0;i<8;i++)
	{
		if ( (cc & 0x80) == 0 )
			AT1604_Writebit();
		AT1604_Plus();
		cc = cc << 1;
	}
	
	return 0;
}

/*================================================================ 
* 
* �� �� ��:	uchar	AT1604_UpdSnAC(uchar Area,uchar *SnAC)
* 
* �Ρ�����:	Area: ������1
*		SnAC:	���������������� 1bytes
* 
* ��������:	����AT88SC1604����������������
* 
* �� �� ֵ:	0:	�ɹ�		����:	ʧ��
*
================================================================*/ 
uchar	AT1604_UpdSnAC(uchar Area,uchar *SnAC)
{
	uint i;
	
	
	if(Area==1)
		i=S1AC_BIT_ADDR/8;
	else
		return 3;
	
	if( AT1604_Erase(i,1) )	return 2;
	return AT1604_Write(i,1,SnAC);
}

/*================================================================ 
* 
* �� �� ��:	uchar	AT1604_UpdEZn(uchar Area,uchar *EZn)
* 
* �Ρ�����:	Area: ������1��2��3��4
*		EZn:	������������ 2bytes
* 
* ��������:	����AT88SC1604������������,����Level 2ģʽ��EZn���ɶ�,�ʸú���ûЧ�����EZn�Ƿ���ȷ,��Level 1ģʽ�¿ɻض�EZn���бȽ�,
*		��Level 2ģʽ��ֻ�ܸ����µ�������ϵ���AT1604_ChkEZn()����Ч������Ƿ���ȷ
* 
* �� �� ֵ:	0:	�ɹ�		����:	ʧ��
*
================================================================*/ 
uchar	AT1604_UpdEZn(uchar Area,uchar *EZn)
{
	uint i;
	uchar cc;
  
	/*if( (Area<1) || (Area>4) )	return 3;
  
	i = (EZ1_BIT_ADDR + (Area-1)*4144)/8 ;*/
	
	if(Area==1)
		i=EZ1_BIT_ADDR/8;
	else if(Area==2)
		i=EZ2_BIT_ADDR/8;
	else if(Area==3)
		i=EZ3_BIT_ADDR/8;
	else if(Area==4)
		i=EZ4_BIT_ADDR/8;
	else 
		return 0x40;
	

	if( AT1604_Erase(i,2) )	return 2;
	
	//EX0 = 0;
	AT1604_Reset();
	AT1604_Inc(i*8);
	cc = *EZn;
	for (i=0;i<8;i++)
	{
		if ( (cc & 0x80) == 0 )
			AT1604_Writebit();
		AT1604_Plus();
		cc = cc << 1;
	}
	cc = *(EZn+1);
	for (i=0;i<8;i++)
	{
		if ( (cc & 0x80) == 0 )
			AT1604_Writebit();
		AT1604_Plus();
		cc = cc << 1;
	}
	
	return 0;
}

/*================================================================ 
* 
* �� �� ��:	uchar	AT1604_UpdEnAC(uchar Area,uchar *EnAC)
* 
* �Ρ�����:	Area: ������1��2��3��4
*		EnAC:	�������������������� 1bytes
* 
* ��������:	����AT88SC1604��������������������
* 
* �� �� ֵ:	0:	�ɹ�		����:	ʧ��
*
================================================================*/ 
uchar	AT1604_UpdEnAC(uchar Area,uchar *EnAC)
{
	uint i;
	
	/*if( (Area<1) || (Area>4) )	return 3;
	
	i = (E1AC_BIT_ADDR + (Area-1)*4144)/8 ;*/
	
	if(Area==1)
		i=EZ1_BIT_ADDR/8;
	else if(Area==2)
		i=EZ2_BIT_ADDR/8;
	else if(Area==3)
		i=EZ3_BIT_ADDR/8;
	else if(Area==4)
		i=EZ4_BIT_ADDR/8;
	else 
		return 0x40;
	
	if( AT1604_Erase(i,1) )	return 2;
	return AT1604_Write(i,1,EnAC);
}

/*================================================================ 
* 
* �� �� ��:	uchar	AT1604_UpdMTZ ( uchar *MTZ )
* 
* �Ρ�����:	MTZ:	���������� 2bytes
* 
* ��������:	����AT88SC1604������
* 
* �� �� ֵ:	0:	�ɹ�		����:	ʧ��
*
================================================================*/ 
uchar	AT1604_UpdMTZ ( uchar *MTZ )
{
	if( AT1604_Erase(MTZ_BIT_ADDR/8,2) )	return 2;
	return AT1604_Write(MTZ_BIT_ADDR/8,2,MTZ);
}
//�ۺϸ��¸�����
//���������룬�������룬��������
uchar	AT1604_UpdSCSCn(uint offset,uchar *SC)
{
	uchar i,cc;
	uint offsetbit;
	offsetbit=offset*8;
	
	switch(offsetbit)
	{//��������
		case SC_BIT_ADDR:
		case SC1_BIT_ADDR:
		case EZ1_BIT_ADDR:
		case EZ2_BIT_ADDR:
		case EZ3_BIT_ADDR:
		case EZ4_BIT_ADDR:
		//��������
		case SC2_BIT_ADDR:
		case SC3_BIT_ADDR:
		case SC4_BIT_ADDR:	
			break;	
		default:
			return 0xe1;			
		break;	
	}
	
	
	if( AT1604_Erase(offset,2) )	return 0x81;
	
	//EX0 = 0;
	AT1604_Reset();
	AT1604_Inc(offsetbit);
	cc = *SC;
	for (i=0;i<8;i++)
	{
		if ( (cc & 0x80) == 0 )
			AT1604_Writebit();
		AT1604_Plus();
		cc = cc << 1;
	}
	cc = *(SC+1);
	for (i=0;i<8;i++)
	{
		if ( (cc & 0x80) == 0 )
			AT1604_Writebit();
		AT1604_Plus();
		cc = cc << 1;
	}
	
	return 0;
}
/*******************************************************************************
����Ϊ�Զ��庯��
********************************************************************************/
//������void	AT1604_Read(uint Addr,uint Len,uchar *DataBuf)
//��һ�ֽ�uchar	AT1604_ReadByte(void)
int16 Read_1604_Bit(uint16 StartBitAddress,uint16 NOBit,char * Data);
int16 Write_1604_Byte(uint16 StartByteAddress,char * Data);//uchar	AT1604_WriteByte(uchar OutData)
int16 Write_1604_Array(uint16 ByteAddress,uint16 ByteCount,char *Data);//uchar	AT1604_Write(uint Addr,uint Len,uchar *DataBuf)
int16 Erase_1604_Byte(uint16 ByteAddress);//ʹ��uchar	AT1604_EraseByte(void)
int16 Erase_1604_Array(uint16 ByteAddress,uint16 ByteCount);//ʹ��uchar	AT1604_Erase(uint Addr,uint Len)
int16 Verify_1604_SC(uint8 * SCArray);//��ʹ��uchar	AT1604_ChkSC(uchar *SC)����
int16 Verify_1604_SC1(uint8 *SCArray);//��ʹ��uchar	AT1604_ChkSCn(uchar Area,uchar *SCn)����
int16 Verify_1604_SC2(uint8 *SCArray);//��ʹ��uchar	AT1604_ChkSCn(uchar Area,uchar *SCn)����
int16 Verify_1604_SC3(uint8 *SCArray); //��ʹ��uchar	AT1604_ChkSCn(uchar Area,uchar *SCn)����
int16 Verify_1604_SC4(uint8 *SCArray); //��ʹ��uchar	AT1604_ChkSCn(uchar Area,uchar *SCn)����
int16 Verify_1604U_SC2(uint8 *SCArray);//���ȷ�������2��3��4����������������������AT1604_ChkUSCn��
int16 Verify_1604U_SC3(uint8 *SCArray);//���ȷ�������2��3��4�������������������� AT1604_ChkUSCn��
int16 Verify_1604U_SC4(uint8 *SCArray);//���ȷ�������2��3��4�������������������� AT1604_ChkUSCn��
int16 Verify_1604_EZ1(uint8 *EZArray);//��ʹ��uchar	AT1604_ChkEZn(uchar Area,uchar *EZn)����
int16 Verify_1604_EZ2(uint8 *EZArray);//��ʹ��uchar	AT1604_ChkEZn(uchar Area,uchar *EZn)����
int16 Verify_1604_EZ3(uint8 *EZArray); //��ʹ��uchar	AT1604_ChkEZn(uchar Area,uchar *EZn)����
int16 Verify_1604_EZ4(uint8 *EZArray); //��ʹ��uchar	AT1604_ChkEZn(uchar Area,uchar *EZn)����
int16 Verify_1604U_EZ2(uint8 *EZArray);//���ȷ���������ʹ��uchar	AT1604_ChkUEZn(uchar Area,uchar *EZn)
int16 Verify_1604U_EZ3(uint8 *EZArray);//���ȷ���������ʹ��uchar	AT1604_ChkUEZn(uchar Area,uchar *EZn)
int16 Verify_1604U_EZ4(uint8 *EZArray);//���ȷ���������ʹ��uchar	AT1604_ChkUEZn(uchar Area,uchar *EZn)
int16 Fuse_High_1604(void);
int16 Fuse_Low_1604(void);

//-------------102----------------------------------------
#define AT102_SC_BIT_ADDR	80//������λ��ַ
#define AT102_MFZFUS_BIT_ADDR		1456
#define AT102_EC2ENFUS_BIT_ADDR	1529
#define AT102_ISSUEFUS_BIT_ADDR	1552
//������˶�
uchar	AT102_ChkSC(uchar *SC)
{
	uchar i;

//	EX0 = 0;
	AT1604_Reset();
	AT1604_Inc(AT102_SC_BIT_ADDR);
	
	AT1604_CMPByte(SC,2);
	
	for(i=4;i>0;i--)
	{
		//INT0 = 1;
		//AT1604_Plus();

		AT1604_IO_ctlod();//IO����
		if ( SLEIO_I == 0 )	AT1604_Plus();
		else 
		if( AT1604_Writebit() )	return (0x10|i);
		else
		if( AT1604_Erasebit() )	return (0x20|i);
		else	return 0;
	}
	return 0x30; 
}

uchar	AT102_Write(uint Addr,uint Len,uchar *DataBuf)
{
	uint i;

	//EX0 = 0;
	AT1604_Reset();
	i = Addr;
	i = i << 3;
	AT1604_Inc(i);
	for (i=0;i<Len;i++)
		if( AT1604_WriteByte(*(DataBuf+i)) )	return 1;
	return 0;
}
//��������˶�
uchar	AT102_Com_ChkUSCn(uint offset,uchar len,uchar *SCn)
{
  
	//EX0 = 0;
	AT1604_Reset();
 
	AT1604_Inc(offset);
	AT1604_CMPByte(SCn,len);
	
    AT1604_IO_ctlod();//IO����
	if ( SLEIO_I )	 return 0;
	else return 0x30;
}
//��������:	����AT102�洢������
uchar	AT102_Erase(uint Addr,uint Len)
{
	uint i;

	//EX0 = 0;
	AT1604_Reset();
	i = Addr;
	i = i << 3;
	AT1604_Inc(i);
	for (i=0;i<Len;i++)
		if( AT1604_EraseByte() )	return 1;
	return 0;
}
//����AT88SC102��������������
uchar	AT102_UpdSCSCn(uint off,uint len,uchar *SC)
{
	uchar i,cc;
	
  if( AT1604_Erase(off,len) )	return 2;
  	
	AT1604_Reset();
	AT1604_Inc(off);
	
	for(i=0;i<len;)
	{
		if( AT1604_WriteByte(*(SC+i)) )	return 1;
		i+=1;	
	}	
	return 0;
}
//ģ����˿
void AT102_SimulateFuss(uint8 c)
{
	if(c==0)SLEFUS_L;
	else
		SLEFUS_H;	
}

void C102WriteFus(int address, int bitlen)
{ uint i;
	AT1604_Reset();
	SLEFUS_H;
	AT1604_Inc(address);
	SLERST_H;
  Delayus(20);//��ʱ1us
	for (i=0;i<bitlen;i++)
	{
		AT1604_Writebit();	
	}
}
//�۶�AT88SC102��MFZ��˿
void	AT102_BlowMFZFuse( void )
{
	uint8 da=0x00;
	C102WriteFus(AT102_MFZFUS_BIT_ADDR, 16);
	SLERST_L;
}

void	AT102_BlowEC2ENFuse(void)
{
	uint8 da=0x00;
  C102WriteFus(AT102_EC2ENFUS_BIT_ADDR, 1);
	SLERST_L;
	return;
}
//�������۶�AT88SC102��Issue��˿��
void	AT102_BloweIssuFus( void )
{	
	uint8 da=0x00;
	C102WriteFus(AT102_ISSUEFUS_BIT_ADDR, 16);
	SLERST_L;
}