/************************************************************************ 
* 
*　 文件名:	AT1604.c 
* 
*　 文件描述:	逻辑加密卡at88sc1604公共库函数集 
* 
*　 创建人: 	dxs, 2007年3月8日 
* 
*　 版本号:	1.0 
* 
*　 修改记录: 
* 
管脚顺序
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
#define SLECLK_H	DS1_CLK_PORT->BSRR=DS1_CLK_PIN//GPIO_SetBits(DS1_CLK_PORT,DS1_CLK_PIN)		  /*时钟高*/
#define SLECLK_L	DS1_CLK_PORT->BRR=DS1_CLK_PIN//GPIO_ResetBits(DS1_CLK_PORT,DS1_CLK_PIN)	  /*时钟低*/
#define SLEIO_H		DS1_IO_PORT->BSRR=DS1_IO_PIN//GPIO_SetBits(DS1_IO_PORT,DS1_IO_PIN)		  //数据线高
#define SLEIO_L		DS1_IO_PORT->BRR=DS1_IO_PIN//GPIO_ResetBits(DS1_IO_PORT,DS1_IO_PIN)	  //数据线低
#define SLEIO_I		(DS1_IO_PORT->IDR&DS1_IO_PIN)//GPIO_ReadInputDataBit(DS1_IO_PORT,DS1_IO_PIN)	 //输入数据，返回值为unsigned char
#define SLERST_H	RST_Select(0x10)		   //复位高
#define SLERST_L	RST_Select(CARD_FIRST)		//复位低
#define SC1_35VH 	 GPIO_SetBits(DS1_35V_PORT, DS1_35V_PIN)
#define SC1_35VL 	 GPIO_ResetBits(DS1_35V_PORT, DS1_35V_PIN)

#define SLEPGM_H DS1_AUX2_PORT->BSRR=DS1_AUX2_PIN//GPIO_SetBits(DS1_AUX2_PORT, DS1_AUX2_PIN)
#define SLEPGM_L DS1_AUX2_PORT->BRR=DS1_AUX2_PIN//GPIO_ResetBits(DS1_AUX2_PORT, DS1_AUX2_PIN)
#define SLEFUS_H DS1_AUX1_PORT->BSRR=DS1_AUX1_PIN//GPIO_SetBits(DS1_AUX1_PORT, DS1_AUX1_PIN)
#define SLEFUS_L DS1_AUX1_PORT->BRR=DS1_AUX1_PIN//GPIO_ResetBits(DS1_AUX1_PORT, DS1_AUX1_PIN)

#define SLEIO_IN_CTR 0
#define SLEIO_OUT_CTR 1
//不等分卡
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


extern volatile uint8 	Card_PowerStatus;//0为未上电，1为上电
extern void Delayus(uint32 num);//单位为0.5us 
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
* 函 数 名:	void	AT1604_Reset(void)
* 
* 参　　数:	无
* 
* 功能描述:	使1604卡地址复位为0
* 
* 返 回 值:	无
*
================================================================*/ 

void	AT1604_Reset(void)
{
	SC1_CMDVCCL; 
	Card_PowerStatus=1;//上电
	
	
	//SLEFUS_H;
	SLEPGM_L;
	//AT1604_IO_ctl(SLEIO_IN_CTR);//IO输入
	AT1604_IO_ctlod();
	//AT1604_IO_ctl(SLEIO_OUT_CTR);//
	//SLEIO_L;
	Delayus(20);//延时1us

    
  SLERST_H;
  Delayus(20);//延时1us
	SLECLK_H;
	//
	Delayus(6);//延时1us
	SLECLK_L;
	//AT1604_IO_ctl(SLEIO_IN_CTR);//IO输入
	Delayus(20);//延时1us
	Delayus(20);//延时1us
	SLERST_L;
    Delayus(20);//延时2us  
	return;
}

/*================================================================ 
* 
* 函 数 名:	void	AT1604_Plus(void)
* 
* 参　　数:	无
* 
* 功能描述:	使1604卡地址计数器加1
* 
* 返 回 值:	无
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
* 函 数 名:	void	AT1604_Inc(uint Step)
* 
* 参　　数:	uint Step	需要前进的地址数
* 
* 功能描述:	使1604卡地址计数器向前Step
* 
* 返 回 值:	无
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
* 函 数 名:	uchar	AT1604_Writebit(void)
* 
* 参　　数:	无
* 
* 功能描述:	1604卡位写操作
* 
* 返 回 值:	0:	成功		1:	失败	
*
================================================================*/ 



uchar	AT1604_Writebit(void)
{	
	uint32 tmp;
	
	SLEPGM_H;
	Delayus(6);
	AT1604_IO_ctlpp();
	SLEIO_L;//写
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
* 函 数 名:	uchar	AT1604_Erasebit(void)
* 
* 参　　数:	无
* 
* 功能描述:	1604卡位擦除操作
* 
* 返 回 值:	0:	成功		1:	失败
*
================================================================*/ 

uchar	AT1604_Erasebit(void)
{	
	
	uint32 tmp;
	
	SLEPGM_H;
	Delayus(6);

	AT1604_IO_ctlpp();
	SLEIO_H;//写
	Delayus(6);
	SLECLK_H;
	Delayus(6);
	
	SLEPGM_L;
	Delayus(6);
	Delay_Ms(10);
	AT1604_IO_ctlod();//必须放后面
	SLECLK_L;
	
	
	
	
	 
	
	Delayus(6);
	if(SLEIO_I!=0){return 0;}
	return 1;
}


/*================================================================ 
* 
* 函 数 名:	uchar	AT1604_ReadByte(void)
* 
* 参　　数:	无
* 
* 功能描述:	从1604卡读一字节数据
* 
* 返 回 值:	读出数据
*
================================================================*/ 

uchar	AT1604_ReadByte(void)
{
	uchar i,cc,d;

	cc = 0;
	for (i=0;i<8;)
	{
		//INT0 = 1;		//置MCU的I0口为输入状况
		
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
* 函 数 名:	uchar	AT1604_WriteByte(uchar OutData)
* 
* 参　　数:	uchar OutData	需写入的数据
* 
* 功能描述:	向1604卡写一字节数据
* 
* 返 回 值:	0:	成功		n>0:	第n位写错误
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
* 函 数 名:	void	AT1604_CMPByte(uchar *CompareData,uchar CmpLen)
* 
* 参　　数:	uchar CompareData	比较的数据
*		uchar CmpLen		比较的数据长度
* 
* 功能描述:	与1604卡比较安全代码
* 
* 返 回 值:	无
*
================================================================*/ 

void	AT1604_CMPByte(uchar *CompareData,uchar CmpLen)
{
uchar i,cc;
	
	//AT1604_IO_ctl(SLEIO_OUT_CTR);//IO输出
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
	//AT1604_IO_ctl(SLEIO_IN_CTR);//IO输入
	return ;
}
/*================================================================ 
* 
* 函 数 名:	uchar	AT1604_EraseByte(void)
* 
* 参　　数:	无
* 
* 功能描述:	擦除1604卡一字节
* 
* 返 回 值:	0:	成功		1:	失败
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
* 函 数 名:	void	AT1604_OpenCard( void )
* 
* 参　　数:	无
* 
* 功能描述:	给1604卡上电
* 
* 返 回 值:	无
*
================================================================*/
void AT1604GPIO_Ini(void)
{
	//	初始化管脚
	GPIO_InitTypeDef GPIO_InitStructure;
	//---关电
	SC1_CMDVCCH; //cmd
	SLERST_L;//rst	L
	//---关闭UART
	USART_DeInit(DS1_UARTX);
	//USART_Cmd(DS1_UARTX, DISABLE); 
	//---设置CLK
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;    //开漏输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;				
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = DS1_CLK_PIN;
  	GPIO_Init(DS1_CLK_PORT, &GPIO_InitStructure);
	GPIO_ResetBits(DS1_CLK_PORT, DS1_CLK_PIN);//CLK
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;    //开漏输出
	//其余的管脚
	GPIO_InitStructure.GPIO_Pin = DS1_AUX1_PIN;//上拉输入
  GPIO_Init(DS1_AUX1_PORT, &GPIO_InitStructure);
 GPIO_SetBits(DS1_AUX1_PORT, DS1_AUX1_PIN); 

  GPIO_InitStructure.GPIO_Pin = DS1_AUX2_PIN;//上拉输入
  GPIO_Init(DS1_AUX2_PORT, &GPIO_InitStructure);
  GPIO_ResetBits(DS1_AUX2_PORT, DS1_AUX2_PIN);
	//---设置IO

	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;    //开漏输出
	GPIO_InitStructure.GPIO_Pin = DS1_IO_PIN;
  	GPIO_Init(DS1_IO_PORT, &GPIO_InitStructure);
	GPIO_SetBits(DS1_IO_PORT, DS1_IO_PIN);//IO
	
		

	
	Delay_Ms(10);
	//SLERST_H;
	SC1_35VH;//结合CMD进入停止模式
	Card_PowerStatus=0;

} 

void	AT1604_OpenCard(void)
{//Beep_run(beep_able);
	//		System_WaitTime(20);
	//		Beep_run(beep_disable);
	
	
	AT1604GPIO_Ini();
	//上电
   SC1_CMDVCCL;
   Delay_Ms(50);
   Card_PowerStatus=1 ;//0为未上电，1为上电
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
* 函 数 名:	void	AT1604_CloseCard( void )
* 
* 参　　数:	无
* 
* 功能描述:	给1604卡下电
* 
* 返 回 值:	无
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
	AT1604_IO_ctlod();//IO输入
	//_CardSetPower(0);
	//下电
   SC1_CMDVCCH;
   Delayus(200);
   Card_PowerStatus=0 ;//0为未上电，1为上电
//	IE0 = 0;
//	EX0 = 1;
	return;
}

/*================================================================ 
* 
* 函 数 名:	uchar	AT1604_ChkSC(uchar *SC)
* 
* 参　　数:	uchar *SC	AT88SC1604主密码,长度为2bytes
* 
* 功能描述:	验证AT88SC1604卡的主密码
* 
* 返 回 值:	0: 成功	(return&0x0f)-1:剩余密码重试次数	0x30:未知错误
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

		AT1604_IO_ctlod();//IO输入
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
* 函 数 名:	uchar	AT1604_ChkSCn(uchar Area,uchar *SCn)
* 
* 参　　数:	Area: 分区号1
*		uchar *SCn	AT88SC1604分区密码,长度为2bytes
* 
* 功能描述:	验证AT88SC1604卡的分区密码，为不等分卡
* 
* 返 回 值:	0: 成功	(return&0x0f)-1:剩余密码重试次数	0x30:未知错误	0x40:分区号错
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
		AT1604_IO_ctlod();//IO输入
		if ( SLEIO_I == 0 )	AT1604_Plus();
		else 
		if( AT1604_Writebit() )	return (0x10|i);
		else
		if( AT1604_Erasebit() )	return (0x20|i);
		else	return 0;
	}
	return 0x30;
}
/*===========综合===================================================== 
* 
* 函 数 名:	uchar	AT1604_Com_ChkSCSCn(uint offset,uchar *SCn)
* 综合为后部带计数的密码校验，核对后会判定计数来决定是否核对正确
* 参　　数:	offset: 主号及分区1及分区1,2,3,4的搽除密码存储地址位号
*		uchar *SCn	AT88SC1604主密码及分区1密码,长度为2bytes
* 
* 功能描述:	验证AT88SC1604卡的主号及分区1及分区1,2,3,4的搽除密码，为不等分卡
* 
* 返 回 值:	0: 成功	(return&0x0f)-1:剩余密码重试次数	0x30:未知错误	0x40:分区号错
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
		AT1604_IO_ctlod();//IO输入
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

/*===============综合================================================= 
* 
* 函 数 名:	uchar	AT1604_Com_ChkUSCn(uint offset,uchar *SCn)
* 
* 参　　数:	offset: 分区2,3,4密码存储地址位号
*		uchar *SCn	AT88SC1604分区2,3,4密码,长度为2bytes
* 
* 功能描述:	验证AT88SC1604卡的分区2,3,4密码，为不等分卡
* 比对后，不进行计数的核对
* 返 回 值:	0: 成功		0x30:未知错误	0x40:分区号错
*
================================================================*/
uchar	AT1604_Com_ChkUSCn(uint offset,uchar *SCn)
{
  
	//EX0 = 0;
	AT1604_Reset();
 
	AT1604_Inc(offset-1);
	
	AT1604_CMPByte(SCn,2);
	
    AT1604_IO_ctlod();//IO输入
	if ( SLEIO_I )	 return 0;
	else return 0x30;
}
//综合检验密码
//统一了带计数及不带计数的密码校验
uchar	AT1604_Com_ChkKey(uint offset,uint16 len,uchar *SCn)
{
	uint8 i;
	uint offsetbit;
	uint16 sentstatus=0;//回复状态
	offsetbit=offset*8;
	
	
	switch(offsetbit)
			{//带计数的
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
				//不带计数
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
	if(really==0)	//实际熔丝
	{
		AT1604_Reset();
		SLEFUS_H;
		AT1604_Inc(FUS_BIT_ADDR);
		 SLERST_H;
     Delayus(20);//延时1us
     AT1604_Writebit();
	}
	else//模拟熔丝
	{
		SLEFUS_L;
	}
}
/*================================================================ 
* 
* 函 数 名:	uchar	AT1604_ChkUSCn(uchar Area,uchar *SCn)
* 
* 参　　数:	Area: 分区号2、3、4
*		uchar *SCn	AT88SC1604分区密码,长度为2bytes
* 
* 功能描述:	验证AT88SC1604卡的分区密码，为不等分卡
* 
* 返 回 值:	0: 成功		0x30:未知错误	0x40:分区号错
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
	
    AT1604_IO_ctlod();//IO输入
	if ( SLEIO_I  )	 return 0;
	else return 0x30;
}
/*================================================================ 
* 
* 函 数 名:	uchar	AT1604_ChkEZn(uchar Area,uchar *EZn)
* 
* 参　　数:	Area: 分区号1、2、3、4
*		uchar *EZn	AT88SC1604分区擦除密码,长度为2bytes
* 
* 功能描述:	验证AT88SC1604卡的分区擦除密码，不等分区卡
* 
* 返 回 值:	0: 成功	(return&0x0f)-1:剩余密码重试次数	0x30:未知错误	0x40:分区号错
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
		AT1604_IO_ctlod();//IO输入
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
* 函 数 名:	void	AT1604_Read(uint Addr,uint Len,uchar *DataBuf)
* 
* 参　　数:	Addr: 偏移地址 (0--2047)
*		Len : 长度
*		DataBuf	  : 读出数据的缓冲区地址
* 
* 功能描述:	读AT88SC1604存储区数据
* 
* 返 回 值:	无
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
* 函 数 名:	uchar	AT1604_Erase(uint Addr,uint Len)
* 
* 参　　数:	Addr: 偏移地址 (0--2047)
*		Len : 长度
* 
* 功能描述:	擦除AT88SC1604存储区数据
* 
* 返 回 值:	0:	成功		1:	失败
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
* 函 数 名:	uchar	AT1604_Write(uint Addr,uint Len,uchar *DataBuf)
* 
* 参　　数:	Addr: 偏移地址 (0--2047)
*		Len : 长度
*		DataBuf	  : 写数据的缓冲区地址
* 
* 功能描述:	写AT88SC102数据,如果调用该函数进行改写操作，需要先执行Erase操作
* 
* 返 回 值:	0:	成功		1:	失败
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
* 函 数 名:	uchar	AT1604_UpdSC(uchar *SC)
* 
* 参　　数:	SC:	主密码 2bytes
* 
* 功能描述:	更新AT88SC1604主密码,因在Level 2模式下SC不可读,故该函数没效验更新SC是否正确,在Level 1模式下可回读SC自行比较,
*		在Level 2模式下只能给卡下电后重新上电用AT1604_ChkSC()函数效验更新是否正确
* 
* 返 回 值:	0:	成功		其它:	失败
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
* 函 数 名:	uchar	AT1604_UpdSCAC(uchar *SCAC)
* 
* 参　　数:	SCAC:	主密码错误计数器 1bytes
* 
* 功能描述:	更新AT88SC1604主密码错误计数器
* 
* 返 回 值:	0:	成功		其它:	失败
*
================================================================*/ 
uchar	AT1604_UpdSCAC(uchar *SCAC)
{
	if( AT1604_Erase(SCAC_BIT_ADDR/8,1) )	return 2;
	return AT1604_Write(SCAC_BIT_ADDR/8,1,SCAC);
}



/*================================================================ 
* 
* 函 数 名:	uchar	AT1604_UpdSCn(uchar Area,uchar *SCn)
* 
* 参　　数:	Area: 分区号1、2、3、4
*		SCn:	分区密码 2bytes
* 
* 功能描述:	更新AT88SC1604分区密码,因在Level 2模式下SCn不可读,故该函数没效验更新SCn是否正确,在Level 1模式下可回读SCn自行比较,
*		在Level 2模式下只能给卡下电后重新上电用AT1604_ChkSCn()函数效验更新是否正确
* 
* 返 回 值:	0:	成功		其它:	失败
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
* 函 数 名:	uchar	AT1604_UpdSnAC(uchar Area,uchar *SnAC)
* 
* 参　　数:	Area: 分区号1
*		SnAC:	分区密码错误计数器 1bytes
* 
* 功能描述:	更新AT88SC1604分区密码错误计数器
* 
* 返 回 值:	0:	成功		其它:	失败
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
* 函 数 名:	uchar	AT1604_UpdEZn(uchar Area,uchar *EZn)
* 
* 参　　数:	Area: 分区号1、2、3、4
*		EZn:	分区擦除密码 2bytes
* 
* 功能描述:	更新AT88SC1604分区擦除密码,因在Level 2模式下EZn不可读,故该函数没效验更新EZn是否正确,在Level 1模式下可回读EZn自行比较,
*		在Level 2模式下只能给卡下电后重新上电用AT1604_ChkEZn()函数效验更新是否正确
* 
* 返 回 值:	0:	成功		其它:	失败
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
* 函 数 名:	uchar	AT1604_UpdEnAC(uchar Area,uchar *EnAC)
* 
* 参　　数:	Area: 分区号1、2、3、4
*		EnAC:	分区擦除密码错误计数器 1bytes
* 
* 功能描述:	更新AT88SC1604分区擦除密码错误计数器
* 
* 返 回 值:	0:	成功		其它:	失败
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
* 函 数 名:	uchar	AT1604_UpdMTZ ( uchar *MTZ )
* 
* 参　　数:	MTZ:	测试区数据 2bytes
* 
* 功能描述:	更新AT88SC1604测试区
* 
* 返 回 值:	0:	成功		其它:	失败
*
================================================================*/ 
uchar	AT1604_UpdMTZ ( uchar *MTZ )
{
	if( AT1604_Erase(MTZ_BIT_ADDR/8,2) )	return 2;
	return AT1604_Write(MTZ_BIT_ADDR/8,2,MTZ);
}
//综合更新各密码
//包含中密码，分区密码，擦除密码
uchar	AT1604_UpdSCSCn(uint offset,uchar *SC)
{
	uchar i,cc;
	uint offsetbit;
	offsetbit=offset*8;
	
	switch(offsetbit)
	{//带计数的
		case SC_BIT_ADDR:
		case SC1_BIT_ADDR:
		case EZ1_BIT_ADDR:
		case EZ2_BIT_ADDR:
		case EZ3_BIT_ADDR:
		case EZ4_BIT_ADDR:
		//不带计数
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
以下为自定义函数
********************************************************************************/
//连续读void	AT1604_Read(uint Addr,uint Len,uchar *DataBuf)
//读一字节uchar	AT1604_ReadByte(void)
int16 Read_1604_Bit(uint16 StartBitAddress,uint16 NOBit,char * Data);
int16 Write_1604_Byte(uint16 StartByteAddress,char * Data);//uchar	AT1604_WriteByte(uchar OutData)
int16 Write_1604_Array(uint16 ByteAddress,uint16 ByteCount,char *Data);//uchar	AT1604_Write(uint Addr,uint Len,uchar *DataBuf)
int16 Erase_1604_Byte(uint16 ByteAddress);//使用uchar	AT1604_EraseByte(void)
int16 Erase_1604_Array(uint16 ByteAddress,uint16 ByteCount);//使用uchar	AT1604_Erase(uint Addr,uint Len)
int16 Verify_1604_SC(uint8 * SCArray);//可使用uchar	AT1604_ChkSC(uchar *SC)函数
int16 Verify_1604_SC1(uint8 *SCArray);//可使用uchar	AT1604_ChkSCn(uchar Area,uchar *SCn)函数
int16 Verify_1604_SC2(uint8 *SCArray);//可使用uchar	AT1604_ChkSCn(uchar Area,uchar *SCn)函数
int16 Verify_1604_SC3(uint8 *SCArray); //可使用uchar	AT1604_ChkSCn(uchar Area,uchar *SCn)函数
int16 Verify_1604_SC4(uint8 *SCArray); //可使用uchar	AT1604_ChkSCn(uchar Area,uchar *SCn)函数
int16 Verify_1604U_SC2(uint8 *SCArray);//不等分区卡的2，3，4分区无密码错误次数计数，AT1604_ChkUSCn，
int16 Verify_1604U_SC3(uint8 *SCArray);//不等分区卡的2，3，4分区无密码错误次数计数 AT1604_ChkUSCn，
int16 Verify_1604U_SC4(uint8 *SCArray);//不等分区卡的2，3，4分区无密码错误次数计数 AT1604_ChkUSCn，
int16 Verify_1604_EZ1(uint8 *EZArray);//可使用uchar	AT1604_ChkEZn(uchar Area,uchar *EZn)函数
int16 Verify_1604_EZ2(uint8 *EZArray);//可使用uchar	AT1604_ChkEZn(uchar Area,uchar *EZn)函数
int16 Verify_1604_EZ3(uint8 *EZArray); //可使用uchar	AT1604_ChkEZn(uchar Area,uchar *EZn)函数
int16 Verify_1604_EZ4(uint8 *EZArray); //可使用uchar	AT1604_ChkEZn(uchar Area,uchar *EZn)函数
int16 Verify_1604U_EZ2(uint8 *EZArray);//不等分区卡，可使用uchar	AT1604_ChkUEZn(uchar Area,uchar *EZn)
int16 Verify_1604U_EZ3(uint8 *EZArray);//不等分区卡，可使用uchar	AT1604_ChkUEZn(uchar Area,uchar *EZn)
int16 Verify_1604U_EZ4(uint8 *EZArray);//不等分区卡，可使用uchar	AT1604_ChkUEZn(uchar Area,uchar *EZn)
int16 Fuse_High_1604(void);
int16 Fuse_Low_1604(void);

//-------------102----------------------------------------
#define AT102_SC_BIT_ADDR	80//主密码位地址
#define AT102_MFZFUS_BIT_ADDR		1456
#define AT102_EC2ENFUS_BIT_ADDR	1529
#define AT102_ISSUEFUS_BIT_ADDR	1552
//主密码核对
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

		AT1604_IO_ctlod();//IO输入
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
//擦除密码核对
uchar	AT102_Com_ChkUSCn(uint offset,uchar len,uchar *SCn)
{
  
	//EX0 = 0;
	AT1604_Reset();
 
	AT1604_Inc(offset);
	AT1604_CMPByte(SCn,len);
	
    AT1604_IO_ctlod();//IO输入
	if ( SLEIO_I )	 return 0;
	else return 0x30;
}
//功能描述:	擦除AT102存储区数据
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
//更新AT88SC102主密码或擦除密码
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
//模拟熔丝
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
  Delayus(20);//延时1us
	for (i=0;i<bitlen;i++)
	{
		AT1604_Writebit();	
	}
}
//熔断AT88SC102的MFZ熔丝
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
//函数将熔断AT88SC102的Issue熔丝。
void	AT102_BloweIssuFus( void )
{	
	uint8 da=0x00;
	C102WriteFus(AT102_ISSUEFUS_BIT_ADDR, 16);
	SLERST_L;
}