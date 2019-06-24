/************************************************************************ 
* 
*　 文件名:	AT1608.c 
* 
*　 文件描述:	逻辑加密卡at88sc1608公共库函数集 
* 
*　 创建人: 	dxs, 2007年3月9日 
* 
*　 版本号:	1.0 
* 
*　 修改记录: 
* 
管脚顺序
VCC--C1	  C5--GND
RST--C2	  C6--N/A
SCL--C3	  C7--SDA
N/A--C4	  C8--N/A
************************************************************************/ 
#include  "stdlib.h"//含随机数生成函数
#include <string.h>	//含字节拷贝函数
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


#define SLEIO_IN_CTR 0
#define SLEIO_OUT_CTR 1

extern void Delayus(uint32 num);//单位为0.5us

extern volatile uint8 	Card_PowerStatus;//0为未上电，1为上电

uchar	flag;
uchar	Ri,Si,Ti;
uchar	R_I_O_data[9];
uchar	S_I_O_data[9];
uchar	T_I_O_data[9];

/*extern	void	_Delay(uint mSecond);
extern	void	_CardSetCard(uchar ICNO);
extern	void	_CardSetPower(uchar Level);
extern	void	_EnMemcard(uchar Level);
extern	void	_CardSetReset(uchar Level);
extern	void	_CardSetClock(uchar Level);
extern	void 	_CardSetFUS(uchar Level);
extern	void	_CardSetPGM(uchar Level); 
extern	uchar	_Random(void); */

void	AT1608_Start_bit(void);
void	AT1608_Stop_bit(void);
void	AT1608_mack(void);
void	AT1608_mnack(void);
uchar	AT1608_cack(void);
void	AT1608_PLUS(void);
void	AT1608_OutB(uchar OutData);
uchar	AT1608_InB(void);
uchar	AT1608_WriteByte(uchar Level,uchar Addr,uchar buff);

void	AT1608_Arithmetic(void);
uchar	AT1608_Arithmetic_last(void);
uchar	AT1608_Arithmetic_times(uchar Arithmetic_times);
void	AT1608_Arithmetic_key(uchar key);
void	AT1608_Authentication(uchar *CI_data,uchar *GC_data,uchar *Q0_data,uchar *Q1_data,uchar *Q2_data );
/**************************************************

**************************************************/
void AT1608_IO_ctlpp(void)
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
void AT1608_IO_ctlod(void)
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

void	AT1608_Start_bit(void)
{

	SLEIO_H;
	Delayus(20);
	SLECLK_H;
	Delayus(20);
	SLEIO_L;
	Delayus(20);
	SLECLK_L;
	Delayus(20);
}

void	AT1608_Stop_bit(void)
{
	SLEIO_L;
	Delayus(4);
	SLECLK_H;
	Delayus(4);
	SLEIO_H;
	Delayus(4);
	SLECLK_L;
	Delayus(4);
}

void	AT1608_mack(void)
{
	SLEIO_L;Delayus(4);
	SLECLK_H;Delayus(4);
	SLECLK_L;Delayus(4);
	SLEIO_H;Delayus(4);
}

void	AT1608_mnack(void)
{
	SLEIO_H;Delayus(4);
	SLECLK_H;Delayus(4);
	SLECLK_L;Delayus(4);
	SLEIO_L;Delayus(4);
}

/*uchar	AT1608_cack(void)
{
	AT1608_IO_ctl(SLEIO_IN_CTR);//IO输入
	SLECLK_H;Delayus(4);
	if(SLEIO_I)
	{
		SLECLK_L;Delayus(4);
		AT1608_IO_ctl(SLEIO_OUT_CTR);//IO输出
		return 1;	
	}
	else
	{
		SLECLK_L;Delayus(4);
		AT1608_IO_ctl(SLEIO_OUT_CTR);//IO输出
		return 0;
	}
}*/
uchar	AT1608_cack(void)
{
	uchar ucErrTime=0;
	SLEIO_H;
	Delayus(20);
	SLECLK_H;Delayus(20);
	while(SLEIO_I)
	{
		ucErrTime++;
		if(ucErrTime>200)
		{break;
			//不做错误处理，实际应用时很多卡无法检测到ACK
		}
	}
	SLECLK_L;Delayus(20);
		SLEIO_H;
		Delayus(20);
		return 0;
}
void	AT1608_PLUS(void)
{
	SLECLK_H;Delayus(20);
	SLECLK_L;Delayus(20);
	//_CardSetClock(1);
	//_CardSetClock(0);
}

void	AT1608_OutB(uchar OutData)
{AT1608_IO_ctlpp();
	for (flag=0;flag<8;flag++)
	{
		if ( (OutData & 0x80) == 0x80 ) SLEIO_H;//SDA = 1;
		else  SLEIO_L;//SDA = 0;
		Delayus(20);
		AT1608_PLUS();
		OutData = OutData << 1;
	}
	AT1608_IO_ctlod();
	flag = AT1608_cack();
}

uchar	AT1608_InB(void)
{
	uchar i;
	uchar InData;
	
	InData = 0;
	SLEIO_H;
	for (i=0;i<8;i++)
	{
		//InData = InData<<1;
    InData=((InData<<1)&0xfe);
		SLECLK_H;
		Delayus(20);
		//if ( SLEIO_I != 0x00)
		//	InData = InData + 0x01;
		InData=InData+(SLEIO_I?0x01:0);	
		SLECLK_L;
		Delayus(20);
	}
	return InData;
}

uchar	AT1608_WriteByte(uchar Level,uchar Addr,uchar buff)
{
	AT1608_Stop_bit();
	AT1608_Start_bit();
	if(Level)	AT1608_OutB(0xB4);
	else		AT1608_OutB(0xB0);
	if(flag)	return 1;
	AT1608_OutB(Addr);
	if(flag)	return 1;
	AT1608_OutB(buff);
	if(flag)	return 1;
	AT1608_Stop_bit();
	Delayus(20000);
	//_Delay(5);
	return 0;
}

void	AT1608_Arithmetic(void)
{
	uchar temp,buff[9];
	
	memcpy(buff,R_I_O_data,8);
	temp = (((buff[6] >> 4 ) & 0x01) + (buff[6] << 1)) & 0x1f;
	buff[6] = buff[5];
	buff[5] = buff[4];
	buff[4] = buff[3];
	buff[3] = buff[2] ^ Ri;
	buff[2] = buff[1];
	buff[1] = buff[0];
	buff[0] = buff[4] + temp;
	
	if(buff[0] > 31)
	{
		if(buff[0] % 31)
			buff[0] %= 31;
		else
			buff[0] = 31;
	}
	memcpy(R_I_O_data,buff,8);
	
	memcpy(buff,S_I_O_data,8);
	temp = (((buff[6] >> 6 ) & 0x01) + (buff[6] << 1 ) ) & 0x7f;
	buff[6] = buff[5];
	buff[5] = buff[4] ^ Si;
	buff[4] = buff[3];
	buff[3] = buff[2];
	buff[2] = buff[1];
	buff[1] = buff[0];
	buff[0] = buff[6] + temp;	

	if(buff[0] > 127)
	{
		if(buff[0] % 127)
			buff[0] %= 127;
		else
			buff[0] = 127;
	}
	memcpy(S_I_O_data,buff,8);
	
	memcpy(buff,T_I_O_data,5);
	temp = buff[4];
	buff[4] = buff[3];
	buff[3] = buff[2];
	buff[2] = buff[1] ^ Ti;	
	buff[1] = buff[0];
	buff[0] = buff[3] + temp;
	
	if(buff[0] > 31)
	{
		if(buff[0] % 31)
			buff[0] %= 31;
		else
			buff[0] = 31;
	}
	memcpy(T_I_O_data,buff,5);
}

uchar	AT1608_Arithmetic_last(void)
{
        uchar Si_;

	Si = Ri = Ti = 0;
	
	AT1608_Arithmetic();

	Ri = (R_I_O_data[0] ^ R_I_O_data[4] ) & 0x1f;
	Ti = (T_I_O_data[0] ^ T_I_O_data[3] ) & 0x1f;	
	Si =  S_I_O_data[0];	

	Si_ = 0xff - Si ;

	return (((Si & Ti) | (Si_ & Ri)) & 0x0f );// | dxs
}

uchar	AT1608_Arithmetic_times(uchar Arithmetic_times)
{
	uchar i,last;

	for(i=0;i<Arithmetic_times;i++)
	{
		last = AT1608_Arithmetic_last();
	}
	return last;	
}

void	AT1608_Arithmetic_key(uchar key)
{
	Ri = key & 0x1f;
	Si = (( key >> 5 ) & 0x07 ) + (( key << 3 ) & 0x78 );
	Ti = ( key >> 3 ) & 0x1f;
	
	AT1608_Arithmetic();	
}

void	AT1608_Authentication(uchar *CI_data,uchar *GC_data,uchar *Q0_data,uchar *Q1_data,uchar *Q2_data )
{
	uchar i,temp;

	memset(R_I_O_data,0,8);
        memset(S_I_O_data,0,8);
        memset(T_I_O_data,0,8);
        
	for(i=0;i<4;i++)
	{
		AT1608_Arithmetic_key(CI_data[2*i + 0]);
		AT1608_Arithmetic_key(CI_data[2*i + 1]);
		AT1608_Arithmetic_key(Q0_data[i]) ;
	}
		
	for(i=0;i<4;i++)
	{
		AT1608_Arithmetic_key(GC_data[2*i + 0]);
		AT1608_Arithmetic_key(GC_data[2*i + 1]);
		AT1608_Arithmetic_key(Q0_data[4+i]) ;
	}
		
	for(i=0;i<8;i++)
	{
		temp  = AT1608_Arithmetic_times(2) << 4;
		Q2_data[i] = AT1608_Arithmetic_times(2) + temp;

		temp  = AT1608_Arithmetic_times(2) << 4;
		Q1_data[i] = AT1608_Arithmetic_times(2) + temp;
	}	
}
/*
uchar	AT1608_Arithmetic_Test()
{
	int i;
	uchar GC_data[8]={0xff,0xfd,0xfd,0xfd,0xfd,0xfd,0xfd,0xfd};
	uchar CI_data[8]={0xFB,0x30,0xE7,0xF9,0xB3,0xF9,0x68,0x19};
	uchar Q0_data[8]={0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};
	uchar Q1_data0[8]={0x5a,0x55,0x34,0x0a,0x11,0xa7,0x5c,0x61};
	uchar Q2_data0[8]={0x51,0x03,0x2f,0xa8,0x83,0x6d,0xc8,0x18};
	uchar Q1_data[8];
	uchar Q2_data[8];
	
        AT1608_Authentication(CI_data,GC_data,Q0_data,Q1_data,Q2_data );
        
        if(memcmp(Q1_data,Q1_data0,8))	return 1;
        if(memcmp(Q2_data,Q2_data0,8))	return 2;
	return 0;
        
}
*/
/*================================================================ 
* 
* 函 数 名:	void	AT1608_OpenCard(uchar *RstData)
* 
* 参　　数:	RstData	4字节复位数据指针
* 
* 功能描述:	给1608卡上电复位
* 
* 返 回 值:	无
*
================================================================*/ 
void AT1608GPIO_Ini(void)
{
	//	初始化管脚
	GPIO_InitTypeDef GPIO_InitStructure;
	//---关电
	SC1_CMDVCCH; //cmd
	SLERST_L;//rst	L
	//---关闭UART
	USART_Cmd(DS1_UARTX, DISABLE); 
	//---设置CLK
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;				
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = DS1_CLK_PIN;
  	GPIO_Init(DS1_CLK_PORT, &GPIO_InitStructure);
	GPIO_SetBits(DS1_CLK_PORT, DS1_CLK_PIN);//CLK
	//---设置IO
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;//GPIO_Mode_IPU; //上拉输入
	GPIO_InitStructure.GPIO_Pin = DS1_IO_PIN;
  	GPIO_Init(DS1_IO_PORT, &GPIO_InitStructure);

	Delay_Ms(10);
	SLERST_H;
	SC1_35VH;//结合CMD进入停止模式
	Card_PowerStatus=0;
	//SC1_CMDVCCL;
	//--------------------------------	
	SLECLK_L;
	SLEIO_H;
	SLERST_L;
	//上电
   	SC1_CMDVCCL;
   	Delay_Ms(50);
	Card_PowerStatus=1;
	
	SLECLK_L;
	Delayus(20);
	SLECLK_H;
	Delayus(20);

	SLECLK_L;
	Delayus(20);

	AT1608_IO_ctlod();
	SLEIO_H;
	Delay_Ms(10);
} 

void	AT1608_OpenCard(uchar *RstData)
{unsigned char i,j;
	SLECLK_L;
	
	SLERST_H;
	Delayus(20);
	AT1608_PLUS();
	SLERST_L;
	Delayus(20);
	AT1608_IO_ctlod();
	SLEIO_H;
	for(i=0;i<4;)
	{
		RstData[i] = 0;
		for(j=0;j<8;)
		{
			RstData[i]=((RstData[i]>>1)&0x7f);			
			//RstData[i] >>= 1;
			//_CardSetClock(1);
			SLECLK_H;
			Delayus(20);
			RstData[i]=RstData[i]+(SLEIO_I?0x80:0);
			//if ( SLEIO_I != 0x00)
			//	RstData[i] += 0x80;
		//	_CardSetClock(0);
			SLECLK_L;
			Delayus(20);
			j+=1;
		}
		i+=1;
	}
	return;

}

/*================================================================ 
* 
* 函 数 名:	void	AT1608_CloseCard( void )
* 
* 参　　数:	无
* 
* 功能描述:	给1608卡下电
* 
* 返 回 值:	无
*
================================================================*/ 
void	AT1608_CloseCard(void)
{
   SLERST_L;
	SLEFUS_L;
	SLEPGM_L;
	SLECLK_L;
	//下电
   SC1_CMDVCCH;
   Delayus(200);
   Card_PowerStatus=0;
   return;
}

/*================================================================ 
* 
* 函 数 名:	uchar	AT1608_SetAZ(uchar AZ)
* 
* 参　　数:	AZ 应用区号(0--7)
* 
* 功能描述:	设置应用区
* 
* 返 回 值:	0:	成功		其它:	失败
*
================================================================*/ 
uchar	AT1608_SetAZ(uchar AZ)
{
	if(AZ>7) return 2;
	
	AT1608_Stop_bit();
	AT1608_Start_bit();
	AT1608_OutB(0xB2);
	if(flag)	return 1;
	AT1608_OutB(AZ);
	if(flag)	return 1;
	AT1608_Stop_bit();
	return 0;
}

/*================================================================ 
* 
* 函 数 名:	uchar	AT1608_Read(uchar Level,uchar Addr,uint Len,uchar *DataBuf)
* 
* 参　　数:	Level: 1--设置区	0--应用区
*		Addr: 偏移地址 设置区(0--127)	应用区(0--255)
*		Len : 长度
*		DataBuf	  : 读出数据的缓冲区地址
* 
* 功能描述:	读AT88SC1608存储区数据
* 
* 返 回 值:	0:	成功		其它:	失败
*
================================================================*/ 
uchar	AT1608_Read(uchar Level,uchar Addr,uint Len,uchar *DataBuf)
{
	uint i;
	//0x80 fuses
	if( (Level)&&((Addr+Len)>129) )	return 3;
	if( (!Level)&&((Addr+Len)>256) ) return 2;
	
	//AT1608_Stop_bit();
//	AT1608_PLUS();
//	AT1608_PLUS();
	//AT1608_PLUS();
//	AT1608_PLUS();
	AT1608_Start_bit();
	if(Level)	AT1608_OutB(0xB5);
	else		AT1608_OutB(0xB1);
	if(flag)	return 1;
	AT1608_OutB(Addr);
	if(flag)	return 2;
	for(i=0;i<Len-1;i++)
	{
		DataBuf[i]=AT1608_InB();
		AT1608_mack();
	}
	DataBuf[i]=AT1608_InB();
	AT1608_mnack();
	AT1608_Stop_bit();
	return 0;
}

/*================================================================ 
* 
* 函 数 名:	uchar	AT1608_Write(uchar Level,uchar Addr,uint Len,uchar *DataBuf)
* 
* 参　　数:	Level: 1--设置区	0--应用区
*		Addr: 偏移地址 设置区(0--127)	应用区(0--255)
*		Len : 长度
*		DataBuf	  : 写入数据的缓冲区地址
* 
* 功能描述:	写AT88SC1608存储区数据
* 
* 返 回 值:	0:	成功		其它:	失败
*
================================================================*/ 
uchar	AT1608_Write(uchar Level,uchar Addr,uint Len,uchar *DataBuf)
{
	uint i;
	
	if( (Level)&&((Addr+Len)>128) )	return 3;
	if( (!Level)&&((Addr+Len)>256) ) return 2;
	
	for(i=0;i<Len;i++)
		if(AT1608_WriteByte(Level,Addr+i,DataBuf[i]))	return 1;
	return 0;
}

/*================================================================ 
* 
* 函 数 名:	uchar	AT1608_ReadFuse(uchar *Fuse)
* 
* 参　　数:	Fuse:	熔丝状况 1bytes
* 
* 功能描述:	读AT88SC1608熔丝状况
* 
* 返 回 值:	0--OK	1--Error
*
================================================================*/ 
uchar	AT1608_ReadFuse(uchar *Fuse)
{
	AT1608_Stop_bit();
	AT1608_Start_bit();
	AT1608_OutB(0xB5);
	if(flag)	return 1;
	AT1608_OutB(0x80);
	if(flag)	return 1;
	*Fuse=AT1608_InB();
	AT1608_mnack();
	AT1608_Stop_bit();
	return 0;
}

/*================================================================ 
* 
* 函 数 名:	uchar	AT1608_WriteFuse(void)
* 
* 参　　数:	无
* 
* 功能描述:	写AT88SC1608熔丝
* 
* 返 回 值:	0--OK	1--Error
*
================================================================*/ 
uchar	AT1608_WriteFuse(void)
{
	AT1608_Stop_bit();
	AT1608_Start_bit();
	AT1608_OutB(0xB4);
	if(flag)	return 1;
	AT1608_OutB(0x80);
	if(flag)	return 1;
	AT1608_Stop_bit();
	return 0;
}

/*================================================================ 
* 
* 函 数 名:	uchar	AT1608_VerifyPassword(uchar Index,uchar *Password)
* 
* 参　　数:	uchar Index 密码索引号 写密码索引号(0--7) 读密码索引号(0x80--0x87)
*		uchar *Password	AT88SC1608密码,长度为3bytes
* 
* 功能描述:	验证AT88SC1608卡的读写密码
* 
* 返 回 值:	0: 成功	0x10、0x20:于卡通讯错误	其它:密码效验错误次数,最多8次
*
================================================================*/ 
uchar	AT1608_VerifyPassword(uchar Index,uchar *Password)
{
	uchar i,j,k,l;
	
	l = Index&0x07;
	if(Index>=0x80)	l |= 0x08;
	
	AT1608_Stop_bit();
	AT1608_Start_bit();
	AT1608_OutB(0xB3);
	if(flag)	return 0x10;
	AT1608_OutB(l);
	if(flag)	return 0x20;
	for(i=0;i<3;i++)
	{
		AT1608_OutB(Password[i]);
		if(flag)	return 0x30;
	}
	AT1608_Stop_bit();
	Delayus(40000);
	//_Delay(20);
	
	i = ((Index&0x07)<<3) + 0x40;
	if(Index>=0x80)	i+=0x04;
	
	if(AT1608_Read(1,i,1,&j))	return 0x40;
	if(j==0xff)	return 0;
	k=0;
	for(i=0;i<8;i++)
	{
		if( (j&0x01)==0x00 )	k++;
		j>>=1;
	}
	return k;
}

uchar	AT1608_InitAuth(uchar *Q0)
{
	uchar i;
	
	AT1608_Stop_bit();
	AT1608_Start_bit();
	AT1608_OutB(0xB6);
	if(flag)	return 1;
	for(i=0;i<8;i++)
	{
		AT1608_OutB(Q0[i]);
		if(flag)	return 1;
	}
	AT1608_Stop_bit();
	Delayus(40000);
	//_Delay(20);
	
	return 0;
}

uchar	AT1608_VerifyAuth(uchar *Q1)
{
	uchar i,j,k;
	
	AT1608_Stop_bit();
	AT1608_Start_bit();
	AT1608_OutB(0xB7);
	if(flag)	return 0x10;
	for(i=0;i<8;i++)
	{
		AT1608_OutB(Q1[i]);
		if(flag)	return 0x10;
	}
	AT1608_Stop_bit();
	Delayus(40000);
	//_Delay(20);
	
	if(AT1608_Read(1,0x20,1,&j))	return 0x20;
	if(j==0xff)	return 0;
	k=0;
	for(i=0;i<8;i++)
	{
		if( (j&0x01)==0x00 )	k++;
		j>>=1;
	}
	return k;
}

/*================================================================ 
* 
* 函 数 名:	uchar	AT1608_Auth(uchar *Gc)
* 
* 参　　数:	uchar *Gc	8bytes 密钥
* 
* 功能描述:	AT88SC1608卡安全认证
* 
* 返 回 值:	0: 成功	0x10、0x20、0x30、0x40、0x50:于卡通讯错误	0x60:卡非法	其它:安全认证错误次数,最多8次
*
================================================================*/
uchar	AT1608_Auth(uchar *Gc)
{
	uchar i,Ci[9],Q0[9],Q1[9],Q2[9];
	
	if(AT1608_Read(1,0x28,8,Ci))	return 0x30;
	for(i=0;i<8;i++)	Q0[i] = rand();//_Random();
	AT1608_Authentication(Ci,Gc,Q0,Q1,Q2);
	if(AT1608_InitAuth(Q0))	return 0x40;
	i = AT1608_VerifyAuth(Q1);
	if(i)	return i;
	if(AT1608_Read(1,0x28,8,Ci))	return 0x50;
	if(memcmp(Ci,Q2,8))	return 0x60;
	return 0;
}
uchar	AT1608_AuthWithRand(uchar *Gc,uchar *QT0)
{
	uchar i,Ci[9],Q0[9],Q1[9],Q2[9];
	
	if(AT1608_Read(1,0x28,8,Ci))	return 0x30;
	for(i=0;i<8;i++)	Q0[i] = QT0[i];//rand();//_Random();
	AT1608_Authentication(Ci,Gc,Q0,Q1,Q2);
	if(AT1608_InitAuth(Q0))	return 0x40;
	i = AT1608_VerifyAuth(Q1);
	if(i)	return i;
	if(AT1608_Read(1,0x28,8,Ci))	return 0x50;
	if(memcmp(Ci,Q2,8))	return 0x60;
	return 0;
}
/*****************************************************************
以下为自定义函数
*****************************************************************/
int16 Select_1608_User_Zone(uint16 ZoneAddr);
int16 Read_1608_User_Zone(uint16 ByteAddr,uint16 NOB,char * Data);
int16 Write_1608_User_Zone(uint16 ByteAddr,uint16 NOB,char * Data);
int16 Read_1608_Configuration(uint16 ByteAddr,uint16 NOB,char * Data);
int16 Write_1608_Configuration(uint16 ByteAddr,uint16 NOB,char * Data);
int16 Read_1608_Fuses(char * Fuses);
int16 Write_1608_Fuses(void);
int16 Verify_1608_Password(uint16 ReadWrite,uint16 SetNumber,char *Password);
int16 Init_1608_Authentication(char * Q0);
int16 Verify_1608_Authentication(char * Q1);
//---------------------------------------
//----------153--------------------------
//---------------------------------------
uchar	AT153_WriteByte(uchar zone,uchar Addr,uchar buff)
{
	uchar i;
	i=zone;
	i=((i<<2)&0x0c)+0xB0;
	AT1608_Stop_bit();
	AT1608_Start_bit();
	AT1608_OutB(i);

	if(flag)	return 1;
	AT1608_OutB(Addr);
	if(flag)	return 1;
	AT1608_OutB(buff);
	if(flag)	return 1;
	AT1608_Stop_bit();
	Delayus(20000);
	//_Delay(5);
	return 0;
}
/*================================================================ 
* 
* 函 数 名:	uchar	AT153_Write(uchar Level,uchar Addr,uint Len,uchar *DataBuf)
* 
* 参　　数:	zone: 0-2--应用区 3--配置区
*		Addr: 偏移地址 (0--63)	
*		Len : 长度
*		DataBuf	  : 写入数据的缓冲区地址
* 
* 功能描述:	写AT88SC153存储区数据
* 
* 返 回 值:	0:	成功		其它:	失败
*
================================================================*/ 
uchar	AT153_Write(uchar zone,uchar Addr,uint Len,uchar *DataBuf)
{
	uint i;
	if(zone>3)return 1;
	if((Addr+Len)>64) return 2;

	
	for(i=0;i<Len;i++)
		if(AT153_WriteByte(zone,Addr+i,DataBuf[i]))	return 1;
	return 0;
}
/*================================================================ 
* 
* 函 数 名:	uchar	AT153_Read(uchar Level,uchar Addr,uint Len,uchar *DataBuf)
* 
* 参　　数:	Level: 1--设置区	0--应用区
*		Addr: 偏移地址 设置区(0--127)	应用区(0--255)
*		Len : 长度
*		DataBuf	  : 读出数据的缓冲区地址
* 
* 功能描述:	读AT88SC153存储区数据
* 
* 返 回 值:	0:	成功		其它:	失败
*
================================================================*/ 
uchar	AT153_Read(uchar zone,uchar Addr,uint Len,uchar *DataBuf)
{
	uint i;
	//0x80 fuses
	if(zone>3)return 1;
	if((Addr+Len)>64) return 2;
	
	i=zone;
	i=((i<<2)&0x0c)+0xB1;
	//AT1608_Stop_bit();
//	AT1608_PLUS();
//	AT1608_PLUS();
	//AT1608_PLUS();
//	AT1608_PLUS();
	AT1608_Start_bit();
	AT1608_OutB(i);
	if(flag)	return 3;
	AT1608_OutB(Addr);
	if(flag)	return 4;
	for(i=0;i<Len-1;i++)
	{
		DataBuf[i]=AT1608_InB();
		AT1608_mack();
	}
	DataBuf[i]=AT1608_InB();
	AT1608_mnack();
	AT1608_Stop_bit();
	return 0;
}
uchar	AT153_InitAuth(uchar *Q0)
{
	uchar i;
	
	AT1608_Stop_bit();
	AT1608_Start_bit();
	AT1608_OutB(0xB2);
	if(flag)	return 1;
	for(i=0;i<8;i++)
	{
		AT1608_OutB(Q0[i]);
		if(flag)	return 1;
	}
	AT1608_Stop_bit();
	Delayus(40000);
	//_Delay(20);
	
	return 0;
}
uchar	AT153_VerifyAuth(uchar *Q1)
{
	uchar i,j,k;
	
	AT1608_Stop_bit();
	AT1608_Start_bit();
	AT1608_OutB(0xB6);
	if(flag)	return 0x10;
	for(i=0;i<8;i++)
	{
		AT1608_OutB(Q1[i]);
		if(flag)	return 0x10;
	}
	AT1608_Stop_bit();
	Delayus(40000);
	//_Delay(20);
	
	if(AT153_Read(3,0x20,1,&j))	return 0x20;
	if(j==0xff)	return 0;
	k=0;
	for(i=0;i<8;i++)
	{
		if( (j&0x01)==0x00 )	k++;
		j>>=1;
	}
	return k;
}
/*================================================================ 
* 
* 函 数 名:	uchar	AT153_VerifyPassword(uchar Index,uchar *Password)
* 
* 参　　数:	uchar Index 密码索引号 写密码索引号(0--7) 读密码索引号(0x80--0x87)
*		uchar *Password	AT88SC153密码,长度为3bytes
* 
* 功能描述:	验证AT88SC153卡的读写密码
* 默认认证密码454545
* 返 回 值:	0: 成功	0x10、0x20:于卡通讯错误	其它:密码效验错误次数,最多4次
*
================================================================*/ 
//错误计数，高4位跟低4位同效，如错一次，为EE，两次为CC
uchar	AT153_VerifyPassword(uchar Index,uchar *Password)
{
	uchar i,j,k,l;
	
	
	AT1608_Stop_bit();
	AT1608_Start_bit();
	AT1608_OutB(Index);
	if(flag)	return 0x10;
	for(i=0;i<3;i++)
	{
		AT1608_OutB(Password[i]);
		if(flag)	return 0x20;
	}
	AT1608_Stop_bit();
	Delayus(40000);
	//要发两次
	AT1608_Stop_bit();
	AT1608_Start_bit();
	AT1608_OutB(Index);
	if(flag)	return 0x10;
	for(i=0;i<3;i++)
	{
		AT1608_OutB(Password[i]);
		if(flag)	return 0x20;
	}
	AT1608_Stop_bit();
	Delayus(40000);
	
	i=0;
	switch(Index)
	{
		case  0xbf:
			i+=4;
		case  0xb7:	
			i+=4;
		case  0xbb:
			i+=4;
		case  0xb3:
			i+=0x30;
		break;
		default:
			return 0x30;		
	}
	
	if(AT153_Read(3,i,1,&j))	return 0x40;
	//	return j;
		
		
	if(j==0xff)	return 0;
	k=0;
	for(i=0;i<4;i++)
	{
		if( (j&0x01)==0x00 )	k++;
		j>>=1;
	}
	return k;
}
/*================================================================ 
* 
* 函 数 名:	uchar	AT153_ReadFuse(uchar *Fuse)
* 
* 参　　数:	Fuse:	熔丝状况 1bytes
* 
* 功能描述:	读AT88SC153熔丝状况
* 
* 返 回 值:	0--OK	1--Error
*
================================================================*/ 
uchar	AT153_ReadFuse(uchar *Fuse)
{
	AT1608_Stop_bit();
	AT1608_Start_bit();
	AT1608_OutB(0xBe);
	if(flag)	return 1;
	*Fuse=AT1608_InB();
	AT1608_mnack();
	AT1608_Stop_bit();
	return 0;
}

/*================================================================ 
* 
* 函 数 名:	uchar	AT153_WriteFuse(void)
* 
* 参　　数:	无
* 
* 功能描述:	写AT88SC153熔丝
* 
* 返 回 值:	0--OK	1--Error
*
================================================================*/ 
uchar	AT153_WriteFuse(uchar fus)
{
	AT1608_Stop_bit();
	AT1608_Start_bit();
	AT1608_OutB(0xBa);
	if(flag)	return 1;
	AT1608_OutB(fus&0x07);
	if(flag)	return 1;
	AT1608_Stop_bit();
	return 0;
}