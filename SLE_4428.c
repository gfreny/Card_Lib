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
#define SLERST_L	RST_Select(0x00)		//复位低
#define SC1_35VH 	 GPIO_SetBits(DS1_35V_PORT, DS1_35V_PIN)
#define SC1_35VL 	 GPIO_ResetBits(DS1_35V_PORT, DS1_35V_PIN)

#define SLEIO_IN_CTR 0
#define SLEIO_OUT_CTR 1
unsigned char sle4428_outgoing(unsigned	char *rdbuf,unsigned int size,unsigned int n);
extern volatile uint32 	systicnum;
extern volatile uint8 	Card_PowerStatus;//0为未上电，1为上电
void sel4428_IO_ctl(uint8 iotype)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	if(iotype==SLEIO_IN_CTR)//上拉
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	else
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; //GPIO_Mode_Out_PP;

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;				  	
	GPIO_InitStructure.GPIO_Pin = DS1_IO_PIN;
  	GPIO_Init(DS1_IO_PORT, &GPIO_InitStructure);
	GPIO_SetBits(DS1_IO_PORT, DS1_IO_PIN);
}
void sle4428GPIO_Ini()
{	
	GPIO_InitTypeDef GPIO_InitStructure;
	//---关电
	SC1_CMDVCCH; //cmd
	SLERST_L;//rst	L
	//---关闭UART
	USART_Cmd(DS1_UARTX, DISABLE); 
	//---设置IO
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;				
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = DS1_CLK_PIN;
  	GPIO_Init(DS1_CLK_PORT, &GPIO_InitStructure);
	GPIO_SetBits(DS1_CLK_PORT, DS1_CLK_PIN);//CLK

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;    //开漏输出
	GPIO_InitStructure.GPIO_Pin = DS1_IO_PIN;
  	GPIO_Init(DS1_IO_PORT, &GPIO_InitStructure);
	GPIO_SetBits(DS1_IO_PORT, DS1_IO_PIN);//IO

	//其余
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Pin = DS1_AUX1_PIN;//上拉输入
  GPIO_Init(DS1_AUX1_PORT, &GPIO_InitStructure);
  GPIO_SetBits(DS1_AUX1_PORT, DS1_AUX1_PIN); 

  GPIO_InitStructure.GPIO_Pin = DS1_AUX2_PIN;//上拉输入
  GPIO_Init(DS1_AUX2_PORT, &GPIO_InitStructure);
  GPIO_SetBits(DS1_AUX2_PORT, DS1_AUX2_PIN);

	Delay_Ms(10);
	SLERST_H;
	SC1_35VH;//结合CMD进入停止模式
	
	SLECLK_H;
	SLEIO_H;
	SLERST_H;
	SC1_CMDVCCL;
	Card_PowerStatus=1;//上电
}

/******************************  Delayus ****************************************
* @	函数名：Delayus
* @	功能说明：us级别延时
* @ 输入参数1：
* @ 输入参数2：
* @ 输入参数3：
* @	输出参数：
* @ 注意事项：使用维持阻塞模式，动用systemtick ，定义个全局变量system_tick_num
* @ 版本信息：10.5.21 pqf
* @ 调用示例：
*********************************************************************************************/ 

/*void Delayus(uint32 num)
{
	TIM4_Cmd_OP(num);
	while(Time4_over==0x00);		
}*/
/******************************  delay12us() ****************************************
* @	函数名：delay12us()
* @	功能说明：调用Delayus完成的小玩意儿
* @ 输入参数1：
* @ 输入参数2：
* @ 输入参数3：
* @	输出参数：
* @ 注意事项：
* @ 版本信息：10.5.21 pqf
* @ 调用示例：
*********************************************************************************************/
/*void delay12us()
{
   //Delayus(10);
   TIM4_Cmd_OP(12);
	while(Time4_over==0x00);
}*/
/******************************  delay4us() ****************************************
* @	函数名：delay4us()
* @	功能说明：调用Delayus完成的小玩意儿
* @ 输入参数1：
* @ 输入参数2：
* @ 输入参数3：
* @	输出参数：
* @ 注意事项：
* @ 版本信息：10.5.21 pqf
* @ 调用示例：
*********************************************************************************************/
/*void delay4us()
{
  //Delayus(3);
  TIM4_Cmd_OP(4);
	while(Time4_over==0x00);
}*/



// 1024x8 bit EEPROM              //
// 1024x1 bit protection memory   //
// sle4428卡复位与响应复位函数    // 
/******************************   ****************************************
* @	函数名：sle4428_reset
* @	功能说明：复位函数
* @ 输入参数1：
* @ 输入参数2：
* @ 输入参数3：
* @	输出参数：
* @ 注意事项：
* @ 版本信息：10-5-28	pqf
* @ 调用示例：
*********************************************************************************************/
void sle4428_reset(void)
{
	unsigned char i=31;
	if(GPIO_ReadInputDataBit(DS1_OFF_PORT,DS1_OFF_PIN)==0)
	{SC1_CMDVCCH;
	 GPIO_SetBits(DS1_35V_PORT, DS1_35V_PIN);
	 SC_Delay();
	}
	SC1_CMDVCCL; Card_PowerStatus=1;//上电
	sel4428_IO_ctl(SLEIO_OUT_CTR);
	SLEIO_H;
	SLECLK_L;
	SLERST_L;
	delay12us();            // tPOR  
	delay12us();            // tPOR  
	SLERST_H;	
	delay4us();
	SLECLK_H;
	delay12us();
	SLECLK_L;
	delay4us();
	delay4us();
	SLERST_L;
	delay4us();
	while(i--)
	{
		SLECLK_H;
		delay12us();
		SLECLK_L;
		delay12us();
	}
	SLEIO_H;   
}

 /******************************   ****************************************
* @	函数名sle4428_si：
* @	功能说明：sle4428卡串行输入一个字节函数,或说CPU向卡写一个字节
* @ 输入参数1：要写入的数据
* @ 输入参数2：
* @ 输入参数3：
* @	输出参数：
* @ 注意事项：
* @ 版本信息：10-5-28	pqf
* @ 调用示例：
*********************************************************************************************/ 
void sle4428_si(unsigned char dat)       // IC input a byte
{
   unsigned char j=8,i;
  sel4428_IO_ctl(SLEIO_OUT_CTR);
   	   do {
	   		i=dat&0x01;
		  if(i) SLEIO_H;
		  else SLEIO_L;
	      SLECLK_H;
	      delay12us();
	      SLECLK_L;
	      dat>>=1;
		  delay12us();
	   }  while(--j);  
  
}

/******************************   ****************************************
* @	函数名：sle4428_so
* @	功能说明：sle4428卡串行输出一个字节函数,或说CPU从卡读一个字节
* @ 输入参数1：
* @ 输入参数2：
* @ 输入参数3：
* @	输出参数：读到的字节
* @ 注意事项：
* @ 版本信息：10-5-28	pqf
* @ 调用示例：
*********************************************************************************************/
unsigned char sle4428_so(void)           // IC output a byte
{
   unsigned char rbyte=0,i=8,st; 
   sel4428_IO_ctl(SLEIO_IN_CTR); 
	SLEIO_H;
	while(i--)
	{ 
		rbyte>>=1;
		SLECLK_H;
		st=SLEIO_I;
		if (st) rbyte|=0x80;    // 先送低位
		delay4us();
		SLECLK_L;
		delay4us();
	}
	return(rbyte);	 
}


/******************************   ****************************************
* @	函数名：sle4428_command
* @	功能说明： sle4428卡命令模式函数
* @ 输入参数1：命令
* @ 输入参数2：	地址
* @ 输入参数3：数据
* @	输出参数：
* @ 注意事项：
* @ 版本信息：10-5-28	pqf
* @ 调用示例：
*********************************************************************************************/
void sle4428_command(unsigned char control,unsigned int short address,unsigned char dat)
{
   union aa                 // 联合体
   {
     char a[2];
     unsigned int short b;
   } c;  
   sel4428_IO_ctl(SLEIO_OUT_CTR);
		c.b=address;               
		c.a[1]<<=6;
		control|=c.a[1];
		SLERST_H;
		delay12us();               // tRE
		SLECLK_L;
		sle4428_si(control);
		sle4428_si(c.a[0]);
		sle4428_si(dat);
		SLEIO_H;
		delay12us();
		SLERST_L;
		delay12us();
		SLECLK_H;
		delay12us();
		SLECLK_L;
		delay12us();
}


/******************************   ****************************************
* @	函数名：sle4428_rPB
* @	功能说明：连带保护位读sle4428卡一个字节函数
* @ 输入参数1：读数据的地址	  addr=0-127
* @ 输入参数2：
* @ 输入参数3：
* @	输出参数：读到的数据+保护位标志
* @ 注意事项：如果保护位为0x00表示 数据没有被保护 0xFF表示数据被保护
* @ 版本信息：10-5-28	pqf
* @ 调用示例：
*********************************************************************************************/
unsigned int sle4428_rPB(unsigned int addr)              // read a byte with protect bit
{ 
   unsigned int short dat;
   unsigned char i,st;
   sle4428_reset();
   sle4428_command(0x0c,addr,0xff);     // 带保护位读命令
   
   dat=sle4428_so();	   // test 此处需要添加  读写错误函数
	st=SLEIO_I;   
   if (st) i=0;              // 把保护位内容赋予数据符号位
   else i=1;
 
   dat<<=8;
   if(i)
   dat|=0x00FF;
   return(dat);
}
uint8 sle4428_rPB2(uint8 addr,uint8 *dat)
{
	
	sle4428_reset();
   sle4428_command(0x0c,addr,0xff);     // 带保护位读命令
   
   *dat=sle4428_so();	   // test 此处需要添加  读写错误函数
   
   if(SLEIO_I)//无保护
   return 0;
   else return 0xff;
}
/******************************   ****************************************
* @	函数名：sle4428_rdEC
* @	功能说明：读错误计数
* @ 输入参数1：
* @ 输入参数2：
* @ 输入参数3：
* @	输出参数： 错误计数
* @ 注意事项：输出的数据就为错误计数
* @ 版本信息：10-5-28	pqf
* @ 调用示例：
*********************************************************************************************/
unsigned char sle4428_rdEC(void)                     // read EC
{	
	unsigned char temp;
	sle4428_reset();
	sle4428_command(0x0e,1021,0xff);
	temp = sle4428_so();	
	return temp;  
}

/******************************   ****************************************
* @	函数名：sle4428_rd
* @	功能说明：批量读数据
* @ 输入参数1：数据块指针
* @ 输入参数2：数据块大小
* @ 输入参数3：数据块个数
* @ 输入参数4：数据开始地址
* @	输出参数：
* @ 注意事项：
* @ 版本信息：10-5-28	pqf
* @ 调用示例：
*********************************************************************************************/
unsigned char sle4428_rd(void *rdbuf,unsigned int size,unsigned int n,unsigned int addr)
{ 
   unsigned char *dp;
   dp=rdbuf; 
   sle4428_reset();
   sle4428_command(0x0e,addr,0xff);
   return(sle4428_outgoing(dp,size,n));
}

/******************************   ****************************************
* @	函数名：sle4428_outgoing：
* @	功能说明：sle4428卡输出模式函数
* @ 输入参数1：数据指针
* @ 输入参数2：数据块大小
* @ 输入参数3：数据块个数
* @	输出参数： 读出的数据个数
* @ 注意事项： 读出的数据个数同想要读出的数据个数相同则读成功
* @ 版本信息：10-5-28	pqf
* @ 调用示例：
*********************************************************************************************/
unsigned char sle4428_outgoing(unsigned	char *rdbuf,unsigned int size,unsigned int n)
{
   unsigned int i,j=0;
  
   	   while(n--)
	   {
	      i=size;
	      while(i--)
	      {	      
	         *rdbuf++=sle4428_so();
			  j++;
	      }
	     
	   }
	   SLERST_H;
	   return(j);

}

/******************************   ****************************************
* @	函数名：sle4428_programming
* @	功能说明：sle4428卡编程模式函数
* @ 输入参数1：无用参数
* @ 输入参数2：
* @ 输入参数3：
* @	输出参数：
* @ 注意事项：输入参数其实并没有用到
* @ 版本信息：10-5-28	pqf
* @ 调用示例：
********************************************************************************/
int16 sle4428_programming(unsigned char n)     // waiting program
{ 
	unsigned char st;
	uint32 tmp;
		SLEIO_H;
		st=SLEIO_I;
		tmp=systicnum+5;
		while(st)
		{
			SLECLK_H;
			delay12us();  delay4us();
			SLECLK_L;
			delay4us(); delay4us();
			st=SLEIO_I;
			if(systicnum>tmp) return -1;
		}
		SLEIO_H; 
		return 0;  
}

/******************************   ****************************************
* @	函数名：sle4428_comp
* @	功能说明：sle4428卡比较模式函数
* @ 输入参数1：
* @ 输入参数2：
* @ 输入参数3：
* @	输出参数：
* @ 注意事项：
* @ 版本信息：10-5-28	pqf
* @ 调用示例：
*********************************************************************************************/
int16 sle4428_comp(void)
{
	unsigned char st;
	uint32 tmp;
	SLEIO_H;   
	tmp=systicnum+5;
	do 
	{
		delay4us();
		SLECLK_H;
		delay12us();
		SLECLK_L;
		delay4us();
		st=SLEIO_I;
		if(systicnum>tmp)return -1;
	}  while(st);
	return 0;
}

/****************************** sle4428_vPSC  ****************************************
* @	函数名：sle4428_vPSC
* @	功能说明：校验sle4428卡密码函数
* @ 输入参数1：密码数据指针
* @ 输入参数2：
* @ 输入参数3：
* @	输出参数：效验密码之后的密码错误计数
* @ 注意事项：返回 8正确 核对
* @ 版本信息：10-5-28	pqf
* @ 调用示例：
*********************************************************************************************/
int16 sle4428_vPSC(unsigned char *vdbuf)
{ 
   unsigned char wcounter,i=0;
   wcounter=sle4428_rdEC();                 // 读错误计数器
   
   if (wcounter&0x01) wcounter&=0xfe;
   else 
      if (wcounter&0x02) wcounter&=0xfc;
      else 
         if (wcounter&0x04) wcounter&=0xf8;
         else 
            if (wcounter&0x08) wcounter&=0xf0;
            else 
               if (wcounter&0x10) wcounter&=0xe0;
               else 
                  if (wcounter&0x20) wcounter&=0xc0;
                  else 
                     if (wcounter&0x40) wcounter&=0x80;
                     else 
                        wcounter=0x00;


	sle4428_reset();
	sle4428_command(0x32,1021,wcounter);     // 写错误计数器
	if(sle4428_programming(102)==-1) return -1;
	
	sle4428_command(0x0d,1022,vdbuf[0]);     // 比较 PSC byte 1
	if(sle4428_comp()==-1)return -2;
	sle4428_command(0x0d,1023,vdbuf[1]);     // 比较 PSC bye 2
	if(sle4428_comp()==-1)return -3;
	
	sle4428_command(0x33,1021,0xff);         // 擦错误计数器
	if(sle4428_programming(102)==-1) return -4;
	wcounter=sle4428_rdEC();                 // 再读错误计数器
	
	while(wcounter)
	{
		if (wcounter&0x80) i++;   // 把错误计数器的'bit mask'值转换为十进制数 
		wcounter<<=1; 
	}
	return(i);                  // 返回计数(=8,正确,否则错)
}


/******************************   ****************************************
* @	函数名：sle4428_rb
* @	功能说明：写一个数据
* @ 输入参数1：输入地址 addr=0-1020
* @ 输入参数2：
* @ 输入参数3：
* @	输出参数：写入数据的回读
* @ 注意事项：
* @ 版本信息：10-5-28	pqf
* @ 调用示例：
*********************************************************************************************/
unsigned char sle4428_rb(unsigned int addr)
{
   sle4428_reset();
   sle4428_command(0x0e,addr,0xff);
   return(sle4428_so());
}


/****************************** sle4428_wb  ****************************************
* @	函数名sle4428_wb：
* @	功能说明：向sle4428卡写一个字节函数 
* @ 输入参数1：地址  0-1020
* @ 输入参数2：数据 需要写入的数据
* @ 输入参数3：
* @	输出参数： 写入数据之后回读
* @ 注意事项：需要在效验密码正确之后
* @ 版本信息：10-5-28	pqf
* @ 调用示例：
*********************************************************************************************/
int16 sle4428_wb(unsigned int short addr,unsigned char dat) 
{  
   sle4428_reset();
   sle4428_command(0x33,addr,dat);
   if(sle4428_programming(203)==-1) return -1;
   return(sle4428_rb(addr));
}


/******************************   ****************************************
* @	函数名： sle4428_wr
* @	功能说明：向主存储器写入一个数据块数据
* @ 输入参数1：存数据块指针
* @ 输入参数2：数据块大小
* @ 输入参数3：数据块个数
* @	输出参数：写入的正确的
* @ 注意事项：若返回的数据同写入的数据个数 写入成功
* @ 版本信息：10-5-28
* @ 调用示例：
*********************************************************************************************/
int16 sle4428_wr(unsigned char *wrbuf,unsigned int short size,unsigned int short n,unsigned int  short addr)
{ 
   unsigned char *dp;
   unsigned int short i,j=0;
   dp=wrbuf;
   sle4428_reset();
   while(n--)
   {
      i=size;
      while(i--)
      {
         if (sle4428_wb(addr,*dp)!=*dp) 		 return-1;  // 写不确返回
         addr++;
         dp++; 
		 j++ ;
      }
   
   }
   return(j);                                      // j=n,正确,否则错
}


/******************************   ****************************************
* @	函数名：sle4428_wPB
* @	功能说明： 连带保护位向sle4428卡写一个字节函数
* @ 输入参数1：地址   0-127
* @ 输入参数2：要写入的位置
* @ 输入参数3：
* @	输出参数：读保护位
* @ 注意事项：需要在保护位没有写保护且该位已经写入相应数据的条件下才能写保护，
			  比如有位未写保护，且为0x55，必须写入0x55才能成功
* @ 版本信息：10-5-28 pqf
* @ 调用示例：
*********************************************************************************************/
int sle4428_wPB(unsigned int short addr,unsigned char dat) 
{
   sle4428_reset();
   sle4428_command(0x30,addr,dat);      // 比较命令:正确则写,否则不写
   if(sle4428_comp()==-1)return -1;
   return(sle4428_rPB(addr));
}

// 修改sle4428卡密码函数, 输入形参为存放密码数的指针  0 成功 1 失败  // 
/******************************   ****************************************
* @	函数名：sle4428_uPSC
* @	功能说明： 修改sle4428卡密码函数
* @ 输入参数1：输入的密码
* @ 输入参数2：
* @ 输入参数3：
* @	输出参数： 失败 1  成功 0
* @ 注意事项：必须验证密码之后效验
* @ 版本信息：10-5-28 pqf
* @ 调用示例：
*********************************************************************************************/
int16 sle4428_uPSC(unsigned		char *scbuf)
{                                         
  
   sle4428_reset();
   sle4428_command(0x33,1022,scbuf[0]);
   if(sle4428_programming(203)==-1) return -1;
   sle4428_command(0x33,1023,scbuf[1]);
   if(sle4428_programming(203)==-1) return -1;
   if (sle4428_vPSC(scbuf)==8) return 0;
   else return 1;
}  
 /******************************   ****************************************
* @	函数名：WM_4428_Protec_Read
* @	功能说明：带保护位读数据
* @ 输入参数1：数据首指针
* @ 输入参数2：地址
* @ 输入参数3：需要读取的长度
* @	输出参数：0
* @ 注意事项：int 型需要声明为short
* @ 版本信息：10-5-28 pqf
* @ 调用示例：
*********************************************************************************************/
unsigned char WM_4428_Protec_Read(unsigned int short *buffer,unsigned int short addr,unsigned int short num)
{	
	unsigned int i;
	for(i=0;i<num;i++)
	{
		*buffer++=sle4428_rPB(addr++); 		
	}
	return 0;
}
/******************************   ****************************************
* @	函数名： WM_4428_Check_Password
* @	功能说明：效验密码
* @ 输入参数1：密码首指针
* @ 输入参数2：
* @ 输入参数3：
* @	输出参数：失败 1  成功 0
* @ 注意事项：
* @ 版本信息：10-5-28 pqf
* @ 调用示例：
*********************************************************************************************/
unsigned char WM_4428_Check_Password(unsigned char *pwd)
{
	if ( sle4428_vPSC(pwd)==0x08 )//=8效验正确
	return 0x00;
	else return 0x01;
}
/******************************   ****************************************
* @	函数名：MW_Read_Err_Counter
* @	功能说明：读取卡片错误计数
* @ 输入参数1：
* @ 输入参数2：
* @ 输入参数3：
* @	输出参数：卡片的错误计数
* @ 注意事项：返回值做大为7
* @ 版本信息：10-5-28 pqf
* @ 调用示例：
*********************************************************************************************/
unsigned char MW_4428_Read_Err_Counter(void)
{
	unsigned char temp,i,pig=0x01,num=0;
	temp = sle4428_rdEC();
	for(i=0;i<8;i++)
	{
		if(temp&pig)
		num++;
		pig<<=1;
	}
  return num;
}
unsigned char MW_4428_ReadHex_Err_Counter(void)
{
	unsigned char temp;
	temp = sle4428_rdEC();
  return temp;
}
/******************************   ****************************************
* @	函数名：MW_Write_Password
* @	功能说明：更改密码
* @ 输入参数1：输入密码数组指针
* @ 输入参数2：
* @ 输入参数3：
* @	输出参数：失败1  成功0
* @ 注意事项：
* @ 版本信息：10-5-28 pqf
* @ 调用示例：
*********************************************************************************************/
unsigned char MW_4428_Write_Password(unsigned char *pwd)
{
   	if( sle4428_uPSC(pwd) )
	return 1;
	else return 0;
}
/******************************   ****************************************
* @	函数名：MW_Write
* @	功能说明：写数据
* @ 输入参数1：数据首指针
* @ 输入参数2：首地址
* @ 输入参数3： 需要写入的数据字节长度
* @	输出参数：成功 0 已经有数据 1 写锁定 2写数据失败
* @ 注意事项：
* @ 版本信息：10-5-28 pqf
* @ 调用示例：
*********************************************************************************************/
unsigned char MW_4428_Write(unsigned char *buffer,unsigned int short addr,unsigned int short num)
{
	unsigned int short addr_temp,i,temp;
	addr_temp=addr;
	for(i=0;i<num;i++){
		temp = sle4428_rPB(addr_temp++);
		if((temp&0x00FF)==0x00FF)  return 1;//已经写保护
	}

	i=sle4428_wr(buffer,num,1,addr);   //i中存放的是写操作错误的数据的编号
	if(i==num)return 0;
	else return 2;
}
/******************************   ****************************************
* @	函数名：MW_4428_Protect_Write
* @	功能说明：带保护位写操作
* @ 输入参数1：数据首指针
* @ 输入参数2：地址
* @ 输入参数3：写入的字节数
* @	输出参数：已经写保护 1（不执行操作返回）   写数据失败2   写保护位失败  3     成功 0
* @ 注意事项：若要写入的数据中有一个已经写保护了则终止操作
* @ 版本信息：10-5-28 pqf
* @ 调用示例：
*********************************************************************************************/
unsigned char MW_4428_Protect_Write(unsigned char *buffer,unsigned int short addr,unsigned int short num)
{
	unsigned int short addr_temp,i,temp;
	addr_temp = addr;
 	for(i=0;i<num;i++){
	temp = sle4428_rPB(addr_temp++);
	if((temp&0x00FF)==0x00FF)  return 1;//已经写保护
	}
	i=sle4428_wr(buffer,num,1, addr);
	if(i==num)
	{  for(i=0;i<num;i++)
		{
			sle4428_wPB(addr,*buffer++) ;  // 连带保护位向sle4428卡写一个字节函数 // 
			temp=sle4428_rPB(addr++);     // read a byte with protect bit
			if((temp&&0x00FF)==0x00 )return 3;		//写保护位失败
	  	}
	}else return 2;//写数据失败

	return 0;
}

/******************************   ****************************************
* @	函数名：MW_4428_Protec_Bit
* @	功能说明：保护4428卡的一个位
* @ 输入参数1：地址
* @ 输入参数2：需要保护的位数
* @ 输入参数3：
* @	输出参数：
* @ 注意事项：如果有受保护位则 返回 1  写保护位失败 2  成功 0
* @ 版本信息：10-5-28 pqf
* @ 调用示例：
*********************************************************************************************/
unsigned char MW_4428_Protec_Bit(unsigned int short addr,unsigned int short num)
{
	unsigned int short  addr_temp,i,temp;
	unsigned char buffer[128];
	addr_temp=addr;
	for(i=0;i<num;i++)
	{
		temp=sle4428_rPB(addr_temp++);    
		if((temp&0x00FF)==0x00FF)  return 1;
	}

	 sle4428_rd(buffer,num,1,addr);
	 	addr_temp=addr;
	 for(i=0;i<num;i++)
	{
			sle4428_wPB(addr_temp,buffer[i]);  // 连带保护位向sle4428卡写一个字节函数 // 
			temp=sle4428_rPB(addr_temp++);     // read a byte with protect bit
			if((temp&&0x00FF)==0x00 )return 2;		//写保护位失败
	 }
	return 0;

}
unsigned char MW_4428_Read(unsigned char *buffer,unsigned int short addr,unsigned int short num)
{
	unsigned int short temp	;
	temp=sle4428_rd(buffer,num,1,addr);
	if(temp==num)
	return 0;
	else return 1;
	 
}




int16 Read_4428_With_PB(uint16 StartPos,uint16 NOB,char *Bfr,char *PB_Bfr)
{
	uint16 n,i;
	uint8 st;
	sle4428_reset();
   sle4428_command(0x0c,StartPos,0xff);     // 带保护位读命令
   for(i=0;i<NOB;i++)
   {
   		Bfr[i]=sle4428_so();
   		SLECLK_H;
			st=SLEIO_I;
			if (SLEIO_I) PB_Bfr[i]=0x01;//0x00;//无写保护
			else PB_Bfr[i]=0x00;//0xff;//写保护
			delay4us();
			SLECLK_L;
			delay4us();
   }
   return 0;
}
int16 Read_4428_NO_PB(uint16 StartPos,uint16 NOB,char *Bfr)
{
	//	unsigned char *dp;
  // dp=rdbuf; 
   sle4428_reset();
   sle4428_command(0x0e,StartPos,0xff);
   sle4428_outgoing(Bfr,1,NOB);
   return 0;
}
int16 Write_4428(uint16 StartPos,char DestByte,char PBSetFlag)
{
	unsigned int short addr_temp,i,temp;
      int16 j;
	
	if(PBSetFlag==0)
	{
		temp = sle4428_rPB(StartPos);
		if((temp&0x00FF)==0x00FF)  return -2;//已经写保护
		i=sle4428_wr(&DestByte,1,1,StartPos);   //i中存放的是写操作错误的数据的编号
		if(i==1)return 0;
		else return -1;
	}
	else//带保护位写函数
	{
	temp = sle4428_rPB(StartPos);
	if((temp&0x00FF)==0x00FF)  return 1;//已经写保护
	
	i=sle4428_wr(&DestByte,1,1, StartPos);
	if(i==1)
	{  
			j=sle4428_wPB(StartPos,DestByte) ;  // 连带保护位向sle4428卡写一个字节函数 // 
			if(j==-1) return -1;
			temp=sle4428_rPB(StartPos);     // read a byte with protect bit
			if((temp&&0x00FF)==0x00 )return -3;		//写保护位失败
	  	
	}else return -1;//写数据失败

	return 0;
	}
	

}
int16 Verify_4428_PSC(char PSC1,char PSC2)
{
	unsigned char wcounter,i=0;
   wcounter=sle4428_rdEC();                 // 读错误计数器
   
   if (wcounter&0x01) wcounter&=0xfe;
   else 
      if (wcounter&0x02) wcounter&=0xfc;
      else 
         if (wcounter&0x04) wcounter&=0xf8;
         else 
            if (wcounter&0x08) wcounter&=0xf0;
            else 
               if (wcounter&0x10) wcounter&=0xe0;
               else 
                  if (wcounter&0x20) wcounter&=0xc0;
                  else 
                     if (wcounter&0x40) wcounter&=0x80;
                     else 
                        wcounter=0x00;


	sle4428_reset();
	sle4428_command(0x32,1021,wcounter);     // 写错误计数器
	if(sle4428_programming(102)==-1) return -1;
	
	sle4428_command(0x0d,1022,PSC1);     // 比较 PSC byte 1
	if(sle4428_comp()==-1)return -2;
	sle4428_command(0x0d,1023,PSC2);     // 比较 PSC bye 2
	if(sle4428_comp()==-1)return -3;
	
	sle4428_command(0x33,1021,0xff);         // 擦错误计数器
	if(sle4428_programming(102)==-1) return -4;
	wcounter=sle4428_rdEC();                 // 再读错误计数器
	
	while(wcounter)
	{
		if (wcounter&0x80) i++;   // 把错误计数器的'bit mask'值转换为十进制数 
		wcounter<<=1; 
	}
	if(i==8)return 0;
	else return i;
}
int16 Read_4428_SM(char *SM_Bfr,char *SM_PB_Bfr)
{
	uint16 n,i;
	uint8 st;
	sle4428_reset();
   sle4428_command(0x0c,1021,0xff);     // 带保护位读命令
   for(i=0;i<3;i++)
   {
   		SM_Bfr[i]=sle4428_so();
   		SLECLK_H;
			st=SLEIO_I;
			if (SLEIO_I) SM_PB_Bfr[i]=0x01;//0x00;//无写保护
			else SM_PB_Bfr[i]=0x00;//0xff;//写保护
			delay4us();
			SLECLK_L;
			delay4us();
   }
   return 0;
}
//-------------------------------------
/******************************   ****************************************
* @	函数名：sle4428_rPB
* @	功能说明：连带保护位读sle4428卡一个字节函数
* @ 输入参数1：读数据的地址	  addr=0-127
* @ 输入参数2：
* @ 输入参数3：
* @	输出参数：读到的数据+保护位标志
* @ 注意事项：如果保护位为0xff表示 数据没有被保护 0x00表示数据被保护
* @ 版本信息：10-5-28	pqf
* @ 调用示例：
*********************************************************************************************/
unsigned int HCsle4428_rPB(unsigned int addr)              // read a byte with protect bit
{ 
   unsigned int short dat;
   unsigned char i,st;
   sle4428_reset();
   sle4428_command(0x0c,addr,0xff);     // 带保护位读命令
   
   dat=sle4428_so();	   // test 此处需要添加  读写错误函数
	st=SLEIO_I;   
   if (st) i=1;              // 把保护位内容赋予数据符号位
   else i=0;
 
   dat<<=8;
   if(i)
   dat|=0x00FF;
   return(dat);
}
int16 HCRead_4428_NO_PB(uint16 StartPos,uint16 NOB,char *Bfr)
{
	//	unsigned char *dp;
  // dp=rdbuf; 
   sle4428_reset();
   sle4428_command(0x0e,StartPos,0xff);
   sle4428_outgoing(Bfr,1,NOB);
   return 0;
}
int16 HCRead_4428_With_PB(uint16 StartPos,uint16 NOB,char *Bfr)
{
	uint16 n,i;
	uint8 st;
	sle4428_reset();
   sle4428_command(0x0c,StartPos,0xff);     // 带保护位读命令
   for(i=0;i<NOB;)
   {
   		Bfr[i*2]=sle4428_so();
   		SLECLK_H;
			st=SLEIO_I;
			if (SLEIO_I) Bfr[i*2+1]=0xff;//0x00;//无写保护
			else Bfr[i*2+1]=0x00;//写保护
			delay4us();
			SLECLK_L;
			delay4us();
			i+=1;
   }
   return 0;
}
unsigned char HCWrite_4428_ONLY_DAT(unsigned short StartPos,unsigned short num,unsigned char *buffer)
{
	unsigned  short addr_temp,i,temp;
	addr_temp=StartPos;
	for(i=0;i<num;i++){
		temp = HCsle4428_rPB(addr_temp++);
		if((temp&0x00FF)==0x0000)  return 1;//已经写保护
	}

	i=sle4428_wr(buffer,num,1,StartPos);   //i中存放的是写操作错误的数据的编号
	if(i==num)return 0;
	else return 2;
}
int16 HCWrite_4428_Byte(uint16 StartPos,char DestByte,char PBSetFlag)
{
	unsigned int short addr_temp,i,temp;
      int16 j;
	
	if(PBSetFlag==0)
	{
		temp = HCsle4428_rPB(StartPos);
		if((temp&0x00FF)==0x0000)  return -2;//已经写保护
		i=sle4428_wr(&DestByte,1,1,StartPos);   //i中存放的是写操作错误的数据的编号
		if(i==1)return 0;
		else return -1;
	}
	else//带保护位写函数
	{
	temp = HCsle4428_rPB(StartPos);
	if((temp&0x00FF)==0x0000)  return -2;//已经写保护
	
	i=sle4428_wr(&DestByte,1,1, StartPos);
	if(i==1)
	{  
			j=sle4428_wPB(StartPos,DestByte) ;  // 连带保护位向sle4428卡写一个字节函数 // 
			if(j==-1) return -1;
			temp=HCsle4428_rPB(StartPos);     // read a byte with protect bit
			if((temp&&0x00FF)==0xff )return -3;		//写保护位失败
	  	
	}else return -1;//写数据失败

	return 0;
	}
}
unsigned char  HCWrite_4428_With_PB(unsigned short StartPos,unsigned short num,unsigned char *buffer)
{
	unsigned  short addr_temp,i,temp;
	int16 t;
	addr_temp=StartPos;
	for(i=0;i<num;){
		
		t=HCWrite_4428_Byte(addr_temp++,*buffer++,1);
		if(t!=0) return 1;
			i+=1;
	}
	return 0;
}
int16 HCRead_4428_DAT_PB(uint16 StartPos,uint16 NOB,char *Bfr,char *PB_Bfr)
{
	uint16 n,i;
	uint8 st;
	sle4428_reset();
   sle4428_command(0x0c,StartPos,0xff);     // 带保护位读命令
   for(i=0;i<NOB;i++)
   {
   		Bfr[i]=sle4428_so();
   		SLECLK_H;
			st=SLEIO_I;
			if (SLEIO_I) PB_Bfr[i]=0xff;//无写保护
			else PB_Bfr[i]=0x00;//写保护
			delay4us();
			SLECLK_L;
			delay4us();
   }
   return 0;
}
//比对是否数据一致，加入保护
//固话数据
//返回实际写入正确的字节个数
uint16 HCWrite_4428_Only_PB(unsigned short StartPos,unsigned short num,unsigned char *buffer)
{
	uint16 n,i;
	uint8 st,Bfr,PB_Bfr;
	int16 t;
	sle4428_reset();
	
	
	for(n=0;n<num;)
	{
	sle4428_command(0x0c,StartPos+n,0xff);     // 带保护位读命令
	Bfr=sle4428_so();
  SLECLK_H;
	st=SLEIO_I;
	if (SLEIO_I) PB_Bfr=0xff;//无写保护
	else PB_Bfr=0x00;//写保护
	delay4us();
	SLECLK_L;
	delay4us();
	//比较
	if(Bfr!=buffer[n]) return n;
	if(PB_Bfr==0xff)//无保护，则加
	{
		t=HCWrite_4428_Byte(StartPos+n,buffer[n],1);
		if(t!=0) return 1;
	}
	n+=1;
	}
	return n;
	
}
//读错误计数，密码，以及保护位
int16 HCRead_4428_SM(char *SM_Bfr,char *SM_PB_Bfr)
{
	uint16 n,i;
	uint8 st;
	sle4428_reset();
   sle4428_command(0x0c,1021,0xff);     // 带保护位读命令
   for(i=0;i<3;i++)
   {
   		SM_Bfr[i]=sle4428_so();
   		SLECLK_H;
			st=SLEIO_I;
			if (SLEIO_I) SM_PB_Bfr[i]=0xff;//0x00;//无写保护
			else SM_PB_Bfr[i]=0x00;//0xff;//写保护
			delay4us();
			SLECLK_L;
			delay4us();
   }
   return 0;
}
int16 HCWrite_4428_EC(char wcounter)
{
	sle4428_reset();
	sle4428_command(0x32,1021,wcounter);     // 写错误计数器
	if(sle4428_programming(102)==-1) return -1;
	else
		return 0;
}
//核对密码
int16 HCVerify_4428_PSC(char PSC1,char PSC2)
{
	unsigned char wcounter,i=0;
   wcounter=sle4428_rdEC();                 // 读错误计数器
   
   if (wcounter&0x01) wcounter&=0xfe;
   else 
      if (wcounter&0x02) wcounter&=0xfc;
      else 
         if (wcounter&0x04) wcounter&=0xf8;
         else 
            if (wcounter&0x08) wcounter&=0xf0;
            else 
               if (wcounter&0x10) wcounter&=0xe0;
               else 
                  if (wcounter&0x20) wcounter&=0xc0;
                  else 
                     if (wcounter&0x40) wcounter&=0x80;
                     else 
                        wcounter=0x00;


	sle4428_reset();
	sle4428_command(0x32,1021,wcounter);     // 写错误计数器
	if(sle4428_programming(102)==-1) return -1;
	
	sle4428_command(0x0d,1022,PSC1);     // 比较 PSC byte 1
	if(sle4428_comp()==-1)return -2;
	sle4428_command(0x0d,1023,PSC2);     // 比较 PSC bye 2
	if(sle4428_comp()==-1)return -3;
	
	sle4428_command(0x33,1021,0xff);         // 擦错误计数器
	if(sle4428_programming(102)==-1) return -4;
	wcounter=sle4428_rdEC();                 // 再读错误计数器
	
	while(wcounter)
	{
		if (wcounter&0x80) i++;   // 把错误计数器的'bit mask'值转换为十进制数 
		wcounter<<=1; 
	}
	if(i==8)return 0;
	else return i;
}