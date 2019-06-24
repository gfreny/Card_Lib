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
extern volatile uint32 	systicnum;
extern volatile uint8 	Card_PowerStatus;//0为未上电，1为上电
unsigned char sle4442_so(void);
unsigned int sle4442_outgoing(unsigned char *rdbuf,unsigned char size,unsigned char n);

int16 Read_4442_NO_PB(uint16 StartPos,uint16 NOB,uint8 *Bfr);
int16 Read_4442_PB(char *PB_Bfr);
int16 Write_4442(uint16 StartPos,char DestByte,char PBSetFlag);
int16 Verify_4442_PSC(char PSC1,char PSC2,char PSC3);
int16 Read_4442_SM(char *SM_Bfr);
int16 Write_4442_SM(uint16 SMAddress,char SMByte);

void sel4442_IO_ctl(uint8 iotype)
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
void sle4442GPIO_Ini()
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
	//5V
	SC1_35VH;//结合CMD进入停止模式
	
	SLECLK_H;
	SLEIO_H;
	SLERST_H;
	SC1_CMDVCCL; Card_PowerStatus=1;//上电
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

/******************************  sle4442_reset ****************************************
* @	函数名：sle4442_reset
* @	功能说明：sle4442卡复位
* @ 输入参数1：
* @ 输入参数2：
* @ 输入参数3：
* @	输出参数：
* @ 注意事项：
* @ 版本信息：10.5.21 pqf
* @ 调用示例：
*********************************************************************************************/
unsigned char sle4442_readrstbyte(void)
{
	uint8 tmp,i;
	tmp=0;
	for(i=0;i<8;i++)
	{
		tmp=((tmp>>1)&0x7f);
		tmp=tmp+(SLEIO_I?0x80:0);
    delay12us();
		SLECLK_H;
		//读取数据
		delay12us();
     SLECLK_L;
     delay12us(); 	
	}
	return tmp;		
}
unsigned char sle4442_reset(uint8 *rst)           // init reset
{
   unsigned char i=32; 
   //sel4442_IO_ctl(SLEIO_IN_CTR);
   if(GPIO_ReadInputDataBit(DS1_OFF_PORT,DS1_OFF_PIN)==0)
{	SC1_CMDVCCH;GPIO_SetBits(DS1_35V_PORT, DS1_35V_PIN);
	 SC_Delay();
	 }
   SC1_CMDVCCL;Card_PowerStatus=1;//上电
sel4442_IO_ctl(SLEIO_OUT_CTR);
SLEIO_L;
   SLECLK_L;
 
   SLERST_L;

   delay12us();
   SLERST_H;
 
   delay4us();
   SLECLK_H;
   delay12us();
   SLECLK_L;
   delay4us();
   SLERST_L;
   sel4442_IO_ctl(SLEIO_IN_CTR);
   delay12us();

   for(i=0;i<4;i++)
	 rst[i]=sle4442_readrstbyte();//sle4442_readrstbyte();
   
   sel4442_IO_ctl(SLEIO_OUT_CTR);
   SLEIO_H;
   return 0;
}
/******************************  sle4442_si ****************************************
* @	函数名：sle4442_si
* @	功能说明：sle4442卡串行输入一个字节函数,或说CPU向卡写一个字节
* @ 输入参数1：要读出数据的地址
* @ 输入参数2：
* @ 输入参数3：
* @	输出参数：
* @ 注意事项：
* @ 版本信息：10.5.21 pqf
* @ 调用示例：
*********************************************************************************************/
unsigned char sle4442_si(unsigned char dat)      // write a byte
{
   unsigned char i=8;
   while(i)
   {
      //SLEIO=(bit)(dat&0x01);     // 先写入字节的低位
	  if(dat&0x01) SLEIO_H;
	  else SLEIO_L;  
	  delay4us();//test
      SLECLK_H;
      delay4us();
      SLECLK_L;
      dat>>=1;                   // 右移一位
      i--;
	  delay4us();
   }
   return 0;
}
/******************************  sle4442_so ****************************************
* @	函数名：sle4442_so
* @	功能说明：sle4442卡串行输出一个字节函数,或说CPU从卡读一个字节
* @ 输入参数1：
* @ 输入参数2：
* @ 输入参数3：
* @	输出参数：
* @ 注意事项：
* @ 版本信息：10.5.21 pqf
* @ 调用示例：
*********************************************************************************************/
unsigned char sle4442_so(void)           // read a byte
{
   unsigned char i=8;
   unsigned char rbyte=0;
   //sel4442_IO_ctl(SLEIO_IN_CTR);
   SLEIO_H;
   while(i)
   {
      rbyte>>=1;                // 接收字节右移1位,D7=0
      SLECLK_H;
	  

      if (SLEIO_I) rbyte|=0x80; // 先输出低位 
	  delay4us();
      
      SLECLK_L;
      i--;
	  delay4us();
   }
  
   return rbyte;               // 返回读到的字节 
}
unsigned char sle4442_soDir(void)           // read a byte
{
   unsigned char i=8;
   unsigned char rbyte=0;
   //sel4442_IO_ctl(SLEIO_IN_CTR);
   SLEIO_H;
   while(i)
   {
      rbyte<<=1;                // 接收字节右移1位,D7=0
      SLECLK_H;
	  rbyte&=0xfe;

      if (SLEIO_I) rbyte|=0x01; // 先输出低位 
	  delay4us();
      
      SLECLK_L;
      i--;
	  delay4us();
   }
  
   return rbyte;               // 返回读到的字节 
}
void sle4442_command(unsigned char control,unsigned char address,unsigned char dat)
{
	 sel4442_IO_ctl(SLEIO_OUT_CTR);
   SLECLK_H;
   SLEIO_H;                    // 
   delay12us();
   SLEIO_L;                    // Start from IFD
   delay4us();                 // t3
   SLECLK_L;
   delay4us();
   sle4442_si(control);
   sle4442_si(address);
   sle4442_si(dat);
   delay4us();
   SLEIO_L;
   delay4us();                 // t6
   SLECLK_H;
   delay4us();
   SLEIO_H;                    // Stop from IFD
}
int16 sle4442_process(void)     // waiting deal
{
	uint32 tmp;
 	Delayus(2);    
   SLECLK_L;
   sel4442_IO_ctl(SLEIO_IN_CTR);
	SLEIO_H;
   delay4us();                 // t8
   SLECLK_H;
   delay4us(); 
	SLECLK_L;
	tmp=systicnum+50;
   //SLEIO_H;
   do {
    	delay4us();
      	SLECLK_H;
      	delay4us();
      	SLECLK_L;
		if(systicnum>tmp) return -1;    	  
   }  while(!SLEIO_I);
   //while(!ATIN&&!SLEIO);    // 无卡或SLEIO=1退出
   SLEIO_H;
   delay4us();  
   return 0;    
}
/******************************  sle4442_outgoing ****************************************
* @	函数名：sle4442_outgoing
* @	功能说明： sle4442卡输出模式函数  
* @ 输入参数1：数据组指针
* @ 输入参数2：数据块大小
* @ 输入参数3：数据块个数
* @	输出参数：
* @ 注意事项：
* @ 版本信息：2010.5.21 pqf
* @ 调用示例：
*********************************************************************************************/
unsigned int sle4442_outgoing(unsigned char *rdbuf,unsigned char size,unsigned char n)
{
   unsigned char i,j=0;
   
   SLECLK_L;
   sel4442_IO_ctl(SLEIO_IN_CTR);
   SLEIO_H;
   delay4us();                             // t7
   while(n--)
   {
      i=size;
      while(i--)
      {
         //if (ATIN) return(j);        // 若抽卡,返回

         *rdbuf++=sle4442_so();
	 
      }
      j++;
   }
   SLEIO_H;
   return 0;//(j);          
}
unsigned int sle4442_outgoingshort(unsigned char *rdbuf,unsigned short size,unsigned short n)
{
   unsigned short i,j=0;
   
   SLECLK_L;
   sel4442_IO_ctl(SLEIO_IN_CTR);
   SLEIO_H;
   delay4us();                             // t7
   while(n--)
   {
      i=size;
      while(i--)
      {
         //if (ATIN) return(j);        // 若抽卡,返回

         *rdbuf++=sle4442_so();
	 
      }
      j++;
   }
   SLEIO_H;
   return 0;//(j);          
}
unsigned int sle4442_outgoingDir(unsigned char *rdbuf,unsigned char size,unsigned char n)
{
   unsigned char i,j=0;
   
   SLECLK_L;
   sel4442_IO_ctl(SLEIO_IN_CTR);
   SLEIO_H;
   delay4us();                             // t7
   while(n--)
   {
      i=size;
      while(i--)
      {
         //if (ATIN) return(j);        // 若抽卡,返回

         *rdbuf++=sle4442_soDir();
	 
      }
      j++;
   }
   SLEIO_H;
   return 0;//(j);          
}
unsigned int sle4442_outgoingDirshort(unsigned char *rdbuf,unsigned short size,unsigned short n)
{
   unsigned short i,j=0;
   
   SLECLK_L;
   sel4442_IO_ctl(SLEIO_IN_CTR);
   SLEIO_H;
   delay4us();                             // t7
   while(n--)
   {
      i=size;
      while(i--)
      {
         //if (ATIN) return(j);        // 若抽卡,返回

         *rdbuf++=sle4442_soDir();
	 
      }
      j++;
   }
   SLEIO_H;
   return 0;//(j);          
}
/******************************  sle4442_rb ****************************************
* @	函数名：sle4442_rb
* @	功能说明从sle4442卡主存贮器读一个字节函数：
* @ 输入参数1：卡地址
* @ 输入参数2：
* @ 输入参数3：
* @	输出参数：
* @ 注意事项：
* @ 版本信息：10.5.21 pqf
* @ 调用示例：
*********************************************************************************************/
signed int sle4442_rb(unsigned char addr)           // read a byte from main memory
{ 
//   if (!sle4442_reset()) return(-1);
   sle4442_command(0x30,addr,0xff);   // 卡主存贮器的读命令
   sel4442_IO_ctl(SLEIO_IN_CTR);
   SLECLK_H;
   delay4us();
   SLECLK_L;
   delay4us();
   return(sle4442_so());              // 卡串行输出一个字节
} 
/******************************  sle4442_rdEC ****************************************
* @	函数名：sle4442_rdEC
* @	功能说明：读sle4442卡错误计数器函数 
* @ 输入参数1：卡地址
* @ 输入参数2：
* @ 输入参数3：
* @	输出参数：
* @ 注意事项：
* @ 版本信息：10.5.21 pqf
* @ 调用示例：
*********************************************************************************************/
char sle4442_rdEC(uint8 *ec)            // read EC
{
	//uint8 *dp;
	//dp=ec;
	uint8 buf[4];
	sle4442_reset(&buf[0]);
   sle4442_command(0x31,0,0);      // 读卡错误计数器的命令 
   sel4442_IO_ctl(SLEIO_IN_CTR);
   SLECLK_H;
   delay4us();
   SLECLK_L;
   delay4us(); 
   return(sle4442_outgoing(ec,4,1)); 
   //return(sle4442_so());
}
 /****************************** sle4442_wb  ****************************************
* @	函数名：sle4442_wb
* @	功能说明：写数据
* @ 输入参数1：地址
* @ 输入参数2：数据
* @ 输入参数3：
* @	输出参数：
* @ 注意事项：
* @ 版本信息：2010.5.21 pqf
* @ 调用示例：
*********************************************************************************************/
/*signed int sle4442_wb(unsigned char addr,char dat)  // write a byte to main memory
{
   sle4442_command(0x38,addr,dat);        // 卡主存贮器的写命令
   sle4442_process();
   return(sle4442_rb(addr));              // 返回写入后重读出的内容供检查
}*/
/**
不带保护位的读主存
**/
void BreakOperate(void)
{
	uint8 i;
  SLECLK_L;
  delay4us();
  SLERST_H;
  for(i=0;i<50;i++);
  SLERST_L;
}
int16 Read_4442_NO_PB(uint16 StartPos,uint16 NOB,uint8 *Bfr)
{
	uint8 buf[4];
	sle4442_reset(&buf[0]);
	sle4442_command(0x30,StartPos,0xff);      // 读命令
   
   sle4442_outgoing(Bfr,1,NOB);  // 卡输出
   BreakOperate();
   return 0;
}
/*int16 Read_4442_NO_PB(uint16 StartPos,uint16 NOB,uint8 *Bfr)
{
	uint8 *dp;
	dp=Bfr;
	sle4442_command(0x30,StartPos,0xff);      // 读命令
   
   return(sle4442_outgoing(dp,1,NOB));  // 卡输出
}*/
/**
校验3BYTE的PSC
**/
int16 Verify_4442_PSC(char PSC1,char PSC2,char PSC3)
{
	uint8 temp[4];
	uint8 i;
	sle4442_rdEC(&temp[0]);
	if((temp[0]&0x07)!=0)                 	//第一个字节是错误记数，则直接退出
	{
		if((temp[0]&0x07)==0x07)               	//00000111
          i=0x06;
      else if((temp[0]&0x07)==0x06)    	//00000110
          i=0x04;
			else if((temp[0]&0x07)==0x04)           	//00000100
          i=0x00;
          
     sle4442_command(0x39,0,i);             // 去一个错误计数器位 
     if(sle4442_process()==-1)return -1;                     // 写卡等候  
     
     temp[1]=PSC1;
     temp[2]=PSC2;
     temp[3]=PSC3;
      for(i=1;i<4;i++)            
   	{
      sle4442_command(0x33,i,temp[i]);   // 校验密码的命令
      if(sle4442_process()==-1)return -2;   
   	} 
   	sle4442_command(0x39,0,0xff);          // 擦除错误计数器
   	if(sle4442_process()==-1)return -3; 
   	
   	sle4442_rdEC(&temp[0]);
   	if((temp[0]&0x07)==0x07) return 0;//成功
     
	}
	return -1;			
}

/*int16 Write_4442(uint16 StartPos,char DestByte,char PBSetFlag)
{
		uint8 i,buf[2];
		uint8 buff[4];
		sle4442_reset(&buff[0]);
		if(StartPos>=32)
		{
			sle4442_command(0x38,StartPos,DestByte);        // 卡主存贮器的写命令
   		if(sle4442_process()==-1)return -1; 

		for(i=0;i<250;i++)delay12us();

		sle4442_command(0x30,StartPos,0xff); 
		sle4442_outgoing(&i,1,1);
		BreakOperate();
   		//i=sle4442_rb(StartPos);
   		if(i==DestByte)return 0;
   			else return -1;
			}
			else
			{
					sle4442_command(0x38,StartPos,DestByte);        // 卡主存贮器的写命令
   		if(sle4442_process()==-1)return -1; 
   		sle4442_command(0x30,StartPos,0xff); 
		sle4442_outgoing(&i,1,1);
		BreakOperate();
   		if(i!=DestByte)return -1;
   			if(PBSetFlag==1)
			{
			 sle4442_command(0x3c,StartPos,DestByte);         // 保护存贮器的写命令
      	if(sle4442_process()==-1)return -1; 
			}
   			
				}
	
   return 0;              // 返回写入后重读出的内容供检查
}*/
int16 Write_4442(uint16 StartPos,char DestByte,char PBSetFlag)
{
		uint8 i,buf[2];
		uint8 buff[4];
		sle4442_reset(&buff[0]);
		if(StartPos>=32)
		{
			sle4442_command(0x38,StartPos,DestByte);        // 卡主存贮器的写命令
   		if(sle4442_process()==-1)return -1; 

		for(i=0;i<250;i++)delay12us();

		sle4442_command(0x30,StartPos,0xff); 
		sle4442_outgoing(&i,1,1);
		BreakOperate();
   		if(i==DestByte)return 0;
   			else return -1;
			}
			else
			{
					sle4442_command(0x38,StartPos,DestByte);        // 卡主存贮器的写命令
   		if(sle4442_process()==-1)return -1; 
   		sle4442_command(0x30,StartPos,0xff); 
		sle4442_outgoing(&i,1,1);
		BreakOperate();
   		if(i!=DestByte)return -1;
   			if(PBSetFlag==1)
			{
			 sle4442_command(0x3c,StartPos,DestByte);         // 保护存贮器的写命令
      	if(sle4442_process()==-1)return -1; 
			}
   			
				}
	
   return 0;              // 返回写入后重读出的内容供检查
}
int16 Read_4442_PB(char *PB_Bfr)
{
	uint8 buf[4];
	sle4442_reset(&buf[0]);
	sle4442_command(0x34,0xFF,0xFF);   // 读保护位的命令
	return(sle4442_outgoingDir(PB_Bfr,4,1));
}
//--读安全寄存器，EC,PSC1,PSC2,PSC3
int16 Read_4442_SM(char *SM_Bfr)
{
	uint8 buf[4];
	sle4442_reset(&buf[0]);
	sle4442_command(0x31,0xFF,0xFF);   // 读密码的命令
   return(sle4442_outgoing(SM_Bfr,4,1)); 	
}
int16 Write_4442_SM(uint16 SMAddress,char SMByte)
{	
	uint8 buf[4];
	sle4442_reset(&buf[0]);
	sle4442_command(0x39,SMAddress,SMByte);  // 写命令
      if(sle4442_process()==-1)return -1; 
      return 0;	
}
//-------------------------------
unsigned char HCWrite_4442_ONLY_DAT(unsigned short StartPos,unsigned short num,unsigned char *buffer)
{ uint8 i;
	int16 st;
	//if(CPUCard_Insert==CARD_NONEInsert)return 1;//无卡
	for(i=0;i<num;)
	{
		st=Write_4442(StartPos+i,buffer[i],0);
		if(st!=0)return i;
		i+=1;	
	}
	return 0;
}
int16 HCRead_4442_NO_PB(uint16 StartPos,uint16 NOB,char *Bfr)
{
	uint8 buf[4];
	//if(CPUCard_Insert==CARD_NONEInsert)return -1;//无卡
	sle4442_reset(&buf[0]);
	sle4442_command(0x30,StartPos,0xff);      // 读命令
   
   sle4442_outgoingshort(Bfr,1,NOB);  // 卡输出
   BreakOperate();
   return 0;
}
//读保护位4BYTE
int16 HCRead_4442_PB(char *PB_Bfr)
{
	uint8 buf[4];
	//if(CPUCard_Insert==CARD_NONEInsert)return -1;//无卡
	sle4442_reset(&buf[0]);
	sle4442_command(0x34,0xFF,0xFF);   // 读保护位的命令
	return(sle4442_outgoingDirshort(PB_Bfr,4,1));
}
//--读安全寄存器，EC,PSC1,PSC2,PSC3
int16 HCRead_4442_SM(char *SM_Bfr)
{
	uint8 buf[4];
	sle4442_reset(&buf[0]);
	sle4442_command(0x31,0xFF,0xFF);   // 读密码的命令
   return(sle4442_outgoing(SM_Bfr,4,1)); 	
}
/**
校验3BYTE的PSC
**/
int16 HCVerify_4442_PSC(char PSC1,char PSC2,char PSC3)
{
	uint8 temp[4];
	uint8 i;
	uint8 buf[4];
	sle4442_reset(&buf[0]);
	sle4442_rdEC(&temp[0]);
	if((temp[0]&0x07)!=0)                 	//第一个字节是错误记数，则直接退出
	{
		if((temp[0]&0x07)==0x07)               	//00000111
          i=0x06;
      else if((temp[0]&0x07)==0x06)    	//00000110
          i=0x04;
			else if((temp[0]&0x07)==0x04)           	//00000100
          i=0x00;
          
     sle4442_command(0x39,0,i);             // 去一个错误计数器位 
     if(sle4442_process()==-1)return -1;                     // 写卡等候  
     
     temp[1]=PSC1;
     temp[2]=PSC2;
     temp[3]=PSC3;
      for(i=1;i<4;i++)            
   	{
      sle4442_command(0x33,i,temp[i]);   // 校验密码的命令
      if(sle4442_process()==-1)return -2;   
   	} 
   	sle4442_command(0x39,0,0xff);          // 擦除错误计数器
   	if(sle4442_process()==-1)return -3; 
   	
   	sle4442_rdEC(&temp[0]);
   	if((temp[0]&0x07)==0x07) return 0;//成功
     
	}
	return -1;			
}
unsigned char  HCWrite_4442_With_PB(unsigned short StartPos,unsigned short num,unsigned char *buffer)
{
	uint8 i;
	int16 st;
	for(i=0;i<num;)
	{
		st=Write_4442(StartPos+i,buffer[i],1);
		if(st!=0)return i;
		i+=1;	
	}
	return 0;
}
int16 HCsle4442_uPSC(char *scbuf)
{
	uchar i;
	 uint8 buf[4];
	sle4442_reset(&buf[0]);
  // if (!sle4442_reset()) return(-1);
   for(i=1;i<4;i++)                        // 密码由3字节组成
   {
      sle4442_command(0x39,i,scbuf[i-1]);  // 写命令
      if(sle4442_process()==-1)return -1;
   }
   if(HCVerify_4442_PSC(scbuf[0],scbuf[1],scbuf[2])==0)return 0;
   return(1);
}