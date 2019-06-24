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
#define SLERST_L	RST_Select(0x00)		//��λ��
#define SC1_35VH 	 GPIO_SetBits(DS1_35V_PORT, DS1_35V_PIN)
#define SC1_35VL 	 GPIO_ResetBits(DS1_35V_PORT, DS1_35V_PIN)

#define SLEIO_IN_CTR 0
#define SLEIO_OUT_CTR 1
unsigned char sle4428_outgoing(unsigned	char *rdbuf,unsigned int size,unsigned int n);
extern volatile uint32 	systicnum;
extern volatile uint8 	Card_PowerStatus;//0Ϊδ�ϵ磬1Ϊ�ϵ�
void sel4428_IO_ctl(uint8 iotype)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	if(iotype==SLEIO_IN_CTR)//����
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
	//---�ص�
	SC1_CMDVCCH; //cmd
	SLERST_L;//rst	L
	//---�ر�UART
	USART_Cmd(DS1_UARTX, DISABLE); 
	//---����IO
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;				
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = DS1_CLK_PIN;
  	GPIO_Init(DS1_CLK_PORT, &GPIO_InitStructure);
	GPIO_SetBits(DS1_CLK_PORT, DS1_CLK_PIN);//CLK

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;    //��©���
	GPIO_InitStructure.GPIO_Pin = DS1_IO_PIN;
  	GPIO_Init(DS1_IO_PORT, &GPIO_InitStructure);
	GPIO_SetBits(DS1_IO_PORT, DS1_IO_PIN);//IO

	//����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Pin = DS1_AUX1_PIN;//��������
  GPIO_Init(DS1_AUX1_PORT, &GPIO_InitStructure);
  GPIO_SetBits(DS1_AUX1_PORT, DS1_AUX1_PIN); 

  GPIO_InitStructure.GPIO_Pin = DS1_AUX2_PIN;//��������
  GPIO_Init(DS1_AUX2_PORT, &GPIO_InitStructure);
  GPIO_SetBits(DS1_AUX2_PORT, DS1_AUX2_PIN);

	Delay_Ms(10);
	SLERST_H;
	SC1_35VH;//���CMD����ֹͣģʽ
	
	SLECLK_H;
	SLEIO_H;
	SLERST_H;
	SC1_CMDVCCL;
	Card_PowerStatus=1;//�ϵ�
}

/******************************  Delayus ****************************************
* @	��������Delayus
* @	����˵����us������ʱ
* @ �������1��
* @ �������2��
* @ �������3��
* @	���������
* @ ע�����ʹ��ά������ģʽ������systemtick �������ȫ�ֱ���system_tick_num
* @ �汾��Ϣ��10.5.21 pqf
* @ ����ʾ����
*********************************************************************************************/ 

/*void Delayus(uint32 num)
{
	TIM4_Cmd_OP(num);
	while(Time4_over==0x00);		
}*/
/******************************  delay12us() ****************************************
* @	��������delay12us()
* @	����˵��������Delayus��ɵ�С�����
* @ �������1��
* @ �������2��
* @ �������3��
* @	���������
* @ ע�����
* @ �汾��Ϣ��10.5.21 pqf
* @ ����ʾ����
*********************************************************************************************/
/*void delay12us()
{
   //Delayus(10);
   TIM4_Cmd_OP(12);
	while(Time4_over==0x00);
}*/
/******************************  delay4us() ****************************************
* @	��������delay4us()
* @	����˵��������Delayus��ɵ�С�����
* @ �������1��
* @ �������2��
* @ �������3��
* @	���������
* @ ע�����
* @ �汾��Ϣ��10.5.21 pqf
* @ ����ʾ����
*********************************************************************************************/
/*void delay4us()
{
  //Delayus(3);
  TIM4_Cmd_OP(4);
	while(Time4_over==0x00);
}*/



// 1024x8 bit EEPROM              //
// 1024x1 bit protection memory   //
// sle4428����λ����Ӧ��λ����    // 
/******************************   ****************************************
* @	��������sle4428_reset
* @	����˵������λ����
* @ �������1��
* @ �������2��
* @ �������3��
* @	���������
* @ ע�����
* @ �汾��Ϣ��10-5-28	pqf
* @ ����ʾ����
*********************************************************************************************/
void sle4428_reset(void)
{
	unsigned char i=31;
	if(GPIO_ReadInputDataBit(DS1_OFF_PORT,DS1_OFF_PIN)==0)
	{SC1_CMDVCCH;
	 GPIO_SetBits(DS1_35V_PORT, DS1_35V_PIN);
	 SC_Delay();
	}
	SC1_CMDVCCL; Card_PowerStatus=1;//�ϵ�
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
* @	������sle4428_si��
* @	����˵����sle4428����������һ���ֽں���,��˵CPU��дһ���ֽ�
* @ �������1��Ҫд�������
* @ �������2��
* @ �������3��
* @	���������
* @ ע�����
* @ �汾��Ϣ��10-5-28	pqf
* @ ����ʾ����
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
* @	��������sle4428_so
* @	����˵����sle4428���������һ���ֽں���,��˵CPU�ӿ���һ���ֽ�
* @ �������1��
* @ �������2��
* @ �������3��
* @	����������������ֽ�
* @ ע�����
* @ �汾��Ϣ��10-5-28	pqf
* @ ����ʾ����
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
		if (st) rbyte|=0x80;    // ���͵�λ
		delay4us();
		SLECLK_L;
		delay4us();
	}
	return(rbyte);	 
}


/******************************   ****************************************
* @	��������sle4428_command
* @	����˵���� sle4428������ģʽ����
* @ �������1������
* @ �������2��	��ַ
* @ �������3������
* @	���������
* @ ע�����
* @ �汾��Ϣ��10-5-28	pqf
* @ ����ʾ����
*********************************************************************************************/
void sle4428_command(unsigned char control,unsigned int short address,unsigned char dat)
{
   union aa                 // ������
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
* @	��������sle4428_rPB
* @	����˵������������λ��sle4428��һ���ֽں���
* @ �������1�������ݵĵ�ַ	  addr=0-127
* @ �������2��
* @ �������3��
* @	�������������������+����λ��־
* @ ע������������λΪ0x00��ʾ ����û�б����� 0xFF��ʾ���ݱ�����
* @ �汾��Ϣ��10-5-28	pqf
* @ ����ʾ����
*********************************************************************************************/
unsigned int sle4428_rPB(unsigned int addr)              // read a byte with protect bit
{ 
   unsigned int short dat;
   unsigned char i,st;
   sle4428_reset();
   sle4428_command(0x0c,addr,0xff);     // ������λ������
   
   dat=sle4428_so();	   // test �˴���Ҫ���  ��д������
	st=SLEIO_I;   
   if (st) i=0;              // �ѱ���λ���ݸ������ݷ���λ
   else i=1;
 
   dat<<=8;
   if(i)
   dat|=0x00FF;
   return(dat);
}
uint8 sle4428_rPB2(uint8 addr,uint8 *dat)
{
	
	sle4428_reset();
   sle4428_command(0x0c,addr,0xff);     // ������λ������
   
   *dat=sle4428_so();	   // test �˴���Ҫ���  ��д������
   
   if(SLEIO_I)//�ޱ���
   return 0;
   else return 0xff;
}
/******************************   ****************************************
* @	��������sle4428_rdEC
* @	����˵�������������
* @ �������1��
* @ �������2��
* @ �������3��
* @	��������� �������
* @ ע�������������ݾ�Ϊ�������
* @ �汾��Ϣ��10-5-28	pqf
* @ ����ʾ����
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
* @	��������sle4428_rd
* @	����˵��������������
* @ �������1�����ݿ�ָ��
* @ �������2�����ݿ��С
* @ �������3�����ݿ����
* @ �������4�����ݿ�ʼ��ַ
* @	���������
* @ ע�����
* @ �汾��Ϣ��10-5-28	pqf
* @ ����ʾ����
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
* @	��������sle4428_outgoing��
* @	����˵����sle4428�����ģʽ����
* @ �������1������ָ��
* @ �������2�����ݿ��С
* @ �������3�����ݿ����
* @	��������� ���������ݸ���
* @ ע����� ���������ݸ���ͬ��Ҫ���������ݸ�����ͬ����ɹ�
* @ �汾��Ϣ��10-5-28	pqf
* @ ����ʾ����
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
* @	��������sle4428_programming
* @	����˵����sle4428�����ģʽ����
* @ �������1�����ò���
* @ �������2��
* @ �������3��
* @	���������
* @ ע��������������ʵ��û���õ�
* @ �汾��Ϣ��10-5-28	pqf
* @ ����ʾ����
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
* @	��������sle4428_comp
* @	����˵����sle4428���Ƚ�ģʽ����
* @ �������1��
* @ �������2��
* @ �������3��
* @	���������
* @ ע�����
* @ �汾��Ϣ��10-5-28	pqf
* @ ����ʾ����
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
* @	��������sle4428_vPSC
* @	����˵����У��sle4428�����뺯��
* @ �������1����������ָ��
* @ �������2��
* @ �������3��
* @	���������Ч������֮�������������
* @ ע��������� 8��ȷ �˶�
* @ �汾��Ϣ��10-5-28	pqf
* @ ����ʾ����
*********************************************************************************************/
int16 sle4428_vPSC(unsigned char *vdbuf)
{ 
   unsigned char wcounter,i=0;
   wcounter=sle4428_rdEC();                 // �����������
   
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
	sle4428_command(0x32,1021,wcounter);     // д���������
	if(sle4428_programming(102)==-1) return -1;
	
	sle4428_command(0x0d,1022,vdbuf[0]);     // �Ƚ� PSC byte 1
	if(sle4428_comp()==-1)return -2;
	sle4428_command(0x0d,1023,vdbuf[1]);     // �Ƚ� PSC bye 2
	if(sle4428_comp()==-1)return -3;
	
	sle4428_command(0x33,1021,0xff);         // �����������
	if(sle4428_programming(102)==-1) return -4;
	wcounter=sle4428_rdEC();                 // �ٶ����������
	
	while(wcounter)
	{
		if (wcounter&0x80) i++;   // �Ѵ����������'bit mask'ֵת��Ϊʮ������ 
		wcounter<<=1; 
	}
	return(i);                  // ���ؼ���(=8,��ȷ,�����)
}


/******************************   ****************************************
* @	��������sle4428_rb
* @	����˵����дһ������
* @ �������1�������ַ addr=0-1020
* @ �������2��
* @ �������3��
* @	���������д�����ݵĻض�
* @ ע�����
* @ �汾��Ϣ��10-5-28	pqf
* @ ����ʾ����
*********************************************************************************************/
unsigned char sle4428_rb(unsigned int addr)
{
   sle4428_reset();
   sle4428_command(0x0e,addr,0xff);
   return(sle4428_so());
}


/****************************** sle4428_wb  ****************************************
* @	������sle4428_wb��
* @	����˵������sle4428��дһ���ֽں��� 
* @ �������1����ַ  0-1020
* @ �������2������ ��Ҫд�������
* @ �������3��
* @	��������� д������֮��ض�
* @ ע�������Ҫ��Ч��������ȷ֮��
* @ �汾��Ϣ��10-5-28	pqf
* @ ����ʾ����
*********************************************************************************************/
int16 sle4428_wb(unsigned int short addr,unsigned char dat) 
{  
   sle4428_reset();
   sle4428_command(0x33,addr,dat);
   if(sle4428_programming(203)==-1) return -1;
   return(sle4428_rb(addr));
}


/******************************   ****************************************
* @	�������� sle4428_wr
* @	����˵���������洢��д��һ�����ݿ�����
* @ �������1�������ݿ�ָ��
* @ �������2�����ݿ��С
* @ �������3�����ݿ����
* @	���������д�����ȷ��
* @ ע����������ص�����ͬд������ݸ��� д��ɹ�
* @ �汾��Ϣ��10-5-28
* @ ����ʾ����
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
         if (sle4428_wb(addr,*dp)!=*dp) 		 return-1;  // д��ȷ����
         addr++;
         dp++; 
		 j++ ;
      }
   
   }
   return(j);                                      // j=n,��ȷ,�����
}


/******************************   ****************************************
* @	��������sle4428_wPB
* @	����˵���� ��������λ��sle4428��дһ���ֽں���
* @ �������1����ַ   0-127
* @ �������2��Ҫд���λ��
* @ �������3��
* @	���������������λ
* @ ע�������Ҫ�ڱ���λû��д�����Ҹ�λ�Ѿ�д����Ӧ���ݵ������²���д������
			  ������λδд��������Ϊ0x55������д��0x55���ܳɹ�
* @ �汾��Ϣ��10-5-28 pqf
* @ ����ʾ����
*********************************************************************************************/
int sle4428_wPB(unsigned int short addr,unsigned char dat) 
{
   sle4428_reset();
   sle4428_command(0x30,addr,dat);      // �Ƚ�����:��ȷ��д,����д
   if(sle4428_comp()==-1)return -1;
   return(sle4428_rPB(addr));
}

// �޸�sle4428�����뺯��, �����β�Ϊ�����������ָ��  0 �ɹ� 1 ʧ��  // 
/******************************   ****************************************
* @	��������sle4428_uPSC
* @	����˵���� �޸�sle4428�����뺯��
* @ �������1�����������
* @ �������2��
* @ �������3��
* @	��������� ʧ�� 1  �ɹ� 0
* @ ע�����������֤����֮��Ч��
* @ �汾��Ϣ��10-5-28 pqf
* @ ����ʾ����
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
* @	��������WM_4428_Protec_Read
* @	����˵����������λ������
* @ �������1��������ָ��
* @ �������2����ַ
* @ �������3����Ҫ��ȡ�ĳ���
* @	���������0
* @ ע�����int ����Ҫ����Ϊshort
* @ �汾��Ϣ��10-5-28 pqf
* @ ����ʾ����
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
* @	�������� WM_4428_Check_Password
* @	����˵����Ч������
* @ �������1��������ָ��
* @ �������2��
* @ �������3��
* @	���������ʧ�� 1  �ɹ� 0
* @ ע�����
* @ �汾��Ϣ��10-5-28 pqf
* @ ����ʾ����
*********************************************************************************************/
unsigned char WM_4428_Check_Password(unsigned char *pwd)
{
	if ( sle4428_vPSC(pwd)==0x08 )//=8Ч����ȷ
	return 0x00;
	else return 0x01;
}
/******************************   ****************************************
* @	��������MW_Read_Err_Counter
* @	����˵������ȡ��Ƭ�������
* @ �������1��
* @ �������2��
* @ �������3��
* @	�����������Ƭ�Ĵ������
* @ ע���������ֵ����Ϊ7
* @ �汾��Ϣ��10-5-28 pqf
* @ ����ʾ����
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
* @	��������MW_Write_Password
* @	����˵������������
* @ �������1��������������ָ��
* @ �������2��
* @ �������3��
* @	���������ʧ��1  �ɹ�0
* @ ע�����
* @ �汾��Ϣ��10-5-28 pqf
* @ ����ʾ����
*********************************************************************************************/
unsigned char MW_4428_Write_Password(unsigned char *pwd)
{
   	if( sle4428_uPSC(pwd) )
	return 1;
	else return 0;
}
/******************************   ****************************************
* @	��������MW_Write
* @	����˵����д����
* @ �������1��������ָ��
* @ �������2���׵�ַ
* @ �������3�� ��Ҫд��������ֽڳ���
* @	����������ɹ� 0 �Ѿ������� 1 д���� 2д����ʧ��
* @ ע�����
* @ �汾��Ϣ��10-5-28 pqf
* @ ����ʾ����
*********************************************************************************************/
unsigned char MW_4428_Write(unsigned char *buffer,unsigned int short addr,unsigned int short num)
{
	unsigned int short addr_temp,i,temp;
	addr_temp=addr;
	for(i=0;i<num;i++){
		temp = sle4428_rPB(addr_temp++);
		if((temp&0x00FF)==0x00FF)  return 1;//�Ѿ�д����
	}

	i=sle4428_wr(buffer,num,1,addr);   //i�д�ŵ���д������������ݵı��
	if(i==num)return 0;
	else return 2;
}
/******************************   ****************************************
* @	��������MW_4428_Protect_Write
* @	����˵����������λд����
* @ �������1��������ָ��
* @ �������2����ַ
* @ �������3��д����ֽ���
* @	����������Ѿ�д���� 1����ִ�в������أ�   д����ʧ��2   д����λʧ��  3     �ɹ� 0
* @ ע�������Ҫд�����������һ���Ѿ�д����������ֹ����
* @ �汾��Ϣ��10-5-28 pqf
* @ ����ʾ����
*********************************************************************************************/
unsigned char MW_4428_Protect_Write(unsigned char *buffer,unsigned int short addr,unsigned int short num)
{
	unsigned int short addr_temp,i,temp;
	addr_temp = addr;
 	for(i=0;i<num;i++){
	temp = sle4428_rPB(addr_temp++);
	if((temp&0x00FF)==0x00FF)  return 1;//�Ѿ�д����
	}
	i=sle4428_wr(buffer,num,1, addr);
	if(i==num)
	{  for(i=0;i<num;i++)
		{
			sle4428_wPB(addr,*buffer++) ;  // ��������λ��sle4428��дһ���ֽں��� // 
			temp=sle4428_rPB(addr++);     // read a byte with protect bit
			if((temp&&0x00FF)==0x00 )return 3;		//д����λʧ��
	  	}
	}else return 2;//д����ʧ��

	return 0;
}

/******************************   ****************************************
* @	��������MW_4428_Protec_Bit
* @	����˵��������4428����һ��λ
* @ �������1����ַ
* @ �������2����Ҫ������λ��
* @ �������3��
* @	���������
* @ ע�����������ܱ���λ�� ���� 1  д����λʧ�� 2  �ɹ� 0
* @ �汾��Ϣ��10-5-28 pqf
* @ ����ʾ����
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
			sle4428_wPB(addr_temp,buffer[i]);  // ��������λ��sle4428��дһ���ֽں��� // 
			temp=sle4428_rPB(addr_temp++);     // read a byte with protect bit
			if((temp&&0x00FF)==0x00 )return 2;		//д����λʧ��
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
   sle4428_command(0x0c,StartPos,0xff);     // ������λ������
   for(i=0;i<NOB;i++)
   {
   		Bfr[i]=sle4428_so();
   		SLECLK_H;
			st=SLEIO_I;
			if (SLEIO_I) PB_Bfr[i]=0x01;//0x00;//��д����
			else PB_Bfr[i]=0x00;//0xff;//д����
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
		if((temp&0x00FF)==0x00FF)  return -2;//�Ѿ�д����
		i=sle4428_wr(&DestByte,1,1,StartPos);   //i�д�ŵ���д������������ݵı��
		if(i==1)return 0;
		else return -1;
	}
	else//������λд����
	{
	temp = sle4428_rPB(StartPos);
	if((temp&0x00FF)==0x00FF)  return 1;//�Ѿ�д����
	
	i=sle4428_wr(&DestByte,1,1, StartPos);
	if(i==1)
	{  
			j=sle4428_wPB(StartPos,DestByte) ;  // ��������λ��sle4428��дһ���ֽں��� // 
			if(j==-1) return -1;
			temp=sle4428_rPB(StartPos);     // read a byte with protect bit
			if((temp&&0x00FF)==0x00 )return -3;		//д����λʧ��
	  	
	}else return -1;//д����ʧ��

	return 0;
	}
	

}
int16 Verify_4428_PSC(char PSC1,char PSC2)
{
	unsigned char wcounter,i=0;
   wcounter=sle4428_rdEC();                 // �����������
   
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
	sle4428_command(0x32,1021,wcounter);     // д���������
	if(sle4428_programming(102)==-1) return -1;
	
	sle4428_command(0x0d,1022,PSC1);     // �Ƚ� PSC byte 1
	if(sle4428_comp()==-1)return -2;
	sle4428_command(0x0d,1023,PSC2);     // �Ƚ� PSC bye 2
	if(sle4428_comp()==-1)return -3;
	
	sle4428_command(0x33,1021,0xff);         // �����������
	if(sle4428_programming(102)==-1) return -4;
	wcounter=sle4428_rdEC();                 // �ٶ����������
	
	while(wcounter)
	{
		if (wcounter&0x80) i++;   // �Ѵ����������'bit mask'ֵת��Ϊʮ������ 
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
   sle4428_command(0x0c,1021,0xff);     // ������λ������
   for(i=0;i<3;i++)
   {
   		SM_Bfr[i]=sle4428_so();
   		SLECLK_H;
			st=SLEIO_I;
			if (SLEIO_I) SM_PB_Bfr[i]=0x01;//0x00;//��д����
			else SM_PB_Bfr[i]=0x00;//0xff;//д����
			delay4us();
			SLECLK_L;
			delay4us();
   }
   return 0;
}
//-------------------------------------
/******************************   ****************************************
* @	��������sle4428_rPB
* @	����˵������������λ��sle4428��һ���ֽں���
* @ �������1�������ݵĵ�ַ	  addr=0-127
* @ �������2��
* @ �������3��
* @	�������������������+����λ��־
* @ ע������������λΪ0xff��ʾ ����û�б����� 0x00��ʾ���ݱ�����
* @ �汾��Ϣ��10-5-28	pqf
* @ ����ʾ����
*********************************************************************************************/
unsigned int HCsle4428_rPB(unsigned int addr)              // read a byte with protect bit
{ 
   unsigned int short dat;
   unsigned char i,st;
   sle4428_reset();
   sle4428_command(0x0c,addr,0xff);     // ������λ������
   
   dat=sle4428_so();	   // test �˴���Ҫ���  ��д������
	st=SLEIO_I;   
   if (st) i=1;              // �ѱ���λ���ݸ������ݷ���λ
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
   sle4428_command(0x0c,StartPos,0xff);     // ������λ������
   for(i=0;i<NOB;)
   {
   		Bfr[i*2]=sle4428_so();
   		SLECLK_H;
			st=SLEIO_I;
			if (SLEIO_I) Bfr[i*2+1]=0xff;//0x00;//��д����
			else Bfr[i*2+1]=0x00;//д����
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
		if((temp&0x00FF)==0x0000)  return 1;//�Ѿ�д����
	}

	i=sle4428_wr(buffer,num,1,StartPos);   //i�д�ŵ���д������������ݵı��
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
		if((temp&0x00FF)==0x0000)  return -2;//�Ѿ�д����
		i=sle4428_wr(&DestByte,1,1,StartPos);   //i�д�ŵ���д������������ݵı��
		if(i==1)return 0;
		else return -1;
	}
	else//������λд����
	{
	temp = HCsle4428_rPB(StartPos);
	if((temp&0x00FF)==0x0000)  return -2;//�Ѿ�д����
	
	i=sle4428_wr(&DestByte,1,1, StartPos);
	if(i==1)
	{  
			j=sle4428_wPB(StartPos,DestByte) ;  // ��������λ��sle4428��дһ���ֽں��� // 
			if(j==-1) return -1;
			temp=HCsle4428_rPB(StartPos);     // read a byte with protect bit
			if((temp&&0x00FF)==0xff )return -3;		//д����λʧ��
	  	
	}else return -1;//д����ʧ��

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
   sle4428_command(0x0c,StartPos,0xff);     // ������λ������
   for(i=0;i<NOB;i++)
   {
   		Bfr[i]=sle4428_so();
   		SLECLK_H;
			st=SLEIO_I;
			if (SLEIO_I) PB_Bfr[i]=0xff;//��д����
			else PB_Bfr[i]=0x00;//д����
			delay4us();
			SLECLK_L;
			delay4us();
   }
   return 0;
}
//�ȶ��Ƿ�����һ�£����뱣��
//�̻�����
//����ʵ��д����ȷ���ֽڸ���
uint16 HCWrite_4428_Only_PB(unsigned short StartPos,unsigned short num,unsigned char *buffer)
{
	uint16 n,i;
	uint8 st,Bfr,PB_Bfr;
	int16 t;
	sle4428_reset();
	
	
	for(n=0;n<num;)
	{
	sle4428_command(0x0c,StartPos+n,0xff);     // ������λ������
	Bfr=sle4428_so();
  SLECLK_H;
	st=SLEIO_I;
	if (SLEIO_I) PB_Bfr=0xff;//��д����
	else PB_Bfr=0x00;//д����
	delay4us();
	SLECLK_L;
	delay4us();
	//�Ƚ�
	if(Bfr!=buffer[n]) return n;
	if(PB_Bfr==0xff)//�ޱ��������
	{
		t=HCWrite_4428_Byte(StartPos+n,buffer[n],1);
		if(t!=0) return 1;
	}
	n+=1;
	}
	return n;
	
}
//��������������룬�Լ�����λ
int16 HCRead_4428_SM(char *SM_Bfr,char *SM_PB_Bfr)
{
	uint16 n,i;
	uint8 st;
	sle4428_reset();
   sle4428_command(0x0c,1021,0xff);     // ������λ������
   for(i=0;i<3;i++)
   {
   		SM_Bfr[i]=sle4428_so();
   		SLECLK_H;
			st=SLEIO_I;
			if (SLEIO_I) SM_PB_Bfr[i]=0xff;//0x00;//��д����
			else SM_PB_Bfr[i]=0x00;//0xff;//д����
			delay4us();
			SLECLK_L;
			delay4us();
   }
   return 0;
}
int16 HCWrite_4428_EC(char wcounter)
{
	sle4428_reset();
	sle4428_command(0x32,1021,wcounter);     // д���������
	if(sle4428_programming(102)==-1) return -1;
	else
		return 0;
}
//�˶�����
int16 HCVerify_4428_PSC(char PSC1,char PSC2)
{
	unsigned char wcounter,i=0;
   wcounter=sle4428_rdEC();                 // �����������
   
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
	sle4428_command(0x32,1021,wcounter);     // д���������
	if(sle4428_programming(102)==-1) return -1;
	
	sle4428_command(0x0d,1022,PSC1);     // �Ƚ� PSC byte 1
	if(sle4428_comp()==-1)return -2;
	sle4428_command(0x0d,1023,PSC2);     // �Ƚ� PSC bye 2
	if(sle4428_comp()==-1)return -3;
	
	sle4428_command(0x33,1021,0xff);         // �����������
	if(sle4428_programming(102)==-1) return -4;
	wcounter=sle4428_rdEC();                 // �ٶ����������
	
	while(wcounter)
	{
		if (wcounter&0x80) i++;   // �Ѵ����������'bit mask'ֵת��Ϊʮ������ 
		wcounter<<=1; 
	}
	if(i==8)return 0;
	else return i;
}