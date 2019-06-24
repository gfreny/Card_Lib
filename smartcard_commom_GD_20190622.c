#include "stm32f10x.h"
#include "platform_config.h"
//---------------------------------------------------------------------
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


#define OK 0
#define Over_Time 0x12
#define MCRC_ERR  0x13
#define PPS_ERR  0x14

//社保错误代码
//执行成功
#define IFD_OK          				0
//卡片类型不对
#define IFD_ICC_TypeError     		1
//无卡
#define IFD_ICC_NoExist	  		2
//有卡未上电
#define IFD_ICC_NoPower       		3
//卡片无应答
#define IFD_ICC_NoResponse		4
//读卡器连接错
//#define IFD_ConnectError              	11
//未建立连接(没有执行打开设备函数)
//#define IFD_UnConnected        	12
//(动态库)不支持该命令
//#define IFD_BadCommand  		13
//(发给动态库的)命令参数错
#define IFD_ParameterError           14
//信息校验和出错
//#define IFD_CheckSumError		15

//内部使用
#define IFD_ICC_TSError     		100
#define IFD_ICC_T0Error     		101
//--------------状态码--------------------------------------------------------

//--------卡座序号----------------------------------------
#define CARD_NONEInsert 0
#define CARD_FIRSRInsert 1
#define CARD_SECONDInsert 2
#define CARD_DOUBLEInsert 3


#define Card_Staus_ADPUing 0x01
#define Card_Staus_Poweroff 0x00

#define CARD_FIRST 0  //上面插卡
#define CARD_SECOND 1//底部大卡座
#define CARD_THIRD 2//SIM1小卡
#define CARD_FOURTH 3//SIM2小卡
#define CARD_FIFTH 4//SIM3小卡
#define CARD_SIXTH 5//SIM4小卡
#define CARD_Default 7//
//--设置电压
#define Card_0V 0
#define Card_18V 1
#define Card_30V 2
#define Card_50V 3

//-----------------------------------------------------------------------

#define Use_HCV200 0//使用汇卡双界面协议
#define Use_DCD3 0//使用德卡D3协议
#define Use_DCT10 1//使用德卡T10协议
#define Use_DES 0//德生定制协议

#if (Use_HCV200==1)//汇卡双界面协议
#include "system_defineHC.h"
//卡型定义
#define CardType_sam0 HC_Card_CPU_sam0
#define CardType_SLE4442 HC_Card_SLE4442
#define CardType_SLE4428 HC_Card_SLE4428
#define CardType_AT24C01A HC_Card_AT24C01A
#define CardType_AT24C02 HC_Card_AT24C02
#define CardType_AT24C04 HC_Card_AT24C04
#define CardType_AT24C08 HC_Card_AT24C08
#define CardType_AT24C16 HC_Card_AT24C16
#define CardType_AT24C32 HC_Card_AT24C32
#define CardType_AT24C64 HC_Card_AT24C64
#define CardType_AT24C128 HC_Card_AT24C128
#define CardType_AT24C256 HC_Card_AT24C256
#define CardType_AT24C512 HC_Card_AT24C512
#define CardType_AT24C1024 HC_Card_AT24C1024
#define CardType_AT88SC102 HC_Card_AT88SC102
#define CardType_AT88SC1604 HC_Card_AT88SC1604
#define CardType_AT88SC1608 HC_Card_AT88SC1608
#define CardType_AT88SC153 HC_Card_AT88SC153
#define CardType_AT45D041 HC_Card_AT45D041

#else 
#endif

#if (Use_DCT10==1)//德卡T10协议
#include "system_defineT10.h"
//卡型定义
#define CardType_sam0 T10_Card_CPU_sam0
#define CardType_SLE4442 T10_Card_SLE4442
#define CardType_SLE4428 T10_Card_SLE4428
#else 
#endif

#if (Use_DCD3==1)//德卡D3协议
#include "system_defineD3.h"
//卡型定义
#define CardType_sam0 D3_cardtype_cpu
#define CardType_SLE4442 D3_cardtype_4442
#define CardType_SLE4428 D3_cardtype_4428
#else 
#endif

#if (Use_DES==1)//德生协议
#include "system_defineDES.h"
//卡型定义
#define CardType_sam0 DES_Card_Contactcpu//DES_cardtype_cpu
#else 
#endif
/***************宏编译**********************/
#define Use_Card_CPU_sam0 1//=1支持大卡座CPU =0不支持大卡座
#define Use_Card_SLE4442 0//=1支持4442卡型，=0不支持
#define Use_Card_SLE4428 0
#define Use_Card_AT24CX 0
#define Use_Card_AT88SC102 0
#define Use_Card_AT88SC1604 0
#define Use_Card_AT88SC1608 0
#define Use_Card_AT88SC153 0
#define Use_Card_AT45D041 0

#define Use_CardLun_FIRST 1//=1支持大卡座 =0不支持大卡座
#define Use_CardLun_SECOND 1//=1支持SAM1 =0不支持SAM1
#define Use_CardLun_THIRD 1//=1支持SAM2 =0不支持SAM2
#define Use_CardLun_FOURTH 0//=1支持SAM3 =0不支持SAM3
#define Use_CardLun_FIFTH 0//=1支持SAM4 =0不支持SAM4
#define Use_CardLun_SIXTH 0//=1支持SAM5 =0不支持SAM5

#define AUto_Card_PPS 0//=1复位后自动PPS，=0单独PPS

#define ADPU_WithPCBNAD 0//0 不需要PCB及NAD，1需要
#define ADPU_WithCRC 0//0 不需要CRC，1需要

#define SC_Use_U1 0//主卡使用UART1口
#define SAM_Use_U1 0//PSAM卡使用UART1口
/***************end**********************/
#define atr_delayclksum 10800//复位时字符间距，固定9600波特率9600为10752个4MHz时钟
#define MAX_Card 3
//--------------------------------------------------------
#define error_ok 0
#define error_p 1
#define error_timeout 2
#define error_resum 3
uint8 USARTDoWith_ByteReceive(uint8 *Data, uint32 TimeOut);//处理超时和重发错误

#define SC1_CMDVCCH   GPIO_SetBits(CARD_CMD_PORT, CARD_CMD_PIN)
#define SC1_CMDVCCL   GPIO_ResetBits(CARD_CMD_PORT, CARD_CMD_PIN)


#define SC1_35VH 	 GPIO_SetBits(CARD_VSEL1_PORT, CARD_VSEL1_PIN)
#define SC1_35VL 	 GPIO_ResetBits(CARD_VSEL1_PORT, CARD_VSEL1_PIN)

#define T0_PROTOCOL        0x00  /* T0 protocol */
#define T1_PROTOCOL        0x01  /* T0 protocol */

#define __TIM2_DIER               0x0001                  // 61
#define TIMX_CR1_CEN         ((unsigned short)0x0001)

#define Card_OK OK 
#define Card_OverTime Over_Time 
#define Card_CRCERR MCRC_ERR  
#define Card_PPSERR PPS_ERR  
#define Card_I2CERR I2C_ERR

#define Apdu_comm_0 0//CLA+INS+P1+P2
#define Apdu_comm_1 1//CLA+INS+P1+P2   +LE
#define Apdu_comm_2 2//CLA+INS+P1+P2   +LC  +[DATA]
#define Apdu_comm_3 3//CLA+INS+P1+P2   +LC  +[DATA]  +LE
#define T1_NAN 0
#define T1_PCB 0

/*******************************************************/
//------
#define is_over 0
#define not_over 1

  
  /*uart3 clock*/
/*card  mode of operate-------*/
#define CARD_MODE_Special 0x00 //专用模式
#define CARD_MODE_Consult 0x01 //协商模式

#define DIRECT             0x3B  /* Direct bit convention */
#define INDIRECT           0x3F  /* Indirect bit convention */
#define SETUP_LENGTH       20
#define HIST_LENGTH        20
#define D_LENGTH           15

#define default_data 0x00
#define other_data 0x01

#define coldrest 0
#define warmrest 1


/*------------------------------*/
/* CARD Parameter structure -----------------------------------------------*/
typedef struct 
{
  	uint8 CARD_DIRECT;           /* 0x3b:DIRECT,0x3f: INDIRECT */
  	uint8 CARD_PROTOCOL;         /* 0:T0;1:T1;other: err*/
  	uint8 CARD_Voltage;          /*0:5V; 1:3V*/
  	uint32 CARD_F;               /*372:default*/
  	uint32 CARD_D;               /*1:default*/
  	uint8 CARD_Ipp;
  	uint8 CARD_PI;
  	uint8 CARD_N;
  	uint8 CARD_TRANSFER_STATION;
  	uint8 CARD_TYPE_STATION;//0合适，1无卡，2型号不符
  	uint8 CARD_W;
  	//T1
  	uint8 CARD_CWI;
  	uint8 CARD_BWI;
  	uint8 CARD_IFSC;
  	uint8 CARD_IFSD;
} CARD_Parameter;

/* ATR structure - Answer To Reset -------------------------------------------*/
typedef struct
{
  	uint8 TS;               /* Bit Convention */
  	uint8 T0;               /* High nibble = Number of setup byte; low nibble = Number of historical byte */
	  uint8 Tlength;
	  uint8 Hlength;
	  int16 TA[3];
	  int16 TB[3];
	  int16 TC[3];
	  int16 TD[4];
	  int16 H[16];	  
	  int16 TCK;
} SC_ATRWord;
SC_ATRWord SC_A2RWord;

/************T1************************************/
/*******************************************************/

CARD_Parameter Parameter[MAX_Card];
volatile uint8 Select_Card_NO;
uint8 Card_TypeSave;
uint8 Card_RunStatus[MAX_Card];

volatile uint8 CPUCard_Insert=0x00;

volatile uint8 	Card_PowerStatus;//0为未上电，1为上电
volatile int8 Timer2_over;

uint32 delay_time[MAX_Card],delay_time_t1byte[MAX_Card],delay_time_t1wait[MAX_Card];
uint16 delay_byte[MAX_Card];
uint8 pcb_tmp[MAX_Card];
uint8 NAD_tmp[MAX_Card];

volatile uint8 SCData = 0;

#define TIM2_Stop() Timer2_over

/****************************************
**Function
****************************************/
int16 Card_PowerOff(void);
void SC1_CPUIOConfig(void);
void SC_Delay(void);
void SC1_UART_Init(void);
void SAM_UART_Init(void);
void SC1_UART_InitAgian(uint8 default_setting);
void SAM_UART_InitAgian(uint8 default_setting);
void SC_ParityErrorHandler(void);
void TIM2_Init(uint16 Prescale);
void TIM2_Start(uint16 delay);
void TIM2_Close(void);
int16 SC_USART_SendData(USART_TypeDef* USARTx, uint16 Data);
void SC_USART_SendData_DoWithErr(USART_TypeDef* USARTx, uint16 Data);
void delay_Init(void);
void Parameter_Init(uint8 SC_Voltagetmp);
void SC_A2R_Init(void);
uint16 Card_Init(void);
word Card_Protocol(word Protocol,uint16 F,uint16 D);
uint16 SC4442_PowerON(uint8 SC_Vol,uint8 *bufsc);

uint16 SC_Rest(uint8 cold_warm,uint8 SC_Voltagetmp,uint16 *Length,uint8 * Atr);

uint16 Card_Apdu_T0(uint16 Length,uint8 * Command,uint8 * Response);
uint16 Card_Apdu_T1(uint16 Length,uint8 * Command,uint8 * Response);
int16 Card_Insert(void);

uint16 Card_ColdRest(uint8 SC_Voltagetmp,uint16 *Length,uint8 * Atr);
uint16 Card_WarmRest(uint16 *Length,uint8 * Atr);
uint16 Card_Apdu(word *Length,uint8 * Command,uint8 * Response);
const uint32 F_Table[16] = //{0, 372, 558, 744, 1116, 1488, 2232, 0,
													{372, 372, 558, 744, 1116, 1488, 2232, 0,
                          0, 512, 768, 1024, 1536, 2048, 0, 0};
const uint32 D_Table[10] = {0, 1, 2, 4, 8, 16, 32, 0,12,20};
const uint8 I_Table[4] = {25, 50, 0, 0};


extern void	Delay_Ms(u16	delaytime);//1ms

volatile uint8 	card_errsum=0 ;//读卡器重发次数
#define Card_SentErrSum 4//置错多少次后正确置位
volatile uint8 card_errresum=0;//读卡器接收错误次数
#define card_errresumlong 4
//-----需要按实际情况移植-----------------------------
void IO_SelectCard(uint8 lun)
{
GPIO_ResetBits(SAM_SEL_PORT, SAM_SELA_PIN);
	switch(lun)
	{
		case CARD_SECOND://1
		break;
		case CARD_THIRD://2
			GPIO_SetBits(SAM_SEL_PORT, SAM_SELA_PIN);
		break;
		default:

		break;
	}
}
//-----------end-----------------------------
void RST_Select(uint8 type)
{
	switch(type)
	{		
#if (Use_CardLun_FIRST==1)		
    //--L
		case CARD_FIRST:
			GPIO_ResetBits(CARD_RST_PORT, CARD_RST_PIN);
		break;
		//------H
		case (CARD_FIRST|0X10):
			GPIO_SetBits(CARD_RST_PORT, CARD_RST_PIN);
		break;
#else 
#endif		
#if (Use_CardLun_SECOND==1)		
//--L
		case CARD_SECOND:
			GPIO_ResetBits(SAM1_RST_PORT, SAM1_RST_PIN);
		break;
		//------H
		case (CARD_SECOND|0X10):
			GPIO_SetBits(SAM1_RST_PORT, SAM1_RST_PIN);
		break;
#else 
#endif		
#if (Use_CardLun_THIRD==1)	
//--L	
		case CARD_THIRD:
			GPIO_ResetBits(SAM2_RST_PORT, SAM2_RST_PIN);
		break;
		//------H
		case (CARD_THIRD|0X10):
			GPIO_SetBits(SAM2_RST_PORT, SAM2_RST_PIN);
		break;
#else 
#endif		
#if (Use_CardLun_FOURTH==1)	
//--L	
		case CARD_FOURTH:
			GPIO_ResetBits(SAM3_RST_PORT, SAM3_RST_PIN);
		break;
		//------H
		case (CARD_FOURTH|0X10):
			GPIO_SetBits(SAM3_RST_PORT, SAM3_RST_PIN);
		break;
#else 
#endif		
#if (Use_CardLun_FIFTH==1)	
//--L	
		case CARD_FIFTH:
			GPIO_ResetBits(SAM4_RST_PORT, SAM4_RST_PIN);
		break;
		//------H
		case (CARD_FIFTH|0X10):
			GPIO_SetBits(SAM4_RST_PORT, SAM4_RST_PIN);
		break;
#else 
#endif		

		default:
			
		break;	
	}
}
uint16_t USART_ReceiveData_MY(USART_TypeDef* USARTx)
{	uint32 c;
  c=USARTx->SR;
  return (uint16_t)(USARTx->DR & (uint16_t)0x01FF);
}
void SC_Delay(void)
{
   uint16 k;
   for(k=0;k<50000;k++); 
}
void SC_UART_ColdInit(void)
{
	
}
void SAM_UART_ColdInit(void)
{
	
}
int16 SC_USART_SendData(USART_TypeDef* USARTx, uint16 Data)
{
	uint16 tmp2=0,i;
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_DATA(Data)); 
   card_errsum=0;   		   
  /* Transmit Data */
  if(Parameter[Select_Card_NO].CARD_DIRECT!=DIRECT)
  {
  	//取反向
  	for(i=0;i<8;i++)
  		tmp2|=(Data<<(15-2*i))&(1<<(15-i));	
  	Data=tmp2;
  }
  
  i=USARTx->SR;
  i=USARTx->DR;
  USARTx->SR&=0xffdf;//清除RXNE，防止第一次无法正确判断到重发错误
  USARTx->DR = (Data & (u16)0x01FF);//读取SR，写入DR，将清除TC,以防第一个字节未实际发送就被后面的给覆盖掉
  while((USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)&&(card_errsum<Card_SentErrSum));
	
  if(card_errsum>=Card_SentErrSum) return -1;
  else return 0;
}
void SC_USART_SendData_DoWithErr(USART_TypeDef* USARTx, uint16 Data)
{
	uint16 tmp2=0,i;
  // Check the parameters 
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_DATA(Data)); 
      		   
  // Transmit Data 
  if(Parameter[Select_Card_NO].CARD_DIRECT!=DIRECT)
  {
  	//取反向
  	for(i=0;i<8;i++)
  		tmp2|=(Data<<(15-2*i))&(1<<(15-i));	
  	Data=tmp2;
  }
  i=USARTx->SR;
  i=USARTx->DR;
  USARTx->SR&=0xffdf;
  USARTx->DR = (Data & (u16)0x01FF);
  //需要注意实际情况是否需要判断
  while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
}


/************************************************
**串口2初始化 大卡座1
************************************************/
/*******************************************************************************
* Function Name  : ds8023_UART_IRQHandler
* Description    : This function handles NCN6001_UART global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

#if (Use_CardLun_FIRST==1)
void SC_ParityErrorHandler(void)
{	
  	SC_USART_SendData_DoWithErr(CARD_UARTX, SCData);
}
void SC_irq(void)
{
  /* If a Frame error is signaled by the card */
  if(USART_GetITStatus(CARD_UARTX, USART_IT_FE) != RESET)//帧错误
  {
  	USART_ClearITPendingBit(CARD_UARTX, USART_IT_FE);
    USART_ReceiveData_MY(CARD_UARTX);
    card_errsum+=1;
    if(card_errsum<Card_SentErrSum)
    /* Resend the byte that failed to be received (by the Smartcard) correctly */
    	SC_ParityErrorHandler();
  }
  
  /* If the NCN6001_UART detects a parity error */
  if(USART_GetITStatus(CARD_UARTX, USART_IT_PE) != RESET)//校验错误
  {
  	while(USART_GetFlagStatus(CARD_UARTX, USART_FLAG_RXNE) == RESET)//必须一直等待RXNE置1
    {
    }
  	USART_ClearITPendingBit(CARD_UARTX, USART_IT_PE);
    /* Enable NCN6001_UART RXNE Interrupt (until receiving the corrupted byte) */
    /* Flush the NCN6001_UART DR register */
    USART_ReceiveData_MY(CARD_UARTX);
    //可能需要检测错误次数
		card_errresum+=1;
  }
  
  /* If a Overrun error is signaled by the card */
  if(USART_GetITStatus(CARD_UARTX, USART_IT_ORE) != RESET)//过载错误
  {
  	/* Clear the USART3 Frame error pending bit */
  	USART_ClearITPendingBit(CARD_UARTX, USART_IT_ORE);
    USART_ReceiveData_MY(CARD_UARTX);
  }
  /* If a Noise error is signaled by the card */
  if(USART_GetITStatus(CARD_UARTX, USART_IT_NE) != RESET)//噪声错误
  {
  	/* Clear the USART3 Frame error pending bit */
    USART_ClearITPendingBit(CARD_UARTX, USART_IT_NE);
    USART_ReceiveData_MY(CARD_UARTX);
  }
}
void SC_UART_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
  	USART_ClockInitTypeDef USART_ClockInitStructure;
  	
#if (SC_Use_U1==1)//主卡使用UART1口
	/* Enable TDA8020_UART clock */
  RCC_APB2PeriphClockCmd(CARD_UART_RCC, ENABLE);
#else 
/* Enable TDA8020_UART clock */
  RCC_APB1PeriphClockCmd(CARD_UART_RCC, ENABLE);
#endif
  	
                           
  /* Configure USART2 CK(PA.4) as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = CARD_CLK_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(CARD_CLK_PORT, &GPIO_InitStructure);
  
  /* Configure USART2 Tx (PA.2) as alternate function open-drain */
  GPIO_InitStructure.GPIO_Pin = CARD_IO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(CARD_IO_PORT, &GPIO_InitStructure);
  /* USART2 configuration ------------------------------------------------------*/
  /* USART2 configured as follow:
        - Word Length = 9 Bits
        - 0.5 Stop Bit
        - Even parity
        - BaudRate =  baud
        - Hardware flow control disabled (RTS and CTS signals)
        - Tx and Rx enabled
        - USART Clock enabled
  */
  /* USART Clock set to 4.5 MHz (PCLK1 (36 MHZ) / 2/4) */
  USART_SetPrescaler(CARD_UARTX, CARD_UART_Frequency); 
 	
  /* USART Guard Time set to 16 Bit */
  USART_SetGuardTime(CARD_UARTX, delay_byte[Select_Card_NO]-10);
  
  USART_ClockInitStructure.USART_Clock = USART_Clock_Enable;
  USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
  USART_ClockInitStructure.USART_CPHA = USART_CPHA_1Edge;
  USART_ClockInitStructure.USART_LastBit = USART_LastBit_Enable;
  USART_ClockInit(CARD_UARTX, &USART_ClockInitStructure);

  USART_InitStructure.USART_BaudRate = (uint32)CARD_UART_RATE*(uint32)Parameter[Select_Card_NO].CARD_D/(uint32)Parameter[Select_Card_NO].CARD_F;
  USART_InitStructure.USART_WordLength = USART_WordLength_9b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1_5;
  USART_InitStructure.USART_Parity = USART_Parity_Even;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(CARD_UARTX, &USART_InitStructure); 
   
  if(Parameter[Select_Card_NO].CARD_PROTOCOL==T0_PROTOCOL)
  {
  /* Enable the USART2 Parity Error Interrupt */
  USART_ITConfig(CARD_UARTX, USART_IT_PE, ENABLE);

  /* Enable the USART2 Framing Error Interrupt */
  USART_ITConfig(CARD_UARTX, USART_IT_ERR, ENABLE);
  }
  else if(Parameter[Select_Card_NO].CARD_PROTOCOL==T1_PROTOCOL)
  {
  /* Enable the TDA8020_UART Parity Error Interrupt */
  USART_ITConfig(CARD_UARTX, USART_IT_PE, DISABLE);

  /* Enable the TDA8020_UART Framing Error Interrupt */
  USART_ITConfig(CARD_UARTX, USART_IT_ERR, DISABLE);
  }
  else ;


  	/* Enable USART2 Receive and Transmit interrupts */
  /* Enable USART2 */
  USART_Cmd(CARD_UARTX, ENABLE);

  /* Enable the NACK Transmission */
  USART_SmartCardNACKCmd(CARD_UARTX, ENABLE);

  /* Enable the Smartcard Interface */
  USART_SmartCardCmd(CARD_UARTX, ENABLE);	
}
void SC_UART_InitAgain(uint8 default_setting)
{
	USART_InitTypeDef USART_InitStructure;
	if(default_setting!=other_data) return;
	
	
	// USART Guard Time set to 16 Bit 
  USART_SetGuardTime(CARD_UARTX, delay_byte[Select_Card_NO]-10);
  

  USART_InitStructure.USART_BaudRate = (uint32)CARD_UART_RATE*(uint32)Parameter[Select_Card_NO].CARD_D/(uint32)Parameter[Select_Card_NO].CARD_F;
  USART_InitStructure.USART_WordLength = USART_WordLength_9b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1_5;
  USART_InitStructure.USART_Parity = USART_Parity_Even;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(CARD_UARTX, &USART_InitStructure); 
   
  if(Parameter[Select_Card_NO].CARD_PROTOCOL==T0_PROTOCOL)
  {
  // Enable the USART2 Parity Error Interrupt 
  USART_ITConfig(CARD_UARTX, USART_IT_PE, ENABLE);

  // Enable the USART2 Framing Error Interrupt 
  USART_ITConfig(CARD_UARTX, USART_IT_ERR, ENABLE);
  }
  else if(Parameter[Select_Card_NO].CARD_PROTOCOL==T1_PROTOCOL)
  {
  // Enable the TDA8020_UART Parity Error Interrupt 
  USART_ITConfig(CARD_UARTX, USART_IT_PE, DISABLE);

  // Enable the TDA8020_UART Framing Error Interrupt 
  USART_ITConfig(CARD_UARTX, USART_IT_ERR, DISABLE);
  }
  else ;
}
#else 
#endif
/************************************************
  SAM卡座
************************************************/
void SAM_ParityErrorHandler(void)
{	
  	SC_USART_SendData_DoWithErr(SAM_UARTX, SCData);
}
void SAMUART_irq(void)
{
  /* If a Frame error is signaled by the card */
  if(USART_GetITStatus(SAM_UARTX, USART_IT_FE) != RESET)//帧错误
  {
  	USART_ClearITPendingBit(SAM_UARTX, USART_IT_FE);
    USART_ReceiveData_MY(SAM_UARTX);
    card_errsum+=1;
    if(card_errsum<Card_SentErrSum)
    /* Resend the byte that failed to be received (by the Smartcard) correctly */
    	SAM_ParityErrorHandler();
  }
  
  /* If the NCN6001_UART detects a parity error */
  if(USART_GetITStatus(SAM_UARTX, USART_IT_PE) != RESET)//校验错误
  {
  	while(USART_GetFlagStatus(SAM_UARTX, USART_FLAG_RXNE) == RESET)//必须一直等待RXNE置1
    {
    }
  	USART_ClearITPendingBit(SAM_UARTX, USART_IT_PE);
    /* Enable NCN6001_UART RXNE Interrupt (until receiving the corrupted byte) */
    /* Flush the NCN6001_UART DR register */
    USART_ReceiveData_MY(SAM_UARTX);
    //可能需要检测错误次数
		card_errresum+=1;
  }
  
  /* If a Overrun error is signaled by the card */
  if(USART_GetITStatus(SAM_UARTX, USART_IT_ORE) != RESET)//过载错误
  {
  	/* Clear the USART3 Frame error pending bit */
  	USART_ClearITPendingBit(SAM_UARTX, USART_IT_ORE);
    USART_ReceiveData_MY(SAM_UARTX);
  }
  /* If a Noise error is signaled by the card */
  if(USART_GetITStatus(SAM_UARTX, USART_IT_NE) != RESET)//噪声错误
  {
  	/* Clear the USART3 Frame error pending bit */
    USART_ClearITPendingBit(SAM_UARTX, USART_IT_NE);
    USART_ReceiveData_MY(SAM_UARTX);
  }
}
void SAM_UART_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
  	USART_ClockInitTypeDef USART_ClockInitStructure;
#if (SAM_Use_U1==1)//PSAM卡使用UART1口
/* Enable TDA8020_UART clock */
  RCC_APB2PeriphClockCmd(SAM_UART_RCC, ENABLE);
#else 
	/* Enable TDA8020_UART clock */
  RCC_APB1PeriphClockCmd(SAM_UART_RCC, ENABLE);
#endif  	
  	
                           
  /* Configure USART2 CK(PA.4) as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = SAM_CLK_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(SAM_CLK_PORT, &GPIO_InitStructure);
  
  /* Configure USART2 Tx (PA.2) as alternate function open-drain */
  GPIO_InitStructure.GPIO_Pin = SAM_IO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(SAM_IO_PORT, &GPIO_InitStructure);
  /* USART2 configuration ------------------------------------------------------*/
  /* USART2 configured as follow:
        - Word Length = 9 Bits
        - 0.5 Stop Bit
        - Even parity
        - BaudRate =  baud
        - Hardware flow control disabled (RTS and CTS signals)
        - Tx and Rx enabled
        - USART Clock enabled
  */
  /* USART Clock set to 4.5 MHz (PCLK1 (36 MHZ) / 2/4) */
  USART_SetPrescaler(SAM_UARTX, SAM_UART_Frequency); 
 	
  /* USART Guard Time set to 16 Bit */
  USART_SetGuardTime(SAM_UARTX, delay_byte[Select_Card_NO]-10);
  
  USART_ClockInitStructure.USART_Clock = USART_Clock_Enable;
  USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
  USART_ClockInitStructure.USART_CPHA = USART_CPHA_1Edge;
  USART_ClockInitStructure.USART_LastBit = USART_LastBit_Enable;
  USART_ClockInit(SAM_UARTX, &USART_ClockInitStructure);

  USART_InitStructure.USART_BaudRate = (uint32)SAM_UART_RATE*(uint32)Parameter[Select_Card_NO].CARD_D/(uint32)Parameter[Select_Card_NO].CARD_F;
  USART_InitStructure.USART_WordLength = USART_WordLength_9b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1_5;
  USART_InitStructure.USART_Parity = USART_Parity_Even;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(SAM_UARTX, &USART_InitStructure); 
   
  if(Parameter[Select_Card_NO].CARD_PROTOCOL==T0_PROTOCOL)
  {
  /* Enable the USART2 Parity Error Interrupt */
  USART_ITConfig(SAM_UARTX, USART_IT_PE, ENABLE);

  /* Enable the USART2 Framing Error Interrupt */
  USART_ITConfig(SAM_UARTX, USART_IT_ERR, ENABLE);
  }
  else if(Parameter[Select_Card_NO].CARD_PROTOCOL==T1_PROTOCOL)
  {
  /* Enable the TDA8020_UART Parity Error Interrupt */
  USART_ITConfig(SAM_UARTX, USART_IT_PE, DISABLE);

  /* Enable the TDA8020_UART Framing Error Interrupt */
  USART_ITConfig(SAM_UARTX, USART_IT_ERR, DISABLE);
  }
  else ;


  	/* Enable USART2 Receive and Transmit interrupts */
  /* Enable USART2 */
  USART_Cmd(SAM_UARTX, ENABLE);

  /* Enable the NACK Transmission */
  USART_SmartCardNACKCmd(SAM_UARTX, ENABLE);

  /* Enable the Smartcard Interface */
  USART_SmartCardCmd(SAM_UARTX, ENABLE);	
}

void SAM_UART_InitAgain(uint8 default_setting)
{

}
/*******************************************************************************
* Function Name  : USART_ByteReceive
* Description    : Receives a new data while the time out not elapsed.
* Input          : None
* Output         : None
* Return         : An ErrorStatus enumuration value:
*                         - SUCCESS: New data has been received
*                         - ERROR: time out was elapsed and no further data is 
*                                  received
*******************************************************************************/
//-------------v1.25 2009-12-18 edit1:start
//正确返回0
//参数错误1
//超时返回2
//发送错误次数超过返回3
uint8 USARTDoWith_ByteReceive(uint8 *Data, uint32 TimeOut)
{
	uint8 i;
	uint16 tmp=0,tmp2=0;
	uint16 tmpu1,tmpu2;
	if(TimeOut==0)
		return error_p;
	tmpu1=(TimeOut>>16);
	tmpu2=(TimeOut&0xffff);
  card_errresum=0;//卡片发送奇偶错误次数记录
	while(tmpu1>0)
	{
		TIM2_Start(0xffff);
		switch(Select_Card_NO)
		{
#if (Use_CardLun_FIRST==1)			
			case CARD_FIRST:
				while((TIM2_Stop()!=is_over)&&(USART_GetFlagStatus(CARD_UARTX, USART_FLAG_RXNE) == RESET)&&(card_errresum<card_errresumlong));
			break;
#else 
#endif			
			default:
				while((TIM2_Stop()!=is_over)&&(USART_GetFlagStatus(SAM_UARTX, USART_FLAG_RXNE) == RESET)&&(card_errresum<card_errresumlong));
			break;
		}
	    		
		TIM2_Close();
		if(card_errresum>=card_errresumlong)//重发次数超过
		return error_resum;
		if(Timer2_over==not_over)
		{
			if(Parameter[Select_Card_NO].CARD_DIRECT!=DIRECT)
  			{
				switch(Select_Card_NO)
				{
#if (Use_CardLun_FIRST==1)					
					case CARD_FIRST:
						tmp=(uint16)USART_ReceiveData_MY(CARD_UARTX);
					break;
#else 
#endif					
					default:
						tmp=(uint16)USART_ReceiveData_MY(SAM_UARTX);
					break;
				}
		 			
  			for(i=0;i<8;i++)
  			tmp2|=(tmp<<(15-2*i))&(1<<(15-i)); 		
  			*Data=(uint8)((~(tmp2>>8))&0xff); 		
  		}
  		else//社保仅支持正向卡
		{	
			  switch(Select_Card_NO)
			{
#if (Use_CardLun_FIRST==1)				
				case CARD_FIRST:
					*Data =(uint16)USART_ReceiveData_MY(CARD_UARTX);
				break;
#else 
#endif				
				default:
					*Data =(uint16)USART_ReceiveData_MY(SAM_UARTX);
				break;
			}
		}
  			
   		return error_ok;	
		}
		tmpu1-=1;
	}
	
	TIM2_Start(tmpu2);
	switch(Select_Card_NO)
	{
#if (Use_CardLun_FIRST==1)		
		case CARD_FIRST:
		while((TIM2_Stop()!=is_over)&&(USART_GetFlagStatus(CARD_UARTX, USART_FLAG_RXNE) == RESET)&&(card_errresum<card_errresumlong));
		break;
#else 
#endif		
		default:
		while((TIM2_Stop()!=is_over)&&(USART_GetFlagStatus(SAM_UARTX, USART_FLAG_RXNE) == RESET)&&(card_errresum<card_errresumlong));
		break;
	}

	
	TIM2_Close();
	if(card_errresum>=card_errresumlong)//重发次数超过
		return error_resum;
	
	if(Timer2_over==not_over)
	{
		if(Parameter[Select_Card_NO].CARD_DIRECT!=DIRECT)
  		{	
			switch(Select_Card_NO)
			{
#if (Use_CardLun_FIRST==1)				
				case CARD_FIRST:
				tmp=(uint16)USART_ReceiveData_MY(CARD_UARTX);
				break;
#else 
#endif				
				default:
				tmp=(uint16)USART_ReceiveData_MY(SAM_UARTX);
				break;
			}
  			for(i=0;i<8;i++)
  			tmp2|=(tmp<<(15-2*i))&(1<<(15-i)); 		
  			*Data=(uint8)((~(tmp2>>8))&0xff); 		
  		}
  		else
		{	
			switch(Select_Card_NO)
			{
#if (Use_CardLun_FIRST==1)				
				case CARD_FIRST:
				*Data =(uint16)USART_ReceiveData_MY(CARD_UARTX);
				break;
#else 
#endif				
				default:
				*Data =(uint16)USART_ReceiveData_MY(SAM_UARTX);
				break;
			}
		}
  		
   		return error_ok;
	}
	else
		return error_timeout;
}

/*******************************************************************************
* Function Name  : SC_ParityErrorHandler
* Description    : Resends the byte that failed to be received (by the Smartcard)
*                  correctly.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM2_irq(void)
{	
  if (TIM2->SR & (1<<0)) 
  {                        
    TIM2->SR &= ~(1<<0);                          // clear UIF flag
    Timer2_over=is_over;
  }
} // end TIM2_IRQHandler
//HCLK=48MHZ,PCLK1=48/2,TX=48/2*2
//HCLK=72MHZ,PCLK1=72/2,TX=72/2*2
void TIM2_Init(uint16 Prescale)//需要注意下时钟是否是72MHZ
{ 
	if((RCC->APB1ENR&RCC_APB1ENR_TIM2EN)==0)
  	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  	//2013-8-3精确定时，分频为48MHz/P+1
  TIM2->PSC = Prescale*16-1;//16;                               // set prescaler

  TIM2->CR1 &= ~0x0001;                              // disable timer
  TIM2->SR &= ~(1<<0);                          // clear UIF flag
  Timer2_over=not_over;
  TIM2->CR1 = (1<<7)|(1<<4)|(1<<3)|(1<<2);

  TIM2->CR2 = 0;
  TIM2->DIER = __TIM2_DIER;                             // enable interrupt
  NVIC->ISER[0] |= (1 << (TIM2_IRQn& 0x1F));     // enable interrupt
  TIM2->EGR|=(1<<0);//更新预分频寄存器
	TIM2->ARR = 40000;
  TIM2->CNT=40000;  
}
void TIM2_Start(uint16 delay)
{			
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	TIM2->CR1 &= ~0x0001;                              // disable timer
	TIM2->SR &= ~(1<<0);                          // clear UIF flag	
	TIM2->ARR = delay-1;
  TIM2->CNT=delay-1;
  Timer2_over=not_over;
  TIM2->CR1 |= TIMX_CR1_CEN;	
  //GPIO_ResetBits(DS1_AUX1_PORT, DS1_AUX1_PIN);  
}
void TIM2_Close(void)
{ 
	TIM2->CR1 &= ~0x0001;                              // disable timer
	TIM2->SR &= ~(1<<0);                          // clear UIF flag	
}

void delay_Init(void)//分别初始化
{  //2013-9-6
	uint32 tmp;
	//delay_time[Select_Card_NO]=(uint32)Parameter[Select_Card_NO].CARD_W*Parameter[Select_Card_NO].CARD_D*960+12;//时间不够
	//20140822问题6，7
	//delay_time[Select_Card_NO]=10800;
	delay_time[Select_Card_NO]=(uint32)Parameter[Select_Card_NO].CARD_W*Parameter[Select_Card_NO].CARD_D*960+12+100;
	if((Parameter[Select_Card_NO].CARD_N==255)||(Parameter[Select_Card_NO].CARD_N==0))
	{
		if(Parameter[Select_Card_NO].CARD_PROTOCOL==T1_PROTOCOL) 	delay_byte[Select_Card_NO]=11;
		else 	delay_byte[Select_Card_NO]=12;
	}
	else 
	{
	 	delay_byte[Select_Card_NO]=(uint16)(12+Parameter[Select_Card_NO].CARD_N);
	}
	//T1
	tmp=1;
	delay_time_t1byte[Select_Card_NO]=(tmp<<Parameter[Select_Card_NO].CARD_CWI)+11;
	//delay_time_t1byte[Select_Card_NO]=(uint32)(1<<Parameter[Select_Card_NO].CARD_CWI)+11;//错误pow(double x,double y)x的y次方
	//delay_time_t1wait[Select_Card_NO]=((uint32)1<<Parameter[Select_Card_NO].CARD_BWI)*960;
	tmp=1;
	delay_time_t1wait[Select_Card_NO]=(tmp<<Parameter[Select_Card_NO].CARD_CWI)*960+11;
}
void Parameter_Init(uint8 SC_Voltagetmp)
{
	Parameter[Select_Card_NO].CARD_DIRECT=DIRECT;
	Parameter[Select_Card_NO].CARD_PROTOCOL=T0_PROTOCOL;
	Parameter[Select_Card_NO].CARD_Voltage=SC_Voltagetmp;
	Parameter[Select_Card_NO].CARD_F=372;
	Parameter[Select_Card_NO].CARD_D=1;
	Parameter[Select_Card_NO].CARD_Ipp=50;
	Parameter[Select_Card_NO].CARD_PI=5;
	Parameter[Select_Card_NO].CARD_N=0;
	Parameter[Select_Card_NO].CARD_W=10;
	Parameter[Select_Card_NO].CARD_TRANSFER_STATION=CARD_MODE_Consult;
	Parameter[Select_Card_NO].CARD_TYPE_STATION=Card_OK;
	Parameter[Select_Card_NO].CARD_CWI=13;
	Parameter[Select_Card_NO].CARD_BWI=4;
	Parameter[Select_Card_NO].CARD_IFSC=32;
	delay_Init();
}
void SC_A2RWord_Init(void)
{
	uint8 i;
	SC_A2RWord.TS=0;
	SC_A2RWord.T0=0;
	SC_A2RWord.Tlength=0;
	SC_A2RWord.Hlength=0;
	for(i=0;i<3;)
	{
		SC_A2RWord.TA[i]=-1;
		SC_A2RWord.TB[i]=-1;
		SC_A2RWord.TC[i]=-1;
		SC_A2RWord.TD[i]=-1;
		i+=1;
		}
		SC_A2RWord.TD[3]=-1;
		for(i=0;i<16;i++)
			SC_A2RWord.H[i]=-1;
		SC_A2RWord.TCK=-1;		
}

/**************************************************/
/***********************************************/
int16 Card_SelectLun(uint8 lun)
{
	if(Select_Card_NO==lun) return 0;
	Select_Card_NO=lun;
	TIM2->PSC = Parameter[Select_Card_NO].CARD_F*12;//16;
	//需要做其他的一些操作
	//串口波特率
	//定时器初始化
   	switch(Select_Card_NO)
	{
		case CARD_FIRST:
	
		break;
		case CARD_SECOND:
		case CARD_THIRD:
		case CARD_FOURTH:
		case CARD_FIFTH:
		case CARD_SIXTH:
			IO_SelectCard(Select_Card_NO);
			SAM_UART_InitAgain(other_data);
		break;
		default:

		break;
	}
	return 0;
}

uint16 SC_FindReplyResetWord(uint16 *Length,uint8 * Atr)
{
	uint8 i,Data,crc,to_useonly,k;
	//TS需要屏蔽奇偶校验位
	switch(Select_Card_NO)
	{
#if (Use_CardLun_FIRST==1)		
		case CARD_FIRST:
			//Enable the NCN6001_UART Parity Error Interrupt 
  			USART_ITConfig(CARD_UARTX, USART_IT_PE, DISABLE);

  			// Enable the NCN6001_UART Framing Error Interrupt 
  			USART_ITConfig(CARD_UARTX, USART_IT_ERR, DISABLE);
  	
			(void)USART_ReceiveData_MY(CARD_UARTX);
		break;
#else 
#endif		
		case CARD_SECOND: //大卡座
		case CARD_THIRD:
		case CARD_FOURTH:
		case CARD_FIFTH:
		case CARD_SIXTH:
			// Enable the NCN6001_UART Parity Error Interrupt 
  			USART_ITConfig(SAM_UARTX, USART_IT_PE, DISABLE);

  			// Enable the NCN6001_UART Framing Error Interrupt 
  			USART_ITConfig(SAM_UARTX, USART_IT_ERR, DISABLE);
  	
			(void)USART_ReceiveData_MY(SAM_UARTX);
		break;
		default:
			return IFD_ParameterError;
	}
	//timesavvv1=systicnum;	
//400～40000时钟周期内需要外加一字节发送12ETU
//k=USARTDoWith_ByteReceive(&Data, 11116);//过小
//k=USARTDoWith_ByteReceive(&Data,   11250);//40050
//k=USARTDoWith_ByteReceive(&Data,     12500);//12.45ms为9600波特率固定时间44464时钟，过大
//k=USARTDoWith_ByteReceive(&Data,     11500);//2014-08-26待测试，偏小
//20141027问题2
k=USARTDoWith_ByteReceive(&Data,     12500);//2018-01-17测试
//k=USARTDoWith_ByteReceive(&Data,     12000);//2014-10-16待测试
//2014-08-08问题2
 	if( k== error_resum)//错误次数	
 	{
 		return IFD_ICC_TSError;//IFD_ICC_TypeError;
 		}
 	else if((k==error_p)||(k==error_timeout))
  	{	
		//	timesavvv2=systicnum;	
	return IFD_ICC_NoResponse;
	}
	else ;//成功
//		LED3_run(0);
  //重新对T2校对频率
  TIM2_Close();
  TIM2_Init(Parameter[Select_Card_NO].CARD_F);

	switch(Select_Card_NO)
	{
#if (Use_CardLun_FIRST==1)		
		case CARD_FIRST:
			// Enable the NCN6001_UART Parity Error Interrupt 
  			USART_ITConfig(CARD_UARTX, USART_IT_PE, ENABLE);

  			// Enable the NCN6001_UART Framing Error Interrupt 
  			USART_ITConfig(CARD_UARTX, USART_IT_ERR, ENABLE);
		break;
#else 
#endif		
		case CARD_SECOND: //大卡座
		case CARD_THIRD:
		case CARD_FOURTH:
		case CARD_FIFTH:
		case CARD_SIXTH:
			 /* Enable the NCN6001_UART Parity Error Interrupt */
  			USART_ITConfig(SAM_UARTX, USART_IT_PE, ENABLE);

  			/* Enable the NCN6001_UART Framing Error Interrupt */
  			USART_ITConfig(SAM_UARTX, USART_IT_ERR, ENABLE);
		break;
		default:
		  	return IFD_ParameterError;
	}
	SC_A2RWord.TS=Data;
	*Atr++=Data;
	if(SC_A2RWord.TS==0x3b)
  	Parameter[Select_Card_NO].CARD_DIRECT=DIRECT;
  //2013-8-3不支持逆向卡
  //else if(SC_A2R[Select_Card_NO].TS==0x03)
  //	Parameter[Select_Card_NO].CARD_DIRECT=INDIRECT;	
  else//数据出错
  	{return IFD_ICC_TSError;}
  	
  	//---T0
  	k=USARTDoWith_ByteReceive(&Data, atr_delayclksum);
 	if( k== error_resum)//错误次数	
 	{
 		return IFD_ICC_TypeError;
 		}
 	else if((k==error_p)||(k==error_timeout))
  	{	
		//	timesavvv2=systicnum;	
	return IFD_ICC_NoResponse;
	}
	else ;//成功

  SC_A2RWord.T0=Data;
  crc=Data;
  *Atr++=Data;
  to_useonly=0;//只使用T=0
  SC_A2RWord.TD[0]=Data;
  SC_A2RWord.Hlength = SC_A2RWord.T0 & (u8)0x0F;//历史字符长度
  SC_A2RWord.Tlength =2;
	//接收余下的数据
  for(i=0;i<3;)
  {
  	if(SC_A2RWord.TD[i]&0x10)//TA
  	{	
  		k=USARTDoWith_ByteReceive(&Data, atr_delayclksum);
 			if( k== error_resum)//错误次数	
 			{
 				return IFD_ICC_TypeError;
 			}
 			else if((k==error_p)||(k==error_timeout))
  		{	
				return IFD_ICC_T0Error;//IFD_ICC_NoResponse;
			}
			else ;//成功
  		  			
  		SC_A2RWord.TA[i]=Data;
  		SC_A2RWord.Tlength+=1;
  		crc^=Data;
  		*Atr++=Data;
  	}
  	if(SC_A2RWord.TD[i]&0x20)//TB
  	{	
  		k=USARTDoWith_ByteReceive(&Data, atr_delayclksum);
 			if( k== error_resum)//错误次数	
 			{
 				return IFD_ICC_TypeError;
 			}
 			else if((k==error_p)||(k==error_timeout))
  		{	
				return IFD_ICC_T0Error;//IFD_ICC_NoResponse;
			}
			else ;//成功
  			  			
  		SC_A2RWord.TB[i]=Data;
  		SC_A2RWord.Tlength+=1;
  		crc^=Data;
  		*Atr++=Data;
  	}
  	if(SC_A2RWord.TD[i]&0x40)//TC
  	{	
  		k=USARTDoWith_ByteReceive(&Data, atr_delayclksum);
 			if( k== error_resum)//错误次数	
 			{
 				return IFD_ICC_TypeError;
 			}
 			else if((k==error_p)||(k==error_timeout))
  		{	
				return IFD_ICC_T0Error;//IFD_ICC_NoResponse;
			}
			else ;//成功
  						
  		SC_A2RWord.TC[i]=Data;
  		SC_A2RWord.Tlength+=1;
  		crc^=Data;
  		*Atr++=Data;
  	}
  	if(SC_A2RWord.TD[i]&0x80)//TD
  	{	
  		k=USARTDoWith_ByteReceive(&Data, atr_delayclksum);
 			if( k== error_resum)//错误次数	
 			{
 				return IFD_ICC_TypeError;
 			}
 			else if((k==error_p)||(k==error_timeout))
  		{	
				return IFD_ICC_T0Error;//IFD_ICC_NoResponse;
			}
			else ;//成功
 			 			
  		SC_A2RWord.TD[i+1]=Data;
  		SC_A2RWord.Tlength+=1;
  		crc^=Data;
  		*Atr++=Data;
  		//if((Data&0x0f)==0x0f)//社保T=15存在，则需要发送TCK
  		if(Data&0x0f)//ISO7816,非只使用T=0
  			to_useonly=1;
  	}
  	else//没有后续
  	{
  		break;
  		} 	
  	i+=1;	
  }
  //接收历史字符
  for(i=0;i<SC_A2RWord.Hlength;)
  {
  	k=USARTDoWith_ByteReceive(&Data, atr_delayclksum);
 			if( k== error_resum)//错误次数	
 			{
 				return IFD_ICC_TypeError;
 			}
 			else if((k==error_p)||(k==error_timeout))
  		{	
				return IFD_ICC_T0Error;//IFD_ICC_NoResponse;
			}
			else ;//成功
  			
  		SC_A2RWord.H[i]=Data; 
  		SC_A2RWord.Tlength+=1;
  		crc^=Data;
  		*Atr++=Data;	
  	i+=1;	
  }
  //TCK
  if(to_useonly==1)
  {
  	k=USARTDoWith_ByteReceive(&Data, atr_delayclksum);
 			if( k== error_resum)//错误次数	
 			{
 				return IFD_ICC_TypeError;
 			}
 			else if((k==error_p)||(k==error_timeout))
  		{	
				return IFD_ICC_T0Error;//IFD_ICC_NoResponse;
			}
			else ;//成功
  			
  		SC_A2RWord.TCK=Data; 
  		SC_A2RWord.Tlength+=1;
  		crc^=Data;
  		*Atr++=Data;
  		if(crc!=0) 
  			return IFD_ICC_TypeError;
  }
  *Length=SC_A2RWord.Tlength;
	//处理参数
	if(SC_A2RWord.TD[1]!=-1)//TD1存在，
	{
		//ISO7816
		if(((SC_A2RWord.TD[1]&0x0f)==0)||((SC_A2RWord.TD[1]&0x0f)==1))
		//社保规范只支持T=0
		//if((SC_A2RWord.TD[1]&0x0f)==0)
		{
			Parameter[Select_Card_NO].CARD_PROTOCOL=SC_A2RWord.TD[1]&0x0f;
		}
		else
			return IFD_ICC_TypeError;
	}
	
	//ISO7816
	if(SC_A2RWord.TA[1]!=-1)//TA2存在为专用模式
	{
		Parameter[Select_Card_NO].CARD_TRANSFER_STATION=CARD_MODE_Special;	
		if(((SC_A2RWord.TA[1]&0x10)==0)&&(SC_A2RWord.TA[0]!=-1))//bit4=0,使用TA1确定的F,D
		{ 
			//if(SC_A2RWord.TA[0]!=-1)//TA1存在
			{
			Parameter[Select_Card_NO].CARD_F=F_Table[((SC_A2RWord.TA[0]&0xf0)>>4)&0x0f];
    	Parameter[Select_Card_NO].CARD_D=D_Table[SC_A2RWord.TA[0]&0x0f];
      }
    	Parameter[Select_Card_NO].CARD_PROTOCOL = SC_A2RWord.TA[1] & (uint8)0x0F;
		}
	}
	else//不存在，为协商模式
	{
		Parameter[Select_Card_NO].CARD_TRANSFER_STATION=CARD_MODE_Consult;
		if(SC_A2RWord.TA[0]!=-1)//TA1存在，确定FD
		{
			//协商模式因为F数组第0，1都为372，故需要保持A1
			Parameter[Select_Card_NO].CARD_F=F_Table[(SC_A2RWord.TA[0]>>4)&0x0f];
    	Parameter[Select_Card_NO].CARD_D=D_Table[SC_A2RWord.TA[0]&0x0f];		
		}		
	}
  //PI,I
  //Parameter[Select_Card_NO].CARD_Ipp=50;
	//Parameter[Select_Card_NO].CARD_PI=5;
	if((SC_A2RWord.TB[0]!=-1)&&((SC_A2RWord.TB[0]&0x80)==0))//TB1存在确定PI
	{
		Parameter[Select_Card_NO].CARD_Ipp=(SC_A2RWord.TB[0]>>5)&0X03;	
		Parameter[Select_Card_NO].CARD_PI=SC_A2RWord.TB[0]&0X1F;
	}
	if(SC_A2RWord.TB[1]!=-1)//TB2存在确定P
	{
		Parameter[Select_Card_NO].CARD_PI=SC_A2RWord.TB[1];	
	}
//保护时间N
  if(SC_A2RWord.TC[0]!=-1)//TC1存在确定N
  {
  	/*if((SC_A2RWord.TC[0]==0)||(SC_A2RWord.TC[0]==255))
  	{
  		//需要处理T=0,T=1
  		Parameter[Select_Card_NO].CARD_N=0;
  	}
  	else*///留到delay_Init(void)再处理
  	{
  		//需要处理T=0,T=1
  		Parameter[Select_Card_NO].CARD_N=SC_A2RWord.TC[0];
  	}
  		
  }
  //字符最大时间间隔
  if(SC_A2RWord.TC[1]!=-1)//TC2存在确定W
  {
  	Parameter[Select_Card_NO].CARD_W=SC_A2RWord.TC[1];
  }
  //T=1协议处理
  //原未判断是否存在
  if(((SC_A2RWord.TD[2]&0x0f)==1)&&(SC_A2RWord.TD[2]!=-1))//TD2存在且T=1确定协议参数
  {//第一次出现T=1
  	Parameter[Select_Card_NO].CARD_PROTOCOL=T1_PROTOCOL;
  	if(SC_A2RWord.TA[2]!=-1)//TA3存在确定IFSC
  		Parameter[Select_Card_NO].CARD_IFSC=SC_A2RWord.TA[2];	
  	if(SC_A2RWord.TB[2]!=-1)//TB3存在确定CWI,BWI
  	{	
  		Parameter[Select_Card_NO].CARD_CWI=(SC_A2RWord.TB[2]&0x0f);
  		Parameter[Select_Card_NO].CARD_BWI=((SC_A2RWord.TB[2]>>4)&0x0f);
  	}
  	//还有一个校验码需要用TC3来确定
  }
  //留到后面再处理
  /*
  //初始化各个时间参数
  delay_Init();//分别初始化
  //判断社保标准
  //TA1任意值
  //TB1任意值
  //TC1任意值
  //TD1拒绝T=0,T=1外的卡,放置前面处理
  //if(((SC_A2RWord.TD[1]&0x0f)!=0)&&((SC_A2RWord.TD[1]&0x0f)!=1))//TD1存在，
  //	return IFD_ICC_TypeError;
  //TA2任意值，若专有模式，且FD改变，应该后续立即使用该条件
  if(((SC_A2RWord.TA[1]&0x10)==0)&&(SC_A2RWord.TA[0]!=-1))//该算法已证实TA[1]存在，故不再做存在判断
		{ 
			//FD改变了
			if((Parameter[Select_Card_NO].CARD_F!=372)||(Parameter[Select_Card_NO].CARD_D!=1))
			{
				switch(Select_Card_NO)
				{
					case CARD_FIRST:
						SC1_UART_InitAgain(other_data);
					break;
					case CARD_SECOND: //大卡座
					case CARD_THIRD:
					case CARD_FOURTH:
					case CARD_FIFTH:
					case CARD_SIXTH:
						SAM_UART_InitAgain(other_data);
					break;
					default:
					return IFD_ParameterError;
	      }
			}
     }
     //注意若TC1改变了字符间距时间，需要重新处理保护时间
     if(SC_A2RWord.TC[0]!=-1)//TC1存在
     {
     		if((SC_A2RWord.TC[0]!=0)&&(SC_A2RWord.TC[0]!=255))
     		{
     			switch(Select_Card_NO)
					{
						case CARD_FIRST:
							USART_SetGuardTime(DS1_UARTX, delay_byte[Select_Card_NO]-10);
						break;
						case CARD_SECOND: //大卡座
						case CARD_THIRD:
						case CARD_FOURTH:
						case CARD_FIFTH:
						case CARD_SIXTH:
							USART_SetGuardTime(SAM_UARTX, delay_byte[Select_Card_NO]-10);
						break;
						default:
							return IFD_ParameterError;
					}	
     		}
     		
     }*/
     
  	//TB2任意值
  	//TC2任意值
    //TD2拒绝T=0,T=F外的卡
    /*if(SC_A2RWord.TD[2]!=-1)
    {
    	if(((SC_A2RWord.TD[2]&0x0f)!=0)&&((SC_A2RWord.TD[2]&0x0f)!=0x0f))//TD1存在，
  			return IFD_ICC_TypeError;
    }
    //TA3拒绝非XI=0,1,2，UI=1，2，3的卡
    if(SC_A2RWord.TA[2]!=-1)
    {
    	//if(((SC_A2RWord.TA[2]&0x3f)!=1)&&((SC_A2RWord.TA[2]&0x3f)!=2)&&((SC_A2RWord.TA[2]&0x3f)!=3))
    	//	return IFD_ICC_TypeError;	
			//UI在1，2，3
			if(((SC_A2RWord.TA[2]&0x3f)!=1)&&((SC_A2RWord.TA[2]&0x3f)!=2)&&((SC_A2RWord.TA[2]&0x3f)!=3))
    		return IFD_ICC_TypeError;
			//XI=0,1,2
			//if(((SC_A2RWord.TA[2]&0xc0)!=0)&&((SC_A2RWord.TA[2]&0xc0)!=1)&&((SC_A2RWord.TA[2]&0xc0)!=2))
    	//	return IFD_ICC_TypeError;
    	//原错误
    	//TA3的XI为任意值
			
    }
    //TB3任意值
  	//TC3任意值
    //TCK在T=0，且T=15不存在时应该拒绝回送了TCK的卡
    if(SC_A2RWord.TCK!=-1)
    {
    	if((Parameter[Select_Card_NO].CARD_PROTOCOL==T0_PROTOCOL)&&	(to_useonly==0))
    		return IFD_ICC_TypeError;
    }*/
    Card_RunStatus[Select_Card_NO]=Card_Staus_ADPUing;
  return Card_OK;
}

uint16 SC_Rest(uint8 cold_warm,uint8 SC_Voltagetmp,uint16 *Length,uint8 * Atr)
{
  uint16 i;
  uint8 crc=0;

 // uint8 tmp;
  //上电，上时钟
  //if((Select_Card_NO==CARD_FIRST)||(Select_Card_NO==CARD_SECOND))    
  //初始化各参数
  //SC_A2R_Init();
  SC_A2RWord_Init();
  Parameter_Init(SC_Voltagetmp);
  
  //开始的分频为1M
  TIM2_Close();
  TIM2_Init(4);//TIM2_Init(Parameter[Select_Card_NO].CARD_F);
  
  
  pcb_tmp[Select_Card_NO]=0;
  NAD_tmp[Select_Card_NO]=0;
//冷复位跟热复位有区别
  if(cold_warm==coldrest)//冷复位需要初始化串口以及其他的初始化
  {
  	switch(Select_Card_NO)
  	{
#if (Use_CardLun_FIRST==1)  		
  		case CARD_FIRST: //大卡座 			
  			Card_RunStatus[Select_Card_NO]=Card_Staus_Poweroff;
  			SC1_CMDVCCH;	
  			RST_Select(CARD_FIRST);//冷复位H电平			
  			SC_UART_Init();
  			for(i=0;i<5000;i++);  
  			break;
#else 
#endif  			
  		case CARD_SECOND: //大卡座
		case CARD_THIRD:
		case CARD_FOURTH:
		case CARD_FIFTH:
		//case CARD_SIXTH:
			 Card_RunStatus[Select_Card_NO]=Card_Staus_Poweroff;
       RST_Select(Select_Card_NO);
  		SAM_UART_ColdInit();//SAM因为共用，故不做再次处理
  		for(i=0;i<5000;i++);
			break;
			default:
				break;
  		}
  	
  }

  /******************复位**************************/
  switch(Select_Card_NO)
  {
#if (Use_CardLun_FIRST==1)  	
  	case CARD_FIRST: //大卡座
		if(SC_Voltagetmp==Card_30V)
		   SC1_35VL;
		else 
		   SC1_35VH;
		
		//RST复位	，兼容热复位
		RST_Select(CARD_FIRST);
		for(i=0;i<5000;i++);
//需要延时至少400时钟_2013-5-24
    SC1_CMDVCCL;
		for(i=0;i<5000;i++);
     RST_Select(CARD_FIRST|0X10);
		Card_PowerStatus=1;
		break;
#else 
#endif		
	    case CARD_SECOND: //大卡座
		case CARD_THIRD:
		case CARD_FOURTH:
		case CARD_FIFTH:
		case CARD_SIXTH:		
		//RST复位，兼容热复位
		RST_Select(Select_Card_NO);
		for(i=0;i<5000;i++);
		RST_Select(Select_Card_NO|0X10);
		for(i=0;i<5000;i++);		
		break;
	default:
		return IFD_ParameterError;
  }
	i=SC_FindReplyResetWord(Length,Atr); 
  if(i!=Card_OK)//超时或不支持卡型
		return i;
  
		return Card_OK;
	
}

uint16 Card_CoolRest(uint8 SC_Voltagetmp,uint16 *Length,uint8 * Atr) 
{
	uint16 i,j;

	if(Select_Card_NO==CARD_FIRST)
	{
		if(CPUCard_Insert==CARD_NONEInsert)return IFD_ICC_NoExist;//无卡
		}

	Card_RunStatus[Select_Card_NO]=Card_Staus_Poweroff;
   i=j=Card_OK;
#if (Use_CardLun_FIRST==1)   
	if(Select_Card_NO==CARD_FIRST)//CPU卡
	{
		if(Card_TypeSave==CardType_sam0)//D3_cardtype_cpu)
		{ 
			
			i=SC_Rest(coldrest,SC_Voltagetmp,Length,Atr);
			//需要判断是否有数据接收但错误还是无数据
			//20140808问题3，4
			//20141027问题3
			if((i==IFD_ICC_TSError)||(i==IFD_ICC_T0Error))//需要热复位
			{
				Delay_Ms(10);
				
				j=SC_Rest(warmrest,SC_Voltagetmp,Length,Atr);
				//需要判断是否有数据接收但错误还是无数据
			  if(j==IFD_ICC_NoResponse)//无数据
			  {
					//释放接口	
					Card_PowerOff();
					return IFD_ICC_NoResponse;
				}
				else if((j==IFD_ICC_TypeError)||(j==IFD_ParameterError)||(j==IFD_ICC_TSError)||(j==IFD_ICC_T0Error))//冷复位失败转热复位
				{
						//释放接口	
					Card_PowerOff();
					return IFD_ICC_TypeError;
				}
				else ;//return 0;	

			}
			else if(i==Card_OK)//进入下一步
			{
				;
			}
			else if(i==IFD_ICC_NoResponse)//无数据
			{
				//释放接口	
				Card_PowerOff();
				return IFD_ICC_NoResponse;
			}
			else//其他错误都释放
			{
				//释放接口	
				Card_PowerOff();
				return IFD_ICC_TypeError;
			}
#if (AUto_Card_PPS==1)
			//进行PPS	
			//如果协商模式，且FD发生变化
			if((Parameter[Select_Card_NO].CARD_TRANSFER_STATION==CARD_MODE_Consult)
				 &&((Parameter[Select_Card_NO].CARD_F!=372)||(Parameter[Select_Card_NO].CARD_D!=1)))	
			{
				j=Card_Protocol(0,Parameter[Select_Card_NO].CARD_F,Parameter[Select_Card_NO].CARD_D);
				if(j!=Card_OK)	return IFD_ICC_TypeError;
				//是否需要改变波特率
				//串口初始化已经改变了波特率及安全时间
				delay_Init();
				if(((Parameter[Select_Card_NO].CARD_F!=372)||(Parameter[Select_Card_NO].CARD_D!=1))||Parameter[Select_Card_NO].CARD_N!=0)
				{
						
				SC_UART_InitAgain(other_data);			    
      	//重新对T2校对频率
      	//可能存在未考虑D的问题，但未考虑D则延时会加大，先测试
     		TIM2_Close();
     		TIM2_Init(Parameter[Select_Card_NO].CARD_F);
				}	
			}		
#else 
#endif			
			 return 0;
		}

		else return 1;	
	}
	else 
#else 
#endif		
	{//SAM卡
		
		i=SC_Rest(coldrest,SC_Voltagetmp,Length,Atr);
		//需要判断是否有数据接收但错误还是无数据
			if((i==IFD_ICC_TSError)||(i==IFD_ICC_T0Error))//需要热复位
			{
				Delay_Ms(10);
				
				j=SC_Rest(warmrest,SC_Voltagetmp,Length,Atr);
				//需要判断是否有数据接收但错误还是无数据
			  if(j==IFD_ICC_NoResponse)//无数据
			  {
					//释放接口	
					Card_PowerOff();
					return IFD_ICC_NoResponse;
				}
				else if((j==IFD_ICC_TypeError)||(j==IFD_ParameterError)||(j==IFD_ICC_TSError)||(j==IFD_ICC_T0Error))//冷复位失败转热复位
				{
						//释放接口	
					Card_PowerOff();
					return IFD_ICC_TypeError;
				}
				else ;//return 0;	

			}
			else if(i==Card_OK)//进入下一步
			{
				;
			}
			else if(i==IFD_ICC_NoResponse)//无数据
			{
				//释放接口	
				Card_PowerOff();
				return IFD_ICC_NoResponse;
			}
			else//其他错误都释放
			{
				//释放接口	
				Card_PowerOff();
				return IFD_ICC_TypeError;
			}
			 return 0;		
	}	
}
uint16 Card_DoWithPPS(uint16 F,uint16 D)
{
	uint16 j;
	//如果FD发生变化
	if((Parameter[Select_Card_NO].CARD_F!=372)||(Parameter[Select_Card_NO].CARD_D!=1))//卡支持PPS
	{
		if((F!=372)||(D!=1))//需要更改
		{
			j=Card_Protocol(0,F_Table[F],D_Table[D]);
			if(j!=Card_OK)	return IFD_ICC_TypeError;
			//是否需要改变波特率
			//串口初始化已经改变了波特率及安全时间
			delay_Init();	
				SC_UART_InitAgain(other_data);			    
      	//重新对T2校对频率
      	//可能存在未考虑D的问题，但未考虑D则延时会加大，先测试
     		TIM2_Close();
     		TIM2_Init(F_Table[F]);	
		}	
	}
}
uint16 Card_WarmRest(uint16 *Length,uint8 * Atr)
{
	if(Select_Card_NO==CARD_FIRST)
	{
		if(CPUCard_Insert==CARD_NONEInsert)return IFD_ICC_NoExist;//无卡
		}

	Card_RunStatus[Select_Card_NO]=Card_Staus_Poweroff;
	return (SC_Rest(warmrest,Parameter[Select_Card_NO].CARD_Voltage,Length,Atr));
}

uint16 Card_Init(void)
{
#if (Use_CardLun_FIRST==1)	
  /***********PSAM****************/
  Select_Card_NO=CARD_FIRST;
  Parameter_Init(Card_0V);
  Card_TypeSave=CardType_sam0;//D3_cardtype_cpu;
  Card_RunStatus[Select_Card_NO]=Card_Staus_Poweroff;
  SC_UART_Init();
#else 
#endif	
  /****PSIM2************************/
#if (Use_CardLun_SECOND==1) 		
    Select_Card_NO=CARD_SECOND;
   Parameter_Init(Card_0V);
  //Card_TypeSave=HC_Card_sam1;
  Card_RunStatus[Select_Card_NO]=Card_Staus_Poweroff;
#else 
#endif
  
#if (Use_CardLun_THIRD==1) 
  Select_Card_NO=CARD_THIRD;
  Parameter_Init(Card_0V);
 // Card_TypeSave[Select_Card_NO]=HC_Card_sam2;
  Card_RunStatus[Select_Card_NO]=Card_Staus_Poweroff;
#else 
#endif  
#if (Use_CardLun_FOURTH==1) 
  Select_Card_NO=CARD_FOURTH;
  Parameter_Init(Card_0V);
  //Card_TypeSave[Select_Card_NO]=HC_Card_sam3;
  Card_RunStatus[Select_Card_NO]=Card_Staus_Poweroff;
#else 
#endif
#if (Use_CardLun_FIFTH==1) 
  Select_Card_NO=CARD_FIFTH;
  Parameter_Init(Card_0V);
  //Card_TypeSave[Select_Card_NO]=HC_Card_sam4;
  Card_RunStatus[Select_Card_NO]=Card_Staus_Poweroff;
#else 
#endif
#if (Use_CardLun_SIXTH==1) 
  Select_Card_NO=CARD_SIXTH;
  Parameter_Init(Card_0V);
 // Card_TypeSave[Select_Card_NO]=HC_Card_sam5;
  Card_RunStatus[Select_Card_NO]=Card_Staus_Poweroff;
#else 
#endif
 
 /* 
 
//  SAM1_IOH;//通道切换到SAM1
*/	
  SAM_UART_Init();
  TIM2_Init(Parameter[Select_Card_NO].CARD_F);
	Select_Card_NO=CARD_SECOND;
	IO_SelectCard(CARD_SECOND);
  return Card_OK;	  
}


/************************************************************/
//释放后RST始终为L
int16 Card_PowerOff(void)
{	
	//uint8 tmp;
	GPIO_InitTypeDef GPIO_InitStructure;
	Card_RunStatus[Select_Card_NO]=Card_Staus_Poweroff;
	switch(Select_Card_NO)
	{
#if (Use_CardLun_FIRST==1)		
			case CARD_FIRST:
				RST_Select(CARD_FIRST|0X10);//rst	L
				//SC1_35VH;//结合CMD进入停止模式
				SC1_CMDVCCH; //cmd
		

		if(CPUCard_Insert==CARD_NONEInsert)return IFD_ICC_NoExist;//无卡

				Delay_Ms(10);
				Card_RunStatus[Select_Card_NO]=Card_Staus_Poweroff;
				Card_PowerStatus=0;
			break;
#else 
#endif			
			case CARD_SECOND: //大卡座
			case CARD_THIRD:
			case CARD_FOURTH:
			case CARD_FIFTH:
			case CARD_SIXTH:
				 //RST H
				 RST_Select(Select_Card_NO|0X10);
				 
				 Delay_Ms(3);
				 Card_RunStatus[Select_Card_NO]=Card_Staus_Poweroff;
				//RST_Default();
				 //掉电
				 //SAM1_35VL;
			break;			
			default:

			break;
	}
	Parameter_Init(Card_0V);
	return 0;
}
//-----------------
uint16 Card_Apdu(word *Length,uint8 * Command,uint8 * Response)
{
	uint16 tmp,tmp2,leng,i,leng2;
	uint8 com_tmp[5]={0,0xc0,0,0,0};
	uint8 *res;
		
		if(Select_Card_NO==CARD_FIRST)
	{
		if(CPUCard_Insert==CARD_NONEInsert)return IFD_ICC_NoExist;//无卡
		}
		
  if(Card_RunStatus[Select_Card_NO]==Card_Staus_Poweroff)	return IFD_ICC_NoPower;//有卡未上电
  
	res=Response;
	if(Parameter[Select_Card_NO].CARD_PROTOCOL==T0_PROTOCOL)
	{ 
	 	tmp=Card_Apdu_T0(*Length,Command,res);
	 	
	 	switch(Select_Card_NO)
		{
#if (Use_CardLun_FIRST==1)			
		case CARD_FIRST:
			/* Disable the DMA Receive (Reset DMAR bit only) */  
      USART_DMACmd(CARD_UARTX, USART_DMAReq_Rx, DISABLE);
		break;
#else 
#endif		
		case CARD_SECOND: //大卡座
		case CARD_THIRD:
		case CARD_FOURTH:
		case CARD_FIFTH:
		case CARD_SIXTH:
			/* Disable the DMA Receive (Reset DMAR bit only) */  
      USART_DMACmd(SAM_UARTX, USART_DMAReq_Rx, DISABLE);
		break;
		default:
		   return IFD_ParameterError;
	//	break;
		}	
    if(tmp==Card_OverTime) return IFD_ICC_NoResponse;
    else		
		return tmp;
	}
	else 
		{
	 	tmp=Card_Apdu_T1(*Length,Command,Response);
		return tmp;
	}
}
uint16 Card_Apdu_T0(uint16 Length,uint8 * Command,uint8 * Response)
{ 	
	uint16 length,i;//,k;
	uint8 *tmp_buf,*tmp_response,k;
	uint8 type_comm;
	uint8 locData;
	uint16 leng_comm_remain;//未发送完成的命令字节数
	uint16 leng_respond_remain;//未收完的数据字节数
	uint16 sw1_sw2;
	uint8 send_ins;
	uint8 ins_tmp=0;//0正常，1得保存INS
	//------
	//sent1.5etu
switch(Select_Card_NO)
	{
#if (Use_CardLun_FIRST==1)		
		case CARD_FIRST:
			CARD_UARTX->CR2&=0XCFFF;
			CARD_UARTX->CR2|=0X3000;
		break;
#else 
#endif		
		case CARD_SECOND: //大卡座
		case CARD_THIRD:
		case CARD_FOURTH:
		case CARD_FIFTH:
		case CARD_SIXTH:
			SAM_UARTX->CR2&=0XCFFF;
			SAM_UARTX->CR2|=0X3000;
		break;
		default:
		   return 	IFD_ParameterError;
		//break;
	}
	//app中胡改变了时钟分频
	//重新对T2校对频率
    // TIM2_Close();
    // TIM2_Init(Parameter[Select_Card_NO].CARD_F);
	
	
	tmp_buf=Command;
	
	//找到命令格式
	if(Length==4)
		type_comm=Apdu_comm_0;
	else 	if(Length==5)
		type_comm=Apdu_comm_1;
	else if(Length>5)
	{
		//[4]==0则为256
		if(tmp_buf[4]==0)//256
			length=256;
		else
			length=tmp_buf[4];
		
		if(Length==(length+5))	
			type_comm=Apdu_comm_2;
		else if(Length==(length+6))	
			type_comm=Apdu_comm_3;
		else
			return IFD_ParameterError;
	}
	else
		return IFD_ParameterError;	
		
//初始化
	leng_comm_remain=0;
	leng_respond_remain=0;;
	//回复也许要初始话
	sw1_sw2=0;
	//提取命令头
	tmp_buf=Command;
// Send header -------------------------------------------------------------
 switch(Select_Card_NO)
	{
#if (Use_CardLun_FIRST==1)		
		case CARD_FIRST:
		
			// Enable the DMA Receive (Set DMAR bit only) to enable interrupt generation
     //in case of a framing error FE   
      USART_DMACmd(CARD_UARTX, USART_DMAReq_Rx, ENABLE);
			
			
			SCData = *tmp_buf++;
  		if(SC_USART_SendData(CARD_UARTX, SCData)!=0)//发送错误
  			return Card_OverTime;
  		//while(USART_GetFlagStatus(DS1_UARTX, USART_FLAG_TC) == RESET); 
  
  		send_ins=*tmp_buf++;
  		SCData =send_ins;
  		if(SC_USART_SendData(CARD_UARTX, SCData)!=0)//发送错误
  			return Card_OverTime;
  		//while(USART_GetFlagStatus(DS1_UARTX, USART_FLAG_TC) == RESET);
   
  		SCData = *tmp_buf++;
  		if(SC_USART_SendData(CARD_UARTX, SCData)!=0)//发送错误
  			return Card_OverTime;
 		//while(USART_GetFlagStatus(DS1_UARTX, USART_FLAG_TC) == RESET); 
  
  		SCData = *tmp_buf++;
  		if(SC_USART_SendData(CARD_UARTX, SCData)!=0)//发送错误
  			return Card_OverTime;
  		//while(USART_GetFlagStatus(DS1_UARTX, USART_FLAG_TC) == RESET);
		break;
#else 
#endif		
		case CARD_SECOND: //大卡座
		case CARD_THIRD:
		case CARD_FOURTH:
		case CARD_FIFTH:
		case CARD_SIXTH:
			/* Enable the DMA Receive (Set DMAR bit only) to enable interrupt generation
     in case of a framing error FE */  
  		USART_DMACmd(SAM_UARTX, USART_DMAReq_Rx, ENABLE);
			
			
			SCData = *tmp_buf++;
  		if(SC_USART_SendData(SAM_UARTX, SCData)!=0)//发送错误
  			return Card_OverTime;
  		//while(USART_GetFlagStatus(SAM_UARTX, USART_FLAG_TC) == RESET); 
  
  		send_ins=*tmp_buf++;
  		SCData =send_ins;
  		if(SC_USART_SendData(SAM_UARTX, SCData)!=0)//发送错误
  			return Card_OverTime;
  		//while(USART_GetFlagStatus(SAM_UARTX, USART_FLAG_TC) == RESET);
   
  		SCData = *tmp_buf++;
  		if(SC_USART_SendData(SAM_UARTX, SCData)!=0)//发送错误
  			return Card_OverTime;
 		//while(USART_GetFlagStatus(SAM_UARTX, USART_FLAG_TC) == RESET); 
  
  		SCData = *tmp_buf++;
  		if(SC_USART_SendData(SAM_UARTX, SCData)!=0)//发送错误
  			return Card_OverTime;
  		//while(USART_GetFlagStatus(SAM_UARTX, USART_FLAG_TC) == RESET);
		break;
		default:
			return IFD_ParameterError;
	//	break;
	}




		

  
	switch(type_comm)
	{
		case Apdu_comm_0:
			SCData=0x00;

			switch(Select_Card_NO)
		{
#if (Use_CardLun_FIRST==1)			
		case CARD_FIRST:
			if(SC_USART_SendData(CARD_UARTX, SCData)!=0)//发送错误
  			return Card_OverTime;
		break;
#else 
#endif		
		case CARD_SECOND: //大卡座
		case CARD_THIRD:
		case CARD_FOURTH:
		case CARD_FIFTH:
		case CARD_SIXTH:
			if(SC_USART_SendData(SAM_UARTX, SCData)!=0)//发送错误
  			return Card_OverTime;
		break;
		default:
		   return IFD_ParameterError;
		}	
	
			break;
		case Apdu_comm_1:
			leng_respond_remain=*tmp_buf++;
			if(leng_respond_remain==0)leng_respond_remain=256;//256
			
			SCData = leng_respond_remain;
				switch(Select_Card_NO)
		{
#if (Use_CardLun_FIRST==1)			
		case CARD_FIRST:
			if(SC_USART_SendData(CARD_UARTX, SCData)!=0)//发送错误
  			return Card_OverTime;
		break;
#else 
#endif		
		case CARD_SECOND: //大卡座
		case CARD_THIRD:
		case CARD_FOURTH:
		case CARD_FIFTH:
		case CARD_SIXTH:
			if(SC_USART_SendData(SAM_UARTX, SCData)!=0)//发送错误
  			return Card_OverTime;
		break;
		default:
		   return IFD_ParameterError;
		}
			break;
		case Apdu_comm_2:
			leng_comm_remain=*tmp_buf++;
			if(leng_comm_remain==0)leng_comm_remain=256;//256
				
			SCData = leng_comm_remain;
				switch(Select_Card_NO)
		{
#if (Use_CardLun_FIRST==1)			
		case CARD_FIRST:
			if(SC_USART_SendData(CARD_UARTX, SCData)!=0)//发送错误
  			return Card_OverTime;
		break;
#else 
#endif		
		case CARD_SECOND: //大卡座
		case CARD_THIRD:
		case CARD_FOURTH:
		case CARD_FIFTH:
		case CARD_SIXTH:
			if(SC_USART_SendData(SAM_UARTX, SCData)!=0)//发送错误
  			return Card_OverTime;
		break;
		default:
		   return IFD_ParameterError;
		}		
			break;
		case Apdu_comm_3:
			leng_comm_remain=*tmp_buf++;
			if(leng_comm_remain==0)leng_comm_remain=256;
			leng_respond_remain=*(tmp_buf+leng_comm_remain);
			if(leng_respond_remain==0)leng_respond_remain=256;
			SCData = leng_comm_remain;
				switch(Select_Card_NO)
		{
#if (Use_CardLun_FIRST==1)				
		case CARD_FIRST:
			if(SC_USART_SendData(CARD_UARTX, SCData)!=0)//发送错误
  			return Card_OverTime;
		break;
#else 
#endif		
		case CARD_SECOND: //大卡座
		case CARD_THIRD:
		case CARD_FOURTH:
		case CARD_FIFTH:
		case CARD_SIXTH:
			if(SC_USART_SendData(SAM_UARTX, SCData)!=0)//发送错误
  			return Card_OverTime;
		break;
		default:
		   return IFD_ParameterError;
		}			
			break;
		default:
			return 	IFD_ParameterError;
	}		
	length=0;  
	tmp_response=&Response[2];//从第2个字节开始，前2字节为长度
	*Response=0;
  *(Response+1)=0;
	tmp_buf=&Command[5];//命令的后续
	switch(Select_Card_NO)
	{
#if (Use_CardLun_FIRST==1)			
		case CARD_FIRST:
			(void)USART_ReceiveData_MY(CARD_UARTX);
		break;
#else 
#endif		
		case CARD_SECOND: //大卡座
		case CARD_THIRD:
		case CARD_FOURTH:
		case CARD_FIFTH:
		case CARD_SIXTH:
		   (void)USART_ReceiveData_MY(SAM_UARTX);
		break;
		default:
		   return 	IFD_ParameterError;
	}	
 //---用于测试ADPU中卡发错数据能否重发-----
//----------
	//回复处理
command_a:
	//接收0.5etu
switch(Select_Card_NO)
	{
#if (Use_CardLun_FIRST==1)		
		case CARD_FIRST:
			CARD_UARTX->CR2&=0XCFFF;
			CARD_UARTX->CR2|=0X1000;
		break;
#else 
#endif		
		case CARD_SECOND: //大卡座
		case CARD_THIRD:
		case CARD_FOURTH:
		case CARD_FIFTH:
		case CARD_SIXTH:
			SAM_UARTX->CR2&=0XCFFF;
			SAM_UARTX->CR2|=0X1000;
		break;
		default:
		   return 	IFD_ParameterError;
	}
  do//等待过程字节
  {
    locData=0;
    
    k=USARTDoWith_ByteReceive(&locData , delay_time[Select_Card_NO]);
 		if( k== error_resum)//错误次数	
 		{
 			return IFD_ICC_TypeError;
 		}
 		else if((k==error_p)||(k==error_timeout))
  	{	
			//	timesavvv2=systicnum;	
			return IFD_ICC_NoResponse;
		}
		else ;//成功

  }while (locData == 0x60);	//null
  //存储过程字节2014-06-03
  if(ins_tmp==1)
  {
  	if(((locData & (u8)0xF0) == 0x60) || ((locData & (u8)0xF0) == 0x90))//sw1,sw2
  	{
  		}
  	else//第三类指令INS
  	{
  		*tmp_response++=locData;
    	length+=1;
    	sw1_sw2 = locData;
    	ins_tmp=2;	
  	} 	
  }
  
//-------
  
  
  if(((locData & (u8)0xF0) == 0x60) || ((locData & (u8)0xF0) == 0x90))//sw1,sw2
    {//接收到SW1，等待接收SW2
    	if(ins_tmp==2)//第三类指令有INS
    	{
    		// SW1 received 
    		sw1_sw2=(sw1_sw2<<8)&0xff00;
        sw1_sw2|=(locData&0xff);
      	//sw1_sw2 = locData;
      	*tmp_response++=locData;
      	length+=1;	
      	//不接收SW2
    	}
    	else//无INS
    	{
    		// SW1 received 
      	sw1_sw2 = locData;
      	*tmp_response++=locData;
      	length+=1;
     
      	k=USARTDoWith_ByteReceive(&locData , delay_time[Select_Card_NO]);
 				if( k== error_resum)//错误次数	
 				{
 					return IFD_ICC_TypeError;
 				}
 				else if((k==error_p)||(k==error_timeout))
  			{	
					return IFD_ICC_NoResponse;
				}
				else //成功
				{
					// SW2 received 
        	sw1_sw2=(sw1_sw2<<8)&0xff00;
        	sw1_sw2|=(locData&0xff);
        	*tmp_response++=locData;
      		length+=1;
				}   		
    	}
      
      
      *Response=(uint8)((length>>8)&0xff);	
      *(Response+1)=(uint8)(length&0xff);           
      return sw1_sw2 ;    
    }
    else if(((locData^send_ins)==0)||((locData^send_ins)==0x01))//发送或接收完余下的数据
    {//ACK字节高7位全部等于或互补于INS字节的对应位
      if(leng_comm_remain!=0)//继续发送完以后的
      {
      	//sent1.5etu
				switch(Select_Card_NO)
				{
#if (Use_CardLun_FIRST==1)					
					case CARD_FIRST:
						CARD_UARTX->CR2&=0XCFFF;
						CARD_UARTX->CR2|=0X3000;
					break;
#else 
#endif					
					case CARD_SECOND: //大卡座
					case CARD_THIRD:
					case CARD_FOURTH:
					case CARD_FIFTH:
					case CARD_SIXTH:
						SAM_UARTX->CR2&=0XCFFF;
						SAM_UARTX->CR2|=0X3000;
					break;
					default:
		   		return 	IFD_ParameterError;
				}
      	
      	
      	TIM2_Start(delay_byte[Select_Card_NO]-10);
				while(TIM2_Stop()!=is_over);
				TIM2_Close();
        for(i=0;i<leng_comm_remain;i++)
        {
        	SCData = *tmp_buf++;
					switch(Select_Card_NO)
					{
#if (Use_CardLun_FIRST==1)						
						case CARD_FIRST:
							if(SC_USART_SendData(CARD_UARTX, SCData)!=0)//发送错误
  							return Card_OverTime;
							(void)USART_ReceiveData_MY(CARD_UARTX);
						break;
#else 
#endif						
						case CARD_SECOND: //大卡座
						case CARD_THIRD:
						case CARD_FOURTH:
						case CARD_FIFTH:
						case CARD_SIXTH:
							if(SC_USART_SendData(SAM_UARTX, SCData)!=0)//发送错误
  							return Card_OverTime;
							(void)USART_ReceiveData_MY(SAM_UARTX);
						break;
						default:
		   				return IFD_ParameterError;
					}			       	
        } 
         //--2014-07-10
         switch(Select_Card_NO)
				{
#if (Use_CardLun_FIRST==1)					
					case CARD_FIRST:
						USART_DMACmd(CARD_UARTX, USART_DMAReq_Rx, DISABLE);
					break;
#else 
#endif					
					case CARD_SECOND: //大卡座
					case CARD_THIRD:
					case CARD_FOURTH:
					case CARD_FIFTH:
					case CARD_SIXTH:
						USART_DMACmd(SAM_UARTX, USART_DMAReq_Rx, DISABLE);
					break;
					default:
		   			return IFD_ParameterError;
				}	
         //---end 
        leng_comm_remain=0; 
        if(type_comm==Apdu_comm_2)
        	ins_tmp=1;           
      	goto command_a;	
      }
      if(leng_respond_remain!=0)//继续接受余下的
      {
      	for(i=0;i<leng_respond_remain;i++)
      	{			
      		k=USARTDoWith_ByteReceive(&locData , delay_time[Select_Card_NO]);
 					if( k== error_resum)//错误次数	
 					{
 						return IFD_ICC_TypeError;
 					}
 					else if((k==error_p)||(k==error_timeout))
  				{	
						return IFD_ICC_NoResponse;
					}
					else //成功
      		{
      			*tmp_response++=locData;
      			length+=1;
      			*Response=(uint8)((length>>8)&0xff);	
      			*(Response+1)=(uint8)(length&0xff);
      		}
      	}
      	
      	
      	leng_respond_remain=0;
      }	 
      goto command_a;    
    }
  else if(((locData^send_ins)==0xff)||((locData^send_ins)==0xfe))//发送或接收余下的一字节数据
  {
      if(leng_comm_remain!=0)//继续发送一个字节	
      {
      TIM2_Start(delay_byte[Select_Card_NO]-10);
	while(TIM2_Stop()!=is_over);
	TIM2_Close();
      	SCData = *tmp_buf++;	
      	leng_comm_remain-=1;

		switch(Select_Card_NO)
		{
#if (Use_CardLun_FIRST==1)			
		case CARD_FIRST:
			if(SC_USART_SendData(CARD_UARTX, SCData)!=0)//发送错误
  			return Card_OverTime;
		break;
#else 
#endif		
		case CARD_SECOND: //大卡座
		case CARD_THIRD:
		case CARD_FOURTH:
		case CARD_FIFTH:
		case CARD_SIXTH:
			if(SC_USART_SendData(SAM_UARTX, SCData)!=0)//发送错误
  			return Card_OverTime;
		break;
		default:
		   return IFD_ParameterError;
		}		        
  		  goto command_a;
      }	
      if(leng_respond_remain!=0)//继续接受余下的一个字节
      {
      	k=USARTDoWith_ByteReceive(&locData , delay_time[Select_Card_NO]);
 				if( k== error_resum)//错误次数	
 				{
 					return IFD_ICC_TypeError;
 				}
 				else if((k==error_p)||(k==error_timeout))
  			{	
					return IFD_ICC_NoResponse;
				}
				else //成功
				{
					*tmp_response++=locData;
      		length+=1;
      		*Response=(uint8)((length>>8)&0xff);	
      		*(Response+1)=(uint8)(length&0xff);
				}   		   		
    		leng_respond_remain-=1;  			
      }	
      goto command_a;	     	
  }
  else
  	goto command_a;	
}

uint16 Card_Apdu_T1(uint16 Length,uint8 * Command,uint8 * Response)
{
	uint8 *tmp_buf,*tmp_buf_resp;
	uint16 tmp,leng,len,sw1_sw2,i;
	uint8 CRC_t1,k;
	uint8 locData;
	tmp_buf=Command;
	//查看命令是否错误
	if(Length>5)
	{
		if((Length!=(tmp_buf[4]+5))&&(Length!=(tmp_buf[4]+6)))
			return IFD_ParameterError;	
	}
	else if(Length<4)
		return IFD_ParameterError;
	else ;
	//---发送NAN PCB LEN命令	
	CRC_t1=0;
	SCData = NAD_tmp[Select_Card_NO];//T1_NAN;
	switch(Select_Card_NO)
		{
#if (Use_CardLun_FIRST==1)			
		case CARD_FIRST:
			SC_USART_SendData(CARD_UARTX, SCData);
		break;
#else 
#endif		
		case CARD_SECOND: //大卡座
		case CARD_THIRD:
		case CARD_FOURTH:
		case CARD_FIFTH:
		case CARD_SIXTH:
			SC_USART_SendData(SAM_UARTX, SCData);
		break;
		default:
		   return IFD_ParameterError;
		}
  
  CRC_t1^=SCData;
	SCData = pcb_tmp[Select_Card_NO];
	pcb_tmp[Select_Card_NO]=pcb_tmp[Select_Card_NO]?0:0x40;

	switch(Select_Card_NO)
		{
#if (Use_CardLun_FIRST==1)			
		case CARD_FIRST:
			SC_USART_SendData(CARD_UARTX, SCData);
		break;
#else 
#endif		
		case CARD_SECOND: //大卡座
		case CARD_THIRD:
		case CARD_FOURTH:
		case CARD_FIFTH:
		case CARD_SIXTH:
			SC_USART_SendData(SAM_UARTX, SCData);
		break;
		default:
		   return IFD_ParameterError;
		}

  CRC_t1^=SCData;
  SCData = Length; 

  switch(Select_Card_NO)
		{
#if (Use_CardLun_FIRST==1)			
		case CARD_FIRST:
			SC_USART_SendData(CARD_UARTX, SCData);
		break;
#else 
#endif		
		case CARD_SECOND: //大卡座
		case CARD_THIRD:
		case CARD_FOURTH:
		case CARD_FIFTH:
		case CARD_SIXTH:
			SC_USART_SendData(SAM_UARTX, SCData);
		break;
		default:
		   return IFD_ParameterError;
		}

  CRC_t1^=SCData;
  //---发送命令包
  for(tmp=0;tmp<Length;tmp++)
  {
  	SCData = *tmp_buf++;
	switch(Select_Card_NO)
		{
#if (Use_CardLun_FIRST==1)			
		case CARD_FIRST:
			SC_USART_SendData(CARD_UARTX, SCData);
		break;
#else 
#endif		
		case CARD_SECOND: //大卡座
		case CARD_THIRD:
		case CARD_FOURTH:
		case CARD_FIFTH:
		case CARD_SIXTH:
			SC_USART_SendData(SAM_UARTX, SCData);
		break;
		default:
		   return IFD_ParameterError;
		}
	
	CRC_t1^=SCData;	
  }
  //--发送CRC
  SCData = CRC_t1;
  switch(Select_Card_NO)
		{
#if (Use_CardLun_FIRST==1)			
		case CARD_FIRST:
			SC_USART_SendData(CARD_UARTX, SCData);
			(void)USART_ReceiveData_MY(CARD_UARTX);
		break;
#else 
#endif		
		case CARD_SECOND: //大卡座
		case CARD_THIRD:
		case CARD_FOURTH:
		case CARD_FIFTH:
		case CARD_SIXTH:
			SC_USART_SendData(SAM_UARTX, SCData);
			(void)USART_ReceiveData_MY(SAM_UARTX);
		break;
		default:
		   return IFD_ParameterError;
		}
  //--接收数据
  
  len=0;
  leng=0;
	CRC_t1=0;
	tmp_buf_resp=&Response[2];
	tmp_buf=&Response[0];
	*tmp_buf=0;
	*(tmp_buf+1)=0;
	//NAD
	k=USARTDoWith_ByteReceive(&locData , delay_time_t1wait[Select_Card_NO]);
 		if( k== error_resum)//错误次数	
 		{
 			return IFD_ICC_TypeError;
 		}
 		else if((k==error_p)||(k==error_timeout))
  	{	
			return IFD_ICC_NoResponse;
		}
		else ;//成功
	#if (ADPU_WithPCBNAD==0)
	//0 不需要PCB及NAD
	//NAD_tmp[Select_Card_NO]=locData;
	#else 
	//1 需要PCB及NAD
	*tmp_buf_resp++=locData;
	leng++;
	#endif
	

	CRC_t1^=locData;
	//leng++;
	*tmp_buf=(leng>>8)&0xff;
	*(tmp_buf+1)=leng&0xff;
	//PCB
	k=USARTDoWith_ByteReceive(&locData , delay_time_t1byte[Select_Card_NO]);
 		if( k== error_resum)//错误次数	
 		{
 			return IFD_ICC_TypeError;
 		}
 		else if((k==error_p)||(k==error_timeout))
  	{	
			return IFD_ICC_NoResponse;
		}
		else ;//成功
	#if (ADPU_WithPCBNAD==0)
	//0 不需要PCB及NAD,pcb,nad都在main中处理
	//pcb_tmp[Select_Card_NO]=locData;	
	#else 
	//1 需要PCB及NAD
	*tmp_buf_resp++=locData;
	leng++;
	#endif
	
	
	
	CRC_t1^=locData;
	//leng++;
	*tmp_buf=(leng>>8)&0xff;
	*(tmp_buf+1)=leng&0xff;
	//LEN
	k=USARTDoWith_ByteReceive(&locData , delay_time_t1byte[Select_Card_NO]);
 		if( k== error_resum)//错误次数	
 		{
 			return IFD_ICC_TypeError;
 		}
 		else if((k==error_p)||(k==error_timeout))
  	{	
			return IFD_ICC_NoResponse;
		}
		else ;//成功
#if (ADPU_WithPCBNAD==0)
	//0 不需要PCB及NAD,LEN
	
#else 
	//1 需要PCB及NAD
	*tmp_buf_resp++=locData;
	leng++;
#endif
	
	CRC_t1^=locData;
	len=locData;
	//leng++;
	*tmp_buf=(leng>>8)&0xff;
	*(tmp_buf+1)=leng&0xff;
	//--INF
	for(i=0;i<len;i++)
	{	
		k=USARTDoWith_ByteReceive(&locData , delay_time_t1byte[Select_Card_NO]);
 		if( k== error_resum)//错误次数	
 		{
 			return IFD_ICC_TypeError;
 		}
 		else if((k==error_p)||(k==error_timeout))
  	{	
			return IFD_ICC_NoResponse;
		}
		else ;//成功
		*tmp_buf_resp++=locData;
		CRC_t1^=locData;
		leng++;
		*tmp_buf=(leng>>8)&0xff;
		*(tmp_buf+1)=leng&0xff;	
	}
	//取SW1、SW2
	sw1_sw2=*(tmp_buf_resp-2);
	sw1_sw2=((sw1_sw2<<8)&0xff00)|((*(tmp_buf_resp-1))&0xff);
	//CRC
	k=USARTDoWith_ByteReceive(&locData , delay_time_t1byte[Select_Card_NO]);
 		if( k== error_resum)//错误次数	
 		{
 			return IFD_ICC_TypeError;
 		}
 		else if((k==error_p)||(k==error_timeout))
  	{	
			return IFD_ICC_NoResponse;
		}
		else ;//成功
#if (ADPU_WithCRC==0)
  //0 不需要CRC
#else 
  //1 需要CRC
  *tmp_buf_resp++=locData;
	leng++;
	*tmp_buf=(leng>>8)&0xff;
	*(tmp_buf+1)=leng&0xff;
#endif
	if(CRC_t1!=locData)	{return Card_CRCERR;}
			
	return sw1_sw2;
	
}

int16 Card_Insert(void)//新增掉电判断
{
	return CPUCard_Insert;
}
int16 Card_InsertAgain(void)//新增掉电判断
{
#if (Use_CardLun_FIRST==1)	
	uint32 t;	
	if(GPIO_ReadInputDataBit(CARD_INT_PORT,CARD_INT_PIN))//SAM0常开未插卡
	{
		SC1_CMDVCCL;
		SC_Delay();
		SC1_CMDVCCH;
		SC_Delay();
		if(GPIO_ReadInputDataBit(CARD_INT_PORT,CARD_INT_PIN))		
		{LEDC_run(0);	CPUCard_Insert=CARD_FIRSRInsert;return CARD_FIRSRInsert;}
		else
		{Card_PowerStatus=0;//掉电
			LEDC_run(1);	CPUCard_Insert=CARD_NONEInsert;return CARD_NONEInsert;}
	}
	else
	{
		SC1_CMDVCCH;
		SC_Delay();Card_PowerStatus=0;//掉电
		if(GPIO_ReadInputDataBit(CARD_INT_PORT,CARD_INT_PIN))
		{LEDC_run(0);	CPUCard_Insert=CARD_FIRSRInsert;return CARD_FIRSRInsert;}
	 	else
	 	{LEDC_run(1);	CPUCard_Insert=CARD_NONEInsert;return CARD_NONEInsert;}
	}
#else 
	return CARD_NONEInsert;
#endif	
}

void Card_SelectTypeLun(uint8 lun,uint8 type)
{
	
	switch(Select_Card_NO)
	{
		case CARD_FIRST:
			
			if(type==CardType_sam0)
			{
					
				SC_UART_Init();	
			}
#if (Use_Card_SLE4442==1)
			else if(type==CardType_SLE4442)
			{
				sle4442GPIO_Ini();
			}
#else 
#endif
#if (Use_Card_SLE4428==1)			
			else if(type==CardType_SLE4428)
			{
				sle4428GPIO_Ini();
			}
#else 
#endif	
#if (Use_Card_AT24CX==1)			
			else if((type==CardType_AT24C01A)||(type==CardType_AT24C02)||(type==CardType_AT24C04)||(type==CardType_AT24C08)
				     ||(type==CardType_AT24C16)||(type==CardType_AT24C32)||(type==CardType_AT24C64)||(type==CardType_AT24C128)
				     ||(type==CardType_AT24C256)||(type==CardType_AT24C512)||(type==CardType_AT24C1024))
			{
				AT24CXXGPIO_Ini();
				//重新上电
			}
#else 
#endif	
#if (Use_Card_AT88SC102==1)		
			else if(type==CardType_AT88SC102)
			{
				
				AT1604_OpenCard( );//重新上电
			}
#else 
#endif	
#if (Use_Card_AT88SC1604==1)			
			else if(type==CardType_AT88SC1604)
			{
				AT1604_OpenCard( );//重新上电
			}
#else 
#endif	
#if (Use_Card_AT88SC1608==1)		
			else if(type==CardType_AT88SC1608)
			{
				//AT1608_OpenCard( );//重新上电
                AT1608GPIO_Ini();
			}
#else 
#endif	
#if (Use_Card_AT88SC153==1)			
            else if(type==CardType_AT88SC153)
			{
				//AT1608_OpenCard( );//重新上电
                AT1608GPIO_Ini();
			}
#else 
#endif	
#if (Use_Card_AT45D041==1)			
			else if(type==CardType_AT45D041)
			{
				//AT45DBGPIO_Ini();//重新上电
			}
#else 
#endif			
			else ;
		break;
		
		default:
		break;
	}	
	
}
void Card_SelectType(char typecard)
{
#if (Use_Card_CPU_sam0==1)	
	if(Select_Card_NO!=CARD_FIRST)	return;

    //如果掉电的话，则重新初始化管脚
	if(Card_PowerStatus!=0)//卡有上电
	{
		if(Card_TypeSave==typecard)	return;
	}
	//卡没有电，或者换个卡型
	Card_TypeSave=typecard;
	Card_SelectTypeLun(Select_Card_NO,typecard);
#else 
#endif	
}

/**********************************************/
uint8 Card_GetStatus(uint8 lun)
{
	return Card_RunStatus[lun];		
}

word Card_Protocol(word Protocol,uint16 F,uint16 D)
{	
	uint8 F_tmp,D_tmp,k;
	uint8 locData = 0;
	uint8 CRC_tmp=0;

	//参数不符合
	if((Protocol!=T0_PROTOCOL)&&(Protocol!=T1_PROTOCOL)) {return IFD_ParameterError;}
	
		//F数组0，1相同，故应使用保持的A1
		F_tmp=((SC_A2RWord.TA[0]>>4)&0x0f);
		switch(D)
		{
			case 1:D_tmp=1;
				break;	
			case 2:D_tmp=2;
				break;
			case 4:D_tmp=3;
				break;
			case 8:D_tmp=4;
				break;
			case 16:D_tmp=5;
				break;
			case 32:D_tmp=6;
				break;
			case 12:D_tmp=8;
				break;
			case 20:D_tmp=9;
				break;	
			default:D_tmp=1;
				break;
		}
		switch(Select_Card_NO)
	{
#if (Use_CardLun_FIRST==1)		
		case CARD_FIRST:
			USART_DMACmd(CARD_UARTX, USART_DMAReq_Rx, ENABLE);
			 SCData = 0xFF;
      CRC_tmp^=SCData;

  		if(SC_USART_SendData(CARD_UARTX, SCData)!=0)//发送错误
  			return Card_OverTime;
  		
  		// Send PTS0 
      SCData = 0x10|(Protocol&0x0f);
      CRC_tmp^=SCData;

  		if(SC_USART_SendData(CARD_UARTX, SCData)!=0)//发送错误
  			return Card_OverTime;
       //Send PTS1 
     SCData = ((F_tmp<<4)&0xf0)|(D_tmp&0x0f); 
      CRC_tmp^=SCData;

  		if(SC_USART_SendData(CARD_UARTX, SCData)!=0)//发送错误
  			return Card_OverTime;
      // Send PCK 
      SCData = CRC_tmp; 

  		if(SC_USART_SendData(CARD_UARTX, SCData)!=0)//发送错误
  			return Card_OverTime;
  			
  		(void)USART_ReceiveData_MY(CARD_UARTX);
  		
			// Disable the DMA Receive (Reset DMAR bit only)   
      USART_DMACmd(CARD_UARTX, USART_DMAReq_Rx, DISABLE);
		break;
#else 
#endif		
		case CARD_SECOND: //大卡座
		case CARD_THIRD:
		case CARD_FOURTH:
		case CARD_FIFTH:
		case CARD_SIXTH:

			break;
		default:
			return IFD_ParameterError;
      
     } 
     //----------接收
     	k=USARTDoWith_ByteReceive(&locData , atr_delayclksum);
 			if( k== error_resum)//错误次数	
 			{
 				return IFD_ICC_TypeError;
 			}
 			else if((k==error_p)||(k==error_timeout))
  		{	
				return IFD_ICC_NoResponse;
			}
			else ;
      if(locData!=0xff) {return Card_PPSERR;}
      
      
      k=USARTDoWith_ByteReceive(&locData , atr_delayclksum);
 			if( k== error_resum)//错误次数	
 			{
 				return IFD_ICC_TypeError;
 			}
 			else if((k==error_p)||(k==error_timeout))
  		{	
				return IFD_ICC_NoResponse;
			}
			else ;   	
      if(locData!=(0x10|(Protocol&0x0f))) {return Card_PPSERR;}
      
      k=USARTDoWith_ByteReceive(&locData , atr_delayclksum);
 			if( k== error_resum)//错误次数	
 			{
 				return IFD_ICC_TypeError;
 			}
 			else if((k==error_p)||(k==error_timeout))
  		{	
				return IFD_ICC_NoResponse;
			}
			else ;
      if(locData!=(((F_tmp<<4)&0xf0)|(D_tmp&0x0f)))	{return Card_PPSERR;}
      
      k=USARTDoWith_ByteReceive(&locData , atr_delayclksum);
 			if( k== error_resum)//错误次数	
 			{
 				return IFD_ICC_TypeError;
 			}
 			else if((k==error_p)||(k==error_timeout))
  		{	
				return IFD_ICC_NoResponse;
			}
			else ;
      if(locData!=CRC_tmp) {return Card_PPSERR;}
      //不管成功与否，都改变	
      return Card_OK;
}
//------------
#if (Use_CardLun_FIRST==1)
void exit_cardpress_irq(void)
{uint8 i;
	if(EXTI_GetITStatus(CARD_INT_Line) != RESET)
	{	
	if(CARD_CMD_PORT->ODR&CARD_CMD_PIN)
		{
			if(GPIO_ReadInputDataBit(CARD_INT_PORT,CARD_INT_PIN))//H
			{	
				CPUCard_Insert=1;
				LEDC_run(0);//亮灯
			}
			else
			{	
				CPUCard_Insert=0;
				LEDC_run(1);//灭灯
				Card_PowerStatus=0;//下电
			}
		}
		else //L ,低电平时只会出现拔卡或IC出现硬件错误
		{	GPIO_SetBits(CARD_CMD_PORT, CARD_CMD_PIN);
			for(i=0;i<10;i++);
			//重新检测
			if(GPIO_ReadInputDataBit(CARD_INT_PORT,CARD_INT_PIN))//H
			{	
				CPUCard_Insert=1;
				LEDC_run(0);//亮灯
			}
			else
			{	
				CPUCard_Insert=0;
				LEDC_run(1);//灭灯
				Card_PowerStatus=0;//下电
			}
			
			//SC1_CMDVCCHH();//重新检测卡必须CMD为高
			//LEDC_run(1);//灭灯	
			
			//CPUCard_Insert=0;
		}
		EXTI_ClearFlag(CARD_INT_Line);
        EXTI_ClearITPendingBit(CARD_INT_Line);	
		Card_RunStatus[0]=Card_Staus_Poweroff;
	}
}
#else 
#endif
void CARD_Exit_Configuration(void)
{
#if (Use_CardLun_FIRST==1)	
	EXTI_InitTypeDef EXTI_InitStructure;
    GPIO_EXTILineConfig(CARD_INT_PortSource, CARD_INT_PinSource);  

    // Configure CLK2 EXTI line 
    EXTI_InitStructure.EXTI_Line = CARD_INT_Line;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;//EXTI_Trigger_Falling;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure); 
#else 
#endif    
}
uint8 CARD_GETPROTOCOL(void)
{
	return(Parameter[Select_Card_NO].CARD_PROTOCOL);	
}
