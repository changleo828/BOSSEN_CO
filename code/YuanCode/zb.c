/*******************************************************/
/*模块名：   瓦斯检测仪主函数                          */
/*功能描述： 实现了LCD的显示和CO传感器数据的检测       */
/*版本：     V1。0                                     */
/*平台：     MSP430F149 主晶振 8.000000M 副晶振 32768Hz*/
/*******************************************************/
#include <msp430x14x.h>
//nclude <msp430def.h>
#include "ht1621.h"
#include "DS1302.h"

uint  SecendFlag=0;
uchar SendDataFlag=0;   //发送数据标志
uchar DelDataFlag=0;    //删除数据标志
uchar HalfMinuteFlag=0;
uchar ShowNowTimeFlag=0;
uchar BeepFlag=0;       //蜂鸣器响标志
uchar total_num=0;
uchar chuanganqi_type=0;

uchar baojing_type=0;
//uchar Guanji_Flag=0;
uchar Zijian_Flag=0;
uchar Jiance_Time_Flag=0;
uchar No_Dianchi_Flag=0;
uchar fmq_Flag=0;

uchar value_dig[6];
uchar Show_Value_H=0;
long Show_Value_L=0;
uchar Max_Value_H=0;
uchar Max_Value_L=0;
uchar Show_Max_Value_Flag=0;
uchar BaojingSaveRecordFlag=0; //报警时保存数据标志

uchar SendConfigFlag=0;

#define JianceTime 3

uchar const ChValue1[]={0x0f,0x00,0x0d,0x09,0x02,0x0b,0x0f,0x01,0x0f,0x0b};
uchar const ChValue2[]={0x0d,0x0d,0x0b,0x0f,0x0f,0x0e,0x0e,0x0d,0x0f,0x0f};


#define BackLightOn  P1OUT |=  BIT4  //打开背景指示灯
#define BackLightOff P1OUT &= ~BIT4  //关闭背景指示灯

#define BEEPOn  P2OUT |=  BIT4  //打开蜂鸣器
#define BEEPOff P2OUT &= ~BIT4  //关闭蜂鸣器

//#define BEEPOff P1OUT |=  BIT5  //打开蜂鸣器
//#define BEEPOn  P1OUT &= ~BIT5  //关闭蜂鸣器

#define POWEROn  P2OUT |=  BIT1  //打开电源
#define POWEROff P2OUT &= ~BIT1  //关闭电源

#define LED1On  P1OUT |=  BIT6  //打开led1
#define LED1Off P1OUT &= ~BIT6  //关闭led1

#define LED2On  P1OUT |=  BIT7  //打开led2
#define LED2Off P1OUT &= ~BIT7  //关闭led2

#define SPEN P4OUT &= ~BIT2

#define KEY1 (uchar)((P2IN & BIT2)&0xff)  //增加
#define KEY2 (uchar)((P1IN & BIT1)&0xff)  //电源
#define KEY3 (uchar)((P1IN & BIT3)&0xff)  //回车
#define KEY4 (uchar)((P1IN & BIT2)&0xff)  //减少

uchar Record_str[500];

uchar PPMFlag=0;
uchar BaifenbiFlag=0;
uchar now_num=0;

uint Dianchi_Value=0;
long CO_Value=0;

unsigned char Current_time[7];
unsigned char Current_time_Buffer[7];
/////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
#define all_2   255
uint time_value=0;
uint Value_CO[all_2];//2
volatile long CO_buf=0;
volatile long CO_Value_2=0;
volatile long CO_buf_3=0;
uint time_3=0;
uint Value_CO_3[all_2];//3
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
void port_init()
{
  P1DIR |=BIT4+BIT5+BIT6+BIT7;  //蜂鸣器管脚使能输出 背景指示灯
  P1DIR &= 0xf0;
  P2DIR &=~BIT2;
  P2DIR |= BIT0+BIT4+BIT1;
  P4DIR |= BIT2;
  P5DIR |=BIT1+BIT3+BIT4;
  P3DIR |=BIT1+BIT2+BIT3;
  P3SEL |= 0x30;                        
}

void usart0_init()  //已经测试成功
{
 UCTL0=0x10;//CHAR0
 UTCTL0=0x20;//SSEL0.1
 URCTL0=0x08;//URXEIE0
 U0BR1=0x00;
 U0BR0=0x45;
 UMCTL0=0xAA;
}

//call this routine to initialise all peripherals
void init_devices(void)
{
 //stop errant interrupts until set up
 _DINT(); //disable all interrupts
 ME1=0X00; //disable sfr peripherals
 ME2=0X00; 
 IE1=0x00; //disable sfr interrupts
 IE2=0x00; 
 //initialise other peripherals
 port_init();
 usart0_init();

 ME1=0xC0;//UTXE0,URXE0
 ME2=0x00;
 IE1=0x40;//URXIE0
 IE2=0x00;
                             
 _EINT(); //re-enable interrupts
}

void delay(void)
{
  unsigned int i=0;
  for(i=0;i<38000;i++);
}

#define TOTAL_NUM 0xA000
#define NOW_NUM   0xA200
#define RECORD_ADD 0xA400
#define TYPE_CHUANGANQI 0xA600

//#define CO  0xf000
#define CO  0xB000

#define LEL 0xB200
#define HS  0xB400
#define CH4 0xB600
#define BAOJING_TYPE 0xB800
#define RECORD0 0xBA00    
//数据
//#define RECORD1 0xFF00
#define RECORD1 0xBC00 
 //数据

//#define HIGH_H 0xFF00
#define HIGH_H 0xBE00   
//地址

#define HIGH_L 0xC000 
//地址
#define TWA_H 0xC200
//地址
#define TWA_L 0xC400  
//地址
#define STEL_H 0xC600
//地址
#define STEL_L 0xC800
//地址

#define XIUZHENG_FUHAO 0xCA00
#define XIUZHENG_VALUE 0xCC00

#define ADD_POS 0xCE00

void Write_Flash_OneByte(uint add,uchar value)
{
  uchar *Flash_ptr;          // Flash pointer
  //uint i;
  Flash_ptr = (uchar *) add;  // Initialize Flash pointer
  _DINT();                   
  FCTL3 = 0x0A500;           
  FCTL1 = 0x0A502;          
  *Flash_ptr = 0;           
  FCTL3 = 0x0A500;          
  FCTL1 = 0x0A540;         
  *Flash_ptr = value;       
  FCTL1 = FWKEY;            
  FCTL3 = FWKEY + LOCK;     
  _EINT();
}

void Write_Flash_String(uint add,uchar * value,uint len)
{
  uchar *Flash_ptr;          // Flash pointer
  uint i;
  Flash_ptr = (uchar *) add; 
  _DINT();                  
  FCTL3 = 0x0A500;           
  FCTL1 = 0x0A502;         
  *Flash_ptr=0;
  FCTL3 = 0x0A500;          
  FCTL1 = FWKEY +WRT;        
  for(i=0;i<len;i++)
    Flash_ptr[i] = value[i];    
  FCTL1 = FWKEY;           
  FCTL3 = FWKEY + LOCK;     
  _EINT();
}

void Del_Flash_String(uint add)
{
  uchar *Flash_ptr;         
 // uint i;
  Flash_ptr = (uchar *) add; 
  _DINT();                   
  FCTL3 = 0x0A500;         
  FCTL1 = 0x0A502;         
  *Flash_ptr=0;
  FCTL1 = FWKEY;            
  FCTL3 = FWKEY + LOCK;     
  _EINT();
}

void SendByte(uchar ch)
{
    uchar i=0;
    for(i=0;i<10;i++);
    while (!(IFG1 & UTXIFG0));         
    TXBUF0 = ch;
}
//interrupt handlers 
/***bcd_data(<0x255,>0)***/
unsigned char BCD2HEX(unsigned char bcd_data)
{
    unsigned char temp;
	temp= (uchar)((unsigned char)(bcd_data/16)*10)+(uchar)(bcd_data%16);
    return temp;
}

unsigned char HEX2BCD(unsigned char hex_data)
{
    unsigned char temp;
	temp= (uchar)((unsigned char)(hex_data/10)*16)+(uchar)(hex_data%10);
    return temp;
}

void ShowTotalTime(void)  //显示全部时间
{
  Ht1621WrOneData(7, time_dig[0]);  
  Ht1621WrOneData(6 ,time_dig[1]);
  Ht1621WrOneData(5 ,time_dig[2]);  
  Ht1621WrOneData(4 ,time_dig[3]);  
  Ht1621WrOneData(3 ,time_dig[4]);  
  Ht1621WrOneData(2 ,time_dig[5]); 
  Ht1621WrOneData(1 ,time_dig[6]);
  Ht1621WrOneData(0 ,time_dig[7]); 
  Ht1621WrOneData(21,time_dig[8]);
  Ht1621WrOneData(22,time_dig[9]); 
  Ht1621WrOneData(23,time_dig[10]);
  Ht1621WrOneData(24,time_dig[11]);
  Ht1621WrOneData(25,time_dig[12]);
  Ht1621WrOneData(26,time_dig[13]); 
  Ht1621WrOneData(27,time_dig[14]);
  Ht1621WrOneData(28,time_dig[15]);
}

uchar const ChTime0[]={0x08,0x0d,0x0e}; 
uchar const ChTime1[]={0x0f,0x00,0x0b,0x09,0x04,0x0d,0x0f,0x08,0x0f,0x0d};
uchar const ChTime2[]={0x0d,0x0d,0x0e,0x0f,0x0f,0x0b,0x0b,0x0d,0x0f,0x0f};
uchar const ChTime3[]={0x00,0x02}; 

uchar Num_dig[4];
void ShowTotalNum(uchar position)  
{
  Num_dig[0]=ChTime1[position/10];  
  if(PPMFlag)  Num_dig[1]=ChTime2[position/10];
    else Num_dig[1]=ChTime2[position/10] & 0x07;
  Num_dig[2]=ChTime1[position%10]; 
  if(BaifenbiFlag)  Num_dig[3]=ChTime2[position%10];
    else Num_dig[3]=ChTime2[position%10] & 0x07;
  Ht1621WrOneData(29,Num_dig[0]); 
  Ht1621WrOneData(20,Num_dig[1]);
  Ht1621WrOneData(19,Num_dig[2]);
  Ht1621WrOneData(18,Num_dig[3]);
}

void ShowTimeSecend(void)  
{
   if(ShowSecendFlag)
   {
      Ht1621WrOneData(26,time_dig[13]|0x08);
   }
   else   
   {
      Ht1621WrOneData(26,time_dig[13]&0x07);
   }
}


#pragma vector = TIMERA0_VECTOR           //TA0中断服务,用来参考稳定
__interrupt void ta0_isr(void)
{
 SecendFlag++;
 if(SecendFlag>=120) 
 {
   SecendFlag=0;
   if(ShowSecendFlag) ShowSecendFlag=0;
   else ShowSecendFlag=1;
   if(Zijian_Flag==0) //  if((Guanji_Flag==0)&&(Zijian_Flag==0)) 
     ShowTimeSecend();
   HalfMinuteFlag++;
   if((HalfMinuteFlag%JianceTime)==0)
     Jiance_Time_Flag=1; 
   if(HalfMinuteFlag>=30)  
   {
     HalfMinuteFlag=0;
	 if(Zijian_Flag==0)// if((Guanji_Flag==0)&&(Zijian_Flag==0))
	                     ShowNowTimeFlag=1;  
   }
 }
 CCR0 = 50000; 
}

void Read_Record0(void)
{
  uint i=0;
  uchar *ch=(uchar *)RECORD0;
  for(i=0;i<500;i++)
    Record_str[i]=ch[i];
}

void Read_Record1(void)
{
  uint i=0;
  uchar *ch=(uchar *)RECORD1;
  for(i=0;i<500;i++)
    Record_str[i]=ch[i];
}

uchar Add_Pos=0;
void Write_One_Record(uchar isbaoji)
{
  uint i=0;
  uchar One_Record_str[10];
  for(i=0;i<7;i++)
    One_Record_str[i]=Current_time[i];
  One_Record_str[5]=Add_Pos;
  if(isbaoji==2) 
    One_Record_str[7]=chuanganqi_type|0x80;
  else if(isbaoji==1)
    One_Record_str[7]=chuanganqi_type|0x60;
  else One_Record_str[7]=chuanganqi_type;
  One_Record_str[8]=Show_Value_H;
  One_Record_str[9]=Show_Value_L;
  if(now_num<50)
  {
    Read_Record0();
	for(i=now_num*10;i<now_num*10+10;i++)
       Record_str[i]=One_Record_str[i-now_num*10];
	Write_Flash_String(RECORD0,Record_str,500);
  }
  else
  {
    Read_Record1();
	for(i=now_num*10-500;i<now_num*10+10-500;i++)
       Record_str[i]=One_Record_str[i+500-now_num*10];
	Write_Flash_String(RECORD1,Record_str,500);
  }  
}

uchar Read_Total_Num(void)
{
  uchar *ch=(uchar *)TOTAL_NUM;
  return ch[0];
}

uchar Read_Now_Num(void)
{
  uchar *ch=(uchar *)NOW_NUM;
  return ch[0];
}

uchar Read_Type_Chuanganqi(void)
{
  uchar *ch=(uchar *)TYPE_CHUANGANQI;
  return ch[0];
}

uchar Read_Baojing_Type(void)
{
  uchar *ch=(uchar *)BAOJING_TYPE;
  return ch[0];
}	
///////////////////////////////////////////
uchar Read_HIGH_H(void)
{
  uchar *ch=(uchar *)HIGH_H;
  return ch[0];
}

uchar Read_HIGH_L(void)
{
  uchar *ch=(uchar *)HIGH_L;
  return ch[0];
}

uchar Read_TWA_H(void)
{
  uchar *ch=(uchar *)TWA_H;
  return ch[0];
}


uchar Read_TWA_L(void)
{
  uchar *ch=(uchar *)TWA_L;
  return ch[0];
}


uchar Read_STEL_H(void)
{
  uchar *ch=(uchar *)STEL_H;
  return ch[0];
}

uchar Read_STEL_L(void)
{
  uchar *ch=(uchar *)STEL_L;
  return ch[0];
}

uchar Read_XIUZHENG_FUHAO(void)
{
  uchar *ch=(uchar *)XIUZHENG_FUHAO;
  return ch[0];
}

uchar Read_XIUZHENG_VALUE(void)
{
  uchar *ch=(uchar *)XIUZHENG_VALUE;
  return ch[0];
}

uchar Read_ADD_POS(void)
{
  uchar *ch=(uchar *)ADD_POS;
  return ch[0];
}
uchar Dianchi_dig;
//AD 每个值 为 0.00044V
// 4096格 
//电池分 4段  <3.2V  没电  1.6
//3.2~3.4  一格  1.6--1.7
//3.4~3.5  二格  1.7--1.75
//   >3.5  三格  1.75
void ShowDianchi(void)
{
  if(Dianchi_Value>0x0F89) Dianchi_dig=0x0f;
  else if((Dianchi_Value<=0x0F89)&&(Dianchi_Value>0x0F79))  
  {
    if(Dianchi_dig==0x0f);
	else Dianchi_dig=0x0b;
  }
  else if(Dianchi_Value>0x0F17) Dianchi_dig=0x0b;
  else if((Dianchi_Value<=0x0F17)&&(Dianchi_Value>0x0F07))  
  {
    if(Dianchi_dig==0x0b);
	else Dianchi_dig=0x09;
  }
  else if(Dianchi_Value>0x0E34) Dianchi_dig=0x09;
  else if((Dianchi_Value<=0x0E34)&&(Dianchi_Value>0x0E24))  
  {
    if(Dianchi_dig==0x09);
	else Dianchi_dig=0x08;
  }
  else Dianchi_dig=0x08;
  if(Dianchi_dig==0x08) No_Dianchi_Flag=1;
  else No_Dianchi_Flag=0;
  Ht1621WrOneData(17,Dianchi_dig);
}

  
void SetFirstTime(void)
{
    Current_time[0]=1;   //09 年 4 月17日 08 :25 星期5
	Current_time[1]=0x01;
	Current_time[2]=0x12;
	Current_time[3]=0x03;      //ri
	Current_time[4]=0x08;      //yue
	Current_time[5]=0x07;
	Current_time[6]=0x12;      //nian
	Set1302(Current_time);
}

void CheckFirstTime(void)  //第一次运行程序检查
{
  total_num=Read_Total_Num();
  if(total_num>99) //第一次执行 ,设置时间 ，设置类型等等
  {
    SetFirstTime();//*///设置时间
	////////////////////////////////////////
	Write_Flash_OneByte(TOTAL_NUM,0);  //设置总记录次数为0
	Write_Flash_OneByte(NOW_NUM,0);
	Write_Flash_OneByte(TYPE_CHUANGANQI,1);
	Write_Flash_OneByte(BAOJING_TYPE,0);
	
	Write_Flash_OneByte(HIGH_H,0);   //Write_Flash_OneByte(HIGH_H,9);
	Write_Flash_OneByte(HIGH_L,24);  //Write_Flash_OneByte(HIGH_L,99);
	Write_Flash_OneByte(TWA_H,9);
	Write_Flash_OneByte(TWA_L,99);
	Write_Flash_OneByte(STEL_H,9);
	Write_Flash_OneByte(STEL_L,99);
	/*
    Write_Flash_OneByte(HIGH_H,6);    
	Write_Flash_OneByte(HIGH_L,0);
	Write_Flash_OneByte(TWA_H,6);
	Write_Flash_OneByte(TWA_L,50);
	Write_Flash_OneByte(STEL_H,6);
	Write_Flash_OneByte(STEL_L,0);
	*/
	Write_Flash_OneByte(XIUZHENG_FUHAO,0);
	Write_Flash_OneByte(XIUZHENG_VALUE,0);
	Write_Flash_OneByte(ADD_POS,0);
	total_num=0;
    ////////////////////////////////////////
  }
}
//设置时钟数据地址 格式为：秒 分 时 日 月 星期 年
unsigned char ChangeTimeFormat(void)  //测试成功
{
  uchar ch=0;
  Get1302(Current_time);
  if(Current_time[0]==0x80)
  {
    SetFirstTime();
	Get1302(Current_time);
  }
  ch=BCD2HEX(Current_time[6]);  
  if(ch>29) 
  {
     SetFirstTime();
	 return 0;
  }
  time_dig[0]=ChTime0[ch/10];   
  time_dig[1]=ChTime1[ch%10];time_dig[2]=ChTime2[ch%10]; 
  ch=BCD2HEX(Current_time[4]);
  if(ch>12) 
  {
    SetFirstTime();
	return 0;
  }
  time_dig[3]=ChTime3[ch/10];  
  time_dig[4]=ChTime1[ch%10];time_dig[5]=ChTime2[ch%10]; 
  ch=BCD2HEX(Current_time[3]);
  if(ch>31)  
  {
    SetFirstTime();
	return 0;
  }
  time_dig[6]=ChTime1[ch/10];time_dig[7]=ChTime2[ch/10]&0x07; 
  time_dig[8]=ChTime1[ch%10];time_dig[9]=ChTime2[ch%10]&0x07;  
  ch=BCD2HEX(Current_time[2]); 
  if(ch>23)  
  {
    SetFirstTime();
	return 0;
  } 
  time_dig[10]=ChTime1[ch%10];time_dig[11]=ChTime2[ch%10]&0x07;  
  if((ch/10)==0) 
  {
    time_dig[7]=time_dig[7]&0x07;
	time_dig[9]=time_dig[9]&0x07;
	time_dig[11]=time_dig[11]&0x07;  
  }
  else if((ch/10)==1) 
  {
    time_dig[7]=time_dig[7]&0x07;
	time_dig[9]=time_dig[9]|0x08;
	time_dig[11]=time_dig[11]|0x08;
  }
  else if((ch/10)==2) 
  {
    time_dig[7]=time_dig[7]|0x08;
	time_dig[9]=time_dig[9]&0x07;
	time_dig[11]=time_dig[11]|0x08;
  }
  ch=BCD2HEX(Current_time[1]); 
  if(ch>59) ch=ch%59;
  time_dig[12]=ChTime1[ch/10];time_dig[13]=(time_dig[13]&0x08)|(ChTime2[ch/10]&0x07);  
  time_dig[14]=ChTime1[ch%10];time_dig[15]=ChTime2[ch%10];     
  ShowTotalTime();
  return 1;
}


void ShowTimeFormat(void) 
{ 
  uchar ch=0;
  time_dig[0]=ChTime0[Current_time_Buffer[6]/10];   
  time_dig[1]=ChTime1[Current_time_Buffer[6]%10];time_dig[2]=ChTime2[Current_time_Buffer[6]%10]; 
  ch=Current_time_Buffer[4];
  time_dig[3]=ChTime3[ch/10];  
  time_dig[4]=ChTime1[ch%10];time_dig[5]=ChTime2[ch%10]; 
  ch=Current_time_Buffer[3];
  time_dig[6]=ChTime1[ch/10];time_dig[7]=ChTime2[ch/10]&0x07; 
  //buffer[0]=time_dig[7];  //储存10ADEF
  time_dig[8]=ChTime1[ch%10];time_dig[9]=ChTime2[ch%10]&0x07;  
  //buffer[1]=time_dig[9];  //储存10C
  ch=Current_time_Buffer[2];
  time_dig[10]=ChTime1[ch%10];time_dig[11]=ChTime2[ch%10]&0x07;  
  if((ch/10)==0) 
  {
    time_dig[7]=time_dig[7]&0x07;
	time_dig[9]=time_dig[9]&0x07;
	time_dig[11]=time_dig[11]&0x07; 
  }
  else if((ch/10)==1) 
  {
    time_dig[7]=time_dig[7]&0x07;
	time_dig[9]=time_dig[9]|0x08;
	time_dig[11]=time_dig[11]|0x08;
  }
  else if((ch/10)==2) 
  {
    time_dig[7]=time_dig[7]|0x08;
	time_dig[9]=time_dig[9]&0x07;
	time_dig[11]=time_dig[11]|0x08;
  }
  ch=Current_time_Buffer[1]; 
  time_dig[12]=ChTime1[ch/10];time_dig[13]=(time_dig[13]&0x08)|(ChTime2[ch/10]&0x07);  
  time_dig[14]=ChTime1[ch%10];time_dig[15]=ChTime2[ch%10];     
  ShowTotalTime();
}

void ShowTimeNone(uchar ch) 
{ 
  if(ch==0) 
  {
    Ht1621WrOneData(6 ,0);
    Ht1621WrOneData(5 ,0x08); 
  }
  else if(ch==1) 
  {
    Ht1621WrOneData(3 ,0);  
    Ht1621WrOneData(2 ,0x08);  //7
  }
  else if(ch==2)
  {
    Ht1621WrOneData(21 ,0); 
	if((Current_time_Buffer[2]/10)==1) 
       Ht1621WrOneData(22 ,0x08);  //7
	else Ht1621WrOneData(22 ,0x00);  //7
  }
  else if(ch==3) 
  {
    Ht1621WrOneData(23 ,0); 
	if((Current_time_Buffer[2]/10)==0) 
      Ht1621WrOneData(24 ,0x00);  //7
	else Ht1621WrOneData(24 ,0x08);  //7
  }
  else if(ch==4) 
  {
    Ht1621WrOneData(27 ,0);  
    Ht1621WrOneData(28 ,0x08);  //7
  }
}

uchar Special_dig[3];
uchar hign_baojing_type=0;
uchar stel_baojing_type=0;
uchar twa_baojing_type=0;
void ShowChuanganqiType(void)
{
 	if((chuanganqi_type==4)||(chuanganqi_type==2)) 
	{
	  BaifenbiFlag=1;
	  PPMFlag=0;
	}
	else if((chuanganqi_type==1)||(chuanganqi_type==3)) 
	{
	  BaifenbiFlag=0;
	  PPMFlag=1;
	}
	if(chuanganqi_type==1)
	{
      Special_dig[0]=0;
	  if(stel_baojing_type==1) Special_dig[0] |=0x06;
	  if(twa_baojing_type==1) Special_dig[0] |=0x0A;
	  if((stel_baojing_type==0) && (twa_baojing_type==0))
	  {
	     Special_dig[0]=0x02;
	  }
	  if(hign_baojing_type==1) Special_dig[1]=0x04;
	  else Special_dig[1]=0x00;
	  //Special_dig[2]=0x00;
	}
	 if(chuanganqi_type==4)
	{
	  Special_dig[0]=0;
	  if(stel_baojing_type==1) Special_dig[0] |=0x04;
	  if(twa_baojing_type==1) Special_dig[0] |=0x08;
	  if((stel_baojing_type==0) && (twa_baojing_type==0))
	  {
	     Special_dig[0]=0x00;
	  }
	  if(hign_baojing_type==1) Special_dig[1]=0x04;
	  else Special_dig[1]=0x00;
	}
	Ht1621WrOneData(8,Special_dig[0]);
	Ht1621WrOneData(9,Special_dig[1]);
	Ht1621WrOneData(14,Special_dig[2]);	  
}

void ShowNormalType(void)
{
  	Ht1621WrOneData(8,Special_dig[0]);
	Ht1621WrOneData(9,Special_dig[1]);
	Ht1621WrOneData(14,Special_dig[2]);	
}

void ShowValue(void)
{
   uchar lcd_Value_H=0;
   uchar lcd_Value_L=0;
   if(Show_Max_Value_Flag==1) 
   {
     lcd_Value_H=Max_Value_H;
	 lcd_Value_L=Max_Value_L;
   }
   else
   {
     lcd_Value_H=Show_Value_H;
	 lcd_Value_L=Show_Value_L;
   }
   if(chuanganqi_type==4) 
   {
     value_dig[0]=ChValue1[lcd_Value_H%10];
	 value_dig[1]=ChValue2[lcd_Value_H%10];
     value_dig[2]=ChValue1[lcd_Value_L/10];
	 value_dig[3]=ChValue2[lcd_Value_L/10]&0x07;
     value_dig[4]=ChValue1[lcd_Value_L%10];
	 value_dig[5]=ChValue2[lcd_Value_L%10]&0x07; 
   }
   else if(chuanganqi_type==1) 
   {
     value_dig[0]=ChValue1[lcd_Value_H%10];
	 value_dig[1]=ChValue2[lcd_Value_H%10]&0x07;
     value_dig[2]=ChValue1[lcd_Value_L/10];
	 value_dig[3]=ChValue2[lcd_Value_L/10]&0x07;
     value_dig[4]=ChValue1[lcd_Value_L%10];
	 value_dig[5]=ChValue2[lcd_Value_L%10]&0x07; 
   }
   Ht1621WrOneData(10,value_dig[0]);
   Ht1621WrOneData(11,value_dig[1]);
   Ht1621WrOneData(12,value_dig[2]);
   Ht1621WrOneData(13,value_dig[3]);
   Ht1621WrOneData(15,value_dig[4]);
   Ht1621WrOneData(16,value_dig[5]);
   ShowChuanganqiType();  //显示传感器和报警类型
}  

void ShansuoT6(void)
{
  Special_dig[1]=Special_dig[1]|0x02;
  Ht1621WrOneData(9,Special_dig[1]);
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
   delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  Special_dig[1]=Special_dig[1]&0x0d;
  Ht1621WrOneData(9,Special_dig[1]);
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
   delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
} 
  
void ShansuoT7(void)
{
  Special_dig[2]=Special_dig[2]|0x02;
  Ht1621WrOneData(14,Special_dig[2]);
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
   delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  Special_dig[2]=Special_dig[2]&0x0d;
  Ht1621WrOneData(14,Special_dig[2]);
  delay(); 
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
} 

void ShansuoT8(void)
{
  Special_dig[2]=Special_dig[2]|0x08;
  Ht1621WrOneData(14,Special_dig[2]);
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
   delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  Special_dig[2]=Special_dig[2]&0x07;
  Ht1621WrOneData(14,Special_dig[2]);
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
   delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
  delay();
} 

void ShansuoNone(void)
{
   Special_dig[1]=Special_dig[1]&0x0d;
   Special_dig[2]=Special_dig[2]&0x0d;
   Special_dig[2]=Special_dig[2]&0x07;
   Ht1621WrOneData(8,Special_dig[0]);
   Ht1621WrOneData(9,Special_dig[1]);
   Ht1621WrOneData(14,Special_dig[2]);
}

void ShowValueNone(void)
{
   Ht1621WrOneData(10,0);
   Ht1621WrOneData(11,0);
   Ht1621WrOneData(12,0);
   Ht1621WrOneData(13,0);
   Ht1621WrOneData(15,0);
   Ht1621WrOneData(16,0); 
}

uchar RXflag=0;
uchar SendTimeFlag=0;
uchar GetTimeFlag=0;
uchar GetTimeBuffer[7];
uchar TimeNum=0;

/*#pragma vector = UART0RX_VECTOR
__interrupt void usart0_rx (void)
{
  switch(RXflag)
  {
    case 0: 
	   if( RXBUF0==0xAA)  RXflag=1;
	   break;
    case 1: 
	   if( RXBUF0==0xBB)  {SendDataFlag=1;RXflag=0;}
	   else if(RXBUF0==0xCC) {DelDataFlag=1;RXflag=0;}
	   else if(RXBUF0==0xDD) {SendTimeFlag=1;RXflag=0;}
	   else if(RXBUF0==0xFF) {SendConfigFlag=1;RXflag=0;}
	   else if(RXBUF0==0xEE) {RXflag=2;TimeNum=0;}
	   break;
	case 2:
	   GetTimeBuffer[TimeNum]=RXBUF0;
	   TimeNum++;
	   if(TimeNum>=7) {GetTimeFlag=1;RXflag=0;TimeNum=0;}
	   break;
	default:
	   RXflag=0;
	   break;	
  }   
}*/

//在这里修改显示参数 
//当在大气中的值 单位 为mV 每一个值代表 0.44mV  10相当于 10*0.44= 4.4mV  //0.61mV
#define value_0  105//10     除1.39
//在浓度为 23ppm 的值 
#define value_22  110
#define value_23  120//初45
#define value_24  130
//在浓度为 102ppm 的值  
//define value_100  227
//在浓度为 200ppm 的值  
//#define value_200  838
//在浓度为 300ppm 的值  
//#define value_300 1125
//在浓度为 350ppm 的值 
#define value_349  360
#define value_350  400//410
#define value_351  440
//在浓度为 400ppm 的值  
//#define value_400 1385
#define value_499  540
//在浓度为 499ppm 的值 
#define value_500  600//615
#define value_501  660

//在浓度为 600ppm 的值  
//#define value_600  2855
//在浓度为 700ppm 的值  
//#define value_700  3215
//在浓度为 800ppm 的值  
//#define value_800  3575
//在浓度为 850ppm 的值
#define value_849  900
#define value_850  1046  //1046
#define value_851  1128
//在浓度为 900ppm 的值  
//#define value_900  3935
//在浓度为 999ppm 的值  
#define value_999  1218  //1230

void ChangeValue(void) //转换数据
{
  uint xiuzheng_value=0;
  //uint cobuffer=0;
  if(CO_Value<=value_0)  // 在大气中时
  {
	Show_Value_H=0;
	Show_Value_L=0;
  }  
 else if((CO_Value>value_0)&&(CO_Value<=value_22) )  //小于23ppm
  {
    Show_Value_H=0;
	Show_Value_L=(23-0)*(CO_Value-value_0)/(value_22-value_0)+0 ;
  }
  else if((CO_Value>value_22)&&(CO_Value<=value_24))   //等于 23ppm 时
  {
    Show_Value_H=0;
	Show_Value_L=24;
  }
   /*else if((CO_Value>value_0)&&(CO_Value<value_100))  // 大于0ppm 小于100ppm
  {
    Show_Value_H=0;
	Show_Value_L=(100-0)*(CO_Value-value_0)/(value_100-value_0)+0 ;
	Show_Value_L=Show_Value_L%100;
  }
   else if(CO_Value==(value_100+0))   //等于 100ppm 时
  {
    Show_Value_H=1;
	Show_Value_L=00;
  }
  else if((CO_Value>value_100)&&(CO_Value<value_200))//100--<200
	{
	   Show_Value_H=1;
	   Show_Value_L=(200-100)*(CO_Value-value_100)/(value_200-value_100)+100 ;
	   Show_Value_L=Show_Value_L%100;
	}
  else if(CO_Value==(value_200+0))   //等于 200ppm 时
  {
    Show_Value_H=2;
	Show_Value_L=00;
  }
  else if((CO_Value>value_200)&&(CO_Value<value_300))//200--<300
	{
	   Show_Value_H=2;
	   Show_Value_L=(300-200)*(CO_Value-value_200)/(value_300-value_200)+200 ;
	   Show_Value_L=Show_Value_L%100;
	}
  else if(CO_Value==(value_300+0))   //等于 300ppm 时
  {
    Show_Value_H=3;
	Show_Value_L=00;
  }
  */
  else if((CO_Value>value_23)&&(CO_Value<=value_349))  // 大于0ppm 小于350ppm   (CO_Value>value_0)&&(CO_Value<=value_350)
  { 
      Show_Value_L=(349-24)*(CO_Value-value_23)/(value_349-value_23)+24 ;   //Show_Value_L=(350-value_0)*(CO_Value-value_0)/(value_350-value_0)+0 ;
	  Show_Value_H=Show_Value_L/100 ;
	  Show_Value_L=Show_Value_L%100;
  }
  else if((CO_Value>value_349)&&(CO_Value<=value_351))
  {
        Show_Value_H=3;
	Show_Value_L=50;
  }
  else if((CO_Value>value_351)&&(CO_Value<value_499))  // 大于400ppm 小于499ppm
  {
       //Show_Value_H=4 ;//
       Show_Value_L=(499-351)*(CO_Value-value_351)/(value_499-value_351)+351 ;
	   Show_Value_H=Show_Value_L/100 ;
	   Show_Value_L=Show_Value_L%100;
  }
  else if(CO_Value>value_499&&CO_Value<=value_501)   //等于 499ppm 时
  {
    Show_Value_H=5;
	Show_Value_L=00;
  }

  /*else if((CO_Value>value_350)&&(CO_Value<value_400))  // 大于350ppm 小于400ppm
  {
	   Show_Value_H=3 ;//
	   Show_Value_L=(400-350)*(CO_Value-value_350)/(value_400-value_350)+350 ;
	   Show_Value_L=Show_Value_L%100;
  }
  else if(CO_Value==(value_400+0))  //等于 400ppm 时
  {
    Show_Value_H=4;
	Show_Value_L=00;
  }
  
  else if((CO_Value>value_100)&&(CO_Value<value_500))  // 大于400ppm 小于499ppm
  {
       //Show_Value_H=4 ;//
       Show_Value_L=(500-100)*(CO_Value-value_100)/(value_500-value_100)+100 ;
	   Show_Value_H=Show_Value_L/100 ;
	   Show_Value_L=Show_Value_L%100;
  }
  else if(CO_Value==(value_500+0))   //等于 499ppm 时
  {
    Show_Value_H=5;
	Show_Value_L=00;
  }
  else if((CO_Value>value_499)&&(CO_Value<value_600))  // 大于499ppm 小于600ppm
  {
    Show_Value_L=(600-499)*(CO_Value-value_499)/(value_600-value_499)+499 ;
	   Show_Value_H=Show_Value_L/100 ;//
	   Show_Value_L=Show_Value_L%100;
  }
  else if(CO_Value==(value_600+0))   //等于 600ppm 时
  {
    Show_Value_H=6;
	Show_Value_L=00;
  }
  else if((CO_Value>value_600)&&(CO_Value<value_700))  // 大于600ppm 小于700ppm
  {    
       Show_Value_H=6 ;
       Show_Value_L=(700-600)*(CO_Value-value_600)/(value_700-value_600)+600 ;
	   Show_Value_L= Show_Value_L%100;
  }
  else if(CO_Value==(value_700+0))   //等于 700ppm 时
  {
    Show_Value_H=7;
	Show_Value_L=00;
  }
  else if((CO_Value>value_500)&&(CO_Value<value_800))  // 大于700ppm 小于800ppm
  {
       Show_Value_H=7 ;
       Show_Value_L=(800-700)*(CO_Value-value_700)/(value_800-value_700)+700 ;
	   Show_Value_L= Show_Value_L%100;
  }
  else if(CO_Value==(value_800+0))   //等于 800ppm 时
  {
    Show_Value_H=8;
	Show_Value_L=00;
  }*/
  else if((CO_Value>value_500 )&&(CO_Value<=value_850))  // 大于800ppm 小于850ppm
  {    
       Show_Value_L=(850-500)*(CO_Value-value_500 )/(value_850-value_500)+500 ;
	   Show_Value_H=Show_Value_L/100 ;//
	   Show_Value_L=Show_Value_L%100;
  }
  else if(CO_Value>value_849&&CO_Value<=value_851)   //等于 850ppm 时
  {
    Show_Value_H=8;
	Show_Value_L=50;
  }
  /*else if((CO_Value>value_850)&&(CO_Value<value_900))  // 大于850ppm 小于900ppm
  {   
       Show_Value_H=8 ;
       Show_Value_L=(900-850)*(CO_Value-value_850)/(value_900-value_850)+850 ;
	   Show_Value_L=Show_Value_L%100;
  }
  else if(CO_Value==(value_900+0))   //等于 900ppm 时
  {
    Show_Value_H=9;
	Show_Value_L=00;
  }*/
  else if((CO_Value>value_850)&&(CO_Value<value_999))//900--<999
	{
	   Show_Value_L=(999-850)*(CO_Value-value_850)/(value_999-value_850)+850 ;
	   Show_Value_H=Show_Value_L/100 ;
	   Show_Value_L=Show_Value_L%100;   
	}
  else if(CO_Value>=(value_999+0))   //等于 999ppm 时
  {
    Show_Value_H=9;
	Show_Value_L=99;
  }

  xiuzheng_value=(uint)(Read_XIUZHENG_FUHAO()&0x7f)*256+Read_XIUZHENG_VALUE();
  if((Read_XIUZHENG_FUHAO()&0x80)==0)  
  {
      if( (Show_Value_H==0)&&(Show_Value_L==0) ) goto NEXTDOOR;
	  Show_Value_H=((Show_Value_H*100+Show_Value_L)+xiuzheng_value)/100;
	  Show_Value_L=((Show_Value_H*100+Show_Value_L)+xiuzheng_value)%100;
  }
  else 
  {
    if( (Show_Value_H==0)&&(Show_Value_L==0) ) goto NEXTDOOR;
    if(xiuzheng_value>(Show_Value_H*100+Show_Value_L))
	{  Show_Value_H=0; Show_Value_L=0;}
	else 
	{
	  Show_Value_H=((Show_Value_H*100+Show_Value_L)-xiuzheng_value)/100;
	  Show_Value_L=((Show_Value_H*100+Show_Value_L)-xiuzheng_value)%100;
	}
  }
  if(Show_Value_H>9) 
  {
    Show_Value_H=9;
	Show_Value_L=99;   
  }
 NEXTDOOR: 
  if(Show_Value_H>Read_HIGH_H()) hign_baojing_type=1;
  else if((Show_Value_H==Read_HIGH_H())&&(Show_Value_L>=Read_HIGH_L())) hign_baojing_type=1;
  else hign_baojing_type=0;
  if(Show_Value_H>Read_STEL_H()) stel_baojing_type=1;
  else if((Show_Value_H>=Read_STEL_H())&&(Show_Value_L>=Read_STEL_L())) stel_baojing_type=1;
  else stel_baojing_type=0;
  if(Show_Value_H>Read_TWA_H()) twa_baojing_type=1;
  else if((Show_Value_H>=Read_TWA_H())&&(Show_Value_L>=Read_TWA_L())) twa_baojing_type=1;
  else twa_baojing_type=0;
}

void adc12_init()
{
  P6SEL |= BIT3+BIT7;  //电池检测 和 传感器检测
  ADC12CTL0 = ADC12ON+MSC+SHT0_8;      // ADC12CTL0 = ADC12ON+MSC+SHT0_8+REFON+REF2_5V;
  ADC12CTL1 = SHP+CONSEQ_3;             
  ADC12MCTL0 = INCH_3+SREF_3;     // ADC12MCTL0 = INCH_3+SREF_3;       
  ADC12MCTL1 = INCH_7+SREF_3+EOS;       
  ADC12CTL0 |= ENC;                 
}

uchar BaojingFlag=0;
void CheckBaojing(void)
{
  if((hign_baojing_type==0)&&(stel_baojing_type==0)&&(twa_baojing_type==0)) 
    BaojingFlag=0;
  else 
    BaojingFlag=1;
}

void GetMaxValue(void)
{
  //if(Show_Value_H>Max_Value_H)
  //{
   // Max_Value_H=Show_Value_H;
	//Max_Value_L=Show_Value_L;
  //}
  //else if( (Show_Value_H==Max_Value_H) && (Show_Value_L>Max_Value_L))
  //{
    //Max_Value_H=Show_Value_H;
	//Max_Value_L=Show_Value_L;
  //} 
  CheckBaojing();
}

void ADC_do_it(void)
{
  unsigned char i=0;
  long CO_buffer=0;
  long Dianchi_buffer=0;
  for(i=0;i<99;i++)
  { 
 	ADC12CTL0 |= ADC12SC;                 
	while ((ADC12IFG & BIT0)==0);
	CO_buffer=CO_buffer+ADC12MEM0;
	Dianchi_buffer=Dianchi_buffer+ADC12MEM1;
  }
  CO_Value=(uint)((CO_Value+CO_buffer)/100); 
  Dianchi_Value=(uint)((Dianchi_Value+Dianchi_buffer)/100);
}

void ShowEPP(void)
{
   value_dig[0]=0x0f;value_dig[1]=0x02;
   value_dig[2]=0x07;value_dig[3]=0x03;
   value_dig[4]=0x07;value_dig[5]=0x03;
   Ht1621WrOneData(10,value_dig[0]);
   Ht1621WrOneData(11,value_dig[1]);
   Ht1621WrOneData(12,value_dig[2]);
   Ht1621WrOneData(13,value_dig[3]);
   Ht1621WrOneData(15,value_dig[4]);
   Ht1621WrOneData(16,value_dig[5]);
   hign_baojing_type=0;
   stel_baojing_type=0;
   twa_baojing_type=0;
   ShowChuanganqiType();
}

uint Filtering(uint *m,uint n)
{//n:数组中元素个数
   uint max=0,min=0;
   uint i; 
   uint he;  
   for(i=0;i<n-1;i++)
   {/*
      if(m[i]>max) max=m[i];
	  if(m[i]<min) min=m[i];
	  */
	  if( *(m+i)>max   )  max=*(m+i);
	  if( *(m+i)<min   )  min=*(m+i);
   }
   he= (max+min);  
   return he;
}
void SaveRecord(uchar isbaoji)
{
   Get1302(Current_time);
   Write_One_Record(isbaoji);
   if(total_num<99) total_num++;
   now_num=(now_num+1)%99;
   ShowTotalNum(Add_Pos);
   Write_Flash_OneByte(TOTAL_NUM,total_num);  
   Write_Flash_OneByte(NOW_NUM,now_num);
}

void Deal_AD_Value(void)
{
	ChangeValue();
	GetMaxValue();
	ShowValue();
}

 
void main()
{
        uint i=0;
	uchar time_buffer=0;
	uchar BackLightOnFlag=0;
	uchar BackLightBuffer=0;
	uint ZidongJianValueBefore=0;
	uint ZidongJianValueNow=0;
	//uchar guanji_time=0;
	uchar backlightflag=0;
	////////////////////////////////////////////////////////////////////////////
	///////////////////////////////
	//unsigned char iii=0;
  	//long CO_buffer=0;  
  	//long CO_VAL=0;
  	//uint ADC_CO[all_2];//第一轮软件滤波时，存储数值
	///////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////
    WDTCTL = WDTPW + WDTHOLD;
	BCSCTL1&=~XT2OFF;            
    do
    {
      IFG1 &= ~OFIFG;                
      for (i = 0xFF; i > 0; i--);   
    }
    while ((IFG1 & OFIFG) != 0);    
    BCSCTL2 =SELM_2+SELS;     
	init_devices();
	adc12_init();
	BackLightOff;BEEPOff;LED1Off;LED2Off;SPEN;
        Ht1621_Init();
	HT1621_all_off();
	CheckFirstTime();  
    /////////////////////////////////////////////////////////////
	CCTL0 = CCIE;                      
    CCR0 = 50000;
    TACTL = TASSEL_2 + MC_2;       //MCLK ;lianxumoshi  
	FCTL2 = FWKEY + FSSEL0 + FN0;   //原始时钟      
	/////////////////////////////////////////////////////////////
	///////////////////////////////////
	//RESTART: //重新开始
	//开机处理
	Zijian_Flag=1; 
	backlightflag=0;
	//BackLightOnFlag=0;
	//POWEROn;
        
                BackLightOn;
                BackLightOnFlag=1;
                if( (BackLightOnFlag==1)&&(((HalfMinuteFlag+30-BackLightBuffer)%30)>=20))
		{ //20秒后自动关闭背光电源
		   BackLightOff;
		   BackLightOnFlag=0;
		   BackLightBuffer=0;
		}
	
    HT1621_all_on();  
	for(i=0;i<50000;i++);
	for(i=0;i<4;i++)
	{
	  time_buffer=HalfMinuteFlag;SecendFlag=0;
	  LED1On;BEEPOn;LED2On;
	  while(time_buffer==HalfMinuteFlag); 
	  BEEPOff;LED1Off;LED2Off;
	  time_buffer=HalfMinuteFlag;SecendFlag=0;
	  while(time_buffer==HalfMinuteFlag); 
	}
	HT1621_all_off();
	Zijian_Flag=0;
	/////////////////////////////////////////////
	if(ChangeTimeFormat()==0) 
	{
	   ChangeTimeFormat();
         }     
	Special_dig[2]=0; 
	chuanganqi_type=Read_Type_Chuanganqi();  //读取传感器类型 共4种  1--4
	baojing_type =Read_Baojing_Type(); //读取报警类型 共4种 1--4
    ShowChuanganqiType();  //显示传感器和报警类型
	
	total_num=Read_Total_Num(); //读取记录数 0-99
	now_num = Read_Now_Num();   //读取最新一条记录数的位置 0-98
	Add_Pos=Read_ADD_POS();
	ShowTotalNum(Add_Pos);  //显示记录数
    //////////////////////////////////
	
    ADC12CTL0 |= ADC12SC;                
	while ((ADC12IFG & BIT0)==0);
	CO_Value=ADC12MEM0;/////////////////////////////////////////////////////////
	                    ////////////////////////////////////////////////////////
	
	
	/*for(iii=0;iii<(all_2-1);iii++)
	{
	    ADC12CTL0 |= ADC12SC;                
		while ((ADC12IFG & BIT0)==0);
		CO_buffer=CO_buffer+ADC12MEM0;
		ADC_CO[iii]=ADC12MEM0;
	}
	CO_VAL=(uint)((CO_VAL+CO_buffer-Filtering(ADC_CO,all_2))/(all_2-2)); 
	CO_buf=CO_buf+CO_VAL;
	Value_CO[time_value]=CO_VAL;
	time_value++;
	if(time_value>=all_2)//进行了   次
	{
	      CO_Value_2=(uint)( (  CO_buf- Filtering(Value_CO,all_2) )/(all_2-2)  );  		   
		  time_value=0;		  
		  CO_buf=0; 
		  CO_buf_3=CO_buf_3+CO_Value_2;
		  Value_CO_3[time_3]=CO_Value_2;
		  time_3++;
		  
	}
	if(time_3>=all_2)//
	{
	     CO_Value=(uint)((CO_buf_3- Filtering(Value_CO_3,all_2) )/(all_2-2)); 
		 time_3=0;
		 CO_buf_3=0;
	}
	
	
	Dianchi_Value=ADC12MEM1;
	ShowDianchi();  //显示电池容量
    /////////////////////////////////
	//////////////////////////显示数值
	Deal_AD_Value();*/
	if(BaojingFlag==1)
	{ 
		BackLightOn;
		backlightflag=1;
	}
	///////////////////////////////////////////////////////////
 while(1)
    {
      
		if(ShowNowTimeFlag==1)//if((ShowNowTimeFlag==1)&&(Guanji_Flag==0))
		{   //每半分钟显示一次时间
		    ShowNowTimeFlag=0;
			ChangeTimeFormat();
			if(No_Dianchi_Flag==1)
			{
              Dianchi_dig=0x00;
              Ht1621WrOneData(17,Dianchi_dig);
			  BEEPOn;delay();BEEPOff;delay();
			  Dianchi_dig=0x08;
              Ht1621WrOneData(17,Dianchi_dig);
			  BEEPOn;delay();BEEPOff;delay();
			 // guanji_time++;
			  //if(guanji_time>2) //响3次以后关机
			  //{
			    //guanji_time=0;
			    //goto GUANJIFLAG;
			  //} 
			}
	         }
		if((Jiance_Time_Flag==1)&&(fmq_Flag==0))//if((Jiance_Time_Flag==1)&&(Guanji_Flag==0))
		{   //每3显示一次值
		    P1OUT ^= BIT7;   //LED2On
		    Jiance_Time_Flag=0;
			ADC_do_it();
			ShowDianchi();  //显示电池容量
			Deal_AD_Value();
			if((BaojingFlag==1) && (BaojingSaveRecordFlag==0) )
			{ 
			  BaojingSaveRecordFlag=1;
			  SaveRecord(2);
			  BackLightOn;
			  backlightflag=1;
			  BackLightOnFlag=1;
			}
			else if(BaojingFlag==0)
			{ 
			   BaojingSaveRecordFlag=0;
			   if(backlightflag==1) 
			   {
			     backlightflag=0;
			     BackLightOff;
			   }
			}
	        }
		if(BaojingFlag==1)//if((BaojingFlag==1)&&(Guanji_Flag==0))
		{  //气体超标报警
                  BackLightOn;
		   BEEPOn; LED1On; LED2On;
                   fmq_Flag=1;
		   ShowValueNone();
		   //if(KEY2==0) goto GUANJIDOOR;
		   delay();delay();
                   delay();delay();
                   delay();delay();
                   delay();delay();
                   delay();delay();
                   delay();delay();
                   delay();delay();
                   delay();delay();
                   delay();delay();
                   delay();delay();
                   delay();delay();
                   delay();delay();
                   
		   BEEPOff; LED1Off;  LED2Off; 
                   fmq_Flag=0;
	       ShowValue();
		  // if(KEY2==0) goto GUANJIDOOR;
		   delay();delay();
                   delay();delay();
                   delay();delay();
                   delay();delay();
                   delay();delay();
                   delay();delay();
                   delay();delay();
                   delay();delay();
                   delay();delay();
                   delay();delay();
                   delay();delay();
                   delay();delay();
		}
	
		if( (BackLightOnFlag==1)&&(((HalfMinuteFlag+30-BackLightBuffer)%30)>=20))
		{ //20秒后自动关闭背光电源
		   BackLightOff;
		   BackLightOnFlag=0;
		   BackLightBuffer=0;
		}
                if(KEY4==0)  //箭头下 的按键//if((KEY4==0)&&(Guanji_Flag==0))  
		{
                  time_buffer=HalfMinuteFlag;
                  delay();
                  while(((KEY4==0)&&(((HalfMinuteFlag+30-time_buffer)%30)<=4)));
                        if( ((HalfMinuteFlag+30-time_buffer)%30)>=3)
                        {
                        }
                        else
                        {
                          
                         if(BackLightOnFlag==0)//短按键处理 并且在开机情况下
			  {
			     BackLightOnFlag=1;
			     BackLightOn;  //打开背景灯 30秒后关闭
			     BackLightBuffer=HalfMinuteFlag;
			  }
			  else if(BackLightOnFlag==1)//短按键处理 并且在开机情况下
			  {
			     BackLightOnFlag=0;
			     BackLightOff;  //打开背景灯 30秒后关闭
			  }
                         delay();delay();
                        }
                        
                        
                  //BackLightOn;
                //BackLightOnFlag=1;
                //if( (BackLightOnFlag==1)&&(((HalfMinuteFlag+30-BackLightBuffer)%30)>=20))
		//{ //20秒后自动关闭背光电源
		   //BackLightOff;
		   //BackLightOnFlag=0;
		   //BackLightBuffer=0;
		//}
	          if( (BackLightOnFlag==1)&&(((HalfMinuteFlag+30-BackLightBuffer)%30)>=20))
		   { //20秒后自动关闭背光电源
		   BackLightOff;
		   BackLightOnFlag=0;
		   BackLightBuffer=0;
		  }
		}
		if(KEY1==0)//if((KEY1==0)&&(Guanji_Flag==0))
		{
                      //BackLightOn;
                //BackLightOnFlag=1;
                //if( (BackLightOnFlag==1)&&(((HalfMinuteFlag+30-BackLightBuffer)%30)>=20))
		//{ //20秒后自动关闭背光电源
		  // BackLightOff;
		   //BackLightOnFlag=0;
		   //BackLightBuffer=0;
		//}
		   time_buffer=HalfMinuteFlag;
		   delay();
                  //while(((KEY1==0)&&(((HalfMinuteFlag+30-time_buffer)%30)<=4)));
		   
                    
                      
                    while(((KEY1==0)&&(((HalfMinuteFlag+30-time_buffer)%30)<=4)));
                        if( ((HalfMinuteFlag+30-time_buffer)%30)>=3)
                          
		        {   
                              
		          
			      Show_Max_Value_Flag=0;  //如果是峰值显示 退回原来正常模式
				  BackLightOn;
			      BEEPOn;delay();delay();BEEPOff;
				  baojing_type =1; //读取报警类型 共4种 1--4
                  ShowChuanganqiType();  //显示传感器和报警类型
				  while(KEY3==0);
			      while(KEY3)
				  {
				  	 Write_Flash_OneByte(XIUZHENG_FUHAO,0);
				     Write_Flash_OneByte(XIUZHENG_VALUE,0);
			         for(i=0;i<6;i++)  
                                  ShansuoT6(); 
				     Show_Value_H=0; Show_Value_L=0;
	                              ShowValue();  //显示值
				     for(i=0;i<8;i++) 
                                      ShansuoT7();
					 for(i=0;i<8;i++)  
					 {
					  ShansuoT7();
					   ADC_do_it();
			           ShowDianchi();  //显示电池容量
					   ChangeValue();
	                   ShowValue();
					 }
					 for(i=0;i<8;i++)  
					 {
					   ShansuoT8();
					   ADC_do_it();
			           ShowDianchi();  //显示电池容量
					   ChangeValue();
	                   ShowValue();
					 }
					 //////////////////
					 //检测浓度  同时声光显示
					 for(i=0;i<4;i++)
					 {
					   delay(); delay();delay(); delay();delay(); delay();BEEPOn; 
					   delay(); delay();delay(); delay();delay(); delay();BEEPOff; 
					 }
					 ZidongJianValueBefore=Show_Value_H*100+Show_Value_L;
					 while(KEY3)
					 {
					   delay();
					   if(KEY1==0) 
					   {
					       if(Show_Value_L==99)
						   { Show_Value_L=0; Show_Value_H=(Show_Value_H+1)%10;}
						   else
						   {
						     Show_Value_L=(Show_Value_L+1)%100;
						   }
					   }
					   else if(KEY4==0) 
					   {
					       if(Show_Value_L==0)
						   { Show_Value_L=99; Show_Value_H=(Show_Value_H+9)%10;}
						   else
						   {
						     Show_Value_L=(Show_Value_L+99)%100;
						   }
					   }
					   ShowValue(); 
					 }
				  } 
				  while(KEY3==0);
				  ShansuoNone();  
				  SaveRecord(1);  
				  ZidongJianValueNow=Show_Value_H*100+Show_Value_L;
			      if(ZidongJianValueNow>=ZidongJianValueBefore)  
			      {
			        Write_Flash_OneByte(XIUZHENG_FUHAO,(uchar)((ZidongJianValueNow-ZidongJianValueBefore)/256));
				   Write_Flash_OneByte(XIUZHENG_VALUE,(uchar)((ZidongJianValueNow-ZidongJianValueBefore)%256));
			      }
			      else
			      {   
			        Write_Flash_OneByte(XIUZHENG_FUHAO,(uchar)((ZidongJianValueBefore-ZidongJianValueNow)/256)|0x80);
				    Write_Flash_OneByte(XIUZHENG_VALUE,(uchar)((ZidongJianValueBefore-ZidongJianValueNow)%256));
			      }
				  BackLightOff;
				  BEEPOn;delay();BEEPOff;
			   
		   }
                else
                {
                     if(BackLightOnFlag==0)//短按键处理 并且在开机情况下
			  {
			     BackLightOnFlag=1;
			     BackLightOn;  //打开背景灯 30秒后关闭
			     BackLightBuffer=HalfMinuteFlag;
			  }
			  else if(BackLightOnFlag==1)//短按键处理 并且在开机情况下
			  {
			     BackLightOnFlag=0;
			     BackLightOff;  //打开背景灯 30秒后关闭
			  }
                }
		   delay();delay();///////////////////////////////////////////////////////////////////////////////////////////////////////end
		}
                if( (BackLightOnFlag==1)&&(((HalfMinuteFlag+30-BackLightBuffer)%30)>=20))
		{ //20秒后自动关闭背光电源
		   BackLightOff;
		   BackLightOnFlag=0;
		   BackLightBuffer=0;
		}
		//////////////////////////////////////////////////////////////////////
                if(KEY3==0)//if((KEY1==0)&&(Guanji_Flag==0))
		{
                  
                     //BackLightOn;
                //BackLightOnFlag=1;
                //if( (BackLightOnFlag==1)&&(((HalfMinuteFlag+30-BackLightBuffer)%30)>=20))
		//{ //20秒后自动关闭背光电源
		   //BackLightOff;
		   //BackLightOnFlag=0;
		   //BackLightBuffer=0;
		//}
		   time_buffer=HalfMinuteFlag;
		   delay();
                  while(((KEY3==0)&&(((HalfMinuteFlag+30-time_buffer)%30)<=4)));
		   
                        if( ((HalfMinuteFlag+30-time_buffer)%30)>=3)
                        {
                                       hign_baojing_type =0; //读取报警类型 共4种 1--4
						stel_baojing_type =0; //读取报警类型 共4种 1--4
						twa_baojing_type =0; //读取报警类型 共4种 1--4
			            Show_Max_Value_Flag=0;  //如果是峰值显示 退回原来正常模式
			            delay();
					    hign_baojing_type =1; //读取报警类型 共4种 1--4
                        ShowChuanganqiType();  //显示传感器和报警类型
				        Show_Value_H=Read_HIGH_H();
				        Show_Value_L=Read_HIGH_L(); //读取保存值
						while(KEY3==0);
			            while(KEY3)
				        {  
					         delay();
                                                 delay();
					         if(KEY1==0) 
					         {
					             if(Show_Value_L==99)
					             {
					               Show_Value_L=0;
					               Show_Value_H=(Show_Value_H+1)%10;
					             }
					             else
					                Show_Value_L=Show_Value_L+1;
					         }
					         else if(KEY4==0)
					         {
					              if(Show_Value_L==0)
					              {
					                 Show_Value_L=99;
					                 Show_Value_H=(Show_Value_H+9)%10;
					              }
					              else
					                 Show_Value_L=Show_Value_L-1;
					         }
                                                
					        ShowValueNone(); 
                                                delay();
                                                delay();
                                                delay();
                                                delay();
                                                delay();
                                                delay();
					         
						     if(KEY1==0)
					         {
					              if(Show_Value_L==99)
					              {
					                 Show_Value_L=0;
					                 Show_Value_H=(Show_Value_H+1)%10;
					              }
					         else
					              Show_Value_L=Show_Value_L+1;
					         }
					         else if(KEY4==0) 
					         {
					              if(Show_Value_L==0)
					              {
					                 Show_Value_L=99;
					                 Show_Value_H=(Show_Value_H+9)%10;
					              }
					              else
					                 Show_Value_L=Show_Value_L-1;
					          }
                                                
                                                
					          ShowValue();//ShowValueNone();
                                                delay();
                                                delay();
                                                delay();
                                                delay();
                                                delay();
                                                delay();
				        } 
					    while(KEY3==0);
					    Write_Flash_OneByte(HIGH_H,Show_Value_H);
				        Write_Flash_OneByte(HIGH_L,Show_Value_L);
				        delay();
						hign_baojing_type =0;
				        stel_baojing_type =1; //读取报警类型 共4种 1--4
                        ShowChuanganqiType();  //显示传感器和报警类型
				        Show_Value_H=Read_STEL_H();
				        Show_Value_L=Read_STEL_L();
				        while(KEY3)
				        {  
					      delay();
					      if(KEY1==0)
					      {
					          if(Show_Value_L==99)
					          {
					            Show_Value_L=0;Show_Value_H=(Show_Value_H+1)%10;
					          }
					          else  Show_Value_L=Show_Value_L+1;
					       }
					       else if(KEY4==0) 
					       {
					           if(Show_Value_L==0)
					           {
					              Show_Value_L=99;Show_Value_H=(Show_Value_H+9)%10;
					           }
					           else  Show_Value_L=Show_Value_L-1;
					        }
					        ShowValueNone();
					        delay();
                                                delay();
                                                delay();
                                                delay();
                                                delay();
                                                delay();
							if(KEY1==0)
					        {
					          if(Show_Value_L==99)
					          {
					            Show_Value_L=0;Show_Value_H=(Show_Value_H+1)%10;
					          }
					          else  Show_Value_L=Show_Value_L+1;
					        }
					        else if(KEY4==0) 
					        {
					           if(Show_Value_L==0)
					           {
					              Show_Value_L=99;Show_Value_H=(Show_Value_H+9)%10;
					           }
					           else  Show_Value_L=Show_Value_L-1;
					        }
					        ShowValue();//ShowValueNone();
                                                delay();
                                                delay();
                                                delay();
                                                delay();
                                                delay();
                                                delay();
				         }
					     while(KEY3==0);
				         Write_Flash_OneByte(STEL_H,Show_Value_H);
				         Write_Flash_OneByte(STEL_L,Show_Value_L);
					     delay();
						 stel_baojing_type =0;
				         twa_baojing_type =1; //读取报警类型 共4种 1--4
                         ShowChuanganqiType();  //显示传感器和报警类型
				         Show_Value_H=Read_TWA_H();
				         Show_Value_L=Read_TWA_L();
				         while(KEY3)
				         {  
					      delay();
					      if(KEY1==0) 
					      {
					          if(Show_Value_L==99)
					          {
					            Show_Value_L=0;Show_Value_H=(Show_Value_H+1)%10;
					          }
					          else  Show_Value_L=Show_Value_L+1;
					       }
					       else if(KEY4==0) 
					        {
					           if(Show_Value_L==0)
					           {
					              Show_Value_L=99;Show_Value_H=(Show_Value_H+9)%10;
					           }
					           else  Show_Value_L=Show_Value_L-1;
					        }
					        ShowValueNone();
					        delay();
                                                delay();
                                                delay();
                                                delay();
                                                delay();
                                                delay();
							if(KEY1==0) 
					        {
					          if(Show_Value_L==99)
					          {
					            Show_Value_L=0;Show_Value_H=(Show_Value_H+1)%10;
					          }
					          else  Show_Value_L=Show_Value_L+1;
					        }
					        else if(KEY4==0)
					        {
					           if(Show_Value_L==0)
					           {
					              Show_Value_L=99;Show_Value_H=(Show_Value_H+9)%10;
					           }
					           else  Show_Value_L=Show_Value_L-1;
					        }
					       ShowValue();// ShowValueNone();
                                                delay();
                                                delay();
                                                delay();
                                                delay();
                                                delay();
                                                delay();
				          }
					      while(KEY3==0);
					      Write_Flash_OneByte(TWA_H,Show_Value_H);
				          Write_Flash_OneByte(TWA_L,Show_Value_L);
					      delay();
				          twa_baojing_type =0; //读取报警类型 共4种 1--4
                          ShowChuanganqiType();  //显示传感器和报警类型
					      /////////时间设定////////////////////
					      ChangeTimeFormat(); 
					      for(i=0;i<7;i++)   Current_time_Buffer[i]=BCD2HEX(Current_time[i]);//获取时间参数
					      for(i=0;i<5;i++)
					     {
					       while(KEY3) //等待回车键
				           {  //从 年  月  日  时  分
					        delay();
					        if(KEY1==0)
                                                 
                                                 {
                                                  delay();
                                                  delay();
                                                  delay();
                                                  delay();
                                                  delay();
                                                  delay();
                                                  if(KEY1==0)
					            {
					            if(i==0) Current_time_Buffer[6]=(Current_time_Buffer[6]+1)%30;
                                                               
                                                               
								else if(i==1) Current_time_Buffer[4]=(Current_time_Buffer[4]+1)%13;
                                                              
                                                                
                                                               
								else if(i==2) Current_time_Buffer[3]=(Current_time_Buffer[3]+1)%32;
                                                               
								else if(i==3) Current_time_Buffer[2]=(Current_time_Buffer[2]+1)%24;
                                                               
								else if(i==4) Current_time_Buffer[1]=(Current_time_Buffer[1]+1)%60;
                                                               
					           }
                                                  }
					        else if(KEY4==0) 
					        {
					            if(i==0) Current_time_Buffer[6]=(Current_time_Buffer[6]+29)%30;
                                                                
								else if(i==1) Current_time_Buffer[4]=(Current_time_Buffer[4]+12)%13;
                                                               
								else if(i==2) Current_time_Buffer[3]=(Current_time_Buffer[3]+31)%32;
                                                                
								else if(i==3) Current_time_Buffer[2]=(Current_time_Buffer[2]+23)%24;
                                                                
								else if(i==4) Current_time_Buffer[1]=(Current_time_Buffer[1]+59)%60;
					        }
					        ShowTimeFormat();
					        delay();
                                                delay();
                                                  delay();
                                                  delay();
                                                  delay();
                                                  delay();
							if(KEY1==0) 
					        {
					            if(i==0) Current_time_Buffer[6]=(Current_time_Buffer[6]+1)%30;
                                                               
								else if(i==1) Current_time_Buffer[4]=(Current_time_Buffer[4]+1)%13; 
                                                                
								else if(i==2) Current_time_Buffer[3]=(Current_time_Buffer[3]+1)%32;
                                                               
								else if(i==3) Current_time_Buffer[2]=(Current_time_Buffer[2]+1)%24;
                                                               
								else if(i==4) Current_time_Buffer[1]=(Current_time_Buffer[1]+1)%60;
					        }
					        else if(KEY4==0) 
					        {
					            if(i==0) Current_time_Buffer[6]=(Current_time_Buffer[6]+29)%30;
                                                                
								else if(i==1) Current_time_Buffer[4]=(Current_time_Buffer[4]+12)%13;
                                                                
								else if(i==2) Current_time_Buffer[3]=(Current_time_Buffer[3]+31)%32;
                                                                
								else if(i==3) Current_time_Buffer[2]=(Current_time_Buffer[2]+23)%24;
                                                                
								else if(i==4) Current_time_Buffer[1]=(Current_time_Buffer[1]+59)%60;
					        }
					        ShowTimeNone(i);
				         } 
						 while(KEY3==0);
						 delay();delay();
					}
					  for(i=0;i<7;i++)   Current_time[i]=HEX2BCD(Current_time_Buffer[i]);
			          Set1302(Current_time);
					  ADC_do_it();
			          ShowDianchi();  
					  Deal_AD_Value();
                               
                                
                             
                        }
                         else
                                {
                                  if(BackLightOnFlag==0)//短按键处理 并且在开机情况下
			           {
			            BackLightOnFlag=1;
			            BackLightOn;  //打开背景灯 30秒后关闭
			            BackLightBuffer=HalfMinuteFlag;
			            }
			            else if(BackLightOnFlag==1)//短按键处理 并且在开机情况下
			            {
			             BackLightOnFlag=0;
			             BackLightOff;  //打开背景灯 30秒后关闭
			             }
                                  
                                }
                               
                       delay();delay();
                        //BackLightOff;
                        
                }
		
		 if( (BackLightOnFlag==1)&&(((HalfMinuteFlag+30-BackLightBuffer)%30)>=20))
		{ //20秒后自动关闭背光电源
		   BackLightOff;
		   BackLightOnFlag=0;
		   BackLightBuffer=0;
		}
		
		
		
    }
       
   
}

