/********************************************************
��������: HT1621оƬ���Գ���
�� �� �ˣ��� ��
�� ����1.0
˵ ���������Գ����ܹ�����HT1621��ÿһ���ֶΣ����ε���
ÿһ���ֶ�
���ʱ�䣺2009��06��13��
********************************************************/
#define uchar unsigned char
#define uint unsigned int

#define BIAS   0x52  //0b1000 0101 0010 1/3duty 4com
#define SYSDIS 0X00  //0b1000 0000 0000 ����ϵͳ������LCDƫѹ������
#define SYSEN  0X02  //0b1000 0000 0010 ��ϵͳ����
#define LCDOFF 0X04  //0b1000 0000 0100 ��LCDƫѹ
#define LCDON  0X06  //0b1000 0000 0110 ��LCDƫѹ
#define XTAL   0x28  //0b1000 0010 1000 �ⲿ��ʱ��
#define RC256  0X30  //0b1000 0011 0000 �ڲ�ʱ��
#define TONEON 0X12  //0b1000 0001 0010 ���������
#define TONEOFF 0X10 //0b1000 0001 0000 �ر��������
#define WDTDIS 0X0A //0b1000 0000 1010 ��ֹ���Ź�

/*HT1621����λ��Һ��ģ��ӿڶ��壬�������ѵ���Ҫ���ģ�*/
/*����˿�HT1621���ݶ˿� */
#define HT1621_WR1       P5OUT |= BIT3  
#define HT1621_WR0       P5OUT &= ~BIT3 
#define HT1621_DATA1     P5OUT |= BIT1 
#define HT1621_DATA0     P5OUT &= ~BIT1 
#define HT1621_CS1       P5OUT |= BIT4 
#define HT1621_CS0       P5OUT &= ~BIT4 

uchar ShowSecendFlag=0;

uchar time_dig[17];  //���ʱ������

/*delay us*/
void SmallDelay(void) //5,7,9
{
  uchar i=0;
  for(i=0;i<4;i++);
}

//delay ms
void DelayMS(uint iMs)
{
  uint i,j;
  for(i=0;i<iMs;i++)
    for(j=0;j<30;j++) SmallDelay();
}

/******************************************************
д���ݺ���,cntΪ��������λ��,���ݴ���Ϊ��λ��ǰ
*******************************************************/
void Ht1621Wr_Data(uchar Data,uchar cnt)
{
    uchar i;
    for (i=0;i<cnt;i++)
    {
        HT1621_WR0;
        SmallDelay();
        if(Data&0x80) HT1621_DATA1; 
           else HT1621_DATA0;
        //SmallDelay();
        HT1621_WR1;
        SmallDelay();
        Data<<=1;
    }
}
/********************************************************
�������ƣ�void Ht1621WrCmd(uchar Cmd)
��������: HT1621����д�뺯��
ȫ�ֱ�������
����˵����CmdΪд����������
����˵������
�� �� �ˣ�ZHCE
�� ����1.0
˵ ����д�������ʶλ100
********************************************************/
void Ht1621WrCmd(uchar Cmd)
{
  HT1621_CS0;
  SmallDelay();
  Ht1621Wr_Data(0x80,4); //д�������־100
  Ht1621Wr_Data(Cmd,8); //д����������
  HT1621_CS1;
  SmallDelay();
}
/********************************************************
�������ƣ�void Ht1621WrOneData(uchar Addr,uchar Data)
��������: HT1621��ָ����ַд�����ݺ���
ȫ�ֱ�������
����˵����AddrΪд���ʼ��ַ��DataΪд������
����˵������
�� �� �ˣ�ZHCE
�� ����1.0
˵ ������ΪHT1621������λ4λ������ʵ��д������Ϊ�����ĺ�4λ
********************************************************/
void Ht1621WrOneData(uchar Addr,uchar Data)
{
  HT1621_CS0;
  Ht1621Wr_Data(0xa0,3); //д�����ݱ�־101
  Ht1621Wr_Data(Addr<<2,6); //д���ַ����
  Ht1621Wr_Data(Data<<4,4); //д������
  HT1621_CS1;
  SmallDelay();
}
/********************************************************
�������ƣ�void Ht1621WrAllData(uchar Addr,uchar *p,uchar cnt)
��������: HT1621����д�뷽ʽ����
ȫ�ֱ�������
����˵����AddrΪд���ʼ��ַ��*pΪ����д������ָ�룬
cntΪд����������
����˵������
�� �� �ˣ�ZHCE
�� ����1.0
˵ ����HT1621������λ4λ���˴�ÿ������Ϊ8λ��д������
������8λ����
********************************************************/
void Ht1621WrAllData(uchar Addr,uchar *p,uchar cnt)
{
    uchar i;
    HT1621_CS0;
    Ht1621Wr_Data(0xa0,3); //д�����ݱ�־101
    Ht1621Wr_Data(Addr<<2,6); //д���ַ����
    for (i=0;i<cnt;i++)
    {
        Ht1621Wr_Data(*p,8); //д������
        p++;
    }
    HT1621_CS1;
    SmallDelay();
}
/********************************************************
�������ƣ�void Ht1621_Init(void)
��������: HT1621��ʼ��
ȫ�ֱ�������
����˵������
����˵������
�� �� �ˣ�ZHCE
�� ����1.0
˵ ������ʼ����Һ���������ֶξ���ʾ
********************************************************/
void Ht1621_Init(void)
{
  HT1621_CS1;
  HT1621_WR1;
  HT1621_DATA1;
  DelayMS(1); //��ʱʹLCD������ѹ�ȶ�
  Ht1621WrCmd(LCDOFF);
  Ht1621WrCmd(BIAS);
  Ht1621WrCmd(RC256); //ʹ���ڲ�����
  Ht1621WrCmd(SYSDIS);
  Ht1621WrCmd(WDTDIS);
  Ht1621WrCmd(SYSEN);
  Ht1621WrCmd(LCDON);
}

/**------------------------------------------------------------------------- 
                           Name: all_off(���1621��ʾ) 
---------------------------------------------------------------------------*/ 
void HT1621_all_off(void) 
{ 
   uchar i; 
   uchar addr=0; 
   for(i=0;i<34;i++) 
   { 
      Ht1621WrOneData(addr,0x00); 
      addr+=1; 
   } 
} 
/**************************************************************************** 
                           Name: all_on(ȫ������1621) 
****************************************************************************/ 
void HT1621_all_on(void) 
{ 
   uchar i; 
   uchar addr=0; 
   for(i=0;i<34;i++) 
   { 
     Ht1621WrOneData(addr,0xff); 
     addr+=1; 
   } 
} 

