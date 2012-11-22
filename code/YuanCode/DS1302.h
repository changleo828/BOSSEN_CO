/********************************************************
��������: DS1302оƬ���Գ���
�� �� �ˣ��� ��
�� ����1.0
˵ ������ȡʱ�������ʱ��
���ʱ�䣺2009��06��13��
********************************************************/
 /* ʵʱʱ������������ */
 /* ʵʱʱ�Ӹ�λ������ */
 /* ʵʱʱ��ʱ�������� */
 //���Գɹ�
#define DS1302_DATAOn  P3OUT |= BIT2
#define DS1302_DATAOff P3OUT &= ~BIT2
#define DS1302_WRITE   P3DIR |= BIT2
#define DS1302_READ    P3DIR &= ~BIT2
#define DS1302_RSTOn   P3OUT |= BIT1
#define DS1302_RSTOff  P3OUT &= ~BIT1
#define DS1302_CLKOn   P3OUT |= BIT3
#define DS1302_CLKOff  P3OUT &= ~BIT3
/* �������� */
void RTInputByte(unsigned char d);             /* ���� 1Byte */
unsigned char RTOutputByte(void);              /* ��� 1Byte */
void W1302(unsigned char ucAddr, unsigned char ucDa);/* ��DS1302д������ */
unsigned char R1302(unsigned char ucAddr);    /* ��ȡDS1302ĳ��ַ������ */
void Set1302(unsigned char *pClock);          /* ����ʱ�� */
void Get1302(unsigned char curtime[]);        /* ��ȡ1302��ǰʱ�� */

void DSdelay(void)
{
  unsigned char i=0;
  for(i=0;i<4;i++);
}
/****************************************************** 
�� �� ����RTInputByte()
�� �ܣ�ʵʱʱ��д��һ�ֽ�
˵ ������DS1302д��1Byte���� (�ڲ�����)
��ڲ�����d  д������� 
�� �� ֵ���� 
*****************************************************/
void RTInputByte(unsigned char d) 
{ 
       unsigned char i=0;
	   DS1302_WRITE;
       for(i=8; i>0; i--)
        {
		  if(d&0x01)  DS1302_DATAOn;
		  else DS1302_DATAOff;
          DS1302_CLKOn;
		  DSdelay();
          DS1302_CLKOff;
          d = d >> 1; 
        } 
}
/****************************************************** 
�� �� ����RTOutputByte()
�� �ܣ�ʵʱʱ�Ӷ�ȡһ�ֽ�
˵ ������DS1302��ȡ1Byte���� (�ڲ�����)
��ڲ������� 
�� �� ֵ��temp  
*****************************************************/
unsigned char RTOutputByte(void) 
{ 
       unsigned char i;
       unsigned char temp=0;
	   DS1302_READ;
       for(i=8; i>0; i--)
         {
           temp=temp>>1;
		   if(P3IN&BIT2) temp=temp|0x80;
           DS1302_CLKOn;
		   DSdelay();
           DS1302_CLKOff;
		   DSdelay();
         } 
       return(temp); 
}
/****************************************************** 
�� �ܣ���DS1302д������
˵ ������д��ַ,��д����/���� (�ڲ�����)
�� �ã�RTInputByte()
��ڲ�����ucAddr��DS1302��ַ, ucData��Ҫд������
�� �� ֵ����    
*****************************************************/
void W1302(unsigned char ucAddr, unsigned char ucData)
{
        DS1302_RSTOff;
        DS1302_CLKOff;
		DSdelay();
        DS1302_RSTOn;
        RTInputByte(ucAddr); /* ��ַ,���� */
        RTInputByte(ucData); /* д1Byte���� */
        DSdelay();
		DS1302_CLKOn;
        DS1302_RSTOff;
		DSdelay();
} 
/****************************************************** 
�� �� ����R1302()
�� �ܣ���ȡDS1302ĳ��ַ������
˵ ������д��ַ,�������/���� (�ڲ�����)
�� �ã�RTInputByte(),RTOutputByte()
��ڲ�����ucAddr��DS1302��ַ
�� �� ֵ��ucData����ȡ������   
*****************************************************/
unsigned char R1302(unsigned char ucAddr)
{
       unsigned char ucData;
       DS1302_RSTOff;
       DS1302_CLKOff;
	   DSdelay();
       DS1302_RSTOn;
       RTInputByte(ucAddr);     /* ��ַ,���� */
       ucData = RTOutputByte(); /* ��1Byte���� */
       DSdelay();
	   DS1302_CLKOn;
       DS1302_RSTOff;
	   DSdelay();
       return(ucData);
}
/****************************************************** 
�� �� ����W1302T_String()
�� �ܣ���DS1302д��ʱ������(���ֽڷ�ʽ)
˵ ������д��ַ,��д����/����
�� �ã�RTInputByte() 
��ڲ�����pWClock��ʱ�����ݵ�ַ ��ʽΪ���� �� ʱ �� �� ���� �� ����
8Byte (BCD��)1B 1B 1B 1B 1B 1B 1B 1B
�� �� ֵ����   
*****************************************************/
void W1302T_String(unsigned char *pWClock)
{
        unsigned char i;
        W1302(0x8e,0x00);      /* ��������,WP=0,д���� */
        DS1302_RSTOff;
        DS1302_CLKOff;
        DSdelay();
		DS1302_RSTOn;
        RTInputByte(0xbe);     /* 0xbe��ʱ�Ӷ��ֽ�д���� */
        for (i = 8; i>0; i--) /* 8Byte = 7Byte ʱ������ + 1Byte ���� */
         {
           RTInputByte(*pWClock); /* д1Byte���� */
           pWClock++;
         }
        DSdelay();
		DS1302_CLKOn;
        DS1302_RSTOff;
		DSdelay();
} 
/****************************************************** 
�� �� ����R1302T_String()
�� �ܣ���ȡDS1302ʱ������
˵ ������д��ַ/����,�������(ʱ�Ӷ��ֽڷ�ʽ)
�� �ã�RTInputByte(),RTOutputByte()
��ڲ�����pRClock����ȡʱ�����ݵ�ַ ��ʽΪ���� �� ʱ �� �� ���� ��
7Byte (BCD��)1B 1B 1B 1B 1B 1B 1B
�� �� ֵ����   
*****************************************************/
void R1302T_String(unsigned char *pRClock)
{
       unsigned char i;
       DS1302_RSTOff;
       DS1302_CLKOff;
       DSdelay();
	   DS1302_RSTOn;
       RTInputByte(0xbf);            /* 0xbf��ʱ�Ӷ��ֽڶ����� */
       for (i=8; i>0; i--) 
        {
          *pRClock = RTOutputByte(); /* ��1Byte���� */
          pRClock++;
         }
       DSdelay();
	   DS1302_CLKOn;
       DS1302_RSTOff;
	   DSdelay();
}
/******************************************************
�� �� ����W1302R_String()
�� �ܣ���DS1302�Ĵ�����д������(���ֽڷ�ʽ)
˵ ������д��ַ,��д����(�Ĵ������ֽڷ�ʽ)
�� �ã�RTInputByte()
��ڲ�����pWReg���Ĵ������ݵ�ַ
�� �� ֵ����   
*****************************************************/
void W1302R_String(unsigned char *pWReg)
{
        unsigned char i;
        W1302(0x8e,0x00);     /* ��������,WP=0,д���� */
        DS1302_RSTOff;
        DS1302_CLKOff;
        DSdelay();
		DS1302_RSTOn;
        RTInputByte(0xfe);    /* 0xbe��ʱ�Ӷ��ֽ�д���� */
        for (i=31; i>0; i--) /* 31Byte �Ĵ������� */
         {
           RTInputByte(*pWReg); /* д1Byte���� */
           pWReg++;
         }
        DSdelay();
		DS1302_CLKOn;
        DS1302_RSTOff;
		DSdelay();
} 
/******************************************************/
/**�� �� ����R1302R_String()
�� �ܣ���ȡDS1302�Ĵ�������
˵ ������д��ַ,�������/����(�Ĵ������ֽڷ�ʽ)
�� �ã�RTInputByte(),RTOutputByte()
��ڲ�����pRReg���Ĵ������ݵ�ַ
�� �� ֵ����  
*****************************************************/
void R1302R_String(unsigned char *pRReg)
{  
        unsigned char i;
        DS1302_RSTOff;
        DS1302_CLKOff;
        DSdelay();
		DS1302_RSTOn;
        RTInputByte(0xff);    /* 0xff��ʱ�Ӷ��ֽڶ����� */
        for (i=31; i>0; i--) /* 31Byte �Ĵ������� */
         {
           *pRReg = RTOutputByte(); /* ��1Byte���� */
           pRReg++;
         }
        DSdelay();
		DS1302_CLKOn;
        DS1302_RSTOff;
		DSdelay();
}
/******************************************************/
/**�� �� ����Set1302()
�� �ܣ����ó�ʼʱ��
˵ ������д��ַ,�������/����(�Ĵ������ֽڷ�ʽ)
�� �ã�W1302()
��ڲ�����pClock������ʱ�����ݵ�ַ ��ʽΪ���� �� ʱ �� �� ���� ��
7Byte (BCD��)1B 1B 1B 1B 1B 1B 1B
�� �� ֵ����   
*****************************************************/
void Set1302(unsigned char *pClock) 
{
       unsigned char i;
       unsigned char ucAddr = 0x80; 
       W1302(0x8e,0x00);         /* ��������,WP=0,д���� */
       for(i =7; i>0; i--)
        { 
          W1302(ucAddr,*pClock); /* �� �� ʱ �� �� ���� �� */ 
          pClock++;
          ucAddr +=2;
        }
        W1302(0x8e,0x80);        /* ��������,WP=1,д���� */
}
/******************************************************/
/**�� �� ����Get1302()
�� �ܣ���ȡDS1302��ǰʱ��
˵ ����
�� �ã�R1302() 
��ڲ�����ucCurtime�����浱ǰʱ���ַ����ǰʱ���ʽΪ���� �� ʱ �� �� ���� �� 
7Byte (BCD��) 1B 1B 1B 1B 1B 1B 1B
�� �� ֵ����    
******************************************************/
void Get1302(unsigned char ucCurtime[]) 
{
       unsigned char i;
       unsigned char ucAddr = 0x81;
       for (i=0; i<7; i++)
        {
           ucCurtime[i] = R1302(ucAddr);/* ��ʽΪ���� �� ʱ �� �� ���� �� */
           ucAddr += 2;
        }
}

