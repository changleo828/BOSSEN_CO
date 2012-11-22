/********************************************************
功能描述: DS1302芯片测试程序
设 计 人：冯 军
版 本：1.0
说 明：读取时间和设置时间
完成时间：2009年06月13日
********************************************************/
 /* 实时时钟数据线引脚 */
 /* 实时时钟复位线引脚 */
 /* 实时时钟时钟线引脚 */
 //测试成功
#define DS1302_DATAOn  P3OUT |= BIT2
#define DS1302_DATAOff P3OUT &= ~BIT2
#define DS1302_WRITE   P3DIR |= BIT2
#define DS1302_READ    P3DIR &= ~BIT2
#define DS1302_RSTOn   P3OUT |= BIT1
#define DS1302_RSTOff  P3OUT &= ~BIT1
#define DS1302_CLKOn   P3OUT |= BIT3
#define DS1302_CLKOff  P3OUT &= ~BIT3
/* 函数声明 */
void RTInputByte(unsigned char d);             /* 输入 1Byte */
unsigned char RTOutputByte(void);              /* 输出 1Byte */
void W1302(unsigned char ucAddr, unsigned char ucDa);/* 向DS1302写入数据 */
unsigned char R1302(unsigned char ucAddr);    /* 读取DS1302某地址的数据 */
void Set1302(unsigned char *pClock);          /* 设置时间 */
void Get1302(unsigned char curtime[]);        /* 读取1302当前时间 */

void DSdelay(void)
{
  unsigned char i=0;
  for(i=0;i<4;i++);
}
/****************************************************** 
函 数 名：RTInputByte()
功 能：实时时钟写入一字节
说 明：往DS1302写入1Byte数据 (内部函数)
入口参数：d  写入的数据 
返 回 值：无 
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
函 数 名：RTOutputByte()
功 能：实时时钟读取一字节
说 明：从DS1302读取1Byte数据 (内部函数)
入口参数：无 
返 回 值：temp  
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
功 能：往DS1302写入数据
说 明：先写地址,后写命令/数据 (内部函数)
调 用：RTInputByte()
入口参数：ucAddr：DS1302地址, ucData：要写的数据
返 回 值：无    
*****************************************************/
void W1302(unsigned char ucAddr, unsigned char ucData)
{
        DS1302_RSTOff;
        DS1302_CLKOff;
		DSdelay();
        DS1302_RSTOn;
        RTInputByte(ucAddr); /* 地址,命令 */
        RTInputByte(ucData); /* 写1Byte数据 */
        DSdelay();
		DS1302_CLKOn;
        DS1302_RSTOff;
		DSdelay();
} 
/****************************************************** 
函 数 名：R1302()
功 能：读取DS1302某地址的数据
说 明：先写地址,后读命令/数据 (内部函数)
调 用：RTInputByte(),RTOutputByte()
入口参数：ucAddr：DS1302地址
返 回 值：ucData：读取的数据   
*****************************************************/
unsigned char R1302(unsigned char ucAddr)
{
       unsigned char ucData;
       DS1302_RSTOff;
       DS1302_CLKOff;
	   DSdelay();
       DS1302_RSTOn;
       RTInputByte(ucAddr);     /* 地址,命令 */
       ucData = RTOutputByte(); /* 读1Byte数据 */
       DSdelay();
	   DS1302_CLKOn;
       DS1302_RSTOff;
	   DSdelay();
       return(ucData);
}
/****************************************************** 
函 数 名：W1302T_String()
功 能：往DS1302写入时钟数据(多字节方式)
说 明：先写地址,后写命令/数据
调 用：RTInputByte() 
入口参数：pWClock：时钟数据地址 格式为：秒 分 时 日 月 星期 年 控制
8Byte (BCD码)1B 1B 1B 1B 1B 1B 1B 1B
返 回 值：无   
*****************************************************/
void W1302T_String(unsigned char *pWClock)
{
        unsigned char i;
        W1302(0x8e,0x00);      /* 控制命令,WP=0,写操作 */
        DS1302_RSTOff;
        DS1302_CLKOff;
        DSdelay();
		DS1302_RSTOn;
        RTInputByte(0xbe);     /* 0xbe：时钟多字节写命令 */
        for (i = 8; i>0; i--) /* 8Byte = 7Byte 时钟数据 + 1Byte 控制 */
         {
           RTInputByte(*pWClock); /* 写1Byte数据 */
           pWClock++;
         }
        DSdelay();
		DS1302_CLKOn;
        DS1302_RSTOff;
		DSdelay();
} 
/****************************************************** 
函 数 名：R1302T_String()
功 能：读取DS1302时钟数据
说 明：先写地址/命令,后读数据(时钟多字节方式)
调 用：RTInputByte(),RTOutputByte()
入口参数：pRClock：读取时钟数据地址 格式为：秒 分 时 日 月 星期 年
7Byte (BCD码)1B 1B 1B 1B 1B 1B 1B
返 回 值：无   
*****************************************************/
void R1302T_String(unsigned char *pRClock)
{
       unsigned char i;
       DS1302_RSTOff;
       DS1302_CLKOff;
       DSdelay();
	   DS1302_RSTOn;
       RTInputByte(0xbf);            /* 0xbf：时钟多字节读命令 */
       for (i=8; i>0; i--) 
        {
          *pRClock = RTOutputByte(); /* 读1Byte数据 */
          pRClock++;
         }
       DSdelay();
	   DS1302_CLKOn;
       DS1302_RSTOff;
	   DSdelay();
}
/******************************************************
函 数 名：W1302R_String()
功 能：往DS1302寄存器数写入数据(多字节方式)
说 明：先写地址,后写数据(寄存器多字节方式)
调 用：RTInputByte()
入口参数：pWReg：寄存器数据地址
返 回 值：无   
*****************************************************/
void W1302R_String(unsigned char *pWReg)
{
        unsigned char i;
        W1302(0x8e,0x00);     /* 控制命令,WP=0,写操作 */
        DS1302_RSTOff;
        DS1302_CLKOff;
        DSdelay();
		DS1302_RSTOn;
        RTInputByte(0xfe);    /* 0xbe：时钟多字节写命令 */
        for (i=31; i>0; i--) /* 31Byte 寄存器数据 */
         {
           RTInputByte(*pWReg); /* 写1Byte数据 */
           pWReg++;
         }
        DSdelay();
		DS1302_CLKOn;
        DS1302_RSTOff;
		DSdelay();
} 
/******************************************************/
/**函 数 名：R1302R_String()
功 能：读取DS1302寄存器数据
说 明：先写地址,后读命令/数据(寄存器多字节方式)
调 用：RTInputByte(),RTOutputByte()
入口参数：pRReg：寄存器数据地址
返 回 值：无  
*****************************************************/
void R1302R_String(unsigned char *pRReg)
{  
        unsigned char i;
        DS1302_RSTOff;
        DS1302_CLKOff;
        DSdelay();
		DS1302_RSTOn;
        RTInputByte(0xff);    /* 0xff：时钟多字节读命令 */
        for (i=31; i>0; i--) /* 31Byte 寄存器数据 */
         {
           *pRReg = RTOutputByte(); /* 读1Byte数据 */
           pRReg++;
         }
        DSdelay();
		DS1302_CLKOn;
        DS1302_RSTOff;
		DSdelay();
}
/******************************************************/
/**函 数 名：Set1302()
功 能：设置初始时间
说 明：先写地址,后读命令/数据(寄存器多字节方式)
调 用：W1302()
入口参数：pClock：设置时钟数据地址 格式为：秒 分 时 日 月 星期 年
7Byte (BCD码)1B 1B 1B 1B 1B 1B 1B
返 回 值：无   
*****************************************************/
void Set1302(unsigned char *pClock) 
{
       unsigned char i;
       unsigned char ucAddr = 0x80; 
       W1302(0x8e,0x00);         /* 控制命令,WP=0,写操作 */
       for(i =7; i>0; i--)
        { 
          W1302(ucAddr,*pClock); /* 秒 分 时 日 月 星期 年 */ 
          pClock++;
          ucAddr +=2;
        }
        W1302(0x8e,0x80);        /* 控制命令,WP=1,写保护 */
}
/******************************************************/
/**函 数 名：Get1302()
功 能：读取DS1302当前时间
说 明：
调 用：R1302() 
入口参数：ucCurtime：保存当前时间地址。当前时间格式为：秒 分 时 日 月 星期 年 
7Byte (BCD码) 1B 1B 1B 1B 1B 1B 1B
返 回 值：无    
******************************************************/
void Get1302(unsigned char ucCurtime[]) 
{
       unsigned char i;
       unsigned char ucAddr = 0x81;
       for (i=0; i<7; i++)
        {
           ucCurtime[i] = R1302(ucAddr);/* 格式为：秒 分 时 日 月 星期 年 */
           ucAddr += 2;
        }
}

