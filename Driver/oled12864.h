#ifndef	_OLED12864_H_
#define _OLED12864_H_





#define RST    PTC9
#define DC     PTC8
#define D0     PTC6
#define D1     PTC7

#define DC_HIGH gpio_set (DC,1)
#define DC_LOW  gpio_set (DC,0)

#define D0_HIGH gpio_set (D0,1)
#define D0_LOW  gpio_set (D0,0)

#define D1_HIGH gpio_set (D1,1)
#define D1_LOW  gpio_set (D1,0)

#define RST_HIGH gpio_set (RST,1)
#define RST_LOW  gpio_set (RST,0)



 extern uint8  reverse; //反转显示
 
 void OLED_Set_Pos(uint8 x, uint8 y);
 void OLED_WrLogo(uint8 data);//画图形专用
 void OLED_WrCmd(uint8 cmd);
 void OLED_Init(void);
 void OLED_CLS(void);
 //重载OLED_WrDat
 void OLED_WrDat(uint8 data, uint8 revs);
 //显示一个6x8标准ASCII字符
void OLED_P6x8Char(char ch);
 void OLED_P6x8Str(uint8 x,uint8 y,char ch[]);
//将一个char型数转换成3位数进行显示 
void OLED_PrintValueI(unsigned char x, unsigned char y, int data);
void OLED_PrintValueF(unsigned char x, unsigned char y, float data, unsigned char num);
 void OLED_Fill(uint8 dat);
 void OLED_Draw_Logo(void);
 void  OLED_Draw_camera();
#endif