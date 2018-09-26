/*
 * @file       main.c
 * @brief      ������
 */
#include "include.h"
#define Servo_Kp 2.5
#define Servo_Kd 0.1

/*************
��������ʾ���ֵ������ͷ������
**************/
uint16 edgposition[CAMERA_H];

float Variable[20]; //��������Դ洢���������͵���λ��ͼ�񴰿�

void sendimg();          //����ͼ����λ��

float Distance;
uint8  RoadType=0;
float Delt_error,Middle_Err;
float Previous_Error[12];
float sum_err;

float character_distance;//�����������λ��

uint8 Stop;


uint8   LMR[CAMERA_H][3];

uint8  road_type;
uint16 cont;

float motor1_out,motor2_out; 
int duty = 25;
//������


int sever_middle=140;  //����ڱۻ�����������Ҫ����ʵ������޸ģ�������(155/1000)*10ms=1.55ms 1000�����徫�� 
int sever_range=25;    //����һ�¶���ڶ��ķ��ȣ���ֹ������ɻ�е�𻵣���Լ����25�ȣ�����ʵ������޸ģ�
int servo_middle=140;  //����ڱۻ�����������Ҫ����ʵ������޸ģ�������(155/1000)*10ms=1.55ms 1000�����徫�� 
int servo_range=25;    //����һ�¶���ڶ��ķ��ȣ���ֹ������ɻ�е�𻵣���Լ����25�ȣ�����ʵ������޸ģ�
int duty_servo=0;
//������
int16 servo_duty=0;

float pid_p,pid_i,pid_d,PID_TURN=0;
float controll;
//���������

void my_putchar(char temp)
{
    uart_putchar(UART0,temp); //����ʵ�ʵĴ��ں����޸�
}

/*����֪ͨ��λ���µ�һ�����ݿ�ʼ��Ҫ�������ݱ��뷢����*/
void Send_Begin()
{
  my_putchar(0x55);
  my_putchar(0xaa);
  my_putchar(0x11);
}
void Send_Variable()
{
  uint8 i=0,ch=0;
  float temp=0;
  uint8 Variable_num=16;
  my_putchar(0x55);
  my_putchar(0xaa);
  my_putchar(0xff);
  my_putchar(0x01);
  my_putchar(Variable_num);
 for(i=0;i<Variable_num;i++)
  {
    temp=Variable[i];
    ch=BYTE0(temp);
    my_putchar(ch);
    ch=BYTE1(temp);
    my_putchar(ch);
    ch=BYTE2(temp);
    my_putchar(ch);
    ch=BYTE3(temp);
    my_putchar(ch);
  }
     my_putchar(0x01);
}


/*!
 *  @brief      ����ͼ����λ����ʾ  
 */
void sendimg()
{
   uint8 ch=0;
  float temp=0;
  uint16 i=0,num;
  my_putchar(0x55);
  my_putchar(0xaa);
  my_putchar(0xff);
  my_putchar(0xa2);
  my_putchar(0x1); //С��״̬
  
  num=cont+2+180+36;  
  //ͳ�ƽ�Ҫ����������� 2����ΪҪ����ؼ��ּ�0xf0��0xf2
  //180�Ǳ��ߵ�λ 36�Ǳ�����λ ���������Ͳ�Ҫ���ϣ�
  
  my_putchar(BYTE0(num)); 
  my_putchar(BYTE1(num));
 for(i=0;i< cont;i++)
 {
     my_putchar(img_edg[i]);
 }
 my_putchar(0xf0);  //����ͼ�����ݷ�����
 /******************�Ǻ�Χ�����Ŀ��Բ�����*******************/
 for(i=0;i<180;i++)
 {
  my_putchar( LMR[i/60][i%60]);
 }
  for(i=0;i<9;i++)
  {
    temp=Variable[i];
    ch=BYTE0(temp);
    my_putchar(ch);
    ch=BYTE1(temp);
    my_putchar(ch);
    ch=BYTE2(temp);
    my_putchar(ch);
    ch=BYTE3(temp);
    my_putchar(ch);
  }
 /*****************************************************/
 my_putchar(0xf2); //�����������ݶ�������
 
}

void Push_And_Pull(float *buff,int len,float newdata)
{
 int i;
 for(i=len-1;i>0;i--)
 {
   *(buff+i)=*(buff+i-1);
   sum_err +=*(buff+i); 
 }
   *buff=newdata;
   sum_err +=*buff;
}
   
float Slope_Calculate_Uint8(uint8 begin,uint8 end,uint8 *p)    //��С���˷����б��
{
  float xsum=0,ysum=0,xysum=0,x2sum=0;
   uint8 i=0;
   float result=0;
   static float resultlast;
   p=p+begin;
   for(i=begin;i<end;i++)
   {
	   xsum+=i;
	   ysum+=*p;
	   xysum+=i*(*p);
	   x2sum+=i*i;
	   p=p+1;
   }
  if((end-begin)*x2sum-xsum*xsum) //�жϳ����Ƿ�Ϊ�� 
  {
    result=((end-begin)*xysum-xsum*ysum)/((end-begin)*x2sum-xsum*xsum);
    resultlast=result;
  }
  else
  {
   result=resultlast;
  }
  return result;
}

void fix_break_line() //�޸��Ͽ�����
{
  
}

void Search()
{
  //�ӵײ���������
  float Middle_Err_Sum=0,slope;
  static int i,j,find;
  uint8 left_cont=0,right_cont=0;
  static float Middle_Temp=0;
  int leftfind=0,rightfind=0;
  int AllWhileStartLine=0,AllWhileEndLine=0;
  static int break_line_left=0,break_line_right=0,continue_line_left=0,continue_line_right=0;
  
  int search_end_line=0;
  for(i=0;i<60;i++)  //�������
  {  
        LMR[0][i]=0; //���������
        LMR[1][i]=0;  //��������
        LMR[2][i]=80; //�ұ�������
  }
  
  leftfind=0;
  rightfind=0;
  break_line_left=0;
  break_line_right=0;
  continue_line_left=0;
  continue_line_right=0;
  
  for(i=59;i>0;i--) //�ӵ�59�п�ʼ����
  {
    if(edgposition[i]==0&&(i!=0)) //ȫ���� ��Ϊ����  ???
    {
      break;
    }
    
    j=edgposition[i];//���������ؿ�ʼ��λ��  j����ڱ�� ������� j+1������ ���ұ��� 
    
    if(i==59)  //�ײ���ʼ��
    {   
        while(img_edg[j]!=255)
       {  
        if((img_edg[j]<55)&&(img_edg[j+1]>25))  //�����С��55 �ұ��ش���25
        {
          if((img_edg[j+1]-img_edg[j])>25) //�ұ���-����ش���20
          {
            LMR[0][i]=img_edg[j];
            if(img_edg[j+1]==255)
            {
               LMR[2][i]=80;      
            }
            else
            {
               LMR[2][i]=img_edg[j+1];
            }
            break;//while
          }
        } 
        if(img_edg[j+1]==255)
        {
          break;//while
        }
        j=j+2;
       }
       
    //  if(LMR[0][i]==0)  break_line_left=59;
     // if(LMR[2][i]==80) break_line_right=59;
    }
    else   //���ǵײ���ʼ��
    { 
        find=0;
        while(img_edg[j]!=255)
       {
         
        if((img_edg[j]<=LMR[2][i+1])&&(img_edg[j+1]>=LMR[0][i+1])&&(img_edg[j+1]-img_edg[j])>8)     //�����С����һ�е��ұ��� �ұ��ش�����һ�е����������ͨ��
        {
          find=1;
          if(LMR[0][i]==0&&(img_edg[j]!=0))
          {
            if(break_line_left!=0&&(continue_line_left==0))
            {
              if(img_edg[j]>LMR[0][break_line_left]&&img_edg[j]<55)
              {
                LMR[0][i]=img_edg[j];
                continue_line_left=i;
                leftfind=1;
              }
            }
            else
            {
                if(LMR[2][i]==80)
                {
                 LMR[0][i]=img_edg[j];
                 leftfind=1;
                }
            }
          } 

          if(img_edg[j+1]!=255&&(LMR[2][i]==80))
          {
            if(break_line_right!=0&&(continue_line_right==0)&&(img_edg[j+1]>20))
            {
              if(img_edg[j+1]<LMR[2][break_line_right])
              {
                LMR[2][i]=img_edg[j+1];
                continue_line_right=i;
                rightfind=1;
              }
            }
            else
            {
                if((RoadType==1))
                 {
                   if(img_edg[j+1]>30)
                   {
                    LMR[2][i]=img_edg[j+1];
                    rightfind=1;
                   }
                 }
                else
                {
                   LMR[2][i]=img_edg[j+1];
                   rightfind=1;
                }
            }
          }
        }
        if(img_edg[j+1]==255) //���е������ؽ�����
        {
          if(img_edg[j]==0) //˵������Ϊȫ�� 
          { 
            if(AllWhileStartLine==0)
            {
              AllWhileStartLine=i; //ȫ���п�ʼ��λ��
            }
            AllWhileEndLine=i;

            if((rightfind&&leftfind&&(RoadType==0)&&i>20)||(AllWhileStartLine-AllWhileEndLine)>10)
              //�����߱��߶��ҵ��� �ų�����20�еĸ���
            {
             RoadType=1; //����ʮ����
             Middle_Temp=Middle_Err;
             character_distance=Distance;
            }
          }
          break;//while
        }
        j=j+2;
       }
       //������������ͨ����㷨
       
       
       
       if(RoadType==1)  //��ʮ���ڶ����߽������⴦��
       { 
         if(left_cont>=4)
         {
           if(((LMR[0][i]<(LMR[0][i+1]-1))||(LMR[0][i]==0))&&i>20)
           {
                LMR[0][i]=0;
                if(break_line_left==0)break_line_left=i+1;         //������չ���߶���
           }
         }
         if(LMR[0][i]!=0) //�ҵ�����߱���
         {
           left_cont++;       
         }
         else
         {
           left_cont=0;
         }
         
         if(right_cont>=4)
         {
           if(((LMR[2][i]>(LMR[2][i+1]+1))||(LMR[2][i]==80))&&i>20)
           {
                LMR[2][i]=80;
                 if(break_line_right==0)break_line_right=i+1;       //������չ���߶���
           }
         }
         if(LMR[2][i]!=80) //�ҵ����ұ���
         {
           right_cont++;
         }
         else
         {
           right_cont=0;
         }
       }
       
       if(find==0)//û���ҵ���ͨ����
       {
         search_end_line=i;
         i++; 
         break;//for
       }
  } 
 }
 if(RoadType==1)
 {
   if(AllWhileEndLine==0||AllWhileEndLine<20)RoadType=0;
   if(continue_line_left!=0)
   {
       for(i=continue_line_left-1;(i>continue_line_left-10)&&(i>0);i--) //�����п��������� ���һ�� 
      {
        if((LMR[0][i]!=0)&&(LMR[0][i-1]!=0))
        {
          if(ABS(LMR[0][i]-LMR[0][i-1])<2)
          {
              if(ABS(LMR[0][i-1]-LMR[0][i-2])<2)
            {
              continue_line_left=i;
              break;
            }
          }
        }
      }
      
      for(i=break_line_left;i>=continue_line_left;i--) //��ʼ����
      { 
        slope=(LMR[0][break_line_left]- LMR[0][continue_line_left])*1.0/(break_line_left-continue_line_left);
        LMR[0][i]= LMR[0][break_line_left]-(int)((break_line_left-i)*slope);
      }
   }
   
   if(break_line_right!=0&&continue_line_right!=0)
   {
       for(i=continue_line_right;(i>continue_line_right-10)&&(i>0);i--) //�����п��������� ���һ�� 
      {
        if((LMR[2][i]!=0)&&(LMR[2][i-1]!=0))
        {
          if(ABS(LMR[2][i]-LMR[2][i-1])<2)
          {
            continue_line_right=i;
            break;
          }
        }
      }
         for(i=break_line_right;i>=continue_line_right;i--) //��ʼ����
      { 
        slope=(LMR[2][break_line_right]- LMR[2][continue_line_right])*1.0/(break_line_right-continue_line_right);
        LMR[2][i]= LMR[2][break_line_right]-(int)((break_line_right-i)*slope);
      }
      
   }
   
   
   
 }
  for(i=0;i<59;i++)
   {
     if(LMR[0][i]!=0||LMR[2][i]!=80)
     {
        LMR[1][i]=(LMR[0][i]+LMR[2][i])/2;
     } 
    
    if(search_end_line<25)
    {
      if(i>=25&&i<28)
      {
        Middle_Err_Sum=Middle_Err_Sum+ LMR[1][i]-40;
      }
    }
    else
    {
      if((i>=search_end_line-3)&&(i<search_end_line))
      {
          Middle_Err_Sum=Middle_Err_Sum+ LMR[1][i]-40;
      }
    }      
   }
    Middle_Err_Sum=Middle_Err_Sum/3;
  
   if(RoadType==1)
   {
      for(i=AllWhileEndLine;i>10;i--)
     {
       if(LMR[0][i]!=0&&LMR[2][i]!=80)
       {
           if(LMR[0][i-2]!=0&&LMR[2][i-2]!=80)
         {
          Middle_Err_Sum=(LMR[0][i-2]+LMR[2][i-2])/2-40;
          break;
         }
       }
     }
   }
 
if(search_end_line>55&&Distance>1) Stop=1;


  Middle_Err= Middle_Err_Sum;   //�������
  Push_And_Pull(Previous_Error,10,Middle_Err);   //Previous_Error[12]ǰʮ��Ԫ�ش������ʮ��ʱ�̵��������
  //Delt_error=-10*Slope_Calculate(0,10,Previous_Error);   //���ʮ��ʱ�̵�����������ʱ���б��
 
 }  

void get_edge()   //�������ٳ˷�����
{
  
  static int16 i=0,j=0,last=0,x=0,y=0,n=0;
  uint8 temp=0,find=0;
  cont=0;  //�������
  for(i=0;i<60;i++)
  {
    last=0;  //��ʼ��Ϊ��
    x=i*10;
    find=0;
    edgposition[i]=0;
    edgposition[i]=0;
    for(j=0;j<10;j++)
    {
      if(imgbuff_process[x+j]==0xff)  //imgbuff_process ����600���ֽڣ�ÿ��10���ֽ�
      {
        if(last==0)
        {
              y=j<<3;      //���ƶ�3�൱�ڳ���8
              if(find==0)
              {
                edgposition[i]=cont;
              }
              img_edg[cont++]=y;    
              find=1;
        }
         last=1;
         continue;
      }
       if(imgbuff_process[x+j]==0)
      {
        if(last==1)
        {          
               y=j<<3;
              if(find==0)
              {
                edgposition[i]=cont;
              }
              img_edg[cont++]=y;    
              find=1;
        }
         last=0;
         continue;
      }
      
      for(n=7;n>=0;n--) //ÿ���ֽ�8λ���Ӹߵ���
      {
            temp=(imgbuff_process[x+j]>>n)&1; // ��ȡ�õ�����ֵ ��0��1��     
            if(temp!=last) //����һ��ֵ����ͬ ������������            
            {
               y=j<<3;  
               if(find==0)
              {
                edgposition[i]=cont;
              }
               img_edg[cont++]=y+7-n;   
               find=1;
            }
              last=temp;                //�洢�õ��ֵ
      } 
    } 
     img_edg[cont++]=0xff;   

  }
}

/*img_edg��һ��һά���� ��¼������ͷÿ�е������ص�����ֵ  ÿ�������� �������ؿ�ʼ���ɺڱ�ף�Ȼ����½��أ��ɰױ�ڣ� 
   0xff����ָʾ���е����������ˣ���ʼ��¼��һ��
   ���ÿ��ͼ���԰�ɫ���ֿ�ʼ����ô������������ʼλ��Ϊ0����Ϊÿ�г�ʼֵΪ�ڣ��ʻ���һ��������
   �������ȫ�� ��ô���м�¼Ϊ0xff
   �������Ϊȫ�� ��¼Ϊ 0 0xff
 
   
   0xff����������ֵ������������һ��

   edgposition[i]�����i�е� ������ �� img_edg ���������

 */


void UART0_RX_IRQHandler()
{
  static uint8 recv=0,recievingdata=0,cnt=0;
  static uint8 predata[4];
  static uint8  data[100];
  uint8 update_variable;
  while(uart_query(UART0)==1)  
  {
    uart_getchar (UART0,(char*)(&recv));  //����ʵ�ʵĴ������޸�  
    if(recievingdata)
    {
      if(cnt>=56)
      {
        if(recv==2)//У��֡β
        {
          update_variable=1;
        }
        recievingdata=0;
      }
      data[cnt++]=recv;
    }    
    if((predata[1]==85)&&(predata[0]==170)&&(2==recv))
    {
      recievingdata=1;
      cnt=0;
    }  
    predata[3]=predata[2];
    predata[2]=predata[1];
    predata[1]=predata[0];
    predata[0]=recv;
  }
 // uart_rx_irq_en(UART0);//ʹ�ܴ��ڽ����жϣ���ֹ�������жϱ��ر� 
    
}

void Motor_Out() //��������������
{
  
 motor1_out=LIMIT(motor1_out,0.99,-0.99);  //��������ֵ������-1~1 ֮�䣬-1����ת��죬1������ת��죻
 motor2_out=LIMIT(motor2_out,0.99,-0.99);
  //���1
  if(motor1_out>=0) //ռ�ձ�Ϊ������ת
  {
     FTM_PWM_Duty(FTM0,FTM_CH0,(int)(motor1_out*10000));//ռ�ձȾ���Ϊ10000 ��ռ�ձ�*ռ�ձȾ���
     FTM_PWM_Duty(FTM0,FTM_CH2,0);
  }
  else   //Ϊ���ͷ�ת
  {
     FTM_PWM_Duty(FTM0,FTM_CH0,0);
     FTM_PWM_Duty(FTM0,FTM_CH2,(int)(-motor1_out*10000));
  }
  
  //���2
    if(motor2_out>=0) //ռ�ձ�Ϊ������ת
  {
     FTM_PWM_Duty(FTM0,FTM_CH1,(int)(motor2_out*10000)); 
     FTM_PWM_Duty(FTM0,FTM_CH3,0);
  }
  else   //��ת
  {
     FTM_PWM_Duty(FTM0,FTM_CH1,0);
     FTM_PWM_Duty(FTM0,FTM_CH3,(int)(-motor2_out*10000));
  }
}

uint8 my_img_edge[];
uint8 center[60]={0};
uint8 center_w=0;
void my_get_edge()
{
  static int16 i=0,j=0,last=0,x=0,y=0,n=0;
  uint8 temp=0,find=0;
  int16 my_i=0,my_j=0;
  uint8 line_i;
  int16 sum_line_w=0;
  int16 sum_w=0;
  
  cont=0;
  
  //  clear Edge Img
  for(my_i=0;my_i<4800;my_i++)
  {
    my_img_edge[my_i]=0;
  }
   
  for(i=0;i<60;i++)//��������
  {
    sum_line_w=0;
    sum_w=0;
    for(j=0;j<80;j++)//ÿ����80
    {
      sum_line_w+=j*img[i*80+j];
      sum_w+=img[i*80+j];
    }
    center[i]=sum_line_w/sum_w;
    my_img_edge[80*i+center[i]]=1;
  }
}

void Cam_Algorithm(int16 * p_servo_duty)
{
  static float last_err=0;
  float road_center=0;
  float center_err=0;
  
  int i=0;
  
  for(i=30;i<60;i++)//����20��~59�еĵ�·���߾�ֵ
  {
    road_center+=center[i];
  }
  road_center=road_center/30;
  //  �� 0===========***===80 ��
  center_err=road_center-40.0;
  //Simple PID
  *p_servo_duty=(int)(Servo_Kp*center_err+Servo_Kd*(center_err-last_err));
  last_err=center_err;
  
}

void controller()
{
 pid_p=1; pid_i=0; pid_d=0;
 PID_TURN=pid_p*Middle_Err+pid_i*sum_err+pid_d*(Middle_Err-Previous_Error[1]); 
}

//��������������
//-20~20  
void Servo_Out(int16 servo_duty) 
{
  //Restrict the servo Range
  if (servo_duty>=servo_range)  //��������˷�Χ ռ�ձ�������50%�ڣ���ֹ�������
    { 
      servo_duty=servo_range;
    }
  if(servo_duty<=-servo_range)
    {
      servo_duty=-servo_range;
    }
  // Output
   FTM_PWM_Duty(FTM1,FTM_CH0,servo_middle+servo_duty);    //�������������ڶ���е㸽�����Ұڶ� 
  
}

void  main(void)
{
   OLED_Init();
   OLED_Draw_Logo();
   DELAY_MS(2000);
   OLED_CLS();
  
   uart_init(UART0,1500000);                //ע�⣡����ʵ�ʵĲ��������޸ģ�
   camera_init();
 
   set_vector_handler(PORTC_VECTORn ,PORTC_IRQHandler);    
   set_vector_handler(DMA0_VECTORn ,DMA0_IRQHandler);    
   
   set_vector_handler(UART0_RX_TX_VECTORn,UART0_RX_IRQHandler);  
   uart_rx_irq_en(UART0);
   NVIC_SetPriority(UART0_RX_TX_IRQn,0);
   
   
   led_init(); //������
   FTM_PWM_init(FTM0,FTM_CH0,10*1000,0);          // ��� PWM  PTC1�����Ƶ��Ϊ10Khz��
   FTM_PWM_init(FTM0,FTM_CH1,10*1000,0);          // ��� PWM  PTC2�����Ƶ��Ϊ10Khz��
   FTM_PWM_init(FTM0,FTM_CH2,10*1000,0);          // ��� PWM  PTC3�����Ƶ��Ϊ10Khz��
   FTM_PWM_init(FTM0,FTM_CH3,10*1000,0);          // ��� PWM  PTC4�����Ƶ��Ϊ10Khz��
   
   FTM_PWM_init(FTM1,FTM_CH0,100,sever_middle);   //��� PWM  PTA12�����Ƶ��Ϊ100hz,����Ϊ10ms
    
   while(1)
    {
      motor1_out=-duty*0.0035; //ת��Ϊʵ��ռ�ձ�
      motor2_out=duty*0;  
      Motor_Out();
      
      if(new_img)
      {
       new_img=0;
       //get_edge();
       //Search();
       
       
       img_extract(img,imgbuff_process,CAMERA_SIZE); 
       my_get_edge();
       Cam_Algorithm(&servo_duty);
       Servo_Out(servo_duty);
      
       
       Send_Begin();
       Send_Variable();
       sendimg();                  //���͵���λ��= 
       
       img_extract(img,imgbuff_process,CAMERA_SIZE);
       OLED_Draw_camera();
  
      }      
    }
    
}



