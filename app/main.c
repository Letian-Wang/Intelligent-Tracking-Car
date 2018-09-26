/*
 * @file       main.c
 * @brief      主函数
 */
#include "include.h"
#define Servo_Kp 2.5
#define Servo_Kd 0.1

/*************
本程序将演示板二值化摄像头的运用
**************/
uint16 edgposition[CAMERA_H];

float Variable[20]; //该数组可以存储变量并发送到上位机图像窗口

void sendimg();          //发送图像到上位机

float Distance;
uint8  RoadType=0;
float Delt_error,Middle_Err;
float Previous_Error[12];
float sum_err;

float character_distance;//赛道特征点的位置

uint8 Stop;


uint8   LMR[CAMERA_H][3];

uint8  road_type;
uint16 cont;

float motor1_out,motor2_out; 
int duty = 25;
//电机相关


int sever_middle=140;  //舵机摆臂回正的脉宽，需要根据实际情况修改，现在是(155/1000)*10ms=1.55ms 1000是脉冲精度 
int sever_range=25;    //限制一下舵机摆动的幅度，防止打死造成机械损坏（大约正负25度，根据实际情况修改）
int servo_middle=140;  //舵机摆臂回正的脉宽，需要根据实际情况修改，现在是(155/1000)*10ms=1.55ms 1000是脉冲精度 
int servo_range=25;    //限制一下舵机摆动的幅度，防止打死造成机械损坏（大约正负25度，根据实际情况修改）
int duty_servo=0;
//舵机相关
int16 servo_duty=0;

float pid_p,pid_i,pid_d,PID_TURN=0;
float controll;
//控制器相关

void my_putchar(char temp)
{
    uart_putchar(UART0,temp); //根据实际的串口号来修改
}

/*用来通知上位机新的一组数据开始，要保存数据必须发送它*/
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
 *  @brief      发送图像到上位机显示  
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
  my_putchar(0x1); //小车状态
  
  num=cont+2+180+36;  
  //统计将要传输的数据量 2是因为要传输关键字即0xf0和0xf2
  //180是边线的位 36是变量的位 如果不传输就不要加上！
  
  my_putchar(BYTE0(num)); 
  my_putchar(BYTE1(num));
 for(i=0;i< cont;i++)
 {
     my_putchar(img_edg[i]);
 }
 my_putchar(0xf0);  //代表图像数据发完了
 /******************星号围起来的可以不传输*******************/
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
 my_putchar(0xf2); //代表整个数据都发完了
 
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
   
float Slope_Calculate_Uint8(uint8 begin,uint8 end,uint8 *p)    //最小二乘法拟合斜率
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
  if((end-begin)*x2sum-xsum*xsum) //判断除数是否为零 
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

void fix_break_line() //修复断开的线
{
  
}

void Search()
{
  //从底部往上搜线
  float Middle_Err_Sum=0,slope;
  static int i,j,find;
  uint8 left_cont=0,right_cont=0;
  static float Middle_Temp=0;
  int leftfind=0,rightfind=0;
  int AllWhileStartLine=0,AllWhileEndLine=0;
  static int break_line_left=0,break_line_right=0,continue_line_left=0,continue_line_right=0;
  
  int search_end_line=0;
  for(i=0;i<60;i++)  //清空数组
  {  
        LMR[0][i]=0; //左边线数列
        LMR[1][i]=0;  //中线数列
        LMR[2][i]=80; //右边线数列
  }
  
  leftfind=0;
  rightfind=0;
  break_line_left=0;
  break_line_right=0;
  continue_line_left=0;
  continue_line_right=0;
  
  for(i=59;i>0;i--) //从第59行开始搜线
  {
    if(edgposition[i]==0&&(i!=0)) //全黑行 置为丢线  ???
    {
      break;
    }
    
    j=edgposition[i];//该行跳变沿开始的位置  j代表黑变白 即左边线 j+1代表变黑 即右边线 
    
    if(i==59)  //底部开始行
    {   
        while(img_edg[j]!=255)
       {  
        if((img_edg[j]<55)&&(img_edg[j+1]>25))  //左边沿小于55 右边沿大于25
        {
          if((img_edg[j+1]-img_edg[j])>25) //右边沿-左边沿大于20
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
    else   //不是底部开始行
    { 
        find=0;
        while(img_edg[j]!=255)
       {
         
        if((img_edg[j]<=LMR[2][i+1])&&(img_edg[j+1]>=LMR[0][i+1])&&(img_edg[j+1]-img_edg[j])>8)     //左边沿小于上一行的右边线 右边沿大于上一行的左边线是连通域；
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
        if(img_edg[j+1]==255) //该行的跳变沿结束了
        {
          if(img_edg[j]==0) //说明该行为全白 
          { 
            if(AllWhileStartLine==0)
            {
              AllWhileStartLine=i; //全白行开始的位置
            }
            AllWhileEndLine=i;

            if((rightfind&&leftfind&&(RoadType==0)&&i>20)||(AllWhileStartLine-AllWhileEndLine)>10)
              //且两边边线都找到过 排除顶部20行的干扰
            {
             RoadType=1; //进入十字了
             Middle_Temp=Middle_Err;
             character_distance=Distance;
            }
          }
          break;//while
        }
        j=j+2;
       }
       //以上是搜索连通域的算法
       
       
       
       if(RoadType==1)  //在十字内对搜线进行特殊处理
       { 
         if(left_cont>=4)
         {
           if(((LMR[0][i]<(LMR[0][i+1]-1))||(LMR[0][i]==0))&&i>20)
           {
                LMR[0][i]=0;
                if(break_line_left==0)break_line_left=i+1;         //往外扩展或者断线
           }
         }
         if(LMR[0][i]!=0) //找到了左边边线
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
                 if(break_line_right==0)break_line_right=i+1;       //往外扩展或者断线
           }
         }
         if(LMR[2][i]!=80) //找到了右边线
         {
           right_cont++;
         }
         else
         {
           right_cont=0;
         }
       }
       
       if(find==0)//没有找到连通区域
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
       for(i=continue_line_left-1;(i>continue_line_left-10)&&(i>0);i--) //续线行可能有问题 检测一下 
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
      
      for(i=break_line_left;i>=continue_line_left;i--) //开始补线
      { 
        slope=(LMR[0][break_line_left]- LMR[0][continue_line_left])*1.0/(break_line_left-continue_line_left);
        LMR[0][i]= LMR[0][break_line_left]-(int)((break_line_left-i)*slope);
      }
   }
   
   if(break_line_right!=0&&continue_line_right!=0)
   {
       for(i=continue_line_right;(i>continue_line_right-10)&&(i>0);i--) //续线行可能有问题 检测一下 
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
         for(i=break_line_right;i>=continue_line_right;i--) //开始补线
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


  Middle_Err= Middle_Err_Sum;   //中线误差
  Push_And_Pull(Previous_Error,10,Middle_Err);   //Previous_Error[12]前十个元素储存最近十个时刻的中线误差
  //Delt_error=-10*Slope_Calculate(0,10,Previous_Error);   //最近十个时刻的中线误差关于时间的斜率
 
 }  

void get_edge()   //尽量减少乘法运算
{
  
  static int16 i=0,j=0,last=0,x=0,y=0,n=0;
  uint8 temp=0,find=0;
  cont=0;  //跳变点编号
  for(i=0;i<60;i++)
  {
    last=0;  //初始化为黑
    x=i*10;
    find=0;
    edgposition[i]=0;
    edgposition[i]=0;
    for(j=0;j<10;j++)
    {
      if(imgbuff_process[x+j]==0xff)  //imgbuff_process 储存600个字节，每行10个字节
      {
        if(last==0)
        {
              y=j<<3;      //左移动3相当于乘以8
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
      
      for(n=7;n>=0;n--) //每个字节8位，从高到低
      {
            temp=(imgbuff_process[x+j]>>n)&1; // 获取该点像素值 （0或1）     
            if(temp!=last) //与上一个值不相同 出现了跳变沿            
            {
               y=j<<3;  
               if(find==0)
              {
                edgposition[i]=cont;
              }
               img_edg[cont++]=y+7-n;   
               find=1;
            }
              last=temp;                //存储该点的值
      } 
    } 
     img_edg[cont++]=0xff;   

  }
}

/*img_edg是一个一维数组 记录了摄像头每行的跳变沿的坐标值  每行跳变沿 由上升沿开始（由黑变白）然后接下降沿（由白变黑） 
   0xff用于指示该行的跳变沿完了，开始记录下一行
   如果每行图像以白色部分开始，那么该行跳变沿起始位置为0，因为每行初始值为黑，故会有一个跳变沿
   如果该行全黑 那么该行记录为0xff
   如果该行为全白 记录为 0 0xff
 
   
   0xff代表本行坐标值结束，进入下一行

   edgposition[i]代表第i行的 跳变沿 在 img_edg 中坐标起点

 */


void UART0_RX_IRQHandler()
{
  static uint8 recv=0,recievingdata=0,cnt=0;
  static uint8 predata[4];
  static uint8  data[100];
  uint8 update_variable;
  while(uart_query(UART0)==1)  
  {
    uart_getchar (UART0,(char*)(&recv));  //根据实际的串口来修改  
    if(recievingdata)
    {
      if(cnt>=56)
      {
        if(recv==2)//校验帧尾
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
 // uart_rx_irq_en(UART0);//使能串口接收中断，防止出错串口中断被关闭 
    
}

void Motor_Out() //电机控制输出函数
{
  
 motor1_out=LIMIT(motor1_out,0.99,-0.99);  //电机的输出值限制在-1~1 之间，-1代表反转最快，1代表正转最快；
 motor2_out=LIMIT(motor2_out,0.99,-0.99);
  //电机1
  if(motor1_out>=0) //占空比为正，正转
  {
     FTM_PWM_Duty(FTM0,FTM_CH0,(int)(motor1_out*10000));//占空比精度为10000 ，占空比*占空比精度
     FTM_PWM_Duty(FTM0,FTM_CH2,0);
  }
  else   //为负就反转
  {
     FTM_PWM_Duty(FTM0,FTM_CH0,0);
     FTM_PWM_Duty(FTM0,FTM_CH2,(int)(-motor1_out*10000));
  }
  
  //电机2
    if(motor2_out>=0) //占空比为正，正转
  {
     FTM_PWM_Duty(FTM0,FTM_CH1,(int)(motor2_out*10000)); 
     FTM_PWM_Duty(FTM0,FTM_CH3,0);
  }
  else   //反转
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
   
  for(i=0;i<60;i++)//逐行搜索
  {
    sum_line_w=0;
    sum_w=0;
    for(j=0;j<80;j++)//每行有80
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
  
  for(i=30;i<60;i++)//计算20行~59行的道路中线均值
  {
    road_center+=center[i];
  }
  road_center=road_center/30;
  //  左 0===========***===80 右
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

//舵机控制输出函数
//-20~20  
void Servo_Out(int16 servo_duty) 
{
  //Restrict the servo Range
  if (servo_duty>=servo_range)  //如果超出了范围 占空比限制在50%内，防止输出过大
    { 
      servo_duty=servo_range;
    }
  if(servo_duty<=-servo_range)
    {
      servo_duty=-servo_range;
    }
  // Output
   FTM_PWM_Duty(FTM1,FTM_CH0,servo_middle+servo_duty);    //舵机控制输出，在舵机中点附近左右摆动 
  
}

void  main(void)
{
   OLED_Init();
   OLED_Draw_Logo();
   DELAY_MS(2000);
   OLED_CLS();
  
   uart_init(UART0,1500000);                //注意！根据实际的波特率来修改！
   camera_init();
 
   set_vector_handler(PORTC_VECTORn ,PORTC_IRQHandler);    
   set_vector_handler(DMA0_VECTORn ,DMA0_IRQHandler);    
   
   set_vector_handler(UART0_RX_TX_VECTORn,UART0_RX_IRQHandler);  
   uart_rx_irq_en(UART0);
   NVIC_SetPriority(UART0_RX_TX_IRQn,0);
   
   
   led_init(); //电机相关
   FTM_PWM_init(FTM0,FTM_CH0,10*1000,0);          // 电机 PWM  PTC1输出，频率为10Khz、
   FTM_PWM_init(FTM0,FTM_CH1,10*1000,0);          // 电机 PWM  PTC2输出，频率为10Khz、
   FTM_PWM_init(FTM0,FTM_CH2,10*1000,0);          // 电机 PWM  PTC3输出，频率为10Khz、
   FTM_PWM_init(FTM0,FTM_CH3,10*1000,0);          // 电机 PWM  PTC4输出，频率为10Khz、
   
   FTM_PWM_init(FTM1,FTM_CH0,100,sever_middle);   //舵机 PWM  PTA12输出，频率为100hz,周期为10ms
    
   while(1)
    {
      motor1_out=-duty*0.0035; //转化为实际占空比
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
       sendimg();                  //发送到上位机= 
       
       img_extract(img,imgbuff_process,CAMERA_SIZE);
       OLED_Draw_camera();
  
      }      
    }
    
}



