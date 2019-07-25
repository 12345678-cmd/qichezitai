#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "mpu6050.h"
#include "math.h"
#include "inv_mpu.h"

#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "lcd.h"
#include "key.h"
#include "usmart.h" 
#include "malloc.h"  
#include "MMC_SD.h" 
#include "ff.h"  
#include "exfuns.h"
#include "rtc.h" 

#define pi 3.1415

//第四小组姿态检测程序4.6版本
//串口通信   
//华师几米联合实验室


/****************SD卡定义的变量***************************/

u8 tbuf[80];
u8 bb[8];

u32 lo=0;
float h=0;		  

const char a1[8]="俯仰角";	
const char a2[8]="横滚角";
const char a3[8]="航向角";	


const u8 newline[]={0X0D,0X0A};





/*************加速度定义的变量***************************/
static float Q1=1e-6;
static float R1=2e-5;
float K1=0;
float X1=0;
float P1=0;
int cout1=0;             		//用于加速度
int axnum=0,aznum=0;
static int downnum=5;       //减速的计数值
static int upnum=5;       	//加速的计数值
static int wuchanum=2;    	//加速度误差判断值
static int wuchacount=0;  	//加速度误差计数值
int downcount=0;            //减速判断的计数值
int upcount=0;            	//加速判断的计数值
int Xdowncount=0;           //X轴方向上减速的次数
int Xupcount=0;          		//X轴方向上加速的次数
float LenX,axsum,average,azsum,LenZ,ave;
static float Downmin=-0.08; //自定义减速的阈值
static float Upmax=0.05;		//自定义加速的阈值
float offset,offset2;				//自定义的误差偏置
float Xxpast1,Xypast1,Xzpast1;
float Pxpast1,Pypast1,Pzpast1;
float S=4.0/65536; 					//加速度的灵敏度

/*************角速度定义的变量*****************************/
static float R2=2e-5,R3=2e-5;
static float Q2=1e-6,Q3=1e-6;
float K2=0,K3=0;
float X2=0;
float P2=0;
int cout2=0,cout3=0;             
float Xxpast2,Xypast2,Xzpast2;
float Pxpast2,Pypast2,Pzpast2;

float Xxpast3,Xypast3,Xzpast3;
float Pxpast3,Pypast3,Pzpast3;
float anglex,angley,anglez;

/****************转弯判断变量****************/
int right=0;
int left=0;
int t=0;
int yaw_cnt;
int left_cnt=0;
int stright_cnt=0;
int right_cnt=0;
float yaw_past[20];
float judge;

/*************输出xy坐标定义的变量************/
float t_xy=0.2;
float v_now=0;
float xx=0,yy=0,x=0,y=0;
float angle_xy;


/********************************SD卡读写函数*********************************/
void write_sd(float aacx,float ax,float yaw)
{	  		
	if(t==0)
	{
		mf_open(tbuf,0x02);//配置文件为写入模式
		delay_ms(5);	
		mf_lseek(lo);//将写入位置移至已写入数据末端
	}
			
	/*****************写入数据*****************/
	sprintf((TCHAR*)bb,"%f",aacx);	
	mf_write((const TCHAR *)bb,8);//写入X轴加速度原始数据
	mf_write((uint8_t*)newline,2);

	sprintf((TCHAR*)bb,"%f",ax);	
	mf_write((const TCHAR *)bb,8);//写入X轴加速度经过平均处理后的数据
	mf_write((uint8_t*)newline,2);
	
	sprintf((TCHAR*)bb,"%f",Xxpast1);	
	mf_write((const TCHAR *)bb,8);//写入X轴加速度经过平均、卡尔曼滤波处理后的数据
	mf_write((uint8_t*)newline,2);
	
	sprintf((TCHAR*)bb,"%f",Xypast1);	
	mf_write((const TCHAR *)bb,8);//写入Y轴仅经过卡尔曼滤波处理后的数据
	mf_write((uint8_t*)newline,2);
	
	sprintf((TCHAR*)bb,"%f",yaw);	
	mf_write((const TCHAR *)bb,8);//写入调用DMP库得到的航向角
	mf_write((uint8_t*)newline,2);

	sprintf((TCHAR*)bb,"%f",x);	
	mf_write((const TCHAR *)bb,8);//写入坐标x
	mf_write((uint8_t*)newline,2);
						
	sprintf((TCHAR*)bb,"%f",y);	
	mf_write((const TCHAR *)bb,8);//写入坐标y
	mf_write((uint8_t*)newline,2);

	sprintf((TCHAR*)bb,"%d",Xupcount);	
	mf_write((const TCHAR *)bb,8);//写入加速次数
	mf_write((uint8_t*)newline,2);
		
	sprintf((TCHAR*)bb,"%d",Xdowncount);	
	mf_write((const TCHAR *)bb,8);//写入减速次数
	mf_write((uint8_t*)newline,2);

	sprintf((TCHAR*)bb,"%d",left);	
	mf_write((const TCHAR *)bb,8);//写入左转次数
	mf_write((uint8_t*)newline,2);
	
	sprintf((TCHAR*)bb,"%d",right);	
	mf_write((const TCHAR *)bb,8);//写入右转次数
	mf_write((uint8_t*)newline,2);
	
	mf_write((uint8_t*)newline,2);
	
	
	if(t==5)
	{
		mf_tell();//获取已写入的数据的末端位置
		lo = mf_tell();
		printf("\r\n当前已写入的数据的末端位置 = %d \r\n",lo);
		mf_close();//每次写入完毕，关闭文件，否则数据将丢失
		t=0;
	}
	else t++;
//		printf("已创建%04d年%02d月%02d日%02d时%02d分数据\n",calendar.w_year,calendar.w_month,calendar.w_date,calendar.hour,calendar.min);
//		printf("现在的时间是：%d年%d月%d日%d时%d分%d秒\n\n",calendar.w_year,calendar.w_month,calendar.w_date,calendar.hour,calendar.min,calendar.sec);
}


/****************************加速度卡尔曼滤波函数*****************************/
void Accelerkalman(float ax,float ay, float az) 
{
	if(cout1==0)
	{
		Xxpast1=ax;
		Pxpast1=ax;
		 
		Xypast1=ay;
		Pypast1=ay;
		 
		Xzpast1=az;
		Pzpast1=az;		 
		 
		cout1++;
	}
	
	K1=Pxpast1/(Pxpast1+R1);
	Xxpast1=Xxpast1+K1*(ax-Xxpast1);
	Pxpast1=Pxpast1-K1*Pxpast1+Q1;
	 
	K1=Pypast1/(Pypast1+R1);
	Xypast1=Xypast1+K1*(ay-Xypast1);
	Pypast1=Pypast1-K1*Pypast1+Q1;
	 
	K1=Pzpast1/(Pzpast1+R1);
	Xzpast1=Xzpast1+K1*(az-Xzpast1);
	Pzpast1=Pzpast1-K1*Pzpast1+Q1;
	 
	printf("滤波后x轴加速度为 %f\r\n",Xxpast1);
	printf("滤波后y轴加速度为 %f\r\n",Xypast1);
	printf("滤波后z轴加速度为 %f\r\n",Xzpast1);
	 
 }


/****************************角速度卡尔曼滤波函数*****************************/
void Angle_Vkalman(float gx,float gy, float gz) 
{
	if(cout2==0)
	{
		Xxpast2=gx;
		Pxpast2=gx;
		 
		Xypast2=gy;
		Pypast2=gy;
		 
		Xzpast2=gz;
		Pzpast2=gz;		 
		 
		cout2++;
	}
	K2=Pxpast2/(Pxpast2+R2);
	Xxpast2=Xxpast2+K2*(gx-Xxpast2);
	Pxpast2=Pxpast2-K2*Pxpast2+Q2;
	 
	K2=Pypast2/(Pypast2+R2);
	Xypast2=Xypast2+K2*(gy-Xypast2);
	Pypast2=Pypast2-K2*Pypast2+Q2;
	 
	K2=Pzpast2/(Pzpast2+R2);
	Xzpast2=Xzpast2+K2*(gz-Xzpast2);
	Pzpast2=Pzpast2-K2*Pzpast2+Q2;
	 
	printf("滤波后x轴角速度为 %f\r\n",Xxpast2);
	printf("滤波后y轴角速度为 %f\r\n",Xypast2);
	printf("滤波后z轴角速度为 %f\r\n",Xzpast2);

}	


/*****************************角度卡尔曼滤波函数******************************/
void angelkalman(float angel)
{
	if(cout3==0)
	{
		Xxpast3=angel;
		Pxpast3=angel; 
		cout1++;
	}
	 
	K3=Pxpast3/(Pxpast3+R3);
	Xxpast3=Xxpast3+K3*(angel-Xxpast3);
	Pxpast3=Pxpast3-K3*Pxpast3+Q3;
	 
}
 
 
/*********************根据加速度计算各轴与水平方向的夹角**********************/
void angle(float Xxpast1,float Xypast1,float Xzpast1)
{
	anglex=(180/pi)*atan(Xxpast1 / sqrt(Xypast1*Xypast1 + Xzpast1*Xzpast1));
	angley=(180/pi)*atan(Xypast1 / sqrt(Xxpast1*Xxpast1+Xzpast1*Xzpast1));
	anglez=(180/pi)*atan(Xzpast1 / sqrt(Xxpast1*Xxpast1 +Xypast1*Xypast1));
	printf("anglex %f\r\n",anglex);
	printf("angley %f\r\n",angley);
	printf("anglez %f\r\n",anglez);
}



/*********************************计算xy坐标**********************************/
void cal_xy(float yaw)
{
	if (abs(Xxpast1)<0.02) 
		Xxpast1=0;
	 
	v_now=v_now+Xxpast1*t_xy;//更新x轴上的速度

	angle_xy = yaw*pi/180;
  xx=(1/2*Xxpast1/(1000*S)*t_xy*t_xy)*cos(angle_xy);
  yy=(1/2*Xxpast1/(1000*S)*t_xy*t_xy)*sin(angle_xy);
//  xx=(v_now*t_xy+1/2*Xxpast1*t_xy*t_xy)*cos(angle_xy);
//  yy=(v_now*t_xy+1/2*Xxpast1*t_xy*t_xy)*sin(angle_xy);	

	
	if (angle_xy>0&&angle_xy<pi/2)
	{   
		xx=abs(xx);
		yy=abs(yy);
	}     
  else if ((angle_xy>pi/2)&&((angle_xy)<pi))
	{  
		yy=abs(yy);
		if (xx>0)
			xx=-xx;
	}
	else if( (angle_xy<-pi/2)&&(angle_xy>-pi))
	{
		if (xx>0)
			xx=-xx;
            
		if (yy>0)
			yy=-yy;
	}       
	else if ((angle_xy<0)&&(angle_xy>-pi/2))
  {
		xx=abs(xx);
		if (yy>0)
			yy=-yy;
	}          
    
	x=x+xx;
	y=y+yy;
}

/************************判断x轴的加速状态及其加速次数************************/
void AccelertedVelocityJudge(float Xxpast1)
{
	int Axy=0;    //XY轴上的加速度方向
	if(axnum<=5)
	{
		axsum+=Xxpast1;
		if(axnum==5)
		{
			average=axsum/6;
			offset=0-average;
		}
		axnum++;
		Axy=0;
		printf("X轴方向上的加速度方向为 %d\r\n",Axy);
		}
	else
	{
		LenX=Xxpast1+offset;
		if(LenX<Downmin)
		{
			downcount++;
			if(downcount>=downnum)
			{
//					if(Xxpast1>=0)
//					{
//					 Axy=1;
//					}
//					else			 
//					{
//					 Axy=-1;
//					}
				Axy=-1;
				printf("X轴方向上的加速度方向为 %d\r\n",Axy);
			}
			else
			{
				Axy=0;
				printf("X轴方向上的加速度方向为 %d\r\n",Axy);					
			}
		}
		else
		{
			downcount=0;
			if(LenX>Upmax)
			{
				upcount++;
				if(upcount>=upnum)
				{
					Axy=1;
					printf("X轴方向上的加速度方向为 %d\r\n",Axy);
				}
				else
				{
					Axy=0;
					printf("X轴方向上的加速度方向为 %d\r\n",Axy);					
				}
			}
			else
			{
				if(Axy==1)	
				{
					wuchacount++;
					if(wuchacount>=wuchanum)
					{
						upcount=0;
						Axy=0;
						printf("X轴方向上的加速度方向为 %d\r\n",Axy);	
					}
					else
					{
						Axy=1;
						printf("X轴方向上的加速度方向为 %d\r\n",Axy); 
					}
				}
				else
				{
					upcount=0;
					Axy=0;
					printf("X轴方向上的加速度方向为 %d\r\n",Axy);	
				}			 
			}
		}
	}
	if(downcount==downnum)
	{	 
		Xdowncount++;
	}
	if(upcount==upnum)
	{
		Xupcount++;
	}
	printf("X轴方向上的加速次数为 %d\r\n",Xupcount);
	printf("X轴方向上的减速次数为 %d\r\n",Xdowncount);
}


/********************************转弯判断函数*********************************/
void Yaw_Judge(float yaw)
{	
	if(yaw_cnt<20)
	{
		yaw_past[yaw_cnt]=yaw;
	}
	else
	{
		judge=yaw-yaw_past[(yaw_cnt%20)];
		if(judge>15)
		{
			left_cnt=left_cnt+1;
			if(left_cnt>5)
			{
				stright_cnt=0;
				right_cnt=0;
			}
			if(left_cnt>20)
			{
				left_cnt=0;
				left=left+1;
			}
		}
		else if(judge<-15)
		{
			right_cnt=right_cnt+1;
			if(right_cnt>5)
			{
				stright_cnt=0;
				left_cnt=0;
			}
			if(right_cnt>20)
			{
				right_cnt=0;
				right=right+1;
			}
		}
		else
		{
			stright_cnt=stright_cnt+1;
			if(stright_cnt>5)
			{
				left_cnt=0;
				right_cnt=0;
			}
		}
		yaw_past[(yaw_cnt%20)]=yaw;
	}
}


/***********************************主函数************************************/
int main(void)
{	
/*******************************SD卡定义的变量********************************/	  
	u32 total,free;
	
/******************************温度计定义的变量*******************************/	 
  float old_temp,new_temp;//温度原始数据和转换后的数据
	
/******************************加速度定义的变量*******************************/
  u16 times=0; 	
  short aacx,aacy,aacz;		//加速度传感器原始数据
  float ax,ay,az;    //转换后的数据
  float ax1,ay1,az1;
	float sumx=0,sumy=0,sumz=0; 
	int count_num=0;
	float maxx,maxy,maxz,minx,miny,minz;
	float yaw_sum;
 
/*****************************角速度定义的变量********************************/ 
  short ggx,ggy,ggz;      //角速度原始数据
  float GS=4000.0/65536;  //陀螺仪的灵敏度
  float gx,gy,gz; 	
	float pitch,roll,yaw=0;  //定义俯仰角、横滚角、航向角
	
	
/**********************************初始化*************************************/
	delay_init();	    	 //延时函数初始化	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// 设置中断优先级分组2
	uart_init(115200);	 //串口初始化为115200
	MPU_Init();					//初始化MPU6050，采样频率50HZ		
	mpu_dmp_init();
	
	
/*******************************SD卡初始化************************************/	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断优先级分组2 
	delay_init();	    		//延时函数初始化	  
	uart_init(115200);	 	//串口初始化为115200
 	exfuns_init();				//为fatfs相关变量申请内存				 												    
	usmart_dev.init(72);	
 	mem_init();						//初始化内存池	
	while(RTC_Init())			//RTC初始化	，一定要初始化成功
	{ 
		printf("RTC ERROR!   ");	
		delay_ms(800);
		printf("RTC Trying...");	
	}	
	

	while(SD_Initialize())					//检测SD卡
	{
		printf("SD Card Error!");
		delay_ms(2000);
	}								   	
 	exfuns_init();												//为fatfs相关变量申请内存				 
	f_mount(fs[0],"0:",1); 								//挂载SD卡 
 	f_mount(fs[1],"1:",1); 								//挂载FLASH.	  
	while(exf_getfree("0",&total,&free))	//得到SD卡的总容量和剩余容量
	{
		printf("Fatfs Error!");
		delay_ms(2000);
	}			 
	

/*********************************SD卡创建文件********************************/
	sprintf((TCHAR*)tbuf,"0:/%04d年%02d月%02d日%02d时%02d分%02d秒数据.txt",calendar.w_year,calendar.w_month,calendar.w_date,calendar.hour,calendar.min,calendar.sec);

	mf_open(tbuf,0x04);//创建一个新的文件，若文件存在则函数失败
	delay_ms(10);
	mf_close();

	
/***************************************************/	
//		mf_open(tbuf,0x02);//配置文件为写入模式
	
	while(1)
	{
		if(USART_RX_STA&0x8000)
		{					   
			times++;	
		}
		else
		{
			times++;
			
			if(times%2==0)
			{
        
/***************************获取外界温度********************************/
//				old_temp=MPU_Get_Temperature();//获取温度原始数据
//				new_temp=36.53+old_temp/340;   //转换后得到温度值(单位为℃)
//				printf("\r\n"); 
//				printf("外界温度 %f\r\n",new_temp);

				
/*******************获取三轴加速度、三轴角速度原始数据************************/
				MPU_Get_Accelerometer(&aacx,&aacy,&aacz);//获取加速度原始数据 
				mpu_dmp_get_data(&pitch,&roll,&yaw);	

/***************************加速度数据转换处理********************************/
				if(count_num==0)
				{
					maxx=aacx*S;
					minx=aacx*S;
				
					maxy=aacy*S;
					miny=aacy*S;				
				
					maxz=aacz*S;
					minz=aacz*S;	

					sumx+=aacx*S;
					sumy+=aacy*S;
					sumz+=aacz*S;	

					yaw_sum+=yaw;	
					
					count_num +=1;							
				}						
				else if(count_num<10)
				{					
					count_num +=1;  
					
					ax1=aacx*S;
					ay1=aacy*S;
					az1=aacz*S;
				
					if(ax1>maxx)
					{
						maxx=ax1;
					}				
					if(ax1<minx)
					{
						minx=ax1;
					}
					
					if(ay1>maxy)
					{
						maxy=ay1;
					}				
					if(ay1<miny)
					{
						miny=ay1;
					}	

					if(az1>maxz)
					{
						maxz=az1;
					}				
					if(az1<minz)
					{
						minz=az1;
					}	
				
					sumx+=ax1;
					sumy+=ay1;
					sumz+=az1;
				
					yaw_sum+=yaw;
				}
				if(count_num==10)
				{
					ax=(sumx-maxx-minx)/8;
					ay=(sumy-maxy-miny)/8;
					az=(sumz-maxz-minz)/8;
					
					Accelerkalman(ax,ay,az);   //调用卡尔曼滤波函数					
					
					yaw=yaw_sum/10;
					count_num=0;
					sumx=0;
					sumy=0;
					sumz=0;
				
         	cal_xy(yaw);	     //计算xy坐标
		
				}	
				Yaw_Judge(yaw);		
				AccelertedVelocityJudge(Xxpast1);		
				write_sd(aacx,ax,yaw);		
			}	
//			delay_ms(10);   
		}
	}	 
}
