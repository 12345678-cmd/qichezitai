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

//����С����̬������4.6�汾
//����ͨ��   
//��ʦ��������ʵ����


/****************SD������ı���***************************/

u8 tbuf[80];
u8 bb[8];

u32 lo=0;
float h=0;		  

const char a1[8]="������";	
const char a2[8]="�����";
const char a3[8]="�����";	


const u8 newline[]={0X0D,0X0A};





/*************���ٶȶ���ı���***************************/
static float Q1=1e-6;
static float R1=2e-5;
float K1=0;
float X1=0;
float P1=0;
int cout1=0;             		//���ڼ��ٶ�
int axnum=0,aznum=0;
static int downnum=5;       //���ٵļ���ֵ
static int upnum=5;       	//���ٵļ���ֵ
static int wuchanum=2;    	//���ٶ�����ж�ֵ
static int wuchacount=0;  	//���ٶ�������ֵ
int downcount=0;            //�����жϵļ���ֵ
int upcount=0;            	//�����жϵļ���ֵ
int Xdowncount=0;           //X�᷽���ϼ��ٵĴ���
int Xupcount=0;          		//X�᷽���ϼ��ٵĴ���
float LenX,axsum,average,azsum,LenZ,ave;
static float Downmin=-0.08; //�Զ�����ٵ���ֵ
static float Upmax=0.05;		//�Զ�����ٵ���ֵ
float offset,offset2;				//�Զ�������ƫ��
float Xxpast1,Xypast1,Xzpast1;
float Pxpast1,Pypast1,Pzpast1;
float S=4.0/65536; 					//���ٶȵ�������

/*************���ٶȶ���ı���*****************************/
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

/****************ת���жϱ���****************/
int right=0;
int left=0;
int t=0;
int yaw_cnt;
int left_cnt=0;
int stright_cnt=0;
int right_cnt=0;
float yaw_past[20];
float judge;

/*************���xy���궨��ı���************/
float t_xy=0.2;
float v_now=0;
float xx=0,yy=0,x=0,y=0;
float angle_xy;


/********************************SD����д����*********************************/
void write_sd(float aacx,float ax,float yaw)
{	  		
	if(t==0)
	{
		mf_open(tbuf,0x02);//�����ļ�Ϊд��ģʽ
		delay_ms(5);	
		mf_lseek(lo);//��д��λ��������д������ĩ��
	}
			
	/*****************д������*****************/
	sprintf((TCHAR*)bb,"%f",aacx);	
	mf_write((const TCHAR *)bb,8);//д��X����ٶ�ԭʼ����
	mf_write((uint8_t*)newline,2);

	sprintf((TCHAR*)bb,"%f",ax);	
	mf_write((const TCHAR *)bb,8);//д��X����ٶȾ���ƽ������������
	mf_write((uint8_t*)newline,2);
	
	sprintf((TCHAR*)bb,"%f",Xxpast1);	
	mf_write((const TCHAR *)bb,8);//д��X����ٶȾ���ƽ�����������˲�����������
	mf_write((uint8_t*)newline,2);
	
	sprintf((TCHAR*)bb,"%f",Xypast1);	
	mf_write((const TCHAR *)bb,8);//д��Y��������������˲�����������
	mf_write((uint8_t*)newline,2);
	
	sprintf((TCHAR*)bb,"%f",yaw);	
	mf_write((const TCHAR *)bb,8);//д�����DMP��õ��ĺ����
	mf_write((uint8_t*)newline,2);

	sprintf((TCHAR*)bb,"%f",x);	
	mf_write((const TCHAR *)bb,8);//д������x
	mf_write((uint8_t*)newline,2);
						
	sprintf((TCHAR*)bb,"%f",y);	
	mf_write((const TCHAR *)bb,8);//д������y
	mf_write((uint8_t*)newline,2);

	sprintf((TCHAR*)bb,"%d",Xupcount);	
	mf_write((const TCHAR *)bb,8);//д����ٴ���
	mf_write((uint8_t*)newline,2);
		
	sprintf((TCHAR*)bb,"%d",Xdowncount);	
	mf_write((const TCHAR *)bb,8);//д����ٴ���
	mf_write((uint8_t*)newline,2);

	sprintf((TCHAR*)bb,"%d",left);	
	mf_write((const TCHAR *)bb,8);//д����ת����
	mf_write((uint8_t*)newline,2);
	
	sprintf((TCHAR*)bb,"%d",right);	
	mf_write((const TCHAR *)bb,8);//д����ת����
	mf_write((uint8_t*)newline,2);
	
	mf_write((uint8_t*)newline,2);
	
	
	if(t==5)
	{
		mf_tell();//��ȡ��д������ݵ�ĩ��λ��
		lo = mf_tell();
		printf("\r\n��ǰ��д������ݵ�ĩ��λ�� = %d \r\n",lo);
		mf_close();//ÿ��д����ϣ��ر��ļ����������ݽ���ʧ
		t=0;
	}
	else t++;
//		printf("�Ѵ���%04d��%02d��%02d��%02dʱ%02d������\n",calendar.w_year,calendar.w_month,calendar.w_date,calendar.hour,calendar.min);
//		printf("���ڵ�ʱ���ǣ�%d��%d��%d��%dʱ%d��%d��\n\n",calendar.w_year,calendar.w_month,calendar.w_date,calendar.hour,calendar.min,calendar.sec);
}


/****************************���ٶȿ������˲�����*****************************/
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
	 
	printf("�˲���x����ٶ�Ϊ %f\r\n",Xxpast1);
	printf("�˲���y����ٶ�Ϊ %f\r\n",Xypast1);
	printf("�˲���z����ٶ�Ϊ %f\r\n",Xzpast1);
	 
 }


/****************************���ٶȿ������˲�����*****************************/
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
	 
	printf("�˲���x����ٶ�Ϊ %f\r\n",Xxpast2);
	printf("�˲���y����ٶ�Ϊ %f\r\n",Xypast2);
	printf("�˲���z����ٶ�Ϊ %f\r\n",Xzpast2);

}	


/*****************************�Ƕȿ������˲�����******************************/
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
 
 
/*********************���ݼ��ٶȼ��������ˮƽ����ļн�**********************/
void angle(float Xxpast1,float Xypast1,float Xzpast1)
{
	anglex=(180/pi)*atan(Xxpast1 / sqrt(Xypast1*Xypast1 + Xzpast1*Xzpast1));
	angley=(180/pi)*atan(Xypast1 / sqrt(Xxpast1*Xxpast1+Xzpast1*Xzpast1));
	anglez=(180/pi)*atan(Xzpast1 / sqrt(Xxpast1*Xxpast1 +Xypast1*Xypast1));
	printf("anglex %f\r\n",anglex);
	printf("angley %f\r\n",angley);
	printf("anglez %f\r\n",anglez);
}



/*********************************����xy����**********************************/
void cal_xy(float yaw)
{
	if (abs(Xxpast1)<0.02) 
		Xxpast1=0;
	 
	v_now=v_now+Xxpast1*t_xy;//����x���ϵ��ٶ�

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

/************************�ж�x��ļ���״̬������ٴ���************************/
void AccelertedVelocityJudge(float Xxpast1)
{
	int Axy=0;    //XY���ϵļ��ٶȷ���
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
		printf("X�᷽���ϵļ��ٶȷ���Ϊ %d\r\n",Axy);
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
				printf("X�᷽���ϵļ��ٶȷ���Ϊ %d\r\n",Axy);
			}
			else
			{
				Axy=0;
				printf("X�᷽���ϵļ��ٶȷ���Ϊ %d\r\n",Axy);					
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
					printf("X�᷽���ϵļ��ٶȷ���Ϊ %d\r\n",Axy);
				}
				else
				{
					Axy=0;
					printf("X�᷽���ϵļ��ٶȷ���Ϊ %d\r\n",Axy);					
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
						printf("X�᷽���ϵļ��ٶȷ���Ϊ %d\r\n",Axy);	
					}
					else
					{
						Axy=1;
						printf("X�᷽���ϵļ��ٶȷ���Ϊ %d\r\n",Axy); 
					}
				}
				else
				{
					upcount=0;
					Axy=0;
					printf("X�᷽���ϵļ��ٶȷ���Ϊ %d\r\n",Axy);	
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
	printf("X�᷽���ϵļ��ٴ���Ϊ %d\r\n",Xupcount);
	printf("X�᷽���ϵļ��ٴ���Ϊ %d\r\n",Xdowncount);
}


/********************************ת���жϺ���*********************************/
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


/***********************************������************************************/
int main(void)
{	
/*******************************SD������ı���********************************/	  
	u32 total,free;
	
/******************************�¶ȼƶ���ı���*******************************/	 
  float old_temp,new_temp;//�¶�ԭʼ���ݺ�ת���������
	
/******************************���ٶȶ���ı���*******************************/
  u16 times=0; 	
  short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����
  float ax,ay,az;    //ת���������
  float ax1,ay1,az1;
	float sumx=0,sumy=0,sumz=0; 
	int count_num=0;
	float maxx,maxy,maxz,minx,miny,minz;
	float yaw_sum;
 
/*****************************���ٶȶ���ı���********************************/ 
  short ggx,ggy,ggz;      //���ٶ�ԭʼ����
  float GS=4000.0/65536;  //�����ǵ�������
  float gx,gy,gz; 	
	float pitch,roll,yaw=0;  //���帩���ǡ�����ǡ������
	
	
/**********************************��ʼ��*************************************/
	delay_init();	    	 //��ʱ������ʼ��	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// �����ж����ȼ�����2
	uart_init(115200);	 //���ڳ�ʼ��Ϊ115200
	MPU_Init();					//��ʼ��MPU6050������Ƶ��50HZ		
	mpu_dmp_init();
	
	
/*******************************SD����ʼ��************************************/	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�����ж����ȼ�����2 
	delay_init();	    		//��ʱ������ʼ��	  
	uart_init(115200);	 	//���ڳ�ʼ��Ϊ115200
 	exfuns_init();				//Ϊfatfs��ر��������ڴ�				 												    
	usmart_dev.init(72);	
 	mem_init();						//��ʼ���ڴ��	
	while(RTC_Init())			//RTC��ʼ��	��һ��Ҫ��ʼ���ɹ�
	{ 
		printf("RTC ERROR!   ");	
		delay_ms(800);
		printf("RTC Trying...");	
	}	
	

	while(SD_Initialize())					//���SD��
	{
		printf("SD Card Error!");
		delay_ms(2000);
	}								   	
 	exfuns_init();												//Ϊfatfs��ر��������ڴ�				 
	f_mount(fs[0],"0:",1); 								//����SD�� 
 	f_mount(fs[1],"1:",1); 								//����FLASH.	  
	while(exf_getfree("0",&total,&free))	//�õ�SD������������ʣ������
	{
		printf("Fatfs Error!");
		delay_ms(2000);
	}			 
	

/*********************************SD�������ļ�********************************/
	sprintf((TCHAR*)tbuf,"0:/%04d��%02d��%02d��%02dʱ%02d��%02d������.txt",calendar.w_year,calendar.w_month,calendar.w_date,calendar.hour,calendar.min,calendar.sec);

	mf_open(tbuf,0x04);//����һ���µ��ļ������ļ���������ʧ��
	delay_ms(10);
	mf_close();

	
/***************************************************/	
//		mf_open(tbuf,0x02);//�����ļ�Ϊд��ģʽ
	
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
        
/***************************��ȡ����¶�********************************/
//				old_temp=MPU_Get_Temperature();//��ȡ�¶�ԭʼ����
//				new_temp=36.53+old_temp/340;   //ת����õ��¶�ֵ(��λΪ��)
//				printf("\r\n"); 
//				printf("����¶� %f\r\n",new_temp);

				
/*******************��ȡ������ٶȡ�������ٶ�ԭʼ����************************/
				MPU_Get_Accelerometer(&aacx,&aacy,&aacz);//��ȡ���ٶ�ԭʼ���� 
				mpu_dmp_get_data(&pitch,&roll,&yaw);	

/***************************���ٶ�����ת������********************************/
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
					
					Accelerkalman(ax,ay,az);   //���ÿ������˲�����					
					
					yaw=yaw_sum/10;
					count_num=0;
					sumx=0;
					sumy=0;
					sumz=0;
				
         	cal_xy(yaw);	     //����xy����
		
				}	
				Yaw_Judge(yaw);		
				AccelertedVelocityJudge(Xxpast1);		
				write_sd(aacx,ax,yaw);		
			}	
//			delay_ms(10);   
		}
	}	 
}
