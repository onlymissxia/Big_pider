// C Code for driving Adafruit Mini PanTilt kit (http://www.adafruit.com/product/1967
// via a pca9685 board from a Raspberry Pi B
// Servos connected to channels 0(Pan) and 1(Tilt) on pca9685
//
// Original Python version https://github.com/pimoroni/PanTiltFacetracker/
// Dependent on wiringPi for accessing I2C bus
//
// Compile using:
// gcc -Wall -lwiringPi -o pca9685 pca9685.c
#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include <math.h>
#include <time.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <wiringSerial.h>
#include <string.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <unistd.h>

#include <errno.h>
#include <netdb.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include "Bpider.h"
#include "pca9685.h"
extern  float Angle_count;
unsigned char fb,fc;
//sever importate data
int sockfd;

/*
 * 函数名：I_UWB_LPS_Tag_DateFrame0_Unpack
 * 描述  ：
 * 输入  ：无
 * 输出  ：无
 */
uint8_t I_UWB_LPS_Tag_DateFrame0_Unpack(unsigned char uwbdata)
{
	static unsigned char uwbRxBuffer[I_UWB_MAX];
	static unsigned char uwbRxCnt=0;
	uwbRxBuffer[uwbRxCnt++]=uwbdata;
	if(uwbRxBuffer[0] == 0x55)
	{
		if(uwbRxCnt<I_UWB_MAX)
		{
			return 0;
		}
		else
		{	
			uwb_tag_id = uwbRxBuffer[2];
			uwb_tag_position_x = (float)Byte32(sint32 ,uwbRxBuffer[6],uwbRxBuffer[5],uwbRxBuffer[4], 0) / 256000.0f;
			uwb_tag_position_y = (float)Byte32(sint32 ,uwbRxBuffer[9],uwbRxBuffer[8],uwbRxBuffer[7], 0) / 256000.0f;
			uwb_tag_position_z = (float)Byte32(sint32 ,uwbRxBuffer[12],uwbRxBuffer[11],uwbRxBuffer[10], 0) / 256000.0f;
			flag_uwb = 1;
			return 1;
			
		}
	}
	else
	{
		uwbRxCnt=0;
		return 0;
	}

	return 0;
}

void CopeSerialData(unsigned char ucData)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;
	
	ucRxBuffer[ucRxCnt++]=ucData;
	if (ucRxBuffer[0]!=0x55)
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {return;}
	else
	{
		switch(ucRxBuffer[1])
		{
			//case 0x50: 	memcpy(&stcTime,&ucRxBuffer[2],8);break;
			//case 0x51:	memcpy(&stcAcc,&ucRxBuffer[2],8);break;
			//case 0x52:	memcpy(&stcGyro,&ucRxBuffer[2],8);break;
			case 0x53:	memcpy(&stcAngle,&ucRxBuffer[2],8);break;
			case 0x54:	memcpy(&stcMag,&ucRxBuffer[2],8);break;
			//case 0x55:	memcpy(&stcDStatus,&ucRxBuffer[2],8);break;
			//case 0x56:	memcpy(&stcPress,&ucRxBuffer[2],8);break;
			//case 0x57:	memcpy(&stcLonLat,&ucRxBuffer[2],8);break;
			//case 0x58:	memcpy(&stcGPSV,&ucRxBuffer[2],8);break;
		}
		ucRxCnt=0;
		flag_jy901=1;
	}
}

//数据打包发送
void send_data(void)
{	
	if (stable_flag==1 && flag_uwb==1)
	{	
		flag_uwb=0;
		stable_flag=0;//发送完毕
		char data_all[100];
	sprintf(data_all,"number:%duwb:%.3f %.3f %.3fangle:%.3f",uwb_tag_id,uwb_tag_position_x,uwb_tag_position_y,uwb_tag_position_z,Angle_count);
	send(sockfd, data_all, strlen(data_all), 0);
	}
}
//计算完成之后再打包发送
//1发现出现一次错误数据电机
void stable_data(void)
{	
	if(flag_jy901==1)
	{
	flag_jy901=0;stable_flag=1;//计算完毕
	Angle_count=abs((float)stcAngle.Angle[2]/32768*180);
	//printf("%f\n", Angle_count);	
	}
}
/*串口开启中断*/
uint8_t interruput(void)
{
	int buffer_count_one,buffer_count_two;
	buffer_count_one = serialDataAvail(fb);
//	printf("%d\n",buffer_count);
	if(buffer_count_one > 0)
	{
		CopeSerialData(serialGetchar(fb));
		buffer_count_one = 0;
		//serialFlush(fb);
	}
	buffer_count_two = serialDataAvail(fc);
//printf("%d",buffer_count_two);
	if(buffer_count_two > 0)
	{
		buffer_count_two=0;
		CopeSerialData(serialGetchar(fc));
		//serialFlush(fc);
	}
}

void serial_check(void)
{

	if((fc = serialOpen("/dev/ttyUSB0",115200))>0)
	{
		printf("com1 success\n");
	}
	else
	{
		printf("com1 erorr");
	}
	if((fb = serialOpen("/dev/ttyAMA0",115200))>0)
	{
		printf("come2 success\n");
	}
	else
	{
		printf("com2 error\n" );
	}
}
void WiringPi_check(void)
{
	if(wiringPiSetup()==-1)
	{
	printf("wiringPi error\n");
	}
	else 
	printf("wiringPi ok\n");
}
void Set_Client(void)
{
    struct sockaddr_in their_addr;
    while((sockfd = socket(AF_INET,SOCK_STREAM,0)) == -1);
    printf("We get the sockfd~\n");
    their_addr.sin_family = AF_INET;
    their_addr.sin_port = htons(8000);
    their_addr.sin_addr.s_addr=inet_addr("192.168.1.39");
    bzero(&(their_addr.sin_zero), 8);
    
    while(connect(sockfd,(struct sockaddr*)&their_addr,sizeof(struct sockaddr)) == -1);
    printf("Get the Server~Cheers!\n");
    /*
    numbytes = recv(sockfd, buf, BUFSIZ,0);//接收服务器端信息  
    buf[numbytes]='\0';  
    printf("%s",buf);
    sprintf(buf,"welcome to your Bpider");
    numbytes = send(sockfd, buf, strlen(buf), 0);
	*/
 /*  
  while(1)
    {
        numbytes=recv(sockfd,buf,BUFSIZ,0);    
    }
    close(sockfd);
*/
}
//循环查询server数据并进行电机的控制
void Ask_motor(void)
{
 	numbytes=recv(sockfd,buf,BUFSIZ,0);   
    if(numbytes==0)
        {
        	printf("RECV-ERROR\n");
        	Set_Client();
        } 
    else
    	{
	    switch(buf[0])
	        {
	        	case kMotor_U:Motor_U(buf[1],buf[2]);break;
	        	case kMotor_D:Motor_D(buf[1],buf[2]);break;
	        	case kMotor_L:Motor_L(buf[1],buf[2]);break;
	        	case kMotor_R:Motor_R(buf[1],buf[2]);break;
	        	case kMotor_S:S_motor();break;
	        	default:printf("Unknown command%s\n", buf);
	        }
        	memset(buf, 0, sizeof(buf));
    	}
    			usleep(1000);//1000us
}
/*
***********************************************************************
***********************解析-计算-发送***********************************
***********************************************************************
*/
PI_THREAD (func)
{
	while(1)
	{
		interruput();
		stable_data();
		//usleep(1000);//1000us
		//send_data();	
 	}
}
/*
***********************************************************************
***********************初始化-连接TCP-循环等待***************************
***********************************************************************
*/
int main()
{
	
	WiringPi_check();
	serial_check();
	Motor_GPIO_init();
	pca9685_init();

//多线性开起
	piThreadCreate(func);
//连接外部客户端
	Set_Client();
	while(1)
	{
		Ask_motor();
    }
    return 0;
}
