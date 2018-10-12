#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include <math.h>
#include <time.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include "pca9685.h"
#include <wiringSerial.h>
#include <string.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <unistd.h>

float Angle_count;
/*说明：PWM使用的是
		0通道是前进电机
		1通道是旋转电机
*/
uint8_t setUpDevice(uint16_t fd)
{
	setAllPWM(fd,0,0);
	wiringPiI2CWriteReg8(fd,MODE2,OUTDRV);
	wiringPiI2CWriteReg8(fd,MODE1,ALLCALL);
	delay(0.005);
	uint8_t mode1=wiringPiI2CReadReg8(fd,MODE1);
	mode1=mode1 & SLEEP;
	wiringPiI2CWriteReg8(fd,MODE1,mode1);
	delay(0.005);
	setPWMFreq(fd,50);
	return 0;
}


uint8_t Forword(uint16_t fd, int16_t deg){
	//move(fd, PAN, convAxis(PAN,deg));
	setPWM(fd, PAN, 0, (int) 10*deg);
	return 0;
}


uint8_t Rotate(uint16_t fd, int16_t deg){
	//move(fd, TILT, convAxis(TILT,deg));
	setPWM(fd, TILT, 0, (int) 10*deg);
	return 0;
}


uint8_t move(uint16_t fd, uint8_t s, int16_t deg){
// Calculate pulse length
	float pwm = 570.0 + ((deg) * 1700.0);
	pwm = (4096.0/20000.0) * pwm;
	printf("deg:%d pwm:%f\n",deg,pwm);
	setPWM(fd, s, 0, (int) pwm);
	return 0;
}


// Shift North 0 deg/South 180 deg to anticlockwise servo 180 deg pan
// Shift Up 90 deg/Hori 0 deg to top down servo 145 deg tilt
// Includes offset and scale constants to correct pwm calculation for specific servos
int16_t convAxis(uint8_t s, int16_t deg){
	if (!s) {
		if ((deg>PANMAX) || (deg<PANMIN)){
			deg = (PANMAX-PANMIN)/2; //Out of Range then go to centre
		}
		deg = PANOFFSET+((270-deg)*PANSCALE);
	} else {
		if ((deg>TILTMAX) || (deg<TILTMIN)){
			deg = TILTOFFSET+((90-TILTMIN)*TILTSCALE); // OoR go to minimum
		}
		deg = TILTOFFSET+((90-deg)*TILTSCALE);
	}
	return deg;
}


uint8_t setPWMFreq(uint16_t fd, uint16_t freq){
	float prescaleval = CLOCKFREQ;
	prescaleval = prescaleval/4096.0;
	prescaleval = prescaleval/freq;
	prescaleval = prescaleval - 1.0;
	float prescale = (prescaleval + 0.5);
	uint8_t oldmode = wiringPiI2CReadReg8(fd,MODE1);
	uint8_t newmode = (oldmode & 0x7F) | 0x10;
	wiringPiI2CWriteReg8(fd,MODE1, newmode);
	wiringPiI2CWriteReg8(fd,PRESCALE, (int)prescale);
	wiringPiI2CWriteReg8(fd,MODE1, oldmode);
	delay(0.005);
	wiringPiI2CWriteReg8(fd,MODE1, oldmode | 0x80);
	return 0;
}

uint8_t setPWM(uint16_t fd, uint8_t channel, uint16_t on, uint16_t off){
	//printf("SetPWM:%d",off);
	wiringPiI2CWriteReg8(fd, LED0_ON_L+4*channel, on & 0xFF);
	wiringPiI2CWriteReg8(fd, LED0_ON_H+4*channel, on >> 8);
	wiringPiI2CWriteReg8(fd, LED0_OFF_L+4*channel, off & 0xFF);
	wiringPiI2CWriteReg8(fd, LED0_OFF_H+4*channel, off >> 8);
	return 0;
}


uint8_t setAllPWM(uint16_t fd, uint16_t on, uint16_t off){
	wiringPiI2CWriteReg8(fd, ALL_LED_ON_L, on & 0xFF);
	wiringPiI2CWriteReg8(fd, ALL_LED_ON_H, on >> 8);
	wiringPiI2CWriteReg8(fd, ALL_LED_OFF_L, off & 0xFF);
	wiringPiI2CWriteReg8(fd, ALL_LED_OFF_H, off >> 8);
	return 0;
}


uint8_t sleepOn(uint16_t fd){
	uint8_t oldmode = wiringPiI2CReadReg8(fd,MODE1);
	uint8_t newmode = (oldmode & 0x7F) | 0x10;
	wiringPiI2CWriteReg8(fd,MODE1, newmode);
	return 0;
}
uint8_t Motor_GPIO_init(void)
{
	pinMode(GPIO_PIN0, OUTPUT); // 设置引脚为输出
	pinMode(GPIO_PIN1, OUTPUT); // 设置引脚为输出
	pinMode(GPIO_PIN2, OUTPUT); // 设置引脚为输出
	pinMode(GPIO_PIN3, OUTPUT); // 设置引脚为输出
	printf("Motor is ok\n");
}
uint8_t pca9685_init(void)
{
//	wiringPiISR(io,INT_EDGE_BOTH,interruput);
	fd = wiringPiI2CSetup(PCA9685_ADDR);
	setUpDevice(fd);
	printf("pca9685 is ok\n");;
}
//前控制函数
void Motor_U(uint8_t time,uint8_t speed)
{
	if(time>0)
	{
	U_motor();
	Forword(fd,speed);
	sleep(time);
	S_motor();
	}
	else
	printf("未输入正确参数\n");
}
//后控制函数
void Motor_D(uint8_t time,uint8_t speed)
{
	if(time>0)
	{
	D_motor();
	Forword(fd,speed);
	sleep(time);
	S_motor();
	}
	else
	printf("未输入正确参数\n");
}
//左控制函数

void Motor_L(uint8_t count,uint8_t speed)
{
	uint8_t count_now;
	if(count>0)
	{	printf("L\n");
		count_now = abs(Angle_count);
  	while(abs(Angle_count-count_now)<count)
  	{
    	L_motor();
		Rotate(fd,speed);
  	}
  	//printf("%f\n", abs(Angle_count-count_now));
		S_motor();
	}
	else
	printf("未输入正确参数\n");
}
//右控制函数
void Motor_R(uint8_t count,uint8_t speed)
{
	uint8_t count_now=0;
	if(count>0)
	{	printf("R\n");
		count_now = abs(Angle_count);
  	while(abs(Angle_count-count_now)<count)
  	{
	R_motor();
	Rotate(fd,speed);
	}
	//Rotate_count(count);
	S_motor();
	}
	else
		printf("未输入正确参数\n");
}

void U_motor(void)
{
	digitalWrite(GPIO_PIN0, 1); // 设置输出高电平
	digitalWrite(GPIO_PIN1, 0);
}
void D_motor(void)
{
	digitalWrite(GPIO_PIN0, 0); // 设置输出高电平
	digitalWrite(GPIO_PIN1, 1);
}
void L_motor(void)
{
	digitalWrite(GPIO_PIN2, 1); // 设置输出高电平
	digitalWrite(GPIO_PIN3, 0);

}
void R_motor(void)
{
	digitalWrite(GPIO_PIN2, 0); // 设置输出高电平
	digitalWrite(GPIO_PIN3, 1);
}
void S_motor(void)
{
	digitalWrite(GPIO_PIN0, 0); // 设置输出高电平
	digitalWrite(GPIO_PIN1, 0);
	digitalWrite(GPIO_PIN2, 0); // 设置输出高电平
	digitalWrite(GPIO_PIN3, 0);
}

