#ifndef __PCA9685_H_
#define __PCA9685_H_
static unsigned char fd;


// I2C address(default) of pca9685

#define PCA9685_ADDR		(0x40)
// Registers
#define MODE1			(0x00)
#define MODE2			(0x01)
#define SUBADR1			(0x02)
#define SUBADR2			(0x03)
#define SUBADR3			(0x04)
#define PRESCALE		(0xFE)
#define LED0_ON_L		(0x06)
#define LED0_ON_H		(0x07)
#define LED0_OFF_L		(0x08)
#define LED0_OFF_H		(0x09)
#define ALL_LED_ON_L		(0xFA)
#define ALL_LED_ON_H		(0xFB)
#define ALL_LED_OFF_L		(0xFC)
#define ALL_LED_OFF_H		(0xFD)
// Bits
#define RESTART			(0x80)
#define SLEEP			(0x10)
#define ALLCALL			(0x01)
#define INVRT			(0x10)
#define OUTDRV			(0x04)

#define PAN			(0)
#define TILT			(1)
#define FREQUENCY		(50)
#define CLOCKFREQ		(25000000)
#define PANOFFSET		(1)
#define PANSCALE		(1.4)
#define TILTOFFSET		(30)
#define TILTSCALE		(1.43)
#define PANMAX			(270)
#define PANMIN			(90)
#define TILTMAX			(90)
#define TILTMIN			(-45)

//motor
#define GPIO_PIN0 0
#define GPIO_PIN1 1
#define GPIO_PIN2 2
#define GPIO_PIN3 3
uint8_t Motor_GPIO_init(void);
void Motor_U(uint8_t time,uint8_t speed);
void Motor_D(uint8_t time,uint8_t speed);
void Motor_L(uint8_t count,uint8_t speed);
void Motor_R(uint8_t count,uint8_t speed);
void L_motor(void);
void R_motor(void);
void U_motor(void);
void D_motor(void);
void S_motor(void);
uint8_t setUpDevice(uint16_t fd);
uint8_t setPWMFreq(uint16_t fd, uint16_t freq);
uint8_t setPWM(uint16_t fd, uint8_t channel, uint16_t on, uint16_t off);
uint8_t setAllPWM(uint16_t fd, uint16_t on, uint16_t off);
uint8_t move(uint16_t fd, uint8_t s, int16_t deg);
uint8_t Forword(uint16_t fd, int16_t deg);
uint8_t Rotate(uint16_t fd, int16_t deg);
int16_t convAxis(uint8_t s, int16_t deg);
uint8_t sleepOn(uint16_t fd);
uint8_t pca9685_init(void);


#endif
