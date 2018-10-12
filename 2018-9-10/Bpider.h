#ifndef __BPIDER_H_
#define __BPIDER_H_
char flag_jy901,flag_uwb,flag_socket,stable_flag;

//UWB
#define I_UWB_MAX      128
#define Byte32(Type, Byte4, Byte3, Byte2, Byte1)  ((Type)( (((uint32_t)(Byte4))<<24) | (((uint32_t)(Byte3))<<16) | (((uint32_t)(Byte2))<<8) | ((uint32_t)(Byte1))))

//uwb
float uwb_tag_id;
float uwb_tag_position_x;
float uwb_tag_position_y;
float uwb_tag_position_z;
typedef signed int sint32;

//socket
#define BUFSIZ 100
static char buf[BUFSIZ];
int numbytes;
typedef enum {
kMotor_U,    
kMotor_D,  
kMotor_L,   
kMotor_R,
kMotor_S,   
kFrontLeds,
kAuto       
}MessageType ;

//jy901
struct SAngle
{
	short Angle[3];
	short T;
};
struct SMag
{
	short h[3];
	short T;
};
struct SMag 	stcMag;
struct SAngle 	stcAngle;


void CopeSerialData(unsigned char ucData);

uint8_t interruput(void);
void serial_check(void);
uint8_t pca9685_pwn();
uint8_t I_UWB_LPS_Tag_DateFrame0_Unpack(unsigned char uwbdata);
void Set_Client(void);
void send_data(void);
void WiringPi_check(void);
void Ask_motor(void);
void stable_data(void);
#endif