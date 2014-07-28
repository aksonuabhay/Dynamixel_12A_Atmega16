/*
 * dynamixel.h
 *
 * Created: 4/6/2014 1:25:29 AM
 *  Author: Abhay-HP
 */ 
#define F_CPU 16000000UL
#ifndef DYNAMIXEL_H_
#define DYNAMIXEL_H_
#include <avr/io.h>
#include "util/delay.h"
#include "avr/interrupt.h"
#include "../../../Bumblebee/model/state/Pose2D.h"
#include "../../../Bumblebee/model/description/MotorID.h"

#define STREAM_SIZE 80
#define SIZE 21
#define DATA_LENGTH 2
#define DIRECTION_PORT PORTA
#define DIRECTION_PIN 0x01;
// EEPROM AREA  ///////////////////////////////////////////////////////////
#define AX_MODEL_NUMBER_L           0
#define AX_MODEL_NUMBER_H           1
#define AX_VERSION                  2
#define AX_ID                       3
#define AX_BAUD_RATE                4
#define AX_RETURN_DELAY_TIME        5
#define AX_CW_ANGLE_LIMIT_L         6
#define AX_CW_ANGLE_LIMIT_H         7
#define AX_CCW_ANGLE_LIMIT_L        8
#define AX_CCW_ANGLE_LIMIT_H        9
#define AX_SYSTEM_DATA2             10
#define AX_LIMIT_TEMPERATURE        11
#define AX_DOWN_LIMIT_VOLTAGE       12
#define AX_UP_LIMIT_VOLTAGE         13
#define AX_MAX_TORQUE_L             14
#define AX_MAX_TORQUE_H             15
#define AX_RETURN_LEVEL             16
#define AX_ALARM_LED                17
#define AX_ALARM_SHUTDOWN           18
#define AX_OPERATING_MODE           19
#define AX_DOWN_CALIBRATION_L       20
#define AX_DOWN_CALIBRATION_H       21
#define AX_UP_CALIBRATION_L         22
#define AX_UP_CALIBRATION_H         23

// RAM AREA  //////////////////////////////////////////////////////////////
#define AX_TORQUE_ENABLE            24
#define AX_LED                      25
#define AX_CW_COMPLIANCE_MARGIN     26
#define AX_CCW_COMPLIANCE_MARGIN    27
#define AX_CW_COMPLIANCE_SLOPE      28
#define AX_CCW_COMPLIANCE_SLOPE     29
#define AX_GOAL_POSITION_L          30
#define AX_GOAL_POSITION_H          31
#define AX_GOAL_SPEED_L             32
#define AX_GOAL_SPEED_H             33
#define AX_TORQUE_LIMIT_L           34
#define AX_TORQUE_LIMIT_H           35
#define AX_PRESENT_POSITION_L       36
#define AX_PRESENT_POSITION_H       37
#define AX_PRESENT_SPEED_L          38
#define AX_PRESENT_SPEED_H          39
#define AX_PRESENT_LOAD_L           40
#define AX_PRESENT_LOAD_H           41
#define AX_PRESENT_VOLTAGE          42
#define AX_PRESENT_TEMPERATURE      43
#define AX_REGISTERED_INSTRUCTION   44
#define AX_PAUSE_TIME               45
#define AX_MOVING                   46
#define AX_LOCK                     47
#define AX_PUNCH_L                  48
#define AX_PUNCH_H                  49

// Status Return Levels ///////////////////////////////////////////////////////////////
#define AX_RETURN_NONE              0
#define AX_RETURN_READ              1
#define AX_RETURN_ALL               2

// Instruction Set ///////////////////////////////////////////////////////////////
#define AX_PING                     1
#define AX_READ_DATA                2
#define AX_WRITE_DATA               3
#define AX_REG_WRITE                4
#define AX_ACTION                   5
#define AX_RESET                    6
#define AX_SYNC_WRITE               131

// Specials ///////////////////////////////////////////////////////////////
#define OFF                         0
#define ON                          1
#define LEFT						0
#define RIGTH                       1
#define AX_BYTE_READ                1
#define AX_BYTE_READ_POS            2
#define AX_RESET_LENGTH				2
#define AX_ACTION_LENGTH			2
#define AX_ID_LENGTH                4
#define AX_LR_LENGTH                4
#define AX_SRL_LENGTH               4
#define AX_RDT_LENGTH               4
#define AX_LEDALARM_LENGTH          4
#define AX_SALARM_LENGTH            4
#define AX_TL_LENGTH                4
#define AX_VL_LENGTH                6
#define AX_CM_LENGTH                6
#define AX_CS_LENGTH                5
#define AX_CCW_CW_LENGTH            8
#define AX_BD_LENGTH                4
#define AX_TEM_LENGTH               4
#define AX_MOVING_LENGTH            4
#define AX_RWS_LENGTH               4
#define AX_VOLT_LENGTH              4
#define AX_LED_LENGTH               4
#define AX_TORQUE_LENGTH            4
#define AX_POS_LENGTH               4
#define AX_GOAL_LENGTH              5
#define AX_MT_LENGTH                5
#define AX_PUNCH_LENGTH             5
#define AX_SPEED_LENGTH             5
#define AX_GOAL_SP_LENGTH           7
#define AX_ACTION_CHECKSUM			250
#define BROADCAST_ID                254
#define AX_START                    255
#define AX_CCW_AL_L                 255
#define AX_CCW_AL_H                 3
#define TIME_OUT                    10         
#define TX_DELAY_TIME				400        
#define Tx_MODE                     1
#define Rx_MODE                     0
#define LOCK                        1

#define BAUD 1000000UL
#define USART_UBBR_VALUE0 ((F_CPU/(BAUD*8))-1) //Running in double speed u2x=1 mode

static unsigned char stream[STREAM_SIZE];
static uint16_t counter_recv=0,counter_read=0;
enum mode{
	Rx_mode,Tx_mode
};
class DynamixelUSART
{
	public:
	void Stream_Reset();
	void USART0_Init(uint8_t ubrr0);
	void sendData(unsigned char uchar);
	void sendData(const unsigned char *ustring);
	void USART0_End(void);
	unsigned char recvData();
	void switchCom(mode m);
	//void recvStream(unsigned char *Stream);
};

class Dynamixel:public DynamixelUSART
{
	public:
	void begin(uint8_t ubrr0,uint8_t ubrr1);
	void begin(uint8_t ubrr0);
	void end(void);
	uint16_t action(void);
	uint16_t reset(unsigned char ID);
	uint16_t ping(unsigned char ID);
	uint16_t error();
	
	uint16_t move(unsigned char ID,uint16_t Position);
	uint16_t moveSpeed(unsigned char ID,uint16_t Position,uint16_t Speed);
	uint16_t setEndless(unsigned char ID,bool Status);
	uint16_t turn(unsigned char ID, bool SIDE,uint16_t Speed);
	uint16_t moveRW(unsigned char ID,uint16_t Position);
	uint16_t moveSpeedRW(unsigned char ID,uint16_t Position,uint16_t Speed);
	
	uint16_t readTemperature(unsigned char ID);
	uint16_t readVoltage(unsigned char ID);
	uint16_t readPosition(unsigned char ID);
	uint16_t readSpeed(unsigned char ID);
	uint16_t readLoad(unsigned char ID);
	void sendMotorSYNC(const Pose pose);
	protected:
	private:
	unsigned char Checksum;
	uint16_t Temperature;
	uint16_t Position;
	uint16_t Voltage;
	uint16_t Speed;
	uint16_t Load;
/*
	{
		int check = 0;
		sendData(AX_START);
		sendData(AX_START);
		sendData(BROADCAST_ID);
		check+=BROADCAST_ID;
		sendData((DATA_LENGTH+1)*SIZE+4);
		check+=(DATA_LENGTH+1)*SIZE+4;
		sendData(AX_SYNC_WRITE);
		check+=AX_SYNC_WRITE;
		sendData(P_GOAL_POSITION_L);
		check+=P_GOAL_POSITION_L;
		sendData(DATA_LENGTH);
		check+=DATA_LENGTH;
		for (uint8_t i=0;i<SIZE;i++)
		{
			sendData(i);
			check+=i;
			sendData(pose->Motor[i]);
			check+=pose->Motor[i];
			sendData(pose->Motor[i]>>8);
			check+=(pose->Motor[i])>>8;
		}
		sendData(~(uint8_t(check)));
		return error();
	}*/
};
#endif /* DYNAMIXEL_H_ */