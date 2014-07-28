/*
 * dynamixel.cpp
 *
 * Created: 4/6/2014 1:32:22 AM
 *  Author: Abhay-HP
 */ 
#include "dynamixel.h"

#include "utils/Communication.h"

void DynamixelUSART::Stream_Reset()
{
	stream[0]=0x00;
	stream[1]=0x00;
	counter_recv=0;
	counter_read=0;	
}

void DynamixelUSART::USART0_Init(uint8_t ubrr)
{
	DDRA |= 0x01;
//Set Baud rate
	UBRRH=(unsigned char)(ubrr>>8);
	UBRRL=(unsigned char)ubrr;

	/*Set Frame Format
	
	Asynchronous mode
	No Parity
	1 StopBit
	char size 8

	*/

	UCSRA = (1<<U2X);
	UCSRC = (1<<URSEL)|(3<<UCSZ0);

	/*Enable Interrupts
	RXCIE- Receive complete
	UDRIE- Data register empty

	Enable The recevier and transmitter

	*/
	UCSRB = (1<<RXCIE)|(1<<RXEN)|(1<<TXEN);
	sei();
}
ISR(USART_RXC_vect)
{
	if (counter_recv>=80)
	{
		counter_recv=0;
	}
	stream[counter_recv++] = UDR; // Fetch the received byte value into the variable 
}


void DynamixelUSART::USART0_End()
{
	UBRRH = 0;
	UBRRL = 0;
	UCSRC = 0;
	UCSRB = 0;
}

void DynamixelUSART::sendData(unsigned char uchar)
{
	//Wait For Transmitter to become ready
	while(!(UCSRA & (1<<UDRE)));

	//Now write
	UDR = uchar;

}

void DynamixelUSART::sendData(const unsigned char *ustring)
{
	while ( *ustring )
	{
		while (!(UCSRA&(1<<UDRE)));
		UDR=*ustring++;
	}
}

unsigned char DynamixelUSART::recvData()
{
	return stream[counter_read++];
}

void DynamixelUSART::switchCom(mode m)
{
	switch (m)
	{
		case Rx_mode:
			PORTA &=(~0x01);
			break;
		case Tx_mode:
			PORTA|=0x01;
			break;
		default:
			break;
	}
}

/*
void DynamixelUSART::recvStream(unsigned char *Stream)
{
	switchCom(Rx_mode);
	uint16_t i=0,len=4;
	unsigned char *tStream;
	for (i=0;i<len;i++)
	{
		tStream[i]=recvData();
		if (i==3)
		{
			len=tStream[3];
		}
	}
	Stream=tStream;
}*/



void Dynamixel::begin(uint8_t ubrr0)
{
	USART0_Init(ubrr0);
}

void Dynamixel::end()
{
	USART0_End();
}

uint16_t Dynamixel::move(unsigned char ID, uint16_t Position	)
{
	unsigned char Position_H,Position_L;				//Place which can give error
	Position_H = Position >> 8;           // 16 bits - 2 x 8 bits varia                     bles
	Position_L = Position;
	Checksum = (~(ID + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_GOAL_POSITION_L + Position_L + Position_H))&0xFF;
	//while (!(UCSR0A&(1<<UDRE0)));
	switchCom(Tx_mode);
	sendData(AX_START);                 // Send Instructions over Serial
	sendData(AX_START);
	sendData(ID);
	sendData(AX_GOAL_LENGTH);
	sendData(AX_WRITE_DATA);
	sendData(AX_GOAL_POSITION_L);
	sendData(Position_L);
	sendData(Position_H);
	sendData(Checksum);
	_delay_us(TX_DELAY_TIME);
	switchCom(Rx_mode);
	return error();//error();
}

uint16_t Dynamixel::moveRW(unsigned char ID, uint16_t Position)
{
	unsigned char Position_H,Position_L;				//Place which can give error
	Position_H = Position >> 8;           // 16 bits - 2 x 8 bits variables
	Position_L = Position;
	Checksum = (~(ID + AX_GOAL_LENGTH + AX_REG_WRITE + AX_GOAL_POSITION_L + Position_L + Position_H))&0xFF;

	switchCom(Tx_mode);
	sendData(AX_START);                 // Send Instructions over Serial
	sendData(AX_START);
	sendData(ID);
	sendData(AX_GOAL_LENGTH);
	sendData(AX_REG_WRITE);
	sendData(AX_GOAL_POSITION_L);
	sendData(Position_L);
	sendData(Position_H);
	sendData(Checksum);
	_delay_us(TX_DELAY_TIME);
	switchCom(Rx_mode);
	return error();
}

uint16_t Dynamixel::moveSpeed(unsigned char ID, uint16_t Position, uint16_t Speed)
{
	unsigned char Position_H,Position_L,Speed_H,Speed_L;
	Position_H = Position >> 8;
	Position_L = Position;                // 16 bits - 2 x 8 bits variables
	Speed_H = Speed >> 8;
	Speed_L = Speed;                      // 16 bits - 2 x 8 bits variables
	Checksum = (~(ID + AX_GOAL_SP_LENGTH + AX_WRITE_DATA + AX_GOAL_POSITION_L + Position_L + Position_H + Speed_L + Speed_H))&0xFF;
	
	switchCom(Tx_mode);
	sendData(AX_START);                // Send Instructions over Serial
	sendData(AX_START);
	sendData(ID);
	sendData(AX_GOAL_SP_LENGTH);
	sendData(AX_WRITE_DATA);
	sendData(AX_GOAL_POSITION_L);
	sendData(Position_L);
	sendData(Position_H);
	sendData(Speed_L);
	sendData(Speed_H);
	sendData(Checksum);
	_delay_us(TX_DELAY_TIME);
	switchCom(Rx_mode);
	return error();
}

void Dynamixel::sendMotorSYNC(const Pose pose)
{
	int check = 0;
	switchCom(Tx_mode);
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
		sendData(i+1);
		check+=i+1;
		sendData(pose.Motor[i]);
		check+=pose.Motor[i];
		sendData(pose.Motor[i]>>8);
		check+=(pose.Motor[i])>>8;
	}
	sendData(~(uint8_t(check)));
	switchCom(Rx_mode);
}

uint16_t Dynamixel::error()
{
	uint16_t err;
	if (recvData()==0xff && recvData()==0xff)
	{
		err=stream[counter_read+2];
		Stream_Reset();
	}
	else
	{
		err=-1;
		Stream_Reset();
	}
	return err;
}

uint16_t Dynamixel::moveSpeedRW(unsigned char ID, uint16_t Position, uint16_t Speed)
{
	 char Position_H,Position_L;
	 Position_H = Position >> 8;           // 16 bits - 2 x 8 bits variables
	 Position_L = Position;
	 Checksum = (~(ID + AX_GOAL_LENGTH + AX_REG_WRITE + AX_GOAL_POSITION_L + Position_L + Position_H))&0xFF;

	 switchCom(Tx_mode);
	 sendData(AX_START);                 // Send Instructions over Serial
	 sendData(AX_START);
	 sendData(ID);
	 sendData(AX_GOAL_LENGTH);
	 sendData(AX_REG_WRITE);
	 sendData(AX_GOAL_POSITION_L);
	 sendData(Position_L);
	 sendData(Position_H);
	 sendData(Checksum);
	 _delay_us(TX_DELAY_TIME);
	 switchCom(Rx_mode);
	 return error();
}

uint16_t Dynamixel::ping(unsigned char ID)
{
	Checksum = (~(ID + AX_READ_DATA + AX_PING))&0xFF;
	
	switchCom(Tx_mode);
	sendData(AX_START);
	sendData(AX_START);
	sendData(ID);
	sendData(AX_READ_DATA);
	sendData(AX_PING);
	sendData(Checksum);
	_delay_us(TX_DELAY_TIME);
	switchCom(Rx_mode);
	return error();
}

uint16_t Dynamixel::reset(unsigned char ID)
{
	Checksum = (~(ID + AX_RESET_LENGTH + AX_RESET))&0xFF;
	
	switchCom(Tx_mode);
	sendData(AX_START);
	sendData(AX_START);
	sendData(ID);
	sendData(AX_RESET_LENGTH);
	sendData(AX_RESET);
	sendData(Checksum);
	_delay_us(TX_DELAY_TIME);
	switchCom(Rx_mode);
	return error();
}

uint16_t Dynamixel::setEndless(unsigned char ID,bool Status)
{
	if ( Status ) {
		char AX_CCW_AL_LT = 0;     // Changing the CCW Angle Limits for Full Rotation.
		Checksum = (~(ID + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_CCW_ANGLE_LIMIT_L))&0xFF;
		
		switchCom(Tx_mode);
		sendData(AX_START);                // Send Instructions over Serial
		sendData(AX_START);
		sendData(ID);
		sendData(AX_GOAL_LENGTH);
		sendData(AX_WRITE_DATA);
		sendData(AX_CCW_ANGLE_LIMIT_L );
		sendData(AX_CCW_AL_LT);
		sendData(AX_CCW_AL_LT);
		sendData(Checksum);
		_delay_us(TX_DELAY_TIME);
		switchCom(Rx_mode);
		return error();
	}
	else
	{
		turn(ID,0,0);
		Checksum = (~(ID + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_CCW_ANGLE_LIMIT_L + AX_CCW_AL_L + AX_CCW_AL_H))&0xFF;
		
		switchCom(Tx_mode);
		sendData(AX_START);                 // Send Instructions over Serial
		sendData(AX_START);
		sendData(ID);
		sendData(AX_GOAL_LENGTH);
		sendData(AX_WRITE_DATA);
		sendData(AX_CCW_ANGLE_LIMIT_L);
		sendData(AX_CCW_AL_L);
		sendData(AX_CCW_AL_H);
		sendData(Checksum);
		_delay_us(TX_DELAY_TIME);
		switchCom(Rx_mode);
		return error();
	}
}

uint16_t Dynamixel::turn(unsigned char ID, bool SIDE, uint16_t Speed)
{
	if(SIDE == 0)
	{                          // Move Left///////////////////////////
		unsigned char Speed_H,Speed_L;
		Speed_H = Speed >> 8;
		Speed_L = Speed;                     // 16 bits - 2 x 8 bits variables
		Checksum = (~(ID + AX_SPEED_LENGTH + AX_WRITE_DATA + AX_GOAL_SPEED_L + Speed_L + Speed_H))&0xFF;
	
		switchCom(Tx_mode);
		sendData(AX_START);                // Send Instructions over Serial
		sendData(AX_START);
		sendData(ID);
		sendData(AX_SPEED_LENGTH);
		sendData(AX_WRITE_DATA);
		sendData(AX_GOAL_SPEED_L);
		sendData(Speed_L);
		sendData(Speed_H);
		sendData(Checksum);
		_delay_us(TX_DELAY_TIME);
		switchCom(Rx_mode);
		return error();
	}
	else
	{                                            // Move Rigth////////////////////
		unsigned char Speed_H,Speed_L;
		Speed_H = (Speed >> 8) + 4;
		Speed_L = Speed;                     // 16 bits - 2 x 8 bits variables
		Checksum = (~(ID + AX_SPEED_LENGTH + AX_WRITE_DATA + AX_GOAL_SPEED_L + Speed_L + Speed_H))&0xFF;
	
		switchCom(Tx_mode);
		sendData(AX_START);                // Send Instructions over Serial
		sendData(AX_START);
		sendData(ID);
		sendData(AX_SPEED_LENGTH);
		sendData(AX_WRITE_DATA);
		sendData(AX_GOAL_SPEED_L);
		sendData(Speed_L);
		sendData(Speed_H);
		sendData(Checksum);
		_delay_us(TX_DELAY_TIME);
		switchCom(Rx_mode);
		return error();
	}
}

uint16_t Dynamixel::action()
{
	switchCom(Tx_mode);
	sendData(AX_START);                // Send Instructions over Serial
	sendData(AX_START);
	sendData(BROADCAST_ID);
	sendData(AX_ACTION_LENGTH);
	sendData(AX_ACTION);
	sendData(AX_ACTION_CHECKSUM);
	_delay_us(TX_DELAY_TIME);
	switchCom(Rx_mode);
	return error();
}




uint16_t Dynamixel::readTemperature(unsigned char ID)
{
	Checksum = (~(ID + AX_TEM_LENGTH  + AX_READ_DATA + AX_PRESENT_TEMPERATURE + AX_BYTE_READ))&0xFF;
	
	switchCom(Tx_mode);
	sendData(AX_START);
	sendData(AX_START);
	sendData(ID);
	sendData(AX_TEM_LENGTH);
	sendData(AX_READ_DATA);
	sendData(AX_PRESENT_TEMPERATURE);
	sendData(AX_BYTE_READ);
	sendData(Checksum);
	_delay_us(TX_DELAY_TIME);
	switchCom(Rx_mode);
	if (recvData()==0xff&&recvData()==0xff)
	{
		if (recvData()==ID&&stream[counter_read+1]==0x00)
		{
			Temperature=stream[counter_read+2];
			Stream_Reset();
		}
		else
		{
			Temperature=-1;
			Stream_Reset();
		}
	}
	else
	{
		Temperature=-1;
		Stream_Reset();
	}
	return Temperature;
}

uint16_t Dynamixel::readPosition(unsigned char ID)
{
	Checksum = (~(ID + AX_POS_LENGTH  + AX_READ_DATA + AX_PRESENT_POSITION_L + AX_BYTE_READ_POS))&0xFF;
	//while (!(UCSR0A&(1<<UDRE0)));
	switchCom(Tx_mode);
	sendData(AX_START);
	sendData(AX_START);
	sendData(ID);
	sendData(AX_POS_LENGTH);
	sendData(AX_READ_DATA);
	sendData(AX_PRESENT_POSITION_L);
	sendData(AX_BYTE_READ_POS);
	sendData(Checksum);
	_delay_us(TX_DELAY_TIME);
	_delay_ms(50);
	switchCom(Rx_mode);
	if (recvData()==0xff&&recvData()==0xff)
	{
		if (recvData()==ID&&stream[counter_read+1]==0x00)
		{
			Position = stream[counter_read+2]<< 8;
			Position = Position + stream[counter_read+3];
			Stream_Reset();
		}
		else
		{
			Position=-1;
			Stream_Reset();
		}
	}
	else
	{
		Position=-1;
		Stream_Reset();
	}
	return Position;
}

uint16_t Dynamixel::readVoltage(unsigned char ID)
{
	Checksum = (~(ID + AX_VOLT_LENGTH  + AX_READ_DATA + AX_PRESENT_VOLTAGE + AX_BYTE_READ))&0xFF;
	
	switchCom(Tx_mode);
	sendData(AX_START);
	sendData(AX_START);
	sendData(ID);
	sendData(AX_VOLT_LENGTH);
	sendData(AX_READ_DATA);
	sendData(AX_PRESENT_VOLTAGE);
	sendData(AX_BYTE_READ);
	sendData(Checksum);
	_delay_us(TX_DELAY_TIME);
	switchCom(Rx_mode);
	
	if (recvData()==0xff&&recvData()==0xff)
	{
		if (recvData()==ID&&stream[counter_read+1]==0x00)
		{
			Voltage=stream[counter_read+2];
			Stream_Reset();
		}
		else
		{
			Voltage=-1;
			Stream_Reset();
		}
	}
	else
	{
		Voltage=-1;
		Stream_Reset();
	}
	return Voltage;
}

uint16_t Dynamixel::readSpeed(unsigned char ID)
{
	Checksum = (~(ID + AX_POS_LENGTH  + AX_READ_DATA + AX_PRESENT_SPEED_L + AX_BYTE_READ_POS))&0xFF;
	
	switchCom(Tx_mode);
	sendData(AX_START);
	sendData(AX_START);
	sendData(ID);
	sendData(AX_POS_LENGTH);
	sendData(AX_READ_DATA);
	sendData(AX_PRESENT_SPEED_L);
	sendData(AX_BYTE_READ_POS);
	sendData(Checksum);
	_delay_us(TX_DELAY_TIME);
	switchCom(Rx_mode);
	if (recvData()==0xff&&recvData()==0xff)
	{
		if (recvData()==ID&&stream[counter_read+1]==0x00)
		{
			Speed = stream[counter_read+2]<< 8;
			Speed = Speed + stream[counter_read+3];
			Stream_Reset();
		}
		else
		{
			Speed=-1;
			Stream_Reset();
		}
	}
	else
	{
		Speed=-1;
		Stream_Reset();
	}
	return (Speed);     
}

uint16_t Dynamixel::readLoad(unsigned char ID)
{
	Checksum = (~(ID + AX_POS_LENGTH  + AX_READ_DATA + AX_PRESENT_LOAD_L + AX_BYTE_READ_POS))&0xFF;
	
	switchCom(Tx_mode);
	sendData(AX_START);
	sendData(AX_START);
	sendData(ID);
	sendData(AX_POS_LENGTH);
	sendData(AX_READ_DATA);
	sendData(AX_PRESENT_LOAD_L);
	sendData(AX_BYTE_READ_POS);
	sendData(Checksum);
	_delay_us(TX_DELAY_TIME);
	switchCom(Rx_mode);
	if (recvData()==0xff&&recvData()==0xff)
	{
		if (recvData()==ID&&stream[counter_read+1]==0x00)
		{
			Load=stream[counter_read+2];
			Stream_Reset();
		}
		else
		{
			Load=-1;
			Stream_Reset();
		}
	}
	else
	{
		Load=-1;
		Stream_Reset();
	}
	return (Load);     
}