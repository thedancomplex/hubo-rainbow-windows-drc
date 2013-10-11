
#include "CAN.h"
#include "APIs.h"
#include <math.h>
#include "RTX.h"


bool RBsetAndRequestBoardInformation(unsigned char _canch, unsigned char _bno, unsigned char _canr)
{
	// MsgID		Byte0	Byte1	Byte2
	// CMD_TXDF		BNO		0x01	CAN rate
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;		// board no.
	tempData[1] = 0x01;		// command
	tempData[2] = _canr;	// CAN communication rate(Hz)
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 3, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBrequestBoardStatus(unsigned char _canch, unsigned char _bno)
{
	// MsgID		Byte0	Byte1
	// CMD_TXDF		BNO		0x02
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;		// board no.
	tempData[1] = 0x02;		// command
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 2, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBrequestEncoderPosition(unsigned char _canch, unsigned char _bno, unsigned char _fes, unsigned char continuous)
{
	// MsgID		Byte0	Byte1	Byte2
	// CMD_TXDF		BNO		0x03	FES
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;		// board no.
	tempData[1] = 0x03;		// command
	tempData[2] = _fes;		// for finger board
	tempData[3] = continuous;
	// _fes = 0x00 : Request M0, M1 and M2 encoder position
	// _fes = 0x01 : Request M3 and M4 encoder position
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 4, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBrequestCurrentValue(unsigned char _canch, unsigned char _bno)
{
	// MsgID		Byte0	Byte1
	// CMD_TXDF		BNO		0x04
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;		// board no.
	tempData[1] = 0x04;		// command
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 2, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBResetEncoderZero(unsigned char _canch, unsigned char _bno, unsigned char _ch)
{
	// MsgID		Byte0	Byte1	Byte2
	// CMD_TXDF		BNO		0x06	CH
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;		// board no.
	tempData[1] = 0x06;		// command
	tempData[2] = _ch;		// motor channel
	// _ch = 0x00 : 1st channel
	// _ch = 0x01 : 2nd channel
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 3, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBsetMotorPositionGain0(unsigned char _canch, unsigned char _bno, int _kp, int _ki, int _kd)
{
	// MsgID		Byte0	Byte1	Byte2	Byte3	Byte4	Byte5	Byte6	byte7
	// CMD_TXDF		BNO		0x07	KP		KP		KI		KI		KD		KD
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;		// board no.
	tempData[1] = 0x07;		// command
	tempData[2] = (unsigned char)(_kp&0xFF);		// position control p-gain
	tempData[3] = (unsigned char)((_kp>>8)&0xFF);	// position control p-gain
	tempData[4] = (unsigned char)(_ki&0xFF);		// position control i-gain
	tempData[5] = (unsigned char)((_ki>>8)&0xFF);	// position control i-gain
	tempData[6] = (unsigned char)(_kd&0xFF);		// position control d-gain
	tempData[7] = (unsigned char)((_kd>>8)&0xFF);	// position control d-gain
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 8, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBsetMotorPositionGain1(unsigned char _canch, unsigned char _bno, int _kp, int _ki, int _kd)
{
	// MsgID		Byte0	Byte1	Byte2	Byte3	Byte4	Byte5	Byte6	byte7
	// CMD_TXDF		BNO		0x08	KP		KP		KI		KI		KD		KD
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;		// board no.
	tempData[1] = 0x08;		// command
	tempData[2] = (unsigned char)(_kp&0xFF);		// position control p-gain
	tempData[3] = (unsigned char)((_kp>>8)&0xFF);	// position control p-gain
	tempData[4] = (unsigned char)(_ki&0xFF);		// position control i-gain
	tempData[5] = (unsigned char)((_ki>>8)&0xFF);	// position control i-gain
	tempData[6] = (unsigned char)(_kd&0xFF);		// position control d-gain
	tempData[7] = (unsigned char)((_kd>>8)&0xFF);	// position control d-gain
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 8, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBsetMotorCurrentGain0(unsigned char _canch, unsigned char _bno, int _kp, int _ki, int _kd)
{
	// MsgID		Byte0	Byte1	Byte2	Byte3	Byte4	Byte5	Byte6	byte7
	// CMD_TXDF		BNO		0x09	KP		KP		KI		KI		KD		KD
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;		// board no.
	tempData[1] = 0x09;		// command
	tempData[2] = (unsigned char)(_kp&0xFF);		// current control p-gain
	tempData[3] = (unsigned char)((_kp>>8)&0xFF);	// current control p-gain
	tempData[4] = (unsigned char)(_ki&0xFF);		// current control i-gain
	tempData[5] = (unsigned char)((_ki>>8)&0xFF);	// current control i-gain
	tempData[6] = (unsigned char)(_kd&0xFF);		// current control d-gain
	tempData[7] = (unsigned char)((_kd>>8)&0xFF);	// current control d-gain
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 8, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBsetMotorCurrentGain1(unsigned char _canch, unsigned char _bno, int _kp, int _ki, int _kd)
{
	// MsgID		Byte0	Byte1	Byte2	Byte3	Byte4	Byte5	Byte6	byte7
	// CMD_TXDF		BNO		0x0A	KP		KP		KI		KI		KD		KD
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;		// board no.
	tempData[1] = 0x0A;		// command
	tempData[2] = (unsigned char)(_kp&0xFF);		// current control p-gain
	tempData[3] = (unsigned char)((_kp>>8)&0xFF);	// current control p-gain
	tempData[4] = (unsigned char)(_ki&0xFF);		// current control i-gain
	tempData[5] = (unsigned char)((_ki>>8)&0xFF);	// current control i-gain
	tempData[6] = (unsigned char)(_kd&0xFF);		// current control d-gain
	tempData[7] = (unsigned char)((_kd>>8)&0xFF);	// current control d-gain
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 8, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBenableFETDriver(unsigned char _canch, unsigned char _bno, unsigned char _enable)
{
	// MsgID		Byte0	Byte1	Byte2
	// CMD_TXDF		BNO		0x0B	enable/disable	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;		// board no.
	tempData[1] = 0x0B;		// command
	tempData[2] = _enable;	// FET driver on/off
	// _enable = 0x00: disable(turn-off) FET driver
	// _enable = 0x01: enable(turn-on) FET driver
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 3, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBpwmCommand2ch(unsigned char _canch, unsigned char _bno, int _duty1, int _duty2, unsigned char _zeroduty)
{
	// MsgID		Byte0	Byte1	Byte2		Byte3	Byte4	Byte5	Byte6
	// CMD_TXDF		BNO		0x0D	_zeroduty	DIR0	DUTY0	DIR1	DUTY1
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;							// board no.
	tempData[1] = 0x0D;							// command
	tempData[2] = _zeroduty;					// Zero Duty
	// _zeroduty = 0x00 : Enforce zero duty to stop motor. Back EMF will break the motor
	// _zeroduty = 0x01 : Pulse out to run motor in specified PWM duty and direction
	if(_duty1 >= 0)  
		tempData[3] = 0x00;		// 1st motor direction : CCW
	else 
		tempData[3] = 0x01;					// 1st motor direction : CW
	tempData[4] = (unsigned char)(abs(_duty1));	// 1st motor duty
	
	if(_duty2 >= 0)  
		tempData[5] = 0x00;		// 2nd motor direction : CCW
	else
		tempData[5] = 0x01;					// 2nd motor direction : CW
	tempData[6] = (unsigned char)(abs(_duty2));	// 2nd motor duty
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 7, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBpwmCommand3ch(unsigned char _canch, unsigned char _bno, int _duty1, int _duty2, int _duty3, unsigned char _zeroduty)
{
	// MsgID		Byte0	Byte1	Byte2		Byte3		Byte4		Byte5	
	// CMD_TXDF		BNO		0x0D	_zeroduty	DIR0_DUTY0	DIR1_DUTY1	DIR2_DUTY2
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;							// board no.
	tempData[1] = 0x0D;							// command
	tempData[2] = _zeroduty;					// Zero Duty
	// _zeroduty = 0x00 : Enforce zero duty to stop motor. Back EMF will break the motor
	// _zeroduty = 0x01 : Pulse out to run motor in specified PWM duty and direction
	if(_duty1 >= 0)
	{
		tempData[3] = 0x00;									// 1st motor direction : CCW
		tempData[3] |= ((unsigned char)_duty1)&0x7F;		// 1st motor duty
	}
	else 
	{
		tempData[3] = 0x00;	tempData[3] |= 0x80;			// 1st motor direction : CW
		tempData[3] |= ((unsigned char)abs(_duty1))&0x7F;	// 1st motor duty
	}
	if(_duty2 >= 0)
	{
		tempData[4] = 0x00;									// 2nd motor direction : CCW
		tempData[4] |= ((unsigned char)_duty2)&0x7F;		// 2nd motor duty
	}
	else 
	{
		tempData[4] = 0x00;	tempData[4] |= 0x80;			// 2nd motor direction : CW
		tempData[4] |= ((unsigned char)abs(_duty2))&0x7F;	// 2nd motor duty
	}
	if(_duty3 >= 0)
	{
		tempData[5] = 0x00;									// 3rd motor direction : CCW
		tempData[5] |= ((unsigned char)_duty3)&0x7F;		// 3rd motor duty
	}
	else 
	{
		tempData[5] = 0x00;	tempData[5] |= 0x80;			// 3rd motor direction : CW
		tempData[5] |= ((unsigned char)abs(_duty3))&0x7F;	// 3rd motor duty
	}
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 6, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBpwmCommand5ch(unsigned char _canch, unsigned char _bno, int _duty1, int _duty2, int _duty3, int _duty4, int _duty5, unsigned char _zeroduty)
{
	// MsgID		Byte0	Byte1	Byte2		Byte3		Byte4		Byte5		Byte6		Byte7
	// CMD_TXDF		BNO		0x0D	_zeroduty	DIR0_DUTY0	DIR1_DUTY1	DIR2_DUTY2	DIR3_DUTY3	DIR4_DUTY4
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;							// board no.
	tempData[1] = 0x0D;							// command
	tempData[2] = _zeroduty;					// Zero Duty
	// _zeroduty = 0x00 : Enforce zero duty to stop motor. Back EMF will break the motor
	// _zeroduty = 0x01 : Pulse out to run motor in specified PWM duty and direction
	if(_duty1 >= 0)
	{
		tempData[3] = 0x00;									// 1st motor direction : CCW
		tempData[3] |= ((unsigned char)_duty1)&0x7F;		// 1st motor duty
	}
	else 
	{
		tempData[3] = 0x00;	tempData[3] |= 0x80;			// 1st motor direction : CW
		tempData[3] |= ((unsigned char)abs(_duty1))&0x7F;	// 1st motor duty
	}
	if(_duty2 >= 0)
	{
		tempData[4] = 0x00;									// 2nd motor direction : CCW
		tempData[4] |= ((unsigned char)_duty2)&0x7F;		// 2nd motor duty
	}
	else 
	{
		tempData[4] = 0x00;	tempData[4] |= 0x80;			// 2nd motor direction : CW
		tempData[4] |= ((unsigned char)abs(_duty2))&0x7F;	// 2nd motor duty
	}
	if(_duty3 >= 0)
	{
		tempData[5] = 0x00;									// 3rd motor direction : CCW
		tempData[5] |= ((unsigned char)_duty3)&0x7F;		// 3rd motor duty
	}
	else 
	{
		tempData[5] = 0x00;	tempData[5] |= 0x80;			// 3rd motor direction : CW
		tempData[5] |= ((unsigned char)abs(_duty3))&0x7F;	// 3rd motor duty
	}
	if(_duty4 >= 0)
	{
		tempData[6] = 0x00;									// 4th motor direction : CCW
		tempData[6] |= ((unsigned char)_duty4)&0x7F;		// 4th motor duty
	}
	else 
	{
		tempData[6] = 0x00;	tempData[6] |= 0x80;			// 4th motor direction : CW
		tempData[6] |= ((unsigned char)abs(_duty4))&0x7F;	// 4th motor duty
	}
	if(_duty5 >= 0)
	{
		tempData[7] = 0x00;									// 5th motor direction : CCW
		tempData[7] |= ((unsigned char)_duty5)&0x7F;		// 5th motor duty
	}
	else 
	{
		tempData[7] = 0x00;	tempData[7] |= 0x80;			// 5th motor direction : CW
		tempData[7] |= ((unsigned char)abs(_duty5))&0x7F;	// 5th motor duty
	}
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 8, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBenableFeedbackControl(unsigned char _canch, unsigned char _bno)
{
	// MsgID		Byte0	Byte1
	// CMD_TXDF		BNO		0x0E
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;		// board no.
	tempData[1] = 0x0E;		// command
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 2, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBdisableFeedbackControl(unsigned char _canch, unsigned char _bno)
{
	// MsgID		Byte0	Byte1
	// CMD_TXDF		BNO		0x0F
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;		// board no.
	tempData[1] = 0x0F;		// command
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 2, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBsetControlMode(unsigned char _canch, unsigned char _bno, unsigned char _mode)
{
	// MsgID		Byte0	Byte1	Byte2
	// CMD_TXDF		BNO		0x10	_mode
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;		// board no.
	tempData[1] = 0x10;		// command
	tempData[2] = _mode;	// control mode
	// _mode = 0x00 : position control mode
	// _mode = 0x01 : current control mode
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 3, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBfindLimitGoOffset(unsigned char _canch, unsigned char _bno, unsigned char _ch, unsigned char _mode, unsigned char _dir, long _offset1, long _offset2)
{
	// MsgID		Byte0	Byte1	Byte2			Byte3		Byte4		Byte5		Byte6		Byte7
	// CMD_TXDF		BNO		0x11	CH_MODE_DIR		OFFSET0H	OFFSET0L	OFFSET1H	OFFSET1L	OFFSET0_OFFSET1
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;							// board no.
	tempData[1] = 0x11;							// command
	if(_ch == 0x00) tempData[2] = 0x00;			// 1st channel
	else if(_ch == 0x01) tempData[2] = 0x10;	// 2nd channel
	else tempData[2] = 0xF0;					// all channels
	if(_mode == 0x00) tempData[2] |= 0x00;		// get offset value from _offset1 and _offset2
	else tempData[2] |= 0x02;					// ignore _offset1 and _offset2
	if(_dir == 0x00) tempData[2] |= 0x00;		// search direction "0"
	else tempData[2] |= 0x01;					// search direction "1"
	tempData[3] = ((_offset1>>8) & (0xFF));		// 1st channel offset
	tempData[4] = (_offset1 & 0xFF);			// 1st channel offset
	tempData[5] = ((_offset2>>8) & (0xFF));		// 2nd channel offset
	tempData[6] = (_offset2 & 0xFF);			// 2nd channel offset
	tempData[7] = ((_offset1>>16) & (0x0F)) | (((_offset2>>16) & (0x0F)) << 4); // 1st and 2nd channel offset and sign
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 8, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBsetDeadzone(unsigned char _canch, unsigned char _bno, unsigned char _ch, unsigned char _deadzone)
{
	// MsgID		Byte0	Byte1	Byte2
	// CMD_TXDF		BNO		0x20+CH	_deadzone
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;			// board no.
	tempData[1] = 0x20+_ch;		// command
	tempData[2] = _deadzone;	// deadzone
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 3, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBsetHomeSearchParameter(unsigned char _canch, unsigned char _bno, unsigned char _ch, unsigned char _rotlimit, unsigned char _dir, long _offset)
{
	// MsgID		Byte0	Byte1		Byte2	Byte3	Byte4	Byte5	Byte6	Byte7
	// CMD_TXDF		BNO		0x30+CH		SRL		SDR		OFFSET	OFFSET	OFFSET	OFFSET
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;						// board no.
	tempData[1] = 0x30+_ch;					// command
	tempData[2] = _rotlimit;				// maximum number of turns to find limit switch
	tempData[3] = _dir;						// search direction
	tempData[4] = (_offset & 0xFF);			// offset
	tempData[5] = ((_offset>>8) & (0xFF));	// offset
	tempData[6] = ((_offset>>16) & (0xFF));	// offset
	tempData[7] = ((_offset>>24) & (0x0F)); // offset
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 8, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBsetEncoderResolution(unsigned char _canch, unsigned char _bno, unsigned char _ch, unsigned int _res)
{
	// MsgID		Byte0	Byte1		Byte2		Byte3
	// CMD_TXDF		BNO		0x38+CH		ENCODER		ENCODER	
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;					// board no.
	tempData[1] = 0x38+_ch;				// command
	tempData[2] = (_res & 0xFF);		// encoder res.
	tempData[3] = ((_res>>8) & (0xFF));	// encoder res.
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 4, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBsetMaxAccVel(unsigned char _canch, unsigned char _bno, unsigned char _ch, unsigned int _acc, unsigned int _vel)
{
	// MsgID		Byte0	Byte1		Byte2	Byte3
	// CMD_TXDF		BNO		0x40+CH		ACC		VEL	
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;					// board no.
	tempData[1] = 0x40+_ch;				// command
	tempData[2] = (_acc & 0xFF);		// maximum acceleration
	tempData[3] = ((_acc>>8) & (0xFF));	// maximum acceleration
	tempData[4] = (_vel & 0xFF);		// maximum velocity
	tempData[5] = ((_vel>>8) & (0xFF));	// maximum velocity
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 6, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBsetLowerPosLimit(unsigned char _canch, unsigned char _bno, unsigned char _ch, long _limit, unsigned char _mode)
{
	// MsgID		Byte0	Byte1		Byte2	Byte3	Byte4	Byte5	Byte6
	// CMD_TXDF		BNO		0x50+CH		_mode	LIMIT	LIMIT	LIMIT	LIMIT
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;						// board no.
	tempData[1] = 0x50+_ch;					// command
	tempData[2] = _mode;					// setting mode
	// _mode = 0x00 : ignore new limit value and disable limit
	// _mode = 0x01 : ignore new limit value and enable limit
	// _mode = 0x02 : update new limit value and disable limit
	// _mode = 0x03 : update new limit value and enable limit
	tempData[3] = (_limit & 0xFF);			// limit position
	tempData[4] = ((_limit>>8) & (0xFF));	// limit position
	tempData[5] = ((_limit>>16) & (0xFF));	// limit position
	tempData[6] = ((_limit>>24) & (0xFF));	// limit position
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 7, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBsetUpperPosLimit(unsigned char _canch, unsigned char _bno, unsigned char _ch, long _limit, unsigned char _mode)
{
	// MsgID		Byte0	Byte1		Byte2	Byte3	Byte4	Byte5	Byte6
	// CMD_TXDF		BNO		0x56+CH		_mode	LIMIT	LIMIT	LIMIT	LIMIT
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;						// board no.
	tempData[1] = 0x56+_ch;					// command
	tempData[2] = _mode;					// setting mode
	// _mode = 0x00 : ignore new limit value and disable limit
	// _mode = 0x01 : ignore new limit value and enable limit
	// _mode = 0x02 : update new limit value and disable limit
	// _mode = 0x03 : update new limit value and enable limit
	tempData[3] = (_limit & 0xFF);			// limit position
	tempData[4] = ((_limit>>8) & (0xFF));	// limit position
	tempData[5] = ((_limit>>16) & (0xFF));	// limit position
	tempData[6] = ((_limit>>24) & (0xFF));	// limit position
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 7, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBsetHomeMaxAccVel(unsigned char _canch, unsigned char _bno, unsigned char _ch, unsigned char _acc, unsigned char _vel1, unsigned char _vel2, unsigned char _mode, unsigned char _duty)
{
	// MsgID		Byte0	Byte1		Byte2	Byte3	Byte4	Byte5	Byte6
	// CMD_TXDF		BNO		0x60+CH		ACC		VEL1	VEL2	_mode	_duty
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;			// board no.
	tempData[1] = 0x60+_ch;		// command
	tempData[2] = _acc;			// acceleration
	tempData[3] = _vel1;		// maximun velocity to reach the limit switch
	tempData[4] = _vel2;		// maximum velocity to reach the offset position
	tempData[5] = _mode;		// search mode
	// _mode = 0x00 : limit switch and index(z-phase)
	// _mode = 0x01 : limit switch only
	// _mode = 0x02 : no limit switch. mechanical limit
	tempData[6] = _duty;		// PWM duty(%) for mechanical limt search mode
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 7, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBgainOverride(unsigned char _canch, unsigned char _bno, unsigned char _override1, unsigned char _override2, unsigned int _duration)
{
	// MsgID		Byte0	Byte1	Byte2	Byte3	Byte4		Byte5
	// CMD_TXDF		BNO		0x6F	OVER1	OVER2	DURATION	DURATION
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;							// board no.
	tempData[1] = 0x6F;							// command
	tempData[2] = _override1;					// override value(%) for 1st channel
	tempData[3] = _override2;					// override value(%) for 2nd channel
	tempData[4] = (_duration & 0xFF);			// duration in msec
	tempData[5] = ((_duration>>8) & (0xFF));	// duration in msec
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 6, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBsetBoardNumber(unsigned char _canch, unsigned char _bno, unsigned char _newbno, unsigned char _canr)
{
	// MsgID		Byte0	Byte1	Byte2	Byte3
	// CMD_TXDF		BNO		0xF0	NEW_BNO	CAN_RATE
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;			// board no.
	tempData[1] = 0xF0;			// command
	tempData[2] = _newbno;		// new board number
	tempData[3] = _canr;		// CAN communication rate(Hz) 
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 4, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBsetJamPwmSat(unsigned char _canch, unsigned char _bno, unsigned int _jam, unsigned int _sat, unsigned char _satduty, unsigned char _jamduty)
{
	// MsgID		Byte0	Byte1	Byte2	Byte3	Byte4	Byte5	Byte6		Byte7
	// CMD_TXDF		BNO		0xF2	JAM		JAM		SAT		SAT		SAT_DUTY	JAM_DUTY
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;					// board no.
	tempData[1] = 0xF2;					// command
	tempData[2] = (_jam & 0xFF);		// duration in msec for JAM detection
	tempData[3] = ((_jam>>8) & (0xFF));	// duration in msec for JAM detection
	tempData[4] = (_sat & 0xFF);		// duration in msec for PWM saturation detection
	tempData[5] = ((_sat>>8) & (0xFF));	// duration in msec for PWM saturation detection
	tempData[6] = _satduty;				// PWM duty for saturation detection
	tempData[7] = _jamduty;				// PWM duty for JAM detection
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 8, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBsetErrorBound(unsigned char _canch, unsigned char _bno, unsigned int _ierror, unsigned int _berror, unsigned int _teeror)
{
	// MsgID		Byte0	Byte1	Byte2		Byte3		Byte4		Byte5		Byte6		Byte7
	// CMD_TXDF		BNO		0xF3	I_ERROR		I_ERROR		B_ERROR		B_ERROR		T_ERROR		T_ERROR
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;						// board no.
	tempData[1] = 0xF3;						// command
	tempData[2] = (_ierror & 0xFF);			// maximum input difference error (new ref - old ref)
	tempData[3] = ((_ierror>>8) & (0xFF));	// maximum input difference error (new ref - old ref)
	tempData[4] = (_berror & 0xFF);			// maximum error (ref - encoder pos)
	tempData[5] = ((_berror>>8) & (0xFF));	// maximum error (ref - encoder pos)
	tempData[6] = (_teeror & 0xFF);			// maximum temerature warning
	tempData[7] = ((_teeror>>8) & (0xFF));	// maximum temerature warning
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 8, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBinitializeBoard(unsigned char _canch, unsigned char _bno)
{
	// MsgID		Byte0	Byte1
	// CMD_TXDF		BNO		0xFA
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;			// board no.
	tempData[1] = 0xFA;			// command
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 2, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBrequestParameter(unsigned char _canch, unsigned char _bno, unsigned char _para)
{
	// MsgID		Byte0	Byte1	Byte2
	// CMD_TXDF		BNO		0x24	PARAMETER
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;			// board no.
	tempData[1] = 0x24;			// command
	tempData[2] = _para;		// parameter request
	// OF = 0 : channel 0, 1 and 2
	// OF = 1 : channel 3 and 4
	// _para = channel*6 + OF + 1 : Motor position control gains and encoder resolution
	// _para = channel*6 + OF + 2 : Deadzone, home search direction, home search mode and home search limit
	// _para = channel*6 + OF + 3 : Home offset and lower position limit
	// _para = channel*6 + OF + 4 : Upper limit position, maximum acceleration, maximum velocity and maximum PWM
	// _para = channel*6 + OF + 5 : Current limit and motor current control gains
	// _para = channel*6 + OF + 6 : motor current control gains
	// _para = 20 : Board number, CAN comunication rate, board type and maximun acceleration for home limit search
	// _para = 21 : Maximun velocity to limit switch, maximun velocity to offset position, duration for JAM detection and duration for saturation detection
	// _para = 22 : PWM duty for saturation, PWM duty for JAM and error bound values
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 3, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBsendReference2ch(unsigned char _canch, unsigned char _bno, long _ref1, long ref2)
{
	// MsgID				Byte0	Byte1	Byte2	Byte3	Byte4	Byte5
	// REF_BASE_TXDF+BNO	REF0	REF0	REF0	REF1	REF1	REF1
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = (unsigned char)(_ref1 & 0x000000FF);			// ref. of 1st channel
	tempData[1] = (unsigned char)( (_ref1>>8) & 0x000000FF );	// ref. of 1st channel
	tempData[2] = (unsigned char)( (_ref1>>16) & 0x000000FF );	// ref. of 1st channel
	tempData[3] = (unsigned char)(ref2 & 0x000000FF);			// ref. of 2nd channel
	tempData[4] = (unsigned char)( (ref2>>8) & 0x000000FF );	// ref. of 2nd channel
	tempData[5] = (unsigned char)( (ref2>>16) & 0x000000FF );	// ref. of 2nd channel
	
	// Send CAN message
	result = PushCANMsg(_canch, REF_BASE_TXDF+_bno, tempData, 6, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBsendReference3ch(unsigned char _canch, unsigned char _bno, char _ref1, char _ref2, char _ref3)
{
	// MsgID				Byte0	Byte1	Byte2
	// REF_BASE_TXDF+BNO	REF0	REF1	REF2
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _ref1;	// ref. of 1st channel
	tempData[1] = _ref2;	// ref. of 2nd channel
	tempData[2] = _ref3;	// ref. of 3rd channel

	// Send CAN message
	result = PushCANMsg(_canch, REF_BASE_TXDF+_bno, tempData, 3, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBsendReference5ch(unsigned char _canch, unsigned char _bno, char _ref1, char _ref2, char _ref3, char _ref4, char _ref5)
{
	// MsgID				Byte0	Byte1	Byte2	Byte3	Byte4
	// REF_BASE_TXDF+BNO	REF0	REF1	REF2	REF3	REF4
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _ref1;	// ref. of 1st channel
	tempData[1] = _ref2;	// ref. of 2nd channel
	tempData[2] = _ref3;	// ref. of 3rd channel
	tempData[1] = _ref4;	// ref. of 4th channel
	tempData[2] = _ref5;	// ref. of 5th channel

	// Send CAN message
	result = PushCANMsg(_canch, REF_BASE_TXDF+_bno, tempData, 5, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBnullFTSensor(unsigned char _canch, unsigned char _bno, unsigned char _mode)
{
	// MsgID		Byte0	Byte1	Byte2
	// CMD_TXDF		BNO		0x81	_mode
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;			// board no.
	tempData[1] = 0x81;			// command
	tempData[2] = _mode;		// NULL mode
	// _mode = 0x00 : FT sensor
	// _mode = 0x02 : Inclinometers in FT sensor
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 3, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBsetFTCoefficient0(unsigned char _canch, unsigned char _bno, int _coeff1, int _coeff2, int _coeff3)
{
	// MsgID		Byte0	Byte1	Byte2	Byte3	Byte4	Byte5	Byte6	Byte7
	// CMD_TXDF		BNO		0xA0	SF00	SF00	SF01	SF01	SF02	SF02
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;						// board no.
	tempData[1] = 0xA0;						// command
	tempData[2] = (_coeff1 & 0xFF);			// scale factor 00
	tempData[3] = ((_coeff1>>8) & (0xFF));	// scale factor 00
	tempData[4] = (_coeff2 & 0xFF);			// scale factor 01
	tempData[5] = ((_coeff2>>8) & (0xFF));	// scale factor 01
	tempData[6] = (_coeff3 & 0xFF);			// scale factor 02
	tempData[7] = ((_coeff3>>8) & (0xFF));	// scale factor 02
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 8, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBsetFTCoefficient1(unsigned char _canch, unsigned char _bno, int _coeff1, int _coeff2, int _coeff3)
{
	// MsgID		Byte0	Byte1	Byte2	Byte3	Byte4	Byte5	Byte6	Byte7
	// CMD_TXDF		BNO		0xA1	SF10	SF10	SF11	SF11	SF12	SF12
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;						// board no.
	tempData[1] = 0xA1;						// command
	tempData[2] = (_coeff1 & 0xFF);			// scale factor 10
	tempData[3] = ((_coeff1>>8) & (0xFF));	// scale factor 10
	tempData[4] = (_coeff2 & 0xFF);			// scale factor 11
	tempData[5] = ((_coeff2>>8) & (0xFF));	// scale factor 11
	tempData[6] = (_coeff3 & 0xFF);			// scale factor 12
	tempData[7] = ((_coeff3>>8) & (0xFF));	// scale factor 12
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 8, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBsetFTCoefficient2(unsigned char _canch, unsigned char _bno, int _coeff1, int _coeff2, int _coeff3)
{
	// MsgID		Byte0	Byte1	Byte2	Byte3	Byte4	Byte5	Byte6	Byte7
	// CMD_TXDF		BNO		0xA2	SF20	SF20	SF21	SF21	SF22	SF22
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;						// board no.
	tempData[1] = 0xA2;						// command
	tempData[2] = (_coeff1 & 0xFF);			// scale factor 20
	tempData[3] = ((_coeff1>>8) & (0xFF));	// scale factor 20
	tempData[4] = (_coeff2 & 0xFF);			// scale factor 21
	tempData[5] = ((_coeff2>>8) & (0xFF));	// scale factor 21
	tempData[6] = (_coeff3 & 0xFF);			// scale factor 22
	tempData[7] = ((_coeff3>>8) & (0xFF));	// scale factor 22
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 8, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBsetInclinometerSF(unsigned char _canch, unsigned char _bno, int _sf1, int _sf2, int _sf3)
{
	// MsgID		Byte0	Byte1	Byte2	Byte3	Byte4	Byte5	Byte6	Byte7
	// CMD_TXDF		BNO		0xA5	SF0		SF0		SF1		SF1		SF2		SF2
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;						// board no.
	tempData[1] = 0xA5;						// command
	tempData[2] = (_sf1 & 0xFF);			// scale factor 20
	tempData[3] = ((_sf1>>8) & (0xFF));		// scale factor 20
	tempData[4] = (_sf2 & 0xFF);			// scale factor 21
	tempData[5] = ((_sf2>>8) & (0xFF));		// scale factor 21
	tempData[6] = (_sf3 & 0xFF);			// scale factor 22
	tempData[7] = ((_sf3>>8) & (0xFF));		// scale factor 22
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 8, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBsetFTBoardNumberFilterFrequency(unsigned char _canch, unsigned char _bno, unsigned char _newbno, unsigned int _freq)
{
	// MsgID		Byte0	Byte1	Byte2		Byte3	Byte4
	// CMD_TXDF		BNO		0xA8	NEW_BNO		FREQ	FREQ
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;						// board no.
	tempData[1] = 0xA8;						// command
	tempData[2] = _newbno;					// new board number
	tempData[4] = (_freq & 0xFF);		// low-pass filter(1st order) cut-off frequency
	tempData[5] = ((_freq>>8) & (0xFF));	// low-pass filter(1st order) cut-off frequency
	// cut-off frequency = _freq/10
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 8, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBinitializeFTBoard(unsigned char _canch, unsigned char _bno)
{
	// MsgID		Byte0	Byte1	Byte2
	// CMD_TXDF		BNO		0xFA	0xAA	
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;						// board no.
	tempData[1] = 0xFA;						// command
	tempData[2] = 0xAA;						// command
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 3, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBrequestFTParameter(unsigned char _canch, unsigned char _bno, unsigned char _para)
{
	// MsgID		Byte0	Byte1	Byte2
	// CMD_TXDF		BNO		0x24	_para	
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;						// board no.
	tempData[1] = 0x24;						// command
	tempData[2] = _para;					// parameter request
	// _para = 0x01 : SF00 SF01 SF02 FREQ
	// _para = 0x02 : SF10 SF11 SF12
	// _para = 0x03 : SF20 SF21 SF22
	// _para = 0x04 : SFI0 SFI1 SFI2
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 3, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBrequestFTSensorData(unsigned char _canch, unsigned char _sbno, unsigned char _mode)
{
	// MsgID				Byte0	Byte1
	// SEND_SENSOR_TXDF		_sbno	_mode	
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _sbno;					// sensor board no. if _sbno = 0xFF : all sensor boards
	tempData[1] = _mode;					// requested data mode
	// _mode = 0x00 : request FT and tilt in digit
	// _mode = 0x02 : request FT and tilt with scale
	// _mode = 0x03 : request FT with scale and tilt in digit
	// _mode = 0x04 : request FT in digit and tilt with scale
	// _mode = 0x11 : request FT in digit
	// _mode = 0x12 : request FT with scale
	// _mode = 0x21 : request tilt in digit
	// _mode = 0x22 : request tilt with scale
	// _mode = 0x13 : request gyro and temperature 
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 2, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBnulllIMUSensor(unsigned char _canch, unsigned char _bno)
{
	// MsgID		Byte0	Byte1
	// CMD_TXDF		BNO		0x81
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;			// board no.
	tempData[1] = 0x81;			// command
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 2, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBrequestCalibration(unsigned char _canch, unsigned char _bno)
{
	// MsgID		Byte0	Byte1
	// CMD_TXDF		BNO		0x82
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;			// board no.
	tempData[1] = 0x82;			// command
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 2, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBrequestIMUParameter(unsigned char _canch, unsigned char _bno, unsigned char _para)
{
	// MsgID		Byte0	Byte1	Byte2
	// CMD_TXDF		BNO		0x24	_para	
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;						// board no.
	tempData[1] = 0x24;						// command
	tempData[2] = _para;					// parameter request
	// _para = 0x01 : ACC_X_GAIN ACC_Y_GAIN ACC_Z_GAIN
	// _para = 0x02 : ACC_X_BIOS ACC_Y_BIOS ACC_Z_BIOS
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 3, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBrequestIMUSensorData(unsigned char _canch, unsigned char _sbno)
{
	// MsgID				Byte0	Byte1
	// SEND_SENSOR_TXDF		_sbno	0x00
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _sbno;					// sensor board no.
	tempData[1] = 0x00;					// requested data mode
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 2, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBsetSwitchFunction(unsigned char _canch, unsigned char _bno, unsigned char _func)
{
	// MsgID		Byte0	Byte1	Byte2
	// CMD_TXDF		BNO		0x81	_func	
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;						// board no.
	tempData[1] = 0x81;						// command
	tempData[2] = _func;					// function
	// _func = 0x00 : time reset to zero
	// _func = 0x01 : turn on sensor 12V
	// _func = 0x02 : turn off sensor 12V
	// _func = 0x04 : execute power on sequence to turn on 48V
	// _func = 0x05 : execute power off sequence to turn off 48V
	// _func = 0x07 : enable beeper
	// _func = 0x08 : disable beeper
	// _func = 0x0A : turn on PC1
	// _func = 0x0B : turn off PC1
	// _func = 0x0C : turn on PC2
	// _func = 0x0D : turn off PC2
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 3, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBrequestAlram(unsigned char _canch, unsigned char _bno, unsigned char _alram)
{
	// MsgID		Byte0	Byte1	Byte2
	// CMD_TXDF		BNO		0x81	_alram	
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;						// board no.
	tempData[1] = 0x82;						// command
	tempData[2] = _alram;					// alram mode
	// _alram = 0x00 : alram off
	// _alram = 0x01 : alram 1
	// _alram = 0x02 : alram 2
	// _alram = 0x03 : alram 3
	// _alram = 0x04 : alram 4
	// _alram = else : beeps for "else*100 msec"
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 3, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBrequestVoltageCurrent(unsigned char _canch, unsigned char _bno)
{
	// MsgID		Byte0	Byte1
	// CMD_TXDF		BNO		0xE0
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;				// board no.
	tempData[1] = 0xE0;				// command
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 2, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBrequestStatus(unsigned char _canch, unsigned char _bno)
{
	// MsgID		Byte0	Byte1
	// CMD_TXDF		BNO		0xE1
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;				// board no.
	tempData[1] = 0xE1;				// command
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 2, 0);
	
	if(result == 0x00) return true;
	else return false;
}

// !--------------------------APIs CDI PROG--------------------------!
bool RBJointCTMPWMCommand2ch(unsigned int _canch, unsigned int _bno, int _duty1, int _duty2, unsigned int _zeroduty)
{
	// MsgID		Byte0	Byte1	Byte2		Byte3	Byte4	Byte5	Byte6
	// CMD_TXDF		BNO		0x0D	_zeroduty	DIR0	DUTY0	DIR1	DUTY1
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;							// board no.
	tempData[1] = 0x1D;							// command, CDI edition, for CTM (BackEMF compensated), duty 0~1023 digit
	tempData[2] = _zeroduty;					// Zero Duty
	// _zeroduty = 0x00 : Enforce zero duty to stop motor. Back EMF will break the motor
	// _zeroduty = 0x01 : Pulse out to run motor in specified PWM duty and direction
	if(_duty1 >= 0)  tempData[3] =  0x01;		// 1st motor direction : CCW
	else tempData[3] = 0x00;					// 1st motor direction : CW
	if(_duty2 >= 0)  tempData[3] = (0x01<<4) | (tempData[3]);		// 2nd motor direction : CCW
	else tempData[3] = (0x00<<4) | (tempData[3]);					// 2nd motor direction : CW
	
	tempData[4] = (unsigned int)(abs(_duty1)>>8 & (0xFF));	// 1st motor duty
	tempData[5] = (unsigned int)(abs(_duty1) & (0xFF));	// 1st motor duty
	tempData[6] = (unsigned int)(abs(_duty2)>>8 & (0xFF));	// 2nd motor duty
	tempData[7] = (unsigned int)(abs(_duty2) & (0xFF));	// 1st motor duty
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 8, 0);
	
	if(result == 0x00) return true;
	else return false;
}
// !-----------------------------------------------------------------!


//-------------------------------------------------------- Inhyeok
bool RBpwmCommandHR2ch(unsigned char _canch, unsigned char _bno, int _duty1, int _duty2, unsigned char _zeroduty) // High resolution, 0.1%
{
	// MsgID		Byte0	Byte1	Byte2		Byte3	Byte4	Byte5	Byte6
	// CMD_TXDF		BNO		0x0D	0x03		DIR0	DUTY0	DIR1	DUTY1
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	unsigned short stemp;
	
	// CAN message data assign
	tempData[0] = _bno;							// board no.
	tempData[1] = 0x0D;							// command
	if(_zeroduty == 0)
		tempData[2] = 0;
	else if(_zeroduty == 0x04)		
		tempData[2] = 0x04;						// High Resolution PWM override in position control mode
	else
		tempData[2] = 0x03;						// High Resolution PWM mode
	
	stemp = (unsigned short)(abs(_duty1));	// 1st motor duty
	if(stemp > 1000)
		stemp = 1000;
	tempData[3] = (unsigned char)((stemp & 0x0F00) >> 4);  // 1st motor direction : CCW

	if(_duty1 < 0)  		 
		tempData[3] |= 0x01;	// 1st motor direction : CW
	tempData[4] = (unsigned char)(stemp & 0x00FF);
	
	stemp = (unsigned short)(abs(_duty2));	// 1st motor duty
	if(stemp > 1000)
		stemp = 1000;
	tempData[5] = (unsigned char)((stemp & 0x0F00) >> 4);  // 2nd motor direction : CCW
	if(_duty2 < 0)  
		tempData[5] |= 0x01;					// 2nd motor direction : CW
	tempData[6] = (unsigned char)(stemp & 0x00FF);	// 2nd motor duty
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 7, 0);
	
	if(result == 0x00) 
		return true;
	else 
		return false;
}


//-------------------------------------------------------- Inhyeok
bool RBsetSwitchingMode(unsigned char _canch, unsigned char _bno, char _mode) // switching mode of H-bridge, complementary or non-complementary
{
	// MsgID		Byte0	Byte1	Byte2		
	// CMD_TXDF		BNO		0x13	_mode
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];

	// CAN message data assign
	tempData[0] = _bno;							// board no.
	tempData[1] = 0x13;							// command
	tempData[2] = _mode;		//  (0x00 : complementary, 0x01 : not complementary)
	
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 3, 0);
	
	if(result == 0x00) 
		return true;
	else 
		return false;
}

//-------------------------------------------------------- Inhyeok
float RBamp2duty2ch(float amp) // return a PWM duty to generate the amp for 2ch bldc controller
{
	int i;	
	const float duty_table[13] = {0.0f, 0.08f, 0.1f, 0.12f, 0.14f, 0.16f, 0.18f, 0.2f, 0.22f, 0.24f, 0.26f, 0.28f, 0.3f};  
	const float amp_table[13] =  { 0.f,  0.1f, 0.2f,  0.5f,  0.8f,  1.2f,  1.7f,  2.f,  2.5f,   3.f,   4.f,  4.5f,  5.f};
	const int n_table = 13;	

	for(i=0; i<n_table-1; i++)
	{
		if((fabs(amp)>=amp_table[i])&&(fabs(amp)<amp_table[i+1]))
			break;
	}
	
	if(fabs(amp) >= amp_table[n_table-1])
	{
		if(amp >= 0.f)
			return duty_table[n_table-1];		
		else
			return -duty_table[n_table-1];		
	}
	else
	{
		if(amp >= 0.f)
			return (((float)fabs(amp)-amp_table[i])/(amp_table[i+1]-amp_table[i])*(duty_table[i+1]-duty_table[i]) + duty_table[i]); 
		else
			return -(((float)fabs(amp)-amp_table[i])/(amp_table[i+1]-amp_table[i])*(duty_table[i+1]-duty_table[i]) + duty_table[i]); 
	}
}

bool RBgainOverrideHR(unsigned char _canch, unsigned char _bno, unsigned int _override1, unsigned int _override2, unsigned int _duration) //inhyeok, High Resolution (0.1%)
{
	// MsgID		Byte0	Byte1	Byte2	Byte3	Byte4		Byte5		Byte6	Byte7
	// CMD_TXDF		BNO		0x71	OVER1	OVER2	DURATION	DURATION	OVER1	OVER2
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	
	// CAN message data assign
	tempData[0] = _bno;							// board no.
	tempData[1] = 0x71;							// command
	tempData[2] = (_override1&0xFF);			// override value(0.1%) for 1st channel
	tempData[3] = (_override2&0xFF);			// override value(0.1%) for 2nd channel
	tempData[4] = (_duration & 0xFF);			// duration in msec
	tempData[5] = ((_duration>>8) & (0xFF));	// duration in msec
	tempData[6] = ((_override1>>8) & (0x0F));	// override value(0.1%) for 1st channel
	tempData[7] = ((_override2>>8) & (0x0F));	// override value(0.1%) for 2nd channel

	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 8, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBsetFrictionParameter(unsigned char _canch, unsigned char _bno, unsigned int _vel_saturation1, unsigned int _amp_compen1, unsigned int _vel_saturation2, unsigned int _amp_compen2) // inhyeok
{
	// MsgID		Byte0	Byte1	Byte2				Byte3				Byte4						Byte5				Byte6				Byte7
	// CMD_TXDF		BNO		0xB0	VEL_Saturation1		VEL_Saturation1		Ampere_Compensation1(mA)	VEL_Saturation2		VEL_Saturation2		Ampere_Compensation2(mA)

	int result;
	
	// CAN message data
	unsigned char tempData[8];

	
	// CAN message data assign
	tempData[0] = _bno;							// board no.
	tempData[1] = 0xB0;							// command

	tempData[2] = _vel_saturation1 & 0xFF;
	tempData[3] = (_vel_saturation1>>8) & 0xFF;
	tempData[4] = _amp_compen1 & 0xFF;
	tempData[5] = _vel_saturation2 & 0xFF;
	tempData[6] = (_vel_saturation2>>8) & 0xFF;
	tempData[7] = _amp_compen2 & 0xFF;

	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 8, 0);
	
	if(result == 0x00) return true;
	else return false;
}

bool RBenableFrictionCompensation(unsigned char _canch, unsigned char _bno, unsigned char _enable1, unsigned char _enable2) // inhyeok
{
	// MsgID		Byte0	Byte1	Byte2
	// CMD_TXDF		BNO		0xB1	ENABLE

	int result;
	
	// CAN message data
	unsigned char tempData[8];

	
	// CAN message data assign
	tempData[0] = _bno;							// board no.
	tempData[1] = 0xB1;							// command

	tempData[2] = (_enable1&0x01) | ((_enable2&0x01)<<1);
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 3, 0);
	
	if(result == 0x00) return true;
	else return false;
}


bool RBsetMaxDuty(unsigned char _canch, unsigned char _bno, unsigned char _duty1, unsigned char _duty2, unsigned char _duty3) // inhyeok
{
	// MsgID		Byte0	Byte1	Byte2	Byte3	Byte4
	// CMD_TXDF		BNO		0xB2	%duty1	%duty2	%duty3

	int result;
	
	// CAN message data
	unsigned char tempData[8];

	
	// CAN message data assign
	tempData[0] = _bno;							// board no.
	tempData[1] = 0xB2;							// command

	tempData[2] = _duty1&0xFF;
	tempData[3] = _duty2&0xFF;
	tempData[4] = _duty3&0xFF;
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 5, 0);
	
	if(result == 0x00) return true;
	else return false;
}

