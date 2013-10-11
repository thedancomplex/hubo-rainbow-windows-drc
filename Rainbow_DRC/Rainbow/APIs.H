// For motor controllers and general functions //
//------------------------------------------------------------------------------------------------//
// Set and request board information
bool RBsetAndRequestBoardInformation(unsigned char _canch, unsigned char _bno, unsigned char _canr);

// Request board status and error flags
bool RBrequestBoardStatus(unsigned char _canch, unsigned char _bno);

// Request encoder position
bool RBrequestEncoderPosition(unsigned char _canch, unsigned char _bno, unsigned char _fes, unsigned char continuous); //inhyeok

// Request current value
bool RBrequestCurrentValue(unsigned char _canch, unsigned char _bno);

// Reset encoder to zero
bool RBResetEncoderZero(unsigned char _canch, unsigned char _bno, unsigned char _ch);

// Set 1st channel motor position control gain
bool RBsetMotorPositionGain0(unsigned char _canch, unsigned char _bno, int _kp, int _ki, int _kd);

// Set 2nd channel motor position control gain
bool RBsetMotorPositionGain1(unsigned char _canch, unsigned char _bno, int _kp, int _ki, int _kd);

// Set 1st channel motor current control gain
bool RBsetMotorCurrentGain0(unsigned char _canch, unsigned char _bno, int _kp, int _ki, int _kd);

// Set 2nd channel motor current control gain
bool RBsetMotorCurrentGain1(unsigned char _canch, unsigned char _bno, int _kp, int _ki, int _kd);

// FET driver enable/disable
bool RBenableFETDriver(unsigned char _canch, unsigned char _bno, unsigned char _enable);

// Openloop PWM duty command in percent(%) for 1 or 2 channel motor controller
bool RBpwmCommand2ch(unsigned char _canch, unsigned char _bno, int _duty1, int _duty2, unsigned char _zeroduty);

// Openloop PWM duty command in percent(%) for 3 channel motor controller
bool RBpwmCommand3ch(unsigned char _canch, unsigned char _bno, int _duty1, int _duty2, int _duty3, unsigned char _zeroduty);

// Openloop PWM duty command in percent(%) for 5 channel motor controller
bool RBpwmCommand5ch(unsigned char _canch, unsigned char _bno, int _duty1, int _duty2, int _duty3, int _duty4, int _duty5, unsigned char _zeroduty);

// Turn on the feedback control
bool RBenableFeedbackControl(unsigned char _canch, unsigned char _bno);

// Turn off the feedback control
bool RBdisableFeedbackControl(unsigned char _canch, unsigned char _bno);

// Set control mode
bool RBsetControlMode(unsigned char _canch, unsigned char _bno, unsigned char _mode);

// Find the limit switch and go to the offset position
bool RBfindLimitGoOffset(unsigned char _canch, unsigned char _bno, unsigned char _ch, unsigned char _mode, unsigned char _dir, long _offset1, long _offset2);

// Set deadzone
bool RBsetDeadzone(unsigned char _canch, unsigned char _bno, unsigned char _ch, unsigned char _deadzone);

// Set home search parameter
bool RBsetHomeSearchParameter(unsigned char _canch, unsigned char _bno, unsigned char _ch, unsigned char _rotlimit, unsigned char _dir, long _offset);

// Set encoder resolution
bool RBsetEncoderResolution(unsigned char _canch, unsigned char _bno, unsigned char _ch, unsigned int _res);

// Set maximum acceleration and velocity
bool RBsetMaxAccVel(unsigned char _canch, unsigned char _bno, unsigned char _ch, unsigned int _acc, unsigned int _vel);

// Set lower position limit
bool RBsetLowerPosLimit(unsigned char _canch, unsigned char _bno, unsigned char _ch, long _limit, unsigned char _mode);

// Set upper position limit
bool RBsetUpperPosLimit(unsigned char _canch, unsigned char _bno, unsigned char _ch, long _limit, unsigned char _mode);

// Set acceleration and velocity during home search
bool RBsetHomeMaxAccVel(unsigned char _canch, unsigned char _bno, unsigned char _ch, unsigned char _acc, unsigned char _vel1, unsigned char _vel2, unsigned char _mode, unsigned char _duty);

// Gain override
bool RBgainOverride(unsigned char _canch, unsigned char _bno, unsigned char _override1, unsigned char _override2, unsigned int _duration);

// Set board number with new board number
bool RBsetBoardNumber(unsigned char _canch, unsigned char _bno, unsigned char _newbno, unsigned char _canr);

// Set JAM and PWM saturation limit
bool RBsetJamPwmSat(unsigned char _canch, unsigned char _bno, unsigned int _jam, unsigned int _sat, unsigned char _satduty, unsigned char _jamduty);

// Set error bound
bool RBsetErrorBound(unsigned char _canch, unsigned char _bno, unsigned int _ierror, unsigned int _berror, unsigned int _teeror);

// Initialize the board
bool RBinitializeBoard(unsigned char _canch, unsigned char _bno);

// Request parameter
bool RBrequestParameter(unsigned char _canch, unsigned char _bno, unsigned char _para);

// Send reference - 2ch board
bool RBsendReference2ch(unsigned char _canch, unsigned char _bno, long _ref1, long _ref2);

// Send reference - 3ch board
bool RBsendReference3ch(unsigned char _canch, unsigned char _bno, char _ref1, char _ref2, char _ref3);

// Send reference - 5ch board
bool RBsendReference5ch(unsigned char _canch, unsigned char _bno, char _ref1, char _ref2, char _ref3, char _ref4, char _ref5);
//------------------------------------------------------------------------------------------------//

// For FT sensors //
//------------------------------------------------------------------------------------------------//
// Request to execute NULL the sensors
bool RBnullFTSensor(unsigned char _canch, unsigned char _bno, unsigned char _mode);

// Set FT sensor decoupling matrix coefficient0
bool RBsetFTCoefficient0(unsigned char _canch, unsigned char _bno, int _coeff1, int _coeff2, int _coeff3);

// Set FT sensor decoupling matrix coefficient1
bool RBsetFTCoefficient1(unsigned char _canch, unsigned char _bno, int _coeff1, int _coeff2, int _coeff3);

// Set FT sensor decoupling matrix coefficient2
bool RBsetFTCoefficient2(unsigned char _canch, unsigned char _bno, int _coeff1, int _coeff2, int _coeff3);

// Set inclinometer scale factor
bool RBsetInclinometerSF(unsigned char _canch, unsigned char _bno, int _sf1, int _sf2, int _sf3);

// Set board number and filter frquency
bool RBsetFTBoardNumberFilterFrequency(unsigned char _canch, unsigned char _bno, unsigned char _newbno, unsigned int _freq);

// Initialize with default values
bool RBinitializeFTBoard(unsigned char _canch, unsigned char _bno);

// Request parameters
bool RBrequestFTParameter(unsigned char _canch, unsigned char _bno, unsigned char _para);

// Request FT sensor data
bool RBrequestFTSensorData(unsigned char _canch, unsigned char _sbno, unsigned char _mode);
//------------------------------------------------------------------------------------------------//

// For IMU sensors //
//------------------------------------------------------------------------------------------------//
// Request to execute the sensor nulling
bool RBnulllIMUSensor(unsigned char _canch, unsigned char _bno);

// Request to execute calibration
bool RBrequestCalibration(unsigned char _canch, unsigned char _bno);

// Request parameters
bool RBrequestIMUParameter(unsigned char _canch, unsigned char _bno, unsigned char _para);

// Request IMU sensor data
bool RBrequestIMUSensorData(unsigned char _canch, unsigned char _sbno);
//------------------------------------------------------------------------------------------------//

// For power management board
// Set switch function
bool RBsetSwitchFunction(unsigned char _canch, unsigned char _bno, unsigned char _func);

// Request alram
bool RBrequestAlram(unsigned char _canch, unsigned char _bno, unsigned char _alram);

// Request voltage and current
bool RBrequestVoltageCurrent(unsigned char _canch, unsigned char _bno);

// Request time and status information
bool RBrequestStatus(unsigned char _canch, unsigned char _bno);

// !-----------------------APIs.h CDI PROG--------------------------!
// Send PWM Digit Value Directly
bool RBJointCTMPWMCommand2ch(unsigned int _canch, unsigned int _bno, int _duty1, int _duty2, unsigned int _zeroduty);


// Openloop PWM duty command in permil(0.1%) for 1 or 2 channel motor controller (by Inhyeok)
bool RBpwmCommandHR2ch(unsigned char _canch, unsigned char _bno, int _duty1, int _duty2, unsigned char _zeroduty);

// Set the switching mode of H-bridges : complementary or not complementary  (by Inhyeok)
bool RBsetSwitchingMode(unsigned char _canch, unsigned char _bno, char _mode); 

// return a PWM duty to generate the amp for 2ch BLDC controller, inhyeok
float RBamp2duty2ch(float amp);

// Gain override, high resolution (0.1%), inhyeok
bool RBgainOverrideHR(unsigned char _canch, unsigned char _bno, unsigned int _override1, unsigned int _override2, unsigned int _duration_ms);

// Set joint friction parameters, inhyeok
bool RBsetFrictionParameter(unsigned char _canch, unsigned char _bno, unsigned int _vel_saturation1, unsigned int _amp_compen1, unsigned int _vel_saturation2, unsigned int _amp_compen2);

// Enable or disable joint friction compensation, inhyeok
bool RBenableFrictionCompensation(unsigned char _canch, unsigned char _bno, unsigned char _enable1, unsigned char _enable2);

// Set maximum duty of joints, inhyeok
bool RBsetMaxDuty(unsigned char _canch, unsigned char _bno, unsigned char _duty1, unsigned char _duty2, unsigned char _duty3);