/**********************************************************************************************
 * PID controller - Version 0.1
 * by Massimiliano Mosca
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#include <pid.h>

pidController::pidController(double* input, double* setPoint, double* output, 
	                         double bias, double Kp, double Ki, double Kd)
{
	_input = input;
	_output = output;
	_setPoint = setPoint;
	_bias = bias; 
	_Kp = Kp; 
	_Ki = Ki; 
	_Kd = Kd;
	_iTerm = 0.0;
	_lastError = 0.0;
	_lastTime = millis();
}

void pidController::setTuningParameters(double bias, double Kp, double Ki, double Kd)
{
	_bias = bias; 
	_Kp = Kp; 
	_Ki = Ki; 
	_Kd = Kd;
}

void pidController::run()
{
	unsigned long currentTime = millis();
	unsigned long dTime = currentTime - _lastTime;
	double error = *_setPoint - *_input;
	double dTerm = (error - _lastError) / dTime;
	
	_iTerm += _Ki * error * dTime;
	if(_iTerm > _maxOutput) _iTerm = _maxOutput;
	else if(_iTerm < _minOutput) _iTerm = _minOutput; 
	
	*_output = _bias + _Kp * error + _iTerm + _Kd * dTerm;
	if(*_output > _maxOutput) *_output = _maxOutput;
	else if(*_output < _minOutput) *_output = _minOutput; 
	
	_lastError = error;
	_lastTime = currentTime;
}

void pidController::setOutputLimits(double min, double max)
{
	if(min > max) return;
   	_minOutput = min;
   	_maxOutput = max;

   	if(_iTerm > _maxOutput) _iTerm = _maxOutput;
	else if(_iTerm < _minOutput) _iTerm = _minOutput;
	if(*_output > _maxOutput) *_output = _maxOutput;
	else if(*_output < _minOutput) *_output = _minOutput;
}

double pidController::getBias(){ return  _bias; }
double pidController::getKp()  { return  _Kp; }
double pidController::getKi()  { return  _Ki; }
double pidController::getKd()  { return  _Kd; }