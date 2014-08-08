/**********************************************************************************************
 * PID controller - Version 0.1
 * by Massimiliano Mosca
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#ifndef pid_h
#define pid_h

#include "Arduino.h"

class pidController
{
  public:
  	pidController(double*,double*,double*,double,double,double,double);
    void setTuningParameters(double,double,double,double);
    void setOutputLimits(double min, double max);
    void run();
  
  	/* Read only functions */
    double getBias();
    double getKp();
    double getKi();
    double getKd();
 
  private:
  	/* Parameters used in the controller */
  	double _bias; 
    double _Kp; //proportional gain
    double _Ki; //integral gain
    double _Kd; //derivative gain

    /* Input and output variables */
    double *_input;
    double *_output;
    double *_setPoint;
    double _minOutput;
    double _maxOutput;

    /* Variable needed during the computation */
    double _iTerm;  //integral
    double _lastError; //value of the last calculated error
    unsigned long _lastTime; 

};

#endif