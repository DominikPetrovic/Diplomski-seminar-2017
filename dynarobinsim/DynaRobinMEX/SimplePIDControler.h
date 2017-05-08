#ifndef PID_H_
#define PID_H_
#include <math.h>

#include "ConfigODE.h"
//Define parameter
//#define epsilon 0.01
//#define dt 0.01             //100ms loop time
//#define MAX  4                   //For Current Saturation
//#define MIN -4
//#define Kp  0.1
//#define Kd  0.01
//#define Ki  0.005

class SimplePIDControler
{

public:
	explicit SimplePIDControler(const double Kp, const double Ki, const double Kd, const double dT, 
		const double dE, const double MinOut, const double MaxOut):
	Kp(Kp), Ki(Ki), Kd(Kd), dt(dT), epsilon(dE), _MIN(MinOut), _MAX(MaxOut){}

	double PIDcal(double setpoint,double actual_position);

private:
	const double Kp;
	const double Ki;
	const double Kd;
	const double dt;
	const double epsilon;
	const double _MIN;
	const double _MAX;
};
#endif /*PID_H_*/