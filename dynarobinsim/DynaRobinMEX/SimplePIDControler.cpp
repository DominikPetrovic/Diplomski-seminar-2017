#include "SimplePIDControler.h"
#include "stdio.h"
#include "math.h"

double SimplePIDControler::PIDcal(double setpoint,double actual_position)
{
	static double pre_error = 0;
	static double integral = 0;
	double error;
	double derivative;
	double output;

	
	//Caculate P,I,D
	error = setpoint - actual_position;

	//In case of error too small then stop intergration
	if(fabs(error) > epsilon)
	{
		integral = integral + error*dt;
	}
	derivative = (error - pre_error)/dt;
	output = KP*error + KI*integral + KD*derivative; //!!!

	//Saturation Filter
	if(output > _MAX)
	{
		output = _MAX;
	}
	else if(output < _MIN)
	{
		output = _MIN;
	}
        //Update error
        pre_error = error;
		//printf("P: %d I: %d D: %d set: %d actual: %d\n", KP, KI, KD, setpoint, actual_position);
 return output;
}