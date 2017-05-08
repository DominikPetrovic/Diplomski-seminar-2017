#ifndef _DYNAROBIN_IKIN_H
#define _DYNAROBIN_IKIN_H

#define PI			3.1415
#define FACTOR		1

#include "ALGLIB/AlglibIncludeAll.h"
#include "ALGLIB/AlglibCognitus.h"



class DynaRobinIKin{
	

public:
	DynaRobinIKin();
	~DynaRobinIKin();
	
	vector<double> *FL_last_res;
	vector<double> *FR_last_res; 
	vector<double> *BL_last_res; 
	vector<double> *BR_last_res;

	bool CalculateJacobianInverse(vector<double> FL, vector<double> FR, vector<double> BL, vector<double> BR, vector<double> ksi);
	bool CalculateInverseKinematics(vector<double> FootPosition, double KneeSide, vector<double> *JointPosition);
	bool CalculateDirectKinematics(vector<double> JointPosition, vector<double> *FootPosition);

private:

	double l1;
    double l2;
    double l3;
	double l4;
    double l5;
    double l6;
		
	AlglibCognitusC * agc;
};

#endif
