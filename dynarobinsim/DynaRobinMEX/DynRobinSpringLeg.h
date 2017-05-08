#ifndef _DynRobinSpringLeg
#define _DynRobinSpringLeg

#include "ConfigODE.h"
#include "SimplePIDControler.h"

#define TINY_NUMBER 0.000000001
#define TINY TINY_NUMBER

class SimplePIDControler;

class DynRobinSpringLeg
{
	public:
	// ODE
	dWorldID		m_world; 
	dSpaceID		m_space;
	dJointID		joints[9];
	bodyGeo			link[6];
	dJointFeedback	feedBack[9];

	DynRobinSpringLeg(double DT=0.01); // Ovo srediti
	~DynRobinSpringLeg();
	int Init(dWorldID world, dSpaceID space, double Dx, double Dy, double L1,double L2, double flip);
	int flipSide();
	int Translate(dReal ax, dReal ay, dReal az);
	int Rotate(dReal ax, dReal ay, dReal az, dReal angle);
	
	// Stanja
	dReal springLenght;
	dReal StartTime;
	dReal SpringLenght;
	dReal AlphaRef;
	dReal FootForce 			[3];
	dReal FootSpeedLinGlobal	[3];
	dReal FootSpeedAngGlobal	[3];
	dReal LegPosGlobal			[3];
	dReal KneePosGlobal			[3];
	dReal FootPosGlobal			[3];
	dReal JointPos 				[4];
	dReal JointVel 				[4];
	dReal LegRefPos				[3];
	dReal JointRefPos 			[4];
	dReal JointTorque 			[3];
	dReal LegEnergy				   ;
	dReal Contact;

	// Upravljanje
	void	UpdateLegState		();
	void 	SetAlphaRef			(dReal newAlpha);
	void 	SetLegRef			(dReal x, dReal y, dReal z, dReal choice);
	int		setJointTorque		(int jId, dReal torque);
	int		getJointFeedback	(int jId, dJointFeedback* feedBack);
	int		getJointPosition	(int jId, dReal* x);
	int		setJointPosition	(int jId, dReal Position, dReal Speed);
	int		getJointVelocity	(int jId, dReal* dX);
	int		setJointVelocity	(int jId, dReal torque);
	int		initFeedbackMeasure	();
	double	GetLegEnergy		();
	
	dJointID*	getJoints				() {return joints;}	
	dJointID* 	getShoulderJoint 		() {return &(joints[6]);}
	dJointID*	getShoulderJointMotor	() {return &(joints[7]);}

	SimplePIDControler* PID[5];


private:
	double DDx, DDy, DL1, DL2;
	bool isInit;
	int side; //1 initial side, -1 fliped side
};
#endif