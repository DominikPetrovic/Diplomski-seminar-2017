#ifndef _DYNAROBIN_ODE
#define _DYNAROBIN_ODE

#include "ConfigODE.h"
#include "DynRobinSpringLeg.h"
#include "DynRobinBody.h"

class DynaRobinODE
{
public:
	dReal body_mass[4];
	dReal TimeStep;
	dReal FootSpeedLinLocal	[4][3];
	dReal FootSpeedAngLocal	[4][3];
	dReal FootPosLocal		[4][3];
	dReal FootContactForce	[4][3];
	dReal BodyFrontPosLocal	[3];
	dReal BodyBackPosLocal	[3];
	dReal LegPosLocal		[4][3];
	dReal KneePosLocal		[4][3];
	dReal LegJointPos		[4][3];
	dReal TIME;
	

	// Konstruktor dekonstruktor
	DynaRobinODE(dReal TimeStepVar, dReal _body_mass[4], dReal Visualization);
	~DynaRobinODE();

	void StartODEThread			();
	void stopODESimulation		();
	void StopODEThread			();
	void OneStep				();
	bool SimulationFinished		();
	void UpdateNewRobotState	();
	void GetRobotState			();
	void updateSpringGraphics	();

	// Implementacije simulatora
	void	StopODESimulation	();
	void	MakeRobot			();
	void	InitODE				();

	// Implementacije dobavljanja informacija
	void	SetAlphaReference	(int index, dReal Alpha);
	void	SetLegReference		(int index, dReal x, dReal y, dReal z, dReal choice);
	void	SetSpineReference	(dReal q0, dReal q1);
	

private:
	void LinkBodyWithLeg(double xB, double yB, double Dx, double Dy, double L1, double L2, double HEIGHT_TOTAL);
	bool odeInitOK;
	bool m_stoprequested;
	

};

#endif