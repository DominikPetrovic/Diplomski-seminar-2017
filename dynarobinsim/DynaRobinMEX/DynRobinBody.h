#ifndef _DYNROBINBODY
#define _DYNROBINBODY

#include "ConfigODE.h"

#define _NB_OF_BODY_COMP 3
class SimplePIDControler;

class DynRobinBody
{
	public:
	dReal			massOffset[3];
	bodyGeo			link[3];
	dWorldID		m_world;
	dSpaceID		m_space;
	dJointID		joint;
	dJointID		spine[4];
	dJointFeedback	feedBack[4];

	dReal BodyPosition		[3];
	dReal BodyFront			[3];
	dReal BodyBack			[3];
	dReal BodyRotationX		[3];
	dReal BodyRotationY		[3];
	dReal BodyRotationZ		[3];
	dReal BodySpeedLinGlobal[3];
	dReal BodySpeedAngGlobal[3];
	dReal BodySpeedLinLocal [3];
	dReal BodySpeedAngLocal [3];
	dReal BodyOrientation   [3];
	dReal BodyEnergy;
	// Konstruktor dekonstruktor
	DynRobinBody(dReal DT=0.01);
	~DynRobinBody();

	// Standardno
	void Init			(dWorldID world, dSpaceID space, dReal dimmensions[3],dReal masses[4]);
	void Translate		(dReal ax, dReal ay, dReal az);
	void Rotate			(dReal ax, dReal ay, dReal az, dReal angle);

	// upravljanje
	void SetSpineRef(dReal q[2]);
	double GetBodyEnergy();
	SimplePIDControler* PID[2];
	int initFeedbackMeasure(); 
	// Informacije
	void UpdateBodyState();

private:
	bool isInit;
	dReal body_dimmensions[3];
};
#endif