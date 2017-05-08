#include "DynRobinBody.h"
#include "SimplePIDControler.h"

dMass additionalMass[5];

static dReal DT_local;



DynRobinBody::DynRobinBody(dReal DT) :m_world(0), m_space(0), joint(0), isInit(false) {

	body_dimmensions[0] = 0.30; 
	body_dimmensions[1] = 0.15; 
	body_dimmensions[2] = 0.005; 
	DT_local=DT;

	for (int i = 0; i < 2; i++)	PID[i] = new SimplePIDControler(KP, KI, KD, DT, EPSILON, MIN, MAX);
}

DynRobinBody::~DynRobinBody()
{
};

void DynRobinBody::Init(dWorldID world, dSpaceID space, dReal dimmensions[3], dReal masses[4]) {
	dMass mass;
	m_world = world;
	m_space = space;

	body_dimmensions[0] = dimmensions[0];
	body_dimmensions[1] = dimmensions[1];
	body_dimmensions[2] = dimmensions[2];

	//each block has volumen  body_dimmensions[0]/2 * body_dimmensions[1]/2*body_dimmensions[2]/2
	dReal DENSITY[4];
	
	dReal volumen = body_dimmensions[0] / 3 * body_dimmensions[1] / 2
			* body_dimmensions[2] / 2;
	for (int i = 0; i < 4; i++)
		DENSITY[i] = masses[i] / volumen;

	if (!world || !space)
		return;

	// def. komada

	link[0].body = dBodyCreate(m_world);
	dBodySetPosition(link[0].body, body_dimmensions[0]*(1/3), 0, 0);
	dMassSetZero(&mass);
	dMassSetBoxTotal(&mass, masses[0], body_dimmensions[0] / 4, body_dimmensions[1]/2, body_dimmensions[2]);
	dMassTranslate(&mass, body_dimmensions[0]*(1/3), 0, 0);
	dBodySetMass(link[0].body, &mass);

	link[1].body = dBodyCreate(m_world);
	dBodySetPosition(link[1].body, 0, 0, 0);
	dMassSetZero(&mass);
	dMassSetBoxTotal(&mass, masses[1], body_dimmensions[0]/7, body_dimmensions[1]/4, body_dimmensions[2]);
	dBodySetMass(link[1].body, &mass);

	link[2].body = dBodyCreate(m_world);
	dBodySetPosition(link[2].body, -body_dimmensions[0]*(1/3), 0, 0);
	dMassSetZero(&mass);
	dMassSetBoxTotal(&mass, masses[2], body_dimmensions[0] / 4, body_dimmensions[1]/2, body_dimmensions[2]);
	dMassTranslate(&mass, -body_dimmensions[0]*(1/3), 0, 0);
	dBodySetMass(link[2].body, &mass);

	// geometrije komada

	link[0].geom[0] = dCreateBox(space, body_dimmensions[0]/4, body_dimmensions[1]/3, body_dimmensions[2]);
	dGeomSetBody(link[0].geom[0], link[0].body);
	dGeomSetOffsetPosition(link[0].geom[0], body_dimmensions[0]/2, 0, 0);
	dGeomSetData(link[0].geom[0], (void*) "white");

	link[1].geom[0] = dCreateBox(space, body_dimmensions[0]/6, body_dimmensions[1]/7, body_dimmensions[2]);
	dGeomSetBody(link[1].geom[0], link[1].body);
	dGeomSetOffsetPosition(link[1].geom[0], 0, 0, 0);
	dGeomSetData(link[1].geom[0], (void*) "white");

	link[2].geom[0] = dCreateBox(space, body_dimmensions[0]/4, body_dimmensions[1]/3, body_dimmensions[2]);
	dGeomSetBody(link[2].geom[0], link[2].body);
	dGeomSetOffsetPosition(link[2].geom[0], -body_dimmensions[0]/2, 0, 0);
	dGeomSetData(link[2].geom[0], (void*) "white");

	// zglobovi kralježnice
	dReal kp1 = 20000000;
	dReal kd1 = 2;

	// prednji spine zglob
	spine[0] = dJointCreateHinge(m_world, 0);
	dJointAttach(spine[0], link[1].body, link[0].body);
	dJointSetHingeAnchor(spine[0], body_dimmensions[0]/4, 0.0, 0.0);
	dJointSetHingeAxis(spine[0], 0, 1, 0);
	dJointSetHingeParam(spine[0], dParamStopCFM, 1 / (DT_local * kp1 + kd1));
	dJointSetHingeParam(spine[0], dParamStopERP, (DT_local * kp1) / (DT_local * kp1 + kd1));



#ifdef Kicma

	// zadnji spine zglob
	spine[1] = dJointCreateHinge(m_world, 0);
	dJointAttach(spine[1], link[1].body, link[2].body);
	dJointSetHingeAnchor(spine[1], -body_dimmensions[0]/4, 0.0, 0.0);
	dJointSetHingeAxis(spine[1], 0, 0, 1);
	dJointSetHingeParam(spine[1], dParamStopCFM, 1 / (DT_local * kp1 + kd1));
	dJointSetHingeParam(spine[1], dParamStopERP, (DT_local * kp1) / (DT_local * kp1 + kd1));



#else

	// zadnji spine zglob
	spine[1] = dJointCreateHinge(m_world, 0);
	dJointAttach(spine[1], link[1].body, link[2].body);
	dJointSetHingeAnchor(spine[1], -body_dimmensions[0]/4, 0.0, 0.0);
	dJointSetHingeAxis(spine[1], 0, 1, 0);
	dJointSetHingeParam(spine[1], dParamStopCFM, 1 / (DT_local * kp1 + kd1));
	dJointSetHingeParam(spine[1], dParamStopERP, (DT_local * kp1) / (DT_local * kp1 + kd1));
#endif

	//motori kralježnice

	spine[2] = dJointCreateAMotor(m_world,0);
	dJointAttach(spine[2],link[1].body, link[0].body);
	dJointSetAMotorNumAxes(spine[2], 3);
	dJointSetAMotorAxis(spine[2], 0, 1, 1, 0, 0);
	dJointSetAMotorAxis(spine[2], 1, 1, 0, 1, 0);
	dJointSetAMotorAxis(spine[2], 2, 1, 0, 0, 1);
	dJointSetAMotorParam(spine[2], dParamVel1, 0);
	dJointSetAMotorParam(spine[2], dParamVel2, 0);
	dJointSetAMotorParam(spine[2], dParamVel3, 0);

	spine[3] = dJointCreateAMotor(m_world,0);
	dJointAttach(spine[3],link[1].body, link[2].body);
	dJointSetAMotorNumAxes(spine[3], 3);
	dJointSetAMotorAxis(spine[3], 0, 1, 1, 0, 0);
	dJointSetAMotorAxis(spine[3], 1, 1, 0, 1, 0);
	dJointSetAMotorAxis(spine[3], 2, 1, 0, 0, 1);
	dJointSetAMotorParam(spine[3], dParamVel1, 0);
	dJointSetAMotorParam(spine[3], dParamVel2, 0);
	dJointSetAMotorParam(spine[3], dParamVel3, 0);

	isInit = true;
	initFeedbackMeasure();
}

void DynRobinBody::Translate(dReal ax, dReal ay, dReal az) {
	if (!isInit)
		return;

	const dReal* R1;

	for (int i = 0; i < _NB_OF_BODY_COMP; i++) {
		R1 = dBodyGetPosition(link[i].body);
		dBodySetPosition(link[i].body, R1[0] + ax, R1[1] + ay, R1[2] + az);

	}
}

void DynRobinBody::Rotate(dReal ax, dReal ay, dReal az, dReal angle) {
	if (!isInit)
		return;

	dMatrix3 R;
	dRFromAxisAndAngle(R, ax, ay, az, angle);
	dBodySetRotation(link[0].body, R);
	// ?
}

void DynRobinBody::SetSpineRef(dReal q[2]){
	double cq[2];
	double PIDout[2];
	double Speed=7800;

	for(int i=0;i<2;i++) {
		cq[i]=dJointGetHingeAngle(spine[i]);
		PIDout[i]=PID[i]->PIDcal(q[i],cq[i]);
		if(q[i]<-1 && cq[i]>1) cq[i] = cq[i]- 2*M_PI;
		if(q[i]>1 && cq[i]<-1) cq[i] = cq[i]+ 2*M_PI;
		if(PIDout[i]>Speed*0.1047){
			PIDout[i]= Speed*0.1047;
		}else if (PIDout[i]<-Speed*0.1047){
			PIDout[i]=-Speed*0.1047;
		}
#ifdef Kicma
		
		if (i==0){
		dJointSetAMotorParam(spine[i+2], dParamVel1 , 0);
		dJointSetAMotorParam(spine[i+2], dParamVel2 , PIDout[i]*2);
		dJointSetAMotorParam(spine[i+2], dParamVel3 , 0);
		dJointSetAMotorParam(spine[i+2], dParamFMax1, 70);
		dJointSetAMotorParam(spine[i+2], dParamFMax2, 70);
		dJointSetAMotorParam(spine[i+2], dParamFMax3, 70);	
		}

		else {
		dJointSetAMotorParam(spine[i+2], dParamVel1 , 0);
		dJointSetAMotorParam(spine[i+2], dParamVel2 , 0);
		dJointSetAMotorParam(spine[i+2], dParamVel3 , PIDout[i]*2);
		dJointSetAMotorParam(spine[i+2], dParamFMax1, 70);
		dJointSetAMotorParam(spine[i+2], dParamFMax2, 70);
		dJointSetAMotorParam(spine[i+2], dParamFMax3, 70);	
		}


#else
		dJointSetAMotorParam(spine[i+2], dParamVel1 , 0);
		dJointSetAMotorParam(spine[i+2], dParamVel2 , PIDout[i]*2);
		dJointSetAMotorParam(spine[i+2], dParamVel3 , 0);
		dJointSetAMotorParam(spine[i+2], dParamFMax1, 70);
		dJointSetAMotorParam(spine[i+2], dParamFMax2, 70);
		dJointSetAMotorParam(spine[i+2], dParamFMax3, 70);
		
#endif
	
	}
}

void DynRobinBody::UpdateBodyState() {
	const dReal* Dummy;
	dVector3 result;

	BodyEnergy = GetBodyEnergy();

	Dummy = dBodyGetPosition(link[0].body);
	memcpy(BodyFront,Dummy, 3*sizeof(dReal));

	Dummy = dBodyGetPosition(link[1].body);
	memcpy(BodyPosition,Dummy, 3*sizeof(dReal));

	Dummy = dBodyGetPosition(link[2].body);
	memcpy(BodyBack,Dummy, 3*sizeof(dReal));

	Dummy = dBodyGetRotation(link[1].body);
	BodyRotationX[0]=Dummy[0];BodyRotationX[1]=Dummy[4];BodyRotationX[2]=Dummy[8];
	BodyRotationY[0]=Dummy[1];BodyRotationY[1]=Dummy[5];BodyRotationY[2]=Dummy[9];
	BodyRotationZ[0]=Dummy[2];BodyRotationZ[1]=Dummy[6];BodyRotationZ[2]=Dummy[10];
	
	Dummy = dBodyGetRotation(link[1].body);
	BodyOrientation[0] = atan2(Dummy[9], Dummy[10]);	//phi
	BodyOrientation[1] = asin(-Dummy[8]);		//theta
	BodyOrientation[2] = atan2(Dummy[4], Dummy[0]);	//greek Y

	Dummy =  dBodyGetLinearVel(link[1].body); 
	memcpy(BodySpeedLinGlobal,Dummy, 3*sizeof(dReal));

	Dummy =  dBodyGetAngularVel(link[1].body);
	memcpy(BodySpeedAngGlobal,Dummy, 3*sizeof(dReal));
	
	Dummy =  dBodyGetAngularVel(link[1].body); 
	dBodyVectorFromWorld(link[0].body, Dummy[0], Dummy[1], Dummy[2], result);
	memcpy(BodySpeedAngLocal,Dummy, 3*sizeof(dReal));

	Dummy =  dBodyGetLinearVel(link[1].body); 
	dBodyVectorFromWorld(link[0].body, Dummy[0], Dummy[1], Dummy[2], result);
	memcpy(BodySpeedLinLocal,Dummy, 3*sizeof(dReal));

}

double DynRobinBody::GetBodyEnergy(){
	double FrontSpineSpeed, BackSpineSpeed;
	dJointFeedback *FrontSpineFeedback, *BackSpineFeedback;
	FrontSpineFeedback	= dJointGetFeedback(spine[2]);
	BackSpineFeedback	= dJointGetFeedback(spine[3]);
	FrontSpineSpeed		= dJointGetHingeAngleRate(spine[0]);
	BackSpineSpeed		= dJointGetHingeAngleRate(spine[1]);
	
	//  u inicijalizaciji fali SetFeedback
	return FrontSpineFeedback->t1[1]*FrontSpineSpeed+BackSpineFeedback->t1[1]*BackSpineSpeed;
	
}

int DynRobinBody::initFeedbackMeasure() 
{
	if (!isInit)
		return 0;

	for (int i = 2; i < 4 ; i++) 
	{
		if (spine[i])
			dJointSetFeedback(spine[i], &feedBack[i]);
		else
			return 0;
	}

	return 1;
}