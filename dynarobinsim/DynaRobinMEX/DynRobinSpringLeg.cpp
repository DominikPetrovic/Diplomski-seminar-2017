#include "DynRobinSpringLeg.h"

static dReal DT_local;

DynRobinSpringLeg::DynRobinSpringLeg(dReal DT) :	m_world(0), m_space(0), isInit(false), side(1) 
{
	memset(joints, 0, sizeof(joints));
	memset(feedBack, 0, sizeof(feedBack));

	for (int i = 0; i < 5; i++)	PID[i] = new SimplePIDControler(KP, KI, KD, DT, EPSILON, MIN, MAX);

	DT_local = DT;
	Contact=0;
}

DynRobinSpringLeg::~DynRobinSpringLeg(){
}


int DynRobinSpringLeg::Init(dWorldID world, dSpaceID space, dReal Dx, dReal Dy, dReal L1, dReal L2, double flip) {

	dMass mass;
	dMatrix3 R;
	m_world = world;
	m_space = space;

	DDx = Dx;
	DDy = Dy;
	DL1 = L1;
	DL2 = L2;

	if (!world || !space) {
		return 0;
	}

	//Ovo je nadkoljenica
	link[0].body = dBodyCreate(m_world);
	dRFromAxisAndAngle(R, 0, 0, 1, PI / 2);
	dBodySetRotation(link[0].body, R);
	dBodySetPosition(link[0].body, 0, 0, 0);
	dMassSetZero(&mass);
	dMassSetBoxTotal(&mass, 0.276, 0.04, 0.04, L1); // Postavljanje mase i centra mase
	dBodySetMass(link[0].body, &mass);

	//Sredisnji dio
	link[0].geom[0] = dCreateBox(space, Dx, Dy, L1-Dy);
	dGeomSetBody(link[0].geom[0], link[0].body);
	dGeomSetOffsetPosition(link[0].geom[0], 0, 0, 0);
	dGeomSetData(link[0].geom[0], (void*) "red");

	//valjak za zglob
	link[0].geom[1] = dCreateCylinder(space, Dy/4, Dx);
	dGeomSetBody(link[0].geom[1], link[0].body);
	dRSetIdentity(R);
	dRFromAxisAndAngle(R, 0, 1, 0, PI / 2);
	dGeomSetOffsetRotation(link[0].geom[1], R);
	dGeomSetOffsetPosition(link[0].geom[1], 0.0, 0.0, L1 / 2);
	dGeomSetData(link[0].geom[1], (void*) "red");

	//Ovo je podkoljenica
	link[1].body = dBodyCreate(m_world);
	dRFromAxisAndAngle(R, 0, 0, 1, PI / 2);
	dBodySetRotation(link[1].body, R);
	dBodySetPosition(link[1].body, 0, 0, -L1 / 2 - L2 / 2);
	dMassSetZero(&mass);
	dMassSetBoxTotal(&mass, 0.276, 0.04, 0.04, L2); // Postavljanje mase i centra mase Masa prije promjene : 0.13
	dBodySetMass(link[1].body, &mass);

	//Sredisnji dio
	link[1].geom[0] = dCreateBox(space, Dx, Dy, L2 - Dy);
	dGeomSetBody(link[1].geom[0], link[1].body);
	dGeomSetOffsetPosition(link[0].geom[0], 0, 0, -Dy);
	dGeomSetData(link[1].geom[0], (void*) "blue");

	//valjak za zglob
	link[1].geom[1] = dCreateCylinder(space, Dy/4, Dx);
	dGeomSetBody(link[1].geom[1], link[1].body);
	dRSetIdentity(R);
	dRFromAxisAndAngle(R, 0, 1, 0, PI / 2);
	dGeomSetOffsetRotation(link[1].geom[1], R);
	dGeomSetOffsetPosition(link[1].geom[1], 0.0, 0.0, L2 / 2);
	dGeomSetData(link[1].geom[0], (void*) "blue");

	// Ovdje dolazi dio za oprugu
	link[2].body = dBodyCreate(m_world);
	dRFromAxisAndAngle(R, 0, 0, 1, PI / 2);
	dBodySetRotation(link[2].body, R);
	dBodySetPosition(link[2].body, 0, 0, -L1 / 2 - L2 - 0.04);
	dMassSetZero(&mass);
	dMassSetBoxTotal(&mass, 0.06, 0.01, 0.01, 0.01); // Postavljanje mase i centra mase
	dBodySetMass(link[2].body, &mass);

	//Sredisnji dio
	link[2].geom[0] = dCreateBox(space, Dx/4, Dy/4, 0.005);
	dGeomSetBody(link[2].geom[0], link[2].body);
	dGeomSetOffsetPosition(link[0].geom[0], 0, 0, 0);
	dGeomSetData(link[2].geom[0], (void*) "green");
	
	//valjak za zglob
	link[2].geom[1] = dCreateCylinder(space, Dy / 4, 0.03);
	dGeomSetBody(link[2].geom[1], link[2].body);
	dRSetIdentity(R);
	dRFromAxisAndAngle(R, 0, 0, 1, PI / 2);
	dGeomSetOffsetRotation(link[2].geom[1], R);
	dGeomSetOffsetPosition(link[2].geom[1], 0.0, 0.0, 0.02);
	dGeomSetData(link[2].geom[1], (void*) "green");

	//stopalo
	link[3].body = dBodyCreate(m_world);
	dRFromAxisAndAngle(R, 0, 0, 1, PI / 2);
	dBodySetRotation(link[3].body, R);
	dBodySetPosition(link[3].body, 0, 0, -L1 / 2 - L2 - 0.05 - 0.059707);
	dMassSetZero(&mass);
	dMassSetBoxTotal(&mass, 0.001, 0.01, 0.01, 0.01); // Postavljanje mase i centra mase
	dBodySetMass(link[3].body, &mass);

	//dodirna povrsina
	link[3].geom[0] = dCreateSphere(space, 0.01);
	dGeomSetBody(link[3].geom[0], link[3].body);
	dGeomSetOffsetPosition(link[3].geom[0], 0, 0, 0);
	dGeomSetData(link[3].geom[0], (void*) "green");

	//valjak na ramenu
	link[4].body=dBodyCreate(m_world);
	dRFromAxisAndAngle(R, 0, 1, 0, PI/2);
	dBodySetRotation(link[4].body, R);
	dBodySetPosition(link[4].body, 0, flip*0.05, L1/2);
	dMassSetZero(&mass);
	dMassSetBoxTotal(&mass, 0.001, 0.01, 0.01, 0.01);
	dBodySetMass(link[4].body, &mass);

	link[4].geom[0]=dCreateCylinder(space, 0.02, 0.04);
	dGeomSetBody(link[4].geom[0], link[4].body);
	dGeomSetOffsetPosition(link[4].geom[0], 0, 0, 0);
	dGeomSetData(link[4].geom[0], (void*) "red");
	
	//stopalo
	link[5].body = dBodyCreate(m_world);
	dRFromAxisAndAngle(R, 0, 0, 1, PI / 2);
	dBodySetRotation(link[5].body, R);
	dBodySetPosition(link[5].body, 0, 0, -L1 / 2 - L2 - 0.05 - 0.07);
	dMassSetZero(&mass);
	dMassSetBoxTotal(&mass, 0.001, 0.01, 0.01, 0.01); // Postavljanje mase i centra mase
	dBodySetMass(link[5].body, &mass);

	//dodirna povrsina
	link[5].geom[0] = dCreateSphere(space, 0.01);
	dGeomSetBody(link[5].geom[0], link[5].body);
	dGeomSetOffsetPosition(link[5].geom[0], 0, 0, 0);
	dGeomSetData(link[5].geom[0], (void*) "red");

	// JOINTOVI

	dReal kp1 = 20000000;
	dReal kd1 = 2;

	// rame
	joints[0] = dJointCreateHinge(m_world, 0);
	dJointAttach(joints[0], link[4].body, link[0].body);
	dJointSetHingeAnchor(joints[0], 0.0, 0.0, L1 / 2);
	dJointSetHingeAxis(joints[0], 0, 1, 0);
	dJointSetHingeParam(joints[0], dParamStopCFM, 1 / (DT_local * kp1 + kd1));
	dJointSetHingeParam(joints[0], dParamStopERP,
			(DT_local * kp1) / (DT_local * kp1 + kd1));

	// koljeno
	joints[1] = dJointCreateHinge(m_world, 0);
	dJointAttach(joints[1], link[0].body, link[1].body);
	dJointSetHingeAnchor(joints[1], 0.0, 0.0, -L1 / 2);
	dJointSetHingeAxis(joints[1], 0, 1, 0);
	dJointSetHingeParam(joints[1], dParamLoStop, -PI / 2 - PI / 6);
	dJointSetHingeParam(joints[1], dParamHiStop, PI / 2 + PI / 6);
	dJointSetHingeParam(joints[1], dParamStopCFM, 1 / (DT_local * kp1 + kd1));
	dJointSetHingeParam(joints[1], dParamStopERP,
			(DT_local * kp1) / (DT_local * kp1 + kd1));
	

	//podkoljenica
	joints[2] = dJointCreateFixed(m_world, 0);
	dJointAttach(joints[2], link[1].body, link[2].body);
	dJointSetFixed(joints[2]);


	//opruga
	joints[3] = dJointCreateSlider(m_world, 0);
	dJointAttach(joints[3], link[2].body, link[3].body);
	dJointSetSliderAxis(joints[3], 0, 0, 1);

	// Postavljanje opruge
	kp1 = 2000;
	kd1 = 1;
	dJointSetSliderParam(joints[3], dParamLoStop, -0.001);
	dJointSetSliderParam(joints[3], dParamHiStop, 0);
	dJointSetSliderParam(joints[3], dParamStopCFM, 1 / (DT_local * kp1 + kd1));
	dJointSetSliderParam(joints[3], dParamStopERP,
			(DT_local * kp1) / (DT_local * kp1 + kd1));

	// rame
	joints[4] = dJointCreateAMotor(m_world,0);
	dJointAttach(joints[4], link[4].body, link[0].body);
	dJointSetAMotorNumAxes(joints[4], 3);
	dJointSetAMotorAxis(joints[4], 0, 1, 1, 0, 0);
	dJointSetAMotorAxis(joints[4], 1, 1, 0, 1, 0);
	dJointSetAMotorAxis(joints[4], 2, 1, 0, 0, 1);
	dJointSetAMotorParam(joints[4], dParamVel1, 0);
	dJointSetAMotorParam(joints[4], dParamVel2, 0);
	dJointSetAMotorParam(joints[4], dParamVel3, 0);

	// koljeno
	joints[5] = dJointCreateAMotor(m_world,0);
	dJointAttach(joints[5],link[0].body, link[1].body);
	dJointSetAMotorNumAxes(joints[5], 3);
	dJointSetAMotorAxis(joints[5], 0, 1, 1, 0, 0);
	dJointSetAMotorAxis(joints[5], 1, 1, 0, 1, 0);
	dJointSetAMotorAxis(joints[5], 2, 1, 0, 0, 1);
	dJointSetAMotorParam(joints[5], dParamVel1, 0);
	dJointSetAMotorParam(joints[5], dParamVel2, 0);
	dJointSetAMotorParam(joints[5], dParamVel3, 0);
	//dJointSetAMotorMode (joints[1], dAMotorUser );


	//podkoljenica

	//valjak rame
	joints[6] = dJointCreateHinge(m_world, 0);
	dJointAttach(joints[6], 0, 0);
	dJointSetHingeAnchor(joints[6], 0.0, flip*0.05, L1/2);
	dJointSetHingeAxis(joints[6], 1, 0, 0);
	dJointSetHingeParam(joints[6], dParamStopCFM, 1 / (DT_local * kp1 + kd1));
	dJointSetHingeParam(joints[6], dParamStopERP,
			(DT_local * kp1) / (DT_local * kp1 + kd1));

	// motor rame
	joints[7] = dJointCreateAMotor(m_world,0);
	dJointAttach(joints[7], 0, 0);
	dJointSetAMotorNumAxes(joints[7], 3);
	
	// motor rame
	joints[8] = dJointCreateFixed(m_world, 0);
	dJointAttach(joints[8], link[3].body, link[5].body);
	dJointSetFixed(joints[8]);


	isInit = true;
	initFeedbackMeasure();
	return 1;
}

void DynRobinSpringLeg::SetAlphaRef(dReal newAlpha) {
	AlphaRef=newAlpha;
	dReal length=0, stiffness=0, damping=0;
	try{
		stiffness=1000*(0.3275*exp(newAlpha*0.01458)+0.3632+4.606e-18*pow(newAlpha,8));
		damping=(3.557*exp(newAlpha*0.005142)-2.64);
		length=0.01*(-0.00000654257167784*newAlpha*newAlpha - 0.01475*newAlpha + 5.933);

		dJointSetSliderParam (joints[3],dParamStopCFM,1/(DT_local*stiffness+damping));
		dJointSetSliderParam (joints[3],dParamStopERP,(DT_local*stiffness)/(DT_local*stiffness+damping));
		setJointPosition(3, 0.0599-length, 99);
	}
	catch (const char* msg ){
	
	
	}

}

void DynRobinSpringLeg::SetLegRef(dReal x, dReal y, dReal z, dReal choice) {
	dReal K1, K2, K3, z0;
	dReal q123, q1, q2, q3, q0;

#ifndef DIRECT
	static dReal PreviousX=0;
	static dReal PreviousY=0;
	static dReal PreviousZ=-0.20;

	// z0 postaje z iz perspektive stare noge
	// iz y računam zakret u ramenu
	q0=atan(y/z);
	z0=z/cos(q0);

	K1 = x;
	K2 = z0;
	q123 = 0;
	if (((DL1 * DL1 + 2 * DL1 * DL2 + DL2 * DL2 - K1 * K1 - K2 * K2)
					/ (-DL1 * DL1 + 2 * DL1 * DL2 - DL2 * DL2 + K1 * K1
					+ K2 * K2))>=0)
	{
		K1 = x;
		K2 = z0;
		
		PreviousX=x;
		PreviousY=y;
		PreviousZ=z0;
	}else {
		K1 = PreviousX;
		K2 = PreviousZ;
	}
	K3 = sqrt(
			(DL1 * DL1 + 2 * DL1 * DL2 + DL2 * DL2 - K1 * K1 - K2 * K2)
					/ (-DL1 * DL1 + 2 * DL1 * DL2 - DL2 * DL2 + K1 * K1
							+ K2 * K2));

	q1 = (-2// promijenio sam predznak q1
			* atan(
					(2 * DL1 * K1
							- choice
									* (DL1 * DL1 + DL2 * DL2 - K1 * K1 - K2 * K2
											- 2 * DL1 * DL2) * K3)
							/ (DL1 * DL1 - 2 * DL1 * K2 - DL2 * DL2 + K1 * K1
									+ K2 * K2)));
	q2 = (choice * 2 * atan(K3));
	
	

	
	LegRefPos[0]=x;
	LegRefPos[1]=y;
	LegRefPos[2]=z;
#else
	q0=x;
	q1=y;
	q2=z;
#endif
	JointRefPos[0]=q0;
	JointRefPos[1]=q1;
	JointRefPos[2]=q2;
	JointRefPos[3]=0;

	setJointPosition(0, q1, 7800);
	setJointPosition(1, q2, 7800);
	setJointPosition(6, q0, 7800);
}

int DynRobinSpringLeg::initFeedbackMeasure() 
{
	if (!isInit)
		return 0;

	for (int i = 4; i < 9 ; i++) 
	{
		if (joints[i])
			dJointSetFeedback(joints[i], &feedBack[i]);
		else
			return 0;
	}

	return 1;
}

int DynRobinSpringLeg::Translate(dReal ax, dReal ay, dReal az) 
{
	if (!isInit)
		return 0;

	const dReal* R1;
	for (int i = 0; i < 5; i++) 
	{
		R1 = dBodyGetPosition(link[i].body);
		dBodySetPosition(link[i].body, R1[0] + ax, R1[1] + ay, R1[2] + az);
	}
	return 1;
}

int DynRobinSpringLeg::setJointTorque(int jId, dReal torque) {
	if (!isInit)
		return 0;
	if (jId < 0)
		return 0;
	if (jId >3 && jId!=6)
		return 0;

	if (joints[jId]) {
		if (jId < 2||jId==6) {
			dJointAddHingeTorque(joints[jId], torque);
		} else {
			dJointAddSliderForce(joints[jId], torque);
		}
		return 1;
	}
	return 0;
}

int DynRobinSpringLeg::getJointFeedback(int jId, dJointFeedback* feedBack) {
	if (initFeedbackMeasure() == 0)
		return 0;

	if(jId < 0 || jId > 8)
		return 0;
	
	dJointFeedback* pfeedBack = dJointGetFeedback(joints[jId]);
	memcpy(feedBack, pfeedBack, sizeof(dJointFeedback));
	return 1;
}

int DynRobinSpringLeg::setJointVelocity(int jId, dReal torque) {
	if (!isInit)
		return 0;
	if (jId < 0 || (jId>4 && jId!=6))
		return 0;

	if (joints[jId]) {
		if (jId < 2 || jId==6) {
			dJointSetHingeParam(joints[jId], dParamVel, torque);
		} else {
			dJointSetSliderParam(joints[jId], dParamVel, torque);
		}
		return 1;
	}
	return 0;
}

int DynRobinSpringLeg::getJointVelocity(int jId, dReal* dX) {
	if (!isInit)
		return 0;
	if (jId < 0 || (jId > 4 && jId!=6))
		return 0;

	if (joints[jId]) {
		if (jId < 2 || jId==6) {
			*dX = dJointGetHingeAngleRate(joints[jId]);
		} else {
			*dX = dJointGetSliderPositionRate(joints[jId]);
		}
		return 1;
	}
	return 0;
}

int DynRobinSpringLeg::getJointPosition(int jId, dReal* x) {
	if (!isInit)
		return 0;
	if (jId < 0 || (jId > 4 && jId!=6))
		return 0;

	if (joints[jId]) {
		if (jId < 2 || jId==6) {
			*x = dJointGetHingeAngle(joints[jId]);
		} else {
			*x = dJointGetSliderPosition(joints[jId]);
		}
		return 1;
	}
	return 0;
}

int DynRobinSpringLeg::setJointPosition(int jId, dReal Position, dReal Speed) {
	if(!isInit)	
		return 0;
	if(jId < 0 || (jId > 4 && jId!=6))
		return 0;


	double PIDout = 0, currentPos=0, currentSpeed=0;
	int pom_id;

	if(joints[jId])
	{
		if(jId<2||jId==6){
			currentPos  =dJointGetHingeAngle(joints[jId]);
			if ((fabs(Position-currentPos))<0.01){
				int i=6;
			}
			if(jId==6){
				pom_id=3;
			} else {
				pom_id=jId;
			}
			PIDout = PID[pom_id]->PIDcal(Position, currentPos);
			
			if(Position<-1 && currentPos>1) 
			{
				currentPos = currentPos- 2*M_PI;
			}
			if(Position>1 && currentPos<-1) 
			{
				currentPos = currentPos+ 2*M_PI;
			}

			// Ovdje ide model motora - Samo max i min brzina 
			if(PIDout>Speed*0.1047){
				PIDout= Speed*0.1047;
			}else if (PIDout<-Speed*0.1047){
				PIDout=-Speed*0.1047;
			}
			if(jId==6){
				dJointSetAMotorParam(joints[7], dParamFMax1, 70);
				dJointSetAMotorParam(joints[7], dParamFMax2, 70);
				dJointSetAMotorParam(joints[7], dParamFMax3, 70);
			} else {
				dJointSetAMotorParam(joints[jId+4], dParamFMax1, 70);
				dJointSetAMotorParam(joints[jId+4], dParamFMax2, 70);
				dJointSetAMotorParam(joints[jId+4], dParamFMax3, 70);
			}

			switch (jId){
			case 0:
				dJointSetAMotorParam(joints[4], dParamVel1 , 0);
				dJointSetAMotorParam(joints[4], dParamVel2 , PIDout*2);
				dJointSetAMotorParam(joints[4], dParamVel3 , 0);
			case 1:
				dJointSetAMotorParam(joints[5], dParamVel1 , 0);
				dJointSetAMotorParam(joints[5], dParamVel2 , PIDout*2);
				dJointSetAMotorParam(joints[5], dParamVel3 , 0);
			case 6:
				dJointSetAMotorParam(joints[7], dParamVel1 , PIDout*2);
				dJointSetAMotorParam(joints[7], dParamVel2 , 0);
				dJointSetAMotorParam(joints[7], dParamVel3 , 0);
			}

		}else{
			dJointSetSliderParam (joints[3],dParamLoStop, -Position);
			dJointSetSliderParam (joints[3],dParamHiStop, -Position);
		}
		return 1;
	}

	return 1;
}

void DynRobinSpringLeg::UpdateLegState	(){

	const dReal* Dummy;
	dReal Rotation[3];
	dReal Position1[3];
	dReal Position2[3];
	dReal norm, Radius, Length;
	double HipSpeed, KneeSpeed;
	dJointFeedback *HipFeedback, *KneeFeedback, *FootFeedback;
	dJointFeedback Feedback;


	LegEnergy= GetLegEnergy();

	HipFeedback =dJointGetFeedback(joints[4]);
	KneeFeedback=dJointGetFeedback(joints[5]);

	HipSpeed	=dJointGetHingeAngleRate(joints[0]);
	KneeSpeed	=dJointGetHingeAngleRate(joints[1]);

	
	JointTorque[0] = HipFeedback->t1[1]*HipSpeed;
	JointTorque[1] = KneeFeedback->t1[1]*KneeSpeed;


	JointPos[0] = dJointGetHingeAngle(joints[0]);
	JointPos[1] = dJointGetHingeAngle(joints[1]);
	JointPos[3] = dJointGetHingeAngle(joints[6]);

	JointVel[0] = dJointGetHingeAngleRate(joints[0]);
	JointVel[1] = dJointGetHingeAngleRate(joints[1]);
	JointVel[3] = dJointGetHingeAngleRate(joints[6]);
	
	// Kontakt
	
	Dummy = dGeomGetPosition(link[3].geom[0]);
	FootPosGlobal[0]=Dummy[0]; FootPosGlobal[1]=Dummy[1]; FootPosGlobal[2]=Dummy[2];	//otPosGlobal[0]=Dummy[0], Dummy, 3*sizeof(dReal));

	if (FootPosGlobal[2]<0.008) Contact=1;
	else Contact=0;

	Dummy = dGeomGetPosition(link[2].geom[1]);
	LegPosGlobal[0]=Dummy[0]; LegPosGlobal[1]=Dummy[1]; LegPosGlobal[2]=Dummy[2];		//memcpy(LegPosGlobal , Dummy, 3*sizeof(dReal));
	Dummy = dGeomGetPosition(link[1].geom[1]);
	KneePosGlobal[0]=Dummy[0]; KneePosGlobal[1]=Dummy[1]; KneePosGlobal[2]=Dummy[2];		//memcpy(LegPosGlobal , Dummy, 3*sizeof(dReal));
	Dummy = dBodyGetLinearVel(link[3].body);
	FootSpeedLinGlobal[0]=Dummy[0]; FootSpeedLinGlobal[1]=Dummy[1]; FootSpeedLinGlobal[2]=Dummy[2];		//memcpy(FootSpeedLinGlobal, Dummy, 3*sizeof(dReal));
	Dummy = dBodyGetAngularVel(link[3].body);
	FootSpeedAngGlobal[0]=Dummy[0]; FootSpeedAngGlobal[1]=Dummy[1]; FootSpeedAngGlobal[2]=Dummy[2]; //memcpy(FootSpeedAngGlobal, Dummy, 3*sizeof(dReal));
	
	FootFeedback =dJointGetFeedback(joints[8]);
	FootForce[0]=FootFeedback->f2[0];
	FootForce[1]=FootFeedback->f2[1];
	FootForce[2]=FootFeedback->f2[2];
	
}

double DynRobinSpringLeg::GetLegEnergy(){
	double HipSpeed, KneeSpeed, ShoulderSpeed;
	dJointFeedback *HipFeedback, *KneeFeedback, *ShoulderFeedback;
	HipFeedback		=dJointGetFeedback(joints[4]);
	KneeFeedback	=dJointGetFeedback(joints[5]);
	ShoulderFeedback=dJointGetFeedback(joints[5]);

	HipSpeed		=dJointGetHingeAngleRate(joints[0]);
	KneeSpeed		=dJointGetHingeAngleRate(joints[1]);
	ShoulderSpeed	=dJointGetHingeAngleRate(joints[6]);

	return HipFeedback->t1[1]*HipSpeed+KneeFeedback->t1[1]*KneeSpeed+ShoulderFeedback->t1[1]*ShoulderSpeed;
	
}