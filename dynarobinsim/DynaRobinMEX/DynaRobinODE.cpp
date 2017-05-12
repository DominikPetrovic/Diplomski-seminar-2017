#include "DynaRobinODE.h"
#include "ode/ode.h"
#include "drawstuff/drawstuff.h"
#include "math.h"
#include "windows.h"
#include "tchar.h"
#include "DynRobinSpringLeg.h"
#include "DynRobinBody.h"
#include "DynRobinTerrain.h"
#include "math.h"

#include <thread>
#include <mutex>
#include <memory>
#include <chrono>
#include <assert.h>


// Potrebno je korigirati u skladu s svojim
#define PATH_TO_TEXTURES "C:/Users/Zvonimir/Desktop/Materijali/2.semestar/Diplomski-seminar-2017/dynarobinsim/OpenDynamicsEngine/drawstuff/textures"
						
#define NUM_OF_JOINTS 170
#define NUM_OF_CONTACT_POINTS 60

#ifdef dDOUBLE
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawLine dsDrawLineD
#endif

#pragma warning(disable:4244 4305)


extern dReal Visualization;
extern dReal SampleTime;

bool m_stoprequested;
bool keyboardPress=false;
bool callbackActive=false;

char keyboardChar;
int actuCounter=0;
int rotation=0;
double counter = 0;

std::shared_ptr<std::thread> m_thread;

dsFunctions fn;
dWorldID world;
dSpaceID space;
dJointGroupID contactgroup;
dGeomID ground;

DynRobinSpringLeg	RobotLegs[4];	//Desna prednje (DP), LP, DZ, LZ (iz robotove prespektive) 
DynRobinBody		RobotBody;		//jedna ploca od 4 dijela, zadaju se mase izvana
//DynRobinTerrain		RobotTerrain;	// Cilindri po podu

//body parameters
static double body_mass[4];

//simulation
static bool simLoopInProgress;
static bool NewDemandFromMatlab;
static bool ConfigurationInProgress;
static bool SimulationStepFinished;
static bool stoprequested;
double TimeStep;
double TIME=0;

float alpha = 1.0;				
int LINK_NUM,GEOM_NUM;

// OK
void LinkBodyWithLeg(double xB, double yB, double Dx, double Dy, double L1, double L2, double HEIGHT_TOTAL)
{

	for (int i = 0; i < 4; i++) {
		
		dJointID* tempJoint = RobotLegs[i].getShoulderJoint();
		dJointID* tempJointMotor = RobotLegs[i].getShoulderJointMotor();

		if (i<2){
			dJointAttach(*tempJoint,RobotBody.link[2].body,RobotLegs[i].link[4].body);
			dJointAttach(*tempJointMotor,RobotBody.link[2].body,RobotLegs[i].link[4].body);
		} else {
			dJointAttach(*tempJoint,RobotBody.link[0].body,RobotLegs[i].link[4].body);
			dJointAttach(*tempJointMotor,RobotBody.link[0].body,RobotLegs[i].link[4].body);
		}
		
		dJointSetAMotorAxis(*tempJointMotor, 0, 1, 1, 0, 0);
		dJointSetAMotorAxis(*tempJointMotor, 1, 1, 0, 1, 0);
		dJointSetAMotorAxis(*tempJointMotor, 2, 1, 0, 0, 1);
		dJointSetAMotorParam(*tempJointMotor, dParamVel1, 0);
		dJointSetAMotorParam(*tempJointMotor, dParamVel2, 0);
		dJointSetAMotorParam(*tempJointMotor, dParamVel3, 0);

		switch (i) {
		case 0:
			dJointSetHingeAnchor(*tempJoint, -xB, 0.05-yB, HEIGHT_TOTAL);
			break;
		case 1:
			dJointSetHingeAnchor(*tempJoint, -xB, yB-0.05, HEIGHT_TOTAL);
			break;
		case 2:
			dJointSetHingeAnchor(*tempJoint, xB, 0.05-yB, HEIGHT_TOTAL);
			break;
		case 3:
			dJointSetHingeAnchor(*tempJoint, xB, yB-0.05, HEIGHT_TOTAL);
			break;
		}

	}
}
// OK
static void makeRobot()
{
	dMass mass;
	dReal HEIGHT_TOTAL = 0.40;
	dReal L1 = 0.10, L2 = 0.12, Dx = 0.04, Dy = 0.04; // Dimenzije za noge
	dReal body_dimmensions[3] = { 0.36, 0.24, 0.04 }; // Dimenzije za tijelo
	dReal masses[4] = { 0.200, 0.100, 0.200, body_mass[3] };
	dReal xB = 0, yB = 0;
	
	// Stvaranje terena
	//RobotTerrain.Init(world,space,1);
	//RobotTerrain.Rotate(0,1,0,PI/2);

	// Stvaranje tijela
	RobotBody.Init(world, space, body_dimmensions, masses);
	RobotBody.Translate(xB, yB, HEIGHT_TOTAL);

	// Stvaranje nogu
	RobotLegs[0].Init(world, space, Dx, Dy, L1, L2, 1);
	RobotLegs[1].Init(world, space, Dx, Dy, L1, L2, -1);
	RobotLegs[2].Init(world, space, Dx, Dy, L1, L2, 1);
	RobotLegs[3].Init(world, space, Dx, Dy, L1, L2, -1);
	xB = body_dimmensions[0] / 2 - Dx / 2;
	yB = body_dimmensions[1] / 2 + Dy / 2 + 0.005;
	RobotLegs[0].Translate(-xB, -yB, HEIGHT_TOTAL - L1 / 2);
	RobotLegs[1].Translate(-xB,  yB, HEIGHT_TOTAL - L1 / 2);
	RobotLegs[2].Translate( xB, -yB, HEIGHT_TOTAL - L1 / 2);
	RobotLegs[3].Translate( xB,  yB, HEIGHT_TOTAL - L1 / 2);
	LinkBodyWithLeg(xB, yB, Dx, Dy, L1, L2, HEIGHT_TOTAL);
	
}
// OK
static void drawGeom(dGeomID g)
{
	const dReal *pos, *rot;
	dReal r,l;
	dVector3 sides;
	float R=0.0,G=0,B=220.0,A=1;
	float rbg[3] = {1, 1, 1};


	int type;
	
	pos = dGeomGetPosition(g);
	rot = dGeomGetRotation(g);

	char *s = (char *) dGeomGetData(g);
	if(s != NULL)
	{
		if(strcmp(s,"white") == 0)
		{ 
			R=1.1;G=1.1;B=1.1;A=alpha;
		}
		else if(strcmp(s,"black") == 0)
		{
			rbg[0]=0.3;rbg[1]=0.3;rbg[2]=0.3;
		}
		else if(strcmp(s,"green") == 0)
		{
			R=0.0;G=1.0;B=0.0;A=alpha;
		}
		else if(strcmp(s,"red") == 0)
		{
			R=1.0;G=0.0;B=0.0;A=alpha;
		}
		else if(strcmp(s,"vizir") == 0)
		{
			R=0.3;G=0.3;B=0.3;A=alpha-0.2;
			if(A<0) A=0.0;
		}	
	}

	dsSetColorAlpha(R,G,B,A);
	
	type = dGeomGetClass(g);
	if(type == dBoxClass)
	{
		dGeomBoxGetLengths(g,sides);
		dsDrawBox(pos,rot,sides);
	}
	else if(type == dCylinderClass)
	{
		dGeomCylinderGetParams(g,&r,&l);
		dsDrawCylinder(pos,rot,l,r);
	}
	else if(type == dCapsuleClass)
	{
		dGeomCapsuleGetParams(g,&r,&l);
		dsDrawCapsule(pos,rot,l,r);
	}
	else if(type == dSphereClass)
	{
		r=dGeomSphereGetRadius(g);
		dsDrawSphere(pos,rot,r);
	}
}
// OK
static void drawRobot(){

	for(int j=0;j<3;j++)
	{
		dGeomID g = dBodyGetFirstGeom(RobotBody.link[j].body);
		drawGeom(g);	

		while(g = dBodyGetNextGeom(g))	drawGeom(g);	
	}
	
	for(int i=0;i<4;i++){
		for(int j=0;j<6;j++)
		{
			dGeomID g = dBodyGetFirstGeom(RobotLegs[i].link[j].body);		
			drawGeom(g);		
	
			while(g = dBodyGetNextGeom(g))	drawGeom(g);	
		}
	}

	//for(int j=0;j<RobotTerrain.getBodyCount();j++)
	//{
	//	dGeomID g = dBodyGetFirstGeom(RobotTerrain.link[j].body);		
	//	drawGeom(g);		
	
	//	while(g = dBodyGetNextGeom(g))	drawGeom(g);	
	//}


	}
// OK
static void nearCallback(void *data, dGeomID o1, dGeomID o2) {
	
	dBodyID b1 = dGeomGetBody(o1);
	dBodyID b2 = dGeomGetBody(o2);

	if (b1 && b2 && dAreConnectedExcluding(b1,b2,dJointTypeContact)) return;
	if (b1 && b2 && dAreConnected (b1,b2)) return;

	bool cond1 = ((o1 == ground) || (o2 == ground));
	
	//if(!cond1) return;

	static const int N = 50;
  	dContact contact[N];
  	int n = dCollide(o1,o2,N,&contact[0].geom,sizeof(dContact));
   	if (n > 0) {
		if(n > N)
			n = N;
    	for (int i=0; i<n; i++) {
    		contact[i].surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1;
    		contact[i].surface.mu   = 8.0;//8.0;//5.0 //dInfinity; //2.0;
    		contact[i].surface.soft_erp = 0.2;
    		contact[i].surface.soft_cfm =0.005;//1e-5;// 0.005
	  		dJointID c = dJointCreateContact(world,contactgroup,&contact[i]);
			dJointAttach(c,b1,b2);

			dReal *pos = contact[i].geom.pos;			
    	}
  	}	

	
	
}

static void simLoop(int pause)
{
	
	if(stoprequested == true)
	{
		dsStop();
		return;
	}

	simLoopInProgress = true;
	if(ConfigurationInProgress==true)
	{
		return;
	}
	if(NewDemandFromMatlab==true)
	{
		dSpaceCollide(space,0,&nearCallback);
		TimeStep=0.01; // Ovo popraviti bilo je 0.01
		dWorldStep(world, TimeStep);
		TIME = TIME + TimeStep;
		dJointGroupEmpty(contactgroup);
		SimulationStepFinished = true;
		NewDemandFromMatlab = false;

		/* PODESAVANJE KAMERE POCETAK */
		const dReal *Position5, *Rotation5;
		float Pz=0.3;
		float CameraPos[3];
		float CameraRot[3];
		double NagibX, NagibY;
		double Kut;
		dReal Daljina=0.8; // Udaljenost 2 metra;
		dReal Visina=0.4;
		dVector3 PosVector;

		dBodyGetRelPointPos (RobotBody.link[0].body, 0 ,-Daljina, Visina, PosVector); // Dobijem poziciju u globalnom koordinatnom sustavu u odnosu na tijelo
	
		Position5=dBodyGetPosition(RobotBody.link[0].body);
		// Pozicija kamere - PosVector
		// Pozicija robota - Position5
		
		NagibY=(1.0*(Position5[1]-PosVector[1]));
		NagibX=(1.0*(Position5[0]-PosVector[0]));

		Kut=atan2(NagibY,NagibX);
		Rotation5=dBodyGetRotation(RobotBody.link[0].body);

		// Napravit geometriju za kameru
		CameraPos[0]=(float)(PosVector[0]);			CameraPos[1]=(float)(PosVector[1]);		CameraPos[2]=0.6;
		CameraRot[0]=(float)(1*Kut*180/3.14);		CameraRot[1]=-20;						CameraRot[2]=0;

		if (Visualization>0) dsSetViewpoint(CameraPos, CameraRot);
	}
	drawRobot();
	simLoopInProgress = false;
}
// OK
void command(int cmd)
{
	keyboardPress=true;
	keyboardChar=cmd;
	switch(cmd){
	case 'c':
		break;
	case 'v':
		break;
	}
}
// OK
void drawstuffSetup(){
	fn.version = DS_VERSION;
	fn.start = NULL;//&start;
	fn.step = &simLoop;
	fn.command = &command;
	fn.path_to_textures = PATH_TO_TEXTURES;
}
// OK
static void start(){	
	
	
}
// OK
static void ODE_THREAD_LOOP()
{	

	dsSimulationLoop(0,0,800,600,&fn);
	
	return;	
}

// Pripadaju klasi DynaRobinODE
DynaRobinODE::DynaRobinODE(dReal TimeStepVar, dReal _body_mass[4], dReal Visualization)
{
	TimeStep = TimeStepVar;
	for(int i=0;i<4;i++) body_mass[i]=_body_mass[i];
}
DynaRobinODE::~DynaRobinODE()
{
}
void DynaRobinODE::InitODE()
{
	stoprequested = false;
	drawstuffSetup();
	dInitODE2(0);	
	world = dWorldCreate();	
	space = dHashSpaceCreate(0);
	contactgroup = dJointGroupCreate(0);
	ground = dCreatePlane(space,0,0,1,0);
	dWorldSetGravity(world,0,0,-9.81);
	dWorldSetCFM(world,1e-5);
	dWorldSetERP(world,0.9);
	dWorldSetContactSurfaceLayer(world, 0.001);
	makeRobot();
}
void DynaRobinODE::StartODEThread()
{
	SimulationStepFinished = false;
	try
	{
		assert(!m_thread);
		m_thread = std::shared_ptr<std::thread>(new std::thread(&ODE_THREAD_LOOP));
		odeInitOK = true;
	}
	catch(...)
	{
		odeInitOK = false;
	}
}

void DynaRobinODE::stopODESimulation()
{
	SimulationStepFinished=false;
	stoprequested = true;
}

void DynaRobinODE::StopODEThread()
{
	try
	{
		stoprequested = true;
		while(simLoopInProgress==true) 
		{
			std::this_thread::yield();
		}

		dJointGroupDestroy (contactgroup);
		dSpaceDestroy (space);
		dWorldDestroy (world);
		dsStop();
		dCloseODE();
		std::this_thread::sleep_for(std::chrono::seconds(2));
		
		if(odeInitOK == true)
		{
			assert(m_thread);
			m_thread->join();
			m_thread.reset();
		}
	}
	catch(...)
	{
	}
}

void DynaRobinODE::OneStep()
{
	SimulationStepFinished=false;
	NewDemandFromMatlab = true;

}

bool DynaRobinODE::SimulationFinished()
{
	return SimulationStepFinished;
}

void DynaRobinODE::UpdateNewRobotState()
{
	while(simLoopInProgress==true) std::this_thread::yield();

	ConfigurationInProgress=false;
}

void DynaRobinODE::GetRobotState()
{
	while(simLoopInProgress==true) 	std::this_thread::yield();
	dVector3 Dummy;
	RobotBody.UpdateBodyState();
	for(int i=0;i<4;i++) 
	{
		RobotLegs[i].UpdateLegState();
		dBodyGetPosRelPoint (RobotBody.link[1].body, RobotLegs[i].LegPosGlobal[0] , RobotLegs[i].LegPosGlobal[1] , RobotLegs[i].LegPosGlobal[2] , Dummy);
		LegPosLocal	[i][0]=Dummy[0];LegPosLocal	[i][1]=Dummy[1];LegPosLocal	[i][2]=Dummy[2];
		dBodyGetPosRelPoint (RobotBody.link[1].body, RobotLegs[i].KneePosGlobal[0], RobotLegs[i].KneePosGlobal[1], RobotLegs[i].KneePosGlobal[2], Dummy);
		KneePosLocal[i][0]=Dummy[0];KneePosLocal[i][1]=Dummy[1];KneePosLocal[i][2]=Dummy[2];
		dBodyGetPosRelPoint (RobotBody.link[1].body, RobotLegs[i].FootPosGlobal[0], RobotLegs[i].FootPosGlobal[1], RobotLegs[i].FootPosGlobal[2], Dummy);
		FootPosLocal[i][0]=Dummy[0];FootPosLocal[i][1]=Dummy[1];FootPosLocal[i][2]=Dummy[2];
		
		FootContactForce[i][0]=RobotLegs[i].FootForce[0]; FootContactForce[i][1]=RobotLegs[i].FootForce[1];	FootContactForce[i][2]=RobotLegs[i].FootForce[2];

		RobotLegs[i].getJointPosition(6,&LegJointPos[i][0]);
		RobotLegs[i].getJointPosition(0,&LegJointPos[i][1]);
		RobotLegs[i].getJointPosition(1,&LegJointPos[i][2]);
	}
	
	dBodyGetPosRelPoint (RobotBody.link[1].body, RobotBody.BodyFront[0], RobotBody.BodyFront[1], RobotBody.BodyFront[2] , BodyFrontPosLocal);
	dBodyGetPosRelPoint (RobotBody.link[1].body, RobotBody.BodyBack[0], RobotBody.BodyBack[1], RobotBody.BodyBack[2] , BodyBackPosLocal);

}

void DynaRobinODE::SetAlphaReference(int index, dReal Alpha){
	RobotLegs[index].SetAlphaRef(Alpha);
}

void DynaRobinODE::SetLegReference(int index, dReal x, dReal y, dReal z, dReal choice){
	RobotLegs[index].SetLegRef(x,y,z,choice);
}

void DynaRobinODE::SetSpineReference(dReal q0, dReal q1){
	dReal q[2];
	q[0]=q0;
	q[1]=q1;
	RobotBody.SetSpineRef(q);
}
