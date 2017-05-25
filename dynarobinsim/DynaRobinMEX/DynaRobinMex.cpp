// DynaRobinMex.cpp : Defines the exported functions for the DLL application.
//

#define S_FUNCTION_NAME  DynaRobinMex
#define S_FUNCTION_LEVEL 2

#define K_SAMPLE_TIME ssGetSFcnParam(S,0) //  SAMPLE_TIME
#define BODY_MASS ssGetSFcnParam(S,1) //  BODY MASS
#define VISUALIZATION_ON ssGetSFcnParam(S,2) //  6.5


#define PORT_WIDTH 1 

#include <yvals.h>
#include <mex.h>
#include <time.h>
#define CHAR16_T
#include <simstruc.h>
#include "DynRobinBody.h"
#include "DynRobinSpringLeg.h"
#include "DynaRobinODE.h"
#include <Windows.h>

#pragma warning(disable:4244 4305 4996)

extern DynRobinSpringLeg RobotLegs[4]; //Desna prednje (DP), LP, DZ, LZ (iz robotove prespektive) 
extern DynRobinBody RobotBody; //jedna ploca od 4 dijela, zadaju se mase izvana

dReal Visualization;
dReal SampleTime;

static void mdlInitializeSizes(SimStruct *S)
{
	ssSetNumSFcnParams(S, 3);//This is the number of expected parameters your S-function block expects
	if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return; //Parameter mismatch will be reported by Simulink 

	ssSetNumContStates(S, 0);
	ssSetNumDiscStates(S, 0);

#ifndef DIRECT
	/* set number of input ports, their widths and direct feed thru flags */
	if (!ssSetNumInputPorts(S, 6)) return;
	ssSetInputPortWidth(S, 0, PORT_WIDTH * 3); //FLFootReference
	ssSetInputPortWidth(S, 1, PORT_WIDTH * 3);    //FRFootReference
	ssSetInputPortWidth(S, 2, PORT_WIDTH * 3); //BLFootReference
	ssSetInputPortWidth(S, 3, PORT_WIDTH * 3); //BRFootReference
	ssSetInputPortWidth(S, 4, PORT_WIDTH * 4); //Alpha Reference
	ssSetInputPortWidth(S, 5, PORT_WIDTH * 2); //Spine reference
#else
	/* set number of input ports, their widths and direct feed thru flags */
	if (!ssSetNumInputPorts(S, 6)) return;
	ssSetInputPortWidth(S, 0, PORT_WIDTH * 3); //FLJointPosition
	ssSetInputPortWidth(S, 1, PORT_WIDTH * 3);    //FRJointPosition
	ssSetInputPortWidth(S, 2, PORT_WIDTH * 3); //BLJointPosition
	ssSetInputPortWidth(S, 3, PORT_WIDTH * 3); //BRJointPosition
	ssSetInputPortWidth(S, 4, PORT_WIDTH * 4); //Alpha Reference
	ssSetInputPortWidth(S, 5, PORT_WIDTH * 2); //Spine reference
#endif

#ifndef DIRECT
											   /* set the number of output ports and their widths */
	if (!ssSetNumOutputPorts(S, 21)) return;

	ssSetOutputPortWidth(S, 0, 3); //BodyPositionGlobal 
	ssSetOutputPortWidth(S, 1, 3); //BodyRollPitchYaw   
	ssSetOutputPortWidth(S, 2, 3); //RotationX  
	ssSetOutputPortWidth(S, 3, 3); //RotationY    
	ssSetOutputPortWidth(S, 4, 3); //RotationZ   
	ssSetOutputPortWidth(S, 5, 3); //FLJointPosition 
	ssSetOutputPortWidth(S, 6, 3); //FRJointPosition 
	ssSetOutputPortWidth(S, 7, 3); //BLJointPosition 
	ssSetOutputPortWidth(S, 8, 3); //BRJointPosition 
	ssSetOutputPortWidth(S, 9, 4); //FootPosXLocal 
	ssSetOutputPortWidth(S, 10, 4); //FootPosYLocal 
	ssSetOutputPortWidth(S, 11, 4); //FootPosZLocal 
	ssSetOutputPortWidth(S, 12, 3); //FLForce 
	ssSetOutputPortWidth(S, 13, 3); //FRForce 
	ssSetOutputPortWidth(S, 14, 3); //BLForce 
	ssSetOutputPortWidth(S, 15, 3); //BRForce
	ssSetOutputPortWidth(S, 16, 3); //AngularVelocityLocal
	ssSetOutputPortWidth(S, 17, 3); //LinearVelovictyLocal
	ssSetOutputPortWidth(S, 18, 4); //FootPosXGlobal 
	ssSetOutputPortWidth(S, 19, 4); //FootPosYGlobal 
	ssSetOutputPortWidth(S, 20, 4); //FootPosZGlobal
#else
	if (!ssSetNumOutputPorts(S, 30)) return;

	ssSetOutputPortWidth(S, 0, 3); //BodyPositionGlobal 
	ssSetOutputPortWidth(S, 1, 3); //BodyRollPitchYaw   
	ssSetOutputPortWidth(S, 2, 3); //RotationX  
	ssSetOutputPortWidth(S, 3, 3); //RotationY    
	ssSetOutputPortWidth(S, 4, 3); //RotationZ   
	ssSetOutputPortWidth(S, 5, 4); //LegPosXGlobal 
	ssSetOutputPortWidth(S, 6, 4); //LegPosYGlobal 
	ssSetOutputPortWidth(S, 7, 4); //LegPosZGlobal 
	ssSetOutputPortWidth(S, 8, 4); //LegPosXLocal 
	ssSetOutputPortWidth(S, 9, 4); //LegPosYLocal 
	ssSetOutputPortWidth(S, 10, 4); //LegPosZLocal 
	ssSetOutputPortWidth(S, 11, 4); //FootPosXGlobal 
	ssSetOutputPortWidth(S, 12, 4); //FootPosYGlobal 
	ssSetOutputPortWidth(S, 13, 4); //FootPosZGlobal 
	ssSetOutputPortWidth(S, 14, 4); //FootPosXLocal 
	ssSetOutputPortWidth(S, 15, 4); //FootPosYLocal 
	ssSetOutputPortWidth(S, 16, 4); //FootPosZLocal 
	ssSetOutputPortWidth(S, 17, 4); //LegEnergy 
	ssSetOutputPortWidth(S, 18, 1); //SpineEnergy 
	ssSetOutputPortWidth(S, 19, 4); //SpringLength 
	ssSetOutputPortWidth(S, 20, 3); //FrontBodyPosLocal 
	ssSetOutputPortWidth(S, 21, 3); //BackBodyPosLocal 
	ssSetOutputPortWidth(S, 22, 3); //FrontBodyPosGlobal 
	ssSetOutputPortWidth(S, 23, 3); //BackBodyPosGlobal 
	ssSetOutputPortWidth(S, 24, 4); //KneeLocalX 
	ssSetOutputPortWidth(S, 25, 4); //KneeLocalY 
	ssSetOutputPortWidth(S, 26, 4); //KneeLocalZ
	ssSetOutputPortWidth(S, 27, 4); //KneeGlobalX 
	ssSetOutputPortWidth(S, 28, 4); //KneeGlobalY 
	ssSetOutputPortWidth(S, 29, 4); //KneeGlobalZ 

#endif

	ssSetNumSampleTimes(S, 1);
	ssSetNumRWork(S, 0);
	ssSetNumIWork(S, 0);
	ssSetNumPWork(S, 2); // OdeWorld
	ssSetNumModes(S, 0);
	ssSetNumNonsampledZCs(S, 0);

	ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);
	ssSetOptions(S, (SS_SIM | SS_GENERATED_S_FUNCTION));

}

static void mdlInitializeSampleTimes(SimStruct *S)
{
	ssSetSampleTime(S, 0, mxGetScalar(K_SAMPLE_TIME));
	ssSetOffsetTime(S, 0, 0.0);
}


#define MDL_START
static void mdlStart(SimStruct *S)
{
	dReal body_mass[4];
	SampleTime = (dReal)mxGetPr(K_SAMPLE_TIME)[0];
	Visualization = (dReal)mxGetPr(VISUALIZATION_ON)[0];
	for (int i = 0; i<4; i++) body_mass[i] = (dReal)mxGetPr(BODY_MASS)[i];

	//INIT ODE
	DynaRobinODE * Dynarobin = new DynaRobinODE(SampleTime, body_mass, Visualization);
	Dynarobin->InitODE();
	Dynarobin->StartODEThread();
	Dynarobin->OneStep();
	Dynarobin->UpdateNewRobotState();
	Dynarobin->OneStep();
	Dynarobin->GetRobotState();

#ifndef DIRECT 
	Dynarobin->SetLegReference(0, 0.00, 0.00, -0.22, -1);
	Dynarobin->SetLegReference(1, 0.00, 0.00, -0.22, -1);
	Dynarobin->SetLegReference(2, 0.00, 0.00, -0.22, 1);
	Dynarobin->SetLegReference(3, 0.00, 0.00, -0.22, 1);
#else 
	Dynarobin->SetLegReference(0, 0.00, 0.20, -0.20, 1);
	Dynarobin->SetLegReference(1, 0.00, 0.20, -0.20, 1);
	Dynarobin->SetLegReference(2, 0.00, 0.20, -0.20, 1);
	Dynarobin->SetLegReference(3, 0.00, 0.20, -0.20, 1);
#endif
	Dynarobin->SetSpineReference(0.0, 0.0);
	Dynarobin->SetAlphaReference(0, 250);
	Dynarobin->SetAlphaReference(1, 250);
	Dynarobin->SetAlphaReference(2, 250);
	Dynarobin->SetAlphaReference(3, 250);

	ssSetPWorkValue(S, 0, Dynarobin);
}


#define MDL_UPDATE
static void mdlUpdate(SimStruct *S, int_T tid)
{

#ifndef DIRECT
	DynaRobinODE *Dynarobin = (DynaRobinODE*)ssGetPWorkValue(S, 0);

	InputRealPtrsType FL_Ref = ssGetInputPortRealSignalPtrs(S, 0); // FLFootReference
	InputRealPtrsType FR_Ref = ssGetInputPortRealSignalPtrs(S, 1); // FRFootReference
	InputRealPtrsType BL_Ref = ssGetInputPortRealSignalPtrs(S, 2); // BLFootReference
	InputRealPtrsType BR_Ref = ssGetInputPortRealSignalPtrs(S, 3); // BRFootReference

	Dynarobin->SetLegReference(0, *FL_Ref[0], *FL_Ref[1], *FL_Ref[2], -1);
	Dynarobin->SetLegReference(1, *FR_Ref[0], *FR_Ref[1], *FR_Ref[2], -1);
	Dynarobin->SetLegReference(2, *BL_Ref[0], *BL_Ref[1], *BL_Ref[2], 1);
	Dynarobin->SetLegReference(3, *BR_Ref[0], *BR_Ref[1], *BR_Ref[2], 1);
#else 
	DynaRobinODE *Dynarobin = (DynaRobinODE*)ssGetPWorkValue(S, 0);

	InputRealPtrsType FLRef = ssGetInputPortRealSignalPtrs(S, 0); // X Reference
	InputRealPtrsType FRRef = ssGetInputPortRealSignalPtrs(S, 1); // Y Reference
	InputRealPtrsType BLRef = ssGetInputPortRealSignalPtrs(S, 2); // Z Reference
	InputRealPtrsType BRRef = ssGetInputPortRealSignalPtrs(S, 3); // Choice Reference
	Dynarobin->SetLegReference(0, *FLRef[0], *FLRef[1], *FLRef[2], 0);
	Dynarobin->SetLegReference(1, *FRRef[0], *FRRef[1], *FRRef[2], 0);
	Dynarobin->SetLegReference(2, *BLRef[0], *BLRef[1], *BLRef[2], 0);
	Dynarobin->SetLegReference(3, *BRRef[0], *BRRef[1], *BRRef[2], 0);
#endif

	InputRealPtrsType ARef = ssGetInputPortRealSignalPtrs(S, 4); // Alpha Reference
	InputRealPtrsType SRef = ssGetInputPortRealSignalPtrs(S, 5); // Spine reference
	for (int i = 0; i<4; i++) {
		Dynarobin->SetAlphaReference(i, *ARef[i]);
	}
	Dynarobin->SetSpineReference(*SRef[0], *SRef[1]);
	Dynarobin->UpdateNewRobotState();
	Dynarobin->OneStep();

	while (Dynarobin->SimulationFinished() == false)
	{
		Sleep(0);
	}
}

static void mdlOutputs(SimStruct *S, int_T tid)
{

	DynaRobinODE *Dynarobin = (DynaRobinODE*)ssGetPWorkValue(S, 0);

#ifndef DIRECT 
	real_T* BodyPositionGlobal = ssGetOutputPortRealSignal(S, 0);
	real_T* BodyRollPitchYaw = ssGetOutputPortRealSignal(S, 1);
	real_T* BodyRotationX = ssGetOutputPortRealSignal(S, 2);
	real_T* BodyRotationY = ssGetOutputPortRealSignal(S, 3);
	real_T* BodyRotationZ = ssGetOutputPortRealSignal(S, 4);
	real_T* FLPosition = ssGetOutputPortRealSignal(S, 5);
	real_T* FRPosition = ssGetOutputPortRealSignal(S, 6);
	real_T* BLPosition = ssGetOutputPortRealSignal(S, 7);
	real_T* BRPosition = ssGetOutputPortRealSignal(S, 8);
	real_T* FootPosXLocal = ssGetOutputPortRealSignal(S, 9);
	real_T* FootPosYLocal = ssGetOutputPortRealSignal(S, 10);
	real_T* FootPosZLocal = ssGetOutputPortRealSignal(S, 11);
	real_T* FLForce = ssGetOutputPortRealSignal(S, 12);
	real_T* FRForce = ssGetOutputPortRealSignal(S, 13);
	real_T* BLForce = ssGetOutputPortRealSignal(S, 14);
	real_T* BRForce = ssGetOutputPortRealSignal(S, 15);
	real_T* AngVelLoc = ssGetOutputPortRealSignal(S, 16);
	real_T* LinVelLoc = ssGetOutputPortRealSignal(S, 17);
	real_T* FootPosXGlobal = ssGetOutputPortRealSignal(S, 18);
	real_T* FootPosYGlobal = ssGetOutputPortRealSignal(S, 19);
	real_T* FootPosZGlobal = ssGetOutputPortRealSignal(S, 20);

	Dynarobin->GetRobotState();
	memcpy(BodyPositionGlobal, RobotBody.BodyPosition, 3 * sizeof(dReal));
	memcpy(BodyRollPitchYaw, RobotBody.BodyOrientation, 3 * sizeof(dReal));
	memcpy(BodyRotationX, RobotBody.BodyRotationX, 3 * sizeof(dReal));
	memcpy(BodyRotationY, RobotBody.BodyRotationY, 3 * sizeof(dReal));
	memcpy(BodyRotationZ, RobotBody.BodyRotationZ, 3 * sizeof(dReal));
	memcpy(FLPosition, Dynarobin->LegJointPos[0], 3 * sizeof(dReal));
	memcpy(FRPosition, Dynarobin->LegJointPos[1], 3 * sizeof(dReal));
	memcpy(BLPosition, Dynarobin->LegJointPos[2], 3 * sizeof(dReal));
	memcpy(BRPosition, Dynarobin->LegJointPos[3], 3 * sizeof(dReal));
	memcpy(FLForce, Dynarobin->FootContactForce[0], 3 * sizeof(dReal));
	memcpy(FRForce, Dynarobin->FootContactForce[1], 3 * sizeof(dReal));
	memcpy(BLForce, Dynarobin->FootContactForce[2], 3 * sizeof(dReal));
	memcpy(BRForce, Dynarobin->FootContactForce[3], 3 * sizeof(dReal));
	memcpy(AngVelLoc, RobotBody.BodySpeedAngLocal, 3 * sizeof(dReal));
	memcpy(LinVelLoc, RobotBody.BodySpeedLinLocal, 3 * sizeof(dReal));
	for (int i = 0; i<4; i++) {
		/*LegPosXLocal[i]  =Dynarobin->LegPosLocal[i][0];
		LegPosYLocal[i]  =Dynarobin->LegPosLocal[i][1];
		LegPosZLocal[i]  =Dynarobin->LegPosLocal[i][2];
		KneePosXLocal[i] =Dynarobin->KneePosLocal[i][0];
		KneePosYLocal[i] =Dynarobin->KneePosLocal[i][1];
		KneePosZLocal[i] =Dynarobin->KneePosLocal[i][2]; */
		FootPosXLocal[i] = Dynarobin->FootPosLocal[i][0];
		FootPosYLocal[i] = Dynarobin->FootPosLocal[i][1];
		FootPosZLocal[i] = Dynarobin->FootPosLocal[i][2];
		/*KneePosXGlobal[i]=RobotLegs[i].KneePosGlobal[0];
		KneePosYGlobal[i]=RobotLegs[i].KneePosGlobal[1];
		KneePosZGlobal[i]=RobotLegs[i].KneePosGlobal[2];*/
		FootPosXGlobal[i] = RobotLegs[i].FootPosGlobal[0];
		FootPosYGlobal[i] = RobotLegs[i].FootPosGlobal[1];
		FootPosZGlobal[i] = RobotLegs[i].FootPosGlobal[2];/*
														  LegPosXGlobal[i] =RobotLegs[i].LegPosGlobal[0];
														  LegPosYGlobal[i] =RobotLegs[i].LegPosGlobal[1];
														  LegPosZGlobal[i] =RobotLegs[i].LegPosGlobal[2];
														  LegEnergy[i]     =RobotLegs[i].LegEnergy;
														  SpringLength[i]  =RobotLegs[i].SpringLenght; */

	}
	//SpineEnergy[0]     = RobotBody.BodyEnergy; 
#else
	real_T* BodyPositionGlobal = ssGetOutputPortRealSignal(S, 0);
	real_T* BodyRollPitchYaw = ssGetOutputPortRealSignal(S, 1);
	real_T* BodyRotationX = ssGetOutputPortRealSignal(S, 2);
	real_T* BodyRotationY = ssGetOutputPortRealSignal(S, 3);
	real_T* BodyRotationZ = ssGetOutputPortRealSignal(S, 4);
	real_T* LegPosXGlobal = ssGetOutputPortRealSignal(S, 5);
	real_T* LegPosYGlobal = ssGetOutputPortRealSignal(S, 6);
	real_T* LegPosZGlobal = ssGetOutputPortRealSignal(S, 7);
	real_T* LegPosXLocal = ssGetOutputPortRealSignal(S, 8);
	real_T* LegPosYLocal = ssGetOutputPortRealSignal(S, 9);
	real_T* LegPosZLocal = ssGetOutputPortRealSignal(S, 10);
	real_T* FootPosXGlobal = ssGetOutputPortRealSignal(S, 11);
	real_T* FootPosYGlobal = ssGetOutputPortRealSignal(S, 12);
	real_T* FootPosZGlobal = ssGetOutputPortRealSignal(S, 13);
	real_T* FootPosXLocal = ssGetOutputPortRealSignal(S, 14);
	real_T* FootPosYLocal = ssGetOutputPortRealSignal(S, 15);
	real_T* FootPosZLocal = ssGetOutputPortRealSignal(S, 16);
	real_T* LegEnergy = ssGetOutputPortRealSignal(S, 17);
	real_T* SpineEnergy = ssGetOutputPortRealSignal(S, 18);
	real_T* SpringLength = ssGetOutputPortRealSignal(S, 19);
	real_T* FrontBodyPosLocal = ssGetOutputPortRealSignal(S, 20);
	real_T* BackBodyPosLocal = ssGetOutputPortRealSignal(S, 21);
	real_T* FrontBodyPosGlobal = ssGetOutputPortRealSignal(S, 22);
	real_T* BackBodyPosGlobal = ssGetOutputPortRealSignal(S, 23);
	real_T* KneePosXLocal = ssGetOutputPortRealSignal(S, 24);
	real_T* KneePosYLocal = ssGetOutputPortRealSignal(S, 25);
	real_T* KneePosZLocal = ssGetOutputPortRealSignal(S, 26);
	real_T* KneePosXGlobal = ssGetOutputPortRealSignal(S, 27);
	real_T* KneePosYGlobal = ssGetOutputPortRealSignal(S, 28);
	real_T* KneePosZGlobal = ssGetOutputPortRealSignal(S, 29);
	//real_T* BodyRotation = ssGetOutputPortRealSignal(S,20); ///


	Dynarobin->GetRobotState();
	memcpy(BodyPositionGlobal, RobotBody.BodyPosition, 3 * sizeof(dReal));
	memcpy(BodyRollPitchYaw, RobotBody.BodyOrientation, 3 * sizeof(dReal));
	memcpy(BodyRotationX, RobotBody.BodyRotationX, 3 * sizeof(dReal));
	memcpy(BodyRotationY, RobotBody.BodyRotationY, 3 * sizeof(dReal));
	memcpy(BodyRotationZ, RobotBody.BodyRotationZ, 3 * sizeof(dReal));
	memcpy(FrontBodyPosLocal, Dynarobin->BodyFrontPosLocal, 3 * sizeof(dReal));
	memcpy(BackBodyPosLocal, Dynarobin->BodyBackPosLocal, 3 * sizeof(dReal));
	memcpy(FrontBodyPosGlobal, RobotBody.BodyFront, 3 * sizeof(dReal));
	memcpy(BackBodyPosGlobal, RobotBody.BodyBack, 3 * sizeof(dReal));


#endif


}


static void mdlTerminate(SimStruct *S)
{
	DynaRobinODE *pDynaRobin = (DynaRobinODE*)ssGetPWorkValue(S, 0);//get pointer to class
	pDynaRobin->stopODESimulation();
	pDynaRobin->StopODEThread();
	delete pDynaRobin;
}



#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif