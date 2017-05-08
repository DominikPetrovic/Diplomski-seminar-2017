#define S_FUNCTION_NAME  DynaRobinIKin
#define S_FUNCTION_LEVEL 2

#define K_SAMPLE_TIME		ssGetSFcnParam(S,0)			

#include <yvals.h>
#include <mex.h>
#include <time.h>
#include <simstruc.h>
#include <Windows.h>
#include "DynaRobinIKin.h"

#pragma warning(disable:4244 4305 4996)

DynaRobinIKin	RobotKinematics;

double SampleTime;

static void mdlInitializeSizes(SimStruct *S)
{
	ssSetNumSFcnParams(S,1);//This is the number of expected parameters your S-function block expects
	if (ssGetNumSFcnParams(S)!=ssGetSFcnParamsCount(S)) return; //Parameter mismatch will be reported by Simulink 

	ssSetNumContStates(S,0);
	ssSetNumDiscStates(S,0);

	/* set number of input ports, their widths and direct feed thru flags */
	if (!ssSetNumInputPorts(S,11)) return;
	ssSetInputPortWidth(S,0,1);		// Active
	ssSetInputPortWidth(S,1,3);		// Pos_vel reference
	ssSetInputPortWidth(S,2,3);		// Rot_vel reference
	ssSetInputPortWidth(S,3,3);		// FL leg joints reference	
	ssSetInputPortWidth(S,4,3);		// FR leg joints reference
	ssSetInputPortWidth(S,5,3);		// BL leg joints reference
	ssSetInputPortWidth(S,6,3);		// BR leg joints reference
	ssSetInputPortWidth(S,7,3);		// FL leg joints feedback	
	ssSetInputPortWidth(S,8,3);		// FR leg joints feedback
	ssSetInputPortWidth(S,9,3);		// BL leg joints feedback
	ssSetInputPortWidth(S,10,3);	// BR leg joints feedback

	/* set the number of output ports and their widths */
	if (!ssSetNumOutputPorts(S, 8)) return;
	ssSetOutputPortWidth(S, 0, 3);	// FL XYZ coordinates	  
	ssSetOutputPortWidth(S, 1, 3);	// FR XYZ coordinates	
	ssSetOutputPortWidth(S, 2, 3);	// BL XYZ coordinates
	ssSetOutputPortWidth(S, 3, 3);	// BR XYZ coordinates

	ssSetOutputPortWidth(S, 4, 3);	// FL leg joints	  
	ssSetOutputPortWidth(S, 5, 3);	// FR leg joints	
	ssSetOutputPortWidth(S, 6, 3);	// BL leg joints	
	ssSetOutputPortWidth(S, 7, 3);	// BR leg joints	

	ssSetNumSampleTimes(S,1);
	ssSetNumRWork(S,0);								
	ssSetNumIWork(S,0);
	ssSetNumPWork(S,2);							
	ssSetNumModes(S,0);
	ssSetNumNonsampledZCs(S,0);

	ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);
	ssSetOptions(S,(SS_SIM|SS_GENERATED_S_FUNCTION));

}

static void mdlInitializeSampleTimes(SimStruct *S)
{
	ssSetSampleTime(S,0,mxGetScalar(K_SAMPLE_TIME));
	ssSetOffsetTime(S,0,0.0);	
}


#define MDL_START
static void mdlStart(SimStruct *S)
{
	SampleTime=(double)mxGetPr(K_SAMPLE_TIME)[0];

	//INIT KINEMATICS
	DynaRobinIKin * Kinematics = new DynaRobinIKin();


	ssSetPWorkValue(S, 0, Kinematics);
}


#define MDL_UPDATE
static void mdlUpdate(SimStruct *S, int_T tid)
{
	vector<double> ValueFL, ValueFR, ValueBL, ValueBR, Ref;
	DynaRobinIKin *Kinematics=(DynaRobinIKin*)ssGetPWorkValue(S,0);	
	
	InputRealPtrsType Active		= ssGetInputPortRealSignalPtrs(S,0);
	InputRealPtrsType Pos_vel_ref	= ssGetInputPortRealSignalPtrs(S,1);
	InputRealPtrsType Rot_vel_ref	= ssGetInputPortRealSignalPtrs(S,2);
	InputRealPtrsType FL_ref_joints = ssGetInputPortRealSignalPtrs(S,3);
	InputRealPtrsType FR_ref_joints = ssGetInputPortRealSignalPtrs(S,4);
	InputRealPtrsType BL_ref_joints = ssGetInputPortRealSignalPtrs(S,5);
	InputRealPtrsType BR_ref_joints = ssGetInputPortRealSignalPtrs(S,6);
	InputRealPtrsType FL_leg_joints = ssGetInputPortRealSignalPtrs(S,7);
	InputRealPtrsType FR_leg_joints = ssGetInputPortRealSignalPtrs(S,8);
	InputRealPtrsType BL_leg_joints = ssGetInputPortRealSignalPtrs(S,9);
	InputRealPtrsType BR_leg_joints = ssGetInputPortRealSignalPtrs(S,10);
	
	ValueFL.push_back(*FL_leg_joints[0]);	ValueFL.push_back(*FL_leg_joints[1]);	ValueFL.push_back(*FL_leg_joints[2]);
	ValueFR.push_back(*FR_leg_joints[0]);	ValueFR.push_back(*FR_leg_joints[1]);	ValueFR.push_back(*FR_leg_joints[2]);
	ValueBL.push_back(*BL_leg_joints[0]);	ValueBL.push_back(*BL_leg_joints[1]);	ValueBL.push_back(*BL_leg_joints[2]);
	ValueBR.push_back(*BR_leg_joints[0]);	ValueBR.push_back(*BR_leg_joints[1]);	ValueBR.push_back(*BR_leg_joints[2]);
	Ref.push_back(*Pos_vel_ref[0]);			Ref.push_back(*Pos_vel_ref[1]);			Ref.push_back(*Pos_vel_ref[2]);
	Ref.push_back(*Rot_vel_ref[0]);			Ref.push_back(*Rot_vel_ref[1]);			Ref.push_back(*Rot_vel_ref[2]);

	if(*Active[0]>0.1){
		Kinematics->CalculateJacobianInverse(ValueFL,ValueFR,ValueBL,ValueBR,Ref);
	}else{
		Kinematics->FL_last_res->at(0)=*FL_ref_joints[0];
		Kinematics->FL_last_res->at(1)=*FL_ref_joints[1];
		Kinematics->FL_last_res->at(2)=*FL_ref_joints[2];
		
		Kinematics->FR_last_res->at(0)=*FR_ref_joints[0];
		Kinematics->FR_last_res->at(1)=*FR_ref_joints[1];
		Kinematics->FR_last_res->at(2)=*FR_ref_joints[2];
		
		Kinematics->BL_last_res->at(0)=*BL_ref_joints[0];
		Kinematics->BL_last_res->at(1)=*BL_ref_joints[1];
		Kinematics->BL_last_res->at(2)=*BL_ref_joints[2];
		
		Kinematics->BR_last_res->at(0)=*BR_ref_joints[0];
		Kinematics->BR_last_res->at(1)=*BR_ref_joints[1];
		Kinematics->BR_last_res->at(2)=*BR_ref_joints[2];
	}
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
	
	DynaRobinIKin *Kinematics=(DynaRobinIKin*)ssGetPWorkValue(S,0);	

	vector<double> FL_pos = { 0, 0, 0 },	FR_pos = { 0, 0, 0 },		BL_pos = { 0, 0, 0 },		BR_pos = { 0, 0, 0 };
	vector<double> FL_joints = { 0, 0, 0 }, FR_joints = { 0, 0, 0 },	BL_joints = { 0, 0, 0 },	BR_joints = { 0, 0, 0 };

	double* FL_foot_coordinates = ssGetOutputPortRealSignal(S, 0);
	double* FR_foot_coordinates = ssGetOutputPortRealSignal(S, 1);
	double* BL_foot_coordinates = ssGetOutputPortRealSignal(S, 2);
	double* BR_foot_coordinates = ssGetOutputPortRealSignal(S, 3);

	double* FL_leg_joints = ssGetOutputPortRealSignal(S, 4);
	double* FR_leg_joints = ssGetOutputPortRealSignal(S, 5);
	double* BL_leg_joints = ssGetOutputPortRealSignal(S, 6);
	double* BR_leg_joints = ssGetOutputPortRealSignal(S, 7);

	FL_leg_joints[0] = Kinematics->FL_last_res->at(0);
	FL_leg_joints[1] = Kinematics->FL_last_res->at(1);
	FL_leg_joints[2] = Kinematics->FL_last_res->at(2);
	
	FR_leg_joints[0] = Kinematics->FR_last_res->at(0);
	FR_leg_joints[1] = Kinematics->FR_last_res->at(1);
	FR_leg_joints[2] = Kinematics->FR_last_res->at(2);
	
	BL_leg_joints[0] = Kinematics->BL_last_res->at(0);
	BL_leg_joints[1] = Kinematics->BL_last_res->at(1);
	BL_leg_joints[2] = Kinematics->BL_last_res->at(2);

	BR_leg_joints[0] = Kinematics->BR_last_res->at(0);
	BR_leg_joints[1] = Kinematics->BR_last_res->at(1);
	BR_leg_joints[2] = Kinematics->BR_last_res->at(2);
	

	FL_joints[0] = Kinematics->FL_last_res->at(0);
	FL_joints[1] = Kinematics->FL_last_res->at(1);
	FL_joints[2] = Kinematics->FL_last_res->at(2);

	FR_joints[0] = Kinematics->FR_last_res->at(0);
	FR_joints[1] = Kinematics->FR_last_res->at(1);
	FR_joints[2] = Kinematics->FR_last_res->at(2);

	BL_joints[0] = Kinematics->BL_last_res->at(0);
	BL_joints[1] = Kinematics->BL_last_res->at(1);
	BL_joints[2] = Kinematics->BL_last_res->at(2);

	BR_joints[0] = Kinematics->BR_last_res->at(0);
	BR_joints[1] = Kinematics->BR_last_res->at(1);
	BR_joints[2] = Kinematics->BR_last_res->at(2);


	Kinematics->CalculateDirectKinematics(FL_joints, &FL_pos);
	Kinematics->CalculateDirectKinematics(FR_joints, &FR_pos);
	Kinematics->CalculateDirectKinematics(BL_joints, &BL_pos);
	Kinematics->CalculateDirectKinematics(BR_joints, &BR_pos);

	FL_foot_coordinates[0] = FL_pos.at(0);
	FL_foot_coordinates[1] = FL_pos.at(1);
	FL_foot_coordinates[2] = FL_pos.at(2);

	FR_foot_coordinates[0] = FR_pos.at(0);
	FR_foot_coordinates[1] = FR_pos.at(1);
	FR_foot_coordinates[2] = FR_pos.at(2);

	BL_foot_coordinates[0] = BL_pos.at(0);
	BL_foot_coordinates[1] = BL_pos.at(1);
	BL_foot_coordinates[2] = BL_pos.at(2);

	BR_foot_coordinates[0] = BR_pos.at(0);
	BR_foot_coordinates[1] = BR_pos.at(1);
	BR_foot_coordinates[2] = BR_pos.at(2);

}


static void mdlTerminate(SimStruct *S)
{
	DynaRobinIKin *Kinematics=(DynaRobinIKin*)ssGetPWorkValue(S,0);	

	delete Kinematics;
}



#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
