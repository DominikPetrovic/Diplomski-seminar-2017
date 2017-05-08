#ifndef _COMPLEXBODYBASE
#define _COMPLEXBODYBASE

#include "ode/ode.h"
#include "drawstuff/drawstuff.h"
#include "math.h"
#include "windows.h"
#include "tchar.h"

#define _MAX_GEO 15		//Broj geoma po clanku
#define PI 3.1415

struct bodyGeo
{
	dBodyID body;
	dGeomID geom[_MAX_GEO];
	int nbOfGeom;
	bool isJointBody;
};

//PID parametri
#define EPSILON 0.00001		//anti windup 0.001
//#define DT 0.001             //vrijeme uzrokovanja za PID default 0.01
#define MAX  10000.0              //MIN izlaz iz PID / 10
#define MIN -10000.0				//MAX izlaz iz PID / -10
#define KP  			30						// pojacanje PID  / 6.5
#define KD  			0						// 0.08
#define KI  			30						// 0.005

//Parametri motora [Nm]
#define MAX_TORQUE 2535 //default 35

//Gustoce [kg/m^3]
#define BODY_DENSITY 3700.0 //Al - 2700
#define LEG_DENSITY 200.0

//TODO
//identificirati parametre robota vezane za distibuciju njegove mase
//identificirati parametre motora te sukladno tome namjestiti PID

#endif

