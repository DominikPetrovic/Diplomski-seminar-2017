#include "DynRobinTerrain.h"
#define VertexCount 6
#define IndexCount 18
static dReal DT_local;
static dVector3 Size;
static float Vertices[VertexCount][3];
static dTriIndex Indices[IndexCount];
static dGeomID TriMesh;

DynRobinTerrain::DynRobinTerrain(dReal DT) :m_world(0), m_space(0), isInit(false) {
	DT_local = DT;
}

DynRobinTerrain::~DynRobinTerrain()
{
};

void DynRobinTerrain::Init(dWorldID world, dSpaceID space, int Type) {
	dMatrix3 R;
	dMass mass;
	m_world = world;
	m_space = space;

	if (!world || !space)
		return;

	// def. komada
	link[0].body = dBodyCreate(m_world);
	dBodySetPosition(link[0].body, 0, 4, 0);
	dBodySetKinematic(link[0].body);


	//for (int i=0; i<1;i++){
	link[0].geom[0] = dCreateBox(space, 10, 10, 0.1);
	//dRFromAxisAndAngle(R, 0, 1, 0, -PI/4);
	dGeomSetBody(link[0].geom[0], link[0].body);
	dGeomSetOffsetPosition(link[0].geom[0], 0, 0 * 0.2, 0);
	dGeomSetData(link[0].geom[0], (void*) "white");
	//dGeomSetRotation(link[0].geom[0],R);

	/*link[0].geom[1] = dCreateBox(space, 0.05, 1, 1);
	//dRFromAxisAndAngle(R, 0, 1, 0, -PI /2);
	dGeomSetBody(link[0].geom[1], link[0].body);
	dGeomSetOffsetPosition(link[0].geom[1], 0, -1.5, 0);
	dGeomSetData(link[0].geom[1], (void*) "white");*/
	//dGeomSetRotation(link[0].geom[1], R); 
	//}




	isInit = true;
}


void DynRobinTerrain::Translate(dReal ax, dReal ay, dReal az) {
	const dReal* R1;

	if (!isInit) return;

	R1 = dBodyGetPosition(link[0].body);
	dBodySetPosition(link[0].body, R1[0] + ax, R1[1] + ay, R1[2] + az);

}

void DynRobinTerrain::Rotate(dReal ax, dReal ay, dReal az, dReal angle) {
	dMatrix3 R;
	dReal phi = 0;   //roll
	dReal theta = PI/12;    // pitch
	dReal psi = -PI / 6;   //yaw
	dReal cx = 0;		//c je nova x os
	dReal cy = 0;
	dReal cz = 0;
	dReal dx = sin(psi); // d je nova y os
	dReal dy = cos(psi);
	dReal dz = 0;


	if (!isInit) return;

	//dRFrom2Axes( R, cx, cy, cz, dx, dy, dz);
	//dRFromEulerAngles(R1, 0, 0 , psi);
	//dRFromEulerAngles(R2, 0, angle, 0);
	//dRFromAxisAndAngle(R, ax, ay, az, angle);


	dQuaternion q1;
	dQuaternion q2;
	dQuaternion q3;

	dQFromAxisAndAngle(q1, 0, 0, 1, psi);  // kvaternioni gledaju rotaciju za negativan kut u smjeru kazaljke na satu
	dQFromAxisAndAngle(q2, 0, 1, 0, theta);

	dQMultiply0(q3, q1, q2);
	dQtoR(q3, R);   // pretvori kvaternion u matricu R

	dBodySetRotation(link[0].body, R);
}

int  DynRobinTerrain::getBodyCount(){
	return 1;
}