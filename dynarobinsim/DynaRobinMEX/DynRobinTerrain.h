#ifndef _DynRobinTerrain
#define _DynRobinTerrain

#include "ConfigODE.h"

class DynRobinTerrain
{
	public:
	// ODE
	dWorldID		m_world; 
	dSpaceID		m_space;
	bodyGeo			link[1];

	DynRobinTerrain(double DT=0.01); // Ovo srediti
	~DynRobinTerrain();
	void Init			(dWorldID world, dSpaceID space, int Type);
	void Translate		(dReal ax, dReal ay, dReal az);
	void Rotate			(dReal ax, dReal ay, dReal az, dReal angle);
	int getBodyCount	(void);

private:
	bool isInit;
};
#endif                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           