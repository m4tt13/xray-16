#pragma once
#include <ode/common.h>

struct dxCylinder;
extern XRPHYSICS_API int dCylinderClassUser;

dxGeom* dCreateCylinder(dSpaceID space, dReal r, dReal lz);
void dGeomCylinderSetParams(dGeomID g, dReal radius, dReal length);

XRPHYSICS_API void dGeomCylinderGetParams(dGeomID g, dReal* radius, dReal* length);
