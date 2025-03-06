#pragma once

typedef u32 CGID;
XRPHYSICS_API CGID RegisterGroup();

class CPHObject;
XRPHYSICS_API bool do_obj_collide(const CPHObject& obj1, const CPHObject& obj2);
