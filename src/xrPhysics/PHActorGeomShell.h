#pragma once

#include "PHGeometryOwner.h"
#include "PHObject.h"
#include "MovementBoxDynamicActivate.h"

class CPHActorGeomShell : public CPHGeometryOwner, public CPHObject
{
public:
    CPHActorGeomShell(IPHMovementControl* mov_control);

    void Activate(const Fvector& pos);
    void Deactivate();
    
    virtual dGeomID dSpacedGeom() { return dSpacedGeometry(); }
    virtual void get_spatial_params();
    virtual void PhDataUpdate(dReal step);
    virtual void PhTune(dReal step);
    virtual void InitContact(dContact* c, bool& do_collide, u16 /*material_idx_1*/, u16 /*material_idx_2*/);
    virtual u16 get_elements_number() { return 0; };
    virtual CPHSynchronize* get_element_sync(u16 element) { return NULL; };
    virtual void Enable() { CPHObject::activate(); }
    virtual void Disable() { CPHObject::deactivate(); }
    virtual void SetPosition(const Fvector& pos);
    virtual void SetVelocity(const Fvector& vel) { dBodySetLinearVel(m_body, vel[0], vel[1], vel[2]); }
    virtual void SetBox(const Fbox& box);
    virtual void SetMass(dReal mass);
    virtual void SetMaterial(u16 material) { CPHGeometryOwner::SetMaterial(material); }
    virtual void SetPhysicsRefObject(IPhysicsShellHolder* obj) { CPHGeometryOwner::set_PhysicsRefObject( obj ); }
    virtual void EnableCollision() { CPHObject::collision_enable(); }
    virtual void DisableCollision() { CPHObject::collision_disable(); }
    virtual ECastType CastType() { return CPHObject::tpActorShell; }

private:
    dBodyID m_body;
    IPHMovementControl* m_mov_control;
};

XRPHYSICS_API CPHActorGeomShell* create_actor_shell(IPHMovementControl* mov_control, IPhysicsShellHolder* obj, ObjectContactCallbackFun* cb, void* data, const Fvector& pos, const Fbox& box, dReal mass, u16 material);
XRPHYSICS_API void destroy_actor_shell(CPHActorGeomShell*& pActorShell);
