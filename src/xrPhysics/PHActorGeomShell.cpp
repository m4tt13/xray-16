#include "StdAfx.h"
#include "PHActorGeomShell.h"
#include "IPhysicsShellHolder.h"
#include "PHCollideValidator.h"
#include "PHContactBodyEffector.h"
#include "PHWorld.h"
#include "SpaceUtils.h"
#include "tri-colliderknoopc/dTriList.h"

CPHActorGeomShell::CPHActorGeomShell(IPHMovementControl* mov_control) : 
    m_body(NULL),
    m_mov_control(mov_control)
{
}

void CPHActorGeomShell::Activate(const Fvector& pos)
{
    build();
    m_body = dBodyCreate(0);
    dBodySetGravityMode(m_body, 0);
    dBodySetNoUpdatePosMode(m_body, 1);
    dBodySetPosition(m_body, pos[0], pos[1], pos[2]);
    set_body(m_body);
    Island().AddBody(m_body);
    spatial_register();
}

void CPHActorGeomShell::Deactivate()
{
    spatial_unregister();
    CPHObject::deactivate();
    destroy();
    if (m_body)
    {
        Island().RemoveBody(m_body);
        dBodyDestroy(m_body);
        m_body = NULL;
    }
}

void CPHActorGeomShell::get_spatial_params()
{
    spatialParsFromDGeom(dSpacedGeometry(), spatial.sphere.P, AABB, spatial.sphere.R);
}

void CPHActorGeomShell::PhDataUpdate(dReal step) 
{
    m_mov_control->SetVelocity(cast_fv(dBodyGetLinearVel(m_body)));
    dBodySetAngularVel(m_body, 0.f, 0.f, 0.f);
}

void CPHActorGeomShell::PhTune(dReal step) 
{
    CPHContactBodyEffector* contact_effector = (CPHContactBodyEffector*)dBodyGetData(m_body);
    if (contact_effector)
        contact_effector->Apply();
}

void CPHActorGeomShell::InitContact(dContact* c, bool& do_collide, u16 /*material_idx_1*/, u16 /*material_idx_2*/)
{
    if (dGeomGetClass(c->geom.g1) == dTriListClass || dGeomGetClass(c->geom.g2) == dTriListClass)
        do_collide = false;
}

void CPHActorGeomShell::SetPosition(const Fvector& pos)
{
    dBodySetPosition(m_body, pos[0], pos[1], pos[2]);
    spatial_move();
}

void CPHActorGeomShell::SetBox(const Fbox& box)
{
    Fvector bc, bd;
    box.get_CD(bc, bd);

    Fmatrix xform = Fidentity;
    xform.c = bc;

    CBoxGeom* geom = smart_cast<CBoxGeom*>(Geom(0));
    geom->set_local_form(xform);
    geom->set_local_form_bt(xform);
    geom->set_size(bd);

    spatial_move();
}

void CPHActorGeomShell::SetMass(dReal mass) 
{
    dMass m;
    dMassSetSphere(&m, 1.0f, 100000.0f);
    dMassAdjust(&m, mass);
    dBodySetMass(m_body, &m);
}

CPHActorGeomShell* create_actor_shell(IPHMovementControl* mov_control, IPhysicsShellHolder* obj, ObjectContactCallbackFun* cb, void* data, const Fvector& pos, const Fbox& box, dReal mass, u16 material)
{
    CPHActorGeomShell* pActorShell = xr_new<CPHActorGeomShell>(mov_control);

    Fvector bc, bd;
    box.get_CD(bc, bd);

    Fmatrix xform = Fidentity;
    xform.c = bc;

    Fobb obb;
    obb.xform_set(xform);
    obb.m_halfsize = bd;
    pActorShell->add_Box(obb);

    pActorShell->Activate(pos);

    pActorShell->SetMass(mass);
    pActorShell->SetMaterial(material);
    pActorShell->SetPhObjectInGeomData(pActorShell);
    pActorShell->set_PhysicsRefObject(obj);
    pActorShell->set_ContactCallback(ph_world->default_character_contact_shotmark());
    pActorShell->set_ObjectContactCallback(cb);
    pActorShell->set_CallbackData(data);

    CPHCollideValidator::SetCharacterClass(*pActorShell);

    return pActorShell;
}

void destroy_actor_shell(CPHActorGeomShell*& pActorShell)
{
    if (!pActorShell)
        return;
    pActorShell->Deactivate();
    xr_delete(pActorShell);
    pActorShell = NULL;
}
