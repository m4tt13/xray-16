#include "stdafx.h"
#include "xrEngine/IPhysicsShell.h"
#include "xrPhysics/CalculateTriangle.h"
#include "xrPhysics/ExtendedGeom.h"
#include "xrPhysics/IColisiondamageInfo.h"
#include "xrPhysics/ICollideValidator.h"
#include "xrPhysics/IPHStaticGeomShell.h"
#include "xrPhysics/MathUtilsOde.h"
#include "xrPhysics/params.h"
#include "xrPhysics/PHActorGeomShell.h"
#include "xrPhysics/PHCharacter.h"
#include "xrPhysics/PHSimpleCharacter.h"
#include "xrPhysics/PHObject.h"
#include "xrServerEntities/PHSynchronize.h"
#include "physicsquery.h"
#include "hl2movement.h"
#include "collisionutils.h"

#define	MAX_CLIP_PLANES	5

#define PLAYER_MAX_SAFE_FALL_SPEED	13.1625f // approx 20 feet sqrt( 2 * gravity * 20 * 12 )
#define PLAYER_FALL_PUNCH_THRESHOLD 7.575f // won't punch player's screen/make scrape noise unless player falling at least this fast - at least a 76" fall (sqrt( 2 * g * 76))

#define PUNCH_DAMPING		9.0f		// bigger number makes the response more damped, smaller is less damped
										// currently the system will overshoot, with larger damping values it won't
#define PUNCH_SPRING_CONSTANT	65.0f	// bigger number increases the speed at which the view corrects

const float PLAYER_SPEED_DUCK_MODIFIER	= 0.34f;
const float PLAYER_SPEED_CLIMB_MODIFIER	= 0.34f;
const float ACTOR_SHELL_POSITION_ADJUSTMENT	= 0.1f;

float CHL2Movement::m_flFrameTime = 0.01f;
float CHL2Movement::m_flForwardSpeed = 10.0f;
float CHL2Movement::m_flSideSpeed = 10.0f;
float CHL2Movement::m_flWalkSpeed = 5.0f;
float CHL2Movement::m_flSlowWalkSpeed = 2.5f;
float CHL2Movement::m_flSprintSpeed = 8.0f;
float CHL2Movement::m_flMaxControlSpeed = 8.0f;
float CHL2Movement::m_flJumpPower = 6.75f;
float CHL2Movement::m_flGravity = 20.0f;
float CHL2Movement::m_flMaxVelocity = 100.0f;
float CHL2Movement::m_flNonJumpVelocity = 3.5f;
float CHL2Movement::m_flBounce = 0.0f;
float CHL2Movement::m_flStepSize = 0.5f;
float CHL2Movement::m_flFriction = 8.0f;
float CHL2Movement::m_flStopSpeed = 0.25f;
float CHL2Movement::m_flStandableNormal = 0.525f;
float CHL2Movement::m_flWalkableNormal = 0.525f;
float CHL2Movement::m_flAirMaxWishSpeed = 0.75f;
float CHL2Movement::m_flAccelerate = 10.0f;
float CHL2Movement::m_flAirAccelerate = 1000.0f;
float CHL2Movement::m_flLadderDistance = 0.05f;
float CHL2Movement::m_flLadderLeaveSpeed = 6.75f;
float CHL2Movement::m_flClimbSpeed = 5.0f;
float CHL2Movement::m_flDuckSpeed = 0.4f;
float CHL2Movement::m_flUnDuckSpeed = 0.2f;
float CHL2Movement::m_flPushawayForce = 5000.0f;
float CHL2Movement::m_flPushawayMaxForce = 250.0f;
BOOL CHL2Movement::m_bSpeedometer = TRUE;
BOOL CHL2Movement::m_bStickToGround = FALSE;
BOOL CHL2Movement::m_bEnableBHop = TRUE;
int CHL2Movement::m_nABHMode = 2;

CHL2Movement::CHL2Movement( CPHMovementControl* pMovementControl ) :
    m_pMovControl( pMovementControl ),
    m_pPhysicsRef( nullptr ),
    m_pPhysicsShell( nullptr ),
    m_pLastMaterial( &m_nLastMaterial ),
    m_nTickCount( 1 ),
    m_bActivated( false )
{
    Clear();
}

CHL2Movement::~CHL2Movement( void )
{
    destroy_actor_shell( m_pPhysicsShell );
}

void CHL2Movement::Clear( void )
{
    m_nLastMaterial = GAMEMTL_NONE_IDX;
    m_nInjuriousMaterial = GAMEMTL_NONE_IDX;

	m_nButtons = 0;
	m_nOldButtons = 0;
    m_nOldBoxID = 0;

    m_nExternalImpulseEndTick = u64(-1);

	m_surfaceFriction = 1.0f;
    m_flForwardMove = 0.0f;
	m_flSideMove = 0.0f;
    m_flMaxSpeed = 0.0f;
    m_flDucktime = 0.0f;
    m_flVelocityModifier = 1.0f;
    m_flFallVelocity = 0.0f;
    m_flCameraHeight = m_pMovControl->Box().vMax.y;

	m_dblRemainder = 0.0;

    m_bOnLadder = false;
	m_bOnGround = false;
    m_bCollisionDisabled = false;
    m_bJumped = false;
    m_bContacted = false;
    m_eDucking = DUCKING_NONE;
    m_bIsSprinting = false;

	SetDirection( Fvector().set( 1.0f, 0.0f, 0.0f ) );

	m_vecOrigin.set( 0.0f, 0.0f, 0.0f );
    m_vecOldOrigin.set( 0.0f, 0.0f, 0.0f );
	m_vecVelocity.set( 0.0f, 0.0f, 0.0f );
    m_vecSmoothedVelocity.set( 0.0f, 0.0f, 0.0f );
    m_vecControlVelocity.set( 0.0f, 0.0f, 0.0f );
    m_vecGroundNormal.set( 0.0f, 1.0f, 0.0f );
    m_vecExternalImpusle.set( 0.0f, 0.0f, 0.0f );
    m_vecLadderNormal.set( 1.0f, 0.0f, 0.0f );
    m_vecPunchAngle.set( 0.0f, 0.0f, 0.0f );
    m_vecPunchAngleVel.set( 0.0f, 0.0f, 0.0f );

    m_flContactVelocity = 0.0f;
    m_vecHitDir.set( 0.0f, 1.0f, 0.0f );
    m_vecHitPos.set( 0.0f, 0.0f, 0.0f );
    m_nObjectID = u16(-1);
    m_eHitType = ALife::eHitTypeStrike;
    m_pfnHitCallback = nullptr;
    m_bInitiated = false;
}

void CHL2Movement::UpdateCL( void )
{
    if ( !m_bActivated )
        return;

    m_dblRemainder += Device.fTimeDelta;
	
	int numticks = 0; // how many ticks we will simulate this frame
	if ( m_dblRemainder >= m_flFrameTime )
	{
		numticks = (int)( floor( m_dblRemainder / m_flFrameTime ) );
		m_dblRemainder -= numticks * m_flFrameTime;
	}

    for ( int tick = 0; tick < numticks; tick++ )
    {
        StartMove();
        Move();
        FinishMove();

        m_nTickCount++;
    }

    if ( m_bSpeedometer )
    {
        CGameFont* pFont = UI().Font().pFontGraffiti22Russian;

        if ( pFont )
        {
            float x = Device.dwWidth * 0.5f;
            float y = Device.dwHeight * 0.75f;

            float spd = dXZMag( m_vecVelocity );

            pFont->SetAligment( CGameFont::alCenter );
            pFont->SetColor( color_rgba( 255, 255, 255, 255 ) );
            pFont->Out( x, y, "%.1f - %.1f", spd, spd * 40.0f );
        }
    }
}

void CHL2Movement::Activate( const Fvector& pos ) 
{
    if ( m_bActivated )
        return;

    Clear();
    SetPosition( pos );

    m_pPhysicsShell = create_actor_shell( 
        m_pMovControl, 
        m_pPhysicsRef, 
        ObjectContactCallback, 
        this, 
        pos, 
        m_pMovControl->Box(), 
        m_pMovControl->GetMass(), 
        m_pMovControl->GetMaterial() );

    m_bActivated = true;
}

void CHL2Movement::Deactivate() 
{
    destroy_actor_shell( m_pPhysicsShell );

    m_bActivated = false;
}

void CHL2Movement::SetMass( float mass )
{
    if ( m_pPhysicsShell )
        m_pPhysicsShell->SetMass( mass );
}

void CHL2Movement::SetMaterial( u16 material ) 
{
    if ( m_pPhysicsShell )
        m_pPhysicsShell->SetMaterial( material );
}

void CHL2Movement::SetDirection( const Fvector& dir ) 
{
    m_vecForward = dir;
	m_vecRight.crossproduct( Fvector().set( 0.0f, 1.0f, 0.0f ), m_vecForward );
    m_vecForward.normalize2();
    m_vecRight.normalize2();
}

bool CHL2Movement::GetAndResetJumped() 
{
    bool ret = m_bJumped;
    m_bJumped = false;
    return ret;
}

bool CHL2Movement::GetAndResetContacted() 
{
    bool ret = m_bContacted;
    m_bContacted = false;
    return ret;
}

EEnvironment CHL2Movement::CheckInvironment() const
{
	if ( m_bOnLadder )
        return peAtWall;
    else if ( m_bOnGround )
        return peOnGround;
    else
        return peInAir;
}

void CHL2Movement::GetGroundNormal( Fvector& norm ) const 
{
    if ( m_bOnLadder )
        norm = m_vecLadderNormal;
    else
        norm = m_vecGroundNormal;
}

void CHL2Movement::ApplyImpulse( const Fvector& dir, float force )
{
    if ( m_nExternalImpulseEndTick != u64(-1) )
        return;

    m_nExternalImpulseEndTick = m_nTickCount + TIME_TO_TICKS(0.3f);

    if ( m_bOnLadder || m_bOnGround )
        m_vecExternalImpusle.set( dir );
    else
        m_vecExternalImpusle.set( 0.0f, -1.0f, 0.0f );

    m_vecExternalImpusle.mul( force );
    m_vecExternalImpusle.mul( 1.0f / fixed_step );
    m_vecVelocity.set( 0.0f, 0.0f, 0.0f );
}

void CHL2Movement::SetPosition( const Fvector& pos )
{
    m_vecOrigin = pos;
    m_vecOldOrigin = pos;

    if ( m_pPhysicsShell )
        m_pPhysicsShell->SetPosition( pos );
}

void CHL2Movement::GetIPosition( Fvector& pos ) const
{
    pos.lerp( m_vecOldOrigin, m_vecOrigin, m_dblRemainder / m_flFrameTime );
}

void CHL2Movement::SetVelocity( const Fvector& vel ) 
{ 
    m_vecVelocity = vel; 

    if ( m_pPhysicsShell )
        m_pPhysicsShell->SetVelocity( vel );
}

void CHL2Movement::SetPhysicsRefObject( IPhysicsShellHolder* obj ) 
{ 
    m_pPhysicsRef = obj; 

    if ( m_pPhysicsShell )
        m_pPhysicsShell->SetPhysicsRefObject( m_pPhysicsRef );
}

void CHL2Movement::CollisionEnable( bool bEnable ) 
{ 
    m_bCollisionDisabled = !bEnable; 

    if ( m_pPhysicsShell )
    {
        if ( bEnable )
            m_pPhysicsShell->EnableCollision();
        else
            m_pPhysicsShell->DisableCollision();
    }
}

void CHL2Movement::StartMove( void )
{
    m_flForwardMove = 0.0f;
	m_flSideMove = 0.0f;

	if ( m_nButtons & mcFwd )
		m_flForwardMove += m_flForwardSpeed;
	
	if ( m_nButtons & mcBack )
		m_flForwardMove -= m_flForwardSpeed;
	
	if ( m_nButtons & mcRStrafe )
		m_flSideMove += m_flSideSpeed;
	
	if ( m_nButtons & mcLStrafe )
		m_flSideMove -= m_flSideSpeed;

    AvoidPushawayGeoms();

    bool bInDuck = ( m_nButtons & mcCrouch ) || m_eDucking != DUCKING_NONE || m_pMovControl->BoxID() != 0;

    m_bIsSprinting = false;

    if ( !bInDuck )
    {
	    if ( m_nButtons & mcSprint )
        {
		    m_flMaxSpeed = m_flSprintSpeed;
            m_bIsSprinting = true;
        }
	    else if ( m_nButtons & mcAccel )
        {
		    m_flMaxSpeed = m_flSlowWalkSpeed;
        }
        else
        {
            m_flMaxSpeed = m_flWalkSpeed;
        }
    }
	else
    {
		m_flMaxSpeed = m_flWalkSpeed * PLAYER_SPEED_DUCK_MODIFIER;
    }

    m_flMaxSpeed *= m_flVelocityModifier;

    float spd = ( m_flForwardMove*m_flForwardMove ) + ( m_flSideMove*m_flSideMove );
    if ( ( spd != 0.0f ) && ( spd > m_flMaxSpeed*m_flMaxSpeed ) )
    {
        float fRatio = m_flMaxSpeed / _sqrt( spd );
        m_flForwardMove *= fRatio;
        m_flSideMove    *= fRatio;    
    }

    float flControlSpeed = m_vecControlVelocity.magnitude();
    if ( flControlSpeed != 0.0f )
    {
        m_flForwardMove += m_vecControlVelocity.dotproduct( m_vecForward );
		m_flSideMove    += m_vecControlVelocity.dotproduct( m_vecRight );
        m_flMaxSpeed    = _max( m_flMaxSpeed, _min( flControlSpeed, m_flMaxControlSpeed ) );
    }

    m_nInjuriousMaterial = GAMEMTL_NONE_IDX;

    m_vecOldOrigin = m_vecOrigin;

    m_vecExternalImpusle.mul( m_flFrameTime );
    m_vecExternalImpusle.mul( 1.0f / m_pMovControl->GetMass() );
    m_vecVelocity.add( m_vecExternalImpusle );
    m_vecExternalImpusle.set( 0.0f, 0.0f, 0.0f );

    if ( !m_bOnGround )
		m_flFallVelocity = -m_vecVelocity.y;
}

void CHL2Movement::FinishMove( void )
{
    if ( m_bOnGround )
        m_flFallVelocity = 0.0f;

    if ( m_nTickCount > m_nExternalImpulseEndTick )
    {
        m_nExternalImpulseEndTick = u64(-1);
        m_vecVelocity.set( 0.0f, 0.0f, 0.0f );
    }

    m_vecSmoothedVelocity.sub( m_vecOrigin, m_vecOldOrigin );
    m_vecSmoothedVelocity.mul( 1.0f / m_flFrameTime );

    m_nOldButtons = m_nButtons;

    if ( m_pPhysicsShell )
    {
        float spd = m_vecSmoothedVelocity.magnitude();

        if ( spd > 0.01f )
        {
            m_pPhysicsShell->Enable();
            Fvector adjustedPos = m_vecSmoothedVelocity;
            adjustedPos.normalize2();
            adjustedPos.mul( ACTOR_SHELL_POSITION_ADJUSTMENT );
            adjustedPos.add( m_vecOrigin );
            m_pPhysicsShell->SetPosition( adjustedPos );
        }
        else
        {
			m_pPhysicsShell->Disable();
            m_pPhysicsShell->SetPosition( m_vecOrigin );
        }

        m_pPhysicsShell->SetVelocity( m_vecVelocity );
        m_pPhysicsShell->SetBox( m_pMovControl->Box() );
    }
}

void CHL2Movement::AvoidPushawayGeoms( void )
{
    if ( m_bCollisionDisabled )
        return;

    const Fbox& box = m_pMovControl->Box();

    Fvector ourCenter;
    ourCenter.add( box.vMin, box.vMax );
    ourCenter.mul( 0.5f );
    ourCenter.add( m_vecOrigin );

    Fvector nearestGeomPoint;
	Fvector nearestPlayerPoint;
    Fvector temp;

    Ray_t ray;
	ray.Init( m_vecOrigin, m_vecOrigin, box.vMin, box.vMax );
    CTraceFilterActorMovement traceFilter( m_pPhysicsShell );

    m_PassGeoms.clear();
    PhysicsQuery().EnumerateGeometriesAlongRay( ray, &traceFilter, m_PassGeoms );

    xr_vector<enum_geom_t>::iterator i = m_PassGeoms.begin();
	xr_vector<enum_geom_t>::iterator e = m_PassGeoms.end();

    for ( ; i != e; i++ )
	{
		enum_geom_t &enum_geom = *i;
        geom_info_t &geom_info = enum_geom.geom_info;

        dBodyID body = dGeomGetBody( geom_info.geometry );

        const float minMass = 10.0f; // minimum mass that can push a player back
		const float maxMass = 30.0f; // cap at a decently large value
		float mass = maxMass;
		if ( body )
		{
            dMass m;
            dBodyGetMass( body, &m );
			mass = m.mass;
		}
		clamp( mass, minMass, maxMass );
		
		mass = _max( mass, 0.0f );
		mass /= maxMass; // bring into a 0..1 range

        geom_info.xform.inverse_transform_tiny( temp, ourCenter );
        temp.clamp( geom_info.mins, geom_info.maxs );
        geom_info.xform.transform_tiny( nearestGeomPoint, temp );

        temp.sub( nearestGeomPoint, m_vecOrigin );
        temp.clamp( box.vMin, box.vMax );
        nearestPlayerPoint.add( temp, m_vecOrigin );

        Fvector vPushAway;
        vPushAway.sub( nearestPlayerPoint, nearestGeomPoint );
		float flDist = vPushAway.normalize2();

		const float MaxPushawayDistance = 0.1f;
		if ( flDist > MaxPushawayDistance && !IsPointInBox( Fvector().sub( nearestGeomPoint, m_vecOrigin ), box.vMin, box.vMax ) )
			continue;

        if ( vPushAway.similar( Fvector{ 0.f, 0.f, 0.f } ) )
		{
			vPushAway.sub( ourCenter, nearestGeomPoint );
            flDist = vPushAway.normalize2();
		}

		if ( vPushAway.similar( Fvector{ 0.f, 0.f, 0.f } ) )
		{
            Fvector geomCenter;
            geomCenter.add( geom_info.mins, geom_info.maxs );
            geomCenter.mul( 0.5f );
            geomCenter.add( geom_info.xform.c );
			vPushAway.sub( ourCenter, geomCenter );
            flDist = vPushAway.normalize2();
		}

        flDist = _max( flDist, 0.01f );

        float flForce = m_flPushawayForce / flDist * mass;
		flForce = _min( flForce, m_flPushawayMaxForce );

		vPushAway.mul( flForce );

		m_flForwardMove += vPushAway.dotproduct( m_vecForward );
		m_flSideMove    += vPushAway.dotproduct( m_vecRight );
	}
}

Fvector rgv3tStuckTable[54];

void CreateStuckTable( void )
{
	float x, y, z;
	int idx;
	int i;
	float yi[3];
	static int firsttime = 1;

	if ( !firsttime )
		return;

	firsttime = 0;

	ZeroMemory(rgv3tStuckTable, sizeof(rgv3tStuckTable));

	idx = 0;
	// Little Moves.

    x = z = 0;
	// Y moves
	for (y = -0.01f ; y <= 0.01f ; y += 0.01f)
	{
		rgv3tStuckTable[idx][0] = x;
		rgv3tStuckTable[idx][1] = y;
		rgv3tStuckTable[idx][2] = z;
		idx++;
	}
    y = z = 0;
	// X moves
	for (x = -0.01f ; x <= 0.01f ; x += 0.01f)
	{
		rgv3tStuckTable[idx][0] = x;
		rgv3tStuckTable[idx][1] = y;
		rgv3tStuckTable[idx][2] = z;
		idx++;
	}
	x = y = 0;
	// Z moves
	for (z = -0.01f ; z <= 0.01f ; z += 0.01f)
	{
		rgv3tStuckTable[idx][0] = x;
		rgv3tStuckTable[idx][1] = y;
		rgv3tStuckTable[idx][2] = z;
		idx++;
	}

	// Remaining multi axis nudges.
	for ( x = - 0.01f; x <= 0.01f; x += 0.02f )
	{
		for ( y = - 0.01f; y <= 0.01f; y += 0.02f )
		{
			for ( z = - 0.01f; z <= 0.01f; z += 0.02f )
			{
				rgv3tStuckTable[idx][0] = x;
				rgv3tStuckTable[idx][1] = y;
				rgv3tStuckTable[idx][2] = z;
				idx++;
			}
		}
	}

	// Big Moves.
	x = z = 0;
	yi[0] = 0.0f;
	yi[1] = 0.05f;
	yi[2] = 0.15f;

	for (i = 0; i < 3; i++)
	{
		// Y moves
		y = yi[i];
		rgv3tStuckTable[idx][0] = x;
		rgv3tStuckTable[idx][1] = y;
		rgv3tStuckTable[idx][2] = z;
		idx++;
	}

    y = z = 0;
	// X moves
	for (x = -0.05f ; x <= 0.05f ; x += 0.05f)
	{
		rgv3tStuckTable[idx][0] = x;
		rgv3tStuckTable[idx][1] = y;
		rgv3tStuckTable[idx][2] = z;
		idx++;
	}
	x = y = 0;
	// Z moves
	for (z = -0.05f ; z <= 0.05f ; z += 0.05f)
	{
		rgv3tStuckTable[idx][0] = x;
		rgv3tStuckTable[idx][1] = y;
		rgv3tStuckTable[idx][2] = z;
		idx++;
	}

	// Remaining multi axis nudges.
	for (i = 0 ; i < 3; i++)
	{
		y = yi[i];
		
		for (x = -0.05f ; x <= 0.05f ; x += 0.05f)
		{
			for (z = -0.05f ; z <= 0.05f ; z += 0.05f)
			{
				rgv3tStuckTable[idx][0] = x;
				rgv3tStuckTable[idx][1] = y;
				rgv3tStuckTable[idx][2] = z;
				idx++;
			}
		}
	}
	VERIFY( idx < sizeof(rgv3tStuckTable)/sizeof(rgv3tStuckTable[0]));
}

int CHL2Movement::CheckStuck( void )
{
	Fvector test;
    trace_t traceresult;

    CreateStuckTable();

    TestPlayerPosition( m_vecOrigin, traceresult, true );

    if ( !traceresult.DidHit() )
        return 0;

	for ( int i = 0; i < 54; i++ )
    {
		test.add( m_vecOrigin, rgv3tStuckTable[i] );
		
        TestPlayerPosition( test, traceresult, true );

		if ( !traceresult.DidHit() )
		{
			m_vecOrigin = test;
			return 0;
		}
	}

    return 1;
}

void CHL2Movement::FixPlayerCrouchStuck( bool bAccel )
{
	float i;
	Fvector test;
	trace_t dummy;

	TestPlayerPosition( m_vecOrigin, dummy );
	if ( !dummy.DidHit() )
		return;
	
    Fvector boxSize;
    m_pMovControl->Boxes()[bAccel ? 2 : 1].getsize( boxSize );

	test = m_vecOrigin;
	for ( i = 0.0f; i < boxSize.y; i += 0.05f )
	{
		m_vecOrigin.y += i;
		TestPlayerPosition( m_vecOrigin, dummy );
		if ( !dummy.DidHit() )
			return;
	}

	m_vecOrigin = test; // Failed
}

bool CHL2Movement::CanUnduck( bool bAccel )
{
	trace_t trace;
	Fvector newOrigin = m_vecOrigin;

	if ( m_bOnGround )
	{
        Fvector viewDelta;
		viewDelta.sub( m_pMovControl->Boxes()[bAccel ? 2 : 1].vMin, m_pMovControl->Boxes()[bAccel ? 1 : 0].vMin );

        newOrigin.add( viewDelta );
	}
	else
	{
		// If in air an letting go of croush, make sure we can offset origin to make
		//  up for uncrouching
        Fvector hullSizeNormal;
        hullSizeNormal.sub( m_pMovControl->Boxes()[bAccel ? 1 : 0].vMax, m_pMovControl->Boxes()[bAccel ? 1 : 0].vMin );

	    Fvector hullSizeCrouch;
        hullSizeCrouch.sub( m_pMovControl->Boxes()[bAccel ? 2 : 1].vMax, m_pMovControl->Boxes()[bAccel ? 2 : 1].vMin );

	    Fvector viewDelta;
        viewDelta.sub( hullSizeNormal, hullSizeCrouch );
        viewDelta.mul( -0.5f );

		newOrigin.add( viewDelta );
	}

    Ray_t ray;
    CTraceFilterActorMovement traceFilter( m_pPhysicsShell, m_bCollisionDisabled, false, &m_PassGeoms );

    ray.Init( m_vecOrigin, m_vecOrigin, m_pMovControl->Boxes()[bAccel ? 1 : 0].vMin, m_pMovControl->Boxes()[bAccel ? 1 : 0].vMax );
    PhysicsQuery().TraceRay( ray, &traceFilter, &trace );

    if ( trace.DidHit() )
        return false;

    ray.Init( m_vecOrigin, newOrigin, m_pMovControl->Boxes()[bAccel ? 1 : 0].vMin, m_pMovControl->Boxes()[bAccel ? 1 : 0].vMax );
    PhysicsQuery().TraceRay( ray, &traceFilter, &trace );

	if ( trace.DidHit() )
		return false;	

	return true;
}

void CHL2Movement::FinishUnDuck( bool bAccel )
{
	if ( m_bOnGround )
	{
        Fvector viewDelta;
		viewDelta.sub( m_pMovControl->Boxes()[bAccel ? 2 : 1].vMin, m_pMovControl->Boxes()[bAccel ? 1 : 0].vMin );

        m_vecOrigin.add( viewDelta );
	}
	else
	{
		// If in air an letting go of croush, make sure we can offset origin to make
		//  up for uncrouching
        Fvector hullSizeNormal;
        hullSizeNormal.sub( m_pMovControl->Boxes()[bAccel ? 1 : 0].vMax, m_pMovControl->Boxes()[bAccel ? 1 : 0].vMin );

	    Fvector hullSizeCrouch;
        hullSizeCrouch.sub( m_pMovControl->Boxes()[bAccel ? 2 : 1].vMax, m_pMovControl->Boxes()[bAccel ? 2 : 1].vMin );

	    Fvector viewDelta;
        viewDelta.sub( hullSizeNormal, hullSizeCrouch );
        viewDelta.mul( -0.5f );

		m_vecOrigin.add( viewDelta );
	}

    m_pMovControl->ActivateBox( bAccel ? 1 : 0 );
    m_flCameraHeight = m_pMovControl->Box().vMax.y;
	m_eDucking = DUCKING_NONE;
	m_flDucktime = 0.0f;
	
	// Recategorize position since ducking can change origin
	CategorizePosition();
}

void CHL2Movement::FinishDuck( bool bAccel )
{
    bool bDucked = bAccel ? m_pMovControl->BoxID() == 2 : m_pMovControl->BoxID() == 1;

    m_pMovControl->ActivateBox( bAccel ? 2 : 1 );
    m_flCameraHeight = m_pMovControl->Box().vMax.y;
	m_eDucking = DUCKING_NONE;

	if ( !bDucked )
	{
		if ( m_bOnGround )
		{
            Fvector viewDelta;
            viewDelta.sub( m_pMovControl->Boxes()[bAccel ? 2 : 1].vMin, m_pMovControl->Boxes()[bAccel ? 1 : 0].vMin );

			m_vecOrigin.sub( viewDelta );
		}
		else
		{
            Fvector hullSizeNormal;
            hullSizeNormal.sub( m_pMovControl->Boxes()[bAccel ? 1 : 0].vMax, m_pMovControl->Boxes()[bAccel ? 1 : 0].vMin );

	        Fvector hullSizeCrouch;
            hullSizeCrouch.sub( m_pMovControl->Boxes()[bAccel ? 2 : 1].vMax, m_pMovControl->Boxes()[bAccel ? 2 : 1].vMin );

	        Fvector viewDelta;
            viewDelta.sub( hullSizeNormal, hullSizeCrouch );
            viewDelta.mul( 0.5f );

			m_vecOrigin.add( viewDelta );
		}
	}

	// See if we are stuck?
	FixPlayerCrouchStuck( bAccel );

	// Recategorize position since ducking can change origin
	CategorizePosition();
}

void CHL2Movement::SetDuckedEyeOffset( float duckFraction, bool bAccel )
{
	float fMore = ( m_pMovControl->Boxes()[bAccel ? 2 : 1].vMin.y - 
                    m_pMovControl->Boxes()[bAccel ? 1 : 0].vMin.y );

	float flDuckViewOffset = m_pMovControl->Boxes()[bAccel ? 2 : 1].vMax.y;
    float flStandViewOffset = m_pMovControl->Boxes()[bAccel ? 1 : 0].vMax.y;

    m_flCameraHeight = ( ( flDuckViewOffset - fMore ) * duckFraction ) +
                        ( flStandViewOffset * ( 1 - duckFraction ) );
}

inline float SimpleSpline( float value )
{
	float valueSquared = value * value;

	// Nice little ease-in, ease-out spline-like curve
	return (3 * valueSquared - 2 * valueSquared * value);
}

void CHL2Movement::Duck( void )
{
	int buttonsChanged	= ( m_nOldButtons ^ m_nButtons );	// These buttons have changed this frame
	int buttonsPressed	=  buttonsChanged & m_nButtons;		// The changed ones still down are "pressed"
	int buttonsReleased	=  buttonsChanged & m_nOldButtons;	// The changed ones which were previously down are "released"

	bool bDucked;
    bool bAccel;
    bool bWantsToDuck;
    bool bDuckPressed;
    bool bDuckReleased;
    EDucking eNewDucking;
    EDucking eNewUnDucking;

    switch ( m_pMovControl->BoxID() )
    {
    case 0:
        bDucked = false;
        bAccel = false;
        bWantsToDuck = ( m_nButtons & mcCrouch );
        bDuckPressed = ( buttonsPressed & mcCrouch );
        bDuckReleased = false;
        eNewDucking = DUCKING_NORMAL;
        eNewUnDucking = DUCKING_NORMAL;
        break;
    case 1:
        bDucked = m_eDucking != DUCKING_ACCEL;
        bAccel = m_eDucking == DUCKING_ACCEL;
        bWantsToDuck = ( m_nButtons & mcCrouch ) && ( m_eDucking != DUCKING_ACCEL || ( m_nButtons & mcAccel ) );
        bDuckPressed = ( m_eDucking != DUCKING_NORMAL && ( buttonsPressed & mcAccel ) ) || ( m_nOldBoxID == 0 && ( m_nButtons & mcAccel ) );
        bDuckReleased = ( m_eDucking != DUCKING_ACCEL && ( buttonsReleased & mcCrouch ) ) || ( m_nOldBoxID == 2 && !( m_nButtons & mcCrouch ) );
        eNewDucking = m_eDucking != DUCKING_NONE ? m_eDucking : DUCKING_ACCEL;
        eNewUnDucking = m_eDucking != DUCKING_NONE ? m_eDucking : DUCKING_NORMAL;
        break;
    case 2:
        bDucked = true;
        bAccel = true;
        bWantsToDuck = ( m_nButtons & mcCrouch ) && ( m_nButtons & mcAccel );
        bDuckPressed = false;
        bDuckReleased = m_eDucking == DUCKING_NONE && ( ( buttonsReleased & mcCrouch ) || ( buttonsReleased & mcAccel ) );
        eNewDucking = DUCKING_ACCEL;
        eNewUnDucking = DUCKING_ACCEL;
        break;
    }

    m_nOldBoxID = m_pMovControl->BoxID();

	// Holding duck, in process of ducking or fully ducked?
	if ( bWantsToDuck || m_eDucking != DUCKING_NONE || bDucked )
	{
        // DUCK
		if ( bWantsToDuck )
		{
            // Have the duck button pressed, but the player currently isn't in the duck position.
			if ( bDuckPressed )
			{
				// Use 1 second so super long jump will work
			    m_flDucktime = 1000.0f;
				m_eDucking   = eNewDucking;
                bDucked = m_pMovControl->BoxID() == 2 || ( m_pMovControl->BoxID() == 1 && m_eDucking != DUCKING_ACCEL );
                bAccel = m_pMovControl->BoxID() == 2 || ( m_pMovControl->BoxID() == 1 && m_eDucking == DUCKING_ACCEL );
			}

			float duckmilliseconds = _max( 0.0f, 1000.0f - (float)m_flDucktime );
			float duckseconds = duckmilliseconds / 1000.0f; 

			if ( m_eDucking != DUCKING_NONE )
			{
				// Finish ducking immediately if duck time is over or not on ground
				if ( ( duckseconds > m_flDuckSpeed ) || 
					!m_bOnGround ||
					bDucked )
				{
					FinishDuck( bAccel );
				}
				else
				{
					// Calc parametric time
					float duckFraction = SimpleSpline( duckseconds / m_flDuckSpeed );
					SetDuckedEyeOffset( duckFraction, bAccel );
				}
			}
		}
		else
		{
			if ( bDuckReleased )
			{
				// Use 1 second so super long jump will work
				m_flDucktime = 1000.0f;
				m_eDucking    = eNewUnDucking;  // or unducking
                bDucked = m_pMovControl->BoxID() == 2 || ( m_pMovControl->BoxID() == 1 && m_eDucking != DUCKING_ACCEL );
                bAccel = m_pMovControl->BoxID() == 2 || ( m_pMovControl->BoxID() == 1 && m_eDucking == DUCKING_ACCEL );
			}

			float duckmilliseconds = _max( 0.0f, 1000.0f - (float)m_flDucktime );
			float duckseconds = duckmilliseconds / 1000.0f;

			if ( CanUnduck( bAccel ) )
			{
				if ( m_eDucking != DUCKING_NONE || 
					bDucked ) // or unducking
				{
					// Finish ducking immediately if duck time is over or not on ground
					if ( ( duckseconds > m_flUnDuckSpeed ) || !m_bOnGround )
					{
						FinishUnDuck( bAccel );
					}
					else
					{
						// Calc parametric time
						float duckFraction = SimpleSpline( 1.0f - ( duckseconds / m_flUnDuckSpeed ) );
						SetDuckedEyeOffset( duckFraction, bAccel );
					}
				}
			}
			else
			{
				// Still under something where we can't unduck, so make sure we reset this timer so
				//  that we'll unduck once we exit the tunnel, etc.
				m_flDucktime = 1000.0f;
			}
		}
	}
}

void CHL2Movement::DecayPunchAngle( void )
{
    if ( m_vecPunchAngle.square_magnitude() > 0.001 || m_vecPunchAngleVel.square_magnitude() > 0.001 )
	{
		m_vecPunchAngle.add( Fvector().mul( m_vecPunchAngleVel, m_flFrameTime ) );
		float damping = 1 - (PUNCH_DAMPING * m_flFrameTime);
		
		if ( damping < 0 )
		{
			damping = 0;
		}
		m_vecPunchAngleVel.mul( damping );
		
		// torsional spring
		// UNDONE: Per-axis spring constant?
		float springForceMagnitude = PUNCH_SPRING_CONSTANT * m_flFrameTime;
		clamp( springForceMagnitude, 0.f, 2.f );
		m_vecPunchAngleVel.sub( Fvector().mul( m_vecPunchAngle, springForceMagnitude ) );

        // don't wrap around
        clamp( m_vecPunchAngle.x, -89.f, 89.f );
        clamp( m_vecPunchAngle.y, -179.f, 179.f );
        clamp( m_vecPunchAngle.z, -89.f, 89.f );
	}
	else
	{
		m_vecPunchAngle.set( 0, 0, 0 );
		m_vecPunchAngleVel.set( 0, 0, 0 );
	}
}

void CHL2Movement::ReduceTimers( void )
{
    float frame_msec = 1000.0f * m_flFrameTime;

    if ( m_flDucktime > 0.0f )
    {
	    m_flDucktime -= frame_msec;
	    if ( m_flDucktime < 0.0f )
		    m_flDucktime = 0.0f;
    }
}

void CHL2Movement::Move( void )
{
    DecayPunchAngle();

    ReduceTimers();

    if ( CheckStuck() )
        return;

    Duck();

    if ( !LadderMove() && m_bOnLadder )
		m_bOnLadder = false;

    if ( m_bOnLadder )
    {
	    if (m_nButtons & mcJump)
	    {
		    CheckJumpButton();
	    }

	    TryPlayerMove();

        // Set final flags.
	    CategorizePosition();
    }
    else
    {
        StartGravity();

	    // Was jump button pressed?
	    if (m_nButtons & mcJump)
	    {
		    CheckJumpButton();
	    }
	
	    // Fricion is handled before we add in any base velocity. That way, if we are on a conveyor, 
	    //  we don't slow when standing still, relative to the conveyor.
	    if (m_bOnGround)
	    {
		    m_vecVelocity[1] = 0;
		    Friction();
	    }

	    // Make sure velocity is valid.
	    CheckVelocity();

	    if (m_bOnGround)
	    {
		    WalkMove();
	    }
	    else
	    {
		    AirMove();  // Take into account movement when in air.
	    }

	    // Set final flags.
	    CategorizePosition();

	    // Make sure velocity is valid.
	    CheckVelocity();

	    // Add any remaining gravitational component.
	    FinishGravity();

	    // If we are on ground, no downward velocity.
	    if (m_bOnGround)
	    {
		    m_vecVelocity[1] = 0;
	    }
        CheckFalling();
    }
}

void CHL2Movement::TryPlayerMove( Fvector *pFirstDest, trace_t *pFirstTrace )
{
	int			bumpcount, numbumps;
	Fvector		dir;
	float		d;
	int			numplanes;
	Fvector		planes[MAX_CLIP_PLANES];
	Fvector		primal_velocity, original_velocity;
	Fvector		new_velocity;
	int			i, j;
	trace_t		pm;
	Fvector		end;
	float		time_left, allFraction;		
	
	numbumps  = 4;           // Bump up to four times
	numplanes = 0;           //  and not sliding along any planes

	original_velocity = m_vecVelocity; // Store original velocity
	primal_velocity = m_vecVelocity;
	
	allFraction = 0;
	time_left = m_flFrameTime;   // Total time for this movement operation.

	new_velocity.set( 0.0f, 0.0f, 0.0f );

	for (bumpcount=0 ; bumpcount < numbumps; bumpcount++)
	{
		if ( m_vecVelocity.magnitude() == 0.0 )
			break;

		// Assume we can move all the way from the current origin to the
		//  end point.
		end.mad( m_vecOrigin, m_vecVelocity, time_left );

		// See if we can make it from origin to end point.
		// If their velocity Y is 0, then we can avoid an extra trace here during WalkMove.
		if ( pFirstDest && end == *pFirstDest )
			pm = *pFirstTrace;
		else
			TracePlayerBBox( m_vecOrigin, end, pm );

		allFraction += pm.fraction;

		// If we moved some portion of the total distance, then
		//  copy the end position into the pmove.origin and 
		//  zero the plane counter.
		if( pm.fraction > 0 )
		{	
			if ( numbumps > 0 && pm.fraction == 1 )
			{
				// There's a precision issue with terrain tracing that can cause a swept box to successfully trace
				// when the end position is stuck in the triangle.  Re-run the test with an uswept box to catch that
				// case until the bug is fixed.
				// If we detect getting stuck, don't allow the movement
				trace_t stuck;
				TracePlayerBBox( pm.endpos, pm.endpos, stuck );
				if ( stuck.startsolid || stuck.fraction != 1.0f )
				{
					m_vecVelocity.set( 0.0f, 0.0f, 0.0f );
					break;
				}
			}

			// actually covered some distance
			m_vecOrigin = pm.endpos;
			original_velocity = m_vecVelocity;
			numplanes = 0;
		}

		// If we covered the entire distance, we are done
		//  and can return.
		if (pm.fraction == 1)
		{
			 break;		// moved the entire distance
		}

		// Reduce amount of m_flFrameTime left by total time left * fraction
		//  that we covered.
		time_left -= time_left * pm.fraction;

		// Did we run out of planes to clip against?
		if (numplanes >= MAX_CLIP_PLANES)
		{	
			// this shouldn't really happen
			//  Stop our movement if so.
			m_vecVelocity.set( 0.0f, 0.0f, 0.0f );
			break;
		}

		// Set up next clipping plane
		planes[numplanes] = pm.plane.normal;
		numplanes++;

		// modify original_velocity so it parallels all of the clip planes
		//

		// reflect player velocity 
		// Only give this a try for first impact plane because you can get yourself stuck in an acute corner by jumping in place
		//  and pressing forward and nobody was really using this bounce/reflection feature anyway...
		if ( numplanes == 1 &&
            !m_bOnLadder &&
			!m_bOnGround )	
		{
			for ( i = 0; i < numplanes; i++ )
			{
				if ( planes[i][1] >= m_flWalkableNormal )
				{
					// floor or slope
					ClipVelocity( original_velocity, planes[i], new_velocity, 1 );
					original_velocity = new_velocity;
				}
				else
				{
					ClipVelocity( original_velocity, planes[i], new_velocity, 1.0 + m_flBounce * (1 - m_surfaceFriction) );
				}
			}
			
			m_vecVelocity = new_velocity;
			original_velocity = new_velocity;
		}
		else
		{
			for (i=0 ; i < numplanes ; i++)
			{
				ClipVelocity (
					original_velocity,
					planes[i],
					m_vecVelocity,
					1);

				for (j=0 ; j<numplanes ; j++)
					if (j != i)
					{
						// Are we now moving against this plane?
						if (m_vecVelocity.dotproduct(planes[j]) < 0)
							break;	// not ok
					}
				if (j == numplanes)  // Didn't have to clip, so we're ok
					break;
			}
			
			// Did we go all the way through plane set
			if (i != numplanes)
			{	// go along this plane
				// pmove.velocity is set in clipping call, no need to set again.
				;  
			}
			else
			{	// go along the crease
				if (numplanes != 2)
				{
					m_vecVelocity.set( 0.0f, 0.0f, 0.0f );
					break;
				}
				dir.crossproduct(planes[0], planes[1]);
				dir.normalize2();
				d = dir.dotproduct(m_vecVelocity);
				m_vecVelocity.mul(dir, d);
			}

			//
			// if original velocity is against the original velocity, stop dead
			// to avoid tiny occilations in sloping corners
			//
			d = m_vecVelocity.dotproduct(primal_velocity);
			if (d <= 0)
			{
				m_vecVelocity.set( 0.0f, 0.0f, 0.0f );
				break;
			}
		}
	}

	if ( allFraction == 0 )
	{
		m_vecVelocity.set( 0.0f, 0.0f, 0.0f );
	}

    // Check if they slammed into a wall
	float fLateralStoppingAmount = dXZMag( primal_velocity ) - dXZMag( m_vecVelocity );
	if ( fLateralStoppingAmount > PLAYER_MAX_SAFE_FALL_SPEED )
	    PlayerRoughLandingEffects();
}

void CHL2Movement::ClipVelocity( Fvector& in, Fvector& normal, Fvector& out, float overbounce )
{
	float	backoff;
	float	change;
	int		i;
	
	// Determine how far along plane to slide based on incoming direction.
	backoff = in.dotproduct(normal) * overbounce;

	for (i=0 ; i<3 ; i++)
	{
		change = normal[i]*backoff;
		out[i] = in[i] - change; 
	}
	
	// iterate once to make sure we aren't still moving through the plane
	float adjust = out.dotproduct( normal );
	if( adjust < 0.0f )
	{
		out.sub( Fvector().mul( normal, adjust ) );
	}
}

bool CHL2Movement::LadderMove( void )
{
	trace_t pm;
	Fvector wishdir;
	Fvector end;

	// If I'm already moving on a ladder, use the previous ladder direction
	if ( m_bOnLadder )
	{
		wishdir = -m_vecLadderNormal;
	}
	else
	{
		// otherwise, use the direction player is attempting to move
		if ( m_flForwardMove || m_flSideMove )
		{
			for (int i=0 ; i<3 ; i++)       // Determine x and y parts of velocity
				wishdir[i] = m_vecForward[i]*m_flForwardMove + m_vecRight[i]*m_flSideMove;

			wishdir.normalize2();
		}
		else
		{
			// Player is not attempting to move, no ladder behavior
			return false;
		}
	}

	// wishdir points toward the ladder if any exists
	end.mad( m_vecOrigin, wishdir, m_flLadderDistance );
	TracePlayerBBox( m_vecOrigin, end, pm );

	// no ladder in that direction, return
	if ( pm.fraction == 1.0f || !IsLeaderGeomShell( pm.object ) )
		return false;

	m_bOnLadder = true;

	m_vecLadderNormal = pm.plane.normal;

	// On ladder, convert movement to be relative to the ladder

    float climbSpeed = m_flClimbSpeed;

    if ( m_nButtons & mcCrouch )
		climbSpeed *= PLAYER_SPEED_CLIMB_MODIFIER;

	float forwardSpeed = 0, rightSpeed = 0;
	if ( m_nButtons & mcBack )
		forwardSpeed -= climbSpeed;
	
	if ( m_nButtons & mcFwd )
		forwardSpeed += climbSpeed;
	
	if ( m_nButtons & mcLStrafe )
		rightSpeed -= climbSpeed;
	
	if ( m_nButtons & mcRStrafe )
		rightSpeed += climbSpeed;

	if ( m_nButtons & mcJump )
	{
		m_bOnLadder = false;

        m_vecVelocity.mul( pm.plane.normal, m_flLadderLeaveSpeed );
	}
	else
	{
		if ( forwardSpeed != 0 || rightSpeed != 0 )
		{
			Fvector velocity, perp, cross, lateral, tmp;

			//ALERT(at_console, "pev %.2f %.2f %.2f - ",
			//	pev->velocity.x, pev->velocity.y, pev->velocity.z);
			// Calculate player's intended velocity
			//Vector velocity = (forward * gpGlobals->v_forward) + (right * gpGlobals->v_right);
			velocity.mul( m_vecForward, forwardSpeed );
			velocity.mad( velocity, m_vecRight, rightSpeed );

			// Perpendicular in the ladder plane
			tmp.set( 0.0f, 1.0f, 0.0f );
			perp.crossproduct( tmp, pm.plane.normal );
			perp.normalize2();

			// decompose velocity into ladder plane
			float normal = velocity.dotproduct( pm.plane.normal );

			// This is the velocity into the face of the ladder
			cross.mul( pm.plane.normal, normal );

			// This is the player's additional velocity
			lateral.sub( velocity, cross );

			// This turns the velocity into the face of the ladder into velocity that
			// is roughly vertically perpendicular to the face of the ladder.
			// NOTE: It IS possible to face up and move down or face down and move up
			// because the velocity is a sum of the directional velocity and the converted
			// velocity through the face of the ladder -- by design.
			tmp.crossproduct( pm.plane.normal, perp );

			m_vecVelocity.mad( lateral, tmp, -normal );

			if ( m_bOnGround && normal > 0 )	// On ground moving away from the ladder
			{
				m_vecVelocity.mad( m_vecVelocity, pm.plane.normal, m_flClimbSpeed );
			}
			//pev->velocity = lateral - (CrossProduct( trace.vecPlaneNormal, perp ) * normal);
		}
		else
		{
			m_vecVelocity.set( 0.0f, 0.0f, 0.0f );
		}
	}

	return true;
}

void CHL2Movement::WalkMove( void )
{
	Fvector wishvel;
	float spd;
	Fvector wishdir;
	float wishspeed;

	Fvector dest;
	trace_t pm;
	Fvector forward, right;
	
	forward = m_vecForward;
	right = m_vecRight;
	
	// Zero out y components of movement vectors
	if ( forward[1] != 0 )
	{
		forward[1] = 0;
		forward.normalize2();
	}

	if ( right[1] != 0 )
	{
		right[1] = 0;
		right.normalize2();
	}
		
	// Determine x and z parts of velocity
	wishvel[0] = forward[0]*m_flForwardMove + right[0]*m_flSideMove;
	wishvel[2] = forward[2]*m_flForwardMove + right[2]*m_flSideMove;
	wishvel[1] = 0;     // Zero out y part of velocity

	wishdir = wishvel;  // Determine maginitude of speed of move
	wishspeed = wishdir.normalize2();

	//
	// Clamp to server defined max speed
	//
	if ((wishspeed != 0.0f) && (wishspeed > m_flMaxSpeed))
	{
		wishvel.mul( m_flMaxSpeed/wishspeed );
		wishspeed = m_flMaxSpeed;
	}

	// Set pmove velocity
	m_vecVelocity[1] = 0;
	Accelerate ( wishdir, wishspeed, m_flAccelerate );
	m_vecVelocity[1] = 0;

	spd = m_vecVelocity.magnitude();

	if ( spd < 0.01f )
	{
		m_vecVelocity.set( 0.0f, 0.0f, 0.0f );
		return;
	}

	// first try just moving to the destination	
	dest[0] = m_vecOrigin[0] + m_vecVelocity[0]*m_flFrameTime;
	dest[2] = m_vecOrigin[2] + m_vecVelocity[2]*m_flFrameTime;	
	dest[1] = m_vecOrigin[1];

	// first try moving directly to the next spot
	TracePlayerBBox( m_vecOrigin, dest, pm );

	if ( pm.fraction == 1 )
	{
		m_vecOrigin = pm.endpos;
		StayOnGround();
		return;
	}

	StepMove( dest, pm );
	StayOnGround();
}

void CHL2Movement::Accelerate( Fvector& wishdir, float wishspeed, float accel )
{
	int i;
	float addspeed, accelspeed, currentspeed;

	// See if we are changing direction a bit
	currentspeed = m_vecVelocity.dotproduct(wishdir);

	// Reduce wishspeed by the amount of veer.
	addspeed = wishspeed - currentspeed;

	// If not going to add any speed, done.
	if (addspeed <= 0)
		return;

	// Determine amount of accleration.
	accelspeed = accel * m_flFrameTime * wishspeed * m_surfaceFriction;

	// Cap at addspeed
	if (accelspeed > addspeed)
		accelspeed = addspeed;

	// Adjust velocity.
	for (i=0 ; i<3 ; i++)
	{
		m_vecVelocity[i] += accelspeed * wishdir[i];	
	}
}

void CHL2Movement::StepMove( Fvector &vecDestination, trace_t &trace )
{
	Fvector vecEndPos = vecDestination;
	
	// Try sliding forward both on ground and up 16 pixels
	//  take the move that goes farthest
	Fvector vecPos = m_vecOrigin;
	Fvector vecVel = m_vecVelocity;

	// Slide move down.
	TryPlayerMove( &vecEndPos, &trace );
	
	// Down results.
	Fvector vecDownPos = m_vecOrigin;
	Fvector vecDownVel = m_vecVelocity;

	// Reset original values.
	m_vecOrigin = vecPos;
	m_vecVelocity = vecVel;

	// Move up a stair height.
	vecEndPos = m_vecOrigin;
	vecEndPos.y += m_flStepSize;
	
	TracePlayerBBox( m_vecOrigin, vecEndPos, trace );

	m_vecOrigin = trace.endpos;

	// Slide move up.
	TryPlayerMove();

	vecEndPos = m_vecOrigin;
	vecEndPos.y -= m_flStepSize;
	
	TracePlayerBBox( m_vecOrigin, vecEndPos, trace );

	// If we are not on the ground any more then use the original movement attempt.
	if ( !trace.DidHit() || trace.plane.normal[1] < m_flWalkableNormal )
	{
		m_vecOrigin = vecDownPos;
		m_vecVelocity = vecDownVel;
		return;
	}

	m_vecOrigin = trace.endpos;
	
	// Copy this origin to up.
	Fvector vecUpPos = m_vecOrigin;

	// decide which one went farther
	float flDownDist = ( vecDownPos.x - vecPos.x ) * ( vecDownPos.x - vecPos.x ) + ( vecDownPos.z - vecPos.z ) * ( vecDownPos.z - vecPos.z );
	float flUpDist = ( vecUpPos.x - vecPos.x ) * ( vecUpPos.x - vecPos.x ) + ( vecUpPos.z - vecPos.z ) * ( vecUpPos.z - vecPos.z );
	if ( flDownDist > flUpDist )
	{
		m_vecOrigin = vecDownPos;
		m_vecVelocity = vecDownVel;
	}
	else 
	{
		// copy y value from slide move
		m_vecVelocity.y = vecDownVel.y;
	}
}

void CHL2Movement::StayOnGround( void )
{
	trace_t trace;
	Fvector start = m_vecOrigin;
	Fvector end = m_vecOrigin;
	start.y += 0.05;
	end.y -= m_flStepSize;

	// See how far up we can go without getting stuck

	TracePlayerBBox( m_vecOrigin, start, trace );
	start = trace.endpos;

	// using trace.startsolid is unreliable here, it doesn't get set when
	// tracing bounding box vs. terrain

	// Now trace down from a known safe position
	TracePlayerBBox( start, end, trace );
	if ( trace.fraction > 0.0f &&			// must go somewhere
		trace.fraction < 1.0f &&			// must hit something
		!trace.startsolid &&				// can't be embedded in a solid
		trace.plane.normal[1] >= m_flWalkableNormal ) // can't hit a steep slope that we can't stand on anyway
	{
		m_vecOrigin = trace.endpos;
	}
}

void CHL2Movement::Friction( void )
{
	float	speed, newspeed, control;
	float	friction;
	float	drop;
	
	// Calculate speed
	speed = m_vecVelocity.magnitude();
	
	// If too slow, return
	if (speed < 0.001f)
	{
		return;
	}

	drop = 0;

	// apply ground friction
	if (m_bOnGround)  // On an entity that is the ground
	{
		friction = m_flFriction * m_surfaceFriction;

		// Bleed off some speed, but if we have less than the bleed
		//  threshold, bleed the threshold amount.
		control = (speed < m_flStopSpeed) ? m_flStopSpeed : speed;

		// Add the amount to the drop amount.
		drop += control*friction*m_flFrameTime;
	}

	// scale the velocity
	newspeed = speed - drop;
	if (newspeed < 0)
		newspeed = 0;

	if ( newspeed != speed )
	{
		// Determine proportion of old speed we are using.
		newspeed /= speed;
		// Adjust velocity according to proportion.
		m_vecVelocity.mul( newspeed );
	}
}

void CHL2Movement::AirMove( void )
{
	Fvector		wishvel;
	Fvector		wishdir;
	float		wishspeed;
	Fvector 	forward, right;
	
	forward = m_vecForward;
	right = m_vecRight;

	// Zero out y components of movement vectors
	forward[1] = 0;
	right[1]   = 0;
	forward.normalize2();   // Normalize remainder of vectors
	right.normalize2();     // 

	// Determine x and z parts of velocity
	wishvel[0] = forward[0]*m_flForwardMove + right[0]*m_flSideMove;
	wishvel[2] = forward[2]*m_flForwardMove + right[2]*m_flSideMove;
	wishvel[1] = 0;         // Zero out y part of velocity

	wishdir = wishvel;   // Determine maginitude of speed of move
	wishspeed = wishdir.normalize2();

	//
	// clamp to server defined max speed
	//
	if ( wishspeed != 0 && (wishspeed > m_flMaxSpeed))
	{
		wishvel.mul( m_flMaxSpeed/wishspeed );
		wishspeed = m_flMaxSpeed;
	}
	
	AirAccelerate( wishdir, wishspeed, m_flAirAccelerate );

	TryPlayerMove();
}

void CHL2Movement::AirAccelerate( Fvector& wishdir, float wishspeed, float accel )
{
	int i;
	float addspeed, accelspeed, currentspeed;
	float wishspd;

	wishspd = wishspeed;

	// Cap speed
	if ( wishspd > m_flAirMaxWishSpeed )
		wishspd = m_flAirMaxWishSpeed;

	// Determine veer amount
	currentspeed = m_vecVelocity.dotproduct(wishdir);

	// See how much to add
	addspeed = wishspd - currentspeed;

	// If not adding any, done.
	if (addspeed <= 0)
		return;

	// Determine acceleration speed after acceleration
	accelspeed = accel * wishspeed * m_flFrameTime * m_surfaceFriction;

	// Cap it
	if (accelspeed > addspeed)
		accelspeed = addspeed;
	
	// Adjust pmove vel.
	for (i=0 ; i<3 ; i++)
	{
		m_vecVelocity[i] += accelspeed * wishdir[i];
	}
}

void CHL2Movement::StartGravity( void )
{
    if ( m_pMovControl->IsAffectedByGravity() == FALSE )
        return;

	// Add gravity so they'll be in the correct position during movement
	// yes, this 0.5 looks wrong, but it's not.  
	m_vecVelocity[1] -= ( m_flGravity * 0.5 * m_flFrameTime );
	
	CheckVelocity();
}

void CHL2Movement::FinishGravity( void )
{
    if ( m_pMovControl->IsAffectedByGravity() == FALSE )
        return;

	// Get the correct velocity for the end of the dt 
  	m_vecVelocity[1] -= ( m_flGravity * 0.5 * m_flFrameTime );

	CheckVelocity();
}

void CHL2Movement::CheckVelocity( void )
{
	int i;

	//
	// bound velocity
	//

	Fvector org = m_vecOrigin;

	for (i=0; i < 3; i++)
	{
		// See if it's bogus.
		if (!_valid(m_vecVelocity[i]))
		{
			m_vecVelocity[i] = 0;
		}

		if (!_valid(org[i]))
		{
			org[ i ] = 0;
			m_vecOrigin = org;
		}

		// Bound it.
		if (m_vecVelocity[i] > m_flMaxVelocity) 
		{
			m_vecVelocity[i] = m_flMaxVelocity;
		}
		else if (m_vecVelocity[i] < -m_flMaxVelocity)
		{
			m_vecVelocity[i] = -m_flMaxVelocity;
		}
	}
}

void CHL2Movement::CheckJumpButton( void )
{
	// No more effect
 	if ( !m_bOnGround )
		return;		// in air, so no effect

	if ( ( m_nOldButtons & mcJump ) && !m_bEnableBHop )
		return;		// don't pogo stick

	// In the air now.
	m_bOnGround = false;
    m_bJumped = true;

    // Acclerate upward
	// If we are ducking...
	if ( m_eDucking != DUCKING_NONE || m_pMovControl->BoxID() != 0 )
	{
		// d = 0.5 * g * t^2		- distance traveled with linear accel
		// t = sqrt(2.0 * 45 / g)	- how long to fall 45 units
		// v = g * t				- velocity at the end (just invert it to jump up that high)
		// v = g * sqrt(2.0 * 45 / g )
		// v^2 = g * g * 2.0 * 45 / g
		// v = sqrt( g * 2.0 * 45 )
		
		m_vecVelocity[1] = m_flJumpPower;  // 2 * gravity * height
	}
	else
	{
		m_vecVelocity[1] += m_flJumpPower;  // 2 * gravity * height
	}

    if ( m_nABHMode != 0 )
    {
	    Fvector vecForward = m_vecForward;
	    vecForward.y = 0;
	    vecForward.normalize2();

        float flCurrentVelMag = dXZMag( m_vecVelocity );

	    // We give a certain percentage of the current forward movement as a bonus to the jump speed.  That bonus is clipped
	    // to not accumulate over time.
	    float flSpeedBoostPerc = ( !m_bIsSprinting && m_pMovControl->BoxID() == 0 ) ? 0.5f : 0.1f;
	    float flSpeedAddition = _abs( m_flForwardMove * flSpeedBoostPerc );
	    float flMaxSpeed = m_flMaxSpeed + ( m_flMaxSpeed * flSpeedBoostPerc );
	    float flNewSpeed = ( flSpeedAddition + flCurrentVelMag );

	    // If we're over the maximum, we want to only boost as much as will get us to the goal speed
	    if ( flNewSpeed > flMaxSpeed )
	    {
		    flSpeedAddition -= flNewSpeed - flMaxSpeed;
	    }

	    if ( m_flForwardMove < 0.0f )
		    flSpeedAddition *= -1.0f;

        Fvector vecNewVelocity;
        vecNewVelocity.mul( vecForward, flSpeedAddition );
        vecNewVelocity.add( m_vecVelocity );

        float flNewVelMag = dXZMag( vecNewVelocity );

        if ( m_nABHMode == 1 || flNewVelMag >= flCurrentVelMag )
	        m_vecVelocity = vecNewVelocity;
    }

	FinishGravity();
}

void CHL2Movement::CategorizePosition( void )
{
    const Fbox& box = m_pMovControl->Box();

	Fvector point;
	trace_t pm;

	// Reset this each time we-recategorize, otherwise we have bogus friction when we jump into water and plunge downward really quickly
	m_surfaceFriction = 1.0f;

	// if the player hull point one unit down is solid, the player
	// is on ground
	
	// see if standing on something solid	

	float flOffset = 0.05f;

	point[0] = m_vecOrigin[0];
	point[1] = m_vecOrigin[1] - flOffset;
	point[2] = m_vecOrigin[2];

	Fvector bumpOrigin;
	bumpOrigin = m_vecOrigin;

	// Was on ground, but now suddenly am not
	if ( ( m_vecVelocity[1] > m_flNonJumpVelocity && !m_bStickToGround ) ||
         ( m_vecVelocity[1] > 0.0f && m_bOnLadder ) )   
	{
		m_bOnGround = false;
	}
	else
	{
		// Try and move down.
		TryTouchGround( bumpOrigin, point, box.vMin, box.vMax, pm );

		// Was on ground, but now suddenly am not.  If we hit a steep plane, we are not on ground
		if ( !pm.DidHit() || pm.plane.normal[1] < m_flStandableNormal )
		{
			// Test four sub-boxes, to see if any of them would have found shallower slope we could actually stand on
			TryTouchGroundInQuadrants( bumpOrigin, point, pm );

			if ( !pm.DidHit() || pm.plane.normal[1] < m_flStandableNormal )
			{
				m_bOnGround = false;
				// probably want to add a check for a +y velocity too!
				if ( m_vecVelocity.y > 0.0f )
				{
					m_surfaceFriction = 0.25f;
				}
			}
			else
			{
				m_bOnGround = true;
                CategorizeGroundSurface( pm );
			}
		}
		else
		{
			m_bOnGround = true;  // Otherwise, point to index of ent under us.
            CategorizeGroundSurface( pm );
		}
	}
}

void CHL2Movement::CategorizeGroundSurface( trace_t &pm )
{
    *m_pLastMaterial = pm.material;

    if ( GMLib.GetMaterialByIdx( *m_pLastMaterial )->Flags.test( SGameMtl::flActorObstacle ) )
    {
        trace_t trace;
        Fvector end = m_vecOrigin;
        end.y -= 0.25f;

        TracePlayerBBox( m_vecOrigin, end, trace, true );

        if ( trace.DidHit() )
            *m_pLastMaterial = trace.material;
    }

    if ( GMLib.GetMaterialByIdx( *m_pLastMaterial )->Flags.test( SGameMtl::flInjurious ) )
        m_nInjuriousMaterial = *m_pLastMaterial;

    m_vecGroundNormal = pm.plane.normal;
}

void CHL2Movement::CheckFalling( void )
{
	// this function really deals with landing, not falling, so early out otherwise
	if ( !m_bOnGround || m_flFallVelocity <= 0.0f )
		return;

    if ( m_flFallVelocity >= PLAYER_FALL_PUNCH_THRESHOLD )
		PlayerRoughLandingEffects();

	m_bContacted = true;

    dVector3 vel = { 0.0f, -m_flFallVelocity, 0.0f };

    UpdateStaticDamage( 
        vel, 
        cast_fp( m_vecGroundNormal ),
        cast_fp( m_vecOrigin ),
        GMLib.GetMaterialByIdx( *m_pLastMaterial ) );

	//
	// Clear the fall velocity so the impact doesn't happen again.
	//
	m_flFallVelocity = 0.0f;
}

void CHL2Movement::PlayerRoughLandingEffects( void )
{
	//
	// Knock the screen around a little bit, temporary effect.
	//
	m_vecPunchAngle.z = m_flFallVelocity * 0.52;

    if ( m_vecPunchAngle.x > 8 )
	{
        m_vecPunchAngle.x = 8;
	}
}

void CHL2Movement::TestPlayerPosition( const Fvector& pos, trace_t& pm, bool bWorldOnly )
{
    const Fbox& box = m_pMovControl->Box();

	Ray_t ray;
	ray.Init( pos, pos, box.vMin, box.vMax );
    CTraceFilterActorMovement traceFilter( m_pPhysicsShell, bWorldOnly ? true : m_bCollisionDisabled, false, &m_PassGeoms );

    PhysicsQuery().TraceRay( ray, &traceFilter, &pm );
}

void CHL2Movement::TracePlayerBBox( const Fvector& start, const Fvector& end, trace_t& pm, bool bSkipActorObstacle )
{
    const Fbox& box = m_pMovControl->Box();

	Ray_t ray;
	ray.Init( start, end, box.vMin, box.vMax );
    CTraceFilterActorMovement traceFilter( m_pPhysicsShell, m_bCollisionDisabled, bSkipActorObstacle, &m_PassGeoms );

    PhysicsQuery().TraceRay( ray, &traceFilter, &pm );
}

void CHL2Movement::TryTouchGround( const Fvector& start, const Fvector& end, const Fvector& mins, const Fvector& maxs, trace_t& pm )
{
	Ray_t ray;
	ray.Init( start, end, mins, maxs );
    CTraceFilterActorMovement traceFilter( m_pPhysicsShell, m_bCollisionDisabled, false, &m_PassGeoms );

    PhysicsQuery().TraceRay( ray, &traceFilter, &pm );
}

void CHL2Movement::TryTouchGroundInQuadrants( const Fvector& start, const Fvector& end, trace_t& pm )
{
    const Fbox& box = m_pMovControl->Box();

	Fvector mins, maxs;
	Fvector minsSrc = box.vMin;
	Fvector maxsSrc = box.vMax;

	float fraction = pm.fraction;
	Fvector endpos = pm.endpos;

	// Check the -x, -z quadrant
	mins = minsSrc;
	maxs.set( _min( 0.0f, maxsSrc.x ), maxsSrc.y, _min( 0.0f, maxsSrc.z ) );
	TryTouchGround( start, end, mins, maxs, pm );
	if ( pm.DidHit() && pm.plane.normal[1] >= m_flStandableNormal)
	{
		pm.fraction = fraction;
		pm.endpos = endpos;
		return;
	}

	// Check the +x, +z quadrant
	mins.set( _max( 0.0f, minsSrc.x ), minsSrc.y, _max( 0.0f, minsSrc.z ) );
	maxs = maxsSrc;
	TryTouchGround( start, end, mins, maxs, pm );
	if ( pm.DidHit() && pm.plane.normal[1] >= m_flStandableNormal)
	{
		pm.fraction = fraction;
		pm.endpos = endpos;
		return;
	}

	// Check the -x, +z quadrant
	mins.set( minsSrc.x, minsSrc.y, _max( 0.0f, minsSrc.z ) );
	maxs.set( _min( 0.0f, maxsSrc.x ), maxsSrc.y, maxsSrc.z );
	TryTouchGround( start, end, mins, maxs, pm );
	if ( pm.DidHit() && pm.plane.normal[1] >= m_flStandableNormal)
	{
		pm.fraction = fraction;
		pm.endpos = endpos;
		return;
	}

	// Check the +x, -z quadrant
	mins.set( _max( 0.0f, minsSrc.x ), minsSrc.y, minsSrc.z );
	maxs.set( maxsSrc.x, maxsSrc.y, _min( 0.0f, maxsSrc.z ) );
	TryTouchGround( start, end, mins, maxs, pm );
	if ( pm.DidHit() && pm.plane.normal[1] >= m_flStandableNormal)
	{
		pm.fraction = fraction;
		pm.endpos = endpos;
		return;
	}

	pm.fraction = fraction;
	pm.endpos = endpos;
}

void CHL2Movement::UpdateStaticDamage( 
    const dReal* vel, const dReal* norm, const dReal* pos, SGameMtl* tri_material )
{
    dReal vel_sqr = dDOT( vel, vel );
    dReal contact_vel;

    if ( tri_material->Flags.test( SGameMtl::flPassable ) )
    {
        contact_vel = dSqrt( vel_sqr );
    }
    else
    {
        dReal proj = dFabs( dDOT( vel, norm ) );
        contact_vel = dSqrt( vel_sqr - proj * proj ) * tri_material->fPHFriction;
        contact_vel = _max( contact_vel, proj );
    }

    contact_vel *= tri_material->fBounceDamageFactor;

    if ( contact_vel > m_flContactVelocity )
    {
        m_flContactVelocity = contact_vel;
        m_vecHitDir = cast_fv( norm );
        m_vecHitPos = cast_fv( pos );
        m_nObjectID = u16(-1);
        m_pfnHitCallback = nullptr;
    }
}

void CHL2Movement::UpdateDynamicDamage( 
    IPhysicsShellHolder* obj, const dReal* vel, const dReal* norm, const dReal* pos, SGameMtl* obj_material, dBodyID body )
{
    VERIFY( obj );
    if ( obj->ObjectGetDestroy() )
        return;

    dReal mass = m_pMovControl->GetMass();
    dMass m;
    dBodyGetMass( body, &m );
    dReal obj_mass = m.mass;

    const dReal* obj_vel = dBodyGetLinearVel( body );
    dReal norm_vel = dDOT( vel, norm );
    dReal norm_obj_vel = dDOT( obj_vel, norm );

    if ( norm_vel > norm_obj_vel )
        return;

    dReal ke_self = norm_vel * norm_vel * mass / 2.f;
    dReal ke_obj = norm_obj_vel * norm_obj_vel * obj_mass / 2.f;

    dVector3 pc = {
        vel[0] * mass + obj_vel[0] * obj_mass, 
        vel[1] * mass + obj_vel[1] * obj_mass,
        vel[2] * mass + obj_vel[2] * obj_mass };
    dReal pc_norm = dDOT( pc, norm );
    dReal free_energy = pc_norm * pc_norm / ( mass + obj_mass ) / 2.f;

    dReal damage_factor = m_pMovControl->CollisionDamageFactor();
    damage_factor *= damage_factor;

    dReal accepted_energy = ke_self * damage_factor + ke_obj * object_damage_factor - free_energy;
    dReal contact_vel = 0.f;

    if ( accepted_energy > 0.f )
        contact_vel = dSqrt( accepted_energy / mass * 2.f ) * obj_material->fBounceDamageFactor;

    if ( contact_vel > m_flContactVelocity )
    {
        m_flContactVelocity = contact_vel;
        m_vecHitDir = cast_fv( norm );
        m_vecHitPos = cast_fv( pos );
        m_nObjectID = obj->ObjectID();
        m_pfnHitCallback = obj->ObjectGetCollisionHitCallback();
    }
}

void CHL2Movement::ObjectContactCallback(
    bool& do_colide, bool bo1, dContact& c, SGameMtl* material_1, SGameMtl* material_2 )
{
    if ( !do_colide )
        return;

    dBodyID body = bo1 ? dGeomGetBody( c.geom.g2 ) : dGeomGetBody( c.geom.g1 );

    if ( !body )
    {
        do_colide = false;
        return;
    }

    dxGeomUserData* ud = retrieveGeomUserData( bo1 ? c.geom.g2 : c.geom.g1 );

    if ( ud && ud->ph_object && ud->ph_object->CastType() == CPHObject::tpCharacter )
    {
        do_colide = false;
        return;
    }

    CHL2Movement* mov = ( CHL2Movement* )retrieveGeomUserData( bo1 ? c.geom.g1 : c.geom.g2 )->callback_data;

    mov->m_bContacted = true;

    dVector3 norm;
    if ( bo1 )
        dVectorSet( norm, c.geom.normal );
    else
        dVectorSetInvert( norm, c.geom.normal );

    mov->UpdateDynamicDamage( 
            retrieveRefObject( bo1 ? c.geom.g2 : c.geom.g1 ), 
            cast_fp( mov->m_vecVelocity ), 
            norm,
            c.geom.pos,
            bo1 ? material_2 : material_1,
            body );

    MulSprDmp( c.surface.soft_cfm, c.surface.soft_erp, def_spring_rate * 10.0f, def_dumping_rate );
    c.surface.mu = 0.0f;
}

void CHL2Movement::get_State( SPHNetState& state ) 
{
    state.linear_vel = m_vecVelocity;
    state.angular_vel.set( 0.0f, 0.0f, 0.0f );
    state.force.set( 0.0f, 0.0f, 0.0f );
    state.torque.set( 0.0f, 0.0f, 0.0f );
    state.position = m_vecOrigin;
	state.previous_position = m_vecOldOrigin;
	state.quaternion.identity();
	state.previous_quaternion.identity();
    state.enabled = m_bActivated;
}

void CHL2Movement::set_State( const SPHNetState& state ) 
{
    m_vecVelocity = state.linear_vel;
    m_vecOrigin = state.position;
    m_vecOldOrigin = state.previous_position;
}

void CHL2Movement::get_Box( Fvector& sz, Fvector& c ) const 
{
    const Fbox& box = m_pMovControl->Box();
    box.getsize( sz );
    box.getcenter( c );
    c.add( m_vecOrigin );
}

u16 CHL2Movement::DamageInitiatorID() const 
{
    if ( m_nObjectID != u16(-1) )
    {
        IPhysicsShellHolder* object = smart_cast<IPhysicsShellHolder*>( inl_ph_world().LevelObjects().net_Find( m_nObjectID ) );

        if ( object && !object->ObjectGetDestroy() )
        {
            IDamageSource* ds = object->ObjectCastIDamageSource();
            if ( ds )
                return ds->Initiator();
        }
    }

    return m_pPhysicsRef->ObjectID();
}

IGameObject* CHL2Movement::DamageInitiator() const 
{
    if ( m_nObjectID != u16(-1) )
    {
        IPhysicsShellHolder* object = smart_cast<IPhysicsShellHolder*>( inl_ph_world().LevelObjects().net_Find( m_nObjectID ) );

        if ( object && !object->ObjectGetDestroy() )
            object->IObject();
    }

    return m_pPhysicsRef->IObject();
}

void CHL2Movement::Reinit() 
{ 
    m_flContactVelocity = 0.0f; 
    m_nObjectID = u16(-1);
    m_pfnHitCallback = nullptr;
    m_bInitiated = false; 
}

bool CHL2Movement::GetAndResetInitiated() 
{ 
    bool ret = m_bInitiated; 
    m_bInitiated = false; 
    return ret; 
}
