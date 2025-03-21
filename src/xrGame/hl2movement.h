#ifndef HL2MOVEMENT_H
#define HL2MOVEMENT_H

#define TIME_TO_TICKS( dt )		( (int)( 0.5f + (float)(dt) / CHL2Movement::m_flFrameTime ) )
#define TICKS_TO_TIME( t )		( CHL2Movement::m_flFrameTime *( t ) )

enum EDucking
{
    DUCKING_NONE,
    DUCKING_NORMAL,
    DUCKING_ACCEL
};

class CTraceFilterActorMovement : public CTraceFilter
{
public:
	CTraceFilterActorMovement( CPHObject* pActor, bool bWorldOnly = false, bool bSkipActorObstacle = false, xr_vector<enum_geom_t> *pPassGeoms = nullptr ) : 
        m_pActor( pActor ),
        m_bWorldOnly( bWorldOnly ),
        m_bSkipActorObstacle( bSkipActorObstacle ),
        m_pPassGeoms( pPassGeoms )
    {}

    virtual bool ShouldHitTriangle( u32 nTriangle )
	{
        CDB::MODEL *model = Level().ObjectSpace.GetStaticModel();
        CDB::TRI *tris = model->get_tris();

		Flags32	&flags = GMLib.GetMaterialByIdx( tris[nTriangle].material )->Flags;

        return ( !m_bSkipActorObstacle && flags.test( SGameMtl::flActorObstacle ) ) || !flags.test( SGameMtl::flPassable );
	}
	
	virtual bool ShouldHitObject( CPHObject *pObject )
	{
		return ( pObject->GetSpatialData().type & STYPE_PHYSIC ) &&
               ( !m_pActor || ( pObject != m_pActor && do_obj_collide( *pObject, *m_pActor ) ) );
	}
	
	virtual bool ShouldHitGeometry( CPHObject *pObject, dGeomID geometry )
	{
        if ( m_pPassGeoms )
        {
		    xr_vector<enum_geom_t>::iterator i = m_pPassGeoms->begin();
	        xr_vector<enum_geom_t>::iterator e = m_pPassGeoms->end();

            for ( ; i != e; i++ )
	        {
                enum_geom_t &enum_geom = *i;
                geom_info_t &geom_info = enum_geom.geom_info;

                if ( geom_info.geometry == geometry )
                    return false;
            }
        }

        if ( pObject->CastType() == CPHObject::tpCharacter )
        {
            CPHCharacter* ch = static_cast<CPHCharacter*>( pObject );

            if ( ch->dCap() == geometry )
                return false;
        }

        return true;
	}

    virtual TraceType_t	GetTraceType() const
	{
		return m_bWorldOnly ? TRACE_WORLD_ONLY : TRACE_EVERYTHING;
	}
	
private:
    CPHObject* m_pActor;
    bool m_bWorldOnly;
    bool m_bSkipActorObstacle;
    xr_vector<enum_geom_t> *m_pPassGeoms;
};

class CHL2Movement : 
    public CPHSynchronize, 
    public IPhysicsElement, 
    public ICollisionDamageInfo
{
public:
    CHL2Movement( CPHMovementControl* pMovementControl );
    ~CHL2Movement( void );

    void UpdateCL( void );

    u64 GetTickCount( void ) const { return m_nTickCount; }

    bool IsActivated( void ) const { return m_bActivated; }
    void Activate( const Fvector& pos );
    void Deactivate( void );

    void SetMass( float mass );
    void SetMaterial( u16 material );
    void SetButtons( u32 buttons ) { m_nButtons = buttons; }
    void SetDirection( const Fvector& dir );

    bool GetAndResetJumped( void );
    bool GetAndResetContacted( void );

    EEnvironment CheckInvironment( void ) const;
    void GetGroundNormal( Fvector& norm ) const;

    const u16& GetLastMaterial( void ) const { return *m_pLastMaterial; }
    void SetLastMaterialPtr( u16* p ) { m_pLastMaterial = p; }
    u16 GetInjuriousMaterial( void ) const { return m_nInjuriousMaterial; }

    float GetCameraHeight( void ) const { return m_flCameraHeight; }

    void ApplyImpulse( const Fvector& dir, float force );
    void ApplyForce( const Fvector& force ) { m_vecExternalImpusle.add( force ); }
    void SetControlVelocity( const Fvector& vel ) { m_vecControlVelocity.set( vel.x, 0.0f, vel.z ); }

    void SetVelocityModifier( float modifier ) { m_flVelocityModifier = modifier; }
 
    void GetPosition( Fvector& pos ) const { pos = m_vecOrigin; }
    void SetPosition( const Fvector& pos );

    void GetIPosition( Fvector& pos ) const;

    void GetVelocity( Fvector& vel ) const { vel = m_vecVelocity; }
    void SetVelocity( const Fvector& vel );

    void GetSmoothedVelocity( Fvector& vel ) const { vel = m_vecSmoothedVelocity; }

    void GetPunchAngle( Fvector& ang ) const { ang = m_vecPunchAngle; }

    IPhysicsShellHolder* GetPhysicsRefObject( void ) const { return m_pPhysicsRef; }
    void SetPhysicsRefObject( IPhysicsShellHolder* obj );

    void CollisionEnable( bool bEnable );

    void UpdateStaticDamage( const dReal* vel, const dReal* norm, const dReal* pos, SGameMtl* tri_material );
    void UpdateDynamicDamage( IPhysicsShellHolder* obj, const dReal* vel, const dReal* norm, const dReal* pos, SGameMtl* obj_material, dBodyID body );

public:
    // CPHSynchronize
    virtual void get_State( SPHNetState& state );
    virtual void set_State( const SPHNetState& state );
    virtual void cv2obj_Xfrom( const Fquaternion& q, const Fvector& pos, Fmatrix& xform ) {}
    virtual void cv2bone_Xfrom( const Fquaternion& q, const Fvector& pos, Fmatrix& xform ) {}

public:
    // IPhysicsElement
    virtual const Fmatrix& XFORM() const { return m_pPhysicsRef->ObjectXFORM(); }
    virtual void get_LinearVel( Fvector& velocity ) const { velocity = m_vecVelocity; }
    virtual void get_AngularVel( Fvector& velocity ) const { velocity.set( 0.0f, 0.0f, 0.0f ); }
    virtual void get_Box( Fvector& sz, Fvector& c ) const;
    virtual const Fvector& mass_Center() const { return m_vecVelocity; }
    virtual u16 numberOfGeoms() const { return 0; }
    virtual const IPhysicsGeometry* geometry( u16 i ) const { return nullptr; }

public:
    // ICollisionDamageInfo
    virtual float ContactVelocity() const { return m_flContactVelocity; }
    virtual void HitDir( Fvector& dir ) const { dir = m_vecHitDir; }
    virtual const Fvector& HitPos() const { return m_vecHitPos; }
    virtual u16 DamageInitiatorID() const;
    virtual IGameObject* DamageInitiator() const;
    virtual ALife::EHitType HitType() const { return m_eHitType; }
    virtual void SetHitType( ALife::EHitType type ) { m_eHitType = type; };
    virtual ICollisionHitCallback* HitCallback() const { return m_pfnHitCallback; }
    virtual void Reinit();
    virtual void SetInitiated() { m_bInitiated = true; }
    virtual bool IsInitiated() const { return m_bInitiated; }
    virtual bool GetAndResetInitiated();

private:
    void Clear( void );
    void StartMove( void );
    void FinishMove( void );
    void AvoidPushawayGeoms( void );
    int CheckStuck( void );
    void FixPlayerCrouchStuck( bool bAccel );
    bool CanUnduck( bool bAccel );
    void FinishUnDuck( bool bAccel );
    void FinishDuck( bool bAccel );
    void SetDuckedEyeOffset( float duckFraction, bool bAccel );
    void Duck( void );
    void DecayPunchAngle( void );
    void ReduceTimers( void );
    void Move( void );
    void TryPlayerMove( Fvector *pFirstDest=nullptr, trace_t *pFirstTrace=nullptr );
    void ClipVelocity( Fvector& in, Fvector& normal, Fvector& out, float overbounce );
    bool LadderMove( void );
    void WalkMove( void );
    void Accelerate( Fvector& wishdir, float wishspeed, float accel );
    void StepMove( Fvector &vecDestination, trace_t &trace );
    void StayOnGround( void );
    void Friction( void );
    void AirMove( void );
    void AirAccelerate( Fvector& wishdir, float wishspeed, float accel );
    void StartGravity( void );
    void FinishGravity( void );
    void CheckVelocity( void );
    void CheckJumpButton( void );
    void CategorizePosition( void );
    void CategorizeGroundSurface( trace_t &pm );
    void CheckFalling( void );
    void PlayerRoughLandingEffects( void );
    void TestPlayerPosition( const Fvector& pos, trace_t& pm, bool bWorldOnly = false );
    void TracePlayerBBox( const Fvector& start, const Fvector& end, trace_t& pm, bool bSkipActorObstacle = false );
    void TryTouchGround( const Fvector& start, const Fvector& end, const Fvector& mins, const Fvector& maxs, trace_t& pm );
    void TryTouchGroundInQuadrants( const Fvector& start, const Fvector& end, trace_t& pm );

protected:
    static void ObjectContactCallback(
        bool& do_colide, bool bo1, dContact& c, SGameMtl* material_1, SGameMtl* material_2);

public:
    static float m_flFrameTime;
    static float m_flForwardSpeed;
    static float m_flSideSpeed;
	static float m_flWalkSpeed;
	static float m_flSlowWalkSpeed;
	static float m_flSprintSpeed;
    static float m_flMaxControlSpeed;
	static float m_flJumpPower;
	static float m_flGravity;
	static float m_flMaxVelocity;
	static float m_flNonJumpVelocity;
	static float m_flBounce;
	static float m_flStepSize;
	static float m_flFriction;
	static float m_flStopSpeed;
    static float m_flStandableNormal;
	static float m_flWalkableNormal;
	static float m_flAirMaxWishSpeed;
	static float m_flAccelerate;
	static float m_flAirAccelerate;
    static float m_flLadderDistance;
    static float m_flLadderLeaveSpeed;
    static float m_flClimbSpeed;
    static float m_flDuckSpeed;
    static float m_flUnDuckSpeed;
    static float m_flPushawayForce;
    static float m_flPushawayMaxForce;
    static BOOL m_bSpeedometer;
	static BOOL m_bStickToGround;
	static BOOL m_bEnableBHop;
    static int m_nABHMode;
    
private:
    CPHMovementControl* m_pMovControl;
    IPhysicsShellHolder* m_pPhysicsRef;
    CPHActorGeomShell* m_pPhysicsShell;

    u16 m_nLastMaterial;
    u16* m_pLastMaterial;
    u16 m_nInjuriousMaterial;

	u32 m_nButtons;
	u32 m_nOldButtons;
    u32 m_nOldBoxID;

    u64 m_nTickCount;
    u64 m_nExternalImpulseEndTick;

    EDucking m_eDucking;

	float m_surfaceFriction;
	float m_flForwardMove;
	float m_flSideMove;
	float m_flMaxSpeed;
    float m_flDucktime;
    float m_flVelocityModifier;
    float m_flFallVelocity;
    float m_flCameraHeight;

	double m_dblRemainder;
	
    bool m_bActivated;
    bool m_bOnLadder;
	bool m_bOnGround;
    bool m_bCollisionDisabled;
    bool m_bJumped;
    bool m_bContacted;
    bool m_bIsSprinting;

	Fvector m_vecForward;
	Fvector m_vecRight;
	Fvector m_vecOrigin;
    Fvector m_vecOldOrigin;
	Fvector m_vecVelocity;
    Fvector m_vecSmoothedVelocity;
    Fvector m_vecControlVelocity;
    Fvector m_vecGroundNormal;
    Fvector m_vecExternalImpusle;
    Fvector m_vecLadderNormal;
    Fvector m_vecPunchAngle;
    Fvector m_vecPunchAngleVel;

    xr_vector<enum_geom_t> m_PassGeoms;

private:
    float m_flContactVelocity;
    Fvector m_vecHitDir;
    Fvector m_vecHitPos;
    u16 m_nObjectID;
    ALife::EHitType m_eHitType;
    ICollisionHitCallback* m_pfnHitCallback;
    bool m_bInitiated;
};

#endif // HL2MOVEMENT_H
