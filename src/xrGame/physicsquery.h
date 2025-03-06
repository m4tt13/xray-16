#ifndef PHYSICSQUERY_H
#define PHYSICSQUERY_H

#include "ode/ode/src/collision_kernel.h"

#define PHQUERY_DIST_EPSILON	EPS_L
#define PHQUERY_INVALID_FRAC	-99999.9f
#define PHQUERY_NORMAL_UNDEF	flt_max

class CPHObject;

struct Ray_t
{
	Fvector  m_Start;	// starting point, centered within the extents
	Fvector  m_Delta;	// direction + length of the ray
	Fvector  m_StartOffset;	// Add this to m_Start to get the actual ray start
	Fvector  m_Extents;	// Describes an axis aligned box extruded along a ray
	bool	m_IsRay;	// are the extents zero?
	bool	m_IsSwept;	// is delta != 0?

	void Init( Fvector const& start, Fvector const& end )
	{
		m_Delta.sub( end, start );

		m_IsSwept = ( m_Delta.square_magnitude() != 0.0f );

		m_Extents.set( 0.0f, 0.0f, 0.0f );
		m_IsRay = true;

		// Offset m_Start to be in the center of the box...
		m_StartOffset.set( 0.0f, 0.0f, 0.0f );
		m_Start = start;
	}

	void Init( Fvector const& start, Fvector const& end, Fvector const& mins, Fvector const& maxs )
	{
		m_Delta.sub( end, start );

		m_IsSwept = ( m_Delta.square_magnitude() != 0.0f );

		m_Extents.sub( maxs, mins );
		m_Extents.mul( 0.5f );
		m_IsRay = ( m_Extents.square_magnitude() < 1e-6 );

		// Offset m_Start to be in the center of the box...
		m_StartOffset.add( mins, maxs );
		m_StartOffset.mul( 0.5f );
		m_Start.add( start, m_StartOffset );
		m_StartOffset.mul( -1.0f );
	}

	// compute inverse delta
	Fvector InvDelta() const
	{
		Fvector vecInvDelta;
		for ( int iAxis = 0; iAxis < 3; ++iAxis )
		{
			if ( m_Delta[iAxis] != 0.0f )
			{
				vecInvDelta[iAxis] = 1.0f / m_Delta[iAxis];
			}
			else
			{
				vecInvDelta[iAxis] = flt_max;
			}
		}
		return vecInvDelta;
	}
};

struct cplane_t
{
	Fvector	normal;
	float	dist;
	byte	type;		// for fast side tests
	byte	signbits;	// signx + (signy<<1) + (signz<<1)
};

struct geom_info_t
{
    dGeomID geometry;
    Fmatrix xform;
    Fvector mins;
    Fvector maxs;
};

struct trace_t
{
	Fvector			startpos;	// start position
	Fvector			endpos;		// final position
	cplane_t		plane;		// surface normal at impact

	float			fraction;	// time completed, 1.0 = didn't hit anything

	bool			startsolid;	// if true, the initial point was in a solid area

	u32				triangle;
	
    u16             material;
	CPHObject		*object;
    geom_info_t     geom_info;

	inline void Clear()
	{
		ZeroMemory( this, sizeof( *this ) );
		fraction = 1.0f;
		triangle = -1;
	}

	inline bool DidHit() const
	{
		return ( fraction < 1.0f ) || startsolid;
	}
};

struct enum_geom_t
{
    CPHObject *object;
    geom_info_t geom_info;
};

enum TraceType_t
{
	TRACE_EVERYTHING = 0,
	TRACE_WORLD_ONLY,
	TRACE_OBJECTS_ONLY,
};

class ITraceFilter
{
public:
    virtual bool ShouldHitTriangle( u32 nTriangle ) = 0;
	virtual bool ShouldHitObject( CPHObject *pObject ) = 0;
	virtual bool ShouldHitGeometry( CPHObject *pObject, dGeomID geometry ) = 0;
	virtual TraceType_t	GetTraceType() const = 0;
};

class CTraceFilter : public ITraceFilter
{
public:
	virtual TraceType_t	GetTraceType() const { return TRACE_EVERYTHING; }
};

class CTraceFilterWorldOnly : public ITraceFilter
{
public:
    virtual bool ShouldHitTriangle( u32 nTriangle ) { return true; }
	virtual bool ShouldHitObject( CPHObject *pObject ) { return false; }
    virtual bool ShouldHitGeometry( CPHObject *pObject, dGeomID geometry ) { return false; }
	virtual TraceType_t	GetTraceType() const { return TRACE_WORLD_ONLY; }
};

class CTraceFilterHitAll : public CTraceFilter
{
public:
    virtual bool ShouldHitTriangle( u32 nTriangle ) { return true; }
	virtual bool ShouldHitObject( CPHObject *pObject ) { return true; }
	virtual bool ShouldHitGeometry( CPHObject *pObject, dGeomID geometry ) { return true; }
};

struct TriCache_t
{
	cplane_t m_plane;

	Fvector m_mins;
	Fvector m_maxs;

	Fvector m_crossx[3];
	Fvector m_crossy[3];
	Fvector m_crossz[3];
};

struct TraceInfo_t
{
	Ray_t m_ray;

	Fvector m_end;
	Fvector m_invdelta;

	Fvector m_mins;
	Fvector m_maxs;

	Fvector m_absmins;
	Fvector m_absmaxs;

	ITraceFilter *m_filter;
	
	CPHObject *m_object;

	const Fvector *m_verts;
	const CDB::TRI *m_tris;

	trace_t m_trace;

	xr_vector<ISpatial*> m_enum_results;
    xr_vector<enum_geom_t> *m_enum_geoms;
};

class CPhyQueryHelper
{
public:
	float	m_flStartFrac;
	float	m_flEndFrac;
	Fvector	m_vecImpactNormal;
	float	m_flImpactDist;
};

class CPhysicsQuery
{
public:
	CPhysicsQuery();
	~CPhysicsQuery();

public:
	void Uncache();
	void TraceRay( const Ray_t &ray, ITraceFilter *pTraceFilter, trace_t *pTrace );
    void EnumerateGeometriesAlongRay( const Ray_t &ray, ITraceFilter *pTraceFilter, xr_vector<enum_geom_t>& enum_geoms );

private:
    void Cache();
	void Cache_Create( TriCache_t *pTri, const Fvector &v1, const Fvector &v2, const Fvector &v3 );
	template <int EDGE>	bool Cache_EdgeCrossAxisX( const Fvector &vecEdge, const Fvector &vecOnEdge, const Fvector &vecOffEdge, TriCache_t *pTri );
	template <int EDGE>	bool Cache_EdgeCrossAxisY( const Fvector &vecEdge, const Fvector &vecOnEdge, const Fvector &vecOffEdge, TriCache_t *pTri );
	template <int EDGE>	bool Cache_EdgeCrossAxisZ( const Fvector &vecEdge, const Fvector &vecOnEdge, const Fvector &vecOffEdge, TriCache_t *pTri );

	bool ResolveRayPlaneIntersect( float flStart, float flEnd, const Fvector &vecNormal, float flDist, CPhyQueryHelper *pHelper );

	inline bool FacePlane( TraceInfo_t *ti, TriCache_t *pTri, CPhyQueryHelper *pHelper );
	inline bool AxisPlanesXYZ( TraceInfo_t *ti, TriCache_t *pTri, CPhyQueryHelper *pHelper );

	template <int AXIS>	bool EdgeCrossAxis( TraceInfo_t *ti, const Fvector &vecNormal, CPhyQueryHelper *pHelper );
	inline bool EdgeCrossAxisX( TraceInfo_t *ti, const Fvector &vecNormal, CPhyQueryHelper *pHelper );
	inline bool EdgeCrossAxisY( TraceInfo_t *ti, const Fvector &vecNormal, CPhyQueryHelper *pHelper );
	inline bool EdgeCrossAxisZ( TraceInfo_t *ti, const Fvector &vecNormal, CPhyQueryHelper *pHelper );

	inline void CalcClosestExtents( const Fvector &vecPlaneNormal, const Fvector &vecBoxExtents, Fvector &vecBoxPoint );

	bool SweepAABBTriIntersect( TraceInfo_t *ti, u32 nTriangle );
	bool RayTriIntersect( TraceInfo_t *ti, u32 nTriangle );
	bool AABBTriIntersect( TraceInfo_t *ti, u32 nTriangle );

	template <bool IS_POINT> bool ClipRayToWorld_R( TraceInfo_t *ti, const Opcode::AABBNoLeafNode *node );
	bool IsBoxIntersectingWorld_R( TraceInfo_t *ti, const Opcode::AABBNoLeafNode *node );

	template <bool IS_POINT> void EnumerateObjectsAlongRay_R( TraceInfo_t *ti, ISpatial_NODE *node, const Fvector &vecCenter, float flRadius );
	template <bool IS_POINT> void EnumerateObjectsInBox_R( TraceInfo_t *ti, ISpatial_NODE *node, const Fvector &vecCenter, float flRadius );

    void ComputeFinalTx( dGeomID geom, bool bIsTransform, const dReal *pos, const dReal *rot, dReal *final_pos, dReal *final_rot );

    bool ClipRayToGeometry( TraceInfo_t *ti, geom_info_t &geomInfo, trace_t &geomTrace );
	bool ClipRayToGeometry_R( TraceInfo_t *ti, dGeomID geom, bool bIsTransform = false, const dReal *pos = nullptr, const dReal *rot = nullptr );

    void EnumerateGeometriesAlongRay_R( TraceInfo_t *ti, dGeomID geom, bool bIsTransform = false, const dReal *pos = nullptr, const dReal *rot = nullptr );

private:
	TriCache_t *m_pTriCache;
};

inline void CPhysicsQuery::CalcClosestExtents( const Fvector &vecPlaneNormal, const Fvector &vecBoxExtents,
	Fvector &vecBoxPoint )
{
	( vecPlaneNormal[0] < 0.0f ) ? vecBoxPoint[0] = vecBoxExtents[0] : vecBoxPoint[0] = -vecBoxExtents[0];
	( vecPlaneNormal[1] < 0.0f ) ? vecBoxPoint[1] = vecBoxExtents[1] : vecBoxPoint[1] = -vecBoxExtents[1];
	( vecPlaneNormal[2] < 0.0f ) ? vecBoxPoint[2] = vecBoxExtents[2] : vecBoxPoint[2] = -vecBoxExtents[2];
}

CPhysicsQuery &PhysicsQuery();

#endif // PHYSICSQUERY_H
