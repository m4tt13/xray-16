#include "stdafx.h"
#include "Level.h"
#include "xrMaterialSystem/GameMtlLib.h"
#include "xrPhysics/ExtendedGeom.h"
#include "xrPhysics/PHDynamicData.h"
#include "xrPhysics/PHObject.h"
#include "xrPhysics/dCylinder/dCylinder.h"
#include "OPCODE/Opcode.h"
#include "physicsquery.h"
#include "collisionutils.h"

static const Fvector g_SpatialOffsets[8] =
{
	{ -1, -1, -1 },
	{ 1, -1, -1 },
	{ -1, 1, -1 },
	{ 1, 1, -1 },
	{ -1, -1, 1 },
	{ 1, -1, 1 },
	{ -1, 1, 1 },
	{ 1, 1, 1 }
};

static CPhysicsQuery s_PhysicsQuery;
CPhysicsQuery &PhysicsQuery()
{
	return s_PhysicsQuery;
}

CPhysicsQuery::CPhysicsQuery() : 
    m_pTriCache( nullptr )
{
}

CPhysicsQuery::~CPhysicsQuery()
{
	xr_free( m_pTriCache );
}

void CPhysicsQuery::Uncache()
{
    xr_free( m_pTriCache );
}

void CPhysicsQuery::Cache()
{
	if ( m_pTriCache )
		return;

	CDB::MODEL *model = Level().ObjectSpace.GetStaticModel();

	int tris_count = model->get_tris_count();
	m_pTriCache = xr_alloc<TriCache_t>( tris_count );

	const Fvector *verts = model->get_verts();
	const CDB::TRI *tris = model->get_tris();

	for ( int i = 0; i < tris_count; i++ )
	{
		Cache_Create( &m_pTriCache[i], 
			verts[tris[i].verts[0]],
			verts[tris[i].verts[1]], 
			verts[tris[i].verts[2]] );
	}
}

void CPhysicsQuery::Cache_Create( TriCache_t *pTri, const Fvector &v1, const Fvector &v2, const Fvector &v3 )
{
	// Calculate the plane normal.
	Fvector vecEdges[2];
	vecEdges[0].sub( v2, v1 );
	vecEdges[1].sub( v3, v1 );
	
	Fvector vecNormal;
	vecNormal.crossproduct( vecEdges[0], vecEdges[1] );
	vecNormal.normalize2();

	pTri->m_plane.normal = vecNormal;
	pTri->m_plane.dist = pTri->m_plane.normal.dotproduct( v1 );

	// Calculate the signbits for the plane - fast test.
	pTri->m_plane.signbits = 0;
	pTri->m_plane.type = 5;
	for ( int iAxis = 0; iAxis < 3; ++iAxis )
	{
		if ( pTri->m_plane.normal[iAxis] < 0.0f )
		{
			pTri->m_plane.signbits |= 1 << iAxis;
		}

		if ( pTri->m_plane.normal[iAxis] == 1.0f )
		{
			pTri->m_plane.type = iAxis;
		}
	}

	// Calculate the min and max.
	pTri->m_mins.min( v2, v3 );
	pTri->m_mins.min( v1 );

	pTri->m_maxs.max( v2, v3 );
	pTri->m_maxs.max( v1 );

	// Calculate the edge planes.
	Fvector vecEdge;

	// Edge 1
	vecEdge.sub( v2, v1 );
	Cache_EdgeCrossAxisX<0>( vecEdge, v1, v3, pTri );
	Cache_EdgeCrossAxisY<0>( vecEdge, v1, v3, pTri );
	Cache_EdgeCrossAxisZ<0>( vecEdge, v1, v3, pTri );

	// Edge 2
	vecEdge.sub( v3, v2 );
	Cache_EdgeCrossAxisX<1>( vecEdge, v2, v1, pTri );
	Cache_EdgeCrossAxisY<1>( vecEdge, v2, v1, pTri );
	Cache_EdgeCrossAxisZ<1>( vecEdge, v2, v1, pTri );

	// Edge 3
	vecEdge.sub( v1, v3 );
	Cache_EdgeCrossAxisX<2>( vecEdge, v3, v2, pTri );
	Cache_EdgeCrossAxisY<2>( vecEdge, v3, v2, pTri );
	Cache_EdgeCrossAxisZ<2>( vecEdge, v3, v2, pTri );
}

template <int EDGE>
bool CPhysicsQuery::Cache_EdgeCrossAxisX( const Fvector &vecEdge, const Fvector &vecOnEdge, const Fvector &vecOffEdge, TriCache_t *pTri )
{
	// Calculate the normal - edge x axisX = ( 0.0, edgeZ, -edgeY )
	Fvector vecNormal = { 0.0f, vecEdge.z, -vecEdge.y };
	vecNormal.normalize2();

	// Check for zero length normals.
	if ( ( vecNormal.y == 0.0f ) || ( vecNormal.z == 0.0f ) )
	{
		pTri->m_crossx[EDGE].x = PHQUERY_NORMAL_UNDEF;
		return false;
	}

	// Finish the plane definition - get distance.
	float flDist = ( vecNormal.y * vecOnEdge.y ) + ( vecNormal.z * vecOnEdge.z );

	// Special case the point off edge in plane
	float flOffDist = ( vecNormal.y * vecOffEdge.y ) + ( vecNormal.z * vecOffEdge.z );
	if ( !( _abs( flOffDist - flDist ) < PHQUERY_DIST_EPSILON ) && ( flOffDist > flDist ) )
	{
		// Adjust plane facing - triangle should be behind the plane.
		vecNormal.x = -flDist;
		vecNormal.y = -vecNormal.y;
		vecNormal.z = -vecNormal.z;
	}
	else
	{
		vecNormal.x = flDist;
	}

	pTri->m_crossx[EDGE] = vecNormal;

	// Created the cached edge.
	return true;
}

template <int EDGE>
bool CPhysicsQuery::Cache_EdgeCrossAxisY( const Fvector &vecEdge, const Fvector &vecOnEdge, const Fvector &vecOffEdge, TriCache_t *pTri )
{
	// Calculate the normal - edge x axisY = ( -edgeZ, 0.0, edgeX )
	Fvector vecNormal = { -vecEdge.z, 0.0f, vecEdge.x };
	vecNormal.normalize2();

	// Check for zero length normals
	if ( ( vecNormal.x == 0.0f ) || ( vecNormal.z == 0.0f ) )
	{
		pTri->m_crossy[EDGE].y = PHQUERY_NORMAL_UNDEF;
		return false;
	}

	// Finish the plane definition - get distance.
	float flDist = ( vecNormal.x * vecOnEdge.x ) + ( vecNormal.z * vecOnEdge.z );

	// Special case the point off edge in plane
	float flOffDist = ( vecNormal.x * vecOffEdge.x ) + ( vecNormal.z * vecOffEdge.z );
	if ( !( _abs( flOffDist - flDist ) < PHQUERY_DIST_EPSILON ) && ( flOffDist > flDist ) )
	{
		// Adjust plane facing if necessay - triangle should be behind the plane.
		vecNormal.x = -vecNormal.x;
		vecNormal.y = -flDist;
		vecNormal.z = -vecNormal.z;
	}
	else
	{
		vecNormal.y = flDist;
	}

	pTri->m_crossy[EDGE] = vecNormal;

	// Created the cached edge.
	return true;
}

template <int EDGE>
bool CPhysicsQuery::Cache_EdgeCrossAxisZ( const Fvector &vecEdge, const Fvector &vecOnEdge, const Fvector &vecOffEdge, TriCache_t *pTri )
{
	Fvector vecNormal = { vecEdge.y, -vecEdge.x, 0.0f };
	vecNormal.normalize2();

	// Check for zero length normals
	if ( ( vecNormal.x == 0.0f ) || ( vecNormal.y == 0.0f ) )
	{
		pTri->m_crossz[EDGE].z = PHQUERY_NORMAL_UNDEF;
		return false;
	}

	// Finish the plane definition - get distance.
	float flDist = ( vecNormal.x * vecOnEdge.x ) + ( vecNormal.y * vecOnEdge.y );

	// Special case the point off edge in plane
	float flOffDist = ( vecNormal.x * vecOffEdge.x ) + ( vecNormal.y * vecOffEdge.y );
	if ( !( _abs( flOffDist - flDist ) < PHQUERY_DIST_EPSILON ) && ( flOffDist > flDist ) )
	{
		// Adjust plane facing if necessay - triangle should be behind the plane.
		vecNormal.x = -vecNormal.x;
		vecNormal.y = -vecNormal.y;
		vecNormal.z = -flDist;
	}
	else
	{
		vecNormal.z = flDist;
	}

	pTri->m_crossz[EDGE] = vecNormal;

	// Created the cached edge.
	return true;
}

bool CPhysicsQuery::ResolveRayPlaneIntersect( float flStart, float flEnd, const Fvector &vecNormal, float flDist, CPhyQueryHelper *pHelper )
{
	if ( ( flStart > 0.0f ) && ( flEnd > 0.0f ) )
		return false;

	if ( ( flStart < 0.0f ) && ( flEnd < 0.0f ) )
		return true;

	float flDenom = flStart - flEnd;
	bool bDenomIsZero = ( flDenom == 0.0f );
	if ( ( flStart >= 0.0f ) && ( flEnd <= 0.0f ) )
	{
		// Find t - the parametric distance along the trace line.
		float t = ( !bDenomIsZero ) ? ( flStart - PHQUERY_DIST_EPSILON ) / flDenom : 0.0f;
		if ( t > pHelper->m_flStartFrac )
		{
			pHelper->m_flStartFrac = t;
			pHelper->m_vecImpactNormal = vecNormal;
			pHelper->m_flImpactDist = flDist;
		}
	}
	else
	{
		// Find t - the parametric distance along the trace line.
		float t = ( !bDenomIsZero ) ? ( flStart + PHQUERY_DIST_EPSILON ) / flDenom : 0.0f;
		if ( t < pHelper->m_flEndFrac )
		{
			pHelper->m_flEndFrac = t;
		}
	}

	return true;
}

inline bool CPhysicsQuery::FacePlane( TraceInfo_t *ti, TriCache_t *pTri, CPhyQueryHelper *pHelper )
{
	// Calculate the closest point on box to plane (get extents in that direction).
	Fvector vecExtent;
	CalcClosestExtents( pTri->m_plane.normal, ti->m_ray.m_Extents, vecExtent );

	float flExpandDist = pTri->m_plane.dist - pTri->m_plane.normal.dotproduct( vecExtent );

	float flStart = pTri->m_plane.normal.dotproduct( ti->m_ray.m_Start ) - flExpandDist;
	float flEnd = pTri->m_plane.normal.dotproduct( ti->m_end ) - flExpandDist;

	return ResolveRayPlaneIntersect( flStart, flEnd, pTri->m_plane.normal, pTri->m_plane.dist, pHelper );
}

inline bool CPhysicsQuery::AxisPlanesXYZ( TraceInfo_t *ti, TriCache_t *pTri, CPhyQueryHelper *pHelper )
{
	static const Fvector g_ImpactNormalVecs[2][3] =
	{
		{
			{ -1, 0, 0 },
			{ 0, -1, 0 },
			{ 0, 0, -1 },
		},

		{
			{ 1, 0, 0 },
			{ 0, 1, 0 },
			{ 0, 0, 1 },
		}
	};

	float flDist, flExpDist, flStart, flEnd;

	int iAxis;
	for ( iAxis = 2; iAxis >= 0; --iAxis )
	{
		const float rayStart = ti->m_ray.m_Start[iAxis];
		const float rayExtent = ti->m_ray.m_Extents[iAxis];
		const float rayDelta = ti->m_ray.m_Delta[iAxis];

		// Min
		flDist = pTri->m_mins[iAxis];
		flExpDist = flDist - rayExtent;
		flStart = flExpDist - rayStart;
		flEnd = flStart - rayDelta;

		if ( !ResolveRayPlaneIntersect( flStart, flEnd, g_ImpactNormalVecs[0][iAxis], flDist, pHelper ) )
			return false;

		// Max
		flDist = pTri->m_maxs[iAxis];
		flExpDist = flDist + rayExtent;
		flStart = rayStart - flExpDist;
		flEnd = flStart + rayDelta;

		if ( !ResolveRayPlaneIntersect( flStart, flEnd, g_ImpactNormalVecs[1][iAxis], flDist, pHelper ) )
			return false;
	}

	return true;
}

template <int AXIS>
bool CPhysicsQuery::EdgeCrossAxis( TraceInfo_t *ti, const Fvector &vecPlaneNormal, CPhyQueryHelper *pHelper )
{
	Fvector vecNormal = vecPlaneNormal;

	if ( vecNormal[AXIS] == PHQUERY_NORMAL_UNDEF )
		return true;

	const int OTHER_AXIS1 = ( AXIS + 1 ) % 3;
	const int OTHER_AXIS2 = ( AXIS + 2 ) % 3;

	// Get the pland distance are "fix" the normal.
	float flDist = vecNormal[AXIS];
	vecNormal[AXIS] = 0.0f;

	// Calculate the closest point on box to plane (get extents in that direction).
	Fvector vecExtent;
	//vecExtent[AXIS] = 0.0f;
	vecExtent[OTHER_AXIS1] = ( vecNormal[OTHER_AXIS1] < 0.0f ) ? ti->m_ray.m_Extents[OTHER_AXIS1] : -ti->m_ray.m_Extents[OTHER_AXIS1];
	vecExtent[OTHER_AXIS2] = ( vecNormal[OTHER_AXIS2] < 0.0f ) ? ti->m_ray.m_Extents[OTHER_AXIS2] : -ti->m_ray.m_Extents[OTHER_AXIS2];

	// Expand the plane by the extents of the box to reduce the swept box/triangle
	// test to a ray/extruded triangle test (one of the triangles extruded planes
	// was just calculated above).
	Fvector vecEnd;
	vecEnd[AXIS] = 0;
	vecEnd[OTHER_AXIS1] = ti->m_ray.m_Start[OTHER_AXIS1] + ti->m_ray.m_Delta[OTHER_AXIS1];
	vecEnd[OTHER_AXIS2] = ti->m_ray.m_Start[OTHER_AXIS2] + ti->m_ray.m_Delta[OTHER_AXIS2];

	float flExpandDist = flDist - ( ( vecNormal[OTHER_AXIS1] * vecExtent[OTHER_AXIS1] ) + ( vecNormal[OTHER_AXIS2] * vecExtent[OTHER_AXIS2] ) );
	float flStart = ( vecNormal[OTHER_AXIS1] * ti->m_ray.m_Start[OTHER_AXIS1] ) + ( vecNormal[OTHER_AXIS2] * ti->m_ray.m_Start[OTHER_AXIS2] ) - flExpandDist;
	float flEnd = ( vecNormal[OTHER_AXIS1] * vecEnd[OTHER_AXIS1] ) + ( vecNormal[OTHER_AXIS2] * vecEnd[OTHER_AXIS2] ) - flExpandDist;

	return ResolveRayPlaneIntersect( flStart, flEnd, vecNormal, flDist, pHelper );
}

inline bool CPhysicsQuery::EdgeCrossAxisX( TraceInfo_t *ti, const Fvector &vecNormal, CPhyQueryHelper *pHelper )
{
	return EdgeCrossAxis<0>( ti, vecNormal, pHelper );
}

inline bool CPhysicsQuery::EdgeCrossAxisY( TraceInfo_t *ti, const Fvector &vecNormal, CPhyQueryHelper *pHelper )
{
	return EdgeCrossAxis<1>( ti, vecNormal, pHelper );
}

inline bool CPhysicsQuery::EdgeCrossAxisZ( TraceInfo_t *ti, const Fvector &vecNormal, CPhyQueryHelper *pHelper )
{
	return EdgeCrossAxis<2>( ti, vecNormal, pHelper );
}

bool CPhysicsQuery::SweepAABBTriIntersect( TraceInfo_t *ti, u32 nTriangle )
{
	TriCache_t *pTri = &m_pTriCache[nTriangle];
	
	// Init test data.
	CPhyQueryHelper helper;
	helper.m_flEndFrac = 1.0f;
	helper.m_flStartFrac = PHQUERY_INVALID_FRAC;

	// Make sure objects are traveling toward one another.
	float flDistAlongNormal = pTri->m_plane.normal.dotproduct( ti->m_ray.m_Delta );
	if ( flDistAlongNormal > PHQUERY_DIST_EPSILON )
		return false;

	// Test against the axis planes.
	if ( !AxisPlanesXYZ( ti, pTri, &helper ) )
		return false;

	//
	// There are 9 edge tests - edges 1, 2, 3 cross with the box edges (symmetry) 1, 2, 3.  However, the box
	// is axis-aligned resulting in axially directional edges -- thus each test is edges 1, 2, and 3 vs. 
	// axial planes x, y, and z
	//
	// There are potentially 9 more tests with edges, the edge's edges and the direction of motion!
	// NOTE: I don't think these tests are necessary for a manifold surface.
	//

	// Edges 1-3, interleaved - axis tests are 2d tests
	if ( !EdgeCrossAxisX( ti, pTri->m_crossx[0], &helper ) ) { return false; }
	if ( !EdgeCrossAxisX( ti, pTri->m_crossx[1], &helper ) ) { return false; }
	if ( !EdgeCrossAxisX( ti, pTri->m_crossx[2], &helper ) ) { return false; }

	if ( !EdgeCrossAxisY( ti, pTri->m_crossy[0], &helper ) ) { return false; }
	if ( !EdgeCrossAxisY( ti, pTri->m_crossy[1], &helper ) ) { return false; }
	if ( !EdgeCrossAxisY( ti, pTri->m_crossy[2], &helper ) ) { return false; }

	if ( !EdgeCrossAxisZ( ti, pTri->m_crossz[0], &helper ) ) { return false; }
	if ( !EdgeCrossAxisZ( ti, pTri->m_crossz[1], &helper ) ) { return false; }
	if ( !EdgeCrossAxisZ( ti, pTri->m_crossz[2], &helper ) ) { return false; }

	// Test against the triangle face plane.
	if ( !FacePlane( ti, pTri, &helper ) )
		return false;

	if ( ( helper.m_flStartFrac < helper.m_flEndFrac ) || ( _abs( helper.m_flStartFrac - helper.m_flEndFrac ) < 0.001f ) )
	{
		if ( ( helper.m_flStartFrac != PHQUERY_INVALID_FRAC ) && ( helper.m_flStartFrac < ti->m_trace.fraction ) )
		{
			// Clamp -- shouldn't really ever be here!???
			if ( helper.m_flStartFrac < 0.0f )
			{
				helper.m_flStartFrac = 0.0f;
			}

			ti->m_trace.fraction = helper.m_flStartFrac;
			ti->m_trace.plane.normal = helper.m_vecImpactNormal;
			ti->m_trace.plane.dist = helper.m_flImpactDist;
			ti->m_trace.triangle = nTriangle;
            ti->m_trace.material = ti->m_tris[nTriangle].material;
			return true;
		}
	}

	return false;
}

bool CPhysicsQuery::RayTriIntersect( TraceInfo_t *ti, u32 nTriangle )
{
	TriCache_t *pTri = &m_pTriCache[nTriangle];
	
	float flFrac = IntersectRayWithTriangle( ti->m_ray.m_Start, ti->m_ray.m_Delta, 
		ti->m_verts[ti->m_tris[nTriangle].verts[0]], 
		ti->m_verts[ti->m_tris[nTriangle].verts[1]], 
		ti->m_verts[ti->m_tris[nTriangle].verts[2]], true );

	if ( ( flFrac >= 0.0f ) && ( flFrac < ti->m_trace.fraction ) )
	{
		ti->m_trace.fraction = flFrac;
		ti->m_trace.plane.normal = pTri->m_plane.normal;
		ti->m_trace.plane.dist = pTri->m_plane.dist;
		ti->m_trace.triangle = nTriangle;
		ti->m_trace.material = ti->m_tris[nTriangle].material;
		return true;
	}

	return false;
}

bool CPhysicsQuery::AABBTriIntersect( TraceInfo_t *ti, u32 nTriangle )
{
	TriCache_t *pTri = &m_pTriCache[nTriangle];
	
	if ( IsBoxIntersectingTriangle( ti->m_ray.m_Start, ti->m_ray.m_Extents,
		ti->m_verts[ti->m_tris[nTriangle].verts[0]],
		ti->m_verts[ti->m_tris[nTriangle].verts[1]],
		ti->m_verts[ti->m_tris[nTriangle].verts[2]],
		pTri->m_plane, 0.0f ) )
	{
		ti->m_trace.fraction = 0.0f;
		ti->m_trace.startsolid = true;
		ti->m_trace.triangle = nTriangle;
		ti->m_trace.material = ti->m_tris[nTriangle].material;
		return true;
	}

	return false;
}

template <bool IS_POINT>
bool CPhysicsQuery::ClipRayToWorld_R( TraceInfo_t *ti, const Opcode::AABBNoLeafNode *node )
{
	Fvector vMins = { node->mAABB.GetMin( 0 ), node->mAABB.GetMin( 1 ), node->mAABB.GetMin( 2 ) };
	Fvector vMaxs = { node->mAABB.GetMax( 0 ), node->mAABB.GetMax( 1 ), node->mAABB.GetMax( 2 ) };

	if ( !IS_POINT )
	{
		vMins.sub( ti->m_ray.m_Extents );
		vMaxs.add( ti->m_ray.m_Extents );
	}
	
	if ( IsBoxIntersectingRay( vMins, vMaxs, ti->m_ray.m_Start, ti->m_ray.m_Delta, ti->m_invdelta, PHQUERY_DIST_EPSILON ) )
	{
		if ( node->HasLeaf() )
		{
			u32 nTriangle = node->GetPrimitive();

			if ( ti->m_filter->ShouldHitTriangle( nTriangle ) )
			{
				if ( IS_POINT )
					RayTriIntersect( ti, nTriangle );
				else
					SweepAABBTriIntersect( ti, nTriangle );

				if ( ti->m_trace.fraction == 0.0f )
					return true;
			}
		}
		else
		{
			if ( ClipRayToWorld_R<IS_POINT>( ti, node->GetPos() ) )
				return true;
		}

		if ( node->HasLeaf2() )
		{
			u32 nTriangle = node->GetPrimitive2();

			if ( ti->m_filter->ShouldHitTriangle( nTriangle ) )
			{
				if ( IS_POINT )
					RayTriIntersect( ti, nTriangle );
				else
					SweepAABBTriIntersect( ti, nTriangle );

				if ( ti->m_trace.fraction == 0.0f )
					return true;
			}
		}
		else
		{
			if ( ClipRayToWorld_R<IS_POINT>( ti, node->GetNeg() ) )
				return true;
		}
	}

	return false;
}

bool CPhysicsQuery::IsBoxIntersectingWorld_R( TraceInfo_t *ti, const Opcode::AABBNoLeafNode *node )
{
	Fvector vMins = { node->mAABB.GetMin( 0 ), node->mAABB.GetMin( 1 ), node->mAABB.GetMin( 2 ) };
	Fvector vMaxs = { node->mAABB.GetMax( 0 ), node->mAABB.GetMax( 1 ), node->mAABB.GetMax( 2 ) };

	if ( IsBoxIntersectingBox( ti->m_absmins, ti->m_absmaxs, vMins, vMaxs ) )
	{
		if ( node->HasLeaf() )
		{
			u32 nTriangle = node->GetPrimitive();

			if ( ti->m_filter->ShouldHitTriangle( nTriangle ) )
			{
				if ( AABBTriIntersect( ti, nTriangle ) )
					return true;
			}
		}
		else
		{
			if ( IsBoxIntersectingWorld_R( ti, node->GetPos() ) )
				return true;
		}

		if ( node->HasLeaf2() )
		{
			u32 nTriangle = node->GetPrimitive2();

			if ( ti->m_filter->ShouldHitTriangle( nTriangle ) )
			{
				if ( AABBTriIntersect( ti, nTriangle ) )
					return true;
			}
		}
		else
		{
			if ( IsBoxIntersectingWorld_R( ti, node->GetNeg() ) )
				return true;
		}
	}

	return false;
}

template <bool IS_POINT>
void CPhysicsQuery::EnumerateObjectsAlongRay_R( TraceInfo_t *ti, ISpatial_NODE *node, const Fvector &vecCenter, float flRadius )
{
	float flBoxSize = flRadius * 2.0f;
	Fvector vMins = { vecCenter.x - flBoxSize, vecCenter.y - flBoxSize, vecCenter.z - flBoxSize };
	Fvector vMaxs = { vecCenter.x + flBoxSize, vecCenter.y + flBoxSize, vecCenter.z + flBoxSize };

	if ( !IS_POINT )
	{
		vMins.sub( ti->m_ray.m_Extents );
		vMaxs.add( ti->m_ray.m_Extents );
	}

	if ( IsBoxIntersectingRay( vMins, vMaxs, ti->m_ray.m_Start, ti->m_ray.m_Delta, ti->m_invdelta, PHQUERY_DIST_EPSILON ) )
	{
		xr_vector<ISpatial*>::iterator i = node->items.begin();
		xr_vector<ISpatial*>::iterator e = node->items.end();

		for ( ; i != e; i++ )
		{
			ISpatial *spatial = *i;
			Fsphere &sphere = spatial->GetSpatialData().sphere;

			Fvector vecSphereCenter = sphere.P;
			float flSphereRadius = sphere.R;

			vMins.set( vecSphereCenter.x - flSphereRadius, vecSphereCenter.y - flSphereRadius, vecSphereCenter.z - flSphereRadius );
			vMaxs.set( vecSphereCenter.x + flSphereRadius, vecSphereCenter.y + flSphereRadius, vecSphereCenter.z + flSphereRadius );

			if ( !IS_POINT )
			{
				vMins.sub( ti->m_ray.m_Extents );
				vMaxs.add( ti->m_ray.m_Extents );
			}

			if ( IsBoxIntersectingRay( vMins, vMaxs, ti->m_ray.m_Start, ti->m_ray.m_Delta, ti->m_invdelta, PHQUERY_DIST_EPSILON ) )
				ti->m_enum_results.push_back( spatial );
		}
	}

	float flChildrenRadius = flRadius / 2.0f;

	for ( u32 octant = 0; octant < 8; octant++ )
	{
		ISpatial_NODE *children = node->children[octant];
		if ( !children )
			continue;

		Fvector vecChildrenCenter;			
		vecChildrenCenter.mad( vecCenter, g_SpatialOffsets[octant], flChildrenRadius );

		EnumerateObjectsAlongRay_R<IS_POINT>( ti, children, vecChildrenCenter, flChildrenRadius );
	}
}

template <bool IS_POINT>
void CPhysicsQuery::EnumerateObjectsInBox_R( TraceInfo_t *ti, ISpatial_NODE *node, const Fvector &vecCenter, float flRadius )
{
	float flBoxSize = flRadius * 2.0f;
	Fvector vMins = { vecCenter.x - flBoxSize, vecCenter.y - flBoxSize, vecCenter.z - flBoxSize };
	Fvector vMaxs = { vecCenter.x + flBoxSize, vecCenter.y + flBoxSize, vecCenter.z + flBoxSize };

	if ( ( IS_POINT && IsPointInBox( ti->m_ray.m_Start, vMins, vMaxs ) ) ||
		( !IS_POINT && IsBoxIntersectingBox( ti->m_absmins, ti->m_absmaxs, vMins, vMaxs ) ) )
	{
		xr_vector<ISpatial*>::iterator i = node->items.begin();
		xr_vector<ISpatial*>::iterator e = node->items.end();

		for ( ; i != e; i++ )
		{
			ISpatial *spatial = *i;
			Fsphere &sphere = spatial->GetSpatialData().sphere;

			Fvector vecSphereCenter = sphere.P;
			float flSphereRadius = sphere.R;

			vMins.set( vecSphereCenter.x - flSphereRadius, vecSphereCenter.y - flSphereRadius, vecSphereCenter.z - flSphereRadius );
			vMaxs.set( vecSphereCenter.x + flSphereRadius, vecSphereCenter.y + flSphereRadius, vecSphereCenter.z + flSphereRadius );

			if ( ( IS_POINT && IsPointInBox( ti->m_ray.m_Start, vMins, vMaxs ) ) ||
				( !IS_POINT && IsBoxIntersectingBox( ti->m_absmins, ti->m_absmaxs, vMins, vMaxs ) ) )
				ti->m_enum_results.push_back( spatial );
		}
	}

	float flChildrenRadius = flRadius / 2.0f;

	for ( u32 octant = 0; octant < 8; octant++ )
	{
		ISpatial_NODE *children = node->children[octant];
		if ( !children )
			continue;

		Fvector vecChildrenCenter;
		vecChildrenCenter.mad( vecCenter, g_SpatialOffsets[octant], flChildrenRadius );

		EnumerateObjectsInBox_R<IS_POINT>( ti, children, vecChildrenCenter, flChildrenRadius );
	}
}

void CPhysicsQuery::ComputeFinalTx( dGeomID geom, bool bIsTransform, const dReal *pos, const dReal *rot, dReal *final_pos, dReal *final_rot )
{
    if ( bIsTransform )
	{
		dMULTIPLY0_331( final_pos, rot, dGeomGetPosition( geom ) );
		final_pos[0] += pos[0];
		final_pos[1] += pos[1];
		final_pos[2] += pos[2];
		dMULTIPLY0_333( final_rot, rot, dGeomGetRotation( geom ) );
	}
	else
	{
		CopyMemory( final_pos, dGeomGetPosition( geom ), sizeof( dVector3 ) );
		CopyMemory( final_rot, dGeomGetRotation( geom ), sizeof( dMatrix3 ) );
	}
}

bool CPhysicsQuery::ClipRayToGeometry( TraceInfo_t *ti, geom_info_t &geomInfo, trace_t &geomTrace )
{
    if ( geomTrace.startsolid || ( geomTrace.fraction < ti->m_trace.fraction ) )
	{
		ti->m_trace = geomTrace;
		ti->m_trace.object = ti->m_object;
        ti->m_trace.geom_info = geomInfo;

        dxGeomUserData *ud = retrieveGeomUserData( geomInfo.geometry );

        if ( ud )
            ti->m_trace.material = ud->material;
	}

    return geomTrace.startsolid;
}

bool CPhysicsQuery::ClipRayToGeometry_R( TraceInfo_t *ti, dGeomID geom, bool bIsTransform, const dReal *pos, const dReal *rot )
{
    dVector3 final_pos;
    dMatrix3 final_rot;
    Fmatrix xform;
    Fvector ext, mins, maxs;
    trace_t tr;

	if ( !ti->m_filter->ShouldHitGeometry( ti->m_object, geom ) )
		return false;
	
	int type = dGeomGetClass( geom );
	if ( type == dSphereClass )
	{
		ComputeFinalTx( geom, bIsTransform, pos, rot, final_pos, final_rot );

        xform.identity();
        CopyMemory( &xform.c, final_pos, sizeof( Fvector ) );

		float r = dGeomSphereGetRadius( geom );

		mins.set( final_pos[0] - r, final_pos[1] - r, final_pos[2] - r );
		maxs.set( final_pos[0] + r, final_pos[1] + r, final_pos[2] + r );

		IntersectRayWithBox( ti->m_ray, mins, maxs, PHQUERY_DIST_EPSILON, &tr );

        geom_info_t geom_info;
        geom_info.geometry = geom;
        geom_info.xform = xform;
        geom_info.mins.set( -r, -r, -r );
        geom_info.maxs.set( r, r, r );

		if ( ClipRayToGeometry( ti, geom_info, tr ) )
            return true;
	}
	else if ( type == dBoxClass )
	{
        ComputeFinalTx( geom, bIsTransform, pos, rot, final_pos, final_rot );

		PHDynamicData::DMXPStoFMX( final_rot, final_pos, xform );

		dGeomBoxGetLengths( geom, cast_fp( ext ) );
        ext.mul( 0.5f );

        mins.mul( ext, -1.0f );
		maxs = ext;

		IntersectRayWithOBB( ti->m_ray, xform, mins, maxs, PHQUERY_DIST_EPSILON, &tr );

		geom_info_t geom_info;
        geom_info.geometry = geom;
        geom_info.xform = xform;
        geom_info.mins = mins;
        geom_info.maxs = maxs;

		if ( ClipRayToGeometry( ti, geom_info, tr ) )
            return true;
	}
	else if ( type == dCylinderClassUser )
	{
		ComputeFinalTx( geom, bIsTransform, pos, rot, final_pos, final_rot );

		PHDynamicData::DMXPStoFMX( final_rot, final_pos, xform );

		float r = 1, h = 1;
		dGeomCylinderGetParams( geom, &r, &h );
        ext.set( r, h * 0.5f, r );

        mins.mul( ext, -1.0f );
		maxs = ext;

		IntersectRayWithOBB( ti->m_ray, xform, mins, maxs, PHQUERY_DIST_EPSILON, &tr );

		geom_info_t geom_info;
        geom_info.geometry = geom;
        geom_info.xform = xform;
        geom_info.mins = mins;
        geom_info.maxs = maxs;

		if ( ClipRayToGeometry( ti, geom_info, tr ) )
            return true;
	}
	else if ( type == dGeomTransformClass )
	{
		ComputeFinalTx( geom, bIsTransform, pos, rot, final_pos, final_rot );

		return ClipRayToGeometry_R( ti, dGeomTransformGetGeom( geom ), true, final_pos, final_rot );
	}
	else if ( type == dSimpleSpaceClass )
	{
		dSpaceID space = ( dSpaceID )geom;
		int numGeoms = dSpaceGetNumGeoms( space );
		for ( int i = 0; i < numGeoms; i++ )
		{
			if ( ClipRayToGeometry_R( ti, dSpaceGetGeom( space, i ) ) )
				return true;
		}
	}
	
	return false;
}

void CPhysicsQuery::TraceRay( const Ray_t &ray, ITraceFilter *pTraceFilter, trace_t *pTrace )
{
    Cache();

	CTraceFilterHitAll traceFilter;
	if ( !pTraceFilter )
		pTraceFilter = &traceFilter;

	pTrace->Clear();
	pTrace->startpos.add( ray.m_Start, ray.m_StartOffset );
	pTrace->endpos.add( pTrace->startpos, ray.m_Delta );

	TraceInfo_t ti;

	ti.m_ray = ray;
	ti.m_end.add( ray.m_Start, ray.m_Delta );
	ti.m_invdelta = ray.InvDelta();
	ti.m_mins.mul( ray.m_Extents, -1.0f );
	ti.m_maxs = ray.m_Extents;
	ti.m_absmins.sub( ray.m_Start, ray.m_Extents );
	ti.m_absmaxs.add( ray.m_Start, ray.m_Extents );
	ti.m_filter = pTraceFilter;

	CDB::MODEL *model = Level().ObjectSpace.GetStaticModel();

	ti.m_verts = model->get_verts();
	ti.m_tris = model->get_tris();

	ti.m_trace.Clear();

	float flWorldFraction = 1.0f;

	if ( pTraceFilter->GetTraceType() != TRACE_OBJECTS_ONLY )
	{
		const Opcode::AABBNoLeafTree *tree = ( const Opcode::AABBNoLeafTree* )model->get_tree()->GetTree();
		const Opcode::AABBNoLeafNode *nodes = tree->GetNodes();

		if ( ray.m_IsSwept )
		{
			if ( ray.m_IsRay )
				ClipRayToWorld_R<true>( &ti, nodes );
			else
				ClipRayToWorld_R<false>( &ti, nodes );
		}
		else
		{
			if ( !ray.m_IsRay )
				IsBoxIntersectingWorld_R( &ti, nodes );
		}
		
		if ( ti.m_trace.DidHit() )
		{
			*pTrace = ti.m_trace;
			pTrace->startpos.add( ray.m_Start, ray.m_StartOffset );
			pTrace->endpos.mad( pTrace->startpos, ray.m_Delta, pTrace->fraction );
			
			if ( pTrace->startsolid || ( pTraceFilter->GetTraceType() == TRACE_WORLD_ONLY ) )
				return;
			
			ti.m_end.mad( ray.m_Start, ray.m_Delta, pTrace->fraction );
			ti.m_ray.m_Delta.sub( ti.m_end, ray.m_Start );
			ti.m_ray.m_IsSwept = ( pTrace->fraction != 0.0f );
			ti.m_invdelta = ti.m_ray.InvDelta();

			flWorldFraction = pTrace->fraction;

			ti.m_trace.Clear();
		}
		else
		{
			if ( pTraceFilter->GetTraceType() == TRACE_WORLD_ONLY )
				return;
		}
	}

	if ( ti.m_ray.m_IsSwept )
	{
		if ( ray.m_IsRay )
			EnumerateObjectsAlongRay_R<true>( &ti, g_pGamePersistent->SpatialSpacePhysic.m_root, g_pGamePersistent->SpatialSpacePhysic.m_center, g_pGamePersistent->SpatialSpacePhysic.m_bounds );
		else
			EnumerateObjectsAlongRay_R<false>( &ti, g_pGamePersistent->SpatialSpacePhysic.m_root, g_pGamePersistent->SpatialSpacePhysic.m_center, g_pGamePersistent->SpatialSpacePhysic.m_bounds );
	}
	else
	{
		if ( ray.m_IsRay )
			EnumerateObjectsInBox_R<true>( &ti, g_pGamePersistent->SpatialSpacePhysic.m_root, g_pGamePersistent->SpatialSpacePhysic.m_center, g_pGamePersistent->SpatialSpacePhysic.m_bounds );
		else
			EnumerateObjectsInBox_R<false>( &ti, g_pGamePersistent->SpatialSpacePhysic.m_root, g_pGamePersistent->SpatialSpacePhysic.m_center, g_pGamePersistent->SpatialSpacePhysic.m_bounds );
	}
	
	xr_vector<ISpatial*>::iterator i = ti.m_enum_results.begin();
	xr_vector<ISpatial*>::iterator e = ti.m_enum_results.end();

	for ( ; i != e; i++ )
	{
		CPHObject *object = smart_cast< CPHObject* >( *i );

		if ( !pTraceFilter->ShouldHitObject( object ) )
			continue;

		ti.m_object = object;

		if ( ClipRayToGeometry_R( &ti, object->dSpacedGeom() ) )
			break;
	}

	if ( ti.m_trace.DidHit() )
	{
		*pTrace = ti.m_trace;
		pTrace->startpos.add( ray.m_Start, ray.m_StartOffset );
		pTrace->endpos.mad( pTrace->startpos, ti.m_ray.m_Delta, pTrace->fraction );
		pTrace->fraction *= flWorldFraction;
	}
}

void CPhysicsQuery::EnumerateGeometriesAlongRay_R( TraceInfo_t *ti, dGeomID geom, bool bIsTransform, const dReal *pos, const dReal *rot )
{
    dVector3 final_pos;
    dMatrix3 final_rot;
    Fmatrix xform;
    Fvector ext, mins, maxs;
    trace_t tr;

	if ( !ti->m_filter->ShouldHitGeometry( ti->m_object, geom ) )
		return;
	
	int type = dGeomGetClass( geom );
	if ( type == dSphereClass )
	{
		ComputeFinalTx( geom, bIsTransform, pos, rot, final_pos, final_rot );

        xform.identity();
        CopyMemory( &xform.c, final_pos, sizeof( Fvector ) );

		float r = dGeomSphereGetRadius( geom );

		mins.set( final_pos[0] - r, final_pos[1] - r, final_pos[2] - r );
		maxs.set( final_pos[0] + r, final_pos[1] + r, final_pos[2] + r );

		IntersectRayWithBox( ti->m_ray, mins, maxs, PHQUERY_DIST_EPSILON, &tr );

		if ( tr.DidHit() )
        {
            enum_geom_t enum_geom;
            enum_geom.object = ti->m_object;
            enum_geom.geom_info.geometry = geom;
            enum_geom.geom_info.xform = xform;
            enum_geom.geom_info.mins.set( -r, -r, -r );
            enum_geom.geom_info.maxs.set( r, r, r );
            ti->m_enum_geoms->push_back( enum_geom );
        }
	}
	else if ( type == dBoxClass )
	{
        ComputeFinalTx( geom, bIsTransform, pos, rot, final_pos, final_rot );

		PHDynamicData::DMXPStoFMX( final_rot, final_pos, xform );

		dGeomBoxGetLengths( geom, cast_fp( ext ) );
        ext.mul( 0.5f );

        mins.mul( ext, -1.0f );
		maxs = ext;

		IntersectRayWithOBB( ti->m_ray, xform, mins, maxs, PHQUERY_DIST_EPSILON, &tr );

        if ( tr.DidHit() )
        {
            enum_geom_t enum_geom;
            enum_geom.object = ti->m_object;
            enum_geom.geom_info.geometry = geom;
            enum_geom.geom_info.xform = xform;
            enum_geom.geom_info.mins = mins;
            enum_geom.geom_info.maxs = maxs;
            ti->m_enum_geoms->push_back( enum_geom );
        }
	}
	else if ( type == dCylinderClassUser )
	{
		ComputeFinalTx( geom, bIsTransform, pos, rot, final_pos, final_rot );

		PHDynamicData::DMXPStoFMX( final_rot, final_pos, xform );

		float r = 1, h = 1;
		dGeomCylinderGetParams( geom, &r, &h );
        ext.set( r, h * 0.5f, r );

        mins.mul( ext, -1.0f );
		maxs = ext;

		IntersectRayWithOBB( ti->m_ray, xform, mins, maxs, PHQUERY_DIST_EPSILON, &tr );

        if ( tr.DidHit() )
        {
            enum_geom_t enum_geom;
            enum_geom.object = ti->m_object;
            enum_geom.geom_info.geometry = geom;
            enum_geom.geom_info.xform = xform;
            enum_geom.geom_info.mins = mins;
            enum_geom.geom_info.maxs = maxs;
            ti->m_enum_geoms->push_back( enum_geom );
        }
	}
	else if ( type == dGeomTransformClass )
	{
		ComputeFinalTx( geom, bIsTransform, pos, rot, final_pos, final_rot );

		EnumerateGeometriesAlongRay_R( ti, dGeomTransformGetGeom( geom ), true, final_pos, final_rot );
	}
	else if ( type == dSimpleSpaceClass )
	{
		dSpaceID space = ( dSpaceID )geom;
		int numGeoms = dSpaceGetNumGeoms( space );
		for ( int i = 0; i < numGeoms; i++ )
		{
			EnumerateGeometriesAlongRay_R( ti, dSpaceGetGeom( space, i ) );
		}
	}
}

void CPhysicsQuery::EnumerateGeometriesAlongRay( const Ray_t &ray, ITraceFilter *pTraceFilter, xr_vector<enum_geom_t>& enum_geoms )
{
    Cache();

    CTraceFilterHitAll traceFilter;
	if ( !pTraceFilter )
		pTraceFilter = &traceFilter;

    TraceInfo_t ti;

	ti.m_ray = ray;
	ti.m_invdelta = ray.InvDelta();
	ti.m_absmins.sub( ray.m_Start, ray.m_Extents );
	ti.m_absmaxs.add( ray.m_Start, ray.m_Extents );
    ti.m_enum_geoms = &enum_geoms;
    ti.m_filter = pTraceFilter;

    if ( ray.m_IsSwept )
	{
		if ( ray.m_IsRay )
			EnumerateObjectsAlongRay_R<true>( &ti, g_pGamePersistent->SpatialSpacePhysic.m_root, g_pGamePersistent->SpatialSpacePhysic.m_center, g_pGamePersistent->SpatialSpacePhysic.m_bounds );
		else
			EnumerateObjectsAlongRay_R<false>( &ti, g_pGamePersistent->SpatialSpacePhysic.m_root, g_pGamePersistent->SpatialSpacePhysic.m_center, g_pGamePersistent->SpatialSpacePhysic.m_bounds );
	}
	else
	{
		if ( ray.m_IsRay )
			EnumerateObjectsInBox_R<true>( &ti, g_pGamePersistent->SpatialSpacePhysic.m_root, g_pGamePersistent->SpatialSpacePhysic.m_center, g_pGamePersistent->SpatialSpacePhysic.m_bounds );
		else
			EnumerateObjectsInBox_R<false>( &ti, g_pGamePersistent->SpatialSpacePhysic.m_root, g_pGamePersistent->SpatialSpacePhysic.m_center, g_pGamePersistent->SpatialSpacePhysic.m_bounds );
	}

    xr_vector<ISpatial*>::iterator i = ti.m_enum_results.begin();
	xr_vector<ISpatial*>::iterator e = ti.m_enum_results.end();

    for ( ; i != e; i++ )
	{
		CPHObject *object = smart_cast< CPHObject* >( *i );

		if ( !pTraceFilter->ShouldHitObject( object ) )
			continue;

		ti.m_object = object;

        EnumerateGeometriesAlongRay_R( &ti, object->dSpacedGeom() );
	}
}
