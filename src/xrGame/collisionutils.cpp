//========= Copyright Valve Corporation, All rights reserved. ============//
//
// Purpose: Common collision utility methods
//
// $Header: $
// $NoKeywords: $
//=============================================================================//

#include "stdafx.h"
#include "physicsquery.h"
#include "collisionutils.h"


//-----------------------------------------------------------------------------
// Clears the trace
//-----------------------------------------------------------------------------
static void Collision_ClearTrace( const Fvector &vecRayStart, const Fvector &vecRayDelta, trace_t *pTrace )
{
	pTrace->Clear();
	pTrace->startpos = vecRayStart;
	pTrace->endpos.add( vecRayStart, vecRayDelta );
}


//-----------------------------------------------------------------------------
// Intersects a swept box against a triangle
//-----------------------------------------------------------------------------
float IntersectRayWithTriangle( const Fvector &rayStart, const Fvector &rayDelta,
	const Fvector& v1, const Fvector& v2, const Fvector& v3, bool oneSided )
{
	// This is cute: Use barycentric coordinates to represent the triangle
	// Vo(1-u-v) + V1u + V2v and intersect that with a line Po + Dt
	// This gives us 3 equations + 3 unknowns, which we can solve with
	// Cramer's rule...
	//		E1x u + E2x v - Dx t = Pox - Vox
	// There's a couple of other optimizations, Cramer's rule involves
	// computing the determinant of a matrix which has been constructed
	// by three vectors. It turns out that 
	// det | A B C | = -( A x C ) dot B or -(C x B) dot A
	// which we'll use below..

	Fvector edge1, edge2, org;
	edge1.sub( v2, v1 );
	edge2.sub( v3, v1 );

	// Cull out one-sided stuff
	if ( oneSided )
	{
		Fvector normal;
		normal.crossproduct( edge1, edge2 );
		if ( normal.dotproduct( rayDelta ) >= 0.0f )
			return -1.0f;
	}

	// FIXME: This is inaccurate, but fast for boxes
	// We want to do a fast separating axis implementation here
	// with a swept triangle along the reverse direction of the ray.

	// Compute some intermediary terms
	Fvector dirCrossEdge2, orgCrossEdge1;
	dirCrossEdge2.crossproduct( rayDelta, edge2 );

	// Compute the denominator of Cramer's rule:
	//		| -Dx E1x E2x |
	// det	| -Dy E1y E2y | = (D x E2) dot E1
	//		| -Dz E1z E2z |
	float denom = dirCrossEdge2.dotproduct( edge1 );
	if ( _abs( denom ) < 1e-6f )
		return -1.0f;
	denom = 1.0f / denom;

	// Compute u. It's gotta lie in the range of 0 to 1.
	//				   | -Dx orgx E2x |
	// u = denom * det | -Dy orgy E2y | = (D x E2) dot org
	//				   | -Dz orgz E2z |
	org.sub( rayStart, v1 );
	float u = dirCrossEdge2.dotproduct( org ) * denom;
	if ( ( u < 0.0f ) || ( u > 1.0f ) )
		return -1.0f;

	// Compute t and v the same way...
	// In barycentric coords, u + v < 1
	orgCrossEdge1.crossproduct( org, edge1 );
	float v = orgCrossEdge1.dotproduct( rayDelta ) * denom;
	if ( ( v < 0.0f ) || ( v + u > 1.0f ) )
		return -1.0f;

	// Compute the distance along the ray direction that we need to fudge 
	// when using swept boxes
	float t = orgCrossEdge1.dotproduct( edge2 ) * denom;
	if ( ( t < -1e-3f ) || ( t > 1.0f + 1e-3f ) )
		return -1.0f;

	clamp( t, 0.f, 1.f );
	return t;
}


//-----------------------------------------------------------------------------
// Returns true if a box intersects with a sphere
//-----------------------------------------------------------------------------
bool IsBoxIntersectingSphere( const Fvector& boxMin, const Fvector& boxMax,
	const Fvector& center, float radius )
{
	// See Graphics Gems, box-sphere intersection
	float dmin = 0.0f;
	float flDelta;

	// Unrolled the loop.. this is a big cycle stealer...
	if ( center[0] < boxMin[0] )
	{
		flDelta = center[0] - boxMin[0];
		dmin += flDelta * flDelta;
	}
	else if ( center[0] > boxMax[0] )
	{
		flDelta = boxMax[0] - center[0];
		dmin += flDelta * flDelta;
	}

	if ( center[1] < boxMin[1] )
	{
		flDelta = center[1] - boxMin[1];
		dmin += flDelta * flDelta;
	}
	else if ( center[1] > boxMax[1] )
	{
		flDelta = boxMax[1] - center[1];
		dmin += flDelta * flDelta;
	}

	if ( center[2] < boxMin[2] )
	{
		flDelta = center[2] - boxMin[2];
		dmin += flDelta * flDelta;
	}
	else if ( center[2] > boxMax[2] )
	{
		flDelta = boxMax[2] - center[2];
		dmin += flDelta * flDelta;
	}

	return dmin < radius * radius;
}


//-----------------------------------------------------------------------------
// returns true if there's an intersection between ray and sphere
//-----------------------------------------------------------------------------
bool IsRayIntersectingSphere( const Fvector &vecRayOrigin, const Fvector &vecRayDelta,
	const Fvector& vecCenter, float flRadius, float flTolerance )
{
	// For this algorithm, find a point on the ray  which is closest to the sphere origin
	// Do this by making a plane passing through the sphere origin
	// whose normal is parallel to the ray. Intersect that plane with the ray.
	// Plane: N dot P = I, N = D (ray direction), I = C dot N = C dot D
	// Ray: P = O + D * t
	// D dot ( O + D * t ) = C dot D
	// D dot O + D dot D * t = C dot D
	// t = (C - O) dot D / D dot D
	// Clamp t to (0,1)
	// Find distance of the point on the ray to the sphere center.
	VERIFY( flTolerance >= 0.0f );
	flRadius += flTolerance;

	Fvector vecRayToSphere;
	vecRayToSphere.sub( vecCenter, vecRayOrigin );
	float flNumerator = vecRayToSphere.dotproduct( vecRayDelta );
	
	float t;
	if (flNumerator <= 0.0f)
	{
		t = 0.0f;
	}
	else
	{
		float flDenominator = vecRayDelta.dotproduct( vecRayDelta );
		if ( flNumerator > flDenominator )
			t = 1.0f;
		else
			t = flNumerator / flDenominator;
	}
	
	Fvector vecClosestPoint;
	vecClosestPoint.mad( vecRayOrigin, vecRayDelta, t );
	return ( vecClosestPoint.distance_to_sqr( vecCenter ) <= flRadius * flRadius );

	// NOTE: This in an alternate algorithm which I didn't use because I'd have to use a sqrt
	// So it's probably faster to do this other algorithm. I'll leave the comments here
	// for how to go back if we want to

	// Solve using the ray equation + the sphere equation
	// P = o + dt
	// (x - xc)^2 + (y - yc)^2 + (z - zc)^2 = r^2
	// (ox + dx * t - xc)^2 + (oy + dy * t - yc)^2 + (oz + dz * t - zc)^2 = r^2
	// (ox - xc)^2 + 2 * (ox-xc) * dx * t + dx^2 * t^2 +
	//		(oy - yc)^2 + 2 * (oy-yc) * dy * t + dy^2 * t^2 +
	//		(oz - zc)^2 + 2 * (oz-zc) * dz * t + dz^2 * t^2 = r^2
	// (dx^2 + dy^2 + dz^2) * t^2 + 2 * ((ox-xc)dx + (oy-yc)dy + (oz-zc)dz) t +
	//		(ox-xc)^2 + (oy-yc)^2 + (oz-zc)^2 - r^2 = 0
	// or, t = (-b +/- sqrt( b^2 - 4ac)) / 2a
	// a = DotProduct( vecRayDelta, vecRayDelta );
	// b = 2 * DotProduct( vecRayOrigin - vecCenter, vecRayDelta )
	// c = DotProduct(vecRayOrigin - vecCenter, vecRayOrigin - vecCenter) - flRadius * flRadius;
	// Valid solutions are possible only if b^2 - 4ac >= 0
	// Therefore, compute that value + see if we got it	
}


//-----------------------------------------------------------------------------
// returns true if there's an intersection between two boxes
//-----------------------------------------------------------------------------
bool IsBoxIntersectingBox( const Fvector& boxMin1, const Fvector& boxMax1,
	const Fvector& boxMin2, const Fvector& boxMax2 )
{
	VERIFY( boxMin1[0] <= boxMax1[0] );
	VERIFY( boxMin1[1] <= boxMax1[1] );
	VERIFY( boxMin1[2] <= boxMax1[2] );
	VERIFY( boxMin2[0] <= boxMax2[0] );
	VERIFY( boxMin2[1] <= boxMax2[1] );
	VERIFY( boxMin2[2] <= boxMax2[2] );

	if ( (boxMin1[0] > boxMax2[0]) || (boxMax1[0] < boxMin2[0]) )
		return false;
	if ( (boxMin1[1] > boxMax2[1]) || (boxMax1[1] < boxMin2[1]) )
		return false;
	if ( (boxMin1[2] > boxMax2[2]) || (boxMax1[2] < boxMin2[2]) )
		return false;
	return true;
}


//-----------------------------------------------------------------------------
// returns true if the point is in the box
//-----------------------------------------------------------------------------
bool IsPointInBox( const Fvector& pt, const Fvector& boxMin, const Fvector& boxMax )
{
	VERIFY( boxMin[0] <= boxMax[0] );
	VERIFY( boxMin[1] <= boxMax[1] );
	VERIFY( boxMin[2] <= boxMax[2] );

	if ( ( pt[0] > boxMax[0] ) || ( pt[0] < boxMin[0] ) )
		return false;
	if ( ( pt[1] > boxMax[1] ) || ( pt[1] < boxMin[1] ) )
		return false;
	if ( ( pt[2] > boxMax[2] ) || ( pt[2] < boxMin[2] ) )
		return false;
	return true;
}


//-----------------------------------------------------------------------------
// returns true if there's an intersection between box and ray
//-----------------------------------------------------------------------------
bool IsBoxIntersectingRay( const Fvector& boxMin, const Fvector& boxMax,
	const Fvector& origin, const Fvector& vecDelta,
	const Fvector& vecInvDelta, float flTolerance )
{	
	VERIFY( boxMin[0] <= boxMax[0] );
	VERIFY( boxMin[1] <= boxMax[1] );
	VERIFY( boxMin[2] <= boxMax[2] );

	// FIXME: Surely there's a faster way
	float tmin = -flt_max;
	float tmax = flt_max;

	for ( int i = 0; i < 3; ++i )
	{
		// Parallel case...
		if ( _abs( vecDelta[i] ) < 1e-8f )
		{
			// Check that origin is in the box, if not, then it doesn't intersect..
			if ( ( origin[i] < boxMin[i] - flTolerance ) || ( origin[i] > boxMax[i] + flTolerance ) )
				return false;

			continue;
		}

		// Non-parallel case
		// Find the t's corresponding to the entry and exit of
		// the ray along x, y, and z. The find the furthest entry
		// point, and the closest exit point. Once that is done,
		// we know we don't collide if the closest exit point
		// is behind the starting location. We also don't collide if
		// the closest exit point is in front of the furthest entry point
		float t1 = ( boxMin[i] - flTolerance - origin[i] ) * vecInvDelta[i];
		float t2 = ( boxMax[i] + flTolerance - origin[i] ) * vecInvDelta[i];
		if ( t1 > t2 )
		{
			float temp = t1;
			t1 = t2;
			t2 = temp;
		}

		if (t1 > tmin)
			tmin = t1;

		if (t2 < tmax)
			tmax = t2;

		if (tmin > tmax)
			return false;

		if (tmax < 0)
			return false;

		if (tmin > 1)
			return false;
	}

	return true;
}


//-----------------------------------------------------------------------------
// Intersects a ray against a box
//-----------------------------------------------------------------------------
bool IntersectRayWithBox( const Fvector &vecRayStart, const Fvector &vecRayDelta,
	const Fvector &boxMins, const Fvector &boxMaxs, float flTolerance, BoxTraceInfo_t *pTrace )
{
	int			i;
	float		d1, d2;
	float		f;

	pTrace->t1 = -1.0f;
	pTrace->t2 = 1.0f;
	pTrace->hitside = -1;

	// UNDONE: This makes this code a little messy
	pTrace->startsolid = true;

	for ( i = 0; i < 6; ++i )
	{
		if ( i >= 3 )
		{
			d1 = vecRayStart[i-3] - boxMaxs[i-3];
			d2 = d1 + vecRayDelta[i-3];
		}
		else
		{
			d1 = -vecRayStart[i] + boxMins[i];
			d2 = d1 - vecRayDelta[i];
		}

		// if completely in front of face, no intersection
		if (d1 > 0 && d2 > 0)
		{
			// UNDONE: Have to revert this in case it's still set
			// UNDONE: Refactor to have only 2 return points (true/false) from this function
			pTrace->startsolid = false;
			return false;
		}

		// completely inside, check next face
		if (d1 <= 0 && d2 <= 0)
			continue;

		if (d1 > 0)
		{
			pTrace->startsolid = false;
		}

		// crosses face
		if (d1 > d2)
		{
			f = d1 - flTolerance;
			if ( f < 0 )
			{
				f = 0;
			}
			f = f / (d1-d2);
			if (f > pTrace->t1)
			{
				pTrace->t1 = f;
				pTrace->hitside = i;
			}
		}
		else
		{ 
			// leave
			f = (d1 + flTolerance) / (d1-d2);
			if (f < pTrace->t2)
			{
				pTrace->t2 = f;
			}
		}
	}

	return pTrace->startsolid || (pTrace->t1 < pTrace->t2 && pTrace->t1 >= 0.0f);
}


//-----------------------------------------------------------------------------
// Intersects a ray against a box
//-----------------------------------------------------------------------------
bool IntersectRayWithBox( const Fvector &vecRayStart, const Fvector &vecRayDelta,
	const Fvector &boxMins, const Fvector &boxMaxs, float flTolerance, trace_t *pTrace )
{
	Collision_ClearTrace( vecRayStart, vecRayDelta, pTrace );

	BoxTraceInfo_t trace;

	if ( IntersectRayWithBox( vecRayStart, vecRayDelta, boxMins, boxMaxs, flTolerance, &trace ) )
	{
		pTrace->startsolid = trace.startsolid;
		if (trace.t1 < trace.t2 && trace.t1 >= 0.0f)
		{
			pTrace->fraction = trace.t1;
			pTrace->endpos.mad( pTrace->startpos, vecRayDelta, trace.t1 );
			pTrace->plane.normal.set( 0.0f, 0.0f, 0.0f );
			if ( trace.hitside >= 3 )
			{
				trace.hitside -= 3;
				pTrace->plane.dist = boxMaxs[trace.hitside];
				pTrace->plane.normal[trace.hitside] = 1.0f;
				pTrace->plane.type = trace.hitside;
			}
			else
			{
				pTrace->plane.dist = -boxMins[trace.hitside];
				pTrace->plane.normal[trace.hitside] = -1.0f;
				pTrace->plane.type = trace.hitside;
			}
			return true;
		}

		if ( pTrace->startsolid )
		{
			pTrace->fraction = 0;
			pTrace->endpos = pTrace->startpos;
			pTrace->plane.dist = pTrace->startpos[0];
			pTrace->plane.normal.set( 1.0f, 0.0f, 0.0f );
			pTrace->plane.type = 0;
			return true;
		}
	}

	return false;
}


//-----------------------------------------------------------------------------
// Intersects a ray against a box
//-----------------------------------------------------------------------------
bool IntersectRayWithBox( const Ray_t &ray, const Fvector &boxMins, const Fvector &boxMaxs,
	float flTolerance, trace_t *pTrace )
{
	if ( !ray.m_IsRay )
	{
		Fvector vecExpandedMins, vecExpandedMaxs;
		vecExpandedMins.sub( boxMins, ray.m_Extents );
		vecExpandedMaxs.add( boxMaxs, ray.m_Extents );
		bool bIntersects = IntersectRayWithBox( ray.m_Start, ray.m_Delta, vecExpandedMins, vecExpandedMaxs, flTolerance, pTrace );
		pTrace->startpos.add( ray.m_StartOffset );
		pTrace->endpos.add( ray.m_StartOffset );
		return bIntersects;
	}
	return IntersectRayWithBox( ray.m_Start, ray.m_Delta, boxMins, boxMaxs, flTolerance, pTrace );
}


//-----------------------------------------------------------------------------
// Intersects a ray against an OBB
//-----------------------------------------------------------------------------
bool IntersectRayWithOBB( const Fvector &vecRayStart, const Fvector &vecRayDelta,
	const Fmatrix &matOBBToWorld, const Fvector &vecOBBMins, const Fvector &vecOBBMaxs,
    float flTolerance, trace_t *pTrace )
{
	Collision_ClearTrace( vecRayStart, vecRayDelta, pTrace );

	// FIXME: Make it work with tolerance
	VERIFY( flTolerance >= 0.0f );

	// OPTIMIZE: Store this in the box instead of computing it here
	// compute center in local space
	Fvector vecBoxExtents;
    vecBoxExtents.add( vecOBBMins, vecOBBMaxs ); 
    vecBoxExtents.mul( 0.5f );
	Fvector vecBoxCenter;

	// transform to world space
    matOBBToWorld.transform_tiny( vecBoxCenter, vecBoxExtents );

	// calc extents from local center
	vecBoxExtents.sub( vecOBBMaxs, vecBoxExtents );

	// OPTIMIZE: This is optimized for world space.  If the transform is fast enough, it may make more
	// sense to just xform and call UTIL_ClipToBox() instead.  MEASURE THIS.

	// save the extents of the ray along 
	Fvector extent, uextent;
    Fvector segmentCenter;
	segmentCenter.add( vecRayStart, vecRayDelta );
    segmentCenter.sub( vecBoxCenter );

	extent.set( 0.0f, 0.0f, 0.0f );

	// check box axes for separation
	for ( int j = 0; j < 3; j++ )
	{
		extent[j] = vecRayDelta.x * matOBBToWorld.m[j][0] + vecRayDelta.y * matOBBToWorld.m[j][1] + vecRayDelta.z * matOBBToWorld.m[j][2];
		uextent[j] = _abs( extent[j] );
		float coord = segmentCenter.x * matOBBToWorld.m[j][0] + segmentCenter.y * matOBBToWorld.m[j][1] + segmentCenter.z * matOBBToWorld.m[j][2];
		coord = _abs(coord);

		if ( coord > (vecBoxExtents[j] + uextent[j]) )
			return false;
	}

	// now check cross axes for separation
	float tmp, cextent;
	Fvector cross;
	cross.crossproduct( vecRayDelta, segmentCenter );
	cextent = cross.x * matOBBToWorld.m[0][0] + cross.y * matOBBToWorld.m[0][1] + cross.z * matOBBToWorld.m[0][2];
	cextent = _abs( cextent );
	tmp = vecBoxExtents[1]*uextent[2] + vecBoxExtents[2]*uextent[1];
	if ( cextent > tmp )
		return false;

	cextent = cross.x * matOBBToWorld.m[1][0] + cross.y * matOBBToWorld.m[1][1] + cross.z * matOBBToWorld.m[1][2];
	cextent = _abs( cextent );
	tmp = vecBoxExtents[0]*uextent[2] + vecBoxExtents[2]*uextent[0];
	if ( cextent > tmp )
		return false;

	cextent = cross.x * matOBBToWorld.m[2][0] + cross.y * matOBBToWorld.m[2][1] + cross.z * matOBBToWorld.m[2][2];
	cextent = _abs( cextent );
	tmp = vecBoxExtents[0]*uextent[1] + vecBoxExtents[1]*uextent[0];
	if ( cextent > tmp )
		return false;

	// !!! We hit this box !!! compute intersection point and return
	// Compute ray start in bone space
	Fvector start;
    matOBBToWorld.inverse_transform_tiny( start, vecRayStart );

	// extent is ray.m_Delta in bone space, recompute delta in bone space
	extent.mul( 2.0f );

	// delta was prescaled by the current t, so no need to see if this intersection
	// is closer
	if ( !IntersectRayWithBox( start, extent, vecOBBMins, vecOBBMaxs, flTolerance, pTrace ) )
		return false;

	// Fix up the start/end pos and fraction
    Fvector vecTemp;
	matOBBToWorld.transform_tiny( vecTemp, pTrace->endpos );
	pTrace->endpos = vecTemp;

	pTrace->startpos = vecRayStart;
	pTrace->fraction *= 2.0f;

	// Fix up the plane information
	float flSign = pTrace->plane.normal[ pTrace->plane.type ];
	pTrace->plane.normal[0] = flSign * matOBBToWorld.m[pTrace->plane.type][0];
	pTrace->plane.normal[1] = flSign * matOBBToWorld.m[pTrace->plane.type][1];
	pTrace->plane.normal[2] = flSign * matOBBToWorld.m[pTrace->plane.type][2];
	pTrace->plane.dist = pTrace->endpos.dotproduct( pTrace->plane.normal );
	pTrace->plane.type = 3;

	return true;
}


//-----------------------------------------------------------------------------
// Box support map
//-----------------------------------------------------------------------------
inline void ComputeSupportMap( const Fvector &vecDirection, const Fvector &vecBoxMins,
	const Fvector &vecBoxMaxs, float pDist[2] )
{
	int nIndex = (vecDirection.x > 0.0f);
	pDist[nIndex] = vecBoxMaxs.x * vecDirection.x;
	pDist[1 - nIndex] = vecBoxMins.x * vecDirection.x;

	nIndex = (vecDirection.y > 0.0f);
	pDist[nIndex] += vecBoxMaxs.y * vecDirection.y;
	pDist[1 - nIndex] += vecBoxMins.y * vecDirection.y;

	nIndex = (vecDirection.z > 0.0f);
	pDist[nIndex] += vecBoxMaxs.z * vecDirection.z;
	pDist[1 - nIndex] += vecBoxMins.z * vecDirection.z;
}

inline void ComputeSupportMap( const Fvector &vecDirection, int i1, int i2,
	const Fvector &vecBoxMins, const Fvector &vecBoxMaxs, float pDist[2] )
{
	int nIndex = (vecDirection[i1] > 0.0f);
	pDist[nIndex] = vecBoxMaxs[i1] * vecDirection[i1];
	pDist[1 - nIndex] = vecBoxMins[i1] * vecDirection[i1];

	nIndex = (vecDirection[i2] > 0.0f);
	pDist[nIndex] += vecBoxMaxs[i2] * vecDirection[i2];
	pDist[1 - nIndex] += vecBoxMins[i2] * vecDirection[i2];
}

//-----------------------------------------------------------------------------
// Intersects a ray against an OBB
//-----------------------------------------------------------------------------
static int s_ExtIndices[3][2] = 
{
	{ 2, 1 },
	{ 0, 2 },
	{ 0, 1 },
};

static int s_MatIndices[3][2] = 
{
	{ 1, 2 },
	{ 2, 0 },
	{ 1, 0 },
};

bool IntersectRayWithOBB( const Ray_t &ray, const Fmatrix &matOBBToWorld,
	const Fvector &vecOBBMins, const Fvector &vecOBBMaxs, float flTolerance, trace_t *pTrace )
{
	if ( ray.m_IsRay )
	{
		return IntersectRayWithOBB( ray.m_Start, ray.m_Delta, matOBBToWorld, 
			vecOBBMins, vecOBBMaxs, flTolerance, pTrace );
	}
	
	Collision_ClearTrace( Fvector().add( ray.m_Start, ray.m_StartOffset ), ray.m_Delta, pTrace );

	// Compute a bounding sphere around the bloated OBB
	Fvector vecOBBExtents;
	vecOBBExtents.add( vecOBBMins, vecOBBMaxs );
	vecOBBExtents.mul( 0.5f );

	Fvector vecOBBCenter;
    matOBBToWorld.transform_tiny( vecOBBCenter, vecOBBExtents );

	Fvector vecOBBHalfDiagonal;
	vecOBBHalfDiagonal.sub( vecOBBMaxs, vecOBBMins );
	vecOBBHalfDiagonal.mul( 0.5f );

	float flRadius = vecOBBHalfDiagonal.magnitude() + ray.m_Extents.magnitude();
	if ( !IsRayIntersectingSphere( ray.m_Start, ray.m_Delta, vecOBBCenter, flRadius, flTolerance ) )
		return false;

	// Ok, we passed the trivial reject, so lets do the dirty deed.
	// Basically we're going to do the GJK thing explicitly. We'll shrink the ray down
	// to a point, and bloat the OBB by the ray's extents. This will generate facet
	// planes which are perpendicular to all of the separating axes typically seen in
	// a standard seperating axis implementation.

	// We're going to create a number of planes through various vertices in the OBB
	// which represent all of the separating planes. Then we're going to bloat the planes
	// by the ray extents.
	

	// We're going to do all work in OBB-space because it's easier to do the
	// support-map in this case

	// First, transform the ray into the space of the OBB
	Fvector vecLocalRayOrigin, vecLocalRayDirection;
    matOBBToWorld.inverse_transform_tiny( vecLocalRayOrigin, ray.m_Start );
	matOBBToWorld.inverse_transform_dir( vecLocalRayDirection, ray.m_Delta );

	// Next compute all separating planes
	Fvector pPlaneNormal[15];
	float ppPlaneDist[15][2];

	int i;
	for ( i = 0; i < 3; ++i )
	{
		// Each plane needs to be bloated an amount = to the abs dot product of 
		// the ray extents with the plane normal
		// For the OBB planes, do it in world space; 
		// and use the direction of the OBB (the ith column of matOBBToWorld) in world space vs extents
		pPlaneNormal[i].set( 0.0f, 0.0f, 0.0f );
		pPlaneNormal[i][i] = 1.0f;

		float flExtentDotNormal = 
			_abs( matOBBToWorld.m[i][0] * ray.m_Extents.x ) +
			_abs( matOBBToWorld.m[i][1] * ray.m_Extents.y ) +
			_abs( matOBBToWorld.m[i][2] * ray.m_Extents.z );

		ppPlaneDist[i][0] = vecOBBMins[i] - flExtentDotNormal; 
		ppPlaneDist[i][1] = vecOBBMaxs[i] + flExtentDotNormal;

		// For the ray-extents planes, they are bloated by the extents
		// Use the support map to determine which
		pPlaneNormal[i+3][0] = matOBBToWorld.m[0][i];
		pPlaneNormal[i+3][1] = matOBBToWorld.m[1][i];
		pPlaneNormal[i+3][2] = matOBBToWorld.m[2][i];
		ComputeSupportMap( pPlaneNormal[i+3], vecOBBMins, vecOBBMaxs, ppPlaneDist[i+3] );
		ppPlaneDist[i+3][0] -= ray.m_Extents[i];
		ppPlaneDist[i+3][1] += ray.m_Extents[i];

		// Now the edge cases... (take the cross product of x,y,z axis w/ ray extent axes
		// given by the rows of the obb to world matrix. 
		// Compute the ray extent bloat in world space because it's easier...

		// These are necessary to compute the world-space versions of
		// the edges so we can compute the extent dot products
		float flRayExtent0 = ray.m_Extents[s_ExtIndices[i][0]];
		float flRayExtent1 = ray.m_Extents[s_ExtIndices[i][1]];

		// x axis of the OBB + world ith axis
		pPlaneNormal[i+6].set( 0.0f, -matOBBToWorld.m[2][i], matOBBToWorld.m[1][i] );
		ComputeSupportMap( pPlaneNormal[i+6], 1, 2, vecOBBMins, vecOBBMaxs, ppPlaneDist[i+6] );
		flExtentDotNormal = 
			_abs( matOBBToWorld.m[0][s_MatIndices[i][0]] ) * flRayExtent0 +
			_abs( matOBBToWorld.m[0][s_MatIndices[i][1]] ) * flRayExtent1;
		ppPlaneDist[i+6][0] -= flExtentDotNormal;
		ppPlaneDist[i+6][1] += flExtentDotNormal;
		
		// y axis of the OBB + world ith axis
		pPlaneNormal[i+9].set( matOBBToWorld.m[2][i], 0.0f, -matOBBToWorld.m[0][i] );
		ComputeSupportMap( pPlaneNormal[i+9], 0, 2, vecOBBMins, vecOBBMaxs, ppPlaneDist[i+9] );
		flExtentDotNormal = 
			_abs( matOBBToWorld.m[1][s_MatIndices[i][0]] ) * flRayExtent0 +
			_abs( matOBBToWorld.m[1][s_MatIndices[i][1]] ) * flRayExtent1;
		ppPlaneDist[i+9][0] -= flExtentDotNormal;
		ppPlaneDist[i+9][1] += flExtentDotNormal;

		// z axis of the OBB + world ith axis
		pPlaneNormal[i+12].set( -matOBBToWorld.m[1][i], matOBBToWorld.m[0][i], 0.0f );
		ComputeSupportMap( pPlaneNormal[i+12], 0, 1, vecOBBMins, vecOBBMaxs, ppPlaneDist[i+12] );
		flExtentDotNormal = 
			_abs( matOBBToWorld.m[2][s_MatIndices[i][0]] ) * flRayExtent0 +
			_abs( matOBBToWorld.m[2][s_MatIndices[i][1]] ) * flRayExtent1;
		ppPlaneDist[i+12][0] -= flExtentDotNormal;
		ppPlaneDist[i+12][1] += flExtentDotNormal;
	}

	float enterfrac, leavefrac;
	float d1[2], d2[2];
	float f;

	int hitplane = -1;
	int hitside = -1;
	enterfrac = -1.0f;
	leavefrac = 1.0f;

	pTrace->startsolid = true;

	Fvector vecLocalRayEnd;
	vecLocalRayEnd.add( vecLocalRayOrigin, vecLocalRayDirection );

	for ( i = 0; i < 15; ++i )
	{
		// FIXME: Not particularly optimal since there's a lot of 0's in the plane normals
		float flStartDot = pPlaneNormal[i].dotproduct( vecLocalRayOrigin );
		float flEndDot = pPlaneNormal[i].dotproduct( vecLocalRayEnd );

		// NOTE: Negative here is because the plane normal + dist 
		// are defined in negative terms for the far plane (plane dist index 0)
		d1[0] = -(flStartDot - ppPlaneDist[i][0]);
		d2[0] = -(flEndDot - ppPlaneDist[i][0]);

		d1[1] = flStartDot - ppPlaneDist[i][1];
		d2[1] = flEndDot - ppPlaneDist[i][1];

		int j;
		for ( j = 0; j < 2; ++j )
		{
			// if completely in front near plane or behind far plane no intersection
			if (d1[j] > 0 && d2[j] > 0)
			{
				pTrace->startsolid = false;
				return false;
			}

			// completely inside, check next plane set
			if (d1[j] <= 0 && d2[j] <= 0)
				continue;

			if (d1[j] > 0)
			{
				pTrace->startsolid = false;
			}

			// crosses face
			float flDenom = 1.0f / (d1[j] - d2[j]);
			if (d1[j] > d2[j])
			{
				f = d1[j] - flTolerance;
				if ( f < 0 )
				{
					f = 0;
				}
				f *= flDenom;
				if (f > enterfrac)
				{
					enterfrac = f;
					hitplane = i;
					hitside = j;
				}
			}
			else
			{ 
				// leave
				f = (d1[j] + flTolerance) * flDenom;
				if (f < leavefrac)
				{
					leavefrac = f;
				}
			}
		}
	}

	if (enterfrac < leavefrac && enterfrac >= 0.0f)
	{
		pTrace->fraction = enterfrac;
		pTrace->endpos.mad( pTrace->startpos, ray.m_Delta, enterfrac );

		// Need to transform the plane into world space...
		cplane_t temp;
		temp.normal = pPlaneNormal[hitplane];
        temp.normal.normalize2();
		temp.dist = ppPlaneDist[hitplane][hitside];
		if (hitside == 0)
		{
			temp.normal.mul( -1.0f );
			temp.dist *= -1.0f;
		}
		temp.type = 3;

	    matOBBToWorld.transform_dir( pTrace->plane.normal, temp.normal );
	    pTrace->plane.dist = temp.dist * pTrace->plane.normal.dotproduct( pTrace->plane.normal );
		pTrace->plane.dist += pTrace->plane.normal.x * matOBBToWorld.c[0] + pTrace->plane.normal.y * matOBBToWorld.c[1] + pTrace->plane.normal.z * matOBBToWorld.c[2];
		return true;
	}

	if ( pTrace->startsolid )
	{
		pTrace->fraction = 0;
		pTrace->endpos = pTrace->startpos;
		pTrace->plane.dist = pTrace->startpos[0];
		pTrace->plane.normal.set( 1.0f, 0.0f, 0.0f );
		pTrace->plane.type = 0;
		return true;
	}

	return false;
}


//--------------------------------------------------------------------------
// Purpose:
//
// NOTE:
//	triangle points are given in clockwise order (aabb-triangle test)
//
//    1				edge0 = 1 - 0
//    | \           edge1 = 2 - 1
//    |  \          edge2 = 0 - 2
//    |   \			.
//    |    \		.
//    0-----2		.
//
//--------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Purpose: find the minima and maxima of the 3 given values
//-----------------------------------------------------------------------------
inline void FindMinMax( float v1, float v2, float v3, float &min, float &max )
{
	min = max = v1;
	if ( v2 < min ) { min = v2; }
	if ( v2 > max ) { max = v2; }
	if ( v3 < min ) { min = v3; }
	if ( v3 > max ) { max = v3; }
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
inline bool AxisTestEdgeCrossX2( float flEdgeZ, float flEdgeY, float flAbsEdgeZ, float flAbsEdgeY,
	const Fvector &p1, const Fvector &p3, const Fvector &vecExtents,
	float flTolerance )
{
	// Cross Product( axialX(1,0,0) x edge ): x = 0.0f, y = edge.z, z = -edge.y
	// Triangle Point Distances: dist(x) = normal.y * pt(x).y + normal.z * pt(x).z
	float flDist1 = flEdgeZ * p1.y - flEdgeY * p1.z;
	float flDist3 = flEdgeZ * p3.y - flEdgeY * p3.z;

	// Extents are symmetric: dist = abs( normal.y ) * extents.y + abs( normal.z ) * extents.z
	float flDistBox = flAbsEdgeZ * vecExtents.y + flAbsEdgeY * vecExtents.z;

	// Either dist1, dist3 is the closest point to the box, determine which and test of overlap with box(AABB).
	if ( flDist1 < flDist3 )
	{
		if ( ( flDist1 > ( flDistBox + flTolerance ) ) || ( flDist3 < -( flDistBox + flTolerance ) ) )
			return false;
	}
	else
	{
		if ( ( flDist3 > ( flDistBox + flTolerance ) ) || ( flDist1 < -( flDistBox + flTolerance ) ) )
			return false;
	}

	return true;
}

//--------------------------------------------------------------------------
// Purpose:
//--------------------------------------------------------------------------
inline bool AxisTestEdgeCrossX3( float flEdgeZ, float flEdgeY, float flAbsEdgeZ, float flAbsEdgeY,
	const Fvector &p1, const Fvector &p2, const Fvector &vecExtents,
	float flTolerance )
{
	// Cross Product( axialX(1,0,0) x edge ): x = 0.0f, y = edge.z, z = -edge.y 
	// Triangle Point Distances: dist(x) = normal.y * pt(x).y + normal.z * pt(x).z
	float flDist1 = flEdgeZ * p1.y - flEdgeY * p1.z;
	float flDist2 = flEdgeZ * p2.y - flEdgeY * p2.z;

	// Extents are symmetric: dist = abs( normal.y ) * extents.y + abs( normal.z ) * extents.z
	float flDistBox = flAbsEdgeZ * vecExtents.y + flAbsEdgeY * vecExtents.z;

	// Either dist1, dist2 is the closest point to the box, determine which and test of overlap with box(AABB).
	if ( flDist1 < flDist2 )
	{
		if ( ( flDist1 > ( flDistBox + flTolerance ) ) || ( flDist2 < -( flDistBox + flTolerance ) ) )
			return false;
	}
	else
	{
		if ( ( flDist2 > ( flDistBox + flTolerance ) ) || ( flDist1 < -( flDistBox + flTolerance ) ) )
			return false;
	}

	return true;
}

//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
inline bool AxisTestEdgeCrossY2( float flEdgeZ, float flEdgeX, float flAbsEdgeZ, float flAbsEdgeX,
	const Fvector &p1, const Fvector &p3, const Fvector &vecExtents,
	float flTolerance )
{
	// Cross Product( axialY(0,1,0) x edge ): x = -edge.z, y = 0.0f, z = edge.x
	// Triangle Point Distances: dist(x) = normal.x * pt(x).x + normal.z * pt(x).z
	float flDist1 = -flEdgeZ * p1.x + flEdgeX * p1.z;
	float flDist3 = -flEdgeZ * p3.x + flEdgeX * p3.z;

	// Extents are symmetric: dist = abs( normal.x ) * extents.x + abs( normal.z ) * extents.z
	float flDistBox = flAbsEdgeZ * vecExtents.x + flAbsEdgeX * vecExtents.z;

	// Either dist1, dist3 is the closest point to the box, determine which and test of overlap with box(AABB).
	if ( flDist1 < flDist3 )
	{
		if ( ( flDist1 > ( flDistBox + flTolerance ) ) || ( flDist3 < -( flDistBox + flTolerance ) ) )
			return false;
	}
	else
	{
		if ( ( flDist3 > ( flDistBox + flTolerance ) ) || ( flDist1 < -( flDistBox + flTolerance ) ) )
			return false;
	}

	return true;
}

//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
inline bool AxisTestEdgeCrossY3( float flEdgeZ, float flEdgeX, float flAbsEdgeZ, float flAbsEdgeX,
	const Fvector &p1, const Fvector &p2, const Fvector &vecExtents,
	float flTolerance )
{
	// Cross Product( axialY(0,1,0) x edge ): x = -edge.z, y = 0.0f, z = edge.x
	// Triangle Point Distances: dist(x) = normal.x * pt(x).x + normal.z * pt(x).z
	float flDist1 = -flEdgeZ * p1.x + flEdgeX * p1.z;
	float flDist2 = -flEdgeZ * p2.x + flEdgeX * p2.z;

	// Extents are symmetric: dist = abs( normal.x ) * extents.x + abs( normal.z ) * extents.z
	float flDistBox = flAbsEdgeZ * vecExtents.x + flAbsEdgeX * vecExtents.z;

	// Either dist1, dist2 is the closest point to the box, determine which and test of overlap with box(AABB).
	if ( flDist1 < flDist2 )
	{
		if ( ( flDist1 > ( flDistBox + flTolerance ) ) || ( flDist2 < -( flDistBox + flTolerance ) ) )
			return false;
	}
	else
	{
		if ( ( flDist2 > ( flDistBox + flTolerance ) ) || ( flDist1 < -( flDistBox + flTolerance ) ) )
			return false;
	}

	return true;
}

//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
inline bool AxisTestEdgeCrossZ1( float flEdgeY, float flEdgeX, float flAbsEdgeY, float flAbsEdgeX,
	const Fvector &p2, const Fvector &p3, const Fvector &vecExtents,
	float flTolerance )
{
	// Cross Product( axialZ(0,0,1) x edge ): x = edge.y, y = -edge.x, z = 0.0f
	// Triangle Point Distances: dist(x) = normal.x * pt(x).x + normal.y * pt(x).y
	float flDist2 = flEdgeY * p2.x - flEdgeX * p2.y;
	float flDist3 = flEdgeY * p3.x - flEdgeX * p3.y;

	// Extents are symmetric: dist = abs( normal.x ) * extents.x + abs( normal.y ) * extents.y
	float flDistBox = flAbsEdgeY * vecExtents.x + flAbsEdgeX * vecExtents.y; 

	// Either dist2, dist3 is the closest point to the box, determine which and test of overlap with box(AABB).
	if ( flDist3 < flDist2 )
	{
		if ( ( flDist3 > ( flDistBox + flTolerance ) ) || ( flDist2 < -( flDistBox + flTolerance ) ) ) 
			return false;
	}
	else
	{
		if ( ( flDist2 > ( flDistBox + flTolerance ) ) || ( flDist3 < -( flDistBox + flTolerance ) ) )
			return false;
	}

	return true;
}

//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
inline bool AxisTestEdgeCrossZ2( float flEdgeY, float flEdgeX, float flAbsEdgeY, float flAbsEdgeX,
	const Fvector &p1, const Fvector &p3, const Fvector &vecExtents,
	float flTolerance )
{
	// Cross Product( axialZ(0,0,1) x edge ): x = edge.y, y = -edge.x, z = 0.0f
	// Triangle Point Distances: dist(x) = normal.x * pt(x).x + normal.y * pt(x).y
	float flDist1 = flEdgeY * p1.x - flEdgeX * p1.y;
	float flDist3 = flEdgeY * p3.x - flEdgeX * p3.y;

	// Extents are symmetric: dist = abs( normal.x ) * extents.x + abs( normal.y ) * extents.y
	float flDistBox = flAbsEdgeY * vecExtents.x + flAbsEdgeX * vecExtents.y; 

	// Either dist1, dist3 is the closest point to the box, determine which and test of overlap with box(AABB).
	if ( flDist1 < flDist3 )
	{
		if ( ( flDist1 > ( flDistBox + flTolerance ) ) || ( flDist3 < -( flDistBox + flTolerance ) ) ) 
			return false;
	}
	else
	{
		if ( ( flDist3 > ( flDistBox + flTolerance ) ) || ( flDist1 < -( flDistBox + flTolerance ) ) )
			return false;
	}

	return true;
}

//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
static int BoxOnPlaneSide( const Fvector& emins, const Fvector& emaxs, const cplane_t *p )
{
	float	dist1, dist2;
	int		sides;

	// fast axial cases
	if ( p->type < 3 )
	{
		if ( p->dist <= emins[p->type] )
			return 1;
		if ( p->dist >= emaxs[p->type] )
			return 2;
		return 3;
	}

	// general case
	switch ( p->signbits )
	{
	case 0:
		dist1 = p->normal[0] * emaxs[0] + p->normal[1] * emaxs[1] + p->normal[2] * emaxs[2];
		dist2 = p->normal[0] * emins[0] + p->normal[1] * emins[1] + p->normal[2] * emins[2];
		break;
	case 1:
		dist1 = p->normal[0] * emins[0] + p->normal[1] * emaxs[1] + p->normal[2] * emaxs[2];
		dist2 = p->normal[0] * emaxs[0] + p->normal[1] * emins[1] + p->normal[2] * emins[2];
		break;
	case 2:
		dist1 = p->normal[0] * emaxs[0] + p->normal[1] * emins[1] + p->normal[2] * emaxs[2];
		dist2 = p->normal[0] * emins[0] + p->normal[1] * emaxs[1] + p->normal[2] * emins[2];
		break;
	case 3:
		dist1 = p->normal[0] * emins[0] + p->normal[1] * emins[1] + p->normal[2] * emaxs[2];
		dist2 = p->normal[0] * emaxs[0] + p->normal[1] * emaxs[1] + p->normal[2] * emins[2];
		break;
	case 4:
		dist1 = p->normal[0] * emaxs[0] + p->normal[1] * emaxs[1] + p->normal[2] * emins[2];
		dist2 = p->normal[0] * emins[0] + p->normal[1] * emins[1] + p->normal[2] * emaxs[2];
		break;
	case 5:
		dist1 = p->normal[0] * emins[0] + p->normal[1] * emaxs[1] + p->normal[2] * emins[2];
		dist2 = p->normal[0] * emaxs[0] + p->normal[1] * emins[1] + p->normal[2] * emaxs[2];
		break;
	case 6:
		dist1 = p->normal[0] * emaxs[0] + p->normal[1] * emins[1] + p->normal[2] * emins[2];
		dist2 = p->normal[0] * emins[0] + p->normal[1] * emaxs[1] + p->normal[2] * emaxs[2];
		break;
	case 7:
		dist1 = p->normal[0] * emins[0] + p->normal[1] * emins[1] + p->normal[2] * emins[2];
		dist2 = p->normal[0] * emaxs[0] + p->normal[1] * emaxs[1] + p->normal[2] * emaxs[2];
		break;
	default:
		dist1 = dist2 = 0;		// shut up compiler
		VERIFY( 0 );
		break;
	}

	sides = 0;
	if ( dist1 >= p->dist )
		sides = 1;
	if ( dist2 < p->dist )
		sides |= 2;

	VERIFY( sides != 0 );

	return sides;
}

//-----------------------------------------------------------------------------
// Purpose: Test for an intersection (overlap) between an axial-aligned bounding 
//          box (AABB) and a triangle.
//
// Using the "Separating-Axis Theorem" to test for intersections between
// a triangle and an axial-aligned bounding box (AABB).
// 1. 3 Axis Planes - x, y, z
// 2. 9 Edge Planes Tests - the 3 edges of the triangle crossed with all 3 axial 
//                          planes (x, y, z)
// 3. 1 Face Plane - the triangle plane (cplane_t plane below)
// Output: false = separating axis (no intersection)
//         true = intersection
//-----------------------------------------------------------------------------
bool IsBoxIntersectingTriangle( const Fvector &vecBoxCenter, const Fvector &vecBoxExtents,
	const Fvector &v1, const Fvector &v2, const Fvector &v3,
	const cplane_t &plane, float flTolerance )
{
	// Test the axial planes (x,y,z) against the min, max of the triangle.
	float flMin, flMax;
	Fvector p1, p2, p3;

	// x plane
	p1.x = v1.x - vecBoxCenter.x;
	p2.x = v2.x - vecBoxCenter.x;
	p3.x = v3.x - vecBoxCenter.x;
	FindMinMax( p1.x, p2.x, p3.x, flMin, flMax );
	if ( ( flMin > ( vecBoxExtents.x + flTolerance ) ) || ( flMax < -( vecBoxExtents.x + flTolerance ) ) ) 
		return false; 

	// y plane
	p1.y = v1.y - vecBoxCenter.y;
	p2.y = v2.y - vecBoxCenter.y;
	p3.y = v3.y - vecBoxCenter.y;
	FindMinMax( p1.y, p2.y, p3.y, flMin, flMax );
	if ( ( flMin > ( vecBoxExtents.y + flTolerance ) ) || ( flMax < -( vecBoxExtents.y + flTolerance ) ) ) 
		return false; 

	// z plane
	p1.z = v1.z - vecBoxCenter.z;
	p2.z = v2.z - vecBoxCenter.z;
	p3.z = v3.z - vecBoxCenter.z;
	FindMinMax( p1.z, p2.z, p3.z, flMin, flMax );
	if ( ( flMin > ( vecBoxExtents.z + flTolerance ) ) || ( flMax < -( vecBoxExtents.z + flTolerance ) ) ) 
		return false; 

	// Test the 9 edge cases.
	Fvector vecEdge, vecAbsEdge;

	// edge 0 (cross x,y,z)
	vecEdge.sub( p2, p1 );
	vecAbsEdge.y = _abs( vecEdge.y );
	vecAbsEdge.z = _abs( vecEdge.z );
	if ( !AxisTestEdgeCrossX2( vecEdge.z, vecEdge.y, vecAbsEdge.z, vecAbsEdge.y, p1, p3, vecBoxExtents, flTolerance ) ) 
		return false; 

	vecAbsEdge.x = _abs( vecEdge.x );
	if ( !AxisTestEdgeCrossY2( vecEdge.z, vecEdge.x, vecAbsEdge.z, vecAbsEdge.x, p1, p3, vecBoxExtents, flTolerance ) ) 
		return false; 

	if ( !AxisTestEdgeCrossZ1( vecEdge.y, vecEdge.x, vecAbsEdge.y, vecAbsEdge.x, p2, p3, vecBoxExtents, flTolerance ) ) 
		return false; 

	// edge 1 (cross x,y,z)
	vecEdge.sub( p3, p2 );
	vecAbsEdge.y = _abs( vecEdge.y );
	vecAbsEdge.z = _abs( vecEdge.z );
	if ( !AxisTestEdgeCrossX2( vecEdge.z, vecEdge.y, vecAbsEdge.z, vecAbsEdge.y, p1, p2, vecBoxExtents, flTolerance ) ) 
		return false; 

	vecAbsEdge.x = _abs( vecEdge.x );
	if ( !AxisTestEdgeCrossY2( vecEdge.z, vecEdge.x, vecAbsEdge.z, vecAbsEdge.x, p1, p2, vecBoxExtents, flTolerance ) ) 
		return false; 

	if ( !AxisTestEdgeCrossZ2( vecEdge.y, vecEdge.x, vecAbsEdge.y, vecAbsEdge.x, p1, p3, vecBoxExtents, flTolerance ) ) 
		return false; 

	// edge 2 (cross x,y,z)
	vecEdge.sub( p1, p3 );
	vecAbsEdge.y = _abs( vecEdge.y );
	vecAbsEdge.z = _abs( vecEdge.z );
	if ( !AxisTestEdgeCrossX3( vecEdge.z, vecEdge.y, vecAbsEdge.z, vecAbsEdge.y, p1, p2, vecBoxExtents, flTolerance ) ) 
		return false; 

	vecAbsEdge.x = _abs( vecEdge.x );
	if ( !AxisTestEdgeCrossY3( vecEdge.z, vecEdge.x, vecAbsEdge.z, vecAbsEdge.x, p1, p2, vecBoxExtents, flTolerance ) ) 
		return false; 

	if ( !AxisTestEdgeCrossZ1( vecEdge.y, vecEdge.x, vecAbsEdge.y, vecAbsEdge.x, p2, p3, vecBoxExtents, flTolerance ) ) 
		return false; 

	// Test against the triangle face plane.
	Fvector vecMin, vecMax;
	vecMin.sub( vecBoxCenter, vecBoxExtents );
	vecMax.add( vecBoxCenter, vecBoxExtents );
	if ( BoxOnPlaneSide( vecMin, vecMax, &plane ) != 3 ) 
		return false; 

	return true;
}
