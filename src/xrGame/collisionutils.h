//========= Copyright Valve Corporation, All rights reserved. ============//
//
// Purpose: Common collision utility methods
//
// $Header: $
// $NoKeywords: $
//=============================================================================//

#ifndef COLLISIONUTILS_H
#define COLLISIONUTILS_H


//-----------------------------------------------------------------------------
//
// IntersectRayWithTriangle
//
// Intersects a ray with a triangle, returns distance t along ray.
// t will be less than zero if no intersection occurred
// oneSided will cull collisions which approach the triangle from the back
// side, assuming the vertices are specified in counter-clockwise order
// The vertices need not be specified in that order if oneSided is not used
//
//-----------------------------------------------------------------------------
float IntersectRayWithTriangle( const Fvector &rayStart, const Fvector &rayDelta,
	const Fvector& v1, const Fvector& v2, const Fvector& v3, bool oneSided );


//-----------------------------------------------------------------------------
// IntersectRayWithBox
//
// Purpose: Computes the intersection of a ray with a box (AABB)
// Output : Returns true if there is an intersection + trace information
//-----------------------------------------------------------------------------
bool IntersectRayWithBox( const Fvector &rayStart, const Fvector &rayDelta, const Fvector &boxMins, const Fvector &boxMaxs, 
	float epsilon, trace_t *pTrace );

bool IntersectRayWithBox( const Ray_t &ray, const Fvector &boxMins, const Fvector &boxMaxs, 
	float epsilon, trace_t *pTrace );


//-----------------------------------------------------------------------------
// Intersects a ray against a box
//-----------------------------------------------------------------------------
struct BoxTraceInfo_t
{
	float t1;
	float t2;
	int	hitside;
	bool startsolid;
};

bool IntersectRayWithBox( const Fvector &vecRayStart, const Fvector &vecRayDelta,
	const Fvector &boxMins, const Fvector &boxMaxs, float flTolerance, BoxTraceInfo_t *pTrace );


//-----------------------------------------------------------------------------
// IntersectRayWithOBB
//
// Purpose: Computes the intersection of a ray with a oriented box (OBB)
// Output : Returns true if there is an intersection + trace information
//-----------------------------------------------------------------------------
bool IntersectRayWithOBB( const Fvector &vecRayStart, const Fvector &vecRayDelta,
	const Fmatrix &matOBBToWorld, const Fvector &vecOBBMins, const Fvector &vecOBBMaxs,
    float flTolerance, trace_t *pTrace );

bool IntersectRayWithOBB( const Ray_t &ray, const Fmatrix &matOBBToWorld,
	const Fvector &vecOBBMins, const Fvector &vecOBBMaxs, float flTolerance, trace_t *pTrace );


//-----------------------------------------------------------------------------
// 
// IsBoxIntersectingSphere
//
// returns true if there's an intersection between box and sphere
//
//-----------------------------------------------------------------------------
bool IsBoxIntersectingSphere( const Fvector& boxMin, const Fvector& boxMax,
	const Fvector& center, float radius );


//-----------------------------------------------------------------------------
// returns true if there's an intersection between ray and sphere
//-----------------------------------------------------------------------------
bool IsRayIntersectingSphere( const Fvector &vecRayOrigin, const Fvector &vecRayDelta,
	const Fvector &vecSphereCenter, float flRadius, float flTolerance = 0.0f );


//-----------------------------------------------------------------------------
// 
// IsBoxIntersectingBox
//
// returns true if there's an intersection between two boxes
//
//-----------------------------------------------------------------------------
bool IsBoxIntersectingBox( const Fvector& boxMin1, const Fvector& boxMax1,
	const Fvector& boxMin2, const Fvector& boxMax2 );


//-----------------------------------------------------------------------------
// 
// IsBoxIntersectingRay
//
// returns true if there's an intersection between box and ray
//
//-----------------------------------------------------------------------------
bool IsBoxIntersectingRay( const Fvector& boxMin, const Fvector& boxMax,
	const Fvector& origin, const Fvector& delta,
	const Fvector& invDelta, float flTolerance = 0.0f );


//-----------------------------------------------------------------------------
// 
// IsPointInBox
//
// returns true if the point is in the box
//
//-----------------------------------------------------------------------------
bool IsPointInBox( const Fvector& pt, const Fvector& boxMin, const Fvector& boxMax );


//-----------------------------------------------------------------------------
// IsBoxIntersectingTriangle
//
// Test for an intersection (overlap) between an axial-aligned bounding 
// box (AABB) and a triangle.
//
// Triangle points are in counter-clockwise order with the normal facing "out."
//
// Using the "Separating-Axis Theorem" to test for intersections between
// a triangle and an axial-aligned bounding box (AABB).
// 1. 3 Axis Plane Tests - x, y, z
// 2. 9 Edge Planes Tests - the 3 edges of the triangle crossed with all 3 axial 
//                          planes (x, y, z)
// 3. 1 Face Plane Test - the plane the triangle resides in (cplane_t plane)
//-----------------------------------------------------------------------------
bool IsBoxIntersectingTriangle( const Fvector &vecBoxCenter, const Fvector &vecBoxExtents,
	const Fvector &v1, const Fvector &v2, const Fvector &v3,
	const cplane_t &plane, float flTolerance );


#endif // COLLISIONUTILS_H
