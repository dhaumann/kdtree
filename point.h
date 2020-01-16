/*
Spdx-License-Identifier: MIT
SPDX-FileCopyrightText: 2005-2020 Dominik Haumann <dhaumann@kde.org>
*/
#ifndef KDTREE_POINT_H
#define KDTREE_POINT_H

namespace kdtree
{

/**
 * The class @p Point represents a point in 3D space.
 */
class Point
{
public:
	constexpr Point(float x, float y, float z) noexcept
		: p{x, y, z}
		, dist(0)
	{
	}

	/**
	 * Square distance.
	 */
	float distance2(const float* x)
	{
		// avoid loop
		dist = (x[0] - p[0]) * (x[0] - p[0]) + (x[1] - p[1]) * (x[1] - p[1]) + (x[2] - p[2]) * (x[2] - p[2]);
		return dist;
	}

	float p[3]; ///< point coordinates
	float dist; ///< (cached) distance from point to another
	
	/**
	 * define comparison for 2-Norm^2. This uses the cached dist values!
	 */
	static inline bool smaller_dist(const Point& x, const Point& y)
	{
		return x.dist < y.dist;
	}
};

}

#endif // KDTREE_POINT_H

// kate: indent-width 4; tab-width 4; replace-tabs off;
