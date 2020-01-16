/*
Spdx-License-Identifier: MIT
SPDX-FileCopyrightText: 2005-2020 Dominik Haumann <dhaumann@kde.org>
*/
#ifndef KDTREE_BOUNDINGBOX_H
#define KDTREE_BOUNDINGBOX_H

#ifdef WIN32
#pragma warning(disable:4530)
#endif

#include "point.h"
#include <vector>
#include <cstdint> // uint64_t

namespace kdtree
{

template <class T> class Node;

/**
 * The class @p BoundingBox stores the size of a space partition.
 * It is only used by @p KDTree and thus @p KDTree is a @c friend.
 */
template <class T>
class BoundingBox
{
	friend class Node<T>;
public:
	/// standard constructor
	constexpr BoundingBox() noexcept = default;

	/**
	 * constructor that calculates its size by using the min/max
	 * values of the @p points array. Interval: [begin; end)
	 * @param points vector containing the point data
	 * @param begin start bound
	 * @param end end bound
	 */
	BoundingBox(const std::vector<T>& points, uint64_t begin, uint64_t end);
	~BoundingBox();

	/**
	 * crop the bounding box to a minimal size around the points. This way,
	 * the average distance is bigger so that we can discard more BBs!
	 * @param points points in the bounding box
	 * @param begin start bound
	 * @param end end bound
	 */
	void crop(const std::vector<T>& points, uint64_t begin, uint64_t end);

	/**
	 * get the longest axis of the quadric bounding box space
	 * @return 0 for x direction, 1 for y, 2 for z. The return value
	 *         is used for indexing float[3] variables.
	 */
	int getSplitAxis() const;

	/**
	 * get the distance of one point to this bounding box
	 * @param x the point (float array with 3 entries)
	 * @return float
	 */
	float distance2(const float* x) const;

private:
	float p[3];	///< smallest point-coordinates in all three dimensions
	float q[3];	///< biggest point-coordinates in all three dimensions
};


//
//
// TEMPLATE IMPLEMENTATION
//
//

template <class T>
BoundingBox<T>::BoundingBox(const std::vector<T>& points, uint64_t begin, uint64_t end)
{
	crop(points, begin, end);
}

template <class T>
BoundingBox<T>::~BoundingBox()
{
}

template <class T>
void BoundingBox<T>::crop(const std::vector<T>& points, uint64_t begin, uint64_t end)
{
	p[0] = points[begin].p[0];
	p[1] = points[begin].p[1];
	p[2] = points[begin].p[2];

	q[0] = p[0];
	q[1] = p[1];
	q[2] = p[2];

	for (uint64_t i = begin+1; i < end; ++i)
	{
		// find smallest value along each axis
		if (points[i].p[0] < p[0]) p[0] = points[i].p[0];
		if (points[i].p[1] < p[1]) p[1] = points[i].p[1];
		if (points[i].p[2] < p[2]) p[2] = points[i].p[2];

		// find largest value along each axis
		if (points[i].p[0] > q[0]) q[0] = points[i].p[0];
		if (points[i].p[1] > q[1]) q[1] = points[i].p[1];
		if (points[i].p[2] > q[2]) q[2] = points[i].p[2];
	}
}

template <class T>
int BoundingBox<T>::getSplitAxis() const
{
	// NOTE: We do not need fabs here, which is faster.
	//       See also ::crop() where p gets minimal, and q maximal
//	const float temp1 = fabs(q[0] - p[0]);
//	const float temp2 = fabs(q[1] - p[1]);
//	const float temp3 = fabs(q[2] - p[2]);
	const float temp1 = q[0] - p[0];
	const float temp2 = q[1] - p[1];
	const float temp3 = q[2] - p[2];

	if(temp1 > temp2 && temp1 > temp3)
		return 0;
	else
		return 1 + (temp3 > temp2);
}

template <class T>
float BoundingBox<T>::distance2(const float* x) const
{
/*
//  ____________________________ q
//  |                          |
//  |                          |
//  |                          |
//  |                          |
//  |                          |-------------- x3
//  |                          |
//  |                          |
//  |                          |
//  |                          |
// p----------------------------
//            |                 \
//			  |                  \
//			  |                   \
//			  x1                   x2
*/
	float t0, t1, t2;

	if (x[0] <= p[0])		t0 = p[0] - x[0];
	else if (x[0] >= q[0])	t0 = x[0] - q[0];
	else					t0 = 0.0f;

	if (x[1] <= p[1])		t1 = p[1] - x[1];
	else if (x[1] >= q[1])	t1 = x[1] - q[1];
	else					t1 = 0.0f;

	if (x[2] <= p[2])		t2 = p[2] - x[2];
	else if (x[2] >= q[2])	t2 = x[2] - q[2];
	else					t2 = 0.0f;

	return t0*t0 + t1*t1 + t2*t2;
}

}

#endif // KDTREE_BOUNDINGBOX_H

// kate: indent-width 4; tab-width 4; replace-tabs off;
