/*
Spdx-License-Identifier: MIT
SPDX-FileCopyrightText: 2005-2020 Dominik Haumann <dhaumann@kde.org>
*/
#ifndef KDTREE_NODE_H
#define KDTREE_NODE_H

#ifdef WIN32
#pragma warning(disable:4530)
#endif

#include <vector>
#include <algorithm>

#include "point.h"
#include "boundingbox.h"

namespace kdtree
{

template <class T> class PointCloud;

/**
 * The class @p Node arranges efficient space partitions for the
 * amount of points. The space is stored in a @p BoundingBox.
 */
template <class T> class Node
{
	friend class PointCloud<T>;
public:
	/**
	 * Constructor. Bound interval is: [begin; end) (halb-offen!!!)
	 *
	 * @param points the points
	 * @param begin start of points
	 * @param end end of points
	 */
	Node(std::vector<T>& points, uint64_t begin, uint64_t end);
	~Node();

	/**
	 * Returns, whether the node is a leaf or not. A leaf does not
	 * have further children and thus contains the data.
	 *
	 * @return true, if the node is a leaf
	 */
	inline bool isLeaf() const;

	/**
	 * find the @p k nearest points to given reference point @p p. The result
	 * will be stored in the vector @p result.
	 * @param p reference point
	 * @param k amount of points to find
	 * @param result returned vector containing the points
	 */
	void findKNearest(const float* p, const unsigned int k, std::vector<T>& result);

	/**
	 * find all points in the sphere with center @p m and @p radius. The result
	 * will be stored in the vector @p result.
	 * @param m center of sphere
	 * @param radius radius of sphere
	 * @param result returned vector containing the points
	 */
	void findInRadius(const float* m, const float radius, std::vector<T>& result);

private:
	// children
	Node<T>* left = nullptr;
	Node<T>* right = nullptr;

	// contains index of model points
	std::vector <T>& m_points;

	BoundingBox<T> box;

	uint64_t m_begin;
	uint64_t m_end;

	/**
	 * global which indicates how many points are in a Node.
	 * If there are more than @p N points the Node splits itself into
	 * two child KDTrees with half the points in each one.
	 */
	static constexpr uint64_t N = 50;

	/**
	 * global distance to the most far away point as !square! (means: 2-Norm^2)
	 */
	static float dist;
};


//
//
// TEMPLATE IMPLEMENTATION
//
//

template <class T>
float Node<T>::dist = 1000000000.0f;

/**
 * define comparator '<' needed by std::nth_element()
 */
template <class T>
class SortAxisComparator
{
	int m_sortAxis;
public:
	constexpr SortAxisComparator(int sortAxis) noexcept : m_sortAxis(sortAxis) {}
	inline bool operator()(const T& a, const T& b)
	{
		return a.p[m_sortAxis] < b.p[m_sortAxis];
	}
};

template <class T>
Node<T>::Node(std::vector<T>& points, uint64_t begin, uint64_t end)
	: m_points(points)
	, m_begin(begin)
	, m_end(end)
{
	box.crop(points, begin, end);

	// split on too many points
	if (m_end - m_begin > N)
	{
		const uint64_t median = begin + (end - begin) / 2;
		SortAxisComparator<T> lessThan(box.getSplitAxis());

		std::nth_element(points.begin() + begin,
						 points.begin() + median,
						 points.begin() + end, lessThan);

		left = new Node<T>(points, begin, median);
		right = new Node<T>(points, median, end);
	}
}

template <class T>
Node<T>::~Node()
{
	if (left)  delete left;
	if (right) delete right;
}

template <class T>
bool Node<T>::isLeaf() const
{
	return !left;
}

template <class T>
void Node<T>::findKNearest(const float* p, const unsigned int k, std::vector<T>& result)
{
	if (!isLeaf())
	{
		float tl = left->box.distance2(p);
		float tr = right->box.distance2(p);
		if (tl < dist && tl < tr)
		{
			left->findKNearest(p, k, result);
			if (tr < dist) right->findKNearest(p, k, result);
		}
		else if (tr < dist)
		{
			right->findKNearest(p, k, result);
			if (tl < dist) left->findKNearest(p, k, result);
		}
	}
	else
	{
		for (uint64_t i = m_begin; i < m_end; ++i)
		{
			if (m_points[i].distance2(p) < dist)
			{
				// add point
				if (result.size() < k-1)
				{
					result.push_back(m_points[i]);
				}
				else if (result.size() < k)
				{
					// happens exactly once
					// it is the last point that is found unsorted. Therefore, sort once.
					result.push_back(m_points[i]);
					std::sort(result.begin(), result.end(), T::smaller_dist);
					dist = result.back().dist;
				}
				else
				{
					// size == k, insert sorted, and remove last
					result.insert(
						std::upper_bound(result.begin(),
										 result.end(),
										 m_points[i],
										 T::smaller_dist),
						m_points[i]);

					// remove last point
					result.pop_back();
					dist = result.back().dist;
				}
			}
		}
	}
}

template <class T>
void Node<T>::findInRadius(const float* m, const float radius2, std::vector<T>& result)
{
	if (!isLeaf())
	{
		if (left->box.distance2(m) <= radius2)
		{
			left->findInRadius(m, radius2, result);
		}
		if (right->box.distance2(m) <= radius2)
		{
			right->findInRadius(m, radius2, result);
		}
	}
	else

	// it is a leaf
	for (unsigned int i = m_begin; i < m_end; ++i)
	{
		if (m_points[i].distance2(m) <= radius2)
			result.push_back(m_points[i]);
	}
}

}

#endif // KDTREE_NODE_H

// kate: indent-width 4; tab-width 4; replace-tabs off;
