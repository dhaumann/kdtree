/*
Spdx-License-Identifier: MIT
SPDX-FileCopyrightText: 2005-2020 Dominik Haumann <dhaumann@kde.org>
*/
#ifndef KDTREE_POINTCLOUD_H
#define KDTREE_POINTCLOUD_H

#ifdef WIN32
#pragma warning(disable:4530)
#endif

#include <string>
#include <vector>

#include "point.h"
#include "node.h"

#include <algorithm>

namespace kdtree
{

/**
 * The class @p PointCloud represents a cloud of point data.
 */
template <class T>
class PointCloud
{
public:
	constexpr PointCloud() = default;
	virtual ~PointCloud();

	/**
	 * Find the @p k nearest points to given reference point @p p. The result
	 * will be stored in the vector @p result.
	 * @param p reference point
	 * @param k amount of points to find
	 * @param result returned vector containing the points
	 * @return true on success, false if you forgot to call rebuildTree().
	 */
	bool findKNearest(const float* p, unsigned int k, std::vector<T>& result);

	/**
	 * Find all points in the sphere with center @p m and @p radius. The result
	 * will be stored in the vector @p result.
	 * @param m center of sphere
	 * @param radius2 square radius of sphere
	 * @param result returned vector containing the points
	 * @return true on success, false if you forgot to call rebuildTree().
	 */
	bool findInRadius(const float* m, float radius2, std::vector<T>& result);

	/**
	 * Create the KdTree structure of the current point cloud data.
	 * @note Call this function once you are done with adding cloud data, i.e.,
	 *       before calling findKNearest() and findInRadius().
	 */
	void rebuildTree();

	/**
	 * Clear all items, the PointCloud does not contain any data afterwards.
	 */
	void clear();

	/**
	 * Set all data points. Before the new data is set, the old data is removed.
	 * @note Call rebuildTree() afterwards.
	 */
	void setItems(const std::vector <T>& items);

	/**
	 * Append data points to the already existing point cloud.
	 * @note Call rebuildTree() afterwards.
	 */
	void addItems(const std::vector <T>& items);

	/**
	 * Append a signel item to the already point cloud.
	 * @note Call rebuildTree() afterwards.
	 */
	void addItem(const T& item);

	/**
	 * Get the list of all points as const reference.
	 */
	const std::vector <T>& points() const;

private:
	std::vector <T> m_points;
	kdtree::Node<T>* m_kdtree = nullptr;
};


//
//
// TEMPLATE IMPLEMENTATION
//
//

template <class T>
PointCloud<T>::~PointCloud()
{
	delete m_kdtree;
	m_kdtree = 0;
}

template <class T>
void PointCloud<T>::rebuildTree()
{
	if (m_kdtree) {
		delete m_kdtree;
	}

	m_kdtree = new kdtree::Node<T>(m_points, 0, m_points.size());
}

template <class T>
bool PointCloud<T>::findKNearest(const float* p, unsigned int k, std::vector<T>& result)
{
	result.clear();
	
	if (!m_kdtree) {
		return false;
	}

	if (k >= m_points.size())
	{
		result.assign(m_points.begin(), m_points.end());
		std::nth_element(result.begin(),
						 result.end(),
						 result.end(),
						 T::smaller_dist);
	}
	else if (k > 0)
	{
		kdtree::Node<T>::dist = 100000000.0f;
		m_kdtree->findKNearest(p, k, result);
	}
	
	return true;
}

template <class T>
bool PointCloud<T>::findInRadius(const float* m, float radius2, std::vector<T>& result)
{
	if (!m_kdtree) {
		return false;
	}

	result.clear();
	m_kdtree->findInRadius(m, radius2, result);
	return true;
}

template <class T>
void PointCloud<T>::clear()
{
	delete m_kdtree;
	m_kdtree = 0;
	
	m_points.clear();
}

template <class T>
void PointCloud<T>::setItems(const std::vector <T>& items)
{
	clear();
	m_points = items;
}

template <class T>
void PointCloud<T>::addItems(const std::vector <T>& items)
{
	delete m_kdtree;
	m_kdtree = 0;

	m_points.insert(m_points.end(), items.begin(), items.end());
}

template <class T>
void PointCloud<T>::addItem(const T& item)
{
	delete m_kdtree;
	m_kdtree = 0;

	m_points.push_back(item);
}

template <class T>
const std::vector <T>& PointCloud<T>::points() const
{
	return m_points;
}

}

#endif // KDTREE_POINTCLOUD_H

// kate: indent-width 4; tab-width 4; replace-tabs off;
