/*
Spdx-License-Identifier: MIT
SPDX-FileCopyrightText: 2005-2020 Dominik Haumann <dhaumann@kde.org>
*/
#ifdef WIN32
#pragma warning(disable:4530)
#define WIN32_CONSOLE
#endif

#include <vector>
#include <iostream>

#include "pointcloud.h"
#include "point.h"

// Derive a point class from kdtree::Point.
// - Pass the x/y/z position via constructor.
// - You can freely add additional member data, for example setVariance() and variance().
class MyPoint : public kdtree::Point
{
public:
	constexpr MyPoint(float x, float y, float z) noexcept
		: kdtree::Point(x, y, z)
	{
	}

	void setVariance(float variance) {
		m_variance = variance;
	}

	float variance() const {
		return m_variance;
	}

private:
	float m_variance = 0.0f;
};

int main( int argc, char** argv )
{
	// Create a PointClound of type MyPoint
	kdtree::PointCloud<MyPoint> pointCloud;

	// add points via PointCloud::setItems()
	std::vector<MyPoint> punkte;
	for (int a = 0; a < 10; ++a) {
		for (int b = 0; b < 10; ++b) {
			for (int c = 0; c < 10; ++c) {
				punkte.push_back(MyPoint(a, b, c));
			}
		}
	}
	pointCloud.setItems(punkte);

	// alternatively, add points via addItems()
	punkte.clear();
	for (int a = 0; a < 10; ++a) {
		for (int b = 0; b < 10; ++b) {
			for (int c = 0; c < 10; ++c) {
				punkte.push_back(MyPoint(-a, -b, -c));
			}
		}
	}
	pointCloud.addItems(punkte);

	// once data set is complete, rebuild the kdtree structure once.
	// Only afterwards, it is safe to call findInRadius() and findKNearest()
	pointCloud.rebuildTree();

	// vector that contains results.
	std::vector<MyPoint> result;

	// findInRadius
	float p[] = { 0.0, 0.0, 0.0 };
	float squareRadius = 2*2;
	pointCloud.findInRadius(p, squareRadius, result);
	std::cout << "found " << result.size() << " items in radius." << std::endl;

	// find 10 closest points around p
	pointCloud.findKNearest(p, 10, result);
	std::cout << "found " << result.size() << " nearest items." << std::endl;

	return 0;
}

// kate: indent-width 4; tab-width 4; replace-tabs off;
