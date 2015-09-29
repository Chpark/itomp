/*
 * point_to_triangle_projection.cpp
 *
 *  Created on: May 21, 2014
 *      Author: cheonhyeonpark
 */

#include <itomp_cio_planner/util/point_to_triangle_projection.h>

namespace itomp_cio_planner
{
// TODO INDENTATION
//triangle / point distance
// source http://www.geometrictools.com/Documentation/DistancePoint3Triangle3.pdf
Eigen::Vector3d ProjPoint2Triangle(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1,
								   const Eigen::Vector3d &p2, const Eigen::Vector3d &source_position)
{
	Eigen::Vector3d edge0 = p1 - p0;
	Eigen::Vector3d edge1 = p2 - p0;
	Eigen::Vector3d v0 = p0 - source_position;

	double a = edge0.dot(edge0);
	double b = edge0.dot(edge1);
	double c = edge1.dot(edge1);
	double d = edge0.dot(v0);
	double e = edge1.dot(v0);

	double det = a*c - b*b;
	double s = b*e - c*d;
	double t = b*d - a*e;

	double lower_bound = 0.0, upper_bound = 1.0;


	if ( s + t < det )
	{
		if ( s < 0.0 )
		{
			if ( t < 0.0 )
			{
				if ( d < 0.0 )
				{
					s = std::min(std::max(-d/a, lower_bound), upper_bound);
					t = 0.0;
				}
				else
				{
					s = 0.0;
					t = std::min(std::max(-e/c, lower_bound), upper_bound);
				}
			}
			else
			{
				s = 0.0;
				t = std::min(std::max(-e/c, lower_bound), upper_bound);
			}
		}
		else if ( t < 0.0 )
		{
			s = std::min(std::max(-d/a, lower_bound), upper_bound);
			t = 0.0;
		}
		else
		{
			float invDet = 1.0 / det;
			s *= invDet;
			t *= invDet;
		}
	}
	else
	{
		if ( s < 0.0 )
		{
			double tmp0 = b+d;
			double tmp1 = c+e;
			if ( tmp1 > tmp0 )
			{
				double numer = tmp1 - tmp0;
				double denom = a-2*b+c;
				s = std::min(std::max(numer/denom, lower_bound), upper_bound);
				t = 1-s;
			}
			else
			{
				t = std::min(std::max(-e/c, lower_bound), upper_bound);
				s = 0.f;
			}
		}
		else if ( t < 0.f )
		{
			if ( a+d > b+e )
			{
				double numer = c+e-b-d;
				double denom = a-2*b+c;
				s = std::min(std::max(numer/denom, lower_bound),upper_bound);
				t = 1-s;
			}
			else
			{
				s = std::min(std::max(-e/c, lower_bound), upper_bound);
				t = 0.f;
			}
		}
		else
		{
			double numer = c+e-b-d;
			double denom = a-2*b+c;
			s = std::min(std::max(numer/denom, lower_bound), upper_bound);
			t = 1.0 - s;
		}
	}
	return p0 + s * edge0 + t * edge1;
}

} // namespace itomp_cio_planner
