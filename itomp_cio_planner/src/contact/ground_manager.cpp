/*
 * groundManager.cpp
 *
 *  Created on: Sep 11, 2013
 *      Author: chpark
 */
#include <itomp_cio_planner/contact/ground_manager.h>
#include <itomp_cio_planner/util/planning_parameters.h>

namespace itomp_cio_planner
{

GroundManager GroundManager::instance_;

GroundManager::GroundManager()
{
}

GroundManager::~GroundManager()
{
}

void GroundManager::init()
{
}

double interpolateSqrt(double x, double x1, double x2, double y1, double y2)
{
	//double y = (y2 - y1) * sqrt((x - x1) / (x2 - x1)) + y1;
	double y = (y2 - y1) * (x - x1) / (x2 - x1) + y1;
	return y;
}

void GroundManager::getNearestGroundPosition(const KDL::Vector& in, KDL::Vector& out, KDL::Vector& normal, bool exact) const
{
	const double FOOT_FRONT = 0.2;
	const double FOOT_REAR = 0.2;//0.05;
	const double MARGIN = 0.1;

	normal = KDL::Vector(0, 0, 1);

	double height = 0.0;

	if (PlanningParameters::getInstance()->getTemporaryVariable(2) == 1.0)
	{
		const int NUM_TERRAINS = 7;

		double x[] =
		{ -50.0, -1.0, -0.6, -0.2, 0.2, 0.6, 1.0, 50.0 };
		double z[] =
		{ 0.0, 0.15, 0.3, 0.45, 0.3, 0.15, 0.0, 0.0 };

		for (int i = 0; i < NUM_TERRAINS; ++i)
		{
			if (in.x() >= x[i] && in.x() <= x[i + 1])
			{
				height = z[i];

				if (exact)
					break;

				if (i != 0 && z[i - 1] > z[i])
				{
					if (in.x() - x[i] <= FOOT_REAR)
						height = z[i - 1];
					else if (in.x() - x[i] <= FOOT_REAR + MARGIN)
					{
						height = interpolateSqrt(in.x(), x[i] + FOOT_REAR, x[i] + FOOT_REAR + MARGIN, z[i - 1], z[i]);
					}
				}
				else if (z[i + 1] > z[i])
				{
					if (x[i + 1] - in.x() <= FOOT_FRONT)
						height = z[i + 1];
					else if (x[i + 1] - in.x() <= FOOT_FRONT + MARGIN)
					{
						height = interpolateSqrt(in.x(), x[i + 1] - FOOT_FRONT - MARGIN, x[i + 1] - FOOT_FRONT, z[i],
								z[i + 1]);
					}
				}
			}
		}
	}
	else if (PlanningParameters::getInstance()->getTemporaryVariable(2) == 2.0)
	{
		if (-1.60 - FOOT_FRONT - MARGIN < in.x() && in.x() < -1.60 - FOOT_FRONT)
		{
			if (!exact)
				height = interpolateSqrt(in.x(), -1.60 - FOOT_FRONT - MARGIN, -1.60 - FOOT_FRONT, 0, 0.352);
		}
		else if (-1.60 - FOOT_FRONT < in.x() && in.x() < -1.50 + FOOT_REAR)
		{
			height = 0.352;
		}
		else if (-1.50 + FOOT_REAR < in.x() && in.x() < -1.50 + FOOT_REAR + MARGIN)
		{
			if (!exact)
				height = interpolateSqrt(in.x(), -1.50 + FOOT_REAR, -1.50 + FOOT_REAR + MARGIN, 0.352, 0);
		}
	}
	else if (PlanningParameters::getInstance()->getTemporaryVariable(2) == 3.0)
	{
		height = 0.0;
	}

	// WAFR Personal Space
	if (PlanningParameters::getInstance()->getTemporaryVariable(2) == 11.0)
	{
		const int NUM_TERRAINS = 7;

		double y[] =
		{ -50.0, -5.75, -3.75, -1.75, 1.75, 3.75, 5.75, 50.0 };
		//{ -50.0, -5.65, -3.65, -1.65, 1.65, 3.65, 5.65, 50.0 };
		double z[] =
		{ 0.0, 0.15, 0.3, 0.45, 0.3, 0.15, 0.0, 0.0 };

		for (int i = 0; i < NUM_TERRAINS; ++i)
		{
			if (in.y() >= y[i] && in.y() <= y[i + 1])
			{
				height = z[i];

				//if (exact)
					//break;

				if (i != 0 && z[i - 1] > z[i])
				{
					if (in.y() - y[i] <= FOOT_REAR)
						height = z[i - 1];
					else if (in.y() - y[i] <= FOOT_REAR + MARGIN)
					{
						height = interpolateSqrt(in.y(), y[i] + FOOT_REAR, y[i] + FOOT_REAR + MARGIN, z[i - 1], z[i]);
					}
				}
				else if (z[i + 1] > z[i])
				{
					if (y[i + 1] - in.y() <= FOOT_FRONT)
						height = z[i + 1];
					else if (y[i + 1] - in.y() <= FOOT_FRONT + MARGIN)
					{
						height = interpolateSqrt(in.y(), y[i + 1] - FOOT_FRONT - MARGIN, y[i + 1] - FOOT_FRONT, z[i],
								z[i + 1]);
					}
				}
			}
		}
	}

	out = KDL::Vector(in.x(), in.y(), height);
}

void GroundManager::getSafeGroundPosition(const KDL::Vector& in, KDL::Vector& out) const
{
	const double FOOT_FRONT = 0.2;
	const double FOOT_REAR = 0.05;
	const double MARGIN = 0.1;

	double safeX = in.x();
	double height = 0.0;
	if (PlanningParameters::getInstance()->getTemporaryVariable(2) == 1.0)
	{
		const int NUM_TERRAINS = 7;

		double x[] =
		{ -50.0, -1.0, -0.6, -0.2, 0.2, 0.6, 1.0, 50.0 };
		double z[] =
		{ 0.0, 0.15, 0.3, 0.45, 0.3, 0.15, 0.0, 0.0 };

		for (int i = 0; i < NUM_TERRAINS; ++i)
		{
			if (in.x() >= x[i] && in.x() <= x[i + 1])
			{
				height = z[i];

				if (z[i] < z[i + 1] && x[i + 1] - FOOT_FRONT - MARGIN < in.x())
					safeX = x[i + 1] - FOOT_FRONT - MARGIN;
				if (i != 0 && z[i - 1] < z[i] && in.x() < x[i] + FOOT_REAR)
				{
					safeX = x[i] - FOOT_FRONT - MARGIN;
					height = z[i - 1];
				}

				if (z[i] > z[i + 1] && x[i + 1] - FOOT_FRONT < in.x())
					safeX = x[i + 1] - FOOT_FRONT;
				if (i != 0 && z[i - 1] > z[i] && in.x() < x[i] + FOOT_REAR + MARGIN)
				{
					safeX = x[i] - FOOT_FRONT;
					height = z[i - 1];
				}
			}
		}
	}
	else if (PlanningParameters::getInstance()->getTemporaryVariable(2) == 2.0)
	{
		if (-1.60 - FOOT_FRONT - MARGIN < in.x() && in.x() < -1.50 + FOOT_REAR + MARGIN)
		{
			safeX = -1.60 - FOOT_FRONT - MARGIN;
		}
	}
	else if (PlanningParameters::getInstance()->getTemporaryVariable(2) == 3.0)
	{
		height = 0.0;
	}

	out = KDL::Vector(safeX, in.y(), height);
}

}

