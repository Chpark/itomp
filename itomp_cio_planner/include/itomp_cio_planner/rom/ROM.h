/**
* \file ROM.h
* \brief Helper class that contains the polytope describing
* the range of motion of a given limb. Comes with an evaluation function
* of the maximal ball radius that indicates how close a current configuration is from a boundary
* \author Steve T.
* \version 0.1
* \date 20/08/2014
*
*/
#ifndef _STRUCT_ROM
#define _STRUCT_ROM

#include <Eigen/Dense>

#include <exception>
#include <string>

namespace rom
{
class ROMException: public std::exception
{
public:
    ROMException(const std::string& message) throw()
		:mess_(message){}
 
     virtual const char* what() const throw()
     {
         return mess_.c_str();
     }

	 virtual ~ROMException() throw(){}
 
private:
    std::string mess_;
};


class ROM
{
public:
     ROM(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, const double maxRadius, const double minx, const double miny, const double minz,
		 const double maxx, const double maxy, const double maxz, const int axis1 ,const int axis2, const int axis3);
	 ROM(const ROM& parent);
	 ~ROM();

	/// \brief the distance between a given point and the closest boundary of the polytope.
    /// If the point is outside the polytope, returns a negative distance
	double ResidualRadius(const double x, const double y, const double z) const;

	/// \brief the distance between a given point and the closest boundary of the polytope divided by the Chebyshev radius
    /// If the point is outside the polytope, returns a negative distance
	double NormalizedResidualRadius(const double x, const double y, const double z) const;

public:	
    Eigen::MatrixXd A_;
    Eigen::MatrixXd ANorm_;
    Eigen::VectorXd b_;
    Eigen::VectorXd bNorm_;
    double maxRadius_;
    double minx_, miny_, minz_;
    double maxx_, maxy_, maxz_;
    int  axis1_, axis2_, axis3_;
private:
	Eigen::Vector3d vAxis1_, vAxis2_, vAxis3_;
};

ROM ROMFromFile(const std::string& filepath);

} //namespace rom
#endif //_STRUCT_ROM
