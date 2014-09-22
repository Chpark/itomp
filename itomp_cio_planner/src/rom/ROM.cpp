#include <itomp_cio_planner/rom/ROM.h>
#include <iostream>
#include <fstream>

namespace
{
    std::string remplacerVirgule(const std::string& s)
    {
        //Remplace les ',' par des espaces.
        std::string ret="";
        for(unsigned int i=0;i<s.size();i++)
        {
            if(s[i]==',')
                ret+=' ';
            else
                ret+=s[i];
        }
        return ret;
    }

    Eigen::MatrixXd ReadMatrix(const int& size, std::ifstream& myfile)
    {
        Eigen::MatrixXd res(size, 4);
        std::string line; int i = 0;
        if (myfile.is_open())
        {
            while (myfile.good() && i< size)
            {
                getline (myfile, line);
                if(line.size()> 0)
                {
                    line = remplacerVirgule(line);
                    char z[255],x[255],y[255], b[255];
                    sscanf(line.c_str(),"%s %s %s %s",x,y,z,b);
                    res.block<1,4>(i,0) = Eigen::Vector4d((double)strtod (x, NULL), (double) strtod (y, NULL), (double) strtod (z, NULL), (double) strtod (b, NULL));
                }
                ++i;
            }
        }
        return res;
    }

    Eigen::MatrixXd NormalizedResidualRadiusA(const Eigen::MatrixXd& A)
    {
        Eigen::MatrixXd res(A.rows(), 3);
        for(int i=0; i<A.rows(); ++i)
        {
            double norm = A.block<1,3>(i,0).norm();
            res.block<1,3>(i,0) = A.block<1,3>(i,0) / norm;
        }
        return res;
    }

    Eigen::VectorXd NormalizedResidualRadiusB(const Eigen::MatrixXd& A, const Eigen::VectorXd& b)
    {
        Eigen::VectorXd res(b.rows());
        for(int i=0; i<A.rows(); ++i)
        {
            double norm = A.block<1,3>(i,0).norm();
            res(i) = b(i) / norm;
        }
        return res;
    }
}

rom::ROM::ROM(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, const double maxRadius, const double minx, const double miny, const double minz, const double maxx, const double maxy, const double maxz, const int axis1 ,const int axis2, const int axis3)
    : A_(A)
    , ANorm_(NormalizedResidualRadiusA(A))
    , b_(b)
    , bNorm_(NormalizedResidualRadiusB(A, b))
    , maxRadius_(maxRadius)
    , minx_(minx)
    , miny_(miny)
    , minz_(minz)
    , maxx_(maxx)
    , maxy_(maxy)
    , maxz_(maxz)
    , axis1_(axis1)
    , axis2_(axis2)
    , axis3_(axis3)
{
    vAxis1_ = Eigen::Vector3d((axis1_ == 0) ? 1. : 0.,
                              (axis1_ == 1) ? 1. : 0.,
                              (axis1_ == 2) ? 1. : 0.);

    vAxis2_ = Eigen::Vector3d((axis2_ == 0) ? 1. : 0.,
                              (axis2_ == 1) ? 1. : 0.,
                              (axis2_ == 2) ? 1. : 0.);

    vAxis3_ = Eigen::Vector3d((axis3_ == 0) ? 1. : 0.,
                              (axis3_ == 1) ? 1. : 0.,
                              (axis3_ == 2) ? 1. : 0.);
}

rom::ROM::ROM(const ROM& parent)
    : A_(parent.A_)
    , ANorm_(parent.ANorm_)
    , b_(parent.b_)
    , bNorm_(parent.bNorm_)
    , maxRadius_(parent.maxRadius_)
    , minx_(parent.minx_)
    , miny_(parent.miny_)
    , minz_(parent.minz_)
    , maxx_(parent.maxx_)
    , maxy_(parent.maxy_)
    , maxz_(parent.maxz_)
    , axis1_(parent.axis1_)
    , axis2_(parent.axis2_)
    , axis3_(parent.axis3_)
    , vAxis1_(parent.vAxis1_)
    , vAxis2_(parent.vAxis2_)
    , vAxis3_(parent.vAxis3_)
{
    // NOTHING
}


rom::ROM::~ROM()
{
    // NOTHING
}

double rom::ROM::ResidualRadius(const double x, const double y, const double z) const
{
    Eigen::Vector3d var = (Eigen::AngleAxisd(x, vAxis1_)
            * Eigen::AngleAxisd(y, vAxis2_)
            *  Eigen::AngleAxisd(z, vAxis3_)).matrix().eulerAngles(axis1_,axis2_,axis3_);
    double res = (bNorm_ - ANorm_ * var).minCoeff();
    return res < 0 ? -10 * res : res;
}

double rom::ROM::NormalizedResidualRadius(const double x, const double y, const double z) const
{
    return ResidualRadius(x,y,z) / maxRadius_;
}

rom::ROM rom::ROMFromFile(const std::string& filepath)
{
    Eigen::MatrixXd res;
    std::ifstream myfile (filepath.c_str());
    std::string line;
    double maxRadius_;
    int size_;
    double minx, miny, minz;
    double maxx, maxy, maxz;
    int axe1, axe2 , axe3;
    if (myfile.is_open())
    {
        if(myfile.good())
        {
            char r[255];
            getline (myfile, line);
            sscanf(line.c_str(),"%s",r);
            maxRadius_ = (double) (strtod (r, NULL));
        }
        else
        {
            std::string errmess("In file: " + filepath + "; file ended before finding radius");
            throw(new ROMException(errmess));
        }
        if(myfile.good())
        {
            char s[255];
            getline (myfile, line);
            sscanf(line.c_str(),"%s",s);
            size_ = (int) (strtod (s, NULL));

            getline (myfile, line);
            if(line.size()> 0)
            {
                line = remplacerVirgule(line);
                char caxe1[255],caxe2[255],caxe3[255];
                sscanf(line.c_str(),"%s %s %s",caxe1, caxe2, caxe3);
                axe1 = (int) strtod (caxe1, NULL);
                axe2 = (int) strtod (caxe2, NULL);
                axe3 = (int) strtod (caxe3, NULL);
            }
            getline (myfile, line);
            if(line.size()> 0)
            {
                line = remplacerVirgule(line);
                char cminx[255],cminy[255],cminz[255];
                char cmaxx[255],cmaxy[255],cmaxz[255];
                sscanf(line.c_str(),"%s %s %s %s %s %s",cminx, cmaxx, cminy, cmaxy, cminz, cmaxz);
                minx = (double) strtod (cminx, NULL); miny = (double) strtod (cminy, NULL); minz = (double) strtod (cminz, NULL);
                maxx = (double) strtod (cmaxx, NULL); maxy = (double) strtod (cmaxy, NULL); maxz = (double) strtod (cmaxz, NULL);
            }
        }
        res = ReadMatrix(size_, myfile);
        myfile.close();
    }
    else
    {
        std::string errmess("file not found: " + filepath);
        throw(new ROMException(errmess));
    }
    Eigen::MatrixXd A_ = res.block(0,0,size_,3);
    Eigen::VectorXd b_ = res.block(0,3,size_,1);
    return ROM(A_,b_,maxRadius_,minx, miny, minz, maxx, maxy, maxz, axe1, axe2, axe3);
}
