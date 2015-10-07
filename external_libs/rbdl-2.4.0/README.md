RBDL - Rigid Body Dynamics Library
Copyright (c) 2011-2015 Martin Felis <martin.felis@iwr.uni-heidelberg.de>

Introduction
============

RBDL is a highly efficient C++ library that contains some essential rigid
body dynamics algorithms such as the Articulated Body Algorithm (ABA) for
forward dynamics, Recursive Newton-Euler Algorithm (RNEA) for inverse
dynamics and the Composite Rigid Body Algorithm (CRBA) for the efficient
computation of the joint space inertia matrix. It further contains code for
Jacobians, forward and inverse kinematics, and handling of external
constraints such as contacts and collisions.

The code is developed by Martin Felis <martin.felis@iwr.uni-heidelberg.de>
at the research group [Optimization in Robotics and Biomechanics
(ORB)](http://orb.iwr.uni-heidelberg.de) of the [Interdisciplinary Center
for Scientific Computing (IWR)](http://www.iwr.uni-heidelberg.de) at
[Heidelberg University](http://www.uni-heidelberg.de). The code tightly
follows the notation used in Roy Featherstone''s book "Rigid Body Dynamics
Algorithm".

Recent Changes
==============
  * 23 February 2015: New version 2.4.0:
    * Added sparse range-space method ForwardDynamicsContactsRangeSpaceSparse()
      and ComputeContactImpulsesRangeSpaceSparse() 
    * Added null-space method ForwardDynamicsContactsNullSpace()
      and ComputeContactImpulsesNullSpace() 
    * Renamed ForwardDynamicsContactsLagrangian() to
      ForwardDynamicsContactsDirect() and
      ComputeContactImpulsesLagrangian() to ComputeContactImpulsesDirect()
    * Renamed ForwardDynamicsContacts() to ForwardDynamicsContactsKokkevis()
    * Removed/Fixed CalcAngularMomentum(). The function produced wrong values. The
      functionality has been integrated into CalcCenterOfMass().
    * CalcPointJacobian() does not clear the argument of the result anymore.
      Caller has to ensure that the matrix was set to zero before using this
      function.
    * Added optional workspace parameters for ForwardDynamicsLagrangian() to
      optionally reduce memory allocations
    * Added JointTypeTranslationXYZ, JointTypeEulerXYZ, and JointTypeEulerYXZ
      which are equivalent to the emulated multidof joints but faster.
    * Added optional parameter to CalcCenterOfMass to compute angular momentum.
    * Added CalcBodySpatialJacobian()
    * Added CalcContactJacobian()
    * Added NonlinearEffects()
    * Added solving of linear systems using standard Householder QR
    * LuaModel: Added LuaModelReadFromLuaState()
    * URDFReader: Fixed various issues and using faster joints for floating
      base models
    * Various performance improvements
  * 21 October 2014: New version 2.3.3:
    * critical: fixed ForwardDynamicsContacts with constraints on a body
      that is attached with a fixed joint. Previous versions simply crashed.
      Thanks to Yue Hu for reporting!
    * rbdl_print_version() now properly prints whether URDFReader was enabled
      at build time
    * build system: fixed roblems especially building of the URDFreader
    * build system: all CMake variables for RBDL are now prefixed with RBDL_ 
    * FindRBDL.cmake now can use components to search for the LuaModel or
      URDFReader addon
  * 29 August 2014: New version 2.3.2:
     * critical: fixed ForwardDynamicsLagrangian which used uninitialized values for the joint space inertia matrix (thanks to Benjamin Michaud!)
     * critical: fixed ForwardDynamicsContacts when using 3-dof joints
     * critical: fixed CalcBodyWorldOrientation for fixed joints (thanks to Hilaro Tome!)
     * critical: fixed CompositeRigidBodyDynamics when using 3-dof joints (thanks to Henning Koch!)
   * 13 July 2014: New version 2.3.1:
     * critical: fixed angular momentum computation. Version 2.3.0 produced wrong
       results. (Thanks to Hilario Tome and Benjamin Michaud for reporting!)
     * critical: fixed JointTypeEulerZYX. Previous versions produce wrong results!
     * fixed library version number for the LuaModel addon.
   * 14 March 2014: New version 2.3.0:
     * Joint Space Inertia Matrix does not get cleared anymore when calling CompositeRigidBodyAlgorithm
     * using the default column-major ordering when using Eigen3
     * added experimental joint type JointTypeEulerZYX
     * added energy computations Utils::CalcCenterOfMass, Utils::CalcPotentialEnergy, Utils::CalcKineticEnergy, and Utils::CalcAngularMomentum.
     * Updated URDF loader for ROS Groovy/Hydro (thanks to Benjamin Chrétien!)
   * 06 November 2013: New version 2.2.2: adjusted Body default constructor (inertia matrix now 3x3 identity instead of zero matrix)
   * 04 November 2013: New version 2.2.1: fixed exported library version
   * 28 October 2013: New version 2.2.0: added support for spherical joints that do not suffer from joint singularities
   * 29 September 2013: New version 2.1.0: adjusted build settings and symbol export to be debian compatible. Removed vendor code such as Lua 5.2 and UnitTest++. Must be pre-installed if tests or LuaModel Addon is enabled.
   * 05 September 2013: New version 2.0.1: fixed some errors on older compilers and CMake configuration of examples. No changes required when migrating from 2.0.0.
   * 18 July 2013: API version 2.0.0: removed Eigen3 sources, removed Model::Init(), inverted sign of contact forces/impulses
   * 20 February 2013: removed too specialized RigidBodyDynamics::Body constructor (API version 1.1.0)
   * 29 January 2013: added code for api_version_checking. Current API version is 1.0.0.
   * 11 January 2013: removed Eigen3 sources and relying on an already installed Eigen3 library. Optionally RBDL can be used with the included but slower SimpleMath library.
   * 18 June 2012: added support of luamodel_introduction
   * 01 June 2012: added support of joint_models_fixed
   * 14 May 2012: fixed Body constructor as reported by Maxime Reis
   * 04 April 2012: added benchmark tool for CRBA
   * 01 March 2012: added multi degree of freedom joint_models
   * 06 Februry 2012: restructured constraint handling using RigidBodyDynamics::ConstraintSet
   * 24 January 2012: implemented compact and fast representation of RigidBodyDynamics::Math::SpatialTransform 

Documentation
=============

The documentation is contained in the code and can be extracted with the
tool [doxygen](http://www.doxygen.org).

To create the documentation simply run

    doxygen Doxyfile

which will generate the documentation in the subdirectory ./doc/html. The
main page will then be located in ./doc/html/index.html.

An online version of the generated documentation can be found at
[http://rbdl.bitbucket.org](http://rbdl.bitbucket.org).

Getting RBDL
============

The latest stable code can be obtained from

    https://bitbucket.org/rbdl/rbdl/get/default.zip

The official mercurial repository can be cloned from

    https://bitbucket.org/rbdl/rbdl

(See [http://mercurial.selenic.com/](http://mercurial.selenic.com) for
mercurial clients.)

Building and Installation
=========================

The RBDL is built using CMake
([http://www.cmake.org](http://www.cmake.org)). To compile the library in
a separate directory in Release mode use:

    mkdir build
    cd build/
    cmake -D CMAKE_BUILD_TYPE=Release ../ 
    make

For optimal performance it is highly recommended to install the Eigen3
linear algebra library from
[http://eigen.tuxfamily.org](http://eigen.tuxfamily.org). RBDL also
comes with a simple, albeit much slower math library (SimpleMath) that can
be used by enabling `RBDL_USE_SIMPLE_MATH`, i.e.:

    cmake -D RBDL_USE_SIMPLE_MATH=TRUE ../

Licensing
=========

The library is published under the very permissive zlib free software
license which should allow you to use the software wherever you need. 

This is the full license text (zlib license):

    RBDL - Rigid Body Dynamics Library
    Copyright (c) 2011-2015 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
    
    This software is provided 'as-is', without any express or implied
    warranty. In no event will the authors be held liable for any damages
    arising from the use of this software.
    
    Permission is granted to anyone to use this software for any purpose,
    including commercial applications, and to alter it and redistribute it
    freely, subject to the following restrictions:
    
       1. The origin of this software must not be misrepresented; you must not
       claim that you wrote the original software. If you use this software
       in a product, an acknowledgment in the product documentation would be
       appreciated but is not required.
    
       2. Altered source versions must be plainly marked as such, and must not
       be misrepresented as being the original software.
    
       3. This notice may not be removed or altered from any source
       distribution.

Acknowledgements
================

Work on this library was funded by the [Heidelberg Graduate School of
Mathematical and Computational Methods for the Sciences
(HGS)](http://hgs.iwr.uni-heidelberg.de/hgs.mathcomp/), and the European
FP7 projects [ECHORD](http://echord.eu) (grant number 231143) and
[Koroibot](http://koroibot.eu) (grant number 611909).
