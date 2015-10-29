#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/util/jacobian.h>
#include <itomp_cio_planner/model/rbdl_model_util.h>
#include "dlib/optimization.h"
#include <itomp_cio_planner/optimization/phase_manager.h>

itomp_cio_planner::NewEvalManager* Jacobian::evaluation_manager_ = NULL;

Jacobian::Jacobian()
{
	ComputeJacobian();
}

Jacobian::~Jacobian()
{
	//NOTHING
}

void Jacobian::SetJacobian(const Eigen::MatrixXd& jacobian)
{
	jacobian_ = jacobian;
	Invalidate();
}

void Jacobian::Invalidate()
{
	computeInverse_ = true;
	computeProduct_ = true;
	computeProductInverse_ = true;
	computeJacSVD_ = true;
	computeNullSpace_ = true;
}

namespace
{
/*
    Eigen::Vector3d ComputeRotationAxis(Node* node, Node* root)
    {
        Node* y = node->parent;
        Eigen::Vector3d w_ = node->axis;							// Initialize to local rotation axis
        while (y) {
            Rotate(y->axis, w_, y->value);
            //if(y->id == root->id) break;
            y = y->parent;
        }
        return w_;
    }
    */
}

void Jacobian::ComputeJacobian()
{
	/*
	Eigen::Matrix4d toRootCoordinates = Eigen::Matrix4d::Identity();
	toRootCoordinates.block<3,3>(0,0) = root->toLocalRotation;
	toRootCoordinates.block<3,1>(0,3) = -root->position;
	Eigen::Matrix4d toWorldCoordinates = toRootCoordinates;
	toWorldCoordinates.inverse();
	Invalidate();
	int dim = planner::GetNumChildren(root);
	std::vector<Node*> effectors = planner::GetEffectors(root, true);
	jacobian_ = Eigen::MatrixXd(3 * effectors.size(), dim);
	// Traverse this to find all end effectors
	for(int i=0; i!=effectors.size(); ++i)
	{
	    Node* nodeEffector = effectors[i];
	    Node* currentNode = nodeEffector;
	    Eigen::Vector3d effectorPos = nodeEffector->position;
	    do
	    {
	        currentNode = currentNode->parent;
	        Eigen::Vector3d siMinuspj = effectorPos -
	                currentNode->position; //);
	        Eigen::Vector3d vj = ComputeRotationAxis(currentNode, root);//root->toLocalRotation * currentNode->axis; // ComputeRotationAxis(currentNode); //currentNode->toWorldRotation * currentNode->axis;
	        jacobian_.block<3,1>(3*i,currentNode->id - root->id) = vj.cross(siMinuspj);
	    }
	    while(currentNode->id != root->id);
	}
	*/
}

void Jacobian::GetEllipsoidAxes(Eigen::Vector3d &u1, Eigen::Vector3d &u2, Eigen::Vector3d &u3)
{
	GetJacobianProduct();
	svdProduct_ = Eigen::JacobiSVD<Eigen::MatrixXd>(jacobianProduct_, Eigen::ComputeThinU | Eigen::ComputeThinV);
	u1 = svdProduct_.matrixU().block(0,0,3,1) / (svdProduct_.singularValues()(0) + 0.000000000000000000000000001);
	u2 = svdProduct_.matrixU().block(0,1,3,1) / (svdProduct_.singularValues()(1) + 0.000000000000000000000000001);
	u3 = svdProduct_.matrixU().block(0,2,3,1) / (svdProduct_.singularValues()(2) + 0.000000000000000000000000001);
	// TODO
}

void Jacobian::GetEllipsoidAxes(Eigen::Vector3d& u1, Eigen::Vector3d& u2, Eigen::Vector3d& u3, double& sig1, double& sig2, double& sig3)
{
	GetJacobianProduct();
	svdProduct_ = Eigen::JacobiSVD<Eigen::MatrixXd>(jacobianProduct_, Eigen::ComputeThinU | Eigen::ComputeThinV);
	u1 = svdProduct_.matrixU().block(0,0,3,1);
	u2 = svdProduct_.matrixU().block(0,1,3,1);
	u3 = svdProduct_.matrixU().block(0,2,3,1);
	sig1 = 1. / (svdProduct_.singularValues()(0) + 0.000000000000000000000000001);
	sig2 = 1. / (svdProduct_.singularValues()(1) + 0.000000000000000000000000001);
	sig3 = 1. / (svdProduct_.singularValues()(2) + 0.000000000000000000000000001);
}

void Jacobian::ComputeAll()
{
	ComputeSVD();
	GetJacobianInverse();
	GetJacobianProduct();
	GetJacobianProductInverse();
	GetNullspace();
}

const Eigen::MatrixXd& Jacobian::GetJacobian()
{
	return jacobian_;
}

const Eigen::MatrixXd& Jacobian::GetJacobianInverse()
{
	if(computeInverse_)
	{
		computeInverse_ = false;
		jacobianInverse_ = jacobian_;
		PseudoInverseDLS(jacobianInverse_, 1.f); // tmp while figuring out how to chose lambda
	}
	return jacobianInverse_;
}

const Eigen::MatrixXd& Jacobian::GetNullspace()
{
	if(computeNullSpace_)
	{
		computeNullSpace_ = false;
		Eigen::MatrixXd id = Eigen::MatrixXd::Identity(jacobian_.cols(), jacobian_.cols());
		ComputeSVD();
		Eigen::MatrixXd res = Eigen::MatrixXd::Zero(id.rows(), id.cols());
		for(int i =0; i < svd_.matrixV().cols(); ++ i)
		{
			Eigen::VectorXd v = svd_.matrixV().col(i);
			res += v * v.transpose();
		}
		Identitymin_ = id - res;
	}
	return Identitymin_;
}

void Jacobian::GetNullspace(const Eigen::MatrixXd pseudoId, Eigen::MatrixXd& result)
{
	GetNullspace(); // computing inverse jacobian

	Eigen::MatrixXd id = Eigen::MatrixXd::Identity(Identitymin_.rows(), Identitymin_.cols());
	result = pseudoId - (id + Identitymin_);
}

const Eigen::MatrixXd& Jacobian::GetJacobianProduct()
{
	if(computeProduct_)
	{
		computeProduct_ = false;
		jacobianProduct_ = jacobian_ * jacobian_.transpose();
	}
	return jacobianProduct_;
}

const Eigen::MatrixXd &Jacobian::GetJacobianProductInverse()
{
	if(computeProductInverse_)
	{
		computeProductInverse_ = false;
		Eigen::JacobiSVD<Eigen::MatrixXd> svd = Eigen::JacobiSVD<Eigen::MatrixXd>(jacobianProduct_, Eigen::ComputeFullU | Eigen::ComputeFullV);
		PseudoInverseSVDDLS(jacobianProduct_, svd, jacobianProductInverse_);
	}
	return jacobianProductInverse_;
}

void Jacobian::ComputeSVD()
{
	if(computeJacSVD_)
	{
		computeJacSVD_ = false;
        svd_ = Eigen::JacobiSVD<Eigen::MatrixXd>(jacobian_, Eigen::ComputeThinV);
	}
}

void Jacobian::GetProjection(int point, const Eigen::VectorXd& q, Eigen::VectorXd& a)
{
	Jacobian j;

    RigidBodyDynamics::Model model = evaluation_manager_->getRBDLModel(point);
    std::vector<unsigned int> body_ids;
    for (int i = 0; i < evaluation_manager_->getPlanningGroup()->getNumContacts(); ++i)
    {
        if (itomp_cio_planner::PhaseManager::getInstance()->getPhase() > 2)
            continue;

        if (itomp_cio_planner::PhaseManager::getInstance()->getPhase() > 0)
        {
            if (!evaluation_manager_->getPlanningGroup()->is_fixed_[i])
                continue;
        }

        int rbdl_body_id = evaluation_manager_->getPlanningGroup()->contact_points_[i].getRBDLBodyId();
        body_ids.push_back(rbdl_body_id);
    }

    if (body_ids.size() == 0)
        return;
    Eigen::MatrixXd jacobianMerged = Eigen::MatrixXd::Zero(6 * body_ids.size(), model.qdot_size);
    UpdateKinematicsCustom (model, &q, NULL, NULL);

    for (unsigned int k = 0; k < body_ids.size(); k++)
    {
        Eigen::MatrixXd G (Eigen::MatrixXd::Zero(6, model.qdot_size));
        itomp_cio_planner::CalcPointJacobian6D(model, q, body_ids[k], Eigen::Vector3d::Zero(), G, evaluation_manager_->getPlanningGroup()->rbdl_to_group_joint_, false);

        for (unsigned int j = 0; j < model.qdot_size; j++)
        {
            for (unsigned int i = 0; i < 6; i++)
            {
                unsigned int row = k * 6 + i;
                jacobianMerged(row, j) = G(i,j);
            }
        }
    }

    j.SetJacobian(jacobianMerged);
    a = j.GetNullspace() * a;
}

void Jacobian::scale(dlib::matrix<double, 0, 1>& s)
{
    // normalize der;
    double max_der = 0.1;
    double max_der2 = 1.0;
    itomp_cio_planner::ItompTrajectoryIndex max_index;
    for (int i = 0; i < s.size(); ++i)
    {
        const itomp_cio_planner::ItompTrajectoryIndex& index = evaluation_manager_->getTrajectory()->getTrajectoryIndex(i);
        if (index.sub_component == itomp_cio_planner::ItompTrajectory::SUB_COMPONENT_TYPE_JOINT)
        {
            if (std::abs(s(i)) > max_der)
            {
                max_der = std::abs(s(i));
                max_index = evaluation_manager_->getTrajectory()->getTrajectoryIndex(i);
            }
        }
        if (index.sub_component == itomp_cio_planner::ItompTrajectory::SUB_COMPONENT_TYPE_CONTACT_FORCE)
        {
            if (std::abs(s(i)) > max_der2)
            {
                max_der2 = std::abs(s(i));
            }
        }
    }
    double scale = 0.1 / max_der;
    double scale2 = 1.0 / max_der2;
    for (int i = 0; i < s.size(); ++i)
    {
        const itomp_cio_planner::ItompTrajectoryIndex& index = evaluation_manager_->getTrajectory()->getTrajectoryIndex(i);
        if (index.sub_component == itomp_cio_planner::ItompTrajectory::SUB_COMPONENT_TYPE_JOINT)
            s(i) *= scale;
        if (index.sub_component == itomp_cio_planner::ItompTrajectory::SUB_COMPONENT_TYPE_CONTACT_FORCE)
            s(i) *= scale2;
    }
}

void Jacobian::projectToNullSpace(const dlib::matrix<double, 0, 1>& x, dlib::matrix<double, 0, 1>& s)
{
    return;
    itomp_cio_planner::ItompTrajectoryPtr trajectory = evaluation_manager_->getTrajectoryNonConst();

    Eigen::VectorXd q;
    Eigen::VectorXd a;

    std::vector<unsigned int> projection_indices;
    projection_indices.push_back(0);
    if (itomp_cio_planner::PhaseManager::getInstance()->getPhase() != 0)
    {
        for (unsigned int i = 1; i < trajectory->getNumPoints() - 1; ++i)
            projection_indices.push_back(i);
    }
    projection_indices.push_back(trajectory->getNumPoints() - 1);

    for (unsigned int i = 0; i < projection_indices.size(); ++i)
    {
        unsigned int index = projection_indices[i];
        if (!trajectory->setJointPositions(q, x, index))
            continue;
        trajectory->setJointPositions(a, s, index);
        GetProjection(index, q, a);
        trajectory->getJointPositions(s, a, index);
    }
}
