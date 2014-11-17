#include "PostureSelection.h"
#include "tools/MatrixDefs.h"
#include "ik/IKSolver.h"
#include "ik/VectorAlignmentConstraint.h"
#include "equilibrium/DynamicStability.h"

#include <iostream>

using namespace planner;
using namespace sampling;


namespace //equilibirum stuff
{



bool Stable(planner::State* state)
{
    equilib::T_Transform contacts;
    equilib::T_Transform graps;
    std::vector<Eigen::Vector3d>::iterator posit = state->contactLimbPositions.begin();
    std::vector<Eigen::Vector3d>::iterator normit = state->contactLimbPositionsNormals.begin();
    for(std::vector<int>::const_iterator cit = state->contactLimbs.begin();
        cit != state->contactLimbs.end(); ++cit, ++posit, ++normit)
    {
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        transform.block(0,0,3,3) = matrices::RotationMatrixFromNormal(*normit);
        transform.block(0,3,3,1) = *posit;
        if(true)//((*it)->contactType_ == ::Contact)
        {
            contacts.push_back(transform);
        }
        else
        {
            graps.push_back(transform);
        }
        if(contacts.empty() && graps.empty()) return false;
    }
    matrices::Vector3 acceleration(0,0,0);
    matrices::Vector3 comLocation = state->value->currentPosition;
    matrices::VectorX maxGraspForces(3*graps.size());
    for(int i=0; i<graps.size()*3; ++i)
    {
        maxGraspForces(i)=600;
    }
    return equilib::CheckEquilibrium(contacts, graps, maxGraspForces, acceleration, comLocation, 50, 1, 600);
}

/*
float Robot::StabilityMargin() const
{
    equilib::T_Transform contacts;
    equilib::T_Transform graps;
    for(T_TreeCIT it = pImpl_->trees_.begin(); it != pImpl_->trees_.end(); ++it)
    {
        if((*it)->IsLocked())
        {
            const Obstacle* obs = (*it)->GetObstacleTarget();
            if(obs != 0)
            {
                // get target world coordinates
                matrices::Matrix4 transform = obs->Basis();
                transform.block<3,1>(0,3) = (*it)->GetTarget();
                if((*it)->contactType_ == ::Contact)
                {
                    contacts.push_back(transform);
                }
                else
                {
                    graps.push_back(transform);
                }
            }
        }
    }
    if(contacts.size() + graps.size() <= 0) return false;
    matrices::Vector3 acceleration(0,0,0);
    matrices::Vector3 zero(0,0,0);
    matrices::Vector3 comLocation = matrices::matrix4TimesVect3(this->ToWorldCoordinates(), zero);
    matrices::VectorX maxGraspForces(3*graps.size());
    for(int i=0; i<graps.size()*3; ++i)
    {
        maxGraspForces(i)=600;
    }
    return (float)(equilib::ResidualRadius(contacts, graps, maxGraspForces, acceleration, comLocation, 50, 1, 600));
}

float Robot::StabilityMargin(int id, const matrices::Vector3 position, const matrices::Matrix4& obstacleTransform) const
{
    equilib::T_Transform contacts;
    equilib::T_Transform graps;
    int i = 0;
    for(T_TreeCIT it = pImpl_->trees_.begin(); it != pImpl_->trees_.end(); ++it, ++i)
    {
        if((*it)->IsLocked() && (*it)->GetObstacleTarget() || i == id)
        {
            matrices::Matrix4 transform = (i == id) ? obstacleTransform : (*it)->GetObstacleTarget()->Basis();
            // get target world coordinates
            const matrices::Vector3& wtf = (i == id) ? position : (*it)->GetTarget();
            transform.block<3,1>(0,3) = wtf;
            if((*it)->contactType_ == ::Contact)
            {
                contacts.push_back(transform);
            }
            else
            {
                graps.push_back(transform);
            }
        }
    }
    if(contacts.size() + graps.size() <= 0) return false;
    matrices::Vector3 acceleration(0,0,0);
    matrices::Vector3 zero(0,0,0);
    matrices::Vector3 comLocation = matrices::matrix4TimesVect3(this->ToWorldCoordinates(), zero);
    matrices::VectorX maxGraspForces(3*graps.size());
    for(int i=0; i<graps.size()*3; ++i)
    {
        maxGraspForces(i)=600;
    }
    return (float)(equilib::ResidualRadius(contacts, graps, maxGraspForces, acceleration, comLocation, 50, 1, 600));
}*/
}

namespace
{
    Object* GetEffector(Node* limb)
    {
        if(limb->children.size() != 0)
        {
            Object* res = GetEffector(limb->children[0]);
            if(res) return res;
        }
        return limb->current;
    }

    bool LimbColliding(Node* limb, planner::Object::T_Object& obstacles)
    {
        if(limb->current && limb->current->IsColliding(obstacles))
        {
                return true;
        }
        if(limb->children.size() == 0)
            return false;
        return LimbColliding(limb->children[0], obstacles);
    }

    void GetEffectorsRec(Node* limb, std::vector<Eigen::Vector3d>& res)
    {
        if(limb->children.size() != 0)
        {
            for(std::vector<Node*>::iterator cit = limb->children.begin();
                cit != limb->children.end(); ++cit)
            {
                GetEffectorsRec(*cit,res);
            }
        }
        else
        {
            res.push_back(limb->position);
        }
    }

    std::vector<Eigen::Vector3d> GetEffectorsRec(Node* limb)
    {
        std::vector<Eigen::Vector3d> res;
        GetEffectorsRec(limb,res);
        return res;
    }

    bool InContact(Object::T_Object& effectors, Object* obj, const double epsilon, Eigen::Vector3d&  normal, Eigen::Vector3d&  proj)
    {
        for(Object::T_Object::iterator it = effectors.begin();
            it != effectors.end(); ++it)
        {
            if(!(*it)->InContact(obj,epsilon, normal, proj))
            {
                return false;
            }
        }
        return true;
    }

    const double epsilon = 0.01;
}

Sample* planner::GetPosturesInContact(Robot& robot, Node* limb, const sampling::T_Samples& samples
                                         , Object::T_Object& obstacles, const Eigen::Vector3d& direction, Eigen::Vector3d& position, Eigen::Vector3d& normalVector)
{
    Sample* save = new Sample(limb);
    Sample* res = 0;
    Object* effector = GetEffector(limb);
    //std::vector<Eigen::Vector3d> effectorPos = GetEffectorsRec(limb);
    //Eigen::Vector3d effectorCentroid = planner::GetEffectorCenter(limb);
    double bestManip = std::numeric_limits<double>::min();
    double tmp_manip, tempweightedmanip;
    Eigen::Vector3d dir = direction;
    std::size_t found = limb->tag.find("arm");
    if (found!=std::string::npos)
    {
        dir = Eigen::Vector3d(-0.1,0,0.9);
        dir.normalize();
    }
    for(T_Samples::const_iterator sit = samples.begin(); sit != samples.end(); ++sit)
    {
        tmp_manip = planner::sampling::Manipulability(*sit, dir);
        if(tmp_manip > bestManip)
        {
            Eigen::Vector3d normal, projection;
            LoadSample(*(*sit),limb);
            for(Object::T_Object::iterator oit = obstacles.begin(); oit != obstacles.end(); ++oit)
            {
                if(effector->InContact(*oit,epsilon, normal, projection) && !planner::IsSelfColliding(&robot, limb) && !LimbColliding(limb, obstacles))
                //if(planner::MinDistance(effectorCentroid, *oit, projection, normal) < epsilon && !planner::IsSelfColliding(&robot, limb) && !LimbColliding(limb, obstacles))
                {
                    tempweightedmanip = tmp_manip * dir.dot(normal);
                    if(tempweightedmanip > bestManip)
                    {
                        bestManip = tempweightedmanip;
                        res = *sit;
                        //position = effector->GetPosition();
                        normalVector = normal;
                        position = projection;
                        break;
                    }
                }
            }
        }
    }
    /*So we have our sample. Time to perform some IK to align pose*/
    if(res)
    {
        LoadSample(*res,limb);
        ik::VectorAlignmentConstraint constraint(normalVector);
        std::vector<ik::PartialDerivativeConstraint*> constraints;
        constraints.push_back(&constraint);
        ik::IKSolver solver;
        //solver.AddConstraint(ik::ForceManip);
        {
            int limit = 10;
            //int limit2 = 100;
            while(limit > 0)
            {
                solver.StepClamping(limb, position, position, constraints, true);
                //solver.StepClamping(limb, position, position, constraints, true);
                limit--;
            }
        }
    }
    else
    {
        planner::sampling::LoadSample(*save, limb);
    }
    return res;
}

Sample* planner::GetPosturesInContact(Robot& robot, Node* limb, const sampling::T_Samples& samples
                                         , Object::T_Object& obstacles, const Eigen::Vector3d& direction)
{
    Eigen::Vector3d dummmy, dummmy2;
    return GetPosturesInContact(robot, limb, samples, obstacles, direction, dummmy, dummmy2);
}

sampling::T_Samples planner::GetContactCandidates(Robot& robot, Node* limb, const sampling::T_Samples& samples
                                         , Object::T_Object& obstacles, const Eigen::Vector3d& direction)
{
    Sample* save = new Sample(limb);
    sampling::T_Samples res;
    /*Eigen::Matrix4d toWorldCoordinates = Eigen::Matrix4d::Identity();
    toWorldCoordinates.block<3,3>(0,0) = limb->parent->toWorldRotation;
    toWorldCoordinates.block<3,1>(0,3) = limb->parent->position;*/
    Object* effector = GetEffector(limb);
    std::vector<Eigen::Vector3d> effectorPos = GetEffectorsRec(limb);
    for(T_Samples::const_iterator sit = samples.begin(); sit != samples.end(); ++sit)
    {
        Eigen::Vector3d normal, projection;
        for(Object::T_Object::iterator oit = obstacles.begin(); oit != obstacles.end(); ++oit)
        {
            if(effector->InContact(*oit,epsilon, normal, projection) && !planner::IsSelfColliding(&robot, limb) && !effector->IsColliding(obstacles))
            {
                res.push_back(*sit);
                break;
            }
        }
    }
    planner::sampling::LoadSample(*save, limb);
    return res;
}


T_Samples planner::GetPosturesOnTarget(Robot& robot, Node* limb, const sampling::T_Samples &samples
                                         , Object::T_Object& obstacles, Eigen::Vector3d worldposition)
{
    T_Samples res;
    Object* effector = GetEffector(limb);
    for(T_Samples::const_iterator sit = samples.begin(); sit != samples.end(); ++sit)
    {
        LoadSample(*(*sit),limb);
        if((effector->GetPosition() - worldposition).norm()<epsilon && !effector->IsColliding(obstacles))
        {
            //std::cout << "si si ca arrive " << std::endl;
            res.push_back(*sit);
        }
    }
    return res;
}

namespace
{

    std::vector<int> GetLimbsToContact(const std::vector<int>& inContactBefore, int nbLimbs)
    {
        std::vector<int> res;
        for(int i=0; i < nbLimbs; ++i)
        {
            if(std::find(inContactBefore.begin(), inContactBefore.end(), i)==inContactBefore.end())
            {
                res.push_back(i);
            }
        }
        return res;
    }

    planner::State* Interpolate(planner::CompleteScenario& scenario, const State& previous, const Model* next)
    {
        planner::State* res = new State;
        Robot * robot = new planner::Robot(*(previous.value));
        robot->SetConfiguration(next->englobed);
        res->value = robot;
        std::vector<Node*> limbs;
        for(std::vector<Node*>::iterator it = scenario.limbs.begin()
            ; it!=scenario.limbs.end(); ++it)
        {
            limbs.push_back(planner::GetChild(robot,(*it)->id));
        }
        //try to get back to previous state for starters.
        for(std::vector<int>::const_iterator cit = previous.contactLimbs.begin();
            cit != previous.contactLimbs.end(); ++cit)
        {
            T_Samples samples = GetPosturesOnTarget(*robot, limbs[*cit], scenario.limbSamples[*cit],
                                                    scenario.scenario->objects_, planner::GetEffectors(limbs[*cit])[0]->parent->position);
            if(!samples.empty())
            {
                res->contactLimbs.push_back(*cit);
                planner::sampling::LoadSample(*(samples.front()),limbs[*cit]);
            }
        }
        // create contacts then
        Eigen::Vector3d direction = next->GetPosition() - previous.value->node->position;
        if(direction.norm() != 0)
        {
            direction.normalize();
        }
        else
        {
            direction = Eigen::Vector3d(1,0,0);
        }
        std::vector<int> newContacts = GetLimbsToContact(res->contactLimbs, limbs.size());
        Eigen::Vector3d tmp, normal;
        for(std::vector<int>::const_iterator cit = newContacts.begin();
            cit != newContacts.end(); ++cit)
        {
            Sample* sample = GetPosturesInContact(*robot, limbs[*cit], scenario.limbSamples[*cit],
                                                    scenario.scenario->objects_, direction, tmp, normal);
            if(sample)
            {
                res->contactLimbs.push_back(*cit);
                res->contactLimbPositions.push_back(tmp);
                res->contactLimbPositionsNormals.push_back(normal);
                //planner::sampling::LoadSample(*sample,limbs[*cit]);
            }
        }
        return res;
    }

    void developPathRec(const Model* a, const Model* b, CT_Model& res)
    {
        Eigen::Vector3d line = b->GetPosition() - a->GetPosition();
        // norm
        double step = 0.1;
        if(line.norm() == 0) return;
        Eigen::Vector3d norm = line;
        norm.normalize(); line = line * step;
        Eigen::Vector3d offset;
//Eigen::Vector3d offrot;
        Eigen::Matrix3d offrotmat;
        // TODO rotate
        //  Vr = Va + t .(Vb - Va )

        // euler angle decomposition

        Eigen::Quaterniond qa(a->GetOrientation());
        Eigen::Quaterniond qb(b->GetOrientation());
        Eigen::Quaterniond qres;

//Eigen::Vector3d ea = a->GetOrientation().eulerAngles(2, 0, 2);
//Eigen::Vector3d eb = b->GetOrientation().eulerAngles(2, 0, 2);
        Eigen::Vector3d va = a->GetPosition();
        Eigen::Vector3d vb = b->GetPosition();

        /*Vector3f ea = mat.eulerAngles(2, 0, 2);

        "2" represents the z axis and "0" the x axis, etc. The returned angles are such that we have the following equality:
        * mat == AngleAxisf(ea[0], Vector3f::UnitZ())
        * * AngleAxisf(ea[1], Vector3f::UnitX())
        * * AngleAxisf(ea[2], Vector3f::UnitZ());*/

        /*Perform linear interpolation*/
        float linenorm = (float)line.norm();
        float nbSteps = (float)(linenorm / 0.01);
        float inc = 1 / nbSteps;
        for(double t = 0; t < 1; t = t + inc)
        {
            offset = va + t * (vb - va);
            qres = qa.slerp(t, qb);
            //offrot = ea + t * (eb - ea);
            offrotmat = qres.matrix();
            Model* tmp = new Model(*a);
            tmp->SetPosition(offset);
            tmp->SetOrientation(offrotmat);
            res.push_back(tmp);
        }
        res.push_back(new Model(*b));
    }

    CT_Model developPath(const CT_Model& initpath )
    {
        CT_Model res;
        if(initpath.size() <= 2) return initpath;
        CT_Model::const_iterator it2 = initpath.begin(); ++it2;
        for(CT_Model::const_iterator it = initpath.begin(); it2!=initpath.end(); ++it, ++it2)
        {
            developPathRec(*it, *it2, res);
        }
    return res;
    }

}


planner::T_State planner::PostureSequence(planner::CompleteScenario& scenario)
{
    planner::T_State res;
    State* current = &scenario.initstate;
    //current->stable = Stable(current);
    res.push_back(current);
    CT_Model path = developPath(scenario.path);
    for(CT_Model::iterator it = path.begin(); it!=path.end(); ++it)
    {
        current = Interpolate(scenario, *current, *it);
        current->stable = false;// Stable(current);
        res.push_back(current);
    }
    return res;
}
