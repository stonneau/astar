#include "PostureSelection.h"
#include "tools/MatrixDefs.h"
#include "ik/IKSolver.h"
#include "ik/VectorAlignmentConstraint.h"
#include "equilibrium/DynamicStability.h"
#include "collision/Collider.h"
#include "smoothing/smooth.h"

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


    double LimbLengthRec(const Node* limb)
    {
        double minDistance =  (limb->children.empty()) ?  0 : std::numeric_limits<double>::max();
        double tmp;
        for(std::vector<Node*>::const_iterator cit = limb->children.begin();
            cit != limb->children.end(); ++cit)
        {
            tmp = LimbLengthRec(*cit);
            if(tmp < minDistance) minDistance = tmp;
        }
        return minDistance + (limb->position - limb->parent->position).norm();
    }


    bool NextIsInRange(const planner::Node* limb, const Eigen::Vector3d& target, const Object* objrom,  const Object* point)
    {
        if(point && objrom)
        {
            double totalLength = limb->children.empty() ? 0 :  LimbLengthRec(limb->children.front());
            return (objrom->GetPosition() - target).norm() < totalLength * 0.9;
        }
        return true;
    }

    double CostMaintainContact(const planner::Sphere* nextRom, const Eigen::Vector3d& target)
    {
        if(nextRom)
        {
            return (nextRom->center_ - target).norm();
        }
        return 0;
    }
}

Sample* planner::GetPosturesInContact(Robot& robot, Node* limb, const sampling::T_Samples& samples
                                         , Object::T_Object& obstacles, const Eigen::Vector3d& direction
                                         , Eigen::Vector3d& position, Eigen::Vector3d& normalVector
                                         , planner::CompleteScenario& scenario, const planner::Sphere* rom)
{
    Sample* save = new Sample(limb);
    Sample* res = 0;
    Object* effector = GetEffector(limb);
    //std::vector<Eigen::Vector3d> effectorPos = GetEffectorsRec(limb);
    //Eigen::Vector3d effectorCentroid = planner::GetEffectorCenter(limb);
    double bestManip = std::numeric_limits<double>::min();
    double tmp_manip, tempweightedmanip;
    std::size_t found = limb->tag.find("leg");
    Eigen::Vector3d dir = direction;
    //if(direction.y() < 0 ) dir = -direction;
    Eigen::Vector3d dirn = dir;
    /*if (found==std::string::npos)
    {*/
        dirn = Eigen::Vector3d(0,1.,0.);
        dirn.normalize();
    /*}*/
    dir = robot.currentRotation * dir; //Eigen::Vector3d(0,1,0.);
    dirn = robot.currentRotation * dirn; //Eigen::Vector3d(0,1,0.);
    for(T_Samples::const_iterator sit = samples.begin(); sit != samples.end(); ++sit)
    {
//tmp_manip = direction.y() < 0 ?  planner::sampling::VelocityManipulability(*sit, dir) :  planner::sampling::ForceManipulability(*sit, dir);
        tmp_manip = planner::sampling::ForceManipulability(*sit, dir);
        if(tmp_manip > bestManip)
        {
            Eigen::Vector3d normal, projection;
            LoadSample(*(*sit),limb);
            if(!(planner::IsSelfColliding(&robot, limb) || LimbColliding(limb, obstacles)))
            {
                for(Object::T_Object::iterator oit = obstacles.begin(); oit != obstacles.end(); ++oit)
                {
//if(effector->InContact(*oit,epsilon, normal, projection) && !planner::IsSelfColliding(&robot, limb) && !LimbColliding(limb, obstacles))
                    if(effector->InContact(*oit,epsilon, normal, projection) && planner::SafeTargetDistance(limb,projection,0.9) )//&& NextIsInRange(limb, projection, rom, scenario.scenario->point_))
                    //if(planner::MinDistance(effectorCentroid, *oit, projection, normal) < epsilon && !planner::IsSelfColliding(&robot, limb) && !LimbColliding(limb, obstacles))
                    {
                        tempweightedmanip = tmp_manip * dirn.dot(robot.currentRotation * normal);
                        tempweightedmanip -= CostMaintainContact(rom, projection);
                        if(tempweightedmanip > bestManip)// && (planner::SafeTargetDistance(limb,projection,0.9)))
                        {
                            bestManip = tempweightedmanip;
                            res = * sit; //tempweightedmanip > -10 ? *sit : 0;
                            //position = effector->GetPosition();
                            normalVector = normal;
                            position = projection;
                           // break;
                        }
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
            int limit = 0;
            //int limit2 = 100;
            while(limit > 0 && !solver.StepClamping(limb, position, position, constraints, true))
            {
                //solver.StepClamping(limb, position, position, constraints, true);
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
                                         , Object::T_Object& obstacles, const Eigen::Vector3d& direction, CompleteScenario &scenario)
{
    Eigen::Vector3d dummmy, dummmy2;
    return GetPosturesInContact(robot, limb, samples, obstacles, direction, dummmy, dummmy2, scenario, 0);
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
                                         , Object::T_Object& obstacles, const Eigen::Vector3d& worldposition)
{
    Sample* save = new Sample(limb);
    T_Samples res;
    Object* effector = GetEffector(limb);
    for(T_Samples::const_iterator sit = samples.begin(); sit != samples.end(); ++sit)
    {
        LoadSample(*(*sit),limb);
        if((effector->GetPosition() - worldposition).norm()<2*epsilon && !effector->IsColliding(obstacles))
        {
            //std::cout << "si si ca arrive " << std::endl;
            res.push_back(*sit);
        }
    }
    planner::sampling::LoadSample(*save, limb);
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

    /*planner::State* Interpolate(planner::CompleteScenario& scenario, const State& previous, const Model* next)
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
    }*/

    void Vector3toArray(const Eigen::Vector3d& vect, double* arr)
    {
        for(int i=0; i<3; ++i)
        {
            arr[i] = vect[i];
        }
    }

    planner::State* Interpolate(planner::CompleteScenario& scenario, const State& previous, const Model* next, const Model* nextnext)
    {
        //std::cout << " Satate " <<  std::endl;
        // Create new state and move it to path location
        State* state = new State();
        state->value = new Robot(*previous.value);
        Robot* robot = state->value;
        state->value->SetPosition(next->GetPosition(), false);
        state->value->SetRotation(next->GetOrientation(), true);

        // retrieve indexes of limbs to iterate over them
        std::vector<Node*> limbs;
        for(std::vector<Node*>::iterator it = scenario.limbs.begin()
            ; it!=scenario.limbs.end(); ++it)
        {
            limbs.push_back(planner::GetChild(robot,(*it)->id));
        }

        int lIndex = 0;
        for(std::vector<Node*>::iterator lit = limbs.begin(); lit != limbs.end(); ++lit, ++lIndex)
        {
            bool maintainPreviousTarget = false;
            Eigen::Vector3d target, normal;
            Eigen::Vector3d direction = next->GetPosition() - previous.value->node->position;
            direction = direction.norm() == 0 ? Eigen::Vector3d(0,1,0) : direction;
            direction.normalize();
            if((*lit)->tag.find("leg") != std::string::npos)
            {
                //direction = Eigen::Vector3d(0,1,0);
            }
            if(previous.InContact(lIndex, target, normal) && SafeTargetDistance(*lit,target,0.85)) // limb was in contact, try to maintain it
            {
                /*T_Samples samples = GetPosturesOnTarget(*robot, *lit, scenario.limbSamples[lIndex], scenario.scenario->objects_, target);
                // TODO TRY TO USE IK IN FACT
                if(!samples.empty())
                {
                    state->contactLimbs.push_back(lIndex);
                    state->contactLimbPositions.push_back(target);
                    state->contactLimbPositionsNormals.push_back(normal);
                    planner::sampling::LoadSample(*(samples.front()),*lit);
//std::cout << " limb mainained in contact " << lIndex <<  std::endl;
                }
                else*/
                {
                    state->contactLimbs.push_back(lIndex);
                    state->contactLimbPositions.push_back(target);
                    state->contactLimbPositionsNormals.push_back(normal);
                    int limit = 20;
                    //int limit2 = 100;
                    ik::IKSolver solver;
                    ik::VectorAlignmentConstraint constraint(normal);
                    std::vector<ik::PartialDerivativeConstraint*> constraints;
                    constraints.push_back(&constraint);
                    while(limit > 0 && !solver.StepClamping(*lit, target, direction, constraints, true))
                    {
                        limit--;
                    }
                }

                maintainPreviousTarget = true; // MOVE OUT OF BLOCK WITH IK USE
            }
            if(!maintainPreviousTarget) // could not reach previous target, get a new one (reasons are distance of collision)
            {
// TODO INCLUDE SAFE TARGET DISTANCE
                planner::Sphere* sphere(0);
                if(nextnext)
                {
                    sphere = new Sphere(nextnext->GetOrientation() * scenario.limbRoms[lIndex].center_ + nextnext->GetPosition(), scenario.limbRoms[lIndex].radius_);
                }
                Sample* sample = GetPosturesInContact(*robot, *lit, scenario.limbSamples[lIndex],
                                                      scenario.scenario->objects_, direction, target, normal, scenario, sphere);
                if(sample)
                {
                    state->contactLimbs.push_back(lIndex);
                    state->contactLimbPositions.push_back(target);
                    state->contactLimbPositionsNormals.push_back(normal);
                    //planner::sampling::LoadSample(*sample,limbs[*cit]);
//std::cout << " limb in contact " << lIndex <<  std::endl;
                }
            }
        }
        return state;
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

    CT_Model developPathSpline(const planner::SplinePath& splinePath, const Model& model)
    {
        CT_Model res;
        for(double t =0; t<=1; t = t+0.025)
        {
            Model * tmp = new Model(model);
            Configuration c = splinePath.Evaluate(t);
            tmp->SetPosition(c.first);
            tmp->SetOrientation(c.second);
            res.push_back(tmp);
        }
        return res;
    }


    T_Model::iterator random_element(T_Model::iterator& begin, T_Model::iterator& end)
    {
        const unsigned long n = std::distance(begin, end);
        const unsigned long divisor = (RAND_MAX + 1) / n;

        unsigned long k;
        do { k = std::rand() / divisor; } while (k >= n);

        T_Model::iterator res = begin;
        std::advance(res, k);
        return res;
    }


    std::vector<int> GetConfiguration(T_Model::iterator& c)
    {
        std::vector<int> res;
        // first push position
        for(int i =0; i<3; ++i)
        {
            res.push_back((*c)->GetPosition()(i));
        }
        Eigen::Vector3d euler = (*c)->GetOrientation().eulerAngles(2, 1, 0);
        for(int i =0; i<3; ++i)
        {
            res.push_back(euler(i));
        }
        return res;
    }

    bool SetConfiguration(T_Model::iterator& c, const std::vector<int>& config, planner::Collider& collider)
    {
        Eigen::Vector3d position;
        for(int i =0; i<3; ++i)
        {
            position(i) = config[i];
        }
        (*c)->SetPosition(position);
        Eigen::Matrix3d daf =  Eigen::AngleAxisd(config[3], Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(config[4], Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(config[5], Eigen::Vector3d::UnitX()).matrix();
        (*c)->SetOrientation(daf);
        return !(collider.IsColliding((*c)->englobed));
    }

    T_Model CopyPath(const T_Model& initpath)
    {
        T_Model res;
        for(T_Model::const_iterator it = initpath.begin(); it!=initpath.end(); ++it)
        {
            res.push_back(new Model(**it));
        }
        return res;
    }

    void DeletePath(T_Model& initpath)
    {
        for(T_Model::iterator it = initpath.begin(); it!=initpath.end(); ++it)
        {
            delete(*it);
        }
    }

    CT_Model PartialShortcut(const CT_Model& initpath, planner::Collider& collider )
    {
        int limit = 0; // time allowed
        CT_Model finalPath;
        T_Model res;
        int nbNodes = initpath.size();
        if(nbNodes <= 2) return initpath;
        // copy all nodes
        for(CT_Model::const_iterator it = initpath.begin(); it!=initpath.end(); ++it)
        {
            res.push_back(new Model(*(*it)));
        }

        T_Model::iterator c1,c2;
        while(limit > 0)
        {
            T_Model tmp = CopyPath(res);
            // choose two configs randomly
            c1 = (random_element(++tmp.begin(), --tmp.end()));
            c2 = (random_element(++tmp.begin(), --tmp.end()));
            const unsigned long n = std::distance(c1, c2);
            unsigned int currentIndex = 0;
            // choose one dof (0 = x, ..., 6 = r_x)
            int dof = (std::rand() % (int)(6));
            std::vector<int> pi1, pi2;
            pi1 = GetConfiguration(c1);
            pi2 = GetConfiguration(c2);
            T_Model::iterator it = c1;
            bool collisionFree = true;
            while(it != c2)
            {
                std::vector<int> pit = GetConfiguration(it);
                double normalizedIndex =(double)((double(currentIndex)/(double)(n)));
                pit[dof] =  (1-normalizedIndex) * pi1[dof] + normalizedIndex * pi2[dof];
                collisionFree = SetConfiguration(c2, pit, collider);
                if(!collisionFree) break;
                ++it; ++currentIndex;
            }
            if(collisionFree)
            {
                DeletePath(res);
                res = tmp;
            }
            limit--;
        }
        for(T_Model::iterator it = res.begin(); it!=res.end(); ++it)
        {
            finalPath.push_back(new Model(*(*it)));
        }
        DeletePath(res);
        return finalPath;
    }

}


planner::T_State planner::PostureSequence(planner::CompleteScenario& scenario)
{
    planner::T_State res;
    State* current = &scenario.initstate;
    planner::Collider collider(scenario.scenario->objects_);
    //current->stable = Stable(current);
    res.push_back(current);
    CT_Model path;
    if(scenario.path.size() > 2)
    {
        //scenario.spline = new planner::SplinePath(planner::SplineFromPath(collider,scenario.path,2,2));
        scenario.spline = new planner::SplinePath(planner::SplineShortCut(collider,scenario.path,2,2,0));
        //CT_Model path0 = developPathSpline(*scenario.spline, scenario.scenario->model_);
        CT_Model path0 = developPath(scenario.path);
        path = PartialShortcut(path0, collider);
    }
    else
    {
        path = scenario.path;
    }
    CT_Model::iterator it2 = path.begin(); ++it2;
    for(CT_Model::iterator it = path.begin(); it!=path.end(); ++it, ++it2)
    {
        if(it2 == path.end())
        {
            current = Interpolate(scenario, *current, *it, 0);
        }
        else
        {
            current = Interpolate(scenario, *current, *it, *it2);
        }
        current->stable = false;// Stable(current);
        res.push_back(current);
    }
    return res;
}
