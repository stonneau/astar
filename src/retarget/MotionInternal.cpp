#include "retarget/MotionInternal.h"
#include "prmpath/CompleteScenario.h"
#include "prmpath/PostureSelection.h"

#include "collision/Sphere.h"
#include "prmpath/CompleteScenario.h"
#include "prmpath/ik/IKSolver.h"
#include "prmpath/ik/VectorAlignmentConstraint.h"

struct efort::PImpl
{
    PImpl(planner::CompleteScenario* cScenario)
        : cScenario_(cScenario)
        , states_(cScenario_->states)
    {
        // init contact by limbs
        for(std::size_t i=0; i< cScenario_->limbs.size(); ++i)
        {
            std::vector<Contact> limbContacts;
            contacts_.push_back(limbContacts);
        }
    }

    ~PImpl()
    {
        //delete cScenario_;
    }
    planner::CompleteScenario* cScenario_;
    planner::T_State& states_;
    std::vector<std::vector<Contact> > contacts_;
};

using namespace efort;
using namespace planner;

Frame MotionI::Retarget(const std::size_t frameid) const
{
    // TODO
    Frame frame;
    return frame;
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

    bool LimbColliding(Node* limb, planner::Object::T_Object& obstacles, bool effector = true)
    {
        if( limb->current && ((effector || limb->current != GetEffector(limb)) && limb->current->IsColliding(obstacles)))
        {
                return true;
        }
        if(limb->children.size() == 0)
            return false;
        return LimbColliding(limb->children[0], obstacles);
    }

    void SolveIk(Node* limb, const Eigen::Vector3d& target, const Eigen::Vector3d& normal)
    {
        ik::IKSolver solver;//(0.001f, 0.001f,0.1f);
        ik::VectorAlignmentConstraint constraint(normal);
        std::vector<ik::PartialDerivativeConstraint*> constraints;
        constraints.push_back(&constraint);
        int limit = 30;
        while(limit > 0 )
        {
            solver.StepClamping(limb, target, normal, constraints, true);
            limit--;
        }
    }
}

planner::State* MotionI::Retarget(const std::size_t frameid, const std::vector<Eigen::Vector3d>& targets, Object::T_Object &objects) const
{
    const Frame& cframe = frames_[frameid];
    //planner::Robot* robot = new planner::Robot( *pImpl_->states_[frameid]->value);
    planner::State* state = new planner::State();
    planner::Robot* robot = new planner::Robot( *pImpl_->states_[frameid]->value);
    state->value = robot;
    std::size_t id(0);
    for(std::vector<Contact>::const_iterator cit = cframe.contacts_.begin();
        cit !=cframe.contacts_.end(); ++cit, ++id)
    {
        // get corresponding robot

        Node* limb = planner::GetChild(robot,pImpl_->cScenario_->limbs[cit->limbIndex_]->id);
        Sphere sphereCurrent(robot->currentRotation * robot->constantRotation.transpose() * pImpl_->cScenario_->limbRoms[cit->limbIndex_].center_ + robot->currentPosition,
                              pImpl_->cScenario_->limbRoms[cit->limbIndex_].radius_ * 1.5);
        bool contactMaintained(false);
        if(Contains(sphereCurrent, targets[id]))
        {
            std::cout << "in range" << std::endl;
            SolveIk(limb, targets[id], cit->surfaceNormal_);
            if(!LimbColliding(limb,objects,false))
            {
                contactMaintained = true;
                state->contactLimbPositions.push_back(cit->worldPosition_);
                state->contactLimbPositionsNormals.push_back(cit->surfaceNormal_);
                state->contactLimbs.push_back(cit->limbIndex_);
            }
        }
        if(!contactMaintained)
        {
            Eigen::Vector3d position, normal;
            std::cout << "out of rage " << limb->tag << std::endl;
            std::vector<planner::Sphere*> dm;
            planner::sampling::Sample* nc = planner::GetPosturesInContact(*robot, limb, pImpl_->cScenario_->limbSamples[cit->limbIndex_],objects,cit->surfaceNormal_,position, normal, *(pImpl_->cScenario_), dm);
            if(nc)
            {
                std::cout << "trouve " << limb->tag << std::endl;
                planner::sampling::LoadSample(*nc, limb);
                SolveIk(limb, position, normal);
                state->contactLimbPositions.push_back(position);
                state->contactLimbPositionsNormals.push_back(normal);
                state->contactLimbs.push_back(cit->limbIndex_);
            }
            else
            {
                nc = planner::GetCollisionFreePosture(*robot,limb, pImpl_->cScenario_->limbSamples[cit->limbIndex_],objects);
                if(nc) planner::sampling::LoadSample(*nc, limb);
                state->contactLimbPositions.push_back(cit->worldPosition_);
                state->contactLimbPositionsNormals.push_back(cit->surfaceNormal_);
                state->contactLimbs.push_back(cit->limbIndex_);
            }
        }
    }
    return state;
}

namespace
{
    std::vector<Frame> FramesFromStates(efort::PImpl* pImpl)
    {
        std::vector<Frame> res;
        // pour le moment on charge le chemin
        int numFrame = 0;
        std::vector< std::vector<std::size_t> > contactids; // storing references to contacts created at each frame
        for(planner::T_State::const_iterator sit_1 = pImpl->states_.begin();
            sit_1 != pImpl->states_.end(); ++sit_1, ++numFrame)
        {
            std::vector<std::size_t> frameContactIds;
            for(int i=0; i< pImpl->contacts_.size(); ++i)
            {
                frameContactIds.push_back(-1);
            }
            // create vectors
            State& cState = **sit_1;
            int cid = 0;
            for(std::vector<int>::const_iterator cit = cState.contactLimbs.begin();
                cit != cState.contactLimbs.end(); ++cit, ++cid)
            {
                //find position
                bool newContact(true);
                if(!pImpl->contacts_[*cit].empty())
                {
                    Contact& previous = pImpl->contacts_[*cit].back();
                    if(previous.endFrame_ == numFrame-1 && (previous.worldPosition_ - cState.contactLimbPositions[cid]).norm() < 0.01)
                    {
                        previous.endFrame_ = numFrame;
                        newContact = false;
                    }
                }
                if(newContact)
                {
                    Contact contact;
                    contact.startFrame_ = numFrame;
                    contact.endFrame_ = numFrame;
                    contact.limbIndex_ = *cit;
                    contact.surfaceNormal_ =  cState.contactLimbPositionsNormals[cid];
                    contact.triangleId_ = -1;
                    contact.objectId_ = -1;
                    contact.worldPosition_ =cState.contactLimbPositions[cid];
                    pImpl->contacts_[*cit].push_back(contact);
                }
                frameContactIds[*cit] = pImpl->contacts_[*cit].size()-1;
            }
            contactids.push_back(frameContactIds);
        }
        numFrame = 0;
        for(planner::T_State::const_iterator sit_1 = pImpl->states_.begin();
            sit_1 != pImpl->states_.end(); ++sit_1, ++numFrame)
        {
            Frame frame;
            //frame.configuration_ = planner::AsConfiguration((*sit_1)->value);
            for(int i=0; i< pImpl->contacts_.size(); ++i)
            {
                std::size_t id = contactids[numFrame][i];
                if(id != -1)
                {
                    frame.contacts_.push_back(pImpl->contacts_[i][id]);
                }
            }
            res.push_back(frame);

        }
        return res;
    }
}


MotionI* efort::LoadMotionI(CompleteScenario *scenario)
{
    MotionI* motion = new MotionI;
    motion->pImpl_.reset(new PImpl(scenario));
    motion->frames_ = FramesFromStates(motion->pImpl_.get());
    return motion;
}


