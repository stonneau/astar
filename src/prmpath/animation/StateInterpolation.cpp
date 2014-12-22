#include "prmpath/animation/StateInterpolation.h"
#include "prmpath/ik/IKSolver.h"
#include "prmpath/ik/ObstacleAvoidanceConstraint.h"
#include "prmpath/ik/PartialDerivativeConstraint.h"
#include "prmpath/ik/VectorAlignmentConstraint.h"
#include "prmpath/ik/MatchTargetConstraint.h"

#include "tools/ExpMap.h"

#include <vector>

using namespace std;
using namespace planner;

namespace
{
    planner::C2_Point MakeConfiguration(const planner::State& state)
    {
        matrices::ExpMap emap(state.value->currentRotation);
        return std::make_pair(state.value->currentPosition, emap.log());
    }

    struct InterpolateLine
    {
        InterpolateLine(const Eigen::Vector3d& from, const Eigen::Vector3d& to)
            : from_(from), to_(to){}
        ~InterpolateLine(){}

        Eigen::Vector3d operator () (double t) const
        {
            return from_ + t * (to_ - from_);
        }
        Eigen::Vector3d from_;
        Eigen::Vector3d to_;
    };

    struct DoIk
    {
        DoIk(const planner::CompleteScenario& scenario, const planner::State* state)
            : scenario(scenario)
        {
            for(std::vector<int>::const_iterator cit = state->contactLimbs.begin();
                cit != state->contactLimbs.end(); ++cit)
            {
                planner::Node* limb =  planner::GetChild(state->value,scenario.limbs[*cit]->id);
                std::vector<ik::PartialDerivativeConstraint*> constraints;
                ik::MatchTargetConstraint* constraint = new ik::MatchTargetConstraint(limb);
                constraints.push_back(constraint);
                allconstraints.push_back(constraints);
            }
        }

        ~DoIk()
        {
            for(std::vector< std::vector<ik::PartialDerivativeConstraint*> >::iterator cit = allconstraints.begin();
                cit != allconstraints.end(); ++cit)
            {
                for(std::vector<ik::PartialDerivativeConstraint*>::iterator it2 = cit->begin();
                    it2 != cit->end(); ++it2)
                {
                    delete(*it2);
                }
            }
        }

        void operator ()(planner::State* state)
        {
            ik::IKSolver solver;
            std::vector<Eigen::Vector3d>::iterator posit = state->contactLimbPositions.begin();
            std::vector<Eigen::Vector3d>::iterator normit = state->contactLimbPositionsNormals.begin();
            int limbId = 0;
            for(std::vector<int>::const_iterator cit = state->contactLimbs.begin();
                cit != state->contactLimbs.end(); ++cit, ++posit, ++normit, ++limbId)
            {
                planner::Node* limb =  planner::GetChild(state->value,scenario.limbs[*cit]->id);
                int limite = 100;
                while(limite > 0 && !solver.StepClamping(limb, *posit, *posit, allconstraints[limbId], true))
                {
                    limite--;
                }
            }
        }

        ik::IKSolver solver;
        std::vector< std::vector<ik::PartialDerivativeConstraint*> > allconstraints;
        const planner::CompleteScenario& scenario;
    };

    // index in "to" state vectors
    std::vector<int> GetModifiedContacts(const planner::State& from, const planner::State& to)
    {
        std::vector<int> res;
        int lIndex = 0;
        for(std::vector<int>::const_iterator cit = to.contactLimbs.begin();
            cit != to.contactLimbs.end(); ++cit, ++lIndex)
        {
            const Eigen::Vector3d& positionTo = from.contactLimbPositions[lIndex];

            int lIndex2 = 0;
            bool notfound(true);
            for(std::vector<int>::const_iterator cit2 = from.contactLimbs.begin();
                cit2 != from.contactLimbs.end() && notfound; ++cit2, ++lIndex2)
            {
                if(*cit2 == *cit)
                {
                    const Eigen::Vector3d& positionFrom = to.contactLimbPositions[lIndex2];
                    notfound = false;
                    if((positionFrom - positionTo).norm() > 0.00001)
                    {
                        std::cout << "limb not matching :" <<*cit << std::endl;
                        res.push_back(*cit);
                    }
                    break;
                }
            }
            if(notfound) // new contact created
            {
                res.push_back(*cit);
                std::cout << "limb not matching  NEW :" <<*cit << std::endl;
            }
        }
        std::cout << "limb new :" << res.size() << std::endl;
        return res;
    }

    std::vector<InterpolateLine> ContactInterpolation(const std::vector<int>& contacts, const planner::CompleteScenario& scenario, const planner::State& from, const planner::State& to)
    {
        std::vector<InterpolateLine> res;
        for(std::vector<int>::const_iterator cit = contacts.begin();
            cit != contacts.end(); ++cit)
        {
            // Find position in initial Configuration
            std::cout << " je pousse " << scenario.limbs[*cit]->tag << std::endl;
            res.push_back(InterpolateLine(
                planner::GetEffectorCenter(planner::GetChild(from.value, scenario.limbs[*cit]->id)),
                to.contactLimbPositions[*cit]));
        }
        return res;
    }

    struct InterpolateContacts
    {
        InterpolateContacts(const planner::CompleteScenario& scenario, const planner::State& from, const planner::State& to)
            :involvedContacts_(GetModifiedContacts(from, to))
            ,contactInterpolation_(ContactInterpolation(involvedContacts_, scenario, from, to))
        {
            // NOTHING
        }
        ~InterpolateContacts(){}
        void operator ()(planner::State& current, double time) const
        {
            std::vector<InterpolateLine>::const_iterator intit = contactInterpolation_.begin();
            for(std::vector<int>::const_iterator cit = involvedContacts_.begin();
                cit!= involvedContacts_.end(); ++cit, ++intit)
            {
                current.contactLimbPositions[*cit] = (*intit)(time);
            }
        }
        const std::vector<int> involvedContacts_;
        const std::vector<InterpolateLine> contactInterpolation_;
    };

}

planner::T_State planner::Animate(const planner::CompleteScenario& scenario, const planner::State& from, const planner::State& to, int nbFrames)
{
    InterpolateContacts interpolate(scenario,from,to);
    planner::InterpolatePath path(MakeConfiguration(from),MakeConfiguration(to),0,1);
    planner::T_State res;
    DoIk doIk(scenario,&to);
    res.push_back(new State(&from));
    State* current = new State(&from);
    current->contactLimbPositions = to.contactLimbPositions;
    current->contactLimbPositionsNormals = to.contactLimbPositionsNormals;
    current->contactLimbs = to.contactLimbs;
    double stepsize = double(1) / double(nbFrames-1); double step = stepsize;
    for(int i = 1; i< nbFrames-1; ++i)
    {
        current = new State(current);
        planner::Configuration conf = path.Evaluate(step);
        current->value->SetFullRotation(conf.second, false);
        current->value->SetPosition(conf.first, true);
        interpolate(*current, step);
        doIk(current);
        res.push_back(current);
        step += stepsize;
    }
    res.push_back(new State(&to));
    return res;
}
