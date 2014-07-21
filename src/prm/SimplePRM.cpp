/**
* \file SimplePRM.cpp
* \brief A PRM concrete implementation
* \author Steve T.
* \version 0.1
* \date 07/07/2014
*
*/

#include "SimplePRM.h"
#include "collision/ParserObj.h"
#include "planner/PRM.h" // Template PRM definition

#include "LocalPlanner.h"
#include "Generator.h"

namespace planner
{
    float Distance(const Object* obj1, const Object* obj2)
    {
        // TODO include angles
        const Eigen::Vector3d& a = obj1->GetPosition();
        const Eigen::Vector3d& b = obj2->GetPosition();
        return sqrt((b.x() - a.x()) * (b.x() - a.x()) + (b.y() - a.y()) * (b.y() - a.y()) + (b.z() - a.z()) * (b.z() - a.z()));
    }
	
    typedef PRM<Object, planner::Generator, LocalPlanner, float, 10000> prm_t;

	struct PImpl
	{
        PImpl(const Model& model, Object::T_Object& objects, float neighbourDistance, int size, int k)
            : planner_(objects)
        {
            Generator* gen = new Generator(objects, model); // TODO MEME
            prm_ = new prm_t(gen, &planner_, Distance, neighbourDistance, size, k);
            // feel prmNodes
            for(int i =0; i< prm_->currentIndex_ +1; ++i)
            {
                prmNodes_.push_back(prm_->nodeContents_[i]);
            }
        }
		~PImpl()
		{
			delete prm_;
		}

        Object* operator()()
		{
            return 0;
		}

		const prm_t* prm_;
        LocalPlanner planner_;
        Object::T_Object prmNodes_;

	};

    Object* GenerateConfiguration()
	{
        return 0;
    }
}

using namespace planner;

SimplePRM::SimplePRM(const Model &model, Object::T_Object &objects, float neighbourDistance, int size, int k)
    : model_(model)
    , objects_(objects)
{
    pImpl_.reset(new PImpl(model_, objects_, neighbourDistance, size, k));
}

SimplePRM::~SimplePRM()
{
	// NOTHING
}

const Object::T_Object& SimplePRM::GetPRMNodes() const
{
    return pImpl_->prmNodes_;
}


const std::vector<int> &SimplePRM::GetConnections(int node) const
{
    return pImpl_->prm_->edges_[node];
}

/*Object::T_Object SimplePRM::GetPath(const Object &from, const Object &to, float neighbourDistance)
{
	return pImpl_->prm_->ComputePath(&from, &to, Distance, world_, neighbourDistance);
}*/
