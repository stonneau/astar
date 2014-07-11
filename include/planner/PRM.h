/**
* \file Graph.h
* \brief A Generic PRM planner implementation
* \author Steve T.
* \version 0.1
* \date 07/06/2014
*
* 
*/
#ifndef _CLASS_PRM
#define _CLASS_PRM

#include "astar/Graph.h"
#include "astar/AStar.h"

#include <limits>       // std::numeric_limits

namespace planner
{

/// \class PRM
/// \brief Generic implementation of a probabilistic roadmap (PRM) as a astar::Graph.
/// Generator and LocalPlanner are template class
/// Distance is a method given as a parameter.
/// A modified A* algorithm is used to solve request for NodeContent not included in the graph
template<class NodeContent, class Generator, class LocalPlanner, typename Numeric=float, int Dim=10000>
class PRM : public  astar::Graph<NodeContent, Numeric, Dim, int, true>
{

public:
    typedef astar::Graph<NodeContent, Numeric, Dim, int, true> graph_t;
	typedef Numeric (*Distance)		  (const NodeContent*, const NodeContent*);
	typedef std::vector<const NodeContent*> T_NodeContentPath;
public:
	///\brief Constructor
	///\param generator instance of Generator template class that will generate collision-free configurations using operator()
	///\param localPlanner instance of LocalPlanner class; operator() takes two NodeContent and
	/// returns true whether two nodes can be connected (a collision free-path exists)
	/// \param distance Function used to measure the distance between two nodes. It 
	///  has the signature Numeric (*Distance) (const NodeContent*, const NodeContent* )
	///\param neighbourDistance maximum distance for which a node can be a neighbour of another
	///\param size number of nodes to be generated
	///\param k maximum number of neighbours for a given Node. Default value is 10
	PRM(Generator* generator, const LocalPlanner* localPlanner, Distance distance, Numeric neighbourDistance, int size = Dim, int k=10)
        : graph_t()
        , size_(size)
	{
		for(int i=0; i< size; ++i)
		{
			NodeContent* node = (*generator)();
			int id = AddNode(node);
			int current_index = 0;
			int connected = 0;
            for(typename PRM::T_NodeContentPtr::iterator it = graph_t::nodeContents_.begin();
                 current_index < id && it != graph_t::nodeContents_.end() && connected <k ;
				++it, ++current_index)
			{
				if(current_index != id && distance(node,*it) <= neighbourDistance && (*localPlanner)(node,*it))
				{
                    if(graph_t::edges_[current_index].size() < ((unsigned int) k))
					{
                        graph_t::AddEdge(id, current_index);
						++connected;
					}
				}
			}
		}
	}

	///\brief Destructor
	 ~PRM(){}

	///  \brief Computes a path between arbitrary NodeContent, not necessarily in the graph.
	///  \param from the entry point in the search
	///  \param to the destination that is to be reached
	///  \param dist Function used to measure the distance between two nodes. It 
	///  has the signature Numeric (*Distance) (const NodeContent*, const NodeContent* )
	///  \param localPlanner boolean method that returns true whether two nodes can be connected (a collision free-path exists)
	///	 \param neighbourDistance maximum distance for which a node can be a neighbour of another
	///  \param return : vector of NodeContent traversed to reach the goal. Empty if no pathfinding failed
	 T_NodeContentPath ComputePath(const NodeContent* from, const NodeContent* to, Distance dist, const LocalPlanner* localPlanner, Numeric neighbourDistance) const
	 {
		int start_id = GetClosestPointInGraph(from, dist, localPlanner, neighbourDistance);
		int goal_id = GetClosestPointInGraph(to, dist, localPlanner, neighbourDistance);
        typename astar_t::Path path;
		T_NodeContentPath res;
		if(start_id != -1 && goal_id !=-1)
		{
			astar_t astar(*this);
			if(astar.ComputePath(start_id, goal_id, path, dist))
			{
				res.push_back(from);
				for(std::list<int>::const_iterator it = path.begin(); it != path.end(); ++it)
				{
                    res.push_back(graph_t::nodeContents_[*it]);
				}
				res.push_back(to);
			}
		 }
		 return res;
	 }
	 
private:
	int GetClosestPointInGraph(const NodeContent* node, Distance dist, const LocalPlanner* localPlanner, Numeric neighbourDistance) const //todo this is really expensive at the moment
	{
		Numeric min_distance = std::numeric_limits<Numeric>::max();
		int current_index = 0; 
		int closest_index = -1; 
        for(typename PRM::T_NodeContentPtr::const_iterator it = graph_t::nodeContents_.begin();
                it != graph_t::nodeContents_.end() && current_index < size_;
				++it, ++current_index)
		{
			Numeric current_distance = dist(node,*it);
			if(current_distance < min_distance && current_distance < neighbourDistance && (*localPlanner)(node,*it))
			{
				closest_index = current_index;
				min_distance = current_distance;
			}
		}
		return closest_index;
	}

private:
	typedef astar::AStar<NodeContent, Numeric, Dim, int, true> astar_t;
	const int size_;

private:
	PRM(const PRM&);
	PRM& operator=(const PRM&);
};
} //namespace planner
#endif //_CLASS_PRM
