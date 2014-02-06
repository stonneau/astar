
#ifndef _CLASS_ASTAR
#define _CLASS_ASTAR

#include "Graph.h"
#include <algorithm>
#include <vector>
#include <list>
#include <map>

namespace astar
{
template<typename Numeric=float, typename Index = int>
struct CompareOpenSetValues
{
	CompareOpenSetValues(const std::map<Index,Numeric>& costs)
		: costs_(costs)
	{
		// NOTHING
	}

	~CompareOpenSetValues()
	{
		// NOTHING
	}

	bool operator()(Index a, Index b) const
	{
		return (costs_.at(a) < costs_.at(b));
	}

	const std::map<Index,Numeric>& costs_;
};

template<typename Numeric=float, int Dim=10000, typename Index = int, typename NodeContent = int>
class AStar
{
public:
	typedef Graph<Numeric, Dim, Index, NodeContent> graph_t;
	typedef typename std::list<Index> Path;
	typedef   Numeric   (*Distance)  (const NodeContent*, const NodeContent* );

private:
	typedef typename std::list<Index> OpenSet;
	typedef typename std::map<Index,bool> T_Node;
	typedef typename std::map<Index,Numeric> T_Cost;
	typedef typename std::map<Index,Index> T_Parent;

public:
	 AStar(const graph_t& graph)
		 : graph_(graph)
	 {
		 // NOTHING
	 }

	~AStar()
	{
		// NOTHING
	}

private:
	AStar(const AStar&);
	AStar& operator=(const AStar&);

public:
	bool ComputePath(const Index from, const Index to, Path& path, Distance dist)
	{
		OpenSet openSet;
		T_Node closedSet; // DO I NEED THIS WITH A NON CYCLIC GRAPH ?
		T_Parent cameFrom;
		T_Cost costFromStart;
		T_Cost estimatedTotalCost;
		NodeContent* goal = graph_.nodeContents_[to];
		CompareOpenSetValues<Numeric, Index> compare (estimatedTotalCost); // TODO CHECK TOTAL COST AND NOT LOCAL

		openSet.push_back(from);
		costFromStart[from] = 0;
		estimatedTotalCost[from] = dist(graph_.nodeContents_[from], goal);

		while(openSet.size())
		{
			Index currentNode = openSet.front();
			openSet.pop_front();
			if(currentNode == to)
			{
				ReconstructPath(path, cameFrom, to);
				return true;
			}
			closedSet[currentNode] = true;
			for(graph_t::T_Connexion::const_iterator it = graph_.edges_[currentNode].begin(); it != graph_.edges_[currentNode].end(); ++it)
			{
				if(closedSet.find(*it) != closedSet.end())
				{
					continue;
				}
				Numeric currentGScore = costFromStart[currentNode] + dist(graph_.nodeContents_[currentNode], graph_.nodeContents_[*it]);
				OpenSet::iterator openit = std::find(openSet.begin(), openSet.end(), currentNode);
				T_Cost::iterator costit = costFromStart.find(*it);
				if(openit == openSet.end() || (costit != costFromStart.end() && costit->second > currentGScore))
				{
					cameFrom[*it] = currentNode;
					costFromStart[*it] = currentGScore;
					Numeric currentTotalScrore = currentGScore + dist(graph_.nodeContents_[*it], goal);
					estimatedTotalCost[*it] = currentTotalScrore;
					if(openit == openSet.end())
					{
						// sorted insertion
						openSet.push_front(*it);
						openSet.sort(compare);
					}
				}
			}
		}
		return false;
	}

private:
	void ReconstructPath(Path& path, const T_Parent& cameFrom, const Index to)
	{
		T_Parent::const_iterator it = cameFrom.find(to);
		path.push_front(to);
		if(it != cameFrom.end())
		{
			ReconstructPath(path, cameFrom, it->second);
		}
	}
	
public:
	const graph_t& graph_;
};
} //namespace astar
#endif //_CLASS_ASTAR
