
#include "astar/Graph.h"
#include "astar/AStar.h"

#include <string>
#include <iostream>
#include <cmath>

using namespace std;

namespace astar
{

struct NodeContent
{
	NodeContent(int id)
		: id_(id) {}

	~NodeContent(){}

	const int id_;
};

typedef Graph<NodeContent, float, 10, int> graph_t;
typedef Graph<NodeContent, float, 10, int, true> graph_cycle_t;
typedef AStar<NodeContent, float, 10, int, true> astar_t;
} // namespace astar

using namespace astar;


/*Graph related tests*/

void GraphCreationTest(bool& error)
{
	graph_t graph;
	graph_cycle_t graph_cycle;
	for(int j = 0; j< 4; ++ j)
	{
		graph.AddNode(new NodeContent(j));
		graph_cycle.AddNode(new NodeContent(j));
	}
	graph_cycle.AddEdge(0,1);
	graph_cycle.AddEdge(0,2);
	graph_cycle.AddEdge(2,1);
	if(!graph_cycle.HasCycle())
	{
		error = true;
		std::cout << " cycle not detected in GraphCreationTest " << std::endl;
	}
	graph.AddEdge(0,1);
	graph.AddEdge(0,2);
	graph.AddEdge(0,3);
	if(graph.AddEdge(3,1))
	{
		error = true;
		std::cout << " unexpected cycle detected in GraphCreationTest " << std::endl;
	}
}

namespace
{
	float distTest(const NodeContent* a, const NodeContent* b)
	{
		if((a->id_ == 2) || (b->id_ == 2)) return 100.f;
		return (float)(b->id_ - a->id_);
	}

	float distLowerTest(const NodeContent* a, const NodeContent* b)
	{
		if((a->id_ == 3) || (b->id_ == 3)) return 100.f;
		return (float)(b->id_ - a->id_);
	}
}

void AStartest(bool& error)
{
	graph_cycle_t graph;
	for(int j = 0; j< 10; ++ j)
	{
		graph.AddNode(new NodeContent(j));
	}
	graph.AddEdge(0,1);
	graph.AddEdge(1,2);
	graph.AddEdge(1,3);
	graph.AddEdge(2,4);
	graph.AddEdge(3,4); // insert cycle
	graph.AddEdge(4,5);
	astar_t astar(graph);
	astar_t::Path path;
	astar_t::Path solution; solution.push_back(0); solution.push_back(1); solution.push_back(3); solution.push_back(4); solution.push_back(5);
	if(!astar.ComputePath(0, 5, path, distTest))
	{
		error = true;
		std::cout << " path not found where supposed to find one in AStarTest" << std::endl;
	}
	else if(path.size() != solution.size())
	{
		error = true;
		std::cout << " path differs from solution in AStarTest" << std::endl;
	}
	else
	{
		astar_t::Path::const_iterator itsol = solution.begin();
		int i = 0;
		for(astar_t::Path::const_iterator it = path.begin(); it != path.end(); ++it, ++itsol, ++ i)
		{
			if((*it) != (*itsol))
			{
				error = true;
				std::cout << " path differs from solution in AStarTest at element :" << i << std::endl;
				return;
			}
		}
	}
	
	graph.RemoveEdge(3,4);
	path.clear();
	if(!astar.ComputePath(0, 5, path, distTest))
	{
		error = true;
		std::cout << " path not found where supposed to find one in AStarTest" << std::endl;
	}
	path.pop_front(); path.pop_front();
	if(path.front() != 2)
	{
		error = true;
		std::cout << "path not updated upon edge removal" << std::endl;
	}

	graph.RemoveEdge(2,4);
	path.clear();
	if(astar.ComputePath(0, 5, path, distTest))
	{
		error = true;
		std::cout << " path found where not supposed to find one in AStarTest" << std::endl;
	}

	graph.AddEdge(2,4);
	graph.AddEdge(3,4);
	path.clear();
	solution.clear();
	solution.push_back(0); solution.push_back(1); solution.push_back(2); solution.push_back(4); solution.push_back(5);
	astar.ComputePath(0, 5, path, distLowerTest);
	if(path.size() != solution.size())
	{
		error = true;
		std::cout << " path differs from solution in AStarTest with lowerDistanceTest" << std::endl;
	}
	else
	{
		astar_t::Path::const_iterator itsol = solution.begin();
		int i = 0;
		for(astar_t::Path::const_iterator it = path.begin(); it != path.end(); ++it, ++itsol, ++ i)
		{
			if((*it) != (*itsol))
			{
				error = true;
				std::cout << " path differs from solution in AStarTest with lowerDistanceTest at element :" << i << std::endl;
				return;
			}
		}
	}
}

int main(int argc, char *argv[])
{
	std::cout << "performing tests... \n";
	bool error = false;
	GraphCreationTest(error);
	AStartest(error);
	if(error)
	{
		std::cout << "There were some errors\n";
		return -1;
	}
	else
	{
		std::cout << "no errors found \n";
		return 0;
	}
}

