
#include "astar/Graph.h"
#include "astar/AStar.h"
#include "PRMTest.h"

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

using namespace planner;

namespace
{
	void PrintPath(const std::vector<const Configuration*>& path)
	{
		for(std::vector<const Configuration*>::const_iterator it = path.begin(); it != path.end(); ++it)
		{
			std::cout << (*it)->id_ << std::endl;
		}
	}

	void Checkpath(const std::vector<const Configuration*>& found, const std::vector<const Configuration*>& expected, const std::string& err, bool& error)
	{
		bool errorPath = false;
		if(found.size() != expected.size())
		{
			errorPath = true;
		}
		else
		{
			std::vector<const Configuration*>::const_iterator it = found.begin();
			std::vector<const Configuration*>::const_iterator it2 = expected.begin();
			for(; it != found.end() && (!errorPath); ++it, ++it2)
			{
				if((*it)->id_ != (*it2)->id_)
				{
					errorPath = true;
				}
			}
		}
		if(errorPath)
		{		
            error = true;
			std::cout << "In PRM test" << err << ": expected and found path do not match" << std::endl;
			std::cout << "Expected:" << std::endl;
			PrintPath(expected);
			std::cout << "Got:" << std::endl;
			PrintPath(found);
		}
	}
}

void PRMTests(bool& error)
{
	Configuration c110(1,1,0);
	Configuration c200(2,0,0);
	Configuration c001(0,0,1);
	Configuration c000(0,0,0);
	Configuration c020(0,2,0);
	Configuration c210(2,1,0);
	Configuration c320(3,2,0);
	Configuration c3150(3,1.5,0);

	std::vector<Configuration*> configs;
	configs.push_back(&c110);
	configs.push_back(&c200);
	configs.push_back(&c001);
	configs.push_back(&c000);
	configs.push_back(&c020);
	configs.push_back(&c210);
	configs.push_back(&c320);
	configs.push_back(&c3150);
	Generator generator1(configs);
	Generator generator2(configs);
	Generator generator3(configs);
	World worldNoColl, worldColl; worldColl.AddCollisionBetweenPath(c110.id_,c200.id_);

	// first test connexions
	float distance = sqrt(2.f) + 0.1f;
	PRMTest prm1(&worldNoColl, &generator1, distance, 8, 5);

	//path from point near 000 to point near 330	
	Configuration nc000(0,0.1f,0); // id 8
	Configuration nc320(3,3,0.1f); // id 9
	std::vector<const Configuration*> expectedPath1;
	expectedPath1.push_back(&nc000);
	expectedPath1.push_back(&c000);
	expectedPath1.push_back(&c110);
	expectedPath1.push_back(&c210);
	expectedPath1.push_back(&c320);
    expectedPath1.push_back(&nc320);

	std::vector<const Configuration*> path1 = prm1.GetPath(nc000, nc320);
	std::string errmess("Path Test1");
	Checkpath(path1, expectedPath1, errmess, error);

	//path form 000 to 330
	std::vector<const Configuration*> expectedPath2;
	expectedPath1.push_back(&c000);
	expectedPath1.push_back(&c000);
	expectedPath1.push_back(&c110);
	expectedPath1.push_back(&c210);
	expectedPath1.push_back(&c320);
	expectedPath1.push_back(&c320);

	std::vector<const Configuration*> path2 = prm1.GetPath(c000, c320);

	// no connexion due to collision between 1,1,0 and 2,0,0
	PRMTest prm2(&worldColl, &generator2, distance, 8, 5);
	Configuration nc110(1,1,0);
	
	std::vector<const Configuration*> expectedPath3, expectedPath4;
	expectedPath3.push_back(&nc110);
	expectedPath3.push_back(&c110);
	expectedPath3.push_back(&c200);
	expectedPath3.push_back(&c200);

	expectedPath4.push_back(&nc110);
	expectedPath4.push_back(&c110);
	expectedPath4.push_back(&c210);
	expectedPath4.push_back(&c200);
	expectedPath4.push_back(&c200);

	std::vector<const Configuration*> path3 = prm1.GetPath(nc110, c200);
	std::vector<const Configuration*> path4 = prm2.GetPath(nc110, c200);
	
	errmess = std::string("Path Test3");
	Checkpath(path3, expectedPath3, errmess, error);

	errmess = std::string("Path Test4");
	Checkpath(path4, expectedPath4, errmess, error);

	//no connexions max neighbours number
	PRMTest prm3(&worldColl, &generator3, distance, 8, 3); // less neighbours should remove connexion
	//between  c210 and c3150
	
	std::vector<const Configuration*> expectedPath5, expectedPath6;
	expectedPath5.push_back(&c210);
	expectedPath5.push_back(&c210);
	expectedPath5.push_back(&c320);
	expectedPath5.push_back(&c3150);
	expectedPath5.push_back(&c3150);

	expectedPath6.push_back(&c210);
	expectedPath6.push_back(&c210);
	expectedPath6.push_back(&c3150);
	expectedPath6.push_back(&c3150);

	std::vector<const Configuration*> path5 = prm3.GetPath(c210, c3150);
	std::vector<const Configuration*> path6 = prm2.GetPath(c210, c3150);

	errmess = std::string("Path Test5");
	Checkpath(path5, expectedPath5, errmess, error);

	errmess = std::string("Path Test6");
	Checkpath(path6, expectedPath6, errmess, error);

	// too far
	Configuration c332(3,3,2);
	std::vector<const Configuration*> path7 = prm1.GetPath(c000, c332);
	if(path7.size() != 0)
	{
		error = true;
		std::cout << "if prm test 7; error : no path should be found; found:" << std::endl;
		PrintPath(path7);
	}
}

int main(int argc, char *argv[])
{
	std::cout << "performing tests... \n";
	bool error = false;
	GraphCreationTest(error);
	AStartest(error);
	PRMTests(error);
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

