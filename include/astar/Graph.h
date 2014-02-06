
#ifndef _CLASS_GRAPH
#define _CLASS_GRAPH

#include <vector>
#include <array>
#include <algorithm>    // std::find

namespace astar
{
	
/* Generates root positions compatible with existing contact constraints*/
template<typename Numeric=float, int Dim=10000, typename Index = int, typename NodeContent = int>
class Graph
{
	// friend class AStar; TODO Investigate this
public:
	typedef std::vector<Index> T_Connexion;
	typedef std::array<T_Connexion, Dim> T_Edges;
	typedef std::array<NodeContent*, Dim> T_NodeContentPtr;

public:
	 Graph()
		 : currentIndex_(-1)
	 {
		 // NOTHING
	 }

	~Graph()
	{
		for(int i=0; i <= currentIndex_; ++i)
		{
			delete nodeContents_[i];
		}
	}

private:
	Graph(const Graph&);
	Graph& operator=(const Graph&);

public:
	Index AddNode(NodeContent* node)
	{
		if(++currentIndex_ < Dim)
		{
			nodeContents_[currentIndex_] = node;
		}
		return currentIndex_ < Dim ? currentIndex_ : -1;
	}

	// No cycle allowed
	void AddEdge(Index a, Index b, bool checkForCycles = true)
	{
		if(std::find(edges_[a].begin(), edges_[a].end(), b) == edges_[a].end())
		{
			edges_[a].push_back(b);
			edges_[b].push_back(a);
			if(checkForCycles && HasCycle())
			{
				RemoveEdge(a, b);
			}
		}
	}

	void RemoveEdge(Index a, Index b)
	{
		T_Connexion::iterator it = std::find(edges_[a].begin(), edges_[a].end(), b);
		if(it != edges_[a].end())
		{
			edges_[a].erase(it);
		}
		it = std::find(edges_[b].begin(), edges_[b].end(), a);
		if(it != edges_[b].end())
		{
			edges_[b].erase(it);
		}
	}

	bool HasCycle() const
	{
		 // Mark all the vertices as not visited and not part of recursion
		// stack
		bool *visited = new bool[currentIndex_+1];
		for (int i = 0; i < currentIndex_; i++)
		{
			visited[i] = false;
		}
		bool res = false;
		for (int u = 0; u < currentIndex_; u++)
		{
			if (!visited[u] && HasCycleInternal(u, -1, visited))
			{
				res = true;
				break;
			}
		}
		delete visited;
		return res;
	}

private: 
	bool HasCycleInternal(Index currentNode, Index parent, bool* visited) const
	{
		visited[currentNode] = true;
		T_Connexion::const_iterator it = edges_[currentNode].begin();
		for(; it != edges_[currentNode].end(); ++it)
		{
			if(!visited[*it])
			{
				if(HasCycleInternal(*it, currentNode, visited)) return true;
			}
			else
			{
				if(*it != parent) return true;
			}
		}
		return false;
	}

public:
	T_Edges edges_;
	T_NodeContentPtr nodeContents_;
	int currentIndex_;
};
} //namespace astar
#endif //_CLASS_GRAPH
