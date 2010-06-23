#include <iostream>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/grid_graph.hpp>


#define RANK 2
typedef boost::grid_graph<RANK> GridGraph;
typedef GridGraph::vertex_descriptor Vertex;

int main(int argc, char* argv[]) {
	boost::array<std::size_t, RANK> dimensions = { { 3, 4} };
	GridGraph graph(dimensions);
	return 0;
}
