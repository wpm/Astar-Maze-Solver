
//          Copyright W.P. McNeill 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include <boost/graph/astar_search.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/grid_graph.hpp>
#include <ctime>
#include <iostream>
#include <map>
#include <set>


// Distance traveled in the maze
typedef std::size_t distance;

#define GRID_RANK 2
typedef boost::grid_graph<GRID_RANK> grid;
typedef boost::graph_traits<grid>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<grid>::vertices_size_type vertices_size_type;
typedef boost::graph_traits<grid>::edge_descriptor edge_descriptor;
typedef boost::graph_traits<grid>::vertices_size_type vertices_size_type;

typedef std::set<vertex_descriptor> vertex_set;
typedef boost::vertex_subset_compliment_filter<grid, vertex_set>::type
        filtered_grid;


// A searchable maze
//
// The maze is grid of locations which can either be empty or contain a
// barrier.  You can move to an adjacent location in the grid by going up,
// down, left and right.  Moving onto a barrier is not allowed.  The maze can
// be solved by finding a path from the lower-left-hand corner to the
// upper-right-hand corner.  If no open path exists between these two
// locations, the maze is unsolvable.
//
// The maze is implemented as a filtered grid graph where locations are
// vertices.  Barrier vertices are filtered out of the graph.
//
// A-star search is used to find a path through the maze. Each edge has a
// weight of one, so the total path length is equal to the number of edges
// traversed.
class maze {
public:
  friend std::ostream& operator<<(std::ostream&, const maze&);
  friend maze random_maze(std::size_t, std::size_t);

  maze():m_grid(create_grid(0, 0)),m_barrier_grid(create_barrier_grid()) {};
  maze(std::size_t x, std::size_t y):m_grid(create_grid(x, y)),
       m_barrier_grid(create_barrier_grid()) {};

  // The length of the maze along the specified dimension.
  vertices_size_type length(std::size_t d) const {return m_grid.length(d);}

  // If adding or removing a barrier changes the maze, any previously found
  // solution is discarded.
  void add_barrier(vertex_descriptor u) {
    if (!has_barrier(u))
      m_solution.clear();
    m_barriers.insert(u);
  };
  void remove_barrier(vertex_descriptor u) {
    if (has_barrier(u))
      m_solution.clear();
    m_barriers.erase(u);
  }
  bool has_barrier(vertex_descriptor u) const {
    return m_barriers.find(u) != m_barriers.end();
  }

  // Try to find a path from the lower-left-hand corner source (0,0) to the
  // upper-right-hand corner goal (x-1, y-1).
  vertex_descriptor source() const {return vertex(0, m_grid);}
  vertex_descriptor goal() const {
    return vertex(num_vertices(m_grid)-1, m_grid);
  }


  bool solve();
  bool solved() const {return !m_solution.empty();}
  bool solution_contains(vertex_descriptor u) const {
    return m_solution.find(u) != m_solution.end();
  }

private:
  // Create the underlying rank-2 grid with the specified dimensions.
  grid create_grid(std::size_t x, std::size_t y) {
    boost::array<std::size_t, GRID_RANK> lengths = { {x, y} };
    return grid(lengths);
  }

  // Filter the barrier vertices out of the underlying grid.
  filtered_grid create_barrier_grid() {
    return boost::make_vertex_subset_compliment_filter(m_grid, m_barriers);
  }

  // The grid underlying the maze
  grid m_grid;
  // The underlying maze grid with barrier vertices filtered out
  filtered_grid m_barrier_grid;
  // The barriers in the maze
  vertex_set m_barriers;
  // The vertices on a solution path through the maze
  vertex_set m_solution;
  // The length of the solution path
  distance m_solution_length;
};


// Readable Property Map for edge weights.
struct edge_weight_pmap {
  typedef distance value_type;
  typedef value_type reference;
  typedef edge_descriptor key_type;
  typedef boost::readable_property_map_tag category;
};

typedef boost::property_traits<edge_weight_pmap>::value_type
        edge_weight_pmap_value;
typedef boost::property_traits<edge_weight_pmap>::key_type
        edge_weight_pmap_key;

// All edges are one unit long.
edge_weight_pmap_value get(edge_weight_pmap, edge_weight_pmap_key) {
  return 1;
}

edge_weight_pmap get(boost::edge_weight_t, const grid&) {
  return edge_weight_pmap();
}


// Readable Property Map for vertex indices.
class vertex_index_pmap {
public:
  typedef vertices_size_type value_type;
  typedef value_type reference;
  typedef vertex_descriptor key_type;
  typedef boost::readable_property_map_tag category;

  vertex_index_pmap():m_grid(NULL) {};
  vertex_index_pmap(const grid& g):m_grid(&g) {};

  value_type operator[](key_type u) const {
    // This is backwards from the documentation.
    return get(boost::vertex_index, *m_grid, u);
  }

private:
  const grid* m_grid;
};

typedef boost::property_traits<vertex_index_pmap>::value_type
        vertex_index_pmap_value;
typedef boost::property_traits<vertex_index_pmap>::key_type
        vertex_index_pmap_key;

vertex_index_pmap_value get(vertex_index_pmap pmap, vertex_index_pmap_key u) {
  return pmap[u];
}

vertex_index_pmap get(boost::vertex_index_t, const grid& g) {
  return vertex_index_pmap(g);
}


// Euclidean heuristic for a grid
//
// This calculates the Euclidean distance between a vertex and a goal
// vertex.
class euclidean_heuristic:
      public boost::astar_heuristic<filtered_grid, double>
{
public:
  euclidean_heuristic(vertex_descriptor goal):m_goal(goal) {};

  double operator()(vertex_descriptor v) {
    return sqrt(pow(m_goal[0] - v[0], 2) + pow(m_goal[1] - v[1], 2));
  }

private:
  vertex_descriptor m_goal;
};

// Exception thrown when the goal vertex is found
struct found_goal {};

// Visitor that terminates when we find the goal vertex
struct astar_goal_visitor:public boost::default_astar_visitor {
  astar_goal_visitor(vertex_descriptor goal):m_goal(goal) {};

  // Need const otherwise we get no matching function error at:
  // /opt/local/include/boost/graph/astar_search.hpp:141
  void examine_vertex(vertex_descriptor u, const filtered_grid&) {
    if (u == m_goal)
      throw found_goal();
  }

private:
  vertex_descriptor m_goal;
};


// Solve the maze using A-star search.  Return true if a solution was found.
bool maze::solve() {
  edge_weight_pmap weight = get(boost::edge_weight, m_grid);
  vertex_index_pmap index = get(boost::vertex_index, m_grid);
  // The predecessor map is a vertex-to-vertex mapping.
  typedef std::map<vertex_descriptor, vertex_descriptor> pred_map;
  pred_map predecessor;
  boost::associative_property_map<pred_map> pred_pmap(predecessor);
  // The distance map is a vertex-to-distance mapping.
  typedef std::map<vertex_descriptor, distance> dist_map;
  dist_map distance;
  boost::associative_property_map<dist_map> dist_pmap(distance);

  vertex_descriptor s = source();
  vertex_descriptor g = goal();
  euclidean_heuristic heuristic(g);
  astar_goal_visitor visitor(g);

  try {
    astar_search(m_barrier_grid,
                 source(),
                 heuristic,
                 boost::weight_map(weight).
                 vertex_index_map(index).
                 predecessor_map(pred_pmap).
                 distance_map(dist_pmap).
                 visitor(visitor) );
  } catch(found_goal fg) {
    // Walk backwards from the goal through the predecessor chain adding
    // vertices to the solution path.
    for (vertex_descriptor u = g; u != s; u = predecessor[u])
      m_solution.insert(u);
    m_solution.insert(s);
    m_solution_length = distance[g];
    return true;
  }

  return false;
}


#define BARRIER "#"
// Print the maze as an ASCII map.
std::ostream& operator<<(std::ostream& output, const maze& m) {
  // Header
  for (vertices_size_type i = 0; i < m.length(0)+2; i++)
    output << BARRIER;
  output << std::endl;
  // Body
  for (int y = m.length(1)-1; y >= 0; y--) {
    // Enumerate rows in reverse order and columns in regular order so that
    // (0,0) appears in the lower left-hand corner.  This requires that y be
    // int and not the unsigned vertices_size_type because the loop exit
    // condition is y==-1.
    for (vertices_size_type x = 0; x < m.length(0); x++) {
      // Put a barrier on the left-hand side.
      if (x == 0)
        output << BARRIER;
      // Put the character representing this point in the maze grid.
      vertex_descriptor u = {x, y};
      if (m.solution_contains(u))
        output << ".";
      else if (m.has_barrier(u))
        output << BARRIER;
      else
        output << " ";
      // Put a barrier on the right-hand side.
      if (x == m.length(0)-1)
        output << BARRIER;
    }
    // Put a newline after every row except the last one.
    output << std::endl;
  }
  // Footer
  for (vertices_size_type i = 0; i < m.length(0)+2; i++)
    output << BARRIER;
  if (m.solved())
    output << std::endl << "Solution length " << m.m_solution_length;
  return output;
}

// Generate a maze with a random assignment of barriers.
maze random_maze(vertices_size_type x, vertices_size_type y) {
  maze m(x, y);
  vertices_size_type n = num_vertices(m.m_grid);
  vertex_descriptor s = m.source();
  vertex_descriptor g = m.goal();
  // About one third of the maze should be barriers.
  int barriers = n/3;
  while (barriers > 0) {
    // Choose horizontal or vertical direction.
    std::size_t direction = std::rand() % 2;
    // Walls should be about one quarter of the maze size in this direction.
    vertices_size_type wall = m.length(direction)/4;
    if (wall == 0)
      wall = 1;
    // Create the wall while decrementing the total barrier count.
    vertex_descriptor u = vertex(std::rand() % n, m.m_grid);
    while (wall) {
      // Start and goal spaces should never be barriers.
      if (u != s && u != g) {
        m.m_barriers.insert(u);
        barriers--;
        wall--;
      }
      vertex_descriptor v = m.m_grid.next(u, direction);
      // Stop creating this wall if we reached the maze's edge.
      if (u == v)
        break;
      u = v;
    }
  }
  return m;
}


int main (int argc, char const *argv[]) {
  // The default maze size is 20x10.  A different size may be specified on
  // the command line.
  vertices_size_type x = 20;
  vertices_size_type y = 10;

  if (argc == 3) {
    x = atoi(argv[1]);
    y = atoi(argv[2]);
  }

  std::srand(std::time(0));
  maze m = random_maze(x, y);

  if (m.solve())
    std::cout << "Solved the maze." << std::endl;
  else
    std::cout << "The maze is not solvable." << std::endl;
  std::cout << m << std::endl;
  return 0;
}
