#include <boost/graph/astar_search.hpp>
#include <boost/graph/grid_graph.hpp>
#include <iostream>
#include <math.h>


namespace boost {
// Forward declarations
  struct edge_weight_map;

  // A non-wrapping rank 2 grid
  #define RANK 2
  typedef array<std::size_t, RANK> dimension_array;
  class barrier_grid:public grid_graph<RANK> {
  public:
    // Types required by a-star
    typedef vertex_descriptor vertex_property_type;
    typedef vertices_size_type vertex_index;
    
    barrier_grid(dimension_array& d):grid_graph<RANK>(d) {};
  };

  typedef graph_traits<barrier_grid>::vertex_descriptor vertex_descriptor;
  typedef graph_traits<barrier_grid>::edge_descriptor edge_descriptor;


  /*
  Map from edges to floating point weight values
  */
  struct edge_weight_map {
    typedef float value_type;
    typedef float reference;
    typedef edge_descriptor key_type;
    typedef readable_property_map_tag category;

    reference operator[](key_type e) const {
      // All edges have a weight of one.
      return 1;
    }
  };


  // ReadablePropertyGraph associated types
  template<>
  struct property_map<barrier_grid,
                      edge_weight_t> {
    typedef edge_weight_map type;
    typedef edge_weight_map const_type;
  };
  
  typedef property_map<barrier_grid, edge_weight_t>::const_type
          const_edge_weight_map;
  typedef property_traits<const_edge_weight_map>::reference
          edge_weight_map_reference;
  typedef property_traits<const_edge_weight_map>::key_type
          edge_weight_map_key;
  
  // PropertyMap valid expressions
  edge_weight_map_reference get(const_edge_weight_map, edge_weight_map_key);
  
  inline edge_weight_map_reference
  get(const_edge_weight_map pmap, edge_weight_map_key e) {
    return pmap[e];
  }


  // ReadablePropertyGraph valid expressions
  const_edge_weight_map get(edge_weight_t, const barrier_grid&);
  
  inline const_edge_weight_map get(edge_weight_t, const barrier_grid& g) {
    return const_edge_weight_map();
  }
  
  edge_weight_map_reference get(edge_weight_t,
                                const barrier_grid&,
                                edge_weight_map_key);
  
  inline edge_weight_map_reference get(edge_weight_t tag,
                                       const barrier_grid& g,
                                       edge_weight_map_key e) {
    return get(tag, g)[e];
  }


  // Euclidean heuristic for a grid
  struct euclidean_heuristic:public
    astar_heuristic<barrier_grid, edge_weight_map_reference> {
    euclidean_heuristic(vertex_descriptor goal):m_goal(goal) {}

    float operator()(vertex_descriptor v) {
      return sqrt(pow(m_goal[0]-v[0], 2) + pow(m_goal[1]-v[1], 2));
    }
  private:
    vertex_descriptor m_goal;
  };

  // Exception thrown when the goal is found
  struct found_goal {};
  // Visitor that terminates when we find the goal
  // template<typename barrier_grid, typename vertex_descriptor>
  struct astar_goal_visitor:public default_astar_visitor {
    astar_goal_visitor(vertex_descriptor goal):m_goal(m_goal) {}

    void examine_vertex(vertex_descriptor u, barrier_grid& g) {
      if (u == m_goal)
        throw found_goal();
    }
  private:
    vertex_descriptor m_goal;
  };


  // Print vertices as (x, y) ordered pairs.
  std::ostream& operator<<(std::ostream& output, const vertex_descriptor& v) {
    output << "(" << v[0] << ", " << v[1] << ")";
    return output;
  }

  // Print edges as (x1, y1) -> (x2, y2).
  std::ostream& operator<<(std::ostream& output, const edge_descriptor& e) {
    output << e.first << " -> " << e.second;
    return output;
  }
}
