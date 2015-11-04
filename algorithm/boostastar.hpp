/* Copyright 2015 Martin Werner - <martin.werner@ifi.lmu.de>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef BOOSTASTAR_HPP_INC
#define BOOSTASTAR_HPP_INC

/*! \brief distance_heuristic provides distance heuristic for the A* search using WebMercatorDistance method
 *         
 */

template <class Graph, class CostType, class LocMap>
class distance_heuristic : public astar_heuristic<Graph, CostType>
{
public:
  typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
  distance_heuristic(LocMap l, Vertex goal)
    : m_location(l), m_goal(goal) {}
  CostType operator()(Vertex u)
  {
    return WebMercatorDistance(m_location[u].x,m_location[u].y,
								m_location[m_goal].x,m_location[m_goal].y);
  }
private:
  LocMap m_location;
  Vertex m_goal;
};


struct found_goal {}; // will be thrown on success


/*! \brief astar_goal_visitor terminates the search on the goal and counts edge examinations
 *         
 */
template <class Vertex>
class astar_goal_visitor : public boost::default_astar_visitor
{
	size_t &examines;
	
	WeightMap &weightmap;
public:
  astar_goal_visitor(Vertex goal, WeightMap &_weightmap,size_t &_examines) : m_goal(goal),weightmap(_weightmap),examines(_examines) {}
  template <class Graph>
  void examine_vertex(Vertex u, Graph& g) {
    if(u == m_goal)
      throw found_goal();
  }
  
  template <class Graph>
  void examine_edge(edge_descriptor u, Graph& g) {
	  examines ++;
 }
  
private:
  Vertex m_goal;
};

/*! \brief search_reference performs the BGL A* search using WebMercatorDistance implementation 
 *         
 */

	std::pair<bool, double> search_reference(size_t start, size_t end)
	{
		lastStart = start; lastGoal = end;
		p = vector<mygraph_t::vertex_descriptor>(num_vertices(g));
		d = vector<cost> (num_vertices(g));
		colors = vector<default_color_type>(num_vertices(g));
		
		try {
			astar_search(g, start,
						 distance_heuristic<mygraph_t, cost, std::vector<location>>
						(locations, end),
						predecessor_map(&p[0]).distance_map(&d[0]).
						color_map(&colors[0]).
						visitor(astar_goal_visitor<mygraph_t::vertex_descriptor>(end,weightmap,stat_examines)));
		} catch(found_goal fg) { // found a path to the goal
			shortest_path.clear();
			for(mygraph_t::vertex_descriptor v = end;; v = p[v]) {
				shortest_path.push_front(v);
				if(p[v] == v)
					break;
			}
#ifndef NO_STAT_OUTPUT
	DEFAULT_STATS("Boost A*");
#endif
			return std::make_pair(true,d[end]);
		}
#ifndef NO_STAT_OUTPUT
		DEFAULT_STATS("Boost A*");
#endif
		//cout << "No path" << endl;
		return std::make_pair(false,std::numeric_limits<double>::infinity());;
  	}
  	

#endif
