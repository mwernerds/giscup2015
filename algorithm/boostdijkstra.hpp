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
#ifndef BOOSTDIJKSTRA_HPP_INC
#define BOOSTDIJKSTRA_HPP_INC

/*! \brief dijkstra_goal_visitor ends the search by throwing an expception at the goal vertex and counts edge examinations.
 *         
 *  This implementation is suitable only when vertex_descriptors are size_t (e.g., for vecS storage in the graph).
 */
template <class Vertex>
class dijkstra_goal_visitor : public boost::default_dijkstra_visitor
{
	size_t &examines;
	
	WeightMap &weightmap;
public:
  dijkstra_goal_visitor(Vertex goal, WeightMap &_weightmap,size_t &_examines) : m_goal(goal),weightmap(_weightmap),examines(_examines) {}
  
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


/*! \brief search_boostdijkstra searches from vertex s until vertex end is reached using Dijkstra's algorithm as implemented in the Boost Graph Library
 *         
 *  A slight inconsistency in BGL has led to using the non-named parameter call, which is a bit long and unreadable. See the comment and try out
 * whether the commented-out call works in future versions of Boost...
 */

	std::pair<bool, double> search_boostdijkstra(size_t start, size_t end)
	{
		lastStart = start; lastGoal = end;
		p = vector<mygraph_t::vertex_descriptor>(num_vertices(g));
		d = vector<cost> (num_vertices(g));
		colors = vector<default_color_type>(num_vertices(g));
		/*Note: Different from the documentation, dijkstra shortest path
		 * does not pass the colormap directly, therefore the call 
		 * below does not give expected results and we have to skip the 
		 * named_parameter variant and use the classical (tedious, lengthy)
		 * variant*/
				
		/*dijkstra_shortest_paths(g, start,
                          predecessor_map(&p[0]).
                          distance_map(&d[0]).
                          color_map(make_iterator_property_map(colors.begin(), get(boost::vertex_index, g)))
                          );*/
    weightmap = get(edge_weight,g);
	shortest_path.clear();
     try{
			dijkstra_shortest_paths(g, start, &p[0],&d[0],
									weightmap,
									get(boost::vertex_index, g), // VertexIndexMap
									std::less<double>(), // CompareFunction
									closed_plus<double>(), // CombineFunction
									std::numeric_limits<double>::max(), // DistInf
									(double) 0.0, // DistZero
									dijkstra_goal_visitor<mygraph_t::vertex_descriptor>(end,weightmap,stat_examines),
									&colors[0] // ColorMap
									);
                        
     } catch(found_goal fg) { // found a path to the goal
			for(mygraph_t::vertex_descriptor v = end;; v = p[v]) {
				shortest_path.push_front(v);
				if(p[v] == v)
					break;
			}
#ifndef NO_STAT_OUTPUT
	DEFAULT_STATS("Reference Dijkstra");
#endif
			return std::make_pair(true,d[end]);
		}
#ifndef NO_STAT_OUTPUT
		DEFAULT_STATS("Reference Dijkstra");
#endif
		//cout << "No path" << endl;
		return std::make_pair(false,std::numeric_limits<double>::infinity());
  	}
 	

#endif
