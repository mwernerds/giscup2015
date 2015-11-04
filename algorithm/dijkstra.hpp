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
#ifndef DIJKSTRA_HPP_INC
#define DIJKSTRA_HPP_INC
/*! \brief search_dijkstra searches from vertex s until vertex end is reached using Dijkstra's algorithm
 *         
 *  This implementation is suitable only when vertex_descriptors are size_t (e.g., for vecS storage in the graph).
 */
	std::pair<bool, double> search_dijkstra(size_t s, size_t end)
		{
			lastStart = s; lastGoal = end;
			
			d.resize(num_vertices(g));
			colors.resize(num_vertices(g));
			p.resize(num_vertices(g));
			
			mygraph_t::vertex_iterator u, u_end;
			mygraph_t::out_edge_iterator oe, oe_end;

			// Initialize all data structures
			for (boost::tie(u, u_end) = vertices(g); u != u_end; ++u) 
			{
				d[*u] = std::numeric_limits<double>::infinity();
				colors[*u] = Color::white();
				p[*u] = *u;
			}
			// Start Search
			queue_t Q;
			colors[s] = Color::gray();
			d[s] = 0;
			Q.push(make_pair(d[s],s));

			while(!Q.empty())
			{
				auto u = Q.top();
				Q.pop();
				if (u.first != d[u.second]) 
					continue;
				
				if (u.second == end)
				{
					// Here, we know the shortest path and construct it from the predecessor list p
					shortest_path.clear();
					for(mygraph_t::vertex_descriptor v = end;; v = p[v]) {
						shortest_path.push_front(v);
						if(p[v] == v)
							break;
					}
#ifndef NO_STAT_OUTPUT
		DEFAULT_STATS("Basic Dijkstra");
#endif

					return make_pair(true,d[end]);
				}
				
					
				for (tie(oe, oe_end) = out_edges(u.second,g); 
						oe != oe_end; 
						++oe)
				{
					STAT_EXAMINE();
					
					cost dst = weightmap[*oe]
								+ d[u.second];
					auto v = target(*oe,g);
					if ( dst < d[v])
					{
						d[v] = dst;
						p[v] = u.second;
						
						if (colors[v] == Color::white())
						{
							// discover
							colors[v] = Color::gray();
							Q.push(make_pair(d[v],v));
						}else if(colors[v] == Color::gray())
						{
							// update in queue (we have found a shorter path)
							Q.push(make_pair(d[v],v));
														
						}
					}
				}// end for
				colors[u.second] = Color::black();
			}// while
#ifndef NO_STAT_OUTPUT
DEFAULT_STATS("Basic Dijkstra");
#endif
		
    	return make_pair(false,std::numeric_limits<double>::infinity());	
		};



#endif
